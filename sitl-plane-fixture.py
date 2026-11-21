#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Joe Fernandez
"""Fly a SITL ArduPlane the shape a log-analysis test fixture needs, and prove
the boot parameters actually took before it flies.

A sibling of sitl-copter-fixture.py, which is Copter to the bone (GUIDED
arm, NAV_TAKEOFF, LOITER, ALT_HOLD stick input, LAND). A plane cannot be
flown that way at all, so this is a separate tool rather than a --vehicle
flag on that one.

Three profiles:

  --profile fixedwing   AUTO takeoff from a NAV_TAKEOFF mission item -> an
                        AUTO waypoint leg -> FBWB with roll/pitch stick steps
                        -> CRUISE -> LOITER hold -> RTL -> back to AUTO at the
                        landing sequence -> NAV_LAND and disarm.

  --profile quadplane   QSTABILIZE on the ground after arming -> QHOVER climb
                        and hold -> AUTO (which forward-transitions) -> the
                        fixed wing portion above -> QLOITER (back transition)
                        -> QLAND and disarm.

  --profile autotune    AUTO takeoff -> climb to the AUTO cruise altitude ->
                        AUTOTUNE (mode 8) with the sticks flown the way
                        ArduPilot's own autotest flies them, roll axis to
                        convergence then pitch axis -> back to AUTO at the
                        landing sequence -> NAV_LAND and disarm. It exists to
                        log AutoTune's ATRP records. Plane's AutoTune is nothing like
                        Copter's: mode 8 behaves exactly like FBWA and the
                        tuner only learns from attitude demands the PILOT
                        makes, so the sticks have to be flown. Centred sticks
                        still log ATRP at 25 Hz, they just make no progress.
                        Two further gates, both read from the source rather
                        than guessed: AP_FW_Controller.cpp only calls
                        autotune->update() while airspeed > AIRSPEED_MIN
                        (below it there are no ATRP records at all, not merely
                        no progress), and leaving mode 8 for any reason calls
                        AP_AutoTune::stop(), which saves the gains if D_limit
                        and P_limit are both positive and restores them
                        otherwise; entering the mode calls start(), which
                        resets ff_count, D_limit, P_limit and done_count
                        (AP_AutoTune.cpp) -- so an excursion mid-tune
                        restarts the tune from scratch. The profile never
                        leaves the mode until each axis has reported
                        Finished or its --axis-cap-s (default 360 s) has run
                        out, so PASS does not mean the tune converged: it
                        prints, per axis, whether Finished was seen.

It expects SITL to be already running and talks to it over MAVLink directly:

  sitl.sh 4.7.1 ArduPlane --no-mavproxy -f quadplane -w --speedup 1 \
      --add-param-file="$PWD/examples/plane-default-logbitmask.parm"
  python3 sitl-plane-fixture.py --profile quadplane \
      --check-params examples/plane-default-logbitmask.parm

(sitl.sh runs from the ArduPilot tree, so --add-param-file needs an absolute
path. -f quadplane loads ArduPilot's quadplane.parm, which sets LOG_BITMASK
65534; the example file restores the default, 65535. -f plane already boots
at 65535.)

--check-params reads a parameter file and fetches every name in it off the
running vehicle BEFORE arming, printing set/got per name and refusing to fly
if any name is unknown to the firmware or holds a different value. That is the
whole point on a version bump: a renamed parameter loads as nothing and the
flight is silently wrong (wrong logging bitmask) rather than failing.

Whatever --check-params is given, LOG_BITMASK and Q_ENABLE are read off the
vehicle and printed before arming. Q_ENABLE is how a log analyser tells a
quadplane from a fixed wing, and sim_vehicle.py's eeprom.bin survives between
runs, so a frame flown second inherits the first frame's stored value unless
SITL was launched with -w. Read it, do not assume it.

The hold durations (--fbwb-s, --cruise-s, --loiter-s, --rtl-s, --qhover-s,
--qloiter-s) are WALL CLOCK, not sim time. At --speedup 2 a 40 s loiter is
80 s of log and twice the records; fly the fixture at --speedup 1 if the
stated durations are to mean what they say. The AUTO legs are distance
driven and so are unaffected.

This tool sets no parameters itself (the firmware saves tuned gains during
and after an --profile autotune flight, and every profile overwrites the
stored mission) and arms a vehicle. It is for SITL. Do not
point it at an aircraft.

Run from a Python environment with pymavlink.
"""
import argparse
import math
import re
import sys
import time
import unicodedata

from pymavlink import mavutil


def inert(s):
    """s with every C0 control (newline too), DEL and C1 control as \\xNN, and
    every format, line or paragraph separator character as \\uXXXX, so
    text from the vehicle cannot move the cursor, recolour the terminal or
    start a new line."""
    def esc(c):
        cat = unicodedata.category(c)
        if cat == 'Cc':
            return f"\\x{ord(c):02x}"
        # Cf (bidi overrides, zero-width, soft hyphen), Zl, Zp: invisible too
        return f"\\u{ord(c):04x}" if cat in ('Cf', 'Zl', 'Zp') else c
    return ''.join(esc(c) for c in str(s))

# ArduPlane/mode.h at Plane-4.7.1 (dbe79216), read from the tree rather than
# recalled: the Q modes in particular move between releases.
MODE = {
    'MANUAL': 0, 'CIRCLE': 1, 'STABILIZE': 2, 'TRAINING': 3, 'ACRO': 4,
    'FBWA': 5, 'FBWB': 6, 'CRUISE': 7, 'AUTOTUNE': 8, 'AUTO': 10,
    'RTL': 11, 'LOITER': 12, 'TAKEOFF': 13, 'AVOID_ADSB': 14, 'GUIDED': 15,
    'INITIALISING': 16, 'QSTABILIZE': 17, 'QHOVER': 18, 'QLOITER': 19,
    'QLAND': 20, 'QRTL': 21, 'QAUTOTUNE': 22, 'QACRO': 23, 'THERMAL': 24,
    'LOITER_ALT_QLAND': 25, 'AUTOLAND': 26,
}

ap = argparse.ArgumentParser()
ap.add_argument('--device', default='tcp:127.0.0.1:5760',
                help='MAVLink connection string of the running SITL')
ap.add_argument('--profile', choices=('fixedwing', 'quadplane', 'autotune'),
                default='fixedwing')
ap.add_argument('--check-params', action='append', default=[],
                help='parameter file whose every name must read back equal')
ap.add_argument('--params-only', action='store_true',
                help='check parameters and exit without flying')
ap.add_argument('--cruise-alt', type=float, default=100.0,
                help='AUTO waypoint altitude, m above home')
ap.add_argument('--takeoff-alt', type=float, default=40.0,
                help='NAV_TAKEOFF altitude, m above home')
ap.add_argument('--leg', type=float, default=600.0,
                help='half-diagonal of the AUTO waypoint box, m')
ap.add_argument('--fbwb-s', type=float, default=45.0,
                help='FBWB stick steps, wall-clock seconds')
ap.add_argument('--cruise-s', type=float, default=30.0,
                help='CRUISE hold, wall-clock seconds')
ap.add_argument('--loiter-s', type=float, default=45.0,
                help='LOITER hold, wall-clock seconds')
ap.add_argument('--rtl-s', type=float, default=40.0,
                help='RTL, wall-clock seconds before returning to AUTO')
ap.add_argument('--qhover-s', type=float, default=30.0,
                help='QHOVER hold, wall-clock seconds')
ap.add_argument('--qloiter-s', type=float, default=25.0,
                help='QLOITER hold, wall-clock seconds')
ap.add_argument('--qhover-alt', type=float, default=30.0,
                help='altitude to climb to in QHOVER before AUTO, m')
# --profile autotune only.
ap.add_argument('--axis-cap-s', type=float, default=360.0,
                help='wall-clock cap per autotune axis; on expiry the profile '
                     'moves to the next axis and says so rather than aborting')
ap.add_argument('--flip-s', type=float, default=1.0,
                help='seconds between full-stick reversals during AUTOTUNE')
ap.add_argument('--autotune-throttle', type=int, default=2000,
                help='RC3 held through AUTOTUNE. The autotest leaves it at '
                     '2000 and the tune is validated there; it is also what '
                     'keeps airspeed over AIRSPEED_MIN, which gates the '
                     'tuner entirely')
ap.add_argument('--alt-floor', type=float, default=60.0,
                help='if the plane sinks below this during AUTOTUNE, hold '
                     'up-elevator and full throttle until it recovers. Done '
                     'INSIDE mode 8: leaving would restart the tune')
ap.add_argument('--settle-s', type=float, default=15.0,
                help='centred-stick pause between the roll and pitch axes')
a = ap.parse_args()


def pin_autopilot(m, secs):
    """The autopilot's heartbeat, not whichever arrives first: through a
    router or on a shared link the first one may be a GCS's. None on timeout."""
    end = time.time() + secs
    while time.time() < end:
        msg = m.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
        if (msg and msg.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID
                and msg.type != mavutil.mavlink.MAV_TYPE_GCS
                and msg.get_srcSystem() != 0 and msg.get_srcComponent() != 0):  # 0 = broadcast id
            m.target_system, m.target_component = msg.get_srcSystem(), msg.get_srcComponent()
            return msg
    return None


def recv_from(m, types, sysid, compid, timeout, condition=None):
    """recv_match that takes a message only from the pinned autopilot
    (sysid, compid). A message from anyone else is skipped and the wait goes
    on to its own deadline, so a foreign message neither ends the wait early
    nor is taken as the answer. None on timeout."""
    end = time.time() + timeout
    while True:
        left = end - time.time()
        if left <= 0:
            return None
        msg = m.recv_match(type=types, blocking=True, timeout=left, condition=condition)
        if msg is None:
            return None
        if msg.get_srcSystem() == sysid and msg.get_srcComponent() == compid:
            return msg


try:
    m = mavutil.mavlink_connection(a.device, source_system=255)
except OSError as e:
    sys.exit(f"FAIL: no heartbeat on {a.device}: cannot connect ({e})")
print(f"waiting for heartbeat on {a.device} ...", flush=True)
if pin_autopilot(m, 90) is None:
    sys.exit(f"FAIL: no heartbeat on {a.device}")
SYS, COMP = m.target_system, m.target_component
print(f"  system {SYS} component {COMP}", flush=True)


def rxm(type, blocking=True, timeout=1):
    """recv_match, from the pinned autopilot only."""
    return recv_from(m, type, SYS, COMP, timeout)


def read_param(name, timeout=8.0):
    """Fetch one parameter by name. Returns (value, index) or (None, None)."""
    for _ in range(3):
        m.mav.param_request_read_send(SYS, COMP,
                                      name.encode('ascii'), -1)
        deadline = time.time() + timeout / 3
        while time.time() < deadline:
            msg = rxm(type='PARAM_VALUE', blocking=True, timeout=1)
            if msg is None:
                continue
            got = msg.param_id
            if isinstance(got, bytes):
                got = got.decode('ascii', 'replace')
            if got.rstrip('\x00') == name:
                return msg.param_value, msg.param_index
    return None, None


def parse_param_file(path):
    """[(name, value)] from a parameter file, split like ArduPilot's
    AP_Param::parse_param_line: tokens separated by any of ", =\t", and
    an optional third token that must be @READONLY. A line that is neither
    blank, a comment nor NAME VALUE [@READONLY] is refused with exit 2, naming
    the line: a skipped line is a parameter the flight would silently not
    check."""
    out = []
    with open(path) as fh:
        for n, line in enumerate(fh, 1):
            line = line.split('#')[0].strip()
            if not line:
                continue
            parts = [t for t in re.split('[, =\t]+', line) if t]
            try:
                if len(parts) == 3 and parts[2] == '@READONLY':
                    parts = parts[:2]
                if len(parts) != 2:
                    raise ValueError
                val = float(parts[1])
                if not math.isfinite(val):
                    raise ValueError
            except ValueError:
                sys.stderr.write(f"FAIL: {path!r}: line {n} is not NAME VALUE; "
                                 "refusing to fly with a parameter file that is "
                                 "not all read\n")
                sys.exit(2)
            out.append((parts[0], val))
    if not out:
        sys.stderr.write(f"FAIL: {path!r}: no parameter found\n")
        sys.exit(2)
    return out

# This arms and flies. Only a SITL has SIM_* parameters.
sim_speedup, _ = read_param('SIM_SPEEDUP')
if sim_speedup is None:
    sys.stderr.write(f"REFUSED: {a.device} has no SIM_SPEEDUP parameter, so it is not a "
                     "SITL. This tool arms and flies; it will not run against a real "
                     "vehicle.\n")
    sys.exit(2)
print(f"SIM_SPEEDUP = {sim_speedup:g}: a SITL (or a Sim-on-Hardware build)", flush=True)

# ArduPilot sends only HEARTBEAT until a GCS asks for streams. Asked for only
# now, after the refusal: on a real Plane REQUEST_DATA_STREAM is saved to the
# stream-rate parameters.
m.mav.request_data_stream_send(SYS, COMP, mavutil.mavlink.MAV_DATA_STREAM_ALL, 5, 1)


bad = []
for path in a.check_params:
    wanted = parse_param_file(path)
    print(f"\nchecking {len(wanted)} parameters from {path}", flush=True)
    for name, want in wanted:
        got, idx = read_param(name)
        if got is None:
            print(f"  MISSING  {name:18} want {want!r} -- firmware has no such parameter")
            bad.append((name, want, None))
            continue
        # float compare with the tolerance a 32-bit param round-trip needs
        ok = math.isclose(got, want, rel_tol=1e-5, abs_tol=1e-6)
        print(f"  {'ok     ' if ok else 'MISMATCH'} {name:18} want {want!r:10} got {got!r}")
        if not ok:
            bad.append((name, want, got))

if bad:
    print(f"\nFAIL: {len(bad)} parameter(s) did not take:", file=sys.stderr)
    for name, want, got in bad:
        print(f"  {name}: wanted {want}, "
              f"{'absent' if got is None else f'read {got}'}", file=sys.stderr)
    sys.exit(1)
if a.check_params:
    print("\nall boot parameters took.", flush=True)

# Always report the two that decide what the fixture is, whether or not a
# parameter file named them. Q_ENABLE tells a log analyser whether this is a
# quadplane, and it survives in eeprom.bin across sim_vehicle.py runs without -w.
print("\nairframe and logging, read off the vehicle:", flush=True)
for name in ('LOG_BITMASK', 'Q_ENABLE'):
    got, _ = read_param(name)
    print(f"  {name:12} = {'absent' if got is None else got!r}", flush=True)
    if name == 'Q_ENABLE':
        q_enable = 0.0 if got is None else got
if a.profile == 'quadplane' and q_enable < 0.5:
    sys.exit("FAIL: --profile quadplane but Q_ENABLE is not set; "
             "launch SITL with -f quadplane (and -w, the eeprom persists)")
if a.profile in ('fixedwing', 'autotune') and q_enable >= 0.5:
    sys.exit(f"FAIL: --profile {a.profile} but Q_ENABLE is set; a stale "
             "eeprom.bin from a quadplane run. Relaunch SITL with -w")
# The parameters the AutoTune result has to be read against. Printed, not
# checked: they are the firmware's own defaults and the point is to record
# what they were. The log's PARM records are the authority; this is the
# cross-check, and the "after" print below is what the tune actually saved.
TUNE_PARAMS = (
    'AUTOTUNE_LEVEL', 'AUTOTUNE_AXES', 'AUTOTUNE_OPTIONS',
    'ARSPD_TYPE', 'ARSPD_USE', 'AIRSPEED_MIN', 'AIRSPEED_CRUISE',
    'YAW_RATE_ENABLE', 'SCHED_LOOP_RATE',
    'RLL2SRV_TCONST', 'RLL2SRV_RMAX',
    'RLL_RATE_P', 'RLL_RATE_I', 'RLL_RATE_D', 'RLL_RATE_FF',
    'RLL_RATE_IMAX', 'RLL_RATE_SMAX', 'RLL_RATE_FLTT', 'RLL_RATE_FLTE',
    'RLL_RATE_FLTD',
    'PTCH2SRV_TCONST', 'PTCH2SRV_RMAX_UP', 'PTCH2SRV_RMAX_DN',
    'PTCH_RATE_P', 'PTCH_RATE_I', 'PTCH_RATE_D', 'PTCH_RATE_FF',
    'PTCH_RATE_IMAX', 'PTCH_RATE_SMAX', 'PTCH_RATE_FLTT', 'PTCH_RATE_FLTE',
    'PTCH_RATE_FLTD',
    'YAW_RATE_P', 'YAW_RATE_I', 'YAW_RATE_D', 'YAW_RATE_FF',
)


def report_tune_params(label):
    out = {}
    print(f"\ntuning parameters {label}:", flush=True)
    for name in TUNE_PARAMS:
        got, _ = read_param(name)
        out[name] = got
        print(f"  {name:18} = {'absent' if got is None else got!r}", flush=True)
    return out


params_before = report_tune_params('at boot') if a.profile == 'autotune' else {}

if a.params_only:
    sys.exit(0)

print("\nwaiting for GPS 3D fix (up to 180s)...", flush=True)
deadline = time.time() + 180
while time.time() < deadline:
    msg = rxm(type='GPS_RAW_INT', blocking=True, timeout=5)
    if msg and msg.fix_type >= 3:
        print(f"  3D fix, {msg.satellites_visible} sats", flush=True)
        break
else:
    sys.exit("FAIL: no GPS fix")


def cmd(c, *p):
    p = list(p) + [0] * (7 - len(p))
    m.mav.command_long_send(SYS, COMP, c, 0, *p)
    end = time.time() + 10
    while time.time() < end:
        ack = rxm(type='COMMAND_ACK', timeout=max(end - time.time(), 0.1))
        if ack is not None and ack.command == c:   # an ack for something else is not ours
            return ack
    return None


def set_mode(name):
    """Change mode and confirm on HEARTBEAT.custom_mode, not on a mode string:
    pymavlink's plane mode table is its own, the number is the firmware's."""
    want = MODE[name]
    for _ in range(4):
        m.mav.set_mode_send(SYS,
                            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                            want)
        deadline = time.time() + 5
        while time.time() < deadline:
            hb = rxm(type='HEARTBEAT', blocking=True, timeout=2)
            if hb and hb.custom_mode == want:
                print(f"  mode = {name} ({want})", flush=True)
                return True
    sys.exit(f"FAIL: mode {name} ({want}) not confirmed")


def rc(ch1=1500, ch2=1500, ch3=1500, ch4=1500):
    """Override the four sticks. Channels 5-8 are sent as 0, which releases
    them: the quadplane frame's FLTMODE_CH is 5, not 8."""
    m.mav.rc_channels_override_send(SYS, COMP,
                                    ch1, ch2, ch3, ch4, 0, 0, 0, 0)


def hold(seconds, sticks=None):
    """Burn wall-clock while keeping the link fed, optionally holding sticks.
    Overrides expire after RC_OVERRIDE_TIME (3 s), so they are refreshed."""
    end = time.time() + seconds
    while time.time() < end:
        if sticks:
            rc(*sticks)
        rxm(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)


def home_position(timeout=60):
    """Home as (lat, lon) in degrees, from the vehicle rather than hardcoded."""
    m.mav.command_long_send(SYS, COMP,
                            mavutil.mavlink.MAV_CMD_GET_HOME_POSITION,
                            0, 0, 0, 0, 0, 0, 0, 0)
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = rxm(type=['HOME_POSITION', 'GLOBAL_POSITION_INT'],
                           blocking=True, timeout=5)
        if msg is None:
            continue
        if msg.get_type() == 'HOME_POSITION':
            return msg.latitude / 1e7, msg.longitude / 1e7
    sys.exit("FAIL: no HOME_POSITION")


HOME_LAT, HOME_LON = home_position()
print(f"\nhome {HOME_LAT:.7f}, {HOME_LON:.7f}", flush=True)


def offset(dn, de):
    """Metres north/east of home -> (lat, lon) degrees."""
    lat = HOME_LAT + dn / 111320.0
    lon = HOME_LON + de / (111320.0 * math.cos(math.radians(lat)))
    return lat, lon


# The landing sequence is the autotest's own proven CMAC approach geometry
# (Tools/autotest/ArduPlane_Tests/DO_REPOSITION/flaps.txt, items 7-11),
# converted to metres north/east of that mission's home so it travels with
# whatever home this SITL booted at: a downwind leg, a base and a final.
#
# That mission's DO_LAND_START marker (command 189) is deliberately NOT
# carried over. Measured on Plane-4.7.1 (dbe79216): a mission containing
# DO_LAND_START refuses to arm with "PreArm: DO_LAND_START set and
# RTL_AUTOLAND disabled". Enabling RTL_AUTOLAND is not free either: value 2
# (and 4, with DO_RETURN_PATH_START) sends RTL straight into the landing
# sequence instead of flying the RTL leg, and value 1 flies home first, then
# lands. The tool jumps to the approach with MISSION_SET_CURRENT, so the
# marker earns nothing.
LANDING = [
    (16, 14.0, -226.1, 60.0),     # NAV_WAYPOINT, downwind
    (16, -245.5, -93.1, 55.0),    # NAV_WAYPOINT, base
    (16, -194.6, 22.3, 40.0),     # NAV_WAYPOINT, final
    (21, 24.6, -1.4, 0.0),        # NAV_LAND, near home (24.6 m north)
]


def build_mission():
    """(cmd, dnorth, deast, alt) items. Item 0 is the throwaway home slot
    every QGC WPL mission carries."""
    L = a.leg
    items = [(16, 0.0, 0.0, 0.0)]
    if a.profile != 'quadplane':
        # AUTO takeoff: pitch 15 deg, runway heading, from the autotest's own
        # takeoff waypoint offset.
        items.append((22, 220.8, -91.4, a.takeoff_alt))
    else:
        # The quadplane is already airborne in QHOVER when AUTO is entered,
        # so its mission starts at the first waypoint. A NAV_VTOL_TAKEOFF
        # slot is kept so the item numbering matches the fixed wing's.
        items.append((84, 0.0, 0.0, a.qhover_alt))
    # A four-legged box, long enough that each leg settles on the new heading
    # before the next turn.
    items += [
        (16, L, -L * 0.6, a.cruise_alt),
        (16, L, L * 0.6, a.cruise_alt),
        (16, -L * 0.8, L * 0.6, a.cruise_alt),
        (16, -L * 0.8, -L * 0.6, a.cruise_alt),
        (16, L * 0.6, -L * 0.6, a.cruise_alt),
    ]
    if a.profile != 'quadplane':
        items += LANDING
    return items


MISSION = build_mission()
# The mission item the tool jumps to when it is done flying the manual legs.
LAND_START = len(MISSION) - len(LANDING) if a.profile != 'quadplane' else None


def upload_mission(items):
    print(f"uploading {len(items)} mission items ...", flush=True)
    m.mav.mission_count_send(SYS, COMP, len(items),
                             mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
    deadline = time.time() + 60
    sent = set()
    while time.time() < deadline:
        msg = rxm(type=['MISSION_REQUEST', 'MISSION_REQUEST_INT',
                                 'MISSION_ACK'], blocking=True, timeout=5)
        if msg is None:
            continue
        if msg.get_type() == 'MISSION_ACK':
            if msg.type == 0:
                print(f"  accepted ({len(sent)} items requested)", flush=True)
                return True
            sys.exit(f"FAIL: mission rejected, MAV_MISSION_RESULT {msg.type}")
        seq = msg.seq
        c, dn, de, alt = items[seq]
        lat, lon = offset(dn, de)
        # Item 0 is the home slot: frame 0 (GLOBAL), everything else frame 3
        # (GLOBAL_RELATIVE_ALT), which is what a QGC WPL file carries.
        frame = (mavutil.mavlink.MAV_FRAME_GLOBAL if seq == 0
                 else mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT)
        p1 = 15.0 if c == 22 else 0.0        # NAV_TAKEOFF pitch, degrees
        p4 = 1.0 if c == 21 else 0.0         # param4 is the yaw angle; set to 1 degree for NAV_LAND
        m.mav.mission_item_int_send(
            SYS, COMP, seq, frame, c,
            0, 1, p1, 0.0, 0.0, p4,
            int(round(lat * 1e7)), int(round(lon * 1e7)), alt,
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
        sent.add(seq)
    sys.exit("FAIL: mission upload timed out")


upload_mission(MISSION)


def set_current(seq):
    m.mav.mission_set_current_send(SYS, COMP, seq)
    deadline = time.time() + 10
    while time.time() < deadline:
        msg = rxm(type='MISSION_CURRENT', blocking=True, timeout=2)
        if msg and msg.seq == seq:
            print(f"  current mission item = {seq}", flush=True)
            return True
    sys.exit(f"FAIL: current mission item {seq} not confirmed")


def wait_mission_item(seq, timeout):
    """Return True once the vehicle is flying to item seq or beyond."""
    deadline = time.time() + timeout
    last = None
    while time.time() < deadline:
        msg = rxm(type='MISSION_CURRENT', blocking=True, timeout=2)
        if msg is None:
            continue
        if msg.seq != last:
            last = msg.seq
            print(f"    at mission item {msg.seq}", flush=True)
        if msg.seq >= seq:
            return True
    sys.exit(f"FAIL: mission item {seq} not reached in {timeout:.0f} s "
             f"(at {last})")


def wait_alt(target, timeout, above=True):
    deadline = time.time() + timeout
    peak = -1e9
    while time.time() < deadline:
        msg = rxm(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
        if not msg:
            continue
        alt = msg.relative_alt / 1000.0
        peak = max(peak, alt)
        if (alt >= target) if above else (alt <= target):
            print(f"    {alt:.1f} m", flush=True)
            return True
    sys.exit(f"FAIL: {target:.0f} m not reached in {timeout:.0f} s "
             f"(best {peak:.1f} m)")


def arm():
    print("arming (retrying while prearm settles)...", flush=True)
    why = ''
    for _ in range(40):
        rc(1500, 1500, 1000, 1500)      # throttle low, or prearm refuses
        r = cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
        if r and r.result == 0:
            print("  ARMED", flush=True)
            return
        # The refusal reason only ever arrives as STATUSTEXT; without it a
        # failed arm is a silent two-minute wait.
        end = time.time() + 2
        while time.time() < end:
            st = rxm(type='STATUSTEXT', blocking=True, timeout=1)
            if st and ('rm:' in st.text or 'rm :' in st.text):
                t = inert(st.text)
                if t != why:
                    why = t
                    print(f"    {why}", flush=True)
    sys.exit(f"FAIL: could not arm{': ' + why if why else ''}")


def wait_disarm(timeout):
    """LAND_DISARMDELAY is 20 s after touchdown by default, so be generous."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        hb = rxm(type='HEARTBEAT', blocking=True, timeout=5)
        if hb and not (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            print("  landed and disarmed", flush=True)
            return True
    sys.exit(f"FAIL: still armed {timeout:.0f} s after the landing")


# ------------------------------------------------ AUTOTUNE (mode 8) support

STATUS_LOG = []          # (seconds since AUTOTUNE entry, severity, text)
last_alt = [None]


def pump(seconds, sticks, want=None, t0=None):
    """Hold sticks for `seconds` of wall clock, printing every STATUSTEXT as
    it lands and tracking altitude. Returns the text of the first STATUSTEXT
    containing `want` (returning early), else None.

    hold() cannot be used for the tune: it discards everything but
    GLOBAL_POSITION_INT, and AP_AutoTune's only progress report -- "RollD:",
    "RollP:", "Roll: Finished" -- is STATUSTEXT.
    """
    end = time.time() + seconds
    while time.time() < end:
        alt = last_alt[0]
        if alt is not None and alt < a.alt_floor:
            # Up elevator (low RC2 is nose up) and full throttle, INSIDE mode
            # 8: leaving the mode calls AP_AutoTune::stop() and re-entering restarts the
            # tune. A pitch demand is legitimate tuning input anyway.
            rc(1500, 1200, 2000, 1500)
        else:
            rc(*sticks)
        msg = rxm(type=['STATUSTEXT', 'GLOBAL_POSITION_INT'],
                           blocking=True, timeout=0.5)
        if msg is None:
            continue
        if msg.get_type() == 'GLOBAL_POSITION_INT':
            last_alt[0] = msg.relative_alt / 1000.0
            continue
        text = msg.text
        if isinstance(text, bytes):
            text = text.decode('ascii', 'replace')
        text = text.strip('\x00').strip()
        text = inert(text)
        el = time.time() - t0 if t0 else 0.0
        STATUS_LOG.append((el, msg.severity, text))
        print(f"    [{el:7.1f}s] sev {msg.severity}: {text}", flush=True)
        if want and want in text:
            return text
    return None


def tune_axis(axis, ch, t0):
    """Fly one axis to convergence the way Tools/autotest/arduplane.py's
    run_autotune() does: full stick one way, full stick the other, about once
    a second, the other axis centred, throttle wide open. Returns True if the
    firmware said "<axis>: Finished"."""
    print(f"\n  AUTOTUNE {axis}: full-stick reversals every {a.flip_s:.1f} s, "
          f"cap {a.axis_cap_s:.0f} s", flush=True)
    start = time.time()
    value = 1000
    reported = 0.0
    while time.time() - start < a.axis_cap_s:
        sticks = [1500, 1500, a.autotune_throttle, 1500]
        sticks[ch - 1] = value
        if pump(a.flip_s, tuple(sticks), want=f"{axis}: Finished", t0=t0):
            print(f"  {axis} converged after {time.time() - start:.0f} s",
                  flush=True)
            return True
        value = 2000 if value == 1000 else 1000
        if time.time() - start - reported >= 30:
            reported = time.time() - start
            alt = last_alt[0]
            print(f"    {reported:.0f} s into {axis}, "
                  f"alt {'?' if alt is None else f'{alt:.0f}'} m", flush=True)
    print(f"  {axis} did not report Finished within {a.axis_cap_s:.0f} s; "
          f"moving on", flush=True)
    return False


# ---------------------------------------------------------------- the flight

if a.profile == 'quadplane':
    print("\nQSTABILIZE on the ground ...", flush=True)
    set_mode('QSTABILIZE')
    arm()
    hold(8, sticks=(1500, 1500, 1000, 1500))

    print(f"QHOVER climb to {a.qhover_alt:.0f} m ...", flush=True)
    set_mode('QHOVER')
    # Above mid is climb in QHOVER; the stick has to keep being refreshed.
    end = time.time() + 60
    best = 0.0
    while time.time() < end:
        rc(1500, 1500, 1750, 1500)
        msg = rxm(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
        if msg:
            best = max(best, msg.relative_alt / 1000.0)
        if msg and msg.relative_alt / 1000.0 >= a.qhover_alt:
            print(f"    {msg.relative_alt / 1000.0:.1f} m", flush=True)
            break
    else:
        sys.exit(f"FAIL: QHOVER climb reached {best:.1f} m of {a.qhover_alt:.0f} m in 60 s")
    print(f"QHOVER hold for {a.qhover_s:.0f} s ...", flush=True)
    hold(a.qhover_s, sticks=(1500, 1500, 1500, 1500))

    print("AUTO from the first waypoint: forward transition ...", flush=True)
    rc(1500, 1500, 1500, 1500)
    set_current(2)
    set_mode('AUTO')
else:
    print("\nAUTO takeoff ...", flush=True)
    set_mode('AUTO')
    arm()
    wait_alt(a.takeoff_alt * 0.9, 180)

if a.profile == 'autotune':
    print(f"\nclimbing to {a.cruise_alt:.0f} m in AUTO before AUTOTUNE ...",
          flush=True)
    wait_alt(a.cruise_alt * 0.9, 300)

    print("\nAUTOTUNE (mode 8) ...", flush=True)
    set_mode('AUTOTUNE')
    T0 = time.time()
    # let the mode-entry texts land and the airspeed build at full throttle
    pump(6, (1500, 1500, a.autotune_throttle, 1500), t0=T0)
    roll_ok = tune_axis('Roll', 1, T0)
    print(f"\n  sticks centred for {a.settle_s:.0f} s so roll returns to "
          f"neutral before the pitch axis", flush=True)
    pump(a.settle_s, (1500, 1500, a.autotune_throttle, 1500), t0=T0)
    pitch_ok = tune_axis('Pitch', 2, T0)
    print("\n  sticks centred, leaving the tune", flush=True)
    pump(8, (1500, 1500, a.autotune_throttle, 1500), t0=T0)
    print(f"\nAUTOTUNE span {time.time() - T0:.0f} s: "
          f"roll {'converged' if roll_ok else 'DID NOT converge'}, "
          f"pitch {'converged' if pitch_ok else 'DID NOT converge'}",
          flush=True)
    # Leaving mode 8 calls AP_AutoTune::stop() on all three controllers: gains
    # are saved if both D_limit and P_limit are positive, restored otherwise.
else:
    # The AUTO waypoint leg. Items 2..4 of the box before the manual legs.
    print("AUTO waypoint leg ...", flush=True)
    wait_mission_item(5, 420)

    print(f"FBWB with stick steps for {a.fbwb_s:.0f} s ...", flush=True)
    set_mode('FBWB')
    step = max(1.0, a.fbwb_s / 6)
    for ch1, ch2 in ((1620, 1500), (1500, 1580), (1380, 1500), (1500, 1420),
                     (1620, 1420), (1500, 1500)):
        hold(step, sticks=(ch1, ch2, 1500, 1500))

    print(f"CRUISE for {a.cruise_s:.0f} s ...", flush=True)
    set_mode('CRUISE')
    hold(a.cruise_s / 2, sticks=(1580, 1500, 1500, 1500))
    hold(a.cruise_s / 2, sticks=(1500, 1500, 1500, 1500))

    print(f"LOITER hold for {a.loiter_s:.0f} s ...", flush=True)
    rc(1500, 1500, 1500, 1500)
    set_mode('LOITER')
    hold(a.loiter_s, sticks=(1500, 1500, 1500, 1500))

    print(f"RTL for {a.rtl_s:.0f} s ...", flush=True)
    set_mode('RTL')
    hold(a.rtl_s, sticks=(1500, 1500, 1500, 1500))

if a.profile == 'quadplane':
    print(f"QLOITER for {a.qloiter_s:.0f} s: back transition ...", flush=True)
    set_mode('QLOITER')
    hold(a.qloiter_s, sticks=(1500, 1500, 1500, 1500))
    print("QLAND ...", flush=True)
    set_mode('QLAND')
    wait_disarm(240)
else:
    print("AUTO at the landing sequence ...", flush=True)
    set_current(LAND_START)
    set_mode('AUTO')
    wait_mission_item(len(MISSION) - 1, 300)
    wait_disarm(240)

# release the overrides so the log's RCIN goes back to the sim's own values
m.mav.rc_channels_override_send(SYS, COMP,
                                0, 0, 0, 0, 0, 0, 0, 0)
time.sleep(3)

if a.profile == 'autotune':
    params_after = report_tune_params('after the flight')
    print("\nparameters the tune changed (boot -> end):", flush=True)
    changed = 0
    for name in TUNE_PARAMS:
        b, af = params_before.get(name), params_after.get(name)
        if b is None or af is None:
            continue
        if not math.isclose(b, af, rel_tol=1e-6, abs_tol=1e-9):
            changed += 1
            print(f"  {name:18} {b!r:14} -> {af!r}", flush=True)
    if not changed:
        print("  (none)", flush=True)
    print(f"\nSTATUSTEXT during AUTOTUNE ({len(STATUS_LOG)} messages):",
          flush=True)
    for el, sev, text in STATUS_LOG:
        print(f"  [{el:7.1f}s] sev {sev}: {text}", flush=True)

print(f"\nPASS: {a.profile} fixture flight complete.")
