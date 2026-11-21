#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Joe Fernandez
"""Fly an already-running SITL quadplane or copter tailsitter a hover-only
step profile, and write down when each phase started in the vehicle's own
boot time so the log can be cut by phase afterwards.

  QHOVER arm on the ground -> climb to --alt -> hands-off hold (--hold-s)
  -> yaw stick steps +, -, centred -> roll steps +, -, then pitch steps +, -,
  each followed by centred sticks -> QLAND and disarm.

Hover only. No transitions, no AUTO, no mission: this is the profile for a
yaw-authority question, where the thing being measured is whether the rate
controller can follow a yaw step and hold heading hands-off, and how the
motor split answers roll and pitch demands.

It talks MAVLink to SITL directly, with no ground station in between. Start
SITL without MAVProxy (from an ArduPilot checkout), then run this against it
(needs Python 3 and pymavlink):

  Tools/autotest/sim_vehicle.py -v ArduPlane -f quadplane-copter_tailsitter \\
      --no-mavproxy -w --speedup 1 --add-param-file FILE ...
  python3 sitl-qhover-steps.py --check-params FILE --phases-out phases.json

--check-params (repeatable) reads every name in a parameter file back off the
vehicle before arming and refuses to fly if any is missing or differs: a
renamed parameter loads as nothing and the flight is silently wrong.
Whatever it is given, the airframe-defining parameters (Q_ENABLE,
Q_FRAME_CLASS, Q_FRAME_TYPE, Q_TAILSIT_ENABLE, Q_TAILSIT_MOTMX, Q_OPTIONS,
LOG_BITMASK, SERVO1-8_FUNCTION) are read and printed, and go into the phases file.

--phases-out writes JSON: the parameters read, and each phase's name and
start and end in time_boot_ms, the vehicle's boot time, from SYSTEM_TIME,
ATTITUDE or GLOBAL_POSITION_INT, which is the clock the dataflash log's TimeUS
counts in. vtol-yaw-metrics.py reads it
with --phases.

Stick values are PWM offsets from 1500 on a 1000-2000 channel: --yaw-step 250
is half stick. Hold durations are wall clock, so fly at --speedup 1.

This tool arms a vehicle. It is for SITL. Do not point it at an aircraft: it
reads SIM_SPEEDUP back before anything else and exits 2, flying nothing, if the
vehicle has no SIM_* parameter. The climb must reach --alt within 90 s or it
requests QLAND and exits 1 with FAIL.

PASS means the profile was flown: every hold and step reads the autopilot's
heartbeats, and a disarm or a mode other than QHOVER before the qland phase is
a FAIL (exit 1), with no --phases-out file written. After any failed run in which it
ever armed it always releases the RC override and, only if no disarm was seen,
requests QLAND (not waited for); it never disarms. A passing run prints no
cleanup line. Stick and PWM
arguments are range-checked (a usage error, exit 2) before connecting.
"""
import argparse
import json
import math
import os
import re
import sys
import tempfile
import time
import unicodedata

# ArduPlane/mode.h at Plane-4.7.1 (dbe79216), read from the tree.
MODE = {'QSTABILIZE': 17, 'QHOVER': 18, 'QLOITER': 19, 'QLAND': 20}

def pos_float(s):
    """argparse type: a finite number above 0 (durations, altitudes)."""
    try:
        v = float(s)
    except ValueError:
        raise argparse.ArgumentTypeError(f"not a number: {s!r}")
    if not math.isfinite(v) or v <= 0:
        raise argparse.ArgumentTypeError(f"must be finite and above 0, got {s!r}")
    return v


def pwm_int(s):
    """argparse type: an integer PWM value in 1000..2000."""
    try:
        v = int(s)
    except ValueError:
        raise argparse.ArgumentTypeError(f"not an integer: {s!r}")
    if not 1000 <= v <= 2000:
        raise argparse.ArgumentTypeError(f"must be an integer in 1000..2000, got {s!r}")
    return v


def step_int(s):
    """argparse type: a stick offset whose 1500 +/- step stays in 1000..2000."""
    try:
        v = int(s)
    except ValueError:
        raise argparse.ArgumentTypeError(f"not an integer: {s!r}")
    if not 0 <= v <= 500:
        raise argparse.ArgumentTypeError(
            f"must be an integer in 0..500 so that 1500 +/- step is in 1000..2000, got {s!r}")
    return v


ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
ap.add_argument('--device', default='tcp:127.0.0.1:5760',
                help='MAVLink connection to the running SITL (default tcp:127.0.0.1:5760)')
ap.add_argument('--check-params', action='append', default=[], metavar='FILE',
                help='parameter file ("NAME,value" or "NAME value" lines) to read back off the '
                     'vehicle before arming; refuses to fly if any name is missing or differs '
                     '(repeatable)')
ap.add_argument('--params-only', action='store_true',
                help='check parameters and print the airframe set, then exit without flying')
ap.add_argument('--phases-out', metavar='FILE',
                help='write the parameters read and each phase\'s start and end (vehicle '
                     'time_boot_ms) here as JSON, for vtol-yaw-metrics.py --phases')
ap.add_argument('--alt', type=pos_float, default=10.0, help='climb target, m (default 10)')
ap.add_argument('--climb-pwm', type=pwm_int, default=1750,
                help='throttle stick PWM during the climb (default 1750)')
ap.add_argument('--hold-s', type=pos_float, default=20.0,
                help='hands-off hold after the climb, wall-clock seconds (default 20)')
ap.add_argument('--yaw-step', type=step_int, default=250,
                help='yaw stick offset from 1500 for the yaw steps, PWM (default 250, half stick)')
ap.add_argument('--yaw-step-s', type=pos_float, default=3.0,
                help='length of each yaw step, seconds (default 3)')
ap.add_argument('--rp-step', type=step_int, default=250,
                help='roll and pitch stick offset from 1500 for their steps, PWM (default 250)')
ap.add_argument('--rp-step-s', type=pos_float, default=2.0,
                help='length of each roll and pitch step, seconds (default 2)')
ap.add_argument('--settle-s', type=pos_float, default=3.0,
                help='centred sticks after each group of steps, seconds (default 3)')
a = ap.parse_args()

try:
    from pymavlink import mavutil
except ImportError:
    sys.exit("pymavlink not found. Try: pip install 'pymavlink>=2.4.50'")

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


def parse_param_file(path):
    """[(name, value)] from a parameter file, split as ArduPilot's
    AP_Param::parse_param_line does: tokens separated by any of ", =\t", and
    an optional third token that must be @READONLY. A line that is neither
    blank, a comment nor that, or a file with no parameter at all, is refused
    with exit 1: a skipped line is a parameter the flight would silently not
    check."""
    out = []
    first_bad = None
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
                if first_bad is None:
                    first_bad = n
                continue
            out.append((parts[0], val))
    if first_bad is not None:
        sys.exit(f"FAIL: {inert(path)}: line {first_bad} is not NAME VALUE; "
                 "refusing to fly with a parameter file that is not all read")
    if not out:
        sys.exit(f"FAIL: {inert(path)}: no parameter found")
    return out


# Files are read, and a stale phases file removed, before connecting, so a bad
# file or a failed run leaves nothing for vtol-yaw-metrics.py to pick up. A
# phases file is written only after a landing and a disarm of a profile that
# was flown (the vehicle stayed armed and in QHOVER until the qland phase), and
# only when every phase has a start and end time; the landing-timeout and the
# aborted paths write none. An old file is removed in every mode, --params-only
# included.
PARAM_FILES = [(path, parse_param_file(path)) for path in a.check_params]
if a.phases_out:
    try:
        os.remove(a.phases_out)
    except FileNotFoundError:
        pass
    except OSError as e:
        sys.exit(f"FAIL: cannot remove old {inert(a.phases_out)}: {inert(e)}")

m = mavutil.mavlink_connection(a.device, source_system=255)
print(f"waiting for heartbeat on {a.device} ...", flush=True)
if pin_autopilot(m, 120) is None:
    sys.exit(f"FAIL: no heartbeat on {a.device}")
SYS, COMP = m.target_system, m.target_component


def rx(types, timeout):
    """The next message of these types from the pinned autopilot only."""
    return recv_from(m, types, SYS, COMP, timeout)


last_boot_ms = [None]


class FlightAbort(Exception):
    """The vehicle disarmed, or left QHOVER, before the qland phase."""


WATCH = [False]    # true from the arm until the qland phase starts
ARMED = [False]    # may be armed, and no disarm seen yet
EVER_ARMED = [False]    # set when an arm was sent or seen; never cleared


def watch(hb):
    """Raise FlightAbort if this heartbeat shows the vehicle disarmed or out
    of QHOVER while the profile is still being flown."""
    if not WATCH[0]:
        return
    ph = phases[-1]['name'] if phases else 'start'
    if not hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
        ARMED[0] = False
        raise FlightAbort(f"vehicle disarmed during {ph}")
    if hb.custom_mode != MODE['QHOVER']:
        raise FlightAbort(f"vehicle left QHOVER (custom_mode {hb.custom_mode}) during {ph}")


def note(msg):
    """Track the vehicle clock off any message that carries it."""
    if msg is None:
        return
    t = getattr(msg, 'time_boot_ms', None)
    if t is not None and msg.get_type() in ('SYSTEM_TIME', 'ATTITUDE',
                                            'GLOBAL_POSITION_INT'):
        last_boot_ms[0] = t


def recv(types, timeout=1.0):
    want = list(types) + ['SYSTEM_TIME', 'ATTITUDE']
    if WATCH[0]:
        want.append('HEARTBEAT')
    msg = rx(want, timeout)
    note(msg)
    if msg is not None and msg.get_type() == 'HEARTBEAT':
        watch(msg)
    return msg


def read_param(name, timeout=8.0):
    for _ in range(3):
        m.mav.param_request_read_send(SYS, COMP, name.encode('ascii'), -1)
        deadline = time.time() + timeout / 3
        while time.time() < deadline:
            msg = rx('PARAM_VALUE', 1)
            if msg is None:
                continue
            got = msg.param_id
            if isinstance(got, bytes):
                got = got.decode('ascii', 'replace')
            if got.rstrip('\x00') == name:
                return msg.param_value
    return None


# This arms and flies. Only a SITL has SIM_* parameters.
sim_speedup = read_param('SIM_SPEEDUP')
if sim_speedup is None:
    sys.stderr.write(f"REFUSED: {a.device} has no SIM_SPEEDUP parameter, so it is not a "
                     "SITL. This tool arms and flies; it will not run against a real "
                     "vehicle.\n")
    sys.exit(2)
print(f"SIM_SPEEDUP = {sim_speedup:g}: a SITL (or a Sim-on-Hardware build)", flush=True)

# Streams are asked for only after the refusal: on a real Plane
# REQUEST_DATA_STREAM is saved to the stream-rate parameters.
m.mav.request_data_stream_send(SYS, COMP, mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)

bad = []
for path, wanted in PARAM_FILES:
    print(f"\nchecking {len(wanted)} parameters from {inert(path)}", flush=True)
    ok = 0
    for name, want in wanted:
        got = read_param(name)
        if got is None:
            print(f"  MISSING  {inert(name)}  ({inert(path)})", flush=True)
            bad.append(inert(name))
        elif not math.isclose(got, want, rel_tol=1e-5, abs_tol=1e-6):
            print(f"  MISMATCH {inert(name):18} want {want!r} got {got!r}  "
                  f"({inert(path)})", flush=True)
            bad.append(inert(name))
        else:
            ok += 1
    print(f"  {ok} of {len(wanted)} read back equal", flush=True)
if bad:
    sys.exit(f"FAIL: {len(bad)} parameter(s) did not take: {', '.join(bad)}")

AIRFRAME = ['Q_ENABLE', 'Q_FRAME_CLASS', 'Q_FRAME_TYPE', 'Q_TAILSIT_ENABLE',
            'Q_TAILSIT_MOTMX', 'Q_OPTIONS', 'LOG_BITMASK'] + \
           [f'SERVO{i}_FUNCTION' for i in range(1, 9)]
airframe = {}
print("\nairframe, read off the vehicle:", flush=True)
for name in AIRFRAME:
    airframe[name] = read_param(name)
    print(f"  {name:18} = {airframe[name]!r}", flush=True)
if not airframe.get('Q_ENABLE'):
    sys.exit("FAIL: Q_ENABLE is not set; this profile is for a quadplane")
if a.params_only:
    sys.exit(0)

phases = []


def phase(name):
    """Close the previous phase and open a new one at the vehicle's clock."""
    recv(['SYSTEM_TIME'], timeout=2)
    t = last_boot_ms[0]
    if phases:
        phases[-1]['end_ms'] = t
    phases.append({'name': name, 'start_ms': t})
    print(f"[{t} ms] {name}", flush=True)


def rc(ch1=1500, ch2=1500, ch3=1500, ch4=1500):
    """Override the four sticks; 5-8 released (FLTMODE_CH is 5)."""
    m.mav.rc_channels_override_send(SYS, COMP,
                                    ch1, ch2, ch3, ch4, 0, 0, 0, 0)


peak = {'yaw_rate_dps': 0.0}


def hold(seconds, sticks):
    """Hold sticks for wall-clock seconds, refreshing the override and
    keeping the vehicle clock and the peak yaw rate current."""
    end = time.time() + seconds
    while time.time() < end:
        rc(*sticks)
        msg = recv(['GLOBAL_POSITION_INT'], timeout=0.2)
        if msg is not None and msg.get_type() == 'ATTITUDE':
            peak['yaw_rate_dps'] = max(peak['yaw_rate_dps'],
                                       abs(math.degrees(msg.yawspeed)),
                                       abs(math.degrees(msg.rollspeed)))


def set_mode(name):
    want = MODE[name]
    for _ in range(4):
        m.mav.set_mode_send(SYS,
                            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                            want)
        deadline = time.time() + 5
        while time.time() < deadline:
            hb = rx('HEARTBEAT', 2)
            if hb and hb.custom_mode == want:
                print(f"  mode = {name} ({want})", flush=True)
                return True
    print(f"  WARNING: mode {name} not confirmed", flush=True)
    return False


def arm():
    why = ''
    for _ in range(60):
        rc(1500, 1500, 1000, 1500)
        m.mav.command_long_send(SYS, COMP,
                                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                0, 1, 0, 0, 0, 0, 0, 0)
        # Only the autopilot's ack of this command counts.
        end = time.time() + 5
        while time.time() < end:
            ack = rx('COMMAND_ACK', end - time.time())
            if ack is not None and ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                if ack.result == 0:
                    ARMED[0] = EVER_ARMED[0] = True
                    print("  ARMED", flush=True)
                    return
                break
        else:
            ARMED[0] = EVER_ARMED[0] = True      # an unanswered arm may still have taken
        end = time.time() + 2
        while time.time() < end:
            st = rx('STATUSTEXT', 1)
            if st and 'rm' in st.text and inert(st.text) != why:
                why = inert(st.text)
                print(f"    {why}", flush=True)
    sys.exit(f"FAIL: could not arm: {why}")


def armed():
    """True or False from the autopilot's own heartbeat; None if none came, which
    is not evidence of a landing."""
    hb = rx('HEARTBEAT', 3)
    if hb is None:
        return None
    return bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)


print("\nwaiting for GPS 3D fix ...", flush=True)
deadline = time.time() + 180
while time.time() < deadline:
    msg = rx('GPS_RAW_INT', 5)
    if msg and msg.fix_type >= 3:
        break
else:
    sys.exit("FAIL: no GPS fix")

# Q_OPTIONS bit 18 (ARMVTOL) refuses to arm outside a VTOL mode, so the mode
# is set first.
if not set_mode('QHOVER'):
    sys.exit("FAIL: mode QHOVER not confirmed; not arming")
def failed_run():
    """True while an exception other than a clean exit is leaving the run."""
    e = sys.exc_info()[1]
    return e is not None and not (isinstance(e, SystemExit) and e.code in (0, None))


def cleanup():
    """After a failed run that ever armed: always release the RC override;
    then, only if no disarm was seen, request QLAND (not waited for). One
    try; errors are swallowed with a note. Never disarms in flight."""
    if not EVER_ARMED[0] or not failed_run():
        return
    try:
        m.mav.rc_channels_override_send(SYS, COMP, 0, 0, 0, 0, 0, 0, 0, 0)
        if ARMED[0]:
            set_mode('QLAND')
            print("cleanup: overrides released and QLAND requested after a failed run",
                  flush=True)
        else:
            print("cleanup: overrides released after a failed run "
                  "(vehicle already disarmed)", flush=True)
    except (Exception, SystemExit) as e:
        print(f"cleanup: could not finish ({e})", flush=True)


phase('ground')
try:
    arm()
    WATCH[0] = True

    phase('climb')
    end = time.time() + 90
    alt = 0.0
    while time.time() < end:
        rc(1500, 1500, a.climb_pwm, 1500)
        msg = recv(['GLOBAL_POSITION_INT'], timeout=1)
        if msg is not None and msg.get_type() == 'GLOBAL_POSITION_INT':
            alt = msg.relative_alt / 1000.0
            if alt >= a.alt:
                break
    else:
        print(f"  {alt:.1f} m", flush=True)
        sys.exit(f"FAIL: the climb reached {alt:.1f} m of the {a.alt:g} m target in 90 s; "
                 "QLAND requested")
    print(f"  {alt:.1f} m", flush=True)

    phase('hold')
    hold(a.hold_s, (1500, 1500, 1500, 1500))

    Y, R = a.yaw_step, a.rp_step
    for name, sticks, secs in (
            ('yaw_pos', (1500, 1500, 1500, 1500 + Y), a.yaw_step_s),
            ('yaw_neg', (1500, 1500, 1500, 1500 - Y), a.yaw_step_s),
            ('settle_yaw', (1500, 1500, 1500, 1500), a.settle_s),
            ('roll_pos', (1500 + R, 1500, 1500, 1500), a.rp_step_s),
            ('roll_neg', (1500 - R, 1500, 1500, 1500), a.rp_step_s),
            ('settle_roll', (1500, 1500, 1500, 1500), a.settle_s),
            ('pitch_pos', (1500, 1500 + R, 1500, 1500), a.rp_step_s),
            ('pitch_neg', (1500, 1500 - R, 1500, 1500), a.rp_step_s),
            ('settle_pitch', (1500, 1500, 1500, 1500), a.settle_s)):
        phase(name)
        hold(secs, sticks)

    # One fresh heartbeat before the profile counts as flown: a disarm in the last
    # step must not be taken for a landing.
    hb = rx('HEARTBEAT', 3)
    if hb is not None:
        watch(hb)
    WATCH[0] = False
    phase('qland')
    rc(1500, 1500, 1500, 1500)
    set_mode('QLAND')
    deadline = time.time() + 180
    landed = False
    while time.time() < deadline:
        rc(1500, 1500, 1500, 1500)
        if armed() is False:
            landed = True
            ARMED[0] = False
            break
        recv(['GLOBAL_POSITION_INT'], timeout=0.5)
    phase('disarmed' if landed else 'timeout')
    phases[-1]['end_ms'] = phases[-1]['start_ms']
    m.mav.rc_channels_override_send(SYS, COMP, 0, 0, 0, 0, 0, 0, 0, 0)

    print(f"\npeak |plane roll or yaw rate| over MAVLink ATTITUDE: "
          f"{peak['yaw_rate_dps']:.0f} deg/s",
          flush=True)
    if a.phases_out and landed:
        if any(ph['start_ms'] is None or ph['end_ms'] is None for ph in phases):
            sys.exit("FAIL: a phase has no start or end time (no SYSTEM_TIME seen); "
                     "not writing --phases-out")
        # A non-finite airframe value is written as null, never as NaN or Infinity.
        safe_airframe = {k: (v if v is None or math.isfinite(v) else None)
                         for k, v in airframe.items()}
        out_dir = os.path.dirname(os.path.abspath(a.phases_out))
        tmp = None
        try:
            fd, tmp = tempfile.mkstemp(prefix='.phases-', dir=out_dir)
            with os.fdopen(fd, 'w') as fh:
                json.dump({'airframe': safe_airframe, 'phases': phases,
                           'sticks': {'yaw_step_pwm': Y, 'rp_step_pwm': R}},
                          fh, indent=1, allow_nan=False)
            um = os.umask(0)
            os.umask(um)
            os.chmod(tmp, 0o666 & ~um)
            os.replace(tmp, a.phases_out)
            tmp = None
        except (OSError, ValueError) as e:
            sys.exit(f"FAIL: cannot write {inert(a.phases_out)}: {inert(e)}")
        finally:
            if tmp is not None and os.path.exists(tmp):
                os.unlink(tmp)
        print(f"phases written to {inert(a.phases_out)}", flush=True)
    print("PASS: landed and disarmed" if landed else "FAIL: still armed",
          flush=True)
    sys.exit(0 if landed else 1)
except FlightAbort as e:
    sys.exit(f"FAIL: {e}")
finally:
    cleanup()
