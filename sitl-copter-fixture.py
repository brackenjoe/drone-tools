#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Joe Fernandez
"""Fly a SITL copter well enough to make a log-analysis test fixture: one log
with a hover, yaw turns, a loiter, stick steps and a landing, for a log
analyser's tests. It proves the boot parameters actually took before it flies.

Unlike sitl-flight-test.py, which only proves a build flies, this flies the
shape such a fixture needs:

  GUIDED arm -> NAV_TAKEOFF -> two CONDITION_YAW turns in GUIDED, one each
  way (so the rate controller has something to track) -> LOITER hold ->
  ALT_HOLD with roll and pitch stick steps -> LAND.

It expects SITL to be already running (sitl.sh <tree> ... --no-mavproxy)
and talks to it over MAVLink directly.

  python3 sitl-copter-fixture.py --check-params examples/copter-sitl-4.7.1.parm

--check-params reads a parameter file and fetches every name in it off the
running vehicle BEFORE arming, printing set/got per name and refusing to fly
if any name is unknown to the firmware or holds a different value. That is the
whole point on a version bump: a renamed parameter loads as nothing and the
flight is silently wrong (wrong logging bitmask, wrong simulated pack) rather
than failing.

The --loiter-s, --althold-s and --guided-s holds are WALL CLOCK, not sim
time. At --speedup 2 a 40 s loiter is 80 s of log and twice the records; fly
the fixture at --speedup 1 if the stated durations are to mean what they say.

Run from a Python environment with pymavlink.
"""
import argparse
import math
import re
import sys
import time

from pymavlink import mavutil

ap = argparse.ArgumentParser()
ap.add_argument('--device', default='tcp:127.0.0.1:5760',
                help='MAVLink connection string of the running SITL')
ap.add_argument('--check-params', action='append', default=[],
                help='parameter file whose every name must read back equal')
ap.add_argument('--alt', type=float, default=15.0, help='takeoff altitude, m')
ap.add_argument('--loiter-s', type=float, default=40.0,
                help='LOITER hold, wall-clock seconds')
ap.add_argument('--althold-s', type=float, default=20.0,
                help='ALT_HOLD stick steps, wall-clock seconds')
ap.add_argument('--guided-s', type=float, default=25.0,
                help='the two GUIDED yaw turns together, wall-clock seconds')
ap.add_argument('--params-only', action='store_true',
                help='check parameters and exit without flying')
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


# ArduCopter custom_mode numbers (Mode::Number in ArduCopter/mode.h).
MODE = {'ALT_HOLD': 2, 'GUIDED': 4, 'LOITER': 5, 'LAND': 9}


def set_mode(name):
    """DO_SET_MODE by number to the pinned autopilot, confirmed on its
    HEARTBEAT.custom_mode. An unconfirmed change is a FAIL, not a warning:
    the rest of the flight would not be the flight the log is named for."""
    want = MODE[name]
    deadline = time.time() + 15
    while time.time() < deadline:
        m.mav.command_long_send(SYS, COMP, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                want, 0, 0, 0, 0, 0)
        t = min(time.time() + 3, deadline)
        while time.time() < t:
            hb = rxm(type='HEARTBEAT', timeout=max(t - time.time(), 0.1))
            if hb is not None and hb.custom_mode == want:
                print(f"  mode = {name}", flush=True)
                return True
    sys.exit(f"FAIL: mode {name} ({want}) not confirmed")


def rc(ch1=1500, ch2=1500, ch3=1500, ch4=1500):
    """Override the four sticks. ch3 mid is a zero climb rate in ALT_HOLD."""
    m.mav.rc_channels_override_send(SYS, COMP,
                                    ch1, ch2, ch3, ch4, 0, 0, 0, 0)


def hold(seconds, sticks=None):
    """Burn wall-clock while keeping the link fed, optionally holding sticks."""
    end = time.time() + seconds
    while time.time() < end:
        if sticks:
            rc(*sticks)
        rxm(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)


set_mode('GUIDED')
print("arming (retrying while prearm settles)...", flush=True)
for _ in range(40):
    r = cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
    if r and r.result == 0:
        break
    time.sleep(2)
else:
    sys.exit("FAIL: could not arm")
print("  ARMED", flush=True)

print(f"takeoff to {a.alt} m ...", flush=True)
cmd(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, a.alt)
peak = 0.0
deadline = time.time() + 120
while time.time() < deadline:
    msg = rxm(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
    if not msg:
        continue
    alt = msg.relative_alt / 1000.0
    peak = max(peak, alt)
    if alt >= a.alt * 0.95:
        print(f"  reached {alt:.1f} m", flush=True)
        break
else:
    sys.exit(f"FAIL: only reached {peak:.1f} m of {a.alt} m")

# GUIDED manoeuvre: two 90 degree yaw turns, one each way, so the rate
# controller and the mixer have a signal.
print(f"GUIDED manoeuvre for {a.guided_s:.0f} s ...", flush=True)
cmd(mavutil.mavlink.MAV_CMD_CONDITION_YAW, 90, 25, 1, 1)
hold(a.guided_s / 2)
cmd(mavutil.mavlink.MAV_CMD_CONDITION_YAW, 90, 25, -1, 1)
hold(a.guided_s / 2)

print(f"LOITER hold for {a.loiter_s:.0f} s ...", flush=True)
set_mode('LOITER')
rc(1500, 1500, 1500, 1500)
hold(a.loiter_s, sticks=(1500, 1500, 1500, 1500))

print(f"ALT_HOLD with stick input for {a.althold_s:.0f} s ...", flush=True)
set_mode('ALT_HOLD')
step = max(1.0, a.althold_s / 6)
for ch1, ch2 in ((1560, 1500), (1500, 1560), (1440, 1500), (1500, 1440),
                 (1500, 1500), (1500, 1500)):
    hold(step, sticks=(ch1, ch2, 1500, 1500))

print("landing ...", flush=True)
rc(1500, 1500, 1500, 1500)
set_mode('LAND')
deadline = time.time() + 180
while time.time() < deadline:
    hb = rxm(type='HEARTBEAT', blocking=True, timeout=5)
    if hb and not (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
        print("  landed and disarmed", flush=True)
        break
else:
    sys.exit("FAIL: still armed 180 s after LAND")

# release the overrides so the log's RCIN goes back to the sim's own values
m.mav.rc_channels_override_send(SYS, COMP,
                                0, 0, 0, 0, 0, 0, 0, 0)
time.sleep(3)
print("\nPASS: fixture flight complete.")
