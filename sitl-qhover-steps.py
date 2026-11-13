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
sets QLAND and exits 1 with FAIL.
"""
import argparse
import json
import math
import os
import re
import sys
import time

# ArduPlane/mode.h at Plane-4.7.1 (dbe79216), read from the tree.
MODE = {'QSTABILIZE': 17, 'QHOVER': 18, 'QLOITER': 19, 'QLAND': 20}

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
ap.add_argument('--alt', type=float, default=10.0, help='climb target, m (default 10)')
ap.add_argument('--climb-pwm', type=int, default=1750,
                help='throttle stick PWM during the climb (default 1750)')
ap.add_argument('--hold-s', type=float, default=20.0,
                help='hands-off hold after the climb, wall-clock seconds (default 20)')
ap.add_argument('--yaw-step', type=int, default=250,
                help='yaw stick offset from 1500 for the yaw steps, PWM (default 250, half stick)')
ap.add_argument('--yaw-step-s', type=float, default=3.0,
                help='length of each yaw step, seconds (default 3)')
ap.add_argument('--rp-step', type=int, default=250,
                help='roll and pitch stick offset from 1500 for their steps, PWM (default 250)')
ap.add_argument('--rp-step-s', type=float, default=2.0,
                help='length of each roll and pitch step, seconds (default 2)')
ap.add_argument('--settle-s', type=float, default=3.0,
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
                and msg.type != mavutil.mavlink.MAV_TYPE_GCS):
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
    """s with every C0 control (newline too), DEL and C1 control as \\xNN, so
    text from the vehicle cannot move the cursor, recolour the terminal or
    start a new line."""
    return re.sub('[\x00-\x1f\x7f-\x9f]', lambda c: f"\\x{ord(c.group()):02x}", str(s))


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
# file or a failed run leaves nothing for vtol-yaw-metrics.py to pick up.
PARAM_FILES = [(path, parse_param_file(path)) for path in a.check_params]
if a.phases_out and not a.params_only:
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


m.mav.request_data_stream_send(SYS, COMP, mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)

last_boot_ms = [None]


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
    msg = rx(want, timeout)
    note(msg)
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
print(f"SITL confirmed (SIM_SPEEDUP = {sim_speedup:g})", flush=True)

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
                    print("  ARMED", flush=True)
                    return
                break
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
phase('ground')
arm()

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
    rc(1500, 1500, 1500, 1500)
    set_mode('QLAND')
    m.mav.rc_channels_override_send(SYS, COMP, 0, 0, 0, 0, 0, 0, 0, 0)
    sys.exit(f"FAIL: the climb reached {alt:.1f} m of the {a.alt:g} m target in 90 s; "
             "QLAND set")
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

phase('qland')
rc(1500, 1500, 1500, 1500)
set_mode('QLAND')
deadline = time.time() + 180
landed = False
while time.time() < deadline:
    rc(1500, 1500, 1500, 1500)
    if armed() is False:
        landed = True
        break
    recv(['GLOBAL_POSITION_INT'], timeout=0.5)
phase('disarmed' if landed else 'timeout')
phases[-1]['end_ms'] = phases[-1]['start_ms']
m.mav.rc_channels_override_send(SYS, COMP, 0, 0, 0, 0, 0, 0, 0, 0)

print(f"\npeak |plane roll or yaw rate| over MAVLink ATTITUDE: "
      f"{peak['yaw_rate_dps']:.0f} deg/s",
      flush=True)
if a.phases_out:
    with open(a.phases_out, 'w') as fh:
        json.dump({'airframe': airframe, 'phases': phases,
                   'sticks': {'yaw_step_pwm': Y, 'rp_step_pwm': R}},
                  fh, indent=1)
    print(f"phases written to {a.phases_out}", flush=True)
print("PASS: landed and disarmed" if landed else "FAIL: still armed",
      flush=True)
sys.exit(0 if landed else 1)
