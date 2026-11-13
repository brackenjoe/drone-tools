#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Joe Fernandez
"""Smoke-test a SITL build by flying it. Copter and plane paths differ.

  sitl-flight-test.py                   copter (default)
  sitl-flight-test.py --vehicle plane
  sitl-flight-test.py --device tcp:127.0.0.1:5762

SITL's SERIAL0 (tcp 5760) takes one TCP client, and sitl.sh starts MAVProxy,
which holds it (AP_HAL_SITL/UARTDriver.cpp). So either launch SITL with
--no-mavproxy and use the default tcp:127.0.0.1:5760, or leave MAVProxy running
and use tcp:127.0.0.1:5762 (SERIAL1, which SITL also serves).

Before it arms anything it reads SIM_SPEEDUP back; a vehicle that has no SIM_*
parameter is not SITL, and it refuses and exits 2. Exit 1 is a FAIL.

Copter: GUIDED, arm, NAV_TAKEOFF, verify climb, LAND.
Plane:  TAKEOFF mode, arm, verify climb, RTL. Planes do not land on command
        the way copters do, so the plane path stops once climb is proven.
"""
import sys, time, argparse
from pymavlink import mavutil

ap = argparse.ArgumentParser(
    description="Smoke-test a running SITL by flying it: arm, take off, verify the climb. "
                "Prints PASS and exits 0, or exits non-zero with a FAIL reason. "
                "Do not point it at a real vehicle: it arms and takes off.")
ap.add_argument('--vehicle', choices=['copter', 'plane'], default='copter',
                help='flight path to test: copter takes off in GUIDED and lands; plane '
                     'uses TAKEOFF mode and stops once the climb is proven '
                     '(default: %(default)s)')
ap.add_argument('--device', default='tcp:127.0.0.1:5760',
                help='MAVLink connection string of the SITL (default: %(default)s). SITL '
                     'serves one TCP client on 5760, which sitl.sh gives to MAVProxy: '
                     'start SITL with --no-mavproxy, or use tcp:127.0.0.1:5762 while '
                     'MAVProxy runs')
a = ap.parse_args()


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


TGT = 10.0 if a.vehicle == 'copter' else 25.0

try:
    m = mavutil.mavlink_connection(a.device, source_system=255)
except OSError as e:
    sys.exit(f"FAIL: no heartbeat on {a.device}: cannot connect ({e})")
print(f"waiting for heartbeat ({a.vehicle}) ...")
if pin_autopilot(m, 60) is None:
    sys.exit(f"FAIL: no heartbeat on {a.device}")
SYS, COMP = m.target_system, m.target_component
print(f"  system {SYS} component {COMP}")


def rx(types, timeout):
    """The next message of these types from the pinned autopilot only."""
    return recv_from(m, types, SYS, COMP, timeout)


def set_mode(num):
    """DO_SET_MODE by number to the pinned autopilot: set_mode_apm would look
    the name up by whatever heartbeat last set the link's vehicle type."""
    m.mav.command_long_send(SYS, COMP, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, num,
                            0, 0, 0, 0, 0)


# ArduCopter and ArduPlane custom_mode numbers (Mode::Number in mode.h).
MODES = {'copter': {'GUIDED': 4, 'LAND': 9}, 'plane': {'TAKEOFF': 13, 'RTL': 11}}[a.vehicle]

# ArduPilot sends only HEARTBEAT until a GCS asks for streams.
m.mav.request_data_stream_send(SYS, COMP,
                               mavutil.mavlink.MAV_DATA_STREAM_ALL, 5, 1)
print("requested data streams")

# This arms and takes off. Only a SITL has SIM_* parameters.
pv = None
for _ in range(3):
    m.mav.param_request_read_send(SYS, COMP, b'SIM_SPEEDUP', -1)
    end = time.time() + 3
    while pv is None and time.time() < end:
        got = rx('PARAM_VALUE', 1)
        if got is not None and got.param_id.rstrip('\x00') == 'SIM_SPEEDUP':
            pv = got
    if pv is not None:
        break
if pv is None:
    sys.stderr.write(f"REFUSED: {a.device} has no SIM_SPEEDUP parameter, so it is not a SITL. "
                     "This tool arms and takes off; it will not run against a real vehicle.\n")
    sys.exit(2)
print(f"  SITL confirmed (SIM_SPEEDUP = {pv.param_value:g})")

print("waiting for GPS 3D fix (up to 120s)...")
deadline = time.time() + 120
while time.time() < deadline:
    msg = rx('GPS_RAW_INT', 5)
    if msg and msg.fix_type >= 3:
        print(f"  3D fix, {msg.satellites_visible} sats")
        break
else:
    sys.exit("FAIL: no GPS fix")

def cmd(c, *p):
    p = list(p) + [0]*(7-len(p))
    m.mav.command_long_send(SYS, COMP, c, 0, *p)
    end = time.time() + 10
    while time.time() < end:
        ack = rx('COMMAND_ACK', end - time.time())
        if ack is not None and ack.command == c:   # an ack for something else is not ours
            return ack
    return None

mode = 'GUIDED' if a.vehicle == 'copter' else 'TAKEOFF'
set_mode(MODES[mode])
time.sleep(1)
hb = rx('HEARTBEAT', 5)
if hb is None:
    sys.exit("FAIL: no heartbeat from the autopilot after the mode change")
print(f"  mode = {mavutil.mode_string_v10(hb)}")

print("arming (retrying while prearm settles)...")
for _ in range(30):
    r = cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
    if r and r.result == 0:
        break
    time.sleep(2)
else:
    sys.exit("FAIL: could not arm")
print("  ARMED")

if a.vehicle == 'copter':
    print(f"takeoff to {TGT}m...")
    cmd(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,0,0,0,0,0, TGT)
else:
    print(f"TAKEOFF mode should auto-climb; watching for {TGT}m...")

peak = 0.0
deadline = time.time() + 90
while time.time() < deadline:
    msg = rx('GLOBAL_POSITION_INT', 5)
    if not msg:
        continue
    alt = msg.relative_alt / 1000.0
    peak = max(peak, alt)
    if alt >= TGT * 0.95:
        print(f"  reached {alt:.1f}m")
        break
else:
    sys.exit(f"FAIL: only reached {peak:.1f}m of {TGT}m")

if a.vehicle == 'copter':
    print("landing...")
    set_mode(MODES['LAND'])
    deadline = time.time() + 90
    height = peak
    while time.time() < deadline:
        msg = rx(['HEARTBEAT', 'GLOBAL_POSITION_INT'], 5)
        if msg is None:
            continue
        if msg.get_type() == 'GLOBAL_POSITION_INT':
            height = msg.relative_alt / 1000.0
        elif not (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            print("  landed and disarmed")
            break
    else:
        sys.exit(f"FAIL: still armed and airborne at {height:.1f}m at the landing timeout")
else:
    set_mode(MODES['RTL'])
    print("  RTL set; climb proven")

print(f"\nPASS: {a.vehicle} SITL flew.")
