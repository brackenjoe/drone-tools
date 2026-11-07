#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Joe Fernandez
"""Print a vehicle's firmware version AND the git hash it was built from.

For a dev build ("V4.8.0-dev") the version string is meaningless on its own,
since master moves daily. The git hash is the only thing that identifies the
code, and it is what you need to pin a matching SITL worktree.

The board id is printed as the other tools print it (board_version >> 16, the
number --expect-board takes).

Exit status: 0 printed, 1 no heartbeat, no AUTOPILOT_VERSION, or more than one
flight controller answering as one system id.

SITL's SERIAL0 (tcp 5760) takes one TCP client, and sitl.sh starts MAVProxy,
which holds it (AP_HAL_SITL/UARTDriver.cpp). So launch SITL with --no-mavproxy
to use the default, or use tcp:127.0.0.1:5762 (SERIAL1, which SITL also
serves) while MAVProxy holds 5760.

Usage:
  ap-version.py                       # SITL started with --no-mavproxy, tcp:127.0.0.1:5760
  ap-version.py tcp:127.0.0.1:5762    # SITL with MAVProxy running
  ap-version.py udp:0.0.0.0:14550     # a real vehicle over telemetry
  ap-version.py /dev/ttyACM0 --baud 115200
"""
import sys, time, argparse
from pymavlink import mavutil

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


def autopilot_versions(m, sysid, compid, window=3.0, tries=6):
    """Every distinct board that answers a version request as (sysid, compid).
    One REQUEST_MESSAGE(AUTOPILOT_VERSION) is sent; replies from that same
    (sysid, compid) are collected for `window` seconds, de-duplicated by uid2.
    Two replies with one uid2 are one board heard on two paths; two uids are
    two boards sharing a system id. Re-sent up to `tries` times if nothing
    arrives. Returns [] if no board answers."""
    for _ in range(tries):
        m.mav.command_long_send(sysid, compid, mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0,
                                mavutil.mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION,
                                0, 0, 0, 0, 0, 0)
        end, found = time.time() + window, {}
        while True:
            left = end - time.time()
            if left <= 0:
                break
            v = recv_from(m, 'AUTOPILOT_VERSION', sysid, compid, left)
            if v is None:
                break
            found.setdefault(bytes(v.uid2[:12]).hex().upper(), v)
        if found:
            return list(found.values())
    return []


def refuse_shared_sysid(uids, sysid, compid):
    """Exit with the refusal when more than one board answers as (sysid, compid)."""
    if len(uids) > 1:
        sys.exit(f"REFUSED: {len(uids)} flight controllers answer as system {sysid} "
                 f"component {compid} on this link (uids {', '.join(uids)}). "
                 f"Give each its own system id (MAV_SYSID on 4.7, SYSID_THISMAV before 4.7).")


ap = argparse.ArgumentParser(
    description="Print a vehicle's firmware version and the git hash it was built from. "
                "Exits 1 if more than one flight controller answers as the same system id.")
ap.add_argument('device', nargs='?', default='tcp:127.0.0.1:5760',
                help='MAVLink connection string: tcp:HOST:PORT, udp:HOST:PORT or a serial '
                     'device (default: %(default)s, the SITL port when started with '
                     '--no-mavproxy; with MAVProxy running, which holds 5760, use '
                     'tcp:127.0.0.1:5762)')
ap.add_argument('--baud', type=int, default=115200,
                help='serial baud rate, ignored for network links (default: %(default)s)')
a = ap.parse_args()

m = mavutil.mavlink_connection(a.device, baud=a.baud, source_system=255)
print(f"connecting to {a.device} ...")
if pin_autopilot(m, 60) is None:
    sys.exit("no autopilot heartbeat received")
sysid, compid = m.target_system, m.target_component
print(f"  heartbeat: autopilot system {sysid} component {compid}")

# AUTOPILOT_VERSION is only sent on request.
vers = autopilot_versions(m, sysid, compid)
if not vers:
    sys.exit("no AUTOPILOT_VERSION received")
uids = [bytes(v.uid2[:12]).hex().upper() for v in vers]

for ver in vers:
    fv = ver.flight_sw_version
    gh = bytes(ver.flight_custom_version).decode('ascii', 'ignore').strip('\x00').strip()
    print(f"\nfirmware : {(fv>>24)&0xff}.{(fv>>16)&0xff}.{(fv>>8)&0xff}")
    print(f"git hash : {gh}")
    print(f"board    : {ver.board_version >> 16}")
    print(f"uid      : {bytes(ver.uid2[:12]).hex().upper()}")
if len(vers) > 1:
    print(f"\nWARNING: {len(vers)} flight controllers answer as system {sysid} "
          f"component {compid} on this link (uids {', '.join(uids)}). "
          f"Give each its own system id (MAV_SYSID on 4.7, SYSID_THISMAV before 4.7).")
    sys.exit(1)
print(f"\nTo pin a SITL worktree to exactly this build:")
print(f"  git -C <your ardupilot checkout> fetch origin")
print(f"  git -C <your ardupilot checkout> worktree add --detach ../ap-{gh[:8]} {gh}")
