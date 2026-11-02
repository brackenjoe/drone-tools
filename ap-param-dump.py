#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Joe Fernandez
"""Dump a vehicle's parameters AND the board's own defaults, over MAVFTP.

Why the defaults matter. A .param file alone cannot tell you whether a value
was set deliberately or is just what the firmware came up with, and neither
apm.pdef.xml nor a SITL dump answers it: defaults are board- and frame-
specific, baked in by hwdef and defaults.parm. Without them, "this parameter
changed across an upgrade" cannot be separated from "this parameter's default
moved and it was never stored". That distinction is the whole content of an
upgrade check.

ArduPilot serves both in one file, @PARAM/param.pck?withdefaults=1, which is
also far faster than PARAM_REQUEST_LIST (one FTP transfer versus ~1300
round-trips). Falling back to PARAM_REQUEST_LIST gets you values only, and
this says so loudly rather than writing a defaults file that is really a copy
of the values.

The transfer is ap-ftp-get.py's, not pymavlink's MAVFTP.cmd_get, which fails over
a slow, deep-buffered link (USB/IP over a VPN): the FC sends up to 2000 burst
packets that cannot be interrupted, and cmd_get retries after 1 s and then
discards everything. ap-ftp-get.py (kept beside this script) resets the FC's
sessions, drains the link, and re-asks from the first gap. The session reset
also ends any MAVFTP transfer another ground station has open on the vehicle.

Output is Mission Planner format, "NAME,value", sorted, one file for values
and optionally one for defaults.

Usage:
  ap-param-dump.py out.param
  ap-param-dump.py out.param --defaults out.defaults.param
  ap-param-dump.py out.param --defaults out.defaults.param --device /dev/ttyACM0
  ap-param-dump.py out.param --health health.txt   # sensors, GPS, RC, PreArm

Needs pymavlink (pip install 'pymavlink>=2.4.50'; 2.4.49 can crash at connect,
a TypeError in mavutil.add_message, when a link switches from MAVLink 1 to 2;
fixed in 2.4.50) and ap-ftp-get.py beside this script.

--device defaults to the first /dev/serial/by-id/*ArduPilot* device (the
*-if00 interface preferred), else /dev/ttyACM0 if it exists, else it exits
asking for --device.

Exit status: 0 complete; 1 a failed or inconsistent transfer, or a
PARAM_REQUEST_LIST fallback that got fewer values than the vehicle reported
(the file is still written, with a loud line); 2 bad arguments.

Read-only against the vehicle: it requests, it never sets or saves.
"""
import argparse, glob, importlib.util, os, struct, sys, time

ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
ap.add_argument('out', help='where to write the parameter values')
ap.add_argument('--defaults', help='where to write the board defaults')
ap.add_argument('--health', help='where to write a sensor/PreArm snapshot')
ap.add_argument('--device', default=None,
                help='default: the first /dev/serial/by-id/*ArduPilot* device, '
                     'else /dev/ttyACM0 if it exists')
ap.add_argument('--baud', type=int, default=115200)
ap.add_argument('--health-secs', type=float, default=60.0,
                help='how long to collect STATUSTEXT for (default 60)')
ap.add_argument('--timeout', type=float, default=300.0,
                help='seconds allowed for the MAVFTP transfer (default 300)')
ap.add_argument('--no-ftp', action='store_true', help='skip MAVFTP, use PARAM_REQUEST_LIST')
a = ap.parse_args()

try:
    from pymavlink import mavutil, mavftp
except ImportError:
    sys.exit("pymavlink not found: pip install 'pymavlink>=2.4.50'")


def find_device():
    for pat in ('/dev/serial/by-id/*ArduPilot*-if00', '/dev/serial/by-id/*ArduPilot*'):
        hits = sorted(glob.glob(pat))
        if hits:
            return hits[0]
    if os.path.exists('/dev/ttyACM0'):
        return '/dev/ttyACM0'
    sys.exit("no flight controller found; pass --device")


def write_param_file(path, pairs):
    """Mission Planner format. repr() on the float keeps every bit that came
    off the wire; '%f' would silently round a gain to 6 decimals."""
    with open(path, 'w') as f:
        for name, value in sorted(pairs.items()):
            if value == int(value) and abs(value) < 1e9:
                f.write(f"{name},{int(value)}\n")
            else:
                f.write(f"{name},{repr(float(value))}\n")
    print(f"  wrote {len(pairs)} to {path}")


def load_ftp_get():
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'ap-ftp-get.py')
    if not os.path.exists(path):
        sys.exit("ap-param-dump.py needs ap-ftp-get.py beside it (not found at " + path + ")")
    spec = importlib.util.spec_from_file_location('ap_ftp_get', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def fetch_ftp(m, want_defaults, timeout):
    """Returns (values, defaults, total_params), or None if MAVFTP is not
    available (the open was NACKed or never answered). A transfer that started
    and then failed, timed out or did not check out exits 1: no fallback."""
    ftp = load_ftp_get()
    s = ftp.FtpSession(m)
    n, dt = s.reset()
    print(f"sessions reset; drained {n} stale replies in {dt:.1f} s")
    path = '@PARAM/param.pck?withdefaults=1' if want_defaults else '@PARAM/param.pck'
    try:
        data = s.fetch(path, until_eof=True, timeout=timeout)
    except ftp.FtpOpenError as e:
        print(f"  MAVFTP not available: {e}")
        return None
    except TimeoutError as e:
        sys.exit(f"MAVFTP transfer timed out after {timeout:.0f} s: {e}; nothing written")
    except Exception as e:                          # noqa: BLE001
        sys.exit(f"MAVFTP transfer failed: {type(e).__name__}: {e}; nothing written")

    # No CRC check here, unlike ap-ftp-get.py on a real file. @PARAM is
    # generated per read: AP_Filesystem::crc32 reads in 64-byte blocks
    # (libraries/AP_Filesystem/AP_Filesystem.cpp, crc32), which pads the file
    # differently from our 239-byte reads (AP_Filesystem_Param.cpp pads per
    # read size), and values such as STAT_RUNTIME change between reads. So
    # the check is that it decodes and the counts agree with the header.
    if len(data) < 6:
        sys.exit(f"MAVFTP returned {len(data)} bytes, too short to be a parameter file; nothing written")
    _magic, num, total = struct.unpack("<HHH", data[0:6])
    pdata = mavftp.MAVFTP.ftp_param_decode(data)
    if pdata is None:
        sys.exit(f"MAVFTP returned {len(data)} bytes that do not decode as a parameter file; nothing written")
    values = {name.decode() if isinstance(name, bytes) else name: float(v)
              for name, v, _t in pdata.params}
    defaults = None
    if pdata.defaults is not None:
        defaults = {name.decode() if isinstance(name, bytes) else name: float(v)
                    for name, v, _t in pdata.defaults}
    if not (num == total == len(pdata.params) == len(values)):
        sys.exit(f"parameter counts differ: header num_params {num}, total_params {total}, "
                 f"decoded {len(pdata.params)} ({len(values)} distinct names); nothing written")
    return values, defaults, total


def fetch_request_list(m):
    m.mav.param_request_list_send(m.target_system, m.target_component)
    values, expected, last = {}, None, time.time()
    while time.time() - last < 12:
        msg = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=3)
        if msg is None:
            continue
        last = time.time()
        values[msg.param_id] = float(msg.param_value)
        expected = msg.param_count
        if expected and len(values) >= expected:
            break
    return values, expected


def sys_status_bits():
    """Bit -> name, read out of the dialect rather than typed out here.
    A hand-written table of these is a well-worn way to mislabel a report:
    the numbering is not the order anyone remembers, and a wrong label on
    "RC_RECEIVER present" is worse than no label at all."""
    out = {}
    for key, val in vars(mavutil.mavlink).items():
        if (key.startswith('MAV_SYS_STATUS_') and isinstance(val, int)
                and val > 0 and (val & (val - 1)) == 0):
            out[val.bit_length() - 1] = key.replace(
                'MAV_SYS_STATUS_SENSOR_', '').replace('MAV_SYS_STATUS_', '')
    return out


def snapshot_health(m, path, secs):
    """Ask for the things an upgrade can quietly break, then listen. Nothing
    here writes to the vehicle: RUN_PREARM_CHECKS only triggers a report.
    MAVProxy's 'arm check' is not used because that writes ARMING_CHECK."""
    out = []
    def say(s):
        print(f"  {s}")
        out.append(s)

    for mid in (mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS,
                mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT,
                mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS,
                mavutil.mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION):
        m.mav.command_long_send(m.target_system, m.target_component,
                                mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0,
                                mid, 0, 0, 0, 0, 0, 0)
        time.sleep(0.2)

    # trigger a PreArm run rather than waiting for the periodic report
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_RUN_PREARM_CHECKS, 0,
                            0, 0, 0, 0, 0, 0, 0)

    seen, texts = {}, []
    deadline = time.time() + secs
    while time.time() < deadline:
        msg = m.recv_match(blocking=True, timeout=2,
                           type=['SYS_STATUS', 'GPS_RAW_INT', 'RC_CHANNELS',
                                 'STATUSTEXT', 'SCALED_IMU2', 'SCALED_IMU3',
                                 'RAW_IMU', 'AUTOPILOT_VERSION'])
        if msg is None:
            continue
        t = msg.get_type()
        if t == 'STATUSTEXT':
            s = msg.text.strip()
            if s not in texts:
                texts.append(s)
        else:
            seen[t] = msg

    say(f"=== health snapshot, {secs:.0f} s ===")
    ver = seen.get('AUTOPILOT_VERSION')
    if ver:
        fv = ver.flight_sw_version
        gh = bytes(ver.flight_custom_version).decode('ascii', 'ignore').strip('\x00').strip()
        say(f"firmware {(fv>>24)&0xff}.{(fv>>16)&0xff}.{(fv>>8)&0xff} git {gh} board_version {ver.board_version}")

    ss = seen.get('SYS_STATUS')
    if ss:
        say(f"SYS_STATUS present=0x{ss.onboard_control_sensors_present:08x} "
            f"enabled=0x{ss.onboard_control_sensors_enabled:08x} "
            f"health=0x{ss.onboard_control_sensors_health:08x}")
        for bit, name in sorted(sys_status_bits().items()):
            mask = 1 << bit
            if ss.onboard_control_sensors_present & mask:
                say(f"  bit {bit:2d} {name:24s} present  "
                    f"{'enabled' if ss.onboard_control_sensors_enabled & mask else 'disabled':8s} "
                    f"{'HEALTHY' if ss.onboard_control_sensors_health & mask else 'UNHEALTHY'}")

    g = seen.get('GPS_RAW_INT')
    if g:
        say(f"GPS fix_type={g.fix_type} satellites={g.satellites_visible} "
            f"hdop={g.eph} lat={g.lat} lon={g.lon}")
    else:
        say("GPS: no GPS_RAW_INT received")

    rc = seen.get('RC_CHANNELS')
    if rc:
        chans = [getattr(rc, f'chan{i}_raw') for i in range(1, 17)]
        say(f"RC chancount={rc.chancount} rssi={rc.rssi} first8={chans[:8]}")
    else:
        say("RC: no RC_CHANNELS received")

    for t in ('RAW_IMU', 'SCALED_IMU2', 'SCALED_IMU3'):
        msg = seen.get(t)
        if msg:
            say(f"{t}: mag=({msg.xmag},{msg.ymag},{msg.zmag})")

    say(f"--- STATUSTEXT ({len(texts)}) ---")
    for s in texts:
        say(f"  {s}")
    if not any('PreArm' in s for s in texts):
        say("  (no PreArm text seen; either all checks pass or none reported)")

    with open(path, 'w') as f:
        f.write('\n'.join(out) + '\n')
    print(f"  wrote health snapshot to {path}")


def main():
    device = a.device or find_device()
    print(f"connecting to {device} ...")
    m = mavutil.mavlink_connection(device, baud=a.baud, source_system=250,
                                   source_component=190)
    if m.wait_heartbeat(timeout=30) is None:
        sys.exit(f"no heartbeat on {device}")
    print(f"  heartbeat: system {m.target_system}")

    values = defaults = None
    incomplete = False
    if not a.no_ftp:
        print("fetching @PARAM/param.pck ...")
        got = fetch_ftp(m, bool(a.defaults), a.timeout)
        if got:
            values, defaults, total = got
            print(f"  decoded {len(values)} params (header says {total})")

    if values is None:
        print("falling back to PARAM_REQUEST_LIST (no defaults available) ...")
        values, expected = fetch_request_list(m)
        print(f"  got {len(values)} of {expected}")
        if expected is None or len(values) < expected:
            incomplete = True
            print(f"  INCOMPLETE: got {len(values)} of {expected}; the file is "
                  f"written but is missing parameters; exiting 1")
        if a.defaults:
            print("  WARNING: --defaults requested but MAVFTP failed; "
                  "no defaults file written")

    write_param_file(a.out, values)
    if a.defaults and defaults:
        write_param_file(a.defaults, defaults)
    elif a.defaults and not defaults:
        print("  no defaults returned by the board")

    if a.health:
        snapshot_health(m, a.health, a.health_secs)
    return 1 if incomplete else 0


sys.exit(main())
