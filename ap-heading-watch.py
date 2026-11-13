#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Joe Fernandez
"""Watch an ArduPilot FC's heading, read-only: the EKF's yaw beside the raw
compass heading, once a second, with what either depends on.

Per line: the EKF yaw (ATTITUDE), the compass heading computed here from the
primary compass's field (RAW_IMU x/y/zmag, milligauss, already calibrated and
rotated into the vehicle frame by the FC), tilt-compensated with ATTITUDE's
roll and pitch and corrected by the FC's live COMPASS_DEC, the field strength
|B|, the EKF's flags and compass variance, the GPS fix type and satellite
count, and the ATTITUDE rate it is seeing.

The compass heading is the same arithmetic as Compass::calculate_heading()
(AP_Compass.cpp): tilt-compensated X and Y, atan2(-Y, X), plus declination.
It moves as soon as the field does, where the EKF's yaw is filtered, so it is
the one to watch when something near the compass changes state. A change in
|B| with the vehicle at rest is interference.

With --gyro-from DEG it also carries a gyro-only yaw: RAW_IMU's z (and y)
gyro, turned into a yaw rate with ATTITUDE's roll and pitch and integrated
from DEG on the FC's own RAW_IMU timestamps. RAW_IMU comes at the MAVn_RAW_SENS
rate (often 1 Hz on a flight link), so this is for a vehicle at
rest: there the samples are bias and noise, and their integral is the
drift a compass-less vehicle would carry (Estimate). RAW_IMU carries the gyro as
whole mrad/s (0.057 deg/s, GCS_Common.cpp send_raw_imu), so a bias well under
one step is resolved only through the noise dithering it: the FC's dataflash
IMU log is the better record where there is an SD card. A turn sampled at 1 Hz
is not integrated faithfully.

It never prints a position: GPS_RAW_INT is read for fix_type and
satellites_visible only. It sends only PARAM_REQUEST_READ for COMPASS_DEC and
COMMAND_LONG REQUEST_MESSAGE(AUTOPILOT_VERSION), as sysid 250 component 190,
and no heartbeat.

Usage (needs Python 3 and pymavlink >= 2.4.50):
  python3 ap-heading-watch.py tcp:127.0.0.1:5760 [--secs 60] [--csv out.csv]
  python3 ap-heading-watch.py --selftest      no flight controller needed
"""
import argparse, csv, math, sys, time

mavutil = None  # imported in main() after the arguments, so --help and --selftest need no pymavlink


def tilt_heading(mx, my, mz, roll, pitch, dec):
    """Degrees 0-360 clockwise from true north (magnetic if dec is 0)."""
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    hx = mx * cp + my * sr * sp + mz * cr * sp
    hy = my * cr - mz * sr
    return (math.degrees(math.atan2(-hy, hx) + dec)) % 360.0


def angle_diff(a, b):
    """a - b in degrees, wrapped to -180..+180."""
    return (a - b + 180.0) % 360.0 - 180.0


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


def autopilot_versions(m, sysid, compid, window=3.0, tries=3):
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


def selftest():
    """Offline checks of the heading arithmetic against hand-computed cases."""
    n_ok = n_bad = 0

    def check(name, cond):
        nonlocal n_ok, n_bad
        print(f"{'PASS' if cond else 'FAIL'}  {name}")
        if cond:
            n_ok += 1
        else:
            n_bad += 1

    def near(a, b, tol=1e-6):
        return abs(angle_diff(a, b)) < tol

    r = math.radians
    th = tilt_heading
    # Level vehicle: heading is atan2(-Y, X). Body X forward, Y right, Z down.
    check("level, field +X: 0 deg", near(th(100, 0, 0, 0, 0, 0), 0))
    check("level, field +X, declination +10 deg: 10 deg", near(th(100, 0, 0, 0, 0, r(10)), 10))
    check("level, field +X, declination -5 deg: 355 deg", near(th(100, 0, 0, 0, 0, r(-5)), 355))
    check("level, field +Y (north off the right wing): 270 deg", near(th(0, 100, 0, 0, 0, 0), 270))
    check("level, field -X: 180 deg", near(th(-100, 0, 0, 0, 0, 0), 180))
    check("level, field -Y (north off the left wing): 90 deg", near(th(0, -100, 0, 0, 0, 0), 90))
    check("level, field 10 deg right of the nose (heading 350), declination +20: wraps to 10",
          near(th(math.cos(r(10)), math.sin(r(10)), 0, 0, 0, r(20)), 10))
    check("level, vertical field component changes nothing",
          near(th(100, 0, 400, 0, 0, 0), 0) and near(th(0, 100, -400, 0, 0, 0), 270))
    # Roll right 90 deg: body Z then points west, so a field along +Z means
    # north is west of the aircraft's frame and the nose points east.
    check("roll +90, field along body Z: 90 deg", near(th(0, 0, 100, r(90), 0, 0), 90))
    check("roll +90, field along body X: 0 deg", near(th(100, 0, 0, r(90), 0, 0), 0))
    check("roll -90, field along body Z: 270 deg", near(th(0, 0, 100, r(-90), 0, 0), 270))
    # Pitch 30 deg up, nose north, field 200 north + 400 down (mG):
    # x_b = N cos(p) - D sin(p), z_b = N sin(p) + D cos(p).
    p = r(30)
    mx, mz = 200 * math.cos(p) - 400 * math.sin(p), 200 * math.sin(p) + 400 * math.cos(p)
    check("pitch +30, nose north, tilted field: 0 deg", near(th(mx, 0, mz, 0, p, 0), 0))
    # Whole grid against an independent rotation: world field (N=200, E=0, D=400)
    # into the body frame by R^T of R = Rz(yaw) Ry(pitch) Rx(roll); the heading
    # must come back as the yaw.
    def body_field(yaw, pitch, roll, field=(200.0, 0.0, 400.0)):
        cy, sy, cp, sp, cr, sr = (math.cos(yaw), math.sin(yaw), math.cos(pitch),
                                  math.sin(pitch), math.cos(roll), math.sin(roll))
        R = [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
             [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
             [-sp, cp * sr, cp * cr]]
        return tuple(sum(R[i][j] * field[i] for i in range(3)) for j in range(3))

    worst, n = 0.0, 0
    for yaw in range(0, 360, 15):
        for pitch in (-40, -10, 0, 25, 60):
            for roll in (-70, -30, 0, 20, 80):
                bx, by, bz = body_field(r(yaw), r(pitch), r(roll))
                worst = max(worst, abs(angle_diff(th(bx, by, bz, r(roll), r(pitch), 0), yaw)))
                n += 1
    check(f"{n} attitudes (yaw x pitch x roll) recover the yaw, worst error {worst:.2e} deg",
          worst < 1e-6)
    worst = 0.0
    for dec in (-30, -5, 0, 7, 25):
        bx, by, bz = body_field(r(100), r(15), r(-20))
        worst = max(worst, abs(angle_diff(th(bx, by, bz, r(-20), r(15), r(dec)), 100 + dec)))
    check(f"declination adds to the heading at a tilted attitude, worst error {worst:.2e} deg",
          worst < 1e-6)
    check("angle_diff: 359 - 1 = -2", abs(angle_diff(359, 1) + 2) < 1e-9)
    check("angle_diff: 1 - 359 = +2", abs(angle_diff(1, 359) - 2) < 1e-9)
    check("angle_diff: 90 - 270 = -180 or +180", abs(abs(angle_diff(90, 270)) - 180) < 1e-9)
    print(f"{n_ok} passed, {n_bad} failed")
    return 0 if n_bad == 0 else 1


def main():
    global mavutil
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('device', nargs='?',
                    help='MAVLink connection, e.g. tcp:127.0.0.1:5760 or /dev/serial/by-id/...-if00')
    ap.add_argument('--baud', type=int, default=115200, help='serial baud rate (default 115200)')
    ap.add_argument('--secs', type=float, default=0, help='stop after this long (0: until Ctrl-C)')
    ap.add_argument('--csv', help='also write every line to this CSV file')
    ap.add_argument('--gyro-from', type=float, default=None, metavar='DEG',
                    help='also integrate a gyro-only yaw from RAW_IMU, starting at DEG; for a '
                         'vehicle at rest (1 Hz samples cannot follow a turn)')
    ap.add_argument('--selftest', action='store_true',
                    help='run the offline checks of the heading arithmetic and exit; '
                         'needs no flight controller and no pymavlink')
    a = ap.parse_args()
    if a.selftest:
        return selftest()
    if not a.device:
        ap.error("needs DEVICE (or --selftest)")
    try:
        from pymavlink import mavutil
    except ImportError:
        sys.exit("pymavlink not found. Try: pip install 'pymavlink>=2.4.50'")

    m = mavutil.mavlink_connection(a.device, baud=a.baud, source_system=250,
                                   source_component=190, autoreconnect=False)
    end, hb = time.time() + 30, None
    while time.time() < end:
        msg = m.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
        if (msg and msg.get_srcSystem() and msg.get_srcComponent()   # 0 is broadcast, not a vehicle
                and msg.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID
                and msg.type != mavutil.mavlink.MAV_TYPE_GCS):
            hb = msg
            break
    if hb is None:
        sys.exit("no autopilot heartbeat")
    sysid, compid = hb.get_srcSystem(), hb.get_srcComponent()
    m.target_system, m.target_component = sysid, compid

    # Two boards at one system id interleave their ATTITUDE: a compass that
    # reads wrong by the difference between them. Refuse before any row.
    uids = [bytes(v.uid2[:12]).hex().upper() for v in autopilot_versions(m, sysid, compid)]
    if uids:
        refuse_shared_sysid(uids, sysid, compid)
    else:
        print("no AUTOPILOT_VERSION: could not check that one board answers as "
              f"system {sysid} component {compid}", file=sys.stderr)

    st = {'dec': None, 'dec_t': 0.0, 'att': None, 'att_n': [], 'raw': None,
          'ekf': None, 'gps': None, 'mag_ok': None,
          'gyaw': a.gyro_from, 'graw': None}
    w = csv.writer(open(a.csv, 'w', newline='')) if a.csv else None
    cols = ['utc', 'ekf_yaw', 'mag_hdg', 'diff', 'B_mG', 'roll', 'pitch', 'dec',
            'ekf_flags', 'compass_var', 'gps_fix', 'sats', 'att_hz', 'mag_health']
    if a.gyro_from is not None:
        cols.append('gyro_yaw')
    if w:
        w.writerow(cols)
    print('  '.join(f"{c:>9}" for c in cols[1:]))
    t0, last = time.time(), 0.0
    while not a.secs or time.time() - t0 < a.secs:
        now = time.time()
        if now - st['dec_t'] > 30:
            st['dec_t'] = now
            m.mav.param_request_read_send(sysid, compid, b'COMPASS_DEC', -1)
        msg = m.recv_match(blocking=True, timeout=0.2)
        if (msg is not None and msg.get_srcSystem() == sysid
                and msg.get_srcComponent() == compid):
            t = msg.get_type()
            if t == 'ATTITUDE':
                st['att'] = msg
                st['att_n'].append(now)
            elif t == 'RAW_IMU':
                st['raw'] = msg
                if st['gyaw'] is not None and st['att'] is not None:
                    roll, pitch = st['att'].roll, st['att'].pitch
                    rate = ((msg.ygyro * math.sin(roll) + msg.zgyro * math.cos(roll))
                            / max(math.cos(pitch), 1e-3)) * 1e-3    # mrad/s -> rad/s
                    prev = st['graw']
                    if prev is not None:
                        dt = (msg.time_usec - prev[0]) * 1e-6
                        if 0 < dt < 5:
                            st['gyaw'] += math.degrees(0.5 * (rate + prev[1]) * dt)
                    st['graw'] = (msg.time_usec, rate)
            elif t == 'EKF_STATUS_REPORT':
                st['ekf'] = msg
            elif t == 'GPS_RAW_INT':
                st['gps'] = (msg.fix_type, msg.satellites_visible)
            elif t == 'SYS_STATUS':
                bit = mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_MAG
                if msg.onboard_control_sensors_present & bit:
                    st['mag_ok'] = 'ok' if msg.onboard_control_sensors_health & bit else 'UNHEALTHY'
                else:
                    st['mag_ok'] = 'absent'
            elif t == 'PARAM_VALUE' and msg.param_id == 'COMPASS_DEC':
                st['dec'] = float(msg.param_value)
        if now - last < 1.0 or st['att'] is None:
            continue
        last = now
        st['att_n'] = [x for x in st['att_n'] if now - x <= 2.0]
        att = st['att']
        yaw = math.degrees(att.yaw) % 360.0
        mag = diff = bmag = None
        r = st['raw']
        if r is not None and (r.xmag or r.ymag or r.zmag):
            mag = tilt_heading(r.xmag, r.ymag, r.zmag, att.roll, att.pitch, st['dec'] or 0.0)
            diff = angle_diff(mag, yaw)
            bmag = math.sqrt(r.xmag ** 2 + r.ymag ** 2 + r.zmag ** 2)
        ekf = st['ekf']
        gps = st['gps'] or (None, None)
        row = [time.strftime('%H:%M:%S', time.gmtime(now)),
               f"{yaw:.1f}", '-' if mag is None else f"{mag:.1f}",
               '-' if diff is None else f"{diff:+.1f}",
               '-' if bmag is None else f"{bmag:.0f}",
               f"{math.degrees(att.roll):+.1f}", f"{math.degrees(att.pitch):+.1f}",
               '-' if st['dec'] is None else f"{math.degrees(st['dec']):+.2f}",
               '-' if ekf is None else f"0x{ekf.flags:04x}",
               '-' if ekf is None else f"{ekf.compass_variance:.2f}",
               gps[0], gps[1], f"{len(st['att_n']) / 2.0:.1f}", st['mag_ok']]
        if a.gyro_from is not None:
            row.append(f"{st['gyaw'] % 360.0:.2f}")
        print('  '.join(f"{str(c):>9}" for c in row[1:]) + f"   {row[0]}Z", flush=True)
        if w:
            w.writerow(row)
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(0)
