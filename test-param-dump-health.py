#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Joe Fernandez
"""Offline test of ap-param-dump.py --health and its param-file writer.

Each case runs the tool as a subprocess against
a fake autopilot on udpin:127.0.0.1:PORT (system 1, component 1) that answers
the version, GPS and PreArm requests and PARAM_REQUEST_LIST with hostile data:

  - the health file and stdout carry no lat= or lon= field unless
    --health-position is given (field names only; no coordinate is printed)
  - STATUSTEXT with ESC, BEL and a newline, and a firmware string with ESC:
    no raw control byte reaches stdout or the file, and no extra line
  - a param name with a newline, and a NaN value: refused, exit 1, and an
    existing output file is unchanged

Run:  python3 test-param-dump-health.py [--tools DIR]   (needs pymavlink: requirements.txt)
DIR (default: this file's directory) holds ap-param-dump.py, to run the same
cases against an older revision. Prints PASS/FAIL per case and
"N passed, M failed"; exits 1 on any failure.
"""
import argparse, os, re, subprocess, sys, tempfile, threading, time

os.environ['MAVLINK20'] = '1'
from pymavlink import mavutil            # noqa: E402

AP, GCS = (1, 1), (255, 190)
LAT, LON = 123456789, 234567891          # fake; the digits must never appear in output
port_n = [14700]
TEXT = b'PreArm: a\x1b[31mb\x07c\nFORGEDLINE'
GIT = b'dea\x1bbeef'


class Peer:
    def __init__(self, port, pname=b'REALP', pval=42.0):
        self.port, self.pname, self.pval = port, pname, pval
        self.stop = False
        self.lock = threading.Lock()
        self.p = mavutil.mavlink_connection(f'udpout:127.0.0.1:{port}', source_system=1,
                                            source_component=1)

    def send(self, who, fn, *args):
        with self.lock:
            self.p.mav.srcSystem, self.p.mav.srcComponent = who
            getattr(self.p.mav, fn)(*args)

    def bound(self):
        want = f":{self.port:04X} "
        for _ in range(100):
            with open('/proc/net/udp') as f:
                if any(want in line for line in f):
                    return True
            time.sleep(0.1)
        return False

    def heartbeats(self):
        if not self.bound():
            return
        while not self.stop:
            self.send(GCS, 'heartbeat_send', mavutil.mavlink.MAV_TYPE_GCS,
                      mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 4)
            time.sleep(0.05)
            self.send(AP, 'heartbeat_send', mavutil.mavlink.MAV_TYPE_QUADROTOR,
                      mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA, 0, 0, 4)
            time.sleep(0.2)

    def serve(self):
        mv = mavutil.mavlink
        while not self.stop:
            x = self.p.recv_match(blocking=True, timeout=0.2)
            if x is None:
                continue
            t = x.get_type()
            if t == 'COMMAND_LONG':
                if x.command == mv.MAV_CMD_REQUEST_MESSAGE:
                    mid = int(x.param1)
                    if mid == 148:
                        self.send(AP, 'autopilot_version_send', 0, (4 << 24) | (8 << 16), 0, 0,
                                  5250 << 16, list(GIT), [0] * 8, [0] * 8, 0, 0, 0,
                                  list(bytes.fromhex('AABBCCDDEEFF001122334455')) + [0] * 6)
                    elif mid == 24:
                        self.send(AP, 'gps_raw_int_send', 0, 3, LAT, LON, 0, 90, 0, 0, 0, 12)
                elif x.command == mv.MAV_CMD_RUN_PREARM_CHECKS:
                    self.send(AP, 'statustext_send', 6, TEXT)
            elif t == 'PARAM_REQUEST_LIST':
                self.send(AP, 'param_value_send', self.pname, self.pval, 9, 1, 0)

    def run(self, argv, cwd):
        for fn in (self.heartbeats, self.serve):
            threading.Thread(target=fn, daemon=True).start()
        env = {k: v for k, v in os.environ.items() if k != 'MAVLINK20'}
        try:
            r = subprocess.run(argv, capture_output=True, timeout=60, cwd=cwd, env=env)
            return r.returncode, r.stdout + r.stderr
        except subprocess.TimeoutExpired as e:
            return 124, b"TIMEOUT " + str(e).encode()
        finally:
            self.stop = True
            time.sleep(0.4)


def peer(**kw):
    port_n[0] += 1
    return Peer(port_n[0], **kw)


results = []


def check(name, ok, detail=''):
    results.append(ok)
    if not ok:                           # never echo the fake coordinates
        detail = detail.replace(str(LAT), 'LAT').replace(str(LON), 'LON')
    print(f"{'PASS' if ok else 'FAIL'}  {name}" + ('' if ok else f"\n      {detail[-700:]}"))


# any control byte except \n, as raw bytes or as UTF-8 C1 (our output has no other non-ASCII)
CTRL = re.compile(rb'[\x00-\x09\x0b-\x1f\x7f-\x9f]|\xc2[\x80-\x9f]')


def clean(data):
    return not CTRL.search(data)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--tools', default=os.path.dirname(os.path.abspath(__file__)))
    a = ap.parse_args()
    tool = os.path.join(a.tools, 'ap-param-dump.py')
    py = sys.executable
    wd = tempfile.mkdtemp(prefix='param-dump-health-')
    dev = lambda p: f'udpin:127.0.0.1:{p.port}'          # noqa: E731
    txt = lambda b: b.decode('latin1')                    # noqa: E731

    def dump(p, extra):
        o, h = os.path.join(wd, 'd.param'), os.path.join(wd, 'd.health')
        if os.path.exists(h):
            os.unlink(h)
        rc, so = p.run([py, tool, o, '--device', dev(p), '--no-ftp', '--health', h,
                        '--health-secs', '2'] + extra, wd)
        hs = open(h, 'rb').read() if os.path.exists(h) else b''
        return rc, so, hs

    # position
    rc, so, hs = dump(peer(), [])
    has_gps = b'GPS fix_type=3 satellites=12' in hs
    check("health: default GPS line has fix_type and satellites", rc == 0 and has_gps,
          txt(so))
    check("health: default file and stdout hold no lat= or lon= field",
          rc == 0 and has_gps and not re.search(rb'\b(lat|lon)=', hs + so), txt(so))
    rc2, so2, hs2 = dump(peer(), ['--health-position'])
    check("health: --health-position puts lat= and lon= in the file",
          rc2 == 0 and b'lat=' in hs2 and b'lon=' in hs2, txt(so2))

    # escaping (the first run's output)
    check("health: no raw control byte in stdout or file",
          rc == 0 and bool(hs) and clean(so) and clean(hs), repr(hs) + repr(so))
    forged = lambda b: any(l.strip().startswith(b'FORGEDLINE') for l in b.split(b'\n'))  # noqa: E731
    check("health: STATUSTEXT with a newline stays one line, shown escaped",
          rc == 0 and bool(hs) and not forged(hs) and not forged(so)
          and b'\\x1b' in hs and b'\\x07' in hs and b'\\x0a' in hs, repr(hs))
    check("health: firmware string with ESC escaped",
          rc == 0 and b'git dea\\x1bbeef' in hs, repr(hs))

    # param file
    keep = os.path.join(wd, 'keep.param')
    for label, kw in (("name with a newline", dict(pname=b'BAD\nNAME')),
                      ("NaN value", dict(pval=float('nan')))):
        with open(keep, 'w') as f:
            f.write('KEEP,1\n')
        before = set(os.listdir(wd))
        p = peer(**kw)
        rc, so = p.run([py, tool, keep, '--device', dev(p), '--no-ftp'], wd)
        check(f"param file: {label} refused, existing file unchanged, nothing left behind",
              rc == 1 and b'REFUSED' in so and clean(so) and open(keep).read() == 'KEEP,1\n'
              and set(os.listdir(wd)) == before, f"rc {rc}\n{txt(so)}")

    n_ok = sum(results)
    print(f"{n_ok} passed, {len(results) - n_ok} failed")
    return 0 if n_ok == len(results) else 1


if __name__ == '__main__':
    sys.exit(main())
