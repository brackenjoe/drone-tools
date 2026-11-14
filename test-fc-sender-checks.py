#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Joe Fernandez
"""Offline test that the FC tools take MAVLink replies only from the autopilot.

Each case starts one tool as a subprocess on
udpin:127.0.0.1:PORT and plays a fake link from this process: a GCS heartbeat
first, then the autopilot's (system 1, component 1), and for every request a
FORGED reply from another component (1,2) or another system (2,1) sent BEFORE
the real one. A tool that checks the sender shows the real value; one that
does not shows the forged one.

  ap-version.py       pin is the autopilot's, not the GCS's; real git hash
  ap-param-dump.py    pin; PARAM_VALUE fallback; health snapshot (version, STATUSTEXT)
  ap-param-apply.py   real uid shown, forged --expect-uid refuses; real PARAM_VALUE;
                      armed re-tested after plan and before each PARAM_SET; git string made inert;
                      forged COMMAND_ACK ignored
  ap-mag-cal.py       forged --expect-* refuses; forged COMMAND_ACK ignored;
                      forged STATUSTEXT not shown
                      armed re-checked before ACCEPT (FC arms during the offset reads);
                      STATUSTEXT with ESC and LF printed inert
  ap-heading-watch.py forged ATTITUDE from another component not used;
                      two uids at one sysid refuses before any row
  mavftp-put.py       forged CalcFileCRC32 Ack from another system never verifies an upload
                      or a --verify-only of a missing file (tcp loopback, own fake FC)
  ap-version.py       (also) a non-hex git hash with ESC and LF printed escaped, no worktree
                      line, exit 1
  sitl-flight-test.py,
  sitl-qhover-steps.py  a forged SIM_SPEEDUP from another system on a non-SITL refuses, exit 2,
                      nothing reaches the vehicle; sitl-flight-test: a forged COMMAND_ACK 400
                      and ARMED heartbeat from another system never lead to a takeoff
                      (the fake echoes DO_SET_MODE, as the tool confirms GUIDED before arming);
                      a fake that ignores DO_SET_MODE: exit 1, "not confirmed", no arm command
  pin, one case per tool (ap-version, ap-param-apply identify, ap-heading-watch, ap-mag-cal,
  mavftp-put --verify-only, sitl-flight-test): an autopilot-looking heartbeat from (0,0) arrives
                      first; the tool pins (1,1) (requests go there, the real value shows)

Run:  python3 test-fc-sender-checks.py [--tools DIR]   (needs pymavlink: requirements.txt)
DIR (default: this file's directory) holds the eight tools, to run the same
cases against a copy of an older revision. Prints PASS/FAIL per case and
"N passed, M failed"; exits 1 on any failure.
"""
import argparse, os, re, struct, subprocess, sys, tempfile, threading, time

os.environ['MAVLINK20'] = '1'            # this process packs AUTOPILOT_VERSION.uid2
from pymavlink import mavutil            # noqa: E402

REAL_UID, FORGED_UID = 'AABBCCDDEEFF001122334455', '0102030405060708090A0B0C'
REAL_GIT, FORGED_GIT = b'deadbeef', b'ffffffff'
BOARD = 5250
AP, GCS, OTHER_COMP, OTHER_SYS = (1, 1), (255, 190), (1, 2), (2, 1)
port_n = [14600]


class Peer:
    """The fake link. Options say which forgeries to play."""

    def __init__(self, port, forge_version=None, forge_param=False, forge_ack=False,
                 real_ack=False, forge_text=False, att=False, dual=None, git=REAL_GIT,
                 arm_on_read=False, arm_on_set=False, writable=False, arm_quiet_on_set=0, mute_after_set=0,
                 arm_on_ofs=False, raw_text=None, gps=False, sim=True, forge_speedup=False,
                 forge_arm=False, zero_first=False, echo_mode=False):
        self.port = port
        self.git = git
        self.zero_first = zero_first      # an autopilot-looking heartbeat from (0,0) ahead of every real one
        self.echo_mode, self.custom = echo_mode, 0   # DO_SET_MODE's custom_mode shows in the (1,1) heartbeat
        self.arm_on_read, self.arm_on_set, self.writable = arm_on_read, arm_on_set, writable
        self.arm_quiet_on_set, self.nset = arm_quiet_on_set, 0
        self.mute_after_set = mute_after_set   # stop all heartbeats once this many PARAM_SETs are in
        self.armed = False
        self.arm_on_ofs, self.raw_text = arm_on_ofs, raw_text   # arm on a COMPASS_OFS read; AP text sent after the MAG_CAL start ack
        self.gps, self.sim = gps, sim                           # GPS fix from 1/1; answer SIM_SPEEDUP from 1/1
        self.forge_speedup, self.forge_arm = forge_speedup, forge_arm   # from (2,1): SIM_SPEEDUP; ACK 400 + armed heartbeat
        self.forge_version, self.forge_param, self.forge_ack = forge_version, forge_param, forge_ack
        self.real_ack, self.forge_text, self.att = real_ack, forge_text, att
        self.dual = dual                  # (uid, uid): answer each version request twice from 1/1
        self.seen = []                    # type of every message received
        self.targets = []                 # (target_system, target_component) of every request
        self.stop = False
        self.lock = threading.Lock()
        self.p = mavutil.mavlink_connection(f'udpout:127.0.0.1:{port}', source_system=1,
                                            source_component=1)

    def send(self, who, fn, *args):
        with self.lock:
            self.p.mav.srcSystem, self.p.mav.srcComponent = who
            getattr(self.p.mav, fn)(*args)

    def version(self, who, uid, git):
        self.send(who, 'autopilot_version_send', 0, (4 << 24) | (8 << 16), 0, 0, BOARD << 16,
                  list(git), [0] * 8, [0] * 8, 0, 0, 0, list(bytes.fromhex(uid)) + [0] * 6)

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
            if self.mute_after_set and self.nset >= self.mute_after_set:
                time.sleep(0.05)
                continue
            if self.zero_first:
                self.send((0, 0), 'heartbeat_send', mavutil.mavlink.MAV_TYPE_QUADROTOR,
                          mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA, 0, 0, 4)
            self.send(GCS, 'heartbeat_send', mavutil.mavlink.MAV_TYPE_GCS,
                      mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 4)
            time.sleep(0.05)
            self.send(AP, 'heartbeat_send', mavutil.mavlink.MAV_TYPE_QUADROTOR,
                      mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA, self.mode(), self.custom, 4)
            if self.gps:
                self.send(AP, 'gps_raw_int_send', 0, 3, 0, 0, 0, 0, 0, 0, 0, 9)
            if self.forge_speedup:
                self.send(OTHER_SYS, 'param_value_send', b'SIM_SPEEDUP', 1.0, 9, 1, 0)
            if self.forge_arm:
                self.send(OTHER_SYS, 'command_ack_send', 400, 0)
                self.send(OTHER_SYS, 'heartbeat_send', mavutil.mavlink.MAV_TYPE_QUADROTOR,
                          mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
                          mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED, 0, 4)
            time.sleep(0.2)

    def mode(self):
        return mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED if self.armed else 0

    def arm(self):
        # the armed heartbeat goes out at once, ahead of any reply that follows
        self.armed = True
        self.send(AP, 'heartbeat_send', mavutil.mavlink.MAV_TYPE_QUADROTOR,
                  mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA, self.mode(), 0, 4)

    def attitudes(self):
        # A real ATTITUDE (28.6 deg) then a forged one (171.9 deg), repeated, the
        # forged always last: a tool that takes the forged ones prints 171.9.
        # Repeated because a tool may wait seconds (version check) before it
        # starts to watch.
        self.bound()
        time.sleep(1.0)
        while not self.stop:
            self.send(AP, 'attitude_send', 0, 0.0, 0.0, 0.5, 0, 0, 0)
            self.send(OTHER_COMP, 'attitude_send', 0, 0.0, 0.0, 3.0, 0, 0, 0)
            time.sleep(0.1)

    def serve(self):
        while not self.stop:
            x = self.p.recv_match(blocking=True, timeout=0.2)
            if x is None:
                continue
            t = x.get_type()
            self.seen.append(t if t != 'COMMAND_LONG' else f'CMD{x.command}')
            if t in ('COMMAND_LONG', 'PARAM_REQUEST_READ', 'PARAM_REQUEST_LIST', 'PARAM_SET'):
                self.targets.append((x.target_system, x.target_component))
            if t == 'COMMAND_LONG':
                cmd = x.command
                if cmd == mavutil.mavlink.MAV_CMD_DO_SET_MODE and self.echo_mode:
                    self.custom = int(x.param2)
                if cmd == mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE and int(x.param1) == 148:
                    if self.forge_version:
                        for who in (OTHER_COMP, OTHER_SYS):
                            self.version(who, *self.forge_version)
                    if self.dual:
                        for uid in self.dual:
                            self.version(AP, uid, self.git)
                    else:
                        self.version(AP, REAL_UID, self.git)
                elif cmd == mavutil.mavlink.MAV_CMD_RUN_PREARM_CHECKS and self.forge_text:
                    self.send(OTHER_COMP, 'statustext_send', 6, b'PreArm: FORGED')
                    self.send(AP, 'statustext_send', 6, b'PreArm: REALTEXT')
                elif cmd in (246, 511, 42424, 42425, 42426):
                    if self.forge_ack:
                        for who in (OTHER_SYS, OTHER_COMP):
                            self.send(who, 'command_ack_send', cmd, 0)
                    if self.real_ack:
                        self.send(AP, 'command_ack_send', cmd, 0)
                        if cmd == 42424 and self.forge_text:
                            self.send(OTHER_COMP, 'statustext_send', 6, b'FORGED-TEXT')
                            self.send(AP, 'statustext_send', 6, b'REAL-TEXT')
                        if cmd == 42424 and self.raw_text:
                            self.send(AP, 'statustext_send', 6, self.raw_text)
            elif t == 'PARAM_SET':
                self.nset += 1
                if self.arm_on_set and not self.armed:
                    self.arm()
                if self.arm_quiet_on_set and self.nset == self.arm_quiet_on_set:
                    self.armed = True   # silent: only the periodic heartbeat reports it, as on a real FC
                if self.writable:
                    name = x.param_id.encode() if isinstance(x.param_id, str) else x.param_id
                    self.send(AP, 'param_value_send', name, x.param_value, 9, 1, 0)
            elif t == 'PARAM_REQUEST_READ':
                name = x.param_id.encode() if isinstance(x.param_id, str) else x.param_id
                if self.arm_on_read and not self.armed:
                    self.arm()
                if self.arm_on_ofs and not self.armed and name.startswith(b'COMPASS_OFS'):
                    self.arm()
                if name == b'SIM_SPEEDUP':
                    if self.forge_speedup:
                        self.send(OTHER_SYS, 'param_value_send', name, 1.0, 9, 1, 0)
                    if not self.sim:
                        continue
                if self.forge_param:
                    self.send(OTHER_COMP, 'param_value_send', name, 111.0, 9, 1, 0)
                self.send(AP, 'param_value_send', name, 42.0, 9, 1, 0)
            elif t == 'PARAM_REQUEST_LIST':
                if self.forge_param:
                    self.send(OTHER_COMP, 'param_value_send', b'FORGED', 111.0, 9, 1, 0)
                self.send(AP, 'param_value_send', b'REALP', 42.0, 9, 1, 0)

    def run(self, argv, timeout=60, cwd=None):
        threads = [threading.Thread(target=self.heartbeats, daemon=True),
                   threading.Thread(target=self.serve, daemon=True)]
        if self.att:
            threads.append(threading.Thread(target=self.attitudes, daemon=True))
        for th in threads:
            th.start()
        env = {k: v for k, v in os.environ.items() if k != 'MAVLINK20'}
        try:
            r = subprocess.run(argv, capture_output=True, text=True, timeout=timeout,
                               cwd=cwd, env=env)
            return r.returncode, r.stdout + r.stderr
        except subprocess.TimeoutExpired as e:
            return 124, f"TIMEOUT {e}"
        finally:
            self.stop = True
            time.sleep(0.4)


def peer(**kw):
    port_n[0] += 1
    return Peer(port_n[0], **kw)


def dev(p):
    return f'udpin:127.0.0.1:{p.port}'


def crc32(data):
    t = []
    for i in range(256):
        c = i
        for _ in range(8):
            c = (c >> 1) ^ (0xEDB88320 if c & 1 else 0)
        t.append(c)
    r = 0
    for b in data:
        r = t[(r ^ b) & 0xff] ^ (r >> 8)
    return r & 0xffffffff


def ftp_payload(seq, op, req, size, data=b'', off=0):
    return list((struct.pack('<HBBBBBBI', seq, 0, op, size, req, 0, 0, off) + data).ljust(251, b'\0'))


def ftp_run(argv, forge, honest, missing=False, secs=40, zero=False):
    """mavftp-put against a fake FC on tcp loopback (this process listens). Every
    CalcFileCRC32 (op 14) is answered by the autopilot (1,1) with `honest`, or
    with FileNotFound when `missing`; with `forge` a second Ack from (2,1) follows
    at once, carrying `forge`. Returns (exit code, output)."""
    port_n[0] += 1
    port = port_n[0]
    conn = mavutil.mavlink_connection(f'tcpin:127.0.0.1:{port}', source_system=1, source_component=1)
    mav = conn.mav
    evil = mavutil.mavlink.MAVLink(type('W', (), {'write': lambda s, b: conn.write(b), 'flush': lambda s: None})(),
                                   srcSystem=OTHER_SYS[0], srcComponent=OTHER_SYS[1])
    nobody = mavutil.mavlink.MAVLink(type('W', (), {'write': lambda s, b: conn.write(b), 'flush': lambda s: None})(),
                                     srcSystem=0, srcComponent=0)
    uid = bytes(range(1, 13)) + b'\0' * 6
    p = subprocess.Popen(argv + ['--device', f'tcp:127.0.0.1:{port}', '--retries', '1'],
                         stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                         env={k: v for k, v in os.environ.items() if k != 'MAVLINK20'})
    end, lasthb = time.time() + secs, 0
    while time.time() < end and p.poll() is None:
        if time.time() - lasthb > 1:
            if zero:       # an autopilot-looking heartbeat from (0,0), ahead of the real one
                nobody.heartbeat_send(mavutil.mavlink.MAV_TYPE_QUADROTOR,
                                      mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA, 0, 0, 4)
            mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_QUADROTOR,
                               mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA, 0, 0, 4)
            lasthb = time.time()
        m = conn.recv_match(blocking=False)
        if m is None:
            time.sleep(0.01)
            continue
        t = m.get_type()
        if t == 'COMMAND_LONG' and m.command == mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE:
            mav.autopilot_version_send(0, 0x04070000, 0, 0, BOARD << 16, b'deadbeef', b'\0' * 8,
                                       b'\0' * 8, 0, 0, 0, uid2=list(uid))
        elif t == 'FILE_TRANSFER_PROTOCOL' and m.get_srcSystem() == 250:
            seq, _, op, _, _, _, _, off = struct.unpack('<HBBBBBBI', bytes(m.payload)[:12])
            if op == 14:
                if missing:
                    mav.file_transfer_protocol_send(0, 250, 190, ftp_payload(seq + 1, 129, 14, 2, bytes([10, 0])))
                else:
                    mav.file_transfer_protocol_send(0, 250, 190, ftp_payload(seq + 1, 128, 14, 4, struct.pack('<I', honest)))
                if forge is not None:
                    evil.file_transfer_protocol_send(0, 250, 190, ftp_payload(seq + 1, 128, 14, 4, struct.pack('<I', forge)))
            else:
                mav.file_transfer_protocol_send(0, 250, 190, ftp_payload(seq + 1, 128, op, 0, off=off))
    if p.poll() is None:
        p.kill()
    out = p.communicate()[0]
    conn.close()
    return p.returncode, out


results = []


def check(name, ok, detail=''):
    results.append(ok)
    print(f"{'PASS' if ok else 'FAIL'}  {name}" + ('' if ok else f"\n      {detail[-700:]}"))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--tools', default=os.path.dirname(os.path.abspath(__file__)))
    a = ap.parse_args()
    T = lambda n: os.path.join(a.tools, n)           # noqa: E731
    py = sys.executable
    wd = tempfile.mkdtemp(prefix='fc-sender-')

    # ap-version
    p = peer(forge_version=(FORGED_UID, FORGED_GIT))
    rc, out = p.run([py, T('ap-version.py'), dev(p)], cwd=wd)
    check("ap-version: GCS heartbeat first, pin is the autopilot (requests go to 1/1)",
          bool(p.targets) and set(p.targets) == {AP}, f"targets {p.targets}\n{out}")
    check("ap-version: forged AUTOPILOT_VERSION ignored, real git hash shown",
          rc == 0 and 'git hash : deadbeef' in out and 'ffffffff' not in out, out)

    # ap-param-dump: fallback + health in one run
    p = peer(forge_param=True, forge_version=(FORGED_UID, FORGED_GIT), forge_text=True)
    o, h = os.path.join(wd, 'd.param'), os.path.join(wd, 'd.health')
    rc, out = p.run([py, T('ap-param-dump.py'), o, '--device', dev(p), '--no-ftp',
                     '--health', h, '--health-secs', '2'], cwd=wd)
    check("ap-param-dump: GCS heartbeat first, pin is the autopilot (requests go to 1/1)",
          bool(p.targets) and set(p.targets) == {AP}, f"targets {p.targets}\n{out}")
    got = open(o).read() if os.path.exists(o) else None
    check("ap-param-dump: forged PARAM_VALUE from another component not taken",
          got == 'REALP,42\n', f"{got!r}\n{out}")
    hs = open(h).read() if os.path.exists(h) else ''
    check("ap-param-dump: health snapshot ignores forged AUTOPILOT_VERSION and STATUSTEXT",
          'git deadbeef' in hs and 'ffffffff' not in hs and 'REALTEXT' in hs and 'FORGED' not in hs,
          f"{hs}\n{out}")

    # ap-param-apply
    exp = ['--expect-git', 'deadbeef', '--expect-board', str(BOARD)]
    p = peer(forge_version=(FORGED_UID, FORGED_GIT))
    rc, out = p.run([py, T('ap-param-apply.py'), 'identify', dev(p)], cwd=wd)
    check("ap-param-apply: forged AUTOPILOT_VERSION ignored, real uid shown",
          rc == 0 and f'uid {REAL_UID}' in out and FORGED_UID not in out, out)
    p = peer(forge_version=(FORGED_UID, FORGED_GIT))
    rc, out = p.run([py, T('ap-param-apply.py'), 'identify', dev(p),
                     '--expect-uid', FORGED_UID], cwd=wd)
    check("ap-param-apply: --expect-uid of the forged value refuses",
          rc != 0 and 'REFUSED, wrong flight controller' in out, out)
    pf = os.path.join(wd, 'a.param')
    open(pf, 'w').write('REALP,1\n')
    p = peer(forge_param=True)
    rc, out = p.run([py, T('ap-param-apply.py'), 'plan', dev(p), pf], cwd=wd)
    line = next((l for l in out.splitlines() if l.startswith('REALP')), '')
    check("ap-param-apply: forged PARAM_VALUE from another component not taken",
          rc == 0 and re.search(r'\b42\b', line) is not None and '111' not in line, out)
    p = peer(forge_ack=True)
    rc, out = p.run([py, T('ap-param-apply.py'), 'reboot', dev(p),
                     '--expect-uid', REAL_UID] + exp, cwd=wd)
    check("ap-param-apply: forged COMMAND_ACK from another system ignored",
          rc == 1 and 'ACK result None' in out, out)

    # ap-mag-cal
    mexp = lambda uid, git, board: ['--expect-uid', uid, '--expect-git', git,     # noqa: E731
                                    '--expect-board', str(board)]
    p = peer(forge_version=(FORGED_UID, FORGED_GIT))
    rc, out = p.run([py, T('ap-mag-cal.py'), 'accept', dev(p)]
                    + mexp(FORGED_UID, FORGED_GIT.decode(), BOARD), cwd=wd)
    check("ap-mag-cal: --expect-uid of the forged AUTOPILOT_VERSION refuses",
          rc != 0 and 'REFUSED, wrong flight controller' in out, out)
    p = peer(forge_ack=True)
    rc, out = p.run([py, T('ap-mag-cal.py'), 'accept', dev(p)]
                    + mexp(REAL_UID, 'deadbeef', BOARD), timeout=90, cwd=wd)
    check("ap-mag-cal: forged COMMAND_ACK from another system/component ignored",
          rc == 1 and 'ACK result None' in out, out)
    p = peer(real_ack=True, forge_text=True)
    rc, out = p.run([py, T('ap-mag-cal.py'), 'start', dev(p), '--timeout', '3']
                    + mexp(REAL_UID, 'deadbeef', BOARD), timeout=90, cwd=wd)
    check("ap-mag-cal: forged STATUSTEXT from another component not shown",
          'FC: REAL-TEXT' in out and 'FORGED-TEXT' not in out, out)

    # write: armed state is re-tested after plan and again before each PARAM_SET
    wf = os.path.join(wd, 'w.param')
    open(wf, 'w').write('PA,1\nPB,2\nPC,3\n')
    wexp = ['--expect-uid', REAL_UID] + exp
    npset = lambda p: p.seen.count('PARAM_SET')                                  # noqa: E731
    p = peer(writable=True)
    rc, out = p.run([py, T('ap-param-apply.py'), 'write', dev(p), wf] + wexp, cwd=wd)
    check("ap-param-apply write: disarmed control writes all three, exit 0",
          rc == 0 and npset(p) == 3 and 'wrote 3, every readback matches' in out,
          f"{p.seen}\n{out}")
    p = peer(writable=True, arm_on_read=True)
    rc, out = p.run([py, T('ap-param-apply.py'), 'write', dev(p), wf] + wexp, cwd=wd)
    check("ap-param-apply write: FC arms during plan -> REFUSED, zero PARAM_SET",
          rc != 0 and 'REFUSED: the vehicle is armed' in out and npset(p) == 0,
          f"{p.seen}\n{out}")
    p = peer(writable=True, arm_on_set=True)
    rc, out = p.run([py, T('ap-param-apply.py'), 'write', dev(p), wf] + wexp, cwd=wd)
    check("ap-param-apply write: FC arms after the first PARAM_SET -> STOPPED after 1 of 3, nonzero, no REFUSED",
          rc != 0 and 'STOPPED: the FC armed during the write after 1 of 3 writes' in out
          and 'REFUSED' not in out and npset(p) == 1
          and 'wrote 1 of 3; next was PB' in out, f"{p.seen}\n{out}")

    p = peer(writable=True, mute_after_set=3)
    rc, out = p.run([py, T('ap-param-apply.py'), 'write', dev(p), wf] + wexp, cwd=wd)
    check("ap-param-apply write: heartbeats stop after the last PARAM_SET -> WARNING no heartbeat, 3 writes, exit 1, no REFUSED",
          rc == 1 and npset(p) == 3 and 'no heartbeat arrived' in out and '3 write(s) were made' in out
          and 'REFUSED' not in out and 'every readback matches' not in out, f"{p.seen}\n{out}")

    p = peer(writable=True, arm_quiet_on_set=3)
    rc, out = p.run([py, T('ap-param-apply.py'), 'write', dev(p), wf] + wexp, cwd=wd)
    check("ap-param-apply write: FC arms silently on the last PARAM_SET -> WARNING, exit 1, not success",
          rc == 1 and npset(p) == 3 and 'WARNING: the FC is armed now' in out
          and 'every readback matches' not in out, f"{p.seen}\n{out}")

    # a git string with control bytes: none reach the terminal, no forged line
    p = peer(git=b'a\x1b\x07\r\nZZZ')
    rc, out = p.run([py, T('ap-param-apply.py'), 'identify', dev(p),
                     '--expect-git', 'deadbeef'], cwd=wd)
    bad = [c for c in out if (ord(c) < 32 and c != '\n') or 127 <= ord(c) < 160]
    check("ap-param-apply: control bytes in the git string never reach stdout/stderr",
          'REFUSED, wrong flight controller' in out and not bad
          and not any(l.startswith('ZZZ') for l in out.splitlines())
          and 'git a????ZZZ' in out, f"{bad!r}\n{out!r}")

    # two FCs behind one sysid: refuse (dual) / accept (same uid twice)
    UID_B = '99887766554433221100FFEE'
    why = "REFUSED: 2 flight controllers answer as system 1 component 1"
    both = lambda out: REAL_UID in out and UID_B in out                          # noqa: E731
    p = peer(dual=(REAL_UID, UID_B))
    rc, out = p.run([py, T('ap-param-apply.py'), 'identify', dev(p)], cwd=wd)
    check("ap-param-apply identify: two uids at one sysid refuses, prints both",
          rc != 0 and why in out and both(out), out)
    p = peer(dual=(REAL_UID, UID_B))
    rc, out = p.run([py, T('ap-param-apply.py'), 'plan', dev(p), pf], cwd=wd)
    check("ap-param-apply plan: two uids refuses; no PARAM_REQUEST_READ or PARAM_SET sent",
          rc != 0 and why in out and both(out)
          and not {'PARAM_SET', 'PARAM_REQUEST_READ'} & set(p.seen), f"{p.seen}\n{out}")
    p = peer(dual=(REAL_UID, UID_B))
    rc, out = p.run([py, T('ap-param-apply.py'), 'write', dev(p), pf,
                     '--expect-uid', REAL_UID] + exp, cwd=wd)
    check("ap-param-apply write: two uids refuses even when --expect-uid matches one; no PARAM_SET",
          rc != 0 and why in out and 'PARAM_SET' not in p.seen, f"{p.seen}\n{out}")
    p = peer(dual=(REAL_UID, UID_B), real_ack=True)
    rc, out = p.run([py, T('ap-mag-cal.py'), 'start', dev(p), '--timeout', '3']
                    + mexp(REAL_UID, 'deadbeef', BOARD), timeout=90, cwd=wd)
    check("ap-mag-cal: two uids refuses before any MAG_CAL command",
          rc != 0 and why in out and not {'CMD42424', 'CMD42425', 'CMD42426'} & set(p.seen),
          f"{p.seen}\n{out}")
    p = peer(dual=(REAL_UID, UID_B))
    o2 = os.path.join(wd, 'dual.param')
    rc, out = p.run([py, T('ap-param-dump.py'), o2, '--device', dev(p), '--timeout', '5'],
                    timeout=40, cwd=wd)
    check("ap-param-dump: two uids refuses before FTP or PARAM_REQUEST_LIST, writes nothing",
          rc != 0 and why in out and not {'FILE_TRANSFER_PROTOCOL', 'PARAM_REQUEST_LIST'} & set(p.seen)
          and not os.path.exists(o2), f"{p.seen}\n{out}")
    p = peer(dual=(REAL_UID, UID_B))
    rc, out = p.run([py, T('ap-version.py'), dev(p)], cwd=wd)
    check("ap-version: two uids prints both boards, warns, exits nonzero",
          rc != 0 and out.count('git hash : deadbeef') == 2 and both(out) and 'WARNING: 2 flight' in out,
          out)
    p = peer(dual=(REAL_UID, REAL_UID))
    rc, out = p.run([py, T('ap-param-apply.py'), 'identify', dev(p)], cwd=wd)
    check("ap-param-apply: the SAME uid answering twice is one board, accepted",
          rc == 0 and out.count('uid ' + REAL_UID) == 1 and 'REFUSED' not in out, out)
    p = peer(dual=(REAL_UID, REAL_UID))
    rc, out = p.run([py, T('ap-version.py'), dev(p)], cwd=wd)
    check("ap-version: the SAME uid answering twice is one board, exit 0",
          rc == 0 and out.count('git hash : deadbeef') == 1 and 'WARNING' not in out, out)

    # mavftp-put: a forged CalcFileCRC32 Ack from another system must not verify anything
    lf = os.path.join(wd, 'x.lua')
    data = b'print("hello flight controller")\n' * 20
    with open(lf, 'wb') as f:
        f.write(data)
    want = crc32(data)
    mp = [py, T('mavftp-put.py'), lf, '/APM/scripts/x.lua']
    ident = ['--expect-uid', bytes(range(1, 13)).hex().upper(),
             '--expect-git', 'deadbeef', '--expect-board', str(BOARD)]
    rc, out = ftp_run(mp + ident, None, want)
    check("mavftp-put upload: only the pinned FC answers the CRC, correctly -> VERIFIED, exit 0",
          rc == 0 and 'VERIFIED' in out, f"rc {rc}\n{out}")
    rc, out = ftp_run(mp + ident, want, 0)
    check("mavftp-put upload: forged CRC Ack from (2,1) after the real (wrong) one -> not VERIFIED, exit 1",
          rc == 1 and 'VERIFIED' not in out, f"rc {rc}\n{out}")
    rc, out = ftp_run(mp + ['--verify-only'], want, 0, missing=True)
    check("mavftp-put --verify-only: missing file, forged CRC from (2,1) -> no MATCH, exit 1",
          rc == 1 and 'MATCH' not in out.replace('MISMATCH', ''), f"rc {rc}\n{out}")

    # ap-mag-cal accept: the FC arms on the first COMPASS_OFS read (after the first armed check)
    p = peer(real_ack=True, arm_on_ofs=True)
    rc, out = p.run([py, T('ap-mag-cal.py'), 'accept', dev(p)]
                    + mexp(REAL_UID, 'deadbeef', BOARD), timeout=90, cwd=wd)
    check("ap-mag-cal accept: FC arms during the offset reads -> REFUSED, ACCEPT (42425) never sent",
          rc != 0 and 'REFUSED' in out and 'CMD42425' not in p.seen, f"{p.seen}\n{out}")
    # control: never arms, the same run sends ACCEPT
    p = peer(real_ack=True)
    rc, out = p.run([py, T('ap-mag-cal.py'), 'accept', dev(p)]
                    + mexp(REAL_UID, 'deadbeef', BOARD), timeout=90, cwd=wd)
    check("ap-mag-cal accept: disarmed control sends ACCEPT (42425)",
          'CMD42425' in p.seen and 'REFUSED' not in out, f"{p.seen}\n{out}")
    # ap-mag-cal start: STATUSTEXT with ESC and LF
    p = peer(real_ack=True, raw_text=b'A\x1b[31mB\nFAKE: ok')
    rc, out = p.run([py, T('ap-mag-cal.py'), 'start', dev(p), '--timeout', '3']
                    + mexp(REAL_UID, 'deadbeef', BOARD), timeout=90, cwd=wd)
    check("ap-mag-cal start: STATUSTEXT with ESC and LF -> no raw ESC, no extra line, text shown escaped",
          '\x1b' not in out and not any(l.startswith('FAKE') for l in out.splitlines())
          and 'FC: A\\x1b[31mB\\x0aFAKE: ok' in out, repr(out))

    # ap-version: a git hash that is not hex
    p = peer(git=b'a\x1b[31m\nZ')
    rc, out = p.run([py, T('ap-version.py'), dev(p)], cwd=wd)
    check("ap-version: non-hex hash with ESC and LF -> escaped, no worktree line, exit 1",
          rc == 1 and '\x1b' not in out and not any(l.startswith('Z') for l in out.splitlines())
          and 'not a hex hash' in out and 'worktree add' not in out, f"rc {rc}\n{out!r}")

    # SITL tools: a forged SIM_SPEEDUP on a vehicle that has none
    for tool in ('sitl-flight-test.py', 'sitl-qhover-steps.py'):
        p = peer(sim=False, forge_speedup=True, gps=True)
        rc, out = p.run([py, T(tool), '--device', dev(p)], timeout=40, cwd=wd)
        check(f"{tool}: forged SIM_SPEEDUP from (2,1) on a non-SITL -> REFUSED, exit 2, nothing sent to (1,1)",
              rc == 2 and 'REFUSED' in out and not {'CMD400', 'CMD176', 'SET_MODE', 'CMD22'} & set(p.seen),
              f"rc {rc} {p.seen}\n{out}")
    # sitl-flight-test: forged ACK 400 + ARMED heartbeat while the real vehicle never acks
    p = peer(forge_arm=True, gps=True, echo_mode=True)
    rc, out = p.run([py, T('sitl-flight-test.py'), '--device', dev(p)], timeout=25, cwd=wd)
    check("sitl-flight-test: forged ACK 400 and ARMED heartbeat from (2,1), real FC silent -> no takeoff (22)",
          'CMD400' in p.seen and 'CMD22' not in p.seen and 'ARMED' not in out,
          f"rc {rc} {p.seen[-12:]}\n{out}")

    # sitl-flight-test: a fake that ignores DO_SET_MODE -> no arm
    p = peer(gps=True)
    rc, out = p.run([py, T('sitl-flight-test.py'), '--device', dev(p)], timeout=40, cwd=wd)
    check("sitl-flight-test: DO_SET_MODE ignored -> exit 1, 'not confirmed', ACK 400 (arm) never sent",
          rc == 1 and 'not confirmed' in out and 'CMD400' not in p.seen, f"rc {rc} {p.seen[-12:]}\n{out}")

    # a heartbeat from (0,0) with an autopilot's type and autopilot id arrives first: the pin is (1,1)
    p = peer(zero_first=True)
    rc, out = p.run([py, T('ap-version.py'), dev(p)], cwd=wd)
    check("pin ap-version: autopilot-looking heartbeat from (0,0) first -> pins (1,1), real git hash",
          rc == 0 and bool(p.targets) and set(p.targets) == {AP} and 'git hash : deadbeef' in out,
          f"rc {rc} targets {p.targets}\n{out}")
    p = peer(zero_first=True)
    rc, out = p.run([py, T('ap-param-apply.py'), 'identify', dev(p)], cwd=wd)
    check("pin ap-param-apply identify: heartbeat from (0,0) first -> pins (1,1), real uid",
          rc == 0 and bool(p.targets) and set(p.targets) == {AP} and f'uid {REAL_UID}' in out,
          f"rc {rc} targets {p.targets}\n{out}")
    p = peer(zero_first=True, att=True)
    rc, out = p.run([py, T('ap-heading-watch.py'), dev(p), '--secs', '3'], cwd=wd)
    rows = [l.split()[0] for l in out.splitlines()[1:] if l.strip()]
    check("pin ap-heading-watch: heartbeat from (0,0) first -> pins (1,1), real yaw rows",
          bool(p.targets) and set(p.targets) == {AP} and bool(rows) and set(rows) == {'28.6'},
          f"targets {p.targets} yaw {rows}\n{out}")
    p = peer(zero_first=True, real_ack=True)
    rc, out = p.run([py, T('ap-mag-cal.py'), 'start', dev(p), '--timeout', '3']
                    + mexp(REAL_UID, 'deadbeef', BOARD), timeout=90, cwd=wd)
    check("pin ap-mag-cal: heartbeat from (0,0) first -> pins (1,1), MAG_CAL start sent to it",
          bool(p.targets) and set(p.targets) == {AP} and 'CMD42424' in p.seen and 'REFUSED' not in out,
          f"rc {rc} targets {p.targets} {p.seen}\n{out}")
    rc, out = ftp_run(mp + ['--verify-only'], None, want, zero=True)
    check("pin mavftp-put --verify-only: heartbeat from (0,0) first -> pins (1,1), CRC MATCH, exit 0",
          rc == 0 and 'MATCH' in out.replace('MISMATCH', ''), f"rc {rc}\n{out}")
    p = peer(zero_first=True, gps=True, echo_mode=True)
    rc, out = p.run([py, T('sitl-flight-test.py'), '--device', dev(p)], timeout=25, cwd=wd)
    check("pin sitl-flight-test: heartbeat from (0,0) first -> pins (1,1), every request goes to it, arm reached",
          bool(p.targets) and set(p.targets) == {AP} and 'CMD400' in p.seen,
          f"rc {rc} targets {set(p.targets)} {p.seen[-12:]}\n{out}")

    # ap-heading-watch
    p = peer(att=True)
    rc, out = p.run([py, T('ap-heading-watch.py'), dev(p), '--secs', '3'], cwd=wd)
    rows = [l.split()[0] for l in out.splitlines()[1:] if l.strip()]
    check("ap-heading-watch: forged ATTITUDE from another component not used",
          bool(rows) and set(rows) == {'28.6'}, f"yaw column {rows}\n{out}")

    p = peer(att=True, dual=(REAL_UID, UID_B))
    rc, out = p.run([py, T('ap-heading-watch.py'), dev(p), '--secs', '3'], cwd=wd)
    check("ap-heading-watch: two uids refuses before any heading row",
          rc != 0 and why in out and both(out) and '28.6' not in out, out)
    p = peer(att=True, dual=(REAL_UID, REAL_UID))
    rc, out = p.run([py, T('ap-heading-watch.py'), dev(p), '--secs', '3'], cwd=wd)
    rows = [l.split()[0] for l in out.splitlines()[1:] if l.strip()]
    check("ap-heading-watch: the SAME uid answering twice is one board, accepted",
          rc == 0 and 'REFUSED' not in out and bool(rows) and set(rows) == {'28.6'}, out)

    n_ok = sum(results)
    print(f"{n_ok} passed, {len(results) - n_ok} failed")
    return 0 if n_ok == len(results) else 1


if __name__ == '__main__':
    sys.exit(main())
