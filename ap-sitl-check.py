#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Joe Fernandez
"""One-command ArduPilot SITL check of the ArduPilot tools in this directory:
ap-ftp-get.py, ap-param-dump.py, ap-upgrade-verify.py, mavlink-listen.py and
ap-param-apply.py. It starts one ArduCopter 4.7.1 SITL, puts a slow,
deep-buffered link and a fast link in front of it (mavlink-slowlink.py, beside
this script), runs each tool as a subprocess and counts what it did against
what the SITL holds. For the two-boards cases it starts a second SITL.

    python3 ap-sitl-check.py [--keep] [--tools DIR] [--map PATH]

Run it with a python that has pymavlink >= 2.4.50.

Output is one line per check, `ok ...` or `FAIL ...` with the evidence, then
`passed N, failed M`, the wall time, and an exit status equal to the number of
failed checks (0 = all passed, capped at 100). SKIP lines are printed and not
counted. Exit 101 means it could not run, with a `cannot run:` line and no
`passed` line: the SITL binary or its defaults file is missing, something
already holds one of the ports below, the SITL's git hash is not dbe79216
(ArduPilot's Copter-4.7.1 tag), or a SITL died during a case. Everything it started is stopped by process id on the
way out (also on SIGINT and SIGTERM), and it checks the ports are free again.

Environment:
    AP_SITL       the arducopter SITL binary (default: ardupilot/build/sitl/bin/arducopter
                  beside this script)
    AP_DEFAULTS   the copter.parm defaults file (default:
                  <binary dir>/../../../Tools/autotest/default_params/copter.parm,
                  the ArduPilot tree's own)

Options:
    --tools DIR   run the tools from DIR instead of this script's directory (all
                  of them: ap-param-dump.py imports ap-ftp-get.py from beside
                  itself). The check and mavlink-slowlink.py always load from
                  this script's directory.
    --map PATH    the published rename map copter-4.6.3-to-4.7.1.json (default:
                  maps/copter-4.6.3-to-4.7.1.json beside this script). Not found:
                  the two rows that use it print SKIP.
    --keep        keep the scratch directory (SITL log and eeprom, the dumps,
                  the health files) and say where; otherwise it is removed on
                  exit. The SITL's home is a public test field, so the health
                  files hold no real position.

Ports it needs free: TCP 5820-5823 (the SITL, instance 6), 5830-5833 (the second
SITL, instance 7, for the T cases) and 15820-15829 (the links).

The rig:
  * SITL A: arducopter --model quad --speedup 1 -I 6 -w, home at the public
    CMAC field, defaults copter.parm plus DISARM_DELAY 0 and FS_THR_ENABLE 0,
    working directory scratch/sitl-a. MAVFTP's root is that directory, so
    APM/LOGS/ is planted there before the start: a 300,000 byte 00000001.BIN
    of random bytes, LASTLOG.TXT, and 34 small .BIN files, 36 entries that
    cannot fit one 239 byte ListDirectory reply. A 150,000 byte APM/F5STALE.BIN
    is planted beside it for F5.
  * slow link 15820 -> SERIAL0 (5820), 1800 B/s and 0.15 s each way, a
    slower link than a real one. Its upstream connection opens SERIAL0,
    which the SITL waits for at boot.
  * fast link 15821 -> SERIAL2 (5823), no rate, no latency; its `drop` hook
    makes the loss cases.
  * truth: pymavlink straight to SERIAL1 (5822) as 245/191, never through a
    link: parameter reads, param_count, the GPS fix, the raw PARAM_SETs, and
    arming for the P cases (MAV_CMD_COMPONENT_ARM_DISARM with param2 21196,
    GCS.h magic_force_arm_disarm_value, which skips the prearm checks).
  * before any dump it waits for a 3D fix and HOME_POSITION: the firmware
    stores home as mission item 0, which moves MIS_TOTAL.

Cases (a bullet is one or more counted checks):
  F1  ap-ftp-get.py --list /APM/LOGS over the slow link: every planted name and
      size listed, and at least two ListDirectory requests at distinct offsets
  F2  fetch 00000001.BIN over the fast link: bytes equal, CRC reported verified
  F3  gap refill: one burst reply at 20*239 dropped, the tool re-asks from it
  F4  --last-log DIR
  F5  with the slow link raised to 18000 B/s (about a real USB/IP link's
      rate, 17.7 to 18.9 KiB/s measured), a fetch of a 150,000 byte file is
      killed once the slow link holds 5000 bytes, then --list at once: a backlog
      of >= 100,000 bytes sits ahead of it, it drains the stale replies and the
      listing is complete
  D1  ap-param-dump.py --defaults over the slow link: decoded N params
  D2  the same, killed mid-transfer, rerun at once
  D3  every MAVFTP reply dropped: falls back to PARAM_REQUEST_LIST, values only,
      no defaults file
  D4  one PARAM_VALUE dropped under --no-ftp: exit 1, INCOMPLETE, file written
  D5  --health: GPS fix line, no position unless --health-position (only counts
      are printed, never the file or the tool's output)
  U   ap-upgrade-verify.py on SITL dumps before and after one change, six
      runs: "nothing unexplained", UNEXPLAINED, carried across and MISMATCH,
      with and without the published map
  L   mavlink-listen.py: the autopilot's heartbeat, and nothing sent
  P   ap-param-apply.py on SITL A, pinned by the uid, git and board that
      `identify` prints. The 24 names SERVO9..16_MIN/_MAX/_TRIM (function 0 on
      a quad), put back to their start values on truth before each case, and
      every case counts the PARAM_SET frames the link carried.
        P1 identify; P2 plan changes nothing and sends no PARAM_SET
        P3 write over the slow link: one name, one PARAM_SET, one ok row in --log
        P4 a never-write.txt beside the file: refused before anything connects
        P5 a name 4.7.1 lacks (RTL_ALT, now RTL_ALT_M): refused
        P6 write with a wrong uid, git, board and with no pins: refused
        P7 armed once the tool printed `armed: no`: REFUSED, nothing written
        P8 armed after the 3rd ok, slow link: STOPPED after N of 24, the link,
           the truth and the --log all agree on N
        P9 the same on the fast link: exit 1 (STOPPED or WARNING), never 0, and
           the names that changed are the PARAM_SETs the link carried. After
           SITL A stops, its dataflash log says how many landed armed.
      `reboot` is not exercised: SITL execv()s itself, which would drop the
      links' upstream sockets.
  T   two boards at system 1: SITL B (-I 7) beside A, one link (15822) whose
      frames go to both, so both answer as 1/1, as a mavlink-router
      in front of two boards at one system id does. identify, plan, write (A's pins, --log) and ap-param-dump.py
      all refuse; the boards (SERVO9_TRIM 1511 on A, 1522 on B) are unchanged;
      mavlink-listen.py still works and sends nothing.
"""
import argparse
import atexit
import importlib.util
import json
import os
import random
import re
import shutil
import signal
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time
import unicodedata

HERE = os.path.dirname(os.path.abspath(__file__))
GIT = 'dbe79216'                      # ArduCopter 4.7.1
HOME_LOC = '-35.363261,149.165230,584,353'    # CMAC, ArduPilot's default SITL field
PORT_RANGES = [(5820, 5823), (5830, 5833), (15820, 15829)]
SLOW_RATE, SLOW_LATENCY = 1800, 0.15
CH = 239                              # data bytes in one FILE_TRANSFER_PROTOCOL reply
OP_LIST, OP_BURST, ACK = 3, 15, 128
MSG_PARAM_REQUEST_READ, MSG_PARAM_SET = 20, 23
PNAMES = [f'SERVO{n}_{f}' for n in range(9, 17) for f in ('MIN', 'MAX', 'TRIM')]
TWO_BOARDS_PORT = 15822


def _esc(c):
    o = ord(c)
    if o < 0x20 or 0x7f <= o <= 0x9f:
        return f"\\x{o:02x}"
    if unicodedata.category(c) in ('Cf', 'Zl', 'Zp'):
        return f"\\u{o:04x}"
    return c


def inert(s):
    """s with every C0 control, DEL and C1 control as \\xNN and every format (Cf),
    line and paragraph separator character as \\uXXXX, so text from the vehicle
    cannot move the cursor, recolour the terminal, reorder what is shown or start
    a new line."""
    return ''.join(map(_esc, str(s)))


class StartError(Exception):
    """The check cannot start: exit status 2."""


# ---------------------------------------------------------------- reporting

class Report:
    def __init__(self):
        self.passed = self.failed = self.skipped = 0

    def check(self, name, cond, evidence='', detail=None):
        if cond:
            self.passed += 1
            print(f"ok   {name}" + (f": {inert(evidence)}" if evidence else ''), flush=True)
        else:
            self.failed += 1
            print(f"FAIL {name}" + (f": {inert(evidence)}" if evidence else ''), flush=True)
            for line in (detail or []):
                print(f"       | {inert(line[:200])}", flush=True)
        return bool(cond)

    def skip(self, name, why):
        self.skipped += 1
        print(f"SKIP {inert(name)}: {inert(why)}", flush=True)

    def note(self, text):
        print(f"     {inert(text)}", flush=True)


def tail(text, n=8):
    return text.rstrip().splitlines()[-n:]


# ------------------------------------------------------------ process control

class Procs:
    """Every process this check starts, stopped by its own pid."""

    def __init__(self):
        self.procs, self.links, self.conns = [], [], []
        self.stopped = False

    def add_proc(self, p):
        self.procs.append(p)
        return p

    def stop_proc(self, p):
        if p.poll() is None:
            try:
                p.terminate()
                p.wait(5)
            except subprocess.TimeoutExpired:
                p.kill()
                try:
                    p.wait(5)
                except subprocess.TimeoutExpired:
                    pass
            except OSError:
                pass

    def stop_all(self):
        if self.stopped:
            return
        self.stopped = True
        for p in reversed(self.procs):
            self.stop_proc(p)
        for c in self.conns:
            try:
                c.close()
            except Exception:  # noqa: BLE001
                pass
        for link in self.links:
            try:
                link.stop()
            except Exception:  # noqa: BLE001
                pass
        # a SITL started above is ours; wait for it to be gone before the
        # ports are checked
        for p in self.procs:
            if p.poll() is None:
                try:
                    p.kill()
                    p.wait(5)
                except Exception:  # noqa: BLE001
                    pass


PROCS = Procs()
atexit.register(PROCS.stop_all)


def _on_signal(signum, _frame):
    raise SystemExit(128 + signum)


# ------------------------------------------------------------------- helpers

def port_busy(port):
    s = socket.socket()
    s.settimeout(1)
    try:
        return s.connect_ex(('127.0.0.1', port)) == 0
    finally:
        s.close()


def busy_ports():
    return [p for lo, hi in PORT_RANGES for p in range(lo, hi + 1) if port_busy(p)]


def load_module(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


def ap_crc32(data):
    """ArduPilot crc_crc32: reflected 0xEDB88320, init 0, no final xor. This is
    the check's own copy, so the CRC the tool reports is compared with
    something the tool did not compute."""
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ (0xEDB88320 if crc & 1 else 0)
    return crc


def count_lines(path):
    with open(path, 'rb') as f:
        return f.read().count(b'\n')


# --------------------------------------------------------------------- SITL

class Sitl:
    """One ArduCopter SITL, started with Popen(start_new_session=True) so the pid
    held here is the SITL's own. The instance number sets the ports: SERIAL0 is
    5760 + 10*I, SERIAL1 +2, SERIAL2 +3. Make a second one with another
    instance number and workdir."""

    def __init__(self, binary, defaults, instance, workdir, scratch, home=HOME_LOC,
                 extra=('DISARM_DELAY 0', 'FS_THR_ENABLE 0'), name='A'):
        self.binary, self.defaults, self.instance = binary, defaults, instance
        self.workdir, self.scratch, self.home, self.name = workdir, scratch, home, name
        self.extra = list(extra)
        self.s0 = 5760 + 10 * instance
        self.s1, self.s2 = self.s0 + 2, self.s0 + 3
        self.proc = None

    def start(self):
        os.makedirs(self.workdir, exist_ok=True)
        extra = os.path.join(self.scratch, f'extra-{self.name}.parm')
        with open(extra, 'w') as f:
            f.write('\n'.join(self.extra) + '\n')
        log = open(os.path.join(self.workdir, 'sitl.log'), 'w')
        self.proc = PROCS.add_proc(subprocess.Popen(
            [self.binary, '--model', 'quad', '--speedup', '1', '-I', str(self.instance), '-w',
             '--home', self.home, '--defaults', f'{self.defaults},{extra}'],
            cwd=self.workdir, stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
        log.close()
        return self.proc

    def alive(self):
        return self.proc is not None and self.proc.poll() is None

    def stop(self):
        if self.proc is not None:
            PROCS.stop_proc(self.proc)


def start_link(sl, port, upstream, secs=60, **kw):
    """A SlowLink in front of `upstream` (one HOST:PORT, or a list of them),
    retried while the SITL is still starting and has not bound the port yet.
    Each try is a fresh object, since a failed start() cannot be repeated."""
    end = time.time() + secs
    last = None
    ups = [upstream] if isinstance(upstream, str) else list(upstream)
    while time.time() < end:
        link = sl.SlowLink(port, ups, **kw)
        try:
            link.start()
        except sl.SlowLinkError as e:
            last = e
            time.sleep(0.3)
            continue
        PROCS.links.append(link)
        return link
    raise StartError(f"link {port} -> {ups}: {last}")


# -------------------------------------------------------------------- truth

class Truth:
    """pymavlink straight to a SITL's SERIAL1, as 245/191: never through a link."""

    def __init__(self, mavutil, port, secs=60):
        self.mavutil = mavutil
        self.MAV = mavutil.mavlink
        end = time.time() + secs
        err = None
        while time.time() < end:
            try:
                self.m = mavutil.mavlink_connection(f'tcp:127.0.0.1:{port}', source_system=245,
                                                    source_component=191, autoreconnect=False)
            except OSError as e:
                err = e
                time.sleep(0.3)
                continue
            PROCS.conns.append(self.m)
            hb = self.m.recv_match(type='HEARTBEAT', blocking=True, timeout=30)
            if hb is None:
                raise StartError(f"no heartbeat on truth port {port}")
            self.sysid, self.compid = hb.get_srcSystem(), hb.get_srcComponent()
            return
        raise StartError(f"truth port {port}: {err}")

    def request(self, msgid, typ, tries=5):
        for _ in range(tries):
            self.m.mav.command_long_send(self.sysid, self.compid, self.MAV.MAV_CMD_REQUEST_MESSAGE,
                                         0, msgid, 0, 0, 0, 0, 0, 0)
            msg = self.m.recv_match(type=typ, blocking=True, timeout=2)
            if msg is not None:
                return msg
        raise RuntimeError(f"no {typ} on the truth port")

    def read(self, name):
        """(value, param_count) of one parameter."""
        for _ in range(5):
            self.m.mav.param_request_read_send(self.sysid, self.compid, name.encode(), -1)
            end = time.time() + 1.5
            while time.time() < end:
                msg = self.m.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.3)
                if msg and msg.param_id == name:
                    return float(msg.param_value), int(msg.param_count)
        raise RuntimeError(f"cannot read {name} on the truth port")

    def param_count(self):
        return self.read('SERVO9_TRIM')[1]

    def set_raw(self, name, value):
        """One raw PARAM_SET, retried until the read-back agrees."""
        for _ in range(5):
            self.m.mav.param_set_send(self.sysid, self.compid, name.encode(), value,
                                      self.MAV.MAV_PARAM_TYPE_REAL32)
            time.sleep(0.5)
            if self.read(name)[0] == value:
                return True
        return False

    def pump(self, secs):
        end = time.time() + secs
        while time.time() < end:
            self.m.recv_match(blocking=True, timeout=0.2)

    def drain(self):
        while self.m.recv_match(blocking=False) is not None:
            pass

    def version(self):
        """uid (24 hex digits), git hash and board id from AUTOPILOT_VERSION."""
        v = self.request(self.MAV.MAVLINK_MSG_ID_AUTOPILOT_VERSION, 'AUTOPILOT_VERSION')
        return {'uid': bytes(v.uid2[:12]).hex().upper(),
                'git': bytes(v.flight_custom_version).decode('ascii', 'ignore').strip('\x00').strip(),
                'board': v.board_version >> 16}

    def arm_send(self, on):
        """MAV_CMD_COMPONENT_ARM_DISARM, param2 21196 (skip the prearm checks;
        on a disarm, also accepted in the air). Does not wait."""
        self.m.mav.command_long_send(self.sysid, self.compid, self.MAV.MAV_CMD_COMPONENT_ARM_DISARM,
                                     0, 1 if on else 0, 21196, 0, 0, 0, 0, 0)

    def wait_armed(self, want, secs=8):
        """True once a heartbeat from the pinned autopilot has the armed bit
        equal to `want`. Older heartbeats in the socket are read past."""
        end = time.time() + secs
        while time.time() < end:
            hb = self.m.recv_match(type='HEARTBEAT', blocking=True, timeout=0.5)
            if (hb is not None and hb.get_srcSystem() == self.sysid and hb.get_srcComponent() == self.compid
                    and bool(hb.base_mode & self.MAV.MAV_MODE_FLAG_SAFETY_ARMED) == want):
                return True
        return False

    def wait_fix(self, secs=120):
        """A 3D fix and HOME_POSITION. Returns (fix_type, home_seen)."""
        end, g, home = time.time() + secs, None, False
        while time.time() < end:
            g = self.request(self.MAV.MAVLINK_MSG_ID_GPS_RAW_INT, 'GPS_RAW_INT')
            if g.fix_type >= 3:
                self.m.mav.command_long_send(self.sysid, self.compid, self.MAV.MAV_CMD_REQUEST_MESSAGE,
                                             0, self.MAV.MAVLINK_MSG_ID_HOME_POSITION, 0, 0, 0, 0, 0, 0)
                if self.m.recv_match(type='HOME_POSITION', blocking=True, timeout=2) is not None:
                    home = True
                    break
            time.sleep(2)
        self.pump(5)                      # let the mission write settle
        return (g.fix_type if g else None), home


# ---------------------------------------------------------------- the rig

class Rig:
    """What the cases share: the links, the truth connection, the tool runner."""

    def __init__(self, rep, sl, tools, work, mavutil, map_path):
        self.rep, self.sl, self.tools, self.work = rep, sl, tools, work
        self.mavutil, self.map_path = mavutil, map_path
        self.slow = self.fast = self.truth = None
        self.planted = {}
        self.log_bytes = b''
        self.n_params = None
        self.pstart = None              # the 24 SERVO names' start values, read by P1
        self.truth_id = self.uid = self.board = None   # A's AUTOPILOT_VERSION; uid and board are the pins

    def device(self, link):
        return f'tcp:127.0.0.1:{link.listen_port}'

    def path(self, *parts):
        return os.path.join(self.work, *parts)

    def tool_argv(self, name, args):
        return [sys.executable, os.path.join(self.tools, name)] + [str(a) for a in args]

    def run(self, name, args, timeout=300):
        """Run a tool; (rc, text, seconds). rc is None if it had to be killed at
        the timeout."""
        t0 = time.time()
        env = dict(os.environ, PYTHONUNBUFFERED='1')
        p = PROCS.add_proc(subprocess.Popen(self.tool_argv(name, args), cwd=self.work, env=env,
                                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                                            errors='replace'))
        try:
            text, _ = p.communicate(timeout=timeout)
            rc = p.returncode
        except subprocess.TimeoutExpired:
            p.kill()
            text, _ = p.communicate()
            rc = None
        return rc, text or '', time.time() - t0

    def run_killed(self, name, args, link, above=5000, within=150):
        """Start a tool and kill it, by its own pid, once `link` holds more than
        `above` bytes. A fixed sleep missed the burst: it starts about 10 s in.
        Returns (buffered at the kill or None if never reached, seconds)."""
        t0 = time.time()
        env = dict(os.environ, PYTHONUNBUFFERED='1')
        out = open(self.path('killed.out'), 'w')
        p = PROCS.add_proc(subprocess.Popen(self.tool_argv(name, args), cwd=self.work, env=env,
                                            stdout=out, stderr=subprocess.STDOUT))
        out.close()
        hit = None
        while time.time() - t0 < within and p.poll() is None:
            if link.buffered > above:
                hit = link.buffered
                break
            time.sleep(0.05)
        if p.poll() is None:
            p.kill()
        p.wait()
        return hit, time.time() - t0

    def run_live(self, name, args, trigger, timeout=300):
        """Run a tool and read its lines as they come, reading the truth port
        meanwhile. trigger(line, n_ok) is asked about every line; the first
        time it returns True the truth port arms the vehicle. n_ok counts the
        lines ending in ok or FAILED, the write loop's rows. Returns (rc, text,
        seconds, the (line number, line) that fired it or None)."""
        t0 = time.time()
        env = dict(os.environ, PYTHONUNBUFFERED='1')
        p = PROCS.add_proc(subprocess.Popen(self.tool_argv(name, args), cwd=self.work, env=env,
                                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                                            errors='replace'))
        q, lock = [], threading.Lock()

        def reader():
            for raw in p.stdout:
                with lock:
                    q.append(raw.rstrip('\n'))
            with lock:
                q.append(None)
        threading.Thread(target=reader, daemon=True).start()
        lines, n_ok, fired, eof = [], 0, None, False
        while not eof:
            if time.time() - t0 > timeout:
                p.kill()
                break
            self.truth.drain()
            with lock:
                batch, q[:] = list(q), []
            if not batch:
                time.sleep(0.02)
            for line in batch:
                if line is None:
                    eof = True
                    break
                lines.append(line)
                if re.search(r'\s(ok|FAILED)$', line):
                    n_ok += 1
                if fired is None and trigger(line.strip(), n_ok):
                    self.truth.arm_send(True)
                    fired = (len(lines), line)
        rc = p.wait()
        return rc, '\n'.join(lines), time.time() - t0, fired

    def settle(self, link, secs=60):
        """Drain a link's backlog through a throwaway client, so a threshold on
        `buffered` measures the burst and not the heartbeats of the idle time."""
        s = socket.create_connection(('127.0.0.1', link.listen_port), timeout=3)
        end = time.time() + secs
        s.settimeout(0.3)
        try:
            while time.time() < end and link.buffered > 200:
                try:
                    if not s.recv(65536):
                        break
                except socket.timeout:
                    pass
        finally:
            s.close()
        time.sleep(0.5)
        return link.buffered

    def link_clean(self, group, links=None):
        errs = [e for link in (links or [self.slow, self.fast]) for e in link.errors]
        self.rep.check(f'{group} links: no drop-hook errors', not errs, '; '.join(errs[:3]))

    def end_case(self):
        self.slow.drop = None
        self.fast.drop = None


# -------------------------------------------------------------------- F cases

def parse_listing(text):
    got = {}
    for line in text.splitlines():
        m = re.match(r'file (\S+)  (\d+)$', line)
        if m:
            got[m.group(1)] = int(m.group(2))
    return got


def list_offsets(link):
    """Offsets of the ListDirectory requests the tool sent through `link`."""
    offs = []
    for f in link.sent_up:
        if f.msgid == 110:
            x = rig_ftp_fields(f.payload)
            if x['opcode'] == OP_LIST:
                offs.append(x['offset'])
    return offs


_FTP_FIELDS = None      # mavlink-slowlink's ftp_fields, set once the module is loaded


def rig_ftp_fields(payload):
    return _FTP_FIELDS(payload)


def case_f1(rig):
    rep = rig.rep
    rig.slow.reset_records()
    rc, text, secs = rig.run('ap-ftp-get.py', ['--device', rig.device(rig.slow), '--list', '/APM/LOGS'], 300)
    got = parse_listing(text)
    want = rig.planted
    missing = [n for n, s in want.items() if got.get(n) != s]
    rep.check('F1 --list /APM/LOGS (slow link): exit 0', rc == 0, f'exit {rc}, {secs:.1f} s', tail(text))
    rep.check('F1 every planted name with its size is listed', not missing,
              f'{len(want) - len(missing)} of {len(want)} entries match, {len(got)} files listed'
              + (f'; wrong or absent: {missing[:4]}' if missing else ''), tail(text, 4))
    offs = list_offsets(rig.slow)
    distinct = list(dict.fromkeys(offs))
    rep.check('F1 ListDirectory paged: >= 2 requests at distinct offsets, the later ones nonzero',
              len(distinct) >= 2 and all(o > 0 for o in distinct[1:]),
              f'{len(offs)} requests, distinct offsets {distinct}')


def case_f2(rig):
    rep = rig.rep
    out = rig.path('f2.bin')
    rig.fast.reset_records()
    rc, text, secs = rig.run('ap-ftp-get.py', ['--device', rig.device(rig.fast), '/APM/LOGS/00000001.BIN', out], 300)
    same = os.path.exists(out) and open(out, 'rb').read() == rig.log_bytes
    crc = ap_crc32(rig.log_bytes)
    rep.check('F2 fetch 00000001.BIN (fast link): exit 0', rc == 0, f'exit {rc}, {secs:.1f} s', tail(text))
    rep.check('F2 the local file equals the planted bytes', same,
              f'{len(rig.log_bytes)} bytes' if same else 'differs or missing')
    rep.check('F2 output reports the CRC verified, and it is the planted bytes\' CRC',
              f'VERIFIED: CRC {hex(crc)} matches the board' in text, f'CRC {hex(crc)}', tail(text, 3))


def case_f3(rig):
    rep = rig.rep
    out = rig.path('f3.bin')
    gap = 20 * CH
    state = {'done': False}

    def drop(direction, frame):
        if state['done'] or direction != 'down' or frame.msgid != 110:
            return False
        x = rig_ftp_fields(frame.payload)
        if x['opcode'] == ACK and x['req_opcode'] == OP_BURST and x['offset'] == gap:
            state['done'] = True
            return True
        return False
    rig.fast.reset_records()
    rig.fast.drop = drop
    try:
        rc, text, secs = rig.run('ap-ftp-get.py', ['--device', rig.device(rig.fast), '/APM/LOGS/00000001.BIN', out], 150)
    finally:
        rig.fast.drop = None
    same = os.path.exists(out) and open(out, 'rb').read() == rig.log_bytes
    dropped = rig.fast.dropped
    t_drop = dropped[0][1].t if dropped else None
    reasks = []
    for f in rig.fast.sent_up:
        if f.msgid == 110:
            x = rig_ftp_fields(f.payload)
            if x['opcode'] == OP_BURST and x['offset'] == gap and t_drop is not None and f.t > t_drop:
                reasks.append(f.t - t_drop)
    rep.check('F3 gap refill: exit 0', rc == 0, f'exit {rc}, {secs:.1f} s', tail(text))
    rep.check('F3 the local file equals the planted bytes', same)
    rep.check('F3 exactly one frame dropped, the burst reply at 20*239',
              len(dropped) == 1 and dropped[0][0] == 'down', f'{len(dropped)} dropped')
    rep.check('F3 a BurstReadFile request at 20*239 was sent after the drop', len(reasks) >= 1,
              f'{len(reasks)} re-asks, the first {reasks[0]:.1f} s after the drop' if reasks else 'none')


def case_f4(rig):
    rep = rig.rep
    d = rig.path('f4')
    rig.fast.reset_records()
    rc, text, secs = rig.run('ap-ftp-get.py', ['--device', rig.device(rig.fast), '--last-log', d], 300)
    p = os.path.join(d, '00000001.BIN')
    rep.check('F4 --last-log (fast link): exit 0', rc == 0, f'exit {rc}, {secs:.1f} s', tail(text))
    rep.check('F4 DIR/00000001.BIN equals the planted bytes',
              os.path.exists(p) and open(p, 'rb').read() == rig.log_bytes)


F5_RATE = 18000        # B/s: the real USB/IP link measured 17.7 to 18.9 KiB/s
F5_STALE_MIN = 100000  # bytes that must sit in the link when the --list starts


def case_f5(rig):
    """Stale replies after a kill, at the real link's rate. The slow link is
    set to 18000 B/s for this case (latency unchanged) and put back to 1800 in
    a finally. SITL's serial port reports flow control, so GCS_FTP.cpp skips
    the burst pacing delay and a killed fetch leaves the whole burst in the
    link's buffer at once: a 150,000 byte file is ~165 KB of frames, about 9 s
    at 18000 B/s. At 1800 B/s the same burst of a 300 KB file took ~190 s and
    ap-ftp-get.py gave up after 25 s waiting for a heartbeat. The file is
    APM/F5STALE.BIN, outside APM/LOGS so F1's listing does not change."""
    rep = rig.rep
    out = rig.path('f5.bin')
    rig.slow.rate = float(F5_RATE)
    try:
        left = rig.settle(rig.slow)
        rig.slow.reset_records()
        hit, secs = rig.run_killed('ap-ftp-get.py', ['--device', rig.device(rig.slow), '/APM/F5STALE.BIN', out],
                                   rig.slow)
        rep.check('F5 the fetch was killed with the slow link holding > 5000 bytes', hit is not None,
                  f'{hit} bytes buffered after {secs:.1f} s (settled to {left} before)' if hit is not None
                  else f'never above 5000 in {secs:.0f} s')
        # the killed tool's burst may still be arriving from the SITL: nothing
        # drains the link now, so wait (at most 3 s) until its size stops growing
        at_rerun, end = -1, time.time() + 3
        while rig.slow.buffered != at_rerun and time.time() < end:
            at_rerun = rig.slow.buffered
            time.sleep(0.3)
        rig.slow.reset_records()
        rc, text, secs = rig.run('ap-ftp-get.py', ['--device', rig.device(rig.slow), '--list', '/APM/LOGS'], 420)
    finally:
        rig.slow.rate = float(SLOW_RATE)
    m = re.search(r'drained (\d+) stale replies', text)
    got = parse_listing(text)
    want = rig.planted
    missing = [n for n, s in want.items() if got.get(n) != s]
    rep.note(f'F5 buffered {at_rerun} bytes when the --list started; it took {secs:.1f} s')
    rep.check(f'F5 a stale backlog of >= {F5_STALE_MIN} bytes sat in the link when the --list started',
              at_rerun >= F5_STALE_MIN, f'{at_rerun} bytes')
    rep.check('F5 --list straight after the kill: exit 0', rc == 0, f'exit {rc}, {secs:.1f} s', tail(text))
    rep.check('F5 it prints a "drained N stale replies" line', bool(m), m.group(0) if m else 'no such line',
              tail(text, 4))
    rep.check('F5 the listing is complete', not missing,
              f'{len(want) - len(missing)} of {len(want)} entries match', tail(text, 4))


# -------------------------------------------------------------------- D cases

def decoded_line(text):
    m = re.search(r'decoded (\d+) params \(header says (\d+)\)', text)
    return (int(m.group(1)), int(m.group(2))) if m else None


def case_d1(rig):
    rep = rig.rep
    n = rig.n_params = rig.truth.param_count()
    vals, dfl = rig.path('d1.parm'), rig.path('d1.defaults')
    rig.slow.reset_records()
    rc, text, secs = rig.run('ap-param-dump.py', ['--device', rig.device(rig.slow), '--defaults', dfl, vals], 400)
    dec = decoded_line(text)
    rep.check('D1 MAVFTP dump with --defaults (slow link): exit 0', rc == 0, f'exit {rc}', tail(text))
    rep.check(f'D1 prints "decoded {n} params (header says {n})"', dec == (n, n),
              f'{dec}, truth param_count {n}', tail(text, 4))
    rep.check(f'D1 both files have exactly {n} lines',
              os.path.exists(vals) and os.path.exists(dfl) and count_lines(vals) == n and count_lines(dfl) == n,
              f'{count_lines(vals) if os.path.exists(vals) else None} and '
              f'{count_lines(dfl) if os.path.exists(dfl) else None} lines')
    rep.note(f'D1 took {secs:.1f} s over the slow link')


def case_d2(rig):
    rep = rig.rep
    n = rig.truth.param_count()
    vals, dfl = rig.path('d2.parm'), rig.path('d2.defaults')
    left = rig.settle(rig.slow)
    rig.slow.reset_records()
    hit, secs = rig.run_killed('ap-param-dump.py', ['--device', rig.device(rig.slow), '--defaults', dfl, vals],
                               rig.slow)
    rep.check('D2 the dump was killed with the slow link holding > 5000 bytes', hit is not None,
              f'{hit} bytes buffered after {secs:.1f} s (settled to {left} before)' if hit is not None
              else f'never above 5000 in {secs:.0f} s')
    after_kill = rig.slow.buffered
    rig.slow.reset_records()
    rc, text, secs = rig.run('ap-param-dump.py', ['--device', rig.device(rig.slow), '--defaults', dfl, vals], 400)
    m = re.search(r'drained (\d+) stale replies', text)
    rep.note(f'D2 buffered {after_kill} bytes at the kill; the rerun took {secs:.1f} s')
    rep.check('D2 rerun at once: exit 0', rc == 0, f'exit {rc}', tail(text))
    rep.check('D2 it prints a "drained N stale replies" line', bool(m), m.group(0) if m else 'no such line',
              tail(text, 4))
    rep.check(f'D2 both files have exactly {n} lines',
              os.path.exists(vals) and os.path.exists(dfl) and count_lines(vals) == n and count_lines(dfl) == n)


def case_d3(rig):
    rep = rig.rep
    n = rig.truth.param_count()
    vals, dfl = rig.path('d3.parm'), rig.path('d3.defaults')
    rig.fast.reset_records()
    rig.fast.drop = lambda direction, frame: direction == 'down' and frame.msgid == 110
    try:
        rc, text, secs = rig.run('ap-param-dump.py', ['--device', rig.device(rig.fast), '--defaults', dfl, vals], 300)
    finally:
        rig.fast.drop = None
    rep.note(f'D3 took {secs:.1f} s with every MAVFTP reply dropped')
    rep.check('D3 MAVFTP ignores the open: exit 0 after the fallback', rc == 0, f'exit {rc}', tail(text))
    rep.check('D3 it says it fell back to PARAM_REQUEST_LIST and that no defaults are available',
              'falling back to PARAM_REQUEST_LIST (no defaults available)' in text
              and '--defaults requested but MAVFTP failed' in text, '', tail(text, 6))
    rep.check(f'D3 the values file has {n} lines',
              os.path.exists(vals) and count_lines(vals) == n,
              f'{count_lines(vals) if os.path.exists(vals) else None} lines')
    rep.check('D3 no defaults file is written', not os.path.exists(dfl))


def case_d4(rig):
    rep = rig.rep
    n = rig.truth.param_count()
    vals = rig.path('d4.parm')

    def drop(direction, frame):
        if direction != 'down' or frame.msgid != 22:
            return False
        _v, _count, index, _id, _t = struct.unpack('<fHH16sB', bytes(frame.payload).ljust(25, b'\0')[:25])
        return index == 5
    rig.fast.reset_records()
    rig.fast.drop = drop
    try:
        rc, text, secs = rig.run('ap-param-dump.py', ['--device', rig.device(rig.fast), '--no-ftp', vals], 300)
    finally:
        rig.fast.drop = None
    rep.check('D4 one PARAM_VALUE dropped under --no-ftp: exit 1', rc == 1, f'exit {rc}, {secs:.1f} s', tail(text))
    rep.check('D4 it says INCOMPLETE and how many it got', f'got {n - 1} of {n}' in text and 'INCOMPLETE' in text,
              '', tail(text, 4))
    rep.check(f'D4 the file is still written, with {n - 1} lines',
              os.path.exists(vals) and count_lines(vals) == n - 1,
              f'{count_lines(vals) if os.path.exists(vals) else None} lines')
    rep.check('D4 exactly one frame was dropped', len(rig.fast.dropped) == 1, f'{len(rig.fast.dropped)} dropped')


def case_d5(rig):
    rep = rig.rep
    g = rig.truth.request(rig.truth.MAV.MAVLINK_MSG_ID_GPS_RAW_INT, 'GPS_RAW_INT')
    rep.check('D5 GPS 3D fix on the truth port before the health runs', g.fix_type >= 3,
              f'fix_type {g.fix_type}')
    lat, lon = str(g.lat), str(g.lon)
    for tag, extra in (('default', []), ('position', ['--health-position'])):
        h = rig.path(f'health-{tag}.txt')
        rc, text, secs = rig.run('ap-param-dump.py', ['--device', rig.device(rig.fast), '--health', h,
                                                      '--health-secs', '15'] + extra + [rig.path(f'health-{tag}.parm')], 300)
        body = open(h).read() if os.path.exists(h) else ''
        fix = re.search(r'^GPS fix_type=(\d+) satellites=(\d+)', body, re.M)
        n_lat, n_lon = len(re.findall(r'\blat=', body)), len(re.findall(r'\blon=', body))
        leak = (lat in body, lon in body, lat in text, lon in text)
        if tag == 'default':
            rep.check('D5 --health: exit 0, a GPS fix_type line, no lat= or lon=, neither value in the file '
                      'or on stdout',
                      rc == 0 and bool(fix) and n_lat == 0 and n_lon == 0 and not any(leak),
                      f'exit {rc}, GPS line {bool(fix)}, lat= x{n_lat}, lon= x{n_lon}, '
                      f'value in file/stdout: {any(leak)}')
        else:
            rep.check('D5 --health --health-position: exit 0, lat= and lon= present',
                      rc == 0 and bool(fix) and n_lat >= 1 and n_lon >= 1,
                      f'exit {rc}, GPS line {bool(fix)}, lat= x{n_lat}, lon= x{n_lon}')


# ------------------------------------------------------- upgrade-verify cases

def case_upgrade(rig):
    rep = rig.rep
    changed = 'SERVO9_TRIM'
    before, _ = rig.truth.read(changed)
    dumps = {}
    for tag in ('pre', 'pre2'):
        vals, dfl = rig.path(f'u-{tag}.parm'), rig.path(f'u-{tag}.defaults')
        rc, text, secs = rig.run('ap-param-dump.py', ['--device', rig.device(rig.fast), '--defaults', dfl, vals], 300)
        dumps[tag] = rc == 0 and os.path.exists(vals) and os.path.exists(dfl)
        rep.check(f'U dump {tag} (fast link, --defaults)', dumps[tag], f'exit {rc}, {secs:.1f} s', tail(text))
    ok = rig.truth.set_raw(changed, 1501)
    after, _ = rig.truth.read(changed)
    rep.check(f'U raw PARAM_SET {changed} 1501, read back on the truth port', ok and after == 1501,
              f'{before:g} -> {after:g}')
    vals, dfl = rig.path('u-post.parm'), rig.path('u-post.defaults')
    rc, text, secs = rig.run('ap-param-dump.py', ['--device', rig.device(rig.fast), '--defaults', dfl, vals], 300)
    dumps['post'] = rc == 0 and os.path.exists(vals) and os.path.exists(dfl)
    rep.check('U dump post (fast link, --defaults)', dumps['post'], f'exit {rc}, {secs:.1f} s', tail(text))

    maps = {'empty': [],
            'explains': [{'old': changed, 'new': changed, 'factor': (after) / before}],
            'wrong': [{'old': changed, 'new': changed, 'factor': 1.0}]}
    for k, rows in maps.items():
        with open(rig.path(f'map-{k}.json'), 'w') as f:
            json.dump({'renames': rows}, f)
    pub = rig.map_path
    rows = [
        ('pre vs pre2, no change, empty map', 'pre2', rig.path('map-empty.json'), 0, None),
        ('pre vs pre2, no change, the published 4.6.3-to-4.7.1 map', 'pre2', pub, 0, None),
        ('pre vs post, one change, empty map', 'post', rig.path('map-empty.json'), 1, 'UNEXPLAINED (1)'),
        ('pre vs post, one change, the published map', 'post', pub, 1, 'UNEXPLAINED (1)'),
        ('pre vs post, a map row whose factor explains it', 'post', rig.path('map-explains.json'), 0,
         '1 of 1 carried across exactly'),
        ('pre vs post, a map row with factor 1', 'post', rig.path('map-wrong.json'), 1, 'MISMATCH (1)'),
    ]
    for label, post, mp, want_rc, want_text in rows:
        name = f'U {label}: exit {want_rc}'
        if mp is None or not os.path.exists(mp):
            rep.skip(name, 'no published rename map (pass --map PATH)')
            continue
        if not (dumps['pre'] and dumps[post]):
            rep.check(name, False, 'the dumps it needs were not made')
            continue
        rc, text, _ = rig.run('ap-upgrade-verify.py',
                              [rig.path('u-pre.parm'), rig.path(f'u-{post}.parm'), '--renames', mp,
                               '--pre-defaults', rig.path('u-pre.defaults'),
                               '--post-defaults', rig.path(f'u-{post}.defaults'), '--quiet-ok'], 120)
        good = rc == want_rc and (want_text is None or want_text in text)
        if want_text and 'UNEXPLAINED' in want_text:
            good = good and re.search(rf'^\s+{changed}: ', text, re.M) is not None
        if want_rc == 0:
            good = good and 'nothing unexplained.' in text
        rep.check(name, good, f'exit {rc}' + (f', "{want_text}"' if want_text else ', "nothing unexplained."'),
                  tail(text, 8))


# ------------------------------------------------------------- listen case

def case_listen(rig):
    rep = rig.rep
    rig.fast.reset_records()
    rc, text, secs = rig.run('mavlink-listen.py', ['tcp', f'127.0.0.1:{rig.fast.listen_port}', '--seconds', '5'], 60)
    rep.check('L mavlink-listen.py tcp (fast link): exit 0', rc == 0, f'exit {rc}', tail(text))
    rep.check('L it prints the autopilot heartbeat as system 1 component 1',
              re.search(r'^heartbeat: sysid 1 compid 1,.*autopilot 3 \(ArduPilot\)', text, re.M) is not None,
              '', tail(text, 4))
    rep.check('L it sent nothing: 0 bytes up, no frames up',
              rig.fast.bytes_up == 0 and not rig.fast.sent_up,
              f'{rig.fast.bytes_up} bytes up, {len(rig.fast.sent_up)} frames')


# --------------------------------------------------------- ap-param-apply cases

def param_sets(link):
    """Names of the PARAM_SET frames the tool sent through `link`, in order."""
    out = []
    for f in link.sent_up:
        if f.msgid == MSG_PARAM_SET:
            _v, _ts, _tc, pid, _t = struct.unpack('<fBB16sB', bytes(f.payload).ljust(23, b'\0')[:23])
            out.append(pid.split(b'\0', 1)[0].decode('ascii', 'replace'))
    return out


def frames_up(link, msgid):
    return sum(1 for f in link.sent_up if f.msgid == msgid)


def num(v):
    return str(int(v)) if v == int(v) else repr(v)


def servo_values(truth):
    return {n: truth.read(n)[0] for n in PNAMES}


def write_pfile(path, vals):
    """A Mission Planner "NAME,value" file, in PNAMES order."""
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w') as f:
        for n in PNAMES:
            if n in vals:
                f.write(f'{n},{num(vals[n])}\n')


def plus1(rig, names):
    return {n: rig.pstart[n] + 1 for n in names}


def p_begin(rig, link):
    """Every P case starts from the start values on truth, with `link` holding
    no backlog and no records."""
    bad = [n for n in PNAMES if rig.truth.read(n)[0] != rig.pstart[n] and not rig.truth.set_raw(n, rig.pstart[n])]
    if bad:
        rig.rep.check('P the start values are back on truth', False, f'cannot reset {bad[:4]}')
    if link is rig.slow:
        rig.settle(link)
    link.reset_records()


def pins(rig):
    return ['--expect-uid', rig.uid, '--expect-git', GIT, '--expect-board', rig.board]


def case_p1(rig):
    rep = rig.rep
    rig.pstart = servo_values(rig.truth)
    p_begin(rig, rig.fast)
    rc, text, secs = rig.run('ap-param-apply.py', ['identify', rig.device(rig.fast)], 120)
    m = re.search(r'^FC: ArduPilot (\S+) git (\S+) board (\d+) uid ([0-9A-F]{24}) sysid (\d+)$', text, re.M)
    rep.check('P1 identify (fast link): exit 0', rc == 0, f'exit {rc}, {secs:.1f} s', tail(text))
    ok = bool(m) and m.group(4) == rig.truth_id['uid'] and m.group(2).startswith(GIT) \
        and int(m.group(3)) == rig.truth_id['board']
    rep.check("P1 it names A's uid, git dbe79216 and board, as the truth port reports them", ok,
              m.group(0) if m else 'no FC: line', tail(text, 4))
    rep.check('P1 it prints "armed: no", and sends no PARAM_SET', 'armed: no' in text and not param_sets(rig.fast),
              f'{len(param_sets(rig.fast))} PARAM_SET')
    if m:                              # the pins are what identify printed
        rig.uid, rig.board = m.group(4), m.group(3)


def case_p2(rig):
    rep = rig.rep
    p_begin(rig, rig.fast)
    f = rig.path('p2', 'a.param')
    write_pfile(f, plus1(rig, ['SERVO9_TRIM']))
    rc, text, secs = rig.run('ap-param-apply.py', ['plan', rig.device(rig.fast), f], 120)
    row = re.search(rf'^SERVO9_TRIM\s+{num(rig.pstart["SERVO9_TRIM"])}\s+{num(rig.pstart["SERVO9_TRIM"] + 1)}\s+\S+\s+CHANGE$',
                    text, re.M)
    rep.check('P2 plan, SERVO9_TRIM at current+1: exit 0, a CHANGE row', rc == 0 and bool(row) and '1 to change' in text,
              f'exit {rc}, {secs:.1f} s', tail(text, 4))
    rep.check('P2 zero PARAM_SET on the link, the truth port unchanged',
              not param_sets(rig.fast) and servo_values(rig.truth) == rig.pstart,
              f'{len(param_sets(rig.fast))} PARAM_SET')


def case_p3(rig):
    rep = rig.rep
    p_begin(rig, rig.slow)
    f, tsv = rig.path('p3', 'a.param'), rig.path('p3', 'p3.tsv')
    tgt = plus1(rig, ['SERVO9_TRIM'])
    write_pfile(f, tgt)
    rc, text, secs = rig.run('ap-param-apply.py', ['write', rig.device(rig.slow)] + pins(rig) + ['--log', tsv, f], 300)
    rows = open(tsv).read().splitlines() if os.path.exists(tsv) else []
    cols = rows[0].split('\t') if len(rows) == 1 else []
    after = servo_values(rig.truth)
    want = dict(rig.pstart, **tgt)
    rep.check('P3 write with pins and --log (slow link): exit 0, "wrote 1, every readback matches"',
              rc == 0 and 'wrote 1, every readback matches' in text, f'exit {rc}, {secs:.1f} s', tail(text, 4))
    rep.check('P3 truth shows the target, and no other name moved', after == want,
              f'SERVO9_TRIM {rig.pstart["SERVO9_TRIM"]:g} -> {after["SERVO9_TRIM"]:g}')
    rep.check('P3 one PARAM_SET on the link, for SERVO9_TRIM', param_sets(rig.slow) == ['SERVO9_TRIM'],
              f'{param_sets(rig.slow)}')
    rep.check('P3 the TSV has one row, status ok, for this uid',
              len(cols) >= 8 and cols[1] == rig.uid and cols[2] == 'SERVO9_TRIM' and cols[6] == 'ok',
              f'{len(rows)} rows' + (f', status {cols[6]}' if len(cols) >= 7 else ''))


def case_p4(rig):
    rep = rig.rep
    p_begin(rig, rig.fast)
    d = rig.path('p4')
    f, tsv = os.path.join(d, 'a.param'), os.path.join(d, 'p4.tsv')
    write_pfile(f, plus1(rig, PNAMES))
    with open(os.path.join(d, 'never-write.txt'), 'w') as fh:
        fh.write('# names the owner set by hand\nSERVO9_*\n')
    rc, text, secs = rig.run('ap-param-apply.py', ['write', rig.device(rig.fast)] + pins(rig) + ['--log', tsv, f], 120)
    rep.check('P4 never-write.txt listing SERVO9_*: refused, nonzero exit',
              rc not in (0, None) and 'REFUSED, names someone else owns' in text and 'SERVO9_MIN' in text,
              f'exit {rc}', tail(text, 3))
    rep.check('P4 refused before anything connects: the link saw no bytes, the tool printed no FC: line',
              rig.fast.bytes_up == 0 and not rig.fast.sent_up and 'FC:' not in text and not os.path.exists(tsv),
              f'{rig.fast.bytes_up} bytes up')
    rep.check('P4 the truth port is unchanged', servo_values(rig.truth) == rig.pstart)


def case_p5(rig):
    rep = rig.rep
    p_begin(rig, rig.fast)
    f = rig.path('p5', 'a.param')
    os.makedirs(os.path.dirname(f), exist_ok=True)
    with open(f, 'w') as fh:
        fh.write('RTL_ALT,1500\n')              # 4.7.1 calls it RTL_ALT_M
    rc, text, secs = rig.run('ap-param-apply.py', ['plan', rig.device(rig.fast), f], 120)
    rep.check('P5 plan with RTL_ALT (renamed RTL_ALT_M in 4.7.1): refused "not on this firmware"',
              rc not in (0, None) and 'REFUSED: not on this firmware: RTL_ALT' in text, f'exit {rc}, {secs:.1f} s',
              tail(text, 4))
    rep.check('P5 zero PARAM_SET on the link', not param_sets(rig.fast), f'{len(param_sets(rig.fast))} PARAM_SET')


def case_p6(rig):
    rep = rig.rep
    f = rig.path('p6', 'a.param')
    write_pfile(f, plus1(rig, PNAMES))
    uid, board = rig.uid, int(rig.board)
    wrong = {
        'uid': ['--expect-uid', uid[:-1] + ('0' if uid[-1] != '0' else '1'), '--expect-git', GIT,
                '--expect-board', str(board)],
        'git': ['--expect-uid', uid, '--expect-git', 'deadbeef', '--expect-board', str(board)],
        'board': ['--expect-uid', uid, '--expect-git', GIT, '--expect-board', str(board + 1)],
        'no pins': [],
    }
    for tag, pin in wrong.items():
        p_begin(rig, rig.fast)
        tsv = rig.path('p6', f'p6-{tag.replace(" ", "")}.tsv')
        rc, text, secs = rig.run('ap-param-apply.py', ['write', rig.device(rig.fast)] + pin + ['--log', tsv, f], 120)
        want = 'needs --expect-uid' if tag == 'no pins' else 'REFUSED, wrong flight controller'
        sets = param_sets(rig.fast)
        same = servo_values(rig.truth) == rig.pstart
        rep.check(f'P6 write with a wrong {tag}: refused, nonzero exit, zero PARAM_SET, truth unchanged',
                  rc not in (0, None) and want in text and not sets and same and not os.path.exists(tsv),
                  f'exit {rc}, {len(sets)} PARAM_SET, truth unchanged {same}', tail(text, 3))


def disarm(rig, label):
    """Disarm on truth and check the heartbeat says so."""
    rig.truth.arm_send(False)
    ok = rig.truth.wait_armed(False)
    rig.rep.check(f'{label} disarmed again on the truth port', ok)
    return ok


def case_p7(rig):
    rep = rig.rep
    p_begin(rig, rig.slow)
    f, tsv = rig.path('p7', 'a.param'), rig.path('p7', 'p7.tsv')
    write_pfile(f, plus1(rig, PNAMES))
    try:
        rc, text, secs, fired = rig.run_live('ap-param-apply.py', ['write', rig.device(rig.slow)] + pins(rig)
                                             + ['--log', tsv, f], lambda line, k: line == 'armed: no')
        armed = rig.truth.wait_armed(True, 3)
        sets = param_sets(rig.slow)
        same = servo_values(rig.truth) == rig.pstart
        rep.check('P7 armed on truth at "armed: no" (slow link): the tool exits 1, "REFUSED: the vehicle is armed"',
                  rc == 1 and 'REFUSED: the vehicle is armed' in text and fired is not None and armed,
                  f'exit {rc}, {secs:.1f} s, armed on truth {armed}', tail(text, 3))
        rep.check('P7 zero PARAM_SET on the link, the truth port unchanged, no --log file',
                  not sets and same and not os.path.exists(tsv),
                  f'{len(sets)} PARAM_SET, truth unchanged {same}, log file {os.path.exists(tsv)}')
    finally:
        disarm(rig, 'P7')


STOPPED_RE = r'STOPPED: the FC armed during the write after (\d+) of (\d+) writes; next was (\S+)\.'


def case_p8(rig):
    rep = rig.rep
    p_begin(rig, rig.slow)
    f, tsv = rig.path('p8', 'a.param'), rig.path('p8', 'p8.tsv')
    tgt = plus1(rig, PNAMES)
    write_pfile(f, tgt)
    try:
        rc, text, secs, fired = rig.run_live('ap-param-apply.py', ['write', rig.device(rig.slow)] + pins(rig)
                                             + ['--log', tsv, f], lambda line, k: k >= 3)
        rig.truth.wait_armed(True, 3)
        m = re.search(STOPPED_RE, text)
        n = int(m.group(1)) if m else None
        sets = param_sets(rig.slow)
        after = servo_values(rig.truth)
        moved = [x for x in PNAMES if after[x] != rig.pstart[x]]
        rows = open(tsv).read().splitlines() if os.path.exists(tsv) else []
        rep.check('P8 armed after the 3rd ok (slow link): exit 1, STOPPED after N of 24, never REFUSED',
                  rc == 1 and bool(m) and m.group(2) == '24' and 3 <= n < 24 and 'REFUSED' not in text,
                  f'exit {rc}, {secs:.1f} s, ' + (m.group(0) if m else 'no STOPPED line'), tail(text, 4))
        if m:
            rep.check(f'P8 the link carried N = {n} PARAM_SET, one per name, in file order', sets == PNAMES[:n],
                      f'{len(sets)} PARAM_SET')
            rep.check(f'P8 truth shows exactly the first N changed, each at its target',
                      moved == PNAMES[:n] and all(after[x] == tgt[x] for x in moved), f'{len(moved)} names changed')
            rep.check(f'P8 the TSV has N ok rows', len(rows) == n and all(r.split('\t')[6] == 'ok' for r in rows),
                      f'{len(rows)} rows')
    finally:
        disarm(rig, 'P8')


def case_p9(rig):
    rep = rig.rep
    p_begin(rig, rig.fast)
    f, tsv = rig.path('p9', 'a.param'), rig.path('p9', 'p9.tsv')
    tgt = plus1(rig, PNAMES)
    write_pfile(f, tgt)
    try:
        rc, text, secs, fired = rig.run_live('ap-param-apply.py', ['write', rig.device(rig.fast)] + pins(rig)
                                             + ['--log', tsv, f], lambda line, k: k >= 3)
        rig.truth.wait_armed(True, 3)
        m = re.search(STOPPED_RE, text)
        warned = re.search(r'^WARNING: the FC is armed now, after (\d+) write\(s\)', text, re.M)
        sets = param_sets(rig.fast)
        after = servo_values(rig.truth)
        moved = [x for x in PNAMES if after[x] != rig.pstart[x]]
        how = m.group(0) if m else (warned.group(0) if warned else 'neither')
        rep.check('P9 armed after the 3rd ok (fast link): exit 1, STOPPED or WARNING, never exit 0, never REFUSED',
                  rc == 1 and (bool(m) or bool(warned)) and 'REFUSED' not in text,
                  f'exit {rc}, {secs:.1f} s, {how}', tail(text, 4))
        rep.check('P9 the names that changed on truth are the names the link carried PARAM_SET for',
                  set(moved) == set(sets) and len(sets) == len(set(sets)),
                  f'{len(moved)} changed, {len(sets)} PARAM_SET')
        if m:
            rep.check('P9 STOPPED: N equals the PARAM_SETs on the link', int(m.group(1)) == len(sets),
                      f'N {m.group(1)}, {len(sets)} PARAM_SET')
        elif warned:
            rep.check('P9 WARNING: its write count equals the PARAM_SETs on the link', int(warned.group(1)) == len(sets),
                      f'{warned.group(1)} writes, {len(sets)} PARAM_SET')
        rep.note(f'P9 outcome: {how}; how many landed armed is in the FC log, read after SITL A stops')
    finally:
        disarm(rig, 'P9')


def fc_log_landed(mavutil, workdir):
    """After the SITL has stopped: from its own dataflash log, the number of
    the rig's names written while armed, after the LAST arm. Logging starts at
    the first arm (LOG_DISARMED is 0) and one file spans the later arms, so
    the PARM dump at the start is skipped and each PARAM_SET, which logs twice,
    counts once. Returns (log name, names written while armed, ARM events)
    or None."""
    import glob
    found = None
    for fn in sorted(glob.glob(os.path.join(workdir, 'logs', '*.BIN'))):
        m, first, ev = mavutil.mavlink_connection(fn), None, []
        while True:
            x = m.recv_match(type=['ARM', 'PARM'])
            if x is None:
                break
            if first is None:
                first = x.TimeUS
            if x.get_type() == 'ARM':
                ev.append('ARMED' if x.ArmState else 'DISARMED')
            elif x.TimeUS != first and x.Name in PNAMES and (not ev or ev[-1] != x.Name):
                ev.append(x.Name)
        if 'ARMED' in ev:
            i = len(ev) - 1 - ev[::-1].index('ARMED')
            landed = []
            for e in ev[i + 1:]:
                if e == 'DISARMED':
                    break
                landed.append(e)
            found = (os.path.basename(fn), landed, [e for e in ev if e in ('ARMED', 'DISARMED')])
    return found


# ------------------------------------------------------ two boards at system 1

def wait_ports_free(ports, secs=15):
    end = time.time() + secs
    while time.time() < end and any(port_busy(p) for p in ports):
        time.sleep(0.3)
    return [p for p in ports if port_busy(p)]


def case_two_boards(rig):
    """After the P cases. SITL A's SERIAL2 is freed by stopping the fast link
    and joined, with SITL B's SERIAL1, behind one link: the tool's frames go to
    both boards and both answer as system 1 component 1. B's SERIAL0 is its
    truth connection, which also unblocks its boot."""
    rep = rig.rep
    rig.fast.stop()
    b = Sitl(rig.sitl.binary, rig.sitl.defaults, 7, os.path.join(rig.sitl.scratch, 'sitl-b'),
             rig.sitl.scratch, name='B')
    link = tb = None
    try:
        b.start()
        tb = Truth(rig.mavutil, b.s0)
        link = start_link(rig.sl, TWO_BOARDS_PORT, [f'127.0.0.1:{rig.sitl.s2}', f'127.0.0.1:{b.s1}'])
        ida, idb = rig.truth_id, tb.version()
        rep.check('T both boards report system 1 component 1 on their own truth ports, with different uids',
                  (rig.truth.sysid, rig.truth.compid) == (1, 1) and (tb.sysid, tb.compid) == (1, 1)
                  and ida['uid'] != idb['uid'],
                  f"A {rig.truth.sysid}/{rig.truth.compid} {ida['uid']}, B {tb.sysid}/{tb.compid} {idb['uid']}")
        okA, okB = rig.truth.set_raw('SERVO9_TRIM', 1511), tb.set_raw('SERVO9_TRIM', 1522)
        rep.check('T SERVO9_TRIM is 1511 on A and 1522 on B, read on their own ports', okA and okB)
        d = os.path.join(rig.work, 't')
        os.makedirs(d, exist_ok=True)
        f, tsv = os.path.join(d, 'a.param'), os.path.join(d, 't.tsv')
        with open(f, 'w') as fh:
            fh.write('SERVO9_TRIM,1533\n')
        dev = f'tcp:127.0.0.1:{TWO_BOARDS_PORT}'
        both = lambda text: ida['uid'] in text and idb['uid'] in text

        def boards():
            return rig.truth.read('SERVO9_TRIM')[0], tb.read('SERVO9_TRIM')[0]
        refusal = 'REFUSED: 2 flight controllers answer as system 1 component 1'

        link.reset_records()
        rc, text, secs = rig.run('ap-param-apply.py', ['identify', dev], 120)
        rep.check('T1 identify: exit 1, "REFUSED: 2 flight controllers answer as system 1 component 1", both uids',
                  rc == 1 and refusal in text and both(text), f'exit {rc}, {secs:.1f} s', tail(text, 4))

        link.reset_records()
        rc, text, secs = rig.run('ap-param-apply.py', ['plan', dev, f], 120)
        rep.check('T2 plan: refused before reading any parameter (no table, no PARAM_REQUEST_READ on the link)',
                  rc == 1 and refusal in text and not re.search(r'^name\s+before\s+target', text, re.M)
                  and frames_up(link, MSG_PARAM_REQUEST_READ) == 0 and not param_sets(link),
                  f'exit {rc}, {frames_up(link, MSG_PARAM_REQUEST_READ)} PARAM_REQUEST_READ', tail(text, 3))

        link.reset_records()
        rc, text, secs = rig.run('ap-param-apply.py', ['write', dev] + pins(rig) + ['--log', tsv, f], 120)
        a_now, b_now = boards()
        rep.check("T3 write with A's pins and --log: refused, no log file, zero PARAM_SET on the link",
                  rc == 1 and refusal in text and not os.path.exists(tsv) and not param_sets(link),
                  f'exit {rc}, {len(param_sets(link))} PARAM_SET', tail(text, 3))
        rep.check('T3 A and B are unchanged on their own ports (1511 and 1522)', (a_now, b_now) == (1511, 1522),
                  f'A {a_now:g}, B {b_now:g}')

        vals, dfl = os.path.join(d, 't.parm'), os.path.join(d, 't.defaults')
        link.reset_records()
        rc, text, secs = rig.run('ap-param-dump.py', ['--device', dev, '--defaults', dfl, vals], 120)
        rep.check('T4 ap-param-dump.py --defaults D OUT: refused, neither file exists',
                  rc not in (0, None) and refusal in text and not os.path.exists(vals) and not os.path.exists(dfl),
                  f'exit {rc}, {secs:.1f} s', tail(text, 3))

        link.reset_records()
        rc, text, secs = rig.run('mavlink-listen.py', ['tcp', f'127.0.0.1:{TWO_BOARDS_PORT}', '--seconds', '5'], 60)
        rep.check('T5 mavlink-listen.py: exit 0, and the link saw no bytes from it',
                  rc == 0 and link.bytes_up == 0 and not link.sent_up,
                  f'exit {rc}, {link.bytes_up} bytes up', tail(text, 3))
        rig.link_clean('T', [link])
    finally:
        if tb is not None:
            try:
                tb.m.close()
            except Exception:  # noqa: BLE001
                pass
        if link is not None:
            link.stop()
        b.stop()
    left = wait_ports_free([5830, 5831, 5832, 5833, TWO_BOARDS_PORT])
    rep.check('T SITL B and the two-board link are gone: ports 5830-5833 and 15822 free', not left,
              f'still held: {left}' if left else '')


# ----------------------------------------------------------------- start-up

def plant_logs(workdir):
    """APM/LOGS under the SITL's working directory, which is MAVFTP's root:
    a 300,000 byte log, LASTLOG.TXT and 34 small logs. Returns (planted
    {name: size}, the big file's bytes)."""
    d = os.path.join(workdir, 'APM', 'LOGS')
    os.makedirs(d, exist_ok=True)
    rnd = random.Random(20261103)
    big = bytes(rnd.randrange(256) for _ in range(300000))
    planted = {}

    def put(name, data):
        with open(os.path.join(d, name), 'wb') as f:
            f.write(data)
        planted[name] = len(data)
    put('00000001.BIN', big)
    put('LASTLOG.TXT', b'1\n')
    for nn in range(2, 36):
        put(f'{nn:08d}.BIN', bytes(rnd.randrange(256) for _ in range(100 + 7 * nn)))
    # for F5, one level up so that F1's listing of APM/LOGS stays 36 entries
    with open(os.path.join(workdir, 'APM', 'F5STALE.BIN'), 'wb') as f:
        f.write(bytes(rnd.randrange(256) for _ in range(150000)))
    return planted, big


def start_rig(args, rep, scratch):
    binary = os.path.abspath(args.sitl)
    if not os.access(binary, os.X_OK):
        raise StartError(f"SITL binary not found at {binary}: build ArduCopter 4.7.1 (git {GIT}) "
                         f"for SITL, or set AP_SITL to the arducopter binary")
    defaults = os.path.abspath(args.defaults)
    if not os.path.isfile(defaults):
        raise StartError(f"defaults file not found at {defaults}: set AP_DEFAULTS to the ArduPilot tree's "
                         f"Tools/autotest/default_params/copter.parm")
    try:
        from pymavlink import mavutil
        import pymavlink
    except ImportError:
        raise StartError("pymavlink not found: pip install 'pymavlink>=2.4.50'") from None
    held = busy_ports()
    if held:
        raise StartError(f"TCP port(s) {held} already in use (this check needs 5820-5823, 5830-5833 and "
                         f"15820-15829 free); stop what holds them and rerun")
    sl = load_module(os.path.join(HERE, 'mavlink-slowlink.py'), 'mavlink_slowlink')
    global _FTP_FIELDS
    _FTP_FIELDS = sl.ftp_fields
    tools = os.path.abspath(args.tools or HERE)
    for t in ('ap-ftp-get.py', 'ap-param-dump.py', 'ap-upgrade-verify.py', 'mavlink-listen.py',
              'ap-param-apply.py'):
        if not os.path.isfile(os.path.join(tools, t)):
            raise StartError(f"{t} not found in {tools}")
    work = os.path.join(scratch, 'work')
    os.makedirs(work)
    rig = Rig(rep, sl, tools, work, mavutil, os.path.abspath(args.map) if args.map else None)
    sitl_dir = os.path.join(scratch, 'sitl-a')
    os.makedirs(sitl_dir)
    rig.planted, rig.log_bytes = plant_logs(sitl_dir)
    rig.sitl = Sitl(binary, defaults, 6, sitl_dir, scratch)
    rig.sitl.start()
    rig.slow = start_link(sl, 15820, f'127.0.0.1:{rig.sitl.s0}', rate=SLOW_RATE, latency=SLOW_LATENCY)
    rig.fast = start_link(sl, 15821, f'127.0.0.1:{rig.sitl.s2}')
    if not rig.sitl.alive():
        raise StartError(f"the SITL exited at start; see {sitl_dir}/sitl.log")
    rig.truth = Truth(mavutil, rig.sitl.s1)
    v = rig.truth.request(rig.truth.MAV.MAVLINK_MSG_ID_AUTOPILOT_VERSION, 'AUTOPILOT_VERSION')
    git = bytes(v.flight_custom_version).decode('ascii', 'ignore').strip('\x00').strip()
    fv = v.flight_sw_version
    print(f"SITL: ArduCopter {(fv >> 24) & 0xff}.{(fv >> 16) & 0xff}.{(fv >> 8) & 0xff} git {inert(git)}, "
          f"system {rig.truth.sysid}; pymavlink {pymavlink.__version__}; python {sys.version.split()[0]}",
          flush=True)
    if not git.startswith(GIT):
        raise StartError(f"the SITL reports git {inert(git)}, not {GIT} (ArduCopter 4.7.1)")
    rig.truth_id = rig.truth.version()
    rig.uid, rig.board = rig.truth_id['uid'], str(rig.truth_id['board'])
    print(f"tools from {tools}", flush=True)
    return rig


def run_group(rig, name, fn):
    t0 = time.time()
    try:
        fn(rig)
    except Exception as e:  # noqa: BLE001  a crashed case is a failed check, the rest still run
        rig.rep.check(f'{name} did not complete', False, f'{type(e).__name__}: {e}')
    finally:
        rig.end_case()
    if not rig.sitl.alive():
        raise StartError(f"the SITL died during {name}; see {rig.sitl.workdir}/sitl.log")
    rig.rep.note(f'{name}: {time.time() - t0:.0f} s')


def case_fix(rig):
    fix, home = rig.truth.wait_fix()
    rig.rep.check('GPS 3D fix and HOME_POSITION on the truth port before the dumps', fix is not None and fix >= 3 and home,
                  f'fix_type {fix}, home {home}')


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter,
                                 epilog='exit status: the number of failed checks (at most 100); 101 if it could not run')
    ap.add_argument('--keep', action='store_true', help='keep the scratch directory and say where')
    ap.add_argument('--tools', metavar='DIR', help="run the tools from DIR (default: this script's directory)")
    ap.add_argument('--map', metavar='PATH', help='the published copter-4.6.3-to-4.7.1.json')
    args = ap.parse_args()
    args.sitl = os.environ.get('AP_SITL') or os.path.join(HERE, 'ardupilot', 'build', 'sitl', 'bin', 'arducopter')
    args.defaults = os.environ.get('AP_DEFAULTS') or os.path.join(
        os.path.dirname(os.path.abspath(args.sitl)), '..', '..', '..', 'Tools', 'autotest', 'default_params', 'copter.parm')
    if args.map is None:
        args.map = os.path.join(HERE, 'maps', 'copter-4.6.3-to-4.7.1.json')

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)
    t_start = time.time()
    rep = Report()
    scratch = tempfile.mkdtemp(prefix='ap-sitl-check.')
    print(f"scratch: {scratch}", flush=True)
    status = None
    ports_free = None
    rig = None
    try:
        try:
            rig = start_rig(args, rep, scratch)
            run_group(rig, 'F1', case_f1)
            run_group(rig, 'F2', case_f2)
            run_group(rig, 'F3', case_f3)
            run_group(rig, 'F4', case_f4)
            rig.link_clean('F')
            run_group(rig, 'GPS', case_fix)
            run_group(rig, 'D1', case_d1)
            run_group(rig, 'D2', case_d2)
            run_group(rig, 'D3', case_d3)
            run_group(rig, 'D4', case_d4)
            run_group(rig, 'D5', case_d5)
            rig.link_clean('D')
            run_group(rig, 'U', case_upgrade)
            run_group(rig, 'L', case_listen)
            for name, fn in (('P1', case_p1), ('P2', case_p2), ('P3', case_p3), ('P4', case_p4),
                             ('P5', case_p5), ('P6', case_p6), ('P7', case_p7), ('P8', case_p8),
                             ('P9', case_p9)):
                run_group(rig, name, fn)
            rig.link_clean('P')
            run_group(rig, 'T', case_two_boards)       # stops the fast link
            # last: the killed fetch leaves a 300 KB burst in the slow link's
            # buffer, which would delay every later case on that link
            run_group(rig, 'F5', case_f5)
            rig.link_clean('U/L/F5', [rig.slow])
        except StartError as e:
            print(f"cannot run: {inert(e)}", file=sys.stderr, flush=True)
            status = 101
        finally:
            PROCS.stop_all()
        if status is None and rig is not None:
            try:
                res = fc_log_landed(rig.mavutil, rig.sitl.workdir)
            except Exception as e:  # noqa: BLE001
                res = None
                rep.note(f'P9 the FC log could not be read: {type(e).__name__}: {e}')
            if res:
                name, landed, arms = res
                rep.note(f"P9 SITL A's own dataflash log {name}: ARM events {arms}; after the last ARMED "
                         f"{len(landed)} of {len(PNAMES)} names were written while armed"
                         + (f" ({landed[0]} .. {landed[-1]})" if landed else ''))
            else:
                rep.note('P9 no ARM event in the SITL log, so how many landed armed is unknown')
        if status is None:
            end = time.time() + 15
            while time.time() < end and busy_ports():
                time.sleep(0.3)
            left = busy_ports()
            ports_free = not left
            rep.check('ports freed', ports_free, 'TCP 5820-5823, 5830-5833, 15820-15829 all free' if ports_free
                      else f'still held: {left}')
    finally:
        PROCS.stop_all()
        if args.keep:
            print(f"kept: {scratch}", flush=True)
        else:
            shutil.rmtree(scratch, ignore_errors=True)
    if status == 101:
        return 101
    print(f"\npassed {rep.passed}, failed {rep.failed}" + (f", skipped {rep.skipped}" if rep.skipped else '')
          + f"; wall time {time.time() - t_start:.0f} s")
    return min(rep.failed, 100)


if __name__ == '__main__':
    sys.exit(main())
