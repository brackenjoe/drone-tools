#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Joe Fernandez
"""Upload a file to a flight controller over MAVFTP and verify it landed intact.

pymavlink's MAVFTP.cmd_put returns as soon as it has sent CreateFile, before any
data block is pumped. So a bare cmd_put silently writes an EMPTY file, and it
only appears to work if some later command happens to pump the queued writes for
it. Both failure modes were observed on hardware: one upload produced a 0 byte
file, an earlier one produced a file corrupted in its first 80 bytes because a
directory listing interleaved with the writes. Neither reported an error.

So this pumps the transfer to completion and then verifies against the board's
own CalcFileCRC32. The verify is the point of the tool.

Note ArduPilot's FTP CRC is crc_crc32: reflected poly 0xEDB88320, init 0, and
NO final xor (AP_Filesystem::crc32 seeds checksum = 0). That is not zlib.crc32,
which pre- and post-inverts. Comparing a good file against zlib.crc32 makes it
look corrupt, which will send you hunting a transfer bug that is not there.

THE REMOTE FILE IS DELETED FIRST. Every attempt, the first included, starts by
removing the file at the destination, then writes it. If every attempt fails
the board is left with no file or a partial one, and for a Lua script that
means the script is gone. The failure message says so.

So the board is pinned and identified before anything is touched, as
ap-param-apply.py does. The system id and component id come from the first
AUTOPILOT heartbeat (not a GCS's) and every other source is ignored. The
upload needs --expect-uid (24 hex digits, AUTOPILOT_VERSION.uid2), --expect-git
(the firmware git hash, at least 8 hex digits, compared as an 8-character
prefix) and --expect-board (the board id; ap-version.py prints all three); a
mismatch refuses. It also listens 3 s for AUTOPILOT_VERSION and refuses if more
than one uid answers as that system id. That catches two boards that both
answer then, not one that answers late, two boards with one uid, or a hostile
node. Just before the upload, and before each retry, it takes a fresh
heartbeat and refuses if the vehicle is armed or none arrives in 3 s.
--verify-only changes nothing and needs no --expect-*; --selftest needs no
vehicle.

Without --device it looks in /dev/serial/by-id for an ArduPilot device and
falls back to /dev/ttyACM0 if that exists.

Usage:
  mavftp-put.py script.lua /APM/scripts/script.lua \\
      --expect-uid <24 hex digits> --expect-git dbe79216 --expect-board 5250
  ... same, with --device /dev/ttyACM0
  ... same, with --device tcp:127.0.0.1:5760
  ... same, with --mkdir
  mavftp-put.py script.lua /APM/scripts/script.lua --verify-only
  mavftp-put.py --selftest            # offline, needs no vehicle

Needs pymavlink (pip install pymavlink) for everything except --selftest.
Exit status: 0 verified (or selftest passed), 1 not verified, refused, or
selftest failed.
"""
import argparse, glob, logging, os, re, sys, time

# ArduPilot crc_crc32: reflected table, poly 0xEDB88320, init 0, no final xor
_TAB = []
for _i in range(256):
    _c = _i
    for _ in range(8):
        _c = (_c >> 1) ^ (0xEDB88320 if _c & 1 else 0)
    _TAB.append(_c)


def ap_crc32(data, crc=0):
    for b in data:
        crc = _TAB[(crc ^ b) & 0xff] ^ (crc >> 8)
    return crc & 0xffffffff


mavutil = None  # imported in main(), so --help and --selftest need no pymavlink
ARMED_BIT = 128  # MAV_MODE_FLAG_SAFETY_ARMED


def find_device():
    for pat in ('/dev/serial/by-id/*ArduPilot*-if00', '/dev/serial/by-id/*ArduPilot*'):
        hits = sorted(glob.glob(pat))
        if hits:
            return hits[0]
    if os.path.exists('/dev/ttyACM0'):
        return '/dev/ttyACM0'
    sys.exit("no flight controller found; pass --device")


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


def version_info(v):
    fv = v.flight_sw_version
    return {
        'version': f"{(fv >> 24) & 0xff}.{(fv >> 16) & 0xff}.{(fv >> 8) & 0xff}",
        'git': re.sub(r'[^0-9A-Za-z]', '?',
                      bytes(v.flight_custom_version).decode('ascii', 'ignore').strip('\x00').strip()),
        'board': v.board_version >> 16,
        'uid': bytes(v.uid2[:12]).hex().upper(),
    }


def identity_problems(v, expect_uid, expect_git, expect_board):
    """What differs between the board's identity `v` (version_info) and what
    was expected; an empty list means it is the expected board."""
    bad = []
    if expect_uid and v['uid'] != expect_uid.upper():
        bad.append(f"uid {v['uid']} is not the expected {expect_uid.upper()}")
    if expect_git and not v['git'].lower().startswith(expect_git.lower()[:8]):
        bad.append(f"git {v['git']} is not the expected {expect_git}")
    if expect_board is not None and v['board'] != expect_board:
        bad.append(f"board {v['board']} is not the expected {expect_board}")
    return bad


def check_identity(m, a):
    """Print the board, refuse a shared system id or a wrong identity."""
    sysid, compid = m.target_system, m.target_component
    vs = [version_info(v) for v in autopilot_versions(m, sysid, compid)]
    if not vs:
        sys.exit("no AUTOPILOT_VERSION")
    for v in vs:
        print(f"FC: ArduPilot {v['version']} git {v['git']} board {v['board']} "
              f"uid {v['uid']} sysid {sysid}")
    refuse_shared_sysid([v['uid'] for v in vs], sysid, compid)
    bad = identity_problems(vs[0], a.expect_uid, a.expect_git, a.expect_board)
    if bad:
        sys.exit("REFUSED, wrong flight controller: " + '; '.join(bad))


def refuse_if_armed(m, sysid, compid):
    """Exit REFUSED if the pinned autopilot is armed. Heartbeats queued while a
    slow step ran may be stale, so any armed one among them refuses, and then a
    fresh one is waited for; none in 3 s refuses, as the armed state is unknown."""
    queued = False
    while True:
        msg = m.recv_match(type='HEARTBEAT', blocking=False)
        if msg is None:
            break
        if msg.get_srcSystem() == sysid and msg.get_srcComponent() == compid:
            queued = queued or bool(msg.base_mode & ARMED_BIT)
    hb = recv_from(m, 'HEARTBEAT', sysid, compid, 3.0)
    if hb is None:
        sys.exit("REFUSED: no fresh heartbeat from the FC, so its armed state is unknown")
    if queued or hb.base_mode & ARMED_BIT:
        sys.exit("REFUSED: the vehicle is armed")


class _CRCGrab(logging.Handler):
    """cmd_crc only logs its result, so capture it off the log record."""
    def __init__(self):
        super().__init__()
        self.value = None

    def emit(self, record):
        m = re.search(r'crc: .* (0x[0-9a-fA-F]{8})', record.getMessage())
        if m:
            self.value = int(m.group(1), 16)


def remote_crc(ftp, path):
    grab = _CRCGrab()
    root = logging.getLogger()
    root.addHandler(grab)
    try:
        ftp.cmd_crc([path])
    finally:
        root.removeHandler(grab)
    return grab.value


GONE = "the remote file was removed before the first attempt and is now missing or partial"


def put_verified(new_ftp, local, remote, want, size, retries, settle=(0.3, 0.5), guard=None):
    """Upload `local` to `remote` and verify against the board's CRC.
    `new_ftp()` returns a fresh MAVFTP-like session (cmd_rm, cmd_put,
    process_ftp_reply, cmd_crc). The remote file is removed at the start of
    every attempt, the first included. `guard()`, if given, runs before each
    removal and exits to refuse. Returns 0 once verified, else 1."""
    for attempt in range(1, retries + 1):
        print(f"--- attempt {attempt} ---")
        if guard is not None:
            try:
                guard()
            except SystemExit as e:
                if attempt == 1:
                    raise
                print(f"{e}\nSTOPPED after {attempt - 1} attempt(s): {GONE}.", file=sys.stderr)
                return 1
        # a fresh session per operation; a stale one is a common source of
        # "succeeded but did nothing"
        new_ftp().cmd_rm([remote])
        time.sleep(settle[0])

        ftp = new_ftp()
        ftp.cmd_put([local, remote])
        # the whole point: cmd_put has only sent CreateFile at this stage
        ftp.process_ftp_reply('CreateFile', timeout=60)
        time.sleep(settle[1])

        got = remote_crc(new_ftp(), remote)
        if got is None:
            print("  no CRC returned, retrying")
            continue
        if got == want:
            print(f"\nVERIFIED: 0x{got:08x}, {size} bytes at {remote}")
            return 0
        print(f"  CRC 0x{got:08x} != 0x{want:08x}, retrying")

    print(f"\nFAILED to verify {remote} after {retries} attempts: {GONE}.", file=sys.stderr)
    return 1


def require_expect(a):
    """An upload changes the board, so it must say which board it means."""
    if not a.verify_only and not (a.expect_uid and a.expect_git and a.expect_board is not None):
        sys.exit("upload needs --expect-uid, --expect-git and --expect-board "
                 "(ap-version.py prints them; --verify-only needs none)")


# ---------------------------------------------------------------- selftest

class _FakeBoard:
    """The flight controller's filesystem, handing out fake MAVFTP sessions.
    Like the real cmd_put, a put only creates the file: the data lands when
    process_ftp_reply('CreateFile') pumps it. `faults` is consumed one per
    put: 'ok', 'empty' (never pumped), 'corrupt' (first 80 bytes damaged),
    'nocrc' (the CRC reply never comes)."""

    def __init__(self, faults=()):
        self.files = {}
        self.faults = list(faults)
        self.puts = 0
        self.silent_crc = False

    def session(self):
        return _FakeSession(self)


class _FakeSession:
    def __init__(self, board):
        self.b = board
        self.queued = None
        self.fault = 'ok'

    def cmd_rm(self, args):
        self.b.files.pop(args[0], None)

    def cmd_put(self, args):
        self.b.puts += 1
        self.fault = self.b.faults.pop(0) if self.b.faults else 'ok'
        self.b.silent_crc = self.fault == 'nocrc'   # the next CRC query goes unanswered
        self.b.files[args[1]] = b''           # CreateFile only
        self.queued = (args[0], args[1])

    def process_ftp_reply(self, op, timeout=0):
        if self.queued is None or self.fault == 'empty':
            return
        with open(self.queued[0], 'rb') as fh:
            data = fh.read()
        if self.fault == 'corrupt':
            data = bytes(x ^ 0xFF for x in data[:80]) + data[80:]
        out = b''
        for off in range(0, len(data), 239):  # one FTP payload per write
            out += data[off:off + 239]
        self.b.files[self.queued[1]] = out

    def cmd_crc(self, args):
        path = args[0]
        silent, self.b.silent_crc = self.b.silent_crc, False
        if path not in self.b.files or silent:
            return
        logging.getLogger().info("crc: %s 0x%08x in 0.1s", path, ap_crc32(self.b.files[path]))


def _selftest():
    import tempfile
    import zlib
    results = []

    def case(name):
        def deco(fn):
            try:
                fn()
                ok, why = True, ''
            except Exception as e:  # noqa: BLE001
                ok, why = False, f"{type(e).__name__}: {e}"
            print(f"{'PASS' if ok else 'FAIL'}  {name}" + ('' if ok else f"  ({why})"))
            results.append(ok)
            return fn
        return deco

    payload = bytes((i * 7 + 3) & 0xff for i in range(1000))   # spans 5 chunks of 239
    with tempfile.TemporaryDirectory() as tmp:
        lp = os.path.join(tmp, 'f.bin')
        with open(lp, 'wb') as fh:
            fh.write(payload)
        want = ap_crc32(payload)

        def info_logging():
            root = logging.getLogger()
            old = root.level
            root.setLevel(logging.INFO)
            return root, old

        def put(faults, retries=4):
            board = _FakeBoard(faults)
            root, old = info_logging()
            try:
                rc = put_verified(board.session, lp, '/APM/f.bin', want, len(payload),
                                  retries, settle=(0, 0))
            finally:
                root.setLevel(old)
            return rc, board

        @case("ap_crc32(b'123456789') == 0x2DFD2D88 and differs from zlib.crc32")
        def _():
            assert ap_crc32(b'123456789') == 0x2DFD2D88, hex(ap_crc32(b'123456789'))
            assert zlib.crc32(b'123456789') == 0xCBF43926
            assert ap_crc32(b'123456789') != zlib.crc32(b'123456789')

        @case("ap_crc32 of empty data is 0 (init 0, no final xor)")
        def _():
            assert ap_crc32(b'') == 0

        @case("a good put verifies on attempt 1 and the board holds the exact bytes")
        def _():
            rc, b = put([])
            assert rc == 0 and b.puts == 1 and b.files['/APM/f.bin'] == payload

        @case("a put that is never pumped (empty file) is caught and retried")
        def _():
            rc, b = put(['empty', 'ok'])
            assert rc == 0 and b.puts == 2 and b.files['/APM/f.bin'] == payload

        @case("a corrupted first 80 bytes is caught and retried")
        def _():
            rc, b = put(['corrupt', 'ok'])
            assert rc == 0 and b.puts == 2 and b.files['/APM/f.bin'] == payload

        @case("no CRC reply is retried")
        def _():
            rc, b = put(['nocrc', 'ok'])
            assert rc == 0 and b.puts == 2

        @case("every attempt bad: gives up after the retry count and returns 1")
        def _():
            rc, b = put(['empty'] * 3, retries=3)
            assert rc == 1 and b.puts == 3

        @case("remote_crc reads the value off the log record and unhooks its handler")
        def _():
            board = _FakeBoard()
            board.files['/x'] = payload
            n = len(logging.getLogger().handlers)
            root, old = info_logging()
            try:
                got = remote_crc(board.session(), '/x')
                missing = remote_crc(board.session(), '/missing')
            finally:
                root.setLevel(old)
            assert got == want, got
            assert missing is None
            assert len(logging.getLogger().handlers) == n

        # ---- identity, pinning and the armed refusal, on a fake link
        class Msg:
            def __init__(self, kind, src=(1, 1), **kw):
                self.kind, self.src = kind, src
                self.__dict__.update(kw)

            def get_type(self):
                return self.kind

            def get_srcSystem(self):
                return self.src[0]

            def get_srcComponent(self):
                return self.src[1]

        class Link:
            """A MAVLink connection. `msgs` are already queued (a non-blocking read
            sees them); `fresh` arrive only to a blocking read, as a new heartbeat would."""
            def __init__(self, msgs, fresh=()):
                self.q = list(msgs)
                self.fresh = list(fresh)
                self.sent = 0
                self.mav = self

            def command_long_send(self, *args):
                self.sent += 1

            def recv_match(self, type=None, blocking=True, timeout=None, condition=None):
                want = [type] if isinstance(type, str) else type
                for pool in (self.q, self.fresh) if blocking else (self.q,):
                    for i, msg in enumerate(pool):
                        if want is None or msg.kind in want:
                            return pool.pop(i)
                return None

        def hb(armed, src=(1, 1)):
            return Msg('HEARTBEAT', src, base_mode=ARMED_BIT if armed else 0)

        def refused(fn, *args):
            try:
                fn(*args)
            except SystemExit as e:
                assert str(e).startswith('REFUSED'), str(e)
                return str(e)
            raise AssertionError("did not refuse")

        ident = {'uid': 'AABBCCDDEEFF001122334455', 'git': 'dbe79216ab', 'board': 5250}

        @case("identity: the expected uid, git prefix and board give no problem")
        def _():
            assert identity_problems(ident, 'aabbccddeeff001122334455', 'dbe79216', 5250) == []
            assert identity_problems(ident, 'aabbccddeeff001122334455', 'dbe79216ab12', 5250) == []

        @case("identity: a wrong uid, git or board is named, each on its own")
        def _():
            assert len(identity_problems(ident, '0' * 24, 'dbe79216', 5250)) == 1
            assert len(identity_problems(ident, ident['uid'], 'deadbeef', 5250)) == 1
            assert len(identity_problems(ident, ident['uid'], 'dbe79216', 1)) == 1
            assert len(identity_problems(ident, '0' * 24, 'deadbeef', 1)) == 3

        @case("identity: board 0 is a value, not 'unset'")
        def _():
            assert identity_problems(dict(ident, board=5250), None, None, 0)

        @case("upload without all three --expect-* is refused; verify-only needs none")
        def _():
            ns = argparse.Namespace

            def needs(a):
                try:
                    require_expect(a)
                except SystemExit as e:
                    return 'needs --expect-uid' in str(e)
                return False
            assert needs(ns(expect_uid='A' * 24, expect_git='dbe79216', expect_board=None,
                            verify_only=False))
            assert needs(ns(expect_uid=None, expect_git=None, expect_board=None,
                            verify_only=False))
            require_expect(ns(expect_uid=None, expect_git=None, expect_board=None, verify_only=True))
            require_expect(ns(expect_uid='A' * 24, expect_git='dbe79216', expect_board=0,
                              verify_only=False))

        @case("armed: a disarmed heartbeat from the pinned autopilot passes")
        def _():
            refuse_if_armed(Link([], [hb(False)]), 1, 1)

        @case("armed: an armed heartbeat refuses")
        def _():
            assert 'armed' in refused(refuse_if_armed, Link([], [hb(True)]), 1, 1)

        @case("armed: no heartbeat in the window refuses, state unknown")
        def _():
            assert 'unknown' in refused(refuse_if_armed, Link([]), 1, 1)

        @case("armed: a foreign armed heartbeat is ignored, a foreign disarmed one is not taken")
        def _():
            refuse_if_armed(Link([hb(True, (2, 1))], [hb(False)]), 1, 1)
            assert 'unknown' in refused(refuse_if_armed, Link([], [hb(False, (2, 1))]), 1, 1)

        @case("armed: a stale armed heartbeat queued before a fresh disarmed one refuses")
        def _():
            assert 'armed' in refused(refuse_if_armed, Link([hb(True)], [hb(False)]), 1, 1)

        def version(uid_hex):
            return Msg('AUTOPILOT_VERSION', uid2=bytes.fromhex(uid_hex) + b'\0' * 6)

        @case("shared sysid: two uids answering in the window refuse; one uid heard twice does not")
        def _():
            global mavutil
            import types
            old = mavutil
            mavutil = types.SimpleNamespace(mavlink=types.SimpleNamespace(
                MAV_CMD_REQUEST_MESSAGE=512, MAVLINK_MSG_ID_AUTOPILOT_VERSION=148))
            try:
                two = autopilot_versions(Link([version('AA' * 12), version('BB' * 12)]), 1, 1,
                                         window=0.2)
                assert len(two) == 2
                assert 'answer as system 1' in refused(
                    refuse_shared_sysid, ['AA' * 12, 'BB' * 12], 1, 1)
                one = autopilot_versions(Link([version('AA' * 12), version('AA' * 12)]), 1, 1,
                                         window=0.2)
                assert len(one) == 1
            finally:
                mavutil = old

        @case("guard: a refusal before the first attempt leaves the board's file alone")
        def _():
            board = _FakeBoard()
            board.files['/APM/f.bin'] = b'the old script'
            try:
                put_verified(board.session, lp, '/APM/f.bin', want, len(payload), 3,
                             settle=(0, 0), guard=lambda: sys.exit("REFUSED: the vehicle is armed"))
            except SystemExit:
                pass
            else:
                raise AssertionError("did not refuse")
            assert board.files['/APM/f.bin'] == b'the old script' and board.puts == 0

        @case("guard: runs before every attempt, and a refusal on a retry says the file is gone")
        def _():
            import contextlib
            import io
            board = _FakeBoard(['empty', 'empty'])
            calls = []

            def guard():
                calls.append(1)
                if len(calls) == 2:
                    sys.exit("REFUSED: the vehicle is armed")
            err = io.StringIO()
            root, old = info_logging()
            try:
                with contextlib.redirect_stderr(err):
                    rc = put_verified(board.session, lp, '/APM/f.bin', want, len(payload), 3,
                                      settle=(0, 0), guard=guard)
            finally:
                root.setLevel(old)
            assert rc == 1 and len(calls) == 2 and board.puts == 1
            assert GONE in err.getvalue() and 'REFUSED' in err.getvalue()

        @case("every attempt bad: the final message says the remote file is missing or partial")
        def _():
            import contextlib
            import io
            err = io.StringIO()
            with contextlib.redirect_stderr(err):
                rc, _b = put(['empty'] * 2, retries=2)
            assert rc == 1 and GONE in err.getvalue(), err.getvalue()

    bad = results.count(False)
    print(f"\n{len(results) - bad} passed, {bad} failed")
    return 1 if bad else 0


def build_parser():
    p = argparse.ArgumentParser(
        description="Upload a file to ONE pinned flight controller over MAVFTP and verify it "
                    "against the board's own CRC. Exit 0 only when verified. The remote file "
                    "is deleted before every attempt, the first included, so a failed upload "
                    "leaves the board with no file or a partial one.",
        epilog="Needs pymavlink, except for --selftest. An upload needs all three --expect-* "
               "options and refuses if the vehicle is armed or another flight controller "
               "answers as the same system id.")
    p.add_argument('local', nargs='?', help='local file to upload')
    p.add_argument('remote', nargs='?',
                   help="destination path on the board, e.g. /APM/scripts/script.lua")
    p.add_argument('--device', default=None,
                   help='MAVLink connection string: a serial device, tcp:HOST:PORT or '
                        'udp:HOST:PORT (default: first ArduPilot device in '
                        '/dev/serial/by-id, else /dev/ttyACM0 if it exists)')
    p.add_argument('--expect-uid', help='24 hex digits, the USB serial / AUTOPILOT_VERSION.uid2; '
                   'required to upload')
    p.add_argument('--expect-git', help='the firmware git hash: at least 8 hex digits, compared '
                   'as an 8-character prefix of the FC\'s git hash; required to upload')
    p.add_argument('--expect-board', type=int,
                   help='board id, e.g. 5250 for TBS_LUCID_H7; required to upload')
    p.add_argument('--baud', type=int, default=115200,
                   help='serial baud rate, ignored for network links (default: %(default)s)')
    p.add_argument('--retries', type=int, default=4,
                   help='upload attempts before giving up (default: %(default)s)')
    p.add_argument('--mkdir', action='store_true',
                   help='create the remote parent directory first (needs the same --expect-* '
                        'and the same armed check as the upload)')
    p.add_argument('--verify-only', action='store_true',
                   help='check the remote CRC against the local file, change nothing; needs '
                        'no --expect-*')
    p.add_argument('--selftest', action='store_true',
                   help='run the offline self-test (CRC vector, retry/verify logic against '
                        'a fake board) and exit')
    return p


def main():
    p = build_parser()
    a = p.parse_args()
    if a.selftest:
        return _selftest()
    if not a.local or not a.remote:
        p.error("local and remote are required (or use --selftest)")
    if a.expect_git is not None and not re.fullmatch(r'[0-9a-fA-F]{8,}', a.expect_git):
        p.error(f"--expect-git {a.expect_git!r}: need at least 8 hex digits "
                f"(it is compared as an 8-character prefix)")
    require_expect(a)

    global mavutil
    try:
        from pymavlink import mavutil, mavftp
    except ImportError:
        sys.exit("pymavlink not found. Install it: pip install pymavlink")

    logging.basicConfig(level=logging.INFO, format='  %(message)s')

    if not os.path.isfile(a.local):
        sys.exit(f"no such file: {a.local}")
    with open(a.local, 'rb') as fh:
        blob = fh.read()
    want = ap_crc32(blob)
    print(f"{a.local}: {len(blob)} bytes, expected board CRC 0x{want:08x}")

    device = a.device or find_device()
    print(f"connecting to {device} ...")
    try:
        m = mavutil.mavlink_connection(device, baud=a.baud, source_system=250, source_component=190)
    except Exception as e:
        sys.exit(f"cannot open {device}: {e}")
    if pin_autopilot(m, 25) is None:
        sys.exit(f"no autopilot heartbeat on {device}")
    sysid, compid = m.target_system, m.target_component
    print(f"  heartbeat: autopilot system {sysid} component {compid}")
    if not a.verify_only or a.expect_uid or a.expect_git or a.expect_board is not None:
        check_identity(m, a)

    def new_ftp():
        return mavftp.MAVFTP(m, m.target_system, m.target_component)

    if a.verify_only:
        got = remote_crc(new_ftp(), a.remote)
        if got is None:
            sys.exit("no CRC returned; file probably does not exist")
        ok = got == want
        print(f"remote 0x{got:08x} vs local 0x{want:08x}: {'MATCH' if ok else 'MISMATCH'}")
        return 0 if ok else 1

    def guard():
        refuse_if_armed(m, sysid, compid)

    if a.mkdir:
        parent = os.path.dirname(a.remote)
        if parent and parent != '/':
            guard()
            new_ftp().cmd_mkdir([parent])

    print("note: the remote file is removed at the start of every attempt, the first included")
    return put_verified(new_ftp, a.local, a.remote, want, len(blob), a.retries, guard=guard)


if __name__ == '__main__':
    sys.exit(main())
