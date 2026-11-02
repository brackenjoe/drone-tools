#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Joe Fernandez
"""Read-only MAVFTP download from an ArduPilot flight controller, built for a
slow, deep-buffered link (USB/IP over a VPN, a telemetry radio), and
verified against the board's own CRC.

Why not pymavlink's MAVFTP.cmd_get: over USB/IP across a VPN it
failed every time on a 5 MB log. ArduPilot answers one BurstReadFile with up to
2000 packets sent back to back and not interruptible
(libraries/GCS_MAVLink/GCS_FTP.cpp, BurstReadFile, transfer_size 2000, ArduPilot commit 8555d941),
so a client that gives up leaves up to ~478 KB in flight. pymavlink 2.4.49 retries the
open after 1 s (read_retry_time), changes session, and then discards every
reply, including the ones it asked for, as "unexpected burst" or "wrong session".
This tool resets the FC's sessions, drains the link until it is quiet, opens
once with a long timeout, takes every reply by offset whatever its burst, and
re-asks from the first gap when a burst ends or stalls.

Only OpenFileRO, BurstReadFile, CalcFileCRC32, ListDirectory, TerminateSession
and ResetSessions are sent: nothing that writes to the vehicle. ResetSessions
also ends any MAVFTP session another ground station has open on the same
vehicle, so a transfer in progress there will fail.

  ap-ftp-get.py /APM/LOGS/00000011.BIN out/00000011.BIN
  ap-ftp-get.py --list /APM/LOGS
  ap-ftp-get.py --last-log out/            # reads LASTLOG.TXT, fetches that log

Measured over USB/IP across a VPN: 17.7 to 18.5 KiB/s, a 5,079,040-byte log in
267.7 s, CRC matched.

Importable: FtpSession holds the transfer code, so other tools (ap-param-dump.py)
reuse it. fetch(..., until_eof=True) is for virtual files such as @PARAM whose
size at open is only an estimate. `ap-ftp-get.py --selftest` runs a scripted fake
FC and needs no pymavlink.

Needs pymavlink for the command line: pip install 'pymavlink>=2.4.50'
(2.4.49 can crash at connect, a TypeError in mavutil.add_message, when a link
switches from MAVLink 1 to 2; fixed in 2.4.50).
"""
import argparse, glob, os, struct, sys, time

# opcodes, GCS_FTP.h FTP_OP
OP_TERM, OP_RESET, OP_LIST, OP_OPEN_RO, OP_CRC, OP_BURST = 1, 2, 3, 4, 14, 15
ACK, NACK = 128, 129
ERR_EOF = 6  # MAV_FTP_ERR_EOF, the error code of the NACK that ends a read
CH = 239  # bytes of data per FILE_TRANSFER_PROTOCOL reply


class FtpError(Exception):
    pass


class FtpOpenError(FtpError):
    """The open was NACKed or never answered: the file or MAVFTP is not there."""


_TAB = []
for _i in range(256):
    _c = _i
    for _ in range(8):
        _c = (_c >> 1) ^ (0xEDB88320 if _c & 1 else 0)
    _TAB.append(_c)


def ap_crc32(data):
    """ArduPilot crc_crc32: reflected 0xEDB88320, init 0, NO final xor (not zlib.crc32)."""
    crc = 0
    for b in data:
        crc = _TAB[(crc ^ b) & 0xff] ^ (crc >> 8)
    return crc


class FtpSession:
    """One MAVFTP conversation over a pymavlink-like connection `m` (needs
    m.mav.file_transfer_protocol_send, m.recv_match, m.target_system and
    m.target_component). `stall` is the seconds of silence before a read is
    re-asked; `log` takes one string."""

    term_quiet = 3.0   # fetch's closing TerminateSession: seconds of quiet, and
    term_cap = 60      # the longest it waits for the link to go quiet

    def __init__(self, m, stall=5.0, log=print):
        self.m = m
        self.stall = stall
        self.log = log
        self.seq = 0
        self.last_size = 0   # bytes and seconds of the last successful fetch,
        self.last_dt = 0.0   # excluding the closing drain

    def send(self, op, size=0, offset=0, data=b''):
        self.seq = (self.seq + 1) & 0xffff
        hdr = struct.pack('<HBBBBBBI', self.seq, 0, op, size, 0, 0, 0, offset)
        self.m.mav.file_transfer_protocol_send(
            0, self.m.target_system, self.m.target_component, (hdr + data).ljust(251, b'\0'))

    def recv(self, timeout):
        x = self.m.recv_match(type='FILE_TRANSFER_PROTOCOL', blocking=True, timeout=timeout)
        if x is None:
            return None
        b = bytes(x.payload).ljust(12, b'\0')
        _s, _sess, opc, size, req, bc, _, ofs = struct.unpack('<HBBBBBBI', b[:12])
        return dict(op=opc, size=size, req=req, bc=bc, ofs=ofs, data=b[12:12 + size])

    def drain(self, quiet=4.0, limit=180):
        t = time.time(); last = t; n = 0
        while time.time() - last < quiet and time.time() - t < limit:
            if self.recv(0.5) is not None:
                last = time.time(); n += 1
        return n, time.time() - t

    def reset(self):
        """ResetSessions, then drain until quiet. Returns (replies drained, seconds)."""
        self.send(OP_RESET)
        return self.drain()

    def terminate(self, drain=True, quiet=None, cap=None):
        self.send(OP_TERM)
        if drain:
            return self.drain(self.term_quiet if quiet is None else quiet,
                              self.term_cap if cap is None else cap)

    def ask(self, op, path, timeout, size=None, offset=0, tries=3):
        enc = path.encode()
        for _ in range(tries):
            self.send(op, len(enc) if size is None else size, offset, enc)
            t = time.time()
            while time.time() - t < timeout:
                r = self.recv(0.5)
                if r and r['req'] == op:
                    return r
        return None

    def listdir(self, path):
        out, ofs = [], 0
        while True:
            r = self.ask(OP_LIST, path, 10, offset=ofs)
            if r is None:
                raise FtpError(f"no reply listing {path}")
            if r['op'] == NACK:
                return out
            for e in r['data'].split(b'\0'):
                if e:
                    out.append(e.decode(errors='replace'))
                    ofs += 1
            if not r['data'].strip(b'\0'):
                return out

    def crc(self, remote):
        """The board's CalcFileCRC32 of `remote`, or None."""
        r = self.ask(OP_CRC, remote, 60)
        return struct.unpack('<I', r['data'][:4])[0] if r and r['op'] == ACK else None

    def fetch(self, remote, until_eof=False, timeout=None):
        """Read `remote` and return its bytes.

        until_eof=False trusts the size the open reports. until_eof=True is for
        virtual files whose size at open is only an estimate: the end is the
        first short data reply, or an EndOfFile NACK. `timeout` is for the
        whole call (None: no limit) and raises TimeoutError. A failed open
        raises FtpOpenError. TerminateSession is sent and drained on every
        way out, so the FC is never left with an open session."""
        try:
            return self._fetch(remote, until_eof, timeout)
        finally:
            try:
                self.terminate()
            except Exception as e:  # noqa: BLE001  never mask the real error
                self.log(f"TerminateSession failed: {e}")

    def _fetch(self, remote, until_eof, timeout):
        deadline = None if timeout is None else time.time() + timeout
        r = self.ask(OP_OPEN_RO, remote, 15 if timeout is None else min(15, timeout))
        if not r or r['op'] != ACK:
            raise FtpOpenError(f"open {remote} failed: {r and r['data'][:2]}")
        size = struct.unpack('<I', r['data'][:4])[0]
        if until_eof:
            self.log(f"open ok, {size} bytes announced (an estimate)")
            return self._read_to_eof(deadline)
        self.log(f"open ok, {size} bytes")
        buf = bytearray(size)
        have = set()
        nchunks = (size + CH - 1) // CH
        t0 = last_print = time.time()
        self.send(OP_BURST, CH, 0)
        last_rx = time.time()
        while len(have) < nchunks:
            r = self.recv(0.5)
            now = time.time()
            if deadline is not None and now > deadline:
                raise TimeoutError(f"{remote}: {len(have)} of {nchunks} chunks after {now - t0:.0f} s")
            if r is not None:
                last_rx = now
                if r['op'] == ACK and r['req'] == OP_BURST and r['size'] and r['ofs'] % CH == 0:
                    buf[r['ofs']:r['ofs'] + r['size']] = r['data']
                    have.add(r['ofs'])
                done = r['req'] == OP_BURST and (r['bc'] or r['op'] == NACK)
            else:
                done = now - last_rx > self.stall
            if now - last_print > 10:
                self.log(f"  {len(have) * CH // 1024} of {size // 1024} KiB, {now - t0:.0f} s")
                last_print = now
            if done and len(have) < nchunks:
                gap = next(i * CH for i in range(nchunks) if i * CH not in have)
                self.send(OP_BURST, CH, gap)
                last_rx = now
        self.last_dt = time.time() - t0
        self.last_size = size
        return bytes(buf)

    def _read_to_eof(self, deadline):
        """Read from 0 until the end is known and every chunk below it is held.

        The end comes from the first ACK with 0 < size < CH (end = offset +
        size), or from an EndOfFile NACK. That NACK's offset is not the end
        (GCS_FTP.cpp, BurstReadFile: the reply is zeroed per request and its
        offset is set only by data replies): after a short chunk it is the
        end, after full chunks it is the last full chunk's offset, and if the
        first read of a burst is already past the end it is 0. Decoded below."""
        chunks = {}
        end = None
        req = 0            # offset of the latest burst request
        zero_nacks = 0
        t0 = last_print = time.time()
        self.send(OP_BURST, CH, 0)
        last_rx = time.time()
        while True:
            r = self.recv(0.5)
            now = time.time()
            if deadline is not None and now > deadline:
                raise TimeoutError(f"{len(chunks)} chunks after {now - t0:.0f} s, end "
                                   + ("not found" if end is None else f"at {end}"))
            if r is not None:
                last_rx = now
                if r['req'] == OP_BURST:
                    if r['op'] == ACK and r['size'] and r['ofs'] % CH == 0:
                        chunks[r['ofs']] = r['data']
                        if r['size'] < CH and end is None:
                            end = r['ofs'] + r['size']
                    elif r['op'] == NACK and r['data'][:1] == bytes([ERR_EOF]) and end is None:
                        cand = None
                        if r['ofs'] == 0 and req > 0:
                            cand = req                   # first read of the burst was past the end
                        elif r['ofs'] == 0:
                            if 0 in chunks:
                                cand = CH                # one full chunk, then the end
                            else:
                                zero_nacks += 1          # empty file, or chunk 0 not seen yet
                                if zero_nacks >= 2:
                                    cand = 0
                        elif r['ofs'] % CH == 0:
                            cand = r['ofs'] + CH         # the last full chunk was at ofs
                        if cand is not None and not any(o >= cand for o in chunks):
                            end = cand
                done = r['req'] == OP_BURST and (r['bc'] or r['op'] == NACK)
            else:
                done = now - last_rx > self.stall
            if end is not None:
                need = (end + CH - 1) // CH
                if len(chunks) >= need and all(i * CH in chunks for i in range(need)):
                    break
            if now - last_print > 10:
                self.log(f"  {len(chunks) * CH // 1024} KiB, {now - t0:.0f} s")
                last_print = now
            if done:
                top = (end + CH - 1) // CH if end is not None else len(chunks) + 1
                gap = next(i * CH for i in range(top + 1) if i * CH not in chunks)
                req = gap
                self.send(OP_BURST, CH, gap)
                last_rx = now
        self.last_dt = time.time() - t0
        data = b''.join(chunks[i * CH] for i in range((end + CH - 1) // CH))[:end]
        self.last_size = len(data)
        return data


# ---------------------------------------------------------------- selftest

class _Msg:
    def __init__(self, payload):
        self.payload = payload


class _FakeFC:
    """A scripted FC standing in for the pymavlink connection `m`. Its burst
    read follows BurstReadFile in ArduPilot 4.7.1 (GCS_FTP.cpp): `burst`
    replies per request, a short read sets burst_complete, a read of 0 bytes
    is an EndOfFile NACK whose offset is the reply's last-set offset (0 if no
    data reply went out yet in the burst)."""

    def __init__(self, content, announced=None, burst=2000, drop=(), swallow=0,
                 stale=0, nack_open=False):
        self.content = content
        self.announced = len(content) if announced is None else announced
        self.burst = burst
        self.drop = set(drop)       # chunk offsets lost the first time they are sent
        self.swallow = swallow      # burst requests the FC ignores
        self.nack_open = nack_open
        self.requests = []          # (op, size, offset) as the tool sent them
        self.nack_offsets = []
        self.q = [self._pkt(ACK, OP_BURST, CH, 0, b'\xaa' * CH) for _ in range(stale)]
        self.target_system = self.target_component = 1
        fc = self

        class _Mav:
            def file_transfer_protocol_send(self, net, sysid, comp, payload):
                fc._on_request(bytes(payload))
        self.mav = _Mav()

    @staticmethod
    def _pkt(op, req, size, ofs, data=b'', bc=0):
        hdr = struct.pack('<HBBBBBBI', 0, 0, op, size, req, bc, 0, ofs)
        return _Msg((hdr + data).ljust(251, b'\0'))

    def recv_match(self, type=None, blocking=True, timeout=None):
        if self.q:
            return self.q.pop(0)
        time.sleep(min(timeout or 0.01, 0.01))
        return None

    def _on_request(self, b):
        _s, _sess, op, size, _r, _bc, _z, ofs = struct.unpack('<HBBBBBBI', b[:12])
        self.requests.append((op, size, ofs))
        if op == OP_RESET or op == OP_TERM:
            self.q.append(self._pkt(ACK, op, 0, 0))
        elif op == OP_OPEN_RO:
            if self.nack_open:
                self.q.append(self._pkt(NACK, op, 2, 0, bytes([10, 0])))
            else:
                self.q.append(self._pkt(ACK, op, 4, 0, struct.pack('<I', self.announced)))
        elif op == OP_BURST:
            if self.swallow > 0:
                self.swallow -= 1
                return
            off = 0
            bc = 0
            for i in range(self.burst):
                chunk = self.content[ofs + i * CH: ofs + (i + 1) * CH]
                if not chunk:
                    self.nack_offsets.append(off)
                    self.q.append(self._pkt(NACK, op, 1, off, bytes([ERR_EOF]), bc))
                    return
                off = ofs + i * CH
                bc = 1 if (len(chunk) < CH or i == self.burst - 1) else 0
                if off in self.drop:
                    self.drop.discard(off)
                else:
                    self.q.append(self._pkt(ACK, op, len(chunk), off, chunk, bc))
                if len(chunk) < CH:
                    off += len(chunk)


def _selftest():
    import random
    import zlib
    rnd = random.Random(7)
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

    def session(fc, stall=0.3):
        s = FtpSession(fc, stall=stall, log=lambda *_: None)
        s.term_quiet = 0.05
        return s

    def blob(n):
        return bytes(rnd.randrange(256) for _ in range(n))

    def same(got, want):
        assert got == want, f"got {len(got)} bytes, want {len(want)}"

    def sent(fc, op):
        return [r for r in fc.requests if r[0] == op]

    @case("known size, all chunks")
    def _():
        data = blob(5000)
        fc = _FakeFC(data)
        same(session(fc).fetch('/x'), data)
        assert fc.requests[-1][0] == OP_TERM
        assert len(sent(fc, OP_BURST)) == 1

    @case("known size, a dropped chunk is refilled from the gap")
    def _():
        data = blob(5000)
        fc = _FakeFC(data, drop={CH * 3})
        same(session(fc).fetch('/x'), data)
        assert (OP_BURST, CH, CH * 3) in fc.requests

    @case("until_eof, short last chunk, announced size larger than the file")
    def _():
        data = blob(5000)
        fc = _FakeFC(data, announced=len(data) * 12)
        same(session(fc).fetch('/x', until_eof=True), data)
        assert len(sent(fc, OP_BURST)) == 1

    @case("until_eof, short last chunk, announced size smaller than the file")
    def _():
        data = blob(5000)
        fc = _FakeFC(data, announced=100)
        same(session(fc).fetch('/x', until_eof=True), data)

    @case("until_eof, length a multiple of 239, end by EndOfFile NACK")
    def _():
        data = blob(CH * 7)
        fc = _FakeFC(data, announced=CH * 100)
        same(session(fc).fetch('/x', until_eof=True), data)
        assert fc.nack_offsets == [CH * 6], fc.nack_offsets   # the last full chunk, not the end

    @case("until_eof, multiple of 239, bursts of 4, NACK after a partial burst")
    def _():
        data = blob(CH * 7)
        fc = _FakeFC(data, burst=4)
        same(session(fc).fetch('/x', until_eof=True), data)

    @case("until_eof, multiple of 239 ending on a burst boundary (NACK at burst start)")
    def _():
        data = blob(CH * 8)
        fc = _FakeFC(data, burst=4)
        same(session(fc).fetch('/x', until_eof=True), data)
        assert fc.nack_offsets == [0], fc.nack_offsets

    @case("until_eof, exactly one chunk")
    def _():
        data = blob(CH)
        fc = _FakeFC(data)
        same(session(fc).fetch('/x', until_eof=True), data)

    @case("until_eof, empty file")
    def _():
        fc = _FakeFC(b'', announced=50)
        same(session(fc).fetch('/x', until_eof=True), b'')

    @case("until_eof, dropped chunks and several short bursts")
    def _():
        data = blob(CH * 13 + 40)
        fc = _FakeFC(data, burst=5, drop={CH * 2, CH * 13})
        same(session(fc).fetch('/x', until_eof=True), data)

    @case("stale replies are drained by reset() and ignored")
    def _():
        data = blob(3000)
        fc = _FakeFC(data, stale=40)
        s = session(fc)
        real_drain = s.drain
        s.drain = lambda quiet=4.0, limit=180: real_drain(0.05, 5)
        n, _dt = s.reset()
        assert n >= 41, n          # 40 stale plus the reset's own ACK
        same(s.fetch('/x', until_eof=True), data)

    @case("stale replies before the open are ignored by ask()")
    def _():
        data = blob(3000)
        fc = _FakeFC(data, stale=40)
        same(session(fc).fetch('/x', until_eof=True), data)

    @case("a stall triggers a re-ask (until_eof)")
    def _():
        data = blob(3000)
        fc = _FakeFC(data, swallow=1)
        same(session(fc, stall=0.3).fetch('/x', until_eof=True), data)
        assert len(sent(fc, OP_BURST)) == 2, fc.requests

    @case("a stall triggers a re-ask (known size)")
    def _():
        data = blob(3000)
        fc = _FakeFC(data, swallow=1)
        same(session(fc, stall=0.3).fetch('/x'), data)
        assert len(sent(fc, OP_BURST)) == 2, fc.requests

    @case("timeout raises TimeoutError and TerminateSession is still sent")
    def _():
        fc = _FakeFC(blob(3000), swallow=10 ** 9)
        try:
            session(fc, stall=0.2).fetch('/x', until_eof=True, timeout=0.8)
        except TimeoutError:
            pass
        else:
            raise AssertionError("no TimeoutError")
        assert fc.requests[-1][0] == OP_TERM, fc.requests

    @case("timeout, known size")
    def _():
        fc = _FakeFC(blob(3000), swallow=10 ** 9)
        try:
            session(fc, stall=0.2).fetch('/x', timeout=0.8)
        except TimeoutError:
            pass
        else:
            raise AssertionError("no TimeoutError")
        assert fc.requests[-1][0] == OP_TERM

    @case("a NACKed open raises FtpOpenError and still sends TerminateSession")
    def _():
        fc = _FakeFC(b'x', nack_open=True)
        try:
            session(fc).fetch('/x')
        except FtpOpenError:
            pass
        else:
            raise AssertionError("no FtpOpenError")
        assert fc.requests[-1][0] == OP_TERM

    @case("ap_crc32(b'123456789') is the init-0, no-final-xor value, not zlib's")
    def _():
        # independent bitwise form of the docstring's algorithm
        crc = 0
        for byte in b'123456789':
            crc ^= byte
            for _ in range(8):
                crc = (crc >> 1) ^ (0xEDB88320 if crc & 1 else 0)
        assert crc == 0x2DFD2D88, hex(crc)
        assert ap_crc32(b'123456789') == crc
        assert zlib.crc32(b'123456789') == 0xCBF43926
        assert ap_crc32(b'123456789') != zlib.crc32(b'123456789')

    bad = results.count(False)
    print(f"{len(results) - bad} passed, {bad} failed")
    return 1 if bad else 0


# --------------------------------------------------------------------- CLI

def find_device():
    for pat in ('/dev/serial/by-id/*ArduPilot*-if00', '/dev/serial/by-id/*ArduPilot*'):
        hits = sorted(glob.glob(pat))
        if hits:
            return hits[0]
    sys.exit("no flight controller found; pass --device")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('remote', nargs='?', help='file on the FC, e.g. /APM/LOGS/00000011.BIN')
    ap.add_argument('local', nargs='?', help='where to write it (a directory keeps the remote name)')
    ap.add_argument('--list', metavar='DIR', help='list a directory on the FC and exit')
    ap.add_argument('--last-log', metavar='DIR', help='fetch the log LASTLOG.TXT names into DIR')
    ap.add_argument('--device', help='default: first ArduPilot USB serial device (-if00)')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--stall', type=float, default=5.0, help='seconds of silence before re-asking (default 5)')
    ap.add_argument('--selftest', action='store_true', help='run the offline tests and exit')
    a = ap.parse_args(argv)
    if a.selftest:
        return _selftest()
    if not (a.list or a.last_log or (a.remote and a.local)):
        ap.error('give REMOTE LOCAL, --list DIR or --last-log DIR')

    try:
        from pymavlink import mavutil
    except ImportError:
        sys.exit("pymavlink not found: pip install 'pymavlink>=2.4.50'")

    # Fail before connecting: a missing output directory would otherwise surface
    # only after the whole transfer, and the download would be lost.
    if a.last_log:
        os.makedirs(a.last_log, exist_ok=True)
    elif a.remote and a.local:
        if a.local.endswith(('/', os.sep)):
            os.makedirs(a.local, exist_ok=True)
        else:
            _parent = os.path.dirname(a.local) or '.'
            if not os.path.isdir(_parent):
                sys.exit(f"output directory {_parent} does not exist")

    dev = a.device or find_device()
    print(f"connecting to {dev} ...", flush=True)
    m = mavutil.mavlink_connection(dev, baud=a.baud, source_system=250, source_component=190)
    if m.wait_heartbeat(timeout=25) is None:
        sys.exit(f"no heartbeat on {dev}")
    s = FtpSession(m, stall=a.stall, log=lambda t: print(t, flush=True))

    def get(remote, local):
        try:
            buf = s.fetch(remote)
        except FtpError as e:
            sys.exit(str(e))
        size, dt = s.last_size, s.last_dt
        print(f"got {size} bytes in {dt:.1f} s, {size / max(dt, 1e-3) / 1024:.1f} KiB/s", flush=True)
        board = s.crc(remote)
        local_crc = ap_crc32(buf)
        s.terminate(drain=False)
        if board != local_crc:
            bad = local + '.crc-mismatch'
            open(bad, 'wb').write(buf)
            sys.exit(f"CRC MISMATCH: board {board and hex(board)}, local {hex(local_crc)}; kept as {bad}")
        tmp = local + '.part'
        open(tmp, 'wb').write(buf)
        os.replace(tmp, local)
        print(f"VERIFIED: CRC {hex(local_crc)} matches the board; wrote {local}")

    n, dt = s.reset()
    print(f"sessions reset; drained {n} stale replies in {dt:.1f} s", flush=True)

    if a.list:
        try:
            entries = s.listdir(a.list)
        except FtpError as e:
            sys.exit(str(e))
        for e in entries:
            kind, rest = e[0], e[1:]
            print(('dir  ' if kind == 'D' else 'file ') + rest.replace('\t', '  '))
        s.terminate(drain=False)
        return 0

    if a.last_log:
        r = s.ask(OP_OPEN_RO, '/APM/LOGS/LASTLOG.TXT', 15)
        if not r or r['op'] != ACK:
            sys.exit("cannot open /APM/LOGS/LASTLOG.TXT")
        s.send(OP_BURST, CH, 0)
        txt = b''
        t = time.time()
        while time.time() - t < 15:
            x = s.recv(0.5)
            if x and x['req'] == OP_BURST and x['op'] == ACK:
                txt = x['data']; break
        s.terminate(quiet=3.0, cap=30)
        num = int(txt.strip() or b'0')
        if num <= 0:
            sys.exit(f"LASTLOG.TXT unreadable: {txt!r}")
        a.remote = f'/APM/LOGS/{num:08d}.BIN'
        a.local = a.last_log
        print(f"LASTLOG.TXT names {a.remote}")

    local = a.local
    if os.path.isdir(local):
        local = os.path.join(local, os.path.basename(a.remote))
    get(a.remote, local)
    return 0


if __name__ == '__main__':
    sys.exit(main())
