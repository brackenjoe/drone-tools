#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Joe Fernandez
"""Read-only MAVFTP download from an ArduPilot flight controller, built for a
slow, deep-buffered link (USB/IP over a VPN, a telemetry radio), and
verified against the board's own CRC: a transport check, not proof of the
source.

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

VERIFIED means the board's CRC matches the bytes received from the heartbeat
peer. It is a transport check (nothing was lost or reordered), not proof of the
source: anything that can inject MAVLink on the link and speak as the vehicle can
satisfy it. CRC MISMATCH means the board answered with a CRC that differs from
the local one (bytes kept as LOCAL.crc-mismatch); it is only ever printed with
two real values. UNVERIFIED means no comparison was made: the board did not
answer in time, refused, sent a short reply, or --no-crc was given (bytes kept
as LOCAL.unverified). Exit codes: 0 VERIFIED, 1 failure or MISMATCH, 3
UNVERIFIED. The download is written to LOCAL.unverified before the CRC is
asked for and renamed to LOCAL only on a match, so a Ctrl-C or a dropped
tunnel during the wait keeps it.

The board computes CalcFileCRC32 in its only FTP worker thread, reading the
whole file in 64-byte pieces, and answers nothing else until it is done
(libraries/GCS_MAVLink/GCS_FTP.cpp, AP_Filesystem::crc32, ArduPilot 4.7.1
dbe79216), and MAVFTP is silent meanwhile. On a TBS Lucid H7 (4.7.1
dbe79216) a 218 MB log got no CRC reply within the old tool's 180 s
(2026-11-22); the board's rate is not yet measured, so the default wait,
twice the download time, is an estimate. A log still being written grows
between the download and the CRC, so its CRC cannot match: fetch a closed
log. A request with a new seq makes it compute again;
a request whose seq is the last one's (request seq + 1 == the cached reply's
seq) is answered from the cached reply without recomputing. So the CRC is
asked for once, re-sent with the same seq only to cover a lost packet, and
the wait (--crc-wait) is separate from --timeout, which bounds the download.
After a timeout the FC stays busy until its CRC finishes: wait before the next
MAVFTP command. Only a reply with seq == request seq + 1 is taken.

Replies are accepted only from the system and component of the
first autopilot heartbeat seen on the link, and only when addressed to this
connection; the rest are skipped and counted. That filters out other devices
on a shared link (a ground station, a gimbal, a companion computer), but any
sender can claim those ids, so it is not authentication. MAVLink signing is
the only provenance control, and this tool neither signs nor checks
signatures.

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
import argparse, glob, os, struct, sys, tempfile, time, unicodedata, zlib

# opcodes, GCS_FTP.h FTP_OP
OP_TERM, OP_RESET, OP_LIST, OP_OPEN_RO, OP_CRC, OP_BURST = 1, 2, 3, 4, 14, 15
ACK, NACK = 128, 129
ERR_EOF = 6  # MAV_FTP_ERR_EOF, the error code of the NACK that ends a read
ERR_FAILERRNO = 2  # MAV_FTP_ERR_FAILERRNO: a second data byte carries the errno
FTP_ERR = {0: 'None', 1: 'Fail', 2: 'FailErrno', 3: 'InvalidDataSize', 4: 'InvalidSession',
           5: 'NoSessionsAvailable', 6: 'EOF', 7: 'UnknownCommand', 8: 'FileExists',
           9: 'FileProtected', 10: 'FileNotFound'}  # MAV_FTP_ERR
CH = 239  # bytes of data per FILE_TRANSFER_PROTOCOL reply
MAX_BYTES = 2 << 30  # default cap on one download


def _inert_char(c):
    """One character, or its escape: C0/DEL/C1 as \\xNN; format (Cf), line and
    paragraph separator (Zl, Zp) characters as \\uXXXX, so a bidi override or a
    zero-width character cannot reorder or hide text."""
    o = ord(c)
    if o < 0x20 or 0x7f <= o <= 0x9f:
        return f"\\x{o:02x}"
    if unicodedata.category(c) in ('Cf', 'Zl', 'Zp'):
        return f"\\u{o:04x}" if o < 0x10000 else f"\\U{o:08x}"
    return c


def inert(s):
    """`s` with every C0 control, DEL and C1 control character shown as \\xNN
    (lowercase hex), so text from the vehicle cannot drive the terminal."""
    return ''.join(map(_inert_char, s))


class FtpError(Exception):
    pass


class FtpTimeout(FtpError, TimeoutError):
    """The deadline passed. A TimeoutError too, for callers that catch that."""


class FtpOpenError(FtpError):
    """The open was NACKed or never answered: the file or MAVFTP is not there."""


_TAB = []
for _i in range(256):
    _c = _i
    for _ in range(8):
        _c = (_c >> 1) ^ (0xEDB88320 if _c & 1 else 0)
    _TAB.append(_c)


def _ap_crc32_table(data):
    """The table-loop form of ap_crc32, kept as the reference the selftest
    compares the zlib form with (about 35 s on 607 MB; zlib takes a second)."""
    crc = 0
    for b in data:
        crc = _TAB[(crc ^ b) & 0xff] ^ (crc >> 8)
    return crc


def ap_crc32(data):
    """ArduPilot crc_crc32: reflected 0xEDB88320, init 0, NO final xor. Computed
    as zlib's crc32 with init 0xFFFFFFFF and the final xor undone, which is
    the same function (zlib.crc32(d, 0xFFFFFFFF) ^ 0xFFFFFFFF), but in C."""
    return zlib.crc32(data, 0xFFFFFFFF) ^ 0xFFFFFFFF


class FtpSession:
    """One MAVFTP conversation over a pymavlink-like connection `m` (needs
    m.mav.file_transfer_protocol_send, m.mav.srcSystem, m.mav.srcComponent,
    m.recv_match, m.target_system and m.target_component; a target_component of
    0, which pymavlink leaves unset, accepts any component of the peer system). `stall` is the seconds of silence before a read is
    re-asked; `log` takes one string. `max_bytes` caps one download. `dropped`
    counts replies skipped because they did not come from the heartbeat peer or
    were not addressed to this connection. `stale_crc` counts CalcFileCRC32
    replies whose seq was not the one crc() is waiting for."""

    term_quiet = 3.0   # fetch's closing TerminateSession: seconds of quiet, and
    term_cap = 60      # the longest it waits for the link to go quiet

    def __init__(self, m, stall=5.0, log=print, max_bytes=MAX_BYTES):
        self.m = m
        self.stall = stall
        self.log = log
        self.max_bytes = max_bytes
        self.dropped = 0
        self.stale_crc = 0
        self.seq = 0
        self.last_size = 0   # bytes and seconds of the last successful fetch,
        self.last_dt = 0.0   # excluding the closing drain

    def _xmit(self, seq, op, size, offset, data):
        """Send one request with the given seq (send() takes the next one;
        crc() re-sends a stored one)."""
        hdr = struct.pack('<HBBBBBBI', seq, 0, op, size, 0, 0, 0, offset)
        self.m.mav.file_transfer_protocol_send(
            0, self.m.target_system, self.m.target_component, (hdr + data).ljust(251, b'\0'))

    def send(self, op, size=0, offset=0, data=b''):
        self.seq = (self.seq + 1) & 0xffff
        self._xmit(self.seq, op, size, offset, data)

    def recv(self, timeout):
        end = time.time() + timeout
        while True:
            x = self.m.recv_match(type='FILE_TRANSFER_PROTOCOL', blocking=True,
                                  timeout=max(end - time.time(), 0.001))
            if x is None:
                return None
            if (x.get_srcSystem() == self.m.target_system
                    and self.m.target_component in (0, x.get_srcComponent())
                    and x.target_system == self.m.mav.srcSystem
                    and x.target_component == self.m.mav.srcComponent):
                break
            self.dropped += 1
            if time.time() >= end:
                return None
        b = bytes(x.payload).ljust(12, b'\0')
        seq, _sess, opc, size, req, bc, _, ofs = struct.unpack('<HBBBBBBI', b[:12])
        return dict(seq=seq, op=opc, size=size, req=req, bc=bc, ofs=ofs, data=b[12:12 + size])

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

    def ask(self, op, path, timeout, size=None, offset=0, tries=3, deadline=None):
        """Send `op` and wait for its reply, up to `tries` times. `deadline` (an
        absolute time) ends the wait early with None."""
        enc = path.encode()
        for _ in range(tries):
            if deadline is not None and time.time() >= deadline:
                return None
            self.send(op, len(enc) if size is None else size, offset, enc)
            t = time.time()
            while time.time() - t < timeout and (deadline is None or time.time() < deadline):
                r = self.recv(0.5)
                if r and r['req'] == op:
                    return r
        return None

    def listdir(self, path):
        out, ofs = [], 0
        while True:
            r = self.ask(OP_LIST, path, 10, offset=ofs)
            if r is None:
                raise FtpError(f"no reply listing {inert(path)}")
            if r['op'] == NACK:
                d = r['data']
                if d[:1] == bytes([ERR_EOF]):
                    return out
                code = d[0] if d else None
                extra = ''
                if code is None:
                    what = 'unknown (no code)'
                else:
                    if code == ERR_FAILERRNO and len(d) > 1:
                        extra = f", errno {d[1]}"
                    what = f"{FTP_ERR.get(code, 'unknown')} (code {code}{extra})"
                raise FtpError(f"cannot list {inert(path)}: {what}")
            for e in r['data'].split(b'\0'):
                if e:
                    out.append(e.decode(errors='replace'))
                    ofs += 1
            if not r['data'].strip(b'\0'):
                return out

    def crc(self, remote, wait, progress=None):
        """Ask the board for CalcFileCRC32 of `remote` and wait up to `wait`
        seconds. Returns (status, value): ('ack', crc) for an ACK with 4 or
        more data bytes, ('nack', code or None), ('short', None) for an ACK
        with fewer, ('timeout', None).

        One request, one seq. The FC computes the CRC in its only FTP thread
        and answers nothing else until it is done, and each request with a new
        seq makes it compute again (GCS_FTP.cpp, ArduPilot 4.7.1 dbe79216), so
        a re-ask must reuse the seq: the worker then resends its cached reply
        (request seq + 1 == cached reply seq) without recomputing. The same
        request is re-sent at wait/3 and 2*wait/3 for a lost packet. Only a
        CRC reply with seq == sent seq + 1 is taken; other CRC replies are
        counted in stale_crc. `progress(elapsed_s)` is called about every 30 s."""
        enc = remote.encode()
        self.send(OP_CRC, len(enc), 0, enc)
        seq = self.seq
        want = (seq + 1) & 0xffff
        t0 = time.time()
        end = t0 + wait
        resend = [t0 + wait / 3, t0 + 2 * wait / 3]
        next_progress = t0 + 30
        while True:
            now = time.time()
            if now >= end:
                return ('timeout', None)
            if resend and now >= resend[0]:
                resend.pop(0)
                self._xmit(seq, OP_CRC, len(enc), 0, enc)
            if progress is not None and now >= next_progress:
                progress(now - t0)
                next_progress += 30
            r = self.recv(min(0.5, end - now))
            if r is None or r['req'] != OP_CRC or r['op'] not in (ACK, NACK):
                continue
            if r['seq'] != want:
                self.stale_crc += 1
                continue
            if r['op'] == NACK:
                return ('nack', r['data'][0] if r['data'] else None)
            if len(r['data']) >= 4:
                return ('ack', struct.unpack('<I', r['data'][:4])[0])
            return ('short', None)

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
        r = self.ask(OP_OPEN_RO, remote, 15 if timeout is None else min(15, timeout), deadline=deadline)
        if deadline is not None and time.time() >= deadline and not (r and r['op'] == ACK):
            raise FtpTimeout(f"{inert(remote)}: no open reply within the {timeout:.0f} s deadline")
        if not r or r['op'] != ACK:
            why = inert(r['data'][:2].decode('latin-1')) if r else 'no reply'
            raise FtpOpenError(f"open {inert(remote)} failed: {why}")
        if len(r['data']) < 4:
            raise FtpOpenError(f"open {inert(remote)}: reply too short")
        size = struct.unpack('<I', r['data'][:4])[0]
        if until_eof:
            self.log(f"open ok, {size} bytes announced (an estimate)")
            return self._read_to_eof(deadline)
        if size > self.max_bytes:
            raise FtpError(f"{inert(remote)}: {size} bytes announced, above the {self.max_bytes} cap")
        self.log(f"open ok, {size} bytes")
        chunks = {}          # offset -> data: memory follows what was delivered, not `size`
        nchunks = (size + CH - 1) // CH
        first = 0            # index of the first chunk not held; only moves forward
        t0 = last_print = time.time()
        self.send(OP_BURST, CH, 0)
        last_rx = time.time()
        while first < nchunks:
            r = self.recv(0.5)
            now = time.time()
            if deadline is not None and now > deadline:
                raise FtpTimeout(f"{inert(remote)}: {len(chunks)} of {nchunks} chunks after {now - t0:.0f} s, deadline passed")
            if r is not None:
                last_rx = now
                if (r['op'] == ACK and r['req'] == OP_BURST and 0 <= r['ofs'] < size
                        and r['ofs'] % CH == 0
                        and len(r['data']) == r['size'] == min(CH, size - r['ofs'])):
                    chunks[r['ofs']] = r['data']
                    while first < nchunks and first * CH in chunks:
                        first += 1
                done = r['req'] == OP_BURST and (r['bc'] or r['op'] == NACK)
            else:
                done = now - last_rx > self.stall
            if now - last_print > 10:
                self.log(f"  {len(chunks) * CH // 1024} of {size // 1024} KiB, {now - t0:.0f} s")
                last_print = now
            if done and first < nchunks:
                self.send(OP_BURST, CH, first * CH)
                last_rx = now
        buf = b''.join(chunks[i * CH] for i in range(nchunks))
        if len(buf) != size or len(chunks) != nchunks:
            raise FtpError(f"{inert(remote)}: {len(chunks)} of {nchunks} chunks, {len(buf)} of {size} bytes")
        self.last_dt = time.time() - t0
        self.last_size = size
        return buf

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
        first = 0          # index of the first chunk not held; only moves forward
        t0 = last_print = time.time()
        self.send(OP_BURST, CH, 0)
        last_rx = time.time()
        while True:
            r = self.recv(0.5)
            now = time.time()
            if deadline is not None and now > deadline:
                raise FtpTimeout(f"{len(chunks)} chunks after {now - t0:.0f} s, end "
                                   + ("not found" if end is None else f"at {end}"))
            if r is not None:
                last_rx = now
                if r['req'] == OP_BURST:
                    if (r['op'] == ACK and r['size'] and r['ofs'] % CH == 0
                            and len(r['data']) == r['size']
                            and not (end is not None and r['ofs'] >= end)):
                        if r['ofs'] + r['size'] > self.max_bytes:
                            raise FtpError(f"more than {self.max_bytes} bytes, giving up")
                        chunks[r['ofs']] = r['data']
                        while first * CH in chunks:
                            first += 1
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
                        if cand is not None:
                            # a chunk at or past the end is stray: drop it, take the end
                            for o in [o for o in chunks if o >= cand]:
                                del chunks[o]
                            first = min(first, cand // CH)
                            end = cand
                done = r['req'] == OP_BURST and (r['bc'] or r['op'] == NACK)
            else:
                done = now - last_rx > self.stall
            if end is not None:
                if first >= (end + CH - 1) // CH:
                    break
            if now - last_print > 10:
                self.log(f"  {len(chunks) * CH // 1024} KiB, {now - t0:.0f} s")
                last_print = now
            if done:
                gap = first * CH
                req = gap
                self.send(OP_BURST, CH, gap)
                last_rx = now
        self.last_dt = time.time() - t0
        data = b''.join(chunks[i * CH] for i in range((end + CH - 1) // CH))[:end]
        self.last_size = len(data)
        return data


# ---------------------------------------------------------------- selftest

class _Msg:
    def __init__(self, payload, src=(1, 1), target=(250, 190)):
        self.payload = payload
        self.src = src
        self.target_system, self.target_component = target

    def get_srcSystem(self):
        return self.src[0]

    def get_srcComponent(self):
        return self.src[1]


class _FakeFC:
    """A scripted FC standing in for the pymavlink connection `m`. Its burst
    read follows BurstReadFile in ArduPilot 4.7.1 (GCS_FTP.cpp): `burst`
    replies per request, a short read sets burst_complete, a read of 0 bytes
    is an EndOfFile NACK whose offset is the reply's last-set offset (0 if no
    data reply went out yet in the burst).

    Every reply carries seq = request seq + 1 (setup_reply). The worker is
    serial, as in 4.7.1: a CalcFileCRC32 with a new seq costs `crc_delay`
    seconds, one with the same seq as the last is answered from the cached
    reply at no cost, and no reply to any request goes out before the work
    queued ahead of it is done. `crc_nack` (an error code) answers the CRC with
    a NACK; `stale_crc` puts a CRC ACK with a wrong seq and a wrong value ahead
    of the real one."""

    def __init__(self, content, announced=None, burst=2000, drop=(), swallow=0,
                 stale=0, nack_open=False, inject=(), forge=None, open_data=None,
                 crc_data=None, listing=None, crc_delay=0.0, crc_nack=None,
                 stale_crc=False):
        self.content = content
        self.announced = len(content) if announced is None else announced
        self.burst = burst
        self.drop = set(drop)       # chunk offsets lost the first time they are sent
        self.swallow = swallow      # burst requests the FC ignores
        self.nack_open = nack_open
        self.inject = list(inject)  # messages put on the line at the first burst request
        self.forge = forge          # 'src' or 'target': a forged twin goes ahead of every burst reply
        self.open_data = open_data  # bytes: replaces the OPEN ACK's data
        self.crc_data = crc_data    # bytes: replaces the CRC ACK's data
        self.listing = list(listing or [])  # replies to ListDirectory, one per request: (ACK|NACK, data)
        self.crc_delay = crc_delay
        self.crc_nack = crc_nack
        self.stale_crc = stale_crc
        self.crc_seqs = []          # the seq of every CRC request, duplicates included
        self.delayed = []           # (ready_time, packet), in worker order
        self.worker_free = 0.0
        self._last_crc_seq = None
        self._rseq = 0
        self.requests = []          # (op, size, offset) as the tool sent them
        self.nack_offsets = []
        self.q = [self._pkt(ACK, OP_BURST, CH, 0, b'\xaa' * CH) for _ in range(stale)]
        self.target_system = self.target_component = 1
        fc = self

        class _Mav:
            srcSystem, srcComponent = 250, 190

            def file_transfer_protocol_send(self, net, sysid, comp, payload):
                fc._on_request(bytes(payload))
        self.mav = _Mav()

    @staticmethod
    def _pkt(op, req, size, ofs, data=b'', bc=0, seq=0):
        hdr = struct.pack('<HBBBBBBI', seq, 0, op, size, req, bc, 0, ofs)
        return _Msg((hdr + data).ljust(251, b'\0'))

    def _p(self, *a, seq=None):
        return self._pkt(*a, seq=self._rseq if seq is None else seq)

    def recv_match(self, type=None, blocking=True, timeout=None):
        if self.delayed:
            now = time.time()
            while self.delayed and self.delayed[0][0] <= now:
                self.q.append(self.delayed.pop(0)[1])
        if self.q:
            return self.q.pop(0)
        time.sleep(min(timeout or 0.01, 0.01))
        return None

    def _on_request(self, b):
        seq, _sess, op = struct.unpack('<HBB', b[:4])
        self._rseq = (seq + 1) & 0xffff
        now = time.time()
        start = max(now, self.worker_free)
        cost = 0.0
        if op == OP_CRC and self.crc_nack is None:
            cost = 0.0 if seq == self._last_crc_seq else self.crc_delay
            self.crc_seqs.append(seq)
            self._last_crc_seq = seq
        n0 = len(self.q)
        self._handle(b)
        self.worker_free = start + cost
        if self.worker_free > now + 1e-9:
            new = self.q[n0:]
            del self.q[n0:]
            self.delayed.extend((self.worker_free, p) for p in new)

    def _handle(self, b):
        _s, _sess, op, size, _r, _bc, _z, ofs = struct.unpack('<HBBBBBBI', b[:12])
        self.requests.append((op, size, ofs))
        if op == OP_RESET or op == OP_TERM:
            self.q.append(self._p(ACK, op, 0, 0))
        elif op == OP_LIST:
            if self.listing:
                kind, d = self.listing.pop(0)
                self.q.append(self._p(kind, op, len(d), ofs, d))
        elif op == OP_OPEN_RO:
            if self.nack_open:
                self.q.append(self._p(NACK, op, 2, 0, bytes([10, 0])))
            else:
                d = struct.pack('<I', self.announced) if self.open_data is None else self.open_data
                self.q.append(self._p(ACK, op, len(d), 0, d))
        elif op == OP_CRC:
            if self.crc_nack is not None:
                self.q.append(self._p(NACK, op, 1, 0, bytes([self.crc_nack])))
                return
            if self.stale_crc:
                self.q.append(self._p(ACK, op, 4, 0, struct.pack('<I', 0xdeadbeef),
                                      seq=(self._rseq + 5) & 0xffff))
            d = struct.pack('<I', ap_crc32(self.content)) if self.crc_data is None else self.crc_data
            self.q.append(self._p(ACK, op, len(d), 0, d))
        elif op == OP_BURST:
            self.q.extend(self.inject)
            self.inject = []
            if self.swallow > 0:
                self.swallow -= 1
                return
            off = 0
            bc = 0
            for i in range(self.burst):
                chunk = self.content[ofs + i * CH: ofs + (i + 1) * CH]
                if not chunk:
                    self.nack_offsets.append(off)
                    self.q.append(self._p(NACK, op, 1, off, bytes([ERR_EOF]), bc))
                    return
                off = ofs + i * CH
                bc = 1 if (len(chunk) < CH or i == self.burst - 1) else 0
                if off in self.drop:
                    self.drop.discard(off)
                else:
                    if self.forge:
                        bad = _Msg(self._p(ACK, op, len(chunk), off, b'\xee' * len(chunk), bc).payload,
                                   src={'src': (2, 1), 'comp': (1, 2)}.get(self.forge, (1, 1)),
                                   target=(251, 190) if self.forge == 'target' else (250, 190))
                        self.q.append(bad)
                    self.q.append(self._p(ACK, op, len(chunk), off, chunk, bc))
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

    def session(fc, stall=0.3, **kw):
        s = FtpSession(fc, stall=stall, log=lambda *_: None, **kw)
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

    @case("ap_crc32 (zlib form) equals the table loop on lengths 0 to 100003")
    def _():
        for n in (0, 1, 63, 64, 65, 1000, 100003):
            d = rnd.randbytes(n)
            assert ap_crc32(d) == _ap_crc32_table(d), n

    pkt = _FakeFC._pkt

    @case("known size: a chunk at or past the announced size yields no file")
    def _():
        fc = _FakeFC(blob(CH), swallow=10 ** 9,
                     inject=[pkt(ACK, OP_BURST, CH, CH * 5, b'\xee' * CH, 1)])
        try:
            session(fc, stall=0.2).fetch('/x', timeout=0.8)
        except (FtpError, TimeoutError):
            pass
        else:
            raise AssertionError("a file came back")

    @case("a chunk whose data length differs from its header size is ignored")
    def _():
        data = blob(CH * 3)
        bad = pkt(ACK, OP_BURST, 250, 0, b'\xee' * 239)
        fc = _FakeFC(data, inject=[bad], drop={0})
        same(session(fc).fetch('/x'), data)
        fc = _FakeFC(data, inject=[bad], drop={0})
        same(session(fc).fetch('/x', until_eof=True), data)

    @case("replies from another source, or addressed to another target, are dropped and counted")
    def _():
        data = blob(CH * 4 + 9)
        for forge in ('src', 'comp', 'target'):
            for eof in (False, True):
                fc = _FakeFC(data, forge=forge)
                s = session(fc)
                same(s.fetch('/x', until_eof=eof), data)
                assert s.dropped > 0, (forge, eof, s.dropped)

    @case("target_component 0 (pymavlink never sets it) accepts any component of the peer system")
    def _():
        data = blob(CH * 2 + 5)
        fc = _FakeFC(data)
        fc.target_component = 0
        s = session(fc)
        same(s.fetch('/x'), data)
        assert s.dropped == 0

    @case("an announced size above max_bytes is refused before any data is read")
    def _():
        fc = _FakeFC(blob(5000))
        try:
            session(fc, max_bytes=1000).fetch('/x')
        except FtpError:
            pass
        else:
            raise AssertionError("no FtpError")
        assert not sent(fc, OP_BURST), fc.requests
        assert fc.requests[-1][0] == OP_TERM

    @case("a CRC ACK with fewer than 4 data bytes is ('short', None); a full one is ('ack', crc)")
    def _():
        fc = _FakeFC(blob(100), crc_data=b'')
        assert session(fc).crc('/x', 2) == ('short', None)
        fc = _FakeFC(blob(100), crc_data=b'\x01\x02')
        assert session(fc).crc('/x', 2) == ('short', None)
        data = blob(100)
        assert session(_FakeFC(data)).crc('/x', 2) == ('ack', ap_crc32(data))

    @case("an OPEN ACK with an empty payload raises FtpOpenError")
    def _():
        for eof in (False, True):
            fc = _FakeFC(blob(100), open_data=b'')
            try:
                session(fc).fetch('/x', until_eof=eof)
            except FtpOpenError:
                pass
            else:
                raise AssertionError("no FtpOpenError")
            assert fc.requests[-1][0] == OP_TERM

    @case("until_eof: data past max_bytes raises FtpError")
    def _():
        fc = _FakeFC(blob(5000))
        try:
            session(fc, max_bytes=1000).fetch('/x', until_eof=True)
        except FtpError:
            pass
        else:
            raise AssertionError("no FtpError")

    @case("until_eof: a chunk past a known end is ignored")
    def _():
        data = blob(CH + 10)
        late = [pkt(ACK, OP_BURST, 10, CH, b'\x01' * 10, 1),     # fixes the end at 249
                pkt(ACK, OP_BURST, CH, CH * 2, b'\xee' * CH),
                pkt(ACK, OP_BURST, CH, CH * 3, b'\xee' * CH)]
        fc = _FakeFC(data, inject=late)
        same(session(fc, max_bytes=600).fetch('/x', until_eof=True)[:CH], data[:CH])

    @case("inert maps ESC, BEL and CR, DEL and C1 to \\xNN and leaves the rest")
    def _():
        assert inert('a\x1b[2K\x07\rb') == 'a\\x1b[2K\\x07\\x0db'
        assert inert('\x7f\x85\x9f') == '\\x7f\\x85\\x9f'
        assert inert('plain \u00e9 text') == 'plain \u00e9 text'
        assert inert('a\u202eb\u200bc') == 'a\\u202eb\\u200bc'
        assert inert('\u2028\u2029') == '\\u2028\\u2029'

    @case("wait_autopilot skips GCS and gimbal heartbeats and pins the autopilot's system and component")
    def _():
        class _HB:
            def __init__(self, sysid, comp, autopilot, typ=2):
                self.sc, self.autopilot, self.type = (sysid, comp), autopilot, typ

            def get_srcSystem(self):
                return self.sc[0]

            def get_srcComponent(self):
                return self.sc[1]

        class _Conn:
            target_system = target_component = 0
            q = [_HB(255, 190, MAV_AUTOPILOT_INVALID), _HB(1, 154, MAV_AUTOPILOT_INVALID),
                 _HB(254, 1, 3, MAV_TYPE_GCS), _HB(1, 1, 3)]

            def recv_match(self, type=None, blocking=True, timeout=None):
                return self.q.pop(0) if self.q else None
        c = _Conn()
        hb = wait_autopilot(c, 2)
        assert hb is not None and (c.target_system, c.target_component) == (1, 1), \
            (c.target_system, c.target_component)
        c2 = _Conn()
        c2.q = [_HB(255, 190, MAV_AUTOPILOT_INVALID)]
        assert wait_autopilot(c2, 0.3) is None
        c3 = _Conn()    # broadcast ids 0/0 are not a vehicle; the real autopilot follows
        c3.q = [_HB(0, 0, 3), _HB(0, 1, 3), _HB(1, 0, 3), _HB(1, 1, 3)]
        assert wait_autopilot(c3, 2) is not None and (c3.target_system, c3.target_component) == (1, 1), \
            (c3.target_system, c3.target_component)

    def lister(fc):
        s = session(fc)
        ask = s.ask
        s.ask = lambda op, path, timeout, **kw: ask(op, path, 0.3, tries=1, **kw)
        return s

    def err_of(fn):
        try:
            fn()
        except FtpError as e:
            return str(e)
        raise AssertionError("no FtpError")

    @case("listdir: entries in order, ends on the EndOfFile NACK")
    def _():
        fc = _FakeFC(b'', listing=[(ACK, b'Dsub\0Fa.bin\t12\0'), (ACK, b'Fb.bin\t3\0'),
                                   (NACK, bytes([ERR_EOF]))])
        assert lister(fc).listdir('/d') == ['Dsub', 'Fa.bin\t12', 'Fb.bin\t3']
        assert [r[2] for r in sent(fc, OP_LIST)] == [0, 2, 3], sent(fc, OP_LIST)

    @case("listdir: an empty directory (EndOfFile NACK at offset 0) gives []")
    def _():
        assert lister(_FakeFC(b'', listing=[(NACK, bytes([ERR_EOF]))])).listdir('/d') == []

    @case("listdir: a missing directory (NACK FileNotFound) raises, naming code and path")
    def _():
        m = err_of(lambda: lister(_FakeFC(b'', listing=[(NACK, bytes([10]))])).listdir('/nope'))
        assert m == "cannot list /nope: FileNotFound (code 10)", m

    @case("listdir: FailErrno carries its errno byte; the path is made inert")
    def _():
        m = err_of(lambda: lister(_FakeFC(b'', listing=[(NACK, bytes([2, 2]))])).listdir('/a\x1bb'))
        assert m == "cannot list /a\\x1bb: FailErrno (code 2, errno 2)", m
        m = err_of(lambda: lister(_FakeFC(b'', listing=[(NACK, bytes([99]))])).listdir('/d'))
        assert m == "cannot list /d: unknown (code 99)", m

    @case("listdir: a non-EOF NACK after a first page raises, no partial list")
    def _():
        fc = _FakeFC(b'', listing=[(ACK, b'Fa\0'), (NACK, bytes([1]))])
        m = err_of(lambda: lister(fc).listdir('/d'))
        assert m == "cannot list /d: Fail (code 1)", m

    @case("listdir: a NACK with no data is not EndOfFile")
    def _():
        err_of(lambda: lister(_FakeFC(b'', listing=[(NACK, b'')])).listdir('/d'))

    @case("listdir: no reply at all raises 'no reply listing'")
    def _():
        m = err_of(lambda: lister(_FakeFC(b'')).listdir('/d'))
        assert m == "no reply listing /d", m

    @case("memory follows delivered data: a 1 GiB announce with no data stays under 64 MiB")
    def _():
        def rss():
            for ln in open('/proc/self/status'):
                if ln.startswith('VmRSS'):
                    return int(ln.split()[1]) / 1024
        try:
            open('/proc/self/status').close()
        except OSError:
            print("SKIP  1 GiB announce (no /proc/self/status)")
            return
        fc = _FakeFC(b'', announced=1 << 30, swallow=10 ** 9)
        s = session(fc, stall=0.2)
        assert s.max_bytes == MAX_BYTES
        base, top = rss(), []
        real = s.recv
        s.recv = lambda t: (top.append(rss()), real(t))[1]
        try:
            s.fetch('/x', timeout=1.0)
        except TimeoutError:
            pass
        else:
            raise AssertionError("no TimeoutError")
        assert top and max(top) - base < 64, (base, max(top))

    @case("known size and until_eof: out-of-order and duplicated chunks give the source bytes")
    def _():
        data = blob(CH * 6 + 17)
        chunk = lambda i: data[i * CH:(i + 1) * CH]
        order = [5, 3, 3, 0, 4, 4, 1, 5, 6, 6, 0]
        inj = [pkt(ACK, OP_BURST, len(chunk(i)), i * CH, chunk(i)) for i in order]
        for eof in (False, True):
            fc = _FakeFC(data, inject=list(inj), drop={CH * 2})
            same(session(fc).fetch('/x', until_eof=eof), data)

    @case("a peer answering every burst with bc=1 and no data: time per round stays flat")
    def _():
        class _Stop(Exception):
            pass

        class Chatty(_FakeFC):
            def __init__(self):
                super().__init__(b'', announced=CH * 400000)
                self.ts = []

            def _on_request(self, b):
                op = b[3]
                if op != OP_BURST:
                    return super()._on_request(b)
                self.ts.append(time.perf_counter())
                if len(self.ts) > 2000:
                    raise _Stop
                self.q.append(self._pkt(ACK, OP_BURST, 0, 0, b'', 1))
        for eof in (False, True):
            fc = Chatty()
            try:
                session(fc).fetch('/x', until_eof=eof)
            except _Stop:
                pass
            ts = fc.ts
            first, last = ts[100] - ts[0], ts[-1] - ts[-101]
            assert last <= 2 * first + 0.005, (eof, first, last)

    @case("until_eof: one stray chunk past the EOF end does not stop the read")
    def _():
        data = blob(CH * 7)
        fc = _FakeFC(data, inject=[pkt(ACK, OP_BURST, CH, CH * 50, b'\xee' * CH)])
        t = time.time()
        same(session(fc).fetch('/x', until_eof=True, timeout=5), data)
        assert time.time() - t < 3

    @case("the deadline: a silent FC with timeout 1 s raises FtpTimeout (an FtpError) in about 1 s")
    def _():
        class Silent(_FakeFC):
            def _on_request(self, b):
                self.requests.append((b[3], 0, 0))
        for eof in (False, True):
            fc = Silent(b'')
            t = time.time()
            try:
                session(fc).fetch('/x', until_eof=eof, timeout=1.0)
            except FtpTimeout as e:
                assert isinstance(e, FtpError) and isinstance(e, TimeoutError)
            else:
                raise AssertionError("no FtpTimeout")
            dt = time.time() - t
            assert 0.9 <= dt < 2.0, dt

    @case("output files: a planted <local>.part or .crc-mismatch symlink is not written through")
    def _():
        import shutil
        d = tempfile.mkdtemp()
        try:
            victim = os.path.join(d, 'victim')
            open(victim, 'wb').write(b'keep')
            local = os.path.join(d, 'out.BIN')
            for suffix in ('.part', '.crc-mismatch'):
                os.symlink(victim, local + suffix)
            _write_atomic(local + '.crc-mismatch', b'bad')
            _write_atomic(local, b'good')
            assert open(victim, 'rb').read() == b'keep'
            assert open(local, 'rb').read() == b'good'
            bad = local + '.crc-mismatch'
            assert not os.path.islink(bad) and open(bad, 'rb').read() == b'bad'
            um = os.umask(0)
            os.umask(um)
            assert os.stat(local).st_mode & 0o777 == 0o666 & ~um
            assert sorted(os.listdir(d)) == ['out.BIN', 'out.BIN.crc-mismatch', 'out.BIN.part', 'victim']
        finally:
            shutil.rmtree(d)

    # ---- CalcFileCRC32: one request, one seq (the FC recomputes per new seq)

    import contextlib
    import io
    import shutil
    import types

    def run_get(fc, timeout=30, crc_wait=5, no_crc=False, data=None):
        """The CLI path (_run) on the fake: (exit code, output, files in the
        output directory, the directory's contents by name)."""
        d = tempfile.mkdtemp()
        try:
            a = types.SimpleNamespace(list=None, last_log=None, remote='/x', local=os.path.join(d, 'out.BIN'),
                                      timeout=timeout, crc_wait=crc_wait, no_crc=no_crc)
            s = session(fc)
            real_drain = s.drain
            s.drain = lambda quiet=4.0, limit=180: real_drain(0.05, 5)
            buf = io.StringIO()
            with contextlib.redirect_stdout(buf), contextlib.redirect_stderr(buf):
                try:
                    code = _run(a, s)
                except SystemExit as e:
                    code = e.code
            files = {n: open(os.path.join(d, n), 'rb').read() for n in os.listdir(d)}
            return code, buf.getvalue(), files
        finally:
            shutil.rmtree(d)

    @case("crc(): a CRC slower than the wait gives ('timeout', None) and one distinct seq")
    def _():
        fc = _FakeFC(blob(100), crc_delay=1.5)
        t = time.time()
        assert session(fc).crc('/x', 0.6) == ('timeout', None)
        assert 0.55 <= time.time() - t < 1.2
        assert len(set(fc.crc_seqs)) == 1 and 1 <= len(fc.crc_seqs) <= 3, fc.crc_seqs

    @case("crc(): a lost request is re-sent with the same seq and answered (no second computation)")
    def _():
        fc = _FakeFC(blob(100), crc_delay=0.0)
        s = session(fc)
        n = [0]
        send = fc.mav.file_transfer_protocol_send

        def lossy(net, sysid, comp, payload):
            n[0] += 1
            if n[0] > 1:
                send(net, sysid, comp, payload)
        fc.mav.file_transfer_protocol_send = lossy
        assert s.crc('/x', 1.5) == ('ack', ap_crc32(fc.content))
        assert len(set(fc.crc_seqs)) == 1, fc.crc_seqs

    @case("crc(): a CRC ACK with the wrong seq ahead of the real one is ignored and counted")
    def _():
        data = blob(100)
        fc = _FakeFC(data, stale_crc=True)
        s = session(fc)
        assert s.crc('/x', 2) == ('ack', ap_crc32(data))
        assert s.stale_crc == 1 and s.dropped == 0, (s.stale_crc, s.dropped)

    @case("crc(): the reply seq is the sent seq + 1 modulo 65536")
    def _():
        data = blob(100)
        fc = _FakeFC(data)
        s = session(fc)
        s.seq = 0xfffe
        assert s.crc('/x', 2) == ('ack', ap_crc32(data))
        assert fc.crc_seqs == [0xffff], fc.crc_seqs
        assert s.stale_crc == 0

    @case("crc(): a NACK gives ('nack', code)")
    def _():
        assert session(_FakeFC(blob(10), crc_nack=2)).crc('/x', 2) == ('nack', 2)

    @case("get: CRC slower than --crc-wait: exit 3, bytes kept as .unverified, UNVERIFIED not MISMATCH")
    def _():
        data = blob(3000)
        fc = _FakeFC(data, crc_delay=2.0)
        code, out, files = run_get(fc, crc_wait=0.6)
        assert code == 3, (code, out)
        assert files == {'out.BIN.unverified': data}, sorted(files)
        assert 'UNVERIFIED' in out and 'MISMATCH' not in out, out
        assert 'stays busy' in out and hex(ap_crc32(data)) in out, out
        assert len(set(fc.crc_seqs)) == 1, fc.crc_seqs
        assert fc.requests[-1][0] == OP_TERM

    @case("get: CRC faster than --crc-wait (default wait): VERIFIED, exit 0, no .unverified left")
    def _():
        data = blob(3000)
        fc = _FakeFC(data, crc_delay=0.3)
        code, out, files = run_get(fc, crc_wait=None)
        assert code == 0, (code, out)
        assert files == {'out.BIN': data}, sorted(files)
        assert f"VERIFIED: CRC {hex(ap_crc32(data))} matches the board" in out, out
        assert 'waiting up to 120 s' in out, out

    @case("get: a wrong CRC from the board: MISMATCH with both values, exit 1, kept as .crc-mismatch")
    def _():
        data = blob(3000)
        fc = _FakeFC(data, crc_data=struct.pack('<I', 0x12345678))
        code, out, files = run_get(fc)
        assert code == 1, (code, out)
        assert files == {'out.BIN.crc-mismatch': data}, sorted(files)
        assert f"CRC MISMATCH: board 0x12345678, local {hex(ap_crc32(data))}" in out, out

    @case("get: a NACKed CRC: UNVERIFIED naming the code, exit 3")
    def _():
        data = blob(3000)
        code, out, files = run_get(_FakeFC(data, crc_nack=2))
        assert code == 3, (code, out)
        assert 'UNVERIFIED' in out and 'FailErrno (code 2)' in out and 'MISMATCH' not in out, out
        assert files == {'out.BIN.unverified': data}, sorted(files)

    @case("get: a short CRC reply: UNVERIFIED, exit 3, never MISMATCH")
    def _():
        data = blob(3000)
        code, out, files = run_get(_FakeFC(data, crc_data=b'\x01'))
        assert code == 3 and 'UNVERIFIED' in out and 'short' in out and 'MISMATCH' not in out, (code, out)
        assert list(files) == ['out.BIN.unverified']

    @case("get: a stale CRC ACK ahead of the real one is ignored: VERIFIED")
    def _():
        data = blob(3000)
        code, out, files = run_get(_FakeFC(data, stale_crc=True))
        assert code == 0 and files == {'out.BIN': data}, (code, out, sorted(files))

    @case("get --no-crc: no CRC request is sent, exit 3, bytes kept as .unverified")
    def _():
        data = blob(3000)
        fc = _FakeFC(data)
        code, out, files = run_get(fc, no_crc=True)
        assert code == 3 and 'UNVERIFIED' in out and '--no-crc' in out, (code, out)
        assert files == {'out.BIN.unverified': data}, sorted(files)
        assert not sent(fc, OP_CRC) and fc.crc_seqs == []

    @case("get: --timeout bounds the download only; a CRC slower than --timeout still verifies")
    def _():
        data = blob(3000)
        fc = _FakeFC(data, crc_delay=2.5)
        code, out, files = run_get(fc, timeout=2, crc_wait=8)
        assert code == 0 and files == {'out.BIN': data}, (code, out, sorted(files))

    @case("--crc-wait must be above 0")
    def _():
        buf = io.StringIO()
        for bad in ('0', '-3'):
            with contextlib.redirect_stderr(buf):
                try:
                    main(['--crc-wait', bad, '/x', 'out/y'])
                except SystemExit as e:
                    assert e.code == 2, e.code
                else:
                    raise AssertionError("accepted")
        assert '--crc-wait must be above 0' in buf.getvalue()

    @case("output files: a planted <local>.unverified symlink is replaced, not written through; rename onto a planted .crc-mismatch link too")
    def _():
        d = tempfile.mkdtemp()
        try:
            victim = os.path.join(d, 'victim')
            open(victim, 'wb').write(b'keep')
            local = os.path.join(d, 'out.BIN')
            for suffix in ('.unverified', '.crc-mismatch'):
                os.symlink(victim, local + suffix)
            _write_atomic(local + '.unverified', b'u1')
            assert not os.path.islink(local + '.unverified')
            _write_atomic(local + '.unverified', b'u2')       # a stale one is simply overwritten
            os.replace(local + '.unverified', local + '.crc-mismatch')
            assert not os.path.islink(local + '.crc-mismatch')
            assert open(local + '.crc-mismatch', 'rb').read() == b'u2'
            assert open(victim, 'rb').read() == b'keep'
            assert sorted(os.listdir(d)) == ['out.BIN.crc-mismatch', 'victim']
        finally:
            shutil.rmtree(d)

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


MAV_AUTOPILOT_INVALID = 8   # MAVLink common.xml: a heartbeat from something that is not an autopilot
MAV_TYPE_GCS = 6            # MAVLink common.xml MAV_TYPE: a ground station, never the vehicle


def wait_autopilot(m, timeout):
    """Wait for an autopilot's heartbeat and pin the connection to its system and
    component, for FtpSession's reply check. pymavlink sets only target_system,
    and wait_heartbeat returns the first heartbeat of any kind: on a shared link
    that can be a GCS, a gimbal or a companion computer, and pinning to its
    component would drop every reply from the autopilot. Returns the heartbeat,
    or None at the timeout."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        hb = m.recv_match(type='HEARTBEAT', blocking=True, timeout=max(0.1, deadline - time.time()))
        if hb is None:
            continue
        if hb.autopilot == MAV_AUTOPILOT_INVALID or getattr(hb, 'type', None) == MAV_TYPE_GCS:
            continue
        if not hb.get_srcSystem() or not hb.get_srcComponent():   # 0 is broadcast, not a vehicle
            continue
        if m.target_system not in (0, hb.get_srcSystem()):
            continue
        m.target_system = hb.get_srcSystem()
        m.target_component = hb.get_srcComponent()
        return hb
    return None


def main(argv=None):
    ap = argparse.ArgumentParser(
        description=__doc__.split('\n\n')[0], formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="exit codes: 0 VERIFIED (or a listing); 1 failure or CRC MISMATCH; "
               "3 UNVERIFIED (the board gave no CRC, refused it, or --no-crc: the bytes are kept "
               "in LOCAL.unverified)")
    ap.add_argument('remote', nargs='?', help='file on the FC, e.g. /APM/LOGS/00000011.BIN')
    ap.add_argument('local', nargs='?', help='where to write it (a directory keeps the remote name)')
    ap.add_argument('--list', metavar='DIR', help='list a directory on the FC and exit')
    ap.add_argument('--last-log', metavar='DIR', help='fetch the log LASTLOG.TXT names into DIR')
    ap.add_argument('--device', help='default: first ArduPilot USB serial device (-if00)')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--stall', type=float, default=5.0, help='seconds of silence before re-asking (default 5)')
    ap.add_argument('--max-mib', type=int, default=MAX_BYTES >> 20,
                    help='refuse a file larger than this many MiB (default 2048)')
    ap.add_argument('--timeout', type=float, default=600.0, metavar='S',
                    help='seconds allowed for the download, --last-log included (default 600); '
                         'not applied to --list or to the CRC wait (see --crc-wait)')
    ap.add_argument('--crc-wait', type=float, default=None, metavar='S',
                    help="seconds to wait for the board's CRC after the download "
                         "(default: the larger of 120 and twice the download's seconds)")
    ap.add_argument('--no-crc', action='store_true',
                    help="do not ask for the board's CRC: write LOCAL.unverified and exit 3")
    ap.add_argument('--selftest', action='store_true', help='run the offline tests and exit')
    a = ap.parse_args(argv)
    if a.selftest:
        return _selftest()
    if not (a.list or a.last_log or (a.remote and a.local)):
        ap.error('give REMOTE LOCAL, --list DIR or --last-log DIR')
    if not a.timeout > 0:
        ap.error('--timeout must be above 0')
    if a.crc_wait is not None and not a.crc_wait > 0:
        ap.error('--crc-wait must be above 0')

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
    hb = wait_autopilot(m, 25)
    if hb is None:
        sys.exit(f"no autopilot heartbeat on {dev}")
    s = FtpSession(m, stall=a.stall, log=lambda t: print(t, flush=True),
                   max_bytes=a.max_mib << 20)
    try:
        return _run(a, s)
    finally:
        if s.dropped:
            print(f"skipped {s.dropped} replies not from the heartbeat peer or not addressed to this connection",
                  file=sys.stderr, flush=True)


def _write_atomic(path, data):
    """Write `data` to a fresh temporary file beside `path` (mkstemp: a random
    name, O_EXCL, so a planted symlink is never followed) and rename it over
    `path`. Mode 0o666 & ~umask, as ap-param-dump.py does. Nothing is left
    behind on a failure."""
    d = os.path.dirname(os.path.abspath(path))
    fd, tmp = tempfile.mkstemp(dir=d, prefix=os.path.basename(path) + '.', suffix='.part')
    try:
        with os.fdopen(fd, 'wb') as f:
            f.write(data)
        um = os.umask(0)
        os.umask(um)
        os.chmod(tmp, 0o666 & ~um)
        os.replace(tmp, path)
    except BaseException:
        if os.path.exists(tmp):
            os.unlink(tmp)
        raise


def _run(a, s):
    deadline = None

    def get(remote, local):
        try:
            buf = s.fetch(remote, timeout=max(deadline - time.time(), 0.001))
        except FtpError as e:
            sys.exit(str(e))
        size, dt = s.last_size, s.last_dt
        print(f"got {size} bytes in {dt:.1f} s, {size / max(dt, 1e-3) / 1024:.1f} KiB/s", flush=True)
        local_crc = ap_crc32(buf)
        # Keep the download before the long wait: a Ctrl-C or a dropped tunnel
        # then costs only the check, not the transfer.
        unv = local + '.unverified'
        _write_atomic(unv, buf)
        if a.no_crc:
            print(f"UNVERIFIED: --no-crc, the board's CRC was not asked for; local CRC {hex(local_crc)}; "
                  f"kept as {unv}", flush=True)
            return 3
        wait = a.crc_wait if a.crc_wait is not None else max(120.0, 2 * dt)
        print(f"asking the board for its CRC; waiting up to {wait:.0f} s", flush=True)
        status, val = s.crc(remote, wait, progress=lambda n: print(
            f"  waiting for the board's CRC, {n:.0f} s (MAVFTP on the FC is busy until it answers)", flush=True))
        s.terminate(drain=False)
        if status == 'ack' and val == local_crc:
            os.replace(unv, local)
            print(f"VERIFIED: CRC {hex(local_crc)} matches the board; wrote {local}")
            return 0
        if status == 'ack':
            bad = local + '.crc-mismatch'
            os.replace(unv, bad)
            print(f"CRC MISMATCH: board {hex(val)}, local {hex(local_crc)}; kept as {bad}", file=sys.stderr)
            return 1
        if status == 'timeout':
            why = (f"the board gave no CRC within {wait:.0f} s. The FC's MAVFTP stays busy until its CRC "
                   f"finishes, which may take as long again: wait before the next MAVFTP command")
        elif status == 'nack':
            why = ("the board refused the CRC: no error code" if val is None else
                   f"the board refused the CRC: {FTP_ERR.get(val, 'unknown')} (code {val})")
        else:
            why = "the board's CRC reply was short"
        print(f"UNVERIFIED: {why}; local CRC {hex(local_crc)}; kept as {unv}", flush=True)
        return 3

    n, dt = s.reset()
    print(f"sessions reset; drained {n} stale replies in {dt:.1f} s", flush=True)

    if a.list:
        try:
            entries = s.listdir(a.list)
        except FtpError as e:
            sys.exit(str(e))
        if not entries:
            print('(empty)')
        for e in entries:
            kind, rest = e[0], e[1:]
            print(('dir  ' if kind == 'D' else 'file ') + inert(rest.replace('\t', '  ')))
        s.terminate(drain=False)
        return 0

    deadline = time.time() + a.timeout
    if a.last_log:
        r = s.ask(OP_OPEN_RO, '/APM/LOGS/LASTLOG.TXT', 15, deadline=deadline)
        if not r or r['op'] != ACK:
            sys.exit("cannot open /APM/LOGS/LASTLOG.TXT")
        s.send(OP_BURST, CH, 0)
        txt = b''
        t = time.time()
        while time.time() - t < 15 and time.time() < deadline:
            x = s.recv(0.5)
            if x and x['req'] == OP_BURST and x['op'] == ACK:
                txt = x['data']; break
        s.terminate(quiet=3.0, cap=30)
        digits = txt.strip()
        num = int(digits) if digits.isdigit() and len(digits) <= 8 else 0
        if num <= 0 and time.time() >= deadline:
            sys.exit(f"--timeout {a.timeout:g} s passed reading LASTLOG.TXT")
        if num <= 0:
            sys.exit(f"LASTLOG.TXT unreadable: {txt!r}")
        a.remote = f'/APM/LOGS/{num:08d}.BIN'
        a.local = a.last_log
        print(f"LASTLOG.TXT names {a.remote}")

    local = a.local
    if os.path.isdir(local):
        local = os.path.join(local, os.path.basename(a.remote))
    return get(a.remote, local)


if __name__ == '__main__':
    sys.exit(main())
