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

Needs pymavlink (pip install pymavlink).
"""
import argparse, glob, os, struct, sys, time

# opcodes, GCS_FTP.h FTP_OP
OP_TERM, OP_RESET, OP_LIST, OP_OPEN_RO, OP_CRC, OP_BURST = 1, 2, 3, 4, 14, 15
ACK, NACK = 128, 129
CH = 239  # bytes of data per FILE_TRANSFER_PROTOCOL reply

ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
ap.add_argument('remote', nargs='?', help='file on the FC, e.g. /APM/LOGS/00000011.BIN')
ap.add_argument('local', nargs='?', help='where to write it (a directory keeps the remote name)')
ap.add_argument('--list', metavar='DIR', help='list a directory on the FC and exit')
ap.add_argument('--last-log', metavar='DIR', help='fetch the log LASTLOG.TXT names into DIR')
ap.add_argument('--device', help='default: first ArduPilot USB serial device (-if00)')
ap.add_argument('--baud', type=int, default=115200)
ap.add_argument('--stall', type=float, default=5.0, help='seconds of silence before re-asking (default 5)')
a = ap.parse_args()
if not (a.list or a.last_log or (a.remote and a.local)):
    ap.error('give REMOTE LOCAL, --list DIR or --last-log DIR')

try:
    from pymavlink import mavutil
except ImportError:
    sys.exit("pymavlink not found: pip install pymavlink")

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


def find_device():
    for pat in ('/dev/serial/by-id/*ArduPilot*-if00', '/dev/serial/by-id/*ArduPilot*'):
        hits = sorted(glob.glob(pat))
        if hits:
            return hits[0]
    sys.exit("no flight controller found; pass --device")


dev = a.device or find_device()
print(f"connecting to {dev} ...", flush=True)
m = mavutil.mavlink_connection(dev, baud=a.baud, source_system=250, source_component=190)
if m.wait_heartbeat(timeout=25) is None:
    sys.exit(f"no heartbeat on {dev}")
seq = 0


def send(op, size=0, offset=0, data=b''):
    global seq
    seq = (seq + 1) & 0xffff
    hdr = struct.pack('<HBBBBBBI', seq, 0, op, size, 0, 0, 0, offset)
    m.mav.file_transfer_protocol_send(0, m.target_system, m.target_component, (hdr + data).ljust(251, b'\0'))


def recv(timeout):
    x = m.recv_match(type='FILE_TRANSFER_PROTOCOL', blocking=True, timeout=timeout)
    if x is None:
        return None
    b = bytes(x.payload)
    _s, _sess, opc, size, req, bc, _, ofs = struct.unpack('<HBBBBBBI', b[:12])
    return dict(op=opc, size=size, req=req, bc=bc, ofs=ofs, data=b[12:12 + size])


def drain(quiet=4.0, limit=180):
    t = time.time(); last = t; n = 0
    while time.time() - last < quiet and time.time() - t < limit:
        if recv(0.5) is not None:
            last = time.time(); n += 1
    return n, time.time() - t


def ask(op, path, timeout, size=None, offset=0, tries=3):
    enc = path.encode()
    for _ in range(tries):
        send(op, len(enc) if size is None else size, offset, enc)
        t = time.time()
        while time.time() - t < timeout:
            r = recv(0.5)
            if r and r['req'] == op:
                return r
    return None


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


def listdir(path):
    out, ofs = [], 0
    while True:
        r = ask(OP_LIST, path, 10, offset=ofs)
        if r is None:
            sys.exit(f"no reply listing {path}")
        if r['op'] == NACK:
            return out
        for e in r['data'].split(b'\0'):
            if e:
                out.append(e.decode(errors='replace'))
                ofs += 1
        if not r['data'].strip(b'\0'):
            return out


def get(remote, local):
    r = ask(OP_OPEN_RO, remote, 15)
    if not r or r['op'] != ACK:
        sys.exit(f"open {remote} failed: {r and r['data'][:2]}")
    size = struct.unpack('<I', r['data'][:4])[0]
    print(f"open ok, {size} bytes", flush=True)
    buf = bytearray(size)
    have = set()
    nchunks = (size + CH - 1) // CH
    t0 = last_print = time.time()
    send(OP_BURST, CH, 0)
    last_rx = time.time()
    while len(have) < nchunks:
        r = recv(0.5)
        now = time.time()
        if r is not None:
            last_rx = now
            if r['op'] == ACK and r['req'] == OP_BURST and r['size'] and r['ofs'] % CH == 0:
                buf[r['ofs']:r['ofs'] + r['size']] = r['data']
                have.add(r['ofs'])
            done = r['req'] == OP_BURST and (r['bc'] or r['op'] == NACK)
        else:
            done = now - last_rx > a.stall
        if now - last_print > 10:
            print(f"  {len(have) * CH // 1024} of {size // 1024} KiB, {now - t0:.0f} s", flush=True)
            last_print = now
        if done and len(have) < nchunks:
            gap = next(i * CH for i in range(nchunks) if i * CH not in have)
            send(OP_BURST, CH, gap)
            last_rx = now
    dt = time.time() - t0
    send(OP_TERM)
    drain(3.0, 60)
    print(f"got {size} bytes in {dt:.1f} s, {size / max(dt, 1e-3) / 1024:.1f} KiB/s", flush=True)
    r = ask(OP_CRC, remote, 60)
    board = struct.unpack('<I', r['data'][:4])[0] if r and r['op'] == ACK else None
    local_crc = ap_crc32(buf)
    send(OP_TERM)
    if board != local_crc:
        bad = local + '.crc-mismatch'
        open(bad, 'wb').write(bytes(buf))
        sys.exit(f"CRC MISMATCH: board {board and hex(board)}, local {hex(local_crc)}; kept as {bad}")
    tmp = local + '.part'
    open(tmp, 'wb').write(bytes(buf))
    os.replace(tmp, local)
    print(f"VERIFIED: CRC {hex(local_crc)} matches the board; wrote {local}")


send(OP_RESET)
n, dt = drain()
print(f"sessions reset; drained {n} stale replies in {dt:.1f} s", flush=True)

if a.list:
    for e in listdir(a.list):
        kind, rest = e[0], e[1:]
        print(('dir  ' if kind == 'D' else 'file ') + rest.replace('\t', '  '))
    send(OP_TERM)
    sys.exit(0)

if a.last_log:
    r = ask(OP_OPEN_RO, '/APM/LOGS/LASTLOG.TXT', 15)
    if not r or r['op'] != ACK:
        sys.exit("cannot open /APM/LOGS/LASTLOG.TXT")
    send(OP_BURST, CH, 0)
    txt = b''
    t = time.time()
    while time.time() - t < 15:
        x = recv(0.5)
        if x and x['req'] == OP_BURST and x['op'] == ACK:
            txt = x['data']; break
    send(OP_TERM)
    drain(3.0, 30)
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
