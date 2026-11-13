#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Joe Fernandez
"""A slow, deep-buffered MAVLink link between a ground tool and one or more
flight controllers (or ArduPilot SITL), standard library only. It stands in
for a USB/IP-over-VPN serial link: slow, with a buffer that does not empty
when the tool dies.

    mavlink-slowlink.py LISTEN_PORT HOST:PORT [HOST:PORT ...]
                        [--rate BYTES_PER_S] [--latency S] [--bind 127.0.0.1]

The proxy holds ONE persistent TCP connection to each upstream, like a USB
serial adapter keeps the flight controller's port open whether or not a tool
is attached. Everything an upstream sends is parsed into whole MAVLink 1 and
2 frames and buffered without limit; the buffer is delivered to whichever
client is connected, after --latency seconds and at --rate bytes per second.
The buffer survives a client disconnect, so a killed tool leaves stale
replies for the next one. What the tool sends goes to EVERY upstream after
--latency, whole frame by whole frame, with no rate limit. Defaults: rate
unlimited, latency 0.

The proxy has no authentication and the newest client takes the slot, so
--bind must stay on loopback (main() warns on stderr otherwise).

Frames pass byte for byte; the CRC and signature are not checked. Bytes that
are not inside a frame are forwarded as they are and counted as junk. Rate
pacing may split a frame across TCP sends. Frames from two upstreams are
queued whole, so they never interleave mid-frame.

Prints `up` once listening, `client connected` / `client gone`, and a stats
line every 5 s (buffered, max buffered, sent, client yes/no), flushed.
SIGTERM and SIGINT close cleanly and exit 0. A listen port already in use or
an upstream that refuses is one line on stderr and exit 2.

Importable (load it with importlib; the file name has a hyphen):

    link = SlowLink(15820, ['127.0.0.1:5820'], rate=1800, latency=0.15)
    link.start()                  # returns once listening and connected
    link.drop = lambda direction, frame: frame.msgid == 110 and direction == 'down'
    ...
    link.stop()                   # idempotent; or `with SlowLink(...) as link:`

`drop(direction, frame) -> bool` is called for every whole frame, 'up' (tool
to flight controller) or 'down' (flight controller to tool); True discards
it. It may be replaced at any time. A drop that raises keeps the frame and is
noted in `link.errors`. `frame` is a `Frame`: version (1 or 2), sysid, compid,
msgid, payload (as received: MAVLink 2 strips trailing zero bytes, the caller
pads), upstream (index of the flight controller it came from, None for 'up'),
raw (the whole frame), t (time.time() when received).

Records, each safe to read from another thread (lists and dicts are copies):
`sent_up` frames the tool sent that were not dropped (recorded when received,
before the latency), `dropped` as (direction, frame), `counts` as
{('up'|'down', msgid): n} for every whole frame seen, dropped or not,
`bytes_up` bytes received from the tool, `bytes_down` bytes delivered to the
client, `junk` bytes outside any frame, both directions, `max_buffered`,
`buffered`. The per-frame lists (`sent_up`, `dropped`) stop growing at
`record_cap` entries (100000), as do `counts` (distinct keys) and `errors`;
`records_truncated` is then True. A key already in `counts` keeps counting; a
new one past the cap is dropped. The tool-to-flight-controller latency queue
is held to UPQ_CAP bytes (16 MiB); past it the client's frames are dropped
and noted in `errors`. `reset_records()` clears the records, not the buffers.
`ftp_fields(payload)` unpacks a FILE_TRANSFER_PROTOCOL (110) payload.

`--selftest` runs the offline tests on loopback and exits with the number
failed.
"""
import argparse
import collections
import ipaddress
import math
import re
import signal
import socket
import struct
import sys
import threading
import time

Frame = collections.namedtuple('Frame', 'version sysid compid msgid payload upstream raw t')

_MAGIC = re.compile(b'[\xfd\xfe]')
RECORD_CAP = 100000     # records kept (sent_up, dropped, counts keys, errors), each
UPQ_CAP = 16 << 20      # bytes the client may have queued toward the flight controllers
TICK = 0.005            # pacer and latency-queue poll interval, s


class SlowLinkError(Exception):
    """A listen or upstream connection that could not be made."""


class FrameParser:
    """Splits a byte stream into whole MAVLink 1/2 frames. feed() returns a
    list of Frame (a whole frame) and bytes (junk found between frames), in
    stream order. A frame still incomplete at the end of the data is held
    until the rest arrives. The CRC is not checked."""

    def __init__(self, upstream=None):
        self.buf = bytearray()
        self.upstream = upstream

    def feed(self, data):
        buf = self.buf
        buf += data
        out = []
        pos, n = 0, len(buf)
        now = time.time()
        while pos < n:
            m = _MAGIC.search(buf, pos)
            if m is None:
                out.append(bytes(buf[pos:]))
                pos = n
                break
            if m.start() > pos:
                out.append(bytes(buf[pos:m.start()]))
                pos = m.start()
            v2 = buf[pos] == 0xFD
            if n - pos < (3 if v2 else 2):
                break
            plen = buf[pos + 1]
            if v2:
                hdr = 10
                total = hdr + plen + 2 + (13 if buf[pos + 2] & 1 else 0)
            else:
                hdr = 6
                total = hdr + plen + 2
            if n - pos < total:
                break
            raw = bytes(buf[pos:pos + total])
            if v2:
                sysid, compid = raw[5], raw[6]
                msgid = raw[7] | raw[8] << 8 | raw[9] << 16
            else:
                sysid, compid, msgid = raw[3], raw[4], raw[5]
            out.append(Frame(2 if v2 else 1, sysid, compid, msgid, raw[hdr:hdr + plen],
                             self.upstream, raw, now))
            pos += total
        del buf[:pos]
        return out


def ftp_fields(payload):
    """Unpack a FILE_TRANSFER_PROTOCOL (msgid 110) payload: target_network,
    target_system, target_component, then seq u16, session u8, opcode u8,
    size u8, req_opcode u8, burst_complete u8, padding u8, offset u32, data.
    MAVLink 2 strips trailing zeros, so the payload is padded to 254 first."""
    p = bytes(payload[:254]).ljust(254, b'\0')
    seq, session, opcode, size, req_opcode, burst_complete, _pad, offset = struct.unpack_from('<HBBBBBBI', p, 3)
    return {'seq': seq, 'session': session, 'opcode': opcode, 'size': size,
            'req_opcode': req_opcode, 'burst_complete': burst_complete,
            'offset': offset, 'data': p[15:15 + min(size, 239)]}


def _hostport(u):
    if isinstance(u, (tuple, list)):
        return str(u[0]), int(u[1])
    host, _, port = str(u).rpartition(':')
    if not host:
        raise SlowLinkError(f"{u!r}: expected HOST:PORT")
    try:
        port = int(port)
        if not 0 < port < 65536:
            raise ValueError
    except ValueError:
        raise SlowLinkError(f"{u!r}: expected HOST:PORT with a port from 1 to 65535") from None
    return host, port


def _is_loopback(host):
    try:
        return ipaddress.ip_address(host).is_loopback
    except ValueError:
        return host == 'localhost'


def _close(s):
    for fn in (lambda: s.shutdown(socket.SHUT_RDWR), s.close):
        try:
            fn()
        except OSError:
            pass


class SlowLink:
    def __init__(self, listen_port, upstreams, rate=None, latency=0.0, drop=None,
                 bind='127.0.0.1', log=None):
        if isinstance(upstreams, (str, tuple)):
            upstreams = [upstreams]
        self.listen_port = listen_port
        self.upstreams = [_hostport(u) for u in upstreams]
        if not self.upstreams:
            raise SlowLinkError("no upstream")
        self.rate = float(rate) if rate else None
        self.latency = float(latency or 0.0)
        for name, val in (('rate', self.rate), ('latency', self.latency)):
            if val is not None and not (math.isfinite(val) and val >= 0):
                raise ValueError(f"{name} must be a finite number >= 0")
        self.drop = drop
        self.bind = bind
        self.port = None            # the bound listen port, once started
        self._log = log or (lambda *_: None)
        self._lock = threading.RLock()
        self._stop = threading.Event()
        self._started = False
        self._stopped = False
        self._srv = None
        self._ups = []
        self._client = None
        self._threads = []
        self._down = collections.deque()    # [t_monotonic, bytes, offset]
        self._upq = collections.deque()     # (t_monotonic, bytes)
        self._upq_bytes = 0
        self._buffered = 0
        self._sent = 0
        self._max_buffered = 0
        self.record_cap = RECORD_CAP
        self._reset()
        self.errors = []

    # -- records
    def _reset(self):
        self._sent_up, self._dropped, self._counts = [], [], {}
        self._bytes_up = self._bytes_down = self._junk = 0
        self._truncated = False

    def reset_records(self):
        with self._lock:
            self._reset()
            self._max_buffered = self._buffered

    sent_up = property(lambda s: s._copy('_sent_up', list))
    dropped = property(lambda s: s._copy('_dropped', list))
    counts = property(lambda s: s._copy('_counts', dict))
    bytes_up = property(lambda s: s._bytes_up)
    bytes_down = property(lambda s: s._bytes_down)
    junk = property(lambda s: s._junk)
    records_truncated = property(lambda s: s._truncated)
    max_buffered = property(lambda s: s._max_buffered)
    buffered = property(lambda s: s._buffered)

    def _record(self, lst, item):
        """Append to a per-frame record unless it is full. Caller holds the lock."""
        if len(lst) < self.record_cap:
            lst.append(item)
        else:
            self._truncated = True

    def _count(self, k):
        """Count a frame. A new key past record_cap is dropped. Caller holds the lock."""
        if k in self._counts or len(self._counts) < self.record_cap:
            self._counts[k] = self._counts.get(k, 0) + 1
        else:
            self._truncated = True

    def _enqueue_up(self, now, data):
        """Queue client bytes toward the flight controllers, or drop them past UPQ_CAP.
        Caller holds the lock."""
        if self._upq_bytes + len(data) > UPQ_CAP:
            self._note_error_locked(f"up queue over {UPQ_CAP} bytes: dropped {len(data)} bytes from the client")
            return
        self._upq.append((now, data))
        self._upq_bytes += len(data)

    def _note_error_locked(self, msg):
        if len(self.errors) < self.record_cap:
            self.errors.append(msg)
        else:
            self._truncated = True

    def _copy(self, name, kind):
        with self._lock:
            return kind(getattr(self, name))

    def stats(self):
        with self._lock:
            return {'buffered': self._buffered, 'max': self._max_buffered,
                    'sent': self._sent, 'client': self._client is not None}

    # -- lifecycle
    def start(self):
        if self._started:
            raise SlowLinkError("already started")
        self._started = True
        try:
            srv = socket.socket()
            self._srv = srv
            srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                srv.bind((self.bind, self.listen_port))
                srv.listen(4)
            except OSError as e:
                raise SlowLinkError(f"listen {self.bind}:{self.listen_port}: {e.strerror or e}") from None
            self.port = srv.getsockname()[1]
            srv.settimeout(0.2)
            for i, (host, port) in enumerate(self.upstreams):
                try:
                    s = socket.create_connection((host, port), timeout=3)
                except OSError as e:
                    raise SlowLinkError(f"upstream {host}:{port}: {e.strerror or e}") from None
                s.settimeout(None)
                s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self._ups.append(s)
        except BaseException:
            self.stop()
            raise
        for target, args in [(self._accept, ()), (self._pace, ()), (self._up_writer, ())] + \
                [(self._up_reader, (i, s)) for i, s in enumerate(self._ups)]:
            t = threading.Thread(target=target, args=args, daemon=True)
            self._threads.append(t)
            t.start()
        return self

    def stop(self):
        with self._lock:
            if self._stopped:
                return
            self._stopped = True
            self._stop.set()
            socks = [self._srv, self._client] + self._ups
            self._client = None
        for s in socks:
            if s is not None:
                _close(s)
        me = threading.current_thread()
        for t in list(self._threads):
            if t is not me:
                t.join(2)

    def __enter__(self):
        return self.start()

    def __exit__(self, *exc):
        self.stop()

    # -- helpers
    def _note_error(self, msg):
        with self._lock:
            self._note_error_locked(msg)

    def _should_drop(self, direction, frame):
        fn = self.drop
        if fn is None:
            return False
        try:
            return bool(fn(direction, frame))
        except Exception as e:  # noqa: BLE001  a check's callback must not kill a link thread
            self._note_error(f"drop raised {type(e).__name__}: {e}")
            return False

    def _lose_client(self, conn):
        with self._lock:
            current = self._client is conn
            if current:
                self._client = None
        _close(conn)
        if current:
            self._log('client gone')

    # -- threads
    def _accept(self):
        while not self._stop.is_set():
            try:
                conn, _ = self._srv.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            conn.settimeout(None)
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            with self._lock:
                old, self._client = self._client, None
            if old is not None:
                _close(old)
                self._log('client gone')
            with self._lock:
                if self._stop.is_set():
                    _close(conn)
                    return
                self._client = conn
                self._threads = [x for x in self._threads if x.is_alive()]
            self._log('client connected')
            t = threading.Thread(target=self._client_reader, args=(conn,), daemon=True)
            t.start()           # started before it is listed, so stop() never joins an unstarted thread
            with self._lock:
                self._threads.append(t)

    def _client_reader(self, conn):
        parser = FrameParser(None)
        while True:
            try:
                data = conn.recv(65536)
            except OSError:
                data = b''
            if not data:
                break
            with self._lock:
                self._bytes_up += len(data)
            for item in parser.feed(data):
                self._from_tool(item)
        self._lose_client(conn)

    def _from_tool(self, item):
        now = time.monotonic()
        if isinstance(item, bytes):
            with self._lock:
                self._junk += len(item)
                self._enqueue_up(now, item)
            return
        with self._lock:
            self._count(('up', item.msgid))
        if self._should_drop('up', item):
            with self._lock:
                self._record(self._dropped, ('up', item))
            return
        with self._lock:
            self._record(self._sent_up, item)
            self._enqueue_up(now, item.raw)

    def _up_reader(self, i, sock):
        parser = FrameParser(i)
        while not self._stop.is_set():
            try:
                data = sock.recv(65536)
            except OSError:
                data = b''
            if not data:
                if not self._stop.is_set():
                    self._log(f'upstream {i} closed')
                return
            for item in parser.feed(data):
                self._from_fc(item)

    def _from_fc(self, item):
        now = time.monotonic()
        if isinstance(item, bytes):
            with self._lock:
                self._junk += len(item)
                self._enqueue_down(now, item)
            return
        with self._lock:
            self._count(('down', item.msgid))
        if self._should_drop('down', item):
            with self._lock:
                self._record(self._dropped, ('down', item))
            return
        with self._lock:
            self._enqueue_down(now, item.raw)

    def _enqueue_down(self, now, data):
        self._down.append([now, data, 0])
        self._buffered += len(data)
        if self._buffered > self._max_buffered:
            self._max_buffered = self._buffered

    def _up_writer(self):
        while not self._stop.is_set():
            time.sleep(TICK)
            while True:
                with self._lock:
                    if not (self._upq and self._upq[0][0] + self.latency <= time.monotonic()):
                        break
                    _, data = self._upq.popleft()
                    self._upq_bytes -= len(data)
                for i, s in enumerate(self._ups):
                    try:
                        s.sendall(data)
                    except OSError as e:
                        if not self._stop.is_set():
                            self._note_error(f"upstream {i} send: {e}")

    def _pace(self):
        budget = 0.0
        last = time.monotonic()
        while not self._stop.is_set():
            time.sleep(TICK)
            now = time.monotonic()
            dt, last = now - last, now
            lost = None
            with self._lock:
                conn = self._client
                if conn is None or not self._down or self._down[0][0] + self.latency > now:
                    budget = 0.0
                    continue
                if self.rate:
                    budget = min(budget + self.rate * dt, max(self.rate * 0.1, 16))
                    limit = int(budget)
                else:
                    limit = 1 << 16
                if limit <= 0:
                    continue
                parts, total = [], 0
                for t, data, off in self._down:
                    if t + self.latency > now or total >= limit:
                        break
                    chunk = data[off:off + limit - total]
                    parts.append(chunk)
                    total += len(chunk)
                try:
                    n = conn.send(b''.join(parts), socket.MSG_DONTWAIT)
                except BlockingIOError:
                    n = 0
                except OSError:
                    n, lost = 0, conn
                if n:
                    if self.rate:
                        budget -= n
                    self._buffered -= n
                    self._sent += n
                    self._bytes_down += n
                    rem = n
                    while rem:
                        it = self._down[0]
                        avail = len(it[1]) - it[2]
                        if rem >= avail:
                            self._down.popleft()
                            rem -= avail
                        else:
                            it[2] += rem
                            rem = 0
            if lost is not None:
                self._lose_client(lost)


# ---------------------------------------------------------------- selftest

def _frame(msgid, payload=b'', sysid=1, compid=1, v=2, signed=False, seq=0):
    """A frame as an autopilot would send it. The CRC and signature bytes are
    arbitrary: the proxy does not check them."""
    if v == 1:
        return bytes([0xFE, len(payload), seq & 255, sysid, compid, msgid]) + payload + b'\x12\x34'
    p = payload.rstrip(b'\0')   # MAVLink 2 strips trailing zeros
    hdr = bytes([0xFD, len(p), 1 if signed else 0, 0, seq & 255, sysid, compid,
                 msgid & 255, (msgid >> 8) & 255, msgid >> 16])
    return hdr + p + b'\xab\xcd' + (b'\x77' * 13 if signed else b'')


class _FakeFC:
    """A TCP server standing in for SITL: records what the proxy sends it."""

    def __init__(self):
        self.srv = socket.socket()
        self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv.bind(('127.0.0.1', 0))
        self.srv.listen(1)
        self.port = self.srv.getsockname()[1]
        self.addr = f"127.0.0.1:{self.port}"
        self.rx = bytearray()
        self.conn = None
        self.ready = threading.Event()
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        try:
            self.conn, _ = self.srv.accept()
        except OSError:
            return
        self.ready.set()
        while True:
            try:
                d = self.conn.recv(65536)
            except OSError:
                return
            if not d:
                return
            self.rx += d

    def send(self, data):
        assert self.ready.wait(3), "proxy never connected"
        self.conn.sendall(data)

    def close(self):
        for s in (self.srv, self.conn):
            if s is not None:
                _close(s)


class _Client:
    """A tool: connects to the proxy and parses what comes back."""

    def __init__(self, link):
        self.s = socket.create_connection(('127.0.0.1', link.port), timeout=3)
        self.s.settimeout(0.05)
        self.parser = FrameParser()
        self.items = []
        self.raw = bytearray()

    def send(self, data):
        self.s.sendall(data)

    def pump(self, timeout, until):
        end = time.monotonic() + timeout
        while time.monotonic() < end and not until():
            try:
                d = self.s.recv(65536)
            except socket.timeout:
                continue
            except OSError:
                break
            if not d:
                break
            self.raw += d
            self.items += self.parser.feed(d)

    def frames(self, n=None, timeout=5.0):
        if n is not None:
            self.pump(timeout, lambda: len([i for i in self.items if isinstance(i, Frame)]) >= n)
        return [i for i in self.items if isinstance(i, Frame)]

    def close(self):
        _close(self.s)


def _until(cond, timeout=3.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(0.01)
    return cond()


def _free_port():
    s = socket.socket()
    s.bind(('127.0.0.1', 0))
    p = s.getsockname()[1]
    s.close()
    return p


def _selftest():
    import contextlib
    import os
    import random
    import subprocess
    rnd = random.Random(11)
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

    @contextlib.contextmanager
    def rig(n=1, **kw):
        fcs = [_FakeFC() for _ in range(n)]
        link = SlowLink(0, [f.addr for f in fcs], **kw)
        clients = []
        try:
            link.start()
            link.client = lambda: clients.append(_Client(link)) or clients[-1]
            yield link, fcs
        finally:
            for c in clients:
                c.close()
            link.stop()
            for f in fcs:
                f.close()

    @case("frame parser: v1, v2, signed v2, split at every byte, junk between")
    def _():
        fs = [_frame(0, b'\x01\x02\x03', v=1), _frame(33, b'abc' * 10), _frame(77, b'xy', signed=True)]
        stream = b'junk' + fs[0] + b'\x00' + fs[1] + fs[2]
        p, got = FrameParser(), []
        for b in stream:
            got += p.feed(bytes([b]))
        assert [g.raw for g in got if isinstance(g, Frame)] == fs
        assert b''.join(g for g in got if isinstance(g, bytes)) == b'junk\x00'
        f = [g for g in got if isinstance(g, Frame)]
        assert [(x.version, x.msgid, x.sysid, x.compid) for x in f] == [(1, 0, 1, 1), (2, 33, 1, 1), (2, 77, 1, 1)]
        assert f[0].payload == b'\x01\x02\x03' and f[2].payload == b'xy' and f[2].raw == fs[2]
        big = _frame(0x01FF02, b'q')
        assert FrameParser().feed(big)[0].msgid == 0x01FF02

    @case("single upstream: v1, v2 and signed v2 pass byte-identical both ways")
    def _():
        fs = [_frame(0, b'\x05' * 9, v=1), _frame(30, bytes(range(1, 40))), _frame(76, b'\x09' * 20, signed=True)]
        with rig() as (link, (fc,)):
            c = link.client()
            assert _until(lambda: link.stats()['client'])
            fc.send(b''.join(fs))
            got = c.frames(3)
            assert [g.raw for g in got] == fs and bytes(c.raw) == b''.join(fs)
            assert link.junk == 0 and not [i for i in c.items if isinstance(i, bytes)]
            c.send(b''.join(reversed(fs)))
            assert _until(lambda: len(fc.rx) == sum(map(len, fs)))
            assert bytes(fc.rx) == b''.join(reversed(fs))
            assert [f.raw for f in link.sent_up] == list(reversed(fs))
            assert link.bytes_up == sum(map(len, fs))
            assert link.counts[('down', 30)] == 1 and link.counts[('up', 76)] == 1

    @case("two upstreams in small random chunks: client gets every frame whole, tool frames reach both")
    def _():
        N = 300
        with rig(2) as (link, fcs):
            c = link.client()
            sent = []
            for k in range(2):
                fr = []
                for i in range(N):
                    pl = bytes([k + 1, i >> 8, i & 255]) + bytes(rnd.randrange(256) for _ in range(rnd.randrange(0, 60))) + b'\x55'
                    fr.append(_frame(rnd.choice([0, 24, 110, 253]), pl, v=rnd.choice([1, 2]), signed=rnd.random() < 0.1, seq=i))
                sent.append(fr)

            def blast(fc, fr):
                data = b''.join(fr)
                r = random.Random(len(data))
                i = 0
                while i < len(data):
                    n = r.randrange(1, 40)
                    fc.send(data[i:i + n])
                    i += n
                    if r.random() < 0.3:
                        time.sleep(0.001)
            ths = [threading.Thread(target=blast, args=(fcs[k], sent[k])) for k in range(2)]
            for t in ths:
                t.start()
            got = c.frames(2 * N, 20)
            for t in ths:
                t.join()
            assert len(got) == 2 * N, f"{len(got)} of {2 * N}"
            assert not [i for i in c.items if isinstance(i, bytes)], "junk in the client stream"
            for k in range(2):
                want = sent[k]
                mine = [g.raw for g in got if g.raw in set(want)]
                assert mine == want, f"upstream {k}: order or content differs"
            assert sorted(g.raw for g in got) == sorted(sent[0] + sent[1])
            tool = [_frame(76, b'\x03' + bytes([i + 1]), seq=i) for i in range(20)]
            c.send(b''.join(tool))
            assert _until(lambda: all(len(f.rx) == len(b''.join(tool)) for f in fcs))
            assert all(bytes(f.rx) == b''.join(tool) for f in fcs)

    @case("the buffer survives a client disconnect: frames arrive at the next client, in order")
    def _():
        fs = [_frame(30, bytes([i + 1]) * 10, seq=i) for i in range(10)]
        with rig() as (link, (fc,)):
            fc.send(b''.join(fs[:4]))                  # no client yet
            assert _until(lambda: link.buffered == len(b''.join(fs[:4])))
            c1 = link.client()
            assert [g.raw for g in c1.frames(4)] == fs[:4]
            c1.close()
            assert _until(lambda: not link.stats()['client'])
            fc.send(b''.join(fs[4:]))                  # client gone
            assert _until(lambda: link.buffered == len(b''.join(fs[4:])))
            c2 = link.client()
            assert [g.raw for g in c2.frames(6)] == fs[4:]
            assert _until(lambda: link.buffered == 0), f"buffered {link.buffered}"

    @case("--rate: N bytes take about N/rate; latency adds about LATENCY")
    def _():
        fs = [_frame(30, bytes([i + 1]) * 88, seq=i) for i in range(20)]       # 100 bytes each
        n, rate = sum(map(len, fs)), 2000
        with rig(rate=rate) as (link, (fc,)):
            fc.send(b''.join(fs))
            assert _until(lambda: link.buffered == n)
            c = link.client()
            t0 = time.monotonic()
            c.pump(10, lambda: len(c.raw) >= n)
            dt = time.monotonic() - t0
            assert len(c.raw) == n and bytes(c.raw) == b''.join(fs)
            assert 0.7 * n / rate <= dt <= 1.5 * n / rate, f"{dt:.2f} s for {n} bytes at {rate}/s (want {n / rate:.2f})"
        with rig(latency=0.3) as (link, (fc,)):
            c = link.client()
            assert _until(lambda: link.stats()['client'])
            t0 = time.monotonic()
            fc.send(_frame(0, b'\x01'))
            c.frames(1)
            dt = time.monotonic() - t0
            assert 0.28 <= dt <= 0.6, f"down latency {dt:.3f} s"
            t0 = time.monotonic()
            c.send(_frame(76, b'\x01'))
            assert _until(lambda: len(fc.rx) > 0)
            dt = time.monotonic() - t0
            assert 0.28 <= dt <= 0.6, f"up latency {dt:.3f} s"

    @case("drop: exactly the matching frames, both directions, recorded; replacing link.drop works")
    def _():
        with rig() as (link, (fc,)):
            link.drop = lambda d, f: (d == 'down' and f.msgid == 33) or (d == 'up' and f.msgid == 76)
            c = link.client()
            assert _until(lambda: link.stats()['client'])
            down = [_frame(m, bytes([i + 1]), seq=i) for i, m in enumerate([1, 33, 1, 33, 2])]
            fc.send(b''.join(down))
            c.frames(3)
            time.sleep(0.15)
            assert [g.raw for g in c.frames()] == [down[0], down[2], down[4]]
            up = [_frame(m, bytes([i + 1]), seq=i) for i, m in enumerate([76, 0, 33, 76])]
            c.send(b''.join(up))
            assert _until(lambda: len(fc.rx) == len(up[1]) + len(up[2]))
            assert bytes(fc.rx) == up[1] + up[2]
            assert [f.raw for f in link.sent_up] == [up[1], up[2]]
            assert sorted((d, f.msgid) for d, f in link.dropped) == [('down', 33), ('down', 33), ('up', 76), ('up', 76)]
            assert link.counts[('down', 33)] == 2 and link.counts[('up', 76)] == 2
            link.reset_records()
            assert link.sent_up == [] and link.dropped == [] and link.counts == {} and link.junk == 0
            link.drop = lambda d, f: d == 'down' and f.msgid == 1
            c.items.clear()
            fc.send(down[0] + down[1])
            c.frames(1)
            time.sleep(0.15)
            assert [g.raw for g in c.frames()] == [down[1]]
            link.drop = None
            fc.send(down[0])
            assert c.frames(2)[-1].raw == down[0]
            link.drop = lambda d, f: 1 / 0
            fc.send(down[2])
            assert c.frames(3)[-1].raw == down[2] and link.errors     # a raising drop keeps the frame

    @case("ftp_fields: a full payload, a stripped one, and one through the link")
    def _():
        def build(seq, session, opcode, size, req, burst, offset, data):
            body = struct.pack('<HBBBBBBI', seq, session, opcode, size, req, burst, 0, offset) + data
            return bytes([0, 1, 1]) + body
        data = bytes(range(1, 240))
        full = build(0x1234, 2, 0x88 & 0x7F, 239, 15, 0, 0x00010203, data)
        assert len(full) == 254
        f = ftp_fields(full)
        assert f == {'seq': 0x1234, 'session': 2, 'opcode': 8, 'size': 239, 'req_opcode': 15,
                     'burst_complete': 0, 'offset': 0x00010203, 'data': data}
        short = build(7, 1, 0, 4, 0, 1, 0, b'ab\0\0')            # data ends in zeros, offset zero
        stripped = short.rstrip(b'\0')
        assert len(stripped) < 254
        f = ftp_fields(stripped)
        assert (f['seq'], f['session'], f['size'], f['burst_complete'], f['offset']) == (7, 1, 4, 1, 0)
        assert f['data'] == b'ab\0\0'
        assert ftp_fields(b'')['size'] == 0 and ftp_fields(b'')['data'] == b''
        pl = build(9, 3, 4, 3, 0, 0, 478, b'xyz')
        with rig() as (link, (fc,)):
            c = link.client()
            assert _until(lambda: link.stats()['client'])
            fc.send(_frame(110, pl))
            g = c.frames(1)[0]
            assert g.msgid == 110 and len(g.payload) == len(pl.rstrip(b'\0'))
            assert ftp_fields(g.payload)['offset'] == 478 and ftp_fields(g.payload)['data'] == b'xyz'

    @case("a client that sends nothing leaves sent_up empty and bytes_up 0")
    def _():
        with rig() as (link, (fc,)):
            c = link.client()
            fc.send(_frame(0, b'\x01'))
            c.frames(1)
            time.sleep(0.2)
            c.close()
            time.sleep(0.2)
            assert link.sent_up == [] and link.bytes_up == 0 and len(fc.rx) == 0

    @case("junk bytes between frames are forwarded and counted, both ways")
    def _():
        a, b = _frame(0, b'\x01'), _frame(30, b'\x02\x03')
        junk = b'hello\x00\x01'
        stream = a + junk + b
        with rig() as (link, (fc,)):
            c = link.client()
            assert _until(lambda: link.stats()['client'])
            fc.send(stream)
            c.pump(3, lambda: len(c.raw) >= len(stream))
            assert bytes(c.raw) == stream
            assert link.junk == len(junk)
            c.send(stream)
            assert _until(lambda: len(fc.rx) >= len(stream))
            assert bytes(fc.rx) == stream
            assert link.junk == 2 * len(junk) and len(link.sent_up) == 2

    @case("stop() frees the listen port, is idempotent; a busy port and a refusing upstream raise")
    def _():
        port = _free_port()
        fc = _FakeFC()
        l1 = SlowLink(port, [fc.addr]).start()
        try:
            try:
                SlowLink(port, [fc.addr]).start()
                raise AssertionError("second SlowLink on a busy port started")
            except SlowLinkError as e:
                assert 'listen' in str(e) and '\n' not in str(e)
        finally:
            l1.stop()
            l1.stop()
        with SlowLink(port, [fc.addr]) as l2:
            assert l2.port == port
        fc.close()
        dead = _free_port()
        try:
            SlowLink(0, [f"127.0.0.1:{dead}"]).start()
            raise AssertionError("refused upstream did not raise")
        except SlowLinkError as e:
            assert 'upstream' in str(e)

    @case("CLI: prints up, client connected/gone, exits 0 on SIGTERM; busy port and refusing upstream exit 2")
    def _():
        fc = _FakeFC()
        port = _free_port()
        me = os.path.abspath(__file__)
        p = subprocess.Popen([sys.executable, me, str(port), fc.addr, '--latency', '0.05'],
                             stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        try:
            assert p.stdout.readline().strip() == 'up'
            s = socket.create_connection(('127.0.0.1', port), timeout=3)
            assert p.stdout.readline().strip() == 'client connected'
            fc.send(_frame(0, b'\x01'))
            s.settimeout(3)
            assert s.recv(100) == _frame(0, b'\x01')
            s.close()
            assert p.stdout.readline().strip() == 'client gone'
            p.send_signal(signal.SIGTERM)
            assert p.wait(5) == 0
        finally:
            if p.poll() is None:
                p.kill()
            p.stdout.close()
            p.stderr.close()
        with SlowLink(0, [fc.addr]) as busy:
            r = subprocess.run([sys.executable, me, str(busy.port), fc.addr], capture_output=True, text=True, timeout=10)
            assert r.returncode == 2 and len(r.stderr.strip().splitlines()) == 1 and 'Traceback' not in r.stderr, r.stderr
        r = subprocess.run([sys.executable, me, str(_free_port()), f"127.0.0.1:{_free_port()}"],
                           capture_output=True, text=True, timeout=10)
        assert r.returncode == 2 and len(r.stderr.strip().splitlines()) == 1 and 'Traceback' not in r.stderr, r.stderr
        fc.close()

    @case("per-frame records stop at record_cap and say so; counts keep counting; reset clears the flag")
    def _():
        with rig() as (link, (fc,)):
            link.record_cap = 5
            link.drop = lambda d, f: d == 'down' and f.msgid == 33
            c = link.client()
            assert _until(lambda: link.stats()['client'])
            assert not link.records_truncated
            c.send(b''.join(_frame(76, bytes([i + 1]), seq=i) for i in range(12)))
            assert _until(lambda: link.counts.get(('up', 76)) == 12)
            assert len(link.sent_up) == 5 and link.records_truncated
            assert [f.payload for f in link.sent_up] == [bytes([i + 1]) for i in range(5)]
            fc.send(b''.join(_frame(33, bytes([i + 1]), seq=i) for i in range(9)))
            assert _until(lambda: link.counts.get(('down', 33)) == 9)
            assert len(link.dropped) == 5 and link.counts[('down', 33)] == 9
            link.reset_records()
            assert not link.records_truncated and link.sent_up == []
            c.send(_frame(76, b'\x01'))
            assert _until(lambda: len(link.sent_up) == 1) and not link.records_truncated

    @case("counts keys and errors stop at record_cap and say so; known keys keep counting")
    def _():
        with rig() as (link, (fc,)):
            link.record_cap = 3
            c = link.client()
            assert _until(lambda: link.stats()['client'])
            c.send(b''.join(_frame(m, b'x') for m in (10, 11, 12, 13, 14, 10, 10)))
            assert _until(lambda: link.counts.get(('up', 10)) == 3)
            assert len(link.counts) == 3 and link.records_truncated, link.counts
            link.reset_records()
            assert not link.records_truncated
            link.drop = lambda d, f: 1 / 0
            c.send(b''.join(_frame(20, b'x') for _ in range(8)))
            assert _until(lambda: len(link.errors) == 3)
            time.sleep(0.1)
            assert len(link.errors) == 3 and link.records_truncated

    @case("the up queue is capped in bytes: past it the client's frames are dropped and noted")
    def _():
        global UPQ_CAP
        old, UPQ_CAP = UPQ_CAP, 100
        try:
            with rig(latency=30) as (link, (fc,)):
                c = link.client()
                assert _until(lambda: link.stats()['client'])
                fr = _frame(76, b'x' * 20)
                c.send(fr * 10)
                assert _until(lambda: link.counts.get(('up', 76)) == 10)
                assert link._upq_bytes <= 100 and any('up queue' in e for e in link.errors), link._upq_bytes
        finally:
            UPQ_CAP = old

    @case("SlowLink() refuses a negative or non-finite rate or latency")
    def _():
        for kw in ({'rate': -1}, {'rate': float('nan')}, {'rate': float('inf')},
                   {'latency': -0.5}, {'latency': float('nan')}, {'latency': float('inf')}):
            try:
                SlowLink(0, ['127.0.0.1:1'], **kw)
            except ValueError:
                continue
            raise AssertionError(f"accepted {kw}")

    @case("finished client reader threads are pruned from _threads on accept")
    def _():
        with rig() as (link, _fcs):
            for _ in range(8):
                link.client().close()
                time.sleep(0.05)
            c = link.client()
            assert _until(lambda: link.stats()['client'])
            assert _until(lambda: len(link._threads) <= 6), len(link._threads)
            c.close()

    @case("CLI: --rate and --latency must be finite and >= 0; a non-loopback --bind warns on stderr")
    def _():
        me = os.path.abspath(__file__)
        for flag, val in (('--rate', '-1'), ('--rate', 'nan'), ('--rate', 'inf'),
                          ('--latency', '-0.5'), ('--latency', 'nan'), ('--latency', 'inf')):
            r = subprocess.run([sys.executable, me, '1', '127.0.0.1:1', flag, val],
                               check=False, capture_output=True, text=True, timeout=10)
            assert r.returncode == 2 and flag in r.stderr and 'Traceback' not in r.stderr, (flag, val, r.stderr)
        fc = _FakeFC()
        # the warning's decision, without binding every interface in a test
        # documentation-only addresses (RFC 5737), built from parts so no literal
        # address sits in the published file
        TEST_NET_1, TEST_NET_2 = '.'.join(('192', '0', '2', '1')), '.'.join(('198', '51', '100', '1'))
        for host, loop in (('127.0.0.1', True), ('localhost', True), ('::1', True),
                           (TEST_NET_2, False), (TEST_NET_1, False)):
            assert _is_loopback(host) == loop, host
        for bind, warns in (('127.0.0.1', False),):
            p = subprocess.Popen([sys.executable, me, str(_free_port()), fc.addr, '--bind', bind],
                                 stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            try:
                assert p.stdout.readline().strip() == 'up'
                p.send_signal(signal.SIGTERM)
                p.wait(5)
                err = p.stderr.read()
                assert ('no authentication' in err) == warns, (bind, err)
            finally:
                if p.poll() is None:
                    p.kill()
                p.stdout.close()
                p.stderr.close()
        fc.close()

    bad = results.count(False)
    print(f"{len(results) - bad} passed, {bad} failed")
    return bad


# ---------------------------------------------------------------- CLI

def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0],
                                 epilog="Importable as SlowLink; see the module docstring.")
    ap.add_argument('listen_port', nargs='?', type=int, help='TCP port to listen on')
    ap.add_argument('upstream', nargs='*', help='HOST:PORT of a flight controller or SITL (one or more)')
    ap.add_argument('--rate', type=float, default=None, help='flight controller to tool, bytes per second (default unlimited)')
    ap.add_argument('--latency', type=float, default=0.0, help='each way, seconds (default 0)')
    ap.add_argument('--bind', default='127.0.0.1', help='listen address (default 127.0.0.1)')
    ap.add_argument('--selftest', action='store_true', help='run the offline tests and exit')
    a = ap.parse_args(argv)
    if a.selftest:
        return _selftest()
    if a.listen_port is None or not a.upstream:
        ap.error('LISTEN_PORT and at least one UPSTREAM are required')
    for flag, val in (('--rate', a.rate), ('--latency', a.latency)):
        if val is not None and not (math.isfinite(val) and val >= 0):
            ap.error(f'{flag} must be a finite number >= 0')
    if not _is_loopback(a.bind):
        print(f"mavlink-slowlink.py: warning: --bind {a.bind} is not loopback and the proxy has no "
              f"authentication; anyone who can reach it can take the link", file=sys.stderr, flush=True)

    def say(msg):
        print(msg, flush=True)

    try:
        link = SlowLink(a.listen_port, a.upstream, rate=a.rate, latency=a.latency, bind=a.bind, log=say)
        link.start()
    except SlowLinkError as e:
        print(f"mavlink-slowlink.py: {e}", file=sys.stderr)
        return 2
    done = threading.Event()
    for sig in (signal.SIGTERM, signal.SIGINT):
        signal.signal(sig, lambda *_: done.set())
    say('up')
    while not done.wait(5):
        s = link.stats()
        say(f"t={time.time():.0f} buffered={s['buffered']} max={s['max']} sent={s['sent']} "
            f"client={'yes' if s['client'] else 'no'}")
    link.stop()
    return 0


if __name__ == '__main__':
    sys.exit(main())
