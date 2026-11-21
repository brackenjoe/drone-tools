#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Joe Fernandez
"""Listen-only MAVLink check, standard library only. It never sends MAVLink:
safe against an aircraft that must not be commanded.

Modes:

    mavlink-listen.py tcp HOST:PORT [--seconds S] [--retry S] [--sysid N --compid N]
        connect (a mavlink-router TCP server) and listen
    mavlink-listen.py udp-bind HOST:PORT [--seconds S] [--sysid N --compid N]
        bind and listen (a mavlink-router UDP endpoint, e.g. 127.0.0.1:14550)
    mavlink-listen.py udp-probe HOST:PORT [--seconds S] [--expect-none] [--sysid N --compid N]
        send ONE datagram of plain ASCII (no 0xFD/0xFE byte, so no parser can
        frame it as MAVLink and a router forwards nothing) and listen. A
        mavlink-router v4 UDP server takes any datagram's source as its peer
        (src/endpoint.cpp, UdpEndpoint::_read_msg), so if the probe arrived,
        the flight controller's stream comes back.

An autopilot heartbeat here is a HEARTBEAT that passes its CRC check
(CRC_EXTRA 50) and whose autopilot field is not 8 (MAV_AUTOPILOT_INVALID,
a GCS or companion). They are counted by a scan that looks for a valid
HEARTBEAT at every 0xFD or 0xFE byte, not by the frame parser, so injected or
misframed bytes (a frame that declares a long length, a truncated frame)
cannot swallow a real heartbeat behind them; each UDP datagram is parsed on its
own, a TCP stream continuously. Prints the first one's fields, then, once the
--seconds window ends, a summary line and a count per message id. The
summary gives "N autopilot heartbeats, M armed", M being those with
MAV_MODE_FLAG_SAFETY_ARMED (base_mode bit 128) set, 0 when none, so a
script can check every autopilot heartbeat in the window, not just the
first; heartbeats that fail the CRC are counted on the same line.
Payloads other than HEARTBEAT are never decoded or printed: a stream
carries positions.

--sysid N --compid N (both, 1-255, or neither; one alone is a usage error) pin
the count to one source: only an autopilot heartbeat from exactly that sysid and
compid counts toward N, M and the first-heartbeat print, so a stray or forged
heartbeat from another source cannot stand in for the flight controller. The
pin trusts the sysid and compid a heartbeat claims: without MAVLink signing,
which this tool does not check, a sender using the flight controller's own ids
is counted as the flight controller. The
others are counted apart and appended to the summary line, after its other
parts, as ", pinned to S/C" and ", K heartbeats from other sources ignored (A
armed)"; they still appear in the by-source line. Without the pin every
autopilot heartbeat counts, from any source.

Exit 0 when an autopilot heartbeat was seen (with the pin, one from the pinned source), or with --expect-none when
nothing at all came back; 1 otherwise; 2 on a usage or connection error
(a bad HOST:PORT, or a connect, bind or send that fails: one line, no traceback).

    mavlink-listen.py --selftest
        offline check of the parser and the summary line, no sockets; "N passed, M failed", exit 1 on a failure
"""
import argparse
import math
import re
import socket
import struct
import sys
import time

MODES = __doc__.split("\n\n", 1)[1]  # the rest of the docstring: modes, output, exit codes
PROBE = b"listen-only probe, not MAVLink\n"
assert 0xFD not in PROBE and 0xFE not in PROBE
HB_CRC_EXTRA = 50
MAV_TYPE = {1: "fixed wing", 2: "quadrotor", 13: "hexarotor", 14: "octorotor", 6: "GCS", 18: "onboard controller"}
AUTOPILOT = {3: "ArduPilot", 12: "PX4", 8: "invalid (not an autopilot)"}
STATE = {0: "uninit", 1: "boot", 2: "calibrating", 3: "standby", 4: "active", 5: "critical", 6: "emergency", 7: "poweroff", 8: "flight termination"}


def x25(data, crc=0xFFFF):
    for b in data:
        t = b ^ (crc & 0xFF)
        t = (t ^ (t << 4)) & 0xFF
        crc = ((crc >> 8) ^ (t << 8) ^ (t << 3) ^ (t >> 4)) & 0xFFFF
    return crc


class Parser:
    """Frames MAVLink 1 and 2 out of a byte stream (message counts, junk, heartbeat
    CRC failures) and, separately, scans it for valid heartbeats (hb, armed_hb,
    sources): a scan at every 0xFD/0xFE offset, so a misframed frame cannot hide one."""

    SCAN_KEEP = 279  # the largest MAVLink 2 frame: 10 + 255 + 2 + 13 (signed)

    def __init__(self):
        self.buf = bytearray()
        self.sbuf = bytearray()
        self.sbase = 0          # stream offset of sbuf[0]
        self.counted = set()    # stream offsets of heartbeats already counted, still in sbuf
        self.counts = {}
        self.first_hb = None
        self.hb = 0
        self.armed_hb = 0
        self.bad_hb_crc = 0
        self.junk = 0
        self.sources = {}
        self.pin = None         # (sysid, compid), or None to count every source
        self.other_hb = 0       # pinned only: autopilot heartbeats from any other source
        self.other_armed = 0

    def feed(self, data, now):
        self.sbuf += data
        self._scan(now)
        self.buf += data
        while True:
            i = next((k for k, b in enumerate(self.buf) if b in (0xFD, 0xFE)), None)
            if i is None:
                self.junk += len(self.buf)
                self.buf.clear()
                return
            if i:
                self.junk += i
                del self.buf[:i]
            v2 = self.buf[0] == 0xFD
            hdr = 10 if v2 else 6
            if len(self.buf) < hdr:
                return
            plen = self.buf[1]
            sig = 13 if v2 and (self.buf[2] & 1) else 0
            total = hdr + plen + 2 + sig
            if len(self.buf) < total:
                return
            f = bytes(self.buf[:total])
            del self.buf[:total]
            msgid = f[7] | f[8] << 8 | f[9] << 16 if v2 else f[5]
            self.counts[msgid] = self.counts.get(msgid, 0) + 1
            if msgid == 0 and not self.hb_crc_ok(f, hdr, plen):
                self.bad_hb_crc += 1

    def datagram_boundary(self):
        """Call after each UDP datagram: one datagram never continues in the next."""
        self.junk += len(self.buf)
        self.buf.clear()
        self.sbuf.clear()
        self.counted.clear()
        self.sbase = 0

    def _scan(self, now):
        b = self.sbuf
        n = len(b)
        keep = None
        i = 0
        while i < n:
            if b[i] not in (0xFD, 0xFE):
                i += 1
                continue
            v2 = b[i] == 0xFD
            hdr = 10 if v2 else 6
            if n - i < hdr:
                keep = i if keep is None else keep
                i += 1
                continue
            plen = b[i + 1]
            msgid = b[i + 7] | b[i + 8] << 8 | b[i + 9] << 16 if v2 else b[i + 5]
            if msgid != 0 or plen > 9:
                i += 1
                continue
            total = hdr + plen + 2 + (13 if v2 and (b[i + 2] & 1) else 0)
            if n - i < total:
                keep = i if keep is None else keep
                i += 1
                continue
            f = bytes(b[i:i + total])
            if not self.hb_crc_ok(f, hdr, plen):
                i += 1
                continue
            if self.sbase + i not in self.counted:
                self.counted.add(self.sbase + i)
                sysid, compid = (f[5], f[6]) if v2 else (f[3], f[4])
                self.count_heartbeat(f, v2, hdr, plen, sysid, compid, now)
            i += total
        if keep is None:
            keep = n
        keep = max(keep, n - self.SCAN_KEEP)
        del b[:keep]
        self.sbase += keep
        self.counted = {o for o in self.counted if o >= self.sbase}

    @staticmethod
    def hb_crc_ok(f, hdr, plen):
        crc = x25(f[1:hdr + plen])
        crc = x25(bytes([HB_CRC_EXTRA]), crc)
        return crc == struct.unpack_from("<H", f, hdr + plen)[0]

    def count_heartbeat(self, f, v2, hdr, plen, sysid, compid, now):
        """Count one heartbeat whose CRC has passed."""
        p = f[hdr:hdr + plen] + bytes(9 - plen) if plen < 9 else f[hdr:hdr + 9]
        custom_mode, mtype, ap, base_mode, status, mver = struct.unpack("<IBBBBB", p)
        if ap == 8:          # a GCS or companion, not the flight controller
            return
        key = (sysid, compid, mtype, ap)
        self.sources[key] = self.sources.get(key, 0) + 1
        if self.pin is not None and (sysid, compid) != self.pin:
            self.other_hb += 1
            if base_mode & 128:
                self.other_armed += 1
            return
        self.hb += 1
        if base_mode & 128:
            self.armed_hb += 1
        if self.first_hb is None:
            self.first_hb = now
            print(f"heartbeat: sysid {sysid} compid {compid}, MAVLink{'2' if v2 else '1'}, "
                  f"type {mtype} ({MAV_TYPE.get(mtype, '?')}), autopilot {ap} ({AUTOPILOT.get(ap, '?')}), "
                  f"{'ARMED' if base_mode & 128 else 'disarmed'}, system_status {status} ({STATE.get(status, '?')}), "
                  f"custom_mode {custom_mode}")


def seconds_ok(v):
    """--seconds must be finite and above 0 (0, negative and nan never listen, and inf
    never ends, so --expect-none would pass vacuously)."""
    return math.isfinite(v) and v > 0


def summary_line(p, got, seconds, measured=False):
    """The summary line. Scripts may parse "N autopilot heartbeats, M armed" out of
    it, so those words keep their place; the pin's parts come last. measured: seconds
    is the real time listened (a tcp server closed early), shown with one decimal."""
    shown = f"{seconds:.1f}" if measured else f"{seconds:g}"
    return (f"received {got} bytes in {shown} s: {sum(p.counts.values())} frames, "
            f"{p.hb} autopilot heartbeats, {p.armed_hb} armed" + (f", first at {p.first_hb:.2f} s" if p.first_hb is not None else "")
            + (f", {p.bad_hb_crc} heartbeat CRC failures" if p.bad_hb_crc else "")
            + (f", {p.junk} unframed bytes" if p.junk else "")
            + (f", pinned to {p.pin[0]}/{p.pin[1]}" if p.pin is not None else "")
            + (f", {p.other_hb} heartbeats from other sources ignored ({p.other_armed} armed)" if p.other_hb else ""))


# ---------------------------------------------------------------- selftest
def _hb_frame(v2=True, sysid=1, compid=1, base=0x81, ap=3, mtype=2, trim=False, bad_crc=False, seq=0):
    # trim: system_status and mavlink_version 0, so the v2 trailing-zero trim shortens it
    payload = struct.pack("<IBBBBB", 0, mtype, ap, base, 0 if trim else 4, 0 if trim else 3)
    if v2:
        if trim:
            payload = payload.rstrip(b"\0")
        body = bytes([len(payload), 0, 0, seq, sysid, compid, 0, 0, 0]) + payload
    else:
        body = bytes([len(payload), seq, sysid, compid, 0]) + payload
    crc = x25(bytes([HB_CRC_EXTRA]), x25(body))
    if bad_crc:
        crc ^= 0x5A5A
    return bytes([0xFD if v2 else 0xFE]) + body + struct.pack("<H", crc)


def _selftest():
    import contextlib
    import io

    def run(pieces, udp, pin=None):
        p = Parser()
        p.pin = pin
        with contextlib.redirect_stdout(io.StringIO()):
            for d in pieces:
                p.feed(d, 0.0)
                if udp:
                    p.datagram_boundary()
        return p

    fake = bytes([0xFD, 0x13, 0, 0, 0, 1, 1, 0x1E, 0, 0])
    armed = _hb_frame(base=0x81)
    disarmed = _hb_frame(base=0x01)
    attack = [fake, armed, disarmed] * 3
    results = []

    def case(name, ok, why=""):
        results.append(ok)
        print(f"{'PASS' if ok else 'FAIL'}  {name}" + ("" if ok else f"  ({why})"))

    p = run([armed] * 3, True)
    case("a: three armed v2 datagrams", (p.hb, p.armed_hb) == (3, 3), f"hb {p.hb} armed {p.armed_hb}")
    p = run(attack, True)
    case("b: injector datagrams, per-datagram parsing", (p.hb, p.armed_hb) == (6, 3), f"hb {p.hb} armed {p.armed_hb}")
    p = run([b"".join(attack)], False)
    case("c: the same bytes as one TCP stream", (p.hb, p.armed_hb) == (6, 3), f"hb {p.hb} armed {p.armed_hb}")
    p = run([bytes([0xFD, 21, 0, 0, 0, 1, 1, 0x1E, 0, 0]) + bytes(10) + armed * 3], False)
    case("d: truncated frame then three armed, TCP", (p.hb, p.armed_hb) == (3, 3), f"hb {p.hb} armed {p.armed_hb}")
    p = run([_hb_frame(bad_crc=True)], False)
    case("e: bad CRC not counted, bad_hb_crc incremented", (p.hb, p.bad_hb_crc) == (0, 1), f"hb {p.hb} bad {p.bad_hb_crc}")
    p = run([_hb_frame(ap=8)], False)
    case("f: autopilot 8 not counted", p.hb == 0, f"hb {p.hb}")
    p = run([_hb_frame(v2=False)], False)
    case("g: v1 heartbeat counted", (p.hb, p.armed_hb) == (1, 1), f"hb {p.hb} armed {p.armed_hb}")
    p = run([armed[:9], armed[9:]], False)
    case("h: heartbeat split across two feeds, once", (p.hb, p.armed_hb) == (1, 1), f"hb {p.hb} armed {p.armed_hb}")
    t = _hb_frame(trim=True)
    p = run([t], False)
    case("i: v2 heartbeat with trimmed payload (plen %d)" % t[1], t[1] < 9 and (p.hb, p.armed_hb) == (1, 1),
         f"plen {t[1]} hb {p.hb} armed {p.armed_hb}")
    fc_armed = _hb_frame(sysid=1, compid=1, base=0x81)
    fc_dis = _hb_frame(sysid=1, compid=1, base=0x01)
    oth_dis = _hb_frame(sysid=1, compid=191, base=0x01, ap=3)
    oth_armed = _hb_frame(sysid=2, compid=1, base=0x81)
    p = run([fc_armed, oth_dis], False, (1, 1))
    case("j: pinned, FC armed plus other disarmed", (p.hb, p.armed_hb, p.other_hb, p.other_armed) == (1, 1, 1, 0),
         f"hb {p.hb} armed {p.armed_hb} other {p.other_hb}/{p.other_armed}")
    p = run([oth_dis], False, (1, 1))
    case("k: pinned, FC silent, only 1/191", (p.hb, p.other_hb) == (0, 1) and p.first_hb is None, f"hb {p.hb} other {p.other_hb}")
    p = run([fc_dis, oth_armed], False, (1, 1))
    case("l: pinned, FC disarmed, other armed", (p.hb, p.armed_hb, p.other_hb, p.other_armed) == (1, 0, 1, 1),
         f"hb {p.hb} armed {p.armed_hb} other {p.other_hb}/{p.other_armed}")
    p = run([oth_dis, oth_armed, fc_dis], False)
    case("m: unpinned, every source counts", (p.hb, p.armed_hb, p.other_hb) == (3, 1, 0), f"hb {p.hb} armed {p.armed_hb} other {p.other_hb}")
    # an example consumer's sed, as re: 's/.* \([0-9][0-9]*\) autopilot heartbeats, \([0-9][0-9]*\) armed.*/\1 \2/p'
    gate = re.compile(r".* ([0-9][0-9]*) autopilot heartbeats, ([0-9][0-9]*) armed.*")
    p = run([fc_dis, oth_armed, oth_dis, _hb_frame(bad_crc=True)], False, (1, 1))
    line = summary_line(p, 99, 3)
    m = gate.match(line)
    case("n: a sed of the summary line reads the pinned N and M",
         bool(m) and m.groups() == ("1", "0") and line.endswith(", pinned to 1/1, 2 heartbeats from other sources ignored (1 armed)"), line)
    for v in ("0", "-1", "nan"):
        with contextlib.redirect_stderr(io.StringIO()):
            rc = main(["tcp", "127.0.0.1:1", "--seconds", v])
        case(f"o: --seconds {v} is a usage error, exit 2", rc == 2, f"rc {rc}")
    case("p: --seconds 5 and 0.5 accepted, inf refused",
         seconds_ok(5.0) and seconds_ok(0.5) and not seconds_ok(float("inf")))
    p = Parser()
    line = summary_line(p, 0, 1.2345, measured=True)
    case("q: early close shows measured time, one decimal", line.startswith("received 0 bytes in 1.2 s: "), line)
    line = summary_line(p, 0, 5.0)
    case("r: a full window shows the nominal --seconds", line.startswith("received 0 bytes in 5 s: "), line)
    m = gate.match(summary_line(p, 0, 0.04, measured=True))
    case("s: sed reads N and M with a measured time", bool(m) and m.groups() == ("0", "0"))
    bad = results.count(False)
    print(f"{len(results) - bad} passed, {bad} failed")
    return 1 if bad else 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0], epilog=MODES,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mode", nargs="?", choices=["tcp", "udp-bind", "udp-probe"])
    ap.add_argument("addr", nargs="?", help="HOST:PORT")
    ap.add_argument("--selftest", action="store_true", help="run the offline parser tests and exit")
    ap.add_argument("--seconds", type=float, default=5.0, help="how long to listen (default 5)")
    ap.add_argument("--retry", type=float, default=0.0, help="tcp: keep trying to connect for this long")
    ap.add_argument("--sysid", type=int, help="only heartbeats from this sysid count toward N and M (1-255); needs --compid")
    ap.add_argument("--compid", type=int, help="only heartbeats from this compid count toward N and M (1-255); needs --sysid")
    ap.add_argument("--expect-none", action="store_true", help="udp-probe: success is silence")
    a = ap.parse_args(argv)
    if a.selftest:
        return _selftest()
    if a.mode is None or a.addr is None:
        ap.error("the following arguments are required: mode, addr")
    if not seconds_ok(a.seconds):
        print("--seconds must be a finite number above 0", file=sys.stderr)
        return 2
    if (a.sysid is None) != (a.compid is None):
        print("--sysid and --compid go together: give both or neither", file=sys.stderr)
        return 2
    if a.sysid is not None and not (1 <= a.sysid <= 255 and 1 <= a.compid <= 255):
        print("--sysid and --compid must each be from 1 to 255", file=sys.stderr)
        return 2
    host, _, port = a.addr.rpartition(":")
    try:
        if not host:
            raise ValueError
        port = int(port)
        if not 0 <= port <= 65535:
            raise ValueError
    except ValueError:
        print(f"{a.addr!r}: expected HOST:PORT with a port from 0 to 65535", file=sys.stderr)
        return 2
    p = Parser()
    if a.sysid is not None:
        p.pin = (a.sysid, a.compid)
    t0 = time.monotonic()
    got = 0
    listened, measured = a.seconds, False

    if a.mode == "tcp":
        while True:
            try:
                s = socket.create_connection((host, port), timeout=3)
                break
            except OSError as e:
                if time.monotonic() - t0 >= a.retry:
                    print(f"tcp {a.addr}: {e}", file=sys.stderr)
                    return 2
                time.sleep(0.2)
        print(f"connected to {a.addr} after {time.monotonic() - t0:.1f} s; listening {a.seconds:g} s")
        t1 = time.monotonic()
        s.settimeout(0.5)
        while time.monotonic() - t1 < a.seconds:
            try:
                d = s.recv(65536)
            except socket.timeout:
                continue
            if not d:
                print("connection closed by the server")
                elapsed = time.monotonic() - t1
                if elapsed < a.seconds:
                    listened, measured = elapsed, True
                break
            got += len(d)
            p.feed(d, time.monotonic() - t1)
        s.close()
    else:
        fam = socket.AF_INET6 if ":" in host else socket.AF_INET
        s = socket.socket(fam, socket.SOCK_DGRAM)
        try:
            if a.mode == "udp-bind":
                s.bind((host, port))
            else:
                s.sendto(PROBE, (host, port))
        except OSError as e:
            print(f"{a.mode} {a.addr}: {e}", file=sys.stderr)
            return 2
        if a.mode == "udp-bind":
            print(f"bound {a.addr}; listening {a.seconds:g} s")
        else:
            print(f"sent {len(PROBE)} bytes of ASCII to {a.addr} from port {s.getsockname()[1]}; listening {a.seconds:g} s")
        t1 = time.monotonic()
        s.settimeout(0.5)
        while time.monotonic() - t1 < a.seconds:
            try:
                d, src = s.recvfrom(65536)
            except socket.timeout:
                continue
            except ConnectionRefusedError:
                continue
            got += len(d)
            p.feed(d, time.monotonic() - t1)
            p.datagram_boundary()
        s.close()

    ids = ", ".join(f"{k}x{v}" for k, v in sorted(p.counts.items()))
    print(summary_line(p, got, listened, measured))
    if len(p.sources) > 1:
        print("heartbeats by source: " + ", ".join(
            f"{s}/{c} type {t} autopilot {a} x{n}" for (s, c, t, a), n in sorted(p.sources.items())))
    if ids:
        print(f"message ids (id x count; CRC checked on HEARTBEAT only): {ids}")
    if a.expect_none:
        return 0 if got == 0 else 1
    return 0 if p.hb else 1


if __name__ == "__main__":
    sys.exit(main())
