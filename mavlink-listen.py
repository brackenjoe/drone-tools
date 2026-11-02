#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Joe Fernandez
"""Listen-only MAVLink check, standard library only. It never sends MAVLink:
safe against an aircraft that must not be commanded.

Modes:

    mavlink-listen.py tcp HOST:PORT [--seconds S] [--retry S]
        connect (a mavlink-router TCP server) and listen
    mavlink-listen.py udp-bind HOST:PORT [--seconds S]
        bind and listen (a mavlink-router UDP endpoint, e.g. 127.0.0.1:14550)
    mavlink-listen.py udp-probe HOST:PORT [--seconds S] [--expect-none]
        send ONE datagram of plain ASCII (no 0xFD/0xFE byte, so no parser can
        frame it as MAVLink and a router forwards nothing) and listen. A
        mavlink-router v4 UDP server takes any datagram's source as its peer
        (src/endpoint.cpp, UdpEndpoint::_read_msg), so if the probe arrived,
        the flight controller's stream comes back.

Prints the autopilot's HEARTBEAT fields (CRC checked against CRC_EXTRA 50)
and a count per message id. Payloads other than HEARTBEAT are never decoded
or printed: a stream carries positions.

Exit 0 when an autopilot heartbeat was seen, or with --expect-none when
nothing at all came back; 1 otherwise; 2 on a usage or connection error
(a bad HOST:PORT, or a connect, bind or send that fails: one line, no traceback).
"""
import argparse
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
    """Frames MAVLink 1 and 2 out of a byte stream."""

    def __init__(self):
        self.buf = bytearray()
        self.counts = {}
        self.first_hb = None
        self.hb = 0
        self.bad_hb_crc = 0
        self.junk = 0
        self.sources = {}

    def feed(self, data, now):
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
            if v2:
                sysid, compid = f[5], f[6]
                msgid = f[7] | f[8] << 8 | f[9] << 16
            else:
                sysid, compid, msgid = f[3], f[4], f[5]
            self.counts[msgid] = self.counts.get(msgid, 0) + 1
            if msgid == 0:
                self.heartbeat(f, v2, hdr, plen, sysid, compid, now)

    def heartbeat(self, f, v2, hdr, plen, sysid, compid, now):
        crc = x25(f[1:hdr + plen])
        crc = x25(bytes([HB_CRC_EXTRA]), crc)
        if crc != struct.unpack_from("<H", f, hdr + plen)[0]:
            self.bad_hb_crc += 1
            return
        p = f[hdr:hdr + plen] + bytes(9 - plen) if plen < 9 else f[hdr:hdr + 9]
        custom_mode, mtype, ap, base_mode, status, mver = struct.unpack("<IBBBBB", p)
        if ap == 8:          # a GCS or companion, not the flight controller
            return
        self.hb += 1
        key = (sysid, compid, mtype, ap)
        self.sources[key] = self.sources.get(key, 0) + 1
        if self.first_hb is None:
            self.first_hb = now
            print(f"heartbeat: sysid {sysid} compid {compid}, MAVLink{'2' if v2 else '1'}, "
                  f"type {mtype} ({MAV_TYPE.get(mtype, '?')}), autopilot {ap} ({AUTOPILOT.get(ap, '?')}), "
                  f"{'ARMED' if base_mode & 128 else 'disarmed'}, system_status {status} ({STATE.get(status, '?')}), "
                  f"custom_mode {custom_mode}")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0], epilog=MODES,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mode", choices=["tcp", "udp-bind", "udp-probe"])
    ap.add_argument("addr", help="HOST:PORT")
    ap.add_argument("--seconds", type=float, default=5.0, help="how long to listen (default 5)")
    ap.add_argument("--retry", type=float, default=0.0, help="tcp: keep trying to connect for this long")
    ap.add_argument("--expect-none", action="store_true", help="udp-probe: success is silence")
    a = ap.parse_args()
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
    t0 = time.monotonic()
    got = 0

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
        s.close()

    ids = ", ".join(f"{k}x{v}" for k, v in sorted(p.counts.items()))
    print(f"received {got} bytes in {a.seconds:g} s: {sum(p.counts.values())} frames, "
          f"{p.hb} autopilot heartbeats" + (f", first at {p.first_hb:.2f} s" if p.first_hb is not None else "")
          + (f", {p.bad_hb_crc} heartbeat CRC failures" if p.bad_hb_crc else "")
          + (f", {p.junk} unframed bytes" if p.junk else ""))
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
