#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Joe Fernandez
"""Write ArduPilot parameter files to ONE pinned flight controller, and prove it.

Every write here is pinned to a board by three things read off the vehicle
before anything is sent: the board's unique id (AUTOPILOT_VERSION.uid2; on STM32 boards the MCU id,
the same 24 hex digits as the USB serial in /dev/serial/by-id), the firmware's git hash
and the board id. A mismatch on any of them refuses the whole run. A write is
addressed by system id, though, and two FCs at one system id behind a router
both take it (measured in SITL), so every subcommand also listens 3 s for
AUTOPILOT_VERSION and refuses if more than one uid answers. That catches two
boards that both answer then; not a board that answers late or joins afterwards, two boards with the
same uid, or a hostile node (no MAVLink signing here). Give each FC its own
system id (MAV_SYSID from Copter 4.7, SYSID_THISMAV before).

  identify   read-only: version, git hash, board id, uid, armed state
  plan       read-only: every name in the files read back, with what it
             would change to. A name the firmware does not have refuses the
             run: a renamed parameter written by name is silently nothing
  write      plan, then PARAM_SET each changed name and read it back. Needs a
             fresh heartbeat showing the FC disarmed before the first write
             (REFUSED otherwise, nothing written). Before each later write it
             checks the heartbeats seen since the last one; if the FC armed,
             it stops with STOPPED, exit 1, and says how many writes were
             made. After the last write it checks once more: armed, or no
             heartbeat to tell, is a WARNING and exit 1. A write already sent
             cannot be undone. Prints every change (before, after, status)
             and logs every line, the unchanged ones too (those are logged,
             not printed); exits 1 on any readback that does not match
  reboot     MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, refused while armed

Files are Mission Planner "NAME,value" (MAVProxy "NAME value" also parses),
'#' comments. With several files, a later file's value for a name wins over an
earlier one's, and the plan says which file each value came from.

Names someone else owns: if the directory of any given file holds a
never-write.txt, every name or fnmatch pattern listed in it (one per line,
'#' comments) is refused. A file that sets one refuses the whole run before
anything connects, so a parameter file cannot carry a name its owner set by
hand, whatever command line writes it.

It never saves anything but the named parameters, never sends a heartbeat,
and speaks as sysid 250 component 190, so it is never mistaken for the
operator's GCS (sysid 255) by the GCS failsafe.

Usage (needs pymavlink >= 2.4.50; 2.4.49 can crash at connect, a TypeError in
mavutil.add_message, when a link switches from MAVLink 1 to 2; fixed in 2.4.50):
  ap-param-apply.py identify tcp:127.0.0.1:5760
  ap-param-apply.py plan tcp:127.0.0.1:5760 \\
      --expect-uid <24 hex digits>  --expect-git dbe79216 --expect-board 5250 a.param b.param
  ... write ... --log writes.tsv a.param b.param
  ... reboot tcp:127.0.0.1:5760 --expect-uid ... --expect-git ... --expect-board ...
"""
import argparse, fnmatch, os, re, struct, sys, time

mavutil = None  # imported in main() after the arguments, so --help needs no pymavlink


def never_write(paths):
    """[(pattern, list file)] from a never-write.txt beside any of the files."""
    out, seen = [], set()
    for d in sorted({os.path.dirname(os.path.abspath(p)) for p in paths}):
        lock = os.path.join(d, 'never-write.txt')
        if lock in seen or not os.path.exists(lock):
            continue
        seen.add(lock)
        with open(lock) as fh:
            for raw in fh:
                pat = raw.split('#', 1)[0].strip()
                if pat:
                    out.append((pat, lock))
    return out


def f32(x):
    """The value as the wire carries it: every parameter is a float32 there."""
    return struct.unpack('<f', struct.pack('<f', float(x)))[0]


def fmt(v):
    if v is None:
        return '-'
    v = float(v)
    if v == int(v) and abs(v) < 1e9:
        return str(int(v))
    return repr(f32(v))


def read_files(paths):
    """[(name, value, file)], later files winning, order of first appearance."""
    order, vals = [], {}
    for p in paths:
        with open(p) as fh:
            for n, raw in enumerate(fh, 1):
                line = raw.split('#', 1)[0].strip()
                if not line:
                    continue
                parts = line.replace(',', ' ').split()
                if len(parts) != 2:
                    sys.exit(f"{p}:{n}: cannot parse {raw.rstrip()!r}")
                name, sval = parts
                if len(name) > 16 or not name.replace('_', '').isalnum() or name.upper() != name:
                    sys.exit(f"{p}:{n}: not a parameter name: {name!r}")
                try:
                    val = float(sval)
                except ValueError:
                    sys.exit(f"{p}:{n}: not a number: {sval!r}")
                if name not in vals:
                    order.append(name)
                vals[name] = (val, p)
    return [(nm, vals[nm][0], vals[nm][1]) for nm in order]


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


class FC:
    def __init__(self, device, baud):
        self.m = mavutil.mavlink_connection(device, baud=baud, source_system=250,
                                            source_component=190, autoreconnect=False)
        # Through a router the first heartbeat may be a GCS's: take the
        # autopilot's, not whichever arrives first.
        end, hb = time.time() + 30, None
        while time.time() < end:
            msg = self.m.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
            if (msg and msg.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID
                    and msg.type != mavutil.mavlink.MAV_TYPE_GCS):
                hb = msg
                break
        if hb is None:
            sys.exit(f"no autopilot heartbeat on {device}")
        self.hb = hb
        self.sysid, self.compid = hb.get_srcSystem(), hb.get_srcComponent()
        self.m.target_system, self.m.target_component = self.sysid, self.compid
        self.armed_seen = False

    def armed_queued(self):
        """True if the armed bit was seen on the pinned autopilot's heartbeats
        queued since the last call, or noted while waiting in set(). Does not block."""
        seen, self.armed_seen = self.armed_seen, False
        while True:
            msg = self.m.recv_match(type='HEARTBEAT', blocking=False)
            if msg is None:
                return seen
            if msg.get_srcSystem() == self.sysid and msg.get_srcComponent() == self.compid:
                seen = seen or bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

    def armed(self, refuse_if_silent=True):
        """Armed state from a fresh heartbeat. With no heartbeat in 3 s it exits
        REFUSED, or returns None when refuse_if_silent is False (used after writes,
        where REFUSED would read as "nothing was done")."""
        # a heartbeat queued while a slow step ran may be stale: read those first
        queued = self.armed_queued()
        end, hb = time.time() + 3, None
        while time.time() < end and hb is None:
            msg = self.m.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if msg and msg.get_srcSystem() == self.sysid and msg.get_srcComponent() == self.compid:
                hb = msg
        if hb is None:
            if not refuse_if_silent:
                return True if queued else None
            sys.exit("REFUSED: no fresh heartbeat from the FC, so its armed state is unknown")
        return queued or bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

    def versions(self):
        """One dict per distinct board answering as the pinned (sysid, compid)."""
        out = []
        for v in autopilot_versions(self.m, self.sysid, self.compid):
            fv = v.flight_sw_version
            out.append({
                'version': f"{(fv >> 24) & 0xff}.{(fv >> 16) & 0xff}.{(fv >> 8) & 0xff}",
                'git': re.sub(r'[^0-9A-Za-z]', '?',
                              bytes(v.flight_custom_version).decode('ascii', 'ignore').strip('\x00').strip()),
                'board': v.board_version >> 16,
                'board_version': v.board_version,
                'uid': bytes(v.uid2[:12]).hex().upper(),
            })
        if not out:
            sys.exit("no AUTOPILOT_VERSION")
        return out

    def read(self, name, tries=4, timeout=1.5):
        """(value, type) or None if the firmware has no such name."""
        for _ in range(tries):
            self.m.mav.param_request_read_send(self.sysid, self.compid, name.encode(), -1)
            end = time.time() + timeout
            while time.time() < end:
                msg = recv_from(self.m, 'PARAM_VALUE', self.sysid, self.compid, 0.3)
                if msg and msg.param_id == name:
                    return float(msg.param_value), msg.param_type
        return None

    def set(self, name, value, ptype, tries=3, timeout=2.0):
        for _ in range(tries):
            self.m.mav.param_set_send(self.sysid, self.compid, name.encode(), float(value), ptype)
            end = time.time() + timeout
            while time.time() < end:
                msg = recv_from(self.m, ['PARAM_VALUE', 'HEARTBEAT'], self.sysid, self.compid, 0.3)
                if msg is not None and msg.get_type() == 'HEARTBEAT':
                    # the wait would otherwise discard it, and with it an arm
                    if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                        self.armed_seen = True
                elif msg and msg.param_id == name:
                    return float(msg.param_value)
        return None


def check_identity(fc, a):
    vs = fc.versions()
    for v in vs:
        print(f"FC: ArduPilot {v['version']} git {v['git']} board {v['board']} "
              f"uid {v['uid']} sysid {fc.sysid}")
    refuse_shared_sysid([v['uid'] for v in vs], fc.sysid, fc.compid)
    v = vs[0]
    bad = []
    if a.expect_uid and v['uid'] != a.expect_uid.upper():
        bad.append(f"uid {v['uid']} is not the expected {a.expect_uid.upper()}")
    if a.expect_git and not v['git'].lower().startswith(a.expect_git.lower()[:8]):
        bad.append(f"git {v['git']} is not the expected {a.expect_git}")
    if a.expect_board is not None and v['board'] != a.expect_board:
        bad.append(f"board {v['board']} is not the expected {a.expect_board}")
    if bad:
        sys.exit("REFUSED, wrong flight controller: " + '; '.join(bad))
    return v


def plan(fc, rows):
    out, missing = [], []
    for name, target, src in rows:
        got = fc.read(name)
        if got is None:
            missing.append(name)
            out.append((name, None, None, target, src))
            continue
        before, ptype = got
        out.append((name, before, ptype, target, src))
    return out, missing


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('cmd', choices=['identify', 'plan', 'write', 'reboot'])
    ap.add_argument('device', help='e.g. tcp:127.0.0.1:5760 or /dev/serial/by-id/...-if00')
    ap.add_argument('files', nargs='*')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--expect-uid', help='24 hex digits, the USB serial / AUTOPILOT_VERSION.uid2')
    ap.add_argument('--expect-git', help='the firmware git hash: at least 8 hex digits, compared as an '
                    '8-character prefix of the FC\'s git hash')
    ap.add_argument('--expect-board', type=int, help='board id, e.g. 5250 for TBS_LUCID_H7')
    ap.add_argument('--log', help='write: append one TSV line per parameter here')
    # intermixed: the usage above puts the files after the --expect-* options,
    # which plain parse_args() refuses for a nargs='*' positional
    a = ap.parse_intermixed_args()
    if a.expect_git is not None and not re.fullmatch(r'[0-9a-fA-F]{8,}', a.expect_git):
        ap.error(f"--expect-git {a.expect_git!r}: need at least 8 hex digits "
                 f"(it is compared as an 8-character prefix)")
    global mavutil
    try:
        from pymavlink import mavutil
    except ImportError:
        sys.exit("pymavlink not found. Try: pip install 'pymavlink>=2.4.50'")

    if a.cmd in ('write', 'reboot') and not (a.expect_uid and a.expect_git and a.expect_board is not None):
        sys.exit(f"{a.cmd} needs --expect-uid, --expect-git and --expect-board")
    if a.cmd in ('plan', 'write') and not a.files:
        sys.exit(f"{a.cmd} needs at least one parameter file")

    rows = read_files(a.files) if a.files else []
    owned = never_write(a.files)
    locked = [(name, src, pat, lock) for name, _v, src in rows
              for pat, lock in owned if fnmatch.fnmatchcase(name, pat)]
    if locked:
        sys.exit("REFUSED, names someone else owns: " + '; '.join(
            f"{name} ({src}) matches {pat} in {lock}" for name, src, pat, lock in locked))
    fc = FC(a.device, a.baud)
    v = check_identity(fc, a)
    armed = fc.armed()
    print(f"armed: {'YES' if armed else 'no'}")

    if a.cmd == 'identify':
        return 0

    if a.cmd == 'reboot':
        if armed:
            sys.exit("REFUSED: the vehicle is armed")
        fc.m.mav.command_long_send(fc.sysid, fc.compid,
                                   mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0,
                                   1, 0, 0, 0, 0, 0, 0)
        ack = recv_from(fc.m, 'COMMAND_ACK', fc.sysid, fc.compid, 5,
                        condition='COMMAND_ACK.command==246')
        res = None if ack is None else ack.result
        print(f"reboot: ACK result {res} (0 = accepted)")
        return 0 if res == 0 else 1

    planned, missing = plan(fc, rows)
    w = max(len(r[0]) for r in planned)
    print(f"\n{'name':{w}}  {'before':>12}  {'target':>12}  source")
    for name, before, _t, target, src in planned:
        same = before is not None and f32(before) == f32(target)
        tag = 'MISSING' if before is None else ('same' if same else 'CHANGE')
        print(f"{name:{w}}  {fmt(before):>12}  {fmt(target):>12}  {src}  {tag}")
    if missing:
        sys.exit(f"REFUSED: not on this firmware: {', '.join(missing)}")
    changes = [r for r in planned if f32(r[1]) != f32(r[3])]
    print(f"\n{len(changes)} to change, {len(planned) - len(changes)} already at target")
    if a.cmd == 'plan':
        return 0

    # plan can take seconds per name: the state read before it is stale
    if fc.armed():
        sys.exit("REFUSED: the vehicle is armed")
    log = open(a.log, 'a') if a.log else None
    stamp = time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())
    fails, wrote = [], 0
    print(f"\n{'name':{w}}  {'before':>12}  {'after':>12}")
    for name, before, ptype, target, src in planned:
        if f32(before) == f32(target):
            after, status = before, 'unchanged'
        else:
            if fc.armed_queued():
                if log:
                    log.close()
                print(f"wrote {wrote} of {len(changes)}; next was {name}")
                sys.exit(f"STOPPED: the FC armed during the write after {wrote} of "
                         f"{len(changes)} writes; next was {name}. The writes made cannot be undone.")
            after = fc.set(name, target, ptype)
            wrote += 1
            status = 'ok' if after is not None and f32(after) == f32(target) else 'FAILED'
            if status == 'FAILED':
                fails.append(name)
            print(f"{name:{w}}  {fmt(before):>12}  {fmt(after):>12}  {status}")
        if log:
            log.write(f"{stamp}\t{v['uid']}\t{name}\t{fmt(before)}\t{fmt(target)}\t{fmt(after)}\t{status}\t{src}\n")
    if log:
        log.close()
    # Heartbeats come about once a second, so on a fast link an arm in the last
    # second of the loop can go unseen until now. A write cannot be undone; the
    # run must not report success.
    if wrote:
        now = fc.armed(refuse_if_silent=False)
        if now:
            print(f"\nWARNING: the FC is armed now, after {wrote} write(s): the last writes "
                  "may have landed while it was armed. Check the vehicle before flying.")
            return 1
        if now is None:
            print(f"\nWARNING: {wrote} write(s) were made, then no heartbeat arrived, so "
                  "whether the FC armed during them is unknown. Check the vehicle before flying.")
            return 1
    if fails:
        print(f"\nFAILED readback: {', '.join(fails)}")
        return 1
    print(f"\nwrote {len(changes)}, every readback matches")
    return 0


if __name__ == '__main__':
    sys.exit(main())
