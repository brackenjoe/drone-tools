#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Joe Fernandez
"""ArduPilot's onboard compass calibration, driven over MAVLink, pinned to ONE FC:
`start` begins it and prints the report, `accept` saves the result, `cancel`
discards it. It speaks to one flight controller only, chosen by its board id,
firmware and unique id, and refuses to start or accept while the vehicle is armed.

  start    MAV_CMD_DO_START_MAG_CAL with mask 0, no retry, NOT autosaved, then
           print progress until a MAG_CAL_REPORT arrives or --timeout passes
           (then it cancels). Mask 0 means every compass IN USE: the FC skips,
           without saying so, one that is not healthy or has COMPASS_USEn=0
           (AP_Compass_Calibration.cpp:50-55), and refuses the whole start if
           COMPASS_LEARN is 2 (in-flight learning makes no compass usable for
           yaw, :53, AP_Compass.cpp:2030). The mask the FC actually started is
           printed from the first MAG_CAL_PROGRESS. The report is printed in
           full: fitness, offsets, diagonals, off-diagonals, scale, and the
           orientation check (COMPASS_AUTO_ROT) with its confidence
  accept   MAV_CMD_DO_ACCEPT_MAG_CAL: saves the last successful result,
           including an orientation the check corrected. The FC's ACK cannot
           be trusted alone: it says accepted when nothing was saved (no
           calibration, or none finished: AP_Compass_Calibration.cpp:187-240)
           and failed when some compasses WERE saved (one succeeded, one
           failed). So COMPASS_OFS_X/Y/Z, OFS2 and OFS3 are read before and
           after and each compass is reported as changed or unchanged; exit 0
           only when at least one changed
  cancel   MAV_CMD_DO_CANCEL_MAG_CAL

What `start` does NOT leave alone, though nothing is saved until `accept`:
  - COMPASS_LEARN is set to 0 and saved (AP_Compass_Calibration.cpp:108), and
    `cancel` does not put it back. The tool reads COMPASS_LEARN before `start`
    and prints the value that is about to be lost; restore it by hand if it
    was not 0.
  - The first `start` after a boot sets the FC's "calibrated, requires reboot"
    flag (:99). From then on arming is refused ("Compass calibrated requires
    reboot", AP_Arming.cpp:615-617) until the FC reboots, even after `cancel`.
Not autosaving is still the point of the split: a report is read before
anything is saved to COMPASS_OFS_*, and a poor one is discarded with `cancel`.
The FC itself fails a fit worse than COMPASS_CAL_FIT, but only compass 0 when
it is external gets that threshold; an internal compass, or any compass but the
first, gets TWICE COMPASS_CAL_FIT (:88-96).

Progress and the report are not sent unless asked for: MAG_CAL_PROGRESS and
MAG_CAL_REPORT ride the EXTRA3 stream (GCS_MAVLink_Parameters.cpp:365-366,
GCS_Common.cpp:1142-1143 and :6480-6485), whose Copter default rate is 0, so on
a direct link with no GCS a successful calibration would run to --timeout. So
before `start` this asks, with MAV_CMD_SET_MESSAGE_INTERVAL on this link, for
MAG_CAL_PROGRESS every 0.5 s and MAG_CAL_REPORT every 1 s, and checks both ACKs.
If either is refused it resets the pair and falls back to REQUEST_DATA_STREAM
for EXTRA3 at 4 Hz, which has no ACK: it counts as accepted only if EXTRA3
traffic (VIBRATION, EKF_STATUS_REPORT or AHRS) then arrives. With neither it
refuses (REFUSED, nothing started, nothing changed). When `start` ends, by
report, timeout or error, the intervals are set back to the default (interval
0, the stream's own rate). The stream-rate fallback is NOT undone: on Copter it
is not saved and lasts until the FC reboots.

This writes to a flight controller: `start` begins a calibration and `accept`
saves one. Both refuse while the vehicle is armed, judged from a fresh
HEARTBEAT read just before the command is sent (REFUSED, nothing sent; no
heartbeat in 3 s is also a refusal), and a heartbeat showing it armed while
the command waits for its ACK, resends included, stops the run (STOPPED; the
command may already have been applied). `cancel` is allowed while armed.

Identity is checked the same way as ap-param-apply.py: --expect-uid (the USB
serial), --expect-git and --expect-board must all match or nothing is sent,
and so must a second board answering at the same system id. That catches two
boards that both answer within 3 s; not a board that answers later or joins
afterwards, two boards with the same uid, or a hostile node (no MAVLink
signing here). The system and component id are pinned from the first
autopilot heartbeat; messages from any other source are ignored. Speaks as
sysid 250 component 190 and sends no heartbeat.

Usage (needs Python 3 and pymavlink >= 2.4.50):
  python3 ap-mag-cal.py start tcp:127.0.0.1:5760 \\
      --expect-uid <24 hex digits> --expect-git dbe79216 --expect-board 5250 \\
      [--timeout 300] [--log cal.txt]
  python3 ap-mag-cal.py --selftest        no flight controller needed
"""
import argparse, re, sys, time

mavutil = None  # imported in main() after the arguments, so --help needs no pymavlink

START, ACCEPT, CANCEL = 42424, 42425, 42426
SET_MESSAGE_INTERVAL = 511
MAG_CAL_PROGRESS_ID, MAG_CAL_REPORT_ID = 191, 192
PROGRESS_US, REPORT_US = 500_000, 1_000_000
EXTRA3 = 12                      # MAV_DATA_STREAM_EXTRA3
EXTRA3_PROOF = ['VIBRATION', 'EKF_STATUS_REPORT', 'AHRS']   # EXTRA3 messages (GCS_MAVLink_Parameters.cpp:339-)
# COMPASS_OFS*, one set per compass: (compass index, parameter prefix)
OFS_SETS = [(0, 'COMPASS_OFS'), (1, 'COMPASS_OFS2'), (2, 'COMPASS_OFS3')]
STATUS = {0: 'NOT_STARTED', 1: 'WAITING_TO_START', 2: 'RUNNING_STEP_ONE',
          3: 'RUNNING_STEP_TWO', 4: 'SUCCESS', 5: 'FAILED', 6: 'BAD_ORIENTATION',
          7: 'BAD_RADIUS'}


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


def is_armed(hb):
    """True if a HEARTBEAT has the MAV_MODE_FLAG_SAFETY_ARMED bit set."""
    return bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)


def armed_now(m, sysid, compid, window=3.0):
    """Armed state of the pinned autopilot from a FRESH heartbeat. Heartbeats
    already queued (a slow step may have left stale ones) are read first and
    any armed one counts; then one new heartbeat is awaited for `window` s.
    Exits REFUSED when none arrives, because the armed state is then unknown."""
    queued = False
    while True:
        msg = m.recv_match(type='HEARTBEAT', blocking=False)
        if msg is None:
            break
        if msg.get_srcSystem() == sysid and msg.get_srcComponent() == compid:
            queued = queued or is_armed(msg)
    hb = recv_from(m, 'HEARTBEAT', sysid, compid, window)
    if hb is None:
        sys.exit("REFUSED: no fresh heartbeat from the FC, so its armed state is unknown")
    return queued or is_armed(hb)


def send_command(m, sysid, compid, cmd, params=(), guard_armed=False, tries=3, wait=3.0):
    """COMMAND_LONG to the pinned autopilot; returns the ACK result, None if
    none came in `tries` x `wait` s. HEARTBEATs are read during the wait, not
    discarded: with guard_armed, one showing the vehicle armed ends the run
    (STOPPED), because a re-send is a new command that must not reach an armed
    FC, and the one already sent may have been applied."""
    p = list(params) + [0] * (5 - len(params))
    for sent in range(1, tries + 1):
        m.mav.command_long_send(sysid, compid, cmd, 0, *p, 0, 0)
        end = time.time() + wait
        while True:
            left = end - time.time()
            if left <= 0:
                break
            msg = recv_from(m, ['COMMAND_ACK', 'HEARTBEAT'], sysid, compid, left)
            if msg is None:
                break
            if msg.get_type() == 'HEARTBEAT':
                if guard_armed and is_armed(msg):
                    sys.exit(f"STOPPED: the FC armed while command {cmd} waited for its ACK "
                             f"(sent {sent} time(s)); it may have been applied")
            elif msg.command == cmd:
                return msg.result
    return None


def read_param(m, sysid, compid, name, tries=3, timeout=1.0):
    """The float value of a parameter, or None if the FC does not answer."""
    for _ in range(tries):
        m.mav.param_request_read_send(sysid, compid, name.encode(), -1)
        end = time.time() + timeout
        while time.time() < end:
            msg = recv_from(m, 'PARAM_VALUE', sysid, compid, 0.3)
            if msg and msg.param_id == name:
                return float(msg.param_value)
    return None


def read_offsets(m, sysid, compid):
    """{compass index: (x, y, z) or None} from COMPASS_OFS*_X/Y/Z."""
    out = {}
    for i, pre in OFS_SETS:
        v = [read_param(m, sysid, compid, f"{pre}_{ax}") for ax in 'XYZ']
        out[i] = None if None in v else tuple(v)
    return out


def judge_accept(before, after, ack):
    """(exit code, lines) for an `accept`, from the offsets read before and
    after it and the ACK result. The ACK alone is wrong both ways, so what
    counts is whether a compass's offsets changed. Exit 0 only if one did."""
    lines, changed = [], []
    for i in sorted(after):
        b, a = before.get(i), after[i]
        if b is None or a is None:
            lines.append(f"  compass {i}: offsets not readable, cannot say")
        elif b != a:
            changed.append(i)
            lines.append(f"  compass {i}: SAVED, offsets {b[0]:.1f} {b[1]:.1f} {b[2]:.1f} -> "
                         f"{a[0]:.1f} {a[1]:.1f} {a[2]:.1f}")
        else:
            lines.append(f"  compass {i}: unchanged ({a[0]:.1f} {a[1]:.1f} {a[2]:.1f})")
    if not changed:
        lines.append(f"NOTHING SAVED: no compass's offsets changed (ACK result {ack}; the FC also "
                     f"answers 0 when there was no finished calibration to save)")
        return 1, lines
    saved = ', '.join(str(i) for i in changed)
    if ack != 0:
        lines.append(f"PARTIAL: compass {saved} saved, but the ACK result was {ack}: at least one "
                     f"compass did not accept. The compasses listed unchanged were not saved")
    else:
        lines.append(f"saved: compass {saved}")
    return 0, lines


def enable_mag_cal_messages(cmd, request_stream, stream_seen):
    """Make the FC send MAG_CAL_PROGRESS and MAG_CAL_REPORT on this link; the
    mode that worked, 'interval' or 'stream'. `cmd(id, *params)` returns the
    ACK result; `request_stream()` sends the EXTRA3 stream request;
    `stream_seen()` says whether EXTRA3 traffic then arrived. Exits REFUSED
    when neither way is accepted."""
    got = [cmd(SET_MESSAGE_INTERVAL, MAG_CAL_PROGRESS_ID, PROGRESS_US),
           cmd(SET_MESSAGE_INTERVAL, MAG_CAL_REPORT_ID, REPORT_US)]
    if got == [0, 0]:
        return 'interval'
    for mid in (MAG_CAL_PROGRESS_ID, MAG_CAL_REPORT_ID):   # undo a half-done pair
        cmd(SET_MESSAGE_INTERVAL, mid, 0)
    request_stream()
    if stream_seen():
        return 'stream'
    sys.exit(f"REFUSED: the FC would not send MAG_CAL_PROGRESS/MAG_CAL_REPORT on this link "
             f"(SET_MESSAGE_INTERVAL ACKs {got[0]} and {got[1]}, 0 = accepted; no EXTRA3 traffic "
             f"after a stream request). Nothing was started. Without them a finished "
             f"calibration would never be seen.")


def restore_mag_cal_messages(mode, cmd):
    """Put the two intervals back to the default (0 = the stream's own rate).
    The stream-rate fallback is left as it is. Returns the ACK results."""
    if mode != 'interval':
        return []
    return [cmd(SET_MESSAGE_INTERVAL, MAG_CAL_PROGRESS_ID, 0),
            cmd(SET_MESSAGE_INTERVAL, MAG_CAL_REPORT_ID, 0)]


def format_report(msg):
    """The lines printed for one MAG_CAL_REPORT, in full."""
    return [
        f"REPORT compass {msg.compass_id}: {STATUS.get(msg.cal_status, msg.cal_status)} "
        f"autosaved {msg.autosaved} fitness {msg.fitness:.2f}",
        f"  ofs      {msg.ofs_x:.1f} {msg.ofs_y:.1f} {msg.ofs_z:.1f}",
        f"  diag     {msg.diag_x:.3f} {msg.diag_y:.3f} {msg.diag_z:.3f}",
        f"  offdiag  {msg.offdiag_x:.3f} {msg.offdiag_y:.3f} {msg.offdiag_z:.3f}",
        f"  scale    {msg.scale_factor:.3f}",
        f"  orient   {msg.old_orientation} -> {msg.new_orientation} "
        f"confidence {msg.orientation_confidence:.2f}",
    ]


def all_reports_in(cal_mask, reports):
    """True when every compass set in the report mask has reported."""
    return all((cal_mask >> i) & 1 == 0 or i in reports for i in range(8))


def format_progress(cal_mask, progress, reports):
    """One line with every compass: its latest progress, or its report once done.

    The FC sends MAG_CAL_PROGRESS only for compasses still running, one message
    each per call, starting after the last one sent (ArduPilot 4.7.1
    AP_Compass_Calibration.cpp:244-281), and none for a finished compass, so a
    finished compass is shown by its report. `progress` and `reports` map
    compass_id to the latest message; compasses are the mask bits plus any
    that sent either."""
    ids = sorted({i for i in range(8) if (cal_mask >> i) & 1} | set(progress) | set(reports))
    if not ids:
        return "progress: no compass"
    parts = []
    for i in ids:
        if i in reports:
            s = reports[i].cal_status
            parts.append(f"compass {i}: reported {STATUS.get(s, s)}")
        elif i in progress:
            p = progress[i]
            parts.append(f"compass {i}: {STATUS.get(p.cal_status, p.cal_status)} "
                         f"attempt {p.attempt} {p.completion_pct}%")
        else:
            parts.append(f"compass {i}: no progress yet")
    return "progress: " + " | ".join(parts)


def selftest():
    """Offline checks of the pure parts; no flight controller, no link."""
    from types import SimpleNamespace as NS
    n_ok = n_bad = 0

    def check(name, cond):
        nonlocal n_ok, n_bad
        print(f"{'PASS' if cond else 'FAIL'}  {name}")
        if cond:
            n_ok += 1
        else:
            n_bad += 1

    armed_bit = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
    rep = NS(compass_id=0, cal_status=4, autosaved=0, fitness=0.0312, ofs_x=100.04,
             ofs_y=-200.06, ofs_z=300.0, diag_x=1.0, diag_y=0.9996, diag_z=1.0004,
             offdiag_x=0.0, offdiag_y=-0.0004, offdiag_z=0.012, scale_factor=1.0,
             old_orientation=0, new_orientation=4, orientation_confidence=10.123,
             cal_mask=0b101)
    lines = format_report(rep)
    check("report: 6 lines", len(lines) == 6)
    check("report: header", lines[0] == "REPORT compass 0: SUCCESS autosaved 0 fitness 0.03")
    check("report: offsets", lines[1] == "  ofs      100.0 -200.1 300.0")
    check("report: diagonals", lines[2] == "  diag     1.000 1.000 1.000")
    check("report: off-diagonals", lines[3] == "  offdiag  0.000 -0.000 0.012")
    check("report: scale", lines[4] == "  scale    1.000")
    check("report: orientation", lines[5] == "  orient   0 -> 4 confidence 10.12")
    rep.cal_status = 99
    check("report: unknown status printed as its number", 'compass 0: 99 ' in format_report(rep)[0])
    check("report mask: compass 2 still missing", not all_reports_in(0b101, {0: rep}))
    check("report mask: both in", all_reports_in(0b101, {0: rep, 2: rep}))
    check("report mask: empty mask", all_reports_in(0, {}))

    def prog(status, attempt, pct):
        return NS(cal_status=status, attempt=attempt, completion_pct=pct)

    three = {0: prog(2, 1, 34), 1: prog(2, 1, 50), 2: prog(3, 2, 7)}
    check("progress: all three in order",
          format_progress(0b111, three, {}) == "progress: compass 0: RUNNING_STEP_ONE attempt 1 34% | "
          "compass 1: RUNNING_STEP_ONE attempt 1 50% | compass 2: RUNNING_STEP_TWO attempt 2 7%")
    check("progress: silent compasses say so",
          format_progress(0b111, {0: prog(2, 1, 34)}, {}) == "progress: compass 0: RUNNING_STEP_ONE "
          "attempt 1 34% | compass 1: no progress yet | compass 2: no progress yet")
    check("progress: a report wins over stale progress",
          format_progress(0b11, {0: prog(2, 1, 34), 1: prog(2, 1, 90)}, {1: NS(cal_status=4)})
          == "progress: compass 0: RUNNING_STEP_ONE attempt 1 34% | compass 1: reported SUCCESS")
    check("progress: unknown status printed as its number",
          format_progress(0b1, {0: prog(99, 1, 5)}, {}) == "progress: compass 0: 99 attempt 1 5%")
    check("progress: compass outside the mask still listed",
          format_progress(0b1, {0: prog(2, 1, 5), 2: prog(2, 1, 6)}, {})
          == "progress: compass 0: RUNNING_STEP_ONE attempt 1 5% | "
          "compass 2: RUNNING_STEP_ONE attempt 1 6%")
    check("progress: no compass", format_progress(0, {}, {}) == "progress: no compass")

    def hb(armed, src=(1, 1)):
        h = NS(base_mode=(armed_bit if armed else 0) | 1)
        h.get_srcSystem, h.get_srcComponent = (lambda: src[0]), (lambda: src[1])
        return h

    check("heartbeat: armed bit set", is_armed(hb(True)))
    check("heartbeat: armed bit clear", not is_armed(hb(False)))

    class Fake:
        """recv_match as the pymavlink link gives it: heartbeats already queued
        come back to a non-blocking call, a fresh one to a blocking call."""
        def __init__(self, queued=(), fresh=()):
            self.queued, self.fresh = list(queued), list(fresh)

        def recv_match(self, type=None, blocking=False, timeout=None, condition=None):
            src = self.fresh if blocking else self.queued
            if src:
                return src.pop(0)
            if blocking:
                time.sleep(min(timeout or 0, 0.05))
            return None

    def verdict(fake):
        try:
            return armed_now(fake, 1, 1, 0.2)
        except SystemExit as e:
            return str(e)

    check("armed_now: fresh disarmed heartbeat", verdict(Fake(fresh=[hb(False)])) is False)
    check("armed_now: fresh armed heartbeat", verdict(Fake(fresh=[hb(True)])) is True)
    check("armed_now: a stale armed one queued, fresh disarmed, still armed",
          verdict(Fake([hb(True)], [hb(False)])) is True)
    check("armed_now: armed heartbeat from another component is ignored",
          verdict(Fake([hb(True, (1, 2))], [hb(False)])) is False)
    check("armed_now: silent FC refuses, armed state unknown",
          verdict(Fake()) == "REFUSED: no fresh heartbeat from the FC, so its armed state is unknown")
    check("armed_now: only a foreign fresh heartbeat refuses",
          str(verdict(Fake(fresh=[hb(True, (9, 9))]))).startswith("REFUSED: no fresh heartbeat"))

    # --- the message request: ACKs accepted or refused
    def enable(acks, seen):
        sent, streamed = [], []

        def cmd(cid, *p):
            sent.append((cid,) + p)
            return acks.pop(0) if acks else None
        try:
            mode = enable_mag_cal_messages(cmd, lambda: streamed.append(1), lambda: seen)
        except SystemExit as e:
            mode = str(e)
        return mode, sent, streamed

    mode, sent, streamed = enable([0, 0], False)
    check("messages: both intervals ACKed -> interval mode, no stream request",
          mode == 'interval' and not streamed)
    check("messages: progress then report, in microseconds",
          sent == [(511, 191, 500_000), (511, 192, 1_000_000)])
    mode, sent, streamed = enable([0, 4], True)
    check("messages: report refused -> pair reset, stream fallback used",
          mode == 'stream' and streamed == [1] and sent[2:] == [(511, 191, 0), (511, 192, 0)])
    mode, sent, streamed = enable([None, None], True)
    check("messages: no ACK counts as refused, stream seen -> stream mode", mode == 'stream')
    mode, sent, streamed = enable([4, 4], False)
    check("messages: interval refused and no EXTRA3 traffic -> REFUSED, says why",
          mode.startswith("REFUSED: the FC would not send MAG_CAL_PROGRESS")
          and "ACKs 4 and 4" in mode and "Nothing was started" in mode)
    sent = []
    restore_mag_cal_messages('interval', lambda cid, *p: sent.append((cid,) + p) or 0)
    check("messages: restore sets both intervals to 0", sent == [(511, 191, 0), (511, 192, 0)])
    sent = []
    restore_mag_cal_messages('stream', lambda cid, *p: sent.append((cid,) + p) or 0)
    check("messages: stream fallback is not undone", sent == [])

    # --- accept: per-compass readback decides, not the ACK
    o1, o2 = (1.0, 2.0, 3.0), (4.0, 5.0, 6.0)
    rc, ls = judge_accept({0: o1, 1: o1, 2: o1}, {0: o1, 1: o1, 2: o1}, 0)
    check("accept: ACK 0 but nothing changed -> exit 1, NOTHING SAVED",
          rc == 1 and ls[-1].startswith("NOTHING SAVED"))
    rc, ls = judge_accept({0: o1, 1: o1}, {0: o2, 1: o2}, 0)
    check("accept: both changed, ACK 0 -> exit 0, both named",
          rc == 0 and ls[-1] == "saved: compass 0, 1")
    rc, ls = judge_accept({0: o1, 1: o1}, {0: o2, 1: o1}, 5)
    check("accept: one saved, ACK FAILED -> exit 0 with PARTIAL",
          rc == 0 and ls[-1].startswith("PARTIAL: compass 0 saved")
          and "compass 1: unchanged" in "\n".join(ls))
    rc, ls = judge_accept({0: o1, 1: None}, {0: o1, 1: o2}, 0)
    check("accept: an unreadable compass is not counted as saved", rc == 1 and "not readable" in ls[1])

    # --- the ACK wait: heartbeats are read, an armed one stops the run
    class Link:
        """Scripted wait: each send releases the next batch of messages."""
        def __init__(self, batches):
            self.batches, self.pending, self.sent = list(batches), [], 0
            link = self
            self.mav = NS(command_long_send=lambda *a: link.release())

        def release(self):
            self.sent += 1
            self.pending = self.batches.pop(0) if self.batches else []

        def recv_match(self, type=None, blocking=False, timeout=None, condition=None):
            while self.pending:
                x = self.pending.pop(0)
                if x.get_type() in type:
                    return x
            time.sleep(min(timeout or 0, 0.02))
            return None

    def ack(cmd, result=0, src=(1, 1)):
        x = NS(command=cmd, result=result, get_type=lambda: 'COMMAND_ACK')
        x.get_srcSystem, x.get_srcComponent = (lambda: src[0]), (lambda: src[1])
        return x

    def beat(armed, src=(1, 1)):
        h = hb(armed, src)
        h.get_type = lambda: 'HEARTBEAT'
        return h

    def sc(link, guard, tries=3):
        try:
            return send_command(link, 1, 1, 42425, guard_armed=guard, tries=tries, wait=0.15)
        except SystemExit as e:
            return str(e)

    check("send_command: disarmed heartbeat then the ACK -> result",
          sc(Link([[beat(False), ack(42425, 0)]]), True) == 0)
    check("send_command: armed heartbeat before the ACK -> STOPPED",
          str(sc(Link([[beat(True), ack(42425, 0)]]), True)).startswith("STOPPED: the FC armed"))
    r = sc(Link([[], [beat(True)]]), True)
    check("send_command: armed heartbeat during the RE-SEND's wait -> STOPPED, says sent 2",
          "sent 2 time(s)" in str(r))
    check("send_command: guard off (cancel) ignores an armed heartbeat",
          sc(Link([[beat(True), ack(42425, 0)]]), False) == 0)
    check("send_command: an ACK for another command or from another source is skipped",
          sc(Link([[ack(42424, 0), ack(42425, 0, (9, 9))]]), True, tries=1) is None)
    check("send_command: no ACK in 3 tries -> None, 3 sends",
          (lambda l: (sc(l, True), l.sent))(Link([[], [], []])) == (None, 3))
    print(f"{n_ok} passed, {n_bad} failed")
    return 0 if n_bad == 0 else 1


def main():
    global mavutil
    parts = __doc__.split('\n\n', 1)
    ap = argparse.ArgumentParser(
        description=parts[0], formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=parts[1] + "\nExit codes: 0 done (start: every compass SUCCESS; accept: at least one "
               "compass's offsets changed), 1 refused, the FC did not accept, or accept saved nothing, "
               "2 a compass reported a failure, 3 start timed out and was cancelled.")
    ap.add_argument('cmd', nargs='?', choices=['start', 'accept', 'cancel'],
                    help='start: begin a calibration (not autosaved) and print the report; '
                         'accept: save the last successful result; cancel: discard it')
    ap.add_argument('device', nargs='?',
                    help='MAVLink connection, e.g. tcp:127.0.0.1:5760 or /dev/serial/by-id/...-if00')
    ap.add_argument('--baud', type=int, default=115200, help='serial baud rate (default 115200)')
    ap.add_argument('--expect-uid',
                    help='24 hex digits, the AUTOPILOT_VERSION.uid2 / USB serial of the one '
                         'board allowed (required to run)')
    ap.add_argument('--expect-git',
                    help='firmware git hash, at least 8 hex digits, compared as an 8-character '
                         'prefix (required to run)')
    ap.add_argument('--expect-board', type=int,
                    help='board id, e.g. 5250 (required to run)')
    ap.add_argument('--timeout', type=float, default=300.0,
                    help='start: seconds to wait for the reports before cancelling (default 300)')
    ap.add_argument('--log', help='append everything printed to this file, the REFUSED/STOPPED '
                                  'messages and the exit code too (not argument errors)')
    ap.add_argument('--selftest', action='store_true',
                    help='run the offline checks of the report formatting, the progress line, '
                         'the armed check, the '
                         'message request and the accept verdict, then exit; needs pymavlink, '
                         'no flight controller')
    a = ap.parse_args()
    try:
        from pymavlink import mavutil
    except ImportError:
        sys.exit("pymavlink not found. Try: pip install 'pymavlink>=2.4.50'")
    if a.selftest:
        return selftest()
    if not (a.cmd and a.device and a.expect_uid and a.expect_git and a.expect_board is not None):
        ap.error("needs CMD, DEVICE, --expect-uid, --expect-git and --expect-board")
    if not re.fullmatch(r'[0-9a-fA-F]{8,}', a.expect_git):
        ap.error(f"--expect-git {a.expect_git!r}: need at least 8 hex digits "
                 f"(it is compared as an 8-character prefix)")

    logf = open(a.log, 'a') if a.log else None

    def stamp(s):
        return f"{time.strftime('%H:%M:%S', time.gmtime())}Z {s}"

    def say(s):
        print(stamp(s), flush=True)
        if logf:
            logf.write(stamp(s) + '\n')
            logf.flush()

    try:
        rc = run(a, say)
    except SystemExit as e:
        if logf and isinstance(e.code, str):   # Python prints it to stderr; the log gets it too
            logf.write(stamp(e.code) + '\n')
            logf.flush()
        raise
    if logf:
        logf.write(stamp(f"exit {rc}") + '\n')
        logf.flush()
    return rc


def run(a, say):
    m = mavutil.mavlink_connection(a.device, baud=a.baud, source_system=250,
                                   source_component=190, autoreconnect=False)
    # Through a router the first heartbeat may be a GCS's: pin the autopilot's.
    end, hb = time.time() + 30, None
    while time.time() < end:
        msg = m.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
        if (msg and msg.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID
                and msg.type != mavutil.mavlink.MAV_TYPE_GCS):
            hb = msg
            break
    if hb is None:
        sys.exit("no autopilot heartbeat")
    sysid, compid = hb.get_srcSystem(), hb.get_srcComponent()
    m.target_system, m.target_component = sysid, compid

    vers = autopilot_versions(m, sysid, compid)
    if not vers:
        sys.exit("no AUTOPILOT_VERSION")
    for v in vers:
        say(f"FC: git {bytes(v.flight_custom_version).decode('ascii', 'ignore').strip(chr(0)).strip()} "
            f"board {v.board_version >> 16} uid {bytes(v.uid2[:12]).hex().upper()} sysid {sysid}")
    refuse_shared_sysid([bytes(v.uid2[:12]).hex().upper() for v in vers], sysid, compid)
    ver = vers[0]
    uid = bytes(ver.uid2[:12]).hex().upper()
    git = bytes(ver.flight_custom_version).decode('ascii', 'ignore').strip('\x00').strip()
    board = ver.board_version >> 16
    bad = []
    if uid != a.expect_uid.upper():
        bad.append(f"uid {uid}")
    if not git.lower().startswith(a.expect_git.lower()[:8]):
        bad.append(f"git {git}")
    if board != a.expect_board:
        bad.append(f"board {board}")
    if bad:
        sys.exit("REFUSED, wrong flight controller: " + ', '.join(bad))

    def command(cmd, *params, guard=True):
        return send_command(m, sysid, compid, cmd, params, guard_armed=guard)

    if a.cmd == 'cancel':
        res = command(CANCEL, guard=False)
        say(f"cancel: ACK result {res} (0 = accepted)")
        return 0 if res == 0 else 1

    # start and accept write to the FC: refuse if it is armed, judged from a
    # heartbeat read now, after the slow identity steps, not the first one.
    if armed_now(m, sysid, compid):
        sys.exit("REFUSED: the vehicle is armed")

    if a.cmd == 'accept':
        before = read_offsets(m, sysid, compid)
        res = command(ACCEPT)
        say(f"accept: ACK result {res} (0 = accepted; it also says 0 when nothing was saved)")
        rc, lines = judge_accept(before, read_offsets(m, sysid, compid), res)
        for line in lines:
            say(line)
        return rc

    def num(v):
        return 'unreadable' if v is None else f'{v:g}'

    learn = read_param(m, sysid, compid, 'COMPASS_LEARN')
    say(f"COMPASS_LEARN is {num(learn)} now. `start` sets it to 0 and SAVES that, and `cancel` "
        f"does not put it back; the first start since boot also makes arming refuse until the "
        f"FC reboots, `cancel` or not")

    def request_stream():
        m.mav.request_data_stream_send(sysid, compid, EXTRA3, 4, 1)

    def stream_seen():
        return recv_from(m, EXTRA3_PROOF, sysid, compid, 4) is not None

    mode = enable_mag_cal_messages(command, request_stream, stream_seen)
    say("progress and report requested "
        + ("with SET_MESSAGE_INTERVAL (reset when this ends)" if mode == 'interval' else
           "with the EXTRA3 stream at 4 Hz (stays until the FC reboots)"))
    try:
        res = command(START, 0, 0, 0, 0, 0)
        say(f"start: ACK result {res} (0 = accepted); turn the vehicle through every orientation")
        if res != 0:
            return 1
        # no read-back here: waiting for a PARAM_VALUE would discard the FC's STATUSTEXTs
        say(f"COMPASS_LEARN is now 0 and saved (was {num(learn)}); arming is refused until the "
            f"FC reboots")
        return watch(m, sysid, compid, a.timeout, say, command)
    finally:
        res = restore_mag_cal_messages(mode, command)
        if res:
            say(f"message intervals reset: ACK results {res} (0 = accepted)")


def watch(m, sysid, compid, timeout, say, command):
    """Print progress and reports until every started compass has reported or
    `timeout` s pass (then cancel). Exit code of the run."""
    t0, last_print, reports, mask_said = time.time(), 0.0, {}, False
    progress, mask = {}, 0
    while time.time() - t0 < timeout:
        msg = recv_from(m, ['MAG_CAL_PROGRESS', 'MAG_CAL_REPORT', 'STATUSTEXT'],
                        sysid, compid, 1)
        if msg is None:
            continue
        t = msg.get_type()
        if t != 'STATUSTEXT':
            mask = msg.cal_mask
        if t != 'STATUSTEXT' and not mask_said:
            mask_said = True
            on = [i for i in range(8) if (msg.cal_mask >> i) & 1]
            say(f"started mask {msg.cal_mask:#05b}: compass {', '.join(map(str, on)) or 'none'} "
                f"(mask 0 skips unhealthy and COMPASS_USEn=0 compasses)")
        if t == 'STATUSTEXT':
            say(f"FC: {msg.text}")
        elif t == 'MAG_CAL_PROGRESS':
            progress[msg.compass_id] = msg
            if time.time() - last_print >= 2:
                last_print = time.time()
                say(format_progress(mask, progress, reports))
        elif t == 'MAG_CAL_REPORT' and msg.compass_id not in reports:
            reports[msg.compass_id] = msg
            for line in format_report(msg):
                say(line)
            if all_reports_in(msg.cal_mask, reports):
                say("all reports in. Nothing saved to COMPASS_OFS_*: `accept` saves, `cancel` discards")
                return 0 if all(r.cal_status == 4 for r in reports.values()) else 2
    say(f"timeout after {timeout:.0f} s: cancelling")
    command(CANCEL, guard=False)
    return 3


if __name__ == '__main__':
    sys.exit(main())
