#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Joe Fernandez
"""Diff the self-describing message formats of two ArduPilot dataflash logs.

A .bin log carries its own schema: FMT names every later record's type byte,
length, name, format string and column labels; FMTU carries per-field unit and
multiplier *ids*; UNIT and MULT map those ids to a unit label and a numeric
multiplier. So "what changed in the log between two firmware versions" is
answerable from two logs, with no firmware source and no pymavlink.

  log-fmt-diff.py OLD.BIN NEW.BIN [--json out.json] [--markdown out.md]
                  [--types MSG,CTUN,...] [--all-types] [--label-old 4.6.3]
  log-fmt-diff.py --selftest

By default only DEFAULT_TYPES below, the types a flight review usually
reads, are compared; --all-types compares every
type either log defines. A type present in one log and not the other is
reported as such rather than silently dropped.

Record layouts (identical at Copter 4.6.3 and 4.7.1, checked in source
2026-09-20: libraries/AP_Logger/LogStructure.h:181-210 at 4.6.3 and :187-216
at 4.7.1):
  FMT   0x80, 89 bytes: type:B length:B name:4s format:16s labels:64s
  FMTU  "QBNN"  TimeUS, FmtType, UnitIds, MultIds
  UNIT  "QbZ"   TimeUS, Id, Label
  MULT  "Qbd"   TimeUS, Id, Mult

Stdlib only. Each log is read into memory and parsed in one pass; both are held
at once, so two 300 MB logs need about 600 MB.
"""
from __future__ import annotations

import argparse
import json
import math
import struct
import sys
import unicodedata
from pathlib import Path

HEAD1, HEAD2 = 0xA3, 0x95
FMT_TYPE, FMT_LEN = 0x80, 89
# body size of the records parsed with struct.unpack
_BODY_SIZE = {"FMTU": struct.calcsize("<QB16s16s"),
              "UNIT": struct.calcsize("<Qb64s"),
              "MULT": struct.calcsize("<Qbd")}

# Copter and common types first, then ArduPlane.
DEFAULT_TYPES = {
    "MSG", "PARM", "MODE", "BAT", "CTUN", "RATE", "ATT", "VIBE", "FTN1", "GPS",
    "RSSI", "FTN", "FTNS", "ESC", "RAD", "RCIN", "RCOU", "PSCN", "PSCE",
    "PSCD", "POS", "MOTB", "ISBH", "ISBD", "XKF4", "GYR", "PIDR", "PIDP",
    "PIDY", "ATUN", "ATDE",
    # ArduPlane
    "STAT", "QTUN", "TECS", "NTUN", "ARSP", "PIQP", "PIQR", "PIQY", "ATRP",
}

# Field width in bytes per dataflash format character, and whether pymavlink
# divides the raw integer.
# Source: pymavlink DFReader.FORMAT_TO_STRUCT.
FMT_CHAR = {
    "b": (1, None), "B": (1, None), "h": (2, None), "H": (2, None),
    "i": (4, None), "I": (4, None), "f": (4, None), "d": (8, None),
    "n": (4, None), "N": (16, None), "Z": (64, None),
    "c": (2, 100.0), "C": (2, 100.0), "e": (4, 100.0), "E": (4, 100.0),
    "L": (4, 1e7), "M": (1, None), "q": (8, None), "Q": (8, None),
    "a": (64, None), "g": (2, None),
}


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


def inert(s) -> str:
    """Replace every C0 control, DEL and C1 control character by \\xNN.

    Log strings are written by whoever made the log; printed raw, an ESC
    sequence is interpreted by the reader's terminal. Applied where a string is
    printed or written, never at parse time.
    """
    return "".join(map(_inert_char, str(s)))


_MD_SPECIAL = set("\\`|<>[]!*_#")


def md(s) -> str:
    """inert(), then backslash-escape the Markdown specials."""
    return "".join("\\" + c if c in _MD_SPECIAL else c for c in inert(s))


def code(s) -> str:
    """A string as a Markdown code span. No escape works inside a span, so a
    string with a backtick is written as escaped plain text instead."""
    if "`" in str(s):
        return md(s)
    return "`" + inert(s).replace("|", "\\|") + "`"


def _num(x):
    """A multiplier for output: a non-finite float becomes "nan"/"inf"/"-inf"."""
    if isinstance(x, float) and not math.isfinite(x):
        return "nan" if math.isnan(x) else ("inf" if x > 0 else "-inf")
    return x


def parse_log(path: str) -> dict:
    """One pass over a log, returning its FMT/FMTU/UNIT/MULT tables."""
    fmts: dict[int, dict] = {}          # type byte -> {name, length, format, labels}
    fmtu: dict[int, tuple[str, str]] = {}   # type byte -> (unit ids, mult ids)
    units: dict[str, str] = {}          # unit id char -> label
    mults: dict[str, float] = {}        # mult id char -> multiplier
    counts: dict[str, int] = {}
    lengths = [-1] * 256
    lengths[FMT_TYPE] = FMT_LEN
    firmware = None

    data = memoryview(Path(path).read_bytes())
    i, n, bad = 0, len(data), 0
    while i + 3 <= n:
        if data[i] != HEAD1 or data[i + 1] != HEAD2:
            i += 1
            bad += 1
            continue
        t = data[i + 2]
        ln = lengths[t]
        if ln < 3 or i + ln > n:
            # unknown type, a length that cannot hold the header, or a record
            # running off the end: step one byte, so i always increases
            i += 1
            bad += 1
            continue
        body = data[i + 3:i + ln]
        if t == FMT_TYPE:
            if len(body) < 86:
                bad += 1
                i += ln
                continue
            typ, length = body[0], body[1]
            if length < 3 or (typ == FMT_TYPE and length != FMT_LEN):
                bad += 1
                i += ln
                continue
            name = bytes(body[2:6]).split(b"\0")[0].decode("ascii", "replace")
            form = bytes(body[6:22]).split(b"\0")[0].decode("ascii", "replace")
            labels = bytes(body[22:86]).split(b"\0")[0].decode("ascii", "replace")
            lengths[typ] = length
            fmts[typ] = {"name": name, "length": length,
                         "format": form, "labels": labels}
        else:
            nm = fmts.get(t, {}).get("name")
            if nm in _BODY_SIZE and len(body) != _BODY_SIZE[nm]:
                bad += 1
                i += ln
                continue
            if nm == "FMTU":
                _, ftype, uids, mids = struct.unpack("<QB16s16s", bytes(body))
                fmtu[ftype] = (uids.split(b"\0")[0].decode("ascii", "replace"),
                               mids.split(b"\0")[0].decode("ascii", "replace"))
            elif nm == "UNIT":
                _, uid, label = struct.unpack("<Qb64s", bytes(body))
                units[chr(uid & 0xFF)] = label.split(b"\0")[0].decode("ascii", "replace")
            elif nm == "MULT":
                _, mid, mult = struct.unpack("<Qbd", bytes(body))
                mults[chr(mid & 0xFF)] = _num(mult)
            elif nm == "MSG" and firmware is None:
                txt = bytes(body).split(b"\0")[0]
                # MSG gained ID and Seq fields ahead of the text at 4.7.1, so
                # find the text by its label position rather than assuming.
                off = _text_offset(fmts[t]["format"])
                txt = bytes(body[off:]).split(b"\0")[0]
                s = txt.decode("utf-8", "replace")
                if s.startswith("ArduCopter") or s.startswith("ArduPlane"):
                    firmware = s
        nm = fmts.get(t, {}).get("name", f"?{t}")
        counts[nm] = counts.get(nm, 0) + 1
        i += ln

    return {"path": path, "fmts": fmts, "fmtu": fmtu, "units": units,
            "mults": mults, "counts": counts, "bad_bytes": bad,
            "firmware": firmware, "size": n}


def _text_offset(form: str) -> int:
    """Byte offset of the first Z (string) field inside a record body."""
    off = 0
    for ch in form:
        if ch == "Z":
            return off
        off += FMT_CHAR.get(ch, (0, None))[0]
    return 0


def fields(log: dict, type_name: str) -> list[dict] | None:
    """Per-field name, type char, unit id, unit label, mult id, mult value."""
    for tb, f in log["fmts"].items():
        if f["name"] != type_name:
            continue
        labels = f["labels"].split(",") if f["labels"] else []
        form = f["format"]
        uids, mids = log["fmtu"].get(tb, ("", ""))
        out = []
        for k, lab in enumerate(labels):
            ch = form[k] if k < len(form) else "?"
            uid = uids[k] if k < len(uids) else ""
            mid = mids[k] if k < len(mids) else ""
            out.append({
                "field": lab,
                "type": ch,
                "width": FMT_CHAR.get(ch, (None, None))[0],
                "implicit_div": FMT_CHAR.get(ch, (None, None))[1],
                "unit_id": uid,
                "unit": log["units"].get(uid, "") if uid else "",
                "mult_id": mid,
                "mult": log["mults"].get(mid) if mid else None,
            })
        return out
    return None


def compare(old: dict, new: dict, type_names: list[str]) -> list[dict]:
    rows = []
    for name in type_names:
        fo, fn = fields(old, name), fields(new, name)
        entry = {"type": name,
                 "in_old": fo is not None, "in_new": fn is not None,
                 "count_old": old["counts"].get(name, 0),
                 "count_new": new["counts"].get(name, 0),
                 "fields": [], "verdict": "same", "notes": []}
        if fo is None and fn is None:
            entry["verdict"] = "absent from both logs"
            rows.append(entry)
            continue
        if fo is None:
            entry["verdict"] = "added"
            entry["notes"].append(f"no FMT in {old['path']}")
        if fn is None:
            entry["verdict"] = "removed"
            entry["notes"].append(f"no FMT in {new['path']}")

        by_old = {f["field"]: (k, f) for k, f in enumerate(fo or [])}
        by_new = {f["field"]: (k, f) for k, f in enumerate(fn or [])}
        changed = False
        for fname in list(by_old) + [f for f in by_new if f not in by_old]:
            ko, o = by_old.get(fname, (None, None))
            kn, nw = by_new.get(fname, (None, None))
            diffs = []
            if o is None:
                diffs.append("added")
            elif nw is None:
                diffs.append("removed")
            else:
                if o["type"] != nw["type"]:
                    diffs.append(f"type {o['type']}->{nw['type']}")
                if o["mult"] != nw["mult"]:
                    diffs.append(f"mult {o['mult']}->{nw['mult']}")
                if o["unit"] != nw["unit"]:
                    diffs.append(f"unit {o['unit']!r}->{nw['unit']!r}")
                if ko != kn:
                    diffs.append(f"position {ko}->{kn}")
            if diffs:
                changed = True
            entry["fields"].append({
                "field": fname,
                "old": o, "new": nw,
                "old_index": ko, "new_index": kn,
                "diff": diffs,
            })
        if changed and entry["verdict"] == "same":
            entry["verdict"] = "changed"
        rows.append(entry)
    return rows


def markdown(rows, old, new, label_old, label_new) -> str:
    lo, ln_ = md(label_old), md(label_new)
    out = [f"# FMT/FMTU diff: {lo} vs {ln_}", "",
           f"- {code(old['path'])} -- {md(old['firmware'] or 'firmware string not found')}, "
           f"{old['size']/1e6:.1f} MB",
           f"- {code(new['path'])} -- {md(new['firmware'] or 'firmware string not found')}, "
           f"{new['size']/1e6:.1f} MB", "",
           "Read out of the logs themselves by `log-fmt-diff.py`; the",
           "multiplier and unit columns come from each log's own MULT and UNIT",
           "records, not from a table in this repo.", "",
           "## Verdict per message type", "",
           f"| type | {lo} rows | {ln_} rows | verdict |",
           "|---|---:|---:|---|"]
    for r in rows:
        notes = (' -- ' + '; '.join(md(x) for x in r['notes'])) if r['notes'] else ''
        out.append(f"| {code(r['type'])} | {r['count_old']} | {r['count_new']} | "
                   f"{md(r['verdict'])}{notes} |")
    out += ["", "## Fields, per type", ""]
    for r in rows:
        out.append(f"### {md(r['type'])} ({md(r['verdict'])})")
        out.append("")
        if not r["fields"]:
            out += ["Not defined in either log.", ""]
            continue
        out.append(f"| field | {lo} type | {lo} mult | {lo} unit "
                   f"| {ln_} type | {ln_} mult | {ln_} unit | change |")
        out.append("|---|---|---|---|---|---|---|---|")
        for f in r["fields"]:
            o, nw = f["old"], f["new"]
            def cell(x, k):
                return "--" if x is None else (
                    "" if x[k] in (None, "") else md(x[k]))
            out.append(
                f"| {code(f['field'])} | {cell(o,'type')} | {cell(o,'mult')} | {cell(o,'unit')} "
                f"| {cell(nw,'type')} | {cell(nw,'mult')} | {cell(nw,'unit')} "
                f"| {md(', '.join(f['diff']))} |")
        out.append("")
    return "\n".join(out)


# ---- selftest ---------------------------------------------------------------

def _rec(t: int, body: bytes) -> bytes:
    return bytes([HEAD1, HEAD2, t]) + body


def _fmt(typ: int, length: int, name: str, form: str, labels: str) -> bytes:
    body = (bytes([typ, length]) + name.encode().ljust(4, b"\0")
            + form.encode().ljust(16, b"\0") + labels.encode().ljust(64, b"\0"))
    return _rec(FMT_TYPE, body)


T_MSG, T_FMTU, T_UNIT, T_MULT, T_CTUN = 0x81, 0x82, 0x83, 0x84, 0x85


def _schema(ctun_labels: str = "TimeUS,ThO", extra: bytes = b"") -> bytes:
    """FMT for FMT, then MSG, FMTU, UNIT, MULT and one data type."""
    return (_fmt(FMT_TYPE, FMT_LEN, "FMT", "BBnNZ", "Type,Length,Name,Format,Columns")
            + _fmt(T_MSG, 75, "MSG", "QZ", "TimeUS,Message")
            + _fmt(T_FMTU, 44, "FMTU", "QBNN", "TimeUS,FmtType,UnitIds,MultIds")
            + _fmt(T_UNIT, 76, "UNIT", "QbZ", "TimeUS,Id,Label")
            + _fmt(T_MULT, 20, "MULT", "Qbd", "TimeUS,Id,Mult")
            + _fmt(T_CTUN, 15, "CTUN", "Qf", ctun_labels) + extra)


def _msg(text: bytes) -> bytes:
    return _rec(T_MSG, struct.pack("<Q64s", 0, text))


def _tables(unit: bytes = b"m", mult: float = 0.01) -> bytes:
    return (_rec(T_FMTU, struct.pack("<QB16s16s", 0, T_CTUN, b"s" + unit, b"-A"))
            + _rec(T_UNIT, struct.pack("<Qb64s", 0, ord("s"), b"s"))
            + _rec(T_UNIT, struct.pack("<Qb64s", 0, ord(unit.decode()), b"m"))
            + _rec(T_MULT, struct.pack("<Qbd", 0, ord("A"), mult))
            + _rec(T_CTUN, struct.pack("<Qf", 1, 0.5)))


def _write(d: Path, name: str, data: bytes) -> str:
    p = d / name
    p.write_bytes(data)
    return str(p)


def _run(args: list[str], timeout: float = 10.0):
    import subprocess
    return subprocess.run([sys.executable, "-B", str(Path(__file__).resolve())] + args,
                          capture_output=True, timeout=timeout)


def _ctl(text: str) -> bool:
    return any(ord(c) < 0x20 and c != "\n" or 0x7F <= ord(c) <= 0x9F for c in text)


def selftest() -> int:
    import tempfile
    results: list[tuple[str, bool, str]] = []

    def check(name, cond, detail=""):
        results.append((name, bool(cond), str(detail)))
        print(("PASS  " if cond else "FAIL  ") + name + ("" if cond else "  " + str(detail)))

    def case(name):
        def deco(fn):
            try:
                fn()
            except Exception as e:      # a crash is a failure of the case
                check(name, False, f"{type(e).__name__}: {e}")
            return fn
        return deco

    with tempfile.TemporaryDirectory() as td:
        d = Path(td)
        fw = b"ArduCopter V4.6.3 (abc)"

        @case("clean pair parses and diffs")
        def _():
            a = _write(d, "a.bin", _schema() + _msg(fw) + _tables(b"m", 0.01))
            b = _write(d, "b.bin", _schema("TimeUS,ThO,Extra") + _msg(fw)
                       + _tables(b"m", 0.02))
            pa = parse_log(a)
            ok = (pa["firmware"] == fw.decode() and pa["bad_bytes"] == 0
                  and pa["counts"].get("CTUN") == 1 and pa["mults"].get("A") == 0.01)
            rows = compare(pa, parse_log(b), ["CTUN"])
            diffs = {f["field"]: f["diff"] for f in rows[0]["fields"]}
            check("clean pair parses and diffs",
                  ok and rows[0]["verdict"] == "changed"
                  and diffs["Extra"] == ["added"] and diffs["ThO"] == ["mult 0.01->0.02"],
                  (rows[0]["verdict"], diffs))

        @case("control bytes and injected Markdown are neutralised")
        def _():
            evil = (b"ArduCopter V4\x1b]0;pwn\x07\x1b[2J\n# FORGED\n![x](http://e.example/p.png)")
            a = _write(d, "old|x.bin", _schema() + _msg(evil) + _tables())
            b = _write(d, "new.bin",
                       _schema("TimeUS,Th\x1b[1mO",
                               _fmt(0x86, 11, "E\x1b[1", "Q", "Time"))
                       + _msg(b"ArduCopter V4.7.1\xc2\x9b31m\x7f") + _tables())
            mdp, jp = str(d / "out.md"), str(d / "out.json")
            r = _run([a, b, "--all-types", "--markdown", mdp, "--json", jp])
            out = r.stdout.decode("utf-8", "replace")
            text = (d / "out.md").read_text(encoding="utf-8")
            lines = text.split("\n")
            check("no control character on stdout", r.returncode == 0 and not _ctl(out),
                  (r.returncode, r.stderr[-200:], repr(out[:200])))
            check("no control character in the markdown", not _ctl(text))
            check("no injected markdown line",
                  not any(l.startswith(("# FORGED", "![", "\\# FORGED", "\\![")) for l in lines)
                  and not any(l.lstrip().startswith("# FORGED") for l in lines)
                  and "\\x0a" in text, [l for l in lines if "FORGED" in l])
            check("pipe in a path cannot break the table row",
                  "old\\|x" in text and "old|x" not in text)
            check("escapes are written as \\xNN", "\\x1b" in out and "\\x9b" in out)
        check("inert: U+202E and U+200B (Cf) come out as \\uXXXX, ESC still \\x1b",
              inert('a\u202eb\u200bc') == 'a\\u202eb\\u200bc' and inert('\x1b') == '\\x1b')

        @case("a zero-length FMT definition terminates")
        def _():
            for ln in (0, 1, 2):
                evil = (_schema(extra=_fmt(0x90, ln, "BAD", "Q", "TimeUS"))
                        + _rec(0x90, b"") * 3 + _msg(fw) + _tables())
                a = _write(d, "z.bin", evil)
                try:
                    r = _run([a, a, "--types", "CTUN"], timeout=10)
                    check(f"FMT length {ln} terminates", r.returncode == 0,
                          (r.returncode, r.stderr[-200:]))
                except Exception as e:
                    check(f"FMT length {ln} terminates", False, type(e).__name__)
            # FMT redefining FMT with another length is ignored, later records still parse
            a = _write(d, "z2.bin",
                       _schema(extra=_fmt(FMT_TYPE, 10, "FMT", "B", "X")) + _tables())
            p = parse_log(a)
            check("redefined FMT length is ignored", p["counts"].get("CTUN") == 1
                  and p["bad_bytes"] >= 1, (p["counts"], p["bad_bytes"]))

        @case("short FMTU, UNIT and MULT bodies do not raise")
        def _():
            for nm, t, ln in (("FMTU", T_FMTU, 12), ("UNIT", T_UNIT, 12), ("MULT", T_MULT, 12)):
                sch = (_fmt(FMT_TYPE, FMT_LEN, "FMT", "BBnNZ", "Type,Length,Name,Format,Columns")
                       + _fmt(T_MSG, 75, "MSG", "QZ", "TimeUS,Message")
                       + _fmt(t, ln, nm, "QB", "TimeUS,Id"))
                a = _write(d, "s.bin", sch + _rec(t, b"\0" * (ln - 3)) + _msg(fw))
                try:
                    p = parse_log(a)
                    check(f"short {nm} skipped", p["bad_bytes"] >= 1 and p["firmware"] == fw.decode(),
                          p["bad_bytes"])
                except Exception as e:
                    check(f"short {nm} skipped", False, f"{type(e).__name__}: {e}")
            # an FMT cut off after its header (89-byte record, but file ends short)
            a = _write(d, "s2.bin", _schema() + _fmt(0x91, 5, "X", "Q", "T")[:40])
            try:
                parse_log(a)
                check("truncated FMT does not raise", True)
            except Exception as e:
                check("truncated FMT does not raise", False, f"{type(e).__name__}: {e}")

        @case("NaN multiplier gives valid JSON and no change")
        def _():
            for v, word in ((float("nan"), "nan"), (float("inf"), "inf"), (float("-inf"), "-inf")):
                a = _write(d, "n1.bin", _schema() + _msg(fw) + _tables(b"m", v))
                b = _write(d, "n2.bin", _schema() + _msg(fw) + _tables(b"m", v))
                jp = str(d / "n.json")
                r = _run([a, b, "--types", "CTUN", "--json", jp])

                def bad_const(c):
                    raise ValueError("non-standard JSON constant " + c)
                try:
                    j = json.loads((d / "n.json").read_text(), parse_constant=bad_const)
                    got = j["types"][0]["fields"][1]["old"]["mult"]
                except Exception as e:
                    check(f"{word} multiplier: valid JSON", False, f"{type(e).__name__}: {e}")
                    continue
                check(f"{word} multiplier: valid JSON, identical logs report no change",
                      r.returncode == 0 and got == word and j["types"][0]["verdict"] == "same"
                      and b"0 differ" in r.stdout, (r.returncode, got, r.stderr[-200:]))

    failed = sum(1 for _, ok, _ in results if not ok)
    print(f"{len(results) - failed} passed, {failed} failed")
    return 1 if failed else 0



def main() -> int:
    ap = argparse.ArgumentParser(
        description="Diff the self-describing message formats of two ArduPilot dataflash logs.")
    ap.add_argument("old", nargs="?", help="the older .BIN log")
    ap.add_argument("new", nargs="?", help="the newer .BIN log")
    ap.add_argument("--json", metavar="FILE",
                    help="also write the full comparison to FILE as JSON")
    ap.add_argument("--markdown", metavar="FILE",
                    help="also write the comparison to FILE as a Markdown report")
    ap.add_argument("--types", help="comma separated; default: DEFAULT_TYPES in this script")
    ap.add_argument("--all-types", action="store_true",
                    help="compare every message type either log defines, not just the default set")
    ap.add_argument("--label-old", default="old",
                    help="name for the old log in the output (default: old)")
    ap.add_argument("--label-new", default="new",
                    help="name for the new log in the output (default: new)")
    ap.add_argument("--selftest", action="store_true",
                    help="run the built-in checks on synthetic logs and exit")
    a = ap.parse_args()
    if a.selftest:
        return selftest()
    if not a.old or not a.new:
        ap.error("old and new logs are required")

    old, new = parse_log(a.old), parse_log(a.new)
    if a.all_types:
        names = sorted({f["name"] for f in old["fmts"].values()} |
                       {f["name"] for f in new["fmts"].values()})
    elif a.types:
        names = [s.strip() for s in a.types.split(",") if s.strip()]
    else:
        names = sorted(DEFAULT_TYPES)

    rows = compare(old, new, names)
    changed = [r for r in rows if r["verdict"] not in ("same", "absent from both logs")]
    print(f"{a.label_old}: {inert(old['firmware'])}  {old['size']/1e6:.1f} MB")
    print(f"{a.label_new}: {inert(new['firmware'])}  {new['size']/1e6:.1f} MB")
    print(f"{len(names)} types compared, {len(changed)} differ\n")
    for r in rows:
        mark = " " if r["verdict"] == "same" else "*"
        print(f"{mark} {inert(r['type']):6} {r['verdict']}")
        for f in r["fields"]:
            if f["diff"]:
                print(f"      {inert(f['field']):10} {inert(', '.join(f['diff']))}")

    if a.json:
        Path(a.json).write_text(json.dumps(
            {"old": {"path": old["path"], "firmware": old["firmware"],
                     "size": old["size"], "label": a.label_old},
             "new": {"path": new["path"], "firmware": new["firmware"],
                     "size": new["size"], "label": a.label_new},
             "types": rows}, indent=1, allow_nan=False))
        print(f"\nwrote {a.json}")
    if a.markdown:
        Path(a.markdown).write_text(
            markdown(rows, old, new, a.label_old, a.label_new) + "\n")
        print(f"wrote {a.markdown}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
