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
import struct
import sys
from pathlib import Path

HEAD1, HEAD2 = 0xA3, 0x95
FMT_TYPE, FMT_LEN = 0x80, 89

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
        if ln < 0 or i + ln > n:
            i += 1
            bad += 1
            continue
        body = data[i + 3:i + ln]
        if t == FMT_TYPE:
            typ, length = body[0], body[1]
            name = bytes(body[2:6]).split(b"\0")[0].decode("ascii", "replace")
            form = bytes(body[6:22]).split(b"\0")[0].decode("ascii", "replace")
            labels = bytes(body[22:86]).split(b"\0")[0].decode("ascii", "replace")
            lengths[typ] = length
            fmts[typ] = {"name": name, "length": length,
                         "format": form, "labels": labels}
        else:
            nm = fmts.get(t, {}).get("name")
            if nm == "FMTU":
                _, ftype, uids, mids = struct.unpack("<QB16s16s", bytes(body))
                fmtu[ftype] = (uids.split(b"\0")[0].decode("ascii", "replace"),
                               mids.split(b"\0")[0].decode("ascii", "replace"))
            elif nm == "UNIT":
                _, uid, label = struct.unpack("<Qb64s", bytes(body))
                units[chr(uid & 0xFF)] = label.split(b"\0")[0].decode("ascii", "replace")
            elif nm == "MULT":
                _, mid, mult = struct.unpack("<Qbd", bytes(body))
                mults[chr(mid & 0xFF)] = mult
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
    out = [f"# FMT/FMTU diff: {label_old} vs {label_new}", "",
           f"- `{old['path']}` -- {old['firmware'] or 'firmware string not found'}, "
           f"{old['size']/1e6:.1f} MB",
           f"- `{new['path']}` -- {new['firmware'] or 'firmware string not found'}, "
           f"{new['size']/1e6:.1f} MB", "",
           "Read out of the logs themselves by `log-fmt-diff.py`; the",
           "multiplier and unit columns come from each log's own MULT and UNIT",
           "records, not from a table in this repo.", "",
           "## Verdict per message type", "",
           f"| type | {label_old} rows | {label_new} rows | verdict |",
           "|---|---:|---:|---|"]
    for r in rows:
        out.append(f"| `{r['type']}` | {r['count_old']} | {r['count_new']} | "
                   f"{r['verdict']}{(' -- ' + '; '.join(r['notes'])) if r['notes'] else ''} |")
    out += ["", "## Fields, per type", ""]
    for r in rows:
        out.append(f"### {r['type']} ({r['verdict']})")
        out.append("")
        if not r["fields"]:
            out += ["Not defined in either log.", ""]
            continue
        out.append(f"| field | {label_old} type | {label_old} mult | {label_old} unit "
                   f"| {label_new} type | {label_new} mult | {label_new} unit | change |")
        out.append("|---|---|---|---|---|---|---|---|")
        for f in r["fields"]:
            o, nw = f["old"], f["new"]
            def cell(x, k):
                return "--" if x is None else (
                    "" if x[k] in (None, "") else str(x[k]))
            out.append(
                f"| `{f['field']}` | {cell(o,'type')} | {cell(o,'mult')} | {cell(o,'unit')} "
                f"| {cell(nw,'type')} | {cell(nw,'mult')} | {cell(nw,'unit')} "
                f"| {', '.join(f['diff']) or ''} |")
        out.append("")
    return "\n".join(out)


def main() -> int:
    ap = argparse.ArgumentParser(
        description="Diff the self-describing message formats of two ArduPilot dataflash logs.")
    ap.add_argument("old", help="the older .BIN log")
    ap.add_argument("new", help="the newer .BIN log")
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
    a = ap.parse_args()

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
    print(f"{a.label_old}: {old['firmware']}  {old['size']/1e6:.1f} MB")
    print(f"{a.label_new}: {new['firmware']}  {new['size']/1e6:.1f} MB")
    print(f"{len(names)} types compared, {len(changed)} differ\n")
    for r in rows:
        mark = " " if r["verdict"] == "same" else "*"
        print(f"{mark} {r['type']:6} {r['verdict']}")
        for f in r["fields"]:
            if f["diff"]:
                print(f"      {f['field']:10} {', '.join(f['diff'])}")

    if a.json:
        Path(a.json).write_text(json.dumps(
            {"old": {"path": old["path"], "firmware": old["firmware"],
                     "size": old["size"], "label": a.label_old},
             "new": {"path": new["path"], "firmware": new["firmware"],
                     "size": new["size"], "label": a.label_new},
             "types": rows}, indent=1))
        print(f"\nwrote {a.json}")
    if a.markdown:
        Path(a.markdown).write_text(
            markdown(rows, old, new, a.label_old, a.label_new) + "\n")
        print(f"wrote {a.markdown}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
