#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Joe Fernandez
"""Check that a firmware upgrade actually carried a vehicle's settings across.

"It booted and reports the new version" is not evidence the upgrade worked.
ArduPilot renames parameters between releases and converts the old storage
slots on first boot; if a conversion is missing, the new name comes up at the
new firmware's default and the aircraft is quietly detuned. This compares what
the board holds AFTER the upgrade against what it held BEFORE, times the
conversion factor, and reports every row that does not agree.

Three classes of answer for a renamed parameter, and only the first is a
failure:

  MISMATCH        new value is not old*factor, and it is not explained as
                  DEFAULT-MOVED (old value at the old board default and new
                  value at the new board default). A conversion is missing
                  or wrong.
  DEFAULT-MOVED   old value was the old firmware's default, so it was never
                  written to storage, so there was nothing for the conversion
                  to find; the new name sits at the new default. Behaviour
                  changes and it is worth reading, but no setting was lost.
  OK              new == old*factor within float32 tolerance.

That middle class is why --pre-defaults and --post-defaults matter. Without
the board's own defaults you cannot tell a failed conversion from a default
that moved, and neither apm.pdef.xml nor SITL can supply them: defaults are
board- and frame-specific. Fetch them with ap-param-dump.py --defaults.

The factor table is JSON: {"renames": [{"old":..., "new":..., "factor":...}]}.
Take the factors from the FIRMWARE'S conversion tables, not from the metadata
units field. Units are documentation and can lag a scale change: between
Copter 4.6.3 and 4.7.1 five PSC PID rows scale by 0.1, 0.01 and 0.001 in
AC_PosControl.cpp while their units field implies 1.0.

Usage:
  ap-upgrade-verify.py pre.param post.param --renames map.json \
      --pre-defaults pre.defaults --post-defaults post.defaults

Exit status: 1 if there is any MISMATCH, or any UNEXPLAINED change among the
parameters that are in neither rename list; otherwise 0. NOT COMPARABLE rows (a
renamed name absent from either dump) are reported and do not fail the run:
a feature that is not fitted, e.g. no rangefinder, leaves its parameters out
of a dump, so there is nothing to compare.

Offline. Reads files, talks to nothing.
"""
import argparse, json, math, re, struct, sys, unicodedata

ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
ap.add_argument('pre', help='parameter dump from before the upgrade')
ap.add_argument('post', help='parameter dump from after the upgrade')
ap.add_argument('--renames', required=True, help='JSON factor table')
ap.add_argument('--pre-defaults', help='board defaults on the old firmware')
ap.add_argument('--post-defaults', help='board defaults on the new firmware')
ap.add_argument('--rel-tol', type=float, default=1e-5)
ap.add_argument('--quiet-ok', action='store_true', help='only print problems')
a = ap.parse_args()

# Parameters the firmware itself rewrites, so they differ between any two
# dumps and have nothing to do with an upgrade. Two kinds:
#   counters       monotonic, tick on every boot
#   re-learned     measured or calibrated afresh at startup. Gyro offsets and
#                  their cal temperature are recomputed by the startup gyro
#                  calibration every boot; the ground pressure is whatever
#                  the baro read when it last initialised; the declination
#                  follows COMPASS_AUTODEC and the current position.
# Listing these separately is what keeps a real finding visible: without it,
# ten gyro and baro rows bury the one parameter that actually failed to
# convert.
VOLATILE = ('STAT_BOOTCNT', 'STAT_RUNTIME', 'STAT_FLTTIME', 'STAT_RESET',
            'SYS_NUM_RESETS',
            'INS_GYROFFS_', 'INS_GYR2OFFS_', 'INS_GYR3OFFS_',
            'INS_GYR1_CALTEMP', 'INS_GYR2_CALTEMP', 'INS_GYR3_CALTEMP',
            'INS_ACC1_CALTEMP', 'INS_ACC2_CALTEMP', 'INS_ACC3_CALTEMP',
            'INS_TCAL',
            'BARO1_GND_PRESS', 'BARO2_GND_PRESS', 'BARO3_GND_PRESS',
            'GND_ABS_PRESS',
            'COMPASS_DEC')


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
    """Make a string safe to print: C0, DEL and C1 characters become \\xNN.
    A parameter name comes from a file or a JSON map and can carry an escape
    sequence that a terminal would act on. Applied where text is printed, not
    where it is parsed, so the comparisons still see the real bytes."""
    return ''.join(map(_inert_char, str(s)))


def load(path):
    """Return ({name: value}, number of lines whose value did not parse).
    nan and inf are not parameter values a board stores; they count as
    unparseable rather than as a number that compares oddly."""
    d, bad = {}, 0
    for line in open(path):
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        for sep in (',', '\t', ' '):
            if sep in line:
                name, _, val = line.partition(sep)
                break
        else:
            continue
        try:
            v = float(val.strip())
        except ValueError:
            bad += 1
            continue
        if not math.isfinite(v):
            bad += 1
            continue
        d[name.strip()] = v
    return d, bad


def f32(x):
    """Compare as the board stores them. A gain that survived a float32 round
    trip is not a mismatch just because Python kept more digits."""
    try:
        return struct.unpack('f', struct.pack('f', x))[0]
    except OverflowError:
        return x


def close(x, y):
    x, y = f32(x), f32(y)
    if math.isnan(x) or math.isnan(y):
        return False
    if x == y:
        return True
    if math.isinf(x) or math.isinf(y):
        return False
    scale = max(abs(x), abs(y))
    if scale == 0:
        return False
    return abs(x - y) <= a.rel_tol * scale


def main():
    (pre, pre_bad), (post, post_bad) = load(a.pre), load(a.post)
    pre_def, pre_def_bad = load(a.pre_defaults) if a.pre_defaults else ({}, 0)
    post_def, post_def_bad = (load(a.post_defaults) if a.post_defaults
                              else ({}, 0))
    table = json.load(open(a.renames))['renames']

    print(f"pre  {inert(a.pre)}: {len(pre)} params"
          f"{f', {len(pre_def)} defaults' if pre_def else ''}")
    print(f"post {inert(a.post)}: {len(post)} params"
          f"{f', {len(post_def)} defaults' if post_def else ''}")
    print(f"renames: {len(table)}")
    for path, n in ((a.pre, pre_bad), (a.post, post_bad),
                    (a.pre_defaults, pre_def_bad),
                    (a.post_defaults, post_def_bad)):
        if n:
            print(f"unparseable lines in {inert(path)}: {n}")
    print()

    renamed_old = {r['old'] for r in table}
    renamed_new = {r['new'] for r in table}

    print("=" * 72)
    print("RENAMES")
    print("=" * 72)
    mismatches, moved, absent, ok = [], [], [], 0

    for r in sorted(table, key=lambda r: r['old']):
        old, new, factor = r['old'], r['new'], float(r['factor'])
        if old not in pre:
            absent.append((old, new, 'old name not in the pre dump'))
            continue
        if new not in post:
            absent.append((old, new, 'new name not in the post dump'))
            continue
        want, got = pre[old] * factor, post[new]
        if close(want, got):
            ok += 1
            if not a.quiet_ok:
                print(f"  OK             {inert(old):20s} -> "
                      f"{inert(new):20s} "
                      f"{pre[old]:g} x {factor:g} = {got:g}")
            continue

        # was the old value ever actually stored?
        was_default = (old in pre_def) and close(pre[old], pre_def[old])
        at_new_default = (new in post_def) and close(got, post_def[new])
        if was_default and at_new_default:
            moved.append((old, new, factor, pre[old], want, got,
                          post_def.get(new)))
        else:
            mismatches.append((old, new, factor, pre[old], want, got,
                               pre_def.get(old), post_def.get(new)))

    print(f"\n  {ok} of {len(table)} carried across exactly.")

    if moved:
        print(f"\n  --- DEFAULT-MOVED, param was never stored ({len(moved)}) ---")
        for old, new, factor, pv, want, got, nd in moved:
            print(f"  {inert(old)} -> {inert(new)}")
            print(f"      was {pv:g} (the old default), expected {want:g}, "
                  f"board holds {got:g} (the new default {nd:g})")

    if absent:
        print(f"\n  --- NOT COMPARABLE ({len(absent)}) ---")
        for old, new, why in absent:
            print(f"  {inert(old)} -> {inert(new)}: {why}")

    if mismatches:
        print(f"\n  *** MISMATCH ({len(mismatches)}) ***")
        for old, new, factor, pv, want, got, od, nd in mismatches:
            print(f"  {inert(old)} -> {inert(new)}  factor {factor:g}")
            print(f"      pre {pv!r}  expected {want!r}  ACTUAL {got!r}")
            print(f"      (old default {od!r}, new default {nd!r})")
    else:
        print("\n  no mismatches.")

    print("\n" + "=" * 72)
    print("EVERYTHING ELSE THAT CHANGED")
    print("=" * 72)
    shared = (set(pre) & set(post)) - renamed_old - renamed_new
    changed = [n for n in sorted(shared) if not close(pre[n], post[n])]

    volatile, defmoved, unexplained = [], [], []
    for n in changed:
        if n.startswith(VOLATILE):
            volatile.append(n)
        elif (n in pre_def and n in post_def
              and close(pre[n], pre_def[n]) and close(post[n], post_def[n])):
            defmoved.append(n)
        else:
            unexplained.append(n)

    print(f"  {len(shared)} names in both dumps and in neither rename list; "
          f"{len(changed)} changed value.")
    if volatile:
        print(f"\n  --- rewritten by the firmware, expected to differ "
              f"({len(volatile)}) ---")
        for n in volatile:
            print(f"  {inert(n)}: {pre[n]:g} -> {post[n]:g}")
    if defmoved:
        print(f"\n  --- default moved, param unstored ({len(defmoved)}) ---")
        for n in defmoved:
            print(f"  {inert(n)}: {pre[n]:g} -> {post[n]:g}  (default "
                  f"{pre_def[n]:g} -> {post_def[n]:g})")
    if unexplained:
        print(f"\n  *** UNEXPLAINED ({len(unexplained)}) ***")
        for n in unexplained:
            od = pre_def.get(n)
            nd = post_def.get(n)
            print(f"  {inert(n)}: {pre[n]!r} -> {post[n]!r}  "
                  f"(defaults {od!r} -> {nd!r})")
    else:
        print("\n  nothing unexplained.")

    print("\n" + "=" * 72)
    print("NAMES THAT APPEARED OR VANISHED")
    print("=" * 72)
    gone = sorted(set(pre) - set(post) - renamed_old)
    new_names = sorted(set(post) - set(pre) - renamed_new)
    gone_nondefault = [n for n in gone
                       if not (n in pre_def and close(pre[n], pre_def[n]))]
    print(f"  {len(gone)} gone ({len(gone_nondefault)} of them non-default), "
          f"{len(new_names)} new.")
    if gone_nondefault:
        print("\n  --- gone AND had a non-default value ---")
        for n in gone_nondefault:
            print(f"  {inert(n)} = {pre[n]!r}  (old default {pre_def.get(n)!r})")

    return 1 if (mismatches or unexplained) else 0


sys.exit(main())
