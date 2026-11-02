#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Joe Fernandez
"""Compare the settings tables of two Betaflight builds and classify a `diff all`.

Answers "which lines of this backup survive a firmware change" without
flashing anything. Offline: reads text, touches no serial device.

  bf-settings-diff.py OLD_CAPTURE NEW_CAPTURE [--diff DIFF_FILE]
  bf-settings-diff.py --selftest

A capture is the raw text of CLI `get <substring>` commands (several may be
concatenated, any header lines between them). Each setting is a block:

    name = value
    Allowed range: A - B | Allowed values: X, Y | Array length: N |
    String length: A - B
    Default value: V            (not compared)

Take one in the CLI with get a, get e, get i, get o, get u and get y
(every setting name contains a vowel).

Part 1, always: names only in OLD, only in NEW, and attribute changes.
Part 2, with --diff: one line per diff line:
  same     in both, attributes equal, value valid in NEW; also a name in NEW
           but not in OLD, judged by NEW alone, when its value is valid
  changed  in both, attributes differ; the note says if the value is valid in NEW
  missing  not in NEW; the note lists close NEW-only names as suggestions,
           never as renames
  invalid  in both, attributes equal, value not valid in NEW; also a name in
           NEW but not in OLD whose value is not valid in NEW
  by-hand  every other line (feature, serial, aux, resource, ...)
Skipped, not classified: blank lines, `#` comments, and lines starting with
batch, defaults, save, board_name, manufacturer_id, mcu_id or signature.
`profile N` and `rateprofile N` lines are not classified either; they set the
context (master, profile 2, ...) shown for the lines after them.
Exit 0 if every classified line is `same` and there is no `by-hand` line, 1
otherwise; a diff with no `set` lines exits 1, since at least one `same` is
required. 2 on unreadable input.
"""
import argparse
import difflib
import re
import sys

NAME_RE = re.compile(r'^([a-z0-9_]+) = (.*)$')
NUM = r'(-?\d+(?:\.\d+)?)'
RANGE_RE = re.compile(r'^' + NUM + r'\s+-\s+' + NUM + r'$')
ATTR_PREFIX = (('Allowed range:', 'range'), ('Allowed values:', 'values'),
               ('Array length:', 'array'), ('String length:', 'string'))
IGNORED = {'batch', 'defaults', 'save', 'board_name', 'manufacturer_id',
           'mcu_id', 'signature', 'profile', 'rateprofile'}
SET_RE = re.compile(r'^set\s+([a-z0-9_]+)\s*=\s?(.*)$')


def parse_range(text):
    m = RANGE_RE.match(text.strip())
    if m:
        return (float(m.group(1)), float(m.group(2)))
    return text.strip()


def parse_attr(kind, text):
    text = text.strip()
    if kind == 'range' or kind == 'string':
        return parse_range(text)
    if kind == 'values':
        return tuple(v.strip().upper() for v in text.split(','))
    try:
        return int(text)
    except ValueError:
        return text


def parse_capture(text, label='capture', warn=None):
    """name -> {attr kind: value}. First block wins; a later block with
    different attributes is reported through warn(msg)."""
    table = {}
    cur = None          # (name, attrs) of the block being read
    dup = False

    def close():
        nonlocal cur, dup
        if cur is None:
            return
        name, attrs = cur
        if dup:
            if table[name] != attrs and warn:
                warn(f"{label}: {name} appears again with different attributes: "
                     f"{fmt_attrs(table[name])} vs {fmt_attrs(attrs)}; kept the first")
        else:
            table[name] = attrs
        cur = None

    for raw in text.splitlines():
        line = raw.strip()
        if not line:
            close()
            continue
        m = NAME_RE.match(line)
        if m:
            close()
            name = m.group(1)
            dup = name in table
            cur = (name, {})
            continue
        if cur is None:
            continue
        for prefix, kind in ATTR_PREFIX:
            if line.startswith(prefix):
                cur[1][kind] = parse_attr(kind, line[len(prefix):])
                break
    close()
    return table


def fmt_num(x):
    return str(int(x)) if x == int(x) else repr(x)


def fmt_range(r):
    if isinstance(r, tuple):
        return f"{fmt_num(r[0])} - {fmt_num(r[1])}"
    return str(r)


def fmt_attrs(a):
    if not a:
        return '(none)'
    parts = []
    for kind in ('range', 'values', 'array', 'string'):
        if kind not in a:
            continue
        v = a[kind]
        if kind in ('range', 'string'):
            parts.append(f"{kind} {fmt_range(v)}")
        elif kind == 'values':
            parts.append("values " + ','.join(v))
        else:
            parts.append(f"array {v}")
    return '; '.join(parts)


def attr_changes(old, new):
    out = []
    for kind in ('range', 'values', 'array', 'string'):
        o, n = old.get(kind), new.get(kind)
        if o == n:
            continue
        if kind in ('range', 'string'):
            f = fmt_range
        elif kind == 'values':
            f = lambda v: ','.join(v)
        else:
            f = str
        out.append(f"{kind} {f(o) if o is not None else '-'} -> {f(n) if n is not None else '-'}")
    return out


def check_scalar(v, attrs):
    """None if v is valid for one element under attrs, else the reason."""
    if 'values' in attrs:
        if v.strip().upper() not in attrs['values']:
            return f"'{v}' not in allowed values"
    r = attrs.get('range')
    if isinstance(r, tuple):
        try:
            x = float(v)
        except ValueError:
            return f"'{v}' is not numeric"
        if not (r[0] <= x <= r[1]):
            return f"{v} outside {fmt_range(r)}"
    return None


def validate(value, attrs):
    """None if value is valid under attrs, else the reason."""
    if 'array' in attrs:
        items = [s.strip() for s in value.split(',')]
        n = attrs['array']
        if isinstance(n, int) and len(items) != n:
            return f"{len(items)} elements, array length {n}"
        for it in items:
            why = check_scalar(it, attrs)
            if why:
                return why
        return None
    s = attrs.get('string')
    if isinstance(s, tuple):
        if not (s[0] <= len(value) <= s[1]):
            return f"length {len(value)} outside {fmt_range(s)}"
        return None
    return check_scalar(value, attrs)


def classify_diff(text, old, new):
    """Yield (lineno, context, class, name, value, note) for each reported line."""
    new_only = sorted(set(new) - set(old))
    ctx = 'master'
    for no, raw in enumerate(text.splitlines(), 1):
        line = raw.strip()
        if not line or line.startswith('#'):
            continue
        word = line.split()[0]
        if word in ('profile', 'rateprofile'):
            ctx = line
            continue
        if word in IGNORED:
            continue
        m = SET_RE.match(line)
        if not m:
            yield (no, ctx, 'by-hand', '', line, '')
            continue
        name, value = m.group(1), m.group(2).strip()
        if name not in new:
            sugg = difflib.get_close_matches(name, new_only, n=3)
            note = ('not in NEW; similar NEW-only names (suggestions, not renames): '
                    + ', '.join(sugg)) if sugg else 'not in NEW; no similar NEW-only name'
            if name not in old:
                note = 'not in OLD either; ' + note
            yield (no, ctx, 'missing', name, value, note)
            continue
        if name not in old:
            # in NEW only: nothing to compare against; judge it by NEW alone
            why = validate(value, new[name])
            yield (no, ctx, 'invalid' if why else 'same', name, value,
                   ('not in OLD capture; ' + why) if why else 'not in OLD capture; valid in NEW')
            continue
        why = validate(value, new[name])
        if old[name] != new[name]:
            ch = '; '.join(attr_changes(old[name], new[name]))
            note = ch + ('; value INVALID under NEW: ' + why if why else '; value valid under NEW')
            yield (no, ctx, 'changed', name, value, note)
        elif why:
            yield (no, ctx, 'invalid', name, value, why)
        else:
            yield (no, ctx, 'same', name, value, '')


def table_report(old, new, out):
    only_old = sorted(set(old) - set(new))
    only_new = sorted(set(new) - set(old))
    changed = [n for n in sorted(set(old) & set(new)) if old[n] != new[n]]
    print(f"settings: OLD {len(old)}, NEW {len(new)}; only in OLD {len(only_old)}, "
          f"only in NEW {len(only_new)}, attributes differ {len(changed)}", file=out)
    print(f"\n== only in OLD (removed or renamed): {len(only_old)}", file=out)
    for n in only_old:
        print(f"  {n}    [{fmt_attrs(old[n])}]", file=out)
    print(f"\n== only in NEW (added): {len(only_new)}", file=out)
    for n in only_new:
        print(f"  {n}    [{fmt_attrs(new[n])}]", file=out)
    print(f"\n== attributes differ: {len(changed)}", file=out)
    for n in changed:
        print(f"  {n}", file=out)
        for c in attr_changes(old[n], new[n]):
            print(f"      {c}", file=out)


def read(path):
    with open(path, encoding='utf-8', errors='replace') as f:
        return f.read()


def run(old_text, new_text, diff_text, out, err):
    warns = []

    def warn(m):
        warns.append(m)
        print('warning: ' + m, file=err)

    old = parse_capture(old_text, 'OLD', warn)
    new = parse_capture(new_text, 'NEW', warn)
    if not old or not new:
        print(f"no settings parsed (OLD {len(old)}, NEW {len(new)})", file=err)
        return 2, {}
    table_report(old, new, out)
    if diff_text is None:
        return 0, {}
    print("\n== diff lines  (line  context  class  name  value  note)", file=out)
    counts = {'same': 0, 'changed': 0, 'missing': 0, 'invalid': 0, 'by-hand': 0}
    for no, ctx, cls, name, value, note in classify_diff(diff_text, old, new):
        counts[cls] += 1
        print(f"{no}\t{ctx}\t{cls}\t{name}\t{value}\t{note}", file=out)
    print("\n== summary: " + ', '.join(f"{k} {v}" for k, v in counts.items()), file=out)
    return (0 if counts['same'] and sum(counts.values()) == counts['same'] else 1), counts


def selftest():
    import io
    results = []

    def case(name, cond, detail=''):
        results.append(cond)
        print(('PASS  ' if cond else 'FAIL  ') + name + ('' if cond else '  ' + str(detail)))

    old_t = """### get e
get e
alpha_int = 5
Allowed range: 0 - 10

mode_enum = PT1
Allowed values: PT1, BIQUAD

arr_s = 1,2,3
Array length: 3
Default value: 0,0,0

arr_r = 1,2
Allowed range: 0 - 5
Array length: 2

name_str = abc
String length: 1 - 8

gone_one = 1
Allowed range: 0 - 1

range_chg = 5
Allowed range: 0 - 10

enum_chg = A
Allowed values: A, B

len_chg = 1,2
Array length: 2

dup_one = 1
Allowed range: 0 - 10

> get i
get i
dup_one = 1
profile 1
Allowed range: 0 - 99

fl_range = 0.5
Allowed range: 0.000 - 1.000
"""
    new_t = """# header
get a
alpha_int = 5
Allowed range: 0 - 10

mode_enum = BIQUAD
Allowed values: PT1, BIQUAD

arr_s = 1,2,3
Array length: 3

arr_r = 1,2
Allowed range: 0 - 5
Array length: 2

name_str = abc
String length: 1 - 8

gone_one_x = 1
Allowed range: 0 - 1

range_chg = 5
Allowed range: 0 - 4

enum_chg = A
Allowed values: A, B, C

len_chg = 1,2,3
Array length: 3

dup_one = 1
Allowed range: 0 - 10

fl_range = 0.5
Allowed range: 0.000 - 1.000

only_new = 1
Allowed range: 0 - 1
"""
    diff_t = """# comment
batch start
defaults nosave
board_name X
manufacturer_id Y
mcu_id 1
signature
resource SERVO 1 B05
set alpha_int = 5
set mode_enum = biquad
set arr_s = 1, 2, 3
set arr_r = 1,2
set name_str = abc
set gone_one = 1
set range_chg = 7
set enum_chg = b
set len_chg = 1,2
set dup_one = 1
set fl_range = 0.25
profile 2
set alpha_int = 11
set arr_r = 1,9
set name_str = abcdefghij
set arr_s = 1,2
set mode_enum = nope
set range_chg = 3
set len_chg = 1,2,3
set fl_range = 2
set unknown_zzz = 1
set only_new = 1
rateprofile 1
set alpha_int = 6
feature GPS
aux 0 0 0 1875 2100 0 0
save
"""
    warns = []
    old = parse_capture(old_t, 'OLD', warns.append)
    new = parse_capture(new_t, 'NEW')
    case('parse: names, blank lines and headers ignored', set(old) == {
        'alpha_int', 'mode_enum', 'arr_s', 'arr_r', 'name_str', 'gone_one', 'range_chg',
        'enum_chg', 'len_chg', 'dup_one', 'fl_range'}, sorted(old))
    case('parse: Default value not an attribute', 'default' not in old['arr_s'] and old['arr_s'] == new['arr_s'])
    case('parse: array plus range in one block', old['arr_r'] == {'range': (0.0, 5.0), 'array': 2}, old['arr_r'])
    case('parse: duplicate with different attributes warns, first kept',
         len(warns) == 1 and 'dup_one' in warns[0] and old['dup_one'] == {'range': (0.0, 10.0)}, warns)
    case('parse: profile line between name and attributes is skipped',
         parse_capture('x_y = 1\nprofile 1\nAllowed range: 0 - 9\n')['x_y'] == {'range': (0.0, 9.0)})
    case('parse: identical duplicate is silent', parse_capture(
        'a_b = 1\nAllowed range: 0 - 1\n\na_b = 1\nAllowed range: 0 - 1\n', 'X', lambda m: 1 / 0) == {'a_b': {'range': (0.0, 1.0)}})

    buf = io.StringIO()
    errb = io.StringIO()
    rc, counts = run(old_t, new_t, diff_t, buf, errb)
    report = buf.getvalue()
    case('table: only in OLD', '== only in OLD (removed or renamed): 1\n  gone_one ' in report)
    case('table: only in NEW', '  gone_one_x ' in report and '  only_new ' in report)
    case('table: attribute changes printed old -> new',
         'range 0 - 10 -> 0 - 4' in report and 'values A,B -> A,B,C' in report and 'array 2 -> 3' in report)

    rows = {}
    for no, ctx, cls, name, value, note in classify_diff(diff_t, old, new):
        rows.setdefault((ctx, name or value), []).append((no, cls, value, note))

    def cls_of(ctx, name):
        return rows[(ctx, name)][0][1]
    case('same: int in range', cls_of('master', 'alpha_int') == 'same')
    case('same: enum, case-insensitive', cls_of('master', 'mode_enum') == 'same')
    case('same: array with spaces', cls_of('master', 'arr_s') == 'same')
    case('same: string in length', cls_of('master', 'name_str') == 'same')
    case('same: float range', cls_of('master', 'fl_range') == 'same')
    case('missing: not in NEW', cls_of('master', 'gone_one') == 'missing')
    case('missing: suggestion labelled, not a rename',
         'gone_one_x' in rows[('master', 'gone_one')][0][3] and 'not renames' in rows[('master', 'gone_one')][0][3])
    case('changed: range, value now invalid',
         cls_of('master', 'range_chg') == 'changed' and 'INVALID' in rows[('master', 'range_chg')][0][3])
    case('changed: range, value still valid', rows[('profile 2', 'range_chg')][0][1] == 'changed'
         and 'value valid under NEW' in rows[('profile 2', 'range_chg')][0][3])
    case('changed: enum list grew', cls_of('master', 'enum_chg') == 'changed')
    case('changed: array length, old-size value invalid',
         cls_of('master', 'len_chg') == 'changed' and 'INVALID' in rows[('master', 'len_chg')][0][3])
    case('changed: array length, new-size value valid',
         'value valid under NEW' in rows[('profile 2', 'len_chg')][0][3])
    case('invalid: int out of range', cls_of('profile 2', 'alpha_int') == 'invalid')
    case('invalid: array element out of range', cls_of('profile 2', 'arr_r') == 'invalid')
    case('invalid: string too long', cls_of('profile 2', 'name_str') == 'invalid')
    case('invalid: array wrong count', cls_of('profile 2', 'arr_s') == 'invalid')
    case('invalid: enum member absent', cls_of('profile 2', 'mode_enum') == 'invalid')
    case('invalid: float out of range', cls_of('profile 2', 'fl_range') == 'invalid')
    case('missing: no close name', 'no similar' in rows[('profile 2', 'unknown_zzz')][0][3]
         and rows[('profile 2', 'unknown_zzz')][0][1] == 'missing')
    case('NEW-only name in diff is judged by NEW', cls_of('profile 2', 'only_new') == 'same')
    case('context: master before first profile', ('master', 'alpha_int') in rows)
    case('context: profile line sets context', ('profile 2', 'alpha_int') in rows)
    case('context: rateprofile replaces it', ('rateprofile 1', 'alpha_int') in rows)
    case('by-hand: feature and aux', rows[('rateprofile 1', 'feature GPS')][0][1] == 'by-hand'
         and rows[('rateprofile 1', 'aux 0 0 0 1875 2100 0 0')][0][1] == 'by-hand'
         and rows[('master', 'resource SERVO 1 B05')][0][1] == 'by-hand')
    case('ignored: header, batch, defaults, board, profile, save lines not reported',
         sum(counts.values()) == sum(len(v) for v in rows.values())
         and not any('board_name' in k[1] or 'batch' in k[1] or k[1].startswith('save') for k in rows))
    case('counts', counts == {'same': 9, 'changed': 5, 'missing': 2, 'invalid': 6, 'by-hand': 3}, counts)
    case('exit 1 when anything is not same', rc == 1)

    ok_diff = "profile 1\nset alpha_int = 5\nset name_str = abc\n"
    rc0, c0 = run(old_t, old_t, ok_diff, io.StringIO(), io.StringIO())
    case('exit 0 when all same, no by-hand', rc0 == 0 and c0['same'] == 2, (rc0, c0))
    rc1, _ = run(old_t, old_t, ok_diff + "feature GPS\n", io.StringIO(), io.StringIO())
    case('exit 1 on a by-hand line', rc1 == 1)
    rc2, _ = run('nothing', 'nothing', None, io.StringIO(), io.StringIO())
    case('exit 2 when no settings parse', rc2 == 2)
    rc3, _ = run(old_t, new_t, None, io.StringIO(), io.StringIO())
    case('no --diff: table only, exit 0', rc3 == 0)

    failed = results.count(False)
    print(f"{len(results) - failed} passed, {failed} failed")
    return failed


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('old', nargs='?', help='capture of the older firmware')
    ap.add_argument('new', nargs='?', help='capture of the newer firmware')
    ap.add_argument('--diff', help='a `diff all` backup to classify against NEW')
    ap.add_argument('--selftest', action='store_true')
    a = ap.parse_args()
    if a.selftest:
        return selftest()
    if not a.old or not a.new:
        ap.error('OLD_CAPTURE and NEW_CAPTURE are required')
    try:
        old_t, new_t = read(a.old), read(a.new)
        diff_t = read(a.diff) if a.diff else None
    except OSError as e:
        print(f"cannot read input: {e}", file=sys.stderr)
        return 2
    rc, _ = run(old_t, new_t, diff_t, sys.stdout, sys.stderr)
    return rc


if __name__ == '__main__':
    sys.exit(main())
