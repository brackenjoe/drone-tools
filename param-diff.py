#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Joe Fernandez
"""Diff ArduPilot parameter metadata between two firmware trees.

Answers "what changes if I upgrade" without flashing anything. It compares
parameter metadata only: a parameter's default value is NOT compared, so a
changed default does not show up here. Of the parameters in both trees it
compares Range, Values and Increment, and nothing else.

  param-diff.py <old-tree> <new-tree> [--vehicle ArduCopter]

Generates apm.pdef.xml in each tree if missing, then reports parameters
added, removed (with a name-pairing guess for renames), and those whose
Range, Values or Increment changed.

Needs ArduPilot source trees (it runs Tools/autotest/param_metadata/param_parse.py
when apm.pdef.xml is missing). Standard library only.
"""
import argparse, subprocess, sys
from pathlib import Path
import xml.etree.ElementTree as etree

ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
ap.add_argument('old'); ap.add_argument('new')
ap.add_argument('--vehicle', default='ArduCopter')
a = ap.parse_args()

def pdef(tree):
    d = Path(tree) / 'Tools/autotest/param_metadata'
    x = d / 'apm.pdef.xml'
    if not x.exists():
        print(f"generating metadata for {tree} ...")
        subprocess.run([sys.executable, 'param_parse.py', '--vehicle', a.vehicle,
                        '--format', 'xml'], cwd=d, check=True,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    # The cached file is one per tree, whichever vehicle generated it last; a
    # Copter file diffed as ArduPlane is silently a cross-vehicle diff.
    head = x.read_text(errors='replace')[:4096]
    want = f'<parameters name="{a.vehicle}"'
    if want not in head:
        sys.exit(f"{x} was generated for another vehicle, not {a.vehicle}.\n"
                 f"Move it aside (it is regenerated if missing), or diff "
                 f"`git archive` exports of the two commits instead.")
    return x

def load(x):
    root = etree.parse(str(x)).getroot()
    out = {}
    for p in root.iter('param'):
        name = p.get('name') or ''
        name = name.split(':')[-1]
        f = {c.get('name'): (c.text or '').strip()
             for c in p.iter('field') if c.get('name')}
        # humanName/documentation are attributes on <param>, not <field>s
        f['_human'] = (p.get('humanName') or '').strip()
        f['_doc'] = (p.get('documentation') or '').strip()
        out[name] = f
    return out

old, new = load(pdef(a.old)), load(pdef(a.new))
added   = sorted(set(new) - set(old))
removed = sorted(set(old) - set(new))

print(f"\n{'='*62}\n{a.vehicle}: {Path(a.old).name}  ->  {Path(a.new).name}\n{'='*62}")
print(f"params: {len(old)} -> {len(new)}   (+{len(added)} / -{len(removed)})")

# A rename shows up as removed+added. The human-readable name usually
# survives, so pair on that: it separates true removals from renames.
by_human_new = {}
for n in added:
    h = new[n].get('_human', '')
    if h:
        by_human_new.setdefault(h, []).append(n)
renames, true_removed = [], []
for n in removed:
    h = old[n].get('_human', '')
    cands = by_human_new.get(h, [])
    if h and len(cands) == 1:
        renames.append((n, cands[0], h))
    else:
        true_removed.append(n)

# One candidate on the new side is not enough: two *removed* params can share
# a display name and both claim it, which emits a many-to-one map that reads
# as fact. Real case, 4.6.3 -> 4.7.1: AROT_FW_V_P and PSC_VELXY_P are both
# "Velocity (horizontal) P gain", so both paired to PSC_NE_VEL_P, and the
# wrong one is invisible unless you go looking. Matching is a heuristic; say
# so where it is ambiguous rather than picking a winner.
claims = {}
for o, w, h in renames:
    claims.setdefault(w, []).append(o)
ambiguous = {w: os for w, os in claims.items() if len(os) > 1}
if ambiguous:
    renames = [(o, w, h) for o, w, h in renames if w not in ambiguous]
    for w, os in ambiguous.items():
        true_removed.extend(os)
    true_removed.sort()

renamed_new = {b for _, b, _ in renames}

print(f"\n--- LIKELY RENAMED ({len(renames)}) — scripts and saved .param files break here ---")
for o, w, h in renames:
    print(f"  ~ {o:<22} -> {w:<22} {h[:34]}")

if ambiguous:
    print(f"\n--- AMBIGUOUS ({len(ambiguous)}) — resolve by hand before trusting a map ---")
    for w, os in sorted(ambiguous.items()):
        print(f"  ? {' and '.join(os)} all match {w}")
        print(f"      shared display name: {old[os[0]].get('_human','')!r}")
        print("      listed under TRULY REMOVED below; at most one is a real rename")

print(f"\n--- TRULY REMOVED ({len(true_removed)}) ---")
for n in true_removed:
    print(f"  - {n:<24} {old[n].get('_human','')[:52]}")

added = [n for n in added if n not in renamed_new]
print(f"\n--- ADDED ({len(added)}) ---")
for n in added:
    print(f"  + {n:<24} {new[n].get('_human','')[:58]}")

moved = []
for n in sorted(set(old) & set(new)):
    for key in ('Range', 'Values', 'Increment'):
        o, w = old[n].get(key, ''), new[n].get(key, '')
        if o != w and (o or w):
            moved.append((n, key, o, w))
            break
print(f"\n--- RANGE/VALUES/INCREMENT CHANGED ({len(moved)}; defaults are not compared) ---")
for n, key, o, w in moved[:40]:
    print(f"  ~ {n:<24} {key}: {o[:28]!r} -> {w[:28]!r}")
if len(moved) > 40:
    print(f"  ... and {len(moved)-40} more")
