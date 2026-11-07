#!/bin/bash
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Joe Fernandez
# Launch ArduPilot SITL from a version-matched source tree.

usage() {
  cat <<'EOF'
Usage: sitl.sh [TREE] [VEHICLE] [sim_vehicle.py args...]
       sitl.sh -h | --help

Launch ArduPilot SITL (via Tools/autotest/sim_vehicle.py) from a source tree
that matches the firmware version you want to test.

TREE selects the source tree (default: master):
  master            the main ArduPilot checkout, $AP_TREES/ardupilot
  X.Y.Z             a release, e.g. 4.7.1: the first existing of
                    $AP_TREES/ap-Copter-X.Y.Z, ap-Plane-X.Y.Z, ap-X.Y.Z;
                    ap-Plane-X.Y.Z comes first when VEHICLE is ArduPlane
  HASH              a commit (7-40 hex digits): the first existing of
                    $AP_TREES/ap-plane-HASH, ap-copter-HASH, ap-HASH;
                    ap-plane-HASH comes first when VEHICLE is ArduPlane,
                    and ap-copter-HASH first otherwise
  plane             the first $AP_TREES/ap-plane-* tree
  /abs/path         any ArduPilot tree

VEHICLE is ArduCopter (default) or ArduPlane; it picks the version.h that is
reported and is passed to sim_vehicle.py as -v. Everything after it passes
through to sim_vehicle.py (for example -w to wipe parameters).

Tree layout: one git worktree per version, beside the main checkout, named
ap-<Vehicle>-<version> or ap-<vehicle>-<hash>. ArduPilot tags Copter and
Plane releases on one commit when they ship together, so one worktree can
serve both vehicles: it needs both `./waf copter` and `./waf plane` run in
it, and then build/sitl/bin/ holds arducopter and arduplane side by side.
Do not add a second worktree for the same commit; name the one you have
ap-<version> if it belongs to no one vehicle.

Environment:
  AP_TREES   directory that holds the trees (default: the directory this
             script is in)
  AP_VENV    a Python venv whose bin/ has MAVProxy and pymavlink, put first
             on PATH if it exists (default: $AP_TREES/ardupilot/venv)

On launch it prints the tree, the firmware version and the git hash.
Never pass --map or --console on a headless machine: they need X.

Examples:
  sitl.sh 4.7.1                  Copter 4.7.1
  sitl.sh 4.7.1 ArduPlane        Plane 4.7.1, from the same tree if shared
  sitl.sh master                 upstream head
  sitl.sh 4.6.3 ArduCopter -w    extra arguments pass through
EOF
}

case "${1:-}" in
  -h|--help) usage; exit 0 ;;
esac

SELF="$(readlink -f "$0")"
AP_TREES="${AP_TREES:-$(dirname "$SELF")}"
T="${AP_TREES%/}"

# first existing directory among the arguments; nothing printed if none
first_dir() { local d; for d in "$@"; do [ -d "$d" ] && { echo "$d"; return 0; }; done; return 1; }

SEL="${1:-master}"
# the vehicle is the second argument; it decides which tree is tried first
WANT_VEHICLE="${2:-ArduCopter}"
case "$SEL" in
  master|"") AP="$T/ardupilot" ;;
  /*)        AP="$SEL" ;;
  plane)     AP="$(first_dir "$T"/ap-plane-*/)" || AP="$T/ap-plane-missing" ;;
  [0-9]*.[0-9]*.[0-9]*)
             if [ "$WANT_VEHICLE" = ArduPlane ]; then
               AP="$(first_dir "$T/ap-Plane-$SEL" "$T/ap-Copter-$SEL" "$T/ap-$SEL")" \
                 || AP="$T/ap-Plane-$SEL"
             else
               AP="$(first_dir "$T/ap-Copter-$SEL" "$T/ap-Plane-$SEL" "$T/ap-$SEL")" \
                 || AP="$T/ap-Copter-$SEL"
             fi ;;
  *)
    if [[ "$SEL" =~ ^[0-9a-f]{7,40}$ ]]; then
      if [ "$WANT_VEHICLE" = ArduPlane ]; then
        AP="$(first_dir "$T/ap-plane-$SEL" "$T/ap-copter-$SEL" "$T/ap-$SEL")" \
          || AP="$T/ap-plane-$SEL"
      else
        AP="$(first_dir "$T/ap-copter-$SEL" "$T/ap-plane-$SEL" "$T/ap-$SEL")" \
          || AP="$T/ap-copter-$SEL"
      fi
    else
      echo "unknown tree '$SEL'" >&2
      usage >&2
      exit 1
    fi ;;
esac
AP="${AP%/}"
[ $# -gt 0 ] && shift
VEHICLE="${1:-ArduCopter}"
[ $# -gt 0 ] && shift

case "$VEHICLE" in
  ArduCopter|ArduPlane) ;;
  -*) echo "expected a vehicle (ArduCopter or ArduPlane), got '$VEHICLE'" >&2; usage >&2; exit 1 ;;
esac
case "$VEHICLE" in ArduPlane) VF="$AP/ArduPlane/version.h" ;; *) VF="$AP/ArduCopter/version.h" ;; esac

if [ ! -d "$AP" ]; then
  echo "no such tree: $AP (AP_TREES=$T)" >&2
  exit 1
fi
ls "$AP"/build/sitl/bin/* >/dev/null 2>&1 || echo "note: nothing built in $AP yet"
# One venv serves every tree; it is only MAVProxy/pymavlink tooling.
VENV="${AP_VENV:-$T/ardupilot/venv}"
export PATH="$AP/Tools/autotest:$PATH"
[ -d "$VENV/bin" ] && export PATH="$VENV/bin:$PATH"
echo "tree:    $AP"
echo "version: $(grep -hoE 'THISFIRMWARE +"[^"]+"' "$VF" 2>/dev/null | sed 's/THISFIRMWARE //')  ($(git -C "$AP" rev-parse --short HEAD 2>/dev/null))"
cd "$AP" || exit 1
exec Tools/autotest/sim_vehicle.py -v "$VEHICLE" "$@"
