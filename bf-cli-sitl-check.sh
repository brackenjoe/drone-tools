#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Joe Fernandez
# Offline check of bf-cli.py against Betaflight SITL (no hardware and no flight
# simulator: Betaflight SITL serves the CLI on TCP 5761 by itself, UART1). Its
# eeprom.bin lands in a scratch dir under /tmp, never in the repo.
#
# Needs: bash (uses /dev/tcp), python3 with pyserial, setsid, pgrep.
# Optional: shellcheck (without it that check prints SKIP and is not counted).
# TCP 5761 must be free; if something else holds it the script exits 2.
#
# Build the SITL once:  clone Betaflight at tag 2026.6.2, then `make TARGET=SITL`.
# Run:                  BF_SITL=/path/to/betaflight_SITL.elf bf-cli-sitl-check.sh
#                       (BF_SITL unset: ./betaflight/obj/main/betaflight_SITL.elf beside this script)
# 92 checks (91 when shellcheck is not installed: that one is skipped, not
# counted). Exit status is the number of failed checks (0 = all passed), or 2
# if SITL could not be started. The SITL this script started is killed on exit.
set -u

here=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
tool="$here/bf-cli.py"
sitl="${BF_SITL:-$here/betaflight/obj/main/betaflight_SITL.elf}"
# start_sitl runs it after cd to a scratch dir, so a relative path must be resolved now
if [ -e "$sitl" ]; then sitl="$(cd "$(dirname "$sitl")" && pwd)/$(basename "$sitl")"; fi
port="socket://127.0.0.1:5761"
want_version="2026.6.2"

[ -x "$sitl" ] || { echo "SITL not built at $sitl: clone Betaflight at tag 2026.6.2, run make TARGET=SITL, or set BF_SITL to the betaflight_SITL.elf" >&2; exit 2; }
if (exec 3<>/dev/tcp/127.0.0.1/5761) 2>/dev/null; then
    echo "something else holds TCP 5761 (the Betaflight SITL CLI port); stop it and rerun" >&2
    exit 2
fi
work=$(mktemp -d /tmp/bf-cli-check.XXXXXX)
pass=0
fail=0
sitl_pid=

stop_sitl() {
    if [ -n "$sitl_pid" ]; then
        kill "$sitl_pid" 2>/dev/null
        wait "$sitl_pid" 2>/dev/null
        sitl_pid=
    fi
    # the port is free again before the next start (a quitting SITL may linger a moment)
    for _ in $(seq 1 50); do
        (exec 3<>/dev/tcp/127.0.0.1/5761) 2>/dev/null || break
        sleep 0.1
    done
}
start_sitl() {
    stop_sitl
    (cd "$work" && exec setsid "$sitl" >>"$work/sitl.log" 2>&1) &
    sitl_pid=$!
    for _ in $(seq 1 100); do
        (exec 3<>/dev/tcp/127.0.0.1/5761) 2>/dev/null && return 0
        sleep 0.1
    done
    echo "SITL did not open TCP 5761" >&2
    return 1
}
cleanup() { stop_sitl; rm -rf "$work"; }
trap cleanup EXIT

bf() { python3 "$tool" --port "$port" "$@"; }
ok() { pass=$((pass + 1)); echo "PASS  $1"; }
bad() { fail=$((fail + 1)); echo "FAIL  $1"; [ -n "${2:-}" ] && echo "$2" | sed 's/^/        /'; }
# check NAME EXPECTED_EXIT CMD...  (output in $out, exit in $rc)
run_expect() {
    local name=$1 want=$2
    shift 2
    out=$("$@" 2>&1)
    rc=$?
    if [ "$rc" -eq "$want" ]; then ok "$name (exit $rc)"; else bad "$name: exit $rc, wanted $want" "$out"; fi
}
has() { # has NAME PATTERN : $out contains PATTERN
    if grep -q -- "$2" <<<"$out"; then ok "$1"; else bad "$1: no match for '$2'" "$out"; fi
}
# the config of a backup file, header and timestamps removed
body() { grep -v -E '^# (bf-cli.py|firmware|board|mcu|craft_name|utc|port)[: ]' "$1"; }
snapshot() { # snapshot DIR : backup into DIR, echo the .diff path
    bf backup -o "$1" >/dev/null 2>&1 && ls "$1"/*.diff | tail -n 1
}

start_sitl || exit 2
echo "scratch: $work"

# 1 identify
run_expect "identify" 0 bf identify
has "identify prints $want_version" "firmware: Betaflight $want_version"
has "identify prints the SITL board" "board:    SITL"
has "identify prints an MCU" "mcu:      SIMULATOR"

# 2 run without --save: reboots, not kept
run_expect "run set craft_name unsaved" 0 bf run "set craft_name = BFTEST"
has "unsaved run says so" "not saved"
start_sitl
out=$(bf run "get craft_name" 2>&1)
if grep -q 'craft_name = BFTEST' <<<"$out"; then bad "unsaved craft_name survived the reboot" "$out"; else ok "unsaved craft_name gone after restart"; fi
start_sitl

# 3 run with --save: persists
run_expect "run set craft_name --save" 0 bf run "set craft_name = BFTEST" --save
start_sitl
out=$(bf identify 2>&1)
has "saved craft_name BFTEST after restart (eeprom.bin persisted)" "craft:    BFTEST"
[ -s "$work/eeprom.bin" ] && ok "eeprom.bin is in the scratch dir" || bad "no eeprom.bin in scratch dir"

# 4 backup
mkdir "$work/b1"
run_expect "backup --dump" 0 bf backup -o "$work/b1" --dump
good=$(ls "$work"/b1/*.diff 2>/dev/null | head -n 1)
if [ -n "$good" ]; then
    ok "diff file exists: $(basename "$good")"
    head -n 7 "$good" | grep -q "^# firmware: $want_version" && head -n 7 "$good" | grep -q '^# board: SITL' \
        && head -n 7 "$good" | grep -q '^# port: ' && ok "header present" || bad "header missing" "$(head -n 8 "$good")"
    grep -q '^set craft_name = BFTEST' "$good" && ok "backup contains set craft_name = BFTEST" || bad "backup lacks set craft_name = BFTEST"
else
    bad "no .diff written"
fi
ls "$work"/b1/*.dump >/dev/null 2>&1 && ok ".dump written" || bad "no .dump written"

# 5 run refusals and errors
for c in save defaults bl dfu exit flash_erase msc; do
    out=$(bf run "$c" 2>&1)
    rc=$?
    if [ "$rc" -eq 1 ] && grep -q REFUSED <<<"$out" && ! grep -q '^port:' <<<"$out"; then
        ok "run $c refused before connecting"
    else bad "run $c: exit $rc" "$out"; fi
done
run_expect "run bad parameter" 1 bf run "set nonexistent_param = 1"
has "bad parameter reports the error marker" '###ERROR'

# 5b set read-back (the failed run above ended in `exit`, so SITL is down)
start_sitl
run_expect "run set int" 0 bf run "set motor_poles = 15"
has "int prints before -> target -> after" "motor_poles: 14 -> 15 -> 15"
start_sitl
run_expect "run set enum, lower case" 0 bf run "set motor_pwm_protocol = dshot300"
has "enum read back in the firmware's case" "motor_pwm_protocol: PWM -> dshot300 -> DSHOT300"
start_sitl
run_expect "run set string" 0 bf run "set craft_name = a  b"
has "string read back" "craft_name: BFTEST -> a  b -> a  b"
start_sitl
# 2026.6.2 rejects out-of-range values, but accepts 12.5 for an integer variable and stores 12
run_expect "run set accepted but stored differently" 1 bf run "set rc_smoothing_auto_factor = 12.5" --save
has "mismatch reported" "READBACK MISMATCH"
has "mismatch does not save" "Nothing saved"
start_sitl
out=$(bf run "get rc_smoothing_auto_factor" 2>&1)
has "mismatched set not saved after restart" "rc_smoothing_auto_factor = 30"
start_sitl

# 6 wrong-version restore leaves config unchanged
start_sitl
mkdir "$work/s1" "$work/s2"
before=$(snapshot "$work/s1")
run_expect "restore wrong --expect-version" 1 bf restore "$good" --expect-board SITL --expect-version 2025.12.0 --save
has "wrong version refused" REFUSED
run_expect "restore wrong --expect-board" 1 bf restore "$good" --expect-board NOPE --expect-version "$want_version" --save
after=$(snapshot "$work/s2")
if [ -n "$before" ] && [ -n "$after" ] && diff <(body "$before") <(body "$after") >/dev/null; then
    ok "config unchanged after refused restores"
else bad "config changed or snapshot failed" "$(diff <(body "$before") <(body "$after") 2>&1 | head)"; fi

# 7 restore with an injected bad line
cp "$good" "$work/badline.diff"
echo "set nonexistent_param = 1" >>"$work/badline.diff"
# make the live config differ from the file first, so a wrongly applied restore would show
bf run "set craft_name = OTHER" --save >/dev/null 2>&1
start_sitl
mkdir "$work/s3" "$work/s4"
before=$(snapshot "$work/s3")
run_expect "restore with a bad line" 1 bf restore "$work/badline.diff" --expect-board SITL --expect-version "$want_version" --save
has "bad line named" 'set nonexistent_param = 1'
start_sitl
after=$(snapshot "$work/s4")
if [ -n "$before" ] && [ -n "$after" ] && diff <(body "$before") <(body "$after") >/dev/null \
    && grep -q '^set craft_name = OTHER' "$after"; then
    ok "config unchanged after restart (still OTHER)"
else bad "config changed by the failed restore" "$(diff <(body "$before") <(body "$after"))"; fi

# 8 validate-only restore changes nothing
run_expect "restore without --save" 0 bf restore "$good" --expect-board SITL --expect-version "$want_version"
has "validated, nothing saved" "validated, nothing saved"
start_sitl
out=$(bf identify 2>&1)
has "still OTHER after validate-only restore" "craft:    OTHER"

# 9 good restore with --save
run_expect "restore --save" 0 bf restore "$good" --expect-board SITL --expect-version "$want_version" --save
has "verification diff matches" "verified:"
start_sitl
out=$(bf identify 2>&1)
has "BFTEST back after restore and restart" "craft:    BFTEST"

# 9b firmware error lines in a diff (offline, no SITL involved)
out=$(BF_TOOL="$tool" python3 - <<'PYEOF' 2>&1
import importlib.util, os, sys
sp = importlib.util.spec_from_file_location('bfcli', os.environ['BF_TOOL'])
m = importlib.util.module_from_spec(sp); sp.loader.exec_module(m)
fixture = """# Betaflight / SPEEDYBEEF405V4 (SBF4) 4.5.1 Oct 1 2024 / 10:00:00 (abc) MSP API: 1.46
set angle_p_gain = 125
set horizon_level_strength = 125
###ERROR IN diff: CORRUPTED CONFIG: horizon_level_strength = 125###
set d_min_roll = 46
save"""
text, errs = m.flag_errors(fixture)
ident = {'version': '4.5.1', 'board': 'SPEEDYBEEF405V4', 'mcu': 'STM32F4', 'craft': '< >', 'board_name': 'SPEEDYBEEF405V4'}
hdr = '\n'.join(m.header(ident, 'p', 'diff all', errs))
lines = text.split('\n')
def chk(name, cond):
    print(('PASS ' if cond else 'FAIL ') + name)
chk('error line commented in place', lines[3] == '# bf-cli: firmware reported: ###ERROR IN diff: CORRUPTED CONFIG: horizon_level_strength = 125###')
chk('set line above it untouched', lines[2] == 'set horizon_level_strength = 125' and lines[4] == 'set d_min_roll = 46')
chk('one error returned (backup exits 1 on a non-empty list)', errs == ['ERROR IN diff: CORRUPTED CONFIG: horizon_level_strength = 125'])
chk('header warning present', '# warning: firmware reported: ERROR IN diff: CORRUPTED CONFIG: horizon_level_strength = 125' in hdr)
chk('header keeps craft verbatim', '# craft_name: < >' in hdr)
chk('no error: text unchanged, no warnings', m.flag_errors('set a = 1\nsave') == ('set a = 1\nsave', []))
send, _ = m.restore_lines(text.split('\n'))
chk('restore never sends the comment, still sends the set line', send == ['set angle_p_gain = 125', 'set horizon_level_strength = 125', 'set d_min_roll = 46'])
chk('craft "< >" names the file after the board', m.file_stem(ident, 'T').startswith('SPEEDYBEEF405V4_4.5.1_T'))
chk('craft "MYQUAD" names the file after the craft', m.file_stem(dict(ident, craft='MYQUAD'), 'T').startswith('MYQUAD_'))
chk('empty craft and board_name falls to the target', m.file_stem(dict(ident, craft='', board_name=''), 'T').startswith('SPEEDYBEEF405V4_'))
PYEOF
)
echo "$out"
pass=$((pass + $(grep -c '^PASS ' <<<"$out")))
fail=$((fail + $(grep -c '^FAIL ' <<<"$out")))
grep -q Traceback <<<"$out" && bad "9b offline error-line test crashed" "$out"
[ "$(grep -c '^PASS ' <<<"$out")" -ge 10 ] || bad "9b offline error-line test ran fewer than 10 checks" "$out"

# 9c end to end: a real CORRUPTED CONFIG. 2026.6.2's MSP_SET_PID (msp.c) writes
# the bytes with no range check (the CLI's `set` has one), so set the LEVEL I term
# (horizon_level_strength, range 0-100) to 125 over MSP, in RAM only.
start_sitl
python3 - <<'PYEOF' || bad "9c MSP injection of horizon_level_strength = 125 failed"
import socket, time
def frame(cmd, data=b''):
    b = bytes([len(data), cmd]) + data
    c = 0
    for x in b: c ^= x
    return b'$M<' + b + bytes([c])
s = socket.create_connection(('127.0.0.1', 5761)); s.settimeout(1)
def rx():
    time.sleep(0.5)
    return s.recv(4096)
s.send(frame(112)); r = rx()           # MSP_PID: P, I, D for roll, pitch, yaw, level, mag
pid = bytearray(r[5:5 + 15]); assert len(pid) == 15 and r[:3] == b'$M>'
pid[10] = 125                          # index 3 (PID_LEVEL), I
s.send(frame(202, bytes(pid))); assert rx()[:3] == b'$M>'
PYEOF
mkdir "$work/c1" "$work/c2"
run_expect "backup of a corrupted config exits 1" 1 bf backup -o "$work/c1"
has "backup says written with firmware warnings" "WRITTEN WITH FIRMWARE WARNINGS:"
has "backup names the corrupted setting" "CORRUPTED CONFIG: horizon_level_strength = 125"
has "backup still prints the file path" "wrote $work/c1/"
cf=$(ls "$work"/c1/*.diff 2>/dev/null | head -n 1)
if [ -n "$cf" ] && grep -q '^set horizon_level_strength = 125' "$cf" \
    && grep -q '^# bf-cli: firmware reported: ###ERROR IN diff: CORRUPTED CONFIG: horizon_level_strength = 125###' "$cf" \
    && grep -q '^# warning: firmware reported: ERROR IN diff: CORRUPTED CONFIG' "$cf" \
    && ! grep -q '^###ERROR' "$cf"; then ok "corrupted-config file: set line kept, error commented, header warns"
else bad "corrupted-config file wrong or missing" "$(head -n 12 "${cf:-/dev/null}")"; fi
# restoring that file fails loudly on the bad set line and saves nothing
run_expect "restore of the corrupted-config file fails on the set line" 1 \
    bf restore "$cf" --expect-board SITL --expect-version "$want_version" --save
has "restore names the failing set line" "set horizon_level_strength = 125"
start_sitl

# 9d board already in CLI mode (enter() falls back to a newline). Measured: SITL's CLI mode belongs to the
# UART, not the TCP connection, so a socket closed without `exit` leaves it set.
start_sitl
python3 - <<'PYEOF' || bad "9d could not leave SITL in CLI mode"
import socket, time
s = socket.create_connection(('127.0.0.1', 5761)); s.settimeout(2)
s.send(b'#'); time.sleep(0.8)
assert s.recv(4096).endswith(b'\r\n# ')
s.close()
PYEOF
run_expect "identify on a board left in CLI mode" 0 bf identify
has "identify on a CLI-mode board says so" "note: board was already in CLI mode"
has "identify on a CLI-mode board still reads the version" "firmware: Betaflight $want_version"
out=$(bf identify 2>&1)
has "second identify (mode left by the first exit) reads the version" "firmware: Betaflight $want_version"
if grep -q 'already in CLI mode' <<<"$out"; then bad "normal path printed the CLI-mode note" "$out"; else ok "normal path prints no CLI-mode note"; fi
if python3 - <<'PYEOF'
import socket, time
s = socket.create_connection(('127.0.0.1', 5761)); s.settimeout(2)
s.send(b'#'); time.sleep(0.8); s.recv(4096)
s.send(b'#\r\n'); time.sleep(0.8)
r = s.recv(4096)
assert r.endswith(b'\r\n# ') and b'###ERROR' not in r, r
PYEOF
then ok "a lone # line is a comment: prompt back, no error marker"; else bad "a lone # line produced no prompt or an error marker"; fi
start_sitl

# 9e old firmware (Betaflight 4.x has no exit/save noreboot) and the CLI-mode
# fallback, offline against fake serial objects. SITL is 2026.6.2, so the old
# path cannot run there end to end; these do not touch SITL.
out=$(BF_TOOL="$tool" python3 - <<'PYEOF' 2>&1
import importlib.util, io, os, sys, time, argparse, contextlib
sp = importlib.util.spec_from_file_location('bfcli', os.environ['BF_TOOL'])
m = importlib.util.module_from_spec(sp); sp.loader.exec_module(m)
def chk(name, cond, detail=''):
    print(('PASS ' if cond else 'FAIL ') + name + ('' if cond else f'  [{detail}]'))
SE = m.serial.SerialException
for v, want in (('4.5.1', False), ('4.4.3', False), ('2025.12.0', True), ('2026.6.2', True),
                ('2025.11.9', False), ('2026.12.0', True)):
    chk(f'version gate {v} -> noreboot {want}', m.noreboot_support(v) == (want, True), m.noreboot_support(v))
chk('version gate: unparseable is old and flagged', m.noreboot_support('garbage') == (False, False))

class FakeFC:
    """A tiny Betaflight CLI. mode: starts in CLI or not; version; reboot_on_save."""
    store = {}
    def __init__(self, version='4.5.1', in_cli=False, answers_hash=True, live=None):
        self.version, self.in_cli, self.answers_hash = version, in_cli, answers_hash
        self.out, self.dead, self.ram, self.linebuf = b'', False, None, ''
        self.writes, self.live = [], live
    def reset_input_buffer(self): pass
    def close(self): pass
    def write(self, data):
        if self.dead: raise SE('device disconnected')
        self.writes.append(data)
        if not self.in_cli:
            if data == b'#' and self.answers_hash:
                self.in_cli = True; self.out += b"\r\nEntering CLI Mode\r\n\r\n# "
            return
        # in CLI mode '#' is a typed character: nothing happens until a newline
        self.linebuf += data.decode()
        while '\n' in self.linebuf.replace('\r', '\n'):
            ln, _, self.linebuf = self.linebuf.replace('\r', '\n').partition('\n')
            self.line(ln)
    def line(self, ln):
        if self.ram is None: self.ram = dict(FakeFC.store)
        o = ln.encode() + b'\r\n'
        if ln.startswith('version'):
            o += (f"# Betaflight / STM32F7X2 (S7X2) {self.version} Oct 10 2024 / 12:00:00 (abcdef0) MSP API: 1.46").encode()
        elif ln == 'status': o += b'MCU F722 Clock=216MHz'
        elif ln == 'board_name': o += b'board_name SPEEDYBEEF405V4'
        elif ln.startswith('get '):
            k = ln[4:]; o += f"{k} = {self.ram.get(k, 'OLD')}".encode()
        elif ln.startswith('set '):
            k, _, v = ln[4:].partition(' = '); self.ram[k] = v
        elif ln == 'save noreboot':
            FakeFC.store = dict(self.ram); o += b'saving'
        elif ln == 'save':
            FakeFC.store = dict(self.ram) if not getattr(FakeFC, 'lose', False) else FakeFC.store
            self.out += o + b'\r\n# saving\r\n'; self.pending_dead = True; return
        elif ln.startswith('exit'): self.in_cli = False; return
        self.out += o + b'\r\n\r\n# '
    def read(self, n):
        if self.dead: raise SE('device disconnected')
        if self.out:
            o, self.out = self.out, b''; return o
        if getattr(self, 'pending_dead', False): self.dead = True; raise SE('device disconnected')
        time.sleep(0.01); return b''

# CLI-mode fallback: board in CLI mode ignores '#', answers a newline
f = FakeFC(in_cli=True)
buf = io.StringIO()
with contextlib.redirect_stdout(buf):
    c = m.CLI('x', 115200, 10, ser=f)
chk('enter() falls back to a newline on a CLI-mode board', b'\r\n' in f.writes and 'already in CLI mode' in buf.getvalue(), f.writes)
f = FakeFC(in_cli=False)
buf = io.StringIO()
with contextlib.redirect_stdout(buf):
    c = m.CLI('x', 115200, 10, ser=f)
chk('enter() on a normal board sends no newline and prints nothing', f.writes == [b'#'] and buf.getvalue() == '', (f.writes, buf.getvalue()))
f = FakeFC(in_cli=False, answers_hash=False)
try:
    m.CLI('x', 115200, 4, ser=f); chk('enter() on a dead board raises', False)
except m.NoLink as e:
    chk('enter() on a silent board times out as NoLink', 'timed out' in str(e), e)

# leave() on old firmware sends plain exit
f = FakeFC('4.5.1'); c = m.CLI('x', 115200, 10, ser=f); c.noreboot = False; f.writes.clear(); c.leave(False)
chk('leave(False) on old firmware sends plain exit', f.writes == [b'exit\n'], f.writes)
f = FakeFC('2026.6.2'); c = m.CLI('x', 115200, 10, ser=f); f.writes.clear(); c.leave(False)
chk('leave(False) on new firmware sends exit noreboot', f.writes == [b'exit noreboot\n'], f.writes)

# save_and_reboot: '# saving' then SerialException on the next read
f = FakeFC('4.5.1'); c = m.CLI('x', 115200, 10, ser=f); c.noreboot = False
ok, text = c.save_and_reboot(wait=5)
chk('reboot-tolerant save: saving then link drop is success', ok and 'saving' in text, (ok, text))
class Mute(FakeFC):
    def line(self, ln):
        if ln == 'save': self.out += b'\r\n# '; return
        super().line(ln)
f = Mute('4.5.1'); c = m.CLI('x', 115200, 10, ser=f)
ok, text = c.save_and_reboot(wait=1)
chk('save with no saving and no drop is failure', not ok, (ok, text))

# end to end with the real commands, serial_for_url replaced by the fake
def run_cmd(argv, fcs):
    real = m.serial.serial_for_url
    it = iter(fcs)
    def fake_url(port, **kw):
        f = next(it)
        if isinstance(f, Exception): raise f
        return f
    m.serial.serial_for_url = fake_url
    real_sleep = m.time.sleep
    a = argparse.Namespace(port='/dev/fake', baud=115200, timeout=10, commands=argv, save=True)
    buf = io.StringIO()
    try:
        m.time.sleep = lambda s: real_sleep(min(s, 0.05))
        with contextlib.redirect_stdout(buf):
            try: rc = m.cmd_run(a)
            except m.NoLink as e: print(f"NO LINK: {e}"); rc = 2
    finally:
        m.serial.serial_for_url = real; m.time.sleep = real_sleep
    return rc, buf.getvalue()
FakeFC.store = {}; FakeFC.lose = False
rc, o = run_cmd(['set craft_name = BFTEST'], [FakeFC('4.5.1'), SE('gone'), SE('gone'), FakeFC('4.5.1', in_cli=False)])
chk('old firmware run --save: exit 0 after reboot and re-read', rc == 0 and 'verified: 1 set values' in o, (rc, o))
chk('old firmware run --save: prints the note and the saved line',
    'note: firmware 4.5.1 reboots on exit' in o and 'saved; board rebooting (firmware 4.5.1 has no save noreboot)' in o, o)
FakeFC.store = {}; FakeFC.lose = True
rc, o = run_cmd(['set craft_name = BFTEST'], [FakeFC('4.5.1'), FakeFC('4.5.1')])
chk('old firmware run --save: value not persisted is exit 1', rc == 1 and 'VERIFY MISMATCH' in o, (rc, o))
FakeFC.lose = False
m_reconn = m.reconnect
m.reconnect = lambda port, baud, timeout, wait=20.0, sleep=None: m_reconn(port, baud, timeout, wait=0.3, sleep=lambda s: time.sleep(0.05))
rc, o = run_cmd(['set craft_name = BFTEST'], [FakeFC('4.5.1')] + [SE('gone')] * 200)
m.reconnect = m_reconn
chk('old firmware run --save: port never returns is exit 2 with a clear message',
    rc == 2 and 'did not come back' in o, (rc, o))
rc, o = run_cmd(['set craft_name = BFTEST'], [FakeFC('2026.6.2')])
chk('new firmware run --save keeps save noreboot', rc == 0 and 'saved (no reboot).' in o and 'reboots on exit' not in o, (rc, o))
rc, o = run_cmd(['get craft_name'], [FakeFC('dev-build'), FakeFC('dev-build')])
chk('unparseable version: treated as old and said so', 'treating the firmware as older than 2025.12' in o, o)
PYEOF
)
echo "$out"
pass=$((pass + $(grep -c '^PASS ' <<<"$out")))
fail=$((fail + $(grep -c '^FAIL ' <<<"$out")))
grep -q Traceback <<<"$out" && bad "9e offline old-firmware test crashed" "$out"
[ "$(grep -c '^PASS ' <<<"$out")" -ge 20 ] || bad "9e offline old-firmware test ran fewer than 20 checks" "$out"

# 10 static checks
python3 -m py_compile "$tool" && ok "py_compile" || bad "py_compile"
rm -rf "$here/__pycache__/bf-cli.cpython-"*.pyc 2>/dev/null
if command -v shellcheck >/dev/null 2>&1; then
    shellcheck -S warning "$here/bf-cli-sitl-check.sh" && ok "shellcheck -S warning" || bad "shellcheck"
else
    echo "SKIP  shellcheck (not installed)"
fi

echo
echo "passed $pass, failed $fail"
exit "$fail"
