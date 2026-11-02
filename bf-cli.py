#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Joe Fernandez
"""Drive a Betaflight flight controller's CLI headlessly, with backups and pins.

Why. Betaflight's CLI is a serial text prompt meant for a person at a
terminal. Pasting a diff into it by hand is how a config meant for one board
or firmware lands on another, and how a half-applied restore gets saved. This
tool does the same thing a line at a time, waits for the prompt after every
line, stops on the CLI's error marker, and never saves unless asked to.

  identify   read-only: firmware version, board (target), MCU, craft name, status
             (on firmware before 2025.12 every command ends in a reboot:
             `exit` always reboots there)
  backup     read-only, same caveat: `diff all` (and with --dump `dump all`) to a file whose
             first lines are '#' comments naming firmware, board, MCU, craft
             name, UTC time, port and tool. A firmware `###ERROR` line in the
             output (e.g. CORRUPTED CONFIG) does not stop it: the line is kept
             as a '# bf-cli: firmware reported:' comment, the header gets a
             '# warning:' line, and the exit status is 1 (file still written)
  run        send the given CLI lines; each `set NAME = V` is read back with
             `get NAME` and must hold V; saving only with --save
  restore    put a backup file back, refusing unless the FC's board and
             firmware version equal both --expect-* and the file's own header

Save rules (Betaflight 2026.6.2, src/main/cli/cli.c):
  `exit`           leaves WITHOUT saving and reboots, discarding RAM changes
  `exit noreboot`  leaves without saving or rebooting
  `save noreboot`  writes EEPROM, no reboot
  `defaults nosave` resets RAM to defaults, no write, no reboot
`run` and `restore` end in `save noreboot` only with --save and only when no
line produced the error marker; otherwise they end in `exit`, so the reboot
throws the RAM changes away.
Firmware before 2025.12 (Betaflight 4.x: cli.c cliExit and cliSave ignore their
arguments) has neither noreboot form: `exit` and `save` always reboot. Read
from the `version` line (unparseable counts as old), the tool then leaves with
plain `exit` ("note: firmware V reboots on exit"), and --save sends plain `save`,
takes `saving` plus the link dropping as success, waits up to 20 s for the same
port to return, and verifies by reading the values back.
A board already in CLI mode (left so by an earlier raw session) prints no prompt
for `#`; after 1.5 s the tool sends a newline, which does, and notes it.
Do not restore a diff taken on one firmware version onto another (the
Betaflight CLI guide says the same). `restore` refuses unless the FC's board
and firmware version match --expect-board and --expect-version and the file's
own header, and it first writes a backup of the current config into the given
file's directory.

Usage:
  bf-cli.py identify
  bf-cli.py --port /dev/ttyACM0 backup -o ~/backups --dump
  bf-cli.py --port socket://127.0.0.1:5761 run "get craft_name"
  bf-cli.py run "set craft_name = MYQUAD" --save
  bf-cli.py restore ~/backups/MYQUAD_2026.6.2_20261018T120000Z.diff \\
      --expect-board STM32H743 --expect-version 2026.6.2 --save

Exit codes: 0 ok; 1 refused, read-back or verify mismatch, CLI error, or a
backup written with firmware warnings; 2 could not connect, link lost, or
timed out waiting for the prompt.

`run` refuses, before connecting, any line whose first word is save, defaults,
bl, dfu, exit, flash_erase, msc, motor, escprog, serialpassthrough or
gpspassthrough (they save, reboot, wipe, change mode, spin a motor or hand the
link to an ESC or GPS), and any argument holding a newline. The tool never
saves unless --save is given and never flashes. Needs pyserial;
`socket://host:port` works as a port (Betaflight SITL serves the CLI on TCP
5761, UART1).
"""
import argparse, difflib, glob, os, re, sys, time

try:
    import serial
except ImportError:
    sys.exit("pyserial not found. Try: python3 -m pip install 'pyserial>=3.5' (or apt python3-serial)")

TOOL = 'bf-cli.py'
PROMPT = b'\r\n# '
ERROR_MARK = '###ERROR'
# Commands `run` will not send: they save, reboot, wipe, change mode, drive a
# motor or hand the link over (Betaflight 2026.6.2 cli.c: motor, escprog,
# serialpassthrough, gpspassthrough).
FORBIDDEN = {'save', 'defaults', 'bl', 'dfu', 'exit', 'flash_erase', 'msc',
             'motor', 'escprog', 'serialpassthrough', 'gpspassthrough'}
# Lines of a diff restore does not send, with the reason (reported).
SKIP_REASON = {
    'batch': 'batch start/end: restore does its own error tracking',
    'defaults': 'defaults nosave: restore runs it itself first',
    'save': 'save: restore saves only with --save, after validating',
    'mcu_id': 'mcu_id: read-only chip identity, not configuration',
    'signature': 'signature: board signature, not configuration',
}


class Refused(Exception):
    pass


class NoLink(Exception):
    pass


class PromptTimeout(NoLink):
    pass


NOREBOOT_FROM = (2025, 12)  # first date version with `exit noreboot` / `save noreboot`


def noreboot_support(version):
    """(True if the firmware has the noreboot forms, True if the version parsed)."""
    m = re.match(r'(\d+)\.(\d+)', version or '')
    if not m:
        return False, False
    major, minor = int(m.group(1)), int(m.group(2))
    return (major, minor) >= NOREBOOT_FROM, True


def pick_port(arg):
    if arg:
        return arg, 'given'
    for p in sorted(glob.glob('/dev/serial/by-id/*')):
        if 'betaflight' in os.path.basename(p).lower():
            return p, 'auto: /dev/serial/by-id name contains Betaflight'
    return '/dev/ttyACM0', 'default: no Betaflight by-id entry'


class CLI:
    def __init__(self, port, baud, timeout, ser=None):
        self.timeout = timeout
        self.noreboot = True  # set from the version line once known
        if ser is not None:  # tests hand in a fake serial object
            self.s = ser
        else:
            try:
                self.s = serial.serial_for_url(port, baudrate=baud, timeout=0.1)
            except (serial.SerialException, OSError) as e:
                raise NoLink(f"cannot open {port}: {e}")
        self.s.reset_input_buffer() if hasattr(self.s, 'reset_input_buffer') else None
        self.enter()

    def _read_until_prompt(self, what, timeout=None):
        end = time.time() + (timeout or self.timeout)
        buf, last = b'', time.time()
        while time.time() < end:
            try:
                chunk = self.s.read(4096)
            except (serial.SerialException, OSError) as e:
                raise NoLink(f"link lost while waiting for {what}: {e}")
            if chunk:
                buf += chunk
                last = time.time()
            elif buf.endswith(PROMPT) and time.time() - last > 0.25:
                return buf.decode('latin-1')
        raise PromptTimeout(f"timed out after {timeout or self.timeout:.0f}s waiting for the prompt after {what}")

    def enter(self):
        try:
            self.s.write(b'#')
        except (serial.SerialException, OSError) as e:
            raise NoLink(f"cannot write: {e}")
        total = min(self.timeout, 10)
        quick = min(1.5, total)
        try:
            self._read_until_prompt('entering the CLI with #', quick)
            return
        except PromptTimeout:
            pass
        # Already in CLI mode, `#` is only a typed character and prints no prompt.
        # A newline submits it as a comment line (no error marker) and does.
        try:
            self.s.write(b'\r\n')
        except (serial.SerialException, OSError) as e:
            raise NoLink(f"cannot write: {e}")
        self._read_until_prompt('entering the CLI with # and a newline', max(total - quick, 1))
        print("note: board was already in CLI mode")

    def save_and_reboot(self, wait=15.0):
        """Plain `save` (firmware before 2025.12): it prints `saving`, then the board
        reboots and the link drops. True if `saving` was seen or the link dropped."""
        try:
            self.s.write(b'save\n')
        except (serial.SerialException, OSError) as e:
            raise NoLink(f"cannot write 'save': {e}")
        buf, end, dropped, seen = b'', time.time() + wait, False, None
        while time.time() < end:
            try:
                chunk = self.s.read(4096)
            except (serial.SerialException, OSError):
                dropped = True
                break
            buf += chunk or b''
            if seen is None and b'saving' in buf:
                seen = time.time()
            if seen is not None and time.time() - seen > 3:
                break  # saved, the link is still up; the reconnect copes either way
        text = buf.decode('latin-1')
        if ERROR_MARK in text:
            return False, text
        return (b'saving' in buf or dropped), text

    def cmd(self, line, timeout=None):
        """Output of one CLI line: echo and trailing prompt stripped."""
        try:
            self.s.write(line.encode('ascii') + b'\n')
        except (serial.SerialException, OSError, UnicodeEncodeError) as e:
            raise NoLink(f"cannot write {line!r}: {e}")
        raw = self._read_until_prompt(repr(line), timeout)
        text = raw.replace('\r\n', '\n').replace('\r', '\n')
        lines = text.split('\n')
        if lines and lines[0].strip() == line.strip():
            lines = lines[1:]
        while lines and lines[-1].strip() in ('', '#'):
            lines.pop()
        while lines and lines[0].strip() == '':
            lines.pop(0)
        return '\n'.join(lines)

    def leave(self, reboot):
        """`exit` (reboot, discards RAM changes) or `exit noreboot` (plain `exit` on
        firmware that has no such form: it reboots, there is no other way out)."""
        try:
            self.s.write(b'exit noreboot\n' if not reboot and self.noreboot else b'exit\n')
            time.sleep(0.5)
            self.s.read(4096)
        except (serial.SerialException, OSError):
            pass  # a reboot drops the link; that is the point of it
        try:
            self.s.close()
        except Exception:
            pass


def reconnect(port, baud, timeout, wait=20.0, sleep=time.sleep):
    """A new CLI on the same port once the rebooted board is back, or NoLink."""
    end, last = time.time() + wait, None
    sleep(1.0)  # let the old session actually go down first
    while time.time() < end:
        try:
            return CLI(port, baud, timeout)
        except NoLink as e:
            last = e
            sleep(0.5)
    raise NoLink(f"{port} did not come back within {wait:.0f}s after the save: {last}")


def save_old(cli, port, a, ver):
    """Plain `save` on firmware without `save noreboot`, then a fresh CLI on the
    same port after the reboot. None if the save did not report success."""
    print("\n> save")
    ok, text = cli.save_and_reboot()
    print(text.strip())
    if not ok:
        print("save did not report 'saving' and the link stayed up; nothing verified")
        return None
    print(f"saved; board rebooting (firmware {ver} has no save noreboot)")
    try:
        cli.s.close()
    except Exception:
        pass
    new = reconnect(port, a.baud, a.timeout)
    new.noreboot = False
    return new


def adopt_firmware(cli, ident):
    """Record whether this firmware has the noreboot forms; say so once if not."""
    cli.noreboot, known = noreboot_support(ident['version'])
    if not cli.noreboot:
        if known:
            print(f"note: firmware {ident['version']} reboots on exit")
        else:
            print(f"note: cannot read a version from {ident['version']!r}; treating the firmware "
                  f"as older than 2025.12, so it reboots on exit")


VERSION_RE = re.compile(r'Betaflight / (\S+) \((\S+)\) (\S+) .*?\((\w+)\) MSP API: (\S+)')


def identity(cli):
    out = cli.cmd('version')
    m = VERSION_RE.search(out)
    if not m:
        raise Refused(f"cannot parse `version` output: {out!r}")
    ident = {'board': m.group(1), 'board_id': m.group(2), 'version': m.group(3),
             'git': m.group(4), 'msp': m.group(5), 'version_line': m.group(0)}
    status = cli.cmd('status')
    ident['status'] = status
    mcu = ''
    for ln in status.split('\n'):
        if ln.startswith('MCU'):
            mcu = re.split(r'\s+CLK|,\s*clock', ln, 1)[0].replace('MCU:', '').replace('MCU', '').strip()
            break
    ident['mcu'] = mcu or 'unknown'
    craft = ''
    for ln in cli.cmd('get craft_name').split('\n'):
        m2 = re.match(r'craft_name = (.*)$', ln)
        if m2:
            craft = m2.group(1).strip()
            break
    ident['craft'] = '' if craft == '-' else craft
    bn = ''
    for ln in cli.cmd('board_name').split('\n'):
        if ln.startswith('board_name '):
            bn = ln[len('board_name '):].strip()
    ident['board_name'] = bn
    return ident


SET_RE = re.compile(r'^set\s+(\S+?)\s*=\s*(.*?)\s*$', re.I)


def get_value(cli, name):
    """The value `get NAME` prints (exact-name line), or None if there is none."""
    for ln in cli.cmd(f'get {name}').split('\n'):
        m = re.match(re.escape(name) + r' = (.*)$', ln, re.I)
        if m:
            return m.group(1).strip()
    return None


def same_value(target, got):
    """Equal as the firmware prints it: case and whitespace folded, numbers by value."""
    norm = lambda s: ' '.join(s.split()).lower()
    if norm(target) == norm(got):
        return True
    ts, gs = target.split(','), got.split(',')
    try:
        return len(ts) == len(gs) and all(float(x) == float(y) for x, y in zip(ts, gs))
    except ValueError:
        return False


def utc_stamp():
    return time.strftime('%Y%m%dT%H%M%SZ', time.gmtime())


def header(ident, port, what, warnings=()):
    return [f"# {TOOL} {what}",
            f"# firmware: {ident['version']}",
            f"# board: {ident['board']}",
            f"# mcu: {ident['mcu']}",
            f"# craft_name: {ident['craft'] or '-'}",
            f"# utc: {time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())}",
            f"# port: {port}"] + [f"# warning: firmware reported: {w}" for w in warnings]


def safe(s):
    return re.sub(r'[^A-Za-z0-9._-]+', '_', s).strip('_') or 'x'


def file_stem(ident, stamp):
    """Backup file name without extension. A craft name safe() reduces to nothing
    or to the filler `x` (e.g. an OSD mark like `< >`) is no name: use the board
    name, then the target. The header still records the craft name verbatim."""
    craft = safe(ident['craft']) if ident['craft'] else ''
    name = craft if craft not in ('', 'x') else (ident.get('board_name') or ident['board'])
    return f"{safe(name)}_{safe(ident['version'])}_{stamp}"


def flag_errors(out):
    """(text, errors): every firmware `###ERROR ...###` line kept in place but
    turned into a comment, so a later restore never sends it; errors is the list
    of error texts (the line without its ### marks)."""
    lines, errors = [], []
    for ln in out.split('\n'):
        if ERROR_MARK in ln:
            errors.append(ln.strip().strip('#').strip())
            ln = f"# bf-cli: firmware reported: {ln.strip()}"
        lines.append(ln)
    return '\n'.join(lines), errors


def stable_cmd(cli, line, tries=5):
    """Run a long read-only command until two consecutive outputs are identical.

    Betaflight SITL's TCP serial was measured to garble a long `diff all` about
    1 time in 25 (dropped bytes, stray characters). A saved backup or a
    verification must not carry that, so a long read is accepted only when
    read twice with the same result.
    """
    prev = None
    for _ in range(tries):
        out = cli.cmd(line, timeout=max(cli.timeout, 120))
        if out == prev:
            return out
        prev = out
    raise Refused(f"`{line}` gave a different answer on each of {tries} reads; not trusting it")


def do_backup(cli, ident, port, outdir, dump):
    """Write the diff (and dump) files; return (paths, firmware error texts).
    Firmware error lines do not stop the backup: they are commented in place and
    named in the header."""
    os.makedirs(outdir, exist_ok=True)
    base = file_stem(ident, utc_stamp())
    n = 0
    while os.path.exists(os.path.join(outdir, base + '.diff')):
        n += 1
        base = f"{base.rsplit('_dup', 1)[0]}_dup{n}"
    paths, warnings = [], []
    for cmdline, ext in [('diff all', '.diff')] + ([('dump all', '.dump')] if dump else []):
        out = stable_cmd(cli, cmdline)
        out, errs = flag_errors(out)
        warnings += errs
        if 'save' not in out.split('\n')[-3:] and ext == '.diff':
            raise Refused("`diff all` output did not end with `save`: truncated, not writing it")
        p = os.path.join(outdir, base + ext)
        with open(p, 'w') as fh:
            fh.write('\n'.join(header(ident, port, cmdline, errs)) + '\n\n' + out + '\n')
        paths.append(p)
    return paths, warnings


def print_warnings(warnings):
    print("WRITTEN WITH FIRMWARE WARNINGS:")
    for w in warnings:
        print(f"  {w}")


def read_file_header(path):
    """(header dict, embedded `# Betaflight / ...` dict or None, lines)."""
    with open(path) as fh:
        lines = fh.read().splitlines()
    hdr = {}
    for ln in lines:
        m = re.match(r'# (firmware|board|mcu|craft_name): (.*)$', ln)
        if m:
            hdr.setdefault(m.group(1), m.group(2).strip())
        elif ln.strip() and not ln.startswith('#'):
            break
    emb = None
    for ln in lines:
        m = VERSION_RE.search(ln) if ln.startswith('# Betaflight /') else None
        if m:
            emb = {'board': m.group(1), 'version': m.group(3)}
            break
    return hdr, emb, lines


def first_word(line):
    w = line.strip().split(None, 1)
    return w[0].lower() if w else ''


def restore_lines(lines):
    """(lines to send, {reason: count} skipped). Refuses a file with a dangerous line."""
    send, skipped = [], {}
    for ln in lines:
        s = ln.strip()
        if not s or s.startswith('#'):
            continue
        w = first_word(s)
        if w in SKIP_REASON:
            if w == 'defaults' and s.lower() != 'defaults nosave':
                raise Refused(f"file contains `{s}`, not the plain `defaults nosave`")
            if w == 'save' and s.lower() != 'save':
                raise Refused(f"file contains `{s}`")
            skipped[SKIP_REASON[w]] = skipped.get(SKIP_REASON[w], 0) + 1
        elif w in FORBIDDEN:
            raise Refused(f"file contains a `{w}` line, which restore will not send: {s!r}")
        else:
            send.append(s)
    return send, skipped


def normalized(text):
    """Comparable config lines of a diff: what restore_lines would send."""
    return restore_lines(text.split('\n'))


def connect(a):
    port, how = pick_port(a.port)
    print(f"port: {port} ({how}), {a.baud} baud")
    cli = CLI(port, a.baud, a.timeout)
    return cli, port


def show_ident(ident):
    print(f"firmware: Betaflight {ident['version']} (git {ident['git']}, MSP API {ident['msp']})")
    print(f"board:    {ident['board']} (id {ident['board_id']}, board_name {ident['board_name'] or '-'})")
    print(f"mcu:      {ident['mcu']}")
    print(f"craft:    {ident['craft'] or '-'}")


def cmd_identify(a):
    cli, port = connect(a)
    try:
        ident = identity(cli)
        adopt_firmware(cli, ident)
        show_ident(ident)
        print("\nstatus:\n" + ident['status'])
        return 0
    finally:
        cli.leave(False)


def cmd_backup(a):
    cli, port = connect(a)
    try:
        ident = identity(cli)
        adopt_firmware(cli, ident)
        show_ident(ident)
        paths, warnings = do_backup(cli, ident, port, a.output, a.dump)
        for p in paths:
            print(f"wrote {p}")
        if warnings:
            print_warnings(warnings)
            return 1
        return 0
    finally:
        cli.leave(False)


def cmd_run(a):
    for c in a.commands:
        if '\n' in c or '\r' in c:
            print(f"REFUSED: one CLI line per argument, got a newline in {c!r}")
            return 1
        if first_word(c) in FORBIDDEN:
            print(f"REFUSED: `{first_word(c)}` is not sent by run (saving is --save only). "
                  f"Nothing was sent.")
            return 1
    cli, port = connect(a)
    errors, reboot, sets = [], True, []
    try:
        ident = identity(cli)
        adopt_firmware(cli, ident)
        show_ident(ident)
        for c in a.commands:
            print(f"\n> {c}")
            sm = SET_RE.match(c.strip())
            before = get_value(cli, sm.group(1)) if sm else None
            out = cli.cmd(c)
            print(out)
            if ERROR_MARK in out:
                errors.append(c)
                break
            if sm and before is not None:
                name, target = sm.group(1), sm.group(2)
                sets.append((name, target))
                after = get_value(cli, name)
                print(f"{name}: {before} -> {target} -> {after}")
                if after is None or not same_value(target, after):
                    print(f"READBACK MISMATCH: sent {target!r}, the FC now holds {after!r}")
                    errors.append(c)
                    break
        if errors:
            print(f"\nERROR (CLI error marker or read-back mismatch) on: {errors[0]}. Nothing saved; ending with `exit` (reboot "
                  f"discards RAM changes).")
            return 1
        if a.save:
            if not cli.noreboot:
                new = save_old(cli, port, a, ident['version'])
                if new is None:
                    return 1
                cli = new
                bad = []
                for name, target in sets:
                    got = get_value(cli, name)
                    print(f"after reboot: {name} = {got}")
                    if got is None or not same_value(target, got):
                        bad.append(f"{name}: wanted {target!r}, the FC holds {got!r}")
                if bad:
                    print("VERIFY MISMATCH after the save: " + '; '.join(bad))
                    return 1
                print(f"verified: {len(sets)} set values hold after the reboot.")
                return 0
            print("\n> save noreboot")
            out = cli.cmd('save noreboot')
            print(out)
            if ERROR_MARK in out or 'saving' not in out:
                print("save did not report 'saving'; ending with `exit`")
                return 1
            reboot = False
            print("saved (no reboot).")
        else:
            print("\nnot saved: ending with `exit`, the FC reboots and RAM changes are discarded "
                  "(use --save to keep them).")
        return 0
    finally:
        cli.leave(reboot)


def cmd_restore(a):
    hdr, emb, flines = read_file_header(a.file)
    problems = []
    if 'board' not in hdr or 'firmware' not in hdr:
        problems.append("file has no bf-cli.py header (firmware/board), so it cannot be pinned")
    else:
        if hdr['board'] != a.expect_board:
            problems.append(f"file header board {hdr['board']!r} != --expect-board {a.expect_board!r}")
        if hdr['firmware'] != a.expect_version:
            problems.append(f"file header firmware {hdr['firmware']!r} != --expect-version {a.expect_version!r}")
    if emb and 'board' in hdr and (emb['board'], emb['version']) != (hdr['board'], hdr['firmware']):
        problems.append(f"file's own version line says {emb['board']} {emb['version']}, header says "
                        f"{hdr['board']} {hdr['firmware']}")
    if problems:
        print("REFUSED: " + '; '.join(problems) + ". Nothing was connected to.")
        return 1
    try:
        send, skipped = restore_lines(flines)
    except Refused as e:
        print(f"REFUSED: {e}. Nothing was connected to.")
        return 1
    if not send:
        print("REFUSED: the file holds no configuration lines")
        return 1

    cli, port = connect(a)
    reboot = True
    try:
        ident = identity(cli)
        adopt_firmware(cli, ident)
        show_ident(ident)
        bad = []
        if ident['board'] != a.expect_board:
            bad.append(f"FC board {ident['board']!r} != --expect-board {a.expect_board!r}")
        if ident['version'] != a.expect_version:
            bad.append(f"FC firmware {ident['version']!r} != --expect-version {a.expect_version!r}")
        if bad:
            print("REFUSED, wrong flight controller: " + '; '.join(bad) + ". Nothing was written.")
            return 1
        before, warnings = do_backup(cli, ident, port, os.path.dirname(os.path.abspath(a.file)), False)
        print(f"current config backed up first: {before[0]}")
        if warnings:
            print_warnings(warnings)

        print("\n> defaults nosave")
        out = cli.cmd('defaults nosave')
        if ERROR_MARK in out or 'resetting to defaults' not in out:
            print(out)
            print("defaults nosave did not report success; nothing applied; ending with `exit`")
            return 1
        failing = []
        for ln in send:
            out = cli.cmd(ln)
            if ERROR_MARK in out:
                failing.append((ln, out.strip()))
        print(f"sent {len(send)} lines from {a.file}, {len(failing)} with errors")
        for key, n in skipped.items():
            print(f"not sent ({n}x): {key}")
        if failing:
            print("\nFAILING LINES:")
            for ln, out in failing:
                print(f"  {ln}\n    -> {out}")
            print("\nnothing saved; ending with `exit` (reboot discards everything).")
            return 1
        if not a.save:
            print("\nvalidated, nothing saved. Ending with `exit` (reboot discards the RAM changes).")
            return 0
        if not cli.noreboot:
            new = save_old(cli, port, a, ident['version'])
            if new is None:
                return 1
            cli = new
            print("\nVerifying with `diff all` against the file:")
        else:
            out = cli.cmd('save noreboot')
            if ERROR_MARK in out or 'saving' not in out:
                print(out)
                print("save did not report 'saving'; ending with `exit`")
                return 1
            reboot = False
            print("\nsaved (no reboot). Verifying with `diff all` against the file:")
        after, _ = normalized(stable_cmd(cli, 'diff all'))
        if after == send:
            print(f"verified: {len(send)} configuration lines match the file.")
            return 0
        print("MISMATCH between the FC's `diff all` and the file (- file, + FC):")
        for d in difflib.unified_diff(send, after, 'file', 'FC', n=0, lineterm=''):
            print(d)
        return 1
    finally:
        cli.leave(reboot)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', help='serial device or socket://host:port (default: the first '
                    '/dev/serial/by-id entry containing Betaflight, else /dev/ttyACM0)')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--timeout', type=float, default=30.0,
                    help='seconds to wait for the prompt after one line (diff/dump get at least 120)')
    sub = ap.add_subparsers(dest='cmd', required=True)
    sub.add_parser('identify', help='read-only')
    b = sub.add_parser('backup', help='read-only')
    b.add_argument('-o', '--output', required=True, help='directory for the files')
    b.add_argument('--dump', action='store_true', help='also write `dump all` to a .dump')
    r = sub.add_parser('run', help='send CLI lines; saves only with --save')
    r.add_argument('commands', nargs='+')
    r.add_argument('--save', action='store_true', help='end with `save noreboot` (default: `exit`, which discards)')
    s = sub.add_parser('restore', help='put a backup file back, pinned')
    s.add_argument('file')
    s.add_argument('--expect-board', required=True)
    s.add_argument('--expect-version', required=True)
    s.add_argument('--save', action='store_true', help='save noreboot and verify (default: validate only)')
    a = ap.parse_args()
    try:
        return {'identify': cmd_identify, 'backup': cmd_backup, 'run': cmd_run,
                'restore': cmd_restore}[a.cmd](a)
    except Refused as e:
        print(f"REFUSED: {e}")
        return 1
    except NoLink as e:
        print(f"NO LINK: {e}")
        return 2


if __name__ == '__main__':
    sys.exit(main())
