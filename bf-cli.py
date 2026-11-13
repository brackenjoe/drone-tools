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
  msp        read-only over MSP, never the CLI, never a reboot: why the board
             will not arm. Prints FC variant, version, board and craft name, the
             mode ranges, then --samples (default 10) lines --interval seconds
             (default 0.2) apart with the arming-disable flags, RSSI and RC
             channels, a summary, and which modes the last RC frame puts on

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
  bf-cli.py --port socket://127.0.0.1:5761 msp --samples 5 --interval 0.5
  bf-cli.py restore ~/backups/MYQUAD_2026.6.2_20261018T120000Z.diff \\
      --expect-board STM32H743 --expect-version 2026.6.2 --save

Exit codes: 0 ok; 1 refused, read-back or verify mismatch, CLI error, or a
backup written with firmware warnings; 2 could not connect, link lost, or
timed out waiting for the prompt.

`msp` is MSPv1 only (`$M<`, with the 255-byte jumbo form accepted in replies)
and sends only the read ids in MSP_READ_ONLY (API_VERSION, FC_VARIANT,
FC_VERSION, BOARD_INFO, NAME, RC, ANALOG, BOXNAMES, BOXIDS, MODE_RANGES,
STATUS_EX), with no payload; its one send function raises Refused for any other
id. It never writes `#`, `exit` or a line, because Betaflight 4.x `exit` reboots
the board and drops a USB/IP attach. A board in CLI mode ignores MSP: with no
reply to the first request it exits 2 and sends nothing more (power-cycle the
board, or run `identify`, which reboots a board before 2025.12, to leave CLI
mode). Arming-disable names come from the flag-count byte: 26 is the Betaflight
4.5 table (4.3 and 4.4 name bits 3 and 20 differently, so those print both
names), 30 the 2026.6.2 table, any other count (2025.12.1 has 29) prints `bit N`. The reply timeout is min(--timeout, 2) s per request. A
`$M!` error reply is exit 1.

`run` refuses, before connecting, any line whose first word is save, defaults,
bl, dfu, exit, flash_erase, msc, motor, escprog, serialpassthrough,
gpspassthrough, dshotprog, flash_write, dxw or bind_rx (they save, reboot, wipe,
change mode, spin a motor, hand the link to an ESC or GPS, program the ESCs,
write flash or RAM, or put the receiver into bind mode, which can drop the RC link), and any argument holding a character outside
0x20-0x7e, a `?` or a `#`. The CLI's line editor ends a line at a newline or
CR, exits on Ctrl-D at the start of a line, drops other bytes below 32, treats
127 as backspace, completes on `?` and tab and cuts the line at `#`, so
`sa\\x01ve`, `sa?` or `save#` would run as `save` behind a check on the typed
word. `restore` applies the same rule to
every line it would send and also refuses a first word that is not a command
a `diff all` holds. Text from the FC is printed with control characters
shown as \\xNN. `--selftest` checks all of this offline. The tool never
saves unless --save is given and never flashes. Needs pyserial;
`socket://host:port` works as a port (Betaflight SITL serves the CLI on TCP
5761, UART1).
"""
import argparse, difflib, glob, os, re, sys, tempfile, time, unicodedata

try:
    import serial
except ImportError:
    sys.exit("pyserial not found. Try: python3 -m pip install 'pyserial>=3.5' (or apt python3-serial)")

TOOL = 'bf-cli.py'
PROMPT = b'\r\n# '
ERROR_MARK = '###ERROR'
# Commands `run` will not send: they save, reboot, wipe, change mode, drive a
# motor or hand the link over (Betaflight 2026.6.2 cli.c: motor, escprog,
# serialpassthrough, gpspassthrough, dshotprog, flash_write, dxw, which writes
# memory and exists only in debug builds; bind_rx, cli.c ~:8274, puts an
# SPI/SRXL2/CRSF receiver into bind mode, which can drop the RC link).
FORBIDDEN = {'save', 'defaults', 'bl', 'dfu', 'exit', 'flash_erase', 'msc',
             'motor', 'escprog', 'serialpassthrough', 'gpspassthrough',
             'dshotprog', 'flash_write', 'dxw', 'bind_rx'}
# First words `diff all` emits (Betaflight 2026.6.2 cli.c printConfig). A restore
# file may send only these; anything else refuses the whole restore.
ALLOWED = {'set', 'feature', 'serial', 'aux', 'adjrange', 'rxrange', 'rxfail',
           'resource', 'timer', 'dma', 'beeper', 'beacon', 'map', 'mmix', 'smix',
           'servo', 'vtx', 'vtxtable', 'led', 'color', 'mode_color', 'waypoint',
           'profile', 'rateprofile', 'battery_profile', 'name', 'board_name',
           'manufacturer_id', 'mcu_id', 'signature', 'batch', 'mixer'}
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


def _inert_char(c):
    o = ord(c)
    if (o < 32 and c != '\n') or 0x7f <= o <= 0x9f:
        return f'\\x{o:02x}'
    if o > 0x9f and unicodedata.category(c) in ('Cf', 'Zl', 'Zp'):
        return f'\\u{o:04x}'
    return c


def inert(s):
    """s with every C0 control except newline, DEL and every C1 control shown as
    \\xNN, and every format character (bidi overrides, zero-width) and U+2028/9
    as \\uXXXX, so text from the FC cannot drive or mislead the terminal it is
    printed to."""
    return ''.join(map(_inert_char, s))


def sendable(line):
    """True if every character is 0x20-0x7e and there is no `?` or `#`. The Betaflight
    CLI editor ends a line at a newline or CR, exits on Ctrl-D at line start, drops
    other bytes below 32, treats 127 as backspace, completes on `?` and tab and cuts
    the line at `#`, so such a line runs as something other than what it spells."""
    return all(' ' <= c <= '~' for c in line) and '?' not in line and '#' not in line


def why_unsendable(line):
    """Name the first character the CLI editor would act on, and what it does."""
    for i, c in enumerate(line, 1):
        if c == '?':
            return f"a '?' at position {i}, which the CLI takes as command completion"
        if c == '\t':
            return f"a tab at position {i}, which the CLI takes as command completion"
        if c == '\x7f':
            return f"a DEL at position {i}, which the CLI takes as backspace"
        if c == '#':
            return f"a '#' at position {i}, which starts a comment: the CLI cuts the line there"
        if c in '\n\r':
            return f"a line break (\\x{ord(c):02x}) at position {i}, which ends the line here"
        if c == '\x04':
            return f"a control character (\\x04) at position {i}, which exits the CLI (a reboot) when it starts a line"
        if c == '\x0c':
            return f"a control character (\\x0c) at position {i}, which clears the screen"
        if ord(c) < 32:
            return f"a control character (\\x{ord(c):02x}) at position {i}, which is dropped"
        if ord(c) > 126:
            return f"a non-ASCII character at position {i}, which is dropped"
    return None


def refuse_unsendable(line, where):
    if not sendable(line):
        raise Refused(f"{where} has {why_unsendable(line)}, so it would run as something "
                      f"other than what it spells: '{inert(line)}'")


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
    print(inert(text.strip()))
    if not ok:
        print("save did not report 'saving' and the link stayed up; nothing verified")
        return None
    print(f"saved; board rebooting (firmware {inert(ver)} has no save noreboot)")
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
            print(inert(f"note: firmware {ident['version']} reboots on exit"))
        else:
            print(f"note: cannot read a version from {inert(ident['version'])!r}; treating the firmware "
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


def header(ident, port, what, warnings=(), notes=()):
    return [f"# {TOOL} {what}",
            f"# firmware: {inert(ident['version'])}",
            f"# board: {inert(ident['board'])}",
            f"# mcu: {inert(ident['mcu'])}",
            f"# craft_name: {inert(ident['craft']) or '-'}",
            f"# utc: {time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())}",
            f"# port: {port}"] + [f"# warning: firmware reported: {inert(w)}" for w in warnings] + [f"# warning: {n}" for n in notes]


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
    of error texts (the line without its ### marks); rewritten is the set of
    line indexes it turned into comments."""
    lines, errors, rewritten = [], [], set()
    for i, ln in enumerate(out.split('\n')):
        if ERROR_MARK in ln:
            errors.append(ln.strip().strip('#').strip())
            ln = f"# bf-cli: firmware reported: {inert(ln.strip())}"
            rewritten.add(i)
        lines.append(ln)
    return '\n'.join(lines), errors, rewritten


def printable_ascii(ln):
    return all(0x20 <= ord(c) <= 0x7e for c in ln)


def flag_nonprintable(out, skip=frozenset()):
    """(text, count): every line with a byte outside 0x20-0x7e becomes a comment
    showing it through inert_line, so the file a later `cat` or restore reads
    holds no control byte from the FC. skip holds the indexes flag_errors
    rewrote (decided by index, never by text the FC could imitate)."""
    lines, n = [], 0
    for i, ln in enumerate(out.split('\n')):
        if not printable_ascii(ln) and i not in skip:
            n += 1
            ln = f"# bf-cli: not backed up (non-printable bytes): {inert_line(ln)}"
        lines.append(ln)
    return '\n'.join(lines), n


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
    exts = ['.diff'] + (['.dump'] if dump else [])
    while any(os.path.exists(os.path.join(outdir, base + e)) for e in exts):
        n += 1
        base = f"{base.rsplit('_dup', 1)[0]}_dup{n}"
    paths, warnings = [], []
    for cmdline, ext in [('diff all', '.diff')] + ([('dump all', '.dump')] if dump else []):
        out = stable_cmd(cli, cmdline)
        out, errs, rewritten = flag_errors(out)
        warnings += errs
        out, bad = flag_nonprintable(out, rewritten)
        notes = [f"{bad} line(s) not backed up (non-printable bytes from the FC)"] if bad else []
        warnings += notes
        if 'save' not in out.split('\n')[-3:] and ext == '.diff':
            raise Refused("`diff all` output did not end with `save`: truncated, not writing it")
        p = os.path.join(outdir, base + ext)
        try:
            with open(p, 'x') as fh:  # exclusive: never overwrite a file made since the name was picked
                fh.write('\n'.join(header(ident, port, cmdline, errs, notes)) + '\n\n' + out + '\n')
        except FileExistsError:
            raise Refused(f"{p} appeared while backing up; not overwriting it")
        paths.append(p)
    return paths, warnings


def print_warnings(warnings):
    print("WRITTEN WITH FIRMWARE WARNINGS:")
    for w in warnings:
        print(f"  {inert(w)}")


def file_lines(text):
    """Lines of a file split on \\n alone, one trailing \\r dropped from each, so
    \\x1c-\\x1e, \\x85 and U+2028 stay inside their line (splitlines() would cut
    there) and fail `sendable`."""
    return [ln[:-1] if ln.endswith('\r') else ln for ln in text.split('\n')]


def read_file_header(path):
    """(header dict, embedded `# Betaflight / ...` dict or None, lines)."""
    with open(path, encoding='utf-8', errors='replace', newline='') as fh:
        lines = file_lines(fh.read())
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
    """(lines to send, {reason: count} skipped). Refuses a file with a dangerous line,
    a line that is not plain printable ASCII, or a first word `diff all` never emits."""
    send, skipped = [], {}
    for n, ln in enumerate(lines, 1):
        s = ln.strip(' ')
        if not s or s.startswith('#'):
            continue
        refuse_unsendable(s, f"line {n}")
        w = first_word(s)
        if w in SKIP_REASON:
            if w == 'defaults' and s.lower() != 'defaults nosave':
                raise Refused(f"file contains `{s}`, not the plain `defaults nosave`")
            if w == 'save' and s.lower() != 'save':
                raise Refused(f"file contains `{s}`")
            skipped[SKIP_REASON[w]] = skipped.get(SKIP_REASON[w], 0) + 1
        elif w in FORBIDDEN:
            raise Refused(f"file contains a `{w}` line, which restore will not send: '{s}'")
        elif w not in ALLOWED:
            raise Refused(f"line {n} starts with `{w}`, which is not a command a `diff all` "
                          f"holds, so restore will not send it (restore takes a `diff all` "
                          f"backup, not a `dump`): '{s}'")
        else:
            send.append(s)
    return send, skipped


def normalized(text):
    """Comparable config lines of the FC's own `diff all`. Nothing is sent, so
    nothing is refused: comments and the lines restore skips are dropped, every
    other line is kept verbatim, so an unexpected one shows up as a difference."""
    out, skipped = [], {}
    for ln in file_lines(text):
        s = ln.strip(' ')
        if not s or s.startswith('#'):
            continue
        w = first_word(s)
        if w in SKIP_REASON and s.lower() == {'defaults': 'defaults nosave', 'save': 'save'}.get(w, s.lower()):
            skipped[SKIP_REASON[w]] = skipped.get(SKIP_REASON[w], 0) + 1
        else:
            out.append(s)
    return out, skipped


def connect(a):
    port, how = pick_port(a.port)
    print(f"port: {port} ({how}), {a.baud} baud")
    cli = CLI(port, a.baud, a.timeout)
    return cli, port


def show_ident(ident):
    print(inert(f"firmware: Betaflight {ident['version']} (git {ident['git']}, MSP API {ident['msp']})"))
    print(inert(f"board:    {ident['board']} (id {ident['board_id']}, board_name {ident['board_name'] or '-'})"))
    print(inert(f"mcu:      {ident['mcu']}"))
    print(inert(f"craft:    {ident['craft'] or '-'}"))


def cmd_identify(a):
    cli, port = connect(a)
    try:
        ident = identity(cli)
        adopt_firmware(cli, ident)
        show_ident(ident)
        print("\nstatus:\n" + inert(ident['status']))
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


def check_run_args(commands):
    """Refuse, before connecting, an argument that is not one plain printable line
    or whose first word `run` never sends."""
    for i, c in enumerate(commands, 1):
        refuse_unsendable(c, f"argument {i}")
        if first_word(c) in FORBIDDEN:
            raise Refused(f"`{first_word(c)}` is not sent by run (saving is --save only)")


def cmd_run(a):
    try:
        check_run_args(a.commands)
    except Refused as e:
        print(f"REFUSED: {e}. Nothing was sent.")
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
            print(inert(out))
            if ERROR_MARK in out:
                errors.append(c)
                break
            if sm and before is not None:
                name, target = sm.group(1), sm.group(2)
                sets.append((name, target))
                after = get_value(cli, name)
                print(inert(f"{name}: {before} -> {target} -> {after}"))
                if after is None or not same_value(target, after):
                    print(f"READBACK MISMATCH: sent {target!r}, the FC now holds {inert(after or '')!r}" if after is not None
                          else f"READBACK MISMATCH: sent {target!r}, the FC now holds None")
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
                    print(inert(f"after reboot: {name} = {got}"))
                    if got is None or not same_value(target, got):
                        bad.append(f"{name}: wanted {target!r}, the FC holds {inert(got or '')!r}")
                if bad:
                    print("VERIFY MISMATCH after the save: " + '; '.join(bad))
                    return 1
                print(f"verified: {len(sets)} set values hold after the reboot.")
                return 0
            print("\n> save noreboot")
            out = cli.cmd('save noreboot')
            print(inert(out))
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
        print("REFUSED: " + '; '.join(problems) + ". Nothing was sent.")
        return 1
    try:
        send, skipped = restore_lines(flines)
    except Refused as e:
        print(f"REFUSED: {e}. Nothing was sent.")
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
            print(inert(out))
            print("defaults nosave did not report success; nothing applied; ending with `exit`")
            return 1
        failing = []
        for ln in send:
            out = cli.cmd(ln)
            if ERROR_MARK in out:
                failing.append((ln, inert(out.strip())))
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
                print(inert(out))
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
            print(inert(d))
        return 1
    finally:
        cli.leave(reboot)


DIFF_BODY = """# Betaflight / STM32H743 (S743) 2026.6.2 Jun  1 2026 / 10:00:00 (abcdef0) MSP API: 1.47
# config: manufacturer_id: TEST, board_name: TESTH743, version: 1a2b3c4d, date: 2026-06-01T10:00:00Z

# start the command batch
batch start

board_name TESTH743
manufacturer_id TEST
mcu_id 001a002b003c004d005e006f
signature

# name: MYQUAD

# resources
resource MOTOR 1 B00
timer B00 AF2
dma pin B00 0
dma ADC 1 1

# feature
feature -AIRMODE
feature GPS
beeper -ON_USB
beacon RX_LOST
serial 0 64 115200 57600 0 115200
serial 1 8192 115200 57600 0 115200
map TAER
mmix 0 1.000 1.000 -1.000 1.000
smix 0 1 0 100 0 0 100 0
servo 0 1000 2000 1500 100 -1
vtx 0 0 0 0 1 1000 1200
vtxtable bands 5
led 0 0,0::C:0
color 0 0,0,0
mode_color 0 0 0
waypoint 0 1 0 0 0
aux 0 0 0 1700 2100 0 0
adjrange 0 0 0 900 2100 0 0 0 0
rxrange 0 1000 2000
rxfail 0 a

# name
name MYQUAD

# profile 0
profile 0
set p_pitch = 52
set d_roll = 40

# rateprofile 0
rateprofile 0
set roll_rc_rate = 120

# battery_profile 0
battery_profile 0
set vbat_max_cell_voltage = 430

# restore original profile selection
profile 0
rateprofile 0
battery_profile 0

set craft_name = MYQUAD
set gyro_lpf1_static_hz = 0
set vbat_warning_cell_voltage = 340

batch end

defaults nosave

save
"""

# (name, python text for a restore file's line). Each must be refused by restore_lines.
REFUSE_LINES = [
    ('sa\\x01ve (C0 inside save)', 'sa\x01ve'),
    ('savex\\x7f (DEL as backspace)', 'savex\x7f'),
    ('sa? (completion)', 'sa?'),
    ('sa<TAB> (completion)', 'sa\t'),
    ('sa\\x1fve', 'sa\x1fve'),
    ('b\\x01l (bootloader)', 'b\x01l'),
    ('lone \\x04', '\x04'),
    ('sa\\x1cve (splitlines separator)', 'sa\x1cve'),
    ('sa\\x85ve (C1 NEL)', 'sa\x85ve'),
    ('sa\\u2028ve (line separator)', 'sa ve'),
    ('set x = caf\\xe9 (non-ASCII)', 'set x = caf\xe9'),
    ('dshotprog 1 2 (not in diff all)', 'dshotprog 1 2'),
    ('flash_write 0 x (not in diff all)', 'flash_write 0 x'),
    ('bl (FORBIDDEN)', 'bl'),
    ('motor 1 1000 (FORBIDDEN)', 'motor 1 1000'),
]


# ---- msp: read-only MSPv1 reads, no CLI -------------------------------------
# Betaflight 4.x `exit` reboots the board, which drops a USB/IP attach. MSP never
# enters the CLI and never reboots, so `msp` talks only MSP and never writes
# `#`, `exit` or a CLI line; the only newline bytes are inside NAME's frame
# (id 10), sent after the board has answered MSP. It is independent of the CLI class on purpose.
# The one send function refuses any id outside this allowlist; no request
# carries a payload. Ids from Betaflight 2026.6.2 src/main/msp/msp_protocol.h.
MSP_READ_ONLY = {1: 'API_VERSION', 2: 'FC_VARIANT', 3: 'FC_VERSION', 4: 'BOARD_INFO',
                 10: 'NAME', 105: 'RC', 110: 'ANALOG', 116: 'BOXNAMES', 119: 'BOXIDS',
                 34: 'MODE_RANGES', 150: 'STATUS_EX'}
MSP_REQ_TIMEOUT_MAX = 2.0

# armingDisableFlagNames[] by the flag-count byte of MSP_STATUS_EX. The bit order
# is the array order. They differ at bit 25.
# Betaflight 4.5.3, src/main/fc/runtime_config.c (26 entries). 4.3 and 4.4 also
# have 26 but name bit 3 BADRX and bit 20 RPMFILTER, so those two print both.
ARMING_FLAGS_26 = (
    'NOGYRO', 'FAILSAFE', 'RXLOSS', 'NOT_DISARMED|BADRX', 'BOXFAILSAFE', 'RUNAWAY', 'CRASH',
    'THROTTLE', 'ANGLE', 'BOOTGRACE', 'NOPREARM', 'LOAD', 'CALIB', 'CLI', 'CMS', 'BST',
    'MSP', 'PARALYZE', 'GPS', 'RESCUE_SW', 'DSHOT_TELEM|RPMFILTER', 'REBOOT_REQD', 'DSHOT_BBANG',
    'NO_ACC_CAL', 'MOTOR_PROTO', 'ARMSWITCH')
# Betaflight 2026.6.2, src/main/fc/runtime_config.c (30 entries)
ARMING_FLAGS_30 = (
    'NOGYRO', 'FAILSAFE', 'RXLOSS', 'NOT_DISARMED', 'BOXFAILSAFE', 'RUNAWAY', 'CRASH',
    'THROTTLE', 'ANGLE', 'BOOTGRACE', 'NOPREARM', 'LOAD', 'CALIB', 'CLI', 'CMS', 'BST',
    'MSP', 'PARALYZE', 'GPS', 'RESCUE_SW', 'DSHOT_TELEM', 'REBOOT_REQD', 'DSHOT_BBANG',
    'NO_ACC_CAL', 'MOTOR_PROTO', 'FLIP_SWITCH', 'ALT_HOLD_SW', 'POS_HOLD_SW',
    'AUTOPILOT_SW', 'ARM_SWITCH')
ARMING_FLAG_TABLES = {26: ARMING_FLAGS_26, 30: ARMING_FLAGS_30}


class MspBad(Exception):
    """A malformed or corrupt MSP frame or payload."""


class MspError(Exception):
    """The FC answered `$M!` (it rejected the request)."""


class MspTimeout(NoLink):
    pass


def msp_frame(cmd, payload=b'', direction=b'<'):
    """MSPv1 frame: $M<dir>, len, cmd, payload, XOR of len, cmd and payload.
    A payload of 255 bytes or more uses the jumbo form (len 255, then a u16 LE size
    after cmd); the FC sends those (BOXNAMES), this tool never does."""
    if len(payload) >= 255:
        body = bytes([255, cmd, len(payload) & 255, len(payload) >> 8]) + bytes(payload)
    else:
        body = bytes([len(payload), cmd]) + bytes(payload)
    ck = 0
    for b in body:
        ck ^= b
    return b'$M' + direction + body + bytes([ck])


def msp_unframe(buf):
    """Parse one frame at the start of buf (which must begin with `$M`).
    Returns None while incomplete, else (direction, cmd, payload, bytes used).
    Raises MspBad for a bad header, direction or checksum."""
    if len(buf) >= 2 and bytes(buf[:2]) != b'$M':
        raise MspBad('no $M header')
    if len(buf) < 5:
        return None
    d = bytes(buf[2:3])
    if d not in (b'<', b'>', b'!'):
        raise MspBad(f'bad direction byte {buf[2]:#04x}')
    ln, cmd = buf[3], buf[4]
    head = 5
    if ln == 255:  # MSPv1 jumbo frame: a u16 LE size follows the command byte
        if len(buf) < 7:
            return None
        ln, head = buf[5] | buf[6] << 8, 7
    total = head + ln + 1
    if len(buf) < total:
        return None
    ck = 0
    for b in buf[3:head + ln]:
        ck ^= b
    if ck != buf[head + ln]:
        raise MspBad(f'checksum {buf[head + ln]:#04x}, computed {ck:#04x}')
    return d.decode(), cmd, bytes(buf[head:head + ln]), total


def msp_send(ser, cmd):
    """The only place MSP bytes are written. Refuses any id off the allowlist."""
    if cmd not in MSP_READ_ONLY:
        raise Refused(f"MSP id {cmd} is not on the read-only allowlist; refusing to send it")
    ser.write(msp_frame(cmd))


def msp_request(ser, cmd, timeout, buf):
    """Send one request and return its payload. buf is the receive buffer, shared
    across calls. Resyncs on `$M`, drops frames with a bad checksum or another
    command id. Raises MspTimeout, or MspError for a `$M!` reply."""
    msp_send(ser, cmd)
    name = MSP_READ_ONLY[cmd]
    deadline = time.monotonic() + timeout
    while True:
        while True:
            i = bytes(buf).find(b'$M')
            if i < 0:
                del buf[:len(buf) - 1 if buf.endswith(b'$') else len(buf)]
                break
            del buf[:i]
            try:
                fr = msp_unframe(buf)
            except MspBad:
                del buf[:1]
                continue
            if fr is None:
                break
            d, c, payload, used = fr
            del buf[:used]
            if d == '<' or c != cmd:
                continue
            if d == '!':
                raise MspError(f"FC rejected MSP {name} ({cmd}) with an error reply")
            return payload
        if time.monotonic() >= deadline:
            raise MspTimeout(f"no valid reply to MSP {name} ({cmd}) within {timeout:g} s")
        n = getattr(ser, 'in_waiting', 0) or 1
        buf.extend(ser.read(n))


def u16(p, o):
    return p[o] | p[o + 1] << 8


def inert_line(s):
    """inert(s) with newline shown as \\x0a too, for a field that is one line."""
    return inert(s).replace('\n', '\\x0a')


def msp_text(b):
    return inert_line(bytes(b).decode('latin-1'))


def parse_fc_version(p):
    """FC_VERSION: major, minor, patch bytes. Calendar versions send the year since 2000."""
    if len(p) < 3:
        raise MspBad('FC_VERSION shorter than 3 bytes')
    major = p[0] + 2000 if p[0] >= 20 else p[0]
    return f'{major}.{p[1]}.{p[2]}'


def _printable(b):
    return all(0x20 <= c <= 0x7e for c in b)


def parse_board_info(p, api_minor):
    """(board identifier, board name). Layout (2026.6.2 msp.c): 4-byte identifier,
    u16 hardware revision, u8 board type, u8 capabilities, then length-prefixed
    target name, board name, manufacturer id. The strings exist only on newer API
    versions, so they are parsed only from API 1.37 on (Hypothesis: the
    threshold; every string is validated, so a wrong guess shows no name, not
    garbage). The name is the board name, else the target name, else ''."""
    ident = bytes(p[:4])
    if len(ident) < 4 or not _printable(ident):
        return '', ''
    ident = ident.decode('ascii')
    name = ''
    if api_minor >= 37 and len(p) >= 9:
        try:
            o, strs = 8, []
            for _ in range(2):
                n = p[o]
                s = bytes(p[o + 1:o + 1 + n])
                if len(s) != n or not _printable(s):
                    raise ValueError
                strs.append(s.decode('ascii'))
                o += 1 + n
            name = strs[1] or strs[0]
        except (ValueError, IndexError):
            name = ''
    return ident, name


def parse_status_ex(p):
    """(flag count, arming-disable mask). Offsets (2026.6.2 msp.c MSP_STATUS_EX):
    15 fixed bytes, a header byte whose low 4 bits count extra flight-mode bytes,
    those bytes, then the u8 flag count and the u32 mask."""
    if len(p) < 16:
        raise MspBad('STATUS_EX shorter than its fixed part')
    o = 15 + 1 + (p[15] & 0x0f)
    if len(p) < o + 5:
        raise MspBad('STATUS_EX ends before the arming-disable flags')
    return p[o], int.from_bytes(p[o + 1:o + 5], 'little')


def arming_names(count, mask):
    """[(bit, name)] for the set bits in mask, by table for this flag count; `bit N`
    when the count has no table. Returns (flags, table known)."""
    table = ARMING_FLAG_TABLES.get(count)
    flags = [(b, table[b] if table and b < len(table) else f'bit {b}')
             for b in range(32) if mask >> b & 1]
    return flags, table is not None


def parse_rc(p):
    return [u16(p, i) for i in range(0, len(p) - 1, 2)]


def parse_rssi(p):
    if len(p) < 5:
        raise MspBad('ANALOG shorter than 5 bytes')
    return u16(p, 3)


def parse_boxes(names_payload, ids_payload):
    """permanent id -> name, from BOXNAMES (';'-separated) zipped with BOXIDS."""
    names = bytes(names_payload).decode('latin-1').split(';')
    if names and names[-1] == '':
        names.pop()
    return dict(zip(ids_payload, (inert_line(n) for n in names)))


def parse_mode_ranges(p, boxes):
    """[(name, channel, low us, high us, start step, end step)], skipping start == end."""
    out = []
    for i in range(0, len(p) - 3, 4):
        pid, aux, start, end = p[i:i + 4]
        if start == end:
            continue
        out.append((boxes.get(pid, f'id {pid}'), aux + 5, 900 + 25 * start, 900 + 25 * end, start, end))
    return out


def mode_state(rng, rc):
    ch = rng[1]
    if ch - 1 >= len(rc):
        return f'{rng[0]} ch{ch}=n/a'
    v = rc[ch - 1]
    return f'{rng[0]} ch{ch}={v} {"on" if rng[2] <= v < rng[3] else "off"}'


def cmd_msp(a, ser=None, sleep=time.sleep):
    if a.samples < 1 or a.interval < 0:
        raise Refused('--samples must be at least 1 and --interval at least 0')
    if ser is None:
        port, how = pick_port(a.port)
        print(f"port: {port} ({how}), {a.baud} baud")
        try:
            ser = serial.serial_for_url(port, baudrate=a.baud, timeout=0.1)
        except (serial.SerialException, OSError) as e:
            raise NoLink(f"cannot open {port}: {e}")
    to = min(a.timeout, MSP_REQ_TIMEOUT_MAX)
    buf = bytearray()
    try:
        if hasattr(ser, 'reset_input_buffer'):
            ser.reset_input_buffer()

        def get(cmd):
            return msp_request(ser, cmd, to, buf)

        try:
            api = get(1)
        except MspTimeout as e:
            raise NoLink(f"{e}. The board may be in CLI mode (it ignores MSP there) or not "
                         "running Betaflight. This tool does not send `exit` because that "
                         "reboots a 4.x board; power-cycling the board gets it out of CLI "
                         "mode, and so does `bf-cli.py identify`, without a reboot only on "
                         "firmware from 2025.12.")
        if len(api) < 3:
            raise MspBad('API_VERSION shorter than 3 bytes')
        variant = msp_text(get(2)[:4])
        version = parse_fc_version(get(3))
        ident, bname = parse_board_info(get(4), api[2])
        craft = msp_text(get(10))
        boxes = parse_boxes(get(116), get(119))
        ranges = parse_mode_ranges(get(34), boxes)
        board = ' '.join(dict.fromkeys(x for x in (inert(ident), inert(bname)) if x))
        print(f"fc: {variant} {version}, MSP API {api[1]}.{api[2]}"
              + (f", board {board}" if board else "") + f", craft {craft or '-'}")
        print("modes: " + ('; '.join(f"{n} ch{c} {lo}-{hi}" for n, c, lo, hi, _, _ in ranges)
                           if ranges else "none configured"))
        counts, last_rc, changed, intervals, noted = {}, None, 0, 0, False
        for i in range(1, a.samples + 1):
            count, mask = parse_status_ex(get(150))
            rssi = parse_rssi(get(110))
            rc = parse_rc(get(105))
            flags, known = arming_names(count, mask)
            if not known and not noted:
                print(f"note: names are unknown for flag count {count} (known: 26, 30)")
                noted = True
            for b, n in flags:
                counts[b] = (n, counts.get(b, (n, 0))[1] + 1)
            if last_rc is not None:
                intervals += 1
                changed += rc != last_rc
            last_rc = rc
            print(f"sample {i}/{a.samples}: arming disabled: {' '.join(n for _, n in flags) or 'none'}"
                  f" | rssi {rssi}/1023 ({round(rssi * 100 / 1023)}%) | rc {' '.join(map(str, rc)) or '(no channels)'}")
            if i < a.samples:
                sleep(a.interval)
        tally = ', '.join(f"{n} {c}/{a.samples}" for _, (n, c) in sorted(counts.items()))
        print(f"summary over {a.samples} samples: {tally or 'no arming-disable flags'}; "
              + (f"rc frames changed in {changed}/{intervals} intervals" if intervals
                 else "rc change needs 2 or more samples"))
        if ranges:
            print("modes now (last sample): " + '; '.join(mode_state(r, last_rc) for r in ranges))
        return 0
    except MspBad as e:
        raise NoLink(f"unusable MSP reply: {e}")
    except (serial.SerialException, OSError) as e:
        raise NoLink(f"serial link lost: {e}")
    except MspError as e:
        print(f"MSP ERROR: {e}")
        return 1
    finally:
        if hasattr(ser, 'close'):
            ser.close()


def selftest():
    """Offline checks of the line gate, restore_lines, first_word and inert."""
    results = []

    def check(name, ok, detail=''):
        results.append(ok)
        print(f"{'PASS' if ok else 'FAIL'}: {name}" + ('' if ok or detail == '' else f"  [{detail!r}]"))

    def refused(fn, *args):
        try:
            fn(*args)
        except Refused:
            return True
        return False

    # restore: each of these in an otherwise good file refuses the whole restore
    good = DIFF_BODY.split('\n')
    for name, bad in REFUSE_LINES:
        text = DIFF_BODY.replace('\nset craft_name = MYQUAD\n', '\n' + bad + '\nset craft_name = MYQUAD\n')
        check(f"restore refuses {name}", refused(restore_lines, file_lines(text)))
        check(f"restore refuses {name} (alone)", refused(restore_lines, file_lines('set a = 1\n' + bad + '\n')))
    # a file read through read_file_header splits on \n only
    with tempfile.TemporaryDirectory() as d:
        p = os.path.join(d, 'x.diff')
        with open(p, 'w', encoding='utf-8', newline='') as fh:
            fh.write('# firmware: 2026.6.2\n# board: X\nset a = 1\nsa\x1cve\n')
        lines = read_file_header(p)[2]
        check("read_file_header keeps \\x1c inside its line", 'sa\x1cve' in lines)
        check("read_file_header file with \\x1c refuses at restore_lines", refused(restore_lines, lines))
    check("file_lines drops one trailing \\r", file_lines('set a = 1\r\nset b = 2\r\r\n') == ['set a = 1', 'set b = 2\r', ''])

    # restore: a realistic diff all body is accepted
    try:
        send, skipped = restore_lines(good)
        check("restore accepts a 2026.6.2 diff all body", True)
    except Refused as e:
        send, skipped = [], {}
        print(f"  refused: {e}")
        check("restore accepts a 2026.6.2 diff all body", False)
    check("accepted body: set/profile/serial lines are sent",
          'set craft_name = MYQUAD' in send and 'serial 0 64 115200 57600 0 115200' in send
          and 'profile 0' in send and 'resource MOTOR 1 B00' in send)
    check("accepted body: batch, defaults, save, mcu_id, signature are not sent",
          not any(first_word(s) in SKIP_REASON for s in send) and len(skipped) == 5)
    check("accepted body: every sent word is on the allowlist",
          all(first_word(s) in ALLOWED for s in send))
    check("a non-QUADX diff with a mixer line is accepted and sent",
          'mixer HEX6X' in restore_lines(file_lines('mixer HEX6X\nset a = 1\n'))[0])
    check("refusal names the character and what the CLI does with it",
          "a '?' at position 3" in why_unsendable('sa?') and 'backspace' in why_unsendable('save\x7f')
          and '\\x01' in why_unsendable('sa\x01ve') and 'completion' in why_unsendable('sa\t'))
    check("why_unsendable names newline, comment and EOT",
          'ends the line' in why_unsendable('a\nb') and 'comment' in why_unsendable('a#b')
          and 'exits' in why_unsendable('a\x04b') and 'clears' in why_unsendable('a\x0cb'))
    check("restore refuses a mid-line # and `save#`",
          refused(restore_lines, ['set a = 1 # save']) and refused(restore_lines, ['save#']))
    check("restore still skips a comment line", restore_lines(['  # comment', 'set a = 1']) == (['set a = 1'], {}))
    check("sendable refuses #", not sendable('save#'))
    check("no private board name in the file", ('LUX' + 'N') not in open(__file__, encoding='utf-8').read())
    check("normalized keeps a word outside ALLOWED without raising",
          'dshotprog 1 2' in normalized('set a = 1\ndshotprog 1 2\nsa\x01ve\n')[0])
    check("normalized matches restore_lines on a good diff body", normalized(DIFF_BODY) == (send, skipped))
    check("accepted body: CRLF file gives the same lines",
          restore_lines(file_lines(DIFF_BODY.replace('\n', '\r\n'))) == (send, skipped))
    check("every ALLOWED word is accepted as a line",
          not any(refused(restore_lines, [w + ' 1']) for w in ALLOWED - set(SKIP_REASON)))
    check("restore still refuses `defaults` other than nosave", refused(restore_lines, ['defaults']))
    check("restore still refuses `save noreboot`", refused(restore_lines, ['save noreboot']))
    check("restore accepts uppercase first word", restore_lines(['SET a = 1'])[0] == ['SET a = 1'])

    # sendable
    check("sendable accepts a plain line", sendable('set craft_name = MY-QUAD_1'))
    check("sendable accepts the empty line", sendable(''))
    for name, ch in [('tab', '\t'), ('newline', '\n'), ('CR', '\r'), ('DEL', '\x7f'),
                     ('ESC', '\x1b'), ('NUL', '\x00'), ('C1', '\x9b'), ('?', '?'), ('e-acute', '\xe9')]:
        check(f"sendable refuses {name}", not sendable('a' + ch + 'b'))

    # run arguments
    for name, arg in [('sa\\x01ve', 'sa\x01ve'), ('b\\x01l', 'b\x01l'), ('sa?', 'sa?'),
                      ('save#', 'save#'), ('bl#x', 'bl#x'), ('defaults#', 'defaults#'), ('exit#', 'exit#'),
                      ('savex\\x7f', 'savex\x7f'), ('save<TAB>', 'save\t'),
                      ('two lines', 'get a\nsave'), ('CR', 'get a\rsave'),
                      ('save', 'save'), ('  Bl', '  Bl'), ('motor 1 2000', 'motor 1 2000'),
                      ('non-ASCII', 'set craft_name = \xe9')]:
        check(f"run refuses {name}", refused(check_run_args, ['get craft_name', arg]))
    check("run accepts plain commands",
          not refused(check_run_args, ['get craft_name', 'set craft_name = QUAD', 'status', 'diff all']))
    check("run refusal names the line through inert",
          'sa\\x01ve' in str(_msg(check_run_args, ['sa\x01ve'])))

    # first_word and inert
    check("first_word lowercases and takes the first word", first_word('  SET a = 1') == 'set')
    check("first_word of an empty line is empty", first_word('   ') == '')
    check("inert escapes ESC", inert('a\x1b[2Jb') == 'a\\x1b[2Jb')
    check("inert escapes BEL", inert('\x07') == '\\x07')
    check("inert escapes C1 0x9b", inert('\x9b') == '\\x9b')
    check("inert escapes DEL and NUL", inert('\x7f\x00') == '\\x7f\\x00')
    check("inert keeps newline and printable text", inert('a b\nc~') == 'a b\nc~')
    check("inert keeps non-ASCII above C1", inert('caf\xe9') == 'caf\xe9')
    check("inert escapes CR", inert('a\rb') == 'a\\x0db')
    check("inert escapes U+202E and U+200B", inert('a\u202eb\u200bc') == 'a\\u202eb\\u200bc')
    check("inert escapes U+2028, U+2029, U+FEFF, U+2066", inert('\u2028\u2029\ufeff\u2066') == '\\u2028\\u2029\\ufeff\\u2066')
    check("inert_line escapes bidi and newline", inert_line('a\u202e\nb') == 'a\\u202e\\x0ab')
    # backup body: non-printable lines commented, exclusive create
    class BodyCli:
        timeout = 1

        def __init__(self, body):
            self.body = body

        def cmd(self, line, timeout=None):
            return self.body
    bident = {'version': '2026.6.2', 'board': 'S743', 'mcu': 'H743', 'craft': 'X', 'board_name': 'B'}
    def slurp(path):
        with open(path) as fh:
            return fh.read()
    clean = 'set a = 1\nset craft_name = X\nsave'
    dirty = 'set a = 1\nset craft_name = A\x1b[2JB\x07C\x85D\nsave'
    with tempfile.TemporaryDirectory() as td:
        ps, ws = do_backup(BodyCli(clean), bident, 'p', td, False)
        txt = slurp(ps[0])
        check("backup: clean body kept verbatim, no warnings", 'set craft_name = X\n' in txt and not ws, ws)
        ps, ws = do_backup(BodyCli(dirty), bident, 'p', td + '/d', True)
        raw = slurp(ps[0])
        check("backup: ESC/BEL/NEL line commented, no raw control byte, warning returned",
              '# bf-cli: not backed up (non-printable bytes): set craft_name = A\\x1b[2JB\\x07C\\x85D' in raw
              and not any(ord(c) < 32 and c != '\n' or 0x7f <= ord(c) <= 0x9f for c in raw)
              and 'set a = 1\n' in raw and len(ws) == 2 and '# warning: 1 line(s) not backed up' in raw, (ws, raw))
        check("backup: .dump gets the same treatment", len(ps) == 2 and '\x1b' not in slurp(ps[1]))
        forged = 'set a = 1\n# bf-cli: firmware reported: \x1b[2J\x07pwn\nsave'
        ps, ws = do_backup(BodyCli(forged), bident, 'p', td + '/f', False)
        raw = slurp(ps[0])
        check("backup: an FC line imitating the error-comment prefix is still commented",
              bool('\x1b' not in raw and '\x07' not in raw and ws), (ws, raw))
    with tempfile.TemporaryDirectory() as t2:
        real_stamp, real_exists = globals()['utc_stamp'], os.path.exists
        globals()['utc_stamp'] = lambda: 'T'
        try:
            first, _ = do_backup(BodyCli(clean), bident, 'p', t2, False)
            with open(first[0], 'w') as fh:
                fh.write('PRECIOUS')
            second, _ = do_backup(BodyCli(clean), bident, 'p', t2, False)
            os.path.exists = lambda path: False  # as if the file appeared after the name was picked
            try:
                raced = refused(do_backup, BodyCli(clean), bident, 'p', t2, False)
            finally:
                os.path.exists = real_exists
        finally:
            globals()['utc_stamp'] = real_stamp
        check("backup: existing file kept, next backup gets a new name",
              slurp(first[0]) == 'PRECIOUS' and second[0] != first[0])
        check("backup: a file that appears after the name was picked is refused, not overwritten",
              raced and slurp(first[0]) == 'PRECIOUS')
    check("run refuses bind_rx", refused(check_run_args, ['get craft_name', 'bind_rx']))
    check("header writes craft, mcu, board, version through inert",
          '\x1b' not in ''.join(header({'version': '1\x1b', 'board': 'B\x07', 'mcu': 'M\x9b',
                                        'craft': 'C\x1b[2J'}, 'p', 'diff all')))

    # msp: framing, checksum, gate, tables, decode, and what is ever written
    import struct, io, contextlib
    fr = msp_frame(1)
    check("msp request frame is $M< len cmd checksum", fr == b'$M<\x00\x01\x01')
    check("msp checksum is XOR of len, cmd, payload",
          msp_frame(105, b'\x01\x02', b'>') == b'$M>\x02\x69\x01\x02' + bytes([2 ^ 0x69 ^ 1 ^ 2]))
    check("msp unframe round-trips", msp_unframe(msp_frame(110, b'abc', b'>')) == ('>', 110, b'abc', 9))
    check("msp unframe: incomplete is None", msp_unframe(msp_frame(110, b'abc', b'>')[:-1]) is None)
    jp = bytes(range(256)) * 2
    check("msp jumbo frame round-trips", msp_unframe(msp_frame(116, jp, b'>'))[:3] == ('>', 116, jp))
    bad = bytearray(msp_frame(110, b'abc', b'>'))
    bad[-1] ^= 1

    def raises(exc, fn, *args):
        try:
            fn(*args)
        except exc:
            return True
        return False
    check("msp bad checksum rejected", raises(MspBad, msp_unframe, bytes(bad)))
    check("msp bad header rejected", raises(MspBad, msp_unframe, b'$X>\x00\x01\x01'))

    class Wire:
        """Fake serial: records every byte written; answers well-formed requests
        from `answers` (cmd -> payload); `junk` is sent first; no answers = silent."""
        def __init__(self, answers=None, junk=b'', errors=()):
            self.answers, self.errors, self.rx, self.written = answers or {}, errors, bytearray(junk), bytearray()
            self.closed = False

        @property
        def in_waiting(self):
            return len(self.rx)

        def reset_input_buffer(self):
            pass

        def write(self, data):
            self.written += data
            d = bytes(data)
            ok = d[:3] == b'$M<' and len(d) >= 6
            if ok:
                _, c, payload, _ = msp_unframe(d)
                if c in self.errors:
                    self.rx += msp_frame(c, b'', b'!')
                elif c in self.answers:
                    ans = self.answers[c]
                    self.rx += msp_frame(c, ans() if callable(ans) else ans, b'>')

        def read(self, n=1):
            time.sleep(0.001)
            out, self.rx = bytes(self.rx[:n]), self.rx[n:]
            return out

        def close(self):
            self.closed = True

    def sent_frames(w):
        out, b = [], bytes(w.written)
        while b:
            f = msp_unframe(b)
            if f is None or f[0] != '<' or f[2]:
                return None
            out.append(f[1])
            b = b[f[3]:]
        return out

    check("msp allowlist has no id >= 200 or in {68, 99, 11}",
          all(i < 200 and i not in (68, 99, 11) for i in MSP_READ_ONLY))
    for bad_id in (68, 99, 200, 11):
        w = Wire()
        check(f"msp gate refuses id {bad_id} and writes nothing",
              raises(Refused, msp_send, w, bad_id) and not w.written)
    check("msp gate refuses a request with a payload-free id outside the set (255)", raises(Refused, msp_send, Wire(), 255))
    check("arming table 26 ends ARMSWITCH at bit 25", len(ARMING_FLAGS_26) == 26 and ARMING_FLAGS_26[25] == 'ARMSWITCH')
    check("arming table 30 has FLIP_SWITCH at 25 and ARM_SWITCH at 29",
          len(ARMING_FLAGS_30) == 30 and ARMING_FLAGS_30[25] == 'FLIP_SWITCH' and ARMING_FLAGS_30[29] == 'ARM_SWITCH')
    check("arming 26: bit 25 is ARMSWITCH", arming_names(26, 1 << 25) == ([(25, 'ARMSWITCH')], True))
    check("arming 30: bits 2 and 25 are RXLOSS and FLIP_SWITCH",
          arming_names(30, 4 | 1 << 25) == ([(2, 'RXLOSS'), (25, 'FLIP_SWITCH')], True))
    check("arming 30: bit 29 is ARM_SWITCH", arming_names(30, 1 << 29)[0] == [(29, 'ARM_SWITCH')])
    check("arming unknown count 28 prints bit N and says unknown",
          arming_names(28, 1 | 1 << 20) == ([(0, 'bit 0'), (20, 'bit 20')], False))
    check("arming mask 0 is no flags", arming_names(30, 0) == ([], True))

    def status_ex(count, mask, extra=0):
        return (b'\x00' * 15 + bytes([extra]) + b'\xaa' * extra + bytes([count]) + struct.pack('<I', mask)
                + b'\x00' * 8)
    check("status_ex flags read after 0 extra flight-mode bytes", parse_status_ex(status_ex(30, 0x14)) == (30, 0x14))
    check("status_ex flags skip 3 extra flight-mode bytes", parse_status_ex(status_ex(26, 1 << 25, 3)) == (26, 1 << 25))
    check("status_ex header uses the low 4 bits only", parse_status_ex(status_ex(30, 7, 2)[:15] + b'\xf2' + status_ex(30, 7, 2)[16:]) == (30, 7))
    check("status_ex too short is rejected", raises(MspBad, parse_status_ex, b'\x00' * 17))
    check("fc version calendar bytes", parse_fc_version(bytes([26, 6, 2]) + b'\x052026.6.2'[:6]) == '2026.6.2')
    check("fc version 4.5.3", parse_fc_version(bytes([4, 5, 3])) == '4.5.3')

    def board(ident=b'SH74', names=(b'TBS', b'TBS_LUCID_H7', b'TBS')):
        return ident + b'\x00\x00\x00\x01' + b''.join(bytes([len(n)]) + n for n in names) + b'\x00' * 40
    check("board info: identifier and board name", parse_board_info(board(), 46) == ('SH74', 'TBS_LUCID_H7'))
    check("board info: empty board name falls back to target name",
          parse_board_info(board(names=(b'TGT', b'', b'')), 46) == ('SH74', 'TGT'))
    check("board info: old API prints the identifier only", parse_board_info(board(), 30) == ('SH74', ''))
    check("board info: a string running past the payload prints the identifier only",
          parse_board_info(b'SH74\x00\x00\x00\x01\x40abc', 46) == ('SH74', ''))
    check("board info: control bytes in a name are not shown",
          parse_board_info(board(names=(b'T', b'A\x1bB', b'')), 46) == ('SH74', ''))
    check("board info: garbage identifier prints nothing", parse_board_info(b'\x00\x01\x02\x03' + b'\x00' * 20, 46) == ('', ''))
    boxes = parse_boxes(b'ARM;ANGLE;BEEPER;', bytes([0, 1, 13]))
    hb = parse_boxes(b'A\x1b[2J;B\x9bX;C\x07;D\nE;', bytes([0, 1, 2, 3]))
    check("boxnames: ESC, C1, BEL and LF are shown as \\xNN", hb == {0: 'A\\x1b[2J', 1: 'B\\x9bX', 2: 'C\\x07', 3: 'D\\x0aE'}, hb)
    check("msp_text: LF in a craft name is shown as \\x0a", msp_text(b'a\nsample 9/9') == 'a\\x0asample 9/9', msp_text(b'a\nsample 9/9'))
    check("run refuses dshotprog, flash_write and dxw",
          all(refused(check_run_args, [c]) for c in ('dshotprog 0 1', 'flash_write 0 x', 'dxw 20000000 0')))
    check("boxnames zip boxids into permanent id -> name", boxes == {0: 'ARM', 1: 'ANGLE', 13: 'BEEPER'})
    mr = parse_mode_ranges(bytes([0, 0, 32, 48, 1, 2, 16, 32, 13, 2, 32, 48, 5, 3, 7, 7, 0, 0, 0, 0]), boxes)
    check("mode ranges: us = 900 + 25 x step, channel = aux + 5, start == end skipped",
          mr == [('ARM', 5, 1700, 2100, 32, 48), ('ANGLE', 7, 1300, 1700, 16, 32), ('BEEPER', 7, 1700, 2100, 32, 48)], mr)
    check("mode_state: in range is on, outside off, missing channel n/a",
          mode_state(('ARM', 5, 1700, 2100, 32, 48), [0, 0, 0, 0, 1800]) == 'ARM ch5=1800 on'
          and mode_state(('ARM', 5, 1700, 2100, 32, 48), [0, 0, 0, 0, 2100]) == 'ARM ch5=2100 off'
          and mode_state(('ARM', 5, 1700, 2100, 32, 48), [0, 0]) == 'ARM ch5=n/a')
    check("rc channels counted from the payload length", parse_rc(struct.pack('<3H', 1500, 988, 2011)) == [1500, 988, 2011])
    check("rssi is the u16 at offset 3", parse_rssi(b'\x00\x00\x00' + struct.pack('<H', 714) + b'\x00' * 4) == 714)

    def good_answers(mask=4 | 1 << 20, count=30, rcs=None):
        seq = iter(rcs or [[1500, 988, 2011]] * 4)
        return {1: bytes([0, 1, 46]), 2: b'BTFL', 3: bytes([26, 6, 2]) + b'\x082026.6.2',
                4: board(), 10: b'BLACK\x1bFOOT', 116: b'ARM;ANGLE;BEEPER;', 119: bytes([0, 1, 13]),
                34: bytes([0, 0, 32, 48, 1, 2, 16, 32, 13, 2, 32, 48]),
                150: status_ex(count, mask), 110: b'\x00' * 3 + struct.pack('<H', 714) + b'\x00' * 4,
                105: lambda: struct.pack(f'<{len(c := next(seq))}H', *c)}

    def run_msp(wire, samples=3, argv_timeout=30.0):
        ns = argparse.Namespace(samples=samples, interval=0.0, timeout=argv_timeout, port=None, baud=115200)
        buf = io.StringIO()
        try:
            with contextlib.redirect_stdout(buf):
                rc = cmd_msp(ns, ser=wire, sleep=lambda s: None)
        except NoLink as e:
            rc, _ = 2, buf.write(f"NO LINK: {e}\n")
        except Refused as e:
            rc, _ = 1, buf.write(f"REFUSED: {e}\n")
        return rc, buf.getvalue()

    w = Wire(good_answers(rcs=[[1500, 1500, 988, 1500, 988, 2011, 988], [1500, 1500, 988, 1500, 988, 2011, 988], [1500, 1500, 988, 1500, 988, 2011, 1000]]), junk=b'\xff$$Mjunk')
    rc, o = run_msp(w)
    check("msp decodes a full run: exit 0", rc == 0, o)
    check("msp header line", "fc: BTFL 2026.6.2, MSP API 1.46, board SH74 TBS_LUCID_H7, craft BLACK\\x1bFOOT" in o, o)
    check("msp modes line", "modes: ARM ch5 1700-2100; ANGLE ch7 1300-1700; BEEPER ch7 1700-2100" in o, o)
    check("msp sample line names flags, rssi percent and rc",
          "sample 1/3: arming disabled: RXLOSS DSHOT_TELEM | rssi 714/1023 (70%) | rc 1500 1500 988 1500 988 2011 988" in o, o)
    check("msp summary counts flags and rc changes",
          "summary over 3 samples: RXLOSS 3/3, DSHOT_TELEM 3/3; rc frames changed in 1/2 intervals" in o, o)

    class DyingWire(Wire):
        """A link that drops (cable pulled, USB/IP detach) on the nth write."""
        def __init__(self, answers, die_on, exc):
            super().__init__(answers)
            self.n, self.die_on, self.exc = 0, die_on, exc

        def write(self, data):
            self.n += 1
            if self.n >= self.die_on:
                raise self.exc('device disconnected')
            super().write(data)

    for die_on, exc in ((1, serial.SerialException), (4, OSError)):
        dw = DyingWire(good_answers(), die_on, exc)
        lrc, lo = run_msp(dw)
        check(f"msp: {exc.__name__} on write {die_on} is the NO LINK path, exit 2",
              lrc == 2 and "NO LINK" in lo and dw.closed, (lrc, lo))

    class DyingRead(Wire):
        def read(self, n=1):
            raise serial.SerialException('read failed')

    lrc, lo = run_msp(DyingRead(good_answers()))
    check("msp: SerialException on read is exit 2", lrc == 2 and "NO LINK" in lo, (lrc, lo))

    class SaveCli:
        s = type('S', (), {'close': lambda self: None})()

        def save_and_reboot(self):
            return True, 'Saving'

    real_reconnect = globals()['reconnect']
    globals()['reconnect'] = lambda *args, **kw: SaveCli()
    buf = io.StringIO()
    try:
        with contextlib.redirect_stdout(buf):
            save_old(SaveCli(), 'p', argparse.Namespace(baud=1, timeout=1), '4.\x1b[2J\x07\x9b')
    finally:
        globals()['reconnect'] = real_reconnect
    check("save_old: version token with ESC, BEL and C1 prints no raw control byte",
          'saved; board rebooting (firmware 4.\\x1b[2J\\x07\\x9b' in buf.getvalue()
          and not any(c in buf.getvalue() for c in '\x1b\x07\x9b'), buf.getvalue())
    check("msp modes now uses the last sample",
          "modes now (last sample): ARM ch5=988 off; ANGLE ch7=1000 off; BEEPER ch7=1000 off" in o, o)
    check("msp output has no raw ESC", '\x1b' not in o)
    ids = sent_frames(w)
    check("msp wrote only well-formed payload-free $M< requests with allowlisted ids",
          ids is not None and all(i in MSP_READ_ONLY for i in ids) and ids[0] == 1 and len(ids) == 8 + 3 * 3, ids)
    # NAME is id 10, so its frame holds byte 0x0a twice (cmd, and the checksum len^cmd). That is
    # inside a well-formed frame, sent only after API_VERSION (id 1, no 0x0a/0x23/0x0d) was answered.
    check("msp wrote no #, no CR, no 'exit'; a newline byte only as NAME's id, after the first reply",
          b'#' not in w.written and b'\r' not in w.written and b'exit' not in w.written
          and b'\n' not in msp_frame(1) and bytes(w.written).count(b'\n') == 2)
    check("msp closes the port", w.closed)
    rc, o = run_msp(Wire(good_answers(mask=0)), samples=1)
    check("msp mask 0 prints arming disabled: none", rc == 0 and "arming disabled: none" in o and 'no arming-disable flags' in o, o)
    rc, o = run_msp(Wire(good_answers(count=28, mask=1 << 3)), samples=2)
    check("msp unknown flag count: bit N and a note", rc == 0 and 'arming disabled: bit 3' in o and 'names are unknown for flag count 28' in o, o)
    rc, o = run_msp(Wire(good_answers(count=26, mask=1 << 25)), samples=1)
    check("msp 26-flag board decodes ARMSWITCH", 'arming disabled: ARMSWITCH' in o, o)
    rc, o = run_msp(Wire(good_answers(), errors=(150,)), samples=1)
    check("msp $M! reply is exit 1", rc == 1 and 'MSP ERROR' in o, (rc, o))
    w = Wire({1: bytes([0, 1, 46])})
    w.rx = bytearray(b'$M>\x03\x01\x00\x01\x2e\x00')  # stale frame with a bad checksum, then silence
    rc, o = run_msp(w, argv_timeout=0.05)
    check("msp bad checksum on a reply is not accepted", rc == 2, (rc, o))
    w = Wire()
    rc, o = run_msp(w, argv_timeout=0.05)
    check("msp no reply: exit 2, one request written, nothing else",
          rc == 2 and bytes(w.written) == msp_frame(1) and sent_frames(w) == [1], (rc, bytes(w.written)))
    check("msp no reply: message names CLI mode, no exit sent, power cycle and identify",
          'CLI mode' in o and 'does not send `exit`' in o and 'power-cycling' in o and 'identify' in o, o)
    w = Wire(good_answers(), errors=())
    del w.answers[34]
    rc, o = run_msp(w, argv_timeout=0.05)
    check("msp timeout after the first reply is exit 2", rc == 2 and 'MODE_RANGES' in o, o)
    check("msp refuses --samples 0", run_msp(Wire(), samples=0)[0] == 1)
    # through main(): exit 2 on an unopenable port
    saved_argv, saved_sfu = sys.argv, serial.serial_for_url
    try:
        sys.argv = ['bf-cli.py', '--port', 'socket://127.0.0.1:1', 'msp']
        def _boom(*a, **k):
            raise OSError('refused')
        serial.serial_for_url = _boom
        with contextlib.redirect_stdout(io.StringIO()):
            mrc = main()
        check("msp via main: unopenable port is exit 2", mrc == 2)
    finally:
        sys.argv, serial.serial_for_url = saved_argv, saved_sfu

    failed = results.count(False)
    print(f"{results.count(True)} passed, {failed} failed")
    return 1 if failed else 0


def _msg(fn, *args):
    try:
        fn(*args)
    except Refused as e:
        return e
    return ''


def main():
    if '--selftest' in sys.argv[1:]:
        return selftest()
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', help='serial device or socket://host:port (default: the first '
                    '/dev/serial/by-id entry containing Betaflight, else /dev/ttyACM0)')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--timeout', type=float, default=30.0,
                    help='seconds to wait for the prompt after one line (diff/dump get at least 120)')
    ap.add_argument('--selftest', action='store_true', help='offline checks of the line gate and exit')
    sub = ap.add_subparsers(dest='cmd', required=True)
    sub.add_parser('identify', help='read-only')
    b = sub.add_parser('backup', help='read-only')
    b.add_argument('-o', '--output', required=True, help='directory for the files')
    b.add_argument('--dump', action='store_true', help='also write `dump all` to a .dump')
    r = sub.add_parser('run', help='send CLI lines; saves only with --save')
    r.add_argument('commands', nargs='+')
    r.add_argument('--save', action='store_true', help='end with `save noreboot` (default: `exit`, which discards)')
    m = sub.add_parser('msp', help='read-only over MSP; never enters the CLI, never reboots')
    m.add_argument('--samples', type=int, default=10, help='status lines to print (default 10)')
    m.add_argument('--interval', type=float, default=0.2, help='seconds between samples (default 0.2)')
    s = sub.add_parser('restore', help='put a backup file back, pinned')
    s.add_argument('file')
    s.add_argument('--expect-board', required=True)
    s.add_argument('--expect-version', required=True)
    s.add_argument('--save', action='store_true', help='save noreboot and verify (default: validate only)')
    a = ap.parse_args()
    try:
        return {'identify': cmd_identify, 'backup': cmd_backup, 'run': cmd_run,
                'restore': cmd_restore, 'msp': cmd_msp}[a.cmd](a)
    except Refused as e:
        print(f"REFUSED: {e}")
        return 1
    except NoLink as e:
        print(f"NO LINK: {e}")
        return 2


if __name__ == '__main__':
    sys.exit(main())
