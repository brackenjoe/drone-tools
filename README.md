# drone-tools

Small command-line tools for ArduPilot and Betaflight. Each is one Python
file, except that `ap-param-dump.py` needs `ap-ftp-get.py` beside it and
`bf-cli-sitl-check.sh` is `bf-cli.py`'s test. Clone the repo and run a tool
with `python3`, for example `python3 log-fmt-diff.py OLD.BIN NEW.BIN`.
Tested on Linux.

## Requirements

Python 3. Five tools need only the standard library. The rest need one of
these, installed from PyPI; nothing is bundled here:

| package | for | upstream | licence |
|---|---|---|---|
| `pymavlink>=2.4.50` | the tools that talk to an ArduPilot board | [ArduPilot/pymavlink](https://github.com/ArduPilot/pymavlink) | LGPL-3.0 |
| `pyserial>=3.5` | `bf-cli.py` | [pyserial/pyserial](https://github.com/pyserial/pyserial) | BSD-3-Clause |

    pip install -r requirements.txt

Use pymavlink 2.4.50 or later: 2.4.49 can crash at connect with a
`TypeError` in `mavutil.add_message` when a link switches from MAVLink 1
to 2 (fixed upstream in 2.4.50).

## The tools

| tool | what it does | needs |
|---|---|---|
| `log-fmt-diff.py` | Diffs the message formats of two ArduPilot dataflash logs (`.bin`), per type and per field: name, format character, unit and multiplier. Flags fields that were added, removed, reordered, retyped or rescaled. The units and multipliers come from each log's own FMT, FMTU, UNIT and MULT records, so the answer is the firmware's, not a table's. | stdlib |
| `param-diff.py` | Diffs ArduPilot parameter metadata between two source trees: added, removed, likely renamed, and changes of range, allowed values or increment (not of defaults). Pairs renames by display name, which is a heuristic; ambiguous pairs are listed for you to resolve. | stdlib, two ArduPilot source trees |
| `ap-param-dump.py` | Dumps a board's parameters and its own defaults over MAVFTP. The defaults are what tell "this setting was lost" from "this parameter's default moved". Read-only. | `pymavlink` |
| `ap-upgrade-verify.py` | Offline: checks that a firmware upgrade carried every setting across, from dumps taken before and after and a rename table. Tells a failed conversion from a default that moved under an unstored parameter. | stdlib |
| `ap-param-apply.py` | Writes parameter files to one flight controller, pinned by its unique id, firmware hash and board id, and reads every write back. Refuses a name the firmware does not have. | `pymavlink` |
| `ap-ftp-get.py` | Read-only MAVFTP download from an ArduPilot flight controller, checked against the board's own CRC32 (a transport check, not proof of where the bytes came from). Built for slow, deep-buffered links (USB/IP over a VPN, a telemetry radio), where pymavlink's `cmd_get` gives up and then discards every reply. It sends nothing that writes to the vehicle. | `pymavlink` |
| `mavlink-listen.py` | Listen-only MAVLink check over TCP or UDP: prints the autopilot's heartbeat and a count per message id. Never sends MAVLink. | stdlib |
| `bf-cli.py` | Drives a Betaflight flight controller's CLI headlessly over USB serial or SITL: identify, backup, `set` lines read back, and a restore pinned to board and firmware. Never saves unless asked. | `pyserial` |
| `bf-settings-diff.py` | Compares the settings tables of two Betaflight builds and classifies each line of a `diff all` backup against the newer one: `same`, `changed`, `missing`, `invalid` or `unchecked`, with `by-hand` for lines that are not settings (feature, serial, aux, resource). Offline; it reads text and touches no serial port. | stdlib |

Without `--device` or `--port`, `ap-ftp-get.py`, `ap-param-dump.py` and
`bf-cli.py` look in `/dev/serial/by-id` (Linux only); the last two then fall
back to `/dev/ttyACM0`. `ap-param-apply.py` always takes the device.
Elsewhere, name the port.

`--help` gives each tool's options and a one-paragraph summary; the
script's docstring says what it does and why in more detail.

## Upgrading ArduPilot: param-diff, ap-param-dump, ap-upgrade-verify

    param-diff.py OLD_TREE NEW_TREE [--vehicle ArduCopter]
    ap-param-dump.py before.param --defaults before.defaults [--device DEV]
    # flash the new firmware, let it boot once
    ap-param-dump.py after.param --defaults after.defaults [--device DEV]
    ap-upgrade-verify.py before.param after.param \
        --renames maps/copter-4.6.3-to-4.7.1.json \
        --pre-defaults before.defaults --post-defaults after.defaults

`param-diff.py` generates `apm.pdef.xml` in a tree that lacks one, and
refuses a cached file generated for another vehicle. Its output is a list of
name pairs, not a rename table: it has no conversion factors.

`ap-param-dump.py` reads `@PARAM/param.pck?withdefaults=1` with the same
transfer as `ap-ftp-get.py`, so keep the two files side by side. It resets
the board's MAVFTP sessions first, so a transfer another ground station has
open on the same vehicle will fail. Without MAVFTP it falls back to
`PARAM_REQUEST_LIST`, says so, and writes no defaults; if that read comes
back short, it still writes the file and exits 1. It picks its board the
same way as `ap-param-apply.py` (below), and refuses before reading anything
if two boards answer at one system id, since it would mix their
parameters. `--health FILE` adds
a snapshot of sensors, GPS, RC and the PreArm checks.

`ap-upgrade-verify.py` exits 1 on any MISMATCH (a value not carried
across) or UNEXPLAINED change outside the table, else 0. A row whose name
is missing from either dump is reported as NOT COMPARABLE and does not
fail the run: on an aircraft without a rangefinder, say, those rows are
expected. Read them. A rename table is JSON,
`{"renames": [{"old": ..., "new": ..., "factor": ...}]}`, with
`new = old * factor`. Take factors from the firmware's conversion code, not
from the metadata units field: between Copter 4.6.3 and 4.7.1, five
position-controller rows change scale with no change of unit.

`maps/copter-4.6.3-to-4.7.1.json` is such a table, 102 rows. The `how`
field of each says how it was established: the conversion table it was read
from (file and line), a conversion in place, or a rename in place (same
storage slot, so factor 1). Three rows (helicopter and SITL only) were not
checked and say so. It was used to verify two real upgrades.

## ap-param-apply.py

    ap-param-apply.py identify DEVICE
    ap-param-apply.py plan  DEVICE --expect-uid UID --expect-git HASH --expect-board ID a.param [b.param ...]
    ap-param-apply.py write DEVICE --expect-uid ... --expect-git ... --expect-board ... [--log out.tsv] a.param
    ap-param-apply.py reboot DEVICE --expect-uid ... --expect-git ... --expect-board ...

`identify` and `plan` are read-only; `plan` prints what `write` would
change. `write` prints every change and logs every line with `--log`.
`write` and `reboot` refuse while armed, and on any mismatch of the pins.
`--expect-git` is compared as an 8-character prefix of the firmware hash.
A `never-write.txt` beside a parameter file lists names or patterns that
no file may set.

**Which board it talks to.** It uses the first heartbeat it hears that is
not from a ground station and not marked as a non-autopilot (as gimbals,
cameras and companion computers should mark themselves), and it takes
replies only from that system and component id. That is not a choice of
board: with several autopilots on one link, which one a run reaches depends
on timing. Behind a router, always pass `--expect-uid` (the board's unique id,
`AUTOPILOT_VERSION.uid2`: the MCU id on STM32 boards; `identify` prints it), so that a wrong pick is
refused, and run again. `identify`, and `plan` without pins, simply report
the board they reached.

**Two boards with one system id.** The uid pin checks the identity in the
first reply, but a write is addressed by system id. Behind a router, two
boards at the same id both take it (measured with mavlink-router v4 and
ArduCopter 4.7.1 SITL). So every subcommand asks for identity, listens for
3 s, and refuses if more than one uid answers. The refusal catches two
boards that both answer within those 3 s at the start of the run. It does
not catch a board that answers late or joins afterwards, or two boards
reporting the same uid. That last case covers Linux-based boards cloned
from one image, and boards that report a uid of all zeros. Nor does it
stop a hostile node: without MAVLink signing, which this tool does not use,
any sender can claim a board's ids. It guards against mistakes. Give each
flight controller its own system id (`MAV_SYSID` from Copter 4.7,
`SYSID_THISMAV` before).

## ap-ftp-get.py

    ap-ftp-get.py /APM/LOGS/00000011.BIN out/
    ap-ftp-get.py --list /APM/LOGS
    ap-ftp-get.py --last-log out/        # the log LASTLOG.TXT names
    ap-ftp-get.py ... --device /dev/ttyACM0 --baud 115200 --stall 5 --max-mib 2048
    ap-ftp-get.py --selftest             # offline, stdlib only

A directory given with a trailing `/`, or to `--last-log`, is created if
it does not exist. The tool resets the board's MAVFTP sessions first, so
a transfer another ground station has open on the same vehicle will fail.
The script's docstring explains why pymavlink's own download fails on these
links and what this one does instead.

It takes replies only from the system and component of the first autopilot
heartbeat it sees on the link, and only when they are addressed to it. That
filters out other devices on a shared link (a ground station, a gimbal, a
companion computer), but any sender can claim those ids, so it is not
authentication. It stores a chunk only if the chunk lies inside the size the
board announced, starts on a 239-byte boundary and carries as many bytes as
its header says, and it refuses a file larger than `--max-mib` before
allocating memory. `VERIFIED` means the board's CRC32 matches the bytes
received. That catches a corrupted transfer, but the CRC comes over the same
link, so it does not prove the source. MAVLink signing would, and this tool
neither signs nor checks signatures. On a CRC mismatch, or no CRC reply at
all, the bytes are kept beside the target as `.crc-mismatch`. File
names in `--list` output are printed with control characters shown as
`\xNN` text.

## mavlink-listen.py

    mavlink-listen.py tcp HOST:PORT [--seconds S] [--retry S]
    mavlink-listen.py udp-bind HOST:PORT [--seconds S]
    mavlink-listen.py udp-probe HOST:PORT [--seconds S] [--expect-none]

`udp-probe` sends one datagram of plain ASCII that no parser frames as
MAVLink, so a router that takes any sender as its peer starts streaming
back. Exit 0 on an autopilot heartbeat (or on silence with
`--expect-none`), 1 otherwise, 2 on a usage or connection error.

## bf-cli.py

    bf-cli.py identify
    bf-cli.py backup -o DIR [--dump]
    bf-cli.py run "set craft_name = MYQUAD" [--save]
    bf-cli.py restore FILE --expect-board TARGET --expect-version 2026.6.2 [--save]
    bf-cli.py --port socket://127.0.0.1:5761 identify      # Betaflight SITL

Exit 0 ok; 1 refused, a read-back mismatch, a CLI error, or a backup
written with firmware warnings; 2 no link or a timeout.

`run` refuses `save`, `defaults`, `bl`, `dfu`, `exit`, `flash_erase`,
`msc`, `motor`, `escprog`, `serialpassthrough` and `gpspassthrough`.
`restore` first backs up the current config beside the given file.
On firmware before 2025.12 every command ends in a reboot, because `exit`
always reboots there; the tool reads the version and says so.

`bf-cli-sitl-check.sh` is its test, 92 checks against Betaflight SITL (91
without shellcheck; no hardware). It needs bash, python3 with pyserial, `setsid` and `pgrep`;
`shellcheck` is used if installed. Build SITL in the repo directory and run:

    git clone --depth 1 --branch 2026.6.2 https://github.com/betaflight/betaflight
    make -C betaflight TARGET=SITL
    ./bf-cli-sitl-check.sh

or point `BF_SITL` at a `betaflight_SITL.elf` built elsewhere. It refuses to
start if something already holds TCP 5761. SITL's TCP serial sometimes
garbles or drops a long read (`dump all` most often): in our runs about one
run in three failed one or more of the long-read checks, and a rerun passed.

## bf-settings-diff.py

    bf-settings-diff.py OLD_CAPTURE NEW_CAPTURE [--diff DIFF_ALL_FILE]
    bf-settings-diff.py --selftest

A capture is the text of the CLI's `get a`, `get e`, `get i`, `get o`,
`get u` and `get y` on each firmware (every setting name contains one of
those letters, checked at Betaflight 2026.6.2). Lines that only set context
or carry no setting (`batch`, `defaults`, `save`, `board_name`, `profile`
and the like) are skipped. It exits 0 when every `set` line is `same` and
there is no `by-hand` line, 1 otherwise (including a backup with no `set`
lines), 2 on unreadable input. A setting whose limits cannot be read from
the newer capture (an `Allowed range`, `String length` or `Array length`
line that does not parse, or no limits at all) is `unchecked` (or `changed`,
if its limits also differ between the captures), never `same`. Where the
newer capture gives a numeric `Allowed range`, a value that is not a plain
decimal number (`1_0`, `inf`) is `invalid`. Lines end at `\n` (one trailing
`\r` is dropped, so CRLF backups parse) and fields are separated by space or
tab, not by other Unicode spaces. Text from the files is printed with control
characters shown as `\xNN`, so a capture cannot move the cursor or send
terminal escape sequences. The script's
docstring describes the output.

## log-fmt-diff.py

    log-fmt-diff.py OLD.BIN NEW.BIN [--markdown out.md] [--json out.json]
                    [--types MSG,CTUN,...] [--all-types]
                    [--label-old 4.6.3] [--label-new 4.7.1]
    log-fmt-diff.py --selftest

By default it compares the types listed in `DEFAULT_TYPES` in the script;
`--all-types` compares every type either log defines. A type present in
one log and not the other is reported, not dropped. Each log is read into
memory and parsed in one pass, so comparing two 300 MB logs needs about
600 MB of RAM. Malformed records (a format that declares an impossible
length, or a FMTU, UNIT or MULT record of the wrong size) are skipped, not
trusted. Strings from the log are escaped on the terminal and in the Markdown
report, so a log someone sent you cannot send terminal escape sequences or
add Markdown headings, tables or inline links. A bare URL in the log
stays a bare URL, and GitHub will still make it clickable.

## Licence

GPL-3.0-or-later. See [LICENSE](LICENSE). The dependencies keep their own
licences (above); none is included here.
