# drone-tools

Small command-line tools for ArduPilot and Betaflight. Each is one Python
file (`sitl.sh` is one bash file), except that `ap-param-dump.py` needs
`ap-ftp-get.py` beside it, `bf-cli-sitl-check.sh` is `bf-cli.py`'s test, and
`ap-sitl-check.py`, the test of the tools that talk to an ArduPilot board,
needs `mavlink-slowlink.py` beside it. Clone the repo and run a tool
with `python3`, for example `python3 log-fmt-diff.py OLD.BIN NEW.BIN`.
Tested on Linux. `lua/` holds an ArduPilot Lua script, which runs on the
flight controller, not here.

## Requirements

Python 3. Six of the Python tools need only the standard library;
`sitl.sh` and `bf-cli-sitl-check.sh` are bash. The rest need one of these, installed from PyPI; nothing
is bundled here:

| package | for | upstream | licence |
|---|---|---|---|
| `pymavlink>=2.4.50` | the tools that talk to an ArduPilot board | [ArduPilot/pymavlink](https://github.com/ArduPilot/pymavlink) | LGPL-3.0 |
| `pyserial>=3.5` | `bf-cli.py` | [pyserial/pyserial](https://github.com/pyserial/pyserial) | BSD-3-Clause |
| `numpy>=1.26` | `vtol-yaw-metrics.py` | [numpy/numpy](https://github.com/numpy/numpy) | BSD-3-Clause |

    pip install -r requirements.txt

`lua5.3` runs the Lua script's test.

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
| `mavlink-slowlink.py` | TCP proxy that stands in for a slow, deep-buffered MAVLink link between a ground tool and SITL: a rate and a latency, a buffer that outlives a killed client, whole MAVLink frames, and one link fanned out to several SITLs. Importable, with drop hooks, for tests. | stdlib |
| `ap-sitl-check.py` | The test of the tools that talk to an ArduPilot board, against ArduCopter 4.7.1 SITL (see below). | `pymavlink`, a built SITL |
| `ap-version.py` | Prints a vehicle's firmware version and the git hash it was built from, over SITL, telemetry or USB. A dev version string is meaningless alone; the hash is what pins a matching source tree, and the tool prints the `git worktree add` line to make one. Exits 1 if more than one flight controller answers as the same system id. | pymavlink |
| `sitl.sh` | Launches ArduPilot SITL from a version-matched source tree: one git worktree per release beside the main checkout (`ap-Copter-4.7.1`, `ap-plane-<hash>`), chosen by version or hash, the vehicle's own tree first when there is one; prints the tree, version and hash on launch so you know what you are flying. | bash, an ArduPilot checkout with its worktrees, MAVProxy |
| `sitl-flight-test.py` | Smoke-tests a running SITL by flying it, not by checking that a binary exists: copter GUIDED, arm, take off, land (FAIL if still airborne at the timeout); plane TAKEOFF, arm, verify the climb, RTL. Refuses to arm anything that is not SITL (no `SIM_SPEEDUP` parameter). | pymavlink |
| `mavftp-put.py` | Uploads a file to a flight controller over MAVFTP and verifies it against the board's own CRC, pinned to one board by its USB serial, firmware hash and board id like `ap-param-apply.py`, refusing while armed. pymavlink's `cmd_put` returns before any data is sent, so a bare put can leave an empty or corrupted file and report nothing; this pumps the transfer to completion, verifies, and retries. `--selftest` covers the CRC, the identity and armed decisions and the retry logic offline. | pymavlink |
| `ap-mag-cal.py` | ArduPilot's onboard compass calibration driven over MAVLink, pinned to one board by its USB serial, firmware hash and board id like `ap-param-apply.py`: `start` runs it with the offsets not autosaved and prints the full report, `accept` saves them and reads the offsets back per compass, `cancel` discards the fit. `start` and `accept` refuse while the vehicle is armed. `start` itself saves `COMPASS_LEARN=0` and latches a reboot requirement; the tool says so. | pymavlink |
| `ap-heading-watch.py` | Read-only, once a second: the EKF's yaw beside the compass heading computed from the raw field, the field strength, the EKF flags and compass variance, the GPS fix and satellite count (never a position); optional CSV and a gyro-only yaw for a box that stands still. `--selftest` checks the heading arithmetic. | pymavlink (not for `--selftest`) |
| `sitl-qhover-steps.py` | Flies an already-running SITL quadplane or copter tailsitter a hover-only step profile (climb, hands-off hold, yaw steps, roll and pitch steps, land) and writes each phase's start and end in vehicle time, so a log can be cut by phase. Refuses to arm anything that is not SITL, and to fly if the parameters you name are not on the vehicle; fails if the climb does not reach its target. | pymavlink |
| `vtol-yaw-metrics.py` | Yaw-authority metrics off a hover log: does the yaw rate follow its demand, how far does heading drift hands-off, how hard is the yaw controller working, are the motors saturating, and what the CW-minus-CCW motor split answers to (yaw, roll or pitch demand), with the motors named by number as the props are built. `--selftest` runs on synthetic rows. | pymavlink, numpy |
| `bf-cli.py` | Drives a Betaflight flight controller's CLI headlessly over USB serial or SITL: identify, backup, `set` lines read back, and a restore pinned to board and firmware. Never saves unless asked. `msp` reads why the board will not arm over MSP, without entering the CLI. | `pyserial` |
| `bf-settings-diff.py` | Compares the settings tables of two Betaflight builds and classifies each line of a `diff all` backup against the newer one: `same`, `changed`, `missing`, `invalid` or `unchecked`, with `by-hand` for lines that are not settings (feature, serial, aux, resource). Offline; it reads text and touches no serial port. | stdlib |
| `test-fc-sender-checks.py` | Offline test that `ap-version.py`, `ap-param-dump.py`, `ap-param-apply.py`, `ap-mag-cal.py`, `ap-heading-watch.py`, `mavftp-put.py`, `sitl-flight-test.py` and `sitl-qhover-steps.py` take replies only from the autopilot they pinned (see below). | pymavlink |
| `test-param-dump-health.py` | Offline test of `ap-param-dump.py --health` and its parameter-file writer against a fake autopilot that sends hostile data (see below). | pymavlink |
| `lua/gazer_daynight.lua` | ArduPilot Lua script for a Caddx Gazer camera: makes a two-position RC switch select day or night outright, although the camera's input only toggles (see below). | ArduPilot 4.6 or 4.7 with scripting |

Without `--device` or `--port`, `ap-ftp-get.py`, `ap-param-dump.py`,
`mavftp-put.py` and `bf-cli.py` look in `/dev/serial/by-id` (Linux only);
the last three then fall back to `/dev/ttyACM0`. `ap-param-apply.py` and
`ap-mag-cal.py` always take the device. Elsewhere, name the port.

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
a snapshot meant to be shared when you ask for help. It holds the firmware
version and git hash, the board version, the sensor health bits, the RC
channel count, rssi and the first 8 channels, the raw compass readings, the
GPS fix type, satellites and HDOP, and every status message seen during
`--health-secs` (default 60 s). It leaves out the vehicle's position unless you add
`--health-position`. Status messages can carry identifying text, such as a
board serial in a startup banner, so read the file before you post it. Text
from the vehicle is shown with control characters as `\xNN`, so one
message stays one line. If a parameter name or value would not make a valid
file, the run exits 1 and writes nothing. The `.param` and `.defaults` files
are written to a temporary file and then renamed, so an existing one is
never left half-written.

`ap-upgrade-verify.py` exits 1 on any MISMATCH (a value not carried
across) or UNEXPLAINED change outside the table, else 0. A row whose name
is missing from either dump is reported as NOT COMPARABLE and does not
fail the run: on an aircraft without a rangefinder, say, those rows are
expected. Read them. A line whose value is not a number, including `nan`
or `inf`, is skipped and counted, and a nonzero count is printed; it does not
fail the run, so a renamed parameter with such a value shows as NOT
COMPARABLE. A rename table is JSON,
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
`write` checks again just before the first write, and before each one it
looks at the vehicle's latest heartbeats. If the vehicle arms during the
write, it stops, says how many names it wrote and which was next, and
exits 1. It knows the armed state only from heartbeats, about one a second,
so on any link up to about a second of writes can go out after the vehicle
arms (4 measured on a link with 150 ms delay each way, 20 on loopback).
After the last write it checks once more and, if the vehicle is armed or
no heartbeat comes, it warns, says how many writes were made and exits 1.
A write already sent cannot be undone.
`--expect-git` is compared as an 8-character prefix of the firmware hash,
which is shown with anything other than letters and digits as `?`.
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

## ap-sitl-check.py

The test of `ap-ftp-get.py`, `ap-param-dump.py`, `ap-param-apply.py`,
`ap-upgrade-verify.py` and `mavlink-listen.py` against ArduCopter 4.7.1
SITL: 94 checks, about 7.5 minutes, no hardware. It needs ArduCopter SITL
built from ArduPilot's `Copter-4.7.1` tag, and `pymavlink` in the Python
that runs the check (without it the check exits 101). On Ubuntu, as a
normal user with `sudo` (ArduPilot's script refuses root), in the repo
directory:

    git clone --recurse-submodules --branch Copter-4.7.1 https://github.com/ArduPilot/ardupilot
    ardupilot/Tools/environment_install/install-prereqs-ubuntu.sh
    source ~/venv-ardupilot/bin/activate
    (cd ardupilot && ./waf configure --board sitl && ./waf copter)
    python3 ap-sitl-check.py

The prerequisite scripts are in the tree, so the clone comes first;
`Tools/environment_install/` has one per platform. On Ubuntu 24.04 the
script makes a Python venv, `~/venv-ardupilot`, with what the build needs
and `pymavlink`, and asks whether to activate it in every new shell; the
`source` line above is for the default answer, no. Build and run the
check in that venv. These steps are read from ArduPilot's script at the
tag; they have not been run end to end on a fresh machine. ArduPilot's
[BUILD.md](https://github.com/ArduPilot/ardupilot/blob/Copter-4.7.1/BUILD.md)
covers `waf` itself and says not to run it with `sudo`. The clone with
its submodules is about 1.9 GB; the prerequisites and the build add to
that.

It takes the binary from `ardupilot/build/sitl/bin/arducopter` and the
defaults from the same tree; `AP_SITL` and `AP_DEFAULTS` point it
elsewhere. Its exit status is the number of failed checks, at most 100.
It exits 101, with a `cannot run:` line, if the SITL's git hash is not
`dbe79216`, something holds TCP 5820-5823, 5830-5833 or 15820-15829, or a
SITL dies during a case. It stops only the SITLs it started. `--tools DIR` runs another copy of the tools, which is how
an older version can be shown to fail.

The tools reach SITL through `mavlink-slowlink.py`: a 1800 B/s, 150 ms
link, raised to 18,000 B/s (about a real USB/IP link's rate) for one case,
and an unlimited one that drops chosen frames. Parameter reads and arming go to SITL
directly, so every result is checked against what the board holds. The proxy
has no authentication and the newest client takes its slot, so keep `--bind`
on loopback (it warns otherwise). What it covers:

- `ap-ftp-get.py`: a listing that pages, a CRC-checked fetch, a burst reply
  dropped on purpose and fetched again, `--last-log`, and a rerun through
  replies still in flight from a killed transfer.
- `ap-param-dump.py`: MAVFTP with defaults over the slow link, a killed and
  rerun dump, the `PARAM_REQUEST_LIST` fallback, a short `--no-ftp` read
  exiting 1, and `--health` leaving the position out unless asked.
- `ap-upgrade-verify.py` on SITL dumps: "nothing unexplained", UNEXPLAINED,
  carried across and MISMATCH, each with its exit code.
- `ap-param-apply.py`: identify, plan, a write read back, `never-write.txt`,
  a name the firmware lacks, each wrong pin, and arming before a write
  (slow link) and during one (both links).
- Two SITLs at one system id on one link: `ap-param-apply.py` and
  `ap-param-dump.py` refuse, and neither board changes.
- `mavlink-listen.py` sends nothing.

What it cannot show: SITL does not pace MAVFTP bursts the way a board on a
serial port does (SITL's UART reports flow control), so a killed transfer
leaves the whole file in flight. `reboot` is not covered. The arm during a
write is a race: on the 150 ms link the tool stops partway and says how
far it got; on loopback the loop can finish first, and the check asserts
that the tool then exits 1 with a warning, not how many writes landed
armed.

## SITL: sitl.sh, sitl-flight-test.py, ap-version.py

    AP_TREES=/path/to/trees ./sitl.sh 4.7.1 ArduCopter --no-mavproxy   # Copter 4.7.1
    ./sitl.sh 4.7.1 ArduPlane --no-mavproxy                             # Plane: ap-Plane-4.7.1 if it exists, else the Copter tree
    ./sitl.sh dbe79216 ArduCopter --no-mavproxy                         # a commit
    python3 sitl-flight-test.py [--vehicle copter|plane] [--device tcp:127.0.0.1:5760]
    python3 ap-version.py [tcp:127.0.0.1:5760 | udp:0.0.0.0:14550 | /dev/ttyACM0 --baud 115200]

`sitl.sh` expects one git worktree per version beside the main ArduPilot
checkout, named `ap-<Vehicle>-<version>` or `ap-<vehicle>-<hash>`, under
`AP_TREES` (default: the directory the script is in). ArduPilot tags Copter
and Plane releases on one commit when they ship together, so one worktree
built with both `./waf copter` and `./waf plane` serves both vehicles; do not
add a second worktree for the same commit. Everything after the vehicle
passes through to `sim_vehicle.py`. `AP_VENV` names a venv with MAVProxy
and pymavlink to put first on `PATH`. Never pass `--map` or `--console` on a
headless machine. SITL's first serial port takes one TCP client, and
`sim_vehicle.py` gives it to MAVProxy unless you pass `--no-mavproxy`: with
MAVProxy running, point the tools at `tcp:127.0.0.1:5762` instead.
`sitl-flight-test.py` arms and takes off, so it reads `SIM_SPEEDUP` back
first and refuses, exit 2, when the parameter is absent: a real vehicle
has none, unless it runs one of ArduPilot's Sim-on-Hardware builds, which
have `SIM_*` parameters and pass this check. It pins the first autopilot heartbeat and takes every reply,
`SIM_SPEEDUP` and the arm ACK included, from that system and component
only, and it arms only after a heartbeat confirms the mode (GUIDED, or
TAKEOFF on a plane): an unconfirmed mode is exit 1. No heartbeat is a
failure at once. `ap-version.py` reads
`AUTOPILOT_VERSION` and prints the version, the git hash, the board id (as
the other tools' `--expect-board` takes it), the uid, and the
`git worktree add` line that pins a tree to that hash. A hash that is not
hex is printed escaped, with no `git worktree add` line, and the exit is 1.

## mavftp-put.py

    python3 mavftp-put.py LOCAL /APM/scripts/script.lua --expect-uid <24 hex> --expect-git <hash> --expect-board <id> [--device DEV] [--mkdir] [--retries N]
    python3 mavftp-put.py LOCAL REMOTE --verify-only
    python3 mavftp-put.py --selftest

pymavlink's `MAVFTP.cmd_put` returns as soon as it has sent `CreateFile`,
before any data block is pumped, so a bare put silently writes an empty
file, and a directory listing interleaved with the writes corrupts the
first bytes; both were seen on hardware, with no error reported. This tool
pumps the transfer to completion, then asks the board for its CRC
(`CalcFileCRC32`, ArduPilot's reflected 0xEDB88320 with init 0, which is not
zlib's) and compares; a mismatch, an empty result or a missing CRC reply is
retried up to `--retries` times, and the exit is 0 only on VERIFIED. This
writes to a flight controller: the system and component id are pinned from
the first autopilot heartbeat, the three `--expect-*` values (from
`ap-version.py`) must all match or nothing is sent, a second board at the
same system id is a refusal, and an armed vehicle, or no heartbeat in 3 s,
refuses before `--mkdir` and before every attempt. Each attempt removes
the remote file first, so a failed upload leaves the board without it; the
failure message says so. `--verify-only` needs none of the `--expect-*`
values. The default device is the first ArduPilot USB serial device, else
`/dev/ttyACM0`. The selftest checks the CRC vector, the identity and armed
decisions, the retry and verify logic, and the FTP reply filter against
a fake board. MAVFTP replies count only from the pinned system and
component. When one CRC request gets two different answers the tool asks
once more and takes only a single clean answer; otherwise the attempt fails.

## ap-mag-cal.py

    python3 ap-mag-cal.py start  DEV --expect-uid <24 hex> --expect-git <hash> --expect-board <id> [--timeout 300] [--log cal.txt]
    python3 ap-mag-cal.py accept DEV --expect-uid ... --expect-git ... --expect-board ...
    python3 ap-mag-cal.py cancel DEV --expect-uid ... --expect-git ... --expect-board ...
    python3 ap-mag-cal.py --selftest

This writes to a flight controller. `start` sends `MAV_CMD_DO_START_MAG_CAL`
for every compass in use (mask 0: the board skips an unhealthy compass and
one with `COMPASS_USEn=0`, and refuses outright under `COMPASS_LEARN=2`),
prints the mask it started, every compass's progress on one line at most every
2 s while progress arrives (a finished compass by its report's status), and then the report in full
(fitness, offsets, diagonals, off-diagonals, scale, the orientation check
with its confidence), and cancels on `--timeout`. The fit the board
accepts is `COMPASS_CAL_FIT` for an external first compass and twice that
for the others. Two things `start` changes at once, whatever happens
after: the board saves `COMPASS_LEARN=0` (printed before and after; `cancel`
does not put it back) and, on the first start since boot, refuses to arm
until it reboots ("Compass calibrated requires reboot"). The calibrated
offsets are not saved until `accept` sends `MAV_CMD_DO_ACCEPT_MAG_CAL`; a
poor report is thrown away with `cancel`. The board's ACK to `accept` is
not the truth (it says accepted when nothing was started, and failed when
one compass was saved and another was not), so the tool reads
`COMPASS_OFS*` back for each compass and exits 0 only when at least one
changed, naming which. The progress and report messages are sent only on
request: the tool asks for them with `SET_MESSAGE_INTERVAL` before `start`
(reset when it ends), falls back to the EXTRA3 stream rate if the board
refuses, and refuses to start if neither is accepted. Identity is checked
as `ap-param-apply.py` does: the three `--expect-*` values must all match
the board or nothing is sent, and a second board answering at the same
system id is a refusal (a board answering after the 3 s window, or two
boards with the same uid, is not caught). `start` and `accept` refuse while
the vehicle is armed, judged from a fresh heartbeat read after the
preparatory reads, just before the command is sent, and watched during the
ACK wait and any re-send; no heartbeat in 3 s is
also a refusal. `cancel` is allowed while armed. `--log` receives
everything printed, refusals included. The selftest covers the report
formatting, the progress line, the message request and its fallback, the per-compass
readback verdict, and the armed check against a fake link.

## ap-heading-watch.py

    python3 ap-heading-watch.py DEV [--secs N] [--csv FILE] [--gyro-from DEG]
    python3 ap-heading-watch.py --selftest

Read-only. Each line: the EKF yaw (`ATTITUDE`), the compass heading
computed here from the primary compass's field (`RAW_IMU`, tilt-compensated
with the attitude and corrected by the board's live `COMPASS_DEC`, the same
arithmetic as `Compass::calculate_heading()`), their difference, the field
strength, roll and pitch, declination, the EKF flags and compass variance,
the GPS fix type and satellite count (never a position), the `ATTITUDE`
rate and the compass health. The compass heading moves as soon as the field
does, where the EKF's yaw is filtered, so the pair shows a compass problem
before the EKF reports one. The board must stream `ATTITUDE` and `RAW_IMU`
on the link; a fresh SITL streams nothing until asked. The selftest checks
the heading arithmetic against hand-computed cases and an independent
rotation matrix.

## Yaw authority: sitl-qhover-steps.py and vtol-yaw-metrics.py

    python3 sitl-qhover-steps.py [--device tcp:127.0.0.1:5760] --check-params airframe.parm --phases-out phases.json [--alt 10] [--hold-s 20]
    python3 vtol-yaw-metrics.py LOG.bin --cw 2,3 --ccw 1,4 [--phases phases.json] [--json out.json]
    python3 vtol-yaw-metrics.py --selftest

The question: can the rate controller follow a yaw step and hold heading
hands-off, and what does the motor split answer to. The profile flies an
already-running SITL quadplane or copter tailsitter in QHOVER only (no
transitions, no mission): arm on the ground, climb to `--alt`, hands-off
hold, yaw stick steps each way, then roll and pitch steps, each followed by
centred sticks, then land. It refuses to arm anything without a
`SIM_SPEEDUP` parameter (from the pinned autopilot only), refuses to arm if
QHOVER is not confirmed, refuses a `--check-params` file with any line that
is not `NAME VALUE` (or no parameter at all), refuses to fly if any of its
parameters is missing or differs on the vehicle, removes an old
`--phases-out` file before it starts, lands and fails if
the climb does not reach `--alt` in time, and, only after a landing,
writes each phase's start and end in the vehicle's own boot time. The
metrics tool reads the dataflash log (`.bin`, or a text `.log`, opened as a
log and never handed to pymavlink's connection logic), cuts it by those phases (or `--window`), and
reports the yaw-rate tracking per step, the hands-off heading drift, the
yaw controller's effort, motor saturation, and a regression of the
CW-minus-CCW motor split (motors named by number as the props are built,
`Motor1` = `SERVOn_FUNCTION` 33) against the roll, pitch and yaw outputs: a
split that loads on the yaw output means differential torque is the yaw
actuator; one that loads on roll means a roll demand also moves the
yaw-axis split. Positions are never read. Needs numpy.

## bf-cli.py

    bf-cli.py identify
    bf-cli.py backup -o DIR [--dump]
    bf-cli.py run "set craft_name = MYQUAD" [--save]
    bf-cli.py restore FILE --expect-board TARGET --expect-version 2026.6.2 [--save]
    bf-cli.py msp [--samples 10] [--interval 0.2]
    bf-cli.py --port socket://127.0.0.1:5761 identify      # Betaflight SITL

Exit 0 ok; 1 refused, a read-back mismatch, a CLI error, or a backup
written with firmware warnings; 2 no link or a timeout.

`run` refuses `save`, `defaults`, `bl`, `dfu`, `exit`, `flash_erase`,
`msc`, `motor`, `escprog`, `serialpassthrough`, `gpspassthrough`,
`dshotprog` (programs the ESCs), `flash_write` (writes the flash chip, on
builds with flash tools), `dxw` (writes RAM; debug builds only) and
`bind_rx` (puts the receiver into bind mode, which can drop the RC link).
The Betaflight CLI edits a line before running it. It ends the line at a
newline, drops other control characters, treats DEL as backspace,
completes a command on `?` or tab, exits on Ctrl-D at the start of a line,
and cuts the line at `#`. So `run` and `restore` refuse any line that holds
a character outside printable ASCII, a `?` or a `#` (a whole-line `#`
comment in a file is skipped), before anything is sent. Then the first word
is the command the board will run. `restore` also accepts only the commands
a `diff all` contains (`set`, `feature`, `serial`, `aux`, `resource`,
`mixer` and the like), so a `dump`, a hand-edited file or a file with other
commands is refused whole. It does not judge the settings themselves: a
file of plain `set` and `resource` lines from someone else can still
reconfigure motors, ports and failsafe, and `--save` keeps it, so read a
file before you restore it. The board and version check compares the
file's own header, which is plain text. This behaviour of the CLI was read
from Betaflight 2026.6.2's source and measured on its SITL, not on a board;
the cut at `#` was read from the source only. Older firmware was not
checked, and a diff from another version may hold a command that is not on
the list; it is then refused. Text from the board is printed, and written into
backup headers, with control characters as `\xNN`. A backup line holding
bytes that are not printable ASCII is not kept: it is written as a comment,
and `backup` exits 1, as for a firmware error line (the backup `restore`
takes first prints the warning and goes on). `bf-cli.py --selftest`
runs the gate's cases offline.
`restore` first backs up the current config beside the given file.
On firmware before 2025.12 every command ends in a reboot, because `exit`
always reboots there; the tool reads the version and says so.

`msp` answers "why will it not arm" without that reboot. It is read-only
and speaks MSP, never the CLI: it prints the firmware variant, version,
board and craft name, the mode ranges, then `--samples` lines `--interval`
seconds apart with the arming-disable flags, RSSI and the RC channels, a
summary, and which modes the last RC frame switches on. It is MSPv1 only
and sends eleven read requests with no payload (API_VERSION, FC_VARIANT,
FC_VERSION, BOARD_INFO, NAME, RC, ANALOG, BOXNAMES, BOXIDS, MODE_RANGES,
STATUS_EX); its one send function refuses any other id. A board that is in
CLI mode ignores MSP: with no reply to the first request `msp` exits 2 and
sends nothing more. To leave CLI mode, power-cycle the board or run
`identify`; on firmware before 2025.12 `identify` reboots it too. The flag
names come from the count the board reports: 26 flags is read as the
Betaflight 4.5 table, 30 as the 2026.6.2 table, and any other count prints
`bit N` in place of names; 2025.12.1 has 29, so it gets `bit N`. 4.3 and
4.4 also report 26 but name bit 3 `BADRX` and bit 20 `RPMFILTER`, so those
two print as `NOT_DISARMED|BADRX` and `DSHOT_TELEM|RPMFILTER`. An MSP error reply is exit 1.
The checks run against 2026.6.2 SITL; the 26-flag table was read from
Betaflight 4.5.3's source.

`bf-cli-sitl-check.sh` is its test, 195 checks against Betaflight SITL (194
without shellcheck; no hardware). It needs bash, python3 with pyserial, `setsid`, `pgrep` and `sha256sum`;
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

## Over USB/IP from a phone

A flight controller in the field can be plugged into an Android phone and
shared with a Linux host over a VPN, using
[usbip-dialin](https://github.com/brackenjoe/Android-Usbipdcpp/tree/main/linux/usbip-dialin)
and the [Android-Usbipdcpp fork](https://github.com/brackenjoe/Android-Usbipdcpp)
it ships with. The phone dials out and the host never connects to the
phone. On the host the board is an ordinary USB serial device, so the
tools run unchanged with `--device /dev/serial/by-id/...-if00`. Such a
link is slow and deep-buffered. `ap-ftp-get.py` was built for it, and
`ap-param-dump.py` uses its transfer; the other ArduPilot tools work over
it as over a cable.

Measured over that link:

- 2026-10-20, a TBS Lucid H7 Wing on ArduPlane 4.8.0-dev (`8555d941`),
  `ap-ftp-get.py` as it was then: about 18 KiB/s (17.7 to 18.9 over five
  log transfers). A 5 MB log took about 4.5 minutes and its CRC matched.
  After a transfer was killed on purpose, 798 replies from it were still
  arriving. The next run drained them in 13.4 s and carried on.
  pymavlink 2.4.49's own MAVFTP download failed in four of four attempts
  on one log: its 1 s open retry changes session and throws away the
  replies that follow.
- 2026-11-03, a TBS Lucid H7 on ArduCopter 4.7.1, the tools as published
  here: `ap-param-dump.py --defaults --health` read 1301 parameters and
  the board's defaults in 76 s, 60 s of it the health window.
  `ap-ftp-get.py --list` paged through 32 entries in 5.2 s.

**In use.** On 2026-10-22 an ArduPlane aircraft (a Skyhunter 1800, the
Lucid H7 Wing above) was tuned in three flights in one morning this way,
with nothing on the flight controller but the phone. The work on the host
was done by a Claude Code session, driven by Remote Control from the
Claude app on the same phone that carried the USB/IP connection. Between
flights the pilot plugged the phone in, started the app's server and told
the session the aircraft was up. The session pulled the last flight's log
with `ap-ftp-get.py`, analysed it and proposed the next parameters. On the
pilot's yes it wrote them with `ap-param-apply.py`, pinned to the board's
uid and firmware, and read each one back. Then it said what to fly next.
The third flight ran
AUTOTUNE, which saves its own gains. Not everything went cleanly that
day. After a 400 s timeout of an earlier `ap-param-dump.py`, the board's
MAVFTP stopped answering, even to `--list`, while parameter reads still
worked, and a power cycle cleared it. pymavlink 2.4.49 crashed once at
connect; 2.4.50 fixes that.

What it does not do:

- No firmware flashing. A board that re-enumerates as its bootloader
  drops the connection.
- No MAVLink signing. The tools take replies by system and component id,
  which filters other devices on a link but does not authenticate them.
  Who may attach a device at all is usbip-dialin's job, in layers. It
  always checks the peer's VPN identity: the user is required, a single
  device optional. With `--psk-file` it checks a key, which the fork's
  app keeps behind a fingerprint. It authorises every device except
  keyboards and other input devices, network adapters, Bluetooth and,
  for now, audio. Its README has the details.

## Offline tests: test-fc-sender-checks.py, test-param-dump-health.py

    python3 test-fc-sender-checks.py      # 46 cases, about 5 minutes
    python3 test-param-dump-health.py     # 8 cases

Each case runs one tool as a subprocess against a fake MAVLink peer on
loopback, so neither test needs a board or SITL. In most cases the fake
sends, alongside the real reply from the autopilot (system 1, component 1),
a forged one from another component or another system, and the case
checks that the tool shows the real value, or refuses; the pin cases send
an autopilot-looking heartbeat from (0,0) first. A tool that does not check the
sender shows the forged value. The cases also cover a vehicle that arms
partway through a write, a missing heartbeat after a write, and text from
the vehicle carrying terminal escapes or newlines. `test-param-dump-health.py`
checks that the health file holds no position unless `--health-position`
is given (it looks for the field names and never prints a value), and that
a bad parameter name or a NaN value is refused without touching an
existing file. Both take `--tools DIR` to run the same cases against
another copy of the tools, which is how to show that a case fails on a
version without the check. Each prints PASS or FAIL per case and
`N passed, M failed`, and exits 1 on any failure.

## lua/gazer_daynight.lua

The Caddx Gazer's day/night input is a trigger: each pulse switches the
camera to the mode it is not in, and there is no command for "day" or for
"night". Wiring an RC switch to it through a relay therefore never gives a
switch position that means one mode. The camera also powers up in day
mode every time; it does not remember night across a power cycle. The
script keeps its own model of the camera's mode, compares it with the
switch, and sends one pulse when they disagree. By default it also follows
the camera's power rail on relay instance 3 (RELAY4): while the rail is off
the model is held at day, and when it comes on the script waits for the
camera to boot before bringing it to the switch position.

Built and measured on a TBS Lucid H7 with the day/night pad on output M9,
ArduCopter 4.6.3 and 4.7.1. The channel, the relay instances and the
timings are in the tuning block at the top of the script; the header has
the parameters to set and what was measured. If the camera's power has no
relay, set `RAIL_RELAY = -1`: otherwise the script reads the rail as off,
holds day and never pulses, silently. Copy it to `APM/scripts/` on
the board's SD card and reboot.

    lua5.3 lua/gazer_daynight_test.lua     # stubbed ArduPilot environment

The test is for Lua 5.3, the version ArduPilot uses; 5.4 accepts syntax
that 5.3 rejects.

## Licence

GPL-3.0-or-later. See [LICENSE](LICENSE). The dependencies keep their own
licences (above); none is included here.
