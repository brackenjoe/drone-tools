# drone-tools

Small command-line tools for ArduPilot and Betaflight. Each is one Python
file: clone the repo and run it with `python3`, for example
`python3 log-fmt-diff.py OLD.BIN NEW.BIN`. Two need only the standard
library; `ap-ftp-get.py` needs `pip install pymavlink`.

| tool | what it does | needs |
|---|---|---|
| `log-fmt-diff.py` | Diffs the message formats of two ArduPilot dataflash logs (`.bin`), per type and per field: name, format character, unit and multiplier. Flags fields that were added, removed, reordered, retyped or rescaled. The units and multipliers come from each log's own FMT, FMTU, UNIT and MULT records, so the answer is the firmware's, not a table's. | Python 3, stdlib only |
| `ap-ftp-get.py` | Read-only MAVFTP download from an ArduPilot flight controller, checked against the board's own CRC32. Built for slow, deep-buffered links (USB/IP over a VPN, a telemetry radio), where pymavlink's `cmd_get` gives up and then discards every reply. It sends nothing that writes to the vehicle. | Python 3, `pymavlink`; finding the board without `--device` needs Linux (`/dev/serial/by-id`) |
| `bf-settings-diff.py` | Compares the settings tables of two Betaflight builds and classifies each line of a `diff all` backup against the newer one: `same`, `changed`, `missing` or `invalid`, with `by-hand` for lines that are not settings (feature, serial, aux, resource). Offline; it reads text and touches no serial port. | Python 3, stdlib only |

## log-fmt-diff.py

    log-fmt-diff.py OLD.BIN NEW.BIN [--markdown out.md] [--json out.json]
                    [--types MSG,CTUN,...] [--all-types]
                    [--label-old 4.6.3] [--label-new 4.7.1]

By default it compares the types listed in `DEFAULT_TYPES` in the script;
`--all-types` compares every type either log defines. A type present in
one log and not the other is reported, not dropped. Each log is read into
memory and parsed in one pass, so comparing two 300 MB logs needs about
600 MB of RAM.

## ap-ftp-get.py

    ap-ftp-get.py /APM/LOGS/00000011.BIN out/
    ap-ftp-get.py --list /APM/LOGS
    ap-ftp-get.py --last-log out/        # the log LASTLOG.TXT names
    ap-ftp-get.py ... --device /dev/ttyACM0 --baud 115200 --stall 5

A directory given with a trailing `/`, or to `--last-log`, is created if
it does not exist. The tool resets the board's MAVFTP sessions first, so
a transfer another ground station has open on the same vehicle will fail.
The script's docstring explains why pymavlink's own download fails on these
links and what this one does instead.

## bf-settings-diff.py

    bf-settings-diff.py OLD_CAPTURE NEW_CAPTURE [--diff DIFF_ALL_FILE]
    bf-settings-diff.py --selftest

A capture is the text of the CLI's `get a`, `get e`, `get i`, `get o`,
`get u` and `get y` on each firmware (every setting name contains one of
those letters, checked at Betaflight 2026.6.2). Lines that only set context
or carry no setting (`batch`, `defaults`, `save`, `board_name`, `profile`
and the like) are skipped. It exits 0 when every `set` line is `same` and
there is no `by-hand` line, 1 otherwise (including a backup with no `set`
lines), 2 on unreadable input. The script's docstring describes the output.

## Licence

GPL-3.0-or-later. See [LICENSE](LICENSE).
