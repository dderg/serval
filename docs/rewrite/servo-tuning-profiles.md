# Servo tuning profiles

> See also:
> [`servo-calibration.md`](servo-calibration.md) for the sweep commands that
> produce a tuning session, and
> [`servo-cal-contracts.md`](servo-cal-contracts.md#tuning-profile) for the
> binding file-format contract.

## Why

A calibration session's outcome — the gain set, the inertia ratio, any notch
registers dialed in along the way — lives in the drives' volatile SDO
registers. A power cycle, a drive swap, or reconnecting after a fault erases
it silently; `printer.cfg` never saw the values in the first place. Tuning
profiles are the write-back half of this loop, the read side being the
existing `[motor] params:` block (`servo_param.parse_params_block`, pushed at
claim time with readback verification by
`ethercat_node._push_drive_params`).

## Config surface

```
[motor motor_a]
protocol: ethercat
node: node_a
...
tuning_profile: gains_20260710
params:
  0x2001.0x31: u16 1
```

`tuning_profile: <name>` resolves to
`~/printer_data/config/servo_tuning/<name>.params`. Its entries are pushed
through the same claim-time path as `params:`, in the same order every other
SDO write goes through — **profile entries first, then `params:`** — with the
same per-entry readback verification. A missing profile file is a config
error at parse time (fail loud, no silent skip).

An address set by both the profile and `params:` is a config error naming
the address and both sources — not a precedence rule to remember, one line to
delete.

## File format

Same line syntax as a `[motor] params:` block
(`0xINDEX.SUB: [type] value`, see `servo_param.parse_param_entry`), plus `#`
comment lines carrying provenance:

```
# tuning profile: gains_20260710
# created_utc: 2026-07-10T15:17:02Z
# servo: motor_a
# source: drive readback (SERVO_SAVE_TUNING)
0x2001.1: u16 700
0x2001.2: u16 550
0x2001.3: u16 2273
0x2000.7: u16 150
```

Comment lines and blank lines are skipped by the parser; everything else must
parse as a param entry.

## `SERVO_SAVE_TUNING`

```
SERVO_SAVE_TUNING SERVO=<motor> NAME=<profile> [ADDRS=<addr[:type]>,...]
```

Reads back from the drive and writes
`~/printer_data/config/servo_tuning/<NAME>.params`:

- the gain set — C01.00 position gain (`0x2001.0x01`), C01.01 speed gain
  (`0x2001.0x02`), C01.02 speed integral time (`0x2001.0x03`), all `u16`;
- the load inertia ratio — C00.06 (`0x2000.0x07`), `u16`;
- any addresses in `ADDRS` (comma-separated `0xINDEX.SUB` or
  `0xINDEX.SUB:type`; type defaults to `u16`) — the notch registers a
  campaign varied, for instance.

`NAME` must match `[A-Za-z0-9_-]+`. `SERVO_SAVE_TUNING` never overwrites an
existing profile file — pick a new `NAME`; this mirrors dynamics profiles,
where switching tuning is an explicit config edit, not an implicit
in-place update. The `servo_tuning/` directory is created if missing. A
readback failure (no engine handle, SDO error, or a drive object whose size
doesn't match the assumed type) aborts the whole write with no partial file.

Loaded via a `[servo_tuning]` config section:

```
[servo_tuning]
```

With no `extra_params:` it registers `SERVO_SAVE_TUNING`, `SERVO_DUMP_TUNING`
and `SERVO_TUNE` and no other options are required — see "Tuning panel
backend" below for the full config surface.

## Tuning panel backend

`SERVO_DUMP_TUNING` and `SERVO_TUNE` are the read/write halves of a drive
tuning panel: a curated map of named drive registers (`PANEL_PARAMS` in
`servo_tuning.py`), a command that snapshots every mapped register from one
or more servo motors to JSON, and a command that writes one register and
verifies it landed.

### The C-code address rule

Drive datasheets name registers `CGG.NN` (group, code); the CoE object
dictionary address is `0xINDEX.SUB`. `servo_tuning.c_code_to_addr` converts
one to the other:

- `index = 0x2000 + GG` (the group number, decimal)
- `sub = int(NN, 16) + 1` (the code digits read as hex, then +1 — the
  drive's SDO objects are 1-based where the datasheet code is 0-based)

```
C00.04 -> 0x2000.0x05
C00.06 -> 0x2000.0x07
C01.00 -> 0x2001.0x01
C01.01 -> 0x2001.0x02
C01.02 -> 0x2001.0x03
C01.30 -> 0x2001.0x31
C02.60 -> 0x2002.0x61
```

### `PANEL_PARAMS`

An ordered list of `PanelParam` entries — `name`, `c_code` (its resolved
`addr` is derived, not stored separately), `type_token` (SDO type, default
`u16`), `unit`, `scale` (display divisor, default `1.0`), `group`,
`description`, and `autofill` (`None`, `"gain_position_from_speed"`, or
`"gain_integral_from_speed"` — a UI hint only; the backend never computes
an autofilled value itself). Shipped entries, all verified against the
EtherCAT bench config and `servo_calibration.GAIN_PARAMS`:

| name | c_code | unit | scale | group | autofill |
| --- | --- | --- | --- | --- | --- |
| `position_gain` | C01.00 | 0.1 rad/s | 10 | gains | `gain_position_from_speed` (`round(speed_gain * 1.6)`) |
| `speed_gain` | C01.01 | 0.1 Hz | 10 | gains | — (the autofill source) |
| `integral_time` | C01.02 | 0.01 ms | 100 | gains | `gain_integral_from_speed` (`round(1250000 / speed_gain)`) |
| `freq_cutoff` | C01.03 | Hz | 1 | filters | — (bench rule-of-thumb ≈ speed_gain/10 × 0.4, drive default 200) |
| `adaptive_notch_mode` | C01.30 | — | 1 | notch | — (0=locked, 1=retune after every restart, 2=auto, 3=restart adaptive tuning now) |
| `gain_mode` | C00.04 | — | 1 | load | — (0=manual, 1=standard/stiffness table) |
| `inertia_ratio` | C00.06 | % | 1 | load | — |
| `c02_60` | C02.60 | — | 1 | experimental | — (name unknown; bench-noted value 2000, identify in the vendor manual) |
| `c02_62` | C02.62 | — | 1 | experimental | — (name unknown; bench-noted value 30, identify in the vendor manual) |
| `c02_63` | C02.63 | — | 1 | experimental | — (name unknown; bench-noted value 150, identify in the vendor manual) |

Names, resolved addresses, and type tokens are all validated for
uniqueness/validity at import time and again once `extra_params:` is
merged in (`servo_tuning.validate_param_map`) — a broken map is a config
error, never a silent skip.

### `extra_params:` (config)

```
[servo_tuning]
extra_params:
  notch_freq2 C01.31 u16 Hz 1.0 notch
```

One entry per line: `name C-code type unit scale group` (six
whitespace-separated fields — `unit` can't contain spaces here, unlike the
built-in entries above). Blank lines and `#` comments are skipped. A bad
line, an unknown type, an unparsable scale, or a name/address collision
with an existing entry is a config error naming the line number and the
conflict.

### `SERVO_DUMP_TUNING`

```
SERVO_DUMP_TUNING [MOTORS=all|<name>[,<name>...]]
```

Reads every mapped parameter from the targeted servo motors (default:
every servo motor, via `servo_axis.iter_servo_motors` — the same discovery
`servo_capture` uses) and writes `<captures_root>/drive_state.json`
atomically (write to `.tmp`, then `os.replace`). `captures_root` is
`[servo_calibration] captures_root` when that section exists, else
`~/printer_data/logs/servo_captures` (both `expanduser`d).

```json
{
  "version": 1,
  "created_utc": "2026-07-10T15:17:02Z",
  "params": [
    {
      "name": "position_gain",
      "c_code": "C01.00",
      "addr": "0x2001.0x01",
      "type": "u16",
      "unit": "0.1 rad/s",
      "scale": 10.0,
      "group": "gains",
      "description": "C01.00 position loop gain; autofilled from speed_gain as round(speed_gain * 1.6)",
      "autofill": "gain_position_from_speed"
    }
  ],
  "motors": {
    "motor_a": { "C01.00": 700, "C01.01": 550, "C01.02": 2273, "C00.06": 150 }
  },
  "config_pins": {
    "motor_a": { "C00.06": 150 }
  }
}
```

`motors` keys every targeted motor to a dict of every mapped parameter's
live readback, keyed by `c_code`. `config_pins` is the same shape but
filtered down to the addresses that appear in that motor's own
`[motor] params:` block and/or `tuning_profile` (`motor.get_sdo_params()`)
— the panel's cue for "this one is pinned in `printer.cfg`, editing it here
won't survive a restart until you update the config too." A readback
failure (no engine handle, SDO error, or a size mismatch against the
mapped type) aborts the whole command naming the motor and the parameter;
no partial file is written. One `respond_info` line reports the path,
motor count and param count.

### `SERVO_TUNE`

```
SERVO_TUNE PARAM=<name|C-code|addr> VALUE=<int> [MOTORS=all|<name>[,<name>...]] [TYPE=u8/u16/u32/i8/i16/i32]
```

`PARAM` resolves, in order: a `PANEL_PARAMS` name, a C-code (`CGG.NN`), or
a raw `0xINDEX.SUB` address. A C-code or raw address that isn't in the map
is still allowed — it's written with `TYPE=` (default `u16`) instead of a
mapped type. For each target motor (default: every servo motor) the write
goes through the same engine path `SERVO_PARAM SET` uses
(`engine.sdo_write`), is journaled with `servo_param.record_param_write`
unconditionally — these are user edits and must survive into the next
calibration run's between-runs journal, so the write is never suppressed
— and then verified against `sdo_write`'s own settled readback; a mismatch
or a missing engine handle aborts with a command error naming the motor.
`SERVO_TUNE` never applies an autofill formula itself; the panel UI reads
`autofill` from the dump and issues the derived writes itself, one
register per `SERVO_TUNE` call.

One `respond_info` line reports the param, value, and the motors written.
