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

No options; it only registers `SERVO_SAVE_TUNING`.
