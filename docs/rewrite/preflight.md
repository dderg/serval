# Preflight: plan a print before printing it

Run this before starting a print, from the tree whose code the printer runs:

```sh
uv run python scripts/preflight.py PRINT.gcode --config ~/printer_data/config/printer.cfg
```

It plans the whole file offline through the production pipeline
(`gcode_replay_bench`) under the printer's own config and bed mesh. Exit 0
`PASS`, 1 `FAIL`, 2 `UNVERIFIED`. On a 7800X3D a 27 s print takes ~30 s: one
replay checks every segment, a second, pinned to two cores, is timed.

## What it runs

- **Config**: klippy's own reader (`PrinterConfig.read_main_config`):
  `[include]` files and the `SAVE_CONFIG` block merged, serialized as klippy
  hands it to the planner. Limits, kinematics, shaper and pressure-advance
  chains all come from it.
- **G-code**: every line runs through klippy's `GCodeDispatch` with the
  config's `[gcode_macro]`s (`rename_existing` included) and `[bed_mesh]`.
  A Jinja macro renders with its call's `params`, its own `variable_*`s and
  the status of other macros and `bed_mesh`. A macro that reads any other
  printer status (`printer.toolhead`, heaters, ...) or is a Python macro is
  skipped and reported with its line; nothing it would emit is planned.
- **Mesh**: `--mesh NAME`, else the first profile the print loads
  (`BED_MESH_PROFILE LOAD`, or `bed_mesh_default` at startup), else the saved
  `default` profile, else none. The replay activates it before the first
  move with the toolhead at the origin, through the engine's own activation
  (`SurfaceTransform::activate`). Later loads and clears are listed, not
  replayed.

Each command klippy would execute is treated by class
(`scripts/preflight_gcode.py`, `TREATMENTS`):

| class | treatment |
|---|---|
| moves and motion state: `G0 G1 G4 G90 G91 G92 M82 M83 M18 M84 M204 M220 M221 M400`, velocity limits, `SET_KINEMATIC_POSITION`, G-code offsets and states | planned |
| heater waits: `M109 M190 TEMPERATURE_WAIT` | planned as `M400` (a stop at rest) |
| homing, probing, leveling: `G28 PROBE QUAD_GANTRY_LEVEL BED_MESH_CALIBRATE ...` | dropped; the next move starts from the last commanded position |
| motors outside the toolhead path: `FORCE_MOVE SET_STEPPER_ENABLE ...` | dropped |
| planner setup: `SET_PRESSURE_ADVANCE SET_INPUT_SHAPER ...` | dropped; the plan keeps the config's values |
| `PAUSE RESUME CANCEL_PRINT EXCLUDE_OBJECT` | dropped (a macro's own moves are planned) |
| arcs and firmware retraction: `G2 G3 G10 G11` | `FAIL`: the live host stops the print on them |
| curves: `G5` | `FAIL`: the live host runs it, preflight cannot plan it |
| anything else (`M104 M106 M73 ...`) | no motion, not planned |

Every dropped or unplanned command is counted with its first line.

## Verdict

`FAIL` when the planner panics, refuses a move, emits a segment that does not
evaluate finite, or klippy would raise (a macro's `action_raise_error`, an
unknown mesh profile, `zero_reference_position` missing, a move on an unhomed
axis after `M84`, ...). The report names the panic location, the print line
(and macro chain) of the last move fed and of the last segment emitted, and
that segment's stream time. `UNVERIFIED` when the file holds motion preflight
cannot plan (`SET_KINEMATIC_POSITION` jumping mid-print, ...).

## Throughput

The timed replay reports stream duration, wall time and realtime factor on
this box's two pinned cores, and the worst 5 s stream window. That window is
scaled to a Pi 5 by this box's speed over the Pi on the throughput gate's
calibration replay (`scripts/throughput_gate.py`, measured once per CPU model
and cached in `~/.cache/serval-preflight/pi_speed.json`), and compared with
the gate's 2.0x floor. It is an estimate and says so; only when
`tools/bench/pi_window.json` holds the Pi's own measurement of exactly this
print, config and tree is the Pi number measured. A window below the floor is
a warning, not a `FAIL`.

## Not covered

Heaters and thermal behaviour, homing and probing moves, MCU dispatch (step
generation and compression, step-rate ceilings, queues, the host-MCU link)
and host runtime (pacer, drains on a starved host, pause/resume/cancel, clock
sync). For those, print the file in the simulator: `tools/sim/run.sh --gcode
PRINT.gcode`.
