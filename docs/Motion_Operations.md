# Motion operations and G-code

This page documents the Serval-specific motion command surface. It complements the broader inherited [G-code reference](G-Codes.md). Commands are processed by `klippy/motion.py` and `klippy/extras/gcode_move.py`; their effects are subject to the support and safety limits in [Feature status](Feature_Status.md).

## Units and execution model

Coordinates and distances are millimetres; velocity is mm/s; acceleration is mm/s²; dwell `P` is milliseconds. Serval plans and streams ahead, so accepting a command does not mean the MCU has finished playing it. Use `M400` where a subsequent operation must wait for physical execution.

Classic positioning commands (`G90`, `G91`, `G92`, `M82`, `M83`, `M114`, `M220`, and `M221`) remain owned by `gcode_move`. They maintain G-code coordinates and submit the resulting move to the Serval engine. Inches (`G20`) are explicitly rejected; use millimetres (`G21`).

## Linear moves, dwell, and drain

### `G0` / `G1`

`G0` and `G1` submit a linear move using the current absolute/relative coordinate and extrusion modes. `F` is feed rate in mm/min in G-code, as usual; the host converts it to mm/s. The motion engine validates kinematic travel and extrusion constraints before submission.

A linear junction is not necessarily executed as a sharp polyline corner: the fitter may create a blend inside the configured corner-deviation budget. The commanded endpoint remains the G-code endpoint. For migration and the full limits model, use [Motion configuration reference](Config_Reference_Motion.md).

### `G4 P=<milliseconds>`

Dwell for a non-negative `P` duration. It is ordered in the motion stream. `G4` without `P` is a zero-duration ordered dwell.

### `M400`

Wait until the engine has drained submitted motion **and** the participating MCU execution frontier has caught up. Use before a physical observation, manual interaction, or command sequence that requires completed movement. It is not a recovery mechanism for a timing fault; if the motion system faults, inspect the error, restore a known position, and home as appropriate.

## Curved toolpaths

Serval accepts Bézier curves directly. Curves are rejected while a coordinate transform that cannot preserve the curve is active.

### `G5` — cubic Bézier

`G5` requires `P` and `Q`. `I` and `J` must either both be present or both be omitted.

- `I`, `J`: first XY control point offset from the start point (optional pair).
- `P`, `Q`: second XY control point offset from the endpoint (required).
- `X`, `Y`, `Z`, `E`, `F`: endpoint/extrusion/feedrate fields use the current normal G-code modes.

The host constructs an interior control point for `I/J` when given and one for `P/Q`; Z is distributed through the curve. Invalid numeric values, missing required controls, out-of-range curve geometry, and rejected transforms are errors. Example:

```gcode
G90
G1 X20 Y20 F6000
G5 X60 Y20 I15 J0 P-15 Q0 F9000
```

### `G5.1` — quadratic Bézier

`G5.1` requires at least one of `I` or `J`; an omitted component is zero. The XY control point is relative to the start. Endpoint fields and modes are the same as `G5`.

```gcode
G5.1 X80 Y40 I15 J20 F9000
```

Do not substitute these curves into a production slicer workflow without validating the slicer output and the complete machine configuration in simulation and on the target machine.

## Runtime motion limits

### `SET_VELOCITY_LIMIT`

Query current effective caps by sending the command with no Serval limit parameters:

```gcode
SET_VELOCITY_LIMIT
```

The response reports velocity, acceleration, canonical corner deviation, and its square-corner-velocity equivalent. Set one or more caps:

```gcode
SET_VELOCITY_LIMIT VELOCITY=250 ACCEL=5000 CORNER_DEVIATION=0.04
```

| Parameter | Unit | Rule |
| --- | --- | --- |
| `VELOCITY` | mm/s | Positive temporary velocity cap. |
| `ACCEL` | mm/s² | Positive temporary acceleration cap. |
| `CORNER_DEVIATION` | mm | Non-negative canonical corner budget. |
| `SQUARE_CORNER_VELOCITY` | mm/s | Non-negative legacy alias converted to a corner-deviation budget. |

`CORNER_DEVIATION` and `SQUARE_CORNER_VELOCITY` are aliases for the same concept and cannot be supplied together. `MINIMUM_CRUISE_RATIO` and `ACCEL_TO_DECEL` are accepted only as legacy no-op arguments; they do not alter Serval planning. An invalid runtime corner update warns and leaves the previous corner setting unchanged.

### `RESET_VELOCITY_LIMIT`

```gcode
RESET_VELOCITY_LIMIT
```

Removes temporary velocity, acceleration, and corner-deviation caps, returning to configured values. It does not change the configuration file.

### `M204`

`M204 S=<accel>` sets a runtime acceleration cap. If `S` is absent, both `P` and `T` must be supplied and Serval uses the smaller value. A lone `P` or `T` is invalid and changes nothing. This preserves common slicer output while avoiding separate print/travel acceleration models that Serval does not implement.

## Live post-processor tuning

`SET_POST_PROCESSOR` changes a named `[post_processor]` parameter for future replanning:

```gcode
SET_POST_PROCESSOR NAME=extruder_pa k=0.042
SET_POST_PROCESSOR NAME=x_smooth smooth_time=0.018
```

`NAME` is required and at least one additional `PARAM=VALUE` is required. Parameter names are normalized to lowercase; values must parse as floating point and pass the same native validation as configuration. The update applies from the **next replan**, not retroactively to pieces already dispatched. Use the exact schema in [Motion configuration reference](Config_Reference_Motion.md), and tune only with a safe test procedure—pressure advance and inverse-mode changes can alter motor demand.

## Diagnostics and non-production commands

`DIAG_DUMP` requests live runtime diagnostics from every MCU exposing `runtime_diag_dump`, with output recorded under `printer_data/logs/events/<mcu>.jsonl`. It is useful when collecting a fault report, not a substitute for a controlled recovery procedure.

Commands prefixed `MCU_SIM_` and `_HOME_TEST` are test/simulator interfaces. They are not operator APIs and must not be placed in normal macros or production print files. Servo-specific commands are provided only by the relevant optional servo modules; their hardware and safety requirements are separate from this motion reference.
