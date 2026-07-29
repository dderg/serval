# Migrating a printer configuration

This guide covers the configuration changes required to run a mainline Klipper/Kalico printer configuration on this fork. The motion planner was replaced with a Rust jerk-limited streaming planner. Its motion topology is declared explicitly with `[kinematics]`, `[motor <name>]`, `[axis <name>]`, and `[post_processor <name>]` sections.

The old role-encoded `[stepper_x]`, `[stepper_y]`, and similar sections are rejected. `[printer] kinematics` is also rejected. A configuration with either form must be converted rather than carried forward.

## Worked cartesian example

The following is intentionally a motion-only example. Keep your existing heater, fan, probe, driver, and other non-motion sections as appropriate for your machine. Pin names and dimensions are examples and must be replaced with the values for your printer.

### Mainline-style configuration

```ini
[printer]
kinematics: cartesian
max_velocity: 300
max_accel: 3000
square_corner_velocity: 5
max_z_velocity: 20
max_z_accel: 100

[stepper_x]
step_pin: PA0
dir_pin: PA1
enable_pin: !PA2
microsteps: 16
rotation_distance: 40
endstop_pin: ^PC0
position_endstop: 0
position_max: 220
homing_speed: 50

[stepper_y]
step_pin: PB0
dir_pin: PB1
enable_pin: !PB2
microsteps: 16
rotation_distance: 40
endstop_pin: ^PC1
position_endstop: 0
position_max: 220
homing_speed: 50

[stepper_z]
step_pin: PC2
dir_pin: PC3
enable_pin: !PC4
microsteps: 16
rotation_distance: 8
endstop_pin: ^PC5
position_endstop: 0
position_max: 250
homing_speed: 10

[extruder]
# Heater and thermistor settings omitted here.
pressure_advance: 0.040
pressure_advance_smooth_time: 0.040

[input_shaper]
shaper_type_x: mzv
shaper_freq_x: 40
shaper_type_y: mzv
shaper_freq_y: 40
```

### Fork configuration

```ini
[printer]
max_velocity: 300
max_accel: 3000
square_corner_velocity: 5
max_z_velocity: 20
max_z_accel: 100

[kinematics]
type: cartesian
axis_x: x
x_motors: x_motor
axis_y: y
y_motors: y_motor
axis_z: z
z_motors: z_motor

[motor x_motor]
drive: stepper
step_pin: PA0
dir_pin: PA1
microsteps: 16
rotation_distance: 40

[motor y_motor]
drive: stepper
step_pin: PB0
dir_pin: PB1
microsteps: 16
rotation_distance: 40

[motor z_motor]
drive: stepper
step_pin: PC2
dir_pin: PC3
microsteps: 16
rotation_distance: 8

[motor e_motor]
drive: stepper
step_pin: PD0
dir_pin: PD1
microsteps: 16
rotation_distance: 7

[axis x]
endstop_pin: ^PC0
position_endstop: 0
position_max: 220
homing_speed: 50
post_processors: x_shaping

[axis y]
endstop_pin: ^PC1
position_endstop: 0
position_max: 220
homing_speed: 50
post_processors: y_shaping

[axis z]
endstop_pin: ^PC5
position_endstop: 0
position_max: 250
homing_speed: 10
post_processors: z_shaping

[post_processor x_shaping]
type: smooth_bell
smooth_time: 0.025

[post_processor y_shaping]
type: smooth_bell
smooth_time: 0.025

[post_processor z_shaping]
type: smooth_bell
smooth_time: 0.025

[post_processor extruder_pa]
type: linear_pressure_advance
k: 0.040

[post_processor extruder_smoothing]
type: smooth_triangle
smooth_time: 0.040

[axis e]
follows: x, y, z
motors: e_motor
post_processors: extruder_pa, extruder_smoothing

[extruder]
# Heater and thermistor settings omitted here.
axis: e
max_extrude_only_velocity: 50
max_extrude_only_accel: 5000
```

The fork parser accepts `[printer] square_corner_velocity` as a legacy alias. It also accepts `corner_deviation` instead; set exactly one. `max_z_velocity` and `max_z_accel` must be at most the corresponding global caps, defaulting to those caps when omitted.

`[motor <name>]` requires `drive`, `step_pin`, `dir_pin`, `microsteps`, and either `rotation_distance` or gear-ratio mode. Enable pins, if needed by the hardware integration, belong to the relevant stepper/driver support rather than the topology declaration above. Axis endstop and homing options belong on `[axis <name>]`, not on `[motor <name>]`.

## Option mapping

| Mainline option/section | Fork option or result |
|---|---|
| `[printer] max_velocity` | `[printer] max_velocity` |
| `[printer] max_accel` | `[printer] max_accel` |
| `[printer] max_z_velocity` | `[printer] max_z_velocity` (must be `<= max_velocity`) |
| `[printer] max_z_accel` | `[printer] max_z_accel` (must be `<= max_accel`) |
| `[printer] square_corner_velocity` | Same option, converted to the fork's corner-deviation budget; or use `[printer] corner_deviation`. Do not set both. |
| `[printer] max_jerk` | `[printer] max_jerk` |
| `[printer] max_path_deviation` | Same option; fitter path-deviation tolerance. |
| `[printer] max_accel_deviation` | Same option; fitter acceleration-deviation tolerance. |
| `[extruder] max_extrude_only_velocity` | Same option. |
| `[extruder] max_extrude_only_accel` | Same option. |
| `[printer] max_accel_to_decel` | **No equivalent.** The fork rejects this option loudly. |
| `[printer] minimum_cruise_ratio` | **No equivalent.** The fork rejects this option loudly. |
| `[printer] kinematics` | **No equivalent.** Declare `[kinematics] type: cartesian` or `type: corexy` and its role bindings. |
| `[stepper_x]`, `[stepper_y]`, `[stepper_z]` | **No direct section alias.** Split each into a `[motor <name>]` plus `[axis <name>]`; bind kinematic X/Y/Z motors from `[kinematics]` only. Use `[axis <name>] motors:` for non-kinematic axes such as the extruder. |
| `[input_shaper] shaper_type*`, `shaper_freq*` | **No direct equivalent.** Use named smoothing post-processors and reference them from `[axis <name>] post_processors`. |
| `[extruder] pressure_advance` | `[post_processor <name>] type: linear_pressure_advance`, `k: ...`; put its name in the extruder axis chain. |
| `[extruder] pressure_advance_smooth_time` | `[post_processor <name>] type: smooth_triangle`, `smooth_time: ...`; put its name in the extruder axis chain. |
| `max_x_velocity`, `max_y_velocity`, and other old per-axis velocity limits | **No equivalent.** The fork exposes global velocity/acceleration plus Z-only caps, not independent X/Y limits. |
| legacy `[servo_x]`, `[servo_y]`, `[servo_z]` | **No direct section alias.** Declare a `[motor <name>]` with `drive: servo` and bind it in `[kinematics]`. |
| `[firmware_retraction]` | **No equivalent in the motion model;** the section is rejected. |

Unknown options normally fail validation: with the default `error_on_unused_config_options: True`, an option not consumed by a registered object errors as an invalid option. The explicitly rejected motion sections/options above fail earlier with a specific error.

## Pressure advance

Pressure advance is an axis post-processing stage, not an `[extruder]` scalar. For the linear model, replace:

```ini
[extruder]
pressure_advance: 0.040
```

with:

```ini
[post_processor extruder_pa]
type: linear_pressure_advance
k: 0.040

[axis e]
post_processors: extruder_pa
```

The `linear_pressure_advance` parameter is named `k` and must be non-negative. For nonlinear pressure advance, the recommended models are `tanh_pressure_advance` and `recipr_pressure_advance`; each uses `linear_advance`, `nonlinear_offset`, and `linearization_velocity` parameters.

`SET_PRESSURE_ADVANCE` remains available through the loaded pressure-advance compatibility module. It accepts `EXTRUDER`, `ADVANCE`, and `SMOOTH_TIME`; `ADVANCE` updates the selected linear post-processor's `k`, while `SMOOTH_TIME` updates the selected smooth-triangle processor. If no compatible processor is attached, the command reports that the target is disabled.

## Input shaping and smoothing

The old `[input_shaper]` section (`shaper_type`/`shaper_freq`, including axis-suffixed forms) is rejected. Define a named post-processor and attach it to each axis:

```ini
[post_processor x_shaping]
type: smooth_bell
smooth_time: 0.025

[axis x]
post_processors: x_shaping
```

The available shaping/smoothing kernels are `smooth_zv` (`frequency_hz`), `smooth_mzv` (`frequency_hz`), `smooth_bell` (`smooth_time`), and `smooth_triangle` (`smooth_time`). There is no EI-family kernel in the registry. The recommended default for a new configuration is `smooth_bell`; choose its `smooth_time` from your tuning measurements.

## G-code command differences

| Command | Fork behavior |
|---|---|
| `SET_VELOCITY_LIMIT` | Still registered. Accepts `VELOCITY>0`, `ACCEL>0`, and one of `SQUARE_CORNER_VELOCITY>=0` or `CORNER_DEVIATION>=0`. `MINIMUM_CRUISE_RATIO` and `ACCEL_TO_DECEL` are accepted as command-level legacy no-ops. With no values it reports velocity, acceleration, and corner values. |
| `RESET_VELOCITY_LIMIT` | Still registered; clears the dynamic velocity, acceleration, and corner-deviation caps. |
| `SET_PRESSURE_ADVANCE` | Compatibility command. Accepts optional `EXTRUDER`, `ADVANCE>=0`, and `SMOOTH_TIME>=0`, and updates named post-processors. |
| `SET_INPUT_SHAPER` | Not registered; no command equivalent. Change the post-processor configuration instead. |
| `SET_POST_PROCESSOR` | Fork command. Requires `NAME=` and at least one numeric `<PARAM>=<VALUE>`; updates a named post-processor for the next replan. |
| `TUNING_TOWER` | Still registered. Accepts `COMMAND`, `PARAMETER`, `START`, `FACTOR`, `BAND`, `STEP_DELTA`, `STEP_HEIGHT`, and `SKIP`. It can emit updates to commands such as `SET_PRESSURE_ADVANCE` or `SET_POST_PROCESSOR`. |
| `M204` | Still registered by the motion module. |

After conversion, remove old motion sections and options rather than leaving both models in the file: the old role-encoded sections, `[printer] kinematics`, `[input_shaper]`, `max_accel_to_decel`, and `minimum_cruise_ratio` are rejected.
