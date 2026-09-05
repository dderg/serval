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
corner_deviation: 0.04
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
type: smooth_mzv
frequency_hz: 40

[post_processor y_shaping]
type: smooth_mzv
frequency_hz: 40

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

`corner_deviation` is the canonical corner budget in mm; 0.04 is a good starting value. `square_corner_velocity` is still accepted as a legacy alias, converted internally as `scv² · (√2 − 1) / max_accel` — note this maps typical mainline values to a far tighter budget than intended (5 mm/s at 3000 mm/s² ≈ 0.0035 mm), so prefer setting `corner_deviation` directly. Set exactly one of the two. `max_z_velocity` and `max_z_accel` must be at most the corresponding global caps and default to them.

`[motor <name>]` requires `drive`, `step_pin`, `dir_pin`, `microsteps`, and either `rotation_distance` or gear-ratio mode. Enable pins, if needed by the hardware integration, belong to the relevant stepper/driver support rather than the topology declaration above. Axis endstop and homing options belong on `[axis <name>]`, not on `[motor <name>]`.

## CoreXY

CoreXY declares the same three lanes, but the belt motors are named by
belt rather than by cartesian axis: the role keys are `a_motors` and
`b_motors`, still bound to `axis_x` and `axis_y`.

```ini
[kinematics]
type: corexy
axis_x: x
a_motors: a_motor
axis_y: y
b_motors: b_motor
axis_z: z
z_motors: z_motor
```

`a_motors` is the lane the old `[stepper_x]` drove, `b_motors` the old
`[stepper_y]`. Everything else — the `[axis x]`/`[axis y]` travel and
homing keys, the shaping chains — converts exactly as in the cartesian
example above.

## Stepper drivers and sensorless endstops

A driver section is named after the section it drives, and that is now a
motor rather than a role:

```ini
[tmc2209 stepper_x]     # mainline
[tmc2209 a_motor]       # fork
```

The driver reads its microstep count from `[motor <suffix>]`, so an
unconverted suffix stops the boot with `Could not find config section
'[motor stepper_x]' required by tmc driver`. The driver's own options are
untouched: `run_current`, `home_current`, `sense_resistor`, `diag_pin`,
`driver_SGTHRS`, `stealthchop_threshold`, `interpolate`, and the rest
carry over verbatim.

Sensorless homing follows that rename, because the virtual pin's chip
name comes from the driver section:

```ini
endstop_pin: tmc2209_stepper_x:virtual_endstop   # mainline, on [stepper_x]
endstop_pin: tmc2209_a_motor:virtual_endstop     # fork, on [axis x]
```

`use_sensorless_homing` still defaults to true when the endstop is
virtual, and the homing keys it interacts with — `homing_retract_dist`,
`homing_retract_speed`, `min_home_dist`, `homing_positive_dir`,
`second_homing_speed` — all belong on `[axis <name>]`. Putting any of
them on a `[motor <name>]` is rejected with a message naming the axis
they belong to.

## Option mapping

| Mainline option/section | Fork option or result |
|---|---|
| `[printer] max_velocity` | `[printer] max_velocity` |
| `[printer] max_accel` | `[printer] max_accel` |
| `[printer] max_z_velocity` | `[printer] max_z_velocity` (must be `<= max_velocity`) |
| `[printer] max_z_accel` | `[printer] max_z_accel` (must be `<= max_accel`) |
| `[printer] square_corner_velocity` | Still accepted as a legacy alias. The canonical option is `[printer] corner_deviation` (= `scv² · (√2 − 1) / max_accel`). Set exactly one. |
| `[printer] max_jerk` | `[printer] max_jerk` |
| `[printer] max_path_deviation` | Same option; fitter path-deviation tolerance. |
| `[printer] max_accel_deviation` | Same option; fitter acceleration-deviation tolerance. |
| `[extruder] max_extrude_only_velocity` | Same option. |
| `[extruder] max_extrude_only_accel` | Same option. |
| `[printer] max_accel_to_decel` | **No equivalent.** The fork rejects this option loudly. |
| `[printer] minimum_cruise_ratio` | **No equivalent.** The fork rejects this option loudly. |
| `[printer] kinematics` | **No equivalent.** Declare `[kinematics] type: cartesian` or `type: corexy` and its role bindings. |
| `[stepper_x]`, `[stepper_y]`, `[stepper_z]` | **No direct section alias.** Split each into a `[motor <name>]` plus `[axis <name>]`; bind kinematic X/Y/Z motors from `[kinematics]` only. Use `[axis <name>] motors:` for non-kinematic axes such as the extruder. |
| `[input_shaper] shaper_type*`, `shaper_freq*` | Named smoothing post-processors on the axis chains. `shaper_type: mzv` + `shaper_freq: F` maps to `type: smooth_mzv` + `frequency_hz: F`. No EI-family kernel exists. |
| `[tmc2209 stepper_x]`, `[tmc5160 stepper_y]`, `[tmc2130 ...]` | Rename the suffix to the motor's name (`[tmc2209 a_motor]`). Driver options are unchanged. |
| `endstop_pin: tmc2209_stepper_x:virtual_endstop` | `endstop_pin: tmc2209_<motor>:virtual_endstop` on `[axis <name>]`. |
| `[stepper_*] high_precision_step_compress` | Same opt-in, moved with the stepper fields to `[motor <name>] high_precision_step_compress: True`. Motors default to classic compression. |
| `[extruder] pressure_advance` | `[post_processor <name>] type: linear_pressure_advance`, `k: ...`; put its name in the extruder axis chain. |
| `[extruder] pressure_advance_smooth_time` | `[post_processor <name>] type: smooth_triangle`, `smooth_time: ...`; put its name in the extruder axis chain. |
| `[extruder] max_extrude_cross_section`, `max_extrude_only_distance`, `instantaneous_corner_velocity` | **Rejected loudly** — "the planner has no such concept". Delete them. |
| `[extruder] step_pin`, `dir_pin`, `rotation_distance`, `microsteps` | **Rejected.** Move them to a `[motor <name>]` and name it in `[axis e] motors:`. |
| `[extruder]` with no stepper (heater only) | **Not expressible.** `axis:` is required, the named axis must be a follower, and every declared axis must be motor-mapped — a hotend with no extruder motor is rejected with `axis 'e' is not motor-mapped`. |
| `max_x_velocity`, `max_y_velocity`, and other old per-axis velocity limits | **No equivalent.** The fork exposes global velocity/acceleration plus Z-only caps, not independent X/Y limits. |
| legacy `[servo_x]`, `[servo_y]`, `[servo_z]` | **No direct section alias.** Declare a `[motor <name>]` with `drive: servo` and bind it in `[kinematics]`. |
| `[firmware_retraction]` | **No equivalent in the motion model;** the section is rejected. |
| `[gcode_arcs]` | **Rejected.** The motion engine has no native G2/G3 ingestion yet; slice with arcs disabled so the slicer emits G1 segments. |
| `[resonance_tester] sweeping_period`, `sweeping_accel` | **No equivalent.** Kalico's sweeping vibration test is not here; the rest of the section (`accel_per_hz`, `hz_per_sec`, `min_freq`, `max_freq`, `max_smoothing`, `probe_points`, `accel_chip*`) is unchanged. |

Unknown options normally fail validation: with the default `error_on_unused_config_options: True`, an option not consumed by a registered object errors as an invalid option. The explicitly rejected motion sections/options above fail earlier with a specific error.

## Sections that no longer exist

Beyond the motion model, a set of klippy extras is absent from this
branch, so their sections stop resolving with `Section '<name>' is not a
valid config section`. Some were deleted with the old planner, others
were simply never carried across:

| Section | Note |
|---|---|
| `[input_shaper]` | Replaced by `[post_processor]` chains (see above). |
| `[extruder_smoother]` | Replaced by a `smooth_triangle` post-processor. |
| `[ringing_test]`, `[pa_test]` | Kalico's calibration-pattern generators. The `RINGING_TEST` / `PA_TEST` commands go with them, so macros that call them break too. |
| `[manual_stepper]` | Drives a stepper outside the kinematic model, which the motor/axis split has no room for. Mods that depend on it (filament changers, nozzle wipers, `[trad_rack]`) do not load. |
| `[endstop_phase]` | Incompatible with the new stepper model. |
| `[pwm_tool]` | Not carried. |
| `[probe_eddy_current]`, `[ldc1612]` | Eddy-current probing, deleted with the old CAN connect path. |

A configuration that includes any of these has to drop the section
before klippy will finish parsing. Check your `[include]` files, not just
`printer.cfg` — on a typical Voron tree these live in separate
calibration includes.

## The SAVE_CONFIG block

The autosave block at the end of `printer.cfg` is config like any other,
so it is parsed and rejected on the same rules. A calibrated printer
usually carries at least these:

```ini
#*# [stepper_z]
#*# position_endstop = 116.100
```

Move that value into `[axis z] position_endstop` by hand and delete the
stanza; the same goes for a saved `[input_shaper]` result, which becomes
the `frequency_hz` of a shaping post-processor. Saved heater PID and MPC
stanzas are untouched by the migration and can stay exactly as they are.

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

The old `[input_shaper]` section (`shaper_type`/`shaper_freq`, including axis-suffixed forms) is rejected. The direct replacement for a frequency-tuned shaper is a named `smooth_mzv` post-processor — carry the old `shaper_freq_*` value over as `frequency_hz` — attached to the axis:

```ini
[post_processor x_shaping]
type: smooth_mzv
frequency_hz: 40

[axis x]
post_processors: x_shaping
```

The available shaping/smoothing kernels are `smooth_zv` (`frequency_hz`), `smooth_mzv` (`frequency_hz`), `smooth_bell` (`smooth_time`), and `smooth_triangle` (`smooth_time`). There is no EI-family kernel in the registry.

Recommended defaults: `smooth_mzv` at the measured resonance frequency for
X and Y; `smooth_bell` for Z and for axes without a measured resonance.

Coming from Kalico's smooth shapers (`shaper_type_x: smooth_mzv` with
`smoother_freq_x`), keep your measured frequency and write it as
`frequency_hz`: these are the same kernels, coefficient for coefficient,
with the same window (`0.95625 / f` for `smooth_mzv`, `0.8025 / f` for
`smooth_zv`). No retune is required. Coming from a classic
`shaper_type: mzv` instead, the frequency carries over but the kernel
does not — `smooth_mzv` is the smoothed variant, so expect to verify the
result.

## Macros that read config sections

Macros that introspect the old section names break at runtime, because
those sections no longer exist. This line, straight out of the stock
Voron macro set, is the one most configurations trip over:

```jinja
{% set max_x = printer.configfile.config["stepper_x"]["position_max"]|float %}
```

Read the limits off the toolhead instead. The field exists on mainline
too, so the macro stays portable in both directions:

```jinja
{% set max_x = printer.toolhead.axis_maximum.x|float %}
```

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
