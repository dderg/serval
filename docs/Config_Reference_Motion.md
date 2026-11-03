# Motion configuration reference

This reference documents the fork's Rust motion configuration model. It complements [Config_Migration.md](Config_Migration.md). The motion reader and its defaults are implemented in `rust/planner-config/src/from_doc.rs`; stepper and axis homing options are parsed by `klippy/stepper.py` and `klippy/rail.py`.

## `[printer]`

Global motion limits and fitter tolerances. `max_velocity` and `max_accel` are required.

```
[printer]
max_velocity: 300
#   Required. Must be above 0. Cartesian velocity limit in mm/s.
max_accel: 3000
#   Required. Must be above 0. Cartesian acceleration limit in mm/s^2.
#max_jerk: 0
#   Optional. Default is 0, i.e. jerk limiting is disabled. Only 0 is
#   accepted — the continuous trajectory pipeline rejects a finite jerk
#   limit.
#corner_deviation: 0.04
#   Optional canonical corner budget in mm: how far the planned path may
#   cut a corner's vertex. Default is 0.04, at every max_accel, when
#   neither corner option is set. Must be >= 0. Set this XOR
#   square_corner_velocity.
#square_corner_velocity:
#   Optional legacy alias, no default. Must be >= 0. Set this XOR
#   corner_deviation; it is converted to the corner-deviation budget
#   scv^2 * (sqrt(2) - 1) / max_accel (5 mm/s at 3000 mm/s^2 is 0.0035 mm).
#max_z_velocity: 300
#   Optional. Default is max_velocity. Must be > 0 and <= max_velocity.
#   Z speed limit in mm/s.
#max_z_accel: 3000
#   Optional. Default is max_accel. Must be > 0 and <= max_accel.
#   Z acceleration limit in mm/s^2.
#max_path_deviation: 0.005
#   Optional. Default is 0.005 mm. Must be > 0 and <= 1. Follower axes
#   (the extruder) are fitted to this times their largest extrusion ratio,
#   clamped to 0.01..1.
#max_accel_deviation: 50
#   Optional. Default is 50 mm/s^2. Must be > 0. A follower axis whose
#   pressure advance applies before projection (no kernel after a linear
#   advance, or a nonlinear advance before one) holds each piece of its
#   advanced fit, of duration h, to the looser of this and 8*e/h^2, e being
#   its scaled path deviation: the acceleration error a piece pinned at
#   both ends can carry while staying within e.
```

`corner_deviation` is the canonical corner budget. Setting both corner options is an error. `max_accel_to_decel` and `minimum_cruise_ratio` are explicitly unsupported rather than ignored.

`max_z_velocity` and `max_z_accel` bound the z axis alone, not the toolhead path. The planner applies them to each fitted segment after corner blending, using the z share of that segment's heading and of its curvature. That covers corners between an XY line and an XYZ move, such as layer changes and z-hops: the turn can put z acceleration on the blend even where its heading barely climbs, so the planner slows the blend until z stays within both caps. The caps never raise a limit; they only lower the speed ceiling and the acceleration budget of segments that move z.

### Why the corner budget defaults to 0.04 mm

A config that sets neither corner option used to inherit Klipper's implicit `square_corner_velocity: 5`. That is a budget of 0.0035 mm at 3000 mm/s² and 0.00015 mm at 70000 mm/s², so every corner tighter than about 135° was close to a full stop. The default is now `corner_deviation: 0.04` at every `max_accel`. The value comes from a profile study of four machines, each replaying a dense top-layer print (`tools/sim/gcode/voron_dense_top_layers.gcode`) on one pinned core, as a proxy for a Pi-class host's realtime margin:

- P1: bowden bedslinger, 3k mm/s², PA 0.5.
- P2: i3 direct drive, 5k mm/s², PA 0.05.
- P3: Voron direct drive, 15k mm/s², PA 0.03.
- P4: Trident, 70k mm/s², PA 0.02.

| Corner budget | Dense-print segments | Print time, P1 / P2 / P3 / P4 (s) | Worst single-core realtime factor | Nozzle miss at a vertex, P1 … P4 (mm) |
|---|---|---|---|---|
| `square_corner_velocity: 5` | 18.3k–19.7k | 65.2 / 51.9 / 39.3 / 29.1 | 1.2× (P4) | 0.081 … 0.128 |
| 0.01 | 8.2k | 58.1 / 41.3 / 24.4 / 16.8 | 1.9× (P4) | 0.092 … 0.154 |
| 0.02 | 1.27k | 50.3 / 35.3 / 20.9 / 15.4 | 6.2× (P4) | 0.106 … 0.170 |
| **0.04** | **1.31k** | **50.0 / 35.1 / 20.7 / 15.3** | **6.3× (P4)** | **0.131 … 0.197** |
| 0.08 | 1.21k | 49.8 / 34.9 / 20.7 / 15.3 | 7.9× (P4) | 0.175 … 0.245 |

- **Throughput has a cliff.** The print's facets merge only once the budget exceeds about 0.015 mm: 11.7k segments at 0.005, 3.7k at 0.0125, 1.5k at 0.015, and flat from 0.0175 up. At 0.01 and below, P3 and P4 fall under 3× realtime on one core. 0.04 leaves more than twice the budget the cliff needs, for G-code with coarser facets than this print.
- **Quality is nearly flat from 0.02 to 0.04.** The nozzle's miss at a vertex is mostly the input shaper's own smoothing (the scv-5 row). Going from 0.02 to 0.04 adds about 0.025 mm to it, some 6% of a 0.4 mm line. Corner bead error under exact pressure advance stays at about 0.1% on P2–P4 and 0.9% on P1. Peak extruder acceleration at corners falls 8–11% (P4: 58k to 54k mm/s²), which leaves the extruder more headroom.
- **0.08 buys little.** Its throughput change is small and inconsistent: P4 with exact advance goes from 6.3× to 7.9×, the Trident bench config from 6.9× to 5.9×. It adds another 0.045 mm of vertex miss. With an extruder smoothing kernel, the corner bead error on P1 and P2 grows again (P1 1.5% to 2.8%, P2 1.7% to 2.9%).

Configs that set either option keep it. To keep the old near-stop corners, set `square_corner_velocity: 5` explicitly.

## `[kinematics]`

Declare one supported kinematics type and bind each kinematic lane to an `[axis <name>]` declaration and one or more `[motor <name>]` sections. Kinematic lane motors are declared here only; do not repeat `motors:` on the corresponding X/Y/Z axis sections.

### Cartesian

```
[kinematics]
type: cartesian
axis_x: x
#   Required. Name of the [axis] declaration used for X.
x_motors: x_motor
#   Required, non-empty comma-separated [motor] names for X.
axis_y: y
#   Required. Name of the [axis] declaration used for Y.
y_motors: y_motor
#   Required, non-empty comma-separated [motor] names for Y.
axis_z: z
#   Required. Name of the [axis] declaration used for Z.
z_motors: z_motor
#   Required, non-empty comma-separated [motor] names for Z.
```

### CoreXY

```
[kinematics]
type: corexy
axis_x: x
#   Required. Name of the [axis] declaration used for X.
a_motors: a_motor
#   Required, non-empty comma-separated [motor] names for the A lane.
axis_y: y
#   Required. Name of the [axis] declaration used for Y.
b_motors: b_motor
#   Required, non-empty comma-separated [motor] names for the B lane.
axis_z: z
#   Required. Name of the [axis] declaration used for Z.
z_motors: z_motor
#   Required, non-empty comma-separated [motor] names for Z.
```

Only `cartesian` and `corexy` are accepted. `[printer] kinematics` is not accepted. Each referenced motor section must exist, use a valid `drive`, and all motors in one lane must use the same drive type.

## `[motor <name>]`

A stepper motor section is consumed by `klippy/stepper.py:188-216` and `:219-256`. The enable module also accepts `enable_pin` (`klippy/extras/stepper_enable.py:117-119`).

### Stepper motor

```
[motor x_motor]
drive: stepper
#   Required by the native topology reader. Choice: stepper or servo.
step_pin: PA0
#   Required pin name.
dir_pin: PA1
#   Required pin name; may use the normal Klipper pin polarity syntax.
#enable_pin: !PA2
#   Optional. Default is None (handled by stepper_enable).
rotation_distance: 40
#   Required when gear-ratio mode is not selected. Must be > 0.
#gear_ratio: 80:20
#   Optional. A comma/colon-separated list of ratio pairs. Default is
#   an empty list. Each pair is parsed as two floats. When rotation_distance
#   is absent, gear-ratio mode uses 2*pi as the base rotation distance.
microsteps: 16
#   Required. Integer >= 1.
#full_steps_per_rotation: 200
#   Optional. Default is 200. Integer >= 1 and must be divisible by 4.
#step_pulse_duration:
#   Optional. Default is None. Must be between 0 and 0.001 seconds.
#phase_stepping: False
#   Optional opt-in. Default is False.
```

`gear_ratio` is parsed as pairs (`first:second` or `first,second`) and each pair contributes `first / second` to the gearing. The parser does not apply an explicit numeric bound to the individual ratio values. `rotation_distance` and `gear_ratio` are both passed through the step-distance parser; ordinary linear stepper configuration requires a positive `rotation_distance`, while gear-ratio mode is selected when `rotation_distance` is absent and `gear_ratio` is present.

### Servo motor

`drive: servo` selects the EtherCAT servo parser in `klippy/extras/servo_axis.py:87-143`. Its options are separate from stepper electrical options:

```
[motor x_motor]
drive: servo
#   Required topology choice. This parser also requires protocol: ethercat.
protocol: ethercat
#   Required. Only ethercat is accepted.
node: drive_x
#   Required EtherCAT node name.
ethercat_chain_index: 0
#   Required integer >= 0.
rotation_distance: 40
#   Required float > 0.
encoder_counts_per_rev: 4096
#   Required integer >= 1.
#velocity_ff: False
#   Optional. Default is False.
#ff_max_torque: 30
#   Optional. Default is 30. Must be > 0 and <= the drive torque-percent limit.
#invert_direction: False
#   Optional. Default is False.
#homing_following_error: 2.5
#   Optional when the axis has an endstop. Default is 2.5. Must be > 0.
#homing_max_torque: 50
#   Optional when the axis has an endstop. Default is 50. Must be > 0 and <= the drive torque-percent limit.
#following_error:
#   Optional. Default is None. If set, must be > 0.
#max_torque:
#   Optional. Default is None. If set, must be > 0 and <= the drive torque-percent limit.
#dynamics_profile:
#   Optional path to a dynamics profile. Default is None.
#params:
#   Optional servo SDO parameter block. Default is empty.
#tuning_profile:
#   Optional tuning-profile path/name. Default is None.
```

The servo parser requires `protocol`, `node`, `ethercat_chain_index`, `rotation_distance`, and `encoder_counts_per_rev`. `homing_following_error` and `homing_max_torque` are only active when the axis has an endstop; without one their internal values are zero. The drive torque-percent limit is defined by the servo implementation rather than the motion config reader.

## `[axis <name>]`

Axis declarations are read by `rust/planner-config/src/from_doc.rs:570-591`. Travel and homing options are applied when a kinematic lane constructs a stepper/servo rail (`klippy/motion_kinematics.py:40-62`, `klippy/stepper.py:276-317`); follower axes are built from their motors without this rail parser (`klippy/motion_setup.py:41-51`).

```

[axis x]
#follows: y, z
#   Optional comma-separated axis names. Default is an empty list. Names
#   are normalized to lowercase.
#motors: x_motor
#   Optional comma-separated motor names. Default is an empty list. Use
#   this for non-kinematic/follower axes; kinematic X/Y/Z motors are
#   listed in [kinematics] only.
#post_processors: x_shaping
#   Optional comma-separated named post-processors. Default is an empty list.

# The remaining options apply to a kinematic/homable rail axis. They are
# not required on follower axes such as an extruder axis.
#endstop_pin: ^PC0
#   Optional pin. Default is None. If it contains :virtual_endstop,
#   sensorless-homing defaults change accordingly.
#position_min: 0
#   Optional. Default is 0.
position_max: 220
#   Required for this kinematic rail. Must be above position_min.
#position_endstop:
#   Optional. Defaults to position_min, which defaults to 0.
#homing_speed: 5
#   Optional. Default is 5. Must be above 0.
#homing_retract_dist: 5
#   Optional. Default is 5. Must be >= 0.
#homing_retract_speed: 5
#   Optional. Default is homing_speed. Must be above 0.
#min_home_dist: 5
#   Optional. Default is homing_retract_dist. Must be >= 0.
#homing_positive_dir:
#   Optional. Default is inferred from position_endstop and the travel range;
#   an ambiguous interior endstop is an error.
```

A dual-motor axis (one motor per gantry side) can carry one switch per motor.
Replace the pin with an indented `motor_name: pin` block listing every motor of
that axis's kinematic lane, exactly once each:

```
[axis x]
endstop_pin:
  xm_left: ^PA1
  xm_right: ^PB2
```

The first switch to trip freezes only its own motor while the other side keeps
moving; the last trip stops the axis, so the gantry squares itself against the
two switches. Every switch must be wired to the MCU that drives its motor, and
the axis must map 1:1 to a motor lane — CoreXY x/y share a lane and reject the
keyed form. Virtual/sensorless endstops stay single-switch. Each switch appears
in `QUERY_ENDSTOPS` as `x:<motor_name>`.

Follower/non-kinematic axes only need topology and post-processor fields:

```
[axis e]
follows: x, y, z
#   Optional comma-separated axis names. Default is an empty list.
motors: e_motor
#   Optional comma-separated motor names. Default is an empty list.
post_processors: extruder_pa, extruder_smoothing
#   Optional comma-separated named post-processors. Default is an empty list.
#   On an axis that follows others, a pressure advance (or any derivative-gain
#   stage) needs a smoothing kernel in the same list or a ratio_ramp_time on
#   the axis, and the axis's max_velocity and max_accel; the chain is
#   rejected otherwise.
#max_velocity:
#max_accel:
#   Optional, on an axis that follows others. Must be above 0 (mm/s,
#   mm/s^2). What the axis's motor can do for every move it makes, pressure
#   advance included. The velocity planner slows each extruding move so the
#   commanded extruder velocity and acceleration stay within them, corners
#   and curves included. On the [extruder] axis each one left unset is
#   inherited from max_extrude_only_velocity / max_extrude_only_accel (the
#   same motor). An advance needs both, set or inherited.
#instantaneous_corner_velocity: 1.0
#   Optional, on an axis that follows others. Default is 1.0, as upstream
#   Klipper's [extruder] option. Must be above 0 (mm/s). The velocity
#   change the axis's motor takes at once. Where no smoothing kernel follows
#   the advance (a ratio_ramp_time chain) and a followed axis has no shaping
#   post_processor (commonly z), every acceleration step of that axis makes
#   the advance step the extruder velocity by k * ratio * the step's share
#   along the path. The velocity planner caps the acceleration of extruding
#   moves that move such an axis, and of the moves next to them, so each
#   step stays within this; a larger step is refused with the time it
#   happens at.
#ratio_ramp_time:
#   Optional. Default is no ramp. If set, must be above 0 (seconds) and the
#   axis must follow others. The extruded total of the projection is
#   convolved along the leaders' path with a kernel whose half-width is
#   half the path the leaders cover in about this time
#   (v*ratio_ramp_time/2 at speed v) and closes to zero at every rest.
#   Every extrusion-ratio step (travel to extrusion, a flow change, a wipe)
#   and every step in the ratio's slope (a corner blend's flow ramp) is
#   spread over about v*ratio_ramp_time of path, however finely the path is
#   cut into moves: a cluster of short moves smooths exactly as one long
#   move. The laid ratio stays within the commanded ratios, and the
#   extruded material equals the unsmoothed projection's wherever the ratio
#   is constant across the kernel and at every rest.
#   A smoothing kernel may stay in post_processors next to the ramp.
#   Without one, a pressure advance in the same post_processors needs no
#   kernel. On every chain the advance applies only to the
#   extrusion laid along the leaders' path, as Klipper advances only
#   extruding moves that travel, so retracts, primes and other extrude-only
#   moves keep their own motion and their extrude-only caps; with a kernel,
#   a linear_pressure_advance is applied after the kernel to that
#   extrusion. Either way
#   ratio_ramp_time requires finite [extruder]
#   max_extrude_only_velocity and max_extrude_only_accel. Where the leaders
#   turn around (a wipe that reverses along the line) or come to rest across
#   an extrude-only move or dwell, the advance drops its lead to zero exactly
#   where their speed does: it keeps the lead through the input shaper's
#   spill around the rest, then releases it before the rest and rebuilds it
#   after over that spill plus the same reach, rising from the rest point,
#   so the extruder never asks much more than the advance's own k*r*accel
#   there; a rest the leaders pass through without stopping keeps the full
#   advance. Any extruder position step, or
#   velocity step beyond the leaders' fit rounding, is refused before it is
#   emitted; leaders without an acceleration-continuous kernel produce such
#   steps. The trade-off against a kernel is measured in "Evaluating an
#   extruder chain" in the README.
```

The kinematic lane constructor creates `AxisRail`/servo rails and applies the
travel and homing parser; follower steppers are created directly from their
`[motor]` sections. Therefore a follower axis does not need `position_max` or
the homing options.

`position_endstop` is parsed as `config.getfloat("position_endstop",
config.getfloat("position_min", 0.0))`: it defaults to the configured
`position_min`, and `position_min` defaults to 0. It must lie between
`position_min` and `position_max`. Axis homing options are not valid on
`[motor <name>]`. A kinematic lane must have its motors in the matching
`[kinematics]` role list; `[axis <name>] motors:` is for follower/non-kinematic
axes.


## `[post_processor <name>]`

Every post-processor requires `type`. The parameters below are the complete registry from `rust/trajectory/src/algos/` and must be supplied with the indicated bounds. Parameter values are finite numbers; the post-processor compiler validates the type's expected parameter set.

### `smooth_bell`

```
[post_processor x_shaping]
type: smooth_bell
smooth_time: 0.025
#   Required. Must be non-negative. A value of 0 compiles to no stage.
```

### `smooth_triangle`

```
[post_processor x_shaping]
type: smooth_triangle
smooth_time: 0.025
#   Required. Must be non-negative. A value of 0 compiles to no stage.
```

### `smooth_zv` and `smooth_mzv`

```
[post_processor x_shaping]
type: smooth_zv
frequency_hz: 40
#   Required. Must be positive.

[post_processor y_shaping]
type: smooth_mzv
frequency_hz: 40
#   Required. Must be positive.
```

### `linear_pressure_advance`

```
[post_processor extruder_pa]
type: linear_pressure_advance
k: 0.04
#   Required. Must be non-negative.
```

### `tanh_pressure_advance` and `recipr_pressure_advance`

```
[post_processor extruder_pa]
type: tanh_pressure_advance
linear_advance: 0.04
#   Required. Must be non-negative.
nonlinear_offset: 0.0
#   Required. Must be non-negative.
linearization_velocity: 10.0
#   Required. Must be positive.

[post_processor extruder_pa_recipr]
type: recipr_pressure_advance
linear_advance: 0.04
#   Required. Must be non-negative.
nonlinear_offset: 0.0
#   Required. Must be non-negative.
linearization_velocity: 10.0
#   Required. Must be positive.
```

### `mode_inverse`

```
[post_processor x_mode]
type: mode_inverse
frequency_hz: 40
#   Required. Must be positive.
damping_ratio: 0.7
#   Required. Must be in [0, 1).
```

Attach a named post-processor to an axis with `post_processors: name1, name2`. On an axis that follows others, an advance (`linear_pressure_advance`, `tanh_pressure_advance`, `recipr_pressure_advance`, or another derivative-gain stage with a nonzero gain) must share its chain with a smoothing kernel: an advance turns every extrusion-ratio change taken at speed into an instant position step, which only a kernel makes followable, so the config compiler rejects the chain without one. A zero advance is accepted, and raising it at runtime on such a chain is rejected the same way. The fork has ZV, MZV, bell, and triangle smoothing kernels; it does not provide an EI-family input-shaper type.

To judge an extruder chain (pressure advance plus its smoothing kernel) before printing, run `python3 tools/corner_flow.py printer.cfg`: it reports bead over- and under-fill at standard corners, flow changes and travel transitions, and accepts `--sweep smooth_time=...` and `--set name.param=value`. See [Evaluating an extruder chain](../README.md#evaluating-an-extruder-chain) for how to read it.

## `[extruder]`

The motion-owned portion of `[extruder]` is limited to the axis association and extrude-only caps. Heater, thermistor, and other extruder options remain mainline Klipper options and are outside this motion reference.

```
[extruder]
#axis: e
#   The follower axis the extruder drives. Its [axis] max_velocity and
#   max_accel inherit the caps below when unset.
#max_extrude_only_velocity:
#   Optional. Must be above 0 (mm/s). Default, as in upstream Klipper, is
#   [printer] max_velocity * 4 * nozzle_diameter^2 / filament_area, with
#   filament_area = pi * (filament_diameter/2)^2; no default without both
#   diameters.
#max_extrude_only_accel:
#   Optional. Must be above 0 (mm/s^2). Default is [printer] max_accel
#   scaled the same way.
```

When using the pressure-advance compatibility command, `axis` identifies the `[axis <name>]` chain from which `linear_pressure_advance` (`k`) and `smooth_triangle` (`smooth_time`) targets are resolved. The compatibility command is documented in the migration guide; the configuration itself uses named post-processors.

## Source locations

* `[printer]`, `[kinematics]`, `[axis]`, post-processor declarations, and extruder caps: `rust/planner-config/src/from_doc.rs:333-400, 408-424, 570-642`.
* Stepper motor options and step-distance/gear-ratio parsing: `klippy/stepper.py:188-256`; enable pin registration: `klippy/extras/stepper_enable.py:117-119`.
* Axis travel and homing options: `klippy/stepper.py:307-317` and `klippy/rail.py:26-70`.
* Servo motor options: `klippy/extras/servo_axis.py:87-143`.
* Pressure-advance compatibility axis association: `klippy/extras/pressure_advance_compat.py:106-139`.
* Post-processor registry and parameter bounds: `rust/trajectory/src/algos/*.rs` and `rust/trajectory/src/algos/mod.rs:19-78`.
