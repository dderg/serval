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
#max_jerk: 6000
#   Optional. Default is 0, i.e. jerk limiting is disabled. Must be
#   >= 0. A value of 0 is converted to an unlimited jerk value.
#square_corner_velocity: 5
#   Optional legacy alias. Default is 5. Must be >= 0. Set this XOR
#   corner_deviation; it is converted to the corner-deviation budget.
#corner_deviation: 0.04
#   Optional canonical corner budget in mm. Must be >= 0. 0.04 is a good
#   starting value. Set this XOR square_corner_velocity.
#max_z_velocity: 300
#   Optional. Default is max_velocity. Must be > 0 and <= max_velocity.
#max_z_accel: 3000
#   Optional. Default is max_accel. Must be > 0 and <= max_accel.
#max_path_deviation: 0.005
#   Optional. Default is 0.005 mm. Must be > 0 and <= 1.
#max_accel_deviation: 50
#   Optional. Default is 50 mm/s^2. Must be > 0.
#pieces_wire_budget: 1024
#   Optional. Default is 1024 bytes. Must be 256..8192. Bytes one serial
#   PushPieces transaction may carry. The default is sized for 500 kbaud
#   UART; USB CDC transports move ~1 MB/s and can amortize their round
#   trip over larger frames.
#pieces_inflight: 12
#   Optional. Default is 12. Must be 1..16. PushPieces bundles the host
#   keeps in flight per serial MCU before waiting for the oldest
#   response. 1 restores classic stop-and-wait delivery.
```

`corner_deviation` is the canonical corner budget. Setting both corner options is an error. `max_accel_to_decel` and `minimum_cruise_ratio` are explicitly unsupported rather than ignored.

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
#use_sensorless_homing: False
#   Optional. Default is False unless endstop_pin is a virtual endstop.
#second_homing_speed: 2.5
#   Optional. Default is homing_speed / 2, or homing_speed for a virtual
#   endstop. Must be above 0.
#homing_positive_dir:
#   Optional. Default is inferred from position_endstop and the travel range;
#   an ambiguous interior endstop is an error.
#homing_accel:
#   Optional. Default is None. If set, must be above 0.
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
post_processors: extruder_pa
#   Optional comma-separated named post-processors. Default is an empty list.
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

Attach a named post-processor to an axis with `post_processors: name1, name2`. The fork has ZV, MZV, bell, and triangle smoothing kernels; it does not provide an EI-family input-shaper type.

## `[extruder]`

The motion-owned portion of `[extruder]` is limited to the axis association and extrude-only caps. Heater, thermistor, and other extruder options remain mainline Klipper options and are outside this motion reference.

```
[extruder]
#axis: e
#   Optional pressure-advance compatibility association. Default is None.
#max_extrude_only_velocity:
#   Optional. Default is None. If set, must be above 0 (mm/s).
#max_extrude_only_accel:
#   Optional. Default is None. If set, must be above 0 (mm/s^2).
```

When using the pressure-advance compatibility command, `axis` identifies the `[axis <name>]` chain from which `linear_pressure_advance` (`k`) and `smooth_triangle` (`smooth_time`) targets are resolved. The compatibility command is documented in the migration guide; the configuration itself uses named post-processors.

## Source locations

* `[printer]`, `[kinematics]`, `[axis]`, post-processor declarations, and extruder caps: `rust/planner-config/src/from_doc.rs:333-400, 408-424, 570-642`.
* Stepper motor options and step-distance/gear-ratio parsing: `klippy/stepper.py:188-256`; enable pin registration: `klippy/extras/stepper_enable.py:117-119`.
* Axis travel and homing options: `klippy/stepper.py:307-317` and `klippy/rail.py:26-70`.
* Servo motor options: `klippy/extras/servo_axis.py:87-143`.
* Pressure-advance compatibility axis association: `klippy/extras/pressure_advance_compat.py:106-139`.
* Post-processor registry and parameter bounds: `rust/trajectory/src/algos/*.rs` and `rust/trajectory/src/algos/mod.rs:19-78`.
