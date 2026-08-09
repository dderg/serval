# Serval motion configuration examples

These examples teach the **shape** of a Serval configuration. They are not ready-to-flash printer files: every pin, serial path, rotation distance, travel limit, endstop polarity, motor direction, and safe limit must be replaced with values verified for the actual machine. Start from a backed-up working configuration and follow [Config migration](Config_Migration.md).

## Minimal Cartesian topology

```ini
[mcu]
serial: /dev/serial/by-id/REPLACE-WITH-YOUR-MCU

[printer]
max_velocity: 200
max_accel: 3000
corner_deviation: 0.04
# Set exactly one corner setting. Do not add square_corner_velocity here.

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
step_pin: REPLACE
dir_pin: REPLACE
rotation_distance: 40
microsteps: 16

[motor y_motor]
drive: stepper
step_pin: REPLACE
dir_pin: REPLACE
rotation_distance: 40
microsteps: 16

[motor z_motor]
drive: stepper
step_pin: REPLACE
dir_pin: REPLACE
rotation_distance: 8
microsteps: 16

[axis x]
endstop_pin: ^REPLACE
position_min: 0
position_max: 220
position_endstop: 0
homing_speed: 20

[axis y]
endstop_pin: ^REPLACE
position_min: 0
position_max: 220
position_endstop: 0
homing_speed: 20

[axis z]
endstop_pin: ^REPLACE
position_min: -2
position_max: 250
position_endstop: 0
homing_speed: 8
```

The `[kinematics]` section owns motors for the X/Y/Z lanes. Do **not** repeat `motors:` in the matching kinematic `[axis]` sections. The axis sections own rail travel and homing data. `max_velocity` and `max_accel` are required; units are mm/s and mm/s².


## CoreXY topology

CoreXY names physical diagonal lanes A and B while retaining Cartesian planned axes X and Y:

```ini
[kinematics]
type: corexy
axis_x: x
a_motors: a_motor
axis_y: y
b_motors: b_motor
axis_z: z
z_motors: z_motor

[axis x]
position_min: 0
position_max: 250
# Add the correct endstop and homing settings for this machine.

[axis y]
position_min: 0
position_max: 250

[axis z]
position_min: -2
position_max: 250
```

Define `a_motor`, `b_motor`, and `z_motor` as `[motor]` sections exactly as in the Cartesian pattern. Only `cartesian` and `corexy` are accepted by the current native topology reader. A CoreXY configuration cannot use a keyed, per-motor endstop block for X/Y because those axes do not map 1:1 to one motor lane.

## Follower extruder with pressure advance

An extruder is represented as a follower axis that pays out along actual XYZ path length:

```ini
[motor extruder_motor]
drive: stepper
step_pin: REPLACE
dir_pin: REPLACE
enable_pin: !REPLACE
rotation_distance: 7.1
microsteps: 16

[post_processor extruder_pa]
type: linear_pressure_advance
k: 0.040

[axis e]
follows: x, y, z
motors: extruder_motor
post_processors: extruder_pa
```

Follower axes use `[axis] motors:` because they are not assigned a kinematic lane. They do not need `position_max` or rail homing fields. The follower relationship is geometry-based, so do not add classic planner-specific extruder motion options without checking the migration guide and reference. Pressure-advance output is constrained as motor motion; values must be calibrated for the machine.

## Axis smoothing and ordered chains

```ini
[post_processor x_smooth]
type: smooth_mzv
frequency_hz: 42

[post_processor x_mode]
type: mode_inverse
frequency_hz: 42
damping_ratio: 0.12

[axis x]
post_processors: x_smooth, x_mode
# plus the rail fields from the topology example
```

Order matters: `mode_inverse` requires a preceding smoothing kernel and the compiler rejects an unsafe ordering. For all processor types, units, and bounds—including nonlinear pressure-advance forms—use [Motion configuration reference](Config_Reference_Motion.md). `mode_inverse` has a documented acceleration-model limitation in [Feature status](Feature_Status.md); do not enable it casually.

## Multi-motor Cartesian rail

For a cartesian 1:1 lane with two motors and two switches, declare both motors in the lane and key the endstops by motor name:

```ini
[kinematics]
type: cartesian
axis_x: x
x_motors: x_left, x_right
# other lanes omitted

[axis x]
endstop_pin:
  x_left: ^REPLACE_LEFT
  x_right: ^REPLACE_RIGHT
position_min: 0
position_max: 300
position_endstop: 0
homing_speed: 15
```

Each motor must occur exactly once and each switch must be on the MCU driving its motor. The first switch freezes its associated motor; the last stops the axis, allowing gantry squaring. This is not valid for CoreXY shared lanes, virtual/sensorless endstops, or an axis without a 1:1 motor-lane relationship.

## Pre-flight checklist

- Use one and only one of `corner_deviation` and `square_corner_velocity`.
- Confirm every named axis, motor, post-processor, and MCU exists and names are consistent.
- Confirm physical direction and endstop polarity with motors safely unloaded where possible.
- Flash matching firmware after changing Serval branch/version; restart and resolve parser errors rather than bypassing them.
- Home conservatively, then validate travel limits before heater or print testing.
- Keep the machine attended during first motion after topology, firmware, drive, or transport changes.
