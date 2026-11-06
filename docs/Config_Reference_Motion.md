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
#corner_deviation:
#   Optional corner budget in mm, no default: how far the toolhead path
#   after the input shaper (the shaped path) may miss each corner's G-code
#   vertex. A strict promise: where even a stop at the corner would round
#   it by more (the shaper's floor, see "Corner budget on the shaped
#   path" below), the planner slows the acceleration into and out of that
#   corner until it fits. Must be >= 0. Set this XOR
#   square_corner_velocity; with neither set, each corner may miss by its
#   shaper floor plus 0.04 mm.
#square_corner_velocity:
#   Optional, no default, in mm/s. Must be >= 0. Set this XOR
#   corner_deviation. Each corner may miss its vertex by its shaper floor
#   plus what crossing a sharp 90° corner at this speed adds to the floor
#   through the same kernels, both at the corner's own planned
#   acceleration: the smoothing Klipper's square_corner_velocity gives.
#   Without an X or Y kernel it is the legacy cut
#   scv^2 * (sqrt(2) - 1) / accel at each move's planned acceleration.
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
#   both ends can carry while staying within e. Where no kernel follows the
#   advance, a piece the looser budget would carry past the axis's
#   max_accel plus this is held to this.
```

Setting both corner options is an error. `max_accel_to_decel` and `minimum_cruise_ratio` are explicitly unsupported rather than ignored.

`max_z_velocity` and `max_z_accel` bound the z axis alone, not the toolhead path. The planner applies them to each fitted segment after corner blending, using the z share of that segment's heading and of its curvature. That covers corners between an XY line and an XYZ move, such as layer changes and z-hops: the turn can put z acceleration on the blend even where its heading barely climbs, so the planner slows the blend until z stays within both caps. The caps never raise a limit; they only lower the speed ceiling and the acceleration budget of segments that move z.

### MCU step capacity

Each MCU measures at connect what one step costs it, in clock ticks: once for pulsed motors and once for both-edge (TMC `dedge`) motors. The figure is the timer interrupt time of one isolated motor's step with a cold instruction cache, the worst case a lone motor at speed meets (`printer.mcu.step_benchmark`; see `tools/bench/README.md`, "Self-benchmark"). Klippy hands the planner each MCU's benchmark, clock, stepped motors and the capacity fraction it applies (reported as `printer.mcu.step_capacity`). The planner gives every MCU a capacity of `STEP_CAPACITY_FRACTION` (0.80) of its clock in ticks per second; `rust/motion-pipeline/src/live_config/step_capacity.rs` is the one derivation the live planner and every snapshot share. On the F401 held at the cap, p99.9 step lateness × each motor's step rate stays within 0.5 microstep up to 0.85 for one motor alone, and the error jumps at 0.9. 0.80 keeps a margin from that cliff for printers running more MCU tasks than the bench (`tools/bench/README.md`, "Capacity fraction from a timing budget").

The planner holds every move within the capacity of every MCU it steps. A motor stepping `d` mm per step at `c` ticks per step costs its MCU `c / d` ticks per mm it travels. A motor of a kinematic lane with weights `w` (CoreXY: `a = x + y`, `b = x - y`) travels at `|w·t̂|·v` at path speed `v` along heading `t̂`. On an arc or a blend, the planner takes the largest `|w·t̂|` over every heading the segment sweeps, which bounds the motor speed at every point. The extruder motor travels at `r·(v + k·A)`, the extrusion ratio and the pressure advance's velocity gain `k` over the acceleration budget `A`, as for its own `max_velocity`. Per MCU, the sum of `motor speed × c / d` over the motors it steps must stay within the capacity. This constraint is linear in the speed ceiling and the acceleration budget. Where it binds, the planner trades the two as the extruder's velocity cap does: of the pairs on the capacity line, it takes the one that crosses the run between the slow seams around the move soonest, and it never keeps a ceiling above the speed the plan can reach there. Only moves that ask too much are slowed, and only the MCUs they drive count.

The first move the cap slows emits a `step_capacity_capped` host event at once, later ones at most every 30 s. Each event carries the MCU name, the moves capped since the last report and the worst demand as a multiple of the capacity. At startup klippy logs one line per MCU: the measured ticks per step, the capacity in steps/s for each mode in use, each motor's worst-case steps/s at the configured maximum speeds (`max_velocity` in its worst XY direction, `max_z_velocity`, the extruder's velocity limit) and whether the cap can bind. Simulated MCUs log that their capacity is not modeled, and they are not capped.

The fraction assumes everything else an MCU does fits in the remaining `1 − fraction` of its CPU. That covers tasks, USB and other interrupts, and non-step timers such as software PWM. Klippy checks this assumption while printing. Every 250 ms each stepping MCU reports where its CPU went (`cpu_load`). A window whose non-step share exceeds `1 − fraction` is logged as `mcu.cpu_headroom_exceeded`, at most every 30 s. If, in the same window, the steps at their benchmarked cost plus that non-step load exceed the whole CPU, klippy shuts down: the cap no longer protects step timing on that MCU. The message names the MCU and the shares. The stats line carries the worst window per period as `mcu_nonstep_max` and `mcu_idle_min`. Firmware without `cpu_load` is refused when it drives steppers (`tools/bench/README.md`, "Runtime headroom check").

Klippy refuses a config only when an MCU cannot step a pure move along an axis at `min(that axis's max velocity, 50 mm/s)` with the extruder idle (`STEP_CAPACITY_MIN_AXIS_VELOCITY`; first-layer and perimeter speeds start around there). The error names the MCU, its measured cost and capacity, each motor's demand, and the remedies: fewer microsteps or a larger `rotation_distance`, both-edge stepping, or moving motors to another MCU. A stepper bound to an MCU that reports no step benchmark (a probe MCU without steppers) is also an error. Every axis that an MCU with a measured capacity steps needs extruder limits: `[axis] max_velocity`, or an `[extruder]` bound to it.

Each motor is also limited to what its driver can physically be stepped at. That limit is one step per two pulse widths in pulsed mode (`stepper_full_finish` holds the next step until a pulse width after the unstep) and one per microsecond on both edges. The host checks every shaped track against it before sending.

Worked example: the bench printer (`tools/bench/printer.cfg`: CoreXY, 80 steps/mm on a and b, 400 on z, 414 on e, PA `k = 0.03`, `max_velocity: 1000`, `max_accel: 10000`, F401 at 497 / 240 ticks per step, all pulsed). The capacity is 0.80 × 84 MHz / 497 = 135 k steps/s.

- A pure X or Y travel asks 160 steps/mm, so it is capped at 845 mm/s.
- A 45° diagonal drives only one motor, at 113 steps/mm, and can reach 1195 mm/s.
- Z can reach 338 mm/s, against `max_z_velocity: 25`.
- Extrusion along X or Y at 10000 mm/s² is capped below the travel cap. A 0.4 × 0.2 mm line (ratio 0.033, budgeted 0.037) is capped to 745 mm/s at full acceleration. A 0.8 × 0.3 mm line (ratio 0.100, budgeted 0.105) is capped to 600 mm/s.

The startup line reports about 190 % summed worst-case demand, so the cap can bind.

### Corner budget on the shaped path

The corner budget bounds the shaped path, the one the input shaper hands the motors, not the planned one: the miss of a corner is the closest the shaped path comes to its G-code vertex. Two things round a corner. The planner blends it with a clothoid pair whose apex cuts the vertex by a chosen amount, and the shaper's X and Y kernels pull the path further inward. A kernel averages the motion over its width, so a corner crossed faster, or braked into and accelerated out of harder, is rounded more. Even a full stop at the vertex at acceleration `A` is rounded: that is the corner's floor. It grows with the turn and in proportion to `A` and to the kernels' variance. At 10000 mm/s² behind the bench printer's 50 Hz MZV / 44 Hz ZV pair it is about 0.12 mm at 90° and 0.17 mm at 180°.

The planner predicts the shaped miss of every corner from the configured X and Y kernels (`rust/geometry/src/shaped_corner.rs`). The table belongs to the kernel pair and is filled lazily, a cell the first time a corner needs it, by convolving a canonical corner, worst over its orientation against the axes, with a 20% margin on the shaper's share. Each corner then gets the cut whose apex the shaped path may cross fastest within its budget, and an apex speed that keeps the predicted miss within it. Under `corner_deviation` and `square_corner_velocity`, arcs and merged runs of facets are fitted to a quarter of the budget, and their speed keeps the shaper's inward pull on the curve within the rest. Corners close together trace a curve the shaper shrinks as a whole. A blend's apex is therefore also held to the speed at which a circle through its turn, over the shorter leg to a neighbouring corner, keeps that shrink within the budget's room beyond the cut.

What the budget is depends on how it is set:

- **`corner_deviation`** is the whole budget, the same for every corner. Where a stop at the corner's planned acceleration would round it past the budget, the acceleration of a short approach either side of it (the path a stop at that acceleration covers in about six standard deviations of the wider kernel, ending at any stop on the way) drops to where a stop takes half the budget's room beyond the cut. The corner is held to its budget at a cost in time. At startup the planner logs a `corner_floor` event naming the floor at 90° and 180° at `max_accel` and the acceleration at which each fits the budget.
- **Neither option set**: each corner may miss by its floor at the acceleration it is planned at (after the extruder and step-capacity caps) plus 0.04 mm. The planner never slows a corner for this budget below the acceleration it plans anyway. This budget holds corners alone: a corner is a stop or a blend the planner cuts at a vertex, while arcs, merged runs of facets and clothoids easing between curves are fitted to 0.04 mm and left to the shaper, as before. A machine whose extruder caps hold corners well under `max_accel` keeps its sharp corners, and a corner taken at `max_accel` gets the shaper's own rounding plus 0.04 mm.
- **`square_corner_velocity`**: as above, with the allowance taken from Klipper. It is what crossing a sharp 90° corner at that speed adds to the floor, through the same kernels and at the same acceleration, which is the smoothing a Klipper machine with that `square_corner_velocity` and these shapers prints.

Without an X or Y kernel the shaped path is the planned one and the floor is zero: `corner_deviation` and the default are the planned cut, and `square_corner_velocity` converts to Klipper's junction deviation `scv² · (√2 − 1) / accel`.

The path-accuracy audit measures this on the corpus and every machine profile: `scripts/corpus.sh --path-accuracy` (`rust/pipeline-snapshot/tests/path_accuracy.rs`). It holds every extruding vertex of the shaped path to its budget, and under the default every corner the fitter keeps: a vertex inside a fitted arc run or a consumed cluster of facets is a curve. For the implicit budgets the audit takes the floor at `max_accel`, a looser bound than the planner's.

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
post_processors: extruder_pa
#   Optional comma-separated named post-processors. Default is an empty list.
#   On an axis that follows others, a pressure advance (or any derivative-gain
#   stage) needs the axis's max_velocity and max_accel, set or inherited; the
#   chain is rejected otherwise. The recommended extruder chain is the
#   advance alone; a smoothing kernel after it is an explicit override (see
#   the [post_processor <name>] section). Without one the advance also needs
#   finite [extruder] max_extrude_only_velocity and max_extrude_only_accel,
#   which bound the moves the extruder makes on its own (retracts, primes).
#max_velocity:
#max_accel:
#   Optional, on an axis that follows others. Must be above 0 (mm/s,
#   mm/s^2). What the axis's motor can do for every move it makes, pressure
#   advance included. The velocity planner slows each move so the commanded
#   extruder velocity and acceleration stay within them, corners and curves
#   included, and the axis's ratio ramp (below) derives each ramp time from
#   them. A move is held to the largest extrusion ratio within its reach:
#   its own, and that of every move the followed axes' shaping
#   post_processors and the longest ratio ramp carry onto it however short
#   the moves between, so a short travel between two extrusions accelerates
#   as they do. An extrude-only move does not quite separate the moves on
#   either side: the shaped path keeps moving for the shapers' reach past a
#   raw stop and before a raw start, and the extruder follows that motion at
#   the ratio it reads there. The toolhead holds still wherever an
#   extrude-only move meets motion until the shaped path has settled within
#   half of max_path_deviation of the rest point and the extruder has room
#   for what is left of that motion on top of the retract or prime, never
#   longer than the shapers' reach. On the [extruder] axis each one left
#   unset is inherited from max_extrude_only_velocity /
#   max_extrude_only_accel (the same motor). An advance needs both, set or
#   inherited.
#   On the [extruder] axis max_accel is the extruder motor's
#   pressure-advance-capable acceleration: the filament acceleration the
#   drive follows without skipping while the advance adds k times the
#   path's jerk to every corner and curve. It is a property of the drive,
#   not of the print, so set it explicitly rather than inheriting the
#   extrude-only cap, which only ever had to cover retracts and primes. A
#   value below what the motor can do costs print time at every corner; a
#   value above it skips steps. A direct drive typically follows 20000 or
#   more. A bowden drive pushes the whole filament column against the
#   tube's friction and the nozzle's back pressure, mostly through an
#   ungeared drive gear, and is lower; 5000 is a safe start for an
#   Ender-3-class bowden (with k around 0.5, upstream Klipper's default
#   40 ms advance smoothing already asks about 4400 at its corners). To
#   find the drive's value, print a pressure advance calibration pattern
#   (or any part with sharp corners and fast perimeters) at the tuned k,
#   raising max_accel between prints (it is read at restart). Stop at the
#   first print that shows any of: clicks or knocks from the extruder,
#   under-extrusion or gaps right after corners, or layer lines whose
#   width shifts from layer to layer; set max_accel 20-30% below the last
#   clean value.
#instantaneous_corner_velocity: 1.0
#   Optional, on an axis that follows others. Default is 1.0, as upstream
#   Klipper's [extruder] option. Must be above 0 (mm/s). The velocity
#   change the axis's motor takes at once. Where no smoothing kernel follows
#   the advance and a followed axis has no shaping post_processor (commonly
#   z), every acceleration step of that axis makes the advance step the
#   extruder velocity by k * ratio * the step's share along the path. The
#   velocity planner caps the acceleration of extruding moves that move
#   such an axis, and of the moves within their reach, so each step stays
#   within this; a larger step is refused with the time it happens at.
#   Leaders a shaping post_processor smooths make no such step, and no
#   allowance applies to them.
```

Every axis that follows others ramps its extrusion ratio along the
leaders' path; there is nothing to configure. The extruded total is
convolved along the path with a kernel whose half-width is half the path the
leaders cover in the ramp time τ (`v·τ/2` at speed `v`) and closes to zero at
every rest, so every extrusion-ratio step (travel to extrusion, a flow
change, a wipe) and every step in the ratio's slope (a corner blend's flow
ramp) is spread over about `v·τ` of path, however finely the path is cut
into moves. The laid ratio stays within the commanded ratios, and the
extruded material equals the unsmoothed projection's wherever the ratio is
constant across the kernel and at every rest.

Each step takes its own τ: the shortest between 5 and 30 ms that keeps the
advanced extruder within the axis's `max_velocity` and `max_accel` (V_e,
A_e) at the leaders' speed, acceleration and jerk there, reading the
advance through its slope `k` (a linear advance's `k`, a nonlinear one's
`linear_advance + nonlinear_offset / linearization_velocity`). For a step
`Δ = |Δr|·v` at a steady speed `v` onto a flow `r·v`, that is about
`τ = max(5 ms, √(6·k·Δ/A_e), 1.45·k·Δ/(V_e − r·v))`; without an advance,
`τ = max(5 ms, 1.5·Δ/A_e)`. An axis with neither limit set or inherited
ramps every step over 5 ms. Where even 30 ms would exceed the limits at
the speed the path allows, the velocity planner lowers the toolhead speed
at the step instead, and that time is the cost. Near a rest the kernel's
half-width is at most a quarter of the path left to it, so a step a few
ramp lengths short of an extrude-only move or the end of a print gets only
a fifth of the path to that rest on its side; where that is less than its
τ takes, the velocity planner slows the step until the shorter ramp fits
the limits. The former `ratio_ramp_time` option was
removed; setting it is a config error.

On every chain the advance applies only to the extrusion laid along the
leaders' path, as Klipper advances only extruding moves that travel, so
retracts, primes and other extrude-only moves keep their own motion and
their extrude-only caps; with a kernel, the advance is fitted into that
extrusion before the kernel smooths it. A retract the toolhead makes while
it moves (a wipe: a negative extrusion ratio along the path) is a retract
too, and is not advanced: the advance reads the laid ratio held at zero
wherever it is negative, ramped along the path exactly as the laid ratio
is, so it releases the lead of the extrusion before a wipe over the ramp
into it and rebuilds the lead of the extrusion after over the ramp out of
it, the same ramps the ratio step itself takes, with no velocity step.
The wipe's own filament motion `|r|·v` keeps the extrude-only caps, as in
Klipper: the velocity planner holds the move to
`max_extrude_only_velocity / |r|` and `max_extrude_only_accel / |r|`.
Where the leaders turn around (a
wipe that reverses along the line) or come to rest across an extrude-only
move or dwell, the advance drops its lead to zero exactly where their speed
does, rising from the rest point, so the extruder never asks much more than
the advance's own `k·r·accel` there. At a rest it releases the lead before
and rebuilds it after over the input shaper's spill around the rest alone
wherever the shortest ramp the extruder's limits allow for that lead fits
within the spill, over the spill plus that ramp's reach elsewhere, and over
the spill plus the reach of the longest ramp where a ratio step lies so near
the rest that its own ramp, at the speed the leaders cross it there, would
leave those limits; a rest the leaders pass through without stopping keeps
the full advance. Around a rest, the flow
switches from the move before's ratio to the move after's where the shaped
leaders reach the rest point, even where the shaped axes curve into it
unlike each other and cover more path than their straight distance from it;
the move before's material is spread over that whole path.
Without a kernel, any extruder position step, or velocity step beyond the
leaders' fit rounding, is refused before it is emitted; leaders without an
acceleration-continuous kernel produce such steps. The trade-off against a
kernel is measured in "Evaluating an extruder chain" in the README.

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

Attach a named post-processor to an axis with `post_processors: name1, name2`. On an axis that follows others, an advance (`linear_pressure_advance`, `tanh_pressure_advance`, `recipr_pressure_advance`, or another derivative-gain stage with a nonzero gain) needs the axis's `max_velocity` and `max_accel`, set or inherited from the `[extruder]`'s extrude-only caps; the config compiler rejects the chain without them. It needs no smoothing kernel: the recommended extruder chain is the advance alone, which the velocity planner holds to those limits and whose ratio steps it ramps (see `[axis <name>]` above). The fork has ZV, MZV, bell, and triangle smoothing kernels; it does not provide an EI-family input-shaper type.

A smoothing kernel after the advance on the extruder (`type: smooth_triangle`, named after the advance in `post_processors`) is an explicit override, never a default. It lowers the extruder acceleration corners ask for, so the planner slows extruding corners less, and pays with a bead bulge at every corner that grows with the kernel's width and the corner speed: on the bench printer at 150 mm/s a 45° corner bulges +159% with `smooth_time: 0.04` and +7% with 0.005, where the kernel-free chain lays the commanded bead. Use it only for an extruder too weak to follow the exact advance at a speed you need.

To judge an extruder chain before printing, run `python3 tools/corner_flow.py printer.cfg`: it reports bead over- and under-fill at standard corners, flow changes and travel transitions, and accepts `--set name.param=value`; on a config carrying a kernel override, `--kernel-free` compares it with the chain without the kernel and `--sweep smooth_time=...` varies its width. See [Evaluating an extruder chain](../README.md#evaluating-an-extruder-chain) for how to read it.

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
#   diameters. As in upstream Klipper it also caps a retract the toolhead
#   makes while it moves (a wipe): the move runs at no more than this over
#   its retract ratio.
#max_extrude_only_accel:
#   Optional. Must be above 0 (mm/s^2). Default is [printer] max_accel
#   scaled the same way, and it caps a moving retract's acceleration the
#   same way.
```

When using the pressure-advance compatibility command, `axis` identifies the `[axis <name>]` chain from which `linear_pressure_advance` (`k`) and `smooth_triangle` (`smooth_time`) targets are resolved. The compatibility command is documented in the migration guide; the configuration itself uses named post-processors.

## Source locations

* `[printer]`, `[kinematics]`, `[axis]`, post-processor declarations, and extruder caps: `rust/planner-config/src/from_doc.rs:333-400, 408-424, 570-642`.
* Stepper motor options and step-distance/gear-ratio parsing: `klippy/stepper.py:188-256`; enable pin registration: `klippy/extras/stepper_enable.py:117-119`.
* Axis travel and homing options: `klippy/stepper.py:307-317` and `klippy/rail.py:26-70`.
* Servo motor options: `klippy/extras/servo_axis.py:87-143`.
* Pressure-advance compatibility axis association: `klippy/extras/pressure_advance_compat.py:106-139`.
* Post-processor registry and parameter bounds: `rust/trajectory/src/algos/*.rs` and `rust/trajectory/src/algos/mod.rs:19-78`.
