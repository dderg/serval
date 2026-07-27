# Kalico — sota-motion

This is a fork of [Kalico](https://github.com/KalicoCrew/kalico) (itself a
fork of [Klipper](https://github.com/Klipper3d/klipper)) that replaces the
motion stack. The upstream README, with Kalico's feature list and install
instructions, is at [README_KALICO.md](README_KALICO.md).

Everything here is under active development on the `sota-motion` branch.
The pieces described below exist and run on real hardware, but interfaces,
config formats, and even some of the design decisions may still change.

## Goals

The aim is to print faster without giving up quality, by removing the
approximations that classical planners are built from — trapezoidal
profiles, instant velocity-vector changes at corners, a single
accel-to-decel ratio — and
replacing them with a model where every limit is stated explicitly and
applied where it actually binds. A second goal is to keep the machine
model small: a printer is a set of axes with relations between them, and
the planner does not know what any axis is for.

## The planner

The motion pipeline is four streaming stages — fitter, planner, lowerer,
shaper — each a pure stage on its own thread. The entry point is
`setup_pipeline` in `rust/motion-core/src/worker.rs`.

The fitter turns incoming moves into smooth geometry. Sharp junctions are
replaced with arc and clothoid easing blends inside a corner-deviation
budget (configured directly, or derived from a classic
`square_corner_velocity` value), so cornering speed comes from the axis
limits and the local curvature of the rounded path rather than from a
velocity carve-out at the corner. The input can also be curved to begin
with: G5 / G5.1 cubic Bézier moves are accepted directly.

The planner then finds a jerk-limited velocity profile over a lookahead
window, with limits applied along the path: where the gantry's
acceleration limit binds, the profile rides it; where an extruder flow
limit takes over, it rides that instead.

Jerk is currently a per-axis constraint like velocity and acceleration.
Honestly, it may not stay one: the smoothing post-processors (below)
bound the same physical quantity more directly, and in practice a short
bell kernel (~2 ms) seems to be all the smoothing a fast machine needs.
Whether an explicit jerk limit earns its keep is an open question we're
still testing.

## Axes

Internally there is no toolhead and no extruder concept. A printer is a
set of axes; a kinematics section maps them onto motors, and an axis can
declare that it follows other axes:

```
[kinematics]
type: cartesian
axis_x: x
axis_y: y
axis_z: z
x_motors: x
y_motors: y
z_motors: z

[axis e]
follows: x, y, z
motors: extruder
post_processors: pa
```

A follower axis pays out its commanded displacement in proportion to the
distance actually traveled along the path of the axes it follows. The
extruder is the obvious example, but nothing in the system knows it's an
extruder. Because following is measured along the real path in 3D, the
cases that needed special handling before — vase mode, retract while
z-hopping, extrude-only moves — are just moves.

The global limits are configured the classic way, in `[printer]`:
`max_velocity`, `max_accel`, `max_jerk`, `square_corner_velocity`,
`max_z_velocity`, `max_z_accel`. A more general per-axis-group limit
model may come later, but it isn't there today.

## Post-processors

Input shaping and pressure advance are the same kind of object here: a
linear operator applied to one axis's motion. They are declared the same
way and can be chained:

```
[post_processor is]
type: smooth_bell
smooth_time: 0.018

[post_processor pa]
type: linear_pressure_advance
k: 0.045

[axis x]
post_processors: is

[axis e]
follows: x, y, z
post_processors: pa
```

Seven types exist today. `smooth_bell` and `smooth_triangle` are plain
low-pass kernels parameterized by `smooth_time`. `smooth_zv` and
`smooth_mzv` are frequency-targeted input smoothers parameterized by
`frequency_hz`; their kernel duration is derived from the target
frequency. `linear_pressure_advance` sharpens the extruder signal to
compensate pressure lag. `nonlinear_pressure_advance`
(`linear_advance`, `nonlinear_offset`, `linearization_velocity`) does
the same with a saturating law,
`linear_advance·v + nonlinear_offset·tanh(v / linearization_velocity)`,
so the commanded advance stops growing once flow is past the
linearization velocity — the nozzle pressure response flattens there and
the purely linear model over-advances. `mode_inverse` inverts an
identified second-order resonance (belt compliance, for example) so the
toolhead follows the nominal path with the residual scaling with model
error; it must be preceded in the chain by a short smoothing kernel, and
the config compiler enforces that ordering.

Limits apply to the output of the chain — the signal the motor actually
receives — not to the nominal command. Pressure advance spikes extruder
velocity during acceleration, so corners where that spike would exceed
the flow limit are slowed, and only those.

Post-processor parameters are tunable at runtime: a live update
recompiles the axis chains, revalidates them, and swaps them into the
running planner. Corner deviation and acceleration caps can be changed
the same way.

## Kinematics, motors, drives

Axes are what the planner thinks in; motors are what the printer is
built from. A kinematics module connects the two — cartesian and corexy
are supported today, and other geometries are not yet. A motor is a named
object bound to an axis through that module.

A motor can be driven three ways: classic step/dir, phase stepping (a
stepper opts in with `phase_stepping: 1` in its existing section), or an
EtherCAT servo drive. The planning side doesn't know which. The EtherCAT
path runs on a test bench today — an industrial servo on X, steppers
elsewhere:

```
[ethercat_node node_x]
interface: eth0

[servo_x]
protocol: ethercat
node: node_x
max_torque: 100
velocity_ff: True
params:
  0x2000.0x05: u16 0      # manual gain mode (no auto-tuning)
  0x2000.0x07: u16 37     # load inertia ratio, % (servo-ident fit)
  0x2001.0x01: u16 2200   # position gain, 220 rad/s
  0x2001.0x02: u16 1375   # speed gain, 137.5 Hz
  0x2001.0x03: u16 909    # integral time, 9.09 ms
dynamics_profile: servo_dynamics/dynamics_ident_20260611_181313.toml
```

The `params:` block writes into the drive's object dictionary at startup,
so loop gains live in version-controlled config rather than a vendor
tuning GUI. `dynamics_profile` points at the output of the fork's own
identification routine, which excites the axis, fits its dynamics
(including friction), and feeds the result forward as torque in addition
to velocity feedforward. There is a calibration suite around this —
G-code commands for gain and inertia identification and tracking
measurement, tuning profiles, telemetry capture to file, and a live
dashboard — plus torque-threshold sensorless homing on servo axes.

Standing up the EtherCAT master on a Raspberry Pi 5 (PREEMPT_RT kernel,
IgH master with the native `ec_macb` driver) is documented in
[docs/rewrite/ethercat-igh-macb-install.md](docs/rewrite/ethercat-igh-macb-install.md),
with [docs/rewrite/ethercat-bench-bringup.md](docs/rewrite/ethercat-bench-bringup.md)
for the drive side.

## The MCU plays motion, not steps

The host writes each axis's final motion — planned, followed, shaped —
as polynomial position pieces and streams them to the microcontroller,
which evaluates them at a fixed sample rate and produces step edges or
phase currents from the true continuous position. The MCU holds the
actual trajectory, not a precompiled queue of step times. That is what
makes smooth phase stepping possible, and the same stream feeds servo
drives their position setpoints.

Supported MCU targets: STM32 H7, F4, G0, and a Linux-process MCU. The
motion sample rate is a per-target build option.

## Homing and probing

Homing plans a guarded run toward the endstop and, on trigger, matches
the trip against the streamed trajectory to reconstruct the exact trip
position (including overshoot) and re-anchor the axis. Endstops on
remote MCUs and probe-style triggers (Beacon and similar) go through the
same mechanism. Sensorless homing exists on the servo path via a torque
threshold.

## Development infrastructure

Most day-to-day verification happens off-hardware:

- **Simulator** (`tools/sim/`): runs the real firmware binaries and the
  real host against faked hardware on virtual clocks, in Docker. Used
  for end-to-end G-code runs, homing and probing tests, and comparing
  behavior between branches. Part of CI.
- **Trajectory snapshot tests** (`snapshots/`): drive the real planner
  over a config × G-code matrix and diff the full output trajectory
  against committed baselines, with a browser gallery for reviewing
  before/after when behavior changes.
- **Playground**: the actual pipeline compiled to WASM —
  [dderg.github.io/kalico/playground](https://dderg.github.io/kalico/playground/)
  — paste G-code, tweak config, watch it re-plan in the browser.
- **Structured logging**: host and MCU emit structured events
  (`events/*.jsonl`) instead of free-form log text, queryable with
  VictoriaLogs; the MCU keeps a diagnostic ring and dumps prior-crash
  forensics on reboot.

## Status

Active development; expect breakage and change. The architecture
documents in [docs/rewrite/](docs/rewrite/) describe the design and are
kept closer to the code than this file — when they disagree, trust the
docs and the code.
