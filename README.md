# Kalico — sota-motion

This is a fork of [Kalico](https://github.com/KalicoCrew/kalico) (itself a
fork of [Klipper](https://github.com/Klipper3d/klipper)) that replaces the
motion stack. The upstream README, with Kalico's feature list and install
instructions, is at [README_KALICO.md](README_KALICO.md).

The short version: each move's timing is solved as a time-optimal control
problem instead of being approximated with lookahead heuristics, and the
machine model is reduced to a single concept — axes — with two relations
between them.

## The planner

Classical planners are built from approximations: trapezoidal velocity
profiles, a lookahead queue joining them, a single square-corner-velocity
number standing in for cornering dynamics, an accel-to-decel ratio papering
over the difference between commanded and felt acceleration. All of these
stand in for a question that used to be too expensive to answer directly:
what is the fastest way through this path that stays within every limit?

This planner answers it directly. It discretizes each move's timing and
solves for the fastest profile that satisfies every constraint pointwise
along the path. Where your gantry's acceleration limit binds, the trajectory
rides it; where the extruder's flow limit takes over, it rides that instead.

Square corner velocity is the clearest casualty. Taking a sharp corner at
any nonzero speed means the velocity vector changes direction instantly —
infinite acceleration. SCV is the agreement to permit a small dose of
infinity and cap it with one global number. Here the path is curves end to
end (G5 cubic Bézier input), so turning at speed is just acceleration, and
the same acceleration limits that govern straights govern every turn; where
the input does contain a genuinely sharp junction, junction deviation
(planned) replaces it with real rounded geometry inside a configured
tolerance, which then gets planned like everything else. Cornering speed
emerges per-corner from your limits and the local curvature — never from a
fudge factor, never from infinity.

Motion is also genuinely third-order: jerk is a constraint row like
velocity and acceleration, solved per axis along the path, not a trapezoid
with sharp acceleration steps and a smoothing knob on top.

## Axes

There is no toolhead and no extruder concept in this model. A printer is a
set of axes. Two things can be said about an axis: what it follows, and
which limits cover it.

```
[axis e]
follows: x, y, z

[limit gantry]
axes: x, y
max_velocity: 500
max_accel: 30000

[limit extruder]
axes: e
max_velocity: 75
max_accel: 1500
```

A follower axis pays out its commanded displacement in proportion to the
distance actually traveled along the path of the axes it follows. The
extruder is the obvious example, but nothing in the system knows it's an
extruder. Because following is measured along the real path in 3D, the cases
that needed special handling before — vase mode, retract while z-hopping,
extrude-only moves — are just moves.

Limits work the same way for every axis: each `[limit]` section caps the
combined motion of the axes it names, and all sections constrain the shared
move clock together. A slow extruder limit slows the gantry exactly where the
flow demand would exceed it, and nowhere else. The planner never knows what
any axis is for — which is exactly why adding a second extruder, a paste
head, or anything else that should track the print path is a config section,
not a feature request.

## Kinematics, motors, drives

Axes are what the planner thinks in; motors are what the printer is built
from. A kinematics module connects the two — cartesian and corexy today, a
new geometry is one new module — and in this model a motor is an arbitrary
named object bound to an axis through that module, nothing more (`stepper_x`
on a corexy machine was always a polite fiction, and here the fiction has
nowhere to live). A motor is also whatever actually produces the motion: a
classic step/dir stepper, a phase-stepped one, or an EtherCAT servo — the
fork speaks to servo drives natively. Nothing on the planning side knows or
cares which drive technology sits at the end.

This is running hardware, not a roadmap. From one of the test benches, where
the X axis is an industrial EtherCAT servo and the other axes are steppers
(a stepper opts into phase stepping with `phase_stepping: 1` in its existing
section):

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

The `params:` block writes straight into the drive's object dictionary at
startup — loop gains live in version-controlled printer config, not in a
vendor tuning GUI. And the inertia ratio comment isn't a guess:
`dynamics_profile` points at the output of the fork's own identification
routine, which excites the axis, fits its dynamics, and feeds the result
forward.

Standing this up on a Raspberry Pi 5 — the PREEMPT_RT kernel, the IgH EtherCAT
master built with the native `ec_macb` driver, and the drive bring-up — is
documented in
[Installing the IgH EtherCAT master with native `ec_macb`](docs/rewrite/ethercat-igh-macb-install.md),
with [`ethercat-bench-bringup.md`](docs/rewrite/ethercat-bench-bringup.md) for
the drive side.

## The MCU plays motion, not steps

The host writes every axis's final motion — planned, followed, shaped,
post-processed — as cubic position curves and streams those to the
microcontroller, which simply plays them back. The MCU holds the actual
trajectory, not a precompiled queue of step times. That is what unblocks
smooth phase stepping: a driver that knows the true continuous position at
every instant can place the stator field exactly there, instead of
stair-stepping toward it. The same stream feeds servo drives their position
setpoints.

## Post-processors

Input shaping and pressure advance turn out to be the same kind of object: a
linear operator applied to one axis's motion. One smooths the signal to avoid
exciting resonances, the other sharpens it to compensate pressure lag in the
melt zone. They are declared the same way and can be chained:

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

The smoothing kernels come in two families, and each takes its honest
parameter. `smooth_bell` and `smooth_triangle` are plain low-pass kernels —
no frequency selectivity, just smoothing — so they take `smooth_time`: more
time, more smoothing. `smooth_zv` and `smooth_mzv` are the bleeding_edge_v2
input smoothers (Maxima-optimized polynomials whose lobe structure cancels
a target resonance band), so they take `frequency_hz`: the kernel duration
is derived (`0.8025 / f` and `0.95625 / f` respectively), and making it
longer would move the notch off the resonance, not suppress it harder —
mzv trades a wider window for a broader suppression band.

`mode_inverse` is the third kind of linear operator: kernels smooth, pressure
advance sharpens, and this one inverts. Given an identified belt-compliance
resonance (`frequency_hz`, `damping_ratio`), it commands the motor through the
inverse of that second-order model — `x + (2ζ/ω)·ẋ + (1/ω²)·ẍ` — so the
toolhead follows the nominal path with zero added deviation and zero delay;
residual ringing scales with the model error rather than the excitation. The
ẍ term amplifies high frequencies, so it must be paired with a short smoothing
kernel that bandlimits its input (e.g. `smooth_bell` with `smooth_time:
0.0015`) listed before it in the chain — the config compiler enforces that
ordering.

Limits apply to the output of the chain — the signal the motor actually
receives — rather than to the nominal command. The planner folds the chain
into its constraints: pressure advance spikes extruder velocity during
acceleration, so the corners where that spike would exceed the flow limit
are slowed, and only those. The same rule will let the planner command
tighter nominal corners on shaped axes, knowing the shaper rounds them
before the motor sees them. Post-processor parameters are designed to be
tunable at runtime, and new types (a different pressure advance model, for
instance) plug in as new sections rather than code changes.

## Status

Under heavy development on the `sota-motion` branch. The geometry pipeline is
cubic-Bézier native (G5/G5.1 input) and the time-optimal solver is in place;
the follower and post-processor constraint families are landing now, with the
per-axis emission chain (including runtime tuning) planned and next. The
architecture documents in [`docs/rewrite/`](docs/rewrite/) describe the
design.
