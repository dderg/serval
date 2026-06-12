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
type: smooth_mzv
frequency_hz: 53

[post_processor pa]
type: linear_pressure_advance
k: 0.045

[axis x]
post_processors: is

[axis e]
follows: x, y, z
post_processors: pa
```

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
design documents in
[`docs/superpowers/specs/`](docs/superpowers/specs/) are the source of truth;
start with
[the follower-axes-and-limits design](docs/superpowers/specs/2026-06-12-follower-axes-and-limits-design.md).
