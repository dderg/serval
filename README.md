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
profiles, square corner velocity, a centripetal cap, per-axis acceleration
settings that quietly allow a diagonal move to exceed the configured limit by
√2. All of these stand in for a question that used to be too expensive to
answer directly: what is the fastest way through this path that stays within
every limit?

This planner answers it directly. It discretizes each move's timing and
solves for the fastest profile that satisfies every constraint pointwise
along the path. Where your gantry's acceleration limit binds, the trajectory
rides it; where the extruder's flow limit takes over, it rides that instead.
Cornering falls out of the same math as everything else, so there is no
corner velocity setting and nothing to tune besides the limits themselves.

## Axes

There is no toolhead in the config and no extruder concept in the code. A
printer is a set of axes. Two things can be said about an axis: what it
follows, and which limits cover it.

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
flow demand would exceed it, and nowhere else. Adding a second extruder, a
paste head, or anything else that should track the print path is a config
section, not a feature request.

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
receives — rather than to the nominal command. The planner accounts for this:
it knows the shaper will round a corner before the motor sees it, so it can
command a tighter one; it knows pressure advance spikes extruder velocity
during acceleration, so it slows only the moves where that spike would exceed
the flow limit. Parameters can be changed at runtime, and new post-processor
types (a different pressure advance model, for instance) plug in as new
sections rather than code changes.

## Extrusion bookkeeping

The G-code's extrusion counter advances exactly as written — macros and UIs
see the numbers the file commanded. The filament actually extruded follows
the realized path, which is slightly shorter where shaping rounds a corner,
so slightly less is paid out there. The slicer computed that filament for
road that, after shaping, doesn't exist; extruding it anyway would
overextrude the road that does. The discrepancy is deliberate and never
"corrected."

## Status

Under heavy development on the `sota-motion` branch. The geometry pipeline is
cubic-Bézier native (G5/G5.1 input), the time-optimal solver including
follower and post-processor constraints is in place, and the per-axis
emission chain is being built now. The design documents in
[`docs/superpowers/specs/`](docs/superpowers/specs/) are the source of truth;
start with
[the follower-axes-and-limits design](docs/superpowers/specs/2026-06-12-follower-axes-and-limits-design.md).
