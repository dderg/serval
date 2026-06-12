# Kalico — sota-motion

**A ground-up rewrite of 3D printer motion. The planner computes the
time-optimal trajectory — not a good approximation, the real thing — and the
machine model is so simple it fits in one sentence: a printer is a set of
axes, some of which follow others, all of which obey one shared rulebook.**

This is a fork of [Kalico](https://github.com/KalicoCrew/kalico) (itself a
fork of [Klipper](https://github.com/Klipper3d/klipper)). Everything you know
from Kalico is still here — see [README_KALICO.md](README_KALICO.md) — but
the motion stack underneath is being replaced entirely.

## Your printer is slower than physics requires

Every classic planner is a stack of heuristics: trapezoids, lookahead,
square-corner-velocity, a centripetal cap, per-axis accel knobs that secretly
let diagonals exceed your configured limit by √2. Each one is a safe
approximation of the question nobody could afford to answer exactly:

> *What is the fastest way through this path that violates no limit?*

We answer it exactly. Every move's timing is solved as a constrained
time-optimal problem: the trajectory rides whichever limit binds at each
point along the path — your gantry's acceleration through one corner, your
extruder's flow ceiling through the next — and is provably as fast as those
limits allow everywhere in between. There is no cornering knob to tune,
because cornering is not a special case. There is no "safe" margin baked in,
because the solver does not need one.

## The extruder was never special

Delete the toolhead. Delete the extruder concept. What remains is honest:

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

An axis that `follows` other axes pays out its motion proportionally to the
distance *actually traveled* along their path — an odometer, not a script.
The extruder is just the first follower. From this one rule, whole categories
of special cases simply stop existing: vase mode, retract-with-hop, spiral
lift, extrude-only moves — all ordinary moves now. And because every axis's
limits pour into the same pot, your extruder's flow limit slows the gantry
exactly where it must and nowhere else. Want a second extruder, a paste head,
a fiber tensioner? Declare another axis. The planner doesn't know what an
axis is *for* — and that's precisely why it can plan anything.

## Pressure advance and input shaping are the same thing

One smooths your motion to cancel ringing. The other sharpens your extruder
to cancel pressure lag. Opposite effects — identical mathematics. So they are
one concept here, declared the same way, applied per axis, in chains, tunable
live:

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

And here is the payoff: the planner constrains what comes **out** of that
chain — what your motors and your nozzle actually feel — not the nominal
command. It knows the shaper will round a corner before the motor sees it, so
it may legally command a sharper one. It knows pressure advance will spike
extruder velocity at each accel, so it slows exactly the moves where that
spike would exceed your flow limit — and no others. Your tuning numbers
finally mean what they say.

New post-processor types plug in the same way. Someone's better
pressure-advance model is a config section away, not a fork away.

## The bookkeeping is honest too

The G-code's extrusion counter is a contract, executed exactly as written.
The physical filament paid out follows the *real* road — which is shorter
where smoothing rounds a corner, so proportionally less plastic goes down.
That's not drift; that's the first planner that refuses to overextrude road
that doesn't exist. The books are a contract; the road is physics; nobody
rewrites the books to match the road.

## Where this stands

Under heavy development on the `sota-motion` branch. The geometry pipeline is
cubic-Bézier native (G5/G5.1), the time-optimal solver with follower and
post-processor constraints is in place, and the per-axis emission chain is
being built now. The full design — readable, opinionated, and the actual
source of truth for the code — lives in
[`docs/superpowers/specs/`](docs/superpowers/specs/), starting with
[the follower-axes-and-limits design](docs/superpowers/specs/2026-06-12-follower-axes-and-limits-design.md).

Upstream Kalico — features, documentation, installation:
[README_KALICO.md](README_KALICO.md).
