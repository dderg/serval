# Kalico — sota-motion fork

A fork of [Kalico](https://github.com/KalicoCrew/kalico) (itself a fork of
[Klipper](https://github.com/Klipper3d/klipper)) that replaces the motion
stack end to end: a new planner, a new host↔MCU contract, and a new set of
abstractions for what a printer even *is*. Everything inherited from Kalico
that we have not rewritten still works the Kalico way — see
[README_KALICO.md](README_KALICO.md) for the upstream project, its feature
list, and installation.

This README describes the architecture we are building. It is under heavy
development on the `sota-motion` branch; the design source of truth lives in
[`docs/superpowers/specs/`](docs/superpowers/specs/), most centrally
[the follower-axes-and-limits design](docs/superpowers/specs/2026-06-12-follower-axes-and-limits-design.md).

## The pillars

### 1. Trajectory optimality is non-negotiable

The planner never trades trajectory time for planning convenience. Each
move's timing is discretized and solved as a constrained time-optimal
problem (TOPP via SOCP, with SLP linearization for the non-convex jerk
rows) — the move runs as fast as every constraint allows, pointwise along
the path, not as fast as a heuristic felt safe. Host compute is something we
spend in service of trajectory tightness, never the other way around: if the
host can't keep up, we optimize the implementation, parallelize, or upgrade
the host. We do not ship a cheaper algorithm that produces a measurably
slower print.

*Plainly: the printer should move as fast as physics and your config allow —
the planner's job is to find that speed, the computer's job is to afford the
search.*

### 2. Cubic Bézier all the way down

One geometric primitive flows through the entire pipeline: the uniform cubic
Bézier. G5 maps to it directly; G5.1 degree-elevates to it exactly; legacy
G0/G1/G2/G3 are converted upstream by the `compat` crate and **never reach
the planner** — anything else at the reduce boundary is a hard error, not a
fallback path. No arcs, no rational NURBS, no mixed-degree dispatch, no
per-source special cases anywhere live.

### 3. A printer is a set of axes — nothing else

There is no toolhead concept, no extruder concept, no hardcoded roles. An
axis is a config object; the code never knows what an axis is *for*. Exactly
two relations between axes exist, both explicit in config:

- **`follows`** — a follower axis pays out its commanded displacement
  proportionally to the *realized* distance traveled along the path of the
  axes it follows (the odometer rule). The extruder is just the canonical
  follower of `{x, y, z}`. Smoothed corners shorten the real road, so
  proportionally less follower motion happens — correct output, not error.
- **`[limit]` membership** — every limit names a set of coordinates and caps
  the magnitude of the motion vector restricted to them: velocity, accel,
  jerk, higher derivatives where declared. All sections contribute rows to
  one pot; one solve; no precedence, no cornering knobs, no
  square-corner-velocity. Coverage is mandatory — an axis in no limit
  section fails config load.

*Plainly: instead of "a printer has a toolhead with an extruder," the model
is "axes, some of which follow others, all of which obey one shared
rulebook." The extruder stops being special; vase mode, retract-with-hop,
and spiral lift stop being modes.*

### 4. Post-processors: one abstraction for shapers and pressure advance

An input shaper and pressure advance are the same mathematical object — a
linear time-invariant operator on a per-axis track. One smooths, one
sharpens; structurally they are identical. So there is one config object,
`[post_processor <name>]`, with a `type:` (`smooth_zv`, `smooth_mzv`,
`linear_pressure_advance`, whatever comes next) and runtime-tunable
parameters; an axis applies an ordered list of them. Each type exposes its
emission-time transform and its plan-time linear action, so the solver
constrains the **output** of the chain — what the motor actually feels — not
the nominal signal. The planner rides those limits exactly where they bind.

### 5. Kinematics are swappable modules

A kinematics module (cartesian, corexy, future delta/IDEX/…) is a
self-contained unit: its own config schema, inverse transform, forward
transform, and a linearity declaration. It sits at exactly one pipeline
stage — emission, after per-axis tracks are final. The planner, limits,
followers, and post-processors are all axis-space and blind to which module
is loaded. Motors are arbitrary named hardware objects bound to axes only
through the module (`stepper_x` on a corexy was always a lie, and the lie
has nowhere to live).

### 6. A dumb MCU and a sharp boundary

The MCU plays per-axis cubic tapes. It knows nothing about kinematics,
followers, shapers, or G-code — every track is fully written on the host.
The host↔MCU seam is `extern "C"` + `#[repr(C)]` only: C owns boot,
safety-critical paths, and shared-memory placement; Rust owns the motion
engine. The invariant is documented in
[`docs/kalico-rewrite/mcu-c-rust-boundary.md`](docs/kalico-rewrite/mcu-c-rust-boundary.md)
and designs are required to keep it true with zero edits.

### 7. Rust engine, deliberately boring seams

New code is Rust by default — one source compiled f64 on the host and f32 on
the MCU. C remains where low-level primitives must be trivially debuggable
(e.g. the MCU-side SPSC segment queue). The planner is a **pure function**:
`(geometry, constraint rows, post-processor operators) → timed trajectory` —
deterministic, unit-testable without hardware, callable as an oracle by
anything upstream.

### 8. Fail loudly

Unexpected states raise errors with clear codes instead of being padded,
clamped, or silently recovered. A segment arriving late is a bug we want to
see, not a start time to quietly shift. The same posture applies to config:
legacy fields (`max_accel` in old homes, `square_corner_velocity`,
`[input_shaper]`, `[firmware_retraction]`) are rejected at load with errors
naming the replacement — no silent migration.

### 9. Structured observability

Diagnostics flow through a structured event pipeline (`events/*.jsonl`,
queryable via VictoriaLogs) rather than printf into a flat log. The planner
will report which constraint row binds at every point — "slowed here by
`[limit extruder]` accel via the PA post-processor" — so the coupling
between one axis's config and the whole machine's speed is discoverable the
moment someone asks why.

## The pipeline

```
G-code (G5/G5.1; legacy converted by compat, rejected past this point)
  │  reduce: words → cubic Bézier segments + follower deltas (rust/gcode, rust/geometry)
  ▼
geometry: uniform cubic segments, follower ratios, virtual paths
  │
  ▼
temporal: constrained time-optimal timing (TOPP/SLP, Clarabel SOCP) —
  │       limit rows, follower rows, PA rows, shaper-window rows
  ▼
trajectory: per-axis emission chain —
  │       input track (planned curve, or odometer for followers)
  │       → post-processor chain → fit to C2 cubic pieces
  ▼
kinematics module: axis tracks → motor tracks
  │
  ▼
MCU: per-axis cubic tapes, dumb by design (C queue + Rust engine)
```

Two ledgers close the loop: the G-code's nominal counter (advanced exactly
as written, what macros and UIs see) and the physical realization (nominal
deltas through the odometer). The books are a contract; the road is physics;
nobody rewrites the books to match the road.

## Working on this fork

- Rust workspace lives in [`rust/`](rust/); run tests with
  `cargo nextest run` from `rust/` (not bare `cargo test`).
- CI gate: `./scripts/ci.sh quick` (plus `./scripts/ci.sh py` for `klippy/`
  changes).
- Design documents: [`docs/superpowers/specs/`](docs/superpowers/specs/);
  implementation plans: [`docs/superpowers/plans/`](docs/superpowers/plans/).
- Upstream Kalico documentation, features, and installation:
  [README_KALICO.md](README_KALICO.md).
