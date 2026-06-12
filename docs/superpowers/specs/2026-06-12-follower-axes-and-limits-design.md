# Follower axes and unified limits

Status: approved design, pre-implementation.
Replaces the MCU-resident e-follows-xy removed in `daf8a1aa0` (2026-05-27 MCU
simplification). The MCU contract is untouched by this design.

## 1. The model

A printer is a set of **axes**. An axis is a planner-domain config object. The
code never knows what an axis is *used for* — there is no extruder concept, no
toolhead concept, no hardcoded roles. G-code letters bind to axes at the reduce
boundary; everything past that boundary is uniform.

The system knows exactly two relations between axes, both explicit in config:

- **`follows`** — a follower axis derives its motion from the *realized* path
  of the axes it follows (downstream of those axes' post-processor chains,
  §4): it pays out its commanded displacement proportionally to actual
  distance traveled along that path (the odometer rule). The extruder is the canonical instance: a follower
  of `{x, y, z}`. This is the only directional dependency in the system.
- **`[limit]` membership** — every limit names a set of coordinates and caps
  their combined motion. All limits jointly constrain the one shared clock per
  move. Timing was never per-axis: every axis's limits already pull on the
  common speed (a slow Z cap slows a sloped XY+Z move today). A follower's
  limits join the same pot through the same mechanism — there is no reverse
  dependency, just one rulebook and one solve.

Deleted by this model, each consciously: the toolhead concept, the E modes
(`CoupledToXy` / `Travel` / `Independent`), `HelicalExtrusionUnsupported`, the
centripetal cap, square-corner-velocity, and the MCU's knowledge that any axis
is special. The MCU plays per-axis cubic tapes; every track, including a
follower's, is fully written on the host.

Motors are a separate layer: hardware objects (pins, currents, drive
technology — stepper, servo, whatever comes next) connected to axes only
through the kinematic map. Axis config carries planning
meaning; motor config carries hardware. Limits can in the future be declared
in motor space (§3); the linear kinematic map translates them into ordinary
constraint rows.

The kinematic map is a **swappable module** declared in config, not implied by
motor section names. Each kinematics type (cartesian, corexy, future delta /
IDEX / rotating-table) is a self-contained unit defining four things:

1. **Its own config schema** — which axis roles it binds and which motor
   lists it asks for. Roles bind to declared axis names explicitly
   (`axis_x: x` is a binding, not redundancy — the module assumes no letters).
2. **Inverse transform** (axes → motors) — the emission workhorse.
3. **Forward transform** (motors → axes) — homing and position seeding.
4. **Linearity declaration** — either a constant matrix (cartesian, corexy,
   IDEX), or nonlinear (delta, polar). Linear: motor cubics are exact
   coefficient combinations of axis cubics. Nonlinear: the host samples the
   inverse transform and refits cubic pieces within a declared tolerance.
   Either way the MCU stays dumb — it never learns kinematics exist.

The module sits at exactly one pipeline stage: emission, after the per-axis
chain (§5) produces final axis tracks, before piece fitting. The planner,
limits, follower, and shapers are all axis-space and blind to which module is
loaded. Adding a printer geometry means writing one module, nothing else.

```
[kinematics]
type: corexy
axis_x: x
axis_y: y
axis_z: z
a_motors: motor_a, motor_a1
b_motors: motor_b, motor_b1
z_motors: motor_z0, motor_z1, motor_z2
```

Motor names are arbitrary; a motor has no axis identity outside its
assignment (`stepper_x` on a corexy was always a lie, and the lie has nowhere
to live). A motor is a stepper or a servo — or whatever drive technology comes
next; nothing axis- or planner-side ever cares which. Direct-drive axes
declare their motors in their own section — `[axis e]` carries
`motors: extruder_motor` (the degenerate identity kinematics) — so kinematics
modules claim only the coupled axes they exist for, and nothing is inferred
from the motor side.

Three coverage rules close the config, each failing at load naming the gap:
every axis appears in at least one `[limit]` section (§3); every axis is
motor-mapped exactly once (one kinematics role or its own `motors:` key —
never zero, never twice); every `follows` entry references a declared axis.

Axis names double as G-code word letters (`[axis e]` ↔ word `E`): single
letters, collisions with structural G5 words (I/J/P/Q/F) rejected at load.
Commands that presuppose an axis *purpose* the system does not have (G10/G11
firmware retraction presupposing "the extruder") are unsupported: the
`[firmware_retraction]` section is rejected at config load. If such features
return, they must be expressed in axis terms — separate design.

## 2. G-code input and reduce

Input remains G5/G5.1 only at the reduce boundary. An extruding move is a cubic
Bézier plus one scalar word: the follower axis's displacement for the whole
curve (`E0.05`). Absolute words are normalized to deltas at the parse boundary
(`delta = word − nominal ledger`); the planner's world is deltas only. Absolute
and relative input both stay supported — the difference dies at the boundary.

`classify_e_mode` and the three-way mode split are replaced by one rule:

- **Path length is 3D arc length** of the spatial curve (not XY). Vase mode,
  retract-with-hop, and spiral lift become ordinary moves.
- The follower ratio is `delta / nominal path length`. Travel moves are simply
  `delta = 0`, ratio 0.
- **Follower-only moves** (no spatial displacement): the move's path length is
  the follower's own displacement; the move is planned like any other —
  G-code feedrate applies, the follower's own limit rows cap it. This is the
  one degenerate-case rule in the system: a fallback line, not a mode.

The segment struct loses `e_mode` and `e_independent`; it carries the ratio
(equivalently the delta) and nothing else follower-related.

Within a segment, follower motion is uniform per mm of path — that is what the
slicer's number means and the only thing G5 syntax can express. Nonuniform
within-move extrusion would be future syntax, not future architecture.

The reduce boundary stays the rejection boundary: G0/G1/G2/G3 never reach it;
`compat` converts upstream. Where the geometry comes from and what transforms
it saw before arriving are not this design's concern.

## 3. Limits

One concept: a **`[limit]` section names a set of coordinates and caps the
magnitude of the motion vector restricted to them** — velocity, acceleration,
jerk, and higher derivatives where declared. All sections contribute rows to
one pot; the move runs as fast as every row allows, pointwise along the path.
Rows never conflict, they only intersect; there is no precedence.

```
[limit gantry]
axes: x, y
max_velocity: 500
max_accel: 30000

[limit z]
axes: z
max_accel: 500

[limit extruder]
axes: e
max_velocity: 75
max_accel: 1500
```

- A singleton set is a per-axis cap (box row). A multi-axis set is a norm cap
  (circle row). Same concept at different set sizes, not two features.
- Overlapping sets are legal and meaningful: `{x,y} ≤ 60k` plus `{y} ≤ 40k`
  gives X moves 60k, Y moves 40k, and intermediate directions the intersection
  shape.
- **Coverage is mandatory: every axis must appear in at least one limit
  section, or config fails to load.** No silent unlimited axes, no silent
  global fallback. This also guarantees follower-only moves always have real
  numbers to plan against.
- The norm caps the **total** vector. Along-path and turning components are one
  acceleration; curvature is handled by the same rows that handle straights
  (the `x''(s)·ṡ²` term in the constraint evaluation). No cornering knobs.
- Jerk caps are declared like any other derivative cap, with sensible defaults
  when unset. The `j_max = 2 × a_max` broadcast dies.
- **Legacy config fields fail loudly.** `max_accel` / `max_velocity` in their
  old homes, `square_corner_velocity`, `max_z_accel`, `[input_shaper]`, and
  the rest are rejected at load with errors naming the field as unsupported
  and pointing at the replacement (`[limit]`, `[post_processor]`) syntax. No silent
  migration; users rewrite deliberately.

Today's behavior — global `max_accel` broadcast into per-axis boxes, so
diagonals reach √2× the configured value — dies with the legacy fields.

Reserved syntax, **not built now**: `motors:` instead of `axes:` declares the
set in motor space; the kinematic map translates it into rows in the same pot
(corexy belt motor ⇒ `|ẍ + ÿ| ≤ cap`). Velocity-dependent caps (torque
curves) would be additional keys on the same section. Nothing in this work
depends on either.

Limits are declarative data readable by any component. The planner consuming
them is a **pure function** — `(geometry, rows, post-processor operators) → timed
trajectory` — deterministic, unit-testable without hardware, and callable as an
oracle by any future upstream component. That API shape is a hard requirement
of this work.

## 4. Planning math

The planner discretizes each move's timing into N samples of path progress and
its derivatives (`ṡ, s̈, s⃛`, extended to snap where needed) and solves for the
fastest profile satisfying every constraint row — the existing TOPP/SLP
machinery with new row families. Three additions, all rows, no new solver:

**Follower rows.** A follower's motion relates to path progress through a
constant: `dF/ds = ratio`. It enters the constraint set as an axis whose
direction along the move never changes:
`|ratio|·ṡ ≤ v_max`, `|ratio|·s̈ ≤ a_max` — the same row shape as everything
else.

**Pressure-advance (PA) rows.** PA, `F_cmd = F + k·Ḟ`, is a per-axis post-processor
applied at emission (§5; the unified post-processor abstraction is defined below).
Its plan-time consequence: PA converts path acceleration into
follower velocity demand and path jerk into follower acceleration demand:

```
|ratio·(ṡ + k·s̈)| ≤ v_max        |ratio·(s̈ + k·s⃛)| ≤ a_max
```

Mixed-derivative rows, linear in the solver's existing variables. The planner
slows exactly where these bind (corners), riding the limit pointwise — that is
what constrained time-optimal means; nothing is slowed globally. Jerk caps
under PA require path snap; the planner's derivative order is extended
accordingly (separately testable). Nonlinear PA makes these rows nonlinear in
`ṡ`; the SLP linearization already used for the jerk relaxation handles them.

**Post-processors — one abstraction for shapers and PA.** An input
shaper and pressure advance are the same mathematical object: a linear
time-invariant operator on a per-axis track (the shaper convolves with a
unity-integral kernel; linear PA convolves with `δ + k·δ′`). One attenuates
high frequencies, the other amplifies them; structurally they are identical.
So there is one config object: `[post_processor <name>]` declares a named instance
with a `type:` (`smooth_zv`, `smooth_mzv`, `linear_pressure_advance`, future
models — other PA formulations, third-party transforms — plug in the same
way) and that type's parameters. An axis applies an ordered list
(`post_processors:` on its `[axis]` section). The planner never knows what a post-processor
does or which axes exist — each type exposes exactly two things: its
emission-time transform and its plan-time linear action on the
discretization (the window operator `W` for kernels, the mixed-derivative
shift for PA). Linear post-processors commute, so list order only acquires meaning
when a nonlinear type exists.

**Limits constrain the chain output — what the motor feels.** The old
pre-shaper convention was tradition plus an accident of safety: a smoothing
kernel is contractive (`‖K∗a‖∞ ≤ ‖a‖∞`), so rows on the nominal signal never
under-constrain the motor — they merely over-constrain it. PA amplifies, so
nominal rows there would lie to the motor. One rule covers both: every limit
row is written on the axis's post-chain signal. For shaped axes this is
strictly less conservative than the pre-shaper convention — the planner may
command a sharper nominal corner whose smoothed, realized demand rides the
cap exactly. Staging: rows on nominal spatial signals ship first (the
conservative direction — valid, never motor-unsafe); switching spatial rows
to their windowed post-chain form is a separately planned tightening that
only speeds prints (§6).

Post-processor parameters are **runtime-tunable**: a parameter change applies to
everything planned after it and never rewrites already-planned trajectory —
that is what makes live tuning possible without replanning. Compatibility
shims for mainline tuning commands (`SET_PRESSURE_ADVANCE`, the legacy
`[input_shaper]` section) are deferred, built later on top of the generic
sections and parameter-update path.

**Shaper folding.** The follower's rows are the first instance of the
post-chain rule above: its input is sampled downstream of the followed axes'
post-processor chains, so its rows are written on the shaped signal. Every kernel we
have is a **linear operator** on the discretization (a convolution: shaped
velocity at `t` is a fixed weighted sum of nominal velocities at `t − dᵢ`,
weights independent of the signal — see `rust/trajectory/src/shaper.rs`). So
the follower's rows are written on the shaped combination of plan variables:
known weights, samples a few steps apart. The solver picks all N samples at
once such that the post-chain follower track is in-limit. No feedback, no
iteration — the kernel is not predicted, it is written into the inequality.
Rows coupling nearby samples already exist (jerk); the kernel window widens
the coupling without changing its kind.

Accepted consequences:

- Rows couple across segment boundaries (the shaper window spans them); the
  committed tail of the previous plan enters as constants. Plumbing, not math.
- Solve cost grows with window width — host compute spent on trajectory
  tightness, per the throughput rule.
- The post-processor trait's contract requires exposing your action as a linear
  operator. Every practical shaper and linear PA is one. A hypothetical
  nonlinear post-processor supplies its own local linearization or forces an outer
  loop — that fallback is designed by whoever builds such a post-processor, not here.

Verification: unit tests covering every row family and edge case are the
backbone. `verify-logic` is dispatched only if plan-writing hits a claim we
cannot settle from first principles (e.g., prior art on windowed-constraint
SLP convergence).

## 5. Emission and bookkeeping

Every axis, at emission, runs the same per-axis chain — no axis-specific code
paths, only stages that are configured or skipped:

1. **Input track.** Spatial axis: its planned curve. Follower axis: the
   odometer — integrate the followed axes' realized speed (downstream of
   their post-processor chains; quadrature over exact polynomial derivatives, host
   f64) to get actual distance over time, then
   `track = start + ratio × distance`. Follower-only moves: the track comes
   straight from the plan.
2. **Post-processor chain** (optional, any axis): the axis's configured `post_processors`
   list applied to the track in order — shaper kernels and PA are the same
   stage (§4). Not follower-specific: any axis may declare any chain; the
   architecture does not ask whether it makes sense — that is the user's
   call. Plan-time rows constrain the chain's output, so limits hold where
   the motor feels them.
3. **Fit to cubic pieces**, ship as an ordinary `PushPieces` lane.

"After shaping," wherever this document says it, means: a follower samples
the path it follows downstream of that path's own post-processor chain; the
follower's own chain runs downstream of the sampling, like any axis.

The MCU is untouched: per-axis cubic tapes, no knowledge of followers or
post-processors. `docs/kalico-rewrite/mcu-c-rust-boundary.md` requires zero
edits.

**Two ledgers, by intention** (any follower axis; the extruder is the
canonical instance):

- **Nominal ledger** — the G-code's counter for the axis, advanced exactly as
  written. Absolute words diff against it; macros/UIs see it. The books are a
  contract executed literally, including the author's mistakes: after
  "extrude to 10", firmware-retract to 8, "extrude to 20" means +10 applied
  from the physical 8 → 18.
- **Physical realization** — nominal deltas through the odometer. Where the
  followed path's shaping shortens the real road (smoothed corners),
  proportionally less follower motion happens. That is correct output, not
  error: the slicer computed filament for road that, post-shaping, does not
  all exist; paying out the full delta would overextrude the road that does.
  The shortfall versus the books is accepted, never corrected. Firmware-
  retract offsets physical state only, never the books.

This mirrors the spatial axes exactly: commanded coordinates are a ledger, the
physical path deviates where physics says so, and nobody rewrites the books to
match the road.

**Observability.** The planner knows which constraint row binds at every point
and reports it through the structured log pipeline ("slowed here by
`[limit extruder]` accel via the PA post-processor"). The coupling between one axis's
config and the whole machine's speed becomes discoverable at the moment
someone asks why.

## 6. Work decomposition

Separately plannable, separately testable, in dependency order:

1. **Limits rework.** `[limit]` sections, mandatory coverage check, norm rows
   in the solver, legacy deletion (centripetal cap, SCV, jerk broadcast, old
   fields rejected at load with pointers to the replacement). Independent of
   everything follower-related.
2. **Axis objects & reduce simplification.** Axes as config objects with
   `follows`; 3D arc length; delete `e_mode` / `Independent` / `Travel` /
   `HelicalExtrusionUnsupported`; absolute→delta normalization; follower-only
   moves as regular moves. Mostly deletion.
3. **Planner extension.** Follower rows, PA rows, snap support,
   shaper-operator folding with cross-segment window plumbing. The deep item;
   depends on 1–2.
4. **Per-axis emission chain.** Odometer quadrature; post-processor registry
   (`[post_processor]` sections, ordered per-axis chains, runtime-tunable
   parameters) unifying shaper kernels and linear PA as the first two types;
   piece fitting; two-ledger bookkeeping. Depends on 2; testable against 3's
   output offline (klipper-sim).
5. **Concept removal sweep.** Toolhead and remaining mainline-planner fossils
   out of the Rust side; klippy/bridge seam renamed; thin compat shim for the
   published `toolhead` status object, retired on its own schedule. Includes
   the declared kinematic map (§1): `[kinematics]` section with explicit
   motor-to-role assignment, replacing role-encoding motor section names.
6. **Observability.** Binding-constraint reporting via structured logs. Small,
   rides on 3.

Deferred, consciously — door open, nothing built: **windowed post-chain rows
for spatial axes** (the §4 tightening — spatial rows ship in nominal,
conservative form first; the upgrade reuses plan 3's cut machinery and only
speeds prints), motor-space limits (`motors:` key, syntax reserved),
velocity-dependent caps (torque curves), nonlinear PA (a new post-processor type),
mainline-compatible migration shims (`SET_PRESSURE_ADVANCE`, an
`[input_shaper]` compat flag), automated limit tuning. All additive rows,
keys, sections, or type implementations; none change the architecture.

Hard requirements carried through every plan: the planner stays a pure
function (the unit-test surface and the oracle API); fail loudly everywhere;
the MCU boundary doc stays true with zero edits.
