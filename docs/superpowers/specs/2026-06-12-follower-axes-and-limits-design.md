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
  of the axes it follows (downstream of those axes' shapers): it pays out its
  commanded displacement proportionally to actual distance traveled along that
  path (the odometer rule). The extruder is the canonical instance: a follower
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

Steppers are a separate layer: hardware objects (pins, currents, microsteps)
connected to axes only through the kinematic map. Axis config carries planning
meaning; stepper config carries hardware. Limits can in the future be declared
in stepper space (§3); the linear kinematic map translates them into ordinary
constraint rows.

The kinematic map is a **swappable module** declared in config, not implied by
stepper section names. Each kinematics type (cartesian, corexy, future delta /
IDEX / rotating-table) is a self-contained unit defining four things:

1. **Its own config schema** — which axis roles it binds and which stepper
   lists it asks for. Roles bind to declared axis names explicitly
   (`axis_x: x` is a binding, not redundancy — the module assumes no letters).
2. **Inverse transform** (axes → steppers) — the emission workhorse.
3. **Forward transform** (steppers → axes) — homing and position seeding.
4. **Linearity declaration** — either a constant matrix (cartesian, corexy,
   IDEX), or nonlinear (delta, polar). Linear: stepper cubics are exact
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
a_steppers: stepper_a, stepper_a1
b_steppers: stepper_b, stepper_b1
z_steppers: stepper_z0, stepper_z1, stepper_z2
```

Stepper names are arbitrary; a stepper has no axis identity outside its
assignment (`stepper_x` on a corexy was always a lie, and the lie has nowhere
to live). Direct-drive axes declare their motors in their own section —
`[axis e]` carries `steppers: extruder_motor` (the degenerate identity
kinematics) — so kinematics modules claim only the coupled axes they exist
for, and nothing is inferred from the stepper side.

Three coverage rules close the config, each failing at load naming the gap:
every axis appears in at least one `[limit]` section (§3); every axis is
stepper-mapped exactly once (one kinematics role or its own `steppers:` key —
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
  old homes, `square_corner_velocity`, `max_z_accel`, and the rest are rejected
  at load with errors naming the field as unsupported and pointing at the
  `[limit]` syntax. No silent migration; users rewrite deliberately.

Today's behavior — global `max_accel` broadcast into per-axis boxes, so
diagonals reach √2× the configured value — dies with the legacy fields.

Reserved syntax, **not built now**: `steppers:` instead of `axes:` declares the
set in stepper space; the kinematic map translates it into rows in the same pot
(corexy `stepper_a` ⇒ `|ẍ + ÿ| ≤ cap`). Velocity-dependent caps (torque
curves) would be additional keys on the same section. Nothing in this work
depends on either.

Limits are declarative data readable by any component. The planner consuming
them is a **pure function** — `(geometry, rows, shaper operators) → timed
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

**Post-processor (PA) rows.** A post-processor is a per-axis transform applied
at emission (§5); pressure advance, `F_cmd = F + k·Ḟ`, is the first
implementation. Its plan-time consequence: PA converts path acceleration into
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
The post-processor is a trait from day one; linear PA is the first instance.

**Shaper folding.** Each axis's limits apply to that axis's *input* — for
spatial axes that means pre-shaper, by definition and by tuning convention. A
follower differs only in where its input is sampled: downstream of the followed
axes' shapers. Every shaper we have is a **linear operator** on the
discretization (a convolution: shaped velocity at `t` is a fixed weighted sum
of nominal velocities at `t − dᵢ`, weights independent of the signal — see
`rust/trajectory/src/shaper.rs`). So the follower's rows are written on the
shaped combination of plan variables: known weights, samples a few steps apart.
The solver picks all N samples at once such that the shaped-then-post-processed
follower track is in-limit. No feedback, no iteration — the shaper is not
predicted, it is written into the inequality. Rows coupling nearby samples
already exist (jerk); the shaper window widens the coupling without changing
its kind.

Accepted consequences:

- Rows couple across segment boundaries (the shaper window spans them); the
  committed tail of the previous plan enters as constants. Plumbing, not math.
- Solve cost grows with window width — host compute spent on trajectory
  tightness, per the throughput rule.
- The shaper trait's contract gains one requirement: expose your action as a
  linear operator. Every practical shaper is one. A hypothetical nonlinear
  shaper supplies its own local linearization or forces an outer loop — that
  fallback is designed by whoever builds such a shaper, not here.

Verification: unit tests covering every row family and edge case are the
backbone. `verify-logic` is dispatched only if plan-writing hits a claim we
cannot settle from first principles (e.g., prior art on windowed-constraint
SLP convergence).

## 5. Emission and bookkeeping

Every axis, at emission, runs the same per-axis chain — no axis-specific code
paths, only stages that are configured or skipped:

1. **Input track.** Spatial axis: its planned, shaped curve. Follower axis:
   the odometer — integrate the followed axes' realized speed (downstream of
   their shapers; quadrature over exact polynomial derivatives, host f64) to
   get actual distance over time, then `track = start + ratio × distance`.
   Follower-only moves: the track comes straight from the plan.
2. **Post-processor** (optional, any axis): the configured transform on the
   track. PA is the first implementation. Not follower-specific: any axis may
   declare one; the architecture does not ask whether it makes sense — that is
   the user's call. Plan-time rows fold it in, so limits hold downstream of it
   (the motor feels the post-processed demand).
3. **The axis's own shaper** (optional, any axis — a follower may have one;
   typically it won't). Limits are pre-own-shaper by the same convention as
   every axis.
4. **Fit to cubic pieces**, ship as an ordinary `PushPieces` lane.

"After shaping," wherever this document says it, means: a follower samples the
path it follows after that path's own processing; the follower's own
processing happens downstream of the sampling, like any axis.

The MCU is untouched: per-axis cubic tapes, no knowledge of followers,
post-processors, or shapers. `docs/kalico-rewrite/mcu-c-rust-boundary.md`
requires zero edits.

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
`[limit extruder]` accel via post-processor"). The coupling between one axis's
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
3. **Planner extension.** Follower rows, post-processor rows, snap support,
   shaper-operator folding with cross-segment window plumbing. The deep item;
   depends on 1–2.
4. **Per-axis emission chain.** Odometer quadrature, post-processor trait with
   linear PA first, own-shaper slot, piece fitting, two-ledger bookkeeping.
   Depends on 2; testable against 3's output offline (klipper-sim).
5. **Concept removal sweep.** Toolhead and remaining mainline-planner fossils
   out of the Rust side; klippy/bridge seam renamed; thin compat shim for the
   published `toolhead` status object, retired on its own schedule. Includes
   the declared kinematic map (§1): `[kinematics]` section with explicit
   stepper-to-role assignment, replacing role-encoding stepper section names.
6. **Observability.** Binding-constraint reporting via structured logs. Small,
   rides on 3.

Deferred, consciously — door open, nothing built: stepper-space limits
(`steppers:` key, syntax reserved), velocity-dependent caps (torque curves),
nonlinear PA (trait slot exists), exposing a follower's own shaper in config,
automated limit tuning. All additive rows, keys, or trait implementations; none
change the architecture.

Hard requirements carried through every plan: the planner stays a pure
function (the unit-test surface and the oracle API); fail loudly everywhere;
the MCU boundary doc stays true with zero edits.
