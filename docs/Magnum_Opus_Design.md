# Magnum Opus — motion pipeline design

**Branch**: `magnum-opus` (off `blend-arc`).
**Goal**: a motion pipeline that delivers genuine "fast AND smooth" —
a measurable, class-leading improvement over both mainline Kalico SCV
and the current `blend-arc` arc-blending approach.

## Context: where `blend-arc` landed, why it's not enough

The `blend-arc` branch replaced mainline's sharp-V + SCV corner handling
with circular arcs sized by `corner_deviation`. Measured on a Voron cube
+ benchy at 45k accel with MZV shaping:

| Config | Real time | Sim time |
|---|---|---|
| Kalico `main` (pure SCV) | 24m01s | 1288s post-G28 |
| `blend-arc` cd=0.14 (no vsup) | 24m35s | 1365s post-G28 |
| `blend-arc` cd=0.14 + vsup rule | predicted 22m03s | 1158s post-G28 |

Two rules on `blend-arc` decide *when* to insert an arc:

1. **Shaper-aware suppression** — skip the arc when
   `2·v·sin(φ/2)·σ_T ≤ corner_deviation`; the shaper's own smearing
   already meets the budget.
2. **Velocity-aware suppression** — skip when fork's
   (ramp_to_v_arc + arc_traversal_time) ≥ mainline's SCV-equivalent
   ramp_time at the same corner.

Together these recover the ~2-minute fork advantage the arc approach
originally promised. They also reveal the fundamental limit of
arc-based corner blending on FDM printers: arcs are slower than sharp-V
at ~90°+ corners with short neighbouring segments. There is no amount
of tuning inside the arc-based pipeline that beats this ceiling.

## The fundamental reframe

The `blend-arc` pipeline plans a *commanded* trajectory; the input
shaper then filters it into physical motion. This puts the shaper in
an adversarial role — the planner has to anticipate what the shaper
will do, but it doesn't directly control the physical path.

**Magnum-opus inverts the chain.** Plan the *physical* path we want,
then pre-distort the commanded trajectory so that after the shaper it
lands on the desired physical path. The shaper becomes a known
transfer function, not an adversary.

This enables:

- **No corners in physical space.** The path is C² (curvature-continuous)
  or higher — clothoid-like — with no step in curvature. Shaper input
  has less high-frequency content, so we can raise `max_accel` before
  ringing shows up.
- **No per-junction approximation.** Velocity profile is planned
  globally over a smooth physical path, not through a sequence of
  discrete corner decisions.
- **Shaper is an ally.** We plan *with* it, not *after* it.

## Four pillars

### 1. Feedforward inverse-shaper compensation

Given a desired physical trajectory `p_phys(t)` and a known shaper
impulse response `h(t)`, solve for the commanded trajectory
`p_cmd(t)` such that `p_cmd * h ≈ p_phys` (where `*` is convolution).

Practical approach: since standard input shapers (ZV/MZV/EI/2HEI) are
FIR filters with known impulse sequences `{(A_i, T_i)}`, the inverse
is *also* FIR — but causal inversion is unstable in general. Use the
finite-window deconvolution trick (same approach as the Cho 2018 /
Sencer-Tajima 2015-2020 literature): precompute a short forward-
looking correction kernel that pre-distorts the commanded trajectory.
Stable when the shaper zeros don't land on or near the unit circle,
which is true for typical FDM tuning.

Deliverable: `klippy/blendshaper_inverse.py` — given a shaper
impulse list and a commanded trajectory segment, return the
pre-distorted commanded trajectory. Unit tests verify
`shape(inverse_shape(p)) ≈ p` on ramps, steps, and arcs.

Risk: pre-distorted commanded trajectory may overshoot `max_velocity`
or `max_accel`. Handle by clipping and accepting slight deviation
from desired physical path in the clipped regions.

### 2. Clothoid corner primitive

Replace `blend_geometry`'s circular arc with a **symmetric clothoid
pair** (Euler spiral). Key property: curvature κ(s) is linear in arc
length along the clothoid, so at entry/exit κ = 0 matches the incoming
straight line — no curvature step. The shaper sees no high-frequency
content at the corner boundary.

Geometry:
```
κ(s) = s / α²    (0 ≤ s ≤ L_c), with peak κ = L_c / α² at midpoint
```
Parameters:
- `α` (clothoid scaling, [length]): governs the curvature profile.
- `L_c` (clothoid length, per half): total arc length is 2·L_c.
- Peak curvature `κ_max` (at the clothoid-pair joint) sets the
  velocity ceiling: `v_max = √(a_max / κ_max)`.

Design inputs: target corner deviation `d_max` and turn angle `θ`.
Solve for `(α, L_c)` such that:
- chord deviation from the ideal corner apex ≤ `d_max`,
- endpoint tangents match the surrounding straight segments,
- `κ_max` is minimized (gives highest `v_max`).

Closed-form solutions exist; see Shi 2021 and Tajima-Sencer 2016.

Deliverable: `klippy/blendmath.py::clothoid_geometry` alongside the
existing `arc_geometry`. Same return shape (entry/exit points,
polyline approximation, `v_cap`) so the rest of the pipeline is
unchanged.

### 3. Adaptive per-region acceleration

Today `a_max` is a single global knob that caps both the
centripetal force at corners and the tangential accel on straights.
These have different physical origins:

- **Corner centripetal** is limited by toolhead-and-belt stiffness
  against bending forces — a mechanical limit.
- **Straight-line tangential accel** is limited by stepper torque,
  shaper ringing, and frame racking — different failure modes, in
  practice a higher ceiling.

Split them. Add `max_corner_accel` (default = `max_accel`) and allow
`max_accel` to be higher. On clothoid corners, use `max_corner_accel`
for the centripetal cap. On straight ramps, use the full `max_accel`.

Effect: straight-line ramps into/out of corners compress, without
increasing corner centripetal stress. For the user's voron cube +
benchy at 45k corner / 70k straight this is an estimated additional
50–100 s savings on top of clothoid + feedforward.

Deliverable: thread two accel values through the planner API
(`blendplanner`, `blendmath`, `blendshaper`). No config-level flag —
a single `max_accel` default makes the feature inactive; users opt in
by setting `max_corner_accel` lower than `max_accel`.

### 4. Global velocity optimization (optional, research-scope)

Klipper's look-ahead is greedy: it picks junction velocities
left-to-right, making the best local choice with a fixed look-back
window. A truly global optimizer would pick the velocity profile that
minimizes total time subject to all kinematic constraints.

Cost: substantial (think: LP or QP solver), uncertain gain
(estimated 1–3%), maintenance burden. Defer to post-launch; the
first three pillars are the load-bearing wins.

## Architecture map

```
  gcode input
      │
      ▼
┌────────────────────────────┐
│ blendprepass               │  (unchanged: CollinearCollapser
│                            │   merges near-collinear input moves)
└────────────────────────────┘
      │
      ▼
┌────────────────────────────┐
│ blendplanner               │  (mostly unchanged: emits
│   CornerBlender            │   trunc_prev + corner-primitive +
│                            │   trunc_next sequence; corner-primitive
│                            │   is now clothoid_geometry instead of
│                            │   arc_geometry)
└────────────────────────────┘
      │
      ▼
┌────────────────────────────┐
│ blendshaper                │  (extended: compute velocity bounds
│                            │   on clothoid corners, not arcs)
└────────────────────────────┘
      │
      ▼
┌────────────────────────────┐
│ blendshaper_inverse [NEW]  │  (feedforward inverse compensation of
│                            │   the full commanded trajectory)
└────────────────────────────┘
      │
      ▼
┌────────────────────────────┐
│ klippy toolhead +          │  (unchanged: trapq + stepcompress)
│ chelper                    │
└────────────────────────────┘
```

The `blend-arc`'s two suppression rules survive verbatim but now
decide "clothoid vs sharp-V" (the logic is identical — whichever is
faster at the specific corner).

## Literature anchors

- **Cho (often miscited as Dong/Wang) et al. 2018.** *Input shaping-
  based corner rounding algorithm for machining short line segments*.
  IJAMT 97(1-4):105–116. DOI `10.1007/s00170-018-1922-0`. The
  arc-radius-to-shaper-span relation and distortion compensation
  concept come from here.
- **Sencer, Tajima 2015-2020 series** (IJMTM, Precision Engineering,
  ASME MSEC). Analytical junction velocity under shaper + contour
  tolerance. Our inverse-shaping math lines up with theirs.
- **Biagiotti, Melchiorri 2012 / 2017 / 2019** (Control Engineering
  Practice). FIR-filter-chain approach to smoothness-constrained
  trajectory generation.  The view of "input shaper = FIR" and the
  closed-form deviation formulas originate from this thread.
- **Shi et al. 2021** (RCIM), **Heisel/Shi 2020** (RCIM). Clothoid
  spline corner smoothing with closed-form parameters. Direct
  reference for pillar 2.
- **Tajima, Sencer 2016** (ASME MSEC), **2018** (Precision Engineering).
  Kinematic corner smoothing under vibration constraints. Overlaps
  pillars 1+2.

Our project memory files with pointers:
- `~/.claude/projects/-Users-daniladergachev-Developer-kalico/memory/`
  `project_arc_is_optimal.md` (SUPERSEDED — see caveat in
  `reference_klipper_sim.md` about sim-absolute-time unreliability)

## Validation plan

Each pillar independently testable.

### Per-pillar checkpoints

**Pillar 1 (inverse shaper)**:
- Unit test: `shape(inverse_shape(ramp)) − ramp` max-error within
  1 µm on a ramp from 0 to 1000 mm/s over 10 ms.
- Sim test: batch-mode klippy with pillar 1 active on voron cube;
  total time matches baseline within 1% (feedforward is lossless).
- Hardware test: single straight-line print, watch for ringing at
  increased `max_accel`. Ringing should not worsen vs baseline.

**Pillar 2 (clothoid)**:
- Unit test: `clothoid_geometry` closed-form matches numerical
  integration of κ(s) = s/α² within 0.1 µm on entry/exit positions.
- Sim test: voron cube sim with clothoid + vsup rule; total time
  better than or equal to arc + vsup baseline.
- Hardware test: voron cube print with clothoid. Expected: matches
  or beats the ~22m03s projected with `blend-arc` + vsup.

**Pillar 3 (split accel)**:
- Unit test: planner honors `max_corner_accel < max_accel` by
  capping centripetal independently of tangential.
- Sim test: voron cube with `max_accel = 70000`,
  `max_corner_accel = 45000`. Expected: additional 50–100 s
  savings on top of pillar 2.
- Hardware test: same config. Watch for ringing on straight
  ramps and on corners separately.

**Pillar 4 (global optimizer)**: deferred; revisit after first
three pillars are shipping.

### Final integration target

On voron cube + benchy, 45k corner accel, 70k straight accel:

| Config | Expected time | Source |
|---|---|---|
| Mainline SCV=45 | 24m01s | measured |
| blend-arc + vsup | 22m03s | predicted, magnum-opus scope |
| magnum opus complete | **≤19m30s** | projected |

The magnum-opus projection: ~150 s from clothoid + feedforward
(vs arc + feedforward), plus ~50–100 s from split accel, minus
maintenance noise. If real hits 19m30s it clears both mainline
and the current best fork by a wide margin.

## Sequencing and effort

Order matters — each pillar is testable standalone but their gains
compound.

1. **Pillar 1 first** (inverse-shaper feedforward). No changes to
   the corner primitive; just a new pass on the commanded trajectory.
   Testable against hardware on any gcode. ~1 week engineering + 1
   week tuning.
2. **Pillar 3 next** (split accel). Trivially small code, large
   win once pillar 1 is in place. ~2 days + 1 week tuning.
3. **Pillar 2 last** (clothoid). Biggest code change; swaps the
   `blend_geometry` core. 1-2 weeks engineering, careful migration.
   Hardware validation at each step.
4. **Pillar 4** deferred.

Total: 4-6 weeks engineering, 2-4 weeks tuning + hardware iteration.
Sequencing allows shipping each pillar independently as it lands —
the fork doesn't sit in "not-ready" state for the full duration.

## Compatibility

This design stays within the Kalico planner architecture. It replaces
geometry primitives (arc → clothoid) and adds a new post-processing
stage (inverse shaper), without changing trapq, chelper, or the MCU
firmware. Backward compatibility with `blend-arc` is preserved by
leaving `arc_geometry` in place and gating clothoid behind a config
switch during the transition — but per the fork's no-runtime-flags
policy, the final state is clothoid-only.

## Open design questions

1. **Inverse-shaper truncation**: causal FIR inversion requires a
   forward-looking window. How many milliseconds of lookahead do we
   need for stable inversion on MZV at user's typical freqs? Probably
   ~2–3 × shaper span = 15–20 ms. Does the planner's existing
   look-ahead already cover this, or do we need additional buffering?
2. **Clothoid sub-segmentation granularity**: polyline approximation
   chord error (analogous to current `max_chord_err`). Clothoid is
   smoother than an arc so we can probably loosen. Start at 0.01 mm
   to match, re-evaluate after measurement.
3. **Adaptive accel vs pillar 1 interaction**: if pillar 1 lets us
   raise straight-line accel, does pillar 3's split still matter, or
   do the two together saturate at whatever the mechanical ceiling is?
   Could affect pillar 3's priority in the sequencing.
4. **Velocity-aware suppression on clothoids**: the rule compares
   arc cost to SCV-equivalent cost. On clothoids the "cost" formula
   is different (clothoid has length + curvature profile). Derive
   the analogous comparison.
