# Magnum Opus — motion pipeline design

**Branch**: `magnum-opus` (off `blend-arc`).
**Goal**: a motion pipeline that delivers genuine "fast AND smooth" —
a measurable, class-leading improvement over both mainline Kalico SCV
and the current `blend-arc` arc-blending approach.

## Context: where `blend-arc` landed, why it's not enough

The `blend-arc` branch replaced mainline's sharp-V + SCV corner handling
with circular arcs sized by `corner_deviation`. Measured on a Voron cube
+ benchy at 45k accel with MZV shaping:

| Config | Real time | Sim time (post-G28) |
|---|---|---|
| Kalico `main` (pure SCV) | 24m01s | 1288 s |
| `blend-arc` cd=0.14, no vsup | 24m35s | 1365 s |
| `blend-arc` cd=0.14 + vsup rule | (HW TBD) | 1158 s |
| `blend-arc` cd=0.14, higher straight accel | < 24m01s (HW-confirmed, beats mainline) | — |

Two rules in `blend-arc` decide *when* to insert an arc:

1. **Shaper-aware suppression** — skip the arc when
   `2·v·sin(φ/2)·σ_T ≤ corner_deviation`; the shaper's own smearing
   already meets the budget. σ_T is derived from the IS impulse
   pattern (commit `310a3ee9`).
2. **Velocity-aware suppression** — skip when fork's
   (ramp_to_v_arc + arc_traversal_time) ≥ mainline's SCV-equivalent
   ramp_time at the same corner.

Together these recover the ~2-minute fork advantage on the sim, but
**at default max_accel the arc-blended pipeline is Pareto-dominated
by mainline SCV on hardware** (+34 s slower and quality regression
on shallow turns; see `project_hardware_validation.md`). The win
only materialises when straight-segment accel is raised above corner
accel — which mainline cannot do cleanly because its single `max_accel`
knob must serve both. That observation is the empirical seed for the
whole magnum-opus plan: the planner should respect **different
acceleration budgets in different regions of the path.**

There is no amount of tuning inside the arc-based pipeline that beats
this ceiling — arcs have a constant curvature, so their peak-curvature
budget is set by the tightest corner and can't adapt.

## The fundamental reframe

The `blend-arc` pipeline plans a *commanded* trajectory; the input
shaper (IS) then filters it into physical motion. This puts the shaper
in an adversarial role — the planner has to anticipate what the shaper
will do, but it doesn't directly control the physical path.

**Magnum-opus inverts the chain.** Plan the *physical* path we want,
then pre-distort the commanded trajectory so that after the shaper it
lands on the desired physical path. The shaper becomes a known
transfer function, not an adversary.

This enables:

- **No corners in physical space.** The path is C² (curvature-continuous)
  or higher, with no step in curvature. The toolhead physically
  accelerates along a smooth curve — no shock at corner entry/exit —
  and we can run closer to the mechanical ringing ceiling before
  vibration shows up.
- **No per-junction approximation.** Velocity profile is planned
  globally over a smooth physical path, derived continuously from
  curvature, not through a sequence of discrete corner decisions.
- **Shaper is an ally.** We plan *with* it, not *after* it. The
  commanded trajectory is whatever pre-distorted thing the inverse
  shaper needs to produce — its smoothness doesn't matter; only the
  physical path's does.
- **Extruder tracks the planned (= physical) path directly.** No
  "sync extruder to IS" machinery — the planned path *is* the physical
  motion. One less thing to thread.

## Four pillars

### 1. Feedforward inverse-shaper compensation

Given a desired physical trajectory `p_phys(t)` and a known shaper
transfer function `h(t)`, solve for the commanded trajectory
`p_cmd(t)` such that `p_cmd * h ≈ p_phys` (where `*` is convolution).

**Shaper-agnostic interface.** Pillar 1 must handle both the classic
discrete-impulse FIR shapers (ZV / MZV / EI / 2HEI — current
mainline) and the polynomial **Smooth Input Shapers** from Kalico
bleeding-edge (parallel porting work in flight). Same interface,
different inverse implementations:

- **Discrete FIR** — inverse is also FIR but causal inversion is
  unstable in general. Use the finite-window deconvolution trick
  (Cho 2018 / Sencer-Tajima 2015-2020): precompute a short
  forward-looking correction kernel. Stable when the shaper zeros
  don't land on or near the unit circle — true for typical FDM
  tuning but worth runtime verification.
- **Smooth (polynomial) IS** — polynomial inverses are well-conditioned
  and local. Pillar 1 is substantially cleaner once smooth-IS lands.

**Deliverable**: `klippy/blendshaper_inverse.py` — given a shaper
description (impulse list *or* polynomial form) and a commanded
trajectory segment, return the pre-distorted commanded trajectory.
Unit tests verify `shape(inverse_shape(p)) ≈ p` on ramps, steps, and
smooth curves.

**Risk**: pre-distorted commanded trajectory may overshoot
`max_velocity` or `max_accel`. Handle by clipping and accepting
slight deviation from the desired physical path in the clipped
regions.

### 2. Smooth-accel corner primitive (shape-pluggable)

Replace `blend_geometry`'s circular arc with a **curvature-continuous
smooth curve**. Three candidate shapes — architecture chosen so they
are hot-swappable:

- **Quintic Hermite Bezier** — already implemented on
  `blend-arc-quintic-archive` (628 LOC `blendquintic.py` + 775 LOC
  tests). Lowest implementation cost for MO start.
- **Clothoid (Euler spiral)** — κ linear in arc length, Tajima-Sencer
  lineage, requires Fresnel integrals.
- **Pythagorean-Hodograph (PH) spline** — polynomial with closed-form
  arc length and curvature (Farouki 2008, Manni-Sestini corner-blending
  papers). Likely dominates both alternatives in principle — no
  Fresnel, no Gauss-Legendre quadrature — but unverified for MO
  context.

**Architecture:** the corner primitive is a strategy — one function
signature, multiple implementations:

```python
shape(prev_move, next_move, chord_tolerance) -> (polyline, v_cap_fn)
```

`v_cap_fn(s)` returns the maximum allowed velocity at arc-length `s`
along the blend. That lets the planner build the velocity profile
**continuously along the curve** rather than treating the corner as a
discrete "slow here" region.

**Unified velocity profile (merges former pillars 2 and 3).** On a
curvature-continuous curve, the acceleration budget splits naturally
at each point:

```
a_centripetal(s) = v(s)² · κ(s)
a_tangential(s)  = v(s) · dv/ds
a_total(s)² ≤ a_max_mech²       (vector magnitude)
```

At the peak-curvature midpoint: all budget is centripetal, velocity
is at its local minimum `v_peak = √(a_max / κ_peak)`. At κ = 0 endpoints:
all budget can be tangential (full straight-line accel). In between:
continuous blend. The previous design's split `max_corner_accel` vs
`max_accel` becomes **automatic** — the curve's curvature profile
dictates the allocation, no separate knob needed.

**Sub-segmentation granularity.** trapq is linear-by-design, so the
smooth curve must be decomposed into a polyline before it hits
`trapq`. This is where archived-quintic likely fell short: its
default `max_chord_err=0.01 mm` yielded 4–8 polyline segments per
blend — few enough for the shaper to see discrete κ-steps instead
of a ramp. MO sub-segmentation must be **tight enough that the
residual κ-step at each segment boundary is below the shaper's
rejection bandwidth.** Derived automatically from shaper parameters,
not a user knob. Practical floor: ~20 µm per segment (trapq minimum
move time ≈ 250 µs).

**Deliverable**: `klippy/blendmath.py::smooth_geometry` strategy
interface + `blendquintic` as initial implementation (ported from
archive). Same polyline API as the current `arc_geometry` so the
rest of the pipeline (prepass, planner, shaper, emit) is unchanged.

**Research subagent (to dispatch):** evaluate quintic / clothoid /
PH spline for the MO context — post-inverse-shaper, continuous
smooth-accel profile, ringing-bound operating point. If a clear
winner emerges, swap in before first HW test. Quintic from archive
serves as the working baseline during research.

### 3. Extruder as first-class constraint

Mainline Klipper treats extruder velocity/accel mostly as a
consequence of the XY trajectory — `max_extrude_only_*` limits apply
only to extrude-only moves, never during XY+E motion. On
acceleration-limited extruders (direct-drive, some high-flow setups)
this leaves throughput on the table: the planner picks an XY accel
the extruder physically can't follow, PA blows up, quality degrades.

MO promotes extruder limits to **hard kinematic constraints
threaded through every pillar**:

- **`max_extruder_accel`** (mm/s²) — the filament-path acceleration
  ceiling. Planner picks tangential accel such that extruder accel
  (including PA contribution) stays below this cap at every point
  on the path.
- **`max_extruder_rpm`** (revolutions per minute on the drive pulley)
  — angular-velocity ceiling. Translates to a max filament feed rate
  via drive-gear ratio.
- **Non-linear Pressure Advance** (port from
  `upstream/bleeding-edge-v2`) — required for the constraint check.
  PA's contribution to extruder acceleration is non-trivial and
  velocity-dependent; linear PA mis-estimates it at high flow.

**Real-time tuning.** `SET_EXTRUDER_LIMITS ACCEL=… RPM=…` gcode
command mutates live state during prints — same pattern as
`SET_INPUT_SHAPER`. Lets the user find the right cap empirically
without reslicing.

**Integration with other pillars:**

- Pillar 1's pre-distortion produces a *commanded* trajectory whose
  XY accel may briefly exceed the planned a_max (inversion can
  amplify). Extruder sync follows the *planned* path (= physical
  motion), so the extruder cap is applied to physical accel, not
  commanded — unaffected by pillar 1.
- Pillar 2's `v_cap_fn` along the smooth curve is the tighter of
  `√(a_max_mech / κ(s))` and `v_extruder_max` (from the extruder
  caps, given local flow). Single `v(s)` integrates both constraints.
- Pillar 4 (global optimizer) would treat extruder caps as just more
  constraints in the feasible set.

**Deliverable**: new `klippy/extras/extruder_limits.py` (or extend
`kinematics/extruder.py`) with the cap-enforcement logic + the
`SET_EXTRUDER_LIMITS` command. Non-linear PA ported from
bleeding-edge-v2 as a prerequisite.

### 4. Global velocity optimization (deferred, research-scope)

Klipper's look-ahead is greedy: it picks junction velocities
left-to-right, making the best local choice with a fixed look-back
window. A truly global optimizer would pick the velocity profile that
minimises total time subject to all kinematic constraints (toolhead
accel, extruder accel, shaper ringing margin, jerk, path tolerance).

If shipped, it retires `minimum_cruise_ratio` entirely — the optimal
cruise fraction falls out of the cost function, no heuristic needed.

Cost: substantial (LP / QP solver; real-time performance uncertain).
Estimated gain: 1–3% on top of the other three pillars. Defer until
the first three pillars are shipping; revisit when we have hardware
numbers.

## Architecture map

```
  gcode input
      │
      ▼
┌──────────────────────────────┐
│ blendprepass                 │  (unchanged:
│   CollinearCollapser         │   merges near-collinear
│                              │   input moves)
└──────────────────────────────┘
      │
      ▼
┌──────────────────────────────┐
│ blendplanner                 │  (updated:
│   CornerBlender              │   emits trunc_prev +
│                              │   smooth_primitive +
│                              │   trunc_next; primitive
│                              │   is shape-pluggable
│                              │   (quintic initially))
└──────────────────────────────┘
      │
      ▼
┌──────────────────────────────┐
│ blendshaper                  │  (extended:
│                              │   v_cap_fn along smooth curve;
│                              │   shape-agnostic)
└──────────────────────────────┘
      │
      ▼
┌──────────────────────────────┐
│ extruder_limits [NEW]        │  (pillar 3:
│                              │   caps v(s) by extruder accel/rpm
│                              │   using non-linear PA profile)
└──────────────────────────────┘
      │
      ▼
┌──────────────────────────────┐
│ blendshaper_inverse [NEW]    │  (pillar 1:
│                              │   feedforward inverse compensation
│                              │   of the commanded trajectory;
│                              │   handles both FIR and smooth IS)
└──────────────────────────────┘
      │
      ▼
┌──────────────────────────────┐
│ klippy toolhead + chelper    │  (extended with
│   trapq + HP-stepcompress    │   HP-stepcompress from
│                              │   bleeding-edge-v2)
└──────────────────────────────┘
```

The `blend-arc` two suppression rules survive verbatim but now decide
**"smooth-curve vs sharp-V"** — whichever is faster at the specific
corner. Their math stays the same; only the "smooth-curve cost
formula" changes (curvature-dependent, not a single v_arc number).

## Integration items (not pillars, but required)

These are complementary ports / abstractions that compound the
pillars' wins.

### HP-stepcompress port

From `upstream/bleeding-edge-v2` (commit `a325350d` line of work).
Second-order Taylor term + fixed-point arithmetic in step-time
compression. Reduces per-step error from larger margins to ±1.5%.
Crucial for MO because the current stepcompress algorithm introduces
*systematic artifacts in acceleration profiles at velocity junctions*
— exactly the discrete accel events that excite ringing. A smooth
planner followed by a lossy stepcompress partially undoes the
smoothness we worked for.

Cost: 20–40% max step-rate reduction on low-end MCUs. Negligible on
STM32F4+. User's Trident has ample headroom (buffer_time > 1.9 s
per 2026-04-20 hw validation).

### Smooth Input Shapers port (parallel agent)

Also from bleeding-edge-v2. Polynomial shapers instead of discrete
FIR impulses. Benefits:

- Substantially simpler and more stable inverse (pillar 1).
- Lower HF content at same frequency cancellation (pillar 2's
  segment-boundary κ-step is easier to swallow).
- Better PA sync in existing arch — benefits blend-arc independently.

Porting work is in flight on a separate branch (`smooth-is-port`
off `blend-arc`). Once stable, merges into both `blend-arc` and
`magnum-opus`. Pillar 1's interface stays shaper-agnostic so MO
development is not blocked on this.

Impact: user must re-calibrate shapers after merge (different
frequencies / types may apply); σ_T derivation in `blendmath.py`
needs a polynomial code path.

### Sub-segmentation automatic from shaper bandwidth

(Discussed in pillar 2.) No user knob — derived from shaper
parameters at configure time, subject to the trapq ~20 µm floor.

## Literature anchors

- **Cho et al. 2018.** *Input shaping-based corner rounding algorithm
  for machining short line segments*. IJAMT 97(1-4):105–116. DOI
  `10.1007/s00170-018-1922-0`. Arc-radius-to-shaper-span relation and
  distortion compensation concept.
- **Sencer, Tajima 2015–2020 series** (IJMTM, Precision Engineering,
  ASME MSEC). Analytical junction velocity under shaper + contour
  tolerance. Aligns with pillar 1.
- **Biagiotti, Melchiorri 2012 / 2017 / 2019** (Control Engineering
  Practice). FIR-filter-chain approach to smoothness-constrained
  trajectory generation. "Input shaper = FIR" view and closed-form
  deviation formulas.
- **Shi et al. 2021** (RCIM), **Heisel / Shi 2020** (RCIM). Clothoid
  spline corner smoothing with closed-form parameters. Reference for
  the clothoid candidate in pillar 2.
- **Tajima, Sencer 2016** (ASME MSEC), **2018** (Precision Engineering).
  Kinematic corner smoothing under vibration constraints. Overlaps
  pillars 1 + 2.
- **Farouki 2008.** *Pythagorean-Hodograph Curves: Algebra and
  Geometry Inseparable*. Springer. Bible for PH splines.
- **Manni, Sestini** — series of papers on PH-quintic corner blending.
  Direct reference for the PH candidate in pillar 2.

Project memory with superseded / pointer material:
- `~/.claude/projects/-Users-daniladergachev-Developer-kalico/memory/`
  - `project_arc_is_optimal.md` — SUPERSEDED (paired-ε sim at fixed
    a_max=45k; doesn't measure the ringing-ceiling question MO bets on).
  - `project_ringing_bound_operating_point.md` — notes the untested
    claim MO rests on (curve smoothness → higher usable a_max).
  - `project_hardware_validation.md` — blend-arc cd=0.14 is
    +34 s / Pareto-dominated on real HW at fixed accel; adding
    straight-only accel headroom flips this, motivating MO pillar 3.

## Validation plan

**Integrated-only testing.** Per user decision (2026-04-20), all
pillars land together before any HW test. No per-pillar HW
checkpoints. Rationale: pillars are coupled — pillar 1 unlocks
pillar 2's a_max headroom; pillar 3's extruder constraints shape
pillar 2's v_cap; you can't observe any individual pillar's real
contribution in isolation on a physical print. Sim-level unit tests
per pillar still apply.

### Unit / sim tests (per-pillar, automated)

**Pillar 1 (inverse shaper):**
- `shape(inverse_shape(ramp)) − ramp` within 1 µm on a ramp 0–1000
  mm/s over 10 ms (both FIR and smooth-IS variants).
- Batch-sim: voron cube with pillar 1 active; total time matches
  baseline within 1% (feedforward is lossless in theory).

**Pillar 2 (smooth-accel primitive):**
- Curvature continuity: `κ(s)` function has no step at blend
  boundaries (for each candidate shape).
- `v_cap_fn(s)` integrates tangential + centripetal correctly:
  `v(s)² · κ(s)² + (v · dv/ds)² ≤ a_max²` at every sampled point.
- Adaptive sub-segmentation produces polyline whose max chord error
  and max κ-step-at-boundary both satisfy the auto-derived thresholds.
- Batch-sim: voron cube with quintic + unified v(s); total time
  better than arc + vsup baseline.

**Pillar 3 (extruder constraint):**
- Synthetic extruder cap: verify planner slows XY to respect extruder
  accel cap on a ramp (unit test).
- `SET_EXTRUDER_LIMITS` mutates live state without restart.
- Non-linear PA port passes existing PA regression tests.

**Pillar 4:** deferred.

### Integrated HW test

Single reference print: `Voron_Design_Cube_v7_ABS_22m13s.gcode` on
user's V0 / Trident, exact config match for `max_velocity`,
`max_accel`, `minimum_cruise_ratio=0.1`, shaper freqs, PA, extruder
limits.

**Pass criteria:**
- Time: ≤ 19m30s (vs mainline's 24m01s real). Stretch: ≤ 18m30s.
- Quality: no visible ringing at corners on macro-photography;
  shallow turns stay crisp (not over-rounded like current blend-arc
  cd=0.14).
- Step queue health: `buffer_time` min > 1 s throughout, no
  `Timer too close` / `send_too_old` / `stepcompress` errors,
  `sysload` peak < 2.0.

**Baseline offsets (2026-04-20 calibration)** — sim vs real:

| Config | Sim post-G28 | Real | Offset |
|---|---|---|---|
| Mainline SCV=45 | 1288.5 s | 1441 s | +152.5 s |
| blend-arc cd=0.14, ts=0 | 1365.7 s | 1472 s | +106.3 s |
| blend-arc cd=0.14 + vsup | 1158.2 s | TBD | ~+12 s fork-specific |

Expect the magnum-opus offset to land somewhere between the two
(fork-specific calibration depends on shape/primitive used). Re-run
calibration after first integrated HW test.

## Sequencing and effort

**Build order** (code can land in parallel up to the integrated test):

1. **Integration prerequisites** (parallel agents / sessions):
   - Smooth-IS port → `smooth-is-port` → merge into blend-arc and
     magnum-opus.
   - HP-stepcompress port → merge into magnum-opus.
   - Non-linear PA port → merge into magnum-opus (pillar 3 dep).

2. **Revive quintic on magnum-opus.** Cherry-pick/port from
   `blend-arc-quintic-archive`; adapt to MO's shape-pluggable
   interface; tighten sub-segmentation.

3. **Pillar 3 (extruder constraint).** Thread `max_extruder_accel`
   / `max_extruder_rpm` through the planner; consume non-linear PA;
   add `SET_EXTRUDER_LIMITS`. Unit tests.

4. **Pillar 1 (inverse shaper).** Shaper-agnostic interface; FIR
   deconvolution kernel; smooth-IS polynomial inverse; overshoot
   clipping. Unit tests.

5. **Pillar 2 unified v(s)** along the smooth curve. Replaces
   discrete corner/straight accel split. Unit tests.

6. **PH spline research (subagent, parallel).** If a clear winner
   over quintic emerges, swap before integrated HW test.

7. **Integrated HW test.** One print; full stack.

8. **Pillar 4 (global optimizer)** — deferred until post-ship, revisit
   once HW numbers are in.

**Rough effort estimate:**

- Prerequisites: 1–2 weeks (mostly parallel).
- Pillar 3: 1 week engineering.
- Pillar 1: 1 week engineering + 1 week tuning.
- Pillar 2 v(s): 1 week engineering.
- Shape research: 3–5 days subagent-time.
- Integrated HW test + iteration: 1–2 weeks.
- **Total**: 4–6 weeks wall-clock with parallelisation.

## Compatibility

Stays within the Kalico planner architecture. Replaces geometry
primitives (arc → smooth), adds a new post-processing stage (inverse
shaper), threads an extra constraint class (extruder), and ports two
bleeding-edge modules (smooth-IS, HP-stepcompress). No trapq format
changes. No MCU firmware changes — splines stay on the host; the
MCU continues to receive pre-computed step events.

Per the fork's no-runtime-flags policy, the final state uses
smooth-curve-only blending (no "arc vs smooth" switch). Config surface
is normalised around the new shape at merge time.

## Open design questions

1. **PH spline vs clothoid vs quintic for MO context.** Research
   subagent deliverable. First-order expectation: PH quintic
   dominates on evaluation cost (closed-form) and quintic Hermite
   on implementation cost (already built). Clothoid likely
   out-competed by PH on evaluation cost with no upside.

2. **Sub-segmentation auto-derivation.** What shaper-parameter
   formula should drive `max_chord_err` and `max_d_kappa`? Rough
   intuition: segment boundary κ-step should be below
   `1 / (shaper_freq × 2π × buffer)` in rejection-bandwidth terms.
   Needs a derivation (subagent candidate).

3. **Inverse-shaper lookahead window.** For discrete FIR, finite-
   window deconvolution needs 2–3× shaper span = 15–20 ms. Does
   `blendplanner`'s existing lookahead cover this or do we need a
   separate buffer? Depends on how smooth-IS merges — polynomial
   inverse may be local, eliminating the lookahead question.

4. **Non-linear PA integration with inverse shaper.** PA adjusts
   extruder position based on XY accel. In MO, "XY accel" at the
   extruder is the *physical* (= planned) accel, not the
   pre-distorted commanded accel. The PA module needs to read from
   the planned path, not the commanded one. Minor plumbing, worth
   flagging.

5. **Target numbers at `cruise_ratio=0.05`.** User is testing below
   the 0.1 baseline used in sim configs. If 0.05 becomes the default,
   sim calibration and target times need re-running.

6. **Shape-research swap gate.** Define the condition under which
   the subagent's shape recommendation triggers a swap: measurable
   sim-time delta > X%, or implementation cost < Y hours, or both.
