# Plan 4 — Pillar 2 integration (smooth-IS shaper cap, sub-segmentation, suppression)

**Date**: 2026-04-21
**Branch**: `magnum-opus`
**Pillar**: 2 — smooth-accel corner primitive (geometry portion)
**Status**: design

## Goal

Finish Pillar 2 geometry so the quintic corner primitive has a correct
shaper-derived velocity cap under the **smooth-IS** shaper family (now
the fork's default after Plan 2), a principled polyline sub-segmentation
density, and a shaper-aware suppression rule that does not double-count
chord tolerance. Also refine Plan 3's extruder cap propagation through
the blend and pin endpoint behavior with tests.

The unified `v(s)` along the curve (Pillar 2b) stays deferred. So does
the feedforward inverse shaper (Pillar 1). This plan is the minimum
integration pass that makes smooth-IS-at-corners correct on hardware
without waiting for Pillar 1.

## Context

The quintic Hermite primitive already landed on `magnum-opus`:

- `klippy/blendquintic.py` (598 LOC) — `QuinticShape`, polyline
  subdivision, per-s `v_cap_fn` from local curvature, rotation-jerk
  bound, centripetal cap.
- `test/test_blendquintic.py` (613 LOC) — surface test coverage.
- `klippy/blendplanner.py:100` — wires `QuinticShape.from_moves` in
  `CornerBlender.feed`.

Plan 2 merged the Smooth Input Shaper (SIS) family from
`upstream/bleeding-edge-v2`. Plan 3 added the extruder-first-class
cap (`blendextruder.cap_move`) wired at `toolhead.move`. Neither plan
updated the quintic blend's shaper-bound path.

## Why now

**Blocker on hardware.** `blendmath.py:228-239` explicitly records
smooth-IS axes with `A_axis=0.0` ("Arc-blending's velocity cap today
only consumes the impulse family"), which `blendshaper.py:101-102`
then skips. `_SHAPER_SPAN_FACTOR` (`blendshaper.py:44-51`) has only
FIR shaper names; smooth-* names would raise `ValueError` but never
reach it because the `A_axis=0` filter fires first.

**Net effect**: on any user running a smooth-IS config (`smooth_zv`,
`smooth_mzv`, `smooth_ei`, `smooth_2hump_ei`, `smooth_zvd_ei`,
`smooth_si`), the quintic blend has **no shaper-derived velocity cap
at corners**. The user's Plan 2 hardware validation at clean quality
was running with this silently disabled. Same class of P0 as Plan 3's
`extruder_stepper`-singular bug, caught by opus review not by haiku.

## Scope

### In scope (5 deliverables)

1. **[P0] Smooth-IS shaper cap.** Derive `A_axis` for polynomial
   smoothers. Extend `shaper_span` / `_SHAPER_SPAN_FACTOR` to cover
   the SIS family. Remove the `A_axis=0.0` stub branch. Live for
   `smooth_zv / smooth_mzv / smooth_ei / smooth_2hump_ei /
   smooth_zvd_ei / smooth_si`.

2. **Sub-segmentation density rule.** Replace the current
   `max_chord_err = max(20e-3, 0.2 * corner_deviation)`
   auto-scale (`blendplanner.py:140-150`) with a derivation from
   active shaper's rejection bandwidth, floored at the trapq
   minimum-move-time limit (~20 µm at 100 mm/s). Derivation goes
   to a math subagent.

3. **Shaper-aware suppression rule re-derivation.** Current
   `2·v·sin(φ/2)·σ_T ≤ corner_deviation`
   in `blendmath.py:141-183` is an arc-based derivation that
   **double-counts** for quintic (quintic path already deviates by
   `cd` from the vertex by construction; adding σ_T smear on top
   counts that deviation twice). Replace with: "skip the blend if
   sharp-V traversal under shaper smearing fits inside `cd` AND
   the blend traversal time at the quintic midpoint `v_cap` would
   be slower than sharp-V ramp time."

4. **Per-sub-move Plan 3 cap refinement.** Current blend emit at
   `blendplanner.py:197-199` uses `min(prev.accel, nxt.accel)` and
   `min(prev.max_cruise_v2, nxt.max_cruise_v2, shape_mid_v²)` —
   conservative (picks tighter of two ends). Push to per-sub-move
   precision: each sub-move's flow ratio `k` is the linear
   interpolation of `prev.axes_r[3]`/`nxt.axes_r[3]`; its Plan 3
   `(v_cap, a_cap)` is `blendextruder.cap_move`-equivalent at that
   `k`. Non-blocking, but unlocks throughput on asymmetric
   flow-rate transitions.

5. **Endpoint singularity tests.** `QuinticShape.v_cap_fn(0)` and
   `v_cap_fn(arc_length)` sit at control-point coincidences where
   the local curvature frame is degenerate. Pin the behavior
   (should fall back to prev/next tangent `v_cap`; may currently
   blow up or return inf). Add property tests; clamp if broken.

### Out of scope (deferred)

- **Unified `v(s)` along the curve (Pillar 2b).** The single-scalar
  `v_cap = v_cap_fn(arc_length/2)` used at `blendplanner.py:196`
  stays. A future plan replaces it with per-segment `v(s)`
  integration. That plan also owns absorbing Plan 3's cap directly
  into `v_cap_fn` rather than doing it move-by-move.
- **Feedforward inverse shaper (Pillar 1).** See "Pillar 1 notes —
  deferred to Plan 5" below.
- **Shape-pluggable strategy interface.** Quintic stays
  hard-wired. When/if a research subagent picks a winner between
  quintic / clothoid / PH quintic, extraction of the interface is
  cheap.
- **Clothoid / PH quintic implementations.**

## Pillar 1 notes — deferred to Plan 5

The brainstorming session for Plan 4 started as Pillar 1 (feedforward
inverse shaper) and pivoted when an opus architectural review
surfaced coupling issues. Pillar 1 is moved to Plan 5. The following
decisions from the Pillar 1 brainstorm are preserved for Plan 5 to
pick up:

- **Scope**: Smooth-IS only. No FIR (ZV / MZV / EI / 2HEI) inverse
  branch. Rationale: smooth-IS is the fork's primary forward path
  after Plan 2; FIR inverse has stability/deconvolution overhead we
  don't need. Dropping it simplifies the code meaningfully (no
  precomputed correction kernels per impulse pattern, no unit-circle
  zero verification, no 15-20 ms FIR-deconvolution lookahead).

- **Architecture**: **Option C — fuse `pre_distort` with the shaper
  at the stepper-query layer.** `trapq` stays `= planned physical
  trajectory`; `pre_distort` joins the forward shaper at position
  query time, so effective transform is
  `shaper(pre_distort(planned))`. PA reads `trapq` unchanged.
  Plan 3's extruder cap reads `move.accel` (planned) unchanged.
  **Options A (parallel planned + commanded buffers) and B (PA
  inverts pre_distort on read) rejected** — both break trapq's
  single-source-of-truth invariant.

- **Math form**: **FIR companion kernel** (Biagiotti–Melchiorri 2012
  / 2017). Stable finite-window approximate inverse, fit at
  shaper-config time. NOT a closed-form polynomial inverse and NOT
  a differential operator — compactly-supported LPF has a non-local
  analytic inverse, and a differential inverse amplifies step-gen HF
  noise. Original Magnum Opus design doc (line 96) claims
  "polynomial inverses are well-conditioned and local" — this is
  false as written; design doc wants a correction.

- **Smooth-IS kernel degree**: **6-8**, not 4.
  `shaper_defs.py:96-184` — only `smooth_zv` is degree 4;
  `smooth_mzv / smooth_ei` are degree 6; `smooth_2hump_ei /
  smooth_zvd_ei / smooth_si` are degree 8. Matters for the
  companion-kernel fit conditioning.

- **Overshoot handling**: **feed inverse saturation gain back into
  Pillar 2's `v_cap_fn`** as a planning constraint. Earlier "hard
  clip at mechanical `max_accel`" was wrong — hard clipping injects
  jerk discontinuity that the forward shaper cannot cancel, so you
  get full rebound ringing at saturated corners. Upstream reshape
  (plan conservatively so inverse doesn't saturate) is the correct
  fix. This couples Pillar 1 into Pillar 2's velocity profile.

- **Implementation location**: C-side, alongside
  `klippy/chelper/kin_shaper.c` (query-rate ≫ Python can keep up).
  Python sets coefficients via FFI at shaper (re)config,
  mirroring the existing `input_shaper_set_smoother_params`
  pattern (line 314).

- **Lookahead window**: composed operator support is ~2·T_sm ≈
  40-50 ms at 20 Hz shaper freq — extends step-gen lookahead by
  20-30 ms beyond current. Affects M400, SET_INPUT_SHAPER live
  tuning, SET_PRESSURE_ADVANCE, Plan 3 cap timing.
  `toolhead.note_step_generation_scan_time` plumbing work: budget
  2-3 days beyond "just write the inverse."

- **Effort estimate**: **1.5-2 weeks engineering** (revised up
  from design doc's "1 week"). Plumbing tax for the extended
  lookahead window + boundary-effect handling is real.

- **Literature anchors to read before coding**: Biagiotti–Melchiorri
  2012 "Trajectory planning for automatic machines and robots"
  (§5.8 inversion of smoothing B-spline filters);
  Besset–Béarée 2017 "FIR filter-based online jerk-constrained
  trajectory generation" (closest analog);
  Wang/Altintas 2022-2023 CIRP (predictive feedforward under
  saturation — the "feed saturation into v_cap" approach).

- **Blocker on Pillar 1 until Plan 4 lands**: trapq is
  piecewise-linear in velocity → Dirac in jerk at every segment
  boundary. Feeding a Dirac-jerk signal into the inverse produces
  wide HF content that the shaper can only reject in a narrow band.
  Residual HF is audible ringing at every polyline segment
  boundary. **Pillar 1 quality is gated on Pillar 2's sub-seg
  density being fine enough to keep residue below rejection
  bandwidth.** That's deliverable #2 of this plan.

## Deliverable details

### 1. Smooth-IS shaper cap path

**Problem.** Smooth-IS axes currently contribute nothing to the
quintic blend's shaper-derived velocity cap. The P0.

**Derivation (to math subagent).** The shaper's max-accel
contribution comes from its response to a step-change in commanded
acceleration. For FIR (impulse train `(A_i, T_i)`), this is
`sc.find_shaper_max_accel(impulses)` — already implemented. For
SIS (polynomial kernel `w(t)` on `[-T_sm/2, T_sm/2]`, integral = 1),
the equivalent quantity is `sup_x |∫ x''(t-τ) w(τ) dτ|` under an
idealised acceleration step — a closed-form integral over `w`.

Deliverable from the math subagent:

- Closed-form (or numerically-verified) `A_axis` expression for
  each smooth-IS family kernel.
- Equivalent `shaper_span` for SIS — straightforward: the kernel
  span is `T_sm` directly (field already in
  `shaper_defs.INPUT_SMOOTHERS`).
- Unit tests: compute `A_axis` both analytically and by numerical
  simulation; they must agree to 1e-6.

**Implementation.**

- `blendmath.py::_extract_shapers`: branch on
  `TypedInputSmootherParams`, compute `A_axis` via new helper.
- `blendshaper.py::_SHAPER_SPAN_FACTOR`: add smooth-* entries OR
  route `shaper_span` through a type-aware dispatcher that reads
  `T_sm` for smooth and the damped-period-factor for FIR.
- `blendshaper.py::compute_shaper_bounds`: no change; once `A_axis
  > 0` and `shaper_span` returns a finite value for SIS, the
  existing math works.

**Tests.**

- `test/test_blendmath.py`: add `_extract_shapers` cases for
  smooth-IS configs, verify `A_axis > 0`.
- `test/test_blendshaper.py`: `compute_shaper_bounds` under
  smooth-IS gives finite bounds that scale physically with
  `shaper_freq`.
- `test/test_blendquintic.py`: integration — `QuinticShape`
  velocity cap at a sharp corner under smooth_mzv @ 40 Hz is
  within 5% of the same corner under `mzv @ 40 Hz`.

### 2. Sub-segmentation density rule

**Problem.** `blendplanner.py::_resolve_chord_err` returns
`max(20e-3, 0.2 * corner_deviation)`. That's an arbitrary
auto-scale; no derivation from the shaper's rejection bandwidth.
For Plan 5's feedforward inverse to work, polyline segment
boundaries must not carry κ-step energy above the shaper's notch.

**Derivation (to math subagent).** Kernel `w` of span `T_sm` and
polynomial degree `N` has frequency response bounded by
`|W(2πf)| ≤ (2π f T_sm)^{-N}` (roughly — exact form per
kernel). The segment boundary κ-step `Δκ` at traversal velocity
`v` produces a commanded-jerk pulse of amplitude
`v² · Δκ / Δt` for duration `Δt = Δs / v`, whose DFT magnitude
at frequency `f` is bounded by `v² · Δκ · sinc(π f Δt)`. For
residual physical acceleration below the shaper's 5% vibration
floor at the tuned frequency `f_sh`:

```
Δκ_max ≈ (a_residual_budget) · |W(2π f_sh)|^{-1} / v²
```

At defaults `f_sh = 40 Hz`, `T_sm ≈ 24 ms`, `N = 8`, `v = 300
mm/s`, 5%-of-5000-mm/s² budget: `Δκ_max ≈ 3e-4 mm⁻¹`. Quintic
peak-κ at a 45° corner with `cd = 0.1 mm` is roughly `0.03 mm⁻¹`,
so required segment count is ~100 — segment length ~10 µm.

**Trap: trapq floor is ~20 µm at 100 mm/s** (minimum-move-time ~250
µs). Target segment length is below the floor. Subagent must
confirm numbers and propose the trade-off policy:

- (a) Accept floor at 20 µm; residual κ-step is above
  rejection bandwidth — Pillar 1's shaper cannot fully cancel
  segment-boundary content. Soft-fail: quality degrades slightly
  at high speed.
- (b) Relax chord tolerance locally at peak-κ (widen the quintic
  curvature, lose corner fidelity) to reduce `Δκ_peak` per
  segment. Quantitative trade: how much chord relaxation for
  how much segment-count reduction?
- (c) Velocity-limit the blend so `Δκ · v²` stays below the
  bandwidth budget. Equivalent to lowering `v_cap_fn` — the
  simplest fix, costs throughput.

**Implementation.** `_resolve_chord_err` becomes a function of the
active shaper parameters, not just `corner_deviation`. The
formula and chosen trade-off policy ((a), (b), or (c)) go into the
plan doc after subagent deliverable lands.

**Tests.** Property test: for each smooth-IS config on a 45°
corner, sub-seg boundary κ-step times `v²` is below the shaper's
rejection bandwidth budget at the tuned frequency.

### 3. Shaper-aware suppression re-derivation

**Problem.** `blendmath.py:141-183::suppressed_junction_v` computes
an SCV-equivalent junction velocity from the shaper's
impulse-moment `σ_T`. Used for both the "skip blend and run sharp"
decision and as the quintic-`from_moves`-returns-None fallback.
The formula is arc-based and double-counts chord tolerance when
the blend is quintic.

**Derivation (to math subagent).**

- The quintic traversal deviates from the sharp vertex by exactly
  `cd` by construction (that's the `d_from_deviation` input).
- The sharp-V traversal under shaper smearing deviates from the
  vertex by `~v · sin(φ/2) · σ_T · 2` (the σ_T-based formula).
- Correct suppression rule: **skip blend iff** sharp-V under-shaper
  deviation ≤ `cd` **AND** sharp-V ramp time at the σ_T-derived
  junction velocity is ≤ blend traversal time at the quintic's
  midpoint `v_cap`.
- Currently `2·v·sin(φ/2)·σ_T ≤ cd` is only the first clause, and
  `blendplanner.py:119` uses `suppressed_junction_v` as a cap
  for the `from_moves = None` case — that use is still correct
  (it's the SCV-equivalent cap when there's no blend).

**Implementation.** Rename / factor: keep
`suppressed_junction_v` (SCV-equivalent cap, used by the `None`
branch), add a new `should_suppress_quintic(prev, next, cd,
shape, th)` that returns bool based on the two-clause rule.

**Tests.**

- Suppression boundary: at `v → 0` both clauses trivially satisfied
  → skip blend (sharp is fine). At high `v` and small `cd` no
  suppression.
- Regression: existing tests of `suppressed_junction_v` continue
  passing (it's still the v-cap helper).

### 4. Per-sub-move Plan 3 cap refinement

**Problem.** `blendplanner.py:197-199` takes the tighter of
`prev`/`nxt` Plan-3 caps for all sub-moves in the blend. Sub-moves
near the entry should use prev's flow-ratio cap; near exit, nxt's.
Currently conservative (safe) but leaves throughput.

**Implementation.** For each sub-move constructed at line 201-206:
compute the interpolated flow ratio `k_i` (already done by
`interpolate_extruder` at line 185); call
`blendextruder.cap_move(sub_move, pa_snap, limits)` equivalent
with `k = k_i`. Use the resulting `(v_cap_i, a_cap_i)` instead
of `arc_cap_v` / `arc_accel` for that sub-move.

Caveat: `cap_move` takes a `Move` object, not a raw `k`. Either
call `cap_move(sub_move, …)` directly per sub-move (cost: tiny —
each sub-move already constructs a `Move`), or extract the inner
math into a `cap_k(pa_snap, k, v_target)` helper.

**Tests.**

- Asymmetric flow blend: prev `k = 0.8`, nxt `k = 1.5`. Before:
  all sub-moves use `min(cap_prev, cap_nxt) = cap_nxt` (tighter
  due to higher k). After: sub-moves near entry run faster.
  Verify via `a_cap` / `v_cap` shape along the polyline.
- Symmetric blend: no regression (both caps equal, min =
  interpolated).

### 5. Endpoint singularity tests

**Problem.** At `s = 0` and `s = arc_length`, the quintic's inner
control points coincide with endpoints — local curvature frame
(`_point_frame` at `blendquintic.py:~340`) can be degenerate.
`v_cap_fn(0)` may return `inf` or blow up on tiny denominators.

**Implementation.**

- Test first: call `v_cap_fn(0.0)` and `v_cap_fn(arc_length)` on a
  representative set of corner geometries (symmetric 45°, asymmetric
  120°, near-reversal 170°). Assert finite, positive.
- If any blow up: clamp with `max(PROJECTION_EPS, denom)` in the
  frame computation, OR return the parent move's
  `max_cruise_v` at endpoint (which is the right physical answer
  — at the blend boundary we're tangent to a straight move).

**Tests.** Added to `test/test_blendquintic.py` under a new
`TestEndpoints` class.

## Effort estimate

- Deliverable 1 (smooth-IS cap): 2-3 days incl. math subagent round.
- Deliverable 2 (sub-seg density): 2-3 days incl. math subagent round + trade-off decision.
- Deliverable 3 (suppression re-derivation): 1-2 days incl. math subagent round.
- Deliverable 4 (Plan 3 cap refinement): 1 day.
- Deliverable 5 (endpoint tests): 0.5 day.
- Integration + HW smoke: 2-3 days.

**Total: 1.5-2 weeks engineering** under subagent-driven-development
dispatch. Math subagent dispatches may overlap.

## Validation

Integrated-only per Magnum Opus testing philosophy. After all 5
deliverables land:

- Unit tests: new tests from each deliverable plus existing
  `test_blendquintic.py` / `test_blendshaper.py` / `test_blendmath.py`
  suites pass.
- Batch-sim: `Voron_Design_Cube_v7_ABS_22m13s` under smooth_mzv
  @ 40 Hz on the magnum-opus config produces finite corner
  velocities (current silent no-op means the shaper-cap is inf;
  post-Plan-4 it should be a real number).
- HW smoke: user runs one print; pass if no ringing regressions
  at corners, no Timer-too-close / send-too-old, `sysload` < 2.0.

## Risks

1. **Sub-seg density policy (deliverable 2) lands at floor.** If
   the math subagent confirms `Δκ_max` requires segments below
   the 20 µm trapq floor, we're stuck with partial shaper
   rejection. Plan 4 still lands — the smooth-IS-cap fix is
   independently valuable — but Plan 5's inverse shaper would
   see residual ringing at segment boundaries. Fallback (c)
   (velocity-limit the blend) keeps correctness but costs
   throughput. User decides the trade at plan review.

2. **Smooth-IS `A_axis` derivation takes longer than expected.**
   The analytical integral over polynomial kernels of degree 6-8
   is tractable but tedious. Math subagent may need 2 rounds.
   Budgeted in deliverable 1's 2-3 days.

3. **Suppression re-derivation finds the current SCV-equivalent
   cap is wrong for smooth-IS too** (not just for quintic). If
   so, deliverable 3 expands to also fix the
   `suppressed_junction_v` formula under smooth-IS — tractable
   but would add ~1 day.

4. **Per-sub-move Plan 3 cap changes polyline v-profile
   non-monotonically.** If Plan 3's `cap_move` bumps v up then
   down across the blend (because flow ratio dips mid-corner for
   whatever reason), the `calc_junction` pass in the outer
   lookahead may struggle. Worth a regression test on pathological
   flow-ratio profiles. If it bites, fall back to deliverable 4's
   "use min across the blend" (deferred).

## Successor (planned)

**Plan 5 — Pillar 1 (feedforward inverse shaper).** Picks up the
Pillar 1 notes above. Prerequisite: Plan 4 lands (sub-seg density
sufficient for inverse's HF content to stay below rejection
bandwidth).
