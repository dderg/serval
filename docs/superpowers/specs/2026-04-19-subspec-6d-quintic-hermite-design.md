# Sub-spec 6d — G² Quintic Hermite Geometry Module

**Date:** 2026-04-19 (revised 2026-04-19 after math validation)
**Status:** ready-for-plan
**Depends on:** existing `klippy/blendmath.py` and `klippy/blendshaper.py` (no code change required in those for 6d)
**Unlocks:** 6e (per-corner shape selection), 6g (inverse-shaper benefits from smoother input)

---

## What this sub-spec does

Add a standalone pure-math module `klippy/blendquintic.py` that computes a **G² symmetric quintic Bézier** corner blend — analogous in scope and boundary conventions to the existing `blendmath.py` (which handles G¹ arcs). No integration with the `CornerBlender` yet; that is 6e.

---

## Design goals (resolved)

1. **User-facing quality knob is commanded chord deviation** (`corner_max_deviation` — same semantics as today's arc blender). Post-shaper physical deviation is an incidental benefit of G² continuity, not a user knob. 6g is the separate work that closes the loop on post-shaper deviation directly.
2. **Objective: maximum traversal speed at a fixed commanded deviation.** Subject to existing actuator limits (acceleration, per-axis shaped-acceleration, rotation-jerk from the input shaper, user `max_velocity`).
3. **One shape family, tuned per corner.** The symmetric 6-point quintic has a single free shape parameter `r ∈ (0, 1)` controlling how far inward the inner control points sit along each tangent. `r` is angle-dependent — chosen by closed-form fit so min traversal time is achieved at every corner angle.

---

## Geometry — the six control points

Symmetric quintic Bézier with six control points, parameterized by the tangent length `d` and the shape ratio `r`:

```
Q0 = V − d · e1
Q1 = V − (r·d) · e1
Q2 = V − (r·d) · e1        (Q2 coincident with Q1 ⇒ κ = 0 at entry)
Q3 = V + (r·d) · e2        (Q3 coincident with Q4 ⇒ κ = 0 at exit)
Q4 = V + (r·d) · e2
Q5 = V + d · e2
```

where:
- `V` is the corner vertex
- `e1` is the entry tangent unit vector (from upstream toward `V`)
- `e2` is the exit tangent unit vector (from `V` toward downstream)
- The **deflection angle** `θ` between `e1` and `e2` matches the existing blendmath convention: `θ = 0` when collinear, `θ = π` for a U-turn. `cos θ = e1 · e2`.

Evaluation in Bernstein form at parameter `t ∈ [0, 1]`:

```
B(t) = Σ_{i=0..5} C(5, i) · (1−t)^(5−i) · t^i · Q_i
```

With the coincident-pair structure, this simplifies; implementation should use a De Casteljau evaluator rather than expanded powers for numerical stability.

---

## Closed-form chord deviation

Under the deflection-angle convention, `|e2 − e1| = 2·sin(θ/2)`. Evaluating the Bernstein form at `t = 0.5`:

```
B(0.5) − V = ((1 + 15·r)/32) · d · (e2 − e1)
```

So:

```
deviation(d, r, θ) = ((1 + 15·r) / 16) · d · sin(θ/2)
```

**Inverse** (given the user-set deviation `ε` and the shape `r`):

```
d(ε, r, θ) = 16·ε / ((1 + 15·r) · sin(θ/2))
```

Sanity check: at `r = 4/5`, the coefficient is `13/16`, matching the empirical value in `klipper-sim/examples/shape_ceiling.py`.

---

## Shape parameter `r(θ)` — closed-form fit

**Previous brief locked `r = 4/5`; validation showed that value is near-optimal only near `θ ≈ 150°` (sharp corners) and costs 20–40% traversal time at deflections in the 30°–90° band (shallow to medium corners).** The subagent sweep (151 angles × 3 deviations) established three facts:

1. The optimal `r*` is **independent of commanded deviation `ε`** (to 1e-11 numerical noise).
2. `r*(θ)` ranges from ~0.50 at shallow corners to ~0.87 at near-U-turn corners.
3. A quadratic fit in `θ_deg` is within 0.21% of the true per-corner optimum (worst case, at `θ ≈ 160°`). A runtime optimizer gains 0.19% for ~500× more flops per corner.

**Adopted fit** (deflection angle in radians):

```
r(θ) = 0.5085 − 0.03785·θ + 0.05715·θ²        (θ in radians)

Safety clamp: r ∈ [0.50, 0.86]
```

Equivalent in degrees: `r(θ_deg) = 0.5085 − 6.606e-4·θ_deg + 1.741e-5·θ_deg²`.

Validity window: the fit is validated for `θ ∈ [10°, 160°]` deflection. Outside this window:
- **`θ < 10°`** (near-collinear): the quintic still evaluates cleanly with clamped `r = 0.50`, but the blend is trivially small. Sub-spec 6e decides whether to skip blending entirely.
- **`θ > 160°`** (near-U-turn): control points collapse; arc fallback. Sub-spec 6e handles routing.

---

## Peak curvature

Unlike the arc's constant `1/R`, the quintic's curvature varies along the path. The previous brief's claim that the peak is at `t = 0.5` is **only true for `r ≲ 0.3`**. At the `r` values this design uses (0.50–0.86), the true peak is off-center and up to 50× larger than the midpoint value.

**Consequence: the velocity cap cannot be derived from a closed-form `κ_peak`.** The peak location is the root of a degree-7 polynomial in `t`; no clean formula exists.

**Evaluation strategy:** dense sampling of `κ(t)` at fixed quadrature points, then take the max. Concretely:

```
# Given d, r, θ, and the in-plane unit vectors e1, e2
ts = linspace(0.0, 1.0, K)          # K ≈ 16–24
kappas = [curvature_at(t, Q0..Q5) for t in ts]
kappa_peak = max(kappas)
```

The curvature at a sample `t`:

```
κ(t) = |B'(t) × B''(t)| / |B'(t)|³
```

with `B'(t)` and `B''(t)` from Bernstein derivatives (both available from a standard De Casteljau evaluator). Cost: ~12 multiply-add blocks per sample × ~20 samples = comparable order of magnitude to the arc's one `cos/sin` call. Acceptable.

**Alternative for the implementation to decide:** a Brent-refined peak after a coarse-grid bracketing. Same numerical answer, potentially fewer evaluations. Left to the plan step.

---

## Velocity cap derivation

Mirror `blendmath.blend_geometry`'s structure with three bounds, swapping quintic-specific quantities in for arc-specific ones.

**(a) Centripetal cap** (shape governs how fast the toolhead can circle through peak curvature without exceeding `a_max`):

```
v_cent = √(a_max / κ_peak)
```

`a_max` here is the *full* acceleration limit (consistent with the Pythagorean relaxation landed in commit `2c76bac7`).

**(b) Shaper per-axis velocity cap.** `blendshaper.compute_shaper_bounds` was written for a constant-κ arc; for a curve the shaper bound should be evaluated where it is *most restrictive*, which is not always at the peak-κ point.

Subagent verification showed that evaluating only at the peak-κ point overshoots the true-minimum bound by up to **19%** at shallow corners with axis-rotated bisectors. A **three-point sample** at `t ∈ {0.25, 0.5, 0.75}` cuts the worst-case overshoot to ~6% at O(1) cost. Adopt that.

```
v_step_cap = min over t ∈ {0.25, 0.5, 0.75} of:
    compute_shaper_bounds(
        per-axis shaped accel,
        R = 1 / κ(t),
        n̂ = unit inward normal at t,
        p̂ = blend plane normal,
    )
```

Tangent at `t = 0.5` is the bisector `(e1 + e2)/|e1 + e2|` (clean, by symmetry). Normal at `t = 0.5` is `(e2 − e1)/|e2 − e1|`. At the off-center samples, tangent and normal are computed from `B'(t)` and the principal-normal derivation.

**(c) Rotation-jerk cap** (shaper's rotation-through-the-blend bound). The tangent sweeps through the full deflection `θ` over the blend duration; the existing arc formula in `blend_geometry` uses `θ` and the *arc* radius. For the quintic, the binding parameter is again peak curvature:

```
v_jerk ∝ (j_eff / κ_peak)^(2/3)   (exact prefactor follows the arc derivation)
```

Refine in the plan step: the derivation should use whichever `κ(t)` along the blend maximizes `ω_eff = v · κ(t)`; since `v` is constant along a velocity-capped blend, this is the same `κ_peak`.

**Final cap:** `v_cap = min(v_cent, v_step_cap, v_jerk, max_velocity)`.

---

## Polyline sampling

Quintics do not have a clean `R·(1 − cos(Δθ/2))` chord-error formula. Use **adaptive De Casteljau subdivision**:

```
subdivide(Q0..Q5):
    if flatness(Q0..Q5) < max_chord_err:
        emit segment
    else:
        split at t = 0.5, recurse on each half
```

Flatness metric: max perpendicular distance of `Q1..Q4` from the chord `Q0–Q5`. Standard algorithm, ~3–5 recursion depths at printer tolerances. Expected output: 6–16 polyline points per corner (vs 2–5 for arcs).

---

## E-axis parameterization

Extruder E coordinate must track physical arc length along the quintic. Arc length is not closed-form; use per-sub-segment **Gauss–Legendre 5-point** integration of `|B'(t)|`. Reuse `blendmath.interpolate_extruder`'s callsite structure with a per-segment `s` array instead of uniform spacing.

---

## File layout

```
klippy/blendquintic.py          (new, ~300 LOC estimated)
    - @dataclass QuinticBlend
        (Q0..Q5, tangents, v_cap, kappa_peak, arc_length, plane_normal)
    - quintic_geometry(prev_dir, next_dir, L_prev, L_next, eps, a_max,
                       j_eff, shaper_snapshot) -> QuinticBlend | None
    - blend_from_moves_quintic(prev_move, next_move, eps,
                               toolhead=None) -> QuinticBlend | None
    - segment_quintic(q, max_chord_err) -> polyline
    - interpolate_extruder_quintic(polyline, ...)

test/test_blendquintic.py       (new, ~600 LOC estimated)
    - test_chord_deviation_matches_eps
    - test_curvature_zero_at_endpoints
    - test_tangent_matches_entry_exit
    - test_r_fit_matches_optimum                  (r(θ) within 0.5% of per-angle optimum)
    - test_peak_curvature_evaluator               (numerical max vs dense reference to 1e-6)
    - test_v_cap_centripetal
    - test_v_cap_shaper_bounds_three_point        (3-sample min beats single-sample)
    - test_v_cap_rotation_jerk
    - test_polyline_flatness                      (max chord error ≤ tolerance)
    - test_e_axis_conservation                    (sum of deltas == Δeps · e_per_mm)
    - test_degenerate_near_collinear              (θ < 10°, returns small-blend or None)
    - test_degenerate_near_u_turn                 (θ > 160°, returns None; 6e routes to arc)
    - test_random_corners_property                 (50-run property sweep, mirror test_blendmath.py)
    - test_simulator_parity                        (shape_ceiling.py: post-shaper deviation ≤ arc)
```

---

## Test plan

1. **Numeric geometry**: closed-form deviation matches Bernstein eval at 50 random corners to 1e-9.
2. **Fit accuracy**: `r(θ)` is within 0.5% traversal-time of per-angle optimum across `θ ∈ [10°, 160°]`. Reference: golden-section search over `r` at each angle.
3. **Velocity cap safety**:
   - Centripetal: `v_cent² · κ_peak ≤ a_max` (always).
   - Shaper: dense-sampling check (100+ points along the blend) confirms three-point-sampled cap is never exceeded.
   - Rotation-jerk: property test across angles.
4. **Polyline fidelity**: max chord error of emitted polyline ≤ configured `max_chord_err`.
5. **E-axis conservation**: sum of extruder deltas along polyline equals `(length along quintic) · e_per_mm`, to 1e-6.
6. **Simulator comparison**: extend `klipper-sim/examples/shape_ceiling.py` to assert quintic post-shaper deviation ≤ arc post-shaper deviation at `θ ∈ {30°, 60°, 90°, 120°, 150°}`.
7. **Property tests**: 50-run random-corner sweep with checks for (a) smoothness (no NaN/Inf), (b) monotonicity of `v_cap` in `ε`, (c) `r(θ)` stays inside the `[0.50, 0.86]` clamp.

---

## Scope and exit criteria

**Estimated effort:** 2–3 weeks, broadly:
- 3 days math module: Bernstein evaluator, closed-form deviation, `r(θ)` fit, peak-κ evaluator
- 2 days velocity-cap integration (three-point shaper sampling, centripetal, rotation-jerk)
- 2 days adaptive subdivision + E-axis arc-length
- 3 days test module (reach parity with `test_blendmath.py`)
- 2 days simulator assertions + hardware smoke
- 3 days polish, docs, commit stack

**Done when:**
- All categories in the test plan green
- `test_blendquintic.py` reaches test-count and property parity with `test_blendmath.py`
- `klipper-sim/examples/shape_ceiling.py` extended with quintic assertion; reproduces the measured ≤0.25 mm post-shaper deviation at `ε_cmd = 0.2 mm`, MZV 120 Hz
- No integration with `CornerBlender` yet — that is 6e
- Purely additive module; no production code path uses it until 6e wires it in

---

## After this sub-spec

6e introduces `CornerBlender` shape selection between `blendmath` (arc) and `blendquintic` (quintic) based on `θ`. The arc fallback ranges (`θ < 10°` and `θ > 160°`) are 6e's responsibility. 6g's inverse-shaper pre-compensation benefits from the smoother input the quintic provides, and can proceed in parallel — it does not depend on shape choice, only on the planner emitting a well-defined commanded trajectory.

---

## Revision history

- **2026-04-19 (initial brief)** — empirical shape parameter `r = 4/5` from single-angle experiment; midpoint assumed to be peak curvature; three open questions deferred to implementation.
- **2026-04-19 (revised spec)** — math open questions resolved via two subagent reviews. `r = 4/5` replaced with `r(θ)` quadratic fit. Peak-curvature location identified as off-center (no closed form); dense-sampling evaluator adopted. Shaper velocity bound changed from single-point to three-point evaluation. Deflection-angle convention stated explicitly (matching blendmath).
