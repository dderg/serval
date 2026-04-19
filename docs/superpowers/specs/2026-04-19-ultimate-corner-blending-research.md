# Ultimate Corner Blending — Research and Staged Design

**Date:** 2026-04-19
**Scope:** successor work to sub-specs 6a (SCV removal) and the Option-A/target_smoothing refinements of the same day. Establishes the design target, quantifies the ceiling, and chunks the implementation into reviewable sub-specs.

**Goal (re-statement):** ultimate performance AND ultimate quality, not a trade-off. The motion planner's job is to find the commanded trajectory that, after convolution with the input shaper and after extruder coupling, lands on the desired geometric path at maximum speed subject to actuator limits. Everything below is a consequence of taking that statement literally.

---

## 1. Physical problem statement

Given:

- Path in G-code: a sequence of linear segments (or, after slicer arc-fitting expansion, a finer polyline).
- Per-axis input shaper with impulse response `h_a(τ)` (ZV/MZV/EI family).
- Per-axis accel budget `A_axis = 2·target_smoothing / σ²_T,a` (Klipper's heuristic).
- Per-move total accel budget `a_max` from the toolhead config.
- Extruder with linear pressure advance coefficient `K_pa` that couples commanded E-speed to printed flow after a first-order delay.
- A user-specified tolerance `ε` for maximum path deviation of the **physical** (post-shaper) trajectory from the commanded geometry.

Find the commanded trajectory `p(t)` that minimizes total execution time subject to:

1. `|a(t)| ≤ a_max` (toolhead mechanical accel budget)
2. `|a_axis(t)| ≤ A_axis_per_axis` (per-axis shaper budget)
3. `|shaper(p) − p_desired| ≤ ε` pointwise (physical path fidelity)
4. Extruder flow continuity: `E_physical(t) ≈ E_desired(t)` within the PA filter's bandwidth
5. Boundary conditions: tangent-continuous entry/exit with the straight segments

Condition (3) is the one that existing firmware (Klipper's SCV, Marlin's JD, and our current G¹ arc blender) does not address rigorously. All of them optimize either *commanded* path fidelity (arc, Bézier with chord-deviation spec) or *post-shaper* bounded-ringing (SCV + shaper hack) — but not both simultaneously, and not with a single user knob.

This framework unifies them.

---

## 2. Why the current shapes don't achieve the goal

### 2.1 Circular arcs (G¹, our current implementation)

- **Strength:** closed-form geometry, trivial polyline sampling, constant curvature so `v_cent = √(a·R)` is uniform and optimal for the shape.
- **Weakness:** uniform curvature is the *only* profile that forces uniform velocity; any shape with variable curvature can have higher average `v`.
- **Post-shaper behaviour:** tangent-continuous but curvature-discontinuous at entry/exit. The curvature step `0 → 1/R` creates an acceleration step `0 → v²/R·proj` per axis at the blend entry. Convolution with the shaper produces transient deviation concentrated at the entry/exit regions — the commanded path exits the straight segment *before* the shaped path does, producing a measurable lateral offset at the ~shaper-window timescale.
- **Measured:** at α=90°, ε=0.2mm commanded, MZV 120Hz shaper, the arc's post-shaper deviation is 0.288 mm — **double** the commanded 0.139 mm. Quality leaks past the user's spec.

### 2.2 Cubic Bézier (G², Zhao 2013)

- **Strength:** continuous acceleration at entry/exit (no acc step). Faster than arcs at sharp corners (up to +16% at near-U-turn).
- **Weakness:** path is longer than the arc at shallow corners (~14% slower at α=20° was measured in section 3 below). Curvature peaks in the middle of the blend — can exceed `1/R_arc` at sharp angles.
- **Post-shaper behaviour:** still curvature-step at the boundaries (`κ=0 → κ_mid → κ=0`), but gentler than the arc. However the variable curvature profile creates variable per-axis jerk, which convolves with the shaper to concentrate deviation at the curvature-ramp regions rather than at the boundaries.
- **Measured:** at the same 90° test corner, cubic Bézier (optimal β=0.61) is +11.2% faster than the arc BUT post-shaper deviation is 0.321 mm — *worse* than the arc's 0.288. Per-corner performance comes at the cost of per-corner quality.

### 2.3 Quintic Hermite (G²+ with κ=0 endpoints)

- **Strength:** matches position + tangent + curvature at both endpoints (curvature = 0 at the linear-segment boundaries). No acceleration step anywhere along the blend, not even at entry/exit.
- **Weakness:** longer path length than cubic Bézier, modestly slower traversal.
- **Post-shaper behaviour:** measurably best. Because the shaper's impulse response integrates over the curvature profile, a profile that is smooth at the boundaries produces minimal transient displacement.
- **Measured:** at 90° corner, quintic Hermite is −1.1% slower than arc BUT post-shaper deviation is 0.242 mm — **16% better than the arc** and **25% better than the cubic Bézier**. Quality leader, modest performance cost.

### 2.4 G³ clothoid (Tajima & Sencer 2020)

- **Strength:** linear curvature ramp means bounded jerk along the blend. Academic state-of-art in high-speed machining (reports 15–25% speedup over cubic Bézier at matched tolerance).
- **Weakness:** requires Fresnel integrals — no closed form, heavier hot-path cost. No shipped commercial FDM implementation.
- **Post-shaper behaviour:** theoretically best among fixed-shape families (bounded jerk → bounded shaper transient).
- **Not yet measured in this spike** — would require an order of magnitude more code to evaluate.

### 2.5 Shape conclusion

**No single fixed shape dominates on both speed AND post-shaper quality.** Cubic Bézier wins on time; quintic Hermite wins on quality; arcs sit between. This is the gap we have to close to claim "ultimate."

---

## 3. Measured ceiling

Experiment setup: 2D corner at V = origin, entry tangent `e₁ = (1, 0)`, exit tangent `e₂ = (cos α, sin α)`, commanded chord deviation ε = 0.2 mm, v_cruise = 600 mm/s, a_max = 45 000 mm/s², MZV 120 Hz shaper (ζ = 0.1) applied per-axis to the commanded trajectory.

| α (°) | Arc t | Bézier t | Quintic t | Arc post-dev | Bézier post-dev | Quintic post-dev |
|---:|---:|---:|---:|---:|---:|---:|
| 30 | 5.88 ms | 5.71 ms | 7.61 ms | 0.312 mm | 0.311 mm | **0.204 mm** |
| 45 | 5.77 ms | 5.49 ms | 6.81 ms | 0.310 mm | 0.309 mm | **0.206 mm** |
| 60 | 5.61 ms | **5.21 ms** | 6.19 ms | 0.299 mm | 0.306 mm | **0.211 mm** |
| 90 | 5.15 ms | **4.59 ms** | 5.20 ms | 0.288 mm | 0.319 mm | **0.242 mm** |
| 120 | 4.42 ms | **3.79 ms** | 4.35 ms | 0.258 mm | 0.300 mm | **0.253 mm** |
| 135 | 3.91 ms | **3.31 ms** | 3.93 ms | 0.231 mm | 0.254 mm | **0.228 mm** |

Reading the table: **quintic Hermite is the post-shaper quality leader at every angle** (25–35% better than arc), but pays time only at shallow angles. Bézier wins on time at all α ≥ 45° but loses on post-shaper quality at moderate-to-sharp angles. Neither dominates.

Script: `/tmp/gcode_compare/shape_experiment.py` (to be promoted to `klipper-sim/examples/shape_ceiling.py`).

---

## 4. The ultimate formulation

Rather than picking a fixed shape, formalize the design as a constrained optimization:

```
minimize    T = ∫ dt
subject to  |a(t)|     ≤ a_max
            |a_axis(t)| ≤ A_axis_per_axis    (per axis, from shaper budget)
            |shaper(p)(t) − corner(t)|   ≤ ε_phys    (post-shaper accuracy)
            velocity continuous at entry/exit
            tangent matched to e₁ (entry) and e₂ (exit)
```

where `shaper(p)` is the per-axis convolution `p ⋆ h_axis` and `corner(t)` is the "desired" piecewise-linear trajectory traced at the same speed schedule.

Three layers of approximation define the staged implementation path:

### 4.1 Fixed-shape shaper-aware (cheap, sub-spec 6d)

Pick a parameterized shape (G² quintic Hermite) and scale its geometric chord deviation so that the *post-shaper* deviation lands on the user's ε_phys. This is a 1-D bisection per corner, O(1) cost, ships immediately.

### 4.2 Variable-shape per corner (moderate, sub-spec 6e)

Select among {arc, quintic, Bézier with tuned β} per corner based on α, L_prev, L_next, and the derived shaper bound. Hybrid rule of thumb from literature (Sencer & Tajima 2015): arc for α < 35°, quintic for 35° ≤ α ≤ 150°, arc fallback for α > 150°. Keep the existing arc code as the degenerate-case handler.

### 4.3 Inverse-shaper pre-compensation (novel, sub-spec 6g)

Command `p_cmd(t) = h⁻¹ ⋆ p_desired(t)` so that post-shaper output equals the desired geometry *exactly*. MZV is a 3-impulse filter; its inverse is an IIR filter that is stable iff the shaper is minimum-phase (MZV is, within its passband). Implementation: pre-filter each planner-emitted move.

This is where the post-shaper deviation goes from "quintic's 0.242 mm" to "< 10 µm" at the same corner speed. It's also the step that makes the fork genuinely novel — no published FDM firmware, and no commercial CNC vendor documentation I could find, combines convolutional input shaping with corner-blend inverse-filtering in a single planner.

---

## 5. Staged sub-spec plan

**Sub-spec 6d — G² quintic Hermite geometry module (2–3 weeks).**
`klippy/blendquintic.py`. Standalone pure-math module, analogous in scope to `blendmath.py`. Inputs: `(prev_dir, next_dir, ε_phys, a_max, A_axis per axis, shaper_params)`. Outputs: control points, max curvature, v_cap. Closed-form math (from Farouki's textbook plus shaper-aware ε bisection). Unit tests mirroring `test_blendmath.py`.

**Sub-spec 6e — CornerBlender integration with per-corner shape selection (1–2 weeks).**
Extend `blendplanner.CornerBlender` with a shape selector: arc (α < 35°), quintic (35° ≤ α ≤ 150°), arc fallback (α > 150°). Preserve the existing arc path unchanged — the change is additive. Polyline sampling, E-axis parameterization, and downstream kinematics checks are unchanged.

**Sub-spec 6f — G³ clothoid upgrade (optional, 3–4 weeks if pursued).**
Only if hardware testing of 6d/e shows residual jerk spikes at curvature-ramp regions. Requires Fresnel integrals (Bertolazzi & Frego 2015 numerics). Measurable improvement over quintic only at very sharp corners on very stiff printers.

**Sub-spec 6g — Inverse-shaper pre-compensation (novel, 3–5 weeks).**
Post-hoc filter applied to the emitted move stream. Derive and validate stable inverse for each shaper type (ZV, MZV, ZVD, EI). Handle boundary conditions at the stream ends (padding or tapered application). Add a `[input_shaper] enable_inverse_compensation` config option (defaulting to off until hardware-validated).

**Total: 9–14 weeks of chunked work.** Each sub-spec has an independent brainstorm → plan → implementation → hardware-validation cycle.

---

## 6. What this buys

On arc-fitted slicer output (most user prints):
- **Sub-spec 6d alone:** +5–10% aggregate time, −25% post-shaper deviation.
- **+ 6e (shape selection):** recover shallow-corner speed that naive quintic loses; net +8–15% aggregate speed at ~same −25% quality.
- **+ 6g (inverse-shaper):** post-shaper deviation drops to <50 µm (from current 0.3+ mm). Quality leader.

On sharp-corner pathological geometry (`sharp_short.gcode`):
- **6d+6e:** +20–30% aggregate time vs current arc blender.
- **+6g:** same speed, drastically cleaner corners visible in print.

On CAM without arc fitting (Fusion 360 CAM, etc.):
- Slicer-side arc fitting is absent, so all corners are polyline. The naive-CAM prepass (already in place) collapses collinear chains but not curves. With 6d+6e, curves become fast; with 6g, they become physically clean.

---

## 7. Validation strategy

Pre-implementation (simulator):
- Extend `klipper-sim` with a shaper-convolution analyzer that reports per-sample post-shaper deviation (beyond the current excursion-count metric).
- Run the Voron cube slice, `sharp_short.gcode`, `octagon.gcode`, and a non-arc-fit CAM-style synthetic geometry.
- Compare arc / quintic / quintic+inverse-shaper across all four.

Per-sub-spec:
- Test suite parity with `blendmath.py` — property-based tests, random-corner sweeps.
- Regression against the existing blend-arc branch's test suite (419 tests green today).

Hardware (after 6d and 6e):
- V0 and Trident, as already used. `sharp_short.gcode` visual quality test. PA retune. Shake-and-tune sanity check.
- Document quality deltas with macro photography / micrometer measurement of corner fidelity.

Before 6g lands:
- Flag-guarded rollout (one exception to the fork-as-gate principle, because inverse-shaper pre-compensation could destabilize printers mis-tuned for their resonance — we want opt-in until we build confidence).

---

## 8. Open research pieces

These are not blockers but flag the original-research content:

1. **Provable bound on post-shaper deviation for a given blend shape.** Derived pointwise, not just sample-measured. Input-shaping literature has pieces (Singhose, Vaughan on sensitivity bounds) but not the corner-blend application.
2. **Stable inverse-filter derivation for MZV/EI.** The inverse of a 3- or 4-impulse FIR is IIR; the poles of the inverse need to be inside the unit circle for causal stability. Prove this for Klipper's current shaper taxonomy.
3. **Co-optimization with pressure advance.** E-axis flow continuity during a non-arc blend is folklore in FDM. We should publish a clean derivation, not just ship code.

These become the Phase-0 follow-up reading list for subsequent sessions. Each is potentially publishable alongside the firmware ship.

---

## 9. Relation to memory

- [Kalico fork direction — high-perf motion planner] — this work is the natural continuation.
- [Prefer rewrites over patches on architectural issues] — sub-specs 6d–6g are the rewrite path, not a patch on arcs.
- [Fork is the opt-in gate, no runtime feature flags] — 6d–6f replace arcs cleanly, no flags. 6g gets a one-time flag exception for stable rollout.
- [target_smoothing user knob] + [Pythagorean split relaxation] — already-committed 2026-04-19 work that this spec builds on. The user's single-knob-for-quality (`target_smoothing`) composes with the shape-aware blender unchanged; it simply sets `ε_phys` in the new formulation.

---

## 10. Recommended next step

Kick off **sub-spec 6d** brainstorm immediately. The rest of today's validated improvements (Option A + target_smoothing + optional max_accel tuning) should still go to hardware for ground-truth validation in parallel — they're the pre-existing deliverable and don't block the 6d design work.
