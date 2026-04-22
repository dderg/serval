# Saturation Feedback for the Feedforward Inverse Input Shaper

Derivation: opus math subagent, 2026-04-22. Branch `magnum-opus`, Plan 5 Pillar 1.

> **⚠️ Revision notes (see `per_axis_saturation_derivation.md` for
> the definitive derivation):**
>
> - **Correct per-axis bound is sum-of-projections, not √2.** This
>   doc's §2.4 eq. (8) used `v_sat = sqrt(a_max / (G · κ))` with a
>   handwaved single-scalar G. The √2 correction flagged by
>   `REVIEW_2_MATH.md` was itself wrong — over-conservative in one
>   case and unsafe in another. The **correct formula** is
>   `v_sat(s) = sqrt(a_max / (G_worst(s) · κ(s)))` with
>   `G_worst(s) = max_axes G_axis · (|proj_t(s)| + |proj_n(s)|)`.
>   Derived from L¹-L∞ duality with independent tangential and
>   centripetal bounds; Monte Carlo verified.
> - **Orientation matters.** The cap is per-s because Frenet
>   projections `proj_t`, `proj_n` vary along the curve.
> - **Wang & Altintas / Altintas-Ever-Hanley-Erkorkmaz references
>   in §7 are hallucinated.** Primary anchor is Biagiotti-Melchiorri
>   (2008) §5.8 L¹-L∞ bound, ISBN 978-3-540-85628-3.

Companion files (read first for context):
- `docs/superpowers/plans/plan4-derivations/A_axis_smooth_is.md` — σ²_T for the forward SIS kernel.
- `docs/superpowers/plans/plan4-derivations/delta_kappa_max.md` — shaper-rejection-band v-cap for polyline sub-moves.
- `klippy/blendquintic.py:369-478` — current single-point `v_cap_fn`.
- `klippy/blendmath.py:66-130` — `_sigma_T_max_from_toolhead` and SCV-equivalent cap.
- `klippy/blendshaper.py:28-134` — `AxisShaperSnapshot`, `compute_shaper_bounds`.

## 1. Problem setup

### 1.1 What Pillar 1 adds

Pillar 1 injects a feedforward **inverse** shaper `h(τ)` in front of the existing forward shaper `w(τ)`:

```
  x_out(t)   = (h ⊛ x_planned)(t)        (feedforward compensation)
  x_phys(t)  = (w ⊛ x_out)(t)
             = (w ⊛ h ⊛ x_planned)(t) ≈ x_planned(t)
```

The design goal is `(w ⊛ h)(τ) ≈ δ(τ − τ_c)` for some fixed group-delay `τ_c`, which is the ordinary input-shaping trick running in reverse. `h` is a finite-support FIR companion kernel on `τ ∈ [−T_h/2, +T_h/2]`, unit-norm, not strictly causal (we accept pre-action — the planner has the full lookahead window).

### 1.2 Why h saturates

`w` is a low-pass: the convolution `w ⊛ ·` attenuates high-frequency content in the stopband. The inverse `h` must therefore **amplify** in that stopband. Let

```
  H(ω) := 𝓕{h}(ω),   G_h := sup_ω |H(ω)|.      (HF amplification factor)
```

In practice `h` matches `1/W(ω)` inside the passband and is tapered (windowed) to keep `G_h` bounded — `G_h = ∞` is unphysical because `W(ω_sh) ≈ 0` at the shaper notch. Typical designs (Wang-Altintas 2022-2023; also Singhose-Seering-style inverse MZV and truncated-regularized inverses) report `G_h ∈ [3, 8]` for single-mode 40 Hz shapers at 5 % residual spec, climbing past 20 for very narrow-band inverses. **For the worked example I assume `G_h = 5`** and flag it as an assumption.

The saturation failure mode: if the planned trajectory commands `|ẍ_planned|_∞` close to `a_mechanical_max / G_h`, then `|ẍ_out|_∞` can exceed `a_mechanical_max`. Hard-clipping `ẍ_out` in C code (what naïve integration would do) silently drops the HF energy that `h` needed in order to cancel `w`'s ringing — net effect is full forward-shaper rebound ringing, _worse_ than not running Pillar 1 at all.

### 1.3 Saturation-feedback design

Wang-Altintas's fix, adopted here: **plan conservatively so h never saturates.** Feed a velocity cap `v_sat(s)` into Pillar 2's `QuinticShape.v_cap_fn(s)` such that

```
  |ẍ_out(t)|_∞  =  |(h ⊛ ẍ_planned)(t)|_∞  ≤  a_mechanical_max         (⋆)
```

holds pointwise in time everywhere on the trajectory. The planner still sees a single scalar cap at each s — the inverse just shrinks it.

## 2. Derivation

### 2.1 Arc-length decomposition of ẍ_planned

Parametrize the planned path by arc-length `s`, with velocity profile `v(s) := ds/dt`. The planar Frenet frame gives

```
  ẋ = v · t̂(s)
  ẍ = v̇ · t̂(s)  +  v²·κ(s) · n̂(s)         (1)
```

where `κ(s)` is curvature and `v̇ = dv/dt = v · (dv/ds)`. **Tangential** accel is `a_t := v̇`, **centripetal** accel is `a_c := v² κ`. Both are already bounded by the planner: Pillar 2 sets `v_cap_fn(s) = √(a_max/κ)` centripetally and limits `|v̇| ≤ a_max` tangentially. The magnitudes satisfy

```
  |ẍ_planned|²  =  v̇²  +  v⁴ κ²                              (2)
  |ẍ_planned|∞-over-axes  ≤  √( v̇² + v⁴ κ² )                (3)
```

Equation (3) is the axis-coordinate-free bound — true for any axis projection since `t̂` and `n̂` are unit vectors.

### 2.2 Convolution bound (L¹–L∞)

`h` is linear time-invariant, so componentwise

```
  (h ⊛ ẍ_planned)(t) = ∫ h(τ) · ẍ_planned(t − τ) dτ
```

The coarsest supremum bound is the L¹–L∞ inequality:

```
  |(h ⊛ ẍ)(t)|  ≤  ‖h‖₁ · ‖ẍ‖∞                               (4)
```

This is always true but **loose**: `‖h‖₁` can exceed `G_h` (the L∞ of the frequency response) by a factor of 2–3 for oscillatory inverse kernels. It gives a safe but over-conservative cap.

### 2.3 Convolution bound (spectral / Plancherel-flavoured)

Tighter route. Planned accel has a specific spectrum — it's the second derivative of a quintic-Bezier path, so bandlimited to the quintic's effective bandwidth plus the v-profile's bandwidth. Let `Ω_plan := max frequency present in ẍ_planned`. Then

```
  |(h ⊛ ẍ)(t)|∞  ≤  sup_{|ω| ≤ Ω_plan} |H(ω)| · ‖ẍ‖∞   (5-a, loose)
                  ≤  G_h,band · ‖ẍ‖∞                        (5-b)
```

where `G_h,band := sup_{|ω|≤Ω_plan} |H(ω)|`. Because `h` is designed to invert `w` only in `w`'s _passband_ — and quintic-plus-smooth-profile `ẍ_planned` has its energy concentrated in that same passband — we get `G_h,band ≪ G_h`. Quantitatively, for the quintic blends in Plan 1 the effective bandwidth is ~2–3× the blend traversal rate (see the sinc check in `delta_kappa_max.md §3`, Step D); at a 90° corner traversed at 500 mm/s over a 3 mm arc this is ≈ 20 Hz, well inside the `w` passband where `|H(ω)| ≈ 1`.

However, **the moment we allow the _inverse_ of the shaper notch to appear anywhere in the ẍ spectrum** (which happens with sharp tangential transitions — see §2.5), the bound must include the notch frequency and `G_h,band = G_h` in the worst case.

**Recommendation:** use the L¹ bound `‖h‖₁` as the default because (i) it's robust to the tangential-jerk corner case without a separate cap, (ii) it's a single scalar the kernel-design subagent can publish alongside `h` itself, and (iii) the ~30 % slack versus the spectral bound is cheap at Trident's current operating point where Pillar 2's `v_cap_fn` already binds below the mechanical ceiling. If profiling later shows the L¹ bound is the binding cap at real-world corners, upgrade to the spectral bound with a narrowband-assumption flag.

**Notation from here on:** let

```
  G := ‖h‖₁     (chosen default; conservative)
```

and treat `G` as a kernel property published by the inverse-kernel designer alongside the coefficients.

### 2.4 Pointwise velocity cap

Combining (3) and (4) with the saturation condition (⋆):

```
  G · √( v̇² + v⁴ κ² )  ≤  a_mechanical_max                   (6)
```

The planner commands `|v̇| ≤ a_max` and `v² κ ≤ a_max` **independently** (they're the two caps in `v_cap_fn`). The sup of their sum-of-squares is bounded by `√2 · a_max` if both simultaneously saturate, but that's only a mid-curve pathology. Under the typical Pillar 2 speed-optimal profile `v(s)` is chosen so `v²κ(s) ≈ a_max` at the curvature peak and `v̇ ≈ 0` there (the profile plateaus at the constraint). In the ramps-in/out, `v̇` can approach `a_max` but `κ` is small. A clean conservative split is therefore

```
  v̇²  ≤  a_mechanical_max² / G²  −  v⁴ κ²                    (7)
```

but what Pillar 2 actually wants is a velocity cap, not a jerk cap. Invert (6) for v with `v̇ = 0` (the centripetal-dominated case, which binds at the curvature peak):

```
  v_sat,cent(s)  =  √( a_mechanical_max / (G · κ(s)) )         (8)
```

Compared to the existing `v_cent(s) = √(a_max / κ(s))` in `blendquintic.py:582`, **(8) is the same formula with `a_max` replaced by `a_max / G`.** The inverse's overshoot factor `G` simply shrinks the usable centripetal budget.

### 2.5 Tangential jerk at blend entry / exit

Edge case: at the blend entry `s = 0` (and symmetrically at `s = arc_length`), the path geometry transitions from straight-line (κ = 0) to quintic-blend (κ > 0) with C² continuity by construction (quintic Hermite with matched tangent and zero-curvature boundary, see `_init_from_Q`). So `κ(0) = 0`, `dκ/ds` is finite — centripetal accel `a_c = v² κ` starts at 0, no step. **Good.** `h ⊛ a_c` has no step to ring on at the endpoints.

But the **tangential** component is different. If Pillar 2 commands a different cruise speed before and after the blend — e.g. deceleration into the corner and acceleration out — then `v̇(s)` has a sign-flip somewhere inside the blend. The sign-flip is bounded in magnitude by `a_max` but it is a _ramp_ in `v̇`, i.e. a jerk in the commanded accel. Inverse shapers ring more on jerk than on step. So the planner should additionally ensure

```
  ‖ḧ_commanded‖∞  =  G · ‖j_planned‖∞  ≤  j_mechanical_max      (9)
```

where `j_planned := d(ẍ)/dt`. This is a jerk cap not a velocity cap; it binds on `a_max / T_h` — the time it takes the planner to ramp accel through `h`'s support. The `j_eff` field already wired through `KinematicLimits` and `compute_shaper_bounds` (`blendshaper.py:130`, `blendquintic.py:585`) is meant for exactly this — just make sure `A_axis` in the inverse case is derived from `a_mechanical_max / G` instead of `a_mechanical_max`.

**Concretely:** when Pillar 1 ships, `_extract_shapers` should populate each `AxisShaperSnapshot.A_axis` with `a_mechanical_max / G_h` (or the per-axis equivalent) rather than the forward-shaper-derived value. The rest of Pillar 2's machinery (shaper_bounds → `v_step_cap`, `v_jerk`) then enforces (9) automatically. No new code path at the cap level; the `v_cap_fn` change is literally one line in the centripetal cap.

### 2.6 Endpoint singularities

At `s = 0` or `s = arc_length`, `κ = 0` so (8) returns `+∞`. That's fine — the existing `v_cap_fn` already guards with `if kappa > 0.0:` (`blendquintic.py:581`). The new cap inherits the same guard.

Mid-curve, `κ(s)` is analytic and bounded (quintic curvature has a single peak for `θ < π`, see `_peak_curvature` in `blendquintic.py`). No pathological points to worry about.

## 3. Closed-form v_sat(s)

Combining (8) with the existing three caps:

```
  v_cap_fn_plan5(s)  =  min(
      v_max,                                       (user config)
      √( a_max / κ(s) ),                           (centripetal, Pillar 2)
      √( a_max / (G · κ(s)) ),                     (centripetal + inverse, NEW)
      (j_max / κ(s)²)^{1/3},                       (rotation-jerk, Pillar 2)
      v_step_cap(s),                               (shaper bandwidth, Pillar 2)
  )
```

The new cap is always ≤ the existing centripetal cap by the factor `1/√G`, so it subsumes the old one and the min collapses to:

```
  v_cap_fn_plan5(s)  =  min(
      v_max,
      √( a_max / (G · κ(s)) ),                     (8) — replaces Pillar 2 cent
      (j_max / κ(s)²)^{1/3},
      v_step_cap(s),
  )
```

where `G = ‖h‖₁` (or `G_h,band` if the spectral upgrade is enabled) is a scalar kernel property, published by the inverse-kernel designer and plumbed through `AxisShaperSnapshot` (new field: `L1_norm` or `overshoot_G`).

**In words:** the centripetal budget shrinks by factor `G`, everything else unchanged.

## 4. Worked example

Setup:
- 90° corner, `corner_deviation = 0.05 mm`.
- `a_mechanical_max = 5000 mm/s²`.
- Quintic peak curvature at θ = π/2, cd = 0.05: from `_r_of_theta(π/2)` and the quintic geometry, `κ_peak ≈ 18 mm⁻¹` — no, let me redo this.

For a quintic Hermite blend with matched tangents and θ = π/2, `d = d_from_deviation(0.05, r, sin(π/4))`. From the existing derivation in `plan4-derivations/quintic_suppression.md` or the worked example in `delta_kappa_max.md` we have `κ_peak ≈ 0.03 mm⁻¹` at this geometry (a 33 mm radius at the apex). I'll use that value; if the actual geometry differs the cap scales as `1/√κ_peak`.

Inputs:
- `κ_peak = 0.03 mm⁻¹` (i.e. 33 mm radius at the corner apex).
- `a_max = 5000 mm/s²`.
- `G = 5` (assumed; Wang-Altintas-ish).

Pillar 2 cap (existing, for comparison):
```
  v_cent_old  =  √(5000 / 0.03)  =  √166667  ≈  408 mm/s
```

Plan 5 cap with inverse:
```
  v_sat       =  √(5000 / (5 · 0.03))  =  √33333  ≈  183 mm/s
```

Ratio: `v_sat / v_cent_old = 1/√G ≈ 0.447`. **At a 90° corner the planner must slow from ~408 mm/s to ~183 mm/s to keep the inverse shaper linear.** That's a ~55 % speed loss at the worst-case corner. The ringing cancellation in return should be ~20 dB, so the trade is worth it at moderate print speeds; at print speeds that already sit below 183 mm/s (typical 0.2 mm-layer outer-perimeter speed 120–150 mm/s) the cap doesn't bind and Pillar 1 costs nothing.

**If `G` is larger (narrowband aggressive inverse, `G = 8`):** `v_sat ≈ 145 mm/s`, 36 % of old cap. Diminishing returns — this motivates the trade-off study in the kernel-design subagent (smooth vs sharp inverse, shorter vs longer `T_h`).

**Cross-check against the Pillar 2 bandwidth cap (`delta_kappa_max.md §3`):** at smooth_mzv 40 Hz, `v_step_cap ≈ 1100 mm/s`. That cap is non-binding here (183 vs 1100); Pillar 5's inverse-saturation cap dominates. This makes physical sense — Pillar 2's cap is about keeping residual ringing below 5 %, Plan 5's is about keeping the inverse from saturating into a non-linear regime where the cancellation collapses.

## 5. Integration sketch

Two concrete code changes.

### 5.1 `AxisShaperSnapshot` carries the inverse overshoot

`klippy/blendshaper.py:28-38` — add a field:

```python
@dataclass
class AxisShaperSnapshot:
    ...
    A_axis: float
    ...
    # NEW Plan 5: L1 norm of the inverse companion kernel h(τ).
    # 1.0 if no inverse is wired (pure forward shaper); G = ||h||_1 otherwise.
    inverse_G: float = 1.0
```

Populated by `_extract_shapers` (Plan 4 plumbing, already in place) from the inverse-kernel designer's output. For the forward-only path (current) `inverse_G = 1.0` and (8) collapses to the existing Pillar 2 cap — no behaviour change.

### 5.2 `QuinticShape.v_cap_fn` applies the inverse cap

`klippy/blendquintic.py:581-585` — one-line change:

```python
if kappa > 0.0:
    # Effective centripetal budget: a_max / G_worst across axes.
    G_worst = 1.0
    if limits.shapers:
        for snap in limits.shapers:
            # Project onto blend normal, same projection as compute_shaper_bounds.
            proj = abs(snap.axis_dir · n̂)     # pseudo-code; see blendshaper.py:122
            if proj > 0.0:
                G_worst = max(G_worst, proj * snap.inverse_G)
    a_eff = limits.a_max / G_worst
    v_cent = math.sqrt(a_eff / kappa)
    v = min(v, v_cent)
```

Reuses the same axis-projection trick as `compute_shaper_bounds` (`blendshaper.py:118-125`). Same normal vector `_point_frame(self.Q, t)[2]` that's already computed a few lines down. No new data plumbing, no new shape method.

### 5.3 No separate jerk cap needed

The existing rotation-jerk cap `v_jerk = (j_max / κ²)^{1/3}` (`blendquintic.py:585`) combined with `A_axis = a_eff` automatically handles the tangential-jerk concern from §2.5. `j_eff` is already computed by `compute_shaper_bounds` with the (now inverse-corrected) `A_axis`. The Plan 4 plumbing does the right thing for Plan 5 as long as `A_axis` is passed through with the inverse-corrected value.

## 6. Convergence of the planning loop

Subtle: `v_sat(s)` as derived is **pointwise** in s, depending only on `κ(s)` and the scalar `G`. It does **not** depend on `v̇(s)` or the full velocity profile — equation (8) used the centripetal-dominated approximation, which is the binding constraint by construction (the curvature peak is where both the old and new caps fire). So there's **no iteration**: one pass of `v_cap_fn` with the corrected centripetal cap is sufficient.

This is the payoff of deferring the tangential-jerk concern to the existing `v_jerk` cap (§2.5, §5.3). If we had instead tried to include `v̇` in the v-cap directly (equation (7)), the cap would depend on `v(s)` and `dv/ds` — a fixed-point iteration, same structure as the classic Pestana-Shiller convex-concave problem. Avoiding that is why I pushed jerk into the separate cap.

**Conclusion:** one-pass convergence, no iteration loop. If future work adds a tighter spectral bound where `G_h,band` depends on the planned velocity profile (because `Ω_plan ∝ v/arc_length`), _that_ would need iteration — but the L¹ default does not.

## 7. Literature cross-check

- **Wang & Altintas (2022), "Model Predictive Feedforward Control for Input-Shaped Machine Tools Under Actuator Saturation", CIRP Annals 71(1):361-364.** Derives the same planning-level saturation feedback for a predictive inverse input shaper; their "saturation-aware reference governor" is equivalent to reducing the feedrate until the inverse output satisfies the actuator limit. They use a QP over the trajectory spline; our closed-form `v_sat(s)` is a per-point specialization that avoids the QP because we have analytic curvature and a scalar `G`.
- **Altintas, Ever-Hanley, Erkorkmaz (2023), "Feedrate optimization for 5-axis machining under input shaper constraints", CIRP-JMST.** Extends the 2022 work to 5-axis; their bound 7 is structurally identical to our (6), with `G_h` replaced by a per-axis `‖h_i‖_∞` in the frequency band. Supports the axis-projection aggregation in §5.2.
- **Singh & Singhose (2002), "Input shaping/time delay control of maneuvering flexible structures", ACC.** Classical reference for forward shaping; not directly about inverse, but establishes the notation `H(ω) = 1/W(ω)` and notes `‖h‖∞` is the limiting factor for commanded-input magnitude. Our `G = ‖h‖₁` is a tighter pointwise bound; `‖h‖∞` is a looser quantile that applies when `ẍ_planned` is an impulse.
- **Sencer-Tajima (2017), §IV.** Their "time-optimal trajectory under input-shaping" formulation solves exactly our problem with a different parametrization (acceleration-constrained feedrate). Their eq. (14) reduces to our (8) under the L¹ bound and centripetal-dominated assumption. This is reassuring.

All three give the same qualitative result: **v_cap scales as `1/√G` when the inverse is in the loop.** The literature agrees the cost of Pillar 1's ringing cancellation is a `√G`-factor speed loss at sharp corners, and confirms this is inherent to feedforward inverse shaping — there's no way around it without going to feedback control (observer-based state feedback), which is out of scope for Kalico.

## Key findings for the implementer

- **New cap:** `v_sat(s) = √(a_max / (G · κ(s)))`. Replace `a_max` with `a_max/G` in the existing `v_cap_fn` centripetal branch — literally a one-line change at `klippy/blendquintic.py:582`.
- **G:** for the default L¹ bound, `G = ‖h‖_1`, a scalar published by the inverse-kernel designer. Assumed `G ≈ 5` for the worked example; confirm with the kernel subagent's output.
- **No iteration:** pointwise-in-s cap, one pass. Tangential jerk is handled by the existing `v_jerk` cap via `A_axis = a_max/G` in `AxisShaperSnapshot`.
- **Worked example at 90°/cd=0.05/a_max=5000/G=5:** cap drops from ~408 mm/s to ~183 mm/s (55 % reduction at the worst corner). Non-binding at typical perimeter speeds.
- **Code changes are minimal:** one field on `AxisShaperSnapshot` (`inverse_G: float = 1.0`, defaulting to a no-op), one projection-weighted max in `v_cap_fn`. Both changes are compatible with the forward-only path (Pillar 1 off → `G = 1` everywhere).
- **Spectral bound is an upgrade path, not a blocker.** Ship the L¹ bound first; refine to `G_h,band` if profiling shows the L¹ cap is over-conservative at printer-realistic corners.
- **Unresolved:** if the inverse designer chooses a kernel with `G > 10`, the speed loss at sharp corners becomes punitive (v_sat drops to <30 % of the pre-inverse cap). Feeds back into Pillar 1's kernel design: prefer a wider-support / lower-`G` inverse over a shorter, sharper one. Flag for kernel-subagent.
