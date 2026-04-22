# Per-Axis Saturation Bound for the Feedforward Inverse Shaper

Derivation: opus math subagent, 2026-04-22. Branch `magnum-opus`, Plan 5 Pillar 1.

Resolves the disagreement between `saturation_feedback.md` §2.4 / spec D4 (the
`√2 · G · |proj|` form) and `REVIEW_3_NUMERIC.md` §V7 (the `G · (|proj_t| +
|proj_n|)` form).

Companion references:
- `docs/superpowers/plans/plan5-derivations/saturation_feedback.md` §2 — prior
  derivation that introduced the √2 factor.
- `docs/superpowers/plans/plan5-derivations/REVIEW_3_NUMERIC.md` §V1, §V7 —
  numerical rebuttal with Monte Carlo evidence.
- `docs/superpowers/specs/2026-04-22-plan5-direct-quintic-pillar1-design.md`
  §D4 lines 622–648 — current (incorrect) spec.
- `klippy/blendquintic.py::v_cap_fn` lines 569–595 — the code site that
  consumes this cap.

---

## 1. Problem setup

Per-axis mechanical ceiling: each Cartesian axis has an independent
acceleration limit `a_mech_max`. The feedforward inverse kernel `h(τ)` is a
scalar LTI filter applied componentwise to the commanded axis acceleration,
then followed by the forward shaper `w(τ)`; at the actuator we require

```
    | (h ⊛ ẍ_axis)(t) | ≤ a_mech_max     for all t, for each axis.       (⋆)
```

The planned path, in its Frenet frame, has

```
    ẍ(t) = v̇(t) · t̂  +  v²(t) κ(t) · n̂.                              (1)
```

Under the existing Pillar 2 caps the planner enforces, **independently**,

```
    | v̇(t) |      ≤ a_max,                                             (2a)
    | v²(t) κ(t) | ≤ a_max.                                             (2b)
```

These are two *scalar* constraints on two *orthogonal* Frenet components.
They can saturate simultaneously (nothing in the planner couples them).
Project onto a fixed world-axis unit vector `ê_axis`:

```
    proj_t := t̂ · ê_axis,
    proj_n := n̂ · ê_axis,
    ẍ_axis(t) = v̇(t) · proj_t  +  v²(t) κ(t) · proj_n.                 (3)
```

`(proj_t, proj_n)` is the 2-D Cartesian decomposition of `ê_axis` in the
Frenet basis, so `proj_t² + proj_n² = 1`.

Applying the standard L¹–L∞ convolution bound with `G := ‖h‖₁`,

```
    | (h ⊛ ẍ_axis)(t) |  ≤  G · sup_t | ẍ_axis(t) |.                    (4)
```

We need the tightest closed form for `sup_t |ẍ_axis(t)|` under (2) and (3),
then set it ≤ `a_mech_max / G` to get `v_sat`.

---

## 2. First-principles derivation

### 2.1 Worst-case per-axis accel magnitude

Treating `v̇` and `v²κ` as two **independent** scalars each constrained to
`[−a_max, +a_max]`, the supremum of `|ẍ_axis| = |v̇ · proj_t + v²κ · proj_n|`
is attained by choosing both signs aligned with their respective
coefficients:

```
    sup_{|v̇|,|v²κ| ≤ a_max}  |v̇ · proj_t + v²κ · proj_n|
        =  a_max · ( |proj_t| + |proj_n| ).                              (5)
```

This is the standard L¹-over-coefficients / L∞-over-variables dual. It is
tight — attained by `v̇ = a_max · sign(proj_t)` and
`v²κ = a_max · sign(proj_n)` simultaneously.

Combining with (4):

```
    sup_t | (h ⊛ ẍ_axis)(t) |  ≤  G · ( |proj_t| + |proj_n| ) · a_max.  (6)
```

### 2.2 Case-by-case check

At the curvature peak the planner typically has `v̇ ≈ 0` and
`v²κ ≈ a_max`, but `sup_t` in (4) runs over the entire kernel window
`[t − T_h/2, t + T_h/2]` — so the supremum "sees" the ramp-in / ramp-out
sections where `v̇` can climb to `±a_max`. Both caps *can* be simultaneously
active somewhere inside that window. This is what makes (5) the right
worst-case, not the Euclidean `√(v̇² + v⁴κ²)` of Frenet §2.1 of
`saturation_feedback.md`.

| Case                                 | `proj_t` | `proj_n` | `|ẍ_axis|` worst | `|h ⊛ ẍ_axis|` worst |
|:-------------------------------------|:--------:|:--------:|:----------------:|:--------------------:|
| 1. Tangent aligned (φ = 0)           |   1.0    |   0.0    |   a_max          |   G · a_max          |
| 2. 45° diagonal (φ = π/4)            |  1/√2    |  1/√2    |   √2 · a_max     |   √2 · G · a_max     |
| 3. Normal aligned (φ = π/2)          |   0.0    |   1.0    |   a_max          |   G · a_max          |

The worst case over corner orientation is Case 2, with factor `√2`.
The *isotropic* (orientation-unknown) bound is therefore `√2 · G · a_max`.
The *per-axis-orientation-aware* bound is `(|proj_t| + |proj_n|) · G · a_max`.

### 2.3 Per-axis mechanical cap

Setting (6) ≤ `a_mech_max` and solving for the centripetal-dominated velocity
cap (`v̇ → 0` on the constraint surface; `v²κ` is the actual binder at the
peak):

```
    v²(s) · κ(s)  ≤  a_eff(s)                                            (7)
    a_eff(s)      =  a_mech_max / [ G · (|proj_t(s)| + |proj_n(s)|) ].  (8)

    v_sat(s)      =  sqrt( a_eff(s) / κ(s) )
                  =  sqrt( a_mech_max / [ G · (|proj_t| + |proj_n|) · κ ] ).  (9)
```

Aggregating across both Cartesian axes (X and Y in 2-D), and possibly across
per-axis G (each axis has its own shaper),

```
    G_worst(s)   := max_{axis ∈ {X,Y}}  G_axis · (|proj_t,axis(s)| + |proj_n,axis(s)|).   (10)
    a_eff(s)      =  a_mech_max / G_worst(s).                                              (11)
    v_sat(s)      =  sqrt( a_mech_max / ( G_worst(s) · κ(s) ) ).                           (12)
```

In 2-D with tangent at angle `φ` from X-axis,

```
    proj_{t,X} = cos φ,   proj_{n,X} = −sin φ,   sum_X = |cos φ| + |sin φ|,
    proj_{t,Y} = sin φ,   proj_{n,Y} =  cos φ,   sum_Y = |sin φ| + |cos φ|.
```

`sum_X = sum_Y` always, so for isotropic `G_X = G_Y = G` the per-axis max
collapses to `G · (|cos φ| + |sin φ|)`, peaking at `√2 · G` for `φ = π/4`.

### 2.4 Why the spec's `√2 · G · |proj|` is wrong

Spec D4 writes `G_worst = max_axes |proj| · G_axis` and then multiplies by
`√2`. Interpreting "proj" as the blend normal projection `|proj_n|`, the
formula reports `√2 · |proj_n| · G_axis`, which gives:

| φ        | `|proj_t|+|proj_n|` (truth) | `√2 · |proj_n|` (spec) | verdict                  |
|---------:|:---------------------------:|:----------------------:|:-------------------------|
| 0        |        1.000                |        0.000           | **unsafe** (reports no cap) |
| π/4      |        √2 = 1.414           |        1.000           | **unsafe** (0.71× tight)  |
| π/2      |        1.000                |        √2 = 1.414      | over-conservative (1.41× loose) |

At `φ = 0` (tangent aligned with a world axis — a perfectly common corner,
e.g. any X-aligned polyline vertex), the spec formula **returns zero**,
meaning it reports `a_eff = ∞` and imposes **no** inverse-saturation cap —
but the true per-axis bound is `G · a_max` because `v̇` alone contributes
the entire ẍ_axis for that tangent-aligned axis.

---

## 3. Which formula is right

**Neither spec D4 nor the "safe fallback" (`√2·G_worst` without projection)
is a strict per-axis bound** — spec D4 is unsafe at tangent-aligned axes,
the safe fallback is loose by up to `√2` at axis-aligned geometry.

**The reviewer's formula is correct**: `(|proj_t| + |proj_n|) · G_axis` per
axis, aggregated via max across axes. Numerical verification (1M-sample
Monte Carlo, `/tmp/per_axis_deriv.py`) confirms:

```
 Case                                        sampled_worst  (|pt|+|pn|)  √2·|pn|
 Case 1  (axis ∥ tangent,  φ = 0)                  1.0000       1.0000    0.0000
 Case 2  (45° diagonal,    φ = π/4)                1.4130       1.4142    1.0000
 Case 3  (axis ∥ normal,   φ = π/2)                1.0000       1.0000    1.4142
 Bonus   (φ = 30°)                                 1.3650       1.3660    0.7071
```

The sampled worst matches `(|proj_t| + |proj_n|)` to within Monte Carlo
noise across every orientation. Spec D4's `√2 · |proj_n|` is tight **only**
at `φ = π/2`.

### 3.1 Ranking

```
 Formula                                         Correct?                  Tightness
 ------------------------------------------      -------------             ---------
 Spec D4: √2 · |proj_n| · G                      UNSAFE at |proj_n|→0      variable
 Safe fallback: √2 · G (no projection)           SAFE                      loose by up to √2 at axis-aligned
 Reviewer: (|proj_t|+|proj_n|) · G               SAFE, TIGHT               exact per-axis
```

Only the reviewer's formula is simultaneously safe and per-axis tight.

---

## 4. Numerical verification

Setup: `a_max = 5000 mm/s²`, `κ = 0.03 mm⁻¹`, `G = 2.0`.

| φ      | `proj_t` | `proj_n` | spec D4 v_cap | safe √2·G | reviewer v_cap |
|:------:|:--------:|:--------:|:-------------:|:---------:|:--------------:|
|  0°    |  1.000   |  0.000   | **+∞** (unsafe) | 242.75 mm/s | 288.68 mm/s |
| 15°    |  0.966   |  0.259   |   477.15      | 242.75     | 260.85        |
| 30°    |  0.866   |  0.500   |   343.29      | 242.75     | 246.99        |
| 45°    |  0.707   |  0.707   |   288.68      | 242.75     | 242.75        |
| 60°    |  0.500   |  0.866   |   260.85      | 242.75     | 246.99        |
| 75°    |  0.259   |  0.966   |   246.99      | 242.75     | 260.85        |
| 90°    |  0.000   |  1.000   |   242.75      | 242.75     | 288.68        |

Deltas:
- **At φ = 0°** spec D4 is catastrophically wrong — reports no cap; truth
  says 288.68 mm/s. If the planner honors the spec, it cruises at
  `v_max` through the corner and commands `|h ⊛ ẍ_X|` = `G · a_max = 2 ·
  5000 = 10 000 mm/s²`, double the mechanical limit. Hard-saturates the
  actuator, loses the inverse cancellation, and rings at the full
  forward-shaper amplitude.
- **At φ = 15°** spec D4 over-reports by ~83 % (477 vs 261 mm/s). The
  reported cap is `477 mm/s`; the true safe cap is `261 mm/s`. Planner
  would run ~1.83× too fast → `(v_actual/v_true)² = 3.3×` over-amplitude.
- **At φ = 45°** spec D4 and reviewer agree coincidentally (both give the
  global worst √2 factor).
- **At φ = 90°** spec D4 is *over*-conservative by factor √2 (242.75 vs
  288.68 mm/s). Loses ~16 % throughput without safety benefit.

The safe fallback (`√2 · G` with no projection) loses throughput at
tangent- or normal-aligned corners (always 242.75 mm/s regardless of
orientation), where the true bound allows 288.68 mm/s — a 19 % speed
penalty on axis-aligned corners (common on rectilinear infill).

### 4.1 Impact on spec D4 worked example

For the B-spline variants table with `G ∈ {1.92, 1.92, 2.00, 1.99, 1.95}`
at `pb_max = 0.3·f_sh`, worst `G = 2.003` (bs3). At κ = 0.03 mm⁻¹ and
a_max = 5000:

| formula                              | v_cap at shoulder (mm/s) |
|:------------------------------------:|:------------------------:|
| no inverse (baseline)                |  408.25                  |
| G alone, axis-aligned (φ=0 or π/2)   |  288.46                  |
| G · (|pt|+|pn|) at φ=45°             |  242.56                  |
| spec D4 `√2·G·|proj_n|` at φ=0       |  **+∞ (unsafe)**         |
| safe fallback `√2·G`                 |  242.56 (same as reviewer at 45°) |

The reviewer's correct formula is **orientation-dependent**: `v_cap`
varies from 242.6 mm/s (at 45°) up to 288.5 mm/s (axis-aligned). Spec D4
either reports `+∞` (unsafe) or over-conservatively 242.6 mm/s depending
on the blend-normal orientation — net effect is a grab-bag of under- and
over-restrictions.

---

## 5. Recommended spec fix

Replace spec D4 formula (currently at
`docs/superpowers/specs/2026-04-22-plan5-direct-quintic-pillar1-design.md:622-648`)
with the following.

### 5.1 Corrected v_cap formula

```
   v_cap_fn_plan5(s) = min(
       v_max,
       sqrt( a_max / ( G_worst(s) · κ(s) ) ),    # Pillar 1 saturation cap
       (j_max / κ(s)²)^(1/3),                     # rotation-jerk cap
       v_step_cap(s),                             # shaper-bandwidth cap
   )

   G_worst(s) = max_{axis ∈ enabled}  G_axis · (|proj_t,axis(s)| + |proj_n,axis(s)|)

   proj_t,axis(s) := t̂(s) · ê_axis
   proj_n,axis(s) := n̂(s) · ê_axis
```

No `√2` factor. The axis-projection `(|proj_t| + |proj_n|)` already carries
the "both Frenet components saturate" case tightly; when the axis is
45°-rotated relative to the tangent the factor reaches its maximum `√2`
naturally.

### 5.2 Why the √2 factor went away (resolves the prior math-subagent note)

`saturation_feedback.md` §2.4 derived the √2 from the Euclidean bound
`|ẍ| ≤ √(v̇² + v⁴κ²) ≤ √2 · a_max` on the **magnitude** of the 2-D
Frenet-decomposed accel vector. But the per-*axis* mechanical limit is
not a bound on the Euclidean magnitude — it is a bound on each Cartesian
component separately. Since the L¹–L∞ bound (4) is already componentwise
and `|ẍ_axis| = |v̇ · proj_t + v²κ · proj_n|` is a 1-D scalar, the
correct bound is (5), the L¹-sum over the *two scalar* components.

The `√2` *is* the correct coefficient when you want a single
orientation-free scalar bound ("what's the worst factor I might pay
for any corner orientation?"). But once you have the orientation (you
do — `t̂(s)` and `n̂(s)` are known at every s in the blend), you should
use the orientation-specific `(|proj_t| + |proj_n|)` which is tighter
everywhere and tight-equal only at 45°.

### 5.3 Code integration

In `klippy/blendquintic.py::v_cap_fn` (line 582) and
`klippy/blendshaper.py::compute_shaper_bounds` (line 122):

```python
# At s (or equivalently t via _s_to_t_refined), get Frenet frame.
_, tan, nrm = _point_frame(self.Q, t)

G_worst = 1.0
if limits.shapers:
    for snap in limits.shapers:
        e_axis = snap.axis_dir          # unit vector for this shaper's axis
        proj_t = abs(tan[0]*e_axis[0] + tan[1]*e_axis[1] + tan[2]*e_axis[2])
        proj_n = abs(nrm[0]*e_axis[0] + nrm[1]*e_axis[1] + nrm[2]*e_axis[2])
        factor = snap.inverse_G * (proj_t + proj_n)
        if factor > G_worst:
            G_worst = factor

a_eff = limits.a_max / G_worst
v_cent = math.sqrt(a_eff / kappa)   # no √2
```

Two projections per shaper per s-query (four adds, four muls, two abs),
cheap compared to the existing curvature evaluation.

### 5.4 Answers to the spec's side questions

**Q4: Where does `√2` come from?** From the Euclidean magnitude of
`ẍ_frenet = (v̇, v²κ)` under simultaneous saturation: `|ẍ|₂ ≤
√(a_max² + a_max²) = √2·a_max`. That's a bound on the 2-norm of the
Frenet 2-vector, not on a 1-D axis projection. Using it for per-axis is
a category error — the right per-axis bound is the L¹ dual, which gives
`(|proj_t|+|proj_n|) · a_max`.

**Q5: Where does `|proj_t|+|proj_n|` come from?** Standard L¹-L∞ duality:
with two independent scalar constraints `|c_i| ≤ a_max` and a linear
form `Σ α_i c_i`, the supremum is `Σ |α_i| · a_max`. Here the α_i are
the Frenet projections of the axis vector.

**Q6: Are `v̇` and `v²κ` independent in the Kalico planner?** Yes.
`blendquintic.py::v_cap_fn` lines 579–595 show the centripetal cap
(`v_cent = sqrt(a_max/κ)`) applied independently of any tangential
cap, and there is no coupling constraint like `v̇² + (v²κ)² ≤ a_max²`
anywhere. `v̇ ≤ a_max` is enforced by lookahead acceleration limiting
in `toolhead.py`; `v²κ ≤ a_max` is enforced here. Two independent 1-D
caps → (5) is exactly the right worst-case model.

**Q7: Does the formula depend on `h`'s polarity?** No — the L¹ bound
(4) is used both for positive and sign-indefinite kernels. `G = ‖h‖₁`
absorbs any overshoot pattern. A smooth non-negative `h` has `‖h‖₁ =
∫h = 1` (unit-norm) and `G = 1`, so the inverse cap collapses into
the existing centripetal cap. An oscillatory inverse has `G > 1` and
the cap shrinks. The derivation's validity is orthogonal to sign
pattern.

### 5.5 Numbers to update elsewhere

- `saturation_feedback.md` §2.4 eq. (8): remove the `√2` from the
  revision note at the top of the file. The current note says "Spec D4
  carries the corrected formula" — the new correct formula does *not*
  have `√2`; it has `(|proj_t|+|proj_n|)`.
- `REVIEW_2_MATH.md` §A: annotate that the √2 is orientation-free but
  superseded by the per-axis projection formula.
- Spec D4 line 624: change to `sqrt( a_max / ( G_worst · κ(s) ) )`.
- Spec D4 line 638: change to `G_worst = max_axes G_axis · (|proj_t| +
  |proj_n|)`.
- Spec D7 line 802: already lacks the √2 (currently flagged as
  inconsistent in REVIEW_3); after this fix it becomes consistent.
- `unified_v_of_s.md §8` worked example: TOPP numbers were regenerated
  at `√2·G = 2.83` per REVIEW_3_NUMERIC.md §V5. Under the corrected
  formula, `G_worst` at the curvature peak depends on blend
  orientation; for a 45°-oriented corner it's 2.83 (unchanged), for an
  axis-aligned corner it's 2.003 (looser). The T_opt numbers become
  orientation-dependent.

### 5.6 Risks / caveats

- **Blend orientation matters.** Under the corrected formula, a
  polyline vertex aligned with a Cartesian axis takes less speed
  penalty than a 45°-rotated vertex. That's physically correct (the
  axis-aligned case doesn't mix v̇ and v²κ into the same actuator) but
  it means the TOPP cost varies by orientation. Expected throughput
  gain over the "safe fallback" is ~10–19 % on rectilinear infill,
  zero at 45° corners.
- **Assumes `v̇` and `v²κ` can independently hit `a_max` within one
  kernel window.** True at the ramp-in and ramp-out of the blend
  shoulder (where the curvature peak pulls `v²κ` to `a_max` and the
  tangential profile still has `|v̇| ≈ a_max` from decel/accel). If
  the planner ever coupled them (e.g. a joint `v̇² + v⁴κ² ≤ a_max²`
  cap), the formula could be loosened to the Euclidean version and
  the worst factor would drop from `|pt|+|pn|` to `√(pt²+pn²) = 1`.
  Not currently the case in Kalico.
- **Per-axis G can differ.** If X and Y ship different shapers with
  different `‖h‖₁`, take the max in (10). The current Kalico path has
  one shaper per axis already (`blendshaper.py::AxisShaperSnapshot`
  is per-axis), so this is free.
