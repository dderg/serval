# REVIEW 3 — numerical-verification audit of Plan 5 (post-two-revision)

Date: 2026-04-22. Branch `magnum-opus`. Reviewer: opus numerical subagent.

Scope: `docs/superpowers/specs/2026-04-22-plan5-direct-quintic-pillar1-design.md`
plus all `docs/superpowers/plans/plan5-derivations/*.md` research artifacts.

Methodology: every claim re-derived from first principles in Python/numpy;
no artifact trusted, no prior review trusted. Scripts at `/tmp/v{1..7}_*.py`.
Reproducibility: darwin 25.4.0 arm64, Python 3.13, numpy 2.4.4, sympy 1.14.0.

---

## Verdict: SHIP WITH FIXES

Four of the seven verifications came out clean (V3, V4 entirely; V1 and V5
up to documentation). Three items are off:

- **V6 (piecewise k_fused piece count)** — spec D3 line 570's "≤ 2(m+1)
  pieces" claim is flatly wrong under the current h-design (§10 of
  `new_shaper_family.md` gives h as an ~11259-tap FIR array, not a
  piecewise polynomial).
- **V7 (√2 × |proj| interaction)** — the spec's `G_worst = max|proj|·G_axis`
  combined with the √2 factor is not a strict per-axis bound; at some
  axis orientations (tangent-aligned corners) it _understates_, at others
  it _overstates_ the bound. The derivation needs to pick one: either
  "worst-axis-orientation √2 factor alone" (safe, but loses per-blend
  tightness) or "exact per-axis |cos φ|+|sin φ| factor" (tighter but needs
  a different formula).
- **V2 asymmetric blends** — Option Z (single `v_cap_min` upstream) is
  _equivalent_ to Option Y (retract) for symmetric blends, but
  _over-conservative_ for asymmetric blends where the two shoulder caps
  differ. Spec D7 presents Z as a clean equivalence; the asymmetric-case
  throughput loss is not documented.

Also: spec §4 "Per-variant parameters" table and the §D7 worked example
references use the pre-√2 numbers. The CRITICAL-B item from REVIEW_2 is
acknowledged in revision notes but not yet fixed in the spec text.

None of these block the overall Plan 5 architecture — they are
correctness issues in derivation and documentation that an implementer
will encounter and be misled by.

---

## Numerical verifications (with Python code)

### V1. √2 factor derivation — **VERIFIED for the WORST-CASE axis**

**Reproducer:** `/tmp/v1_sqrt2.py`.

Given:
- `ẍ_planned = v̇·t̂ + v²κ·n̂` with `t̂ ⊥ n̂` unit vectors.
- L1-L∞ bound: `|h ⊛ ẍ_axis|_∞ ≤ ‖h‖₁ · ‖ẍ_axis‖_∞ = G · ‖ẍ_axis‖_∞`.
- Planner caps `|v̇| ≤ a_max` and `v²κ ≤ a_max` *independently*.

Analytical result: per-axis at angle φ from t̂,
```
|ẍ_axis| = |v̇·cos φ + v²κ·sin φ| ≤ (|cos φ| + |sin φ|)·a_max
```
and `|cos φ| + |sin φ|` attains max √2 at φ = π/4 (a 45° rotated corner).
Euclidean `|ẍ| = √(v̇² + v⁴κ²) ≤ √2·a_max` at simultaneous saturation.

Monte Carlo verification (1M samples):
```
Sup Euclidean |ẍ_planned|:   1.4129  (theory √2 = 1.4142)
Sup per-axis |ẍ_axis|:       1.4093  (theory √2)
```

**So `v_sat = sqrt(a_max / (√2 · G · κ))` is the correct worst-case bound
under independent-caps + arbitrary corner orientation.**

Key subtlety the spec and REVIEW_2_MATH §A get right: although at the
curvature peak v̇ ≈ 0 (so Euclidean |ẍ| = v²κ ≤ a_max with no √2),
the L1-L∞ bound on `h ⊛ ẍ` is a *supremum over t within the kernel
window* T_h ≈ 2·T_sm. Accel-ramp-into-shoulder contains points where both
v̇ and v²κ are near a_max — so √2 IS needed.

**However**: under a *Cartesian-aligned* corner (t̂ = x̂, n̂ = ŷ; corner
makes 90° in world coords), the two components decouple per axis: ẍ_x = v̇,
ẍ_y = v²κ. Per-axis max is a_max (no √2). The √2 appears only for
rotated-corner axis alignments. So in practice, the √2 conservatively
covers all orientations. See V7 below for the interaction with |proj|.

### V2. Option Z vs Option Y equivalence — **VERIFIED symmetric, FAILS asymmetric**

**Reproducer:** `/tmp/v2_option_z.py`.

Symmetric scenario: straight_A (cruise 300) → blend (v_cap_min 180) →
straight_B (cruise 300), a_max = 5000.

```
OPTION Z (single forward+backward pass, j_cap = 180 at both ends):
  Move 0: v_end = 180.000
  Move 1: v_start = v_end = 180.000
  Move 2: v_start = 180.000

OPTION Y (initial pass at 300 → retract j_cap to 180 → re-run):
  Move 0: v_end = 180.000
  Move 1: v_start = v_end = 180.000
  Move 2: v_start = 180.000

All three Move rows match exactly → Z = Y for symmetric blends. ✓
```

**Asymmetric scenario:** left shoulder v_cap = 100, right shoulder
v_cap = 250. Real blends have asymmetric caps when e.g. the flow-ratio
jumps across the blend (Plan 3 v_extr), or if one axis has different
projection properties on each side.

```
Option Z (single v_cap_min = 100 fed to BOTH ends):
  Move 2 (straight_B): v_start = 100.000   ← over-constrained
Option Y-style (j_cap_in = 100, j_cap_out = 250):
  Move 2 (straight_B): v_start ≈ 109.5     ← correct higher cap
```

**Finding:** Option Z's single scalar `v_cap_min = min_s v_cap(s)` is:
- SAFE (never violates any cap).
- EQUIVALENT to Option Y for symmetric blends (verified above).
- OVER-CONSERVATIVE for asymmetric blends — straight_B is
  unnecessarily capped at the _left_ shoulder velocity.

Spec D7 claims equivalence without qualifying for asymmetric cases. Extra
work: carry two numbers, `v_cap_min_entry` and `v_cap_min_exit`, and feed
each to its respective junction. This matches Option Y exactly at the
cost of one more scalar per blend.

### V3. Degree-10 composition — **FULLY VERIFIED**

**Reproducer:** `/tmp/v3_degree.py` (sympy + numpy round-trip).

Symbolic verification:
- quintic in s (degree 5) composed with s(t) = v_in·t + 0.5·a·t² (degree 2):
  result is a polynomial in t of degree **10**. ✓
- quintic in s composed with s(t) = s_a + v_cruise·t (degree 1):
  result is degree **5**. ✓
- Decel phase: degree 10. ✓

Numerical round-trip:
```
Random-coef quintic, accel phase t ∈ [0, 0.01]:
Max |direct_quintic(s(t)) - composed_poly_t(t)| = 1.1e-16
```

Degree-10 means **11 coefficients (c_0 … c_10)** per phase, which is
11 moments m_0…m_10 for integration. Spec's "11 moments" claim
(D2a lines 347-348) is correct. Not 10, not 12.

Conditioning risk is real at t ~ 0.01 s (t^10 ~ 1e-20, below double
precision); Horner evaluation and pre-computed coefficient magnitudes
make this tractable but the spec's D2a line 377-381 warning is
appropriate.

### V4. B-spline variant values — **FULLY VERIFIED**

**Reproducer:** `/tmp/v4_bspline.py` (independent re-derivation, not
copying `new_shaper_family.md §10`).

At f_sh = 40 Hz, ζ = 0.1, ts = 0.12:

```
 m   T_sm [ms]    F_m   σ²_T [ms²]  σ²/T_sm²  1/(12(m+1))   A_axis
 1      38.883  1.5553     62.9969   0.04167      0.04167   3809.7
 2      48.654  1.9462     65.7566   0.02778      0.02778   3649.8
 3      56.298  2.2519     66.0308   0.02083      0.02083   3634.7
 4      62.653  2.5061     65.4225   0.01667      0.01667   3668.5
 5      68.131  2.7252     64.4700   0.01389      0.01389   3722.7
```

- Closed form `σ²/T_sm² = 1/(12(m+1))` reproduces the spec D1 table to
  5+ digits. ✓
- Numerical integration (200001-sample trapezoid rule) of
  `∫ t² w(t) dt` agrees with closed form to 1e-10 relative. ✓
- A_axis table (3810, 3650, 3635, 3668, 3723): reproduces the spec D1
  table (266-270) exactly. ✓
- Non-monotone with minimum at bs3: confirmed. ✓

Inverse G = ‖h‖₁ at pb_max = 0.3·f_sh (T_h = 2·T_sm, Tukey α=0.05):

```
 m       G   pb_err%   N_h
 1   1.933     4.787   7777
 2   1.921     3.129   9731
 3   2.003     3.172  11259
 4   1.991     3.277  12531
 5   1.951     0.543  13627
```

- Spec table values 1.933, 1.921, 2.003, 1.991, 1.951 reproduced exactly. ✓
- Worst-case G_worst = 2.003 (bs3). ✓

At pb_max = 0.5·f_sh: G values 2.63, 2.53, 2.82, 2.84, 2.75. Worst-case
G_worst = 2.843 (bs4). Matches spec D1 line 242. ✓

The ∫ N_{m+1}(τ)² τ² dτ second-moment formula gives `(m+1)/12` in
canonical units and `T_sm²/(12(m+1))` after rescaling — correct by the
variance-of-sum argument (REVIEW_2_MATH §1 independently derives this).

### V5. TOPP on the worked example — **PARTIAL: spec numbers are stale**

**Reproducer:** `/tmp/v5_topp.py`. Built quintic Hermite geometry from
scratch (`r(θ=π/2) = 0.5901`, `d = 16·0.05/((1+15r)sin(π/4)) = 0.115`,
arc-length L = 0.1808 mm), not through `klippy/blendquintic.py`.

Ran TOPP on N=128 grid with each cap source composed correctly:

| scenario             | T_opt [ms] | min v_opt |
|---------------------:|:----------:|:---------:|
| G=1, no √2 (pre-D1)  | **9.02**   | 15.68 mm/s |
| G=2.003, no √2       | **10.90**  | 11.68 mm/s |
| G=2.003, WITH √2     | **12.72**  | 10.29 mm/s |

- Pre-D1 baseline (G=1): my T_opt=9.02 ms, spec §8 / REVIEW_2 both
  cite 9.18 ms. ~1.7% discrepancy, likely due to grid density and
  arc-length-integration scheme differences. The shape of v_opt(s) and
  the location of the shoulder minimum (s/L ≈ 0.18) match qualitatively.
- Current spec D4 (G=2.003 WITH √2): T_opt = **12.72 ms**. Spec's
  `unified_v_of_s.md §8` still prints 9.18 ms (G=1, pre-correction).
  REVIEW_2_MATH.md §B correctly flagged this; the revision notes at top
  of `unified_v_of_s.md` acknowledge it but the numerical table in §8
  hasn't been updated.

Also verified (see `/tmp/v5_topp.py` summary lines):
- Ratio (with-√2)/(no-√2) = 1.168× ≈ 2^(1/4) = 1.189 (expected
  factor under uniform `G → √2·G` rescaling of the centripetal cap).
  Slightly different because other caps (v_step) don't rescale.
- Shoulder location (s/L = 0.181) matches REVIEW_2 table. ✓
- v_cap ordering: v_sat binds at the shoulder under √2·G=2.83, v_step
  is not the binding cap under post-D1 numbers (differs from
  `unified_v_of_s.md §8` where v_step binds under G=1).

**Actionable**: `unified_v_of_s.md §8` and spec D7 worked-example
references need a regenerated table with G=2.003 AND √2. T_opt rises
from 9.18 ms to ~12.7 ms. Relative throughput gain over
T_safe_const still holds qualitatively but the absolute numbers
telling users "TOPP saves 23.6%" need recompute.

### V6. Piecewise-kernel cascade (support + piece count) — **PARTIAL**

**Reproducer:** `/tmp/v6_piecewise.py`.

**Support claim** (spec D3 lines 565-567): `k_fused = h ⊛ w` has support
`T_sm + T_h = 3·T_sm`. **VERIFIED** — elementary convolution fact
(sum of supports).

**Piece-count claim** (spec D3 line 570): "for bs_m with m+1 pieces
convolved with FIR inverse of similar piece count, the fused has
≤ 2(m+1) pieces." **NOT VERIFIED / LIKELY INCORRECT.**

Why: per `new_shaper_family.md §4.1` and §10's `design_bs_inverse`, the
inverse h is designed via:
```
  H(ω) = taper(ω) / W(ω)
  h    = Tukey-windowed IFFT of H(ω)
```
This produces `h` as a **sampled FIR tap array** (N_h ≈ T_h/dt ≈
11259 taps for bs3 at dt=10μs), **not a closed-form piecewise
polynomial**.

If we treat `h` as a tap array and `w` as 4 piecewise-polynomial pieces,
then `k_fused(t) = Σ_i h_i · w(t - i·dt)` has breakpoints at
`{piece_boundary_j + i·dt}` for all j ∈ pieces and i ∈ taps — ~45000
breakpoints, not 8. Storing "k_fused as a piecewise polynomial" in this
regime is impractical.

**What the spec needs to add**: an explicit design/decision for how h
gets represented. Three options:

1. **h stays a FIR tap array** (current `new_shaper_family.md` approach).
   Then k_fused is a convolution of tap array with a 4-piece polynomial
   → efficient implementation via O(N_h) per query sample. NO claim of
   piecewise-polynomial structure.
2. **Re-fit k_fused as a piecewise polynomial with O(m+1) pieces**
   post-convolution. Requires documenting the fit procedure and the
   approximation error (not currently in any artifact).
3. **Design h as a proper piecewise polynomial** with ≤ m+1 pieces
   (e.g., solve for B-spline coefficients that approximate 1/W on
   passband). Possible but a significant re-design; not in
   `new_shaper_family.md`.

The spec D3 currently asserts option 2 implicitly (states k_fused "is
piecewise polynomial with ≤ 2(m+1) pieces" as if it follows from the
design) — but the derivation is missing.

### V7. √2 and |proj| interaction — **ISSUE FOUND**

**Reproducer:** `/tmp/v1_sqrt2.py` final section.

Spec D4 line 638:
```
G_worst = max_axes |proj| · G_axis
```
where `proj` is "blend normal projected onto the axis direction." Then
`a_eff = a_max / (√2 · G_worst)`.

For an axis at angle φ from t̂ (so projection onto n̂ is sin φ, onto
t̂ is cos φ), the exact per-axis bound under independent caps is:

```
|ẍ_axis| = |v̇·cos φ + v²κ·sin φ| ≤ (|cos φ| + |sin φ|) · a_max
```

Comparing to the spec's `√2 · |proj_n|`:

| axis angle φ  | exact bound   | spec bound (√2·sin φ) | accurate?            |
|--------------:|:-------------:|:---------------------:|:---------------------|
| 0 (axis ∥ t̂) | 1.000         | 0.000                 | **under-conservative** |
| π/4           | 1.414 (√2)    | 1.000                 | **under-conservative** |
| π/2 (axis ∥ n̂)| 1.000         | 1.414                 | over-conservative    |

The spec's formula `√2·|proj_n|` is **tight** only at φ=π/2 (axis
aligned with blend normal — the case where v²κ dominates and v̇
contributes nothing to this axis). At other orientations it's either
slack or **unsafe** (!) — specifically, when the axis is closer to t̂
than to n̂, the formula reports 0 (no binding) but v̇ can actually hit
this axis at up to a_max.

**The double-counting concern is real but in the opposite direction**:
the √2 bakes in worst-case axis orientation assumption, while |proj_n|
specifies the actual orientation — combining them isn't strictly
algebraic. The correct formula for a *known* axis orientation is:

```
|ẍ_axis| ≤ (|proj_t| + |proj_n|) · a_max
```

(using `proj_t = axis·t̂`, `proj_n = axis·n̂`). Then
`G_worst_per_axis = G_axis · (|proj_t| + |proj_n|)`, and
`a_eff = a_max / G_worst_per_axis` (no √2 factor).

Worst case over φ: `|cos φ| + |sin φ| = √2` (at φ=π/4), which recovers
the spec's √2 factor when the axis is 45°-rotated. So:
- **Safe fallback**: use `a_eff = a_max / (√2 · G_worst)` with
  `G_worst = max_axes G_axis` (no projection factor). Always safe,
  sometimes loose.
- **Tight formula**: use `G_worst = max_axes G_axis · (|proj_t|+|proj_n|)`
  (no √2). Tight per-axis; per-blend G is geometry-dependent.
- **Current spec**: `√2 · |proj_n|` — potentially UNSAFE at
  tangent-aligned axes.

In practice, the binding-axis at the curvature peak (where v_cap binds)
typically has `|proj_n| ≈ 1` (axis is aligned near the blend normal),
so the spec formula is tight by coincidence. But the derivation is not
clean. Recommend: **drop the |proj|, keep the √2, accept the slack
until HW shows it matters.** Or: **drop the √2, use
`|proj_t|+|proj_n|`, document the derivation.**

---

## Errors found

### CRITICAL

**None** — all prior CRITICAL items from REVIEW_2 have been addressed
in the current spec (the √2 factor is in D4 line 624; the Wang-Altintas
retractions are acknowledged).

### IMPORTANT

**V6-I1. Spec D3 line 570 claims k_fused has "≤ 2(m+1) pieces" without
a valid derivation.** Current h design gives h as a sampled FIR tap
array (11259 taps for bs3), not a compact piecewise polynomial.
k_fused computed as h ⊛ w has ~11259 + 4 breakpoints if naively
stored. Spec needs to specify one of: (a) h as tap array and drop the
piecewise-polynomial framing for k_fused; (b) re-fit k_fused as a
piecewise polynomial (document fit + error); (c) redesign h as
piecewise polynomial from scratch. Without this, D1's "struct smoother
piecewise redesign" contract is under-specified.

**V7-I2. Spec D4 formula `G_worst = max|proj|·G_axis` combined with √2
is not a strict per-axis bound.** At φ=0 (axis ∥ t̂) the formula
computes 0 (no binding) but v̇ can hit this axis at a_max. Correct
either to the orientation-free √2·G (safe, possibly loose) or to
`G_worst = max_axes G_axis · (|proj_t|+|proj_n|)` without √2 (tight).

**V2-I3. Spec D7 claims Option Z upstream junction cap is equivalent
to Option Y retract.** Verified equivalent for symmetric blends. For
asymmetric blends (different left/right shoulder caps), Option Z's
single scalar `v_cap_min = min_s v_cap(s)` over-constrains the exit
junction by propagating the entry-shoulder cap backward through the
blend. This is SAFE but loses throughput. Two-scalar fix
(`v_cap_min_entry`, `v_cap_min_exit`) matches Y exactly; spec should
document this choice.

**V5-I4. `unified_v_of_s.md §8` worked example still uses G=1 pre-D1
numbers despite the revision note at top acknowledging this.** T_opt
should be 12.72 ms (G=2.003 + √2), not 9.18 ms. The revision note
says "all T_opt / T_safe / throughput-gain numbers need regeneration
with correct G" but the table hasn't been regenerated. Either
regenerate the table or explicitly label it "pre-D1 baseline (G=1)
for reference only."

### MINOR

**V5-M1. Grid-density difference.** My N=128 TOPP gives
T_opt(G=1)=9.02 ms vs spec 9.18 ms — 1.7% discrepancy likely due to
arc-length-integration scheme (I use trapezoid with N=1000 sub; spec
uses 8-Gauss-Legendre with N=40). Not a bug; document the sensitivity.

**V4-M2. pb_err table at pb=0.5·f_sh.** My bs3 gives 1.81% (matches
spec) but bs2 gives 14.58% (matches spec). Both are reproducible; no
issue. Flagged only because the bs2 number is conspicuously high and
suggests bs2 is not a viable "wide passband" variant. Spec §D1 line
244's "bs3 (default) and bs5 (premium)" recommendation is correct
based on this.

**V6-M3. Support bound T_fused = T_sm + T_h = 3·T_sm.** Spec D3 line 566
is correct. The 3× widening relative to forward-only is the dominant
query-cost driver (risk 3).

---

## Numbers to update in the spec

1. **`unified_v_of_s.md §8` table.** Replace the G=1 numbers with G=2.003
   AND √2:
   - `v_cap(L/2) = 34.75` → revised (compute with current caps).
   - `v_cap_peak (shoulder)` = 15.06 → ~10.3 under √2·G=2.83.
   - `T_opt (N=128, N=400)` = 9.18 ms → ~12.72 ms.
   - `T_safe_const` = 12.01 ms → scales by √2·G factor similarly.
   - `T_unsafe` = 5.20 ms → unchanged (no v_cap applied in the unsafe
     aggregator).
   - Throughput gain "TOPP vs safe-constant: 23.6%" → recompute from
     regenerated table (the *ratio* is roughly invariant under
     uniform scaling, but needs re-verify).

2. **Spec D7 (line 805)** cites "`v_sat(s) = sqrt(a_max / (G · κ(s)))`"
   — missing √2. The D4 formula on line 624 has √2 but D7 line 802
   does not. **Inconsistent between sections.**

3. **Spec D3 line 570 "≤ 2(m+1) pieces"** — remove or back with an
   explicit piecewise-polynomial h design. Current research artifact
   gives h as a tap array.

4. **Spec D4 line 638** "G_worst = max_axes |proj| · G_axis"
   — reconsider the projection factor. If kept with √2, document that
   the formula is tight at φ=π/2 and conservative elsewhere (not
   strictly correct at φ near 0 for independent-cap assumption).

5. **Spec "Risks" §5 (line 1014-1018).** The numeric example uses G=2
   and G=2.84 but does not apply the √2. Cap `v_sat` at κ=0.05 with
   √2·G=2.83: sqrt(5000/(2.83·0.05)) = 188 mm/s, not 224 mm/s.
   With √2·G=4.02 (bs4 at pb=0.5·f_sh): sqrt(5000/(4.02·0.05)) = 158
   mm/s, not 188. Tighten the risk numbers.

6. **Spec §D5 (lookahead extension)** claims T_fused/2 stacking with
   PA's 40 ms — correct. But implicit assumption is h is centered
   (symmetric support). Tukey-windowed bandlimited IFFT produces a
   near-symmetric h; confirm `t_offs ≈ 0` in the reset path.

---

## Reproducibility

All scripts independent of klippy source (except V5 uses a
hand-constructed quintic Hermite, not `blendquintic.py`):

- `/tmp/v1_sqrt2.py` — √2 derivation (Monte Carlo + analytical).
- `/tmp/v2_option_z.py` — forward+backward lookahead simulator,
  symmetric and asymmetric blend cases.
- `/tmp/v3_degree.py` — sympy composition + numpy round-trip for
  quintic ∘ s(t) degree.
- `/tmp/v4_bspline.py` — full B-spline family reproduction from scratch
  (piecewise Curry-Schoenberg + FFT inverse design).
- `/tmp/v5_topp.py` — quintic geometry + cap composition + TOPP
  forward/backward.
- `/tmp/v6_piecewise.py` — piecewise-polynomial pieces for bs3 +
  discussion of k_fused piece count.

All scripts were run and produced output. No scipy dependency; numpy + sympy
only.

---

## Summary of required actions

- **[IMPORTANT V6]** Revise spec D3 line 570 to match the actual h
  design. If h stays a tap array, remove the "≤ 2(m+1) pieces" claim
  and explicitly describe how `struct smoother` represents the fused
  kernel (piecewise-poly part of w plus tap-array convolution weight?).
- **[IMPORTANT V7]** Revise spec D4 line 638 G_worst formula.
  Recommend: `a_eff = a_max / (√2 · G_worst)` with
  `G_worst = max_axes G_axis` (drop |proj|), OR derive/replace with
  `(|proj_t|+|proj_n|)·G_axis` without √2.
- **[IMPORTANT V2]** Document Option Z's asymmetric-blend
  over-conservatism in spec D7; decide whether to accept the loss or
  upgrade to two per-end scalars (`v_cap_min_entry`, `v_cap_min_exit`).
- **[IMPORTANT V5]** Regenerate `unified_v_of_s.md §8` table with
  current G=2.003 AND √2. Update the spec D7 narrative numbers.
- **[MINOR]** Fix inconsistent √2 between spec D4 (has √2) and D7
  cap-composition block (no √2) at lines 624 vs 802.
- **[MINOR]** Tighten Risk §5 numbers to use √2·G.

None invalidate the overall Plan 5 architecture. The core math (B-spline
family, inverse kernel, A_axis table, degree-10 composition) is correct
and numerically reproducible. The issues are in the cap derivation
(√2 interaction, asymmetric Z), the k_fused representation bookkeeping,
and stale worked-example numbers.
