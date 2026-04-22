# REVIEW 2 — math-only adversarial audit of Plan 5 (post-revision)

Date: 2026-04-22. Branch `magnum-opus`. Reviewer: opus math subagent.

Scope: `docs/superpowers/specs/2026-04-22-plan5-direct-quintic-pillar1-design.md`
plus all `docs/superpowers/plans/plan5-derivations/*.md` research artifacts,
with first review of the new `unified_v_of_s.md` (Pillar 2b).

## Verdict: ship with fixes

The shaper family and inverse-kernel numerics (spec §D1, `new_shaper_family.md`)
are mathematically solid — every published number reproduces to 3-4 significant
figures and the closed-form `σ²_T = T_sm²/(12·(m+1))` derives correctly from first
principles. TOPP math in `unified_v_of_s.md` is largely correct as well.

BUT: three substantive errors in derivations that back the v_cap composition
must be fixed before implementation. One is a factor-of-√2 under-bound in the
saturation-feedback cap (safety-relevant). The second is a false monotonicity
claim. The third is an internally inconsistent trapezoid-fit worked example
that cannot be reproduced. None block the overall Plan 5 architecture but they
should be corrected in the spec/research files because implementers will
believe the derivation.

---

## Formulas re-derived from scratch

### 1. Cardinal B-spline second central moment: `σ²_T = T_sm² / (12(m+1))` — CONFIRMED

Derivation from scratch:

- Let `rect_{T_1}` be the unit-integral rectangle on `[-T_1/2, T_1/2]`. Its
  second central moment (variance) is
  `σ²_rect = ∫_{-T_1/2}^{T_1/2} t²·(1/T_1) dt = T_1²/12`.
- The cardinal B-spline of order `m` is the `(m+1)`-fold self-convolution of
  `rect_{T_1}`. Independent zero-mean random variables add variances, so
  variance of the `(m+1)`-fold convolution is `(m+1)·T_1²/12`.
- Total support is `(m+1)·T_1 = T_sm`, so `T_1 = T_sm/(m+1)`.
- Therefore `σ²_T = (m+1)·(T_sm/(m+1))²/12 = T_sm² / (12·(m+1))`. QED.

Formula in `new_shaper_family.md` eq. after §2 and spec §D1 line 253 both
match. The `σ²_T / T_sm²` table in spec §D1 (0.04167, 0.02778, 0.02083,
0.01667, 0.01389) is exactly `1/(12·(m+1))` for m=1..5 — reproduced to
5+ digits (verification script run 2026-04-22).

### 2. Forward kernel Fourier transform: `W(ω) = sinc^{m+1}(ω·T_1/2π)` — CONFIRMED

Derivation:

- `ℱ{rect_{T_1}}(ω) = sinc(ω·T_1/2π)` where `sinc(x) := sin(πx)/(πx)`.
- Fourier of an `(m+1)`-fold convolution = product of Fouriers = `sinc^{m+1}`.
- Zeros at `f = k/T_1` for k = 1, 2, ... → first zero at `f = 1/T_1 =
  (m+1)/T_sm`.

All five "first zero" values in `new_shaper_family.md §2.4` reproduce to 4
digits:

| m | first zero [Hz] (reproduced) | spec value |
|---|------------------------------|-----------|
| 1 | 51.436 | 51.44 |
| 2 | 61.660 | 61.66 |
| 3 | 71.050 | 71.05 |
| 4 | 79.805 | 79.81 |
| 5 | 88.066 | 88.07 |

All > f_sh = 40 Hz, so the invertibility precondition holds for all variants.

### 3. v_jerk = (j_eff/κ²)^(1/3) — CONFIRMED

Derivation via Frenet-Serret in 2D:
- `ẍ = v̇·t̂ + v²κ·n̂`
- `d(v²κ·n̂)/dt = ... + v²κ · dn̂/dt = ... + v²κ · (-κ·v·t̂)`
- Rotational jerk magnitude ≥ v³·κ² (the centripetal-rotation term).
- j_max ≥ v³·κ² → v ≤ (j_max/κ²)^(1/3). ✓

### 4. L1-L∞ bound — CONFIRMED as written but applied loosely; see Errors §A

The bound `|h ⊛ ẍ|_∞ ≤ ‖h‖_1 · ‖ẍ‖_∞` is a textbook Young's inequality (p=1,
q=∞). No issue with the bound itself. The issue is how the derivation uses it
to get `v_sat = √(a_max/(G·κ))` — see critical error A.

---

## Numerical claims verified

Reproduction script matches `new_shaper_family.md §10`. Output matches spec
§D1 tables to 3-4 sig figs everywhere:

### 5. Per-variant T_sm, F_m, A_axis (spec §D1 tables, lines 203-210 and 258-264)

| variant | T_sm reproduced [ms] | F_m | A_axis | spec match |
|---|---|---|---|---|
| bs1 | 38.883 | 1.5553 | 3809.7 | ✓ |
| bs2 | 48.654 | 1.9462 | 3649.8 | ✓ |
| bs3 | 56.298 | 2.2519 | 3634.7 | ✓ |
| bs4 | 62.653 | 2.5061 | 3668.5 | ✓ |
| bs5 | 68.131 | 2.7252 | 3722.7 | ✓ |

Closed-form `A_axis = 2·ts/σ²_T = 24·(m+1)·ts·f_sh²/F_m²` verified to 1e-10
against numerical `M2 - M1²` moment integration on 200001 samples.

Damped residual at (40 Hz, ζ=0.1) with these T_sm values: V = 0.050000
exactly for all five variants (bisection to 50 iterations).

### 6. Inverse kernel G, pb_err, HF_amp (spec §D1 inverse table, lines 231-236)

At pb_max = 0.3·f_sh = 12 Hz:

| variant | G (repro) | pb_err | HF_amp | spec |
|---|---|---|---|---|
| bs1 | 1.933 | 4.79% | 0.082 | 1.933 / 4.79% / 0.08 ✓ |
| bs2 | 1.921 | 3.13% | 0.048 | 1.921 / 3.13% / 0.05 ✓ |
| bs3 | 2.003 | 3.17% | 0.040 | 2.003 / 3.17% / 0.04 ✓ |
| bs4 | 1.991 | 3.28% | 0.035 | 1.991 / 3.28% / 0.04 ✓ |
| bs5 | 1.951 | 0.54% | 0.031 | 1.951 / 0.54% / 0.03 ✓ |

At pb_max = 0.5·f_sh = 20 Hz: G values 2.63, 2.53, 2.82, 2.84, 2.75 — all
reproduce to 3 digits. Worst-case G = 2.84 (bs4) confirmed.

### 7. Quintic curvature profile at 90°/cd=0.05 (spec implicit, unified_v_of_s.md §1.2)

Built QuinticShape by hand from `_r_of_theta(π/2) = 0.5900`,
`d = 16·0.05/((1+15·0.59)·sin(π/4)) = 0.1148 mm`, arc length L = 0.1808 mm.

Curvature samples:

| s/L | κ [mm⁻¹] (reproduced) |
|---|---|
| 0.00 | 0.000 |
| 0.18 | 16.671 (shoulder peak) |
| 0.25 | 13.114 |
| 0.50 | 4.141 (midpoint) |
| 0.75 | 13.114 |
| 0.82 | 16.671 (shoulder peak) |
| 1.00 | 0.000 |

Shoulder location s/L ≈ 0.182 confirmed (`_peak_curvature` finds peak at
t=0.1805, s=32.93 μm, s/L=0.182). Boundary curvatures κ(0)=κ(L)=0 confirmed
(endpoints are C² with matched zero-curvature by quintic Hermite
construction).

### 8. TOPP worked example (unified_v_of_s.md §8)

With G=1 (as implicitly used in §8.1 despite §3's G·κ in the v_sat formula):

| quantity | reproduced | spec §8 |
|---|---|---|
| v_cap(L/2) | 34.75 mm/s | 34.75 ✓ |
| v_cap_peak (shoulder) | 15.07 | 15.06 ✓ |
| T_opt (N=400) | 9.177 ms | 9.177 ✓ |
| T_safe_const | 12.007 ms | 12.01 ✓ |
| T_unsafe = L/v_cap(L/2) | 5.203 ms | 5.20 ✓ |

### 9. v_step values at sample points (unified_v_of_s.md §8.1 table)

| s/L | k | R | |n̂·x̂| | |n̂·ŷ| | v_step (min over axes) | spec |
|---|---|---|---|---|---|---|
| 0.25 | 13.114 | 0.0763 | 0.459 | 0.889 | 17.66 | 17.7 ✓ |
| 0.50 | 4.141 | 0.2415 | 0.707 | 0.707 | 35.23 | **69.5 ✗** |
| 0.82 | 16.67 | 0.0600 | 0.957 | 0.289 | 15.09 | (shoulder) |

The s/L=0.50 value **69.5 in spec is wrong by factor 2×**. Recomputed from
`sqrt(A_axis·R/|n·ê|) = sqrt(3635·0.2415/0.707) = 35.23`. I cannot reproduce
69.5 by any reasonable projection choice. This is a minor bug — the `min` in
v_cap at s/L=0.50 is dominated by v_cent(G=1)=34.75 not v_step, so the final
v_cap number in the spec table is still right.

---

## Errors found (prioritized)

### CRITICAL — A. Saturation cap under-bounds by √2 in the tangential+centripetal combined worst case

Location: `saturation_feedback.md §2.4 eq. 8`, spec §D4 line 541.

Derivation walk-through:
- `ẍ_axis(t) = v̇·t̂_axis + v²·κ·n̂_axis` (Frenet, eq. 1 of saturation_feedback.md)
- Planner commands `|v̇| ≤ a_max` and `v²κ ≤ a_max` *separately* (per §2.4).
- Worst-case per-axis magnitude (via triangle ineq): `|ẍ_axis| ≤ |v̇| + |v²κ| ≤ 2·a_max`.
- Tighter via Cauchy-Schwarz / ℓ²: `|ẍ_axis|² ≤ v̇² + (v²κ)² ≤ 2·a_max²`, so
  `|ẍ_axis| ≤ √2·a_max`.
- L¹-L∞ bound: `|(h ⊛ ẍ)_axis|_∞ ≤ G · ‖ẍ_axis‖_∞ ≤ G·√2·a_max`.

For the cascade not to saturate at `a_mech_max`:
`G·√2·a_max ≤ a_mech_max` ⇒ `a_max ≤ a_mech_max / (G·√2)`.

Spec uses `a_max = a_mech_max / G` (missing the √2 factor). Under the
simultaneous-saturation case — which happens during the accel ramp into the
curvature peak where both components are near their caps — the cascade
output exceeds `a_mech_max` by up to √2 ≈ 1.414.

§2.4 of saturation_feedback.md explicitly acknowledges this: "The sup of
their sum-of-squares is bounded by √2·a_max if both simultaneously
saturate, but that's only a mid-curve pathology." It then waves this off by
noting that at the curvature peak v̇≈0. That's true *at the peak*, but the
L¹-L∞ bound is a supremum over t, not a pointwise equality — the cascade
operator `h⊛·` mixes values of ẍ across a window of ≈T_h = 2·T_sm around t.
So saturation at any t within that window contaminates the bound.

Fix (minimal): use `v_sat(s) = √(a_max / (√2·G·κ(s)))`. Adds a `1/2^(1/4)` ≈
0.84 factor to v_cap at the curvature peak. The existing ~55% speed cut at
G=2 becomes ~62%. Honest and cheap.

Alternative: keep current cap if the planner enforces `|v̇|² + (v²κ)² ≤
a_max²` jointly (so single-axis `|ẍ|` stays ≤ a_max). This is what Pillar 2
currently approximates but without the joint constraint explicit.

Spec's §2.5 discussion of tangential jerk papers over this via the `v_jerk`
cap, but v_jerk bounds the *jerk* (time-derivative of ẍ), not the magnitude
of ẍ itself. Different beast.

**Action: add the √2 factor to v_sat in `saturation_feedback.md §3` eq. 8,
and to spec §D4 formula. Adjust the worked example in §4 accordingly.**

### CRITICAL — B. §8 worked example in `unified_v_of_s.md` uses G=1 despite derivation requiring G>1

Location: `unified_v_of_s.md §8.1 table` and onward.

The v_cap composition (eq. 3 of §3) says `v_sat(s) = sqrt(a_max/(G·κ))`.
For bs3 at pb_max=0.3·f_sh, G=2.003. But the §8.1 numerical table values
match *G=1* exactly:

- At s/L=0.25, k=13.101: table shows v_sat=19.5. Computed: sqrt(5000/13.101)
  = 19.54 — that's G=1. With G=2.003: sqrt(5000/(2.003·13.101)) = 13.81.
- At s/L=0.50, k=4.141: table shows v_sat=34.7 = sqrt(5000/4.141). G=1.

So the entire worked example silently drops the inverse-correction. With
G=2.003 correctly applied, T_opt would be ≈ 13 ms not 9.18 ms, a 40%
throughput loss worse than what §8.3 reports.

The 23.6% "TOPP vs safe-constant" win still holds in relative terms with any
G — the ratio `T_safe / T_opt` is invariant to a uniform G scaling of v_cap
because v_cap is then uniformly scaled. But the absolute T values in the
table are for the G=1 case, which is *old* Pillar 2 behavior, not post-D1.

**Action: re-do §8 with G=2.003 explicitly, and confirm the ratios still
hold. Or reframe §8 as "pre-D1 baseline" and show a separate post-D1 table
with G=2.0.**

### IMPORTANT — C. Spec §4.3 "trap-in-s is ~12% worse than TOPP" cannot be reproduced

Location: `unified_v_of_s.md §4.3 line 292-298`, and spec §D7 line 700.

Claim: "A single cruise at 17.57 gives T = 10.29 ms ... trapezoid-in-s (option a)
as the ship target."

My reproduction: under v_in=v_out=30, cruise at the shoulder minimum
v_cap_peak=15.06 gives **T=9.04 ms** (1.4% worse than T_opt=9.18 ms). Cruise
at 17.57 (violates v_cap at the shoulder, hence unsafe) gives T=8.53 ms.
Scanning cruise in {10,...,25}: I get T=10.08 ms only at cruise=10 mm/s,
which is below the shoulder minimum and nonsensical.

I cannot construct a cruise velocity that gives 10.29 ms under any physical
interpretation of the trapezoid-in-s. The spec's "12% worse than TOPP"
conclusion appears overstated — the actual gap is ~1.5%, which *strengthens*
the case for trap-in-s.

Possible source of the spec number: the author may have used a mid-optimal
(non-bang-bang) accel or derived T_fit under a different boundary model.
Either way, the specific number 10.29 ms as presented in §4.3 doesn't match
§4.3's own stated fitting rule.

**Action: re-compute T_fit from the stated rule ("accel_end_s = s_fwd,
cruise_v = min_s v_opt, decel_start_s = s_back") and correct either the
number or the rule.**

Also affected: spec line 700 ("single-cruise trapezoid-in-s is 12% slower than
TOPP optimal"). Update when §4.3 is corrected.

### IMPORTANT — D. Monotonicity claim (§3.4) false for v_step

Location: `unified_v_of_s.md §3.4 line 199-202`.

Claim: "All four κ-dependent caps are strictly increasing in R(s) = 1/κ(s)."

Counter-example for v_step: at the 90°/cd=0.05 blend, v_step has its minimum
at t=0.15 and t=0.85 with value 15.31 mm/s, but κ-peak (R-min) is at t=0.18
with value 16.37. They don't coincide because `v_step = sqrt(A_axis·R/|n̂·ê|)`
where `|n̂·ê|` varies with s as the tangent rotates. R(s) alone doesn't
determine v_step.

Scan confirming:

| t | κ | v_step_min |
|---|---|---|
| 0.15 | 15.85 | 15.31 |
| 0.18 | 16.37 (peak) | 15.35 |
| 0.20 | 16.37 | 15.35 |
| 0.25 | 13.63 | 17.25 |

Minimum of v_step is at t=0.15 (|n_y| large, R small) and t=0.85 (|n_x|
large, R small) — near but not at κ-peak.

**Consequence for TOPP:** none — TOPP doesn't need monotonicity. But the
spec's rhetorical claim should be corrected.

**Action: amend §3.4 to "v_sat, v_jerk are strictly increasing in R(s);
v_step minimum is typically near the curvature shoulders but not exactly at
κ-peak because the normal-axis projection varies."**

### IMPORTANT — E. "5.3× centripetal overshoot" mis-stated in §1.2

Location: `unified_v_of_s.md §1.2 line 74-77`, spec §D7 line 670.

Claim: "v_cap(L/2) = 34.75 mm/s but the shoulder minimum is 15.06 mm/s. ...
Centripetal accel there exceeds a_max by a factor of (34.75/15.06)² ≈ 5.3."

Audit:
- Shoulder `v_cap_peak = 15.06` is *v_step*, not *v_cent*. v_cent(G=1) at
  shoulder κ=16.67 is `sqrt(5000/16.67) = 17.32`.
- Actual centripetal a_c at shoulder with v=34.75: `v²·κ = 34.75²·16.67 =
  20137 mm/s²`.
- a_c / a_max = 20137/5000 = **4.03**, not 5.3.
- The ratio 5.3 = (34.75/15.06)² is the v² ratio against the *shaper-bandwidth*
  cap (v_step), which has nothing to do with centripetal accel.

So the phrase "centripetal accel exceeds a_max by factor 5.3" conflates
two different caps. The actual centripetal overshoot vs a_max is 4×,
still serious.

**Action: reword to "v² at shoulder exceeds v_cap² by factor (34.75/15.06)² =
5.3. Centripetal accel exceeds a_max by factor 4× since v²·κ_shoulder =
20137 vs a_max=5000." Or simplify: "the aggregator-capped profile violates
at least one per-s constraint by 4-5× at the shoulders."**

### MINOR — F. v_step = 69.5 at s/L=0.50 in §8.1 table is 2× too large

Location: `unified_v_of_s.md §8.1 line 510`.

Reproduction: `sqrt(3635·0.2415/0.707) = 35.23`, not 69.5. Not
catastrophic — v_step is non-binding at s/L=0.50 (v_cent=34.7 binds first).
But the table value is visibly wrong.

**Action: fix to 35.2 (or 34.7 if an alternate projection was used with
matching v_cent; clarify).**

### MINOR — G. `new_shaper_family.md §5.1` column "max on [0, 0.75·f_sh]" invertibility claims

The table shows pb_err 32-65% at 0.75·f_sh. §4.3 claims pb_err ≤5% on
[0, 0.3·f_sh] is the shippable target. The §5.1 table is informational but
could be misread. Minor; consider adding a footnote that values above f_sh/2
are not design targets.

### MINOR — H. Citation provenance

Post-revision, Wang-Altintas 2022/2023 CIRP references are marked retracted
in spec §"Literature anchors" and in `saturation_feedback.md §7` they still
appear in text ("Wang-Altintas 2022-2023"). `saturation_feedback.md §1.2`
still writes "Typical designs (Wang-Altintas 2022-2023 ...)" as if citing
them. The retraction is on the spec side but the research memo still refers
to them as if authoritative.

**Action: strip Wang-Altintas references from `saturation_feedback.md §1.2,
§1.3, §7` as well. The L1-L∞ argument stands alone on
Biagiotti-Melchiorri §5.8.**

Sencer-Tajima 2017 is cited but I cannot verify from this context; their
eq. 14 is stated to reduce to eq. 8 of saturation_feedback.md. Caller should
manually confirm the citation exists and eq. 14 is as described if this is
load-bearing.

---

## Open questions (couldn't verify, flagged)

1. **T_h = 2·T_sm justification.** spec §D1 line 226-228 and
   `new_shaper_family.md §4.1`. The research memo states this without
   derivation. Intuitively the inverse should have support ≥ the forward's
   correlation length ~T_sm; 2× is common practice but is there a decay
   argument? If not documented, the value is probably fine (numerics confirm
   convergence), but a sentence of justification would help.

2. **Cosine taper parameters (pb_max to f_sh transition).** eq. 8 of
   `new_shaper_family.md §4.1` with Tukey α=0.05. Why these parameters? The
   numerical G and pb_err depend on this choice. A brief sensitivity sweep
   or at least a citation would strengthen confidence that these aren't
   over-tuned.

3. **Sencer-Tajima eq. 14 equivalence to v_sat eq. 8.** Stated in
   `saturation_feedback.md §7`. Not verifiable without accessing the paper.
   Load-bearing for claiming multi-anchor literature support.

4. **Pham 2014 theorem 3.1 on arbitrary v_cap(s).** `unified_v_of_s.md
   §2.3` cites this for TOPP correctness. I trust this exists (Pham 2014 is
   a well-known work), but the exact theorem numbering should be
   double-checked — the published paper structure may differ from the
   citation.

5. **Piecewise-polynomial convolution piece-count for k_fused.** spec §D3
   line 488 claims "for bs_m with m+1 pieces convolved with FIR inverse of
   similar piece count, the fused has ≤ 2(m+1) pieces." The general result
   for piecewise polynomial convolution is that piece counts *add* (not
   multiply) and degrees add. So conv of `m+1` pieces with `N_h` samples
   (effectively N_h pieces of degree 0 if h is sampled) would give many more
   pieces. If h is also piecewise polynomial with P pieces of degree d, the
   fused has ≤ (m+1)+P-1 pieces of degree ≤ m+d. The "≤ 2(m+1)" claim
   requires h to have ≤ m+2 pieces, which isn't obvious from the inverse
   design. This is an implementation detail that should be verified when D1
   lands.

---

## Summary of required actions

- [CRITICAL A] Add √2 factor (or equivalent joint-constraint logic) to
  v_sat formula in `saturation_feedback.md §3` and spec §D4.
- [CRITICAL B] Re-run `unified_v_of_s.md §8` worked example with G=2.003
  explicitly, or label the current table as "pre-D1 baseline with G=1".
- [IMPORTANT C] Recompute T_fit in `unified_v_of_s.md §4.3`; correct the
  "12% slower" claim in both §4.3 and spec §D7.
- [IMPORTANT D] Fix monotonicity claim in `unified_v_of_s.md §3.4` to
  acknowledge v_step's dependence on normal-axis projection.
- [IMPORTANT E] Reword the "5.3× centripetal" framing in §1.2 and spec §D7.
- [MINOR F] Fix v_step=69.5 → 35.2 at s/L=0.50 in §8.1 table.
- [MINOR H] Remove Wang-Altintas citations from `saturation_feedback.md`.

None of these invalidate the overall Plan 5 architecture. The B-spline family
math (spec §D1) and inverse numerics are solid. The Pillar 2b TOPP algorithm
is correctly specified; its worked-example numbers need recompute passes but
the algorithm itself checks out.

---

## Reproducibility

Verification scripts used:
- `/tmp/verify_bspline.py` — T_sm, σ²_T, A_axis, F_m, damped residual
  (matches `new_shaper_family.md §10` `verify_all()`).
- `/tmp/verify_inverse.py` — G, pb_err, HF_amp at both pb_max values.
- `/tmp/verify_quintic_curvature.py` — κ(s) profile, shoulder location
  at 90°/cd=0.05.
- `/tmp/verify_vstep.py` — `v_step = sqrt(A_axis·R/|n̂·ê|)` per
  `blendshaper.compute_shaper_bounds`.
- `/tmp/verify_topp_G1.py` — TOPP forward/backward pass, T_opt,
  T_safe_const, T_unsafe at 90°/cd=0.05/a_max=5000/G=1.
- `/tmp/check_monotonicity.py` — v_step vs R scan, disproves §3.4 claim.
- `/tmp/check_L1_bound.py` — manual derivation of √2 missing factor.

Machine: darwin 25.4.0 arm64, Python 3.13 with numpy. No scipy dependency.

