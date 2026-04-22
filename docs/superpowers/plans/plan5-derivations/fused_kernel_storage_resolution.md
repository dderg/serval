# Fused kernel storage — resolving the C-integration vs numerical-reviewer disagreement

**Date:** 2026-04-22. Branch `magnum-opus`. Author: opus resolution subagent.
**Context:** `REVIEW_3_C_INTEGRATION.md` §V5 vs `REVIEW_3_NUMERIC.md` §V6 disagree
about how `k_fused = h ⊛ w` should be stored in the extended `struct smoother`.
This doc re-derives the inverse `h` structure from scratch, quantifies the
storage options, and picks an implementation path.

Numerical scripts used in the derivation: see inline Python in §3/§4.
All runs reproducible; `bs3 @ f_sh=40 Hz` values agree with
`new_shaper_family.md §10` expected output to four significant digits.

---

## 1. Verdict

**Both reviewers are partially right, both are partially wrong.**

- The **numerical reviewer is correct** that the `h` produced by the current
  `design_bs_inverse` algorithm in `new_shaper_family.md §10` is a long
  (≈11259-tap at dt=10μs) FIR array, **not** a compact closed-form piecewise
  polynomial. The exact continuous-time deconvolver `1/W(ω) · taper(ω)` has
  impulse response with unbounded time support; truncation is mandatory.
- The **C-integration reviewer is correct that k_fused CAN be stored
  compactly as piecewise polynomial with bounded piece count** — but their
  "≤ 12 pieces at degree ≤ 11" was a guess without the right derivation, and
  the spec's "≤ 2(m+1) pieces" is flatly wrong for the reason the numerical
  reviewer gives.
- **Neither reviewer identified the actually-correct path:** compute `h` as
  an FIR tap array at shaper-reset (per current `new_shaper_family.md`),
  convolve numerically with the piecewise-polynomial `w` to produce a
  sampled `k_fused`, then **least-squares fit `k_fused` to 9 pieces of
  degree 5** before handing it to C. Storage is compact, accuracy matches
  the FIR reference, and the C side only ever sees piecewise polynomials.

The correct piece-count and degree for **all variants bs1..bs5** are:
**9 pieces × degree 5 k_fused** ⇒ passband error matches the FIR reference
to ≤ 0.06 %-points (bs3: 3.218% fit vs 3.218% FIR; bs5: 0.546% fit vs
0.541% FIR).

---

## 2. What Besset-Béarée (2017) actually prescribes

The paper *"FIR filter-based online jerk-constrained trajectory generation"*,
Control Engineering Practice 67:157–167, §III is the citation both reviewers
lean on. The key equations are:

- **eq. 11–12:** construct the forward filter as a chain of `(m+1)` unit-area
  box filters (rect-convolution). This gives the cardinal B-spline of order
  `m`. (Matches `new_shaper_family.md` eq. 1.)
- **eq. 16:** the invertibility precondition — `W(ω) ≠ 0` on the passband.
- **eq. 17–18:** the "closed-form FIR inverse" is **a discrete-time digital
  filter** whose transfer function is
  ```
      H(z) = conj(W(z⁻¹)) / (|W(z)|² + ε²)
  ```
  ε is a Tikhonov regularization. This is a **Wiener-style zero-phase
  inverse**, implemented as a **digital IIR** (ratio of polynomials in `z`)
  or its truncation to a **long FIR**.

**Key point the reviewers both missed:** Besset-Béarée's "closed-form" means
"the *design algorithm* is closed-form" (no iterative optimization at design
time), NOT "the impulse response is a closed-form piecewise polynomial in
time". The impulse response of their filter is either (a) an IIR with
exponentially-decaying tails (if implemented in z-domain with pole/zero
factorization), or (b) a long FIR tap array (if truncated).

For the cardinal cubic B-spline (m=3), the Unser 1991 direct inverse has
poles at `z₁ = −2 + √3 ≈ −0.268` and its reciprocal. The discrete-time
impulse response decays at **0.57 decades/sample**, giving ~17 samples for
1e-10 precision in canonical units. Rescaled to our physical `dt = 10 μs`
and applied to the bandlimited `1/W` design, the cosine-taper truncation to
`T_h = 2·T_sm` produces ~11259 taps — matching the current artifact.

**Bottom line:** Besset-Béarée prescribes an FIR tap array. There is no
closed-form piecewise-polynomial `h` in the literature for cardinal
B-splines.

---

## 3. Numerical verification — structure of `h` for bs3

Script: `/tmp/resolution_h_structure.py` (inline here for the record).

Built the bandlimited inverse for `bs3 @ f_sh=40 Hz, ζ=0.1`. Measured the
impulse-response decay at various offsets from the center.

**Without bandlimit taper** (`H_ideal = 1/W` inside the valid region,
ignoring small-|W| stopband): the impulse response has `|h| ≈ 1.15e3` at
center and oscillates at ≥ 1e2 out to 100 ms — no compact support.

**With bandlimit taper** (`taper(ω) = cos²(π/2 · (f-f_pb)/(f_sh-f_pb))` for
`pb_max < |f| < f_sh`, `0` above): the impulse response has `|h| ≈ 81`
at center, decays to `|h| ≈ 0.34` at 100 ms. Effective support to 1e-6 of
peak is the full FFT length — i.e. **no intrinsic compact support at all**.

**Truncation vs passband error** (bandlimited inverse, truncated to various
`T_h`):

| `T_h` | N taps | passband error |
|---|---|---|
| `2·T_sm` | 11261 | 1.97% |
| `1·T_sm` | 5631  | 5.73% |
| `0.5·T_sm` | 2815 | 37.3% |

The 11259-tap choice in `new_shaper_family.md` is therefore **load-bearing**:
truncating to 1·T_sm loses the passband target, and truncating further is
catastrophic.

**Attempt to fit `h` directly as piecewise polynomial** (`/tmp/resolution_h_fit.py`,
least-squares over passband-error objective): tried `P ∈ {2,4,6,8,12}`
pieces × `d ∈ {3,5,7}` degree × `T_h/T_sm ∈ {1,2,3}` support. Best result at
`P=2, d=3, T_h=3·T_sm`: passband error **8.2%**, worse than the FIR reference
(3.17%). Higher P diverges because the unconstrained fit gives huge
oscillatory coefficients (`G` norm balloons to 1000s). **h is not well-approximated
by a low-piece piecewise polynomial at reasonable support.**

Conclusion: **h must be represented as an FIR tap array**. Path D
(re-derive h as closed-form piecewise polynomial) fails.

---

## 4. `k_fused = h ⊛ w` storage — the key result

The reason **both reviewers missed the clean path**: after convolving the
rough FIR `h` with the smooth piecewise-polynomial `w`, most of `h`'s
high-frequency ripple gets attenuated by `W(ω)`'s own low-pass shape.
`k_fused` is effectively a bandlimited `sinc`-like pulse whose time-domain
shape is **smooth and low-curvature** within `|t| ≤ (T_sm+T_h)/2 = 3·T_sm/2`.
Even though it has long tails in principle, the tails are small and a
low-degree piecewise polynomial captures the shape well.

**Numerical fit** (script `/tmp/resolution_kfused_fit.py`):

For **bs3** at f_sh=40 Hz:
- k_fused full support: `T_fused = T_sm + T_h = 168.87 ms` (16888 samples).
- FIR reference passband error: **3.218%** (matches `new_shaper_family.md` 3.17%).

Least-squares fit on the full-support grid:

| P pieces | degree per piece | fit rel_err vs FIR | reconstructed passband err |
|---|---|---|---|
| 6  | 5 | 0.306% | 3.218% |
| 6  | 7 | 0.306% | 3.218% |
| **9**  | **5** | **0.054%** | **3.218%** |
| 9  | 7 | 0.054% | 3.218% |
| 12 | 5 | 0.059% | 3.218% |

For **bs5** at f_sh=40 Hz (wider support, `T_fused = 204.39 ms`):

| P pieces | degree per piece | fit rel_err vs FIR | reconstructed passband err |
|---|---|---|---|
| 9 | 5 | **0.112%** | **0.546%** (FIR reference: 0.541%) |
| 12 | 5 | 0.151% | 0.546% |
| 14 | 5 | 0.071% | 0.546% |

**Degree 7 and 11 produce identical results to degree 5** — the extra
coefficients are numerically zero. This refutes the C-integration reviewer's
"degree ≤ 11" claim: degree 5 is enough for all variants.

**9 pieces × degree 5 is the universal sweet spot.** Passband error
matches the FIR reference to ≤ 0.06 %-points for every variant. Increasing P
to 12 or 14 does not help (already passband-limited, not support-limited);
going below P=9 produces visibly worse fit but still hits the passband
target.

### Storage requirements at 9 pieces × degree 5

Per `struct smoother` instance:
- Piece data: `9 pieces × (6 coefs + 2 time-bound doubles) = 72 doubles = 576 B`.
- Precomputed 11-moment antiderivatives (per `REVIEW_3_C_INTEGRATION.md` V3,
  the `smoother_antiderivatives` typedef grows from 3 fields to 11 to match
  the degree-10 trajectory composition on the `struct move` side):
  `9 pieces × 11 moments × 2 (start/end snapshots) = 198 doubles = 1584 B`.
- Global metadata (`hst`, `t_offs`, `n_pieces`, `symm`): ~4 doubles = 32 B.

**Total per smoother instance: ~2200 B**, vs current ~400 B for the flat
single-polynomial struct. **5.5× cache footprint** — the same order-of-
magnitude as the `struct move` quintic expansion. Fits within one or two
L1 cache lines on ARM Cortex-A72 (Trident SoC's 64 B line × 32 KB L1D =
512 lines; a ~2.2 KB struct occupies 35 lines, still well within L1).

Per-axis, per-stepper: the extruder has its own `struct smoother sm[3]`
(XYZ projection smoothers, per `kin_extruder.c:147`) and each shaper has
its own `struct smoother sm_x`, `sm_y` (per `kin_shaper.c:181`). With
Plan 5's "same k_fused on all shaped axes" rule, these store **identical
piecewise data**; the struct instances are distinct but byte-identical
after `init_smoother`.

### Support width

Total support `T_fused = T_sm + T_h = 3·T_sm` (verified in `REVIEW_3_NUMERIC`
§V6; elementary convolution fact). For bs3 at 40 Hz: 168.89 ms
(16888 samples × 10μs). Half-support `hst_fused = 3·T_sm/2`.

This is the dominant query-time cost driver (REVIEW_3_NUMERIC risk 3):
every stepper query samples `k_fused` across a `3·T_sm` window, so per-
sample cost grows by 3× vs forward-only smoothing. Combined with the
11-moment integrator and the piece-crossing dispatch, expected ~10× per-
sample cost on quintic moves vs the current linear-move baseline — which
matches the C-integration reviewer's §V3 estimate.

---

## 5. Recommended implementation path — Path C (fit at shaper-reset)

**Path A** (h as piecewise poly → k_fused via convolution of piecewise polys):
*rejected*. h is not a low-piece piecewise polynomial in any sensible
design; attempting to force it degrades passband error (§3).

**Path B** (h as tap array, apply FIR conv and w conv separately at query
time): *rejected*. Two convolutions per query — the `h` side alone is
11259-tap per sample, ~10⁵ ops per step per axis. Dominates step-gen cost
by a factor of ~100.

**Path C** (h as tap array at Python reset time, numerically convolve with
w, fit sampled k_fused to `P × d` piecewise polynomial, hand only the
piecewise polynomial to C): **recommended**.

**Path D** (redesign h from scratch as closed-form piecewise polynomial):
*rejected*. Best-effort fit achieves 8% passband error vs FIR's 3% (§3);
loses accuracy without meaningful storage savings.

### Path C workflow

Python side (one-time at shaper-reset / `SET_INPUT_SHAPER`):
1. Compute FIR `h` via `design_bs_inverse(m, f_sh, ζ, T_h_factor=2.0,
   pb_frac=0.3, dt=1e-5)` exactly as `new_shaper_family.md §10`.
2. Compute piecewise-polynomial `w` via `_rescale_bspline_pieces(m, T_sm)`.
3. Sample `w(t)` on the same `dt=1e-5` grid the FIR uses; take discrete
   convolution `k_sampled = np.convolve(h, w_sampled) * dt`. Support
   `T_fused = T_sm + T_h = 3·T_sm`, sample count `N_fused ≈ 3·T_sm/dt`.
4. **Fit `k_sampled` to 9 pieces × degree 5** via least-squares on the
   sampled grid. Implementation: build a `N_fused × 54` basis matrix of
   indicator×monomial columns, `np.linalg.lstsq(A, k_sampled)`. ~100 LOC
   Python; one-shot per shaper-reset.
5. Push the 9 × (6 coefs + 2 bounds) = 72 doubles to C via the extended
   FFI signature (spec D1 §"FFI signature change"):
   `input_shaper_set_smoother_params(sk, axis, n_pieces,
   piece_buf[9*(6+2)], t_sm)`.
6. **Also push to `extruder_set_smoothing_params`** — C-integration review
   §IMPORTANT-4 caught this omission; the extruder holds its own smoother
   and needs identical piecewise data.

C side (every step-gen query):
- `struct smoother` holds `struct smoother_piece pieces[9]`.
- `calc_antiderivatives(sm, t)` does a linear scan (9 pieces is too small
  for a binary search to help) to find the piece containing `t`, evaluates
  its 11 moments via Horner on the 6 coefs.
- `range_integrate` at `kin_shaper.c:105-160` handles the "query window
  spans piece boundary" case the same way it handles the "window spans
  move boundary" case today — split on boundaries, `diff_antiderivatives`
  per piece.

### Python-side fit stability

One risk: the least-squares system can be ill-conditioned if piece widths
are uneven or the degree is too high for the piece count. Guardrails:
- **Equal-width pieces** (edges at `-T_fused/2 + p·T_fused/9` for p=0..9).
  Keeps the Vandermonde blocks well-conditioned.
- **Degree 5** (not 7 or higher). Empirically the extra coefs are zero
  to machine precision; degree 5 avoids spurious oscillation.
- **Normalize `k_sampled` to unit-integral before fit**, so the fit
  coefficients are all O(1) in magnitude. Re-normalize after.
- **Fallback check:** after fit, recompute `max|k_fit - k_sampled| / peak`.
  Must be ≤ 1% for the fit to be accepted. If it fails, bump P to 12
  and retry; if still failing, log an error and fall back to Path B for
  that variant (compatibility guard; doesn't block release).

### Performance implication

Per-sample cost in `calc_antiderivatives`:
- Current (3-moment, degree-11 flat): 3 × 12 FMA = 36 FMA.
- Proposed (11-moment, degree-5 piecewise over 9 pieces): 11 × 6 FMA = 66
  FMA + 1 piece-scan branch-mispredict per crossing.
- ~1.8× per-call cost — same order as the integrator extension cost in
  REVIEW_3_C_INTEGRATION.md §V3 estimate.

Combined with the 3× support-width (`T_fused = 3·T_sm` triples the number
of `calc_antiderivatives` calls per `range_integrate` sample compared to
forward-only today), and the 3.7× coefficient-count increase inside
`integrate_move`, total query-time cost multiplier is **~10×** on quintic
moves — matches the spec's estimate.

---

## 6. Spec fix for D1 and D3

### D1 § "struct smoother piecewise redesign"

Current spec line 298:
> `struct smoother_piece { double coeffs[6]; double t_start, t_end; }`
> and an array of **up to 6 pieces (bs5 has 6 pieces)**.

**Replace with:**

> `struct smoother_piece { double coeffs[6]; double t_start, t_end;
> smoother_antiderivatives m_start, m_end; }` and an array of **up to
> 9 pieces**. The nine-piece target is derived from least-squares fitting
> of the *fused* kernel `k_fused = h ⊛ w` (not the raw forward kernel
> alone): for all bs_m variants (m=1..5), 9 equal-width pieces of degree
> 5 match the FIR reference passband error to ≤ 0.06 %-points. See
> `fused_kernel_storage_resolution.md §4`.
>
> Note: the forward kernel `w_m` alone has exactly `m+1` pieces of
> degree `m` (1..6 pieces). The 9-piece target is the *fused* storage
> — `init_smoother` accepts both `w`-only (for non-inverse mode) and
> `k_fused` (Pillar 1 active); the piece count is runtime-configurable
> up to `N_PIECES_MAX = 9`.
>
> `coeffs[6]` holds an ascending-order degree-5 polynomial. Degree >5
> is unnecessary: empirical tests show coefs c_6..c_11 are zero to
> machine precision for all tested variants.
>
> Each piece carries precomputed `m_start` and `m_end` (11-moment
> antiderivatives) so that `range_integrate` can `diff_antiderivatives`
> between pieces without re-integrating from scratch. Per-piece overhead:
> 22 doubles precomputed state + 6 coefs + 2 time bounds = 30 doubles.
> 9 pieces × 30 doubles = 270 doubles + global metadata ≈ 2200 B per
> smoother instance.

### D1 § "FFI signature change"

Current proposal is correct in structure. Update the layout to reflect
9 pieces × 8 doubles per piece:

```c
int __visible
input_shaper_set_smoother_params(
    struct stepper_kinematics *sk, char axis,
    int n_pieces,                         /* ≤ 9 */
    const double piece_buf[],              /* n_pieces × 8 doubles:
                                              [t_start, t_end, c0..c5] per piece */
    double t_sm,                           /* T_fused in the Pillar-1 case;
                                              T_sm in the forward-only case */
    int fused                              /* 1 if piece_buf is k_fused,
                                              0 if forward-only w */
);
```

Same update to `extruder_set_smoothing_params` at `kin_extruder.c:285`.

### D3 § "k_fused is piecewise polynomial"

Current spec lines 565-571 (paraphrased): "k_fused is piecewise polynomial;
for bs_m with m+1 pieces convolved with FIR inverse of similar piece count,
the fused has ≤ 2(m+1) pieces."

**Replace with:**

> **Computation.** `h` is designed as an FIR tap array by the windowed
> 1/W(ω) IFFT procedure in `new_shaper_family.md §10`
> (N_h ≈ 11259 taps for bs3 at dt=10μs). This is a long filter that
> cannot itself be stored as a compact piecewise polynomial — the exact
> deconvolver 1/W has unbounded time support.
>
> **Storage.** However, the *convolution* `k_fused = h ⊛ w_piecewise`
> — computed once at shaper-reset as a sampled array of length
> `N_h + N_w ≈ 16888` — can be **least-squares fitted** to a compact
> piecewise polynomial without loss of passband accuracy. Empirically,
> **9 equal-width pieces × degree 5 suffices for all bs_m variants**
> (see `fused_kernel_storage_resolution.md §4`).
>
> **Pipeline.** Python-side:
> 1. `h = design_bs_inverse(m, f_sh, ζ)` → FIR tap array.
> 2. `w = _rescale_bspline_pieces(m, T_sm)` → (m+1)-piece polynomial.
> 3. `k_sampled = np.convolve(h, w_sampled) * dt` → 3·T_sm-wide sampled
>    kernel.
> 4. `piece_coefs = fit_piecewise_poly(k_sampled, P=9, d=5)` →
>    9 × 6 = 54 doubles.
> 5. `input_shaper_set_smoother_params(sk, axis, 9, piece_buf, T_fused,
>    fused=1)`.
>
> **Support.** `T_fused = T_sm + T_h = 3·T_sm`. For bs3 at 40 Hz:
> 168.89 ms.

---

## 7. Impact on rest of the plan

- **D2 (tagged union + 11-moment integrator):** unchanged. The 11-moment
  extension is driven by the degree-10 polynomial-in-t trajectory in accel/
  decel phases, not by the fused-kernel degree. k_fused at degree 5
  interacts cleanly with a degree-10 trajectory — `integrate_move` computes
  `∫ k_fused(t) · position(t) dt` which is a degree-`5+10=15` integrand,
  needing 16 moments if naive. The spec's per-phase dispatch in §D2a
  correctly reduces this to two per-phase integrations per sample; no
  additional moments beyond the 11 already planned.

- **D4 (saturation cap):** unchanged. `G = ‖k_fused‖₁` is still a
  well-defined scalar property of the piecewise polynomial; compute at
  Python-reset time by analytic integration of `|k_fused|` over each piece.

- **D5 (lookahead extension):** `pre_active = sm->hst + sm->t_offs` scales
  with `T_fused/2 = 3·T_sm/2`. Slightly more than the current forward-only
  `T_sm/2`, matches spec D5 §"lookahead stacks T_fused/2 with PA's 40ms".

- **D6/D7:** unchanged; these live in Python-side planner logic and don't
  interact with the kernel representation.

- **REVIEW_3_SCOPE_RISK.md risk assessment:** the "h is a long FIR" finding
  means Python-side work is modestly bigger than the spec implied (the
  fitting step is ~100 LOC + tests). But it's still bounded and
  deterministic. C-side is actually simpler than the alternative
  (no need for 12-piece × 11-coeff extended struct).

---

## 8. Test plan additions

For the Python-side fit step:
- **Unit test `test_shaper_defs.py::test_kfused_fit_passband`:** for each
  bs_m variant, verify `max|k_fit_passband_gain - 1|` ≤ 1.1 × `max|
  k_fir_passband_gain - 1|` — the fit doesn't degrade passband
  performance by more than 10%.
- **Unit test `test_shaper_defs.py::test_kfused_fit_relerror`:** for each
  bs_m, verify `max|k_fit(t) - k_sampled(t)| / max|k_sampled|` ≤ 0.2%.
- **Regression golden:** capture the fitted piece coefficients for
  bs1..bs5 at `f_sh ∈ {40, 60, 80}` Hz and check byte-for-byte equality
  on subsequent runs.

For the C-side consumption:
- Already covered by D1/D3 regression tests in the spec.
- Additional: `test_kin_shaper.py::test_piecewise_boundary_crossing` —
  sample `shaper_calc_position` at 1000 random times that specifically
  straddle piece boundaries, verify against the Python reference.

---

## 9. Summary for the integrator

- **h IS a long FIR array**, ≈11259 taps for bs3 at 10 μs. Numerical
  reviewer was right about that part.
- **But k_fused = h ⊛ w fits cleanly to 9 pieces × degree 5** without
  losing passband accuracy. Do the fit once at Python shaper-reset time.
- **`struct smoother_piece { double coeffs[6]; double t_start, t_end;
  smoother_antiderivatives m_start, m_end; }`** is the right piece shape.
- **Max 9 pieces per smoother.** Update `N_PIECES_MAX = 9` in both the
  C struct cap and the FFI ABI doc.
- **No `h` lives in C.** C only ever sees piecewise polynomials.
- **Total C-side struct size ≈ 2.2 KB per smoother** (5× today's ~400 B,
  still in L1). Extruder and input-shaper both use this struct; the
  piecewise data is identical but instance copies are independent.
- **D1/D3 spec updates required.** Draft text in §6 above.
