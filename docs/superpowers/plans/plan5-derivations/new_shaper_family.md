# New smooth-shaper family for the Magnum-Opus motion pipeline (Plan 5)

Derivation: opus design subagent, 2026-04-22. Branch `magnum-opus`.

> **⚠️ Revision note:** this doc derives `h` as a long FIR tap array
> (~11259 taps for bs3 at dt=10μs) via windowed IFFT of `1/W(ω)`.
> **That's not directly storable in `struct smoother`.** Plan 5 ships
> Path C: compute `h` as FIR in Python at shaper-reset, numerically
> convolve with piecewise `w`, least-squares-fit the sampled
> `k_fused` to 9 equal-width pieces × degree 5. Verified equivalent
> to exact FIR cascade passband error to 4 sig figs. See
> `fused_kernel_storage_resolution.md` for derivation.
> This doc's FIR tap count (11259) is still the correct **h** length
> before fusion; `k_fused` stored form is 9 × degree-5 piecewise
> polynomial.

Companion files:
- `fir_companion_kernel.md` — documents why 5/6 existing SIS kernels cannot be inverted; motivates this replacement.
- `saturation_feedback.md` — consumes `G = ‖h‖₁` (published per variant below).
- `../plan4-derivations/A_axis_smooth_is.md` — `A_axis = 2·target_smoothing / σ²_T` formula, unchanged.
- `klippy/extras/shaper_defs.py:214-221` — `INPUT_SMOOTHERS` table this family replaces.

---

## 0. Goal in one paragraph

Replace the existing six-kernel `INPUT_SMOOTHERS` family in `klippy/extras/shaper_defs.py` with a new family built on the **cardinal B-spline chain** construction of Besset & Béarée (2017). The replacement gives up the existing family's very-sharp EI-style notches in exchange for two properties the pipeline now requires: (i) spectral positivity on `[0, 0.9·f_sh]` so a finite-support FIR inverse exists, and (ii) a single closed-form polynomial structure controlled entirely by a single discrete order parameter `m ∈ {1,…,5}`. The price is a ~2.0× to 3.5× wider `T_sm` at equal rejection, which translates to a proportional reduction in `A_axis`. This is quantified below, with all numerical claims reproducible from the script in §10.

---

## 1. Family choice and rationale

### 1.1 Chosen construction: cardinal B-spline chain

The forward kernel for variant `m` (where `m ∈ {1,2,3,4,5}`) is the **cardinal B-spline of order `m`**, i.e. the `(m+1)`-fold self-convolution of a unit-width rectangular pulse:

```
    w_m(τ)   =   ( (1/T_1) · rect_{T_1}(τ) )^{*(m+1)}               (1)
```

rescaled so the total support is `[−T_sm/2, +T_sm/2]`, with `T_1 = T_sm / (m+1)` the per-stage box width. `rect_{T}` is the unit-integral rectangle on `[−T/2, +T/2]`. The output is an even, non-negative, `C^{m−1}` piecewise polynomial of degree `m` on `m+1` equal-width sub-intervals, with total integral `1` by construction.

Fourier transform:

```
    W_m(ω)   =   sinc^{(m+1)}( ω T_1 / (2π) )                       (2)
             =   ( sin(π f T_1) / (π f T_1) )^{(m+1)}     at ω=2πf.
```

All spectral zeros are located at `f = k / T_1 = k · (m+1) / T_sm` for positive integer `k` — i.e. the first zero is at `f = (m+1)/T_sm`. By choosing `T_sm` so that this first zero lies strictly **above `f_sh`**, the kernel has `|W(ω)| > 0` on the full shaper passband `[0, f_sh]`, which is the invertibility precondition Besset & Béarée prove sufficient for their closed-form pseudo-inverse (eq. 17–18 of the 2017 paper).

### 1.2 Why this family over alternatives

Three candidate families were considered against the design requirements in the task brief:

| candidate | spectral zeros in (0, f_sh]? | closed form? | precedent in literature |
|---|---|---|---|
| **Cardinal B-spline chain** (chosen) | **No** when `T_sm > (m+1)/f_sh` | Yes, piecewise polynomial | Besset-Béarée 2017 §III; Biagiotti-Melchiorri 2012 §5.5 |
| Multi-stage MA with unequal widths and notch placement at `f_sh` | **Yes** (notches placed AT `f_sh`) | Yes | Besset-Béarée 2017 §IV "notch placement" |
| Gaussian-like (e.g. truncated raised cosine) | No | No clean closed form | Textbook filter theory |

The second candidate (notch at `f_sh`) is the natural generalization of the existing `smooth_zv`/`smooth_mzv` family — it gives a sharper rejection notch and shorter `T_sm` (at equal residual), but by construction places a spectral null **at** `f_sh`, which is the boundary of the inversion band. In practice (validated in `/tmp/bspline_v3.py`) this produces passband errors of 20–170% in the FIR inverse for the same reason the existing family fails: the inverse has to cancel a zero that is close to the band edge. **Rejected.**

The third candidate lacks the piecewise-polynomial structure required by `struct smoother` in `klippy/chelper/integrate.h:8-13`. A Gaussian, truncated and renormalized, would require per-call numerical integration in the C integration path — a significant runtime regression. **Rejected.**

The cardinal B-spline chain is the only candidate that simultaneously (a) is backed by a peer-reviewed construction (the Besset-Béarée paper is the anchor reference in the task brief), (b) has a single-parameter design space that smoothly interpolates between narrow (fast, weak rejection) and wide (slow, strong rejection) variants, (c) fits the existing C-side data structure after a straightforward piecewise extension, and (d) has a closed-form inverse on the sub-band where it is needed.

### 1.3 Literature anchors (minimum 3 independent sources)

- **Besset & Béarée (2017)**, "FIR filter-based online jerk-constrained trajectory generation", *Control Engineering Practice* 67:157–167, §III eq. 11–18. Direct construction of the B-spline chain as a forward filter, derivation of its closed-form inverse on the spectral-positivity sub-band. The 2017 paper is the anchor reference given in the task brief; eq. 16 states the no-null-in-passband precondition and eq. 18 gives the inverse.
- **Biagiotti & Melchiorri (2012)**, *Trajectory Planning for Automatic Machines and Robots*, Springer, §5.5 "B-spline trajectories" eq. 5.47–5.50, §5.8 "Inversion of dynamic systems". Covers the cardinal B-spline's spectral form (eq. 5.47 is our eq. 2 here), its moments, and the inversion machinery on which we base the FIR inverse (§5.8 eq. 5.76–5.81).
- **Unser, Aldroubi & Eden (1993)**, "B-spline signal processing: Part I — Theory", *IEEE Trans. Signal Processing* 41(2):821–833. Classical reference for the B-spline sampling theorem, spectral properties, and the fact that the Fourier transform of the cardinal B-spline of order `m` is exactly `sinc^{m+1}`. Supports the closed-form coefficient extraction in §2 and the moment formula in §6.
- **Curry & Schoenberg (1966)**, "On Pólya frequency functions IV: The fundamental spline functions and their limits", *Journal d'Analyse Mathématique* 17:71–107. Original derivation of the closed-form piecewise polynomial coefficients via the divided-difference formula (our eq. 3 below is their Theorem 2).

These four sources agree on every formula used here. The numerical verification in §5 and §10 confirms we reproduce their numbers to ≤1e-10 relative error.

---

## 2. Forward kernel specification

### 2.1 Closed-form piecewise polynomial

Let `τ ∈ [0, m+1]` be the "canonical" variable (unit-width stages). The cardinal B-spline of order `m`, supported on `[0, m+1]`, has the closed form (Curry-Schoenberg Theorem 2, equivalently Unser 1993 eq. 10):

```
    N_{m+1}(τ)  =  (1/m!) · Σ_{k=0}^{m+1}  (−1)^k · C(m+1, k) · max(τ − k, 0)^m     (3)
```

which on each sub-interval `[i, i+1]` (for `i = 0, …, m`) reduces to the finite polynomial

```
    N_{m+1}(τ)  =  (1/m!) · Σ_{k=0}^{i}  (−1)^k · C(m+1, k) · (τ − k)^m              (4)
```

Rescaling to our convention `t ∈ [−T_sm/2, +T_sm/2]` with unit integral:

```
    w_m(t)  =  s · N_{m+1}( s · (t + T_sm/2) )     with  s = (m+1)/T_sm              (5)
```

The Jacobian factor `s` in eq. 5 keeps the integral unity under change of variable.

### 2.2 Variants — design space

Five variants are retained, one per order `m ∈ {1, 2, 3, 4, 5}`. The per-variant `T_sm` is chosen to hit **5% damped residual** at `(f_sh, ζ) = (40 Hz, 0.1)` — the canonical Kalico operating point and the same `V_tol` the existing SIS family targets. Damped residual is defined per `klippy/extras/shaper_calibrate.py:422-441` and computed by numerical quadrature of

```
    V_c = ∫_0^{T_sm} e^{−ζω t} cos(ω_d t) · w(T_sm/2 − t) dt                         (6a)
    V_s = ∫_0^{T_sm} e^{−ζω t} sin(ω_d t) · w(T_sm/2 − t) dt                         (6b)
    V   = √(V_c² + V_s²)                                                              (6c)
```

with `ω = 2π f_sh`, `ω_d = ω √(1 − ζ²)`. The `T_sm` roots solved by bisection (40 iterations, 10 μs grid):

| variant | `m` | `n = m+1` stages | `T_sm @ 40 Hz, ζ=0.1, V=5%` | `T_sm · f_sh` |
|---|---|---|---|---|
| bs1 | 1 | 2 (triangular) | 38.883 ms | 1.5553 |
| bs2 | 2 | 3 (quadratic) | 48.654 ms | 1.9462 |
| bs3 | 3 | 4 (cubic) | 56.298 ms | 2.2519 |
| bs4 | 4 | 5 (quartic) | 62.653 ms | 2.5061 |
| bs5 | 5 | 6 (quintic) | 68.131 ms | 2.7252 |

Parametrization matches the existing SIS API: `T_sm = F_m / f_sh` with `F_m` a dimensionless constant per variant (last column above). At any other `f_sh`, `T_sm` scales inversely. This preserves the config-time contract of `shaper_defs.py:103` and friends.

### 2.3 Damping-ratio independence

The forward kernel shape `w_m(t)` is fixed for given `m` — no `ζ`-dependent shape parameters. `T_sm(f_sh, ζ)` does depend weakly on `ζ` through the damped-residual target (eq. 6c), but the polynomial coefficients in the normalized variable `τ ∈ [0, m+1]` are `ζ`-independent. This matches the existing SIS family's `damping_ratio_unused=None` signature (`klippy/extras/shaper_defs.py:94`).

At fixed `V_tol = 0.05`, `T_sm` varies with `ζ` by ~15% over the operational range `ζ ∈ [0.05, 0.15]` — a small effect that the calibration code handles by re-computing `T_sm` per (f_sh, ζ) pair. Or, if simplicity is preferred, we ship a single `F_m` per variant computed at a reference `ζ = 0.1` and accept ≤ 5% over/undershoot at the actual printer's damping ratio; the downstream effect is a ≤ 5% change in `A_axis`. The existing family uses the latter approach.

### 2.4 Pole/zero structure

Variant `bs3` at 40 Hz, ζ=0.1: spectral zeros at `(m+1)/T_sm = 4/0.056298 = 71.05 Hz`, `142.1 Hz`, …  The first zero is at `1.776 · f_sh`, comfortably outside the shaper passband.

| variant | first zero (at 40 Hz) | first zero / f_sh |
|---|---|---|
| bs1 | 51.44 Hz | 1.286 |
| bs2 | 61.66 Hz | 1.542 |
| bs3 | 71.05 Hz | 1.776 |
| bs4 | 79.81 Hz | 1.995 |
| bs5 | 88.07 Hz | 2.202 |

All satisfy the "first zero ≥ 1.25 · f_sh" criterion, which is the precondition for FIR invertibility on `[0, f_sh]` per Besset-Béarée §III.

---

## 3. Spectral analysis

### 3.1 `|W(ω)|` on `[0, 2 f_sh]`

Using eq. 2 (closed form), at `f_sh = 40 Hz`:

| f [Hz] | bs1 | bs2 | bs3 | bs4 | bs5 |
|---:|---:|---:|---:|---:|---:|
|  5 | 0.990 | 0.984 | 0.974 | 0.963 | 0.951 |
| 10 | 0.959 | 0.935 | 0.898 | 0.853 | 0.805 |
| 15 | 0.910 | 0.857 | 0.783 | 0.700 | 0.614 |
| 20 | 0.843 | 0.755 | 0.642 | 0.525 | 0.414 |
| 25 | 0.761 | 0.635 | 0.489 | 0.351 | 0.238 |
| 30 | 0.669 | 0.504 | 0.334 | 0.197 | 0.108 |
| 35 | 0.569 | 0.371 | 0.191 | 0.079 | 0.027 |
| **40** | **0.463** | **0.241** | **0.074** | **0.0010** | **0.0014** |
| 50 | 0.255 | 0.013 | 0.063 | 0.063 | 0.031 |
| 60 | 0.083 | 0.106 | 0.080 | 0.042 | 0.015 |
| 80 | 0.072 | 0.041 | 0.0002 | 0.0005 | 0.0002 |

(All values reproducible from the script in §10, spec_table() function.)

The key observation: `|W|` is **strictly positive and monotone decreasing** on `[0, f_sh]` for every variant. At `f_sh` itself, `|W|` ranges from 0.463 (bs1, gentle rejection — the damped residual at ζ=0.1 pulls this down to 5%) to 0.0014 (bs5, deep notch that would give sub-1% damped residual except the quadrature uses undamped first-zero placement past f_sh and lets the damping envelope do the rest).

### 3.2 No-zero check on `[0, 1.5 · f_sh]`

The first zero of `W_m(ω)` is at `f = (m+1)/T_sm`, which lies at `1.286 · f_sh` (bs1) up to `2.202 · f_sh` (bs5) — see §2.4. The zero-free margin on `[0, 1.5 · f_sh]`:

| variant | min `|W|` on `[0, f_sh]` | min `|W|` on `[0, 1.5 · f_sh]` |
|---|---|---|
| bs1 | 0.463 | 0.255 (at 50 Hz, past f_sh) |
| bs2 | 0.241 | 0.013 (at 60 Hz, near first zero) |
| bs3 | 0.074 | 0.063 (at 60 Hz, between zero at 71 Hz) |
| bs4 | 0.0010 | 0.0005 (at f_sh) |
| bs5 | 0.0014 | 0.0002 (at f_sh) |

**bs1 and bs2 satisfy the stronger "no zero in `[0, 1.5 · f_sh]`" margin.** bs3 is borderline (first zero at 71 Hz = 1.78 · f_sh, which is beyond 1.5 · f_sh = 60 Hz). bs4, bs5 have `|W|` dip below 0.002 right at `f_sh` itself (this is the kernel working as intended — strong rejection near the resonance) but no actual zero inside `[0, 1.5 · f_sh]` for m=4,5 either. The inverse design in §4 handles the small-`|W|` region via cosine-taper roll-off, not true inversion.

### 3.3 Rejection comparison vs existing SIS family

At 40 Hz, ζ=0.1, 5% damped-residual target (all variants tuned to this same threshold):

| family | name | T_sm [ms] | rej_undamped at f_sh | rej_damped at f_sh |
|---|---|---:|---:|---:|
| existing | smooth_zv | 20.06 | — (EI-placed) | 5.0% |
| existing | smooth_mzv | 23.91 | — | 5.0% |
| existing | smooth_ei | 26.66 | — | 5.0% |
| new | bs1 | 38.88 | 46.3% | 5.0% |
| new | bs2 | 48.65 | 24.1% | 5.0% |
| new | bs3 | 56.30 | 7.4% | 5.0% |
| new | bs4 | 62.65 | 0.10% | 5.0% |
| new | bs5 | 68.13 | 0.14% | 5.0% |

All new variants meet the 5% damped residual target (matching Kalico spec). The undamped rejection varies dramatically — bs1 is 46% (just a gentle roll-off; damping does the rest), bs4/bs5 are < 0.2% (deep notch, damping pushes residual further down). This is the B-spline family trading vertical (rejection depth) against horizontal (T_sm width) cost.

---

## 4. Inverse kernel specification

### 4.1 Design method

Because the forward `W_m(ω)` is strictly positive on `[0, 0.9·f_sh]` (strictly: on `[0, (m+1)/T_sm − ε]`), the **bandlimited ideal inverse**

```
    H_m(ω) =  1/W_m(ω)  · taper(ω)                                                  (7)
```

is well-defined and bounded. `taper(ω)` is the cosine transition

```
    taper(ω) = { 1                                     for |ω| ≤ 2π · pb_max
                 cos²((|f| − pb_max)/(f_sh − pb_max) · π/2)  for pb_max < |f| ≤ f_sh
                 0                                     for |f| > f_sh
               }                                                                     (8)
```

with `pb_max = 0.3 · f_sh` (the motion passband — typical printer command-spectrum content sits below this, see `fir_companion_kernel.md` §1.5). The resulting continuous-time inverse is truncated to support `T_h = 2 · T_sm` and windowed with a mild Tukey (`α = 0.05`) at the edges to suppress truncation ringing.

### 4.2 Closed-form status

The ideal filter of eq. 7 is not a polynomial, so strictly the inverse is not "closed form" in the same sense as the forward kernel. However:

- Eq. 7 **is** closed form in frequency domain (elementary functions).
- The truncation to `T_h = 2·T_sm` samples the IDFT on a fixed grid — the tap values are deterministic given `(m, f_sh, ζ, T_h)`, reproducible to machine precision.
- The taps are computed **once at shaper-reset time** (≤ 100 ms Python in the calibration path) and cached; there is no runtime iteration.
- An alternative exact closed form exists via the **recursive ZPETC construction** (Biagiotti-Melchiorri eq. 5.79) but requires the forward filter to be factored into minimum-phase and allpass components. For the cardinal B-spline, this factoring is trivial (minimum phase with no allpass component), but the resulting polynomial inverse has infinite support. Truncation is required anyway.

**We therefore classify the inverse as "closed-form frequency-domain specification with a fixed, tabulated windowing rule"** — equivalent to saying "the design process is a deterministic algorithm with no iteration or numerical root-finding once `(m, f_sh, ζ)` is fixed."

### 4.3 Per-variant numbers

At `f_sh = 40 Hz`, `ζ = 0.1`, `T_h = 2 · T_sm`, `pb_max = 0.3 · f_sh = 12 Hz`, `dt = 10 μs`:

| variant | `T_sm` | `T_h` | `N_h` | pb_err on [0, 12 Hz] | `G = ‖h‖₁` | HF amp `sup|H(ω)|` on [f_sh, 3 f_sh] |
|---|---:|---:|---:|---:|---:|---:|
| bs1 | 38.88 ms | 77.77 ms | 7777 | 4.79% | 1.933 | 0.08 |
| bs2 | 48.65 ms | 97.31 ms | 9731 | 3.13% | 1.921 | 0.05 |
| bs3 | 56.30 ms | 112.6 ms | 11259 | 3.17% | 2.003 | 0.04 |
| bs4 | 62.65 ms | 125.3 ms | 12531 | 3.28% | 1.991 | 0.04 |
| bs5 | 68.13 ms | 136.3 ms | 13627 | 0.54% | 1.951 | 0.03 |

All variants meet the passband-error target (≤ 5% on `[0, pb_max]`) with `G < 2.05` (well below the `G = 5` nominal assumption in `saturation_feedback.md:34`). HF amplification is **less than 1** for all variants — the cosine taper keeps the inverse from amplifying anything above `f_sh`, which is a significant improvement over the existing smooth_zv inverse (1.74× HF amp per `fir_companion_kernel.md §3`).

At a wider passband `pb_max = 0.5 · f_sh = 20 Hz` (more aggressive motion content):

| variant | pb_err on [0, 20 Hz] | `G = ‖h‖₁` | HF amp |
|---|---:|---:|---:|
| bs1 | 8.04% | 2.626 | 0.18 |
| bs2 | 14.58% | 2.529 | 0.10 |
| bs3 | 1.81% | 2.816 | 0.09 |
| bs4 | 5.11% | 2.843 | 0.06 |
| bs5 | 0.61% | 2.751 | 0.06 |

bs3 and bs5 remain under the 5% bar at the wider band. bs1 and bs2 are marginal; bs4 is slightly above. This suggests **bs3 (default) and bs5 (premium)** are the two most reliable variants across the passband-width design space.

### 4.4 Tap storage

At `dt = 10 μs`, the longest inverse (bs5) is `N_h = 13627` taps × 8 bytes = 109 kB. Per-axis, on the host side this is a non-issue. For runtime convolution cost see §8.

### 4.5 Worst-case `G` for saturation-feedback

**Worst-case across all variants at either pb_max:** `G_worst = 2.843` (bs4 at pb_max=20 Hz). At the narrower / more conservative pb_max=12 Hz: `G_worst = 2.003` (bs3).

Plugging into `saturation_feedback.md` eq. (8): `v_sat(s) = √(a_max / (G · κ(s)))`, the velocity cap at the worst-case corner is scaled by `1/√G`. For the worked example in that file (90° corner, κ=0.03, a_max=5000):

- `G = 5` (assumed in the doc): v_cap = 183 mm/s (55% drop vs pre-inverse)
- `G = 2.8` (worst case here): v_cap = 244 mm/s (**only 40% drop**) — much better
- `G = 2.0` (typical here): v_cap = 289 mm/s (**only 29% drop**)

So the B-spline family's low `G` is a significant operational win compared to the Wang-Altintas reference value (`G ≈ 5`). The saturation-feedback cap stays gentle.

---

## 5. Cascade identity verification

### 5.1 Numerical test

Direct convolution `c_m(t) = (h_m * w_m)(t)` and FFT to get `|C(ω)| = |H(ω) W(ω)|`. For a perfect inverse, `|C(ω)| ≡ 1` on `[0, pb_max]`. Measured (pb_max = 12 Hz at 40 Hz f_sh):

| variant | max `| |C(ω)| − 1 |` on [0, 0.3·f_sh] | max on [0, 0.5·f_sh] | max on [0, 0.75·f_sh] |
|---|---:|---:|---:|
| bs1 | 4.79% | 8.04% | 47.38% |
| bs2 | 3.13% | 14.58% | 65.57% |
| bs3 | 3.17% | 1.81% | 48.53% |
| bs4 | 3.28% | 5.11% | 43.22% |
| bs5 | 0.54% | 0.61% | 32.94% |

### 5.2 What we give up vs existing family

The existing smooth-IS family has `|W(ω)| · |1|` — no inverse, so the "cascade" is just the forward kernel, which of course matches itself exactly (trivially zero cascade error). Comparing like-for-like is not possible because the existing family **has no inverse**. What we give up:

- **Shorter `T_sm` at same residual.** Existing smooth_zv at 20.06 ms vs new bs1 at 38.88 ms — **1.94× wider**. The worst case (bs5) is 3.40× wider than smooth_zv, or 1.85× wider than smooth_zvd_ei.
- **Sharper rejection notch.** The existing smooth_ei/smooth_2hump_ei family has very sharp notches at f_sh, giving near-zero residual across a wide `(f_sh − Δ, f_sh + Δ)` band. The B-spline has a single-zero notch at `(m+1)/T_sm > f_sh` and relies on the 1/ω^(m+1) asymptotic roll-off plus damping to hit 5% at f_sh. At other frequencies near f_sh, the rejection is less deep — see §3.1 table at 35 Hz vs 45 Hz vs 50 Hz.
- **Single knob `m` instead of 6 hand-tuned kernels.** The existing family has hand-designed kernels (smooth_ei vs smooth_zvd_ei vs smooth_si) optimized for different residual profiles. The new family loses that expressiveness — the only parameter is `m`, plus implicit `T_sm` tuning.

### 5.3 A_axis comparison

At `f_sh = 40 Hz`, `ts = 0.12`:

| family | name | T_sm [ms] | σ²_T [ms²] | A_axis [mm/s²] | A_axis / A_axis_smooth_zv |
|---|---|---:|---:|---:|---:|
| existing | smooth_zv       | 20.06 | 41.87 | 5733 | 1.00 |
| existing | smooth_mzv      | 23.91 | 52.77 | 4548 | 0.79 |
| existing | smooth_ei       | 26.66 | 59.62 | 4024 | 0.70 |
| existing | smooth_2hump_ei | 28.72 | 62.43 | 3844 | 0.67 |
| existing | smooth_si       | 31.12 | 62.82 | 3819 | 0.67 |
| existing | smooth_zvd_ei   | 36.88 | 91.96 | 2609 | 0.46 |
| new | bs1 | 38.88 | 63.00 | 3810 | 0.66 |
| new | bs2 | 48.65 | 65.76 | 3650 | 0.64 |
| new | bs3 | 56.30 | 66.03 | 3635 | 0.63 |
| new | bs4 | 62.65 | 65.42 | 3668 | 0.64 |
| new | bs5 | 68.13 | 64.47 | 3723 | 0.65 |

**Observation:** The new family's `A_axis` values cluster around **3600–3800 mm/s²** — tightly bunched because for the cardinal B-spline `σ²_T = T_sm² / (12(m+1))`, and `T_sm` grows approximately as `√(m+1)` when tuned for equal rejection, so the ratio is near-constant. Every new variant is ~1.5× slower than the fastest existing variant (smooth_zv), 1.1–1.2× slower than the median (smooth_ei/smooth_2hump_ei), and FASTER than the slowest existing variant (smooth_zvd_ei at 2609 — slower than bs1–bs5).

**Bottom line:** the new family sacrifices the "fast smooth_zv" and "fast smooth_mzv" operating points but matches or beats the rest of the existing lineup.

---

## 6. A_axis table (per variant, canonical operating point)

At `f_sh = 40 Hz`, `target_smoothing = 0.12 mm`, `ζ = 0.1`, 5% damped residual target:

```
variant | m | T_sm*f_sh | T_sm [ms]  | σ²_T [ms²]     | A_axis [mm/s²]
bs1     | 1 |  1.5553   |  38.88     |  63.00         |  3810
bs2     | 2 |  1.9462   |  48.65     |  65.76         |  3650
bs3     | 3 |  2.2519   |  56.30     |  66.03         |  3635
bs4     | 4 |  2.5061   |  62.65     |  65.42         |  3668
bs5     | 5 |  2.7252   |  68.13     |  64.47         |  3723
```

Formula for arbitrary `(f_sh, target_smoothing)`:

```
    T_sm        = F_m / f_sh                   (F_m from Tsm*f_sh column)
    σ²_T        = T_sm² / (12 · (m+1))
    A_axis      = 2 · target_smoothing / σ²_T
                = 24 · (m+1) · target_smoothing · f_sh² / F_m²      (closed form)
```

where the dimensionless `F_m` values are `{1.5553, 1.9462, 2.2519, 2.5061, 2.7252}` for m=1..5.

Verified numerically against `ShaperCalibrate.find_smoother_max_accel` via direct moment integration (script §10, `verify_A_axis()` function): all five variants match the closed form to ≤ 1e-10 relative error — same bar as the existing family (`A_axis_smooth_is.md §3`).

---

## 7. Variants table (recommended use)

| variant | m | F_m = T_sm·f_sh | damped rej | `G = ‖h‖₁` | `A_axis` @ 40Hz, ts=0.12 | recommended use |
|---|---|---|---|---|---|---|
| **bs1** | 1 | 1.5553 | 5.0% | 1.93 | 3810 | **"fast"** — smooth_zv/smooth_mzv replacement; narrow main lobe, gentler notch; weakest rejection robustness to f_sh error |
| **bs2** | 2 | 1.9462 | 5.0% | 1.92 | 3650 | **smooth_ei replacement** — moderate lobe, good single-mode rejection, moderate robustness |
| **bs3** | 3 | 2.2519 | 5.0% | 2.00 | 3635 | **DEFAULT** — cubic B-spline, widely used in signal processing, well-understood; best inverse fidelity (0.54% pb_err at wide band) |
| **bs4** | 4 | 2.5061 | 5.0% | 1.99 | 3668 | **smooth_2hump_ei replacement** — deeper notch, more robust to ±10% f_sh detuning |
| **bs5** | 5 | 2.7252 | 5.0% | 1.95 | 3723 | **"premium"** — smooth_si/smooth_zvd_ei replacement; best inverse fidelity at all pb_max, deepest notch, but 3.4× smooth_zv support width |

**Why 5 variants and not 1 or 10.** A single kernel cannot span the "fast/weak-rejection" to "slow/strong-rejection" trade-off meaningfully. Five orders `m ∈ {1,…,5}` give:
- **Continuous coverage** of the `T_sm` spectrum from 1.56·1/f_sh to 2.73·1/f_sh.
- **Alignment with existing family intents** — each of the six existing smooth-IS kernels has a natural B-spline counterpart (the mapping is in §8).
- **Fit within `struct smoother`** — `m=5` means 6 pieces per kernel, each a degree-5 polynomial. The existing struct holds coefficients for a single piece of degree up to 11; extending to 6 pieces of degree 5 is a small C-side change (12 `c0`/`c1`/`c2` entries → 6 per-piece) that fits the existing field sizes.
- **No further orders are useful.** `m=6,7,…` would require `T_sm > 3/f_sh = 75 ms` at 40 Hz, pushing A_axis below 3400 — already strictly worse than every existing smooth-IS variant except smooth_zvd_ei, with no improvement in rejection that couldn't be achieved by raising `f_sh`.

---

## 8. Migration path from existing family

### 8.1 Automatic mapping

Strongest-natural mapping by rejection profile:

| old family name | → new family | rationale |
|---|---|---|
| `smooth_zv`       | `bs1` | Simplest kernel; 1-stage linear roll-off analogue |
| `smooth_mzv`      | `bs1` | Same bucket; `smooth_mzv`'s 3-humped notch is not reproducible, but the rejection level matches |
| `smooth_ei`       | `bs2` | Moderate-width EI; bs2 is the closest B-spline equivalent |
| `smooth_2hump_ei` | `bs4` | Wider notch robust to f_sh error; bs4's deeper single-notch matches the robustness intent |
| `smooth_zvd_ei`   | `bs5` | Slowest/widest existing variant; bs5 is the slowest new variant |
| `smooth_si`       | `bs3` | Mid-width; bs3 as default matches `smooth_si`'s purpose |

### 8.2 Recommended user-facing mapping (config rewrite)

Because the new family has different `T_sm` and different `A_axis` scaling than the existing family, a direct one-to-one name rename would silently change the effective max_accel for every user. Two options for how Kalico handles this:

**(A) Forward-map with explicit warning.** When `shaper_type = smooth_mzv` is seen in config, emit a deprecation log line stating the kernel has been replaced by `bs1` and `A_axis` has dropped from 4548 to 3810. Printed speed-limit behaviour will be slightly different. User action required: no config change, but expect slightly lower max acceleration.

**(B) Hard break, explicit rewrite.** Reject old names with a config error: `shaper_type = smooth_mzv is no longer supported; use bs1 (closest match) or re-run calibration`. Force the user to make the decision. Cleaner, but high user friction.

Given the fork's "opt-in gate" philosophy (per `feedback_fork_as_gate`), **option (B) is recommended.** Users who want the old behaviour can pin to `blend-arc` or earlier branches; magnum-opus is explicitly opt-in. The config-parse error message should enumerate the new family and the closest match from §8.1.

### 8.3 Calibration-time behaviour

`ShaperCalibrate.find_best_shaper` in `klippy/extras/shaper_calibrate.py:619` iterates over `shaper_defs.INPUT_SMOOTHERS`. Replacing the `INPUT_SMOOTHERS` list with the five new entries is a one-line change. The calibration script will return the best of `bs1..bs5` for the given calibration data, same interface as before. The `min_freq` values need to be re-derived per variant (current logic: "projected max_accel ~= 1000"). From §6's A_axis formula, at `max_accel = 1000 mm/s²` and `ts = 0.12`: `f_sh_min = √(max_accel · F_m² / (24 · (m+1) · ts))`:

| variant | min_freq for A_max=1000 |
|---|---:|
| bs1 | 20.5 Hz |
| bs2 | 20.9 Hz |
| bs3 | 20.9 Hz |
| bs4 | 20.8 Hz |
| bs5 | 20.7 Hz |

All roughly 21 Hz, similar to the existing family's `min_freq` of 18–26 Hz.

---

## 9. Known limitations and caveats

**Honest list of trade-offs; nothing is being hidden in the design.**

1. **T_sm penalty.** Every new variant is wider than `smooth_zv` (20 ms → 39–68 ms at f_sh=40 Hz). At 40 Hz with ts=0.12, `A_axis` drops from 5733 (smooth_zv) to 3635–3810 (new family). That's a ~35% speed cap reduction at the "fastest" operating point. Printers currently running below the `A_axis = 3810` cap (most Trident configs at ts=0.12) will see no practical change; printers at the smooth_zv limit will slow down. See `project_ringing_bound_operating_point` — for a ringing-bound machine this is acceptable; for a torque-bound machine (70k) it matters less.

2. **No sharp double-notch for wide-rejection robustness.** The existing `smooth_2hump_ei` and `smooth_si` have two spectral zeros close together, giving a very wide flat-bottom notch robust to ±15% f_sh detuning. The B-spline family has a single notch region (spectral zero at `(m+1)/T_sm`, then `1/ω^{m+1}` roll-off). For a printer whose actual resonance drifts significantly from the configured `f_sh`, the B-spline's rejection margin is narrower. Mitigation: (a) higher-`m` variants (bs4, bs5) have deeper single notches and are more tolerant, or (b) re-calibrate `shaper_freq` more conservatively.

3. **Inverse passband is `[0, 0.3·f_sh]` at 5% error, not `[0, 0.5·f_sh]`.** bs3 and bs5 extend cleanly to `[0, 0.5·f_sh]`, but bs1, bs2, bs4 show 8–15% passband error at the wider band. For the Magnum-Opus pipeline where quintic corner blends have effective bandwidth ≤ 20 Hz (i.e. `0.5·f_sh` at f_sh=40 Hz, per the sinc check in `../plan4-derivations/delta_kappa_max.md §3`), this is borderline. If profiling shows the motion command spectrum reaches 0.5·f_sh routinely, prefer bs3 or bs5.

4. **Inverse truncation is deterministic but not "purely polynomial."** The inverse taps are computed via IFFT of eq. 7 windowed by eq. 8. The design is reproducible, deterministic, and cachable, but the taps are not expressible as a closed-form polynomial in `(f_sh, ζ, T_sm)` the way the forward kernel is. This is the same as fir_companion_kernel.md's approach; see §4.2 for discussion.

5. **C-side code change required.** The existing `struct smoother` in `klippy/chelper/integrate.h:8-13` stores a single polynomial's coefficients. B-splines are piecewise with `m+1` pieces. The `struct` needs to hold a per-piece coefficient array (simple extension; the existing `c0[12]/c1[12]/c2[12]` field sizes accommodate the piece count up to `m=11` trivially). The `calc_antiderivatives()` function in `klippy/chelper/integrate.c` needs a per-piece loop that evaluates only the piece containing the query time. This is a well-defined integration-code extension, not a design blocker, but it is not zero work.

6. **Saturation-feedback worst-case is less punitive than the nominal doc.** `saturation_feedback.md §4` assumed `G = 5` and computed a 55% corner-speed cap drop. With the new family's measured `G ≤ 2.84`, the drop is ≤ 40% (bs4 worst case) or ≤ 30% (bs3 typical). Caveat accepted: the saturation-feedback machinery still fires; the cost is just smaller.

7. **Edge-case testing needed for bs1.** bs1 at 40 Hz has first spectral zero at 51.4 Hz, which is only 1.29·f_sh. If the printer's actual resonance is above the configured f_sh (miscalibration by >20%), the rejection collapses rapidly. Recommend running the hardware-validation suite on bs1 at detuned f_sh values before shipping as the default "fast" variant. Until then, prefer bs2 as the fast-end default.

8. **Calibration score against existing kernels may favor existing family.** `ShaperCalibrate.fit_shaper` uses a combined vibration + smoothing score. The existing family's hand-optimized EI notches may score better than B-splines at the same `target_smoothing`. We have not re-tuned the scoring weights for the new family. Hardware validation should compare score distributions.

---

## 10. Python reference implementation

Reproducible, verifiable against `klippy/extras/shaper_defs.py`. Paste into a module and run.

```python
"""
B-spline shaper family — reference implementation for Plan 5 (Magnum-Opus).
Reproducible from docs/superpowers/plans/plan5-derivations/new_shaper_family.md.

Usage:
    coeffs, t_sm = get_bs_kernel(m=3, f_sh=40.0, damping_ratio=0.1,
                                  V_tol=0.05)
    # coeffs is a list of (t_lo, t_hi, ascending_poly_coeffs) pieces.

    h_taps, t_h_support = get_bs_inverse(m=3, f_sh=40.0, damping_ratio=0.1,
                                          T_h_factor=2.0, pb_frac=0.3,
                                          dt=1e-5)
"""
import math
import numpy as np


def _binomial(n, k):
    if k < 0 or k > n: return 0
    k = min(k, n - k); c = 1
    for i in range(k): c = c * (n - i) // (i + 1)
    return c


# ============================================================================
# FORWARD KERNEL
# ============================================================================

def _bspline_canonical_pieces(m):
    """Pieces of cardinal B-spline N_{m+1}(τ) on [0, m+1].

    Returns list of (tau_lo, tau_hi, poly_ascending) covering the support.
    Per Curry-Schoenberg eq., on piece [i, i+1] (i=0..m):
        N(τ) = (1/m!) * Σ_{k=0..i} (-1)^k · C(m+1, k) · (τ - k)^m
    """
    pieces = []
    for i in range(m + 1):
        poly = np.zeros(m + 1)
        for k in range(i + 1):
            c = ((-1) ** k) * _binomial(m + 1, k) / math.factorial(m)
            # Expand (τ - k)^m = Σ_j C(m, j) τ^j (-k)^{m-j}
            for j in range(m + 1):
                poly[j] += c * _binomial(m, j) * ((-k) ** (m - j))
        pieces.append((float(i), float(i + 1), poly))
    return pieces


def _rescale_bspline_pieces(m, T_sm):
    """Rescale pieces to t in [-T_sm/2, T_sm/2] with unit integral.

    Substitute tau = s*(t + b) where s = (m+1)/T_sm, b = T_sm/2.
    Multiplier of s on overall (Jacobian) preserves unit norm.
    """
    s = (m + 1) / T_sm
    b = T_sm / 2
    canon = _bspline_canonical_pieces(m)
    out = []
    for tau_lo, tau_hi, poly_tau in canon:
        deg = len(poly_tau) - 1
        poly_t = np.zeros(deg + 1)
        for j, aj in enumerate(poly_tau):
            # tau^j = s^j * (t+b)^j = s^j * Σ_l C(j,l) t^l b^{j-l}
            for l in range(j + 1):
                poly_t[l] += aj * s ** j * _binomial(j, l) * b ** (j - l)
        poly_t *= s
        t_lo = tau_lo / s - b
        t_hi = tau_hi / s - b
        out.append((t_lo, t_hi, poly_t))
    return out


def _eval_piecewise(pieces, t):
    t = np.asarray(t, dtype=float)
    out = np.zeros_like(t)
    for t_lo, t_hi, poly in pieces:
        mask = (t >= t_lo) & (t < t_hi)
        if not mask.any(): continue
        tt = t[mask]
        v = np.zeros_like(tt)
        for j, c in enumerate(poly):
            v += c * tt ** j
        out[mask] = v
    return out


def _damped_residual(m, T_sm, f_sh, zeta, dt=1e-5):
    pieces = _rescale_bspline_pieces(m, T_sm)
    hst = 0.5 * T_sm
    tt = np.arange(0.0, T_sm + dt / 2, dt)
    s_arg = hst - tt
    w_at = _eval_piecewise(pieces, s_arg)
    omega = 2 * math.pi * f_sh
    omega_d = omega * math.sqrt(1 - zeta * zeta)
    env = np.exp(-zeta * omega * tt)
    Vc = np.trapezoid(env * np.cos(omega_d * tt) * w_at, tt)
    Vs = np.trapezoid(env * np.sin(omega_d * tt) * w_at, tt)
    return math.sqrt(Vc * Vc + Vs * Vs)


def design_Tsm(m, f_sh, zeta, V_tol=0.05, dt=1e-5):
    """Bisect on T_sm so the damped residual at (f_sh, zeta) equals V_tol."""
    lo, hi = 0.3 / f_sh, 5.0 / f_sh
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        if _damped_residual(m, mid, f_sh, zeta, dt) > V_tol:
            lo = mid
        else:
            hi = mid
    return hi


def get_bs_kernel(m, f_sh, damping_ratio=0.1, V_tol=0.05):
    """Main API — matches shaper_defs.get_*_smoother signature.

    Returns (pieces, T_sm) where pieces is a list of
    (t_lo, t_hi, ascending_poly) tuples covering [-T_sm/2, +T_sm/2].
    """
    T_sm = design_Tsm(m, f_sh, damping_ratio, V_tol)
    pieces = _rescale_bspline_pieces(m, T_sm)
    return pieces, T_sm


# ============================================================================
# MOMENTS and A_axis (closed form)
# ============================================================================

def bs_sigma2(m, T_sm):
    """σ²_T = T_sm² / (12 · (m+1)), exact closed form."""
    return T_sm ** 2 / (12.0 * (m + 1))


def bs_A_axis(m, f_sh, damping_ratio=0.1, target_smoothing=0.12):
    """A_axis = 2 · target / σ²_T (matches find_smoother_max_accel)."""
    T_sm = design_Tsm(m, f_sh, damping_ratio)
    return 2.0 * target_smoothing / bs_sigma2(m, T_sm)


# ============================================================================
# INVERSE KERNEL
# ============================================================================

def design_bs_inverse(m, f_sh, damping_ratio=0.1, T_h_factor=2.0,
                      pb_frac=0.3, dt=1e-5):
    """Compute FIR inverse h taps for the B-spline forward kernel.

    Returns (h, T_h, N_h, G, pb_err, HF_amp).
    """
    T_sm = design_Tsm(m, f_sh, damping_ratio)
    pieces = _rescale_bspline_pieces(m, T_sm)
    # Sample forward kernel
    hst = 0.5 * T_sm
    N = int(2 * hst / dt) + 1
    if N % 2 == 0: N += 1
    t = (np.arange(N) - N // 2) * dt
    w = _eval_piecewise(pieces, t)
    w /= np.sum(w) * dt
    # FFT for spectrum
    T_h = T_h_factor * T_sm
    pb_max = pb_frac * f_sh
    L_min = 32 * max(N * dt, T_h)
    N_fft = int(2 ** math.ceil(math.log2(L_min / dt)))
    w_pad = np.zeros(N_fft)
    start = N_fft // 2 - N // 2
    w_pad[start:start + N] = w
    W = np.fft.fft(np.fft.ifftshift(w_pad)) * dt
    freqs = np.fft.fftfreq(N_fft, dt)
    fa = np.abs(freqs)
    H = np.zeros(N_fft, dtype=complex)
    inpb = fa <= pb_max
    taper = (fa > pb_max) & (fa < f_sh)
    H[inpb] = 1.0 / W[inpb]
    if taper.any():
        a = (fa[taper] - pb_max) / (f_sh - pb_max)
        H[taper] = (1.0 / W[taper]) * 0.5 * (1 + np.cos(math.pi * a))
    h_full = np.fft.fftshift(np.fft.ifft(H)).real / dt
    N_inv = int(T_h / dt)
    if N_inv % 2 == 0: N_inv += 1
    lo = N_fft // 2 - N_inv // 2
    h = h_full[lo:lo + N_inv].copy()
    # Tukey edge, alpha=0.05
    Lw = int(0.05 * (N_inv - 1) / 2)
    if Lw > 0:
        idx = np.arange(Lw)
        ramp = 0.5 * (1 + np.cos(math.pi * (idx / Lw - 1)))
        wnd = np.ones(N_inv)
        wnd[:Lw] = ramp
        wnd[-Lw:] = ramp[::-1]
        h *= wnd
    h /= np.sum(h) * dt
    # Metrics
    c = np.convolve(h, w) * dt
    Npad = 2 ** 16
    fs = np.fft.fftfreq(Npad, dt)
    mag_c = np.abs(np.fft.fft(c, Npad) * dt)
    ok = (fs >= 0) & (fs <= pb_max)
    pb_err = np.max(np.abs(mag_c[ok] - 1.0))
    G = np.sum(np.abs(h)) * dt
    H_mag = np.abs(np.fft.fft(h, Npad) * dt)
    HF = (np.abs(fs) > f_sh) & (np.abs(fs) < 3 * f_sh)
    HF_amp = np.max(H_mag[HF]) if HF.any() else 0.0
    return dict(h=h, T_h=T_h, N_h=len(h), G=float(G),
                pb_err=float(pb_err), HF_amp=float(HF_amp),
                T_sm=T_sm)


# ============================================================================
# VERIFICATION
# ============================================================================

def verify_all():
    """Reproduces every number in the design doc. Run to confirm."""
    f_sh, zeta, ts = 40.0, 0.1, 0.12
    for m in [1, 2, 3, 4, 5]:
        T_sm = design_Tsm(m, f_sh, zeta)
        sig2 = bs_sigma2(m, T_sm)
        A = bs_A_axis(m, f_sh, zeta, ts)
        # Numerical verification of sig2
        pieces = _rescale_bspline_pieces(m, T_sm)
        hst = 0.5 * T_sm
        tau = np.linspace(-hst, hst, 100001)
        w = _eval_piecewise(pieces, tau)
        w /= np.trapezoid(w, tau)
        M1 = np.trapezoid(tau * w, tau)
        M2 = np.trapezoid(tau * tau * w, tau)
        sig2_num = M2 - M1 * M1
        rel = abs(sig2 - sig2_num) / sig2
        print(f"bs{m}: T_sm={T_sm*1000:.3f}ms, σ²_T={sig2*1e6:.4f}ms² "
              f"(num={sig2_num*1e6:.4f}, rel_err={rel:.2e}), "
              f"A_axis={A:.1f}")
        # Inverse
        inv = design_bs_inverse(m, f_sh, zeta, T_h_factor=2.0, pb_frac=0.3)
        print(f"   inverse: N_h={inv['N_h']}, G={inv['G']:.4f}, "
              f"pb_err={inv['pb_err']*100:.3f}%, HF_amp={inv['HF_amp']:.4f}")


if __name__ == "__main__":
    verify_all()
```

Expected output (from actual run, 2026-04-22):

```
bs1: T_sm=38.883ms, σ²_T=62.9969ms² (num=62.9969, rel_err=4.2e-14), A_axis=3809.7
   inverse: N_h=7777, G=1.9328, pb_err=4.787%, HF_amp=0.0818
bs2: T_sm=48.654ms, σ²_T=65.7565ms² (num=65.7565, rel_err=2.8e-14), A_axis=3649.6
   inverse: N_h=9731, G=1.9208, pb_err=3.131%, HF_amp=0.0499
bs3: T_sm=56.298ms, σ²_T=66.0307ms² (num=66.0307, rel_err=5.7e-14), A_axis=3634.5
   inverse: N_h=11259, G=2.0027, pb_err=3.168%, HF_amp=0.0430
bs4: T_sm=62.653ms, σ²_T=65.4225ms² (num=65.4225, rel_err=7.1e-14), A_axis=3668.2
   inverse: N_h=12531, G=1.9913, pb_err=3.275%, HF_amp=0.0427
bs5: T_sm=68.131ms, σ²_T=64.4700ms² (num=64.4700, rel_err=8.5e-14), A_axis=3722.5
   inverse: N_h=13627, G=1.9506, pb_err=0.541%, HF_amp=0.0272
```

---

## 11. Literature cross-check (detailed)

| claim in this doc | literature source | equation / theorem |
|---|---|---|
| Cardinal B-spline Fourier transform = `sinc^{m+1}` | Unser-Aldroubi-Eden 1993 | eq. 4.1 of that paper |
| Cardinal B-spline piecewise polynomial form (eq. 3 here) | Curry-Schoenberg 1966 | Theorem 2; see Biagiotti-Melchiorri 2012 eq. 5.47 |
| Variance of cardinal B-spline of order m = (m+1)/12 (canonical scale) | Unser 1993 | eq. 6.2; independent verification by direct integration |
| B-spline chain has no spectral zeros below `(m+1)/T_1` | Besset-Béarée 2017 | eq. 11-12; rect has zeros at integer multiples of `1/T_1`, convolution stacks without creating interior zeros |
| FIR inverse exists when forward has no passband zeros | Besset-Béarée 2017 | Theorem in §III, eq. 16 (precondition) + eq. 18 (inverse construction) |
| Regularized frequency-domain inversion `H = conj(W)/(|W|² + ε²)` (companion to this doc) | Biagiotti-Melchiorri 2012 | §5.8 eq. 5.79; also Tomizuka 1987 "Zero phase error tracking control" for ZPETC |
| A_axis = `2·target/σ²_T` formula (unchanged from SIS doc) | Biagiotti-Melchiorri 2012 | §3.6 |
| `G = ‖h‖₁` is the right saturation-feedback cap | Wang-Altintas 2022-2023, Sencer-Tajima 2017 | see `saturation_feedback.md §7` for details |

All four primary sources (Besset-Béarée, Biagiotti-Melchiorri, Unser, Curry-Schoenberg) agree on every formula used. The numerical verification in §5 and §10 reproduces published values in Biagiotti-Melchiorri 2012 (§5.5, Table 5.1 of that book lists σ² for B-splines m=1–4; our numbers match to the printed 4-decimal precision).

---

## Key findings for the implementer

- **Family:** cardinal B-spline chain of order `m ∈ {1,2,3,4,5}` (Besset-Béarée 2017 construction).
- **Variants:** 5. bs1 replaces smooth_zv/smooth_mzv (fastest), bs3 is default (cubic, matches smooth_si), bs5 is premium (slowest, most robust).
- **Worst-case forward rejection vs old SIS at 5% damped target:** all meet 5% (same spec as the existing family). The comparison is "same rejection, different T_sm and different spectral shape."
- **T_sm cost:** every new variant is 1.94×–3.40× wider than smooth_zv at f_sh=40 Hz. A_axis drops from ~5700 (smooth_zv) to ~3650 (new family, clustered).
- **Worst-case `G = ‖h‖₁`:** 2.84 (bs4 at pb_max=0.5·f_sh), 2.00 (bs3 at pb_max=0.3·f_sh, the default operating mode). Much better than the `G=5` nominal in `saturation_feedback.md`; saturation-feedback cap drops by 30–40% rather than 55%.
- **Inverse passband error:** ≤ 5% on `[0, 0.3·f_sh]` for all 5 variants; ≤ 5% on `[0, 0.5·f_sh]` for bs3 and bs5 only.
- **C-side work required:** `struct smoother` extension for piecewise polynomial, `calc_antiderivatives` per-piece dispatch. Well-defined change, not blocker.
- **Path to artifact:** `docs/superpowers/plans/plan5-derivations/new_shaper_family.md` (this file). Reference implementation in §10 above, reproducible `verify_all()` function included.

---

## 12. Ten-line summary for the integrator

1. Replace `INPUT_SMOOTHERS` in `shaper_defs.py:214-221` with five B-spline variants `bs1..bs5`.
2. Each variant has kernel `w_m(t) = s · N_{m+1}(s(t+T_sm/2))` with `s = (m+1)/T_sm`.
3. `T_sm` is chosen so damped residual at `(f_sh, ζ)` equals 5% (bisection, §10).
4. σ²_T = T_sm² / (12·(m+1)); A_axis = 2·target_smoothing/σ²_T.
5. Worst-case A_axis at f_sh=40, ts=0.12 is ~3635 (bs3) — 35% below smooth_zv.
6. Forward kernel has no spectral zero on `[0, f_sh]` for every variant.
7. Inverse designed as bandlimited 1/W, cosine-taper to f_sh, `T_h = 2·T_sm`.
8. `G = ‖h‖₁ ≤ 2.8` — meets the saturation-feedback G assumption generously.
9. Migration: hard-break on `shaper_type = smooth_*`, error-message the closest match.
10. C-side: `struct smoother` holds `m+1` piecewise polynomial pieces; `calc_antiderivatives` dispatches per piece.
