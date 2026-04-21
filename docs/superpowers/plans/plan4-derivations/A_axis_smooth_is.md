# A_axis for the Smooth Input Shaper (SIS) family

Derivation: opus math subagent, 2026-04-21.

## 1. Why A_axis is simpler for smooth shapers than for FIR

For an impulse shaper, `find_shaper_max_accel` bisects on the sum-form `_get_shaper_smoothing`, which evaluates a finite pulse train at a 180° velocity cusp. There is no closed form in general.

For a smooth shaper, the kernel `w(τ)` acts as a true low-pass convolution. The cusp test is a pure parabola `x(t) = (a/2) t²`, and convolution with a unit-norm kernel `w(τ)` centered at τ = 0 (after the centroid shift `t_s = M₁/M₀` is subtracted — already part of how these kernels are defined) gives

```
x_sm(0) − x_ideal(0) = (a/2) · σ²_T ,    σ²_T = M₂/M₀ − (M₁/M₀)²
```

where `M_k = ∫ τ^k w(τ) dτ` are the raw polynomial moments of the kernel over `[−T_sm/2, +T_sm/2]`. This is exactly what `_get_smoother_sigma2` already computes. Inverting for the accel whose cusp overshoot hits the `target_smoothing` budget:

```
A_axis  =  2 · target_smoothing / σ²_T
```

**Identical to `ShaperCalibrate.find_smoother_max_accel` (already in the tree at `klippy/extras/shaper_calibrate.py:601`).** There is no frequency-response threshold step analogous to the FIR 5% rule — the cap is derived directly from the geometric cusp test.

## 2. Per-kernel closed forms

Each smoother is stored as coefficient list `C` of a polynomial on `[−T_sm/2, +T_sm/2]`, with `T_sm = F_k / f_sh` (constant `F_k` per kernel, listed in `shaper_defs.py`). Degree is `len(C) − 1`. The second central moment scales as `σ²_T = α_k · T_sm²` with a dimensionless `α_k` that depends only on the kernel's shape. Therefore

```
A_axis = (2·target / α_k) · (f_sh / F_k)²
```

Numerical constants (derived analytically from the stored polynomial coefficients, verified to 1e-10):

| kernel            | degree | F_k (s·Hz) | α_k = σ²/T_sm²  | A_axis / f² at ts=0.12 |
|-------------------|-------:|-----------:|---------------:|-----------------------:|
| smooth_zv         | 4      | 0.80250    | 1.04007e-01    | 3.5831                 |
| smooth_mzv        | 6      | 0.95625    | 9.23253e-02    | 2.8428                 |
| smooth_ei         | 6      | 1.06625    | 8.39409e-02    | 2.5149                 |
| smooth_2hump_ei   | 8      | 1.14875    | 7.56936e-02    | 2.4027                 |
| smooth_zvd_ei     | 8      | 1.47500    | 6.76409e-02    | 1.6309                 |
| smooth_si         | 8      | 1.24500    | 6.48633e-02    | 2.3871                 |

Analytic form of α_k from `M_k = Σ_i C_i · 2·h^{i+k+1} / (i+k+1)` with `h = T_sm/2`, taking only same-parity terms (odd-parity terms over the symmetric interval vanish).

## 3. Verification at f_sh = 40 Hz, ζ = 0.1

Simulation: sample `w(τ)` on 400 001 points over the kernel support, numerically integrate `M₀, M₁, M₂`, recover `σ²_T`, and compare to `ShaperCalibrate.find_smoother_max_accel`:

```
smooth_zv        A_closed=5732.936  A_sim=5732.936  rel_err=4.3e-11
smooth_mzv       A_closed=4548.495  A_sim=4548.495  rel_err=1.2e-10
smooth_ei        A_closed=4023.829  A_sim=4023.829  rel_err=1.3e-10
smooth_2hump_ei  A_closed=3844.333  A_sim=3844.333  rel_err=2.4e-10
smooth_zvd_ei    A_closed=2609.382  A_sim=2609.382  rel_err=2.4e-10
smooth_si        A_closed=3819.384  A_sim=3819.384  rel_err=1.2e-10
```

All six are four orders of magnitude tighter than the 1e-6 bar.

Simulation code (reproducible):

```python
import numpy as np
from klippy.extras import shaper_defs
from klippy.extras.shaper_calibrate import ShaperCalibrate

sc = ShaperCalibrate(printer=None, target_smoothing=0.12)
for name, fn in [('smooth_zv', shaper_defs.get_zv_smoother), ...]:
    C, t_sm = fn(40.0, 0.1, normalize_coeffs=True)
    hst = 0.5 * t_sm
    tau = np.linspace(-hst, hst, 400_001)
    w = sum(c * tau**i for i, c in enumerate(C))
    M0 = np.trapezoid(w, tau)
    M1 = np.trapezoid(tau*w, tau) / M0
    M2 = np.trapezoid(tau*tau*w, tau) / M0
    A_sim = 2 * 0.12 / (M2 - M1*M1)
    A_closed = sc.find_smoother_max_accel((C, t_sm))
    # |A_sim - A_closed| / A_closed < 1e-9
```

## 4. Damping-ratio independence

The SIS kernels in `shaper_defs.py` take `damping_ratio_unused=None` — the kernel shape is fixed. So `A_axis` depends only on `(shaper_type, shaper_freq, target_smoothing)`. `damping_ratio` is accepted by the helper below purely for signature symmetry with the FIR path.

## 5. Drop-in Python for `blendmath.py`

Reuse what's already in the tree (no new derivation needed at the call site):

```python
def _compute_A_axis_smooth_is(shaper_type, shaper_freq, damping_ratio,
                              target_smoothing=0.12):
    """A_axis for a Smooth-IS kernel, in the same units as FIR A_axis.

    Closed-form: A_axis = 2 * target_smoothing / sigma_T^2,
    where sigma_T^2 is the second central moment of the kernel's
    compactly-supported polynomial w(tau).
    """
    from klippy.extras import shaper_defs
    from klippy.extras.shaper_calibrate import ShaperCalibrate

    factory = {s.name: s.init_func for s in shaper_defs.INPUT_SMOOTHERS}
    if shaper_type not in factory or shaper_freq <= 0.0:
        return 0.0
    smoother = factory[shaper_type](shaper_freq, damping_ratio)
    sc = ShaperCalibrate(printer=None, target_smoothing=target_smoothing)
    return float(sc.find_smoother_max_accel(smoother, target_smoothing))
```

For a zero-allocation inner-loop version, cache `(2·target/α_k) / F_k²` per `shaper_type` and return `const · shaper_freq²`.

## 6. Sanity ranges at f_sh = 40 Hz, ts = 0.12

At a representative Trident operating point (40 Hz SIS, target_smoothing = 0.12 mm), expected `A_axis` lies in:

- smooth_zv:       ~5700 mm/s² (fastest, least shaping)
- smooth_mzv:      ~4550 mm/s²
- smooth_ei:       ~4020 mm/s²
- smooth_2hump_ei: ~3840 mm/s²
- smooth_si:       ~3820 mm/s²
- smooth_zvd_ei:   ~2610 mm/s² (slowest, most shaping — widest T_sm)

Order: `zv > mzv > ei > 2hump_ei ≈ si > zvd_ei`, consistent with the kernel-width ordering `F_k`. Use these as the target range in the numerical-verification test: `1e3 < A_axis < 1e4` at 40 Hz, and `A_axis ∝ f_sh²` — doubling to 80 Hz scales every entry by ×4 (~1e4–2.3e4 mm/s²).

## 7. Literature cross-check

Biagiotti & Melchiorri (2012), §3.6, derive the position/velocity smoothing residual of a unit-norm kernel as the second central moment of its impulse response — identical to the `(a/2)·σ²_T` cusp-deflection result used here. Sencer–Tajima (2017) frame the same quantity in frequency-domain terms as a low-frequency Taylor expansion of `W(ω)` around ω=0: `|W(ω)|² ≈ 1 − σ²_T ω² + O(ω⁴)`, from which the quasi-static residual at acceleration step is `(a/2)·σ²_T`. Both support the closed form `A_axis = 2·target/σ²_T` used above.

## Key findings for the implementer

- **The closed-form `A_axis = 2·target_smoothing / σ²_T` is already implemented** for Smooth-IS in `ShaperCalibrate.find_smoother_max_accel` (`klippy/extras/shaper_calibrate.py:601`). No new math is needed — the integration task for `_extract_shapers` is just branch-on-family plus signature parity.
- Kernel degrees are 4, 6, 6, 8, 8, 8. `A_axis` is damping-ratio-independent for SIS (kernels are fixed-shape).
- Numerical verification at (f=40 Hz, ζ=0.1) matches the closed form to ~1e-10 relative error across all six kernels; use `1e-6` as the regression-test bar.
- Expected `A_axis` at 40 Hz / ts=0.12 spans ~2600–5700 mm/s², scaling as `f²`.
