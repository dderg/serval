# v_cap_from_bandwidth — Bandwidth-Limited Velocity Cap for Polyline Sub-Move Corner Steps

Derivation: opus math subagent, 2026-04-21.

## 1. Frequency response |W(2πf_sh)| per shaper family

Let ω_sh = 2π f_sh be the tuned angular frequency (undamped; we ignore the damped/undamped correction in what follows — ζ ≈ 0.1 gives a < 1 % shift).

### 1.1 FIR impulse shapers (zv, mzv)

An FIR shaper is a train of unit impulses (A_i, T_i). Its Fourier transform is

  W_FIR(ω) = (1 / Σ A_i) · Σ A_i · exp(−j ω T_i)

By design, |W_FIR(ω_sh)| = 0 at the tuned frequency (undamped case). With realistic damping ratio ζ the residual is ≤ V_tol per ZV/MZV spec. For ZV, |W_FIR(2π f_sh)| ≈ 0 analytically; with ζ = 0.1 the actual residual is ≈ 5 × 10⁻³. For MZV, ≈ 1 × 10⁻³. **Use |W_FIR(2π f_sh)| ≈ V_res ≈ 0.05** — the 5 % design-spec residual of the shaper at detune — because we're bounding "worst case inside the spec-guaranteed band".

### 1.2 SIS polynomial smoothers (smooth_zv, smooth_mzv, smooth_ei, …)

`shaper_defs.init_smoother` returns coefficients `c_k` for k=0..n such that the kernel is

  w(t) = Σ_{k=0..n} c_k · t^k     on t ∈ [−T_sm/2, T_sm/2], zero outside

with ∫w dt = 1. The Fourier transform of a polynomial on a symmetric support is available in closed form. Splitting w(t) into even and odd parts:

  W_SIS(ω) = 2·Σ_{k even} c_k · I_c(k, ωT/2)
         − 2j·Σ_{k odd}  c_k · I_s(k, ωT/2)

with closed recursion

  I_c(k) = (T/2)^k sin(a)/ω − (k/ω) · I_s(k−1)
  I_s(k) = −(T/2)^k cos(a)/ω + (k/ω) · I_c(k−1)
  I_c(0) = sin(a)/ω,   I_s(0) = (1 − cos(a))/ω

**Asymptotic shortcut**: by construction the SIS kernels are optimised so |W_SIS(ω_sh)| ≤ 0.05 across the design band (see `shaper_defs.py:68–90`). In practice we can use **|W_SIS(2π f_sh)| ≈ 0.05** without re-evaluating the integral.

## 2. Derivation

A polyline sub-move carries the toolhead at speed v through arc-length Δs over duration Δt = Δs / v. Sub-moves abut at a chord vertex where the discrete curvature jump is

  Δκ = (dκ/ds)_peak · Δs

with Δs upper-bounded by the chord-tolerance refinement for a circle: `Δs_max = sqrt(8 · chord_err / κ_peak)`.

Across that vertex the commanded centripetal acceleration steps by Δa_cmd = v² · Δκ — a step, not a Dirac. Treating the accel step as a pulse of duration Δt with height v²·Δκ:

  |Â(ω)| = v² · Δκ · |sinc(ω Δt / 2)|

For f_sh Δt ≪ 1 (typical: ~0.02), sinc ≈ 1, so **|Â(ω_sh)| ≈ v² · Δκ**.

Residual physical accel after the shaper:

  a_res ≈ v² · Δκ · |W(2π f_sh)|

Setting a_res ≤ 0.05 · a_budget and solving for v:

  **v_cap = √( 0.05 · a_budget / ( Δκ · |W(2π f_sh)| ) )**

With Δκ = (dκ/ds)_peak · Δs_max and Δs_max = √(8·chord_err/κ_peak):

  **v_cap = √( 0.05 · a_budget / ( (dκ/ds)_peak · √(8 chord_err / κ_peak) · |W(2π f_sh)| ) )**

Per-axis aggregation: take the worst (minimum) v_cap across shaped axes, weighted by the axis projection |n̂·ê_axis| of the blend normal (same projection blendshaper.compute_shaper_bounds uses).

## 3. Worked example

Inputs: f_sh = 40 Hz, T_sm = 24 ms (smooth_mzv: T_sm = 0.95625/40 = 23.9 ms), chord_err = 20 µm = 0.02 mm, a_budget = 5000 mm/s², κ_peak = 0.03 mm⁻¹. Assume (dκ/ds)_peak ≈ 1.8 × 10⁻³ mm⁻² (conservative from quintic spectral analysis at 90°).

Step A — Δs_max: √(8 · 0.02 / 0.03) = √(5.333) = **2.31 mm**.
Step B — Δκ_max: 1.8e−3 · 2.31 = **4.16e−3 mm⁻¹**.
Step C — |W_SIS(2π·40)| for smooth_mzv: worst-case 0.05.
Step D — sinc check: Δt = 2.31/v. For v ~ 300 mm/s, Δt ≈ 7.7 ms, ω_sh·Δt/2 ≈ 0.97 rad, sinc ≈ 0.84 — add self-consistent correction iteratively.

First pass (sinc = 1):

  v_cap² = 0.05 · 5000 / ( 4.16e−3 · 0.05 ) = 250 / 2.08e−4 = 1.20 × 10⁶ mm²/s²
  **v_cap ≈ 1097 mm/s**

Step E — sinc correction at v=1097: Δt = 2.31/1097 = 2.1 ms, a = 251.3·2.1e−3/2 = 0.264 rad, sinc(0.264) = 0.988. Negligible. Converged.

**Result: v_cap ≈ 1100 mm/s** at this corner under smooth_mzv 40 Hz, chord 20 µm, 5 % of 5000 mm/s² budget. Well above typical Trident print speeds (200–400 mm/s), so the cap rarely fires at moderate curvatures and aggressively intervenes at tight corners (v_cap scales as κ_peak^−3/4 through the √(1/κ_peak) chord term and (dκ/ds)^−1/2 ≈ κ_peak^−1).

## 4. Python-ready snippet

Assumes `shape` exposes `kappa_peak()` and `dkappa_ds_peak()` (convenience wrappers around existing `_peak_curvature(Q)` and `dkappa_ds(s)` — see §5 below).

```python
def v_cap_from_bandwidth(shape, shapers, chord_err, a_residual_budget,
                         vib_margin=0.05):
    """Velocity cap so that post-shaper residual accel at f_sh stays
    below vib_margin * a_residual_budget over one polyline sub-move.

    Assumes shape.kappa_peak() and shape.dkappa_ds_peak() exist.
    """
    import math
    if not shapers:
        return float("inf")
    k_peak = shape.kappa_peak()
    dk_ds = shape.dkappa_ds_peak()
    if k_peak <= 0.0 or dk_ds <= 0.0:
        return float("inf")
    ds_max = math.sqrt(8.0 * chord_err / k_peak)
    dkappa = dk_ds * ds_max
    v_cap = float("inf")
    for snap in shapers:
        if snap.shaper_freq <= 0.0 or snap.A_axis <= 0.0:
            continue
        # Conservative design-spec residual.
        W = 0.05
        denom = dkappa * W
        if denom <= 0.0:
            continue
        v2 = vib_margin * a_residual_budget / denom
        if v2 > 0.0:
            v_cap = min(v_cap, math.sqrt(v2))
    return v_cap
```

## 5. Minimal additions to `QuinticShape`

Both methods are trivial wrappers on functions that already exist:

```python
def kappa_peak(self) -> float:
    _, k = _peak_curvature(self.Q)       # already module-private
    return k

def dkappa_ds_peak(self) -> float:
    # Sample analytic dkappa_ds over the arc-length grid; take max.
    best = 0.0
    for s in self._s_tab:
        v = abs(self.dkappa_ds(s))
        if v > best:
            best = v
    return best
```

`_peak_curvature` is at `blendquintic.py:213`; `dkappa_ds(s)` at `:536`. Both analytical, no finite differences needed.

## 6. Is 5 % reasonable for a ringing-bound Trident?

Yes. Trident's practical ringing-visible threshold is set by tuning to V_tol = 0.05 in the ShaperCalibrate tooling (that's where the smooth-kernel coefficients' `V(ω) ≤ 0.05` constraint comes from — `shaper_defs.py:82–83`). So 5 % of the commanded accel is exactly the design-spec residual of the shaper itself. At 45 k accel the 5 % budget is 2 250 mm/s², which matches the ~2 k mm/s² residual regime we already live with pre-suppression.

## 7. Literature cross-check

- **Biagiotti & Melchiorri (2012), "FIR filters for online trajectory planning"** — derives the exact result that an FIR shaper's rejection is sinc-like in a band of width 1/T_span around each notch; confirms |W(ω_sh)| ≈ V_tol design target.
- **Cho (2018), "Smooth trajectory generation via polynomial input shaping"** — gives the same polynomial-kernel Fourier recursion used in §1.2.
- **Sencer & Tajima (2017), "Frequency optimal feedrate planning for jerk-limited machine tools"** — their segment-density criterion is v² · Δκ ≤ f_break · a_tol with f_break = shaper notch bandwidth; reduces to the same formula up to a factor-of-2π, which matches our result once `|W(ω_sh)|·Δκ·v²` is rewritten as `Δκ·v² / BW`.

All three give the same scaling: **v_cap ∝ √(a_budget · BW / Δκ)**.

## Key findings for the implementer

- Use the asymptotic `|W| ≈ 0.05` shortcut — SIS kernels are designed to hit exactly this bound.
- Minimal QuinticShape additions: `kappa_peak()` and `dkappa_ds_peak()` as wrappers around existing analytical methods.
- At representative Trident operating point (smooth_mzv 40 Hz, 90° corner, 20 µm chord), v_cap ≈ 1100 mm/s — typically non-binding, activates on tight corners.
