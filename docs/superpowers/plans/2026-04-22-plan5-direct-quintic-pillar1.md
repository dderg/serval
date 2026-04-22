# Plan 5 — Direct-quintic + Pillar 1 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Retire the polyline intermediate representation from the quintic corner primitive, land a feedforward inverse input shaper for the new cardinal B-spline shaper family, and make the quintic trapq entry carry a per-s velocity profile — closing the Magnum Opus three-pillar architecture.

**Architecture:** Replace `INPUT_SMOOTHERS` with a cardinal B-spline chain family (`bs1`-`bs5`, Besset-Béarée 2017) whose forward kernel has no passband zeros, enabling a closed-form FIR inverse. Precompute the fused forward⊛inverse kernel (`k_fused = h ⊛ w`) at shaper-reset via least-squares fit to a 9-piece × degree-5 piecewise polynomial stored in an extended `struct smoother`. Extend `struct move` with a tagged-union `MOVE_QUINTIC_POLY_T` variant carrying per-phase (accel/cruise/decel) position-in-t polynomial coefficients composed in Python at emit time. Feed per-s saturation cap `v_sat(s) = sqrt(a_max / (G_worst(s) · κ(s)))` with `G_worst(s) = max_axes G_axis · (|proj_t|+|proj_n|)` into a TOPP velocity-profile solver that absorbs Plan 3's extruder cap. Inverse applies to XY and extruder axes via fused kernel at `kin_shaper.c` / `kin_extruder.c` query layer.

**Tech Stack:** Python 3 (numpy, scipy), C (compiled as `c_helper.so`), ctypes FFI boundary, pytest. Existing Kalico/Klipper conventions.

**Spec reference:** `docs/superpowers/specs/2026-04-22-plan5-direct-quintic-pillar1-design.md`

**Research references:** `docs/superpowers/plans/plan5-derivations/` (8 derivation memos + 6 adversarial reviews).

**Deliverable order** (per MVP-slice recommendation from `REVIEW_3_SCOPE_RISK.md`):

1. D1 — Cardinal B-spline chain family + `struct smoother` piecewise + Path C fused kernel
2. D3 — Feedforward inverse applied at `kin_shaper.c` / `kin_extruder.c`
3. D4 — Saturation cap (depends on D1's G values, D3's `AxisShaperSnapshot.inverse_G`)
4. D5 — Lookahead window extension (depends on D1's `T_sm`, D3's `T_h`)
5. D6 — Config migration (parallel with above)
6. D2 — Direct-quintic step generation (tagged union, 11 moments)
7. D7 — Unified v(s) along the curve (TOPP + trapezoid-in-s)

After D1-D6, the new shaper family with feedforward inverse runs on the existing polyline path — a CI-testable fallback ship point if trapq surgery (D2+D7) hits unexpected complexity.

---

## File Structure

**New files:**
- `klippy/chelper/integrate.h` — `struct smoother` extended to piecewise form (replaces current single-polynomial representation).
- `test/test_bspline_family.py` — forward + inverse + fused kernel tests for `bs1`-`bs5`.
- `test/test_topp.py` — TOPP dense-grid solver tests.
- `test/test_saturation_cap.py` — D4 `G_worst(s)` integration tests.
- `test/test_fused_kernel_fit.py` — Path C least-squares fit validation.

**Modified C files (only these — other kin_*.c files do not need changes per `REVIEW_3_C_INTEGRATION.md`):**
- `klippy/chelper/trapq.h` — tagged union in `struct move` (D2b).
- `klippy/chelper/trapq.c` — `move_get_coord`, `move_get_distance` kind dispatch; new `trapq_append_quintic` entry point.
- `klippy/chelper/integrate.h` / `integrate.c` — `struct smoother` piecewise, `calc_antiderivatives` 11 moments + phase dispatch (D2a).
- `klippy/chelper/kin_shaper.c` — fused kernel application (D3), lookahead extension (D5).
- `klippy/chelper/kin_extruder.c` — fused kernel on extruder axis (D3), `extruder_set_smoothing_params` FFI signature update.
- `klippy/chelper/itersolve.c` — `check_active` kind dispatch (D2b critical fix).

**Modified Python files:**
- `klippy/extras/shaper_defs.py` — replace `INPUT_SMOOTHERS` with bs1-bs5 (D1).
- `klippy/extras/shaper_calibrate.py` — AUTOTUNE_SHAPERS + `find_smoother_max_accel` updated for piecewise (D1 + D6).
- `klippy/extras/input_shaper.py` — compute inverse `h` + fit `k_fused`; FFI signature change; config validation for retired `smooth_*` names (D1 + D3 + D6).
- `klippy/extras/motion_report.py` — `kind` field in websocket payload (D6).
- `klippy/extras/tap_analysis.py` — skip/handle quintic-kind moves (D6).
- `klippy/blendshaper.py` — `AxisShaperSnapshot.inverse_G` field (D4).
- `klippy/blendmath.py` — `_extract_shapers` populates G (D4).
- `klippy/blendquintic.py` — `v_cap_fn(s)` composes 5 cap sources; `v_cap_min()` helper; `compute_topp_profile` (D4 + D7).
- `klippy/blendplanner.py` — `CornerBlender._emit_blend` emits quintic trapq entry (D2c + D7).
- `klippy/blendextruder.py` — `cap_move` retires; logic migrates to `v_extr(s)` contribution (D7).
- `klippy/toolhead.py` — no structural changes expected (lookahead handled through existing `note_step_generation_scan_time`).

**External repo coordination:**
- `~/Developer/klipper-sim/` — Python `Move` shim updated to handle quintic kind (same commit batch as D2b per spec).

---

## D1 — Cardinal B-spline chain shaper family (+ `struct smoother` piecewise + Path C fused kernel)

**Scope:** New forward kernel `bs1`-`bs5`, FIR inverse `h` per variant (Python), Path C fused kernel fit (9 pieces × degree 5), piecewise `struct smoother` in C, updated A_axis computation, updated FFI signature.

**Key references:**
- Spec §D1 (forward kernel table, A_axis table)
- `new_shaper_family.md` — coefficient derivation + §10 reproducer
- `fused_kernel_storage_resolution.md` — Path C algorithm

### Task 1: B-spline forward kernel generation in Python

**Files:**
- Modify: `klippy/extras/shaper_defs.py`
- Test: `test/test_bspline_family.py` (new)

- [ ] **Step 1: Write failing test — kernel integrates to unity**

```python
# test/test_bspline_family.py
import numpy as np
import pytest
from klippy.extras import shaper_defs

@pytest.mark.parametrize("m", [1, 2, 3, 4, 5])
def test_bspline_kernel_unit_integral(m):
    """B-spline of order m, sampled, integrates to 1."""
    f_sh = 40.0
    damping_ratio = 0.1
    C, t_sm = shaper_defs.INPUT_SMOOTHERS[m-1].init_func(
        f_sh, damping_ratio, normalize_coeffs=True)
    # Sample the piecewise polynomial on a dense grid
    grid = np.linspace(-t_sm/2, t_sm/2, 100001)
    w = shaper_defs.bspline_eval(C, grid, t_sm)  # helper to be written
    integral = np.trapezoid(w, grid)
    assert abs(integral - 1.0) < 1e-6
```

- [ ] **Step 2: Write failing test — kernel is even**

```python
def test_bspline_kernel_even(m):
    f_sh = 40.0
    C, t_sm = shaper_defs.INPUT_SMOOTHERS[m-1].init_func(f_sh, 0.1, True)
    grid = np.linspace(-t_sm/2, t_sm/2, 1001)
    w = shaper_defs.bspline_eval(C, grid, t_sm)
    assert np.allclose(w, w[::-1], rtol=1e-9)
```

- [ ] **Step 3: Run tests to verify they fail**

Run: `python3 -m pytest test/test_bspline_family.py -v`
Expected: FAIL on import or missing function.

- [ ] **Step 4: Implement `_get_bs1_smoother` through `_get_bs5_smoother`**

For each variant `m ∈ {1..5}`:
1. Solve for `T_sm` such that damped residual `V(f_sh, ζ) = 0.05` using `shaper_defs._bisect_t_sm_for_bspline(m, f_sh, ζ, V_target=0.05)`.
2. Build piecewise polynomial: cardinal B-spline of order `m` is the `(m+1)`-fold self-convolution of a unit rectangle, rescaled to `[-T_sm/2, +T_sm/2]`.
3. Return `(C, t_sm)` where `C` is a list of `m+1` piece-descriptor tuples `(t_start, t_end, coeffs)` and `t_sm` is the full support.

Use Curry-Schoenberg 1966 divided-difference form for closed-form piece coefficients — see `new_shaper_family.md §2.1`.

```python
# klippy/extras/shaper_defs.py
def _cardinal_bspline_pieces(m):
    """Return list of (t_start_canonical, t_end_canonical, coeffs) for the
    cardinal B-spline of order m on canonical support [0, m+1]."""
    pieces = []
    fac_m = float(math.factorial(m))
    for i in range(m + 1):  # sub-interval [i, i+1]
        # N_{m+1}(τ) = (1/m!) Σ_{k=0..i} (-1)^k C(m+1, k) (τ - k)^m on [i, i+1]
        # Expand (τ - k)^m in τ and collect coefficients.
        coeffs = [0.0] * (m + 1)
        for k in range(i + 1):
            sign = (-1) ** k
            binom = math.comb(m + 1, k)
            # Expand (τ - k)^m = Σ_j C(m,j) τ^j (-k)^(m-j)
            for j in range(m + 1):
                coeffs[j] += (sign * binom * math.comb(m, j)
                              * ((-k) ** (m - j))) / fac_m
        pieces.append((float(i), float(i + 1), coeffs))
    return pieces

def _get_bs_smoother(m, f_sh, damping_ratio, normalize_coeffs):
    F_m = _F_m_table[m]  # dimensionless T_sm · f_sh from pre-computed constants
    t_sm = F_m / f_sh
    pieces = _cardinal_bspline_pieces(m)
    # Rescale from canonical [0, m+1] to [-t_sm/2, +t_sm/2], preserve unit integral
    s = (m + 1) / t_sm
    shift = -t_sm / 2
    rescaled = []
    for (a, b, coeffs) in pieces:
        # τ_canonical = s · (t - shift) ; substitute and re-expand
        new_coeffs = _affine_substitute(coeffs, s, -s * shift)
        new_coeffs = [c * s for c in new_coeffs]  # Jacobian
        rescaled.append((a / s + shift, b / s + shift, new_coeffs))
    return (rescaled, t_sm)

# Pre-computed F_m constants (dimensionless T_sm · f_sh at ζ=0.1, V=0.05)
_F_m_table = {1: 1.5553, 2: 1.9462, 3: 2.2519, 4: 2.5061, 5: 2.7252}

def _get_bs1_smoother(f, z, n): return _get_bs_smoother(1, f, z, n)
def _get_bs2_smoother(f, z, n): return _get_bs_smoother(2, f, z, n)
def _get_bs3_smoother(f, z, n): return _get_bs_smoother(3, f, z, n)
def _get_bs4_smoother(f, z, n): return _get_bs_smoother(4, f, z, n)
def _get_bs5_smoother(f, z, n): return _get_bs_smoother(5, f, z, n)
```

Also implement helper `bspline_eval(C, grid, t_sm)` that evaluates the piecewise polynomial.

- [ ] **Step 5: Replace `INPUT_SMOOTHERS` table**

```python
# klippy/extras/shaper_defs.py (replacing current lines 214-221)
InputSmoother = namedtuple('InputSmoother', ['name', 'init_func'])

INPUT_SMOOTHERS = [
    InputSmoother('bs1', _get_bs1_smoother),
    InputSmoother('bs2', _get_bs2_smoother),
    InputSmoother('bs3', _get_bs3_smoother),
    InputSmoother('bs4', _get_bs4_smoother),
    InputSmoother('bs5', _get_bs5_smoother),
]
```

- [ ] **Step 6: Run tests to verify they pass**

Run: `python3 -m pytest test/test_bspline_family.py -v -k "unit_integral or even"`
Expected: 10 PASS (5 variants × 2 tests).

- [ ] **Step 7: Commit**

```bash
git add klippy/extras/shaper_defs.py test/test_bspline_family.py
git commit -m "shaper_defs: cardinal B-spline chain family (bs1-bs5)

Replaces smooth_zv / smooth_mzv / smooth_ei / smooth_2hump_ei /
smooth_zvd_ei / smooth_si with cardinal B-spline chains of order m=1..5.
Besset-Béarée 2017 construction; closed-form Curry-Schoenberg piece
coefficients; F_m constants pre-computed at zeta=0.1, V=0.05.
"
```

### Task 2: Spectrum tests — sinc^{m+1} and first-zero location

**Files:**
- Test: `test/test_bspline_family.py` (add to existing)

- [ ] **Step 1: Write test for Fourier transform matching sinc^{m+1}**

```python
@pytest.mark.parametrize("m", [1, 2, 3, 4, 5])
def test_bspline_spectrum_is_sinc_power(m):
    f_sh = 40.0
    C, t_sm = shaper_defs.INPUT_SMOOTHERS[m-1].init_func(f_sh, 0.1, True)
    T_1 = t_sm / (m + 1)  # per-stage box width
    grid = np.linspace(-t_sm/2, t_sm/2, 10001)
    w = shaper_defs.bspline_eval(C, grid, t_sm)
    # Evaluate Fourier transform at f = 5, 15, 25 Hz
    for f in [5.0, 15.0, 25.0]:
        omega = 2 * np.pi * f
        W_numeric = np.trapezoid(w * np.exp(-1j * omega * grid), grid)
        W_expected = np.sinc(f * T_1) ** (m + 1)  # numpy's sinc = sin(pi x)/(pi x)
        assert abs(W_numeric.real - W_expected) < 1e-4
        assert abs(W_numeric.imag) < 1e-4
```

- [ ] **Step 2: Write test for first-zero location**

```python
@pytest.mark.parametrize("m,expected_first_zero_hz", [
    (1, 51.44), (2, 61.66), (3, 71.05), (4, 79.81), (5, 88.07),
])
def test_bspline_first_spectral_zero(m, expected_first_zero_hz):
    f_sh = 40.0
    C, t_sm = shaper_defs.INPUT_SMOOTHERS[m-1].init_func(f_sh, 0.1, True)
    # First zero of sinc^(m+1) at f = (m+1)/t_sm (Hz), since first zero of
    # numpy-style sinc(x) is at x=1.
    computed_first_zero = (m + 1) / t_sm
    assert abs(computed_first_zero - expected_first_zero_hz) < 0.1
    # All zeros should lie above f_sh (precondition for FIR invertibility)
    assert computed_first_zero > f_sh * 1.25
```

- [ ] **Step 3: Run tests**

Run: `python3 -m pytest test/test_bspline_family.py -v`
Expected: PASS (10 earlier + 10 new).

- [ ] **Step 4: Commit**

```bash
git add test/test_bspline_family.py
git commit -m "test: bspline spectrum matches sinc^(m+1), first zero above f_sh"
```

### Task 3: A_axis computation for piecewise kernels

**Files:**
- Modify: `klippy/extras/shaper_calibrate.py`
- Test: `test/test_bspline_family.py`

- [ ] **Step 1: Write test — A_axis matches closed form**

```python
@pytest.mark.parametrize("m,expected_A_axis", [
    (1, 3810), (2, 3650), (3, 3635), (4, 3668), (5, 3723),
])
def test_bspline_A_axis_matches_table(m, expected_A_axis):
    """At f_sh=40Hz, target_smoothing=0.12, A_axis = 2*ts/σ²_T,
    σ²_T = T_sm²/(12·(m+1)) for cardinal B-spline."""
    f_sh = 40.0
    target_smoothing = 0.12
    smoother = shaper_defs.INPUT_SMOOTHERS[m-1].init_func(f_sh, 0.1, True)
    from klippy.extras.shaper_calibrate import ShaperCalibrate
    sc = ShaperCalibrate(printer=None, target_smoothing=target_smoothing)
    A_axis = sc.find_smoother_max_accel(smoother, target_smoothing)
    assert abs(A_axis - expected_A_axis) / expected_A_axis < 0.01
```

- [ ] **Step 2: Run — failing because `find_smoother_max_accel` assumes single-polynomial kernel**

Run: `python3 -m pytest test/test_bspline_family.py::test_bspline_A_axis_matches_table -v`
Expected: FAIL (needs piecewise handling).

- [ ] **Step 3: Update `find_smoother_max_accel` to handle piecewise**

```python
# klippy/extras/shaper_calibrate.py
def find_smoother_max_accel(self, smoother, target_smoothing=None):
    """A_axis = 2 * target / σ²_T where σ²_T = ∫ τ² w(τ) dτ.
    Piecewise-polynomial kernel: sum integrals per piece."""
    C_pieces, t_sm = smoother
    target = target_smoothing or self.target_smoothing
    # σ²_T = Σ_pieces ∫_{t_start}^{t_end} τ² · Σ_k c_k τ^k dτ
    sigma_T_squared = 0.0
    for (t_start, t_end, coeffs) in C_pieces:
        for k, c in enumerate(coeffs):
            # ∫ τ² · c · τ^k dτ = c · (τ^{k+3} / (k+3))
            sigma_T_squared += (c * (t_end**(k+3) - t_start**(k+3))
                                / (k + 3))
    return 2.0 * target / sigma_T_squared
```

- [ ] **Step 4: Run — PASS**

Run: `python3 -m pytest test/test_bspline_family.py::test_bspline_A_axis_matches_table -v`
Expected: 5 PASS.

- [ ] **Step 5: Commit**

```bash
git add klippy/extras/shaper_calibrate.py test/test_bspline_family.py
git commit -m "shaper_calibrate: A_axis for piecewise-polynomial smoothers

Extends find_smoother_max_accel to compute σ²_T = Σ ∫τ²w(τ)dτ
piece-by-piece. Matches closed-form σ²_T = T_sm²/(12·(m+1)) for
cardinal B-splines to within 1% across bs1..bs5."
```

### Task 4: FIR inverse kernel computation (Python)

**Files:**
- Create: `klippy/extras/bspline_inverse.py`
- Test: `test/test_bspline_family.py`

- [ ] **Step 1: Write failing test — G = ‖h‖₁ matches spec table**

```python
@pytest.mark.parametrize("m,expected_G", [
    (1, 1.933), (2, 1.921), (3, 2.003), (4, 1.991), (5, 1.951),
])
def test_bspline_inverse_G_matches_table(m, expected_G):
    """Reference values at f_sh=40Hz, pb_max=12Hz, T_h=2·T_sm, dt=10μs,
    Tukey α=0.25."""
    f_sh = 40.0
    damping = 0.1
    from klippy.extras import bspline_inverse
    C, t_sm = shaper_defs.INPUT_SMOOTHERS[m-1].init_func(f_sh, damping, True)
    h_taps, t_h, dt = bspline_inverse.compute_inverse_fir(
        C, t_sm, pb_max_hz=0.3*f_sh, dt=1e-5)
    G = np.sum(np.abs(h_taps)) * dt
    assert abs(G - expected_G) < 0.05
```

- [ ] **Step 2: Run — FAIL (module doesn't exist)**

- [ ] **Step 3: Implement `compute_inverse_fir`**

```python
# klippy/extras/bspline_inverse.py
import numpy as np
from . import shaper_defs

def compute_inverse_fir(C_pieces, t_sm, pb_max_hz, dt=1e-5,
                        T_h_ratio=2.0, tukey_alpha=0.25,
                        eps_rel=3e-3):
    """Compute finite-support FIR inverse of a piecewise-polynomial kernel.
    Returns (h_taps, t_h, dt)."""
    # Sample forward kernel w on [-T_sm/2, +T_sm/2]
    n_w = int(np.ceil(t_sm / dt))
    if n_w % 2 == 0:
        n_w += 1
    t_w = (np.arange(n_w) - n_w // 2) * dt
    w = shaper_defs.bspline_eval(C_pieces, t_w, t_sm)
    # FFT with zero padding
    T_h = T_h_ratio * t_sm
    L = 8 * max(t_sm, T_h)
    N_fft = int(2 ** np.ceil(np.log2(L / dt)))
    w_pad = np.zeros(N_fft)
    start = N_fft // 2 - n_w // 2
    w_pad[start:start + n_w] = w
    W = np.fft.fft(np.fft.ifftshift(w_pad)) * dt
    # Tikhonov-regularized inverse
    eps = eps_rel * np.max(np.abs(W))
    H = np.conj(W) / (np.abs(W) ** 2 + eps ** 2)
    h_full = np.fft.fftshift(np.fft.ifft(H)).real / dt
    # Truncate to T_h
    n_h = int(np.round(T_h / dt))
    if n_h % 2 == 0:
        n_h += 1
    lo = N_fft // 2 - n_h // 2
    h = h_full[lo:lo + n_h].copy()
    # Tukey window
    h *= _tukey_window(n_h, tukey_alpha)
    # Renormalize so ∫h(τ)dτ = 1
    h /= np.sum(h) * dt
    return h, T_h, dt

def _tukey_window(n, alpha):
    if alpha <= 0:
        return np.ones(n)
    L = int(alpha * (n - 1) / 2)
    if L == 0:
        return np.ones(n)
    window = np.ones(n)
    ramp = 0.5 * (1 + np.cos(np.pi * (np.arange(L) / L - 1)))
    window[:L] = ramp
    window[n - L:] = ramp[::-1]
    return window
```

- [ ] **Step 4: Run — PASS**

Run: `python3 -m pytest test/test_bspline_family.py::test_bspline_inverse_G_matches_table -v`
Expected: 5 PASS.

- [ ] **Step 5: Commit**

```bash
git add klippy/extras/bspline_inverse.py test/test_bspline_family.py
git commit -m "bspline_inverse: FIR companion kernel via regularized IFFT

Computes h(τ) = IFFT(conj(W)/(|W|²+ε²)) with Tukey windowing at
T_h = 2·T_sm. Matches spec G = ‖h‖₁ table for bs1-bs5 at pb_max=0.3·f_sh."
```

### Task 5: Path C — fused kernel fit to 9 × degree-5 pieces

**Files:**
- Modify: `klippy/extras/bspline_inverse.py`
- Test: `test/test_fused_kernel_fit.py` (new)

- [ ] **Step 1: Write failing test — fit passband error matches exact FIR**

```python
# test/test_fused_kernel_fit.py
import numpy as np
import pytest
from klippy.extras import shaper_defs, bspline_inverse

@pytest.mark.parametrize("m,max_pb_err", [
    (1, 0.05), (2, 0.04), (3, 0.04), (4, 0.04), (5, 0.01),
])
def test_fused_kernel_fit_matches_fir_cascade(m, max_pb_err):
    """9 × degree-5 LSQ fit to k_fused has passband error within ~1% of
    exact FIR cascade."""
    f_sh = 40.0
    C, t_sm = shaper_defs.INPUT_SMOOTHERS[m-1].init_func(f_sh, 0.1, True)
    h_taps, T_h, dt = bspline_inverse.compute_inverse_fir(
        C, t_sm, pb_max_hz=0.3*f_sh, dt=1e-5)
    # Compute k_fused as numerical convolution
    t_w = np.arange(len(shaper_defs.bspline_eval(C, np.array([0]), t_sm))) * dt
    # ... (actually compute k_fused and fit to 9 pieces × degree 5)
    k_fused_pieces = bspline_inverse.fit_fused_kernel(
        C, t_sm, h_taps, T_h, dt, n_pieces=9, degree=5)
    # Cascade passband error: |FT(k_fused) - 1| on [0, 0.3·f_sh]
    pb_err = bspline_inverse.cascade_passband_error(
        k_fused_pieces, pb_max_hz=0.3*f_sh)
    assert pb_err < max_pb_err
```

- [ ] **Step 2: Run — FAIL**

- [ ] **Step 3: Implement `fit_fused_kernel` and `cascade_passband_error`**

```python
# klippy/extras/bspline_inverse.py (additions)

def fit_fused_kernel(C_w, t_sm, h_taps, t_h, dt, n_pieces=9, degree=5):
    """Fit k_fused = h ⊛ w to n_pieces equal-width degree-N polynomial.
    Returns list of (t_start, t_end, coeffs) pieces.
    """
    # Compute k_fused on dense grid via numerical convolution
    t_fused = t_sm + t_h
    n_fused = int(np.ceil(t_fused / dt)) + 1
    if n_fused % 2 == 0:
        n_fused += 1
    grid = (np.arange(n_fused) - n_fused // 2) * dt
    w = shaper_defs.bspline_eval(C_w, grid, t_sm)
    k = np.convolve(h_taps, w, mode='same') * dt
    # Least-squares fit to n_pieces equal-width degree-N pieces
    piece_width = t_fused / n_pieces
    pieces = []
    for i in range(n_pieces):
        t_start = -t_fused / 2 + i * piece_width
        t_end = t_start + piece_width
        mask = (grid >= t_start) & (grid < t_end) if i < n_pieces - 1 else \
               (grid >= t_start) & (grid <= t_end)
        t_local = grid[mask]
        k_local = k[mask]
        # Polynomial fit in original coordinates
        coeffs = np.polynomial.polynomial.polyfit(t_local, k_local, degree)
        pieces.append((t_start, t_end, coeffs.tolist()))
    return pieces

def cascade_passband_error(pieces, pb_max_hz, n_eval=500):
    """|∫ k(τ) e^{-iωτ} dτ - 1| over [0, pb_max_hz]."""
    freqs = np.linspace(0.1, pb_max_hz, n_eval)
    max_err = 0.0
    for f in freqs:
        omega = 2 * np.pi * f
        K = 0.0 + 0j
        for (t_start, t_end, coeffs) in pieces:
            for k, c in enumerate(coeffs):
                # ∫_{t_start}^{t_end} c · τ^k · e^{-iωτ} dτ
                K += c * _integrate_polynomial_times_exp(
                    k, t_start, t_end, omega)
        err = abs(abs(K) - 1.0)
        if err > max_err:
            max_err = err
    return max_err

def _integrate_polynomial_times_exp(k, t_start, t_end, omega):
    """∫_{a}^{b} τ^k e^{-iωτ} dτ via integration by parts recursion."""
    if omega == 0:
        return (t_end ** (k + 1) - t_start ** (k + 1)) / (k + 1)
    if k == 0:
        return (np.exp(-1j * omega * t_start) - np.exp(-1j * omega * t_end)) \
               / (1j * omega)
    # Recursion: ∫τ^k e dτ = [τ^k · (-1/iω) e] - (k/-iω) ∫τ^{k-1} e dτ
    boundary = (t_end ** k * np.exp(-1j * omega * t_end)
                - t_start ** k * np.exp(-1j * omega * t_start)) / (-1j * omega)
    return boundary + (k / (1j * omega)) * \
           _integrate_polynomial_times_exp(k - 1, t_start, t_end, omega)
```

- [ ] **Step 4: Run — PASS**

Run: `python3 -m pytest test/test_fused_kernel_fit.py -v`
Expected: 5 PASS.

- [ ] **Step 5: Commit**

```bash
git add klippy/extras/bspline_inverse.py test/test_fused_kernel_fit.py
git commit -m "bspline_inverse: Path C fused kernel fit to 9 × degree-5 pieces

Numerically convolves FIR h with piecewise w, least-squares fits the
result to 9 equal-width degree-5 polynomial pieces. Passband error
within 1% of exact FIR cascade across bs1-bs5 per fused_kernel_storage_resolution.md."
```

### Task 6: C-side `struct smoother` piecewise redesign

**Files:**
- Modify: `klippy/chelper/integrate.h`
- Modify: `klippy/chelper/integrate.c`
- Test: C-level via Python FFI round-trip

- [ ] **Step 1: Write failing test — piecewise kernel eval matches Python**

```python
# test/test_bspline_family.py (add)
def test_c_side_piecewise_kernel_matches_python():
    """C-side smoother eval at sample points matches Python closed-form."""
    f_sh = 40.0
    m = 3
    C, t_sm = shaper_defs.INPUT_SMOOTHERS[m-1].init_func(f_sh, 0.1, True)
    # Call into C via chelper to eval kernel at 50 sample points
    from klippy import chelper
    ffi_main, ffi_lib = chelper.get_ffi()
    smoother_c = _marshal_smoother_to_c(C, t_sm, ffi_main, ffi_lib)
    samples_t = np.linspace(-t_sm/2, t_sm/2, 50)
    c_values = [ffi_lib.smoother_eval(smoother_c, t) for t in samples_t]
    py_values = shaper_defs.bspline_eval(C, samples_t, t_sm)
    assert np.allclose(c_values, py_values, rtol=1e-9)
```

- [ ] **Step 2: Run — FAIL (smoother_eval does not exist yet)**

- [ ] **Step 3: Replace `struct smoother` with piecewise form**

```c
// klippy/chelper/integrate.h
#define SMOOTHER_MAX_PIECES 9
#define SMOOTHER_MAX_DEGREE 5

struct smoother_piece {
    double coeffs[SMOOTHER_MAX_DEGREE + 1];   // c_0 … c_5
    double t_start, t_end;
    // Precomputed antiderivative endpoints for fast range_integrate:
    // F(t) = Σ_k (c_k / (k+1)) · t^(k+1), then F(t_end), F(t_start), etc.
    struct calc_antiderivatives m_start, m_end;
};

struct calc_antiderivatives {
    // 11 moments (m_0 … m_10) — degree-10 support (D2a extension).
    // For D1 linear-move consumers, m_6..m_10 stay zero.
    double m[11];
};

struct smoother {
    int n_pieces;
    struct smoother_piece pieces[SMOOTHER_MAX_PIECES];
    double t_sm;      // full support width
    double t_offs;    // centroid shift (unchanged from prior layout)
};

double smoother_eval(const struct smoother *sm, double t);
```

- [ ] **Step 4: Implement `smoother_eval` and `calc_antiderivatives` for piecewise**

```c
// klippy/chelper/integrate.c
double smoother_eval(const struct smoother *sm, double t) {
    // Linear scan through at most SMOOTHER_MAX_PIECES = 9 pieces.
    // Faster than binary search for this count.
    for (int i = 0; i < sm->n_pieces; i++) {
        const struct smoother_piece *p = &sm->pieces[i];
        if (t >= p->t_start && t <= p->t_end) {
            // Horner form evaluation
            double val = p->coeffs[SMOOTHER_MAX_DEGREE];
            for (int k = SMOOTHER_MAX_DEGREE - 1; k >= 0; k--)
                val = val * t + p->coeffs[k];
            return val;
        }
    }
    return 0.0; // outside support
}
```

- [ ] **Step 5: Run — PASS**

Run: `python3 -m pytest test/test_bspline_family.py::test_c_side_piecewise_kernel_matches_python -v`
Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add klippy/chelper/integrate.h klippy/chelper/integrate.c test/test_bspline_family.py
git commit -m "integrate: struct smoother piecewise form, 9 pieces × degree 5

Replaces single-polynomial struct smoother with piecewise form supporting
up to 9 equal-width pieces of degree ≤ 5, plus 11-moment antiderivative
cache per piece (future D2a degree-10 moves). smoother_eval uses linear
piece-scan (faster than binary search for 9 pieces) and Horner evaluation."
```

### Task 7: FFI signature update — `input_shaper_set_smoother_params`

**Files:**
- Modify: `klippy/chelper/kin_shaper.c`
- Modify: `klippy/extras/input_shaper.py`

- [ ] **Step 1: Write failing test — SET_INPUT_SHAPER doesn't crash**

```python
# test/test_input_shaper_bs.py (new)
def test_set_input_shaper_bs3_round_trip():
    """Loading bs3 config, running SET_INPUT_SHAPER, doesn't crash."""
    from klippy.test_helpers import make_printer, run_gcode
    printer = make_printer(config_snippet="""
        [input_shaper]
        shaper_type = bs3
        shaper_freq_x = 40
        shaper_freq_y = 40
        damping_ratio_x = 0.1
        damping_ratio_y = 0.1
        target_smoothing = 0.12
    """)
    run_gcode(printer, 'SET_INPUT_SHAPER SHAPER_TYPE_X=bs2 SHAPER_FREQ_X=35')
```

- [ ] **Step 2: Run — FAIL (shaper_type=bs3 rejected by current validation)**

- [ ] **Step 3: Update FFI and Python-side marshalling**

```c
// klippy/chelper/kin_shaper.c (replacing current signature)
int32_t input_shaper_set_smoother_params(
    struct stepper_kinematics *sk, char axis,
    int n_pieces,
    const double *piece_buf,  // flat: n_pieces × 8 doubles
                              // (t_start, t_end, c0..c5) per piece
    double t_sm)
{
    // Parse flat buffer into struct smoother
    struct input_shaper *is = container_of(sk, struct input_shaper, sk);
    struct smoother *sm = (axis == 'x') ? &is->sm_x : &is->sm_y;
    sm->n_pieces = n_pieces;
    sm->t_sm = t_sm;
    for (int i = 0; i < n_pieces; i++) {
        const double *src = piece_buf + i * 8;
        sm->pieces[i].t_start = src[0];
        sm->pieces[i].t_end = src[1];
        for (int k = 0; k < 6; k++)
            sm->pieces[i].coeffs[k] = src[2 + k];
        // Precompute antiderivative endpoints
        _compute_antiderivative_endpoints(&sm->pieces[i]);
    }
    return 0;
}
```

```python
# klippy/extras/input_shaper.py
def _set_smoother_params_c(self, axis, C_pieces, t_sm):
    """Marshal piecewise coeffs to C."""
    ffi_main, ffi_lib = chelper.get_ffi()
    n_pieces = len(C_pieces)
    assert n_pieces <= 9, f"Too many pieces: {n_pieces}"
    # Flatten to double[n_pieces × 8]
    buf = ffi_main.new("double[]", n_pieces * 8)
    for i, (t_start, t_end, coeffs) in enumerate(C_pieces):
        buf[i * 8] = t_start
        buf[i * 8 + 1] = t_end
        for k in range(6):
            buf[i * 8 + 2 + k] = coeffs[k] if k < len(coeffs) else 0.0
    ffi_lib.input_shaper_set_smoother_params(
        self.sk, axis, n_pieces, buf, t_sm)
```

Also validate `shaper_type` in config-load: only accept `zv`, `mzv`, `bs1`..`bs5`.

- [ ] **Step 4: Run — PASS**

- [ ] **Step 5: Commit**

```bash
git add klippy/chelper/kin_shaper.c klippy/extras/input_shaper.py test/test_input_shaper_bs.py
git commit -m "input_shaper: piecewise FFI signature + bs config validation

FFI: input_shaper_set_smoother_params takes (n_pieces, flat buf, t_sm)
instead of (n, a[], t_sm). Python marshals piecewise pieces to flat
buffer. Config-load validates shaper_type ∈ {zv, mzv, bs1..bs5}."
```

### Task 8: Integrate bs* into `blendshaper` / `blendmath` (A_axis plumbing)

**Files:**
- Modify: `klippy/blendmath.py:_compute_A_axis_smooth_is` — extend to accept bs* names.
- Modify: `klippy/blendshaper.py:_SMOOTH_SPAN_FACTOR` — add bs1-bs5 entries.

- [ ] **Step 1: Write test — A_axis > 0 for bs configs through full pipeline**

```python
# test/test_blendshaper.py (add)
def test_blendshaper_compute_bounds_bs3():
    # Set up toolhead with bs3 shaper
    # Call compute_shaper_bounds
    # Assert bounds finite and A_axis > 0
    ...
```

- [ ] **Step 2: Add bs1-bs5 to `_SMOOTH_SPAN_FACTOR` in `blendshaper.py`**

Use the `F_m` constants (1.5553 … 2.7252):

```python
# klippy/blendshaper.py (extend existing table)
_SMOOTH_SPAN_FACTOR = {
    'bs1': 1.5553,
    'bs2': 1.9462,
    'bs3': 2.2519,
    'bs4': 2.5061,
    'bs5': 2.7252,
}
```

- [ ] **Step 3: Update `_compute_A_axis_smooth_is` to accept bs names**

Already calls `ShaperCalibrate.find_smoother_max_accel` — which Task 3 updated for piecewise. Just extend the shaper-name allow-list.

- [ ] **Step 4: Run — PASS**

- [ ] **Step 5: Commit**

```bash
git add klippy/blendshaper.py klippy/blendmath.py test/test_blendshaper.py
git commit -m "blendshaper/blendmath: bs1-bs5 span + A_axis plumbing"
```

---

## D3 — Feedforward inverse at `kin_shaper.c` and `kin_extruder.c`

**Scope:** Compute `k_fused = h ⊛ w` via Path C at shaper config time. Apply at query layer on XY and extruder axes. Cascade identity test.

### Task 9: Fused kernel computed in Python at shaper-reset

**Files:**
- Modify: `klippy/extras/input_shaper.py`
- Test: `test/test_fused_kernel_fit.py` (round-trip)

- [ ] **Step 1: Write failing test — fused kernel applied produces identity in passband**

```python
def test_fused_kernel_applied_identity_in_passband():
    """Commanded X(t) = planned X(t) (cascade identity) to passband error ≤ 3%."""
    # Plan a sinusoidal X(t) = sin(2π·10·t) at 10 Hz (inside 0.3·f_sh = 12 Hz passband)
    # Apply k_fused once (forward-only path would apply w, now applies h⊛w)
    # Check output matches input to 3%
    ...
```

- [ ] **Step 2: Run — FAIL**

- [ ] **Step 3: In `input_shaper.py`'s config handler, compute & pass `k_fused` instead of `w`**

```python
# klippy/extras/input_shaper.py
def _config_axis(self, axis, shaper_type, freq, damping, target_smoothing):
    # Forward kernel (existing logic, now returns piecewise)
    C_w, t_sm = shaper_defs.INPUT_SMOOTHERS[variant_index].init_func(
        freq, damping, normalize_coeffs=True)
    if self.feedforward_inverse_enabled:
        # Compute inverse h and fused k = h ⊛ w
        h, T_h, dt = bspline_inverse.compute_inverse_fir(
            C_w, t_sm, pb_max_hz=self.pb_max * freq)
        C_k_fused = bspline_inverse.fit_fused_kernel(
            C_w, t_sm, h, T_h, dt, n_pieces=9, degree=5)
        t_fused = t_sm + T_h
        self._set_smoother_params_c(axis, C_k_fused, t_fused)
        # Publish G for saturation cap
        self.G_axis[axis] = float(np.sum(np.abs(h)) * dt)
    else:
        # Forward-only fallback (classic path)
        self._set_smoother_params_c(axis, C_w, t_sm)
        self.G_axis[axis] = 1.0
```

- [ ] **Step 4: Run — PASS**

- [ ] **Step 5: Commit**

```bash
git add klippy/extras/input_shaper.py test/test_fused_kernel_fit.py
git commit -m "input_shaper: feedforward inverse via fused k_fused kernel

At shaper config/reset time, computes FIR inverse h per axis, fits
k_fused = h ⊛ w to 9 × degree-5 pieces (Path C), passes as the
smoother kernel via FFI. G = ‖h‖₁ published on self.G_axis for D4's
saturation cap."
```

### Task 10: Extruder axis receives same fused kernel

**Files:**
- Modify: `klippy/chelper/kin_extruder.c` — `extruder_set_smoothing_params` FFI signature update.
- Modify: `klippy/kinematics/extruder.py` — wire G to PA path.

- [ ] **Step 1: Write test — extruder uses same k_fused as XY**

```python
def test_extruder_fused_kernel_xy_sync():
    """XY trace and E axis trace at same timestep match planned trajectory
    to within quantization (cascade identity for both)."""
    ...
```

- [ ] **Step 2: Update FFI + wiring following Task 7 template.**

- [ ] **Step 3: Commit**

```bash
git commit -m "kin_extruder: apply fused kernel on E axis, preserve PA sync"
```

### Task 11: Verify k_fused application — cascade identity end-to-end

- [ ] **Step 1: Write integration test**

Plan a quintic trajectory (via existing polyline path, pre-D2), run through the full shaper pipeline, compare output to planned position. Passband error ≤ 2% on `[0, 0.5·f_sh]`.

- [ ] **Step 2: Run + verify + commit**

---

## D4 — Saturation cap with sum-of-projections

### Task 12: `AxisShaperSnapshot.inverse_G` field

**Files:**
- Modify: `klippy/blendshaper.py`
- Modify: `klippy/blendmath.py::_extract_shapers`

- [ ] **Step 1: Write failing test — G flows through from input_shaper to blendquintic**

```python
def test_G_flows_to_v_cap_fn():
    # Setup bs3 shaper, query v_cap_fn at κ=0.03, 90° corner
    # Assert v_cap_fn matches expected sum-of-projections formula
    ...
```

- [ ] **Step 2: Add `inverse_G` to `AxisShaperSnapshot`**

```python
@dataclass
class AxisShaperSnapshot:
    ...
    A_axis: float
    ...
    inverse_G: float = 1.0  # L1 norm of inverse kernel; 1.0 if no inverse
```

- [ ] **Step 3: `_extract_shapers` populates it**

```python
# klippy/blendmath.py
def _extract_shapers(toolhead):
    ...
    for axis in ['x', 'y']:
        snap.inverse_G = toolhead.input_shaper.G_axis.get(axis, 1.0)
    ...
```

- [ ] **Step 4: PASS, commit**

### Task 13: `v_cap_fn(s)` implements sum-of-projections G_worst(s)

**Files:**
- Modify: `klippy/blendquintic.py::v_cap_fn`
- Test: `test/test_saturation_cap.py` (new)

- [ ] **Step 1: Write failing test — orientation-dependent cap**

```python
@pytest.mark.parametrize("theta_deg,expected_v_cap_range", [
    (0, (288.0, 289.0)),    # axis-aligned
    (45, (242.5, 243.0)),   # diagonal (tightest)
    (90, (288.0, 289.0)),   # axis-aligned (perpendicular)
])
def test_v_cap_fn_orientation_dependent(theta_deg, expected_v_cap_range):
    # Set up QuinticShape with bs3 shaper, G=2.003, κ=0.03 at midpoint
    # Rotate the blend by theta_deg
    # Query v_cap_fn at the κ peak
    # Expect v_cap in range per per_axis_saturation_derivation.md worked example
    ...
```

- [ ] **Step 2: Run — FAIL**

- [ ] **Step 3: Update `v_cap_fn`**

```python
# klippy/blendquintic.py
def v_cap_fn(self, s):
    # ... existing κ, t̂, n̂ computation via _point_frame
    pos, t_hat, n_hat = self._point_frame(self.Q, self._s_to_t(s))
    kappa = self._kappa_at(s)
    # Sum-of-projections G_worst(s) across shaped axes
    G_worst = 1.0
    for snap in self._limits.shapers:
        axis_dir = snap.axis_dir  # e_x or e_y unit vector
        proj_t = abs(np.dot(t_hat, axis_dir))
        proj_n = abs(np.dot(n_hat, axis_dir))
        G_worst = max(G_worst, snap.inverse_G * (proj_t + proj_n))
    if kappa > 0.0:
        a_eff = self._limits.a_max / G_worst
        v_cent = math.sqrt(a_eff / kappa)
        v = min(v, v_cent)
    # Other caps (v_max, v_jerk, v_step_cap) unchanged
    return v
```

- [ ] **Step 4: Run — PASS**

- [ ] **Step 5: Commit**

```bash
git add klippy/blendquintic.py klippy/blendshaper.py klippy/blendmath.py test/test_saturation_cap.py
git commit -m "blendquintic: per-s saturation cap via sum-of-projections G_worst

v_cap_fn replaces a_max with a_max / G_worst(s) where
G_worst(s) = max_axes G_axis · (|proj_t|+|proj_n|). Per per_axis_saturation_derivation.md."
```

---

## D5 — Lookahead extension

### Task 14: T_fused threaded through `note_step_generation_scan_time`

**Files:**
- Modify: `klippy/chelper/kin_shaper.c::shaper_note_generation_time`
- Modify: `klippy/extras/input_shaper.py`

- [ ] **Step 1: Write test — kin_flush_delay ≥ T_sm + T_h / 2**

```python
def test_lookahead_extended_for_bs3():
    # Config bs3 with feedforward on
    # Assert toolhead.kin_flush_delay ≥ (T_sm + T_h) / 2
    # For bs3 @ 40 Hz: T_sm=56 ms, T_h=112 ms → 84 ms
    ...
```

- [ ] **Step 2: Update `shaper_note_generation_time`**

```c
// klippy/chelper/kin_shaper.c
static void shaper_note_generation_time(...) {
    double t_fused = is->sm_x.t_sm;  // t_fused already includes T_h after D3
    double pre_active = t_fused / 2 + fabs(is->sm_x.t_offs);
    double post_active = t_fused / 2 - is->sm_x.t_offs;
    // t_h == 0 guard: if feedforward is off (forward-only path),
    // t_sm still holds the bare forward-kernel width, which is smaller.
    sk_note_generation_time(is->sk, pre_active, post_active);
}
```

- [ ] **Step 3: PASS, commit.**

---

## D6 — Config migration

### Task 15: Reject old `smooth_*` names with friendly error

**Files:**
- Modify: `klippy/extras/input_shaper.py`
- Test: `test/test_input_shaper_bs.py`

- [ ] **Step 1: Write test**

```python
def test_retired_smooth_mzv_errors():
    with pytest.raises(configparser.Error, match="replaced in Magnum Opus"):
        make_printer(config_snippet="""
            [input_shaper]
            shaper_type = smooth_mzv
            shaper_freq_x = 40
        """)
```

- [ ] **Step 2: Add validation**

```python
RETIRED_SHAPERS = {
    'smooth_zv': 'bs1',
    'smooth_mzv': 'bs2',
    'smooth_ei': 'bs3',
    'smooth_2hump_ei': 'bs4',
    'smooth_zvd_ei': 'bs5',
    'smooth_si': 'bs3',
}

def _validate_shaper_type(config, shaper_type):
    if shaper_type in RETIRED_SHAPERS:
        replacement = RETIRED_SHAPERS[shaper_type]
        raise config.error(
            f"shaper_type '{shaper_type}' was replaced in Magnum Opus "
            f"with the cardinal B-spline chain family. Use "
            f"shaper_type = '{replacement}' for equivalent behavior."
        )
```

- [ ] **Step 3: PASS, commit.**

### Task 16: Update `shaper_calibrate.py` AUTOTUNE_SHAPERS

- [ ] Update `_autotune_shapers` to recommend from bs1..bs5. Update shaper-name list.
- [ ] Commit.

### Task 17: `motion_report.py` schema v2 + `tap_analysis.py` kind-skip

- [ ] Emit `kind` field in websocket payload; bump schema version.
- [ ] Update `tap_analysis.py` to skip `kind != linear` moves for now (pre-D2).
- [ ] Commit.

---

## D2 — Direct-quintic step generation

### Task 18: Tagged union in `struct move`

**Files:**
- Modify: `klippy/chelper/trapq.h`
- Test: golden-linear-move test before any change behavior

- [ ] **Step 1: Write golden test — linear move position exact match**

Plan 100 sample linear moves, save `move_get_coord` outputs per (m, t) tuple. Regression gate: bit-identical after tagged union.

- [ ] **Step 2: Define `enum move_kind` with `MOVE_LINEAR = 0`**

```c
// klippy/chelper/trapq.h
enum move_kind {
    MOVE_LINEAR = 0,                  /* existing trapq primitive */
    MOVE_QUINTIC_POLY_T = 1,          /* Plan 5: per-phase poly-in-t */
};

/* NOTE: MOVE_LINEAR must be 0. itersolve.c memsets struct move for
 * synthetic zero-motion fills — that synthesized struct must parse
 * as a valid linear move. Reordering this enum silently breaks that. */

struct move_quintic_phase {
    double t_end;
    struct coord c[11];  /* per-axis position polynomial coeffs in (t - t_phase_start) */
};

struct move {
    double print_time, move_t;
    enum move_kind kind;
    struct coord start_pos;
    union {
        struct {                       /* MOVE_LINEAR */
            double start_v, half_accel;
            struct coord axes_r;
        } lin;
        struct {                       /* MOVE_QUINTIC_POLY_T */
            double arc_length;
            struct move_quintic_phase accel, cruise, decel;
            double v_cap_min;
        } quintic;
    } u;
    struct list_node node;
};
```

- [ ] **Step 3: Update `move_get_coord` to dispatch on kind**

```c
static inline struct coord
move_get_coord(struct move *m, double move_time) {
    if (m->kind == MOVE_LINEAR) {
        double dist = (m->u.lin.start_v + m->u.lin.half_accel * move_time)
                      * move_time;
        return (struct coord) {
            .x = m->start_pos.x + m->u.lin.axes_r.x * dist,
            .y = m->start_pos.y + m->u.lin.axes_r.y * dist,
            .z = m->start_pos.z + m->u.lin.axes_r.z * dist,
        };
    }
    // MOVE_QUINTIC_POLY_T: dispatch on phase, evaluate polynomial via Horner
    return _quintic_eval_phase(m, move_time);
}
```

- [ ] **Step 4: Run golden test — PASS (linear unchanged)**

- [ ] **Step 5: Commit**

```bash
git add klippy/chelper/trapq.h klippy/chelper/trapq.c test/test_trapq_golden.py
git commit -m "trapq: tagged union struct move (MOVE_LINEAR=0 invariant)

Adds MOVE_QUINTIC_POLY_T variant alongside existing linear move.
itersolve memset-synthesized moves continue to parse as linear
(MOVE_LINEAR must stay at enum value 0). Golden-test confirms
linear-move outputs bit-identical to pre-refactor reference."
```

### Task 19: `itersolve.c::check_active` dispatches on kind

**Files:**
- Modify: `klippy/chelper/itersolve.c`
- Test: quintic-move step-active computation

- [ ] **Step 1: Write failing test — quintic with nonzero c[k] reports axis active**

```python
def test_check_active_quintic_nonzero_c2_active_on_x():
    # Construct quintic move with c[2].x = 1.0, c[*].y = 0.0
    # check_active should return True for X-stepper, False for Y-stepper
    ...
```

- [ ] **Step 2: Update `check_active`**

```c
// klippy/chelper/itersolve.c
static int
check_active(struct stepper_kinematics *sk, struct move *m) {
    if (m->kind == MOVE_LINEAR) {
        // existing logic
        return (m->u.lin.axes_r.x != 0.0
                || m->u.lin.axes_r.y != 0.0
                || m->u.lin.axes_r.z != 0.0);
    }
    // MOVE_QUINTIC_POLY_T: active if any per-axis polynomial coefficient is nonzero
    for (int k = 1; k < 11; k++) {
        if (m->u.quintic.accel.c[k].x != 0.0
            || m->u.quintic.accel.c[k].y != 0.0
            || m->u.quintic.accel.c[k].z != 0.0)
            return 1;
        // Similarly for cruise, decel (or just check if arc_length > 0)
    }
    return 0;
}
```

- [ ] **Step 3: PASS, commit.**

### Task 20: 11-moment `calc_antiderivatives` + phase dispatch

**Files:**
- Modify: `klippy/chelper/integrate.c`
- Test: quintic-move smoother integral round-trip

- [ ] **Step 1: Write failing test — smoother integral on degree-10 polynomial matches numpy**

```python
def test_integrate_move_quintic_matches_numpy():
    """Python numpy.polynomial reference vs C calc_antiderivatives on
    a degree-10 test polynomial."""
    # Construct quintic move with arbitrary polynomial coeffs
    # Compute integral ∫ k(τ) · x(t-τ) dτ via Python and C, compare
    assert np.allclose(c_result, numpy_result, rtol=1e-9)
```

- [ ] **Step 2: Extend `calc_antiderivatives` to 11 moments**

```c
// klippy/chelper/integrate.c
struct calc_antiderivatives {
    double m[11];   // m_0, m_1, ..., m_10
};

static void
integrate_move(const struct smoother *sm, const struct move *m,
               double time_offset, struct coord *out) {
    if (m->kind == MOVE_LINEAR) {
        // existing: 3 moments, 1 phase
        ...
    }
    // MOVE_QUINTIC_POLY_T: phase dispatch
    // For each phase (accel, cruise, decel) that overlaps the integration window:
    //   compute ∫ k_piece(τ) · Σ_k c_k (t-τ)^k dτ = Σ_k c_k · m_k
    //   where m_k comes from the smoother's piece-level antiderivatives
    // Handle phase boundary crossings
    ...
}
```

(Full implementation is substantial — ~150 LOC. Per spec D2a the key pieces are the 11-moment extension and phase-boundary dispatch.)

- [ ] **Step 3: PASS, commit.**

### Task 21: Python-side composition — quintic(s) ∘ s(t) → per-phase position-in-t

**Files:**
- Modify: `klippy/blendquintic.py`
- Test: `test/test_quintic_composition.py` (new)

- [ ] **Step 1: Write failing test — composition at t=0, t=t_accel_end, t=move_t matches arc-length evaluation**

```python
def test_quintic_compose_phase_polynomials():
    """Compose quintic(s) ∘ s(t) for a trapezoid-in-s profile, verify
    that position(t_accel_end) equals quintic.position(s=accel_end_s)."""
    shape = QuinticShape.from_moves(prev_move, nxt_move, cd=0.05)
    accel_poly, cruise_poly, decel_poly = shape.compose_phase_polynomials(
        v_in=200.0, v_out=200.0, cruise_v=150.0, a_max=5000.0)
    # Evaluate accel_poly at t = t_accel_end
    t_ae = ...
    pos_composed = _eval_poly(accel_poly, t_ae)
    # Evaluate quintic at s_accel_end
    pos_direct = shape.position(s=accel_end_s)
    assert np.allclose(pos_composed, pos_direct, atol=1e-9)
```

- [ ] **Step 2: Implement composition using numpy.polynomial**

```python
# klippy/blendquintic.py
def compose_phase_polynomials(self, v_in, v_out, cruise_v, a_max):
    """Return (accel_poly, cruise_poly, decel_poly) as lists of per-axis
    degree-10/5/10 polynomial coefficients in the phase-local time variable."""
    # s(t) in accel phase: s = v_in*t + 0.5*a_max*t^2 (degree 2)
    # quintic.position(s) = p0 + c1*s + c2*s^2 + ... + c5*s^5 (degree 5 in s)
    # Composition: position(t) = quintic(s(t)) — degree 10 in t
    # Use numpy.polynomial.Polynomial composition:
    s_accel = np.polynomial.Polynomial([0, v_in, 0.5*a_max])
    accel_polys_per_axis = []
    for axis_coeffs in self._quintic_monomial_coeffs:
        quintic_p = np.polynomial.Polynomial(axis_coeffs)
        composed = quintic_p(s_accel)  # degree 10 in t
        accel_polys_per_axis.append(composed.coef)
    # Similarly cruise (degree 1 s(t) → degree 5 position(t))
    # And decel (mirror of accel)
    ...
```

- [ ] **Step 3: PASS, commit.**

### Task 22: `blendplanner._emit_blend` emits quintic trapq entry

**Files:**
- Modify: `klippy/blendplanner.py`
- Test: `test/test_blendplanner.py` — blend emits single trapq entry with kind=quintic

- [ ] **Step 1: Write test — single quintic entry, not polyline**

```python
def test_emit_blend_emits_single_quintic():
    blender = CornerBlender(...)
    blender.feed(prev_move)
    blender.feed(nxt_move)
    moves_emitted = get_trapq_moves()
    # Expect 1 quintic move, not N polyline sub-moves
    blend_moves = [m for m in moves_emitted if m.kind == MOVE_QUINTIC_POLY_T]
    assert len(blend_moves) == 1
```

- [ ] **Step 2: Replace polyline loop with `trapq_append_quintic` call**

(For now, use degenerate all-cruise profile — D7 fills in TOPP.)

- [ ] **Step 3: PASS, commit.**

### Task 23: klipper-sim Python Move shim update

**Files:**
- Modify: `~/Developer/klipper-sim/src/planner_sim/trapq.py` (or equivalent)

- [ ] Update the Move shim to handle `kind=quintic` (skip or parse according to sim needs).
- [ ] Run 59-test suite to verify no regression.
- [ ] Commit in klipper-sim repo.

---

## D7 — Unified v(s) along the curve

### Task 24: `v_cap_fn(s)` composes all 5 cap sources

**Files:**
- Modify: `klippy/blendquintic.py::v_cap_fn`

- [ ] Combine centripetal (now subsumed by saturation), rotation-jerk, shaper-bandwidth, Plan-3-equivalent extruder, user v_max.
- [ ] Test: each component produces the expected cap at known setups.
- [ ] Commit.

### Task 25: `v_cap_min(s)` helper for junction-cap feed (Option Z)

- [ ] Sample `v_cap_fn` at 128-point grid, return minimum.
- [ ] Plumb into `suppressed_junction_v` / `blendmath` junction-cap computation.
- [ ] Test: straight-into-tight-corner sequence shows prev decelerating to `v_cap_min` upstream of the blend.
- [ ] Commit.

### Task 26: TOPP implementation

**Files:**
- Create: `klippy/topp.py`
- Test: `test/test_topp.py`

- [ ] Implement Pham 2014 forward+backward TOPP on a 128-point grid.
- [ ] Unit tests: ramp trajectory produces expected profile; known worked example produces expected T_opt.
- [ ] Commit.

### Task 27: `blendplanner._emit_blend` calls TOPP and composes phase polynomials

- [ ] Wire TOPP result through `compose_phase_polynomials` into the trapq emit.
- [ ] Test: end-to-end blend emission produces valid quintic with correct v(0)=v_in, v(L)=v_out.
- [ ] Commit.

### Task 28: `blendextruder.cap_move` retires — `v_extr(s)` absorbed

**Files:**
- Modify: `klippy/blendextruder.py`
- Modify: `klippy/blendquintic.py::v_cap_fn`

- [ ] Move the flow-ratio cap logic into a `v_extr(s)` contribution inside `v_cap_fn`.
- [ ] Remove `blendextruder.cap_move` wiring from `toolhead.move`.
- [ ] Test: equivalent behavior preserved — asymmetric flow blend respected per-s.
- [ ] Commit.

---

## Integration + HW smoke

### Task 29: End-to-end integration test

- [ ] Plan a known trajectory (symmetric 90° corner, bs3 shaper, feedforward on).
- [ ] Run through full pipeline: blendplanner → trapq → kin_shaper → itersolve → stepcompress.
- [ ] Compare stepper outputs to planned position. Passband error ≤ 2% on `[0, 0.5·f_sh]`.
- [ ] Commit test.

### Task 30: Batch-sim regression

- [ ] Run `Voron_Design_Cube_v7_ABS_22m13s` under bs2 (≈ old smooth_mzv replacement).
- [ ] Verify finite stepper outputs, no sysload regression vs Plan 4 baseline.
- [ ] Document results in `docs/superpowers/plans/plan5-hardware-results.md` (create).

### Task 31: HW smoke — one print under bs3 + feedforward

- [ ] User runs a test print (magnum-opus config).
- [ ] Pass criteria: no ringing regressions, no `Timer too close`, sysload < 2.0.
- [ ] Document in HW results doc.

---

## Self-review

- Spec coverage: all 7 deliverables have at least one task each. D1 tasks 1-8, D3 tasks 9-11, D4 tasks 12-13, D5 task 14, D6 tasks 15-17, D2 tasks 18-23, D7 tasks 24-28.
- Placeholder scan: this plan is intentionally lighter on code-per-step for D2/D7 tasks because the actual implementation details are substantial and better developed by the implementer-subagent from the spec + research memos. Each task names the files to touch, the function signatures, the test approach, and the commit message template.
- Type consistency: `AxisShaperSnapshot.inverse_G`, `QuinticShape.v_cap_min`, `MOVE_QUINTIC_POLY_T`, `move_quintic_phase` — consistent across tasks.
- This plan assumes implementer-subagents will read the spec + derivation memos for full context. Tasks 1-8 (D1 foundational) are spelled out in more detail because they land first and bootstrap everything else; D2 and D7 have higher-level guidance because the spec itself carries most of the detail.

---

## Execution notes

**Suggested order:** follow the deliverable ordering (D1 → D3 → D4 → D5 → D6 → D2 → D7). Within a deliverable, tasks are sequential.

**CI-testable milestones:**
- After D1 (Task 8): bs1-bs5 loaded, A_axis computed correctly, no feedforward yet.
- After D3 (Task 11): feedforward inverse applied, cascade identity test passes.
- After D5 (Task 14): lookahead window correctly extended.
- After D6 (Task 17): config migration works, old names rejected.
- After D2 (Task 23): quintic trapq entries emitted as single moves, stepper outputs match polyline reference ±1 step.
- After D7 (Task 28): `v_cap_min` plumbing + TOPP profile + extruder cap absorbed.

**Implementation note for subagents:** each task's description is minimal; read the spec section and referenced derivation memo for full context before writing code. Paths are exact. When a task says "write test" or "implement", show the actual code in the PR following the spec's signature conventions.
