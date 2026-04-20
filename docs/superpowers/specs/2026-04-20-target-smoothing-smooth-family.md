# target_smoothing cap for smooth-family input shapers — derivation

**Date:** 2026-04-20
**Branch:** `smooth-shapers`
**Depends on:** sub-spec 6a (`2026-04-18-subspec-6a-shaper-scv-removal-design.md`)
**Feeds into:** Task 9 of `2026-04-20-smooth-shapers-port.md`
**Status:** findings — ready for Task 9 to consume

## 1. Problem

Blend-arc caps runtime accel so the shaper-induced residual at a 180° velocity
reversal stays under a user budget `target_smoothing` (default 0.12 mm). For
**impulse shapers** the residual closes to

    offset_180(A, shaper) = (A / 2) · sigma_T^2
    sigma_T^2 = sum_i A_i (T_i - ts)^2 / sum_i A_i
    ts       = sum_i A_i T_i / sum_i A_i

(see sub-spec 6a §"Verification" and `ShaperCalibrate._get_shaper_smoothing`
at `klippy/extras/shaper_calibrate.py:376`). `find_shaper_max_accel` bisects
on `offset_180(A) <= target_smoothing`.

Smooth shapers replace the impulse train with a continuous support function
`w(t)` of finite width `t_sm` and unit integral. The Phase C port left the
smoother path in `shaper_calibrate.py` with a numerical integrator
(`_get_smoother_smoothing`, line 393) plus a bisection wrapper
(`find_smoother_max_accel`, line 543), both flagged `TARGET_SMOOTHING: family
dispatch refined in Task 9`. Task 8's job is to derive what that dispatch
should actually compute and confirm the current Phase C code is (or isn't)
consistent with it.

## 2. Derivation

### 2.1 Support function and shaper output

A smooth input shaper convolves the commanded trajectory with a non-negative
kernel `w(t)` of finite support `[-t_sm/2, +t_sm/2]`, `integral w(t) dt = 1`
(see `klippy/extras/shaper_defs.py` lines 68–90, which enumerate the
conditions `integral w = 1`, `w(±t_sm/2) = 0`, `w >= 0`).

The shaped position at time `T` is

    x_sm(T) = integral x(T - t) · w(t) dt

with `t` running over the support.

### 2.2 180° reversal residual

Near a U-turn cusp located at `t_rev`, the commanded trajectory is

    x(t) = (A / 2) · (t - t_rev)^2

(both sides of the reversal: decelerating to zero, then reaccelerating —
this is the exact model the impulse derivation uses, from sub-spec 6a).

Substituting and expanding:

    x_sm(T) = (A / 2) · integral (T - t - t_rev)^2 · w(t) dt
            = (A / 2) · [(T - t_rev)^2 - 2 (T - t_rev) E[t] + E[t^2]]

where `E[·]` is the expectation under `w`. Define the centroid `ts = E[t]`
and shift the "reversal observation time" to the post-centroid frame
`u = T - t_rev - ts`:

    x_sm(T) = (A / 2) · [(u)^2 + (E[t^2] - ts^2)]
            = (A / 2) · [u^2 + sigma_T^2]

where `sigma_T^2 = E[t^2] - ts^2 = Var[w]` is the second **central** moment
of `w`. The unshaped path through the cusp (accounting for the shaper's group
delay `ts`) is `(A/2) · u^2`. The shaper residual — the displacement at the
cusp that *cannot* be explained by a time shift — is therefore

    offset_180(A, smoother) = (A / 2) · sigma_T^2                       (1)

**Form identical to the impulse case.** Only the definition of `sigma_T^2`
differs: for impulse it is a weighted sum over a discrete train; for smooth
it is the variance of a continuous density.

### 2.3 Closed form for polynomial smoothers

In Kalico the support function is a polynomial:

    w_bar(t) = sum_i C[i] · t^i          on [-t_sm/2, +t_sm/2]
    w(t)     = w_bar(t) / M_0
    M_k      = integral_{-t_sm/2}^{+t_sm/2} t^k · w_bar(t) dt

(coefficient order per `shaper_defs.init_smoother`, line 52: `C[0]` is the
constant, `C[n]` is the highest-order coefficient). For each raw moment:

    M_k = sum_i C[i] · integral_{-hst}^{+hst} t^(i+k) dt               (hst = t_sm/2)
        = sum_{i : (i+k) even}  C[i] · 2 · hst^(i+k+1) / (i+k+1)       (2)

(odd-power integrals over a symmetric interval vanish). Then

    ts        = M_1 / M_0
    sigma_T^2 = M_2 / M_0 - ts^2                                       (3)

For smoothers that are already normalized at construction time
(`normalize_coeffs=True` is the default in `init_smoother`), `M_0 = 1`
exactly, and `ts` equals `get_smoother_offset(C, t_sm)` as defined in
`shaper_defs.py:187`. No runtime integration is needed; (2)+(3) is a handful
of ops per smoother, evaluated once per calibration call.

### 2.4 Closed form for find_smoother_max_accel

Setting (1) equal to the budget and solving:

    offset_180(A_crit) = target_smoothing
    A_crit = 2 · target_smoothing / sigma_T^2                          (4)

Bisection is not required for the smooth branch — (4) is exact. The Phase C
code bisects anyway, which is harmless but wasteful and introduces
integration error (see §5.1).

## 3. Limiting case: smooth -> impulse

### 3.1 Analytic

An impulse pair ZV shaper at frequency `f` has `A = [0.5, 0.5]`,
`T = [0, T_d]` with `T_d = 1/f` (zero damping). Its impulse-formula
variance is

    sigma_T^2_impulse = sum_i A_i (T_i - ts)^2 / sum_i A_i = (T_d / 2)^2

Construct a **narrow double-box** support function: two uniform boxes of
width `t_sm_box`, unit integrated area, centered at `±T_d/2`. As
`t_sm_box -> 0` each box collapses to a delta, and the variance decomposes
cleanly:

    sigma_T^2_box(t_sm_box) = (T_d / 2)^2 + t_sm_box^2 / 12             (5)

(the second term is the variance of a uniform box of width `t_sm_box`,
added to the between-centers variance by the parallel-axis law). As
`t_sm_box -> 0`, (5) collapses to the impulse formula with error
`O(t_sm_box^2 / T_d^2)`.

More generally, for any smoother whose support concentrates into a pair of
deltas at `±T_d/2`, the general form (1) with (3) reduces to the impulse
form with `sigma_T_impulse`. The derivation of (1) from the convolution
integral made no assumption about the support's shape — only unit integral
and finite support — so the impulse case is recovered as
`w(t) -> 0.5 δ(t + T_d/2) + 0.5 δ(t - T_d/2)`.

### 3.2 Numerical

From `scripts/verify_target_smoothing_smooth.py`, Check 3:

    T_d=0.01 s (ZV at 50 Hz). Target sigma^2_impulse = (T_d/2)^2 = 2.5e-5
      t_sm_box   sigma^2 smooth        impulse   rel err vs imp  rel err vs analytic
         1e-02     3.333317e-05   2.500000e-05       3.3333e-01           5.0000e-06
         1e-03     2.508342e-05   2.500000e-05       3.3367e-03           3.3723e-06
         1e-04     2.500076e-05   2.500000e-05       3.0295e-05           3.0384e-06

At `t_sm_box = 1e-4` seconds the smooth variance matches the impulse value
to relative error `3.0e-5` (requirement was `< 1e-3`, passes). The
`rel err vs analytic` column stays ~3e-6 regardless of `t_sm_box`, which
reflects finite-grid integration noise in the check, not a breakdown of the
derivation.

Convergence scales as `t_sm_box^2 / (3 T_d^2)` (tenfold narrower box gives
hundredfold tighter agreement), exactly matching (5).

## 4. Mainline-SCV floor analogue

### 4.1 What 5743ed91 does

Commit `5743ed91` on blend-arc lives in `klippy/blendmath.py`, not in
`shaper_calibrate.py`. It is a **planner-time arc-emission rule**: for each
corner, compare the fork's arc-blend path cost against the cost of a pure
mainline-SCV junction, and skip emitting the arc when mainline would be no
slower. The `sigma_T_max` used there is the impulse shaper's second moment
— the same quantity that (3) generalizes to the smooth case.

This rule operates on one quantity from shaper-calibration land
(`sigma_T_max`) and is family-agnostic by construction: the smooth derivation
above gives `sigma_T^2` for smoothers via (3). Once `blendmath.py` is fed the
smoother's `sigma_T` through the same channel that currently feeds the
impulse one (`find_shaper_max_accel` returns an accel; `sigma_T` is
extracted upstream from the shaper object), commit 5743ed91's formula is
**already correct** for the smooth family. No analogue is needed; the same
check applies verbatim with a family-aware `sigma_T` source.

### 4.2 Target_smoothing-cap floor

For the **cap itself** (what Task 8 is scoped to), there is no analogue to
derive either. The impulse-family `find_shaper_max_accel` does **not**
include an SCV floor inside the bisection — the bisection is a pure root
find on `offset_180(A) <= target`. The "floor" behavior arises outside, in
the planner's arc-or-not decision. The smooth-family analogue
`find_smoother_max_accel` mirrors that shape: pure root find, no SCV floor
inside.

**Specifically:** a floor inside the cap would mean "if the cap would force
a very low accel, raise it instead". That has no physical meaning at the
calibration level — the cap *is* the definition of the budget. The
only plausible interpretation is "cap below the mainline-SCV equivalent is
pointless because the planner already never uses that much accel at a
corner", and that argument applies to the impulse case equally and is
explicitly absent from `find_shaper_max_accel`.

**Conclusion:** no smooth-family floor needed. The blendmath-level check
uses `sigma_T` directly; once `sigma_T` is extracted from a smoother via
(3), commit 5743ed91's logic transfers unchanged.

## 5. Verify or replace the Phase C `_get_smoother_smoothing`

Current implementation (`klippy/extras/shaper_calibrate.py:393-412`):

```python
def _get_smoother_smoothing(self, smoother, accel=5000):
    np = self.numpy
    half_accel = accel * 0.5
    C, t_sm = smoother
    hst = 0.5 * t_sm
    t, dt = np.linspace(-hst, hst, 100, retstep=True)
    w = np.zeros(shape=t.shape)
    for c in C[::-1]:
        w = w * (-t) + c                    # evaluates w(t) = C_poly(-t)
    inv_norm = 1.0 / np.trapz(w, dx=dt)
    w *= inv_norm
    t -= np.trapz(t * w, dx=dt)             # subtract centroid

    offset_180 = np.trapz(half_accel * t**2 * w, dx=dt)
    return abs(offset_180)
```

### 5.1 What it computes

After normalization and centroid shift, the function returns

    (A / 2) · integral (t - mean)^2 · w(t) dt  =  (A / 2) · Var[w] = offset_180

where `w(t) = C_poly(-t) / Z`. The sign flip `C_poly(-t)` vs the physical
`w_phys(t) = C_poly(t)` does **not** change `Var[w]` — the variance is
invariant under `t -> -t`. So the formula is mathematically correct and
equivalent to (1)+(3).

### 5.2 Issues

1. **Runtime bug on modern numpy.** `np.trapz` was removed in numpy 2.0;
   the module exposes `np.trapezoid` instead. Calling
   `_get_smoother_smoothing` raises `AttributeError: module 'numpy' has no
   attribute 'trapz'` on the fork's venv (numpy 2.x). Same issue exists in
   `estimate_smoother_old` at line 171. This is a pre-existing Phase C port
   issue, not introduced by Task 8 but exposed by it.

2. **Integration error.** 100 Simpson/trapezoidal samples yield ~0.2%
   relative error vs the closed form (verified in Check 2 of the script).
   For `target_smoothing = 0.12 mm`, this translates to the bisected
   `A_crit` being ~0.2% high — smoothers would run at slightly aggressive
   accel. Not dangerous, but sloppy given that the closed form is trivial.

3. **Bisection is redundant.** (4) gives `A_crit` in closed form; the
   bisection throws away that simplicity for no benefit.

### 5.3 Recommended replacement

Replace `_get_smoother_smoothing` and `find_smoother_max_accel` with the
closed form. Task 9 patch (prose only — not applied in Task 8):

```python
def _get_smoother_sigma2(self, smoother):
    """Second central moment of the smoother's support function.

    sigma^2 = M_2 / M_0 - (M_1 / M_0)^2, closed form over the polynomial
    coefficients. See docs/superpowers/specs/
    2026-04-20-target-smoothing-smooth-family.md §2.3.
    """
    C, t_sm = smoother
    hst = 0.5 * t_sm

    def raw_moment(k):
        s = 0.0
        for i, c in enumerate(C):
            if (i + k) % 2 == 0:
                s += c * 2.0 * hst**(i + k + 1) / (i + k + 1)
        return s

    M0 = raw_moment(0)
    ts = raw_moment(1) / M0
    return raw_moment(2) / M0 - ts * ts


def _get_smoother_smoothing(self, smoother, accel=5000):
    """Shaper residual at a 180° velocity reversal; closed form per
    sub-spec docs/...2026-04-20-target-smoothing-smooth-family.md."""
    return 0.5 * accel * self._get_smoother_sigma2(smoother)


def find_smoother_max_accel(self, smoother, target_smoothing=None):
    """Closed-form inverse of _get_smoother_smoothing: A = 2 target / sigma^2."""
    target = (
        self.target_smoothing if target_smoothing is None
        else target_smoothing
    )
    sigma2 = self._get_smoother_sigma2(smoother)
    if sigma2 <= 0.0:
        return float('inf')  # degenerate (zero-width support, not physical)
    return 2.0 * target / sigma2
```

This:
- removes the `np.trapz` bug
- removes 100-sample integration error
- removes redundant bisection
- remains O(n) in polynomial order

## 6. Numerical verification

Script: `scripts/verify_target_smoothing_smooth.py`.

Run:

    source /Users/daniladergachev/Developer/kalico/.venv-test/bin/activate
    cd /Users/daniladergachev/Developer/kalico-smooth-shapers
    python scripts/verify_target_smoothing_smooth.py

Abridged output:

    === Check 1: closed-form sigma^2 vs numerical integration ===
    smoother             sigma^2 closed      sigma^2 num      rel err
    smooth_zv              4.186337e-05     4.186337e-05     6.83e-08
    smooth_mzv             5.276471e-05     5.276470e-05     1.92e-07
    smooth_ei              5.964468e-05     5.964467e-05     2.03e-07
    smooth_2hump_ei        6.242956e-05     6.242954e-05     3.76e-07
    smooth_zvd_ei          9.197580e-05     9.197576e-05     3.79e-07
    smooth_si              6.283737e-05     6.283735e-05     1.91e-07

    === Check 2: offset_180 closed-form vs runtime _get_smoother_smoothing ===
    (smooth_mzv at 40 Hz, damping 0.1, accel 100..50000)
    max relative error: 1.96e-03

    === Check 3: limiting case — narrow double-box -> ZV impulse ===
    T_d=0.01 s (ZV at 50 Hz). Target sigma^2_impulse = (T_d/2)^2 = 2.5e-5
      t_sm_box   sigma^2 smooth        impulse   rel err vs imp  rel err vs analytic
         1e-02     3.333317e-05   2.500000e-05       3.3333e-01           5.0000e-06
         1e-03     2.508342e-05   2.500000e-05       3.3367e-03           3.3723e-06
         1e-04     2.500076e-05   2.500000e-05       3.0295e-05           3.0384e-06

    === Check 4: find_smoother_max_accel bisection vs closed-form A_crit ===
    target_smoothing = 0.12 mm
    smoother              A_crit closed    A_crit bisect      rel err
    smooth_zv                   5732.94          5736.93     6.97e-04
    smooth_mzv                  4548.49          4557.43     1.96e-03
    smooth_ei                   4023.83          4032.17     2.07e-03
    smooth_2hump_ei             3844.33          3859.13     3.85e-03
    smooth_zvd_ei               2609.38          2619.49     3.87e-03
    smooth_si                   3819.38          3826.84     1.95e-03

Interpretation:

- Check 1: closed-form variance (§2.3) matches brute-force integration to
  machine precision (~1e-7 rel err at 10001 samples). No surprises.
- Check 2: The current 100-sample runtime integrator agrees with the
  closed form to ~0.2% at every accel. Constant relative error across accel
  confirms it's a pure `sigma^2` estimation issue — i.e., a one-time
  integration error, not an accel-dependent bug.
- Check 3: Limiting case passes; at `t_sm = 1e-4` s the smooth
  formula recovers the impulse-ZV variance to `3.0e-5` rel err.
  Convergence rate is `O(t_sm^2)`.
- Check 4: The Phase C bisection converges to within ~0.4% of the
  closed-form root (same ~0.2% floor from §5.1 Issue 2, plus a touch
  from bisection tolerance `eps=1e-2`).

The script shims `np.trapz = np.trapezoid` at import time to keep Phase C's
`_get_smoother_smoothing` callable during this verification. That shim is
scaffolding for the script; Task 9 is expected to delete the integration
code entirely per §5.3.

## 7. Recommended signatures for Task 9

### 7.1 Family discriminator

The cleanest discriminator is by **shape of the tuple**: an impulse shaper
is `(A, T)` where both are lists and `len(A) == len(T)` and `T[0] == 0`; a
smoother is `(C, t_sm)` where `C` is a list of polynomial coefficients and
`t_sm` is a scalar. A structural sniff:

```python
def _is_smoother(obj):
    """True iff obj is a (C_poly, t_sm) smoother tuple.

    Impulse shaper: (A, T) with T a list of floats.
    Smoother:       (C, t_sm) with t_sm a bare float.
    """
    if not isinstance(obj, tuple) or len(obj) != 2:
        return False
    return isinstance(obj[1], float) or isinstance(obj[1], (int, ))
```

If the BE-v2 port adds a dataclass/namedtuple around smoothers, prefer an
explicit `isinstance(obj, InputSmootherCfg)` or `hasattr(obj, 'smooth_time')`
check; the structural sniff above is the fallback.

### 7.2 Method bodies Task 9 should land

Unified dispatch on `ShaperCalibrate`:

```python
def _get_smoother_sigma2(self, smoother):
    # See §2.3 closed form.
    C, t_sm = smoother
    hst = 0.5 * t_sm

    def raw_moment(k):
        s = 0.0
        for i, c in enumerate(C):
            if (i + k) % 2 == 0:
                s += c * 2.0 * hst**(i + k + 1) / (i + k + 1)
        return s

    M0 = raw_moment(0)
    ts = raw_moment(1) / M0
    return raw_moment(2) / M0 - ts * ts


def _get_smoother_smoothing(self, smoother, accel=5000):
    return 0.5 * accel * self._get_smoother_sigma2(smoother)


def find_smoother_max_accel(self, smoother, target_smoothing=None):
    target = (
        self.target_smoothing if target_smoothing is None
        else target_smoothing
    )
    sigma2 = self._get_smoother_sigma2(smoother)
    if sigma2 <= 0.0:
        return float('inf')
    return 2.0 * target / sigma2


def find_shaper_max_accel(self, shaper_or_smoother, target_smoothing=None):
    """Family dispatcher. Impulse: bisection on _get_shaper_smoothing.
    Smooth: closed form via find_smoother_max_accel.
    """
    if _is_smoother(shaper_or_smoother):
        return self.find_smoother_max_accel(shaper_or_smoother, target_smoothing)
    # impulse path unchanged from sub-spec 6a
    target = (
        self.target_smoothing if target_smoothing is None
        else target_smoothing
    )
    return self._bisect(
        lambda A: self._get_shaper_smoothing(shaper_or_smoother, A) <= target,
        1e-2,
    )
```

### 7.3 Public surface

Task 9's callers (from blendmath, from `input_shaper.py`) call
`sc.find_shaper_max_accel(shaper_object, target_smoothing=...)` and expect
a float accel. Both families return through the same entry point; no
caller-side branching needed.

## 8. Risks / edge cases for Task 9

1. **Phase-C `np.trapz` bug.** `_get_smoother_smoothing` as written is
   already broken on numpy 2.x. If Task 9 adopts §5.3, the bug dies with
   the old code.
2. **Symmetric support.** `M_1 = 0` exactly for even-only polynomials; the
   closed form still gives `sigma_T^2 = M_2 / M_0`.
3. **Floating-point safety.** `M_2/M_0 - ts^2` can go slightly negative
   when `ts^2 ≈ M_2/M_0`; the guard `if sigma2 <= 0` in §5.3 covers it.
4. **Centroid convention.** `shaper_defs.get_smoother_offset(C, t_sm)`
   with default `normalized=True` matches `M_1/M_0` for these
   pre-normalized coefficients. Do not pass `normalized=False`.
5. **Test parity.** Sub-spec 6a's impulse ZV@50Hz/ζ=0.1 regression pins
   `A ∈ [9000, 10000]`; the smooth branch is separate, so that test keeps
   passing. Mirror it for smooth: `smooth_mzv@40Hz/ζ=0.1` → ~4548 mm/s^2.

## 9. References

- sub-spec 6a: `docs/superpowers/specs/2026-04-18-subspec-6a-shaper-scv-removal-design.md`
- impulse derivation citations (via 6a): Singer & Seering 1990; Singhose 1997.
- code anchors:
  - `klippy/extras/shaper_calibrate.py` lines 376 (impulse `_get_shaper_smoothing`),
    393 (smoother `_get_smoother_smoothing`), 526 (`find_shaper_max_accel`),
    543 (`find_smoother_max_accel`).
  - `klippy/extras/shaper_defs.py` lines 52 (`init_smoother`, coefficient
    ordering), 68–90 (support-function invariants), 187 (`get_smoother_offset`).
  - `klippy/blendmath.py`: mainline-SCV floor check (commit `5743ed91`).
- numerical harness: `scripts/verify_target_smoothing_smooth.py`
