#!/usr/bin/env python3
"""Numerical verification for the smooth-shaper `target_smoothing` cap.

Companion to docs/superpowers/specs/2026-04-20-target-smoothing-smooth-family.md.

What it checks:
  1. Closed-form second central moment sigma^2 of each smoother's support
     function matches numerical integration (sanity).
  2. offset_180(A, smoother) = (A/2) * sigma^2 matches the current
     `_get_smoother_smoothing` runtime code over a grid of accel values.
  3. Limiting-case: as a narrow double-box support of half-separation T_d/2
     and box width t_sm -> 0, its sigma^2 approaches the impulse-ZV value
     (T_d/2)^2 with O(t_sm^2) convergence.
  4. Root of offset_180(A) = target_smoothing matches both the closed-form
     A_crit = 2*target/sigma^2 and the bisection in `find_smoother_max_accel`.

Run:
  source /Users/daniladergachev/Developer/kalico/.venv-test/bin/activate
  cd /Users/daniladergachev/Developer/kalico-smooth-shapers
  python scripts/verify_target_smoothing_smooth.py
"""
from __future__ import annotations

import math
import sys

import numpy as np

# Make `klippy` importable when run from the worktree root.
sys.path.insert(0, '.')

from klippy.extras import shaper_calibrate, shaper_defs  # noqa: E402


# numpy >= 2.0 renamed np.trapz -> np.trapezoid. Current phase-C code in
# shaper_calibrate.py still uses np.trapz; shim it here so the runtime
# `_get_smoother_smoothing` call survives until Task 9 patches the code.
if not hasattr(np, 'trapz') and hasattr(np, 'trapezoid'):
    np.trapz = np.trapezoid  # type: ignore[attr-defined]


# ----------------------------------------------------------------------
# Derivation reference implementation
# ----------------------------------------------------------------------

def smoother_moment(C, t_sm, k):
    """k-th raw moment of the unnormalized support polynomial
    w_bar(t) = sum_i C[i] t^i on [-t_sm/2, +t_sm/2].

    M_k = integral_{-hst}^{hst} t^k * w_bar(t) dt
        = sum_i C[i] * integral_{-hst}^{hst} t^(i+k) dt
    For (i+k) odd the integral is 0; otherwise 2*hst^(i+k+1)/(i+k+1).
    """
    hst = 0.5 * t_sm
    s = 0.0
    for i, c in enumerate(C):
        if (i + k) % 2 == 0:
            s += c * 2.0 * hst**(i + k + 1) / (i + k + 1)
    return s


def smoother_sigma2(C, t_sm):
    """Second central moment of the (normalized) support function w(t).

    sigma^2 = E[t^2] - (E[t])^2 under w(t) = w_bar(t) / M_0.
    """
    M0 = smoother_moment(C, t_sm, 0)
    M1 = smoother_moment(C, t_sm, 1)
    M2 = smoother_moment(C, t_sm, 2)
    return M2 / M0 - (M1 / M0) ** 2


def offset_180_closed(C, t_sm, accel):
    """offset_180(A, smoother) = (A/2) * sigma_T^2. See derivation section 2."""
    return 0.5 * accel * smoother_sigma2(C, t_sm)


def A_crit_closed(C, t_sm, target):
    """A such that offset_180(A) = target."""
    return 2.0 * target / smoother_sigma2(C, t_sm)


# ----------------------------------------------------------------------
# Checks
# ----------------------------------------------------------------------

def check1_sigma2_numerical(C, t_sm, npts=10001):
    """Brute-force integration of variance vs closed-form."""
    hst = 0.5 * t_sm
    t = np.linspace(-hst, hst, npts)
    # Horner on C (low-to-high) to evaluate sum c_i t^i.
    w = np.zeros_like(t)
    for c in C[::-1]:
        w = w * t + c
    Z = np.trapezoid(w, t)
    w_n = w / Z
    mean_num = np.trapezoid(t * w_n, t)
    var_num = np.trapezoid((t - mean_num) ** 2 * w_n, t)
    return var_num


def check2_offset_180_vs_runtime(sc, smoother, accel_grid):
    """Compare closed-form offset_180 against current phase-C runtime."""
    C, t_sm = smoother
    closed = np.array([offset_180_closed(C, t_sm, A) for A in accel_grid])
    runtime = np.array([sc._get_smoother_smoothing(smoother, A) for A in accel_grid])
    abs_err = np.abs(closed - runtime)
    rel_err = abs_err / np.maximum(np.abs(closed), 1e-15)
    return closed, runtime, rel_err


def narrow_double_box_sigma2(t_sm_box, T_d, npts=200001):
    """Variance of two unit-area boxes centered at +-T_d/2, width t_sm_box.

    As t_sm_box -> 0 this becomes the ZV impulse pair at 1/T_d Hz.
    Analytic: sigma^2 = (T_d/2)^2 + t_sm_box^2 / 12.
    """
    hst_total = 0.5 * T_d + 0.5 * t_sm_box
    t = np.linspace(-hst_total, hst_total, npts)
    w = np.where(np.abs(t - 0.5 * T_d) < 0.5 * t_sm_box, 1.0, 0.0)
    w += np.where(np.abs(t + 0.5 * T_d) < 0.5 * t_sm_box, 1.0, 0.0)
    w /= np.trapezoid(w, t)
    mean = np.trapezoid(t * w, t)
    return np.trapezoid((t - mean) ** 2 * w, t)


def check3_limiting_case():
    """Limit t_sm_box -> 0 of a narrow double-box -> ZV impulse sigma^2."""
    T_d = 0.01  # 50 Hz period * 0.5
    impulse_sigma2 = (T_d / 2) ** 2
    rows = []
    for t_sm_box in [1e-2, 1e-3, 1e-4]:
        sigma2_smooth = narrow_double_box_sigma2(t_sm_box, T_d)
        analytic = (T_d / 2) ** 2 + t_sm_box ** 2 / 12.0
        rel_err_vs_impulse = abs(sigma2_smooth - impulse_sigma2) / impulse_sigma2
        rel_err_vs_analytic = abs(sigma2_smooth - analytic) / analytic
        rows.append((t_sm_box, sigma2_smooth, impulse_sigma2,
                     rel_err_vs_impulse, rel_err_vs_analytic))
    return rows


def check4_root_convergence(sc, smoother, target):
    """find_smoother_max_accel bisection must match A_crit_closed."""
    C, t_sm = smoother
    A_expected = A_crit_closed(C, t_sm, target)
    A_bisect = sc.find_smoother_max_accel(smoother, target_smoothing=target)
    return A_expected, A_bisect, abs(A_expected - A_bisect) / A_expected


# ----------------------------------------------------------------------
# Main
# ----------------------------------------------------------------------

def main():
    # A printer-less ShaperCalibrate for pure math.
    sc = shaper_calibrate.ShaperCalibrate(printer=None)

    print('=== Check 1: closed-form sigma^2 vs numerical integration ===')
    print(f'{"smoother":<18} {"sigma^2 closed":>16} {"sigma^2 num":>16} {"rel err":>12}')
    for cfg in shaper_defs.INPUT_SMOOTHERS:
        sm = cfg.init_func(40.0, 0.1)
        C, t_sm = sm
        sig2_cf = smoother_sigma2(C, t_sm)
        sig2_num = check1_sigma2_numerical(C, t_sm)
        rel = abs(sig2_cf - sig2_num) / sig2_cf
        print(f'{cfg.name:<18} {sig2_cf:>16.6e} {sig2_num:>16.6e} {rel:>12.2e}')

    print()
    print('=== Check 2: offset_180 closed-form vs runtime _get_smoother_smoothing ===')
    print('(smooth_mzv at 40 Hz, damping 0.1, grid of 10 accels from 100 to 50000)')
    sm = [c for c in shaper_defs.INPUT_SMOOTHERS if c.name == 'smooth_mzv'][0]
    smoother = sm.init_func(40.0, 0.1)
    accels = np.linspace(100, 50000, 10)
    closed, runtime, rel_err = check2_offset_180_vs_runtime(sc, smoother, accels)
    print(f'{"A (mm/s^2)":>12} {"closed (mm)":>14} {"runtime (mm)":>14} {"rel err":>12}')
    for A, c, r, e in zip(accels, closed, runtime, rel_err):
        print(f'{A:>12.1f} {c:>14.6e} {r:>14.6e} {e:>12.2e}')
    max_rel = float(rel_err.max())
    print(f'max relative error: {max_rel:.2e}')

    print()
    print('=== Check 3: limiting case — narrow double-box -> ZV impulse ===')
    print('T_d=0.01 s (ZV at 50 Hz). Target sigma^2_impulse = (T_d/2)^2 = 2.5e-5')
    print(f'{"t_sm_box":>10} {"sigma^2 smooth":>16} {"impulse":>14} '
          f'{"rel err vs imp":>16} {"rel err vs analytic":>20}')
    for row in check3_limiting_case():
        t_sm_box, s_sm, s_imp, err_imp, err_an = row
        print(f'{t_sm_box:>10.0e} {s_sm:>16.6e} {s_imp:>14.6e} '
              f'{err_imp:>16.4e} {err_an:>20.4e}')

    print()
    print('=== Check 4: find_smoother_max_accel bisection vs closed-form A_crit ===')
    print('target_smoothing = 0.12 mm')
    print(f'{"smoother":<18} {"A_crit closed":>16} {"A_crit bisect":>16} {"rel err":>12}')
    for cfg in shaper_defs.INPUT_SMOOTHERS:
        sm = cfg.init_func(40.0, 0.1)
        A_exp, A_bis, rel = check4_root_convergence(sc, sm, 0.12)
        print(f'{cfg.name:<18} {A_exp:>16.2f} {A_bis:>16.2f} {rel:>12.2e}')

    print()
    print('OK')


if __name__ == '__main__':
    main()
