# test/test_shaper_calibrate.py
"""Tests for shaper_calibrate post sub-spec 6a (SCV removal).

The canonical reference case: ZV shaper at 50 Hz with damping_ratio=0.1.
Closed-form for offset_180-only smoothing target (0.12 mm):
    T_d = 1 / (f * sqrt(1 - zeta**2)) = 1 / (50 * sqrt(0.99)) ≈ 0.020101 s
    T_1 = 0.5 * T_d ≈ 0.010050 s              (ZV pulse span)
    ts  = 0.5 * T_1 ≈ 0.005025 s              (shaper-centroid shift)
    sigma2 = (T_1 - ts)**2 = ts**2 ≈ 2.525e-5
    A = 0.24 / sigma2 ≈ 9505 mm/s**2          (accel where offset_180 = 0.12)

Task 1 pins the OLD (pre-6a) value from the current implementation.
Tasks 2 and 3 tighten this pin to the post-change closed form.
"""
import math

import pytest

from klippy.extras import shaper_calibrate, shaper_defs


def _zv_50hz():
    """Canonical reference shaper for all regression pins in this file."""
    return shaper_defs.get_zv_shaper(shaper_freq=50.0, damping_ratio=0.1)


def test_find_shaper_max_accel_baseline_preflight():
    """Baseline regression pin — locks current (pre-6a) behavior.

    Task 1 expects the OLD value (with scv=5.0 default) from the current
    implementation. Tasks 2–3 replace this assertion with the closed-form
    offset_180-only value.
    """
    sc = shaper_calibrate.ShaperCalibrate(printer=None)
    shaper = _zv_50hz()
    max_accel = sc.find_shaper_max_accel(shaper, scv=5.0)
    # Old code: max(offset_90(scv=5), offset_180) ≤ 0.12 mm. At the
    # bisection's upper end offset_180 slightly dominates, so the drift
    # from the pure offset_180 answer (~9505) is small but nonzero.
    # Pin to a ±3% band around the expected pre-6a value.
    assert 9000.0 <= max_accel <= 9800.0
