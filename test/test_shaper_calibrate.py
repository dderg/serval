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


def test_get_shaper_smoothing_returns_offset_180_only_closed_form():
    """After 6a, _get_shaper_smoothing returns exactly offset_180:
        (accel / 2) * sigma2_T
    where sigma2_T = sum_i A_i (T_i - ts)**2 / sum_i A_i.

    For ZV @ 50Hz, damping 0.1: sigma2 ≈ 2.525e-5 s**2.
    At accel=10000 mm/s**2: offset_180 ≈ 0.1262 mm.
    """
    sc = shaper_calibrate.ShaperCalibrate(printer=None)
    A, T = _zv_50hz()
    D = sum(A)
    ts = sum(A_i * T_i for A_i, T_i in zip(A, T)) / D
    sigma2 = sum(A_i * (T_i - ts) ** 2 for A_i, T_i in zip(A, T)) / D
    accel = 10000.0
    expected = 0.5 * accel * sigma2
    actual = sc._get_shaper_smoothing(_zv_50hz(), accel=accel)
    assert actual == pytest.approx(expected, rel=1e-9)


def test_get_shaper_smoothing_drops_offset_90_at_low_accel():
    """At low accel + nonzero scv the OLD code's offset_90 term dominated,
    returning a larger number than pure offset_180. After 6a the function
    has no way to see scv, so at the same accel the returned value equals
    offset_180(accel). Picks accel=1000 where offset_90 (old scv=5.0)
    would have been ~0.027 mm vs offset_180 = 0.0126 mm.
    """
    sc = shaper_calibrate.ShaperCalibrate(printer=None)
    A, T = _zv_50hz()
    D = sum(A)
    ts = sum(A_i * T_i for A_i, T_i in zip(A, T)) / D
    sigma2 = sum(A_i * (T_i - ts) ** 2 for A_i, T_i in zip(A, T)) / D
    accel = 1000.0
    expected_offset_180 = 0.5 * accel * sigma2   # ≈ 0.01262 mm
    actual = sc._get_shaper_smoothing(_zv_50hz(), accel=accel)
    assert actual == pytest.approx(expected_offset_180, rel=1e-9)
    # Sanity: confirm we are in the regime where the OLD offset_90 would
    # have been strictly larger than offset_180.
    old_offset_90_rough = math.sqrt(2.0) * 0.5 * (5.0 + 0.5 * accel * (T[1] - ts)) * (T[1] - ts) / D
    assert old_offset_90_rough > expected_offset_180 * 1.5
