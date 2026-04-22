"""Family-dispatch regression for target_smoothing runtime cap.

After smooth-shapers port: ShaperCalibrate.find_shaper_max_accel must
work for both impulse and smooth families. Uses the derivation in
docs/superpowers/specs/2026-04-20-target-smoothing-smooth-family.md.

The invariant test pin for the impulse branch is recorded from the
pre-refactor blend-arc worktree: mzv shaper at 50 Hz with zeta=0.1 and
target_smoothing=0.12 mm yields the float below. The smooth branch is
family-specific and uses the closed-form root A_crit = 2 * target /
sigma_T^2 (see spec §2.3, §2.4).
"""
import math

import pytest

from klippy.extras import shaper_calibrate, shaper_defs


TARGET_SMOOTHING = 0.12  # mm

# Recorded from pre-port blend-arc (Task 9 Step 1):
#   python -c "from klippy.extras import shaper_calibrate, shaper_defs; \
#              sc = shaper_calibrate.ShaperCalibrate(printer=None); \
#              s = shaper_defs.get_mzv_shaper(shaper_freq=50.0, \
#                                             damping_ratio=0.1); \
#              print(repr(sc.find_shaper_max_accel(s, \
#                                                  target_smoothing=0.12)))"
EXPECTED_ACCEL_MZV_50HZ = 7364.826583786309


def _sc():
    """Mirror the ShaperCalibrate construction used in test_shaper_calibrate."""
    return shaper_calibrate.ShaperCalibrate(printer=None)


def _smooth_mzv():
    # Plan 5 replacement: bs2 is the direct analog of the retired smooth_mzv.
    smoother_cfg = [s for s in shaper_defs.INPUT_SMOOTHERS
                    if s.name == "bs2"][0]
    return smoother_cfg.init_func(40.0, 0.1)


def test_impulse_family_unchanged_mzv_50hz():
    """After refactor, impulse branch must reproduce pre-refactor value."""
    sc = _sc()
    shaper = shaper_defs.get_mzv_shaper(shaper_freq=50.0, damping_ratio=0.1)
    accel = sc.find_shaper_max_accel(shaper, target_smoothing=TARGET_SMOOTHING)
    assert accel == pytest.approx(EXPECTED_ACCEL_MZV_50HZ, rel=1e-6)


def test_smooth_family_returns_finite_accel():
    """Smooth branch must return a positive, finite accel."""
    sc = _sc()
    sm = _smooth_mzv()
    accel = sc.find_shaper_max_accel(sm, target_smoothing=TARGET_SMOOTHING)
    assert math.isfinite(accel) and accel > 0.0


def test_smooth_family_tighter_at_smaller_budget():
    """Halving the target_smoothing budget must not raise the cap."""
    sc = _sc()
    sm = _smooth_mzv()
    accel_loose = sc.find_shaper_max_accel(sm, target_smoothing=0.24)
    accel_tight = sc.find_shaper_max_accel(sm, target_smoothing=0.12)
    assert accel_tight <= accel_loose + 1e-9


def test_dispatch_accepts_both_families():
    """Single public entry point, two families, no exceptions."""
    sc = _sc()
    mzv = shaper_defs.get_mzv_shaper(shaper_freq=50.0, damping_ratio=0.1)
    sm = _smooth_mzv()
    assert sc.find_shaper_max_accel(mzv, target_smoothing=TARGET_SMOOTHING) > 0
    assert sc.find_shaper_max_accel(sm, target_smoothing=TARGET_SMOOTHING) > 0


def test_smooth_family_closed_form_matches():
    """find_shaper_max_accel on a smoother must match A_crit = 2*t/sigma^2
    computed directly from the polynomial moments (spec §2.3, §2.4, and
    Plan 5 §D1 for the cardinal B-spline chain piecewise form)."""
    sc = _sc()
    sm = _smooth_mzv()
    C_pieces, t_sm = sm

    def raw_moment(k):
        s = 0.0
        for (a, b, coeffs) in C_pieces:
            for j, c in enumerate(coeffs):
                power = j + k + 1
                s += c * (b ** power - a ** power) / power
        return s

    M0 = raw_moment(0)
    ts = raw_moment(1) / M0
    sigma2 = raw_moment(2) / M0 - ts * ts
    expected = 2.0 * TARGET_SMOOTHING / sigma2
    actual = sc.find_shaper_max_accel(sm, target_smoothing=TARGET_SMOOTHING)
    assert actual == pytest.approx(expected, rel=1e-9)
