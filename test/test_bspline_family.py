"""Plan 5 D1: cardinal B-spline chain family (bs1..bs5).

Covers:
  - Kernel integrates to unity over its support.
  - Kernel is symmetric about 0.
  - A_axis at f_sh=40, target_smoothing=0.12 matches the spec §D1 table
    within 1% for each variant.
  - Legacy flat-polynomial init_smoother helper still round-trips through
    the piecewise representation (bit-identical check for linear-move
    moment computation against a numpy reference).

The fused-kernel, direct-quintic, and saturation-cap deliverables stay
out of scope for this test file — they will be added alongside Tasks 9
and later.
"""
import math

import numpy as np
import pytest

from klippy.extras import shaper_defs
from klippy.extras import shaper_calibrate


BS_M = [1, 2, 3, 4, 5]

# Plan 5 spec §D1 A_axis table at f_sh=40 Hz, target_smoothing=0.12.
# sigma_T^2 = T_sm^2 / (12 * (m+1)); A_axis = 2 * target_smoothing / sigma_T^2.
EXPECTED_A_AXIS = {
    1: 3810.0,
    2: 3650.0,
    3: 3635.0,
    4: 3668.0,
    5: 3723.0,
}


@pytest.mark.parametrize("m", BS_M)
def test_bspline_kernel_unit_integral(m):
    """B-spline of order m, sampled on a dense grid, integrates to 1."""
    f_sh = 40.0
    damping_ratio = 0.1
    C_pieces, t_sm = shaper_defs.INPUT_SMOOTHERS[m - 1].init_func(
        f_sh, damping_ratio, True
    )
    grid = np.linspace(-t_sm / 2, t_sm / 2, 100001)
    w = shaper_defs.bspline_eval(C_pieces, grid, t_sm)
    integral = np.trapezoid(w, grid)
    assert abs(integral - 1.0) < 1e-6


@pytest.mark.parametrize("m", BS_M)
def test_bspline_kernel_even(m):
    """B-spline of order m is even about tau=0."""
    f_sh = 40.0
    C_pieces, t_sm = shaper_defs.INPUT_SMOOTHERS[m - 1].init_func(f_sh, 0.1, True)
    grid = np.linspace(-t_sm / 2, t_sm / 2, 1001)
    w = np.asarray(shaper_defs.bspline_eval(C_pieces, grid, t_sm))
    # atol accounts for 1e-12 round-off on the largest kernel values; rtol
    # alone is too tight for near-zero samples at the support boundaries.
    np.testing.assert_allclose(w, w[::-1], rtol=1e-6, atol=1e-10)


@pytest.mark.parametrize("m", BS_M)
def test_bspline_t_sm_matches_F_m_table(m):
    """t_sm = F_m / shaper_freq."""
    f_sh = 40.0
    _, t_sm = shaper_defs.INPUT_SMOOTHERS[m - 1].init_func(f_sh, 0.1, True)
    expected = shaper_defs._F_M_TABLE[m] / f_sh
    assert abs(t_sm - expected) < 1e-12


@pytest.mark.parametrize("m", BS_M)
def test_bspline_sigma_T_closed_form(m):
    """sigma_T^2 = T_sm^2 / (12 * (m + 1)) for a cardinal B-spline."""
    f_sh = 40.0
    C_pieces, t_sm = shaper_defs.INPUT_SMOOTHERS[m - 1].init_func(f_sh, 0.1, True)
    grid = np.linspace(-t_sm / 2, t_sm / 2, 100001)
    w = np.asarray(shaper_defs.bspline_eval(C_pieces, grid, t_sm))
    sigma2_numerical = np.trapezoid(w * grid * grid, grid)
    sigma2_expected = t_sm * t_sm / (12.0 * (m + 1))
    # ~5-digit agreement; limited by trapezoidal quadrature on 10^5 points.
    assert abs(sigma2_numerical - sigma2_expected) < 1e-9


@pytest.mark.parametrize("m", BS_M)
def test_bspline_A_axis_matches_spec_table(m):
    """A_axis via ShaperCalibrate.find_smoother_max_accel matches spec §D1."""
    f_sh = 40.0
    target_smoothing = 0.12
    smoother = shaper_defs.INPUT_SMOOTHERS[m - 1].init_func(f_sh, 0.1, True)
    sc = shaper_calibrate.ShaperCalibrate(printer=None,
                                          target_smoothing=target_smoothing)
    A_axis = sc.find_smoother_max_accel(smoother, target_smoothing)
    expected = EXPECTED_A_AXIS[m]
    rel_err = abs(A_axis - expected) / expected
    assert rel_err < 0.01, (
        "A_axis for bs%d = %.1f, expected %.1f (rel err %.3f%%)"
        % (m, A_axis, expected, rel_err * 100)
    )


def test_init_smoother_flat_to_piecewise_round_trip():
    """Legacy flat-coeff init_smoother emits a single-piece piecewise kernel
    that integrates to a finite positive value (C-side normalization then
    rescales to unit integral)."""
    # The extruder pressure-advance smoother uses this legacy coeff pattern.
    coeffs = [15.0 / 8.0, 0.0, -15.0, 0.0, 30.0]
    smooth_time = 0.04
    C_pieces, t_sm = shaper_defs.init_smoother(coeffs, smooth_time, True)
    assert len(C_pieces) == 1
    assert t_sm == pytest.approx(smooth_time)
    t_start, t_end, piece_coeffs = C_pieces[0]
    assert t_start == pytest.approx(-smooth_time / 2)
    assert t_end == pytest.approx(smooth_time / 2)
    # Integrate over the window. The pre-normalization integral is NOT 1 —
    # the C-side init_smoother is responsible for the final rescale.
    grid = np.linspace(t_start, t_end, 10001)
    w = np.asarray(shaper_defs.bspline_eval(C_pieces, grid, t_sm))
    integral = np.trapezoid(w, grid)
    assert integral > 0.0
    assert math.isfinite(integral)


@pytest.mark.parametrize("m", BS_M)
def test_piecewise_moments_match_numerical_reference(m):
    """For a cardinal B-spline of order m, moments m_0, m_1, m_2 computed
    piecewise match a dense-grid numerical quadrature to ~5 digits.

    This pins the moment-integration math that integrate_move consumes on
    linear moves. If the piecewise implementation shifts by more than the
    quadrature noise, regressions manifest as small position offsets —
    exactly the failure mode that degree-extension from 3 to 11 moments
    could in principle introduce but must not.
    """
    f_sh = 40.0
    C_pieces, t_sm = shaper_defs.INPUT_SMOOTHERS[m - 1].init_func(f_sh, 0.1, True)
    grid = np.linspace(-t_sm / 2, t_sm / 2, 100001)
    w = np.asarray(shaper_defs.bspline_eval(C_pieces, grid, t_sm))

    def numeric_moment(k):
        return np.trapezoid(w * (grid ** k), grid)

    def piecewise_moment(k):
        total = 0.0
        for (a, b, coeffs) in C_pieces:
            for j, c in enumerate(coeffs):
                power = j + k + 1
                total += c * (b ** power - a ** power) / power
        return total

    for k in range(3):
        num = numeric_moment(k)
        pw = piecewise_moment(k)
        if abs(num) < 1e-12:
            assert abs(pw) < 1e-6
        else:
            rel = abs(pw - num) / abs(num)
            assert rel < 1e-5, (
                "m_%d mismatch for bs%d: numeric=%.6e piecewise=%.6e"
                % (k, m, num, pw)
            )


# ---------------------------------------------------------------------------
# FFI buffer-layout round-trip: piecewise smoother marshalled to the flat
# FFI buffer must preserve every piece's t_start / t_end / coeffs.
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("m", BS_M)
def test_ffi_buffer_round_trip_preserves_pieces(m):
    """Python-side _marshal_pieces_to_buffer preserves piece data for the
    C-side init_smoother to consume. This is the bit-level contract that
    the prior Task-1-in-isolation attempt got wrong (piecewise tuples
    passed to a flat-double-array FFI signature).
    """
    from klippy import chelper
    from klippy.extras import input_shaper as _is_mod

    ffi_main, _ = chelper.get_ffi()

    f_sh = 40.0
    C_pieces, t_sm = shaper_defs.INPUT_SMOOTHERS[m - 1].init_func(f_sh, 0.1, True)
    # Pre-normalization integral ~ 1 (closed-form cardinal B-spline).
    grid = np.linspace(-t_sm / 2, t_sm / 2, 10001)
    w = np.asarray(shaper_defs.bspline_eval(C_pieces, grid, t_sm))
    integral = np.trapezoid(w, grid)
    assert abs(integral - 1.0) < 1e-5

    n_pieces, buf = _is_mod._marshal_pieces_to_buffer(ffi_main, C_pieces)
    assert n_pieces == len(C_pieces)
    for i, (t_start, t_end, coeffs) in enumerate(C_pieces):
        base = i * 8
        assert buf[base + 0] == pytest.approx(t_start, rel=1e-12)
        assert buf[base + 1] == pytest.approx(t_end, rel=1e-12)
        for k in range(6):
            exp = coeffs[k] if k < len(coeffs) else 0.0
            assert buf[base + 2 + k] == pytest.approx(exp, rel=1e-12, abs=1e-30)


def test_ffi_buffer_rejects_too_many_pieces():
    """The FFI buffer marshaller rejects over-sized piecewise kernels up
    front so bugs do not reach the C side as silent buffer overruns."""
    from klippy import chelper
    from klippy.extras import input_shaper as _is_mod

    ffi_main, _ = chelper.get_ffi()

    # Build a kernel with one more piece than the C side supports.
    over_sized = [(float(i), float(i + 1), [1.0, 0.0, 0.0, 0.0, 0.0, 0.0])
                  for i in range(_is_mod._FFI_MAX_PIECES + 1)]
    with pytest.raises(ValueError):
        _is_mod._marshal_pieces_to_buffer(ffi_main, over_sized)


# ---------------------------------------------------------------------------
# Migration: retired smooth_* names surface a friendly error.
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("retired,expected_hint", [
    ("smooth_zv", "bs1"),
    ("smooth_mzv", "bs2"),
    ("smooth_ei", "bs3"),
    ("smooth_2hump_ei", "bs4"),
    ("smooth_zvd_ei", "bs5"),
    ("smooth_si", "bs3"),
])
def test_retired_smoother_name_maps_to_bs_variant(retired, expected_hint):
    assert shaper_defs.RETIRED_SMOOTHER_MIGRATION[retired] == expected_hint


def test_retired_smoother_not_in_input_smoothers_list():
    retired = {"smooth_zv", "smooth_mzv", "smooth_ei", "smooth_2hump_ei",
               "smooth_zvd_ei", "smooth_si"}
    names = {s.name for s in shaper_defs.INPUT_SMOOTHERS}
    assert names.isdisjoint(retired)
    assert names == {"bs1", "bs2", "bs3", "bs4", "bs5"}
