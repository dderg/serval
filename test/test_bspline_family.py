"""Plan 5 D1: cardinal B-spline chain family (bs1..bs5).

Covers:
  - Kernel integrates to unity over its support.
  - Kernel is symmetric about 0.
  - A_axis at f_sh=40, target_smoothing=0.12 matches the spec §D1 table
    within 1% for each variant.
  - Legacy flat-polynomial init_smoother helper still round-trips through
    the piecewise representation.

Plan 8 Chunk 2 Task 13: tests exercising the retired post-hoc shaper
infrastructure — bspline_inverse / fused kernel / extruder_smoother /
_marshal_pieces_to_buffer / get_axis_G — are retired along with the
modules they covered. The remaining tests pin the shaper_defs kernel
math and the migration-error story, both of which the planner
(blendplanner._bake_shaper_polynomial) still depends on.
"""

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


def test_init_smoother_pa_kernel_shape():
    """PA extruder smoother kernel (literal ``[15/8, 0, -15, 0, 30]``
    at t_sm=0.04): boundary value is zero, peak sits at t=0, and
    sigma^2 matches the closed-form expectation."""
    coeffs = [15.0 / 8.0, 0.0, -15.0, 0.0, 30.0]
    smooth_time = 0.04
    C_pieces, t_sm = shaper_defs.init_smoother(coeffs, smooth_time, True)
    assert len(C_pieces) == 1
    assert t_sm == pytest.approx(smooth_time)
    t_start, t_end, piece_coeffs = C_pieces[0]
    assert t_start == pytest.approx(-smooth_time / 2)
    assert t_end == pytest.approx(smooth_time / 2)

    endpoints = np.asarray(
        shaper_defs.bspline_eval(C_pieces, np.array([t_start, t_end]), t_sm)
    )
    np.testing.assert_allclose(endpoints, [0.0, 0.0], atol=1e-9)

    peak_val = shaper_defs.bspline_eval(C_pieces, np.array([0.0]), t_sm)[0]
    assert peak_val > 0.0

    sc = shaper_calibrate.ShaperCalibrate(printer=None)
    sigma2 = sc._get_smoother_sigma2((C_pieces, t_sm))
    expected_sigma2 = smooth_time * smooth_time / 28.0
    assert expected_sigma2 == pytest.approx(5.7142857e-5, rel=1e-6)
    assert sigma2 == pytest.approx(expected_sigma2, rel=1e-2)


def test_init_smoother_ascending_convention_pins_monomial():
    """Direct monomial check: input ``[0, 0, 1]`` must encode w_raw(t) = t^2."""
    smooth_time = 0.04
    C_pieces, _ = shaper_defs.init_smoother([0.0, 0.0, 1.0], smooth_time, False)
    piece_coeffs = C_pieces[0][2]
    assert piece_coeffs == [0.0, 0.0, 1.0]

    C_norm, _ = shaper_defs.init_smoother([0.0, 0.0, 1.0], smooth_time, True)
    piece_coeffs_norm = C_norm[0][2]
    assert piece_coeffs_norm[0] == 0.0
    assert piece_coeffs_norm[1] == 0.0
    assert piece_coeffs_norm[2] == pytest.approx(1.0 / smooth_time ** 3, rel=1e-12)


def test_update_shaper_accepts_smooth_is_name():
    """SET_INPUT_SHAPER SHAPER_TYPE=smooth_mzv must succeed — the smooth-IS
    family is first-class alongside the bs family."""
    from klippy.extras import input_shaper as _is_mod

    factory = _is_mod.ShaperFactory()

    class MockError(Exception):
        pass

    class MockGcmd:
        error = MockError

        def __init__(self, shaper_type):
            self._st = shaper_type

        def get(self, key, default=None):
            if key == "SHAPER_TYPE":
                return self._st
            return default

        def get_float(self, key, default, **kw):
            return default

    p = _is_mod.TypedInputSmootherParams("x", "bs3", None)
    p.smoother_freq = 40.0
    existing = _is_mod.AxisInputSmoother(p)
    updated = factory.update_shaper(existing, MockGcmd("smooth_mzv"))
    assert updated.get_type() == "smooth_mzv"


def test_create_shaper_accepts_smooth_is_name():
    """Config-load path: shaper_type = smooth_mzv must succeed — smooth-IS
    is a first-class member of INPUT_SMOOTHERS."""
    from klippy.extras import input_shaper as _is_mod

    factory = _is_mod.ShaperFactory()

    class MockError(Exception):
        pass

    class MockConfig:
        error = MockError

        def __init__(self, shaper_type):
            self._st = shaper_type

        def get(self, key, default=None):
            if key == "shaper_type":
                return self._st
            if key.startswith("shaper_type_"):
                return self._st
            return default

        def getfloat(self, k, v, **kw):
            return v

    created = factory.create_shaper("x", MockConfig("smooth_mzv"))
    assert created.get_type() == "smooth_mzv"


@pytest.mark.parametrize("m", BS_M)
def test_piecewise_moments_match_numerical_reference(m):
    """For a cardinal B-spline of order m, moments m_0, m_1, m_2 computed
    piecewise match a dense-grid numerical quadrature to ~5 digits."""
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
# Smoother catalog: both families (bs and smooth-IS) are first-class.
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("name", [
    "smooth_zv", "smooth_mzv", "smooth_ei",
    "smooth_2hump_ei", "smooth_zvd_ei", "smooth_si",
])
def test_smooth_is_variant_registered_and_composable(name):
    """Each smooth-IS variant appears in INPUT_SMOOTHERS and produces a
    well-formed one-piece kernel (support = [-t_sm/2, +t_sm/2],
    non-empty coefficient vector)."""
    entry = next(s for s in shaper_defs.INPUT_SMOOTHERS if s.name == name)
    assert entry.min_freq > 0.0
    C_pieces, t_sm = entry.init_func(40.0, 0.1, True)
    assert t_sm > 0.0
    assert len(C_pieces) == 1
    t_start, t_end, coeffs = C_pieces[0]
    assert t_start == pytest.approx(-t_sm / 2)
    assert t_end == pytest.approx(+t_sm / 2)
    assert len(coeffs) >= 5


def test_input_smoothers_catalog_covers_both_families():
    """INPUT_SMOOTHERS lists bs1..bs5 (cardinal B-spline chain) plus the
    six smooth-IS variants from the pre-Plan-5 design."""
    names = {s.name for s in shaper_defs.INPUT_SMOOTHERS}
    assert {"bs1", "bs2", "bs3", "bs4", "bs5"}.issubset(names)
    assert {"smooth_zv", "smooth_mzv", "smooth_ei",
            "smooth_2hump_ei", "smooth_zvd_ei", "smooth_si"}.issubset(names)


# ---------------------------------------------------------------------------
# Fourier-domain: sinc^(m+1) spectrum and first-zero invertibility gate.
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("m", [1, 2, 3, 4, 5])
def test_bspline_spectrum_is_sinc_power(m):
    """Numerical FT of bs_m matches closed-form sinc^(m+1) at representative
    frequencies."""
    f_sh = 40.0
    C_pieces, t_sm = shaper_defs.INPUT_SMOOTHERS[m - 1].init_func(f_sh, 0.1, True)
    T_1 = t_sm / (m + 1)
    grid = np.linspace(-t_sm / 2, t_sm / 2, 20001)
    w = shaper_defs.bspline_eval(C_pieces, grid, t_sm)
    for f in [5.0, 10.0, 15.0, 20.0, 25.0]:
        omega = 2 * np.pi * f
        W_numeric = np.trapezoid(w * np.cos(omega * grid), grid)
        W_expected = np.sinc(f * T_1) ** (m + 1)
        assert abs(W_numeric - W_expected) < 1e-4, (
            "m=%d, f=%.0f Hz: numeric=%.6f, expected=%.6f"
            % (m, f, W_numeric, W_expected))


@pytest.mark.parametrize("m,expected_first_zero_hz", [
    (1, 51.44), (2, 61.66), (3, 71.05), (4, 79.81), (5, 88.07),
])
def test_bspline_first_spectral_zero_above_f_sh(m, expected_first_zero_hz):
    """First zero of W(f) for cardinal B-spline is at f = (m+1)/T_sm.

    Must lie above 1.25 * f_sh for FIR-invertibility (Besset-Béarée 2017 §III).
    """
    f_sh = 40.0
    _, t_sm = shaper_defs.INPUT_SMOOTHERS[m - 1].init_func(f_sh, 0.1, True)
    first_zero = (m + 1) / t_sm
    assert abs(first_zero - expected_first_zero_hz) < 0.1, (
        "m=%d: computed %.2f Hz, expected %.2f Hz" % (m, first_zero,
                                                       expected_first_zero_hz))
    assert first_zero > 1.25 * f_sh, (
        "m=%d: first zero %.2f Hz is within 1.25*f_sh=%.1f Hz — "
        "violates FIR-invertibility precondition (Besset-Béarée §III)"
        % (m, first_zero, 1.25 * f_sh))
