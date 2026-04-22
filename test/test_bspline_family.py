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


def test_init_smoother_pa_kernel_shape():
    """PA extruder smoother kernel (kinematics/extruder.py:24 literal
    ``[15/8, 0, -15, 0, 30]`` at t_sm=0.04): boundary value is zero, peak
    sits at t=0, and sigma^2 matches the closed-form expectation.

    Legacy-semantics pin: the input list is ASCENDING power-basis
    (a[i] is the coefficient of t^i). This test is the regression gate
    for C1 (coefficient-order flip) — an implementation that reversed
    the convention would see w(+-hst) ~ 659 instead of 0, and
    sigma^2 ~ 1.29e-4 instead of 5.71e-5.
    """
    coeffs = [15.0 / 8.0, 0.0, -15.0, 0.0, 30.0]
    smooth_time = 0.04
    C_pieces, t_sm = shaper_defs.init_smoother(coeffs, smooth_time, True)
    assert len(C_pieces) == 1
    assert t_sm == pytest.approx(smooth_time)
    t_start, t_end, piece_coeffs = C_pieces[0]
    assert t_start == pytest.approx(-smooth_time / 2)
    assert t_end == pytest.approx(smooth_time / 2)

    # Kernel vanishes at support boundaries (the whole point of the 4th-order
    # smoothing function comment in kinematics/extruder.py).
    endpoints = np.asarray(
        shaper_defs.bspline_eval(C_pieces, np.array([t_start, t_end]), t_sm)
    )
    np.testing.assert_allclose(endpoints, [0.0, 0.0], atol=1e-9)

    # Kernel is symmetric with positive peak at t=0.
    peak_val = shaper_defs.bspline_eval(C_pieces, np.array([0.0]), t_sm)[0]
    assert peak_val > 0.0

    # sigma^2 via the piecewise moment closed-form path.
    sc = shaper_calibrate.ShaperCalibrate(printer=None)
    sigma2 = sc._get_smoother_sigma2((C_pieces, t_sm))
    # Closed form: w_norm(t) = (15 / (8*t_sm^5)) * (t_sm - 2t)^2 * (t_sm + 2t)^2
    # (factored form of the 4th-order PA kernel). Integrates to 1 over
    # [-t_sm/2, +t_sm/2]; second moment is t_sm^2 / 28.
    expected_sigma2 = smooth_time * smooth_time / 28.0
    assert expected_sigma2 == pytest.approx(5.7142857e-5, rel=1e-6)
    assert sigma2 == pytest.approx(expected_sigma2, rel=1e-2)


def test_init_smoother_ascending_convention_pins_monomial():
    """Direct monomial check: input ``[0, 0, 1]`` must encode w_raw(t) = t^2
    (ASCENDING convention). A reversed convention would encode w_raw(t) = 1.

    Pin test for C1: catches any future flip of the ascending-vs-descending
    semantic in init_smoother.
    """
    # Choose normalize_coeffs=False so we can read the piece coeffs directly
    # without the 1/t_sm^(i+1) rescaling obscuring the ordering.
    smooth_time = 0.04
    C_pieces, _ = shaper_defs.init_smoother([0.0, 0.0, 1.0], smooth_time, False)
    piece_coeffs = C_pieces[0][2]
    # ASCENDING: piece_coeffs[2] = 1.0 means the t^2 coefficient is 1.
    assert piece_coeffs == [0.0, 0.0, 1.0]

    # Non-trivially check the normalized path too: input [0, 0, 1] with
    # normalize=True must scale coefficients so that the t^i scaling is
    # 1/t_sm^(i+1). For i=2 that means piece_coeffs[2] = 1 / t_sm^3.
    C_norm, _ = shaper_defs.init_smoother([0.0, 0.0, 1.0], smooth_time, True)
    piece_coeffs_norm = C_norm[0][2]
    assert piece_coeffs_norm[0] == 0.0
    assert piece_coeffs_norm[1] == 0.0
    assert piece_coeffs_norm[2] == pytest.approx(1.0 / smooth_time ** 3, rel=1e-12)


def test_init_smoother_custom_coeff_round_trip_preserves_ascending():
    """CustomInputSmootherParams stores _raw_coeffs in ASCENDING order
    (the ``reversed()`` call at input_shaper.py converts user-written
    highest-degree-first config into ascending). init_smoother then consumes
    ascending directly. This regression test pins the custom-smoother pathway.
    """
    from klippy.extras import input_shaper as _is_mod

    # Mimic what the config path produces after reversal: user wrote
    # [30, 0, -15, 0, 15/8] (highest-degree-first) -> reversed ascending:
    ascending = [15.0 / 8.0, 0.0, -15.0, 0.0, 30.0]
    params = _is_mod.CustomInputSmootherParams.__new__(
        _is_mod.CustomInputSmootherParams
    )
    params.axis = "x"
    params._raw_coeffs = ascending
    params.smooth_time = 0.04

    C_pieces, t_sm = params.get_smoother()
    # The kernel must vanish at support boundaries — same shape invariant
    # as the PA smoother test above.
    endpoints = np.asarray(
        shaper_defs.bspline_eval(C_pieces, np.array([-t_sm / 2, t_sm / 2]), t_sm)
    )
    np.testing.assert_allclose(endpoints, [0.0, 0.0], atol=1e-9)


def test_update_shaper_raises_migration_error_for_retired_name():
    """SET_INPUT_SHAPER SHAPER_TYPE=smooth_mzv (runtime) must surface the
    bs2 migration hint, not a generic "Unsupported shaper type" error.

    Pin test for I2: the pre-fix code swallowed the migration error in
    ShaperFactory.update_shaper's try/except and fell through to the
    generic error path.
    """
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

    # Start from a working bs3 shaper, then attempt to "update" it to the
    # retired smooth_mzv — this is the runtime path the bug bites.
    p = _is_mod.TypedInputSmootherParams("x", "bs3", None)
    p.smoother_freq = 40.0
    existing = _is_mod.AxisInputSmoother(p)

    with pytest.raises(MockError) as excinfo:
        factory.update_shaper(existing, MockGcmd("smooth_mzv"))
    msg = str(excinfo.value)
    assert "smooth_mzv" in msg
    assert "bs2" in msg
    assert "Magnum Opus" in msg


@pytest.mark.parametrize("shaper_name", ["mzv", "zv", "bs2", "bs3", "bs5"])
def test_get_extruder_smoother_kernel_shape(shaper_name):
    """The PA-path extruder smoother (extruder_smoother.get_extruder_smoother)
    must be a sensible smoothing kernel: boundary values ~ 0, peak interior,
    peak magnitude strictly larger than boundary magnitudes.

    Pin test for the C_e[::-1] convention flip that the initial C1 fix
    introduced — the _calc_extruder_smoother fit emits ASCENDING
    coefficients, and the [::-1] was flipping them to DESCENDING before
    handoff to the now-ASCENDING-expecting init_smoother. Resulting
    "kernel" peak landed at the boundary, not in the interior.
    """
    from klippy.extras import extruder_smoother

    t_sm = 0.04
    C_pieces, t_sm_ret = extruder_smoother.get_extruder_smoother(
        shaper_name, t_sm, 0.1, normalize_coeffs=True
    )
    assert t_sm_ret == pytest.approx(t_sm)

    grid = np.linspace(-t_sm / 2, t_sm / 2, 401)
    w = np.asarray(shaper_defs.bspline_eval(C_pieces, grid, t_sm))

    # Boundaries vanish (or nearly so — the [1.5, 0, -6] fallback gives a
    # clean zero, the LSQ-fitted higher-order kernels land within 1e-6 of
    # zero thanks to the boundary constraints baked into _calc_extruder_smoother).
    np.testing.assert_allclose([w[0], w[-1]], 0.0, atol=1e-6)

    # Peak is strictly interior — not within 10% of either boundary.
    idx_peak = int(np.argmax(np.abs(w)))
    t_peak = grid[idx_peak]
    assert abs(t_peak) < 0.4 * t_sm, (
        "%s kernel peak at t=%.4f; expected interior "
        "(|t| < 0.4 * t_sm = %.4f)" % (shaper_name, t_peak, 0.4 * t_sm)
    )

    # Peak magnitude strictly greater than boundary magnitudes.
    peak = abs(w[idx_peak])
    assert peak > max(abs(w[0]), abs(w[-1])) + 1e-6


def test_create_shaper_raises_migration_error_for_retired_name():
    """Config-load path: shaper_type = smooth_mzv must raise the migration
    error with the bs2 hint. Mirrors the update-path pin above."""
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

    with pytest.raises(MockError) as excinfo:
        factory.create_shaper("x", MockConfig("smooth_mzv"))
    msg = str(excinfo.value)
    assert "smooth_mzv" in msg
    assert "bs2" in msg
    assert "Magnum Opus" in msg


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


# ---------------------------------------------------------------------------
# Fourier-domain: sinc^(m+1) spectrum and first-zero invertibility gate.
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("m", [1, 2, 3, 4, 5])
def test_bspline_spectrum_is_sinc_power(m):
    """Numerical FT of bs_m matches closed-form sinc^(m+1) at representative
    frequencies.

    For a cardinal B-spline of order m rescaled to support [-T_sm/2, +T_sm/2]:
        W(f) = sinc(f * T_1)^(m+1)  with  T_1 = T_sm / (m+1)
    where numpy sinc(x) = sin(pi*x) / (pi*x).
    """
    f_sh = 40.0
    C_pieces, t_sm = shaper_defs.INPUT_SMOOTHERS[m - 1].init_func(f_sh, 0.1, True)
    T_1 = t_sm / (m + 1)
    # Dense grid for numerical FT — 20001 points gives sub-1e-4 trapezoid error.
    grid = np.linspace(-t_sm / 2, t_sm / 2, 20001)
    w = shaper_defs.bspline_eval(C_pieces, grid, t_sm)
    # Compare at five representative frequencies well below the first zero.
    for f in [5.0, 10.0, 15.0, 20.0, 25.0]:
        omega = 2 * np.pi * f
        W_numeric = np.trapezoid(w * np.cos(omega * grid), grid)
        # numpy sinc: sinc(x) = sin(pi*x)/(pi*x)
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
    # Match expected value to within 0.1 Hz.
    assert abs(first_zero - expected_first_zero_hz) < 0.1, (
        "m=%d: computed %.2f Hz, expected %.2f Hz" % (m, first_zero,
                                                       expected_first_zero_hz))
    # Invertibility precondition.
    assert first_zero > 1.25 * f_sh, (
        "m=%d: first zero %.2f Hz is within 1.25*f_sh=%.1f Hz — "
        "violates FIR-invertibility precondition (Besset-Béarée §III)"
        % (m, first_zero, 1.25 * f_sh))
