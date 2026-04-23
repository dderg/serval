"""Tests for the degree-4 Chebyshev piecewise polynomial fitter
(klippy/chelper/cheb_fit.c).

Plan 8 Chunk 3 Task 5.
"""
from __future__ import annotations

import math

import pytest

from klippy.chelper.cheb_fit import (
    CHEB_FIT_COEFFS,
    cheb_eval_mono,
    cheb_fit_function,
    cheb_fit_interval,
    cheb_fit_piecewise,
    cheb_nodes,
)


def test_nodes_in_increasing_order_and_endpoints():
    nodes = cheb_nodes(2.0, 7.0)
    assert len(nodes) == 5
    # Gauss-Lobatto-Chebyshev nodes include both endpoints.
    assert nodes[0] == pytest.approx(2.0)
    assert nodes[-1] == pytest.approx(7.0)
    for i in range(4):
        assert nodes[i] < nodes[i + 1]


def test_zero_function_yields_zero_coeffs():
    samples = [0.0] * CHEB_FIT_COEFFS
    mono = cheb_fit_interval(samples)
    for c in mono:
        assert c == pytest.approx(0.0, abs=1e-15)


def test_constant_function_recovered_exactly():
    samples = [3.5] * CHEB_FIT_COEFFS
    mono = cheb_fit_interval(samples)
    assert mono[0] == pytest.approx(3.5)
    for k in range(1, 5):
        assert mono[k] == pytest.approx(0.0, abs=1e-12)


def test_linear_function_recovered_exactly():
    # f(v) = 2 + 3*v, on [1, 5]. After mapping to t = 2*(v-1)/4 - 1:
    #   v = 3 + 2*t, so f = 2 + 3*(3 + 2*t) = 11 + 6*t.
    v_lo, v_hi = 1.0, 5.0
    f = lambda v: 2.0 + 3.0 * v
    nodes = cheb_nodes(v_lo, v_hi)
    samples = [f(v) for v in nodes]
    mono = cheb_fit_interval(samples)
    assert mono[0] == pytest.approx(11.0, abs=1e-12)
    assert mono[1] == pytest.approx(6.0, abs=1e-12)
    for k in range(2, 5):
        assert mono[k] == pytest.approx(0.0, abs=1e-12)
    # Cross-check via eval.
    for v in [1.0, 2.5, 3.3, 5.0]:
        assert cheb_eval_mono(mono, v_lo, v_hi, v) == pytest.approx(f(v),
                                                                    abs=1e-12)


def test_quartic_function_recovered_exactly():
    # Degree-4 interpolation is exact on any polynomial of degree <= 4.
    v_lo, v_hi = -2.0, 3.0
    coeffs_v = [0.3, -0.7, 1.1, -0.4, 0.05]
    f = lambda v: sum(c * v**k for k, c in enumerate(coeffs_v))
    nodes = cheb_nodes(v_lo, v_hi)
    samples = [f(v) for v in nodes]
    mono = cheb_fit_interval(samples)
    for v in [-2.0, -1.5, 0.0, 1.25, 2.9, 3.0]:
        approx = cheb_eval_mono(mono, v_lo, v_hi, v)
        assert approx == pytest.approx(f(v), abs=1e-10)


def test_tanh_fit_over_global_range_max_error_bound():
    # Research target: with 5 pieces deg-4 on [0, 12.5], tanh fit error
    # below 2e-4. Use the recommended breakpoints at v_lin and 2.5 v_lin
    # plus two more to reach 5 pieces.
    v_lin = 1.0  # normalize x = v/v_lin = v
    v_lo, v_hi = 0.0, 12.5
    breaks = [1.0, 2.5, 5.0, 8.0]  # 5 pieces
    f = lambda v: math.tanh(v / v_lin)
    mono, bounds, err = cheb_fit_function(
        f, v_lo, v_hi, breaks, residual_samples=401,
    )
    # Per pa_piecewise_fit.md §3.1: 5-piece deg-4 tanh adaptive ~< 1e-3.
    # The plan text cites "<2e-4" for a well-placed 5-piece fit; our four
    # breakpoints are near the research-recommended set.
    assert err < 2e-4


def test_recipr_fit_over_global_range_max_error_bound():
    v_lin = 1.0
    v_lo, v_hi = 0.0, 12.5
    breaks = [1.0, 2.5, 5.0, 8.0]
    f = lambda v: 1.0 - 1.0 / (1.0 + v / v_lin)
    mono, bounds, err = cheb_fit_function(
        f, v_lo, v_hi, breaks, residual_samples=401,
    )
    # Research §3.2 cites ~7.1e-3 for 5-piece deg-4 recipr on the global
    # [0, 12.5] range with UNIFORM partition. Our adaptive breaks at
    # {1, 2.5, 5, 8} cluster pieces where recipr curvature is highest
    # (near v=0). Empirically ~4.3e-4 — well below the research bound
    # and below the 1 µm filament budget at nonlinear_offset ~ 0.1.
    assert err < 1e-3


def test_piecewise_continuity_at_breakpoints():
    # The fitter interpolates at endpoints of each sub-interval (Gauss-
    # Lobatto nodes include ±1), so adjacent pieces must agree at their
    # shared breakpoint.
    v_lin = 1.0
    v_lo, v_hi = 0.0, 12.5
    breaks = [1.0, 2.5, 5.0, 8.0]
    f = lambda v: math.tanh(v / v_lin)
    mono, bounds, _ = cheb_fit_function(
        f, v_lo, v_hi, breaks, residual_samples=2,
    )
    for i, br in enumerate(breaks):
        left = cheb_eval_mono(mono[i], bounds[i], bounds[i + 1], br)
        right = cheb_eval_mono(mono[i + 1], bounds[i + 1], bounds[i + 2], br)
        assert left == pytest.approx(right, abs=1e-12)
        assert left == pytest.approx(f(br), abs=1e-12)


def test_piecewise_bad_breaks_rejected():
    # Break outside (v_lo, v_hi) should fail.
    samples = [[0.0] * CHEB_FIT_COEFFS, [0.0] * CHEB_FIT_COEFFS]
    with pytest.raises(ValueError):
        cheb_fit_piecewise(0.0, 1.0, [1.5], samples)
    # Non-monotonic.
    samples3 = [[0.0] * CHEB_FIT_COEFFS] * 3
    with pytest.raises(ValueError):
        cheb_fit_piecewise(0.0, 1.0, [0.7, 0.5], samples3)


def test_shifted_tanh_for_v0_exact_mitigation():
    # Per research §5.1: fit g(v) = f(v) - f(0) so g(0) = 0 exactly,
    # then add f(0) back. Verifies the single-piece fit of g agrees with
    # g at v=0 to machine precision.
    v_lin = 1.0
    v_lo, v_hi = 0.0, 1.0
    f = lambda v: math.tanh(v / v_lin)
    f0 = f(0.0)  # 0.0 but keep general.
    g = lambda v: f(v) - f0
    nodes = cheb_nodes(v_lo, v_hi)
    samples = [g(v) for v in nodes]
    mono = cheb_fit_interval(samples)
    approx_at_0 = cheb_eval_mono(mono, v_lo, v_hi, 0.0)
    assert approx_at_0 == pytest.approx(0.0, abs=1e-14)
