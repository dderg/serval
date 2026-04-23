"""Tests for the non-linear pressure-advance polynomial composer
(klippy/chelper/nonlinear_pa_compose.c).

Plan 8 Chunk 3 Task 6. Validates:
  * Linear-path equivalence when model is NONE or nonlinear_offset is 0.
  * tanh / recipr E position matches direct computation via polynomial
    evaluation + pa_f_nonlin to within the 1 µm filament budget.
  * Cruise at v = v_lin/2: E is approximately linear (f_nonlin is near
    linear for small x).
  * Saturation pass: accel through v >> v_lin accumulates the expected
    asymptotic contribution.
"""
from __future__ import annotations

import math

import pytest

from klippy.chelper.nonlinear_pa_compose import nonlinear_pa_compose

COEFFS_PER_PHASE = 15
AXES = 4
PHASE_STRIDE = COEFFS_PER_PHASE * AXES  # 60
E_SLOT = 3


def _empty_buf(n_phases):
    return [0.0] * (n_phases * PHASE_STRIDE)


def _set(buf, phase, k, axis, value):
    buf[phase * PHASE_STRIDE + k * AXES + axis] = value


def _get(buf, phase, k, axis):
    return buf[phase * PHASE_STRIDE + k * AXES + axis]


def _eval_axis(buf, phase, axis, tau):
    val = _get(buf, phase, COEFFS_PER_PHASE - 1, axis)
    for k in range(COEFFS_PER_PHASE - 2, -1, -1):
        val = val * tau + _get(buf, phase, k, axis)
    return val


def _f_tanh(v, v_lin):
    return math.tanh(max(v, 0.0) / v_lin) if v_lin > 0 else 0.0


def _f_recipr(v, v_lin):
    if v_lin <= 0:
        return 0.0
    x = max(v, 0.0) / v_lin
    return 1.0 - 1.0 / (1.0 + x)


def test_none_model_matches_linear_path_zero_offset():
    """With model=None (or 0 nonlinear_offset), the composer falls back
    to exact linear-only arithmetic. Equivalent to linear_pa_compose
    with k_pa = linear_advance.
    """
    n_phases = 1
    buf = _empty_buf(n_phases)
    v0 = 80.0
    a = 500.0
    _set(buf, 0, 1, 0, v0)
    _set(buf, 0, 2, 0, 0.5 * a)
    extr_r = 0.04
    linear_advance = 0.03
    T = 0.1
    out, residual = nonlinear_pa_compose(
        n_phases, [T], buf,
        axis_n=(1.0, 0.0, 0.0), extr_r=extr_r,
        linear_advance=linear_advance, nonlinear_offset=0.0,
        linearization_velocity=0.0,  # unused when offset is 0
        model=None,
    )
    assert residual == pytest.approx(0.0)
    # For any tau: E = extr_r * (v0*tau + 0.5*a*tau^2) + la * (v0 + a*tau).
    for tau in [0.0, 0.025, 0.05, 0.08, 0.1]:
        x_t = v0 * tau + 0.5 * a * tau * tau
        v_t = v0 + a * tau
        expected = extr_r * x_t + linear_advance * v_t
        got = _eval_axis(out, 0, E_SLOT, tau)
        assert got == pytest.approx(expected, abs=1e-10)


def test_tanh_cruise_at_half_vlin_matches_direct_evaluation():
    """Pure cruise at v = 0.5 * v_lin: V_proj(tau) is constant, so the
    non-linear contribution is a constant NO * tanh(0.5) added to the
    linear-PA E polynomial. Verified at multiple tau sample points.
    """
    n_phases = 1
    buf = _empty_buf(n_phases)
    v_lin = 100.0
    v_cruise = 50.0
    _set(buf, 0, 0, 0, 0.0)
    _set(buf, 0, 1, 0, v_cruise)
    extr_r = 0.05
    linear_advance = 0.03
    nonlinear_offset = 0.05
    T = 0.2
    out, residual = nonlinear_pa_compose(
        n_phases, [T], buf,
        axis_n=(1.0, 0.0, 0.0), extr_r=extr_r,
        linear_advance=linear_advance,
        nonlinear_offset=nonlinear_offset,
        linearization_velocity=v_lin,
        model="tanh",
    )
    # Degenerate: constant v implies constant g, so fit is exact.
    assert residual < 1e-12
    for tau in [0.0, 0.05, 0.1, 0.15, 0.2]:
        x_t = v_cruise * tau
        v_t = v_cruise
        expected = (extr_r * x_t
                    + linear_advance * v_t
                    + nonlinear_offset * _f_tanh(v_t, v_lin))
        got = _eval_axis(out, 0, E_SLOT, tau)
        assert got == pytest.approx(expected, abs=1e-10)


def test_tanh_accel_ramp_through_vlin_filament_budget():
    """Accel ramp through v = v_lin: velocity sweeps [0, 2*v_lin] over
    the phase, passing through the tanh inflection. The Chebyshev fit
    on a single piece should keep E position within the 1 µm filament
    budget at nonlinear_offset = 0.05.
    """
    n_phases = 1
    buf = _empty_buf(n_phases)
    v_lin = 40.0
    v0 = 0.0
    a = 400.0  # 0 -> 80 mm/s over 0.2 s, spans 2*v_lin
    T = 0.2
    _set(buf, 0, 1, 0, v0)
    _set(buf, 0, 2, 0, 0.5 * a)
    extr_r = 0.05
    linear_advance = 0.03
    nonlinear_offset = 0.05
    out, residual = nonlinear_pa_compose(
        n_phases, [T], buf,
        axis_n=(1.0, 0.0, 0.0), extr_r=extr_r,
        linear_advance=linear_advance,
        nonlinear_offset=nonlinear_offset,
        linearization_velocity=v_lin,
        model="tanh",
    )
    filament_err = residual * nonlinear_offset
    # 1 µm = 1e-3 mm filament budget per Phase 0 research §6.
    assert filament_err < 1e-3
    # Verify at a dense grid the E polynomial matches direct
    # computation within filament budget.
    for tau in [0.0, 0.02, 0.05, 0.1, 0.15, 0.18, 0.2]:
        x_t = v0 * tau + 0.5 * a * tau * tau
        v_t = v0 + a * tau
        expected = (extr_r * x_t
                    + linear_advance * v_t
                    + nonlinear_offset * _f_tanh(v_t, v_lin))
        got = _eval_axis(out, 0, E_SLOT, tau)
        assert abs(got - expected) < 1e-3


def test_recipr_accel_ramp_filament_budget():
    """recipr model: same test shape, tighter bound since recipr is
    smoother than tanh near the inflection."""
    n_phases = 1
    buf = _empty_buf(n_phases)
    v_lin = 40.0
    v0 = 0.0
    a = 400.0
    T = 0.2
    _set(buf, 0, 1, 0, v0)
    _set(buf, 0, 2, 0, 0.5 * a)
    extr_r = 0.05
    linear_advance = 0.0
    nonlinear_offset = 0.08
    out, residual = nonlinear_pa_compose(
        n_phases, [T], buf,
        axis_n=(1.0, 0.0, 0.0), extr_r=extr_r,
        linear_advance=linear_advance,
        nonlinear_offset=nonlinear_offset,
        linearization_velocity=v_lin,
        model="recipr",
    )
    filament_err = residual * nonlinear_offset
    assert filament_err < 1e-3
    for tau in [0.0, 0.04, 0.1, 0.16, 0.2]:
        x_t = v0 * tau + 0.5 * a * tau * tau
        v_t = v0 + a * tau
        expected = (extr_r * x_t
                    + linear_advance * v_t
                    + nonlinear_offset * _f_recipr(v_t, v_lin))
        got = _eval_axis(out, 0, E_SLOT, tau)
        assert abs(got - expected) < 1e-3


def test_saturation_region_accumulates_asymptote():
    """Accel from 2*v_lin to 4*v_lin: the tanh nonlinear contribution
    is near saturation (~NO * 1.0 asymptotic), so the E-polynomial's
    non-linear term is nearly constant. Verifies direct evaluation
    matches."""
    n_phases = 1
    buf = _empty_buf(n_phases)
    v_lin = 40.0
    v_start = 80.0  # 2 v_lin
    a = 800.0  # -> 160 mm/s = 4 v_lin over 0.1 s
    T = 0.1
    _set(buf, 0, 1, 0, v_start)
    _set(buf, 0, 2, 0, 0.5 * a)
    extr_r = 0.05
    linear_advance = 0.02
    nonlinear_offset = 0.05
    out, residual = nonlinear_pa_compose(
        n_phases, [T], buf,
        axis_n=(1.0, 0.0, 0.0), extr_r=extr_r,
        linear_advance=linear_advance,
        nonlinear_offset=nonlinear_offset,
        linearization_velocity=v_lin,
        model="tanh",
    )
    filament_err = residual * nonlinear_offset
    # Saturation region is smooth — tiny fit error expected.
    assert filament_err < 1e-4
    for tau in [0.0, 0.03, 0.05, 0.08, 0.1]:
        v_t = v_start + a * tau
        x_t = v_start * tau + 0.5 * a * tau * tau
        expected = (extr_r * x_t
                    + linear_advance * v_t
                    + nonlinear_offset * _f_tanh(v_t, v_lin))
        got = _eval_axis(out, 0, E_SLOT, tau)
        assert abs(got - expected) < 1e-3


def test_multi_phase_independent_fit():
    """Two phases with different tau durations and velocity spans are
    composed independently. Each phase's .e polynomial evaluates to
    the correct E at tau in its phase-local range.
    """
    n_phases = 2
    buf = _empty_buf(n_phases)
    v_lin = 40.0
    # Phase 0: accel from 0 to 60 over T0 = 0.15.
    a0 = 400.0
    T0 = 0.15
    _set(buf, 0, 1, 0, 0.0)
    _set(buf, 0, 2, 0, 0.5 * a0)
    # Phase 1: cruise at 60 over T1 = 0.1.
    v_cruise = 60.0
    T1 = 0.1
    _set(buf, 1, 0, 0, 0.0)
    _set(buf, 1, 1, 0, v_cruise)
    # Absolute move-local phase_t_ends: [T0, T0 + T1]
    phase_t_ends = [T0, T0 + T1]
    extr_r = 0.05
    linear_advance = 0.02
    nonlinear_offset = 0.05
    out, residual = nonlinear_pa_compose(
        n_phases, phase_t_ends, buf,
        axis_n=(1.0, 0.0, 0.0), extr_r=extr_r,
        linear_advance=linear_advance,
        nonlinear_offset=nonlinear_offset,
        linearization_velocity=v_lin,
        model="tanh",
    )
    assert residual * nonlinear_offset < 1e-3
    # Phase 0: sample at tau in [0, T0].
    for tau in [0.0, 0.05, 0.1, T0]:
        v_t = a0 * tau
        x_t = 0.5 * a0 * tau * tau
        expected = (extr_r * x_t
                    + linear_advance * v_t
                    + nonlinear_offset * _f_tanh(v_t, v_lin))
        got = _eval_axis(out, 0, E_SLOT, tau)
        assert abs(got - expected) < 1e-3
    # Phase 1: sample at tau in [0, T1].
    for tau in [0.0, 0.03, 0.07, T1]:
        v_t = v_cruise
        x_t = v_cruise * tau
        expected = (extr_r * x_t
                    + linear_advance * v_t
                    + nonlinear_offset * _f_tanh(v_t, v_lin))
        got = _eval_axis(out, 1, E_SLOT, tau)
        assert abs(got - expected) < 1e-10  # constant v, exact fit


def test_v0_exact_at_start_of_phase():
    """Per Phase 0 §5.1 mitigation: g(0) = 0 should be exact at tau=0
    (no DC kick). Verify by starting the phase at v=0 and inspecting
    the E coefficient c[0] which equals E(tau=0).
    """
    n_phases = 1
    buf = _empty_buf(n_phases)
    v_lin = 40.0
    a = 400.0
    T = 0.1
    _set(buf, 0, 0, 0, 0.0)
    _set(buf, 0, 1, 0, 0.0)
    _set(buf, 0, 2, 0, 0.5 * a)
    extr_r = 0.05
    linear_advance = 0.0
    nonlinear_offset = 0.08
    out, _ = nonlinear_pa_compose(
        n_phases, [T], buf,
        axis_n=(1.0, 0.0, 0.0), extr_r=extr_r,
        linear_advance=linear_advance,
        nonlinear_offset=nonlinear_offset,
        linearization_velocity=v_lin,
        model="tanh",
    )
    # c[0].e = E(tau=0). At v=0 the nonlinear contribution is 0, and
    # linear_advance*V = 0, and extr_r * P = 0 (c[0].x = 0). So c[0].e
    # should be exactly 0.
    assert _get(out, 0, 0, E_SLOT) == pytest.approx(0.0, abs=1e-14)


def test_diagonal_motion_projection_nonlinear():
    """45-degree XY motion: nonlinear fit uses axis_n projection to
    recover scalar V. Verify that E matches direct computation along
    the diagonal."""
    n_phases = 1
    buf = _empty_buf(n_phases)
    inv_sqrt2 = 1.0 / math.sqrt(2.0)
    v = 100.0  # arc speed
    _set(buf, 0, 1, 0, v * inv_sqrt2)
    _set(buf, 0, 1, 1, v * inv_sqrt2)
    v_lin = 40.0
    T = 0.1
    extr_r = 0.05
    nonlinear_offset = 0.06
    out, residual = nonlinear_pa_compose(
        n_phases, [T], buf,
        axis_n=(inv_sqrt2, inv_sqrt2, 0.0), extr_r=extr_r,
        linear_advance=0.0, nonlinear_offset=nonlinear_offset,
        linearization_velocity=v_lin, model="tanh",
    )
    # Constant v -> exact fit.
    assert residual < 1e-12
    for tau in [0.0, 0.05, 0.1]:
        proj_x = v * tau
        expected = (extr_r * proj_x
                    + nonlinear_offset * _f_tanh(v, v_lin))
        got = _eval_axis(out, 0, E_SLOT, tau)
        assert got == pytest.approx(expected, abs=1e-10)
