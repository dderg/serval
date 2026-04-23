"""Tests for the linear pressure-advance polynomial composer
(klippy/chelper/linear_pa_compose.c).

Plan 8 Chunk 3 Task 2: validates the polynomial-arithmetic identity

    E(tau) = extr_r * P_proj(tau) + k_pa * V_proj(tau)

where P_proj is the XYZ position projected onto the unit motion direction
and V_proj is its derivative.
"""
from __future__ import annotations

import math

import pytest

from klippy.chelper.linear_pa_compose import linear_pa_compose


COEFFS_PER_PHASE = 15
AXES = 4  # (x, y, z, e)
PHASE_STRIDE = COEFFS_PER_PHASE * AXES  # 60


def _empty_buf(n_phases):
    return [0.0] * (n_phases * PHASE_STRIDE)


def _set(buf, phase, k, axis, value):
    buf[phase * PHASE_STRIDE + k * AXES + axis] = value


def _get(buf, phase, k, axis):
    return buf[phase * PHASE_STRIDE + k * AXES + axis]


def _eval_axis(buf, phase, axis, t):
    """Horner-eval the polynomial at phase-local time t for the given axis."""
    val = _get(buf, phase, COEFFS_PER_PHASE - 1, axis)
    for k in range(COEFFS_PER_PHASE - 2, -1, -1):
        val = val * t + _get(buf, phase, k, axis)
    return val


def test_zero_pa_e_matches_extr_r_times_xy_projection():
    """k_pa = 0: the composed E polynomial should be exactly extr_r times
    the projected XY polynomial. Verified via Horner-eval at multiple t.
    """
    n_phases = 1
    buf = _empty_buf(n_phases)
    # Single accel phase: x(t) = 5 + 100*t + 0.5 * 800 * t^2 (along X axis).
    # Direction n = (1, 0, 0) → P_proj = X polynomial.
    _set(buf, 0, 0, 0, 5.0)    # c[0].x
    _set(buf, 0, 1, 0, 100.0)  # c[1].x = velocity
    _set(buf, 0, 2, 0, 400.0)  # c[2].x = 0.5 * accel
    extr_r = 0.05  # filament mm per XY mm
    out = linear_pa_compose(
        n_phases, buf, axis_n=(1.0, 0.0, 0.0),
        extr_r=extr_r, k_pa=0.0,
    )
    # Verify: at any t, E(t) == extr_r * X(t).
    for t in [0.0, 0.01, 0.025, 0.05, 0.075, 0.1]:
        x_at_t = 5.0 + 100.0 * t + 400.0 * t * t
        e_at_t = _eval_axis(out, 0, 3, t)
        assert e_at_t == pytest.approx(extr_r * x_at_t, abs=1e-12), (
            f"zero-PA at t={t}: E={e_at_t}, expected {extr_r * x_at_t}"
        )
    # Verify .x/.y/.z preserved bit-identically.
    for k in range(COEFFS_PER_PHASE):
        for axis in range(3):
            assert _get(out, 0, k, axis) == _get(buf, 0, k, axis), (
                f"non-E slot mutated at k={k} axis={axis}"
            )


def test_linear_pa_accel_ramp_kicks_e_velocity():
    """On an accel ramp x(t) = v0*t + 0.5*a*t^2:
       v_x(t) = v0 + a*t, so E(t) = extr_r * x(t) + k_pa * (v0 + a*t).
       Sample at multiple t values and compare.
    """
    n_phases = 1
    buf = _empty_buf(n_phases)
    v0 = 50.0
    a = 1000.0
    _set(buf, 0, 1, 0, v0)        # c[1].x
    _set(buf, 0, 2, 0, 0.5 * a)   # c[2].x = a/2
    extr_r = 0.04
    k_pa = 0.05  # 50 ms PA time constant
    out = linear_pa_compose(
        n_phases, buf, axis_n=(1.0, 0.0, 0.0),
        extr_r=extr_r, k_pa=k_pa,
    )
    for t in [0.0, 0.005, 0.02, 0.05, 0.08, 0.1]:
        x_t = v0 * t + 0.5 * a * t * t
        v_t = v0 + a * t
        expected = extr_r * x_t + k_pa * v_t
        got = _eval_axis(out, 0, 3, t)
        assert got == pytest.approx(expected, abs=1e-10), (
            f"linear-PA accel-ramp at t={t}: got {got}, expected {expected}"
        )


def test_cruise_pa_velocity_term_constant_no_accel_kick():
    """On constant-velocity cruise (acceleration term zero): the linear-PA
    contribution is constant in t (k_pa * v_cruise). Therefore E's
    coefficients beyond degree 1 are zero (no accel kick in E)."""
    n_phases = 1
    buf = _empty_buf(n_phases)
    v_cruise = 80.0
    _set(buf, 0, 0, 0, 12.0)     # start position
    _set(buf, 0, 1, 0, v_cruise)  # constant velocity
    extr_r = 0.06
    k_pa = 0.04
    out = linear_pa_compose(
        n_phases, buf, axis_n=(1.0, 0.0, 0.0),
        extr_r=extr_r, k_pa=k_pa,
    )
    # E(t) = extr_r * (12 + v*t) + k_pa * v
    #      = (extr_r * 12 + k_pa * v) + extr_r * v * t
    expected_c0 = extr_r * 12.0 + k_pa * v_cruise
    expected_c1 = extr_r * v_cruise
    assert _get(out, 0, 0, 3) == pytest.approx(expected_c0)
    assert _get(out, 0, 1, 3) == pytest.approx(expected_c1)
    # All higher coefficients zero — no accel implies no PA kick on accel
    # E coefficients.
    for k in range(2, COEFFS_PER_PHASE):
        assert _get(out, 0, k, 3) == pytest.approx(0.0, abs=1e-12), (
            f"cruise PA produced non-zero E[{k}]"
        )


def test_diagonal_motion_projection():
    """45-degree XY motion: x(t) = y(t) = v*t/sqrt(2). Direction n =
    (1/sqrt(2), 1/sqrt(2), 0). Projection P_proj(t) = v*t (the arc length).
    Verifies the projection step doesn't double-count."""
    n_phases = 1
    buf = _empty_buf(n_phases)
    v = 100.0  # XY arc speed
    inv_sqrt2 = 1.0 / math.sqrt(2.0)
    # Per-axis velocity = v / sqrt(2).
    _set(buf, 0, 1, 0, v * inv_sqrt2)
    _set(buf, 0, 1, 1, v * inv_sqrt2)
    extr_r = 0.05
    k_pa = 0.0
    out = linear_pa_compose(
        n_phases, buf, axis_n=(inv_sqrt2, inv_sqrt2, 0.0),
        extr_r=extr_r, k_pa=k_pa,
    )
    for t in [0.0, 0.01, 0.05, 0.1]:
        # Projected position = v*t. E = extr_r * v*t.
        expected = extr_r * v * t
        got = _eval_axis(out, 0, 3, t)
        assert got == pytest.approx(expected, abs=1e-12)


def test_multi_phase_per_phase_independence():
    """Multi-phase composition: each phase composed independently from its
    own coefficients. Phases with all-zero XY content yield all-zero E."""
    n_phases = 3
    buf = _empty_buf(n_phases)
    # Phase 0: cruise at v=50 mm/s on X.
    _set(buf, 0, 0, 0, 0.0)
    _set(buf, 0, 1, 0, 50.0)
    # Phase 1: zero motion (gap).
    # Phase 2: cruise at v=75 mm/s on X (different speed).
    _set(buf, 2, 0, 0, 0.0)
    _set(buf, 2, 1, 0, 75.0)
    extr_r = 0.05
    k_pa = 0.03
    out = linear_pa_compose(
        n_phases, buf, axis_n=(1.0, 0.0, 0.0),
        extr_r=extr_r, k_pa=k_pa,
    )
    # Phase 0 E[0] = extr_r * 0 + k_pa * 50 = 1.5; E[1] = extr_r * 50 = 2.5.
    assert _get(out, 0, 0, 3) == pytest.approx(k_pa * 50.0)
    assert _get(out, 0, 1, 3) == pytest.approx(extr_r * 50.0)
    # Phase 1: all-zero input -> all-zero E.
    for k in range(COEFFS_PER_PHASE):
        assert _get(out, 1, k, 3) == pytest.approx(0.0, abs=1e-12)
    # Phase 2: parallel to phase 0 with v=75.
    assert _get(out, 2, 0, 3) == pytest.approx(k_pa * 75.0)
    assert _get(out, 2, 1, 3) == pytest.approx(extr_r * 75.0)
