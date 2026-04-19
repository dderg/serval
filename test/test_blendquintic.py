# test/test_blendquintic.py
import math

import pytest

from klippy import blendquintic


def test_module_imports():
    assert blendquintic is not None


def test_quintic_eval_at_endpoints_returns_Q0_and_Q5():
    Q = [
        (0.0, 0.0, 0.0),
        (1.0, 0.0, 0.0),
        (2.0, 0.0, 0.0),
        (3.0, 1.0, 0.0),
        (4.0, 2.0, 0.0),
        (5.0, 3.0, 0.0),
    ]
    p0 = blendquintic._quintic_eval(Q, 0.0)
    p1 = blendquintic._quintic_eval(Q, 1.0)
    assert p0 == pytest.approx(Q[0])
    assert p1 == pytest.approx(Q[5])


def test_quintic_eval_mid_matches_bernstein_direct():
    Q = [
        (0.0, 0.0, 0.0),
        (1.0, 2.0, 0.0),
        (2.0, 4.0, 0.0),
        (3.0, 4.0, 1.0),
        (4.0, 2.0, 1.0),
        (5.0, 0.0, 1.0),
    ]
    t = 0.5
    # Binomial(5, i) = 1, 5, 10, 10, 5, 1
    coeffs = [1, 5, 10, 10, 5, 1]
    omt = 1.0 - t
    expected = (0.0, 0.0, 0.0)
    for i, (c, q) in enumerate(zip(coeffs, Q)):
        w = c * (omt ** (5 - i)) * (t ** i)
        expected = (
            expected[0] + w * q[0],
            expected[1] + w * q[1],
            expected[2] + w * q[2],
        )
    got = blendquintic._quintic_eval(Q, t)
    assert got == pytest.approx(expected, abs=1e-12)


def test_quintic_eval_random_t_matches_bernstein_direct():
    # Randomish but deterministic set of t values.
    Q = [
        (0.1, -0.2, 0.3),
        (1.4, 2.5, -0.6),
        (2.7, 4.8, 0.9),
        (3.0, 3.1, 1.2),
        (4.3, 1.4, 1.5),
        (5.6, -0.7, 1.8),
    ]
    coeffs = [1, 5, 10, 10, 5, 1]
    for t in (0.1, 0.25, 0.37, 0.6, 0.8, 0.95):
        omt = 1.0 - t
        expected = (0.0, 0.0, 0.0)
        for i, (c, q) in enumerate(zip(coeffs, Q)):
            w = c * (omt ** (5 - i)) * (t ** i)
            expected = (
                expected[0] + w * q[0],
                expected[1] + w * q[1],
                expected[2] + w * q[2],
            )
        got = blendquintic._quintic_eval(Q, t)
        assert got == pytest.approx(expected, abs=1e-12)


def test_quintic_first_derivative_at_endpoints_matches_tangent():
    # Symmetric blend control points. e1 = (1,0,0), e2 = (0,1,0).
    # V at origin, d = 2, r = 0.6. Coincident pairs enforce G2.
    V = (0.0, 0.0, 0.0)
    e1 = (1.0, 0.0, 0.0)
    e2 = (0.0, 1.0, 0.0)
    d = 2.0
    r = 0.6
    Q = [
        (V[0] - d * e1[0], V[1] - d * e1[1], V[2] - d * e1[2]),
        (V[0] - r * d * e1[0], V[1] - r * d * e1[1], V[2] - r * d * e1[2]),
        (V[0] - r * d * e1[0], V[1] - r * d * e1[1], V[2] - r * d * e1[2]),
        (V[0] + r * d * e2[0], V[1] + r * d * e2[1], V[2] + r * d * e2[2]),
        (V[0] + r * d * e2[0], V[1] + r * d * e2[1], V[2] + r * d * e2[2]),
        (V[0] + d * e2[0], V[1] + d * e2[1], V[2] + d * e2[2]),
    ]
    # B'(0) = 5 * (Q1 - Q0) = 5 * d * (1 - r) * e1
    d_at_0 = blendquintic._quintic_first_deriv(Q, 0.0)
    expected_0 = (5.0 * d * (1.0 - r), 0.0, 0.0)
    assert d_at_0 == pytest.approx(expected_0, abs=1e-12)

    # B'(1) = 5 * (Q5 - Q4) = 5 * d * (1 - r) * e2
    d_at_1 = blendquintic._quintic_first_deriv(Q, 1.0)
    expected_1 = (0.0, 5.0 * d * (1.0 - r), 0.0)
    assert d_at_1 == pytest.approx(expected_1, abs=1e-12)


def test_quintic_second_derivative_zero_at_endpoints():
    # G2 property: curvature kappa = |B' x B''| / |B'|^3 must be zero at
    # the endpoints for the symmetric quintic with coincident control-point
    # pairs (Q1=Q2 and Q3=Q4).  B'(0) and B''(0) are parallel (both along
    # e1), so their cross product is zero and kappa(0) = 0.  Likewise at t=1.
    # Note: B''(0) itself is NOT zero; it equals 20*(Q0 - Q1) which is
    # non-zero in general.  The plan description was imprecise; this test
    # checks the correct mathematical invariant.
    V = (0.0, 0.0, 0.0)
    e1 = (1.0, 0.0, 0.0)
    e2 = (0.0, 1.0, 0.0)
    d = 2.0
    r = 0.6
    Q = [
        (V[0] - d * e1[0], V[1] - d * e1[1], V[2] - d * e1[2]),
        (V[0] - r * d * e1[0], V[1] - r * d * e1[1], V[2] - r * d * e1[2]),
        (V[0] - r * d * e1[0], V[1] - r * d * e1[1], V[2] - r * d * e1[2]),
        (V[0] + r * d * e2[0], V[1] + r * d * e2[1], V[2] + r * d * e2[2]),
        (V[0] + r * d * e2[0], V[1] + r * d * e2[1], V[2] + r * d * e2[2]),
        (V[0] + d * e2[0], V[1] + d * e2[1], V[2] + d * e2[2]),
    ]
    d1_at_0 = blendquintic._quintic_first_deriv(Q, 0.0)
    dd_at_0 = blendquintic._quintic_second_deriv(Q, 0.0)
    d1_at_1 = blendquintic._quintic_first_deriv(Q, 1.0)
    dd_at_1 = blendquintic._quintic_second_deriv(Q, 1.0)

    def cross3(a, b):
        return (
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        )

    def norm3(v):
        return math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)

    # Cross product B'(t) x B''(t) must be zero (parallel vectors).
    cross_at_0 = cross3(d1_at_0, dd_at_0)
    cross_at_1 = cross3(d1_at_1, dd_at_1)
    assert norm3(cross_at_0) == pytest.approx(0.0, abs=1e-10)
    assert norm3(cross_at_1) == pytest.approx(0.0, abs=1e-10)


def test_quintic_derivatives_match_finite_difference():
    # Verify first derivative matches a centered finite difference.
    Q = [
        (0.1, -0.2, 0.3),
        (1.4, 2.5, -0.6),
        (2.7, 4.8, 0.9),
        (3.0, 3.1, 1.2),
        (4.3, 1.4, 1.5),
        (5.6, -0.7, 1.8),
    ]
    h = 1e-6
    for t in (0.2, 0.5, 0.75):
        p_plus = blendquintic._quintic_eval(Q, t + h)
        p_minus = blendquintic._quintic_eval(Q, t - h)
        fd = (
            (p_plus[0] - p_minus[0]) / (2.0 * h),
            (p_plus[1] - p_minus[1]) / (2.0 * h),
            (p_plus[2] - p_minus[2]) / (2.0 * h),
        )
        d1 = blendquintic._quintic_first_deriv(Q, t)
        assert d1 == pytest.approx(fd, abs=1e-6)


def _build_symmetric_Q(V, e1, e2, d, r):
    return [
        (V[0] - d * e1[0], V[1] - d * e1[1], V[2] - d * e1[2]),
        (V[0] - r * d * e1[0], V[1] - r * d * e1[1], V[2] - r * d * e1[2]),
        (V[0] - r * d * e1[0], V[1] - r * d * e1[1], V[2] - r * d * e1[2]),
        (V[0] + r * d * e2[0], V[1] + r * d * e2[1], V[2] + r * d * e2[2]),
        (V[0] + r * d * e2[0], V[1] + r * d * e2[1], V[2] + r * d * e2[2]),
        (V[0] + d * e2[0], V[1] + d * e2[1], V[2] + d * e2[2]),
    ]


def test_deviation_matches_closed_form_at_r_four_fifths():
    # Known sanity check: r = 0.8, coefficient (1 + 15*0.8)/16 = 13/16
    V = (0.0, 0.0, 0.0)
    e1 = (1.0, 0.0, 0.0)
    e2 = (0.0, 1.0, 0.0)  # theta = pi/2, sin(theta/2) = sin(pi/4) = sqrt(2)/2
    d = 1.0
    r = 0.8
    sin_half = math.sin(math.pi / 4.0)
    expected = (13.0 / 16.0) * d * sin_half

    Q = _build_symmetric_Q(V, e1, e2, d, r)
    B_mid = blendquintic._quintic_eval(Q, 0.5)
    got_numerical = math.sqrt(
        B_mid[0] ** 2 + B_mid[1] ** 2 + B_mid[2] ** 2
    )
    got_closed = blendquintic._deviation_closed_form(d, r, sin_half)
    assert got_numerical == pytest.approx(expected, abs=1e-12)
    assert got_closed == pytest.approx(expected, abs=1e-12)


def test_deviation_closed_form_matches_numerical_across_r_and_theta():
    # Sweep (theta, r) and verify closed-form matches Bezier evaluation.
    V = (0.0, 0.0, 0.0)
    for theta in (0.2, 0.5, 1.0, 1.5708, 2.3, 2.9):
        e1 = (1.0, 0.0, 0.0)
        e2 = (math.cos(theta), math.sin(theta), 0.0)
        sin_half = math.sin(theta / 2.0)
        for r in (0.3, 0.5, 0.7, 0.85):
            d = 1.5
            Q = _build_symmetric_Q(V, e1, e2, d, r)
            B_mid = blendquintic._quintic_eval(Q, 0.5)
            dev_numerical = math.sqrt(sum(c * c for c in B_mid))
            dev_closed = blendquintic._deviation_closed_form(d, r, sin_half)
            assert dev_numerical == pytest.approx(dev_closed, abs=1e-10)


def test_d_from_deviation_is_inverse_of_deviation():
    # Pick (r, theta, eps), compute d, rebuild Q, confirm deviation matches.
    V = (0.0, 0.0, 0.0)
    for theta in (0.4, 1.0, 2.0, 2.6):
        e1 = (1.0, 0.0, 0.0)
        e2 = (math.cos(theta), math.sin(theta), 0.0)
        sin_half = math.sin(theta / 2.0)
        for r in (0.50, 0.65, 0.80):
            for eps in (0.05, 0.2, 0.5):
                d = blendquintic._d_from_deviation(eps, r, sin_half)
                Q = _build_symmetric_Q(V, e1, e2, d, r)
                B_mid = blendquintic._quintic_eval(Q, 0.5)
                dev = math.sqrt(sum(c * c for c in B_mid))
                assert dev == pytest.approx(eps, abs=1e-10)


def test_curvature_zero_at_endpoints_of_symmetric_blend():
    # By construction: Q1 = Q2 and Q3 = Q4 force B''(0) = B''(1) = 0.
    V = (0.0, 0.0, 0.0)
    e1 = (1.0, 0.0, 0.0)
    e2 = (math.cos(1.2), math.sin(1.2), 0.0)
    Q = _build_symmetric_Q(V, e1, e2, d=1.5, r=0.6)
    assert blendquintic._curvature_at(Q, 0.0) == pytest.approx(0.0, abs=1e-9)
    assert blendquintic._curvature_at(Q, 1.0) == pytest.approx(0.0, abs=1e-9)


def test_curvature_matches_finite_difference_reference():
    # Reference: kappa(t) from centered finite differences of |B'|^3 and B'xB''.
    V = (0.0, 0.0, 0.0)
    e1 = (1.0, 0.0, 0.0)
    theta = math.pi / 2.0
    e2 = (math.cos(theta), math.sin(theta), 0.0)
    Q = _build_symmetric_Q(V, e1, e2, d=1.0, r=0.6)
    h = 1e-5
    for t in (0.25, 0.5, 0.75):
        # Finite-difference second derivative of position.
        p_plus = blendquintic._quintic_eval(Q, t + h)
        p_0 = blendquintic._quintic_eval(Q, t)
        p_minus = blendquintic._quintic_eval(Q, t - h)
        fd_first = (
            (p_plus[0] - p_minus[0]) / (2.0 * h),
            (p_plus[1] - p_minus[1]) / (2.0 * h),
            (p_plus[2] - p_minus[2]) / (2.0 * h),
        )
        fd_second = (
            (p_plus[0] - 2.0 * p_0[0] + p_minus[0]) / (h * h),
            (p_plus[1] - 2.0 * p_0[1] + p_minus[1]) / (h * h),
            (p_plus[2] - 2.0 * p_0[2] + p_minus[2]) / (h * h),
        )
        cross = (
            fd_first[1] * fd_second[2] - fd_first[2] * fd_second[1],
            fd_first[2] * fd_second[0] - fd_first[0] * fd_second[2],
            fd_first[0] * fd_second[1] - fd_first[1] * fd_second[0],
        )
        cross_norm = math.sqrt(sum(c * c for c in cross))
        first_norm = math.sqrt(sum(c * c for c in fd_first))
        fd_kappa = cross_norm / (first_norm ** 3)

        kappa = blendquintic._curvature_at(Q, t)
        assert kappa == pytest.approx(fd_kappa, rel=1e-3)
