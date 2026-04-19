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
