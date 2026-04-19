# test/test_blendquintic.py
import math

import pytest

from klippy import blendquintic
from klippy.blendshaper import AxisShaperSnapshot


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


def test_peak_curvature_exceeds_midpoint_for_large_r():
    # For r > ~0.3 at non-shallow angles, the true peak is off-center.
    V = (0.0, 0.0, 0.0)
    e1 = (1.0, 0.0, 0.0)
    theta = math.pi / 2.0
    e2 = (math.cos(theta), math.sin(theta), 0.0)
    Q = _build_symmetric_Q(V, e1, e2, d=1.0, r=0.8)
    kappa_peak, t_peak = blendquintic._peak_curvature(Q)
    kappa_mid = blendquintic._curvature_at(Q, 0.5)
    assert kappa_peak > kappa_mid * 1.5
    # Peak should be off-center (not at t=0.5 for this r).
    assert abs(t_peak - 0.5) > 1e-3


def test_peak_curvature_at_midpoint_for_small_r():
    # For r <= ~0.3 the peak is at the midpoint by symmetry.
    V = (0.0, 0.0, 0.0)
    e1 = (1.0, 0.0, 0.0)
    theta = math.pi / 2.0
    e2 = (math.cos(theta), math.sin(theta), 0.0)
    Q = _build_symmetric_Q(V, e1, e2, d=1.0, r=0.2)
    kappa_peak, t_peak = blendquintic._peak_curvature(Q)
    kappa_mid = blendquintic._curvature_at(Q, 0.5)
    assert kappa_peak == pytest.approx(kappa_mid, rel=1e-2)
    assert t_peak == pytest.approx(0.5, abs=0.05)


def test_peak_curvature_matches_dense_reference():
    # The implementation samples _PEAK_KAPPA_SAMPLES points; reference
    # samples 2001. Both should agree within 1%.
    V = (0.0, 0.0, 0.0)
    e1 = (1.0, 0.0, 0.0)
    for theta, r in [(0.5, 0.50), (1.0, 0.55), (math.pi / 2, 0.6), (2.5, 0.85)]:
        e2 = (math.cos(theta), math.sin(theta), 0.0)
        Q = _build_symmetric_Q(V, e1, e2, d=1.0, r=r)
        kappa_peak, _ = blendquintic._peak_curvature(Q)
        reference_samples = 2001
        ref_max = 0.0
        for i in range(reference_samples):
            t = i / (reference_samples - 1)
            k = blendquintic._curvature_at(Q, t)
            if k > ref_max:
                ref_max = k
        assert kappa_peak == pytest.approx(ref_max, rel=1e-2)


def test_shape_ratio_matches_reference_anchors():
    # Anchor points from the subagent's per-angle optimum table
    # (interior-angle convention converted to deflection).
    # interior 90 deg -> deflection pi/2 -> r = 0.5900
    # interior 60 deg -> deflection 2*pi/3 -> r = 0.6800
    # interior 150 deg -> deflection pi/6 -> r = 0.5044
    r_shallow = blendquintic._shape_ratio(math.radians(30))
    r_mid = blendquintic._shape_ratio(math.radians(90))
    r_wide = blendquintic._shape_ratio(math.radians(120))
    assert r_shallow == pytest.approx(0.5044, abs=0.01)
    assert r_mid == pytest.approx(0.5900, abs=0.01)
    assert r_wide == pytest.approx(0.6800, abs=0.01)


def test_shape_ratio_clamps_to_valid_range():
    # Below clamp floor: 0 rad would give 0.5085, clamped to 0.50.
    # But the formula floor at theta=0 is already >= 0.50, so test
    # extreme small theta doesn't go below 0.50 due to numerical noise.
    r0 = blendquintic._shape_ratio(0.0)
    assert 0.50 <= r0 <= 0.86

    # Far beyond the validity window (theta = pi): r formula -> 0.9539
    # clamped to 0.86.
    r_big = blendquintic._shape_ratio(math.pi)
    assert r_big == 0.86


def test_shape_ratio_monotone_increasing_in_theta():
    # r(theta) should be strictly increasing past the formula minimum.
    # The quadratic has a minimum at ~19 deg; below that r is near the
    # 0.50 clamp floor.  The monotone guarantee holds from ~20 deg onward
    # (the spec validity window starts at 10 deg but the quadratic's vertex
    # is at ~19 deg; values at 15 vs 20 deg differ by only 0.0002 — both
    # effectively at the clamp floor).  We start the check at 20 deg to
    # avoid the non-monotone region near the vertex.
    prev = blendquintic._shape_ratio(math.radians(20))
    for deg in range(25, 165, 5):
        r = blendquintic._shape_ratio(math.radians(deg))
        assert r >= prev
        prev = r


def test_quintic_blend_dataclass_fields():
    q = blendquintic.QuinticBlend(
        Q=(
            (-1.0, 0.0, 0.0),
            (-0.5, 0.0, 0.0),
            (-0.5, 0.0, 0.0),
            (0.0, 0.5, 0.0),
            (0.0, 0.5, 0.0),
            (0.0, 1.0, 0.0),
        ),
        theta=math.pi / 2.0,
        r=0.5900,
        d_consumed=1.0,
        kappa_peak=0.5,
        t_peak=0.2,
        v_cap=100.0,
        entry_tangent=(1.0, 0.0, 0.0),
        exit_tangent=(0.0, 1.0, 0.0),
        plane_normal=(0.0, 0.0, 1.0),
    )
    assert len(q.Q) == 6
    assert q.theta == pytest.approx(math.pi / 2.0)
    assert q.r == pytest.approx(0.5900)
    assert q.d_consumed == 1.0
    assert q.kappa_peak == 0.5
    assert q.t_peak == pytest.approx(0.2)
    assert q.v_cap == 100.0
    assert q.plane_normal == (0.0, 0.0, 1.0)


def test_quintic_geometry_collinear_returns_none():
    prev_dir = (1.0, 0.0, 0.0)
    next_dir = (1.0, 0.0, 0.0)
    result = blendquintic.quintic_geometry(
        prev_dir=prev_dir,
        next_dir=next_dir,
        L_prev=10.0,
        L_next=10.0,
        corner_deviation=0.02,
        a_max=50000.0,
    )
    assert result is None


def test_quintic_geometry_u_turn_returns_degenerate():
    prev_dir = (1.0, 0.0, 0.0)
    next_dir = (-1.0, 0.0, 0.0)
    result = blendquintic.quintic_geometry(
        prev_dir=prev_dir,
        next_dir=next_dir,
        L_prev=10.0,
        L_next=10.0,
        corner_deviation=0.02,
        a_max=50000.0,
    )
    assert result is not None
    assert result.v_cap == 0.0
    assert result.d_consumed == 0.0


def test_quintic_geometry_right_angle_basic():
    # 90 deg corner, 1 mm segments, eps = 0.1 mm. Expect a QuinticBlend
    # with theta = pi/2, r matches _shape_ratio(pi/2), d > 0, kappa_peak > 0.
    prev_dir = (1.0, 0.0, 0.0)
    next_dir = (0.0, 1.0, 0.0)
    result = blendquintic.quintic_geometry(
        prev_dir=prev_dir,
        next_dir=next_dir,
        L_prev=1.0,
        L_next=1.0,
        corner_deviation=0.1,
        a_max=50000.0,
    )
    assert result is not None
    assert result.theta == pytest.approx(math.pi / 2.0, abs=1e-9)
    assert result.r == pytest.approx(blendquintic._shape_ratio(math.pi / 2.0))
    assert result.d_consumed > 0.0
    assert result.kappa_peak > 0.0
    # Centripetal-only cap: v_cap^2 <= a_max / kappa_peak
    assert result.v_cap * result.v_cap == pytest.approx(
        50000.0 / result.kappa_peak, rel=1e-9
    )
    assert result.plane_normal == pytest.approx((0.0, 0.0, 1.0), abs=1e-12)


def test_quintic_geometry_half_segment_cap_limits_d():
    # Very loose corner_deviation should hit the L/2 cap.
    prev_dir = (1.0, 0.0, 0.0)
    next_dir = (0.0, 1.0, 0.0)
    result = blendquintic.quintic_geometry(
        prev_dir=prev_dir,
        next_dir=next_dir,
        L_prev=2.0,
        L_next=4.0,
        corner_deviation=5.0,  # would demand d much bigger than L_prev/2
        a_max=50000.0,
    )
    assert result is not None
    assert result.d_consumed == pytest.approx(1.0, abs=1e-12)  # 0.5 * min(2, 4)


def test_quintic_geometry_with_shaper_bound_matches_centripetal_when_no_shapers():
    prev_dir = (1.0, 0.0, 0.0)
    next_dir = (0.0, 1.0, 0.0)
    base = blendquintic.quintic_geometry(
        prev_dir=prev_dir,
        next_dir=next_dir,
        L_prev=1.0,
        L_next=1.0,
        corner_deviation=0.1,
        a_max=50000.0,
    )
    capped = blendquintic.quintic_geometry_with_shaper(
        base=base,
        shapers=[],
        j_eff=float("inf"),
    )
    assert capped.v_cap == pytest.approx(base.v_cap)


def test_quintic_geometry_with_shaper_bound_tightens_v_cap():
    prev_dir = (1.0, 0.0, 0.0)
    next_dir = (0.0, 1.0, 0.0)
    base = blendquintic.quintic_geometry(
        prev_dir=prev_dir,
        next_dir=next_dir,
        L_prev=1.0,
        L_next=1.0,
        corner_deviation=0.1,
        a_max=50000.0,
    )
    shapers = [
        AxisShaperSnapshot(
            axis="x",
            shaper_type="mzv",
            shaper_freq=60.0,
            damping_ratio=0.1,
            A_axis=10000.0,
        ),
        AxisShaperSnapshot(
            axis="y",
            shaper_type="mzv",
            shaper_freq=60.0,
            damping_ratio=0.1,
            A_axis=10000.0,
        ),
    ]
    capped = blendquintic.quintic_geometry_with_shaper(
        base=base,
        shapers=shapers,
        j_eff=float("inf"),
    )
    assert capped.v_cap <= base.v_cap + 1e-9


def test_quintic_shaper_bound_is_min_across_three_samples():
    # With axis-rotated bisector, single-point evaluation at t=0.5 can
    # overshoot true min; three-point min should be tighter than mid-only.
    prev_dir = (1.0, 0.0, 0.0)
    # 30 deg interior -> deflection 150 deg; rotated 45 deg about z axis
    theta_defl = math.radians(150)
    angle_rot = math.radians(45)
    cos_r = math.cos(angle_rot)
    sin_r = math.sin(angle_rot)
    # Rotate both prev and next so the bisector is NOT aligned with an axis.
    prev_dir = (cos_r, sin_r, 0.0)
    # next_dir = R_rot(Rot_defl(prev_dir))
    tx = math.cos(theta_defl) * cos_r - math.sin(theta_defl) * sin_r
    ty = math.sin(theta_defl) * cos_r + math.cos(theta_defl) * sin_r
    next_dir = (tx, ty, 0.0)
    base = blendquintic.quintic_geometry(
        prev_dir=prev_dir,
        next_dir=next_dir,
        L_prev=1.0,
        L_next=1.0,
        corner_deviation=0.2,
        a_max=45000.0,
    )
    shapers = [
        AxisShaperSnapshot(
            axis="x", shaper_type="mzv", shaper_freq=60.0,
            damping_ratio=0.1, A_axis=10000.0,
        ),
        AxisShaperSnapshot(
            axis="y", shaper_type="mzv", shaper_freq=60.0,
            damping_ratio=0.1, A_axis=10000.0,
        ),
    ]
    three_point = blendquintic.quintic_geometry_with_shaper(
        base=base, shapers=shapers, j_eff=float("inf"),
    )
    # Spot-check dense min over 100+ points along the blend.
    dense = blendquintic._dense_shaper_cap(base, shapers, samples=101)
    # Three-point min should not exceed the dense-sampled min by more
    # than a small margin (the spec target is ~6% worst-case overshoot).
    assert three_point.v_cap <= dense * 1.10 + 1e-9

    # Core claim of this test: three-point must be meaningfully tighter
    # than evaluating the shaper bound only at the midpoint. At this
    # axis-rotated shallow-deflection geometry the midpoint normal is
    # near the bisector but the binding t shifts off-center; a midpoint-
    # only evaluator therefore lets the corner run too fast.
    R_mid = 1.0 / blendquintic._curvature_at(base.Q, 0.5)
    _, _, n_mid = blendquintic._point_frame(base.Q, 0.5)
    mid_only_bounds = blendquintic.blendshaper.compute_shaper_bounds(
        shapers=shapers, R=R_mid, n_hat=n_mid, p_hat=base.plane_normal,
    )
    assert three_point.v_cap < mid_only_bounds.v_step_cap * 0.95


def test_rotation_jerk_cap_applied():
    prev_dir = (1.0, 0.0, 0.0)
    next_dir = (0.0, 1.0, 0.0)
    base = blendquintic.quintic_geometry(
        prev_dir=prev_dir,
        next_dir=next_dir,
        L_prev=1.0,
        L_next=1.0,
        corner_deviation=0.1,
        a_max=50000.0,
    )
    # Very small j_eff: rotation-jerk should dominate v_cap.
    capped = blendquintic.quintic_geometry_with_shaper(
        base=base,
        shapers=[],
        j_eff=1e4,
    )
    R_peak = 1.0 / base.kappa_peak
    v_jerk_expected = (R_peak * math.sqrt(1e4)) ** (2.0 / 3.0)
    # v_cap = min(v_cent, v_jerk). v_jerk is smaller at j_eff=1e4.
    assert capped.v_cap == pytest.approx(min(base.v_cap, v_jerk_expected), rel=1e-9)


def test_rotation_jerk_infinite_does_not_affect_v_cap():
    prev_dir = (1.0, 0.0, 0.0)
    next_dir = (0.0, 1.0, 0.0)
    base = blendquintic.quintic_geometry(
        prev_dir=prev_dir, next_dir=next_dir,
        L_prev=1.0, L_next=1.0,
        corner_deviation=0.1, a_max=50000.0,
    )
    capped = blendquintic.quintic_geometry_with_shaper(
        base=base,
        shapers=[],
        j_eff=float("inf"),
    )
    assert capped.v_cap == pytest.approx(base.v_cap)


class _FakeMove:
    """Minimal Move stub for pure-math tests."""

    def __init__(self, axes_r, move_d, accel, is_kinematic=True):
        self.axes_r = axes_r
        self.move_d = move_d
        self.accel = accel
        self.is_kinematic_move = is_kinematic


def test_blend_from_moves_quintic_skips_non_kinematic():
    prev = _FakeMove((1.0, 0.0, 0.0, 0.0), 10.0, 50000.0, is_kinematic=False)
    nxt = _FakeMove((0.0, 1.0, 0.0, 0.0), 10.0, 50000.0, is_kinematic=True)
    result = blendquintic.blend_from_moves_quintic(prev, nxt, 0.1)
    assert result is None


def test_blend_from_moves_quintic_returns_blend_for_right_angle():
    prev = _FakeMove((1.0, 0.0, 0.0), 1.0, 50000.0)
    nxt = _FakeMove((0.0, 1.0, 0.0), 1.0, 50000.0)
    result = blendquintic.blend_from_moves_quintic(prev, nxt, 0.1)
    assert result is not None
    assert result.theta == pytest.approx(math.pi / 2.0, abs=1e-9)
    assert result.d_consumed > 0.0


def test_blend_from_moves_quintic_uses_stricter_accel():
    prev = _FakeMove((1.0, 0.0, 0.0), 1.0, 30000.0)
    nxt = _FakeMove((0.0, 1.0, 0.0), 1.0, 70000.0)
    result = blendquintic.blend_from_moves_quintic(prev, nxt, 0.1)
    assert result is not None
    # v_cent^2 = a_max / kappa_peak with a_max = min(30000, 70000) = 30000
    assert result.v_cap * result.v_cap == pytest.approx(
        30000.0 / result.kappa_peak, rel=1e-9
    )


def test_blend_from_moves_quintic_rejects_both_j_eff_and_toolhead():
    prev = _FakeMove((1.0, 0.0, 0.0), 1.0, 50000.0)
    nxt = _FakeMove((0.0, 1.0, 0.0), 1.0, 50000.0)

    class _DummyToolhead:
        pass

    with pytest.raises(ValueError):
        blendquintic.blend_from_moves_quintic(
            prev, nxt, 0.1, j_eff=1e7, toolhead=_DummyToolhead(),
        )


def test_segment_quintic_max_chord_error_bound():
    prev_dir = (1.0, 0.0, 0.0)
    next_dir = (0.0, 1.0, 0.0)
    q = blendquintic.quintic_geometry(
        prev_dir=prev_dir, next_dir=next_dir,
        L_prev=1.0, L_next=1.0,
        corner_deviation=0.2, a_max=50000.0,
    )
    tol = 0.005
    poly = blendquintic.segment_quintic(q, max_chord_err=tol)
    # Every consecutive pair is a segment: no point on the curve
    # should be farther from its chord than tol.
    # Sample many reference points along the curve and for each, find
    # the closest chord; check distance <= tol + slack.
    ref_samples = 201
    ref_pts = [blendquintic._quintic_eval(q.Q, i / (ref_samples - 1))
               for i in range(ref_samples)]
    # For each reference point, find the min distance to any chord of
    # the polyline. This is quadratic and fine for a unit test.

    def _point_chord_dist(p, a, b):
        ab = (b[0] - a[0], b[1] - a[1], b[2] - a[2])
        ap = (p[0] - a[0], p[1] - a[1], p[2] - a[2])
        len2 = ab[0] * ab[0] + ab[1] * ab[1] + ab[2] * ab[2]
        if len2 == 0:
            dx = p[0] - a[0]; dy = p[1] - a[1]; dz = p[2] - a[2]
            return math.sqrt(dx * dx + dy * dy + dz * dz)
        tt = (ap[0] * ab[0] + ap[1] * ab[1] + ap[2] * ab[2]) / len2
        tt = max(0.0, min(1.0, tt))
        proj = (a[0] + ab[0] * tt, a[1] + ab[1] * tt, a[2] + ab[2] * tt)
        dx = p[0] - proj[0]; dy = p[1] - proj[1]; dz = p[2] - proj[2]
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    for ref in ref_pts:
        best = min(
            _point_chord_dist(ref, poly[i], poly[i + 1])
            for i in range(len(poly) - 1)
        )
        assert best <= tol * 1.5  # slack for sampling density


def test_segment_quintic_emits_ordered_polyline():
    prev_dir = (1.0, 0.0, 0.0)
    next_dir = (0.0, 1.0, 0.0)
    q = blendquintic.quintic_geometry(
        prev_dir=prev_dir, next_dir=next_dir,
        L_prev=1.0, L_next=1.0,
        corner_deviation=0.2, a_max=50000.0,
    )
    poly = blendquintic.segment_quintic(q, max_chord_err=0.01)
    assert len(poly) >= 3
    assert poly[0] == pytest.approx(q.Q[0])
    assert poly[-1] == pytest.approx(q.Q[5])


def test_segment_quintic_degenerate_returns_single_point():
    prev_dir = (1.0, 0.0, 0.0)
    next_dir = (-1.0, 0.0, 0.0)
    q = blendquintic.quintic_geometry(
        prev_dir=prev_dir, next_dir=next_dir,
        L_prev=1.0, L_next=1.0,
        corner_deviation=0.2, a_max=50000.0,
    )
    poly = blendquintic.segment_quintic(q, max_chord_err=0.01)
    assert poly == [(0.0, 0.0, 0.0)]


def test_interpolate_extruder_conserves_total_e():
    prev_dir = (1.0, 0.0, 0.0)
    next_dir = (0.0, 1.0, 0.0)
    q = blendquintic.quintic_geometry(
        prev_dir=prev_dir, next_dir=next_dir,
        L_prev=1.0, L_next=1.0,
        corner_deviation=0.2, a_max=50000.0,
    )
    poly = blendquintic.segment_quintic(q, max_chord_err=0.005)
    e_per_mm_prev = 0.12
    e_per_mm_next = 0.10
    extruded = blendquintic.interpolate_extruder_quintic(
        poly, q.d_consumed, e_per_mm_prev, e_per_mm_next,
    )
    total_e = extruded[-1][3] - extruded[0][3]
    expected_total = q.d_consumed * (e_per_mm_prev + e_per_mm_next)
    assert total_e == pytest.approx(expected_total, rel=1e-6)


def test_interpolate_extruder_monotone_increasing():
    prev_dir = (1.0, 0.0, 0.0)
    next_dir = (0.0, 1.0, 0.0)
    q = blendquintic.quintic_geometry(
        prev_dir=prev_dir, next_dir=next_dir,
        L_prev=1.0, L_next=1.0,
        corner_deviation=0.2, a_max=50000.0,
    )
    poly = blendquintic.segment_quintic(q, max_chord_err=0.005)
    extruded = blendquintic.interpolate_extruder_quintic(
        poly, q.d_consumed, 0.12, 0.10,
    )
    for i in range(len(extruded) - 1):
        assert extruded[i + 1][3] >= extruded[i][3]


def test_interpolate_extruder_degenerate_polyline():
    # Single-point polyline -> single output point with E = 0.
    poly = [(0.0, 0.0, 0.0)]
    out = blendquintic.interpolate_extruder_quintic(poly, 0.0, 0.12, 0.10)
    assert out == [(0.0, 0.0, 0.0, 0.0)]


def test_random_corners_property_sweep():
    rng = __import__("random").Random(20260419)
    for _ in range(50):
        theta = rng.uniform(math.radians(15), math.radians(160))
        # Random rotation in the XY plane.
        phi = rng.uniform(0.0, 2.0 * math.pi)
        prev_dir = (math.cos(phi), math.sin(phi), 0.0)
        next_dir = (
            math.cos(phi + theta),
            math.sin(phi + theta),
            0.0,
        )
        L_prev = rng.uniform(0.5, 5.0)
        L_next = rng.uniform(0.5, 5.0)
        corner_deviation = rng.uniform(0.02, 0.4)
        a_max = rng.uniform(20000.0, 100000.0)
        q = blendquintic.quintic_geometry(
            prev_dir=prev_dir,
            next_dir=next_dir,
            L_prev=L_prev,
            L_next=L_next,
            corner_deviation=corner_deviation,
            a_max=a_max,
        )
        assert q is not None
        assert 0.50 <= q.r <= 0.86
        # Deviation check: either the tolerance was binding (deviation
        # matches corner_deviation) or the half-segment cap was binding
        # (deviation below corner_deviation, d_consumed == 0.5*min(L)).
        sin_half = math.sin(q.theta / 2.0)
        achieved_dev = blendquintic._deviation_closed_form(
            q.d_consumed, q.r, sin_half,
        )
        assert achieved_dev <= corner_deviation + 1e-9
        # Velocity cap: v^2 * kappa_peak <= a_max
        assert q.v_cap * q.v_cap * q.kappa_peak <= a_max * (1.0 + 1e-6)
        # All control points finite.
        for pt in q.Q:
            for c in pt:
                assert math.isfinite(c)
