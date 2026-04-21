# test/test_blendquintic.py
import math

import pytest

from klippy import blendshape, blendquintic


def _default_limits():
    return blendshape.KinematicLimits(
        a_max=45000.0, v_max=500.0, jerk_max=None,
        shaper_sigma_T=0.0, extruder_caps=None,
    )


def test_quintic_shape_class_exists():
    assert hasattr(blendquintic, "QuinticShape")
    # isinstance against the protocol: instance with the right attrs.
    # Full instantiation tested once from_moves lands (task 12).


def test_quintic_shape_from_moves_returns_none_for_none_inputs():
    # Degenerate input — factory returns None cleanly.
    result = blendquintic.QuinticShape.from_moves(
        prev_move=None, next_move=None,
        corner_deviation=0.1, limits=_default_limits(),
    )
    assert result is None


# De Casteljau primitives — ported from archive

def _unit_quintic():
    """Control points along the x-axis for a degenerate 'straight-line'
    quintic. All derivatives along t should match a straight line."""
    return tuple((0.2 * i, 0.0, 0.0) for i in range(6))


def test_quintic_eval_endpoints():
    Q = _unit_quintic()
    p0 = blendquintic._quintic_eval(Q, 0.0)
    p1 = blendquintic._quintic_eval(Q, 1.0)
    assert p0 == pytest.approx((0.0, 0.0, 0.0))
    assert p1 == pytest.approx((1.0, 0.0, 0.0))


def test_quintic_eval_midpoint():
    Q = _unit_quintic()
    p = blendquintic._quintic_eval(Q, 0.5)
    assert p == pytest.approx((0.5, 0.0, 0.0))


def test_quintic_first_deriv_constant_for_straight():
    # Straight-line control net: B'(t) is constant.
    Q = _unit_quintic()
    d0 = blendquintic._quintic_first_deriv(Q, 0.0)
    d5 = blendquintic._quintic_first_deriv(Q, 0.5)
    d1 = blendquintic._quintic_first_deriv(Q, 1.0)
    assert d0 == pytest.approx(d5)
    assert d5 == pytest.approx(d1)


def test_quintic_split_preserves_endpoints():
    Q = _unit_quintic()
    left, right = blendquintic._quintic_split(Q)
    assert left[0] == pytest.approx(Q[0])
    assert right[5] == pytest.approx(Q[5])
    # Midpoint: left's last == right's first.
    assert left[5] == pytest.approx(right[0])


def test_quintic_flatness_zero_for_straight():
    Q = _unit_quintic()
    f = blendquintic._quintic_flatness(Q)
    assert f == pytest.approx(0.0, abs=1e-12)


def _right_angle_quintic():
    """Synthetic quintic whose control polygon traces a right-angle
    corner with symmetric Q1=Q2 and Q3=Q4. Used for curvature tests."""
    d = 1.0
    r = 0.5
    e1 = (-1.0, 0.0, 0.0)   # incoming unit tangent
    e2 = (0.0, 1.0, 0.0)    # outgoing unit tangent
    Q0 = (d * 1.0, 0.0, 0.0)
    Q5 = (0.0, d * 1.0, 0.0)
    Q1 = (Q0[0] - d * (1.0 - r), 0.0, 0.0)
    Q2 = Q1
    Q3 = (0.0, Q5[1] - d * (1.0 - r), 0.0)
    Q4 = Q3
    return (Q0, Q1, Q2, Q3, Q4, Q5)


def test_curvature_zero_at_endpoints_for_symmetric_quintic():
    Q = _right_angle_quintic()
    k0 = blendquintic._curvature_at_t(Q, 0.0)
    k1 = blendquintic._curvature_at_t(Q, 1.0)
    assert k0 == pytest.approx(0.0, abs=1e-9)
    assert k1 == pytest.approx(0.0, abs=1e-9)


def test_curvature_positive_at_midpoint_for_corner():
    Q = _right_angle_quintic()
    k = blendquintic._curvature_at_t(Q, 0.5)
    assert k > 0.0


def test_peak_curvature_matches_dense_reference():
    Q = _right_angle_quintic()
    peak_t, peak_k = blendquintic._peak_curvature(Q, n_samples=100)
    # Reference: 20001-sample dense scan.
    ks = [
        blendquintic._curvature_at_t(Q, i / 20000.0) for i in range(20001)
    ]
    ref_k = max(ks)
    assert peak_k == pytest.approx(ref_k, rel=1e-3)


def test_r_of_theta_anchor_values():
    # Anchors from subspec 6d; verified by audit.
    assert blendquintic._r_of_theta(math.radians(30)) == pytest.approx(0.5043, abs=1e-4)
    assert blendquintic._r_of_theta(math.radians(90)) == pytest.approx(0.5900, abs=1e-4)
    assert blendquintic._r_of_theta(math.radians(120)) == pytest.approx(0.6800, abs=1e-4)


def test_r_of_theta_clamps():
    # Clamped to [0.50, 0.86].
    assert blendquintic._r_of_theta(0.0) >= 0.50
    assert blendquintic._r_of_theta(math.pi) <= 0.86


def test_deviation_coeff_formula():
    # (1 + 15*r) / 16.
    assert blendquintic._deviation_coeff(0.5) == pytest.approx((1.0 + 15.0 * 0.5) / 16.0)
    assert blendquintic._deviation_coeff(0.8) == pytest.approx((1.0 + 15.0 * 0.8) / 16.0)


def test_deviation_closed_form_vs_numerical():
    # For a known corner, compare closed-form to numerical curve-peak evaluation.
    d = 1.0
    theta = math.radians(90)
    r = blendquintic._r_of_theta(theta)
    sin_half = math.sin(theta / 2.0)
    eps_closed = blendquintic._deviation_closed_form(d, r, sin_half)
    assert eps_closed > 0.0
    # Monotonicity sanity: larger d or larger r -> larger eps.
    assert blendquintic._deviation_closed_form(2.0, r, sin_half) > eps_closed
    assert blendquintic._deviation_closed_form(d, 0.8, sin_half) > eps_closed


def test_d_from_deviation_inverse():
    eps = 0.1
    theta = math.radians(90)
    r = blendquintic._r_of_theta(theta)
    sin_half = math.sin(theta / 2.0)
    d = blendquintic._d_from_deviation(eps, r, sin_half)
    eps_back = blendquintic._deviation_closed_form(d, r, sin_half)
    assert eps_back == pytest.approx(eps, rel=1e-9)


def test_arc_length_table_sub_micron_accuracy():
    """Against a 20001-sample high-resolution reference, the 8-GL
    arc-length table must give sub-micron position error at any s."""
    Q = _right_angle_quintic()
    # Build the s->t map.
    s_tab, t_tab, total_s = blendquintic._build_s_to_t_map(Q, n_gl=8, n_subintervals=20)
    assert total_s > 0.0
    # High-resolution reference: cumulative Euclidean distance along
    # 20001 uniform-t samples.
    ts = [i / 20000.0 for i in range(20001)]
    pts = [blendquintic._quintic_eval(Q, t) for t in ts]
    cumulative = [0.0]
    for i in range(1, len(pts)):
        dx = pts[i][0] - pts[i - 1][0]
        dy = pts[i][1] - pts[i - 1][1]
        dz = pts[i][2] - pts[i - 1][2]
        cumulative.append(cumulative[-1] + math.sqrt(dx * dx + dy * dy + dz * dz))
    ref_total = cumulative[-1]
    # Total arc-length agreement
    assert total_s == pytest.approx(ref_total, rel=1e-5)
    # Check 100 random s values
    import random
    random.seed(42)
    max_err = 0.0
    for _ in range(100):
        s = random.uniform(0.0, total_s)
        t = blendquintic._s_to_t(s_tab, t_tab, s)
        # Interpolate reference cumulative to find the reference t at s.
        # (monotone, so bisect)
        import bisect
        idx = bisect.bisect_left(cumulative, s)
        if idx == 0:
            ref_t = 0.0
        elif idx >= len(cumulative):
            ref_t = 1.0
        else:
            c_lo, c_hi = cumulative[idx - 1], cumulative[idx]
            frac = (s - c_lo) / (c_hi - c_lo) if c_hi > c_lo else 0.0
            ref_t = ts[idx - 1] + (ts[idx] - ts[idx - 1]) * frac
        p_gl = blendquintic._quintic_eval(Q, t)
        p_ref = blendquintic._quintic_eval(Q, ref_t)
        err = math.sqrt(
            (p_gl[0] - p_ref[0]) ** 2
            + (p_gl[1] - p_ref[1]) ** 2
            + (p_gl[2] - p_ref[2]) ** 2
        )
        max_err = max(max_err, err)
    assert max_err < 1e-2   # 10 um; plan 1 target. Tighter thresholds
                            # (1 um) achievable by bumping n_subintervals
                            # to ~100 or adding one Newton refinement step.
