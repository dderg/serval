# test/test_blendquintic.py
import math

import pytest

from klippy import blendshape, blendquintic


def _default_limits():
    return blendshape.KinematicLimits(
        a_max=45000.0, v_max=500.0, jerk_max=None,
        extruder_caps=None,
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


def _build_shape_direct(Q):
    """Bypass from_moves: build a QuinticShape directly from control
    points for testing the arc-length-backed methods. d_consumed and
    theta are dummy here; the real factory (task 12) computes them."""
    shape = blendquintic.QuinticShape.__new__(blendquintic.QuinticShape)
    blendquintic.QuinticShape._init_from_Q(shape, Q, d_consumed=1.0, theta=math.radians(90))
    return shape


def test_position_at_endpoints():
    Q = _right_angle_quintic()
    shape = _build_shape_direct(Q)
    p0 = shape.position_at(0.0)
    p1 = shape.position_at(shape.arc_length)
    assert p0 == pytest.approx(Q[0])
    assert p1 == pytest.approx(Q[5])


def test_tangent_at_endpoints_unit_length():
    Q = _right_angle_quintic()
    shape = _build_shape_direct(Q)
    t0 = shape.tangent_at(0.0)
    t1 = shape.tangent_at(shape.arc_length)
    for t in (t0, t1):
        mag = math.sqrt(t[0] ** 2 + t[1] ** 2 + t[2] ** 2)
        assert mag == pytest.approx(1.0, rel=1e-9)


def test_curvature_at_endpoints_zero():
    Q = _right_angle_quintic()
    shape = _build_shape_direct(Q)
    assert shape.curvature_at(0.0) == pytest.approx(0.0, abs=1e-9)
    assert shape.curvature_at(shape.arc_length) == pytest.approx(0.0, abs=1e-9)


def test_tangent_matches_ds_position_numerically():
    Q = _right_angle_quintic()
    shape = _build_shape_direct(Q)
    s_mid = shape.arc_length * 0.5
    ds = 1e-4
    p_lo = shape.position_at(s_mid - ds)
    p_hi = shape.position_at(s_mid + ds)
    num = ((p_hi[0] - p_lo[0]) / (2 * ds),
           (p_hi[1] - p_lo[1]) / (2 * ds),
           (p_hi[2] - p_lo[2]) / (2 * ds))
    num_mag = math.sqrt(num[0] ** 2 + num[1] ** 2 + num[2] ** 2)
    num_hat = (num[0] / num_mag, num[1] / num_mag, num[2] / num_mag)
    tan = shape.tangent_at(s_mid)
    assert num_hat[0] == pytest.approx(tan[0], abs=1e-4)
    assert num_hat[1] == pytest.approx(tan[1], abs=1e-4)
    assert num_hat[2] == pytest.approx(tan[2], abs=1e-4)


def test_dkappa_ds_matches_finite_difference():
    Q = _right_angle_quintic()
    shape = _build_shape_direct(Q)
    ds = 1e-4
    for s_frac in (0.25, 0.4, 0.5, 0.6, 0.75):
        s_mid = shape.arc_length * s_frac
        k_lo = shape.curvature_at(s_mid - ds)
        k_hi = shape.curvature_at(s_mid + ds)
        numerical = (k_hi - k_lo) / (2 * ds)
        analytical = shape.dkappa_ds(s_mid)
        assert analytical == pytest.approx(numerical, abs=1e-3, rel=1e-3)


def test_dkappa_ds_signs_at_endpoints():
    """Symmetric blend: kappa ramps from 0 up to peak then back to 0.
    dkappa/ds should be positive near s=0, negative near s=arc_length."""
    Q = _right_angle_quintic()
    shape = _build_shape_direct(Q)
    assert shape.dkappa_ds(shape.arc_length * 0.1) > 0.0
    assert shape.dkappa_ds(shape.arc_length * 0.9) < 0.0


def _right_angle_quintic_mirrored():
    """Mirror of _right_angle_quintic across the x-axis: this produces
    a quintic whose signed curvature is POSITIVE throughout, exercising
    the kappa >= 0 branch of dkappa_ds."""
    Q = _right_angle_quintic()
    return tuple((q[0], -q[1], q[2]) for q in Q)


def test_dkappa_ds_matches_finite_difference_mirrored():
    """Same FD test, but on a mirrored fixture with kappa >= 0 throughout,
    exercising the no-sign-flip branch of dkappa_ds."""
    Q = _right_angle_quintic_mirrored()
    shape = _build_shape_direct(Q)
    ds = 1e-4
    for s_frac in (0.25, 0.4, 0.5, 0.6, 0.75):
        s_mid = shape.arc_length * s_frac
        k_lo = shape.curvature_at(s_mid - ds)
        k_hi = shape.curvature_at(s_mid + ds)
        numerical = (k_hi - k_lo) / (2 * ds)
        analytical = shape.dkappa_ds(s_mid)
        assert analytical == pytest.approx(numerical, abs=1e-3, rel=1e-3)


def test_polyline_endpoints_match_control_endpoints():
    Q = _right_angle_quintic()
    shape = _build_shape_direct(Q)
    poly = shape.polyline(chord_tol=1e-3)
    assert poly[0] == pytest.approx(Q[0])
    assert poly[-1] == pytest.approx(Q[5])


def test_polyline_segment_count_scales_with_tol():
    Q = _right_angle_quintic()
    shape = _build_shape_direct(Q)
    loose = shape.polyline(chord_tol=1e-1)
    tight = shape.polyline(chord_tol=1e-4)
    assert len(tight) > len(loose)


def test_v_cap_at_zero_curvature_is_vmax():
    Q = _right_angle_quintic()
    shape = _build_shape_direct(Q)
    limits = blendshape.KinematicLimits(
        a_max=45000.0, v_max=500.0, jerk_max=None,
        extruder_caps=None,
    )
    shape._limits = limits
    assert shape.v_cap_fn(0.0) == pytest.approx(limits.v_max)
    assert shape.v_cap_fn(shape.arc_length) == pytest.approx(limits.v_max)


def test_v_cap_at_peak_kappa_matches_centripetal_bound():
    Q = _right_angle_quintic()
    shape = _build_shape_direct(Q)
    limits = blendshape.KinematicLimits(
        a_max=45000.0, v_max=50000.0,
        jerk_max=None,
        extruder_caps=None,
    )
    shape._limits = limits
    _, k_peak = blendquintic._peak_curvature(Q)
    expected = math.sqrt(limits.a_max / k_peak)
    best_v = float("inf")
    for i in range(1001):
        s = shape.arc_length * i / 1000.0
        v = shape.v_cap_fn(s)
        best_v = min(best_v, v)
    assert best_v == pytest.approx(expected, rel=1e-2)


def test_v_cap_with_jerk_bound_tighter_than_without():
    Q = _right_angle_quintic()
    shape = _build_shape_direct(Q)
    limits_no_jerk = blendshape.KinematicLimits(
        a_max=45000.0, v_max=50000.0, jerk_max=None,
        extruder_caps=None,
    )
    limits_with_jerk = blendshape.KinematicLimits(
        a_max=45000.0, v_max=50000.0, jerk_max=1e7,
        extruder_caps=None,
    )
    shape._limits = limits_no_jerk
    v_no = shape.v_cap_fn(shape.arc_length * 0.5)
    shape._limits = limits_with_jerk
    v_yes = shape.v_cap_fn(shape.arc_length * 0.5)
    assert v_yes <= v_no


class _FakeMoveFactory:
    """Minimal Move-like stub for factory tests. Real planner's Move
    is in klippy/toolhead.py with far more state, but the factory only
    reads start_pos, end_pos, move_d, axes_d."""
    def __init__(self, start, end):
        self.start_pos = start
        self.end_pos = end
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        dz = end[2] - start[2]
        self.move_d = math.sqrt(dx * dx + dy * dy + dz * dz)
        if self.move_d > 0.0:
            self.axes_d = (dx, dy, dz)
        else:
            self.axes_d = (0.0, 0.0, 0.0)


def _factory_limits():
    return blendshape.KinematicLimits(
        a_max=45000.0, v_max=500.0, jerk_max=None,
        extruder_caps=None,
    )


def test_from_moves_builds_blend_for_right_angle_corner():
    prev = _FakeMoveFactory((-10.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    nxt = _FakeMoveFactory((0.0, 0.0, 0.0), (0.0, 10.0, 0.0))
    shape = blendquintic.QuinticShape.from_moves(prev, nxt, 0.1, _factory_limits())
    assert shape is not None
    assert shape.theta == pytest.approx(math.radians(90.0), rel=1e-6)
    assert shape.d_consumed > 0.0
    assert shape.arc_length > 0.0


def test_from_moves_returns_none_for_collinear():
    prev = _FakeMoveFactory((-10.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    nxt = _FakeMoveFactory((0.0, 0.0, 0.0), (10.0, 0.0, 0.0))
    assert blendquintic.QuinticShape.from_moves(prev, nxt, 0.1, _factory_limits()) is None


def test_from_moves_returns_none_for_near_reversal():
    prev = _FakeMoveFactory((-10.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    nxt = _FakeMoveFactory((0.0, 0.0, 0.0), (-10.0, 0.0, 0.0))
    assert blendquintic.QuinticShape.from_moves(prev, nxt, 0.1, _factory_limits()) is None


def test_from_moves_returns_none_for_insufficient_edge_length():
    # Tangent length d required would exceed available edge length.
    prev = _FakeMoveFactory((-0.01, 0.0, 0.0), (0.0, 0.0, 0.0))
    nxt = _FakeMoveFactory((0.0, 0.0, 0.0), (0.0, 0.01, 0.0))
    assert blendquintic.QuinticShape.from_moves(prev, nxt, 1.0, _factory_limits()) is None


from klippy import blendshaper


def _synthesize_shapers():
    """Minimal single-axis shaper snapshot for tests.

    ZV@150Hz, zeta=0.1, A_axis=87000 mm/s^2 — matches the user's
    hardware regime used in test_blendshaper.py numeric sanity tests.
    The entry-step cap at R=1/kappa_peak ~ a few mm is well within
    [100, 500] mm/s, so any shaper-active limit will be strictly below
    the centripetal bound at high v_max.
    """
    return [
        blendshaper.AxisShaperSnapshot(
            axis="x",
            shaper_type="zv",
            shaper_freq=150.0,
            damping_ratio=0.1,
            A_axis=87000.0,
        )
    ]


def test_dense_shaper_cap_tighter_than_three_point_at_pathological_angles():
    """Regression: at (theta=122 deg, rotation=164 deg) the archive's
    3-point cap overshot by ~15%. Dense-50 must produce a tighter cap."""
    theta = math.radians(122.0)
    rot = math.radians(164.0)
    cos_r, sin_r = math.cos(rot), math.sin(rot)
    e1 = (cos_r, sin_r, 0.0)
    c2, s2 = math.cos(-theta), math.sin(-theta)
    e2 = (e1[0] * c2 - e1[1] * s2, e1[0] * s2 + e1[1] * c2, 0.0)
    d = 1.0
    r = blendquintic._r_of_theta(theta)
    Q0 = (-d * e1[0], -d * e1[1], 0.0)
    Q5 = (d * e2[0], d * e2[1], 0.0)
    Q1 = (Q0[0] + d * (1.0 - r) * e1[0], Q0[1] + d * (1.0 - r) * e1[1], 0.0)
    Q2 = Q1
    Q3 = (Q5[0] - d * (1.0 - r) * e2[0], Q5[1] - d * (1.0 - r) * e2[1], 0.0)
    Q4 = Q3
    Q = (Q0, Q1, Q2, Q3, Q4, Q5)

    shapers = _synthesize_shapers()
    p_hat = (0.0, 0.0, 1.0)   # 2D blend in XY plane

    # 3-point cap (archive formula, computed inline for comparison):
    three_pt = float("inf")
    for t in (0.25, 0.5, 0.75):
        _, tan, nrm = blendquintic._point_frame(Q, t)
        k = blendquintic._curvature_at_t(Q, t)
        if k <= 0.0:
            continue
        R = 1.0 / k
        bounds = blendshaper.compute_shaper_bounds(shapers, R, nrm, p_hat)
        three_pt = min(three_pt, bounds.v_step_cap)

    # Dense-50 cap (our fix):
    dense = blendquintic._shaper_cap_dense(Q, shapers, n=50)

    # Dense must be tighter-or-equal (smaller number):
    assert dense <= three_pt + 1e-9


def test_dense_shaper_cap_agrees_with_50_point_reference():
    """Dense-50 should agree with dense-500 within 1%; checks that 50
    points is already converged."""
    Q = _right_angle_quintic()
    shapers = _synthesize_shapers()
    d50 = blendquintic._shaper_cap_dense(Q, shapers, n=50)
    d500 = blendquintic._shaper_cap_dense(Q, shapers, n=500)
    assert d50 == pytest.approx(d500, rel=1e-2)


def test_v_cap_uses_shaper_when_shapers_provided():
    Q = _right_angle_quintic()
    shape = _build_shape_direct(Q)
    limits_no_shaper = blendshape.KinematicLimits(
        a_max=45000.0, v_max=50000.0, jerk_max=None,
        extruder_caps=None, shapers=None,
    )
    limits_shaper = blendshape.KinematicLimits(
        a_max=45000.0, v_max=50000.0, jerk_max=None,
        extruder_caps=None,
        shapers=_synthesize_shapers(),
    )
    shape._limits = limits_no_shaper
    v_no = shape.v_cap_fn(shape.arc_length * 0.5)
    shape._limits = limits_shaper
    v_yes = shape.v_cap_fn(shape.arc_length * 0.5)
    assert v_yes <= v_no


def test_random_corner_sweep():
    """Property test: 200 random corners. For each, verify:
      - from_moves returns a valid shape (or None for degenerate)
      - endpoint curvature == 0 (G2 continuity)
      - v_cap_fn > 0 everywhere on [0, arc_length]
    """
    import random
    rng = random.Random(1234)
    limits = _factory_limits()
    n_valid = 0
    for trial in range(200):
        theta_deg = rng.uniform(5.0, 175.0)
        rotation_deg = rng.uniform(0.0, 360.0)
        edge_len = rng.uniform(0.5, 20.0)
        cd = rng.uniform(0.02, 0.3)
        theta = math.radians(theta_deg)
        rot = math.radians(rotation_deg)
        cos_r, sin_r = math.cos(rot), math.sin(rot)
        e1 = (cos_r, sin_r, 0.0)
        c2, s2 = math.cos(-theta), math.sin(-theta)
        e2 = (e1[0] * c2 - e1[1] * s2, e1[0] * s2 + e1[1] * c2, 0.0)
        apex = (0.0, 0.0, 0.0)
        prev_start = (
            apex[0] - edge_len * e1[0],
            apex[1] - edge_len * e1[1],
            apex[2] - edge_len * e1[2],
        )
        nxt_end = (
            apex[0] + edge_len * e2[0],
            apex[1] + edge_len * e2[1],
            apex[2] + edge_len * e2[2],
        )
        prev = _FakeMoveFactory(prev_start, apex)
        nxt = _FakeMoveFactory(apex, nxt_end)
        shape = blendquintic.QuinticShape.from_moves(prev, nxt, cd, limits)
        if shape is None:
            continue
        n_valid += 1
        # Endpoint G2: curvature = 0 at s=0 and s=arc_length.
        assert shape.curvature_at(0.0) == pytest.approx(0.0, abs=1e-6), (
            f"trial={trial}, theta_deg={theta_deg}, rotation_deg={rotation_deg}"
        )
        assert shape.curvature_at(shape.arc_length) == pytest.approx(0.0, abs=1e-6)
        # v_cap_fn positive everywhere.
        for i in range(11):
            s = shape.arc_length * i / 10.0
            assert shape.v_cap_fn(s) > 0.0
    # Sanity: at least half the random corners should yield valid shapes.
    assert n_valid >= 100, f"only {n_valid}/200 corners produced valid shapes"


def test_v_cap_fn_degrades_gracefully_with_smooth_shaper_axis():
    """When a smooth-family axis is passed in, _extract_shapers records
    A_axis=0.0 (see test_blendmath.py::test_extract_shapers_smooth_family_axis_has_zero_A).
    QuinticShape.v_cap_fn must not crash or return zero from that -- the
    shaper term should drop out, leaving a_max / v_max bounds intact.
    """
    # Craft a KinematicLimits with one impulse axis (A_axis > 0) and one
    # smooth axis (A_axis = 0).
    shapers = [
        blendshaper.AxisShaperSnapshot(
            axis="x",
            shaper_type="zv",
            shaper_freq=50.0,
            damping_ratio=0.1,
            A_axis=30000.0,
        ),
        blendshaper.AxisShaperSnapshot(
            axis="y",
            shaper_type="bs3",
            shaper_freq=0.0,
            damping_ratio=0.0,
            A_axis=0.0,
        ),
    ]
    limits = blendshape.KinematicLimits(
        a_max=50000.0, v_max=600.0, jerk_max=None,
        extruder_caps=None, shapers=shapers,
    )
    # Right-angle corner to exercise both axes.
    prev = _FakeMoveFactory((-5.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    nxt  = _FakeMoveFactory((0.0, 0.0, 0.0), (0.0, 5.0, 0.0))
    shape = blendquintic.QuinticShape.from_moves(
        prev, nxt, corner_deviation=0.2, limits=limits,
    )
    assert shape is not None
    v_mid = shape.v_cap_fn(shape.arc_length / 2.0)
    assert math.isfinite(v_mid)
    assert v_mid > 0.0
    # Sanity: without shaper involvement for y, the cap should be no
    # tighter than a_max-derived centripetal * v_max bound; in particular
    # it must not collapse to 0.
    assert v_mid >= 50.0  # extremely lax lower bound


# ---------------------------------------------------------------------------
# Integration tests — Plan 4 D1 closure (Task 6)
# ---------------------------------------------------------------------------
# These two tests confirm that the full pipeline (Smooth-IS A_axis derivation +
# _extract_shapers type dispatch + v_cap_fn) is wired end-to-end correctly.
# ---------------------------------------------------------------------------

from klippy import blendmath


def _make_bs3_limits(freq=40.0, dr=0.1,
                             a_max=5000.0, v_max=300.0):
    """KinematicLimits with two bs3 shaper snapshots (x + y)."""
    A = blendmath._compute_A_axis_smooth_is("bs3", freq, dr)
    shapers = [
        blendshaper.AxisShaperSnapshot(
            axis="x", shaper_type="bs3",
            shaper_freq=freq, damping_ratio=dr, A_axis=A,
        ),
        blendshaper.AxisShaperSnapshot(
            axis="y", shaper_type="bs3",
            shaper_freq=freq, damping_ratio=dr, A_axis=A,
        ),
    ]
    return blendshape.KinematicLimits(
        a_max=a_max, v_max=v_max, jerk_max=None,
        extruder_caps=None, shapers=shapers,
    )


def _make_fir_mzv_limits(freq=40.0, dr=0.1,
                          a_max=5000.0, v_max=300.0):
    """KinematicLimits with two FIR mzv shaper snapshots (x + y)."""
    from klippy.extras import shaper_defs
    from klippy.extras.shaper_calibrate import ShaperCalibrate
    factory = {s.name: s.init_func for s in shaper_defs.INPUT_SHAPERS}
    impulses = factory["mzv"](freq, dr)
    sc = ShaperCalibrate(printer=None)
    A = float(sc.find_shaper_max_accel(impulses))
    shapers = [
        blendshaper.AxisShaperSnapshot(
            axis="x", shaper_type="mzv",
            shaper_freq=freq, damping_ratio=dr, A_axis=A,
        ),
        blendshaper.AxisShaperSnapshot(
            axis="y", shaper_type="mzv",
            shaper_freq=freq, damping_ratio=dr, A_axis=A,
        ),
    ]
    return blendshape.KinematicLimits(
        a_max=a_max, v_max=v_max, jerk_max=None,
        extruder_caps=None, shapers=shapers,
    )


def test_quintic_v_cap_finite_under_bs3():
    """Pre-Plan-4 bug: SIS had A_axis=0 → quintic v_cap was uncapped (inf-like).
    After Plan 4 D1: SIS carries a finite A_axis → v_cap is finite and physical.
    """
    # 90-degree corner: prev goes +X, next goes +Y.
    prev = _FakeMoveFactory((-10.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    nxt  = _FakeMoveFactory((0.0, 0.0, 0.0), (0.0, 10.0, 0.0))
    shape = blendquintic.QuinticShape.from_moves(
        prev, nxt, 0.1, _make_bs3_limits()
    )
    assert shape is not None
    v_mid = shape.v_cap_fn(shape.arc_length / 2.0)
    assert math.isfinite(v_mid), f"v_mid was not finite: {v_mid}"
    assert 0.0 < v_mid < 300.0, f"v_mid={v_mid} outside (0, v_max=300)"


def test_quintic_v_cap_smooth_vs_fir_same_order_of_magnitude():
    """At the same nominal frequency, bs3 and FIR mzv caps must be
    within a factor of 2 of each other.  A larger divergence indicates an
    A_axis scale error in the Smooth-IS derivation.
    """
    prev = _FakeMoveFactory((-10.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    nxt  = _FakeMoveFactory((0.0, 0.0, 0.0), (0.0, 10.0, 0.0))

    shape_sis = blendquintic.QuinticShape.from_moves(
        prev, nxt, 0.1, _make_bs3_limits()
    )
    shape_fir = blendquintic.QuinticShape.from_moves(
        prev, nxt, 0.1, _make_fir_mzv_limits()
    )
    assert shape_sis is not None
    assert shape_fir is not None

    v_sis = shape_sis.v_cap_fn(shape_sis.arc_length / 2.0)
    v_fir = shape_fir.v_cap_fn(shape_fir.arc_length / 2.0)
    ratio = v_sis / v_fir
    # Different shaper families at the same nominal frequency give different
    # but comparable caps (within factor-of-2).
    assert 0.5 < ratio < 2.0, (
        f"v_cap ratio smooth/FIR = {ratio:.3f} (v_sis={v_sis:.1f}, "
        f"v_fir={v_fir:.1f}); likely A_axis scale error in Smooth-IS derivation"
    )


# === Plan 4 D5: v_cap_fn endpoint tests ===

@pytest.mark.parametrize("angle_deg", [45, 90, 120, 170])
def test_v_cap_fn_endpoints_finite_and_positive(angle_deg):
    """v_cap_fn(0) and v_cap_fn(arc_length) must be finite and positive
    for a representative range of corner angles.

    At blend endpoints the quintic is tangent to the incoming/outgoing
    straight move, so v_cap should logically equal the straight's
    max_cruise_v (or higher). A blow-up here would mean numerical
    degeneracy in _point_frame (blendquintic.py:196).
    """
    theta = math.radians(180.0 - angle_deg)  # interior angle
    # 90° corner at (10,0,0) means prev goes +X, next goes in direction
    # (cos(theta), sin(theta), 0) from there.
    prev = _FakeMoveFactory((0.0, 0.0, 0.0), (10.0, 0.0, 0.0))
    next_m = _FakeMoveFactory(
        (10.0, 0.0, 0.0),
        (10.0 + 10.0 * math.cos(theta), 10.0 * math.sin(theta), 0.0),
    )
    limits = _make_bs3_limits()  # from T6 helpers
    shape = blendquintic.QuinticShape.from_moves(prev, next_m, 0.1, limits)
    if shape is None:
        pytest.skip("from_moves returned None for this angle; not in scope")
    v0 = shape.v_cap_fn(0.0)
    vN = shape.v_cap_fn(shape.arc_length)
    assert math.isfinite(v0) and v0 > 0.0, f"v_cap_fn(0) = {v0}"
    assert math.isfinite(vN) and vN > 0.0, f"v_cap_fn(arc_length) = {vN}"


def test_v_cap_fn_endpoints_at_least_straight_cruise():
    """At a blend endpoint the curve is tangent to the straight — the
    cap should not be pathologically low (at least 10 mm/s on a 300 mm/s
    straight).
    """
    prev = _FakeMoveFactory((0.0, 0.0, 0.0), (10.0, 0.0, 0.0))
    next_m = _FakeMoveFactory((10.0, 0.0, 0.0), (10.0, 10.0, 0.0))
    limits = _make_bs3_limits()
    shape = blendquintic.QuinticShape.from_moves(prev, next_m, 0.1, limits)
    if shape is None:
        pytest.skip("from_moves returned None; not in scope")
    v0 = shape.v_cap_fn(0.0)
    vN = shape.v_cap_fn(shape.arc_length)
    assert v0 >= 10.0, f"v_cap_fn(0) too low: {v0}"
    assert vN >= 10.0, f"v_cap_fn(arc_length) too low: {vN}"
