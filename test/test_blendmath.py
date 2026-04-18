# test/test_blendmath.py
import math

import pytest

from klippy import blendmath


def test_vec_dot():
    assert blendmath.vdot((1.0, 0.0, 0.0), (0.0, 1.0, 0.0)) == 0.0
    assert blendmath.vdot((1.0, 2.0, 3.0), (4.0, 5.0, 6.0)) == 32.0


def test_vec_cross():
    assert blendmath.vcross((1.0, 0.0, 0.0), (0.0, 1.0, 0.0)) == (0.0, 0.0, 1.0)
    assert blendmath.vcross((0.0, 1.0, 0.0), (1.0, 0.0, 0.0)) == (0.0, 0.0, -1.0)


def test_vec_norm():
    assert blendmath.vnorm((3.0, 4.0, 0.0)) == 5.0
    assert blendmath.vnorm((0.0, 0.0, 0.0)) == 0.0


def test_vec_scale():
    assert blendmath.vscale((1.0, 2.0, 3.0), 2.0) == (2.0, 4.0, 6.0)


def test_vec_add_sub():
    assert blendmath.vadd((1.0, 2.0, 3.0), (4.0, 5.0, 6.0)) == (5.0, 7.0, 9.0)
    assert blendmath.vsub((4.0, 5.0, 6.0), (1.0, 2.0, 3.0)) == (3.0, 3.0, 3.0)


def test_vec_normalize():
    n = blendmath.vnormalize((3.0, 4.0, 0.0))
    assert n == pytest.approx((0.6, 0.8, 0.0))

    with pytest.raises(ValueError):
        blendmath.vnormalize((0.0, 0.0, 0.0))


def test_blend_arc_dataclass_fields():
    arc = blendmath.BlendArc(
        R=5.0,
        theta=math.pi / 2,
        d_consumed=5.0,
        v_cap=100.0,
        center=(0.0, 5.0, 0.0),
        entry_pt=(-5.0, 0.0, 0.0),
        exit_pt=(0.0, 5.0, 0.0),
        entry_tangent=(1.0, 0.0, 0.0),
        exit_tangent=(0.0, 1.0, 0.0),
        plane_normal=(0.0, 0.0, 1.0),
    )
    assert arc.R == 5.0
    assert arc.theta == math.pi / 2
    assert arc.d_consumed == 5.0
    assert arc.v_cap == 100.0
    assert arc.center == (0.0, 5.0, 0.0)
    assert arc.entry_pt == (-5.0, 0.0, 0.0)
    assert arc.exit_pt == (0.0, 5.0, 0.0)
    assert arc.entry_tangent == (1.0, 0.0, 0.0)
    assert arc.exit_tangent == (0.0, 1.0, 0.0)
    assert arc.plane_normal == (0.0, 0.0, 1.0)


def test_blend_geometry_collinear_returns_none():
    # Same direction → deflection = 0 → no blend needed
    prev_dir = (1.0, 0.0, 0.0)
    next_dir = (1.0, 0.0, 0.0)
    result = blendmath.blend_geometry(
        prev_dir=prev_dir,
        next_dir=next_dir,
        L_prev=10.0,
        L_next=10.0,
        corner_deviation=0.02,
        a_max=50000.0,
        j_eff=1e7,
    )
    assert result is None


def test_blend_geometry_near_collinear_returns_none():
    # Tiny deflection below threshold → also None
    prev_dir = (1.0, 0.0, 0.0)
    # 1e-8 rad deflection
    eps = 1e-8
    next_dir = (math.cos(eps), math.sin(eps), 0.0)
    result = blendmath.blend_geometry(
        prev_dir=prev_dir,
        next_dir=next_dir,
        L_prev=10.0,
        L_next=10.0,
        corner_deviation=0.02,
        a_max=50000.0,
        j_eff=1e7,
    )
    assert result is None


def test_blend_geometry_u_turn_returns_zero_arc():
    # Anti-parallel directions: theta = pi.
    prev_dir = (1.0, 0.0, 0.0)
    next_dir = (-1.0, 0.0, 0.0)
    result = blendmath.blend_geometry(
        prev_dir=prev_dir,
        next_dir=next_dir,
        L_prev=10.0,
        L_next=10.0,
        corner_deviation=0.02,
        a_max=50000.0,
        j_eff=1e7,
    )
    assert result is not None
    assert result.R == 0.0
    assert result.v_cap == 0.0


def test_blend_geometry_near_u_turn_returns_zero_arc():
    prev_dir = (1.0, 0.0, 0.0)
    # 1e-8 rad shy of U-turn
    eps = 1e-8
    next_dir = (-math.cos(eps), math.sin(eps), 0.0)
    result = blendmath.blend_geometry(
        prev_dir=prev_dir,
        next_dir=next_dir,
        L_prev=10.0,
        L_next=10.0,
        corner_deviation=0.02,
        a_max=50000.0,
        j_eff=1e7,
    )
    assert result is not None
    assert result.R == 0.0
    assert result.v_cap == 0.0


def test_blend_geometry_90deg_tolerance_radius():
    # 90 degree corner, X -> Y.
    # theta = pi/2, so cos(theta/2) = sqrt(2)/2.
    # R_tol = corner_deviation * (sqrt(2)/2) / (1 - sqrt(2)/2)
    prev_dir = (1.0, 0.0, 0.0)
    next_dir = (0.0, 1.0, 0.0)
    corner_dev = 0.02  # mm
    # Adjacent segments much longer than the arc, jerk and accel loose:
    result = blendmath.blend_geometry(
        prev_dir=prev_dir,
        next_dir=next_dir,
        L_prev=1000.0,
        L_next=1000.0,
        corner_deviation=corner_dev,
        a_max=1.0,      # trivial acceleration → v_cap not the limiting factor here
        j_eff=1e30,     # jerk floor effectively disabled
    )
    assert result is not None
    expected_R = corner_dev * (math.sqrt(2) / 2) / (1 - math.sqrt(2) / 2)
    assert result.R == pytest.approx(expected_R, rel=1e-9)
    assert result.theta == pytest.approx(math.pi / 2, rel=1e-9)


def test_blend_geometry_60deg_tolerance_radius():
    # 60 degree deflection: prev along +X, next rotated 60 degrees counter-clockwise.
    prev_dir = (1.0, 0.0, 0.0)
    theta = math.pi / 3
    next_dir = (math.cos(theta), math.sin(theta), 0.0)
    corner_dev = 0.05
    cos_half = math.cos(theta / 2)
    expected_R = corner_dev * cos_half / (1 - cos_half)
    result = blendmath.blend_geometry(
        prev_dir=prev_dir,
        next_dir=next_dir,
        L_prev=1000.0,
        L_next=1000.0,
        corner_deviation=corner_dev,
        a_max=1.0,
        j_eff=1e30,
    )
    assert result is not None
    assert result.R == pytest.approx(expected_R, rel=1e-9)


def test_blend_geometry_midpoint_cap_binds_on_short_segment():
    # 90 deg corner, but one adjacent segment is short.
    # Half-segment rule: R_mid = 0.5 * min(L_prev, L_next) * cot(theta/2)
    #                         = 0.5 * 0.5 * 1.0 = 0.25 mm
    # R_tol should be much larger given the tolerance; verify R_mid wins.
    prev_dir = (1.0, 0.0, 0.0)
    next_dir = (0.0, 1.0, 0.0)
    corner_dev = 5.0  # absurdly loose tolerance so R_tol is the larger value
    L_short = 0.5
    result = blendmath.blend_geometry(
        prev_dir=prev_dir,
        next_dir=next_dir,
        L_prev=L_short,
        L_next=1000.0,
        corner_deviation=corner_dev,
        a_max=1.0,
        j_eff=1e30,
    )
    assert result is not None
    cos_half = math.sqrt(2) / 2
    sin_half = math.sqrt(2) / 2
    expected_R_mid = 0.5 * L_short * cos_half / sin_half  # = 0.25 (half-segment rule)
    assert result.R == pytest.approx(expected_R_mid, rel=1e-9)
    # d_consumed = R * tan(theta/2) = R for 90 deg. At R=0.25, d=0.25 (= L_short/2).
    assert result.d_consumed == pytest.approx(L_short * 0.5, rel=1e-9)


def test_blend_geometry_90deg_geometry_positioning():
    # Corner at origin: prev move ends at (0,0,0) heading +X,
    # next move starts at (0,0,0) heading +Y.
    # In this pure-geometry API we don't pass the vertex; entry/exit are
    # expressed in a local frame relative to the corner vertex. Convention:
    # corner vertex is the origin.
    prev_dir = (1.0, 0.0, 0.0)
    next_dir = (0.0, 1.0, 0.0)
    corner_dev = 0.02
    result = blendmath.blend_geometry(
        prev_dir=prev_dir,
        next_dir=next_dir,
        L_prev=1000.0,
        L_next=1000.0,
        corner_deviation=corner_dev,
        a_max=1.0,
        j_eff=1e30,
    )
    assert result is not None
    R = result.R
    d = result.d_consumed  # should equal R for 90 deg
    # Entry point sits distance d back along prev_dir from the vertex (origin).
    # prev_dir is the direction the toolhead WAS heading, so the entry point
    # lies at origin - d*prev_dir (upstream of the vertex along the incoming ray).
    expected_entry = (-d, 0.0, 0.0)
    expected_exit = (0.0, d, 0.0)
    # Center sits on the angle bisector interior to the corner, distance
    # R from each tangent point. For this 90 deg +X -> +Y corner it's at
    # (-d, d, 0) i.e. (-R, R, 0) in the corner frame.
    expected_center = (-R, R, 0.0)
    # Plane normal: prev_dir x next_dir = (1,0,0) x (0,1,0) = (0,0,1).
    expected_normal = (0.0, 0.0, 1.0)
    assert result.entry_pt == pytest.approx(expected_entry, abs=1e-12)
    assert result.exit_pt == pytest.approx(expected_exit, abs=1e-12)
    assert result.center == pytest.approx(expected_center, abs=1e-12)
    assert result.plane_normal == pytest.approx(expected_normal, abs=1e-12)
    assert result.entry_tangent == prev_dir
    assert result.exit_tangent == next_dir


def test_blend_geometry_centripetal_cap():
    # 90 deg corner with tight accel budget; jerk floor effectively disabled.
    # v_cap_centripetal = sqrt((sqrt(3)/2) * a_max * R)
    prev_dir = (1.0, 0.0, 0.0)
    next_dir = (0.0, 1.0, 0.0)
    corner_dev = 0.02
    a_max = 50000.0
    result = blendmath.blend_geometry(
        prev_dir=prev_dir,
        next_dir=next_dir,
        L_prev=1000.0,
        L_next=1000.0,
        corner_deviation=corner_dev,
        a_max=a_max,
        j_eff=1e30,
    )
    assert result is not None
    expected_v = math.sqrt((math.sqrt(3) / 2) * a_max * result.R)
    assert result.v_cap == pytest.approx(expected_v, rel=1e-9)


def test_blend_geometry_jerk_floor_dominates():
    # Tight jerk budget: v_cap should drop to (R * sqrt(j))^(2/3).
    prev_dir = (1.0, 0.0, 0.0)
    next_dir = (0.0, 1.0, 0.0)
    corner_dev = 0.02
    a_max = 50000.0
    j_eff = 1e4  # very tight jerk → jerk cap should dominate
    result = blendmath.blend_geometry(
        prev_dir=prev_dir,
        next_dir=next_dir,
        L_prev=1000.0,
        L_next=1000.0,
        corner_deviation=corner_dev,
        a_max=a_max,
        j_eff=j_eff,
    )
    assert result is not None
    expected_v_jerk = (result.R * math.sqrt(j_eff)) ** (2.0 / 3.0)
    expected_v_centripetal = math.sqrt((math.sqrt(3) / 2) * a_max * result.R)
    # Jerk cap should win.
    assert expected_v_jerk < expected_v_centripetal
    assert result.v_cap == pytest.approx(expected_v_jerk, rel=1e-9)


def test_blend_geometry_jerk_floor_loose_does_not_bind():
    # Very loose jerk: centripetal should still dominate.
    prev_dir = (1.0, 0.0, 0.0)
    next_dir = (0.0, 1.0, 0.0)
    result = blendmath.blend_geometry(
        prev_dir=prev_dir,
        next_dir=next_dir,
        L_prev=1000.0,
        L_next=1000.0,
        corner_deviation=0.02,
        a_max=50000.0,
        j_eff=1e30,
    )
    assert result is not None
    expected_v_centripetal = math.sqrt((math.sqrt(3) / 2) * 50000.0 * result.R)
    assert result.v_cap == pytest.approx(expected_v_centripetal, rel=1e-9)


import random


def _rand_unit_vec(rng: random.Random) -> blendmath.Vec3:
    # Uniform direction on the XY plane is enough for property tests.
    phi = rng.uniform(0.0, 2.0 * math.pi)
    return (math.cos(phi), math.sin(phi), 0.0)


@pytest.mark.parametrize("seed", range(50))
def test_blend_geometry_property_random_corners(seed):
    rng = random.Random(seed)
    # Random first direction.
    prev_dir = _rand_unit_vec(rng)
    # Random deflection in (0.01 rad, pi - 0.01 rad) to stay away from degenerates.
    theta = rng.uniform(0.01, math.pi - 0.01)
    # Rotate prev_dir by theta about +Z to get next_dir.
    c, s = math.cos(theta), math.sin(theta)
    next_dir = (
        c * prev_dir[0] - s * prev_dir[1],
        s * prev_dir[0] + c * prev_dir[1],
        0.0,
    )
    L_prev = rng.uniform(0.5, 100.0)
    L_next = rng.uniform(0.5, 100.0)
    corner_dev = rng.uniform(0.001, 0.1)
    a_max = rng.uniform(1000.0, 100000.0)
    j_eff = rng.uniform(1e5, 1e9)

    result = blendmath.blend_geometry(
        prev_dir=prev_dir,
        next_dir=next_dir,
        L_prev=L_prev,
        L_next=L_next,
        corner_deviation=corner_dev,
        a_max=a_max,
        j_eff=j_eff,
    )
    assert result is not None
    R = result.R
    d = result.d_consumed

    # 1. Consumed length fits inside both segments.
    assert d <= L_prev + 1e-9
    assert d <= L_next + 1e-9

    # 2. Chord deviation of this single arc: epsilon = R*(1/cos(theta/2) - 1).
    # Must not exceed corner_deviation (unless midpoint cap made R smaller,
    # in which case epsilon <= corner_deviation trivially).
    cos_half = math.cos(theta / 2)
    eps_arc = R * (1.0 / cos_half - 1.0)
    assert eps_arc <= corner_dev + 1e-9

    # 3. v_cap respects centripetal bound.
    a_n_max = (math.sqrt(3) / 2) * a_max
    assert result.v_cap ** 2 <= a_n_max * R + 1e-6

    # 4. v_cap respects jerk floor.
    #    v^(3/2) <= R * sqrt(j_eff)
    assert result.v_cap ** 1.5 <= R * math.sqrt(j_eff) + 1e-6

    # 5. Tangent points lie on the adjacent rays.
    #    entry_pt should be collinear with prev_dir (at -d * prev_dir).
    assert result.entry_pt == pytest.approx(
        (-d * prev_dir[0], -d * prev_dir[1], -d * prev_dir[2]), abs=1e-9
    )
    assert result.exit_pt == pytest.approx(
        (d * next_dir[0], d * next_dir[1], d * next_dir[2]), abs=1e-9
    )

    # 6. Center is distance R from both tangent points.
    from_entry = blendmath.vsub(result.center, result.entry_pt)
    from_exit = blendmath.vsub(result.center, result.exit_pt)
    assert blendmath.vnorm(from_entry) == pytest.approx(R, rel=1e-6)
    assert blendmath.vnorm(from_exit) == pytest.approx(R, rel=1e-6)

    # 7. Center lies on the interior side of the corner (dot with next_dir > 0 from entry_pt).
    interior_check = blendmath.vdot(blendmath.vsub(result.center, result.entry_pt), next_dir)
    assert interior_check > -1e-9


def test_segment_arc_90deg_basic():
    # Build a 90 deg arc with R=10, max_chord_err=0.01.
    # Delta phi per segment: 2*acos(1 - 0.01/10) = 2*acos(0.999) rad ~= 0.0894 rad.
    # Total arc angle (theta) = pi/2 rad. Expected segments ~= (pi/2)/0.0894 ~= 17.56, so 18.
    arc = blendmath.BlendArc(
        R=10.0,
        theta=math.pi / 2,
        d_consumed=10.0,
        v_cap=100.0,
        center=(-10.0, 10.0, 0.0),
        entry_pt=(-10.0, 0.0, 0.0),
        exit_pt=(0.0, 10.0, 0.0),
        entry_tangent=(1.0, 0.0, 0.0),
        exit_tangent=(0.0, 1.0, 0.0),
        plane_normal=(0.0, 0.0, 1.0),
    )
    polyline = blendmath.segment_arc(arc, max_chord_err=0.01)

    # First and last points are entry and exit.
    assert polyline[0] == pytest.approx(arc.entry_pt, abs=1e-9)
    assert polyline[-1] == pytest.approx(arc.exit_pt, abs=1e-9)

    # Every point lies on the arc (distance R from center).
    for pt in polyline:
        d = blendmath.vnorm(blendmath.vsub(pt, arc.center))
        assert d == pytest.approx(arc.R, rel=1e-9)

    # Reasonable point count (theta / delta_phi + 1).
    delta_phi_max = 2.0 * math.acos(1.0 - 0.01 / 10.0)
    expected_segments = math.ceil(arc.theta / delta_phi_max)
    assert len(polyline) == expected_segments + 1


def test_segment_arc_zero_radius_returns_degenerate_polyline():
    # R=0 (U-turn case): polyline is just [entry_pt, exit_pt] (both equal).
    arc = blendmath.BlendArc(
        R=0.0,
        theta=math.pi,
        d_consumed=0.0,
        v_cap=0.0,
        center=(0.0, 0.0, 0.0),
        entry_pt=(0.0, 0.0, 0.0),
        exit_pt=(0.0, 0.0, 0.0),
        entry_tangent=(1.0, 0.0, 0.0),
        exit_tangent=(-1.0, 0.0, 0.0),
        plane_normal=(0.0, 0.0, 0.0),
    )
    polyline = blendmath.segment_arc(arc, max_chord_err=0.01)
    assert polyline == [(0.0, 0.0, 0.0)]


@pytest.mark.parametrize("seed", range(30))
def test_segment_arc_chord_error_bound(seed):
    rng = random.Random(seed + 10_000)
    # Build a valid arc from blend_geometry on a random corner.
    prev_dir = _rand_unit_vec(rng)
    theta = rng.uniform(0.05, math.pi - 0.05)
    c, s = math.cos(theta), math.sin(theta)
    next_dir = (
        c * prev_dir[0] - s * prev_dir[1],
        s * prev_dir[0] + c * prev_dir[1],
        0.0,
    )
    arc = blendmath.blend_geometry(
        prev_dir=prev_dir,
        next_dir=next_dir,
        L_prev=100.0,
        L_next=100.0,
        corner_deviation=rng.uniform(0.01, 0.2),
        a_max=50000.0,
        j_eff=1e8,
    )
    assert arc is not None

    max_chord_err = rng.uniform(0.0005, 0.05)
    polyline = blendmath.segment_arc(arc, max_chord_err=max_chord_err)

    # Each consecutive pair: midpoint's deviation from the arc should be
    # <= max_chord_err (with a small numeric slack).
    for p0, p1 in zip(polyline, polyline[1:]):
        midpoint = ((p0[0] + p1[0]) / 2, (p0[1] + p1[1]) / 2, (p0[2] + p1[2]) / 2)
        # Deviation = R - |midpoint - center| (on arc side, midpoint is inside).
        dist_from_center = blendmath.vnorm(blendmath.vsub(midpoint, arc.center))
        chord_err = arc.R - dist_from_center
        # chord_err should be in [0, max_chord_err + small slack]
        assert chord_err >= -1e-9
        assert chord_err <= max_chord_err + 1e-6


class _FakeMove:
    """Minimal duck-typed stand-in for Kalico's Move class."""

    def __init__(self, axes_r, move_d, accel, max_cruise_v2, is_kinematic_move=True):
        # Kalico's Move.axes_r is a 4-vector [x, y, z, e]; only [:3] is used here.
        self.axes_r = axes_r
        self.move_d = move_d
        self.accel = accel
        self.max_cruise_v2 = max_cruise_v2
        self.is_kinematic_move = is_kinematic_move


def test_blend_from_moves_matches_pure_math():
    prev = _FakeMove(
        axes_r=[1.0, 0.0, 0.0, 0.0],
        move_d=50.0,
        accel=50000.0,
        max_cruise_v2=1e6,
    )
    nxt = _FakeMove(
        axes_r=[0.0, 1.0, 0.0, 0.0],
        move_d=50.0,
        accel=50000.0,
        max_cruise_v2=1e6,
    )
    corner_dev = 0.02
    j_eff = 1e8

    adapter_result = blendmath.blend_from_moves(
        prev_move=prev,
        next_move=nxt,
        corner_deviation=corner_dev,
        j_eff=j_eff,
    )
    core_result = blendmath.blend_geometry(
        prev_dir=(1.0, 0.0, 0.0),
        next_dir=(0.0, 1.0, 0.0),
        L_prev=50.0,
        L_next=50.0,
        corner_deviation=corner_dev,
        a_max=50000.0,  # min(prev.accel, nxt.accel)
        j_eff=j_eff,
    )
    assert adapter_result is not None
    assert core_result is not None
    assert adapter_result.R == pytest.approx(core_result.R, rel=1e-12)
    assert adapter_result.v_cap == pytest.approx(core_result.v_cap, rel=1e-12)


def test_blend_from_moves_non_kinematic_returns_none():
    prev = _FakeMove(
        axes_r=[1.0, 0.0, 0.0, 0.0], move_d=1.0, accel=1.0, max_cruise_v2=1.0
    )
    nxt = _FakeMove(
        axes_r=[0.0, 0.0, 0.0, 1.0],
        move_d=1.0,
        accel=1.0,
        max_cruise_v2=1.0,
        is_kinematic_move=False,
    )
    result = blendmath.blend_from_moves(
        prev_move=prev, next_move=nxt, corner_deviation=0.02, j_eff=1e8
    )
    assert result is None


def test_interpolate_extruder_through_arc():
    # Setup: a blend arc polyline, plus E-axis consumption rates per mm
    # for prev and next moves. The adapter helper should produce a list of
    # (x, y, z, e) points whose E increases monotonically from 0 to the
    # total E consumption across the blend arc length.
    polyline = [
        (-1.0, 0.0, 0.0),
        (-0.9, 0.1, 0.0),
        (-0.5, 0.5, 0.0),
        (-0.1, 0.9, 0.0),
        (0.0, 1.0, 0.0),
    ]
    # Suppose e_per_mm_prev = 0.05, e_per_mm_next = 0.04, and the arc
    # consumes d=1.0 from each side.
    e_per_mm_prev = 0.05
    e_per_mm_next = 0.04
    d_consumed = 1.0

    points_xyze = blendmath.interpolate_extruder(
        polyline,
        d_consumed=d_consumed,
        e_per_mm_prev=e_per_mm_prev,
        e_per_mm_next=e_per_mm_next,
    )

    # First point has E=0 (start of the blend).
    assert points_xyze[0][3] == pytest.approx(0.0, abs=1e-12)
    # Last point has total E = d_consumed * (prev_rate + next_rate) consumed over
    # the two halves of the blend. The blend replaces the final d_consumed mm of
    # the prev move (consuming d_consumed * e_per_mm_prev) plus the first
    # d_consumed mm of the next move (consuming d_consumed * e_per_mm_next).
    expected_total_e = d_consumed * (e_per_mm_prev + e_per_mm_next)
    assert points_xyze[-1][3] == pytest.approx(expected_total_e, rel=1e-9)
    # Monotonic non-decreasing.
    for p0, p1 in zip(points_xyze, points_xyze[1:]):
        assert p1[3] >= p0[3] - 1e-12
    # Length of output matches polyline.
    assert len(points_xyze) == len(polyline)


def test_regression_exact_collinear():
    # Exactly parallel directions.
    assert blendmath.blend_geometry(
        prev_dir=(1.0, 0.0, 0.0),
        next_dir=(1.0, 0.0, 0.0),
        L_prev=10.0,
        L_next=10.0,
        corner_deviation=0.01,
        a_max=1000.0,
        j_eff=1e8,
    ) is None


def test_regression_exact_u_turn():
    result = blendmath.blend_geometry(
        prev_dir=(1.0, 0.0, 0.0),
        next_dir=(-1.0, 0.0, 0.0),
        L_prev=10.0,
        L_next=10.0,
        corner_deviation=0.01,
        a_max=1000.0,
        j_eff=1e8,
    )
    assert result is not None
    assert result.R == 0.0
    assert result.v_cap == 0.0


def test_regression_collinear_threshold_boundary():
    # sin_half = 1e-7 < COLLINEAR_EPS (1e-6), so should be treated as collinear.
    prev_dir = (1.0, 0.0, 0.0)
    # angle = 2 * asin(1e-7) rad
    angle = 2.0 * math.asin(1e-7)
    next_dir = (math.cos(angle), math.sin(angle), 0.0)
    assert blendmath.blend_geometry(
        prev_dir=prev_dir,
        next_dir=next_dir,
        L_prev=10.0,
        L_next=10.0,
        corner_deviation=0.01,
        a_max=1000.0,
        j_eff=1e8,
    ) is None


def test_regression_reversal_threshold_boundary():
    # cos_half = 1e-7 < REVERSAL_EPS (1e-6), so should be treated as U-turn.
    prev_dir = (1.0, 0.0, 0.0)
    # deflection of pi - 2e-7 rad
    angle = math.pi - 2.0 * math.asin(1e-7)
    next_dir = (math.cos(angle), math.sin(angle), 0.0)
    result = blendmath.blend_geometry(
        prev_dir=prev_dir,
        next_dir=next_dir,
        L_prev=10.0,
        L_next=10.0,
        corner_deviation=0.01,
        a_max=1000.0,
        j_eff=1e8,
    )
    assert result is not None
    assert result.R == 0.0
    assert result.v_cap == 0.0


def test_regression_very_short_segment_produces_tiny_arc():
    # Segment shorter than the tolerance-driven arc would want.
    prev_dir = (1.0, 0.0, 0.0)
    next_dir = (0.0, 1.0, 0.0)
    result = blendmath.blend_geometry(
        prev_dir=prev_dir,
        next_dir=next_dir,
        L_prev=0.01,  # 10 microns
        L_next=1000.0,
        corner_deviation=0.5,
        a_max=50000.0,
        j_eff=1e8,
    )
    assert result is not None
    assert result.R == pytest.approx(0.005, rel=1e-9)  # 90 deg: R = 0.5 * L (half-segment rule)
    assert result.v_cap > 0.0


def test_segment_arc_zero_max_chord_err_raises():
    arc = blendmath.BlendArc(
        R=10.0,
        theta=math.pi / 2,
        d_consumed=10.0,
        v_cap=100.0,
        center=(-10.0, 10.0, 0.0),
        entry_pt=(-10.0, 0.0, 0.0),
        exit_pt=(0.0, 10.0, 0.0),
        entry_tangent=(1.0, 0.0, 0.0),
        exit_tangent=(0.0, 1.0, 0.0),
        plane_normal=(0.0, 0.0, 1.0),
    )
    with pytest.raises(ValueError, match="max_chord_err must be positive"):
        blendmath.segment_arc(arc, max_chord_err=0.0)
    with pytest.raises(ValueError, match="max_chord_err must be positive"):
        blendmath.segment_arc(arc, max_chord_err=-1e-3)


class _FakeAxisInputShaper:
    def __init__(self, axis, shaper_type, freq, damping_ratio=0.1):
        self.axis = axis
        self._type = shaper_type
        self._freq = freq
        self._damping = damping_ratio

    class _Params:
        def __init__(self, outer):
            self.shaper_type = outer._type
            self.shaper_freq = outer._freq
            self.damping_ratio = outer._damping

    @property
    def params(self):
        return self._Params(self)


class _FakeInputShaper:
    def __init__(self, shapers):
        self._shapers = shapers

    def get_shapers(self):
        return list(self._shapers)


class _FakePrinterObject:
    def __init__(self, input_shaper):
        self._is = input_shaper

    def lookup_object(self, name, default=None):
        if name == "input_shaper":
            return self._is
        return default


class _FakeToolheadWithShapers:
    def __init__(self, input_shaper):
        self.printer = _FakePrinterObject(input_shaper)


def test_extract_shapers_two_axes():
    is_obj = _FakeInputShaper([
        _FakeAxisInputShaper("x", "zv", 150.0),
        _FakeAxisInputShaper("y", "zv", 80.0),
    ])
    toolhead = _FakeToolheadWithShapers(is_obj)
    snaps = blendmath._extract_shapers(toolhead)
    snaps_by_axis = {s.axis: s for s in snaps}
    assert snaps_by_axis["x"].shaper_freq == 150.0
    assert snaps_by_axis["x"].shaper_type == "zv"
    assert snaps_by_axis["y"].shaper_freq == 80.0
    # A_axis is populated from find_shaper_max_accel — positive for shaped axes.
    assert snaps_by_axis["x"].A_axis > 0.0
    assert snaps_by_axis["y"].A_axis > 0.0
    # X should have larger A_axis (higher frequency, more accel budget).
    assert snaps_by_axis["x"].A_axis > snaps_by_axis["y"].A_axis


def test_extract_shapers_none_toolhead_returns_empty():
    assert blendmath._extract_shapers(None) == []


def test_extract_shapers_no_input_shaper_module_returns_empty():
    class _FakePrinterObjectNoIS:
        def lookup_object(self, name, default=None):
            return default

    class _FakeToolhead:
        printer = _FakePrinterObjectNoIS()

    assert blendmath._extract_shapers(_FakeToolhead()) == []


def test_extract_shapers_unshaped_axis_has_zero_A():
    # Axis with shaper_freq=0 is unshaped → snapshot carries A_axis=0.
    is_obj = _FakeInputShaper([
        _FakeAxisInputShaper("x", "zv", 0.0),
        _FakeAxisInputShaper("y", "zv", 80.0),
    ])
    toolhead = _FakeToolheadWithShapers(is_obj)
    snaps = blendmath._extract_shapers(toolhead)
    snaps_by_axis = {s.axis: s for s in snaps}
    assert snaps_by_axis["x"].shaper_freq == 0.0
    assert snaps_by_axis["x"].A_axis == 0.0
    # shaper_type is mirrored from params regardless of freq
    assert snaps_by_axis["x"].shaper_type == "zv"


def test_blend_from_moves_with_toolhead_derives_j_eff():
    # Set up a 90° XY corner with X=ZV@150Hz, Y=ZV@80Hz. Expect
    # v_cap to match the spec's numeric sanity: ~99.8 mm/s at R=0.5mm.
    prev = _FakeMove(
        axes_r=[1.0, 0.0, 0.0, 0.0], move_d=50.0,
        accel=50000.0, max_cruise_v2=1e12,
    )
    nxt = _FakeMove(
        axes_r=[0.0, 1.0, 0.0, 0.0], move_d=50.0,
        accel=50000.0, max_cruise_v2=1e12,
    )
    is_obj = _FakeInputShaper([
        _FakeAxisInputShaper("x", "zv", 150.0),
        _FakeAxisInputShaper("y", "zv", 80.0),
    ])
    toolhead = _FakeToolheadWithShapers(is_obj)
    # corner_deviation is loose enough that R_tol is large; R_mid caps
    # at min(L)·cot(45°) = 50, so R_tol binds. We still expect R ≈ 0.5mm
    # if we set corner_deviation to produce that.
    # R_tol = corner_deviation · cos(45°)/(1-cos(45°)) = corner_dev · 2.414
    # Solving corner_deviation = 0.5/2.414 ≈ 0.207 mm:
    corner_dev = 0.5 / (math.sqrt(2)/2 / (1 - math.sqrt(2)/2))
    result = blendmath.blend_from_moves(
        prev_move=prev,
        next_move=nxt,
        corner_deviation=corner_dev,
        toolhead=toolhead,
    )
    assert result is not None
    assert result.R == pytest.approx(0.5, rel=1e-6)
    # Final v_cap ~ 99.8 mm/s per spec sanity section (Y rotation-jerk binds).
    assert result.v_cap == pytest.approx(99.8, rel=0.05)


def test_blend_from_moves_without_toolhead_preserves_old_behavior():
    # Pass j_eff directly, no toolhead: identical to pre-change behavior.
    prev = _FakeMove(
        axes_r=[1.0, 0.0, 0.0, 0.0], move_d=50.0,
        accel=50000.0, max_cruise_v2=1e6,
    )
    nxt = _FakeMove(
        axes_r=[0.0, 1.0, 0.0, 0.0], move_d=50.0,
        accel=50000.0, max_cruise_v2=1e6,
    )
    j_eff = 1e8
    corner_dev = 0.02
    adapter_result = blendmath.blend_from_moves(
        prev_move=prev, next_move=nxt,
        corner_deviation=corner_dev, j_eff=j_eff,
    )
    core_result = blendmath.blend_geometry(
        prev_dir=(1.0, 0.0, 0.0), next_dir=(0.0, 1.0, 0.0),
        L_prev=50.0, L_next=50.0,
        corner_deviation=corner_dev, a_max=50000.0, j_eff=j_eff,
    )
    assert adapter_result.R == pytest.approx(core_result.R, rel=1e-12)
    assert adapter_result.v_cap == pytest.approx(core_result.v_cap, rel=1e-12)


def test_blend_from_moves_collinear_with_toolhead_returns_none():
    prev = _FakeMove(
        axes_r=[1.0, 0.0, 0.0, 0.0], move_d=10.0,
        accel=50000.0, max_cruise_v2=1e6,
    )
    nxt = _FakeMove(
        axes_r=[1.0, 0.0, 0.0, 0.0], move_d=10.0,
        accel=50000.0, max_cruise_v2=1e6,
    )
    is_obj = _FakeInputShaper([
        _FakeAxisInputShaper("x", "zv", 150.0),
        _FakeAxisInputShaper("y", "zv", 80.0),
    ])
    toolhead = _FakeToolheadWithShapers(is_obj)
    result = blendmath.blend_from_moves(
        prev_move=prev, next_move=nxt,
        corner_deviation=0.02, toolhead=toolhead,
    )
    assert result is None


def test_blend_from_moves_u_turn_with_toolhead_returns_zero_arc():
    prev = _FakeMove(
        axes_r=[1.0, 0.0, 0.0, 0.0], move_d=10.0,
        accel=50000.0, max_cruise_v2=1e6,
    )
    nxt = _FakeMove(
        axes_r=[-1.0, 0.0, 0.0, 0.0], move_d=10.0,
        accel=50000.0, max_cruise_v2=1e6,
    )
    is_obj = _FakeInputShaper([
        _FakeAxisInputShaper("x", "zv", 150.0),
    ])
    toolhead = _FakeToolheadWithShapers(is_obj)
    result = blendmath.blend_from_moves(
        prev_move=prev, next_move=nxt,
        corner_deviation=0.02, toolhead=toolhead,
    )
    assert result is not None
    assert result.R == 0.0
    assert result.v_cap == 0.0


def test_blend_from_moves_j_eff_and_toolhead_mutually_exclusive():
    # Passing both j_eff and toolhead is ambiguous (toolhead derives j_eff
    # internally; explicit j_eff would be silently ignored). Raise instead.
    prev = _FakeMove(
        axes_r=[1.0, 0.0, 0.0, 0.0], move_d=10.0,
        accel=50000.0, max_cruise_v2=1e6,
    )
    nxt = _FakeMove(
        axes_r=[0.0, 1.0, 0.0, 0.0], move_d=10.0,
        accel=50000.0, max_cruise_v2=1e6,
    )
    is_obj = _FakeInputShaper([
        _FakeAxisInputShaper("x", "zv", 150.0),
    ])
    toolhead = _FakeToolheadWithShapers(is_obj)
    with pytest.raises(ValueError, match="mutually exclusive"):
        blendmath.blend_from_moves(
            prev_move=prev, next_move=nxt,
            corner_deviation=0.02,
            j_eff=1e8,
            toolhead=toolhead,
        )
