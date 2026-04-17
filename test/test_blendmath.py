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
    # R_mid = min(L_prev, L_next) * cot(theta/2) = 0.5 * 1.0 = 0.5 mm
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
    expected_R_mid = L_short * cos_half / sin_half  # = 0.5
    assert result.R == pytest.approx(expected_R_mid, rel=1e-9)
    # d_consumed should equal L_short (90 deg case: d = R).
    assert result.d_consumed == pytest.approx(L_short, rel=1e-9)


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
