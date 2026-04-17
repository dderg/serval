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
