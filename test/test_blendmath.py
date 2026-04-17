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
