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
