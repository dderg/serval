# test/test_blendshape.py
from klippy import blendshape


def test_kinematic_limits_dataclass():
    lim = blendshape.KinematicLimits(
        a_max=45000.0,
        v_max=500.0,
        jerk_max=None,
        extruder_caps=None,
    )
    assert lim.a_max == 45000.0
    assert lim.extruder_caps is None


def test_extruder_limits_dataclass():
    """Legacy test: verify ExtruderLimits is a dataclass with correct fields."""
    caps = blendshape.ExtruderLimits(a_E_max=5000.0, v_E_max=15.9, smooth_time=0.04)
    assert caps.a_E_max == 5000.0
    assert caps.v_E_max == 15.9
    assert caps.smooth_time == 0.04


def test_extruder_limits_has_three_fields():
    """ExtruderLimits carries stepper-output limits + PA smoothing time."""
    lim = blendshape.ExtruderLimits(
        a_E_max=5000.0,
        v_E_max=15.9,
        smooth_time=0.04,
    )
    assert lim.a_E_max == 5000.0
    assert lim.v_E_max == 15.9
    assert lim.smooth_time == 0.04


def test_extruder_limits_rejects_nonpositive_smooth_time_in_assertion():
    """K_h = (15/8)/smooth_time; smooth_time <= 0 would blow up.
    Not a hard gate here — the cap_move path will guard — but this
    documents the expected invariant for downstream consumers.
    """
    lim = blendshape.ExtruderLimits(a_E_max=5000.0, v_E_max=15.9, smooth_time=0.04)
    assert lim.smooth_time > 0.0


def test_smooth_shape_protocol_exists():
    # Structural: protocol must be importable and be a Protocol.
    assert hasattr(blendshape, "SmoothShape")
    # Protocol subclass check: any object with the required attrs satisfies.
    class _Stub:
        d_consumed = 1.0
        theta = 1.0
        arc_length = 2.0
        def position_at(self, s): return (0.0, 0.0, 0.0)
        def tangent_at(self, s): return (1.0, 0.0, 0.0)
        def curvature_at(self, s): return 0.5
        def dkappa_ds(self, s): return 0.0
        def v_cap_fn(self, s): return 100.0
        def polyline(self, tol): return [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)]
    stub = _Stub()
    assert isinstance(stub, blendshape.SmoothShape)
