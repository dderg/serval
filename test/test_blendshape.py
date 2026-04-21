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
    caps = blendshape.ExtruderLimits(accel_max=1000.0, rpm_max=300.0)
    assert caps.accel_max == 1000.0
    assert caps.rpm_max == 300.0


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
