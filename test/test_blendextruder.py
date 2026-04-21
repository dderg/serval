import math
import pytest

from klippy import blendextruder, blendshape


class _FakeMove:
    """Minimal Move stub exposing the attrs cap_move reads."""
    def __init__(self, k, max_cruise_v):
        # axes_r = (x_ratio, y_ratio, z_ratio, e_ratio); cap_move reads axes_r[3] = k.
        self.axes_r = (1.0, 0.0, 0.0, k)
        self.max_cruise_v2 = max_cruise_v ** 2
        self.max_cruise_v = max_cruise_v


def _default_limits():
    return blendshape.ExtruderLimits(
        a_E_max=5000.0, v_E_max=15.9, smooth_time=0.04,
    )


def _default_linear_snap(pa=0.04):
    return blendextruder.PAModelSnapshot(kind="linear", params=(pa,))


def test_cap_move_travel_returns_inf():
    """k=0 (pure XY travel, no extrusion): cap is infinite."""
    move = _FakeMove(k=0.0, max_cruise_v=300.0)
    snap = _default_linear_snap()
    limits = _default_limits()
    v_cap, a_cap = blendextruder.cap_move(move, snap, limits)
    assert math.isinf(v_cap)
    assert math.isinf(a_cap)


def test_cap_move_none_pa_model_returns_inf():
    """No PA model (extruder not configured) — cap is inactive."""
    move = _FakeMove(k=0.04, max_cruise_v=300.0)
    limits = _default_limits()
    v_cap, a_cap = blendextruder.cap_move(move, None, limits)
    assert math.isinf(v_cap)
    assert math.isinf(a_cap)


def test_cap_move_none_limits_returns_inf():
    """Limits not configured — cap is inactive."""
    move = _FakeMove(k=0.04, max_cruise_v=300.0)
    snap = _default_linear_snap()
    v_cap, a_cap = blendextruder.cap_move(move, snap, None)
    assert math.isinf(v_cap)
    assert math.isinf(a_cap)


def test_cap_move_zero_a_max_returns_zero_accel():
    """Degenerate: a_E_max=0 pins a_cap to 0 (cannot accelerate)."""
    move = _FakeMove(k=0.04, max_cruise_v=300.0)
    snap = _default_linear_snap()
    limits = blendshape.ExtruderLimits(a_E_max=0.0, v_E_max=15.9, smooth_time=0.04)
    _, a_cap = blendextruder.cap_move(move, snap, limits)
    assert a_cap == 0.0


def test_pa_model_snapshot_is_immutable():
    """Snapshot carries the model state at construction time."""
    snap = blendextruder.PAModelSnapshot(kind="linear", params=(0.04,))
    assert snap.kind == "linear"
    assert snap.params == (0.04,)
    # Frozen dataclass or namedtuple — mutation should fail.
    with pytest.raises((AttributeError, TypeError, Exception)):
        snap.kind = "tanh"
