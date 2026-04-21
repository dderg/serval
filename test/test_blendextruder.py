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


def test_cap_move_linear_a_cap_closed_form():
    """a_E_cap = a_E_max / (1 + PA * K_h); a_cap = a_E_cap / k."""
    k = 0.04
    move = _FakeMove(k=k, max_cruise_v=300.0)
    pa = 0.04
    snap = blendextruder.PAModelSnapshot(kind="linear", params=(pa,))
    limits = blendshape.ExtruderLimits(a_E_max=5000.0, v_E_max=15.9, smooth_time=0.04)
    K_h = (15.0 / 8.0) / 0.04  # = 46.875
    expected_a_E_cap = 5000.0 / (1.0 + pa * K_h)
    _, a_cap = blendextruder.cap_move(move, snap, limits)
    assert a_cap == pytest.approx(expected_a_E_cap / k, rel=1e-9)


def test_cap_move_linear_v_cap_bounded_by_rpm_term():
    """When (PA * a_E_cap) is small, v_cap ~= v_E_max / k."""
    k = 0.04
    move = _FakeMove(k=k, max_cruise_v=300.0)
    pa = 0.001  # tiny PA
    snap = blendextruder.PAModelSnapshot(kind="linear", params=(pa,))
    limits = blendshape.ExtruderLimits(a_E_max=5000.0, v_E_max=15.9, smooth_time=0.04)
    v_cap, _ = blendextruder.cap_move(move, snap, limits)
    assert v_cap <= 15.9 / k + 1e-6
    assert v_cap > 0.0


def test_cap_move_linear_pa_zero_cap_is_simple_division():
    """PA=0 => cap degenerates to (v_E_max/k, a_E_max/k)."""
    k = 0.05
    move = _FakeMove(k=k, max_cruise_v=300.0)
    snap = blendextruder.PAModelSnapshot(kind="linear", params=(0.0,))
    limits = blendshape.ExtruderLimits(a_E_max=6000.0, v_E_max=20.0, smooth_time=0.04)
    v_cap, a_cap = blendextruder.cap_move(move, snap, limits)
    assert v_cap == pytest.approx(20.0 / k, rel=1e-9)
    assert a_cap == pytest.approx(6000.0 / k, rel=1e-9)


def test_cap_move_linear_high_pa_tight_cap():
    """PA = 0.08 at smooth_time=0.04: 1 + 0.08*46.875 = 4.75
    => a_E_cap = a_E_max / 4.75 (79% reduction)."""
    k = 0.04
    move = _FakeMove(k=k, max_cruise_v=300.0)
    snap = blendextruder.PAModelSnapshot(kind="linear", params=(0.08,))
    limits = blendshape.ExtruderLimits(a_E_max=5000.0, v_E_max=15.9, smooth_time=0.04)
    _, a_cap = blendextruder.cap_move(move, snap, limits)
    expected = (5000.0 / 4.75) / k
    assert a_cap == pytest.approx(expected, rel=1e-6)


def test_bisection_trivial_linear_degenerate_matches_closed_form():
    """Linear PA (f' constant): bisection result matches closed form."""
    pa = 0.001  # small PA so result is feasible
    k = 0.04
    a_E_cap = 1000.0
    v_E_max = 15.9
    snap = blendextruder.PAModelSnapshot(kind="linear", params=(pa,))
    v_from_bisection = blendextruder._solve_velocity_cap_bisection(
        snap, k, a_E_cap, v_E_max
    )
    expected = (v_E_max - pa * a_E_cap) / k
    assert v_from_bisection == pytest.approx(expected, abs=1e-5)


def test_bisection_tanh_monotone_finds_valid_v():
    """At tanh snapshot: find v such that k*v + f'(k*v)*a_E_cap = v_E_max."""
    snap = blendextruder.PAModelSnapshot(
        kind="tanh", params=(0.0, 0.04, 100.0)
    )
    k = 0.04
    a_E_cap = 1000.0
    v_E_max = 15.9
    v = blendextruder._solve_velocity_cap_bisection(snap, k, a_E_cap, v_E_max)
    # Verify the result satisfies the constraint within tolerance.
    stepper_v = k * v + blendextruder._f_prime(snap, k * v) * a_E_cap
    assert stepper_v == pytest.approx(v_E_max, abs=1e-3)


def test_bisection_clamps_at_rpm_bound():
    """When a_E_cap=0, bisection should yield v_E_max / k exactly."""
    snap = blendextruder.PAModelSnapshot(kind="tanh", params=(0.0, 0.04, 100.0))
    k = 0.04
    v = blendextruder._solve_velocity_cap_bisection(snap, k, 0.0, 15.9)
    assert v == pytest.approx(15.9 / k, rel=1e-6)
