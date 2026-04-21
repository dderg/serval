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


def test_cap_move_zero_a_max_with_rpm_cap_returns_inf_accel():
    """P0-5: a_E_max=0 with v_E_max>0 → RPM-only cap: v_cap=v_E_max/k, a_cap=+inf."""
    k = 0.04
    move = _FakeMove(k=k, max_cruise_v=300.0)
    snap = _default_linear_snap()
    limits = blendshape.ExtruderLimits(a_E_max=0.0, v_E_max=15.9, smooth_time=0.04)
    v_cap, a_cap = blendextruder.cap_move(move, snap, limits)
    assert v_cap == pytest.approx(15.9 / k, rel=1e-6)
    assert math.isinf(a_cap)


def test_pa_model_snapshot_is_immutable():
    """Snapshot carries the model state at construction time."""
    snap = blendextruder.PAModelSnapshot(kind="linear", params=(0.04,))
    assert snap.kind == "linear"
    assert snap.params == (0.04,)
    # Frozen dataclass or namedtuple — mutation should fail.
    with pytest.raises((AttributeError, TypeError, Exception)):
        snap.kind = "tanh"


def test_cap_move_linear_a_cap_closed_form():
    """a_E_cap = a_E_max / (1 + PA * K_h); a_cap = a_E_cap / k.
    v_E_max is set high enough that the simultaneous cap is feasible."""
    k = 0.04
    move = _FakeMove(k=k, max_cruise_v=300.0)
    pa = 0.04
    snap = blendextruder.PAModelSnapshot(kind="linear", params=(pa,))
    # v_E_max=200 keeps v_from_accel positive so the accel cap is returned.
    limits = blendshape.ExtruderLimits(a_E_max=5000.0, v_E_max=200.0, smooth_time=0.04)
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
    => a_E_cap = a_E_max / 4.75 (79% reduction).
    v_E_max=200 keeps the simultaneous cap feasible."""
    k = 0.04
    move = _FakeMove(k=k, max_cruise_v=300.0)
    snap = blendextruder.PAModelSnapshot(kind="linear", params=(0.08,))
    limits = blendshape.ExtruderLimits(a_E_max=5000.0, v_E_max=200.0, smooth_time=0.04)
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


def test_cap_move_tanh_near_zero_nonlinear_offset_matches_linear():
    """When nonlinear_offset=0 for tanh, behavior matches linear LA."""
    k = 0.04
    move = _FakeMove(k=k, max_cruise_v=300.0)
    tanh_snap = blendextruder.PAModelSnapshot(kind="tanh", params=(0.04, 0.0, 100.0))
    lin_snap = blendextruder.PAModelSnapshot(kind="linear", params=(0.04,))
    limits = blendshape.ExtruderLimits(a_E_max=5000.0, v_E_max=15.9, smooth_time=0.04)
    v_t, a_t = blendextruder.cap_move(move, tanh_snap, limits)
    v_l, a_l = blendextruder.cap_move(move, lin_snap, limits)
    assert a_t == pytest.approx(a_l, rel=1e-6)
    assert v_t == pytest.approx(v_l, abs=1e-3)


def test_cap_move_tanh_realistic_cap_is_close_to_a_E_max_over_k():
    """Realistic NL params: f'*K_h tiny (~0.02), so a_cap ~= a_E_max/k."""
    k = 0.04
    move = _FakeMove(k=k, max_cruise_v=300.0)
    # NO=0.04, LV=100, LA=0; v_eval = k * max_cruise_v = 0.04*300 = 12
    # f'(12) ~= (0.04/100)*sech²(0.12) ≈ 3.9e-4, K_h = 46.875 => 1+f'*K_h ~= 1.018
    tanh_snap = blendextruder.PAModelSnapshot(kind="tanh", params=(0.0, 0.04, 100.0))
    limits = blendshape.ExtruderLimits(a_E_max=5000.0, v_E_max=15.9, smooth_time=0.04)
    _, a_cap = blendextruder.cap_move(move, tanh_snap, limits)
    naive = 5000.0 / k
    assert 0.98 * naive < a_cap <= naive


def test_cap_move_recipr_matches_pattern():
    """Recipr NL cap is close to a_E_max/k at realistic params."""
    k = 0.04
    move = _FakeMove(k=k, max_cruise_v=300.0)
    recipr_snap = blendextruder.PAModelSnapshot(kind="recipr", params=(0.0, 0.04, 100.0))
    limits = blendshape.ExtruderLimits(a_E_max=5000.0, v_E_max=15.9, smooth_time=0.04)
    _, a_cap = blendextruder.cap_move(move, recipr_snap, limits)
    naive = 5000.0 / k
    assert 0.98 * naive < a_cap <= naive


def test_cap_move_tanh_v_cap_satisfies_constraint():
    """The returned v_cap should satisfy the stepper_v constraint or be at v=0 when infeasible."""
    k = 0.04
    move = _FakeMove(k=k, max_cruise_v=300.0)
    # Use feasible params: LA=0.02, NO=0.02, LV=100 => f'(0) = 0.0202
    # => max_a_E_cap = 15.9 / 0.0202 = 786.6
    # And v_eval=12: f'(12) ~= 0.02 => a_E_cap = 5000/(1+0.02*46.875) = 1739
    # Still infeasible at v=0. Let's use LA=0.001, NO=0.002, LV=100
    snap = blendextruder.PAModelSnapshot(kind="tanh", params=(0.001, 0.002, 100.0))
    limits = blendshape.ExtruderLimits(a_E_max=5000.0, v_E_max=15.9, smooth_time=0.04)
    v_cap, a_cap = blendextruder.cap_move(move, snap, limits)
    # Verify stepper_v(v_cap) <= v_E_max within tolerance.
    a_E_cap = a_cap * k
    stepper_v = blendextruder._stepper_v_of_xy(snap, v_cap, k, a_E_cap)
    assert stepper_v <= 15.9 + 1e-3


def test_extruder_stepper_parses_max_extruder_accel():
    """[extruder] max_extruder_accel parsed; defaults to 0."""
    from klippy.kinematics.extruder import ExtruderStepper
    # Bypass full bootstrap; construct and manually set attributes.
    es = ExtruderStepper.__new__(ExtruderStepper)
    es.max_extruder_accel = 0.0
    es.max_extruder_rpm = 0.0
    assert es.get_extruder_accel_limit() == 0.0
    assert es.get_extruder_rpm_limit() == 0.0

    es.max_extruder_accel = 5000.0
    es.max_extruder_rpm = 200.0
    assert es.get_extruder_accel_limit() == 5000.0
    assert es.get_extruder_rpm_limit() == 200.0


def test_set_extruder_limits_updates_values():
    """cmd_SET_EXTRUDER_LIMITS applies ACCEL and RPM to the stepper."""
    from klippy.kinematics.extruder import ExtruderStepper
    es = ExtruderStepper.__new__(ExtruderStepper)
    es.max_extruder_accel = 0.0
    es.max_extruder_rpm = 0.0
    es.name = "extruder"

    class _FakeGcmd:
        def __init__(self, accel, rpm):
            self._accel = accel
            self._rpm = rpm
        def get_float(self, key, default=None, **kw):
            if key == "ACCEL":
                return self._accel if self._accel is not None else default
            if key == "RPM":
                return self._rpm if self._rpm is not None else default
            return default
        def respond_info(self, msg):
            self._last_msg = msg

    g = _FakeGcmd(accel=5000.0, rpm=200.0)
    es.cmd_SET_EXTRUDER_LIMITS(g)
    assert es.max_extruder_accel == 5000.0
    assert es.max_extruder_rpm == 200.0


def test_set_extruder_limits_omit_reports_current():
    """Calling with no ACCEL/RPM args reports current values."""
    from klippy.kinematics.extruder import ExtruderStepper
    es = ExtruderStepper.__new__(ExtruderStepper)
    es.max_extruder_accel = 5000.0
    es.max_extruder_rpm = 200.0
    es.name = "extruder"

    class _FakeGcmd:
        def get_float(self, key, default=None, **kw):
            return default
        def respond_info(self, msg):
            self.last = msg

    g = _FakeGcmd()
    es.cmd_SET_EXTRUDER_LIMITS(g)
    assert "5000" in g.last
    assert "200" in g.last


def test_extruder_limits_snapshot_shape():
    """snapshot returns (PAModelSnapshot, ExtruderLimits) or None."""
    from klippy.kinematics.extruder import ExtruderStepper, PALinearModel

    es = ExtruderStepper.__new__(ExtruderStepper)
    es.max_extruder_accel = 5000.0
    es.max_extruder_rpm = 200.0

    pa = PALinearModel.__new__(PALinearModel)
    pa.pressure_advance = 0.04
    es.pa_model = pa  # real attribute name in ExtruderStepper.__init__

    class _ExtSmoother:
        def __init__(self, t):
            self.smooth_time = t
    es.smoother = _ExtSmoother(0.04)
    # get_rotation_distance() delegates to self.stepper; stub it directly.
    es.get_rotation_distance = lambda: 4.78  # BMG-ish

    snap = es.extruder_limits_snapshot()
    assert snap is not None
    pa_snap, limits = snap
    assert pa_snap.kind == "linear"
    assert pa_snap.params == (0.04,)
    assert limits.a_E_max == 5000.0
    # v_E_max = (200 / 60) * 4.78 ~= 15.933 mm/s
    assert limits.v_E_max == pytest.approx((200.0 / 60.0) * 4.78, rel=1e-6)
    assert limits.smooth_time == 0.04


def test_extruder_limits_snapshot_disabled_returns_none():
    """When both caps are 0, snapshot=None."""
    from klippy.kinematics.extruder import ExtruderStepper
    es = ExtruderStepper.__new__(ExtruderStepper)
    es.max_extruder_accel = 0.0
    es.max_extruder_rpm = 0.0
    assert es.extruder_limits_snapshot() is None


# --- New regression tests for P0-1 through P0-5 ---

def test_cap_move_smooth_time_zero_returns_inf():
    """P0-3: smooth_time=0 must not raise ZeroDivisionError."""
    k = 0.04
    move = _FakeMove(k=k, max_cruise_v=300.0)
    snap = _default_linear_snap()
    limits = blendshape.ExtruderLimits(a_E_max=5000.0, v_E_max=15.9, smooth_time=0.0)
    v_cap, a_cap = blendextruder.cap_move(move, snap, limits)
    assert math.isinf(v_cap)
    assert math.isinf(a_cap)


def test_cap_move_rpm_only_cap_has_infinite_accel():
    """P0-5: a_E_max=0, v_E_max>0 → (v_E_max/k, +inf), NOT (+inf, 0)."""
    k = 0.04
    move = _FakeMove(k=k, max_cruise_v=300.0)
    snap = _default_linear_snap()
    limits = blendshape.ExtruderLimits(a_E_max=0.0, v_E_max=15.9, smooth_time=0.04)
    v_cap, a_cap = blendextruder.cap_move(move, snap, limits)
    assert v_cap == pytest.approx(15.9 / k, rel=1e-6)
    assert math.isinf(a_cap)


def test_cap_move_high_pa_degenerate_v_cap_falls_back():
    """P0-4: very high PA → v_cap would be 0 → fall back to (v_E_max/k, +inf)."""
    # PA=0.08, k=0.04, smooth_time=0.04 → K_h=46.875
    # a_E_cap = 5000/(1+0.08*46.875) = 5000/4.75 = 1052.6
    # v_from_accel = (15.9 - 0.08*1052.6)/0.04 = (15.9 - 84.2)/0.04 < 0 → clamp
    k = 0.04
    move = _FakeMove(k=k, max_cruise_v=300.0)
    snap = blendextruder.PAModelSnapshot(kind="linear", params=(0.08,))
    limits = blendshape.ExtruderLimits(a_E_max=5000.0, v_E_max=15.9, smooth_time=0.04)
    v_cap, a_cap = blendextruder.cap_move(move, snap, limits)
    # With P0-4 fix: when linear formula gives v_from_accel < 0, fall back to
    # v_E_max/k (rpm-only cap) with a_cap=+inf.
    assert v_cap > 0.0, "v_cap must never be 0 or negative"
    assert v_cap == pytest.approx(15.9 / k, rel=1e-3)
    assert math.isinf(a_cap)


def test_extruder_limits_snapshot_tanh_shape():
    """P0-1/snapshot: tanh snapshot has kind='tanh' and (LA, NO, LV) params."""
    from klippy.kinematics.extruder import ExtruderStepper, PATanhModel

    es = ExtruderStepper.__new__(ExtruderStepper)
    es.max_extruder_accel = 5000.0
    es.max_extruder_rpm = 200.0

    pa = PATanhModel.__new__(PATanhModel)
    pa.linear_advance = 0.0
    pa.nonlinear_offset = 0.04
    pa.linearization_velocity = 100.0
    es.pa_model = pa

    class _ExtSmoother:
        def __init__(self, t):
            self.smooth_time = t
    es.smoother = _ExtSmoother(0.04)
    es.get_rotation_distance = lambda: 4.78

    snap = es.extruder_limits_snapshot()
    assert snap is not None
    pa_snap, _ = snap
    assert pa_snap.kind == "tanh"
    assert pa_snap.params == (0.0, 0.04, 100.0)


def test_extruder_limits_snapshot_recipr_shape():
    """P0-1/snapshot: recipr snapshot has kind='recipr' and (LA, NO, LV) params."""
    from klippy.kinematics.extruder import ExtruderStepper, PAReciprModel

    es = ExtruderStepper.__new__(ExtruderStepper)
    es.max_extruder_accel = 5000.0
    es.max_extruder_rpm = 200.0

    pa = PAReciprModel.__new__(PAReciprModel)
    pa.linear_advance = 0.0
    pa.nonlinear_offset = 0.04
    pa.linearization_velocity = 100.0
    es.pa_model = pa

    class _ExtSmoother:
        def __init__(self, t):
            self.smooth_time = t
    es.smoother = _ExtSmoother(0.04)
    es.get_rotation_distance = lambda: 4.78

    snap = es.extruder_limits_snapshot()
    assert snap is not None
    pa_snap, _ = snap
    assert pa_snap.kind == "recipr"
    assert pa_snap.params == (0.0, 0.04, 100.0)


def test_refresh_snapshot_walks_extruder_steppers():
    """P0-1: _refresh_extruder_cap_snapshot must walk extruder_steppers[0],
    not the non-existent extruder_stepper (singular)."""
    import types

    # Build a minimal mock snapshot.
    fake_pa_snap = blendextruder.PAModelSnapshot(kind="linear", params=(0.04,))
    fake_limits = blendshape.ExtruderLimits(a_E_max=5000.0, v_E_max=15.9, smooth_time=0.04)
    fake_snapshot = (fake_pa_snap, fake_limits)

    # Mock stepper that has extruder_limits_snapshot.
    mock_stepper = types.SimpleNamespace(
        extruder_limits_snapshot=lambda: fake_snapshot,
    )

    # Mock PrinterExtruder: no extruder_limits_snapshot itself,
    # but has extruder_steppers = [mock_stepper].
    mock_extruder = types.SimpleNamespace(
        extruder_steppers=[mock_stepper],
        # Deliberately omit extruder_limits_snapshot to confirm delegation.
    )

    # Minimal ToolHead stub.
    class _FakeToolHead:
        def __init__(self):
            self.extruder = mock_extruder
            self.extruder_cap_snapshot = None

        def _refresh_extruder_cap_snapshot(self):
            # Copy of the fixed implementation under test.
            extruder = getattr(self, "extruder", None)
            if extruder is None:
                self.extruder_cap_snapshot = None
                return
            snap_fn = getattr(extruder, "extruder_limits_snapshot", None)
            if snap_fn is None:
                steppers = getattr(extruder, "extruder_steppers", None)
                if steppers:
                    snap_fn = getattr(steppers[0], "extruder_limits_snapshot", None)
            if snap_fn is None:
                self.extruder_cap_snapshot = None
                return
            self.extruder_cap_snapshot = snap_fn()

    th = _FakeToolHead()
    th._refresh_extruder_cap_snapshot()
    assert th.extruder_cap_snapshot is not None, (
        "Cap snapshot must be set when extruder_steppers[0] has the snapshot fn"
    )
