import pytest

from klippy.extras import homing as homing_mod
from klippy.extras import servo_axis


class FakeErrConfig:
    error = RuntimeError


def test_infer_positive_dir_at_min_is_negative():
    cfg = FakeErrConfig()
    assert servo_axis.infer_positive_dir(cfg, "x", -6.0, -6.0, 235.0) is False


def test_infer_positive_dir_at_max_is_positive():
    cfg = FakeErrConfig()
    assert servo_axis.infer_positive_dir(cfg, "x", 235.0, -6.0, 235.0) is True


def test_infer_positive_dir_mid_range_is_config_error():
    cfg = FakeErrConfig()
    with pytest.raises(RuntimeError, match="position_endstop"):
        servo_axis.infer_positive_dir(cfg, "x", 100.0, -6.0, 235.0)


class FakeRailConfig:
    error = RuntimeError
    _UNSET = object()

    def __init__(self, name, options):
        self._name = name
        self._options = dict(options)

    def get_printer(self):
        return None

    def get_name(self):
        return self._name

    def get(self, option, default=_UNSET):
        return self._lookup(option, default)

    def getfloat(
        self, option, default=_UNSET, above=None, minval=None, maxval=None
    ):
        return self._lookup(option, default)

    def getint(self, option, default=_UNSET, minval=None, maxval=None):
        return self._lookup(option, default)

    def getboolean(self, option, default=_UNSET):
        return self._lookup(option, default)

    def _lookup(self, option, default):
        if option in self._options:
            return self._options[option]
        if default is FakeRailConfig._UNSET:
            raise RuntimeError("missing required option %r" % (option,))
        return default


AXIS_Z_OPTIONS = {
    "position_min": -6.0,
    "position_max": 235.0,
    "endstop_pin": "ec_z:endstop",
    "position_endstop": -6.0,
}

MOTOR_Z_OPTIONS = {
    "protocol": "ethercat",
    "node": "z_drive",
    "rotation_distance": 40.0,
    "encoder_counts_per_rev": 131072,
}

AXIS_KEYS = frozenset(
    (
        "position_min",
        "position_max",
        "endstop_pin",
        "position_endstop",
        "homing_speed",
        "homing_retract_dist",
        "homing_retract_speed",
        "min_home_dist",
    )
)


def make_servo_rail(extra=(), drop=()):
    axis_options = dict(AXIS_Z_OPTIONS)
    motor_options = dict(MOTOR_Z_OPTIONS)
    for key, value in dict(extra).items():
        if key in AXIS_KEYS:
            axis_options[key] = value
        else:
            motor_options[key] = value
    for key in drop:
        axis_options.pop(key, None)
        motor_options.pop(key, None)
    return servo_axis.ServoRail(
        FakeRailConfig("axis z", axis_options),
        FakeRailConfig("motor z_drive", motor_options),
    )


def test_get_homing_info_reflects_homing_config():
    rail = make_servo_rail(extra={"homing_speed": 50.0})
    hi = rail.get_homing_info()
    assert hi.speed == 50.0
    assert hi.position_endstop == -6.0
    assert hi.positive_dir is False


def test_homing_info_reflects_retract_config():
    rail = make_servo_rail(
        extra={"homing_retract_dist": 3.0, "homing_retract_speed": 10.0}
    )
    hi = rail.get_homing_info()
    assert hi.retract_dist == 3.0
    assert hi.retract_speed == 10.0


def test_retract_defaults_match_stepper_rail():
    rail = make_servo_rail(extra={"homing_speed": 50.0})
    hi = rail.get_homing_info()
    assert hi.retract_dist == 5.0
    assert hi.retract_speed == 50.0


def test_no_endstop_pin_means_zero_retract():
    rail = make_servo_rail(drop=("endstop_pin", "position_endstop"))
    hi = rail.get_homing_info()
    assert hi.retract_dist == 0.0
    assert rail.second_homing_speed == 0.0


def test_min_home_dist_parsed_from_axis_config():
    rail = make_servo_rail(extra={"min_home_dist": 12.0})
    assert rail.get_homing_info().min_home_dist == 12.0


def test_min_home_dist_defaults_to_retract_dist():
    rail = make_servo_rail(extra={"homing_retract_dist": 3.0})
    assert rail.get_homing_info().min_home_dist == 3.0


class FakeSectionsConfig:
    def __init__(self, sections):
        self._sections = sections

    def has_section(self, name):
        return name in self._sections


def test_endstop_section_finds_axis():
    cfg = FakeSectionsConfig({"axis x"})
    assert homing_mod._endstop_section(cfg, "x") == "axis x"


def test_endstop_section_none_when_axis_absent():
    cfg = FakeSectionsConfig({"axis y"})
    assert homing_mod._endstop_section(cfg, "x") is None


class FakeStepperEnable:
    def __init__(self):
        self.calls = []

    def motor_debug_enable(self, name, enable):
        self.calls.append((name, enable))


class FakeStepper:
    def __init__(self, name):
        self._name = name

    def get_name(self):
        return self._name


class FakeRail:
    def __init__(self, steppers, name):
        self._steppers = steppers
        self._name = name

    def get_steppers(self):
        return self._steppers

    def get_name(self, short=False):
        return self._name


def test_homing_motor_names_lists_each_stepper():
    rail = FakeRail(
        [FakeStepper("stepper_x"), FakeStepper("stepper_x1")], "stepper_x"
    )
    assert homing_mod._homing_motor_names(rail) == ["stepper_x", "stepper_x1"]


def test_homing_motor_names_uses_servo_rail_name_when_no_steppers():
    rail = FakeRail([], "servo_x")
    assert homing_mod._homing_motor_names(rail) == ["servo_x"]


def make_homing_servo_rail():
    rail = servo_axis.ServoRail.__new__(servo_axis.ServoRail)
    rail.axis = "x"
    rail.name = "servo_x"
    rail.rotation_distance = 40.0
    rail.encoder_counts_per_rev = 131072
    rail.homing_following_error = 2.5
    rail.homing_max_torque = 50.0
    rail.following_error = None
    rail.max_torque = None
    return rail


def test_homing_drive_limits_convert_units():
    rail = make_homing_servo_rail()
    counts, tenth_pct = rail.get_homing_drive_limits()
    assert counts == 8192
    assert tenth_pct == 500


def test_session_drive_limits_none_when_unconfigured():
    rail = make_homing_servo_rail()
    assert rail.get_session_drive_limits() == (None, None)


def test_session_drive_limits_convert_units():
    rail = make_homing_servo_rail()
    rail.following_error = 5.0
    rail.max_torque = 120.0
    assert rail.get_session_drive_limits() == (16384, 1200)


class FakeLimitsEngine:
    def __init__(self):
        self.calls = []

    def set_drive_limits(self, handle, counts, tenth_pct):
        self.calls.append(("set", handle, counts, tenth_pct))

    def restore_drive_limits(self, handle):
        self.calls.append(("restore", handle))

    def finalize_homed_axis(self, handle, axis, pos_mm):
        self.calls.append(("finalize", handle, axis, pos_mm))


def test_homing_limits_guard_sets_and_restores():
    engine = FakeLimitsEngine()
    with homing_mod._servo_drive_limits(engine, 7, (8192, 500)):
        assert engine.calls == [("set", 7, 8192, 500)]
    assert engine.calls == [("set", 7, 8192, 500), ("restore", 7)]


def test_homing_limits_guard_restores_on_error():
    engine = FakeLimitsEngine()
    try:
        with homing_mod._servo_drive_limits(engine, 7, (8192, 500)):
            raise RuntimeError("trip move failed")
    except RuntimeError:
        pass
    assert engine.calls[-1] == ("restore", 7)


def test_homing_limits_guard_noop_without_limits():
    engine = FakeLimitsEngine()
    with homing_mod._servo_drive_limits(engine, None, None):
        pass
    assert engine.calls == []


class FailingRestoreEngine(FakeLimitsEngine):
    def restore_drive_limits(self, handle):
        raise OSError("endpoint gone")


def test_homing_limits_guard_restore_failure_raises_on_success_path():
    engine = FailingRestoreEngine()
    with pytest.raises(OSError, match="endpoint gone"):
        with homing_mod._servo_drive_limits(engine, 7, (8192, 500)):
            pass


def test_homing_limits_guard_restore_failure_does_not_mask_body_error():
    engine = FailingRestoreEngine()
    with pytest.raises(RuntimeError, match="trip move failed"):
        with homing_mod._servo_drive_limits(engine, 7, (8192, 500)):
            raise RuntimeError("trip move failed")


class FakeGcmd:
    error = RuntimeError


class FakeFaultEngine:
    def __init__(self, fault):
        self._fault = fault
        self.taken = []

    def take_drive_fault(self, handle):
        self.taken.append(handle)
        return self._fault


def test_post_trip_fault_check_raises_on_fault():
    engine = FakeFaultEngine(0x8611)
    with pytest.raises(RuntimeError, match="drive fault 0x8611"):
        homing_mod._check_servo_drive_fault(FakeGcmd(), engine, 0, 7)
    assert engine.taken == [7]


def test_post_trip_fault_check_passes_without_fault():
    engine = FakeFaultEngine(None)
    homing_mod._check_servo_drive_fault(FakeGcmd(), engine, 0, 7)
    assert engine.taken == [7]


def test_post_trip_fault_check_skips_non_servo():
    engine = FakeFaultEngine(0x8611)
    homing_mod._check_servo_drive_fault(FakeGcmd(), engine, 0, None)
    assert engine.taken == []


class FakeServoEngine(FakeLimitsEngine):
    def __init__(self, fault=None):
        super().__init__()
        self._fault = fault

    def take_drive_fault(self, handle):
        self.calls.append(("take_fault", handle))
        return self._fault


def run_guarded_trip(engine, se, servo_handle, servo_limits, trip):
    rail = FakeRail([], "servo_x")
    return homing_mod._run_servo_guarded_trip(
        FakeGcmd(), engine, 0, se, rail, servo_handle, servo_limits, trip
    )


class FakeHomingInfo:
    def __init__(self, positive_dir, retract_dist, retract_speed):
        self.positive_dir = positive_dir
        self.retract_dist = retract_dist
        self.retract_speed = retract_speed
        self.position_endstop = -6.0
        self.speed = 50.0
        self.min_home_dist = 0.0


class FakeHomingRail:
    def __init__(self, hi, pos_min, pos_max):
        self._hi = hi
        self._range = (pos_min, pos_max)

    def get_homing_info(self):
        return self._hi

    def get_range(self):
        return self._range

    def get_tmc_current_helpers(self):
        return []

    def get_steppers(self):
        return [FakeStepper("stepper_z")]

    def get_name(self, short=False):
        return "stepper_z"


class FakeKin:
    def __init__(self, rail):
        self._rail = rail

    def _axis_rails(self):
        return {2: self._rail}

    def active_rails(self, *deltas):
        return [self._rail]


class FakeHomingToolhead:
    def __init__(self):
        self._pos = [0.0, 0.0, 0.0, 0.0]
        self.moves = []

    def get_position(self):
        return list(self._pos)

    def set_position(self, newpos, homing_axes=()):
        self._pos = list(newpos)

    def move(self, newpos, speed):
        self.moves.append((list(newpos), speed))
        self._pos = list(newpos)

    def wait_moves(self):
        pass

    def get_last_move_time(self):
        return 0.0


class FakeHomingStepperEnable:
    def motor_enable_group(self, names):
        pass


class FakeHomingPrinter:
    def __init__(self, stepper_enable):
        self._stepper_enable = stepper_enable

    def lookup_object(self, name):
        assert name == "stepper_enable"
        return self._stepper_enable


def run_home_axis(overshoot, retract_dist, positive_dir):
    toolhead = FakeHomingToolhead()
    hi = FakeHomingInfo(positive_dir, retract_dist, retract_speed=10.0)
    rail = FakeHomingRail(hi, pos_min=-6.0, pos_max=235.0)
    kin = FakeKin(rail)
    homer = homing_mod.Homing.__new__(homing_mod.Homing)
    homer.printer = FakeHomingPrinter(FakeHomingStepperEnable())

    direction = 1.0 if positive_dir else -1.0
    trigger_height = hi.position_endstop
    trip_pos = [0.0, 0.0, 100.0]
    final_pos = [0.0, 0.0, 100.0 + direction * overshoot]

    homer.trip_move = lambda *a, **k: None
    homer._set_homing_current = lambda *a, **k: None

    def fake_guarded_trip(*args, **kwargs):
        return trip_pos, final_pos

    class FakeDangerOptions:
        homing_elapsed_distance_tolerance = 0.5

    orig_guarded = homing_mod._run_servo_guarded_trip
    orig_fault = homing_mod._check_servo_drive_fault
    orig_danger = homing_mod.get_danger_options
    homing_mod._run_servo_guarded_trip = fake_guarded_trip
    homing_mod._check_servo_drive_fault = lambda *a, **k: None
    homing_mod.get_danger_options = lambda: FakeDangerOptions()
    try:
        homer._home_axis(
            FakeGcmd(),
            toolhead,
            engine=None,
            kin=kin,
            axis=2,
            entry={"trigger_height": trigger_height, "provider": None},
        )
    finally:
        homing_mod._run_servo_guarded_trip = orig_guarded
        homing_mod._check_servo_drive_fault = orig_fault
        homing_mod.get_danger_options = orig_danger

    return toolhead, trigger_height


def test_retract_compensates_for_overshoot():
    overshoot = 0.7
    retract_dist = 5.0
    toolhead, trigger_height = run_home_axis(
        overshoot, retract_dist, positive_dir=False
    )
    target, speed = toolhead.moves[-1]
    assert target[2] == pytest.approx(trigger_height + retract_dist)
    assert speed == 10.0


def test_retract_lands_at_trigger_minus_dist_positive_dir():
    overshoot = 0.7
    retract_dist = 5.0
    toolhead, trigger_height = run_home_axis(
        overshoot, retract_dist, positive_dir=True
    )
    target, _ = toolhead.moves[-1]
    assert target[2] == pytest.approx(trigger_height - retract_dist)


def test_retract_with_zero_overshoot_unchanged():
    retract_dist = 5.0
    toolhead, trigger_height = run_home_axis(
        0.0, retract_dist, positive_dir=False
    )
    target, _ = toolhead.moves[-1]
    assert target[2] == pytest.approx(trigger_height + retract_dist)


def test_guarded_trip_failure_disables_servo_motor_and_reraises():
    engine = FakeServoEngine()
    se = FakeStepperEnable()

    def trip():
        raise RuntimeError("trip move failed")

    with pytest.raises(RuntimeError, match="trip move failed"):
        run_guarded_trip(engine, se, 7, (8192, 500), trip)
    assert se.calls == [("servo_x", False)]


def test_guarded_trip_latched_fault_disables_servo_motor():
    engine = FakeServoEngine(fault=0x8611)
    se = FakeStepperEnable()
    with pytest.raises(RuntimeError, match="drive fault 0x8611"):
        run_guarded_trip(engine, se, 7, (8192, 500), lambda: (1.0, 2.0))
    assert se.calls == [("servo_x", False)]


def test_guarded_trip_success_keeps_servo_motor_enabled():
    engine = FakeServoEngine()
    se = FakeStepperEnable()
    result = run_guarded_trip(engine, se, 7, (8192, 500), lambda: (1.0, 2.0))
    assert result == (1.0, 2.0)
    assert se.calls == []


def test_guarded_trip_stepper_rail_failure_skips_servo_disable():
    engine = FakeServoEngine()
    se = FakeStepperEnable()

    def trip():
        raise RuntimeError("trip move failed")

    with pytest.raises(RuntimeError, match="trip move failed"):
        run_guarded_trip(engine, se, None, None, trip)
    assert se.calls == []
