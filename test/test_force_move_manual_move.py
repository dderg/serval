import pytest

from klippy.extras.force_move import ForceMove


class FakeEnableLine:
    def __init__(self, enabled):
        self._enabled = enabled

    def is_motor_enabled(self):
        return self._enabled


class FakeStepperEnable:
    def __init__(self, enabled):
        self._enable_line = FakeEnableLine(enabled)

    def lookup_enable(self, name):
        return self._enable_line


class FakeToolhead:
    def __init__(self, binding, max_accel):
        self._binding = binding
        self._max_accel = max_accel
        self.last_nudge = None

    def get_motor_binding(self, name):
        return self._binding

    def get_max_axis_accel(self, axis_idx):
        return self._max_accel

    def submit_nudge(self, mcu_id, axis_idx, motor_idx, dist, speed, accel):
        self.last_nudge = dict(
            mcu_id=mcu_id,
            axis_idx=axis_idx,
            motor_idx=motor_idx,
            dist=dist,
            speed=speed,
            accel=accel,
        )


class CommandError(Exception):
    pass


class FakePrinter:
    command_error = CommandError

    def __init__(self, toolhead, enabled):
        self._toolhead = toolhead
        self._stepper_enable = FakeStepperEnable(enabled)

    def lookup_object(self, name, default=None):
        mapping = {
            "toolhead": self._toolhead,
            "stepper_enable": self._stepper_enable,
        }
        return mapping.get(name, default)


def make_force_move(enabled=True, return_toolhead=False, max_axis_accel=3000.0):
    toolhead = FakeToolhead(binding=(0, 2, 1), max_accel=max_axis_accel)
    printer = FakePrinter(toolhead, enabled)
    fm = ForceMove.__new__(ForceMove)
    fm.printer = printer
    if return_toolhead:
        return fm, toolhead
    return fm


def test_manual_move_raises_when_motor_disabled():
    fm = make_force_move(enabled=False)
    with pytest.raises(CommandError):
        fm.manual_move("stepper_z1", 0.5, 5.0, 100.0)


def test_manual_move_forwards_to_submit_nudge_when_enabled():
    fm, toolhead = make_force_move(enabled=True, return_toolhead=True)
    fm.manual_move("stepper_z1", 0.5, 5.0, 100.0)
    assert toolhead.last_nudge == dict(
        mcu_id=0, axis_idx=2, motor_idx=1, dist=0.5, speed=5.0, accel=100.0
    )


def test_manual_move_defaults_accel_when_zero():
    fm, toolhead = make_force_move(
        enabled=True, return_toolhead=True, max_axis_accel=3000.0
    )
    fm.manual_move("stepper_z1", 0.5, 5.0, 0.0)
    assert toolhead.last_nudge["accel"] == pytest.approx(3000.0)


def test_manual_move_accepts_stepper_object():
    fm, toolhead = make_force_move(enabled=True, return_toolhead=True)

    class FakeStepper:
        def get_name(self):
            return "stepper_z1"

    fm.manual_move(FakeStepper(), 0.5, 5.0, 100.0)
    assert toolhead.last_nudge["motor_idx"] == 1


def test_manual_move_passes_negative_accel_through():
    fm, toolhead = make_force_move(enabled=True, return_toolhead=True)
    fm.manual_move("stepper_z1", 0.5, 5.0, -10.0)
    assert toolhead.last_nudge["accel"] == pytest.approx(-10.0)


class FakeGCmd:
    def __init__(self, params, floats=None):
        self._params = params
        self._floats = floats or {}
        self._responses = []

    def get(self, key, default=None):
        return self._params.get(key, default)

    def get_float(self, key, default=None, above=None, minval=None):
        return self._floats.get(key, default)

    def error(self, msg):
        return CommandError(msg)

    def respond_info(self, msg):
        self._responses.append(msg)


def test_cmd_force_move_parses_and_calls_manual_move():
    fm, toolhead = make_force_move(enabled=True, return_toolhead=True)
    gcmd = FakeGCmd(
        {"STEPPER": "stepper_z1"},
        floats={"DISTANCE": 1.0, "VELOCITY": 5.0, "ACCEL": 100.0},
    )
    fm.cmd_FORCE_MOVE(gcmd)
    assert toolhead.last_nudge["dist"] == pytest.approx(1.0)
    assert toolhead.last_nudge["speed"] == pytest.approx(5.0)
    assert toolhead.last_nudge["accel"] == pytest.approx(100.0)
    assert toolhead.last_nudge["motor_idx"] == 1


def test_cmd_force_move_defaults_accel_when_omitted():
    fm, toolhead = make_force_move(
        enabled=True, return_toolhead=True, max_axis_accel=3000.0
    )
    gcmd = FakeGCmd(
        {"STEPPER": "stepper_z1"}, floats={"DISTANCE": 0.5, "VELOCITY": 5.0}
    )
    fm.cmd_FORCE_MOVE(gcmd)
    assert toolhead.last_nudge["accel"] == pytest.approx(3000.0)
