import pytest
from fakes import FakeConfig, FakeGcode, FakePrinter, FakeStepper
from fakes import FakeToolhead as FakeToolheadBase

from klippy.extras.z_tilt import ZAdjustHelper as ZTiltHelper
from klippy.extras.z_tilt_ng import ZAdjustHelper as ZTiltNgHelper


class FakeForceMoveRecorder:
    def __init__(self):
        self.calls = []

    def manual_move(self, stepper, dist, speed, accel):
        name = stepper if isinstance(stepper, str) else stepper.get_name()
        self.calls.append(dict(name=name, dist=dist, speed=speed, accel=accel))


class FakeToolhead(FakeToolheadBase):
    def __init__(self, max_accel=500.0):
        super().__init__(
            position=[150.0, 150.0, 30.0, 0.0], max_axis_accel=max_accel
        )
        self.wait_moves_called = 0
        self.set_position_calls = []

    def wait_moves(self):
        super().wait_moves()
        self.wait_moves_called += 1

    def set_position(self, newpos, homing_axes=()):
        super().set_position(newpos, homing_axes)
        self.set_position_calls.append(list(newpos))


def make_z_tilt_helper(z_names, max_accel=500.0):
    fm = FakeForceMoveRecorder()
    toolhead = FakeToolhead(max_accel=max_accel)
    gcode = FakeGcode()
    printer = FakePrinter(
        objects={
            "toolhead": toolhead,
            "gcode": gcode,
            "force_move": fm,
        }
    )
    config = FakeConfig()

    helper = ZTiltHelper.__new__(ZTiltHelper)
    helper.printer = printer
    helper.config = config
    helper.z_steppers = [FakeStepper(name=n) for n in z_names]
    return helper, fm, toolhead


def make_z_tilt_ng_helper(z_names, max_accel=500.0):
    fm = FakeForceMoveRecorder()
    toolhead = FakeToolhead(max_accel=max_accel)
    gcode = FakeGcode()
    printer = FakePrinter(
        objects={
            "toolhead": toolhead,
            "gcode": gcode,
            "force_move": fm,
        }
    )
    config = FakeConfig()

    helper = ZTiltNgHelper.__new__(ZTiltNgHelper)
    helper.printer = printer
    helper.config = config
    helper.z_steppers = [FakeStepper(name=n) for n in z_names]
    return helper, fm, toolhead


def test_z_tilt_adjust_steppers_calls_force_move_per_nonzero_delta():
    helper, fm, toolhead = make_z_tilt_helper(z_names=["z", "z1", "z2"])
    helper.adjust_steppers([5.0, 5.0009, 5.002], speed=10.0)
    assert [c["name"] for c in fm.calls] == ["z1", "z2"]
    assert fm.calls[0]["dist"] == pytest.approx(0.0009)
    assert fm.calls[1]["dist"] == pytest.approx(0.002)
    assert all(c["speed"] == 10.0 for c in fm.calls)
    assert all(c["accel"] == pytest.approx(500.0) for c in fm.calls)


def test_z_tilt_adjust_steppers_skips_all_equal():
    helper, fm, toolhead = make_z_tilt_helper(z_names=["z", "z1"])
    helper.adjust_steppers([3.0, 3.0], speed=5.0)
    assert fm.calls == []
    assert toolhead.set_position_calls == [[150.0, 150.0, 27.0, 0.0]]


def test_z_tilt_adjust_steppers_rebases_z_by_negative_reference():
    helper, fm, toolhead = make_z_tilt_helper(z_names=["z", "z1", "z2"])
    helper.adjust_steppers([5.76, -7.80, 7.69], speed=5.0)
    assert [c["name"] for c in fm.calls] == ["z", "z2"]
    assert fm.calls[0]["dist"] == pytest.approx(5.76 - -7.80)
    assert fm.calls[1]["dist"] == pytest.approx(7.69 - -7.80)
    assert toolhead.set_position_calls == [[150.0, 150.0, 37.80, 0.0]]


def test_z_tilt_adjust_steppers_wait_moves_called_once():
    helper, fm, toolhead = make_z_tilt_helper(z_names=["z", "z1", "z2"])
    helper.adjust_steppers([5.0, 5.001, 5.002], speed=10.0)
    assert toolhead.wait_moves_called == 1


def test_z_tilt_ng_adjust_steppers_calls_force_move_per_nonzero_delta():
    helper, fm, toolhead = make_z_tilt_ng_helper(z_names=["z", "z1", "z2"])
    helper.adjust_steppers([5.0, 5.0009, 5.002], speed=10.0)
    assert [c["name"] for c in fm.calls] == ["z1", "z2"]
    assert fm.calls[0]["dist"] == pytest.approx(0.0009)
    assert fm.calls[1]["dist"] == pytest.approx(0.002)
    assert all(c["speed"] == 10.0 for c in fm.calls)
    assert all(c["accel"] == pytest.approx(500.0) for c in fm.calls)


def test_z_tilt_ng_adjust_steppers_skips_all_equal():
    helper, fm, toolhead = make_z_tilt_ng_helper(z_names=["z", "z1"])
    helper.adjust_steppers([3.0, 3.0], speed=5.0)
    assert fm.calls == []
    assert toolhead.set_position_calls == [[150.0, 150.0, 27.0, 0.0]]


def test_z_tilt_ng_adjust_steppers_rebases_z_by_negative_reference():
    helper, fm, toolhead = make_z_tilt_ng_helper(z_names=["z", "z1", "z2"])
    helper.adjust_steppers([5.76, -7.80, 7.69], speed=5.0)
    assert [c["name"] for c in fm.calls] == ["z", "z2"]
    assert toolhead.set_position_calls == [[150.0, 150.0, 37.80, 0.0]]


def test_z_tilt_ng_adjust_steppers_wait_moves_called_once():
    helper, fm, toolhead = make_z_tilt_ng_helper(z_names=["z", "z1", "z2"])
    helper.adjust_steppers([5.0, 5.001, 5.002], speed=10.0)
    assert toolhead.wait_moves_called == 1
