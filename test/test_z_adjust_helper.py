import pytest

from klippy.extras.z_tilt import ZAdjustHelper as ZTiltHelper
from klippy.extras.z_tilt_ng import ZAdjustHelper as ZTiltNgHelper


class FakeStepper:
    def __init__(self, name):
        self._name = name

    def get_name(self):
        return self._name


class FakeGcode:
    def respond_info(self, msg):
        pass


class FakeForceMoveRecorder:
    def __init__(self):
        self.calls = []

    def manual_move(self, stepper, dist, speed, accel):
        name = stepper if isinstance(stepper, str) else stepper.get_name()
        self.calls.append(dict(name=name, dist=dist, speed=speed, accel=accel))


class FakeToolhead:
    def __init__(self, max_accel=500.0):
        self._max_accel = max_accel
        self.wait_moves_called = 0

    def get_max_axis_accel(self, axis_idx):
        return self._max_accel

    def wait_moves(self):
        self.wait_moves_called += 1


class FakeConfig:
    pass


class FakePrinter:
    def __init__(self, force_move, toolhead):
        self._force_move = force_move
        self._toolhead = toolhead
        self._gcode = FakeGcode()

    def load_object(self, config, name):
        if name == "force_move":
            return self._force_move
        raise KeyError(name)

    def lookup_object(self, name, default=None):
        mapping = {
            "gcode": self._gcode,
            "toolhead": self._toolhead,
        }
        return mapping.get(name, default)


def make_z_tilt_helper(z_names, max_accel=500.0):
    fm = FakeForceMoveRecorder()
    toolhead = FakeToolhead(max_accel=max_accel)
    printer = FakePrinter(fm, toolhead)
    config = FakeConfig()

    helper = ZTiltHelper.__new__(ZTiltHelper)
    helper.printer = printer
    helper.config = config
    helper.z_steppers = [FakeStepper(n) for n in z_names]
    return helper, fm, toolhead


def make_z_tilt_ng_helper(z_names, max_accel=500.0):
    fm = FakeForceMoveRecorder()
    toolhead = FakeToolhead(max_accel=max_accel)
    printer = FakePrinter(fm, toolhead)
    config = FakeConfig()

    helper = ZTiltNgHelper.__new__(ZTiltNgHelper)
    helper.printer = printer
    helper.config = config
    helper.z_steppers = [FakeStepper(n) for n in z_names]
    return helper, fm, toolhead


def test_z_tilt_adjust_steppers_calls_force_move_per_nonzero_delta():
    helper, fm, toolhead = make_z_tilt_helper(z_names=["z", "z1", "z2"])
    helper.adjust_steppers([5.0, 5.0009, 5.002], speed=10.0)
    # delta for "z" is 0 (min, skipped via < 1e-6); "z1"=0.0009; "z2"=0.002
    assert [c["name"] for c in fm.calls] == ["z1", "z2"]
    assert fm.calls[0]["dist"] == pytest.approx(0.0009)
    assert fm.calls[1]["dist"] == pytest.approx(0.002)
    assert all(c["speed"] == 10.0 for c in fm.calls)
    assert all(c["accel"] == pytest.approx(500.0) for c in fm.calls)


def test_z_tilt_adjust_steppers_skips_all_equal():
    helper, fm, _ = make_z_tilt_helper(z_names=["z", "z1"])
    helper.adjust_steppers([3.0, 3.0], speed=5.0)
    assert fm.calls == []


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
    helper, fm, _ = make_z_tilt_ng_helper(z_names=["z", "z1"])
    helper.adjust_steppers([3.0, 3.0], speed=5.0)
    assert fm.calls == []


def test_z_tilt_ng_adjust_steppers_wait_moves_called_once():
    helper, fm, toolhead = make_z_tilt_ng_helper(z_names=["z", "z1", "z2"])
    helper.adjust_steppers([5.0, 5.001, 5.002], speed=10.0)
    assert toolhead.wait_moves_called == 1
