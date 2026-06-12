import pytest


class _FakeGCode:
    def __init__(self):
        self.messages = []

    def respond_info(self, msg):
        self.messages.append(msg)


class _CommandError(Exception):
    pass


class _FakeStepper:
    def __init__(self, name):
        self._name = name

    def get_name(self):
        return self._name


class _FakePrinter:
    command_error = _CommandError

    def __init__(self):
        self.gcode = _FakeGCode()
        self.handlers = []

    def register_event_handler(self, event, handler):
        self.handlers.append((event, handler))

    def lookup_object(self, name):
        assert name == "gcode"
        return self.gcode


class _FakeConfig:
    def __init__(self, printer):
        self._printer = printer

    def get_printer(self):
        return self._printer

    def get_name(self):
        return "z_tilt"


def test_adjust_steppers_reports_then_raises_not_implemented():
    from klippy.extras.z_tilt import ZAdjustHelper

    printer = _FakePrinter()
    helper = ZAdjustHelper(_FakeConfig(printer), 2)
    helper.z_steppers = [_FakeStepper("stepper_z"), _FakeStepper("stepper_z1")]
    with pytest.raises(_CommandError, match="not yet implemented"):
        helper.adjust_steppers([0.01, -0.01], 5.0)
    assert any("stepper_z1 = -0.01" in m for m in printer.gcode.messages)
