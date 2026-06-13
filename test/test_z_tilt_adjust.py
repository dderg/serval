class _FakeGCode:
    def __init__(self):
        self.messages = []

    def respond_info(self, msg):
        self.messages.append(msg)


class _FakeStepper:
    def __init__(self, name):
        self._name = name

    def get_name(self):
        return self._name


class _FakeAdjuster:
    def __init__(self):
        self.calls = []

    def adjust(self, stepper_name, delta_mm, speed, accel):
        self.calls.append((stepper_name, delta_mm, speed, accel))


class _FakeToolhead:
    def get_max_axis_accel(self, axis_idx):
        assert axis_idx == 2
        return 100.0


class _FakePrinter:
    def __init__(self):
        self.gcode = _FakeGCode()
        self.toolhead = _FakeToolhead()
        self.adjuster = _FakeAdjuster()
        self.handlers = []

    def register_event_handler(self, event, handler):
        self.handlers.append((event, handler))

    def lookup_object(self, name):
        return {"gcode": self.gcode, "toolhead": self.toolhead}[name]

    def load_object(self, config, name):
        assert name == "motor_adjust"
        return self.adjuster


class _FakeConfig:
    def __init__(self, printer):
        self._printer = printer

    def get_printer(self):
        return self._printer

    def get_name(self):
        return "z_tilt"


def test_adjust_steppers_applies_min_relative_deltas():
    from klippy.extras.z_tilt import ZAdjustHelper

    printer = _FakePrinter()
    helper = ZAdjustHelper(_FakeConfig(printer), 3)
    helper.z_steppers = [
        _FakeStepper("stepper_z"),
        _FakeStepper("stepper_z1"),
        _FakeStepper("stepper_z2"),
    ]
    helper.adjust_steppers([5.76, -7.80, 7.69], 5.0)
    assert printer.adjuster.calls == [
        ("stepper_z", 5.76 - -7.80, 5.0, 100.0),
        ("stepper_z2", 7.69 - -7.80, 5.0, 100.0),
    ]
    assert any("stepper_z1 = 0.0" in m for m in printer.gcode.messages)


def test_adjust_steppers_skips_all_when_level():
    from klippy.extras.z_tilt import ZAdjustHelper

    printer = _FakePrinter()
    helper = ZAdjustHelper(_FakeConfig(printer), 2)
    helper.z_steppers = [_FakeStepper("stepper_z"), _FakeStepper("stepper_z1")]
    helper.adjust_steppers([3.0, 3.0], 5.0)
    assert printer.adjuster.calls == []
