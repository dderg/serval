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


class _FakeForceMoveRecorder:
    def __init__(self):
        self.calls = []

    def manual_move(self, stepper, dist, speed, accel):
        name = stepper if isinstance(stepper, str) else stepper.get_name()
        self.calls.append((name, dist, speed, accel))


class _FakeToolhead:
    def __init__(self):
        self.wait_moves_and_mcu_called = 0

    def get_max_axis_accel(self, axis_idx):
        assert axis_idx == 2
        return 100.0

    def wait_moves_and_mcu(self):
        self.wait_moves_and_mcu_called += 1


class _FakePrinter:
    def __init__(self):
        self.gcode = _FakeGCode()
        self.toolhead = _FakeToolhead()
        self.force_move = _FakeForceMoveRecorder()
        self.handlers = []

    def register_event_handler(self, event, handler):
        self.handlers.append((event, handler))

    def lookup_object(self, name):
        return {"gcode": self.gcode, "toolhead": self.toolhead}[name]

    def load_object(self, config, name):
        assert name == "force_move"
        return self.force_move


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
    assert printer.force_move.calls == [
        ("stepper_z", 5.76 - -7.80, 5.0, 100.0),
        ("stepper_z2", 7.69 - -7.80, 5.0, 100.0),
    ]
    assert any("stepper_z1 = 0.0" in m for m in printer.gcode.messages)
    assert printer.toolhead.wait_moves_and_mcu_called == 1


def test_adjust_steppers_skips_all_when_level():
    from klippy.extras.z_tilt import ZAdjustHelper

    printer = _FakePrinter()
    helper = ZAdjustHelper(_FakeConfig(printer), 2)
    helper.z_steppers = [_FakeStepper("stepper_z"), _FakeStepper("stepper_z1")]
    helper.adjust_steppers([3.0, 3.0], 5.0)
    assert printer.force_move.calls == []
    assert printer.toolhead.wait_moves_and_mcu_called == 1
