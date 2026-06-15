from klippy.extras.force_move import ForceMove


class FakeStepper:
    def __init__(self, name):
        self._name = name

    def get_name(self, short=False):
        return self._name


class FakeToolhead:
    def __init__(self, binding, max_accel):
        self._binding = binding
        self._max_accel = max_accel
        self.calls = []

    def get_motor_binding(self, name):
        return self._binding

    def get_max_axis_accel(self, axis_idx):
        return self._max_accel

    def submit_correction(
        self, mcu_id, axis_idx, motor_idx, segments, speed, accel
    ):
        self.calls.append(
            (mcu_id, axis_idx, motor_idx, list(segments), speed, accel)
        )
        return 0.25


class FakePrinter:
    def __init__(self, toolhead):
        self._toolhead = toolhead

    def lookup_object(self, name, default=None):
        return {"toolhead": self._toolhead}.get(name, default)


def make_force_move(binding=(7, 1, 0), max_accel=3000.0):
    fm = ForceMove.__new__(ForceMove)
    toolhead = FakeToolhead(binding, max_accel)
    fm.printer = FakePrinter(toolhead)
    return fm, toolhead


def test_manual_move_calls_toolhead_with_single_segment():
    fm, toolhead = make_force_move(binding=(7, 1, 0))
    dur = fm.manual_move(FakeStepper("stepper_x1"), 0.4, 12.0, 800.0)
    assert dur == 0.25
    assert toolhead.calls == [(7, 1, 0, [0.4], 12.0, 800.0)]


def test_manual_move_substitutes_machine_accel_when_unset():
    fm, toolhead = make_force_move(binding=(7, 1, 0), max_accel=3000.0)
    fm.manual_move(FakeStepper("stepper_x1"), 0.4, 12.0)
    assert toolhead.calls[0][5] == 3000.0


def test_manual_move_accepts_stepper_name_string():
    fm, toolhead = make_force_move()
    fm.manual_move("stepper_x1", 0.4, 12.0, 800.0)
    assert toolhead.calls[0][:3] == (7, 1, 0)


def test_manual_move_does_not_absorb_negative_accel():
    fm, toolhead = make_force_move(binding=(7, 1, 0), max_accel=3000.0)
    fm.manual_move(FakeStepper("stepper_x1"), 0.4, 12.0, -5.0)
    assert toolhead.calls[0][5] == -5.0
