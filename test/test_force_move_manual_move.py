from klippy.extras.force_move import ForceMove


class FakeBridge:
    def __init__(self):
        self.calls = []

    def submit_correction_sequence(
        self, mcu_id, axis_idx, motor_idx, segments, speed, accel
    ):
        self.calls.append(
            (mcu_id, axis_idx, motor_idx, list(segments), speed, accel)
        )
        return 0.25


class FakeStepper:
    def __init__(self, name):
        self._name = name

    def get_name(self, short=False):
        return self._name


class FakeToolhead:
    def __init__(self, bridge, binding, max_accel):
        self._bridge = bridge
        self._binding = binding
        self._max_accel = max_accel

    def get_motor_binding(self, name):
        return self._binding

    def get_bridge(self):
        return self._bridge

    def get_max_axis_accel(self, axis_idx):
        return self._max_accel


class FakePrinter:
    def __init__(self, toolhead):
        self._toolhead = toolhead

    def lookup_object(self, name, default=None):
        return {"toolhead": self._toolhead}.get(name, default)


def make_force_move(bridge, binding=(7, 1, 0), max_accel=3000.0):
    fm = ForceMove.__new__(ForceMove)
    fm.printer = FakePrinter(FakeToolhead(bridge, binding, max_accel))
    return fm


def test_manual_move_calls_bridge_with_single_segment():
    bridge = FakeBridge()
    fm = make_force_move(bridge, binding=(7, 1, 0))
    dur = fm.manual_move(FakeStepper("stepper_x1"), 0.4, 12.0, 800.0)
    assert dur == 0.25
    assert bridge.calls == [(7, 1, 0, [0.4], 12.0, 800.0)]


def test_manual_move_substitutes_machine_accel_when_unset():
    bridge = FakeBridge()
    fm = make_force_move(bridge, binding=(7, 1, 0), max_accel=3000.0)
    fm.manual_move(FakeStepper("stepper_x1"), 0.4, 12.0)
    assert bridge.calls[0][5] == 3000.0


def test_manual_move_accepts_stepper_name_string():
    bridge = FakeBridge()
    fm = make_force_move(bridge)
    fm.manual_move("stepper_x1", 0.4, 12.0, 800.0)
    assert bridge.calls[0][:3] == (7, 1, 0)


def test_manual_move_does_not_absorb_negative_accel():
    bridge = FakeBridge()
    fm = make_force_move(bridge, binding=(7, 1, 0), max_accel=3000.0)
    fm.manual_move(FakeStepper("stepper_x1"), 0.4, 12.0, -5.0)
    assert bridge.calls[0][5] == -5.0
