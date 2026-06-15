from klippy.extras import homing as homing_mod


class RecordingPrinter:
    def __init__(self):
        self.events = []

    def send_event(self, event, *args):
        self.events.append((event, args))
        return []


class FakeRail:
    def __init__(self, name):
        self._name = name

    def get_name(self, short=False):
        return self._name


class FakeKin:
    def __init__(self, rails):
        self._rails = rails

    def _axis_rails(self):
        return dict(self._rails)


def _homer(printer):
    homer = homing_mod.Homing.__new__(homing_mod.Homing)
    homer.printer = printer
    return homer


def test_homing_state_get_axes_is_a_copy():
    state = homing_mod.HomingState([0, 2])
    axes = state.get_axes()
    assert axes == [0, 2]
    axes.append(1)
    assert state.get_axes() == [0, 2]


def test_emit_home_rails_end_sends_axes_and_rails():
    printer = RecordingPrinter()
    homer = _homer(printer)
    x_rail, z_rail = FakeRail("x"), FakeRail("z")
    kin = FakeKin({0: x_rail, 2: z_rail})
    homer._emit_home_rails_end(kin, [2])
    assert len(printer.events) == 1
    name, (state, rails) = printer.events[0]
    assert name == "homing:home_rails_end"
    assert state.get_axes() == [2]
    assert rails == [z_rail]


def test_emit_home_rails_end_multiple_axes_preserves_order():
    printer = RecordingPrinter()
    homer = _homer(printer)
    x_rail, y_rail, z_rail = FakeRail("x"), FakeRail("y"), FakeRail("z")
    kin = FakeKin({0: x_rail, 1: y_rail, 2: z_rail})
    homer._emit_home_rails_end(kin, [0, 1, 2])
    name, (state, rails) = printer.events[0]
    assert state.get_axes() == [0, 1, 2]
    assert rails == [x_rail, y_rail, z_rail]
