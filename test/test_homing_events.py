from fakes import FakeKin, FakeRail

from klippy.extras import homing as homing_mod


class RecordingPrinter:
    def __init__(self):
        self.events = []

    def send_event(self, event, *args):
        self.events.append((event, args))
        return []


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
    x_rail, z_rail = FakeRail(name="x"), FakeRail(name="z")
    kin = FakeKin(axis_rails={0: x_rail, 2: z_rail})
    homer._emit_home_rails_end(kin, [2])
    assert len(printer.events) == 1
    name, (state, rails) = printer.events[0]
    assert name == "homing:home_rails_end"
    assert state.get_axes() == [2]
    assert rails == [z_rail]


def test_emit_home_rails_end_multiple_axes_preserves_order():
    printer = RecordingPrinter()
    homer = _homer(printer)
    x_rail, y_rail, z_rail = (
        FakeRail(name="x"),
        FakeRail(name="y"),
        FakeRail(name="z"),
    )
    kin = FakeKin(axis_rails={0: x_rail, 1: y_rail, 2: z_rail})
    homer._emit_home_rails_end(kin, [0, 1, 2])
    name, (state, rails) = printer.events[0]
    assert state.get_axes() == [0, 1, 2]
    assert rails == [x_rail, y_rail, z_rail]
