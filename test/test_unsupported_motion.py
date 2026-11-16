import types

import pytest

from klippy import gcode as gcode_module
from klippy.extras import gcode_move

REFUSED = ("G2", "G3", "G10", "G11")
HARMLESS_UNKNOWN = "M2000 S1"


class FakePrinter:
    config_error = RuntimeError

    def __init__(self):
        self.objects = {}

    def register_event_handler(self, event, callback):
        pass

    def get_start_args(self):
        return {}

    def get_reactor(self):
        return types.SimpleNamespace(mutex=lambda: None)

    def lookup_object(self, name, default=None):
        return self.objects.get(name, default)

    def send_event(self, event, *params):
        pass

    def get_state_message(self):
        return "Printer is ready", "ready"

    def request_exit(self, result):
        pass


@pytest.fixture
def dispatch():
    printer = FakePrinter()
    gcode = gcode_module.GCodeDispatch(printer)
    printer.objects["gcode"] = gcode
    config = types.SimpleNamespace(get_printer=lambda: printer)
    gcode_move.GCodeMove(config)
    gcode._handle_ready()
    return gcode


@pytest.mark.parametrize("command", REFUSED)
def test_a_refused_motion_command_raises(dispatch, command):
    line = f"{command} X10 Y10 I5 J0"
    with pytest.raises(dispatch.error) as raised:
        dispatch.run_script_from_command(line)
    message = str(raised.value)
    assert command in message
    assert line in message
    assert "not supported" in message
    assert "slicer" in message


def test_arcs_and_retraction_name_their_slicer_setting(dispatch):
    with pytest.raises(dispatch.error, match="arc fitting"):
        dispatch.run_script_from_command("G3 X1 Y1 I1 J0")
    with pytest.raises(dispatch.error, match="firmware retraction"):
        dispatch.run_script_from_command("G10")


def test_an_unknown_command_is_still_ignored(dispatch):
    dispatch.run_script_from_command(HARMLESS_UNKNOWN)
