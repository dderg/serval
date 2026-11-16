from fakes import FakePrinter

from klippy import gcode as gcode_mod


class _Toolhead:
    def __init__(self):
        self.calls = []

    def get_last_move_time(self):
        self.calls.append("get_last_move_time")
        return 1.0

    def dwell(self, delay):
        self.calls.append(("dwell", delay))

    def wait_moves(self):
        self.calls.append("wait_moves")


class _ExitingPrinter(FakePrinter):
    exit_result = None

    def request_exit(self, result):
        self.exit_result = result


def _ready_dispatch(printer):
    dispatch = gcode_mod.GCodeDispatch.__new__(gcode_mod.GCodeDispatch)
    dispatch.printer = printer
    dispatch.is_printer_ready = True
    return dispatch


def test_firmware_restart_exits_without_playing_out_queued_motion():
    toolhead = _Toolhead()
    printer = _ExitingPrinter(objects={"toolhead": toolhead})

    _ready_dispatch(printer).request_restart("firmware_restart")

    assert toolhead.calls == []
    assert printer.events == []
    assert printer.exit_result == "firmware_restart"


def test_host_restart_still_finishes_queued_motion_first():
    toolhead = _Toolhead()
    printer = _ExitingPrinter(objects={"toolhead": toolhead})

    _ready_dispatch(printer).request_restart("restart")

    assert toolhead.calls == [
        "get_last_move_time",
        ("dwell", 0.5),
        "wait_moves",
    ]
    assert printer.events == ["gcode:request_restart"]
    assert printer.exit_result == "restart"
