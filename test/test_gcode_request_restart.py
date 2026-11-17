from logging import WARNING

import pytest
from fakes import FakeConfig, FakePrinter

from klippy import engine_wait
from klippy import gcode as gcode_mod
from klippy import reactor as reactor_mod
from klippy.extras import virtual_sdcard

API_RESTART_AFTER_S = 0.05


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


class _ReactorPrinter(FakePrinter):
    def request_exit(self, result):
        self.exit_result = result
        self.reactor.end()


def _print_m400_then_api_firmware_restart():
    reactor = reactor_mod.Reactor()
    printer = _ReactorPrinter(reactor=reactor)
    dispatch = gcode_mod.GCodeDispatch(printer)
    dispatch.register_command(
        "M400",
        lambda gcmd: engine_wait.wait_for(
            printer, lambda: None, "wait_moves", engine_wait.UNBOUNDED
        ),
    )
    printer.send_event("klippy:ready")

    def sd_reader(eventtime):
        dispatch.run_script("M400")
        return reactor.NEVER

    def api_firmware_restart(eventtime):
        dispatch.request_restart("firmware_restart")
        return reactor.NEVER

    reactor.register_timer(sd_reader, reactor.NOW)
    reactor.register_timer(
        api_firmware_restart, reactor.monotonic() + API_RESTART_AFTER_S
    )
    reactor.run()
    reactor.finalize()
    return printer


def test_firmware_restart_unwinds_a_waiting_command_quietly(caplog):
    printer = _print_m400_then_api_firmware_restart()

    assert printer.exit_result == "firmware_restart"
    assert printer.shutdown_reasons == []
    assert "Internal error" not in caplog.text
    assert "greenlet terminate" not in caplog.text


def test_a_command_crash_is_still_an_internal_error():
    printer = FakePrinter()
    dispatch = gcode_mod.GCodeDispatch(printer)
    dispatch.register_command("M400", lambda gcmd: 1 / 0)
    printer.send_event("klippy:ready")

    with pytest.raises(ZeroDivisionError):
        dispatch.run_script_from_command("M400")

    assert printer.shutdown_reasons == ['Internal error on command:"M400"']


class _PrintStats:
    def __init__(self):
        self.notes = []

    def reset(self):
        self.notes.append("reset")

    def set_current_file(self, filename):
        self.notes.append("file")

    def note_start(self):
        self.notes.append("start")

    def note_pause(self):
        self.notes.append("pause")

    def note_complete(self):
        self.notes.append("complete")

    def note_error(self, message):
        self.notes.append("error")


class _EmptyTemplate:
    def render(self):
        return ""


class _GcodeMacro:
    def load_template(self, config, option, default=None):
        return _EmptyTemplate()


def test_firmware_restart_unwinds_the_sd_print_parked_in_m400(tmp_path, caplog):
    (tmp_path / "print.gcode").write_text("M400\nG1 X1\n")
    reactor = reactor_mod.Reactor()
    printer = _ReactorPrinter(reactor=reactor)
    dispatch = gcode_mod.GCodeDispatch(printer)
    printer.add_object("gcode", dispatch)
    print_stats = _PrintStats()
    printer.add_object("print_stats", print_stats)
    printer.add_object("gcode_macro", _GcodeMacro())
    virtual_sdcard.VirtualSD(
        FakeConfig(printer, "virtual_sdcard", {"path": str(tmp_path)})
    )
    moves = []
    dispatch.register_command(
        "M400",
        lambda gcmd: engine_wait.wait_for(
            printer, lambda: None, "wait_moves", engine_wait.UNBOUNDED
        ),
    )
    dispatch.register_command("G1", lambda gcmd: moves.append(gcmd))
    printer.send_event("klippy:ready")
    reactor.register_callback(
        lambda e: dispatch.run_script("SDCARD_PRINT_FILE FILENAME=print.gcode")
    )
    reactor.register_callback(
        lambda e: dispatch.request_restart("firmware_restart"),
        reactor.monotonic() + API_RESTART_AFTER_S,
    )

    reactor.run()
    reactor.finalize()

    assert printer.shutdown_reasons == []
    assert print_stats.notes == ["reset", "file", "start"]
    assert moves == []
    assert [
        r.getMessage() for r in caplog.records if r.levelno >= WARNING
    ] == []
