from fakes import FakeReactor

import klippy.printer


def make_failed_connect_printer(monkeypatch):
    printer = klippy.printer.Printer(FakeReactor(), None, {})
    disconnects = []
    printer.register_event_handler(
        "klippy:disconnect", lambda: disconnects.append(1)
    )

    def raise_config_error():
        raise printer.config_error(
            "Option 'velocity_ff' is not valid in section 'servo_x'"
        )

    monkeypatch.setattr(printer, "_read_config", raise_config_error)
    return printer, disconnects


def test_config_error_keeps_printer_alive_and_reporting(monkeypatch):
    printer, disconnects = make_failed_connect_printer(monkeypatch)

    printer._connect(0.0)

    assert "velocity_ff" in printer.state_message
    assert not disconnects, (
        "a config error must not dispatch klippy:disconnect — webhooks must "
        "keep serving the error state to moonraker"
    )


def test_exit_after_failed_connect_dispatches_disconnect_once(monkeypatch):
    printer, disconnects = make_failed_connect_printer(monkeypatch)
    printer._connect(0.0)

    run_result = printer.run()

    assert run_result is None
    assert disconnects == [1]

    printer._dispatch_disconnect()
    assert disconnects == [1]
