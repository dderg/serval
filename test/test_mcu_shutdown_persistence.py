from types import SimpleNamespace

from klippy import mcu


class FakePrinter:
    def __init__(self):
        self.exit_requests = []
        self.shutdown_messages = []

    def request_exit(self, result):
        self.exit_requests.append(result)

    def invoke_async_shutdown(self, message):
        self.shutdown_messages.append(message)


class FakeClockSync:
    def clock32_to_clock64(self, clock):
        return clock


class FakeMcu:
    _handle_shutdown = mcu.MCU._handle_shutdown

    def __init__(self):
        self._name = "mcu"
        self._printer = FakePrinter()
        self._clocksync = FakeClockSync()
        self._is_shutdown = False
        self._shutdown_clock = 0
        self._shutdown_msg = ""
        self._last_runtime_fault = None


def test_latched_firmware_crash_stays_shutdown(monkeypatch):
    monkeypatch.setattr(
        mcu,
        "get_danger_options",
        lambda: SimpleNamespace(log_shutdown_info=False),
    )
    monkeypatch.setattr(mcu, "shutdown_diagnostics", lambda printer, msg: "")
    crashed_mcu = FakeMcu()

    crashed_mcu._handle_shutdown(
        {
            "#name": "is_shutdown",
            "static_string_id": "Timer too close",
            "clock": 123,
        }
    )

    assert crashed_mcu._printer.exit_requests == []
    assert crashed_mcu._printer.shutdown_messages == [
        "Previous MCU 'mcu' shutdown: Timer too close"
    ]


def test_live_firmware_crash_reports_runtime_fault_without_restart(monkeypatch):
    monkeypatch.setattr(
        mcu,
        "get_danger_options",
        lambda: SimpleNamespace(log_shutdown_info=False),
    )
    monkeypatch.setattr(mcu, "shutdown_diagnostics", lambda printer, msg: "")
    crashed_mcu = FakeMcu()
    crashed_mcu._last_runtime_fault = "step scheduling fault"

    crashed_mcu._handle_shutdown(
        {
            "#name": "shutdown",
            "static_string_id": "kalico runtime fault",
            "clock": 456,
        }
    )

    assert crashed_mcu._printer.exit_requests == []
    assert crashed_mcu._printer.shutdown_messages == [
        "MCU 'mcu' shutdown: kalico runtime fault — step scheduling fault"
    ]


class LatchedMotionMcu:
    _recover_latched_peripheral_shutdown = (
        mcu.MCU._recover_latched_peripheral_shutdown
    )

    def __init__(self, reset_cmd, config_reset_cmd):
        self._name = "mcu"
        self._reset_cmd = reset_cmd
        self._config_reset_cmd = config_reset_cmd
        parser = SimpleNamespace(messages_by_name={"runtime_reset": None})
        self._serial = SimpleNamespace(get_msgparser=lambda: parser)


def _latched_error(latched):
    try:
        latched._recover_latched_peripheral_shutdown(None, "is_shutdown")
    except mcu.error as e:
        return str(e)
    raise AssertionError("a latched motion mcu must not resume in place")


def test_a_latched_motion_mcu_with_a_reset_command_points_at_firmware_restart():
    message = _latched_error(LatchedMotionMcu(object(), None))
    assert "FIRMWARE_RESTART" in message
    assert "power cycle" not in message


def test_a_latched_linux_motion_mcu_points_at_firmware_restart():
    message = _latched_error(LatchedMotionMcu(None, object()))
    assert "FIRMWARE_RESTART" in message


def test_a_latched_motion_mcu_without_a_reset_command_needs_a_power_cycle():
    message = _latched_error(LatchedMotionMcu(None, None))
    assert "power cycle" in message
