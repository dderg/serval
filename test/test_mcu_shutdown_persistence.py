from types import SimpleNamespace

from klippy import mcu


class FakePrinter:
    def __init__(self, state="ready"):
        self.exit_requests = []
        self.shutdown_messages = []
        self.state = state

    def request_exit(self, result):
        self.exit_requests.append(result)

    def invoke_async_shutdown(self, message):
        self.shutdown_messages.append(message)

    def get_state_message(self):
        return "", self.state


class FakeClockSync:
    def clock32_to_clock64(self, clock):
        return clock


class FakeMcu:
    _handle_shutdown = mcu.MCU._handle_shutdown
    _resettable = mcu.MCU._resettable

    def __init__(self, reset_cmd=object(), printer_state="ready"):
        self._name = "mcu"
        self._printer = FakePrinter(printer_state)
        self._clocksync = FakeClockSync()
        self._is_shutdown = False
        self._shutdown_clock = 0
        self._shutdown_msg = ""
        self._last_runtime_fault = None
        self._reset_cmd = reset_cmd
        self._config_reset_cmd = None


def _quiet_shutdown_reports(monkeypatch):
    monkeypatch.setattr(
        mcu,
        "get_danger_options",
        lambda: SimpleNamespace(log_shutdown_info=False),
    )
    monkeypatch.setattr(mcu, "shutdown_diagnostics", lambda printer, msg: "")


LATCHED_COMMAND_REQUEST = {
    "#name": "is_shutdown",
    "static_string_id": "Command request",
    "clock": 123,
}


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


class FakeGetConfig:
    def __init__(self, serial):
        self._serial = serial

    def send(self):
        if "clear_shutdown" not in self._serial.sent:
            raise RuntimeError("mcu is in shutdown state: unresolved reason")
        return {"is_config": 1, "crc": 7, "is_shutdown": 0, "move_count": 64}


class FakeSerial:
    def __init__(self, motion):
        self.sent = []
        messages = {"runtime_reset": None} if motion else {}
        self._parser = SimpleNamespace(messages_by_name=messages)

    def get_msgparser(self):
        return self._parser

    def send(self, message):
        self.sent.append(message)


class LatchedMcu:
    _recover_latched_shutdown = mcu.MCU._recover_latched_shutdown
    _resettable = mcu.MCU._resettable

    def __init__(self, reset_cmd=None, config_reset_cmd=None, motion=False):
        self._name = "mcu"
        self._reset_cmd = reset_cmd
        self._config_reset_cmd = config_reset_cmd
        self._serial = FakeSerial(motion)
        self._is_shutdown = True
        self._shutdown_msg = "Command request"
        self._last_runtime_fault = None

    def recover(self):
        get_config = FakeGetConfig(self._serial)
        try:
            get_config.send()
        except RuntimeError as exc:
            return self._recover_latched_shutdown(get_config, exc)
        raise AssertionError("the fake mcu starts latched")


def _latched_error(latched):
    try:
        latched.recover()
    except mcu.error as e:
        return str(e)
    raise AssertionError("a latched resettable mcu must not resume in place")


def test_a_latched_mcu_with_a_reset_command_waits_for_firmware_restart():
    latched = LatchedMcu(reset_cmd=object())
    message = _latched_error(latched)
    assert "FIRMWARE_RESTART" in message
    assert latched._serial.sent == []


def test_a_latched_linux_mcu_waits_for_firmware_restart():
    latched = LatchedMcu(config_reset_cmd=object(), motion=True)
    assert "FIRMWARE_RESTART" in _latched_error(latched)
    assert latched._serial.sent == []


def test_a_latched_motion_mcu_without_a_reset_command_needs_a_power_cycle():
    latched = LatchedMcu(motion=True)
    assert "power cycle" in _latched_error(latched)
    assert latched._serial.sent == []


def test_a_latched_peripheral_without_a_reset_command_is_cleared_in_place():
    latched = LatchedMcu()
    params = latched.recover()
    assert latched._serial.sent == ["clear_shutdown"]
    assert params["is_shutdown"] == 0
    assert not latched._is_shutdown


def test_a_resettable_mcu_latched_at_connect_shuts_the_printer_down(
    monkeypatch,
):
    _quiet_shutdown_reports(monkeypatch)
    latched = FakeMcu(reset_cmd=object(), printer_state="startup")
    latched._handle_shutdown(LATCHED_COMMAND_REQUEST)
    assert latched._printer.shutdown_messages == [
        "Previous MCU 'mcu' shutdown: Command request"
    ]


def test_a_peripheral_latched_at_connect_is_left_to_its_in_place_clear(
    monkeypatch,
):
    _quiet_shutdown_reports(monkeypatch)
    latched = FakeMcu(reset_cmd=None, printer_state="startup")
    latched._handle_shutdown(LATCHED_COMMAND_REQUEST)
    assert latched._printer.shutdown_messages == []
    assert latched._is_shutdown


def test_a_peripheral_latched_after_connect_still_shuts_the_printer_down(
    monkeypatch,
):
    _quiet_shutdown_reports(monkeypatch)
    latched = FakeMcu(reset_cmd=None, printer_state="ready")
    latched._handle_shutdown(LATCHED_COMMAND_REQUEST)
    assert latched._printer.shutdown_messages == [
        "Previous MCU 'mcu' shutdown: Command request"
    ]
