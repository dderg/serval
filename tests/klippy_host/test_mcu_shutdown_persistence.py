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


class FakeReconnectMcu:
    non_critical_recon_event = mcu.MCU.non_critical_recon_event

    def __init__(self):
        self._name = "beacon"
        self._get_status_info = {"non_critical_disconnected": False}
        self.non_critical_disconnected = False
        self.reconnect_interval = 5.0
        self.disconnect_count = 0

    def recon_mcu(self):
        raise mcu.error("serial connection closed")

    def _disconnect(self):
        self.disconnect_count += 1


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


def test_non_critical_reconnect_failure_retries_without_escaping():
    reconnecting_mcu = FakeReconnectMcu()

    next_attempt = reconnecting_mcu.non_critical_recon_event(12.0)

    assert next_attempt == 17.0
    assert reconnecting_mcu.non_critical_disconnected
    assert reconnecting_mcu._get_status_info["non_critical_disconnected"]
    assert reconnecting_mcu.disconnect_count == 1
