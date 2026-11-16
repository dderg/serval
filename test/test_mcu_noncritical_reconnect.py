import pytest

from klippy import mcu, serialhdl, structured_log

RECONNECT_INTERVAL = 2.12
NEVER = float("inf")


class FakeReactor:
    NEVER = NEVER


class FakeGcode:
    def __init__(self):
        self.responses = []

    def respond_info(self, msg, log=True):
        self.responses.append(msg)


class FakePrinter:
    def __init__(self):
        self.events = []

    def send_event(self, name, *params):
        self.events.append(name)


class FakeClockSync:
    def __init__(self):
        self.disconnects = 0

    def disconnect(self):
        self.disconnects += 1


class ReconnectingMcu:
    non_critical_recon_event = mcu.MCU.non_critical_recon_event
    recon_mcu = mcu.MCU.recon_mcu
    _abandon_reconnect = mcu.MCU._abandon_reconnect

    def __init__(self, identify, connect=lambda: None):
        self._name = "beacon"
        self._reactor = FakeReactor()
        self._printer = FakePrinter()
        self._clocksync = FakeClockSync()
        self.gcode = FakeGcode()
        self._get_status_info = {"non_critical_disconnected": True}
        self.non_critical_disconnected = True
        self.reconnect_interval = RECONNECT_INTERVAL
        self._non_critical_reconnect_event_name = "reconnected"
        self._identify = identify
        self._connect_step = connect
        self.serial_disconnects = 0

    def _mcu_identify(self):
        return self._identify()

    def reset_to_initial_state(self):
        pass

    def _connect(self):
        self._connect_step()

    def _disconnect(self):
        self.serial_disconnects += 1


@pytest.fixture
def events(monkeypatch):
    recorded = []
    monkeypatch.setattr(
        structured_log,
        "event",
        lambda subsystem, event, **fields: recorded.append(
            (subsystem, event, fields)
        ),
    )
    return recorded


def _closed_serial():
    raise mcu.error("serial connection closed")


def _identify_ok():
    return True


def test_failed_identify_is_retried_without_escaping(events):
    beacon = ReconnectingMcu(_closed_serial)

    next_attempt = beacon.non_critical_recon_event(12.0)

    assert next_attempt == 12.0 + RECONNECT_INTERVAL
    assert beacon.non_critical_disconnected
    assert beacon._get_status_info["non_critical_disconnected"]
    assert beacon.serial_disconnects == 1
    assert beacon._clocksync.disconnects == 1
    assert beacon._printer.events == []


def test_failure_after_identify_leaves_the_mcu_disconnected(events):
    def config_fails():
        raise serialhdl.error("serial connection closed")

    beacon = ReconnectingMcu(_identify_ok, connect=config_fails)
    beacon._get_status_info["non_critical_disconnected"] = False

    next_attempt = beacon.non_critical_recon_event(12.0)

    assert next_attempt == 12.0 + RECONNECT_INTERVAL
    assert beacon.non_critical_disconnected
    assert beacon._get_status_info["non_critical_disconnected"]
    assert beacon.serial_disconnects == 1
    assert beacon._printer.events == []


def test_failed_reconnect_is_logged_as_a_structured_event(events):
    beacon = ReconnectingMcu(_closed_serial)

    beacon.non_critical_recon_event(12.0)

    assert len(events) == 1
    subsystem, event, fields = events[0]
    assert (subsystem, event) == ("mcu-comms", "non_critical_reconnect_failed")
    assert fields["mcu"] == "beacon"
    assert fields["reason"] == "serial connection closed"


def test_successful_reconnect_stops_the_timer(events):
    beacon = ReconnectingMcu(_identify_ok)

    next_attempt = beacon.non_critical_recon_event(12.0)

    assert next_attempt == NEVER
    assert not beacon.non_critical_disconnected
    assert beacon._printer.events == ["reconnected"]
    assert beacon.gcode.responses == ["mcu: 'beacon' reconnected!"]
    assert events == []


def test_an_unrelated_exception_still_escapes(events):
    def bug():
        raise ZeroDivisionError

    beacon = ReconnectingMcu(bug)

    with pytest.raises(ZeroDivisionError):
        beacon.non_critical_recon_event(12.0)
