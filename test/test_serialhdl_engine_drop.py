import math
import os
import sys
import threading
from types import SimpleNamespace
from unittest import mock

sys.path.insert(
    0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
)

from fakes import FakeEngine, FakePrinter  # noqa: E402

from klippy.engine_mcu import EngineMcu  # noqa: E402
from klippy.serialhdl import EngineCommandChannel, error  # noqa: E402

TRANSPORT_CLOSED = RuntimeError("engine_send: transport closed")


def _reader(engine):
    sr = EngineCommandChannel.__new__(EngineCommandChannel)
    sr.mcu = None
    sr.engine_mcu = EngineMcu(
        FakePrinter(objects={"motion_engine": engine}), "mcu"
    )
    sr.engine_mcu.claim("", 0)
    sr._engine_detached = False
    sr.warn_prefix = ""
    return sr


def test_drop_detected_marks_detached():
    sr = _reader(FakeEngine())
    assert sr._is_engine_transport_drop(TRANSPORT_CLOSED) is True
    assert sr._engine_detached is True


def test_non_drop_error_left_alone():
    sr = _reader(FakeEngine())
    assert sr._is_engine_transport_drop(RuntimeError("other")) is False
    assert sr._engine_detached is False


def test_send_swallows_drop():
    sr = _reader(FakeEngine(raises=TRANSPORT_CLOSED))
    sr.send(b"x")
    assert sr._engine_detached is True


def test_send_reraises_unrelated_runtime_error():
    sr = _reader(FakeEngine(raises=RuntimeError("boom")))
    try:
        sr.send(b"x")
        raise AssertionError("expected RuntimeError")
    except RuntimeError as e:
        assert "boom" in str(e)
    assert sr._engine_detached is False


def test_send_with_response_raises_mainline_error_on_drop():
    sr = _reader(FakeEngine(raises=TRANSPORT_CLOSED))
    try:
        sr.send_with_response(b"x", "resp")
        raise AssertionError("expected serialhdl.error")
    except error:
        pass
    assert sr._engine_detached is True


def test_engine_get_clock_async_swallows_drop():
    sr = _reader(FakeEngine(raises=TRANSPORT_CLOSED))
    sr.engine_get_clock_async()
    assert sr._engine_detached is True


def _response_reader():
    sr = EngineCommandChannel.__new__(EngineCommandChannel)
    sr.mcu = SimpleNamespace(get_name=lambda: "bottom")
    sr.lock = threading.Lock()
    sr.handlers = {}
    sr.handle_default = lambda params: None
    sr.warn_prefix = ""
    return sr


def test_response_dispatch_lag_uses_wire_receive_time():
    sr = _response_reader()
    ev = {
        "name": "analog_in_state",
        "oid": 7,
        "type": "response",
        "#receive_time_raw": 10.0,
        "#runtime_lane": "bulk",
    }
    delayed = sr._engine_handle_response_event(ev, 10.088)
    assert ev["#sent_time"] == 10.0
    assert ev["#receive_time"] == 10.0
    assert delayed["response"] == "analog_in_state"
    assert delayed["oid"] == 7
    assert delayed["lane"] == "bulk"
    assert math.isclose(delayed["lag_s"], 0.088)


def test_unstamped_clock_response_remains_invalid():
    sr = _response_reader()
    ev = {
        "name": "clock",
        "type": "response",
        "#receive_time_raw": 10.0,
        "#runtime_lane": "priority",
    }
    delayed = sr._engine_handle_response_event(ev, 10.001)
    assert delayed is None
    assert ev["#sent_time"] == 0.0
    assert ev["#receive_time"] == 10.0


def test_poller_reports_worst_response_lag_once():
    sr = _response_reader()
    sr._poller_expected_wake = 10.0
    sr._poller_stall_logged = False
    sr._engine_detached = False
    sr.reactor = SimpleNamespace(NEVER=999.0)
    events = iter(
        [
            {
                "name": "analog_in_state",
                "oid": 6,
                "type": "response",
                "#receive_time_raw": 9.95,
                "#runtime_lane": "bulk",
            },
            {
                "name": "buttons_state",
                "oid": 3,
                "type": "response",
                "#receive_time_raw": 9.97,
                "#runtime_lane": "bulk",
            },
            None,
        ]
    )
    sr.engine_mcu = SimpleNamespace(
        is_claimed=lambda: True, take_runtime_event=lambda: next(events)
    )
    with mock.patch("klippy.serialhdl.structured_log.event") as emit:
        assert math.isclose(sr._engine_event_poller(10.001), 10.002)
    emit.assert_called_once()
    fields = emit.call_args.kwargs
    assert fields["mcu"] == "bottom"
    assert fields["response"] == "analog_in_state"
    assert fields["max_lag_s"] == 0.051
    assert fields["delayed_count"] == 2
    assert fields["priority_drained"] == 0
    assert fields["bulk_drained"] == 2


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for fn in fns:
        fn()
        print("ok", fn.__name__)
    print("ALL PASS (%d)" % len(fns))
