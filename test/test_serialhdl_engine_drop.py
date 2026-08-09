import os
import sys

sys.path.insert(
    0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
)

from fakes import FakeEngine, FakePrinter  # noqa: E402

from klippy.engine_mcu import EngineMcu  # noqa: E402
from klippy.serialhdl import (  # noqa: E402
    CommandDelivery,
    EngineCommandChannel,
    error,
)

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


def test_background_delivery_reaches_engine_explicitly():
    engine = FakeEngine()
    sr = _reader(engine)
    sr.send_args("neopixel_update", [], delivery=CommandDelivery.BACKGROUND)
    assert engine.calls[-1] == (
        "engine_send_args",
        7,
        "neopixel_update",
        [],
        {
            "delivery": int(CommandDelivery.BACKGROUND),
            "min_clock": 0,
            "req_clock": 0,
        },
    )


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for fn in fns:
        fn()
        print("ok", fn.__name__)
    print("ALL PASS (%d)" % len(fns))
