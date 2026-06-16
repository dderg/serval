import os
import sys

sys.path.insert(
    0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
)

from klippy.serialhdl import SerialReader, error  # noqa: E402

TRANSPORT_CLOSED = RuntimeError("engine_send: transport closed")


class FakeEngine:
    def __init__(self, exc=None):
        self.exc = exc
        self.sent = []
        self.calls = []
        self.clock_polls = 0

    def engine_send(self, handle, msg):
        if self.exc is not None:
            raise self.exc
        self.sent.append((handle, msg))

    def engine_call(self, handle, msg, response):
        if self.exc is not None:
            raise self.exc
        self.calls.append((handle, msg, response))
        return {}

    def engine_get_clock_async(self, handle):
        if self.exc is not None:
            raise self.exc
        self.clock_polls += 1


class FakeMcu:
    def __init__(self, engine):
        self._motion_engine = engine
        self._engine_handle = 7


def _reader(engine):
    sr = SerialReader.__new__(SerialReader)
    sr.mcu = FakeMcu(engine)
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
    sr = _reader(FakeEngine(exc=TRANSPORT_CLOSED))
    sr.send(b"x")
    assert sr._engine_detached is True


def test_send_reraises_unrelated_runtime_error():
    sr = _reader(FakeEngine(exc=RuntimeError("boom")))
    try:
        sr.send(b"x")
        raise AssertionError("expected RuntimeError")
    except RuntimeError as e:
        assert "boom" in str(e)
    assert sr._engine_detached is False


def test_send_with_response_raises_mainline_error_on_drop():
    sr = _reader(FakeEngine(exc=TRANSPORT_CLOSED))
    try:
        sr.send_with_response(b"x", "resp")
        raise AssertionError("expected serialhdl.error")
    except error:
        pass
    assert sr._engine_detached is True


def test_engine_get_clock_async_swallows_drop():
    sr = _reader(FakeEngine(exc=TRANSPORT_CLOSED))
    sr.engine_get_clock_async()
    assert sr._engine_detached is True


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for fn in fns:
        fn()
        print("ok", fn.__name__)
    print("ALL PASS (%d)" % len(fns))
