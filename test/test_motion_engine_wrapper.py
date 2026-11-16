import pytest
from fakes import FakeCommandError, FakePrinter

from klippy.motion_engine import (
    _STUB_NOOP_METHODS,
    PIPELINE_REQUEST_POLL_S,
    MotionEngineWrapper,
    _StubEngine,
)


class FakeNativeHandle:
    def __init__(self, return_value=None):
        self.calls = []
        self.return_value = return_value

    def submit_move(self, dx, dy, dz, de, feedrate):
        self.calls.append(("submit_move", dx, dy, dz, de, feedrate))
        return self.return_value

    def motion_state_at_clock(self, mcu_handle, clock, host_now, axis):
        self.calls.append(
            ("motion_state_at_clock", mcu_handle, clock, host_now, axis)
        )
        return self.return_value


def make_wrapper(native_handle):
    wrapper = MotionEngineWrapper.__new__(MotionEngineWrapper)
    wrapper._engine = native_handle
    return wrapper


class FakeMcu:
    def get_clocksync(self):
        return self

    def print_time_to_clock(self, print_time):
        return print_time * 1000

    def get_engine_handle(self):
        return 7


class FakeReactor:
    def monotonic(self):
        return 12.5


class PipelineNative:
    def __init__(self, served_on_poll=1, full_tries=0):
        self.calls = []
        self.served_on_poll = served_on_poll
        self.full_tries = full_tries
        self.polls = 0

    def dwell_start(self, duration_s):
        self.calls.append(("dwell_start", duration_s))

    def clear_bed_mesh_start(self):
        self.calls.append(("clear_bed_mesh_start",))
        return 0.25

    def pipeline_request_poll(self):
        self.polls += 1
        self.calls.append(("pipeline_request_poll", self.polls))
        return self.polls >= self.served_on_poll

    def try_update_post_processor(self, name, key, value):
        self.calls.append(("try_update_post_processor", name, key, value))
        if self.full_tries:
            self.full_tries -= 1
            return False
        return True


def make_waiting_wrapper(native, printer):
    wrapper = make_wrapper(native)
    wrapper._printer = printer
    return wrapper


def test_getattr_delegates_to_native_verbatim():
    handle = FakeNativeHandle(return_value=42)
    wrapper = make_wrapper(handle)

    result = wrapper.submit_move(1.0, 2.0, 0.0, 0.1, 80.0)

    assert result == 42
    assert len(handle.calls) == 1
    assert handle.calls[0] == ("submit_move", 1.0, 2.0, 0.0, 0.1, 80.0)


def test_dwell_yields_to_the_reactor_until_the_pipeline_serves_it():
    native = PipelineNative(served_on_poll=3)
    printer = FakePrinter()

    make_waiting_wrapper(native, printer).submit_dwell(0.5)

    assert native.calls == [
        ("dwell_start", 0.5),
        ("pipeline_request_poll", 1),
        ("pipeline_request_poll", 2),
        ("pipeline_request_poll", 3),
    ]
    assert printer.reactor.pauses == pytest.approx(
        [PIPELINE_REQUEST_POLL_S, 2 * PIPELINE_REQUEST_POLL_S]
    )


def test_mesh_clear_returns_its_rebase_once_the_pipeline_adopted_it():
    native = PipelineNative(served_on_poll=2)
    wrapper = make_waiting_wrapper(native, FakePrinter())

    assert wrapper.clear_bed_mesh() == 0.25
    assert native.calls[-1] == ("pipeline_request_poll", 2)


def test_shutdown_aborts_a_dwell_the_pipeline_has_not_served():
    native = PipelineNative(served_on_poll=float("inf"))
    wrapper = make_waiting_wrapper(native, FakePrinter(shutdown=True))

    with pytest.raises(FakeCommandError, match="dwell: shutdown while"):
        wrapper.submit_dwell(0.5)


def test_post_processor_update_retries_while_the_pipe_is_full():
    native = PipelineNative(full_tries=2)
    printer = FakePrinter()

    make_waiting_wrapper(native, printer).update_post_processor(
        "pa", "smooth_time", 0.02
    )

    update = ("try_update_post_processor", "pa", "smooth_time", 0.02)
    assert native.calls.count(update) == 3
    assert len(printer.reactor.pauses) == 2


def test_getattr_does_not_delegate_private_names():
    wrapper = make_wrapper(FakeNativeHandle())
    with pytest.raises(AttributeError):
        wrapper._not_a_real_attribute


def test_stub_engine_raises_on_motion_methods():
    stub = _StubEngine()
    with pytest.raises(RuntimeError, match="_motion_engine not built"):
        stub.submit_move(1.0, 0.0, 0.0, 0.0, 100.0)
    with pytest.raises(RuntimeError, match="_motion_engine not built"):
        stub.init_planner()
    with pytest.raises(RuntimeError, match="_motion_engine not built"):
        stub.some_future_engine_method()


def test_stub_engine_noops_lifecycle_helpers():
    stub = _StubEngine()
    for name in _STUB_NOOP_METHODS:
        assert getattr(stub, name)() is None


def test_motion_state_query_for_one_axis_filters_the_native_request():
    handle = FakeNativeHandle(return_value={"e": (3.0, 0.0, 0.0)})
    wrapper = make_wrapper(handle)
    wrapper._reactor = FakeReactor()

    assert wrapper.motion_state_at(FakeMcu(), print_time=4.25, axis="e") == {
        "e": (3.0, 0.0, 0.0)
    }
    assert handle.calls == [("motion_state_at_clock", 7, 4250, 12.5, 3)]


def test_motion_state_query_rejects_an_unknown_axis():
    wrapper = make_wrapper(FakeNativeHandle())
    wrapper._reactor = FakeReactor()

    with pytest.raises(ValueError, match="axis must be one of"):
        wrapper.motion_state_at(FakeMcu(), print_time=4.25, axis="q")
