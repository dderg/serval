import pytest

from klippy.motion_engine import (
    _STUB_NOOP_METHODS,
    MotionEngineWrapper,
    _StubEngine,
)


class FakeNativeHandle:
    def __init__(self, return_value=None):
        self.calls = []
        self.return_value = return_value

    def submit_nudge(
        self, mcu_id, axis_idx, motor_mask, delta_mm, speed, accel
    ):
        self.calls.append(
            (
                "submit_nudge",
                mcu_id,
                axis_idx,
                motor_mask,
                delta_mm,
                speed,
                accel,
            )
        )
        return self.return_value

    def dispatched_lead_secs(self):
        self.calls.append(("dispatched_lead_secs",))
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


def test_getattr_delegates_to_native_verbatim():
    handle = FakeNativeHandle(return_value=42)
    wrapper = make_wrapper(handle)

    result = wrapper.submit_nudge(7, 1, 0b10, 0.3, 80.0, 5000.0)

    assert result == 42
    assert len(handle.calls) == 1
    assert handle.calls[0] == ("submit_nudge", 7, 1, 0b10, 0.3, 80.0, 5000.0)


def test_getattr_does_not_delegate_private_names():
    wrapper = make_wrapper(FakeNativeHandle())
    with pytest.raises(AttributeError):
        wrapper._not_a_real_attribute


def test_dispatched_lead_secs_forwards_and_coerces_none():
    # The host feed pacing (motion._submit_paced) calls this on the wrapper;
    # the explicit override coerces a native None to 0.0 for the watermark
    # compare.
    handle = FakeNativeHandle(return_value=0.72)
    wrapper = make_wrapper(handle)
    assert wrapper.dispatched_lead_secs() == 0.72
    assert handle.calls == [("dispatched_lead_secs",)]

    none_handle = FakeNativeHandle(return_value=None)
    assert make_wrapper(none_handle).dispatched_lead_secs() == 0.0


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


def test_stub_engine_answers_gate_accessors():
    # The gate must not crash under the config-only stub: every accessor it
    # reads returns a number, never None (which would TypeError the watermark
    # compare).
    stub = _StubEngine()
    assert stub.dispatched_lead_secs() == 0.0
    assert stub.queued_motion_secs() == 0.0


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
