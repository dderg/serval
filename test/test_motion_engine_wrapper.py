from klippy.motion_engine import (
    _STUB_MOTION_METHODS,
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


def make_wrapper(native_handle):
    wrapper = MotionEngineWrapper.__new__(MotionEngineWrapper)
    wrapper._engine = native_handle
    return wrapper


def test_submit_nudge_forwards_verbatim():
    handle = FakeNativeHandle(return_value=42)
    wrapper = make_wrapper(handle)

    result = wrapper.submit_nudge(7, 1, 0b10, 0.3, 80.0, 5000.0)

    assert result == 42
    assert len(handle.calls) == 1
    assert handle.calls[0] == ("submit_nudge", 7, 1, 0b10, 0.3, 80.0, 5000.0)


def test_dead_forwarders_removed():
    assert not hasattr(MotionEngineWrapper, "adjust_motor")
    assert not hasattr(MotionEngineWrapper, "submit_correction_sequence")


def test_stub_motion_methods_updated():
    assert "submit_nudge" in _STUB_MOTION_METHODS
    assert "adjust_motor" not in _STUB_MOTION_METHODS
    assert "submit_correction_sequence" not in _STUB_MOTION_METHODS


def test_dispatched_lead_secs_forwards_and_coerces_none():
    # The host feed pacing (motion._submit_paced) calls this on the wrapper.
    # A missing forwarder raises AttributeError on real hardware — the wrapper
    # only forwards native calls it explicitly defines (no __getattr__).
    handle = FakeNativeHandle(return_value=0.72)
    wrapper = make_wrapper(handle)
    assert wrapper.dispatched_lead_secs() == 0.72
    assert handle.calls == [("dispatched_lead_secs",)]

    none_handle = FakeNativeHandle(return_value=None)
    assert make_wrapper(none_handle).dispatched_lead_secs() == 0.0


def test_stub_engine_answers_gate_accessors():
    # The gate must not crash under the config-only stub: every accessor it reads
    # returns a number, never None (which would TypeError the watermark compare).
    stub = _StubEngine()
    assert stub.dispatched_lead_secs() == 0.0
    assert stub.queued_motion_secs() == 0.0
