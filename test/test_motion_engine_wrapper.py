from klippy.motion_engine import (
    _STUB_MOTION_METHODS,
    MotionEngineWrapper,
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
