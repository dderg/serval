import sys
import types

_fake_native_mod = types.ModuleType("klippy.motion_bridge_native")


class _FakeMotionBridge:
    pass


_fake_native_mod.MotionBridge = _FakeMotionBridge
sys.modules.setdefault("klippy.motion_bridge_native", _fake_native_mod)

from klippy.motion_bridge import (  # noqa: E402
    _STUB_MOTION_METHODS,
    MotionBridgeWrapper,
)


class FakeNativeHandle:
    def __init__(self, return_value=None):
        self.calls = []
        self.return_value = return_value

    def submit_nudge(self, mcu_id, axis_idx, motor_mask, delta_mm, speed, accel):
        self.calls.append(
            ("submit_nudge", mcu_id, axis_idx, motor_mask, delta_mm, speed, accel)
        )
        return self.return_value


def make_wrapper(native_handle):
    wrapper = MotionBridgeWrapper.__new__(MotionBridgeWrapper)
    wrapper._bridge = native_handle
    return wrapper


def test_submit_nudge_forwards_verbatim():
    handle = FakeNativeHandle(return_value=42)
    wrapper = make_wrapper(handle)

    result = wrapper.submit_nudge(7, 1, 0b10, 0.3, 80.0, 5000.0)

    assert result == 42
    assert len(handle.calls) == 1
    assert handle.calls[0] == ("submit_nudge", 7, 1, 0b10, 0.3, 80.0, 5000.0)


def test_dead_forwarders_removed():
    assert not hasattr(MotionBridgeWrapper, "adjust_motor")
    assert not hasattr(MotionBridgeWrapper, "submit_correction_sequence")


def test_stub_motion_methods_updated():
    assert "submit_nudge" in _STUB_MOTION_METHODS
    assert "adjust_motor" not in _STUB_MOTION_METHODS
    assert "submit_correction_sequence" not in _STUB_MOTION_METHODS
