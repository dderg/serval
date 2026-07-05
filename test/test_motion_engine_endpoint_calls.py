import pytest

from klippy.motion_engine import MotionEngineWrapper


class FakeReactor:
    def __init__(self):
        self._time = 100.0
        self.pauses = 0

    def monotonic(self):
        return self._time

    def pause(self, waketime):
        self.pauses += 1
        self._time = waketime


class FakeNativeEngine:
    def __init__(self, polls_until_done=0, error=None):
        self.polls_until_done = polls_until_done
        self.error = error
        self.started = []

    def set_torque_start(self, mcu_handle, value, print_time):
        self.started.append(("set_torque", mcu_handle, value, print_time))
        return 7

    def finalize_homed_axis_start(self, mcu_handle, axis, pos_mm):
        self.started.append(("finalize", mcu_handle, axis, pos_mm))
        return 8

    def endpoint_call_done(self, call_id):
        if self.polls_until_done > 0:
            self.polls_until_done -= 1
            return False
        if self.error is not None:
            raise RuntimeError(self.error)
        return True


def _make_wrapper(native):
    wrapper = MotionEngineWrapper.__new__(MotionEngineWrapper)
    wrapper._engine = native
    wrapper._reactor = FakeReactor()
    return wrapper


def test_set_torque_polls_with_reactor_pauses_until_done():
    native = FakeNativeEngine(polls_until_done=3)
    wrapper = _make_wrapper(native)
    wrapper.set_torque(1, True, 12.5)
    assert native.started == [("set_torque", 1, True, 12.5)]
    assert wrapper._reactor.pauses == 3, (
        "a pending endpoint call must yield to the reactor, not spin or block"
    )


def test_endpoint_call_error_propagates_to_the_caller():
    native = FakeNativeEngine(error="servo torque enable failed: result -312")
    wrapper = _make_wrapper(native)
    with pytest.raises(RuntimeError, match="-312"):
        wrapper.finalize_homed_axis(1, 0, [235.0, 0.0, 0.0])
