import pytest

from klippy.extras.stepper_enable import EnableTracking, StepperEnablePin
from klippy.mcu import MIN_SCHEDULE_LEAD
from klippy.motion import Motion

MOTION_LEAD = 0.25
TMC_ENABLE_WORK_SECS = 0.170


class FakeStepper:
    def __init__(self, name):
        self._name = name
        self._active_callbacks = []

    def add_active_callback(self, cb):
        self._active_callbacks.append(cb)

    def get_name(self, short=False):
        return self._name


class FakeRail:
    def __init__(self, steppers):
        self._steppers = steppers

    def get_steppers(self):
        return self._steppers


class FakeKin:
    def __init__(self, rails):
        self.rails = rails

    def active_rails(self, dx, dy, dz):
        return self.rails


class FakeToolhead:
    _fire_active_callbacks = Motion._fire_active_callbacks

    def __init__(self, kin):
        self.kin = kin
        self.follower_steppers = []
        self.clock = 1000.0

    def get_last_move_time(self):
        return self.clock + MOTION_LEAD


class FakeMcuDigitalOut:
    def __init__(self, toolhead, name, fail_first_send=False):
        self._toolhead = toolhead
        self._name = name
        self._fail_first_send = fail_first_send
        self.set_at = []

    def set_digital(self, print_time, value):
        if self._fail_first_send:
            self._fail_first_send = False
            raise RuntimeError("send failure on %s" % self._name)
        lead = print_time - self._toolhead.clock
        if lead < MIN_SCHEDULE_LEAD:
            raise RuntimeError(
                "digital_out %s scheduled with stale print_time:"
                " lead=%.1fms" % (self._name, lead * 1000.0)
            )
        self.set_at.append((print_time, value))


def make_z_gantry(toolhead_kin_pins=3, failing_pin=None):
    steppers = [FakeStepper("motor_z%d" % i) for i in range(toolhead_kin_pins)]
    th = FakeToolhead(FakeKin([FakeRail(steppers)]))
    pins = []
    for i, s in enumerate(steppers):
        mcu_pin = FakeMcuDigitalOut(
            th, s.get_name(), fail_first_send=(i == failing_pin)
        )
        tracking = EnableTracking(s, StepperEnablePin(mcu_pin, 0))

        def slow_tmc_enable_work(print_time, is_enable, th=th):
            th.clock += TMC_ENABLE_WORK_SECS

        tracking.register_state_callback(slow_tmc_enable_work)
        pins.append(mcu_pin)
    return th, pins


def test_slow_enable_callbacks_do_not_erode_schedule_lead():
    th, pins = make_z_gantry()
    th._fire_active_callbacks((0.0, 0.0, 15.0, 0.0))
    assert all(pin.set_at for pin in pins), (
        "every gantry motor must energize even when earlier motors' TMC"
        " enable work consumes wall time"
    )
    times = [pin.set_at[0][0] for pin in pins]
    assert len(set(times)) == 1, "all motors energize at one print_time"


def test_failed_enable_callback_is_rearmed_for_the_next_move():
    th, pins = make_z_gantry(failing_pin=2)
    with pytest.raises(RuntimeError, match="send failure"):
        th._fire_active_callbacks((0.0, 0.0, 15.0, 0.0))
    assert not pins[2].set_at
    th._fire_active_callbacks((0.0, 0.0, 15.0, 0.0))
    assert pins[2].set_at, (
        "a motor whose enable failed must be retried on the next move,"
        " not silently left de-energized"
    )
