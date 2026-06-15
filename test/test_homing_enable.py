from klippy.extras import homing, stepper_enable
from klippy.extras.stepper_enable import DISABLE_STALL_TIME


class FakeEnableLine:
    def __init__(self):
        self.enabled_at = []

    def motor_enable(self, print_time):
        self.enabled_at.append(print_time)


class FakeToolhead:
    def __init__(self):
        self.dwells = []
        self._t = 100.0
        self.move_time_calls = 0

    def dwell(self, delay):
        self.dwells.append(delay)
        self._t += delay

    def get_last_move_time(self):
        self.move_time_calls += 1
        return self._t

    def resync_parked_servos(self):
        pass


class FakePrinter:
    def __init__(self, toolhead):
        self._toolhead = toolhead

    def lookup_object(self, name):
        assert name == "toolhead"
        return self._toolhead


class FakeStepperEnable:
    motor_enable_group = stepper_enable.PrinterStepperEnable.motor_enable_group

    def __init__(self, toolhead, names):
        self.printer = FakePrinter(toolhead)
        self.enable_lines = {n: FakeEnableLine() for n in names}


def test_group_enable_energizes_every_motor_at_one_print_time():
    th = FakeToolhead()
    se = FakeStepperEnable(th, ["motor_a", "motor_b", "motor_z"])
    se.motor_enable_group(["motor_a", "motor_b", "motor_z"])
    times = [el.enabled_at for el in se.enable_lines.values()]
    assert all(t == times[0] for t in times), "all motors share one print_time"
    assert all(len(t) == 1 for t in times)
    assert th.move_time_calls == 1, (
        "print_time sampled once for the whole group"
    )


def test_group_enable_dwells_once_around_the_batch():
    th = FakeToolhead()
    se = FakeStepperEnable(th, ["motor_a", "motor_b"])
    se.motor_enable_group(["motor_a", "motor_b"])
    assert th.dwells == [DISABLE_STALL_TIME, DISABLE_STALL_TIME], (
        "one settle dwell before and after, not per motor"
    )


class FakeStepper:
    def __init__(self, name):
        self._name = name

    def get_name(self, short=False):
        return self._name


class FakeRail:
    def __init__(self, steppers, name):
        self._steppers = steppers
        self._name = name

    def get_steppers(self):
        return self._steppers

    def get_name(self, short=False):
        return self._name


def test_homing_collects_every_active_rail_motor():
    x = FakeRail([FakeStepper("motor_a")], "stepper_x")
    y = FakeRail([FakeStepper("motor_b")], "stepper_y")
    names = []
    for rail in (x, y):
        names.extend(homing._homing_motor_names(rail))
    assert names == ["motor_a", "motor_b"], "both gantry motors in one batch"


def test_homing_falls_back_to_rail_name_when_no_steppers():
    servo = FakeRail([], "servo_x")
    assert homing._homing_motor_names(servo) == ["servo_x"]
