from fakes import FakeRail, FakeStepper, FakeStepperEnable, FakeToolhead

from klippy.extras import homing
from klippy.extras.stepper_enable import DISABLE_STALL_TIME


def test_group_enable_energizes_every_motor_at_one_print_time():
    th = FakeToolhead(last_move_time=100.0)
    se = FakeStepperEnable(
        toolhead=th, names=["motor_a", "motor_b", "motor_z"], real_methods=True
    )
    se.motor_enable_group(["motor_a", "motor_b", "motor_z"])
    times = [el.enabled_at for el in se.enable_lines.values()]
    assert all(t == times[0] for t in times), "all motors share one print_time"
    assert all(len(t) == 1 for t in times)
    move_time_calls = [c for c in th.calls if c[0] == "get_last_move_time"]
    assert len(move_time_calls) == 1, (
        "print_time sampled once for the whole group"
    )


def test_group_enable_dwells_once_around_the_batch():
    th = FakeToolhead(last_move_time=100.0)
    se = FakeStepperEnable(
        toolhead=th, names=["motor_a", "motor_b"], real_methods=True
    )
    se.motor_enable_group(["motor_a", "motor_b"])
    enabled_at = se.enable_lines["motor_a"].enabled_at[0]
    settles = [c for c in th.calls if c[0] in ("dwell", "dwell_through")]
    assert settles == [
        ("dwell", DISABLE_STALL_TIME),
        ("dwell_through", enabled_at + DISABLE_STALL_TIME),
    ], (
        "one settle before the batch, and motion held a settle past the"
        " shared enable after it, not per motor"
    )


def test_homing_collects_every_active_rail_motor():
    x = FakeRail(steppers=[FakeStepper(name="motor_a")], name="stepper_x")
    y = FakeRail(steppers=[FakeStepper(name="motor_b")], name="stepper_y")
    names = []
    for rail in (x, y):
        names.extend(homing._homing_motor_names(rail))
    assert names == ["motor_a", "motor_b"], "both gantry motors in one batch"


def test_homing_falls_back_to_rail_name_when_no_steppers():
    servo = FakeRail(steppers=[], name="servo_x")
    assert homing._homing_motor_names(servo) == ["servo_x"]
