import pytest
from fakes import FakeStepper, FakeToolhead

from klippy.extras.motors_sync import StepperManualMove


def make_stepper_manual_move():
    toolhead = FakeToolhead(motor_binding=(0, 0, 1))
    smm = StepperManualMove.__new__(StepperManualMove)
    smm.toolhead = toolhead
    smm.travel_speed = 50.0
    smm.travel_accel = 2000.0
    return smm, toolhead


def test_manual_move_loops_submit_nudge_per_segment_and_filters_zero():
    smm, toolhead = make_stepper_manual_move()
    stepper = FakeStepper("stepper_x1")
    smm.manual_move(stepper, [0.1, -0.1, 0.0])
    calls = [c for c in toolhead.calls if c[0] == "submit_nudge"]
    assert len(calls) == 2
    assert calls[0][4] == pytest.approx(0.1)
    assert calls[1][4] == pytest.approx(-0.1)
    assert all((c[1], c[2], c[3]) == (0, 0, 1) for c in calls)
    assert all(
        c[5] == smm.travel_speed and c[6] == smm.travel_accel for c in calls
    )
    assert [c for c in toolhead.calls if c[0] == "flush_step_generation"] == [
        ("flush_step_generation",)
    ]


def test_manual_move_noop_when_all_filtered():
    smm, toolhead = make_stepper_manual_move()
    smm.manual_move(FakeStepper("stepper_x1"), [0.0, 0.000001])
    assert toolhead.calls == []
