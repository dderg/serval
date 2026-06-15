import pytest

from klippy.extras.motors_sync import StepperManualMove


class FakeStepper:
    def __init__(self, name):
        self._name = name

    def get_name(self):
        return self._name


class FakeToolhead:
    def __init__(self):
        self.nudge_calls = []
        self.wait_moves_count = 0

    def get_motor_binding(self, name):
        return (0, 0, 1)

    def submit_nudge(self, mcu_id, axis_idx, motor_idx, dist, speed, accel):
        self.nudge_calls.append(
            dict(
                mcu_id=mcu_id,
                axis_idx=axis_idx,
                motor_idx=motor_idx,
                dist=dist,
                speed=speed,
                accel=accel,
            )
        )

    def wait_moves(self):
        self.wait_moves_count += 1


def make_stepper_manual_move():
    toolhead = FakeToolhead()
    smm = StepperManualMove.__new__(StepperManualMove)
    smm.toolhead = toolhead
    smm.travel_speed = 50.0
    smm.travel_accel = 2000.0
    return smm, toolhead


def test_manual_move_loops_submit_nudge_per_segment_and_filters_zero():
    smm, toolhead = make_stepper_manual_move()
    stepper = FakeStepper("stepper_x1")
    smm.manual_move(stepper, [0.1, -0.1, 0.0])
    calls = toolhead.nudge_calls
    assert len(calls) == 2
    assert calls[0]["dist"] == pytest.approx(0.1)
    assert calls[1]["dist"] == pytest.approx(-0.1)
    assert all(
        (c["mcu_id"], c["axis_idx"], c["motor_idx"]) == (0, 0, 1) for c in calls
    )
    assert all(
        c["speed"] == smm.travel_speed and c["accel"] == smm.travel_accel
        for c in calls
    )
    assert toolhead.wait_moves_count == 1


def test_manual_move_noop_when_all_filtered():
    smm, toolhead = make_stepper_manual_move()
    smm.manual_move(FakeStepper("stepper_x1"), [0.0, 0.000001])
    assert toolhead.nudge_calls == []
    assert toolhead.wait_moves_count == 0
