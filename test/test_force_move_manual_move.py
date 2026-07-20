import pytest
from fakes import (
    FakeCommandError,
    FakeGcmd,
    FakePrinter,
    FakeStepper,
    FakeStepperEnable,
    FakeToolhead,
)

from klippy.extras.force_move import ForceMove


def make_force_move(enabled=True, return_toolhead=False, max_axis_accel=3000.0):
    toolhead = FakeToolhead(
        motor_binding=(0, 2, 1), max_axis_accel=max_axis_accel
    )
    stepper_enable = FakeStepperEnable(enabled=enabled)
    printer = FakePrinter(
        {"toolhead": toolhead, "stepper_enable": stepper_enable}
    )
    fm = ForceMove.__new__(ForceMove)
    fm.printer = printer
    if return_toolhead:
        return fm, toolhead
    return fm


def _last_nudge(toolhead):
    _, mcu_id, axis_idx, motor_idx, dist, speed, accel = toolhead.calls[-1]
    return dict(
        mcu_id=mcu_id,
        axis_idx=axis_idx,
        motor_idx=motor_idx,
        dist=dist,
        speed=speed,
        accel=accel,
    )


def test_manual_move_raises_when_motor_disabled():
    fm = make_force_move(enabled=False)
    with pytest.raises(FakeCommandError):
        fm.manual_move("stepper_z1", 0.5, 5.0, 100.0)


def test_manual_move_forwards_to_submit_nudge_when_enabled():
    fm, toolhead = make_force_move(enabled=True, return_toolhead=True)
    fm.manual_move("stepper_z1", 0.5, 5.0, 100.0)
    assert _last_nudge(toolhead) == dict(
        mcu_id=0, axis_idx=2, motor_idx=1, dist=0.5, speed=5.0, accel=100.0
    )


def test_manual_move_defaults_accel_when_zero():
    fm, toolhead = make_force_move(
        enabled=True, return_toolhead=True, max_axis_accel=3000.0
    )
    fm.manual_move("stepper_z1", 0.5, 5.0, 0.0)
    assert _last_nudge(toolhead)["accel"] == pytest.approx(3000.0)


def test_manual_move_accepts_stepper_object():
    fm, toolhead = make_force_move(enabled=True, return_toolhead=True)
    fm.manual_move(FakeStepper(name="stepper_z1"), 0.5, 5.0, 100.0)
    assert _last_nudge(toolhead)["motor_idx"] == 1


def test_manual_move_passes_negative_accel_through():
    fm, toolhead = make_force_move(enabled=True, return_toolhead=True)
    fm.manual_move("stepper_z1", 0.5, 5.0, -10.0)
    assert _last_nudge(toolhead)["accel"] == pytest.approx(-10.0)


def test_cmd_force_move_parses_and_calls_manual_move():
    fm, toolhead = make_force_move(enabled=True, return_toolhead=True)
    gcmd = FakeGcmd(
        {
            "STEPPER": "stepper_z1",
            "DISTANCE": 1.0,
            "VELOCITY": 5.0,
            "ACCEL": 100.0,
        }
    )
    fm.cmd_FORCE_MOVE(gcmd)
    nudge = _last_nudge(toolhead)
    assert nudge["dist"] == pytest.approx(1.0)
    assert nudge["speed"] == pytest.approx(5.0)
    assert nudge["accel"] == pytest.approx(100.0)
    assert nudge["motor_idx"] == 1


def test_cmd_force_move_defaults_accel_when_omitted():
    fm, toolhead = make_force_move(
        enabled=True, return_toolhead=True, max_axis_accel=3000.0
    )
    gcmd = FakeGcmd({"STEPPER": "stepper_z1", "DISTANCE": 0.5, "VELOCITY": 5.0})
    fm.cmd_FORCE_MOVE(gcmd)
    assert _last_nudge(toolhead)["accel"] == pytest.approx(3000.0)
