"""Dual-motor axes with one endstop switch per motor.

X and Y each drive two motors on a single kinematic lane, each motor
carrying its own switch (keyed `endstop_pin` form). The first switch to
trip must only freeze its own motor -- the lane keeps streaming steps --
and only the last trip of the group ends the homing move and seeds the
axis to position_endstop.

homing_retract_dist is 0 in the config, so each G28 is a single approach
and the switches can be driven by hand from the test.
"""

from __future__ import annotations

import threading
import time

import pytest

from tools.sim import configs

pytestmark = pytest.mark.needs_elf

# Runtime step-queue notification lines (src/linux/runtime_tick_host.c
# step_gpio_lines): cartesian X lane -> gpio18, Y lane -> gpio7.
X_LANE_LINE = 18
Y_LANE_LINE = 7

STEPS_PER_MM = (
    configs.DUAL_MOTOR_MICROSTEPS * 200 / configs.DUAL_MOTOR_ROTATION_DISTANCE
)
MOVING_STEPS = int(0.5 * STEPS_PER_MM)
POST_TRIP_WINDOW_S = 0.3
HOME_TIMEOUT_S = 300.0
POSITION_TOLERANCE_MM = 0.1
MAX_STOP_OVERSHOOT_MM = 1.0


def _boot(sim_world):
    world = sim_world(
        lambda w: configs.dual_motor_xy_config(w.h7_pty, str(w.gcode_dir)),
        dual_mcu=False,
    )
    return world, world.sim_control("h7")


def _lane_steps(control, line: int) -> int:
    resp = control.send(f"get_steps line={line}")
    if not resp.startswith("steps="):
        raise AssertionError(f"get_steps line={line}: {resp!r}")
    return int(resp.split()[0].split("=", 1)[1])


def _wait_until_moving(control, line: int, timeout: float = 60.0) -> None:
    start = _lane_steps(control, line)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if abs(_lane_steps(control, line) - start) >= MOVING_STEPS:
            return
        time.sleep(0.01)
    raise AssertionError(f"lane on line {line} never started stepping")


def _steps_advanced_over(control, line: int, window_s: float) -> int:
    start = _lane_steps(control, line)
    advanced = 0
    deadline = time.monotonic() + window_s
    while time.monotonic() < deadline:
        advanced = max(advanced, abs(_lane_steps(control, line) - start))
        time.sleep(0.01)
    return advanced


class _HomingThread(threading.Thread):
    def __init__(self, world, axis: str):
        super().__init__(daemon=True)
        self.world = world
        self.axis = axis
        self.response = None
        self.failure = None

    def run(self):
        try:
            self.response = self.world.gcode(
                f"G28 {self.axis}", timeout=HOME_TIMEOUT_S
            )
        except Exception as exc:
            self.failure = exc


def _home_with_staggered_trips(world, control, axis, line, first, last):
    homing = _HomingThread(world, axis)
    homing.start()
    try:
        _wait_until_moving(control, line)
        control.set_gpio_input(*first, 1)
        advanced = _steps_advanced_over(control, line, POST_TRIP_WINDOW_S)
        assert advanced >= MOVING_STEPS, (
            f"{axis} lane stopped stepping after only the first of its two"
            f" endstops tripped: {advanced} steps in {POST_TRIP_WINDOW_S}s"
        )
        assert homing.is_alive(), (
            f"G28 {axis} finished on the first endstop trip; the second"
            " switch was never touched"
        )
        control.set_gpio_input(*last, 1)
        homing.join(timeout=HOME_TIMEOUT_S)
    finally:
        control.set_gpio_input(*first, 0)
        control.set_gpio_input(*last, 0)
    assert not homing.is_alive(), f"G28 {axis} never returned"
    assert homing.failure is None, homing.failure
    assert homing.response and not homing.response.get("error"), homing.response
    assert world.shutdown_line() is None, world.log_tail()


def _assert_seeded_to_endstop(world, axis_index: int, label: str) -> None:
    position = world.status({"toolhead": None})["toolhead"]["position"]
    seeded = position[axis_index]
    endstop = configs.DUAL_MOTOR_POSITION_ENDSTOP
    overshoot = seeded - endstop
    assert -MAX_STOP_OVERSHOOT_MM < overshoot <= POSITION_TOLERANCE_MM, (
        f"{label} seeded to {seeded}: expected position_endstop {endstop}"
        f" minus a bounded stop-latency overshoot"
        f" (homing direction is negative, retract is 0)"
    )


def test_boot_and_query_reports_every_switch(sim_world):
    world, _control = _boot(sim_world)
    assert world.shutdown_line() is None, world.log_tail()

    world.gcode_ok("QUERY_ENDSTOPS")
    names = world.status({"query_endstops": None})["query_endstops"][
        "last_query"
    ]
    motors = configs.DUAL_MOTOR_X_MOTORS + configs.DUAL_MOTOR_Y_MOTORS
    for motor in motors:
        matches = [name for name in names if motor in name]
        assert len(matches) == 1, (
            f"expected exactly one endstop entry for motor {motor},"
            f" QUERY_ENDSTOPS reported {sorted(names)}"
        )
    assert any(name.startswith("z") for name in names), (
        f"the single-switch Z axis vanished from QUERY_ENDSTOPS: "
        f"{sorted(names)}"
    )


def test_staggered_trip_homes_x(sim_world):
    world, control = _boot(sim_world)
    _home_with_staggered_trips(
        world,
        control,
        "X",
        X_LANE_LINE,
        configs.DUAL_MOTOR_X_ENDSTOPS[0],
        configs.DUAL_MOTOR_X_ENDSTOPS[1],
    )
    _assert_seeded_to_endstop(world, 0, "X")


def test_staggered_trip_homes_y(sim_world):
    world, control = _boot(sim_world)
    _home_with_staggered_trips(
        world,
        control,
        "Y",
        Y_LANE_LINE,
        configs.DUAL_MOTOR_Y_ENDSTOPS[0],
        configs.DUAL_MOTOR_Y_ENDSTOPS[1],
    )
    _assert_seeded_to_endstop(world, 1, "Y")
