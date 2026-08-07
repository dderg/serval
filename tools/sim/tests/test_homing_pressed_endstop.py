"""Endstop already pressed when G28 arms (issue #359 family).

An insta-trip carries a clock at or before the arm window and before any
recorded motion; the engine clamps its position to the run's start instead
of aborting, so the min_home_dist rehome path (or a plain seed) completes
the home — mainline parity for a carriage parked on its switch.
"""

from __future__ import annotations

import time

import pytest

from tools.sim import configs
from tools.sim.tests.test_homing_min_dist_corexy import (
    Y_ENDSTOP,
    EndstopWall,
    XyTracker,
    _needs_rehome_records,
)
from tools.sim.tests.test_homing_multi_endstop import AXES

pytestmark = pytest.mark.needs_elf


def test_pressed_endstop_recovers_via_rehome(sim_world):
    world = sim_world(
        lambda w: configs.awd_corexy_positive_dir_config(
            w.h7_pty, str(w.gcode_dir)
        ),
        dual_mcu=False,
    )
    control = world.sim_control("h7")
    tracker = XyTracker(control)

    world.mark_log()
    y0 = tracker.xy()[1]
    with EndstopWall(tracker, control, 1, Y_ENDSTOP, wall_mm=y0 - 0.2):
        time.sleep(0.3)
        resp = world.gcode("G28 Y", timeout=300)

    log = world.expect_log("needs rehome:", timeout=15.0)
    records = _needs_rehome_records(log, "Y")
    assert records and records[0][0], (
        f"a switch pressed at arm must read as traveled~0 and take the"
        f" rehome path: {records} (G28: {resp})"
    )
    assert records[0][1] < 1.0, f"insta-trip traveled must be ~0: {records}"
    assert not resp.get("error"), (
        f"pressed-at-arm homing must recover via the rehome pass: {resp}"
    )
    assert world.shutdown_line() is None


def test_multi_endstop_pressed_at_arm_completes(sim_world):
    world = sim_world(
        lambda w: configs.dual_motor_xy_config(w.h7_pty, str(w.gcode_dir))
    )
    control = world.sim_control("h7")
    _, _, endstops, _ = AXES["X"]
    for chip, line in endstops:
        control.set_gpio_input(chip, line, 1)
    try:
        resp = world.gcode("G28 X", timeout=120)
    finally:
        for chip, line in endstops:
            control.set_gpio_input(chip, line, 0)
    assert not resp.get("error"), (
        f"both switches pressed at arm: the run must seed at the endstop"
        f" instead of aborting position-unknown: {resp}"
    )
    pos = world.toolhead_position()
    assert pos[0] == pytest.approx(
        configs.DUAL_MOTOR_POSITION_ENDSTOP, abs=0.5
    ), pos
    assert world.shutdown_line() is None
