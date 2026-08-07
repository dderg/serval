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


def test_pressed_endstop_seeds_without_min_home_dist(sim_world):
    """With the early-trigger guard off, a pressed switch has no rehome pass
    to fall back on: the insta-trip must reconstruct to the run's start and
    seed the axis at position_endstop instead of aborting position-unknown."""
    world = sim_world(
        lambda w: configs.awd_corexy_positive_dir_config(
            w.h7_pty, str(w.gcode_dir)
        ).replace("min_home_dist: 10", "min_home_dist: 0"),
        dual_mcu=False,
    )
    control = world.sim_control("h7")
    world.mark_log()

    control.set_gpio_input(*Y_ENDSTOP, 1)
    try:
        resp = world.gcode("G28 Y", timeout=300)
    finally:
        control.set_gpio_input(*Y_ENDSTOP, 0)

    assert not resp.get("error"), (
        f"a switch pressed at arm must seed at the endstop instead of"
        f" aborting position-unknown: {resp}"
    )
    log = world.expect_log("homing: Y trigger=", timeout=15.0)
    assert "homing: Y trigger=300.0000" in log, log[-2000:]
    pos = world.toolhead_position()
    assert pos[1] == pytest.approx(295.0, abs=0.5), (
        f"seeded at position_endstop 300 then retracted homing_retract_dist"
        f" 5mm: {pos}"
    )
    assert world.shutdown_line() is None
