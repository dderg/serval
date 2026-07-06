"""TMC5160 phase stepping: printing on a phase-stepped axis, and
sensorless (StallGuard virtual endstop) homing with the mode switch."""

import pytest

from tools.sim import configs
from tools.sim.world import EndstopPulser

pytestmark = pytest.mark.needs_elf


def test_phase_stepped_axis_prints(sim_world):
    world = sim_world(
        lambda w: configs.phase_stepping_config(w.h7_pty, str(w.gcode_dir)),
        dual_mcu=False,
    )
    gcode = world.gcode_dir / "phase_test.gcode"
    gcode.write_text(configs.PHASE_TEST_GCODE)
    print_time = world.print_file(gcode, timeout=300)
    assert print_time > 0
    assert world.shutdown_line() is None


def test_sensorless_homing_switches_phase_mode(sim_world):
    world = sim_world(
        lambda w: configs.sensorless_phase_config(w.h7_pty, str(w.gcode_dir)),
        dual_mcu=False,
    )
    with EndstopPulser(world.sim_control("h7"), [(0, 203)]):
        world.gcode_ok("G28 Z", timeout=120)

    log = world.klippy_log_text()
    assert world.shutdown_line() is None

    enter_marker = "phase mode entered"
    exit_marker = "phase mode exited (pulse stepping)"
    first_enter = log.find(enter_marker)
    exit_idx = log.find(exit_marker)
    re_enter = log.find(
        enter_marker, exit_idx + len(exit_marker) if exit_idx >= 0 else 0
    )
    assert first_enter >= 0, (
        "phase mode never entered (post-enable callback did not run)"
    )
    assert exit_idx >= 0, (
        "phase mode never exited around the StallGuard trip move"
    )
    assert first_enter < exit_idx < re_enter, (
        f"phase mode enter/exit/re-enter out of order "
        f"(enter={first_enter} exit={exit_idx} re-enter={re_enter})"
    )

    toolhead = world.status().get("toolhead", {})
    assert "z" in toolhead.get("homed_axes", "")
    pos = toolhead.get("position", [])
    assert len(pos) > 2 and abs(pos[2]) <= 6.0, (
        f"homed Z {pos} too far from position_endstop=0 "
        "(wall=50 steps + retract tolerance)"
    )
