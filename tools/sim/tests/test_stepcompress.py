"""End-to-end coverage for stepping_mode: stepcompress.

The steppers live on the second MCU, built with CONFIG_CLASSIC_STEPPING=y
(tools/sim/configs/sc-sim.config), so the host computes every step time and
ships it over queue_step / set_next_step_dir / reset_step_clock. The piece
-mode case runs in the same file so a stepcompress regression cannot hide
behind a piece-mode pass.
"""

from __future__ import annotations

import pytest

from tools.sim import configs

pytestmark = pytest.mark.needs_elf

STEPS_PER_MM = configs.STEPCOMPRESS_STEPS_PER_MM


def _steps(world, axis: str) -> int:
    line = configs.STEPCOMPRESS_STEP_LINES[axis]
    resp = world.sim_control("f4").send(f"get_steps line={line}")
    if not resp.startswith("steps="):
        raise AssertionError(f"get_steps line={line}: {resp!r}")
    return int(resp.split()[0].split("=")[1])


def _boot(sim_world):
    return sim_world(
        lambda w: configs.stepcompress_config(
            w.h7_pty, w.f4_pty, str(w.gcode_dir)
        ),
        sc_mcu=True,
    )


def test_stepcompress_move_completes(sim_world):
    world = _boot(sim_world)
    world.gcode_ok("SET_KINEMATIC_POSITION X=125 Y=125 Z=125")
    world.gcode_ok("G1 X135 F3000")
    world.gcode_ok("M400")
    assert world.shutdown_line() is None, world.log_tail()
    pos = world.toolhead_position()
    assert pos[0] == pytest.approx(135.0, abs=0.01), pos


def test_stepcompress_step_count_matches_distance(sim_world):
    world = _boot(sim_world)
    world.gcode_ok("SET_KINEMATIC_POSITION X=125 Y=125 Z=125")
    world.gcode_ok("M400")
    world.sim_control("f4").reset_step_times(
        configs.STEPCOMPRESS_STEP_LINES["x"]
    )
    world.gcode_ok("G1 X145 F3000")
    world.gcode_ok("M400")
    assert world.shutdown_line() is None, world.log_tail()
    expected = 20.0 * STEPS_PER_MM
    assert _steps(world, "x") == pytest.approx(expected, abs=1.0)


def test_stepcompress_homing_trips(sim_world):
    world = _boot(sim_world)
    world.gcode_ok("SET_KINEMATIC_POSITION X=125 Y=125 Z=125")
    world.gcode_ok("G28 Z", timeout=120)
    world.gcode_ok("M400")
    assert world.shutdown_line() is None, world.log_tail()
    assert world.status()["toolhead"]["homed_axes"].lower().find("z") >= 0


def test_piece_mode_regression_beside_stepcompress(sim_world):
    world = sim_world(
        lambda w: configs.minimal_config(w.h7_pty, str(w.gcode_dir)),
        dual_mcu=False,
    )
    world.gcode_ok("SET_KINEMATIC_POSITION X=125 Y=125 Z=125")
    world.gcode_ok("G1 X135 F3000")
    world.gcode_ok("M400")
    assert world.shutdown_line() is None, world.log_tail()
    assert world.toolhead_position()[0] == pytest.approx(135.0, abs=0.01)
