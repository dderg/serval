"""Extruder as a lone follower lane on a second stepping_mode: stepcompress MCU.

Mirrors the Voron 0 CAN-toolhead bench: spatial motors on the main MCU,
`[motor extruder]` alone on a stepcompress MCU, so that MCU's axis list is
`[3]` and never starts at lane 0. The single-MCU stepcompress worlds all
carry axes `[0, 1, 2]`, where a motor-indexed retirement report happens to
coincide with the axis index and hides the mismatch.
"""

from __future__ import annotations

import pytest

from tools.sim import configs

pytestmark = pytest.mark.needs_elf

STEPS_PER_MM = configs.STEPCOMPRESS_EXTRUDER_STEPS_PER_MM
STEP_LINE = configs.STEPCOMPRESS_EXTRUDER_STEP_LINE


def _extruder_steps(world) -> int:
    resp = world.sim_control("f4").send(f"get_steps line={STEP_LINE}")
    if not resp.startswith("steps="):
        raise AssertionError(f"get_steps line={STEP_LINE}: {resp!r}")
    return int(resp.split()[0].split("=")[1])


def _boot(sim_world):
    return sim_world(
        lambda w: configs.stepcompress_extruder_config(
            w.h7_pty, w.f4_pty, str(w.gcode_dir)
        ),
        sc_mcu=True,
    )


def _assert_alive(world):
    assert world.shutdown_line() is None, world.log_tail()
    events = world.events_text()
    assert "pump_retirement_stall_fatal" not in events, (
        "the extruder lane's pieces never retired — the stepcompress MCU's"
        " retirement report did not reach axis 3\n" + world.log_tail()
    )


def test_stepcompress_extruder_pure_extrude(sim_world):
    """No spatial motion at all: the only lane producing pieces is the
    follower extruder on the stepcompress MCU."""
    world = _boot(sim_world)
    world.gcode_ok("SET_KINEMATIC_POSITION X=125 Y=125 Z=125")
    world.gcode_ok("M400")
    before = _extruder_steps(world)
    world.gcode_ok("M83")
    world.gcode_ok("G1 E5 F300", timeout=120)
    world.gcode_ok("M400", timeout=120)
    _assert_alive(world)
    moved = abs(_extruder_steps(world) - before)
    assert moved == pytest.approx(5.0 * STEPS_PER_MM, abs=2.0), moved


def test_stepcompress_extruder_extrudes_after_xy_home(sim_world):
    """The bench sequence: XY homing runs while the extruder lane is idle
    (its MCU acks barriers fine), then a pure extrude stalls."""
    world = _boot(sim_world)
    world.gcode_ok("SET_KINEMATIC_POSITION X=125 Y=125 Z=125")
    world.gcode_ok("G28 X", timeout=180)
    world.gcode_ok("M400", timeout=60)
    before = _extruder_steps(world)
    world.gcode_ok("M83")
    world.gcode_ok("G1 E5 F300", timeout=120)
    world.gcode_ok("M400", timeout=120)
    _assert_alive(world)
    moved = abs(_extruder_steps(world) - before)
    assert moved == pytest.approx(5.0 * STEPS_PER_MM, abs=2.0), moved


def test_stepcompress_extruder_moves_with_xy(sim_world):
    """A coordinated move: pieces flow to the spatial MCU and the
    stepcompress extruder MCU in the same segments."""
    world = _boot(sim_world)
    world.gcode_ok("SET_KINEMATIC_POSITION X=125 Y=125 Z=125")
    world.gcode_ok("M400")
    before = _extruder_steps(world)
    world.gcode_ok("M83")
    world.gcode_ok("G1 X135 E2 F1800", timeout=120)
    world.gcode_ok("M400", timeout=120)
    _assert_alive(world)
    assert world.toolhead_position()[0] == pytest.approx(135.0, abs=0.01)
    moved = abs(_extruder_steps(world) - before)
    assert moved == pytest.approx(2.0 * STEPS_PER_MM, abs=2.0), moved
