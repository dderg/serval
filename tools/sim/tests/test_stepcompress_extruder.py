"""Extruder as a lone follower lane on a second, step/dir only MCU.

Mirrors the Voron 0 CAN-toolhead bench: spatial motors on the main MCU,
`[motor extruder]` alone on that MCU, so its axis list is
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


def _extrude_then_reanchor_and_home(world):
    world.gcode_ok("SET_KINEMATIC_POSITION X=125 Y=125 Z=125")
    world.gcode_ok("M83")
    world.gcode_ok("G1 E15 F300", timeout=180)
    world.gcode_ok("G1 E-2 F600", timeout=120)
    world.gcode_ok("M400", timeout=120)
    before = _extruder_steps(world)
    world.gcode_ok("SET_KINEMATIC_POSITION X=60 Y=60 Z=20")
    world.gcode_ok("G28 X", timeout=180)
    world.gcode_ok("M400", timeout=60)
    _assert_alive(world)
    assert "StepRateExceeded" not in world.events_text(), world.log_tail()
    held = abs(_extruder_steps(world) - before)
    assert held <= 2, (
        f"the spatial re-anchor moved the extruder lane by {held} steps"
        f" ({held / STEPS_PER_MM:.4f} mm)\n" + world.log_tail()
    )


def test_stepcompress_extruder_home_after_extrude(sim_world):
    """A spatial re-anchor (SET_KINEMATIC_POSITION, then G28) after a net
    extrude: the follower lane's shim seed and the stream odometer must land
    on the same origin, or the first piece after the re-anchor demands the
    whole extruded distance in one sample."""
    world = _boot(sim_world)
    _extrude_then_reanchor_and_home(world)


def test_stepcompress_extruder_home_after_force_move_and_extrude(sim_world):
    """The bench order: a FORCE_MOVE overlay run on the extruder lane
    precedes the extrude, so the seed path is also exercised against a
    sampler that has carried an overlay frame.

    Each nudge is drained before the next, like the FORCE_MOVE lane tests:
    back-to-back nudges on one lane load the second overlay run behind the
    mcu's pending unstep ("Stepper too far in past", motion.step_load_late),
    which is a nudge pacing defect, not a seed-origin one."""
    world = _boot(sim_world)
    world.gcode_ok("SET_KINEMATIC_POSITION X=125 Y=125 Z=125")
    world.gcode_ok("SET_STEPPER_ENABLE STEPPER=extruder ENABLE=1")
    world.gcode_ok("FORCE_MOVE STEPPER=extruder DISTANCE=5 VELOCITY=5")
    world.gcode_ok("M400", timeout=120)
    world.gcode_ok("FORCE_MOVE STEPPER=extruder DISTANCE=-5 VELOCITY=5")
    world.gcode_ok("M400", timeout=120)
    _extrude_then_reanchor_and_home(world)


def test_stepcompress_extruder_back_to_back_force_moves(sim_world):
    """Two FORCE_MOVE nudges on the same lane with no M400 between them.

    Nothing drains the first overlay run before the second is planned, so the
    second run's first step must still land after the mcu's pending unstep for
    the last step of the first run. Anchoring the second nudge on the wall
    clock instead loaded it behind that unstep: motion.step_load_late, then
    "Stepper too far in past"."""
    world = _boot(sim_world)
    world.gcode_ok("SET_KINEMATIC_POSITION X=125 Y=125 Z=125")
    world.gcode_ok("SET_STEPPER_ENABLE STEPPER=extruder ENABLE=1")
    before = _extruder_steps(world)
    world.gcode_ok("FORCE_MOVE STEPPER=extruder DISTANCE=5 VELOCITY=5")
    world.gcode_ok("FORCE_MOVE STEPPER=extruder DISTANCE=-5 VELOCITY=5")
    world.gcode_ok("M400", timeout=120)
    _assert_alive(world)
    events = world.events_text()
    assert "step_load_late" not in events, world.log_tail()
    net = abs(_extruder_steps(world) - before)
    assert net <= 2, (
        f"the two opposite nudges left the lane {net} steps off its origin\n"
        + world.log_tail()
    )
