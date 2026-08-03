"""FORCE_MOVE on a non-spatial (extruder) motor lane.

The nudge planner used to bound axis_idx by SPATIAL_AXES, so any FORCE_MOVE
targeting the extruder motor (axis 3) failed the command and shut klippy
down. A nudge never runs through kinematics — it is a per-motor lane
profile — so the extruder lane is planned exactly like a spatial one.

Under stepping_mode: stepcompress the nudge then had to survive the shim:
its pieces carry a motor_mask and are relativized to start at zero, so the
sampler steps them against an overlay frame, not the lane's absolute one.

The world's [motor extruder] runs an inverted dir pin, so the sim's raw
step tracker on the step line counts a forward extrude as negative — that
inversion is part of the follower-lane contract this world exercises.
"""

from __future__ import annotations

import pytest

from tools.sim import configs

pytestmark = pytest.mark.needs_elf

STEPS_PER_MM = configs.STEPCOMPRESS_EXTRUDER_STEPS_PER_MM
STEP_LINE = configs.STEPCOMPRESS_EXTRUDER_STEP_LINE
TRACKED_STEPS_PER_MM = -STEPS_PER_MM


def _lane_pos(world) -> int:
    resp = world.sim_control("f4").send(f"get_steps line={STEP_LINE}")
    if not resp.startswith("steps="):
        raise AssertionError(f"get_steps line={STEP_LINE}: {resp!r}")
    return int(resp.split()[0].split("=")[1])


def _boot(sim_world):
    world = sim_world(
        lambda w: configs.stepcompress_extruder_config(
            w.h7_pty, w.f4_pty, str(w.gcode_dir)
        ),
        sc_mcu=True,
    )
    world.gcode_ok("SET_KINEMATIC_POSITION X=125 Y=125 Z=125")
    world.gcode_ok("SET_STEPPER_ENABLE STEPPER=extruder ENABLE=1")
    world.gcode_ok("M400")
    return world


def test_force_move_extruder_advances_the_lane(sim_world):
    world = _boot(sim_world)
    before = _lane_pos(world)
    world.gcode_ok("FORCE_MOVE STEPPER=extruder DISTANCE=5 VELOCITY=5")
    world.gcode_ok("M400")
    assert world.shutdown_line() is None, world.log_tail()
    assert _lane_pos(world) - before == pytest.approx(
        5.0 * TRACKED_STEPS_PER_MM, abs=1.0
    )


def test_force_move_extruder_retracts_the_lane(sim_world):
    world = _boot(sim_world)
    world.gcode_ok("FORCE_MOVE STEPPER=extruder DISTANCE=5 VELOCITY=5")
    world.gcode_ok("M400")
    before = _lane_pos(world)
    world.gcode_ok("FORCE_MOVE STEPPER=extruder DISTANCE=-5 VELOCITY=5")
    world.gcode_ok("M400")
    assert world.shutdown_line() is None, world.log_tail()
    assert _lane_pos(world) - before == pytest.approx(
        -5.0 * TRACKED_STEPS_PER_MM, abs=1.0
    )


def test_force_move_spatial_lane_control(sim_world):
    world = sim_world(
        lambda w: configs.stepcompress_config(
            w.h7_pty, w.f4_pty, str(w.gcode_dir)
        ),
        sc_mcu=True,
    )
    world.gcode_ok("SET_KINEMATIC_POSITION X=125 Y=125 Z=125")
    world.gcode_ok("SET_STEPPER_ENABLE STEPPER=x ENABLE=1")
    world.gcode_ok("M400")
    line = configs.STEPCOMPRESS_STEP_LINES["x"]
    before = int(
        world.sim_control("f4")
        .send(f"get_steps line={line}")
        .split()[0]
        .split("=")[1]
    )
    world.gcode_ok("FORCE_MOVE STEPPER=x DISTANCE=5 VELOCITY=5")
    world.gcode_ok("M400")
    assert world.shutdown_line() is None, world.log_tail()
    after = int(
        world.sim_control("f4")
        .send(f"get_steps line={line}")
        .split()[0]
        .split("=")[1]
    )
    assert after - before == pytest.approx(
        5.0 * configs.STEPCOMPRESS_STEPS_PER_MM, abs=1.0
    )
