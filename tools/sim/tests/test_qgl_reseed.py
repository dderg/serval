import json

import pytest

from tools.sim import configs

pytestmark = pytest.mark.needs_elf


def test_awd_idle_belt_survives_repeated_probe_reseeds(sim_world):
    world = sim_world(
        lambda w: configs.awd_three_z_probe_config(
            w.h7_pty, w.f4_pty, str(w.gcode_dir)
        ),
        dual_mcu=True,
    )
    world.gcode_ok("SET_KINEMATIC_POSITION X=0 Y=0 Z=8")

    for coordinate in (20, 40, 60, 80, 100, 120):
        world.gcode_ok("G1 Z8 F600")
        world.gcode_ok(f"G1 X{coordinate} Y{coordinate} F3000")
        world.gcode_ok("PROBE", timeout=90)
    world.gcode_ok("G1 Z8 F600")
    world.gcode_ok("G1 X140 Y100 F3000")
    world.gcode_ok("M400", timeout=60)

    reanchors = [
        json.loads(line)
        for line in world.events_text().splitlines()
        if '"event":"reanchor_mark"' in line and '"mcu":0' in line
    ]
    assert len(reanchors) >= 12
    assert world.shutdown_line() is None


def test_single_motor_nudges_let_the_idle_belt_rejoin(sim_world):
    """motors_sync-style FORCE_MOVE nudges drive one lane while every other
    lane receives neither pieces nor marks. When the sat-out lane's next
    nudge lands right behind another lane's nudge, the stream is globally
    contiguous (Continuation, no reanchor mark), so only the pump's
    lane-local rejoin sanction keeps its multi-second timeline hole from
    tripping the shim's PieceGap guard (Trident bench crash, motor_b1)."""
    world = sim_world(
        lambda w: configs.awd_three_z_probe_config(
            w.h7_pty, w.f4_pty, str(w.gcode_dir)
        ),
        dual_mcu=True,
    )
    world.gcode_ok("SET_KINEMATIC_POSITION X=50 Y=50 Z=8")
    world.gcode_ok("G1 X51 Y51 F3000")
    world.gcode_ok("M400", timeout=60)

    world.gcode_ok("FORCE_MOVE STEPPER=b1 DISTANCE=5 VELOCITY=20")
    world.gcode_ok("G4 P400")
    for _ in range(3):
        world.gcode_ok("FORCE_MOVE STEPPER=a DISTANCE=5 VELOCITY=20")
        world.gcode_ok("G4 P400")
        world.gcode_ok("FORCE_MOVE STEPPER=a DISTANCE=-5 VELOCITY=20")
        world.gcode_ok("G4 P400")
    world.gcode_ok("FORCE_MOVE STEPPER=a DISTANCE=5 VELOCITY=20")
    world.gcode_ok("FORCE_MOVE STEPPER=b1 DISTANCE=-5 VELOCITY=20")
    world.gcode_ok("M400", timeout=60)

    assert world.shutdown_line() is None
    assert any(
        '"event":"lane_rejoin_gap_mark"' in line
        for line in world.events_text().splitlines()
    ), "the sat-out belt's Continuation resume must be sanctioned by the pump"
