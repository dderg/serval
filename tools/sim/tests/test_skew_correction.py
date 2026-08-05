"""skew_correction rides the host-side gcode_move transform chain: SET_SKEW
shears XY targets in Python before toolhead.move(), so the motion engine only
ever sees plain Cartesian coordinates. This guards that inherited path: an
active skew visibly displaces the commanded position and CLEAR restores the
identity transform."""

import pytest

from tools.sim import configs

pytestmark = pytest.mark.needs_elf

AC, BD, AD = 142.0, 140.0, 99.8


def _cfg(world):
    base = configs.minimal_config(world.h7_pty, str(world.gcode_dir))
    return base + "\n[skew_correction]\n"


def test_set_skew_shears_and_clear_restores_identity(sim_world):
    world = sim_world(_cfg, dual_mcu=False)
    world.gcode_ok("SET_KINEMATIC_POSITION X=125 Y=125 Z=125")

    world.gcode_ok("SET_SKEW CLEAR=1")
    world.gcode_ok("G1 X120 Y120 F3000")
    world.gcode_ok("M400", timeout=60.0)
    assert world.toolhead_position()[:2] == [120.0, 120.0]

    world.gcode_ok(f"SET_SKEW XY={AC},{BD},{AD}")
    world.gcode_ok("G1 X100 Y100 F3000")
    world.gcode_ok("M400", timeout=60.0)
    skewed = world.toolhead_position()
    assert abs(skewed[0] - 100.0) > 0.5, "SET_SKEW had no effect on X"
    assert skewed[1] == 100.0

    world.gcode_ok("SET_SKEW CLEAR=1")
    world.gcode_ok("G1 X100 Y100 F3000")
    world.gcode_ok("M400", timeout=60.0)
    assert world.toolhead_position()[:2] == [100.0, 100.0]

    assert world.shutdown_line() is None
