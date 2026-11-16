"""A print file with a motion command the host cannot run (#867) ends in a
print error at that line, with no MCU shutdown and no later move."""

import pytest

from tools.sim import configs
from tools.sim.world import SimError

pytestmark = pytest.mark.needs_elf

PRINT_TIMEOUT_S = 120
START_X = 125.0
BEFORE_X = 150.0
AFTER_X = 190.0
POSITION_TOLERANCE_MM = 0.5
HARMLESS_UNKNOWN = "M2000 S1"
SKIPPED_ARC = "G2 X170 Y125 I10 J0"
ARC_FILE = f"""\
SET_KINEMATIC_POSITION X={START_X} Y=125 Z=125
G21
{HARMLESS_UNKNOWN}
G1 X{BEFORE_X} F3000
{SKIPPED_ARC}
G1 X{AFTER_X} F3000
M400
"""
CLEAN_FILE = ARC_FILE.replace(SKIPPED_ARC + "\n", "")


def world_printing(sim_world, name: str, text: str):
    world = sim_world(
        lambda w: configs.minimal_config(w.h7_pty, str(w.gcode_dir)),
        dual_mcu=False,
    )
    path = world.gcode_dir / name
    path.write_text(text)
    return world, path


def test_an_arc_in_a_print_ends_it_with_an_error(sim_world):
    world, path = world_printing(sim_world, "arc.gcode", ARC_FILE)
    with pytest.raises(SimError, match="print error") as raised:
        world.print_file(path, timeout=PRINT_TIMEOUT_S)
    message = str(raised.value)
    assert "G2" in message
    assert SKIPPED_ARC in message
    assert "not supported" in message
    assert world.shutdown_line() is None
    world.gcode_ok("M400")
    assert world.toolhead_position()[0] == pytest.approx(
        BEFORE_X, abs=POSITION_TOLERANCE_MM
    )
    assert world.shutdown_line() is None


def test_an_unknown_harmless_command_still_prints(sim_world):
    world, path = world_printing(sim_world, "clean.gcode", CLEAN_FILE)
    world.print_file(path, timeout=PRINT_TIMEOUT_S)
    assert world.toolhead_position()[0] == pytest.approx(
        AFTER_X, abs=POSITION_TOLERANCE_MM
    )
    assert world.shutdown_line() is None
