"""End-to-end print through the virtual SD card on real firmware."""

import pytest

from tools.sim import configs

pytestmark = pytest.mark.needs_elf


def test_self_test_print_completes(sim_world):
    world = sim_world(
        lambda w: configs.minimal_config(w.h7_pty, str(w.gcode_dir)),
        dual_mcu=False,
    )
    gcode = world.gcode_dir / "self_test.gcode"
    gcode.write_text(configs.SELF_TEST_GCODE)
    print_time = world.print_file(gcode, timeout=300)
    assert print_time > 0
    assert world.shutdown_line() is None


def test_motion_state_query_mid_move(sim_world):
    import re

    world = sim_world(
        lambda w: configs.minimal_config(w.h7_pty, str(w.gcode_dir)),
        dual_mcu=False,
    )
    world.gcode_ok("SET_KINEMATIC_POSITION X=150 Y=150 Z=100", timeout=10)
    world.gcode_ok("G4 P500", timeout=15)
    world.gcode_ok("G1 X170 F600", timeout=30)
    world.gcode_ok("M400", timeout=30)

    world.mark_log()
    world.gcode_ok("MCU_SIM_MOTION_STATE T_AGO=1.0", timeout=15)
    out = world.expect_log("x: pos=")
    m = re.search(r"x: pos=([0-9.eE+-]+) vel=([0-9.eE+-]+)", out)
    assert m, f"no x-axis state in response: {out!r}"
    pos, vel = float(m.group(1)), float(m.group(2))
    assert 150.5 < pos < 169.5, (
        "x pos not strictly interior to the move span — endpoint clamp "
        "or wrong print_time?"
    )
    assert vel > 5.0, "x vel not mid-cruise — query landed at rest?"
    assert world.shutdown_line() is None
