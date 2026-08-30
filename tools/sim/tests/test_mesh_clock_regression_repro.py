from pathlib import Path

import pytest

from tools.sim import configs

pytestmark = pytest.mark.needs_elf


def _cfg(world):
    base = configs.bed_mesh_config(world.h7_pty, str(world.gcode_dir))
    base = base.split("#*# <---------------------- SAVE_CONFIG", 1)[0]
    base = base.replace("max_velocity: 100", "max_velocity: 1000")
    base = base.replace("max_accel: 1000", "max_accel: 50000")
    base = base.replace("max_z_velocity: 10", "max_z_velocity: 20")
    base = base.replace("max_z_accel: 100", "max_z_accel: 1000")
    base = base.replace("mesh_min: 20, 20", "mesh_min: 83.6719, 122.632")
    base = base.replace("mesh_max: 70, 70", "mesh_max: 199.242, 176.198")
    base = base.replace("probe_count: 3, 3", "probe_count: 10, 5")
    base = base.replace(
        "probe_count: 10, 5", "probe_count: 10, 5\nalgorithm: bicubic"
    )
    base = base.replace(
        "zero_reference_position: 45, 45", "zero_reference_position: 150, 150"
    )
    base = base.replace("fade_start: 1\nfade_end: 10\nfade_target: 0\n", "")
    return (
        base
        + """
#*# <---------------------- SAVE_CONFIG ---------------------->
#*# DO NOT EDIT THIS BLOCK OR BELOW. The contents are auto-generated.
#*#
#*# [bed_mesh failed]
#*# version = 1
#*# points =
#*#   0.009902, 0.006611, -0.023850, -0.027201, -0.032798, -0.020135, 0.013096, 0.026635, 0.033351, 0.040149
#*#   0.012379, -0.007716, -0.016060, -0.013543, -0.022700, -0.009804, 0.018470, 0.033301, 0.039874, 0.048531
#*#   -0.003875, -0.004488, -0.013045, -0.023070, -0.026537, -0.003544, 0.025704, 0.037109, 0.044800, 0.057221
#*#   -0.001529, 0.004596, 0.000007, -0.012474, -0.024070, -0.001565, 0.026445, 0.042483, 0.052862, 0.063829
#*#   0.011929, 0.013187, 0.004371, -0.003995, -0.014671, 0.002363, 0.026319, 0.043285, 0.058059, 0.068611
#*# min_x = 83.6719
#*# max_x = 199.242
#*# min_y = 122.632
#*# max_y = 176.198
#*# x_count = 10
#*# y_count = 5
#*# mesh_x_pps = 2
#*# mesh_y_pps = 2
#*# algo = bicubic
#*# tension = 0.2
"""
    )


def test_trident_mesh_short_moves_do_not_regress_step_clocks(sim_world):
    world = sim_world(_cfg, dual_mcu=False)
    world.gcode_ok("SET_KINEMATIC_POSITION X=150 Y=150 Z=0.2")
    world.gcode_ok("BED_MESH_PROFILE LOAD=failed")
    path = world.gcode_dir / "failed_first_layer.gcode"
    source = (
        Path(__file__).parent.parent
        / "gcode"
        / "trident_mesh_clock_regression.gcode"
    )
    path.write_text(source.read_text())
    world.print_file(path, timeout=900)
    assert world.shutdown_line() is None
