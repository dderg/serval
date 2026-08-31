from pathlib import Path

import pytest

from tools.sim import configs
from tools.sim.tests.test_mesh_clock_regression_repro import (
    _cfg as exact_mesh_config,
)

pytestmark = pytest.mark.needs_elf

GCODE = (
    Path(__file__).parent.parent
    / "gcode"
    / "trident_follower_cache_regression.gcode"
)


def _config(world):
    base = configs.neptune_print_config(world.h7_pty, str(world.gcode_dir))
    base = base.replace("max_velocity: 300", "max_velocity: 1000")
    base = base.replace("max_accel: 4000", "max_accel: 50000")
    base = base.replace("max_z_accel: 200", "max_z_accel: 1000")
    base = base.replace("microsteps: 16", "microsteps: 1")
    base = base.replace(
        "[motor z]\ndrive: stepper\nstep_pin: gpiochip0/gpio6\ndir_pin: gpiochip0/gpio7\nenable_pin: !gpiochip0/gpio8\nmicrosteps: 1",
        "[motor z]\ndrive: stepper\nstep_pin: gpiochip0/gpio6\ndir_pin: gpiochip0/gpio7\nenable_pin: !gpiochip0/gpio8\nmicrosteps: 8",
    )
    base = base.replace("k: 0.03", "k: 0.02")
    base = base.replace("smooth_time: 0.04", "smooth_time: 0.013")
    base = base.replace(
        "[axis x]\nposition_min: 0\nposition_endstop: 0\nposition_max: 250\nendstop_pin: ^gpiochip0/gpio10\nhoming_speed: 10",
        "[axis x]\nposition_min: 0\nposition_endstop: 0\nposition_max: 250\nendstop_pin: ^gpiochip0/gpio10\nhoming_speed: 10\npost_processors: shaper_x",
    )
    base = base.replace(
        "[axis y]\nposition_min: 0\nposition_endstop: 0\nposition_max: 250\nendstop_pin: ^gpiochip0/gpio11\nhoming_speed: 10",
        "[axis y]\nposition_min: 0\nposition_endstop: 0\nposition_max: 250\nendstop_pin: ^gpiochip0/gpio11\nhoming_speed: 10\npost_processors: shaper_y",
    )
    additions = """[post_processor shaper_x]
type: smooth_mzv
frequency_hz: 195

[post_processor shaper_y]
type: smooth_mzv
frequency_hz: 116

[bed_mesh]
mesh_min: 83.6719, 122.632
mesh_max: 199.242, 176.198
probe_count: 10, 5
algorithm: bicubic
zero_reference_position: 150, 150

"""
    base = base.replace("[virtual_sdcard]", additions + "[virtual_sdcard]", 1)
    exact = exact_mesh_config(world)
    saved_profile = exact[
        exact.index("#*# <---------------------- SAVE_CONFIG") :
    ]
    return base + saved_profile


def test_trident_dense_layers_keep_follower_fit_stable(sim_world):
    world = sim_world(_config, dual_mcu=False, vtime_speed=0.5)
    world.gcode_ok("SET_KINEMATIC_POSITION X=150 Y=150 Z=0.2")
    world.gcode_ok("BED_MESH_PROFILE LOAD=failed")
    world.print_file(GCODE, timeout=900)
    assert world.shutdown_line() is None
