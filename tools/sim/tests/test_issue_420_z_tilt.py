import pytest

from tools.sim import configs

pytestmark = pytest.mark.needs_elf


def _config(world):
    config = configs.probe_config(world.h7_pty, str(world.gcode_dir), "points")
    config = config.replace(
        "max_velocity: 100\nmax_accel: 1000",
        "max_velocity: 1000\nmax_accel: 100000",
    )
    config = config.replace(
        "max_z_velocity: 10\nmax_z_accel: 30",
        "max_z_velocity: 100\nmax_z_accel: 1000",
    )
    config = config.replace(
        "x_motors: x\ny_motors: y", "x_motors: x, x1\ny_motors: y, y1"
    )
    config = config.replace(
        "[post_processor is_xy]\ntype: smooth_bell\nsmooth_time: 0.019125",
        "[post_processor is_xy]\ntype: smooth_zv\nfrequency_hz: 80",
    )
    config = config.replace(
        "points:\n    50, 125\n    200, 125\nspeed: 50\nhorizontal_move_z: 8",
        "points:\n    147.5, 140\n    80, 10\n    215, 10\nspeed: 500\nhorizontal_move_z: 18",
    )
    config += """

[motor x1]
drive: stepper
step_pin: gpiochip0/gpio13
dir_pin: gpiochip0/gpio14
enable_pin: !gpiochip0/gpio15
microsteps: 1
full_steps_per_rotation: 5000
rotation_distance: 40

[motor y1]
drive: stepper
step_pin: gpiochip0/gpio16
dir_pin: gpiochip0/gpio17
enable_pin: !gpiochip0/gpio18
microsteps: 1
full_steps_per_rotation: 5000
rotation_distance: 40
"""
    return config


def test_z_tilt_after_safe_home_does_not_exhaust_step_root_deadline(sim_world):
    world = sim_world(_config, dual_mcu=False)
    world.gcode_ok("SET_KINEMATIC_POSITION X=145 Y=93 Z=5.7")
    world.gcode_ok("Z_TILT_ADJUST", timeout=300)
    assert world.shutdown_line() is None
