"""Two motors on one axis with a 0.2 us step pulse step on the same ticks
under 2 us apart, so the linux MCU's timer loop runs its repeat limit out on
step timers that are not yet due and hands control to the busy tasks. The
loop must not run the head timer on its return before the head's waketime:
the follower batch guard shuts the MCU down with "Step timer dispatched
before its waketime" (#633, #841)."""

import pytest

from tools.sim import configs

pytestmark = pytest.mark.needs_elf

STEPS_PER_MM = 200 * 256 / 8
SPEED_MM_S = 100
STEP_INTERVAL_US = 1e6 / (STEPS_PER_MM * SPEED_MM_S)
TIMER_MIN_TRY_US = 2
X_MOTOR = """microsteps: 256
rotation_distance: 8
step_pulse_duration: 0.0000002
"""
START = "SET_KINEMATIC_POSITION X=20 Y=125 Z=5"
TARGET_X = 80


def _config(world):
    base = configs.minimal_config(world.h7_pty, str(world.gcode_dir))
    config = base.replace("x_motors: x\n", "x_motors: x, x1\n").replace(
        "microsteps: 16\nrotation_distance: 40\n\n[motor y]",
        f"{X_MOTOR}\n[motor y]",
    )
    assert config.count(X_MOTOR) == 1, config
    assert "x_motors: x, x1\n" in config, config
    return (
        config
        + f"""
[motor x1]
drive: stepper
step_pin: gpiochip0/gpio13
dir_pin: gpiochip0/gpio14
enable_pin: !gpiochip0/gpio15
{X_MOTOR}"""
    )


def test_a_dense_follower_step_chain_never_dispatches_a_step_early(sim_world):
    assert STEP_INTERVAL_US < TIMER_MIN_TRY_US
    world = sim_world(_config, dual_mcu=False)
    world.gcode_ok(START)
    world.gcode_ok(f"G1 X{TARGET_X} F{SPEED_MM_S * 60}", timeout=120)
    world.gcode_ok("M400", timeout=120)
    assert world.shutdown_line() is None, world.log_tail()
    assert world.toolhead_position()[0] == pytest.approx(TARGET_X, abs=0.01)
