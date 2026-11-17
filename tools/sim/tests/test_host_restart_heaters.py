from __future__ import annotations

import pytest

from tools.sim import configs

pytestmark = pytest.mark.needs_elf

HEATER_CHIP = 0
HEATER_LINE = 24
PROPORTIONAL_HEATER = f"""
[heater_generic chamber]
heater_pin: gpiochip{HEATER_CHIP}/gpio{HEATER_LINE}
sensor_type: Generic 3950
sensor_pin: analog0
min_temp: 0
max_temp: 130
control: pid
pid_kp: 255
pid_ki: 0
pid_kd: 0

[verify_heater chamber]
check_gain_time: 3600
"""
FULL_POWER_TARGET_C = 100.0
DUTY_UNDER_CHANGE_THRESHOLD = 0.03
HEATER_MAX_DURATION_S = 3.0
SETTLE_S = 2.0
RESTART_DEADLINE_S = 10.0
READY_TIMEOUT_S = 60.0


def _chamber(world):
    return world.status({"heater_generic chamber": None})[
        "heater_generic chamber"
    ]


def test_host_restart_switches_a_low_duty_heater_off(sim_world):
    world = sim_world(
        lambda w: (
            configs.minimal_config(w.h7_pty, str(w.gcode_dir))
            + PROPORTIONAL_HEATER
        ),
        dual_mcu=False,
    )
    control = world.sim_control("h7")
    world.gcode_ok(
        f"SET_HEATER_TEMPERATURE HEATER=chamber TARGET={FULL_POWER_TARGET_C}"
    )
    world.wait_until(
        lambda: control.gpio_output(HEATER_CHIP, HEATER_LINE) == 1,
        timeout=SETTLE_S,
        what="the heater at full power",
    )
    resting_c = _chamber(world)["temperature"]
    pulses_before = control.gpio_edges(HEATER_CHIP, HEATER_LINE)
    low_duty_target_c = resting_c + DUTY_UNDER_CHANGE_THRESHOLD
    world.gcode_ok(
        f"SET_HEATER_TEMPERATURE HEATER=chamber TARGET={low_duty_target_c:.3f}"
    )
    world.wait_until(
        lambda: (
            control.gpio_edges(HEATER_CHIP, HEATER_LINE) >= pulses_before + 2
        ),
        timeout=SETTLE_S,
        what="the heater pulsing under 5 % duty",
    )

    world.inject("h7", delay_command="identify", by_s=HEATER_MAX_DURATION_S)
    world.mark_log()
    world.gcode("RESTART")
    world.expect_log("Restarting printer", timeout=RESTART_DEADLINE_S)

    assert world._wait_ready(READY_TIMEOUT_S), world.log_tail()[-4000:]
    assert world.shutdown_line() is None, world.log_tail()[-4000:]
