"""#423: a heater queue_digital_out that reaches the MCU after its deadline
trips "Timer too close", and the shutdown it spreads must finish on an MCU
whose classic steppers still hold queued moves: stepper_shutdown clears each
move queue before move_reset's relinked free list can loop it.

The heater sits on the main MCU and the steppers on the step/dir MCU: a
heater update leads its deadline by ~0.30 s while queue_step leads by 0.25 s
(SEND_LEAD_SECONDS), so holding one MCU's stream long enough to make the
heater late drains that MCU's step queues first.
"""

from __future__ import annotations

import pytest

from tools.sim import configs

pytestmark = pytest.mark.needs_elf

HEATER_ON_MAIN_MCU = """
[heater_generic chamber]
heater_pin: gpiochip0/gpio24
sensor_type: Generic 3950
sensor_pin: analog0
min_temp: 0
max_temp: 130
control: pid
pid_kp: 63.350
pid_ki: 4.100
pid_kd: 244.691
"""
HEATER_LEAD_S = 0.30
FIELD_LATENESS_S = 0.037
ZIGZAG = "\n".join(
    f"G1 X{20 + 2 * (i % 2)} Y{20 + 2 * (i % 2)} F3000" for i in range(400)
)


def _sc_shutdown_reports(world) -> int:
    return sum(
        "MCU 'sc'" in line and "Command request" in line
        for line in world.klippy_log_text().splitlines()
    )


def test_late_heater_pwm_shuts_down_cleanly(sim_world):
    world = sim_world(
        lambda w: (
            configs.stepcompress_config(w.h7_pty, w.f4_pty, str(w.gcode_dir))
            + HEATER_ON_MAIN_MCU
        ),
        sc_mcu=True,
    )
    world.gcode_ok("SET_KINEMATIC_POSITION X=20 Y=20 Z=20")
    world.gcode_ok("SET_HEATER_TEMPERATURE HEATER=chamber TARGET=60")
    x_steps = configs.STEPCOMPRESS_STEP_LINES["x"]
    sc = world.sim_control("f4")
    start = sc.step_position(x_steps)["steps"]
    world.gcode(ZIGZAG)
    world.wait_until(
        lambda: sc.step_position(x_steps)["steps"] != start,
        timeout=10,
        what="the zigzag stepping on sc",
    )
    world.inject(
        "h7",
        delay_command="queue_digital_out",
        by_s=HEATER_LEAD_S + FIELD_LATENESS_S,
    )

    tripped = "MCU 'mcu' shutdown: Timer too close"
    assert world.wait_for_log_text(tripped), world.log_tail()
    assert world.wait_for_log_text("MCU 'sc' shutdown: Command request"), (
        world.log_tail()
    )
    log = world.klippy_log_text()
    assert "Transition to shutdown state: " + tripped in log

    world.gcode("RESTART")
    world.wait_until(
        lambda: _sc_shutdown_reports(world) >= 2,
        timeout=30,
        what="sc answering the reconnect with its shutdown reason",
        poll=0.1,
    )
    log = world.klippy_log_text()
    assert "Lost communication" not in log
    assert "Timeout with MCU" not in log
