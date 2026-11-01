"""#423: the host produces a heater queue_digital_out ~37 ms past its
deadline while step frames keep flowing on time, so the MCU trips "Timer too
close" with two classic steppers still holding queued moves. The shutdown
must finish (stepper_shutdown clears each move queue before move_reset's
relinked free list can loop it) and leave the MCU answering.
"""

from __future__ import annotations

import pytest

from tools.sim import configs

pytestmark = pytest.mark.needs_elf

HEATER = """
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
SHUTDOWN = "MCU 'mcu' shutdown: Timer too close"


def test_late_heater_pwm_shuts_down_cleanly(sim_world):
    world = sim_world(
        lambda w: configs.minimal_config(w.h7_pty, str(w.gcode_dir)) + HEATER,
        dual_mcu=False,
    )
    world.gcode_ok("SET_KINEMATIC_POSITION X=20 Y=20 Z=20")
    world.gcode_ok("SET_HEATER_TEMPERATURE HEATER=chamber TARGET=60")
    world.gcode(ZIGZAG)
    world.inject(
        "h7",
        late_command="queue_digital_out",
        by_s=HEATER_LEAD_S + FIELD_LATENESS_S,
    )

    assert world.wait_for_log_text(SHUTDOWN), world.log_tail()
    assert (
        "Transition to shutdown state: " + SHUTDOWN in world.klippy_log_text()
    )
    mcu_log = (world.log_dir / "h7.log").read_text(errors="replace")
    assert "[vtime] fault: sending late msgid" in mcu_log
    assert "[mcu-seq] MISMATCH" not in mcu_log

    world.gcode("RESTART")
    world.wait_until(
        lambda: (
            "MCU 'mcu' is_shutdown: Timer too close" in world.klippy_log_text()
        ),
        timeout=30,
        what="the MCU answering the reconnect with its shutdown reason",
        poll=0.1,
    )
    log = world.klippy_log_text()
    assert "Lost communication" not in log
    assert "Timeout with MCU" not in log
