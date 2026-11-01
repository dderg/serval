"""#423: the host produces a heater queue_digital_out ~37 ms past its
deadline while step frames keep flowing on time, so the MCU trips "Timer too
close" with two classic steppers still holding queued moves. The shutdown
must finish (stepper_shutdown clears each move queue before move_reset's
relinked free list can loop it) and leave the MCU answering.

The shutdown longjmps out of the frame carrying the late command, dropping
any frames the MCU read behind it in the same chunk; the MCU then NAKs while
awaiting the frame after the late one until klippy retransmits it.
"""

from __future__ import annotations

import re

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
LATE_FRAME_SENT = re.compile(
    r"\[vtime\] fault: sending late msgid \d+ in frame seq (\d+)"
)
NAK_AWAITING_SEQ = re.compile(
    r"\[mcu-seq\] MISMATCH .* next_sequence=0x([0-9a-f]+)"
)


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
    late_frame = LATE_FRAME_SENT.search(mcu_log)
    assert late_frame
    assert not NAK_AWAITING_SEQ.search(mcu_log, 0, late_frame.start())
    frame_after_late = (int(late_frame.group(1)) + 1) % 16
    awaited_after_late = {
        int(seq, 16) % 16
        for seq in NAK_AWAITING_SEQ.findall(mcu_log, late_frame.end())
    }
    assert awaited_after_late <= {frame_after_late}

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
