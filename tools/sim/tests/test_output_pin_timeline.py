"""Where an output pin change lands on the motion timeline (#893)."""

import pytest

from tools.sim import configs

from .test_resonance_buzz import X_STEP_LINE

pytestmark = pytest.mark.needs_elf

MARK_PIN_NAME = "soft_pwm_pin"
MARK_PIN_LINE = 37
DWELL_S = 1.5
POLL_S = 0.001
NS_PER_S = 1_000_000_000
WAIT_TIMEOUT_S = 30.0


def _cfg(world):
    return configs.heaters_config(world.h7_pty, str(world.gcode_dir))


def _x_steps(control) -> int:
    return control.get_step_times(X_STEP_LINE)["count"]


def test_a_dwell_after_a_pin_set_at_rest_counts_from_the_pin(sim_world):
    world = sim_world(_cfg, dual_mcu=False)
    world.gcode_ok("SET_KINEMATIC_POSITION X=100 Y=100 Z=5")
    path = world.gcode_dir / "pin_at_rest.gcode"
    lines = [
        "G90",
        "G1 X120 F6000",
        "M400",
        f"SET_PIN PIN={MARK_PIN_NAME} VALUE=1",
        f"G4 P{DWELL_S * 1000:.0f}",
        "G1 X100 F6000",
        "M400",
    ]
    path.write_text("\n".join(lines) + "\n")
    control = world.sim_control("h7")

    world.gcode_ok(f"SDCARD_PRINT_FILE FILENAME={path.name}")
    world.wait_until(
        lambda: control.gpio_output(0, MARK_PIN_LINE) == 1,
        timeout=WAIT_TIMEOUT_S,
        what="the pin set at rest",
        poll=POLL_S,
    )
    pin_ns = world.vclock.now_ns()
    rest_ended_ns = control.get_step_times(X_STEP_LINE)["last_ns"]
    steps_at_pin = _x_steps(control)
    world.wait_until(
        lambda: _x_steps(control) > steps_at_pin,
        timeout=WAIT_TIMEOUT_S,
        what="the move after the dwell",
        poll=POLL_S,
    )
    move_ns = world.vclock.now_ns()
    world.wait_print_done(timeout=WAIT_TIMEOUT_S)

    pin_after_rest_s = (pin_ns - rest_ended_ns) / NS_PER_S
    move_after_pin_s = (move_ns - pin_ns) / NS_PER_S
    timeline = (
        f"pin {pin_after_rest_s:.3f} s after the first move's last step, "
        f"next move {move_after_pin_s:.3f} s after the pin"
    )
    assert move_after_pin_s >= DWELL_S - POLL_S, timeline
    assert world.shutdown_line() is None
