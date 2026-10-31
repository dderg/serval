import time

import pytest

from tools.sim import configs

pytestmark = pytest.mark.needs_elf

# The runtime notifies the shim per axis step queue, not per configured step
# pin: X -> gpio18, Y -> gpio7 (src/linux/runtime_tick_host.c
# step_gpio_lines).
X_STEP_LINE = 18
Y_STEP_LINE = 7


def _wait_steps_settled(control, line, minimum, timeout=20.0):
    """Waits for the shim to see at least `minimum` step edges on `line` and
    then for the tick thread to stop adding more, so the net position read
    after this lands on the waveform's end rather than mid-cycle."""
    deadline = time.monotonic() + timeout
    count = control.get_step_times(line)["count"]
    while count < minimum and time.monotonic() < deadline:
        time.sleep(0.05)
        count = control.get_step_times(line)["count"]
    while time.monotonic() < deadline:
        time.sleep(0.3)
        settled = control.get_step_times(line)["count"]
        if settled == count:
            return count
        count = settled
    return count


def test_host_generated_stepper_buzz_returns_to_base(sim_world):
    def config(world):
        return (
            configs.minimal_config(world.h7_pty, str(world.gcode_dir))
            + "\n[resonance_buzz]\n"
        )

    world = sim_world(config, dual_mcu=False)
    control = world.sim_control("h7")
    world.gcode_ok("SET_KINEMATIC_POSITION X=125 Y=125 Z=125")
    before = world.toolhead_position()

    # 0.05mm at 80 steps/mm is a 4-step peak, so 4 cycles of 40Hz must land
    # tens of edges on the X step pin and close on the base step exactly: a
    # no-op buzz also "returns to base", and an unclosed waveform drifts the
    # MCU's step position away from the host's logical one.
    x_base = control.step_position(X_STEP_LINE)["steps"]
    control.reset_step_times(X_STEP_LINE)
    world.gcode_ok(
        "RESONANCE_BUZZ AXIS=X FREQ=40 AMPLITUDE=0.05 DURATION=0.1 RAMP=0.01"
    )
    assert _wait_steps_settled(control, X_STEP_LINE, 8) >= 8
    assert control.step_position(X_STEP_LINE)["steps"] == x_base
    assert world.toolhead_position() == before

    y_base = control.step_position(Y_STEP_LINE)["steps"]
    control.reset_step_times(Y_STEP_LINE)
    world.gcode_ok(
        "RESONANCE_BUZZ_SWEEP AXIS=Y FREQ_START=20 FREQ_END=60 AMPLITUDE=0.03 DURATION=0.1 RAMP=0.01"
    )
    assert _wait_steps_settled(control, Y_STEP_LINE, 4) >= 4
    assert control.step_position(Y_STEP_LINE)["steps"] == y_base
    assert world.toolhead_position() == before

    assert world.shutdown_line() is None
