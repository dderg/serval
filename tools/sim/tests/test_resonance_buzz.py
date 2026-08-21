import pytest

from tools.sim import configs

pytestmark = pytest.mark.needs_elf


def test_host_generated_stepper_buzz_returns_to_base(sim_world):
    def config(world):
        return (
            configs.minimal_config(world.h7_pty, str(world.gcode_dir))
            + "\n[resonance_buzz]\n"
        )

    world = sim_world(config, dual_mcu=False)
    world.gcode_ok("SET_KINEMATIC_POSITION X=125 Y=125 Z=125")
    before = world.toolhead_position()
    world.gcode_ok(
        "RESONANCE_BUZZ AXIS=X FREQ=40 AMPLITUDE=0.05 DURATION=0.1 RAMP=0.01"
    )
    assert world.toolhead_position() == before
    world.gcode_ok(
        "RESONANCE_BUZZ_SWEEP AXIS=Y FREQ_START=20 FREQ_END=60 AMPLITUDE=0.03 DURATION=0.1 RAMP=0.01"
    )
    assert world.toolhead_position() == before
    assert world.shutdown_line() is None
