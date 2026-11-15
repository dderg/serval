"""#411: changing a heater setpoint must restart the idle timeout, as it
does upstream, where M104/SET_HEATER_TEMPERATURE pass through the toolhead's
lookahead and fire `toolhead:sync_print_time`. A user adjusting the nozzle
temperature on an otherwise idle printer must not have the heaters switched
off under them."""

import pytest

from tools.sim import configs

pytestmark = pytest.mark.needs_elf

IDLE_TIMEOUT_S = 4.0
SETPOINT_PERIOD_S = 0.75 * IDLE_TIMEOUT_S
TARGETS = (150.0, 151.0, 152.0, 153.0)

IDLE_SECTIONS = f"""
[verify_heater extruder]
check_gain_time: 3600

[idle_timeout]
timeout: {IDLE_TIMEOUT_S}
"""


def _short_idle_timeout(world):
    return (
        configs.heaters_config(world.h7_pty, str(world.gcode_dir))
        + IDLE_SECTIONS
    )


def _let_printer_time_pass(world, seconds):
    start_ns = world.vclock.now_ns()
    world.wait_until(
        lambda: world.vclock.now_ns() - start_ns >= seconds * 1e9,
        timeout=seconds + 10,
        what=f"{seconds}s of printer time",
        poll=0.1,
    )


@pytest.mark.parametrize(
    "setpoint",
    ["M104 S{}", "SET_HEATER_TEMPERATURE HEATER=extruder TARGET={}"],
)
def test_heater_setpoints_reset_idle_timeout(sim_world, setpoint):
    world = sim_world(_short_idle_timeout, dual_mcu=False)
    world.gcode_ok("SET_KINEMATIC_POSITION X=100 Y=100 Z=5")
    world.gcode_ok("G1 X110 F3000")
    world.gcode_ok("M400", timeout=60)
    for previous, target in zip((None,) + TARGETS, TARGETS):
        status = world.status({"idle_timeout": None, "extruder": None})
        if previous is not None:
            assert status["extruder"]["target"] == previous, status
        assert status["idle_timeout"]["state"] != "Idle", status
        world.gcode_ok(setpoint.format(target))
        _let_printer_time_pass(world, SETPOINT_PERIOD_S)
    status = world.status({"idle_timeout": None, "extruder": None})
    assert status["extruder"]["target"] == TARGETS[-1], status
    assert status["idle_timeout"]["state"] != "Idle", status
