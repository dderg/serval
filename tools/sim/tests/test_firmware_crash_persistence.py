import time

import pytest

from tools.sim import configs

pytestmark = pytest.mark.needs_elf


def test_firmware_crash_survives_host_restart(sim_world):
    world = sim_world(
        lambda w: configs.minimal_config(w.h7_pty, str(w.gcode_dir)),
        dual_mcu=False,
    )

    control = world.sim_control("h7")
    world.gcode_ok("SET_KINEMATIC_POSITION X=125 Y=125 Z=125")
    before = control.step_position(18)["steps"]
    world.gcode_ok("G1 X200 F600")
    deadline = time.monotonic() + 10.0
    while control.step_position(18)["steps"] == before:
        assert time.monotonic() < deadline, "motion never reached the MCU"
        time.sleep(0.01)

    world.gcode("M112")
    assert world.wait_for_log_text("MCU 'mcu' shutdown: Command request")

    world.klippy_proc.terminate()
    world.klippy_proc.wait(timeout=5)
    world._spawn_klippy(world.workdir / "printer.cfg")

    assert world.wait_for_log_text(
        "Previous MCU 'mcu' shutdown: Command request", timeout=30
    )
    time.sleep(3)
    assert world.klippy_proc.poll() is None
    assert (
        "Attempting automated MCU 'mcu' restart" not in world.klippy_log_text()
    )
