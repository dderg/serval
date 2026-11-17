"""`SimWorld.shutdown_line()` sees every way klippy shuts down: an MCU's
own report and klippy's transition, which for a host-side internal error
carries no `shutdown:` (the fuzzer found the FIRMWARE_RESTART + M400 one
that test_firmware_restart_mid_print's check let through)."""

import pytest

from tools.sim.world import SimWorld

pytestmark = pytest.mark.sim_unit

MCU_SHUTDOWN = "MCU 'mcu' shutdown: Timer too close"
HOST_SHUTDOWN = 'Transition to shutdown state: Internal error on command:"M400"'
QUIET_LOG = "Starting Klippy...\nConfigured MCU 'mcu' (1024 moves)\n"


def _world_with_log(tmp_path, text: str) -> SimWorld:
    world = SimWorld(tmp_path)
    world.log_dir.mkdir(parents=True)
    world.klippy_log.write_text(text)
    return world


@pytest.mark.parametrize("line", [MCU_SHUTDOWN, HOST_SHUTDOWN])
def test_a_shutdown_is_found(tmp_path, line):
    world = _world_with_log(tmp_path, QUIET_LOG + line + "\n")
    assert world.shutdown_line() == line


def test_a_clean_log_has_no_shutdown(tmp_path):
    assert _world_with_log(tmp_path, QUIET_LOG).shutdown_line() is None
