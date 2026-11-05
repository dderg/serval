"""FIRMWARE_RESTART resets every simulated MCU family: from a ready
printer, and from MCUs latched in shutdown across a host restart. The
MACH_LINUX firmware resets with config_reset, which re-executes itself."""

from __future__ import annotations

import pytest

from tools.sim import configs

pytestmark = pytest.mark.needs_elf


def _kalico_native(sim_world):
    return sim_world(
        lambda w: configs.minimal_config(w.h7_pty, str(w.gcode_dir)),
        dual_mcu=False,
    )


def _classic_stepcompress(sim_world):
    return sim_world(
        lambda w: configs.stepcompress_config(
            w.h7_pty, w.f4_pty, str(w.gcode_dir)
        ),
        sc_mcu=True,
    )


def _moves_after_restart(world):
    world.mark_log()
    world.gcode_ok("SET_KINEMATIC_POSITION X=125 Y=125 Z=125")
    world.gcode_ok("G1 X135 Y130 F3000")
    world.gcode_ok("M400")
    assert world.toolhead_position()[:2] == pytest.approx([135.0, 130.0])
    assert "shutdown" not in world.log_tail().lower(), world.log_tail()


@pytest.mark.parametrize("boot", [_kalico_native, _classic_stepcompress])
def test_firmware_restart_from_ready(sim_world, boot):
    world = boot(sim_world)
    world.firmware_restart()
    _moves_after_restart(world)


@pytest.mark.parametrize("boot", [_kalico_native, _classic_stepcompress])
def test_firmware_restart_recovers_mcus_latched_across_a_host_restart(
    sim_world, boot
):
    world = boot(sim_world)
    world.gcode("M112")
    settled = world.restart_klippy()
    assert settled in ("shutdown", "error"), (settled, world.log_tail())
    world.firmware_restart()
    _moves_after_restart(world)
