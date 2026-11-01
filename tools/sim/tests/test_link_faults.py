"""world.inject: a dropped frame is retransmitted and a short link stall is
absorbed by the MCU's queued motion; neither shuts the printer down."""

from __future__ import annotations

import pytest

from tools.sim import configs

pytestmark = pytest.mark.needs_elf


def _mcu_log(world) -> str:
    return (world.log_dir / "h7.log").read_text(errors="replace")


def test_dropped_and_stalled_frames_are_survivable(sim_world):
    world = sim_world(
        lambda w: configs.minimal_config(w.h7_pty, str(w.gcode_dir)),
        dual_mcu=False,
    )
    world.gcode_ok("SET_KINEMATIC_POSITION X=125 Y=125 Z=125")

    world.inject("h7", drop_command="get_clock", count=2)
    world.gcode_ok("G1 X150 F3000")
    world.gcode_ok("G4 P3000")
    world.gcode_ok("M400")
    assert _mcu_log(world).count("[vtime] fault: dropping frame") == 2

    world.inject("h7", stall_s=0.05)
    world.gcode_ok("G1 X100 F3000")
    world.gcode_ok("M400")
    assert "[vtime] fault: stalling the link" in _mcu_log(world)

    assert world.shutdown_line() is None, world.log_tail()
    assert world.toolhead_position()[0] == pytest.approx(100.0, abs=0.01)
