"""Streaming with `max_jerk: 0` (the default: no jerk limit).

Unlimited jerk stops with acceleration still applied, so a trailing
derivative-gain stage — linear pressure advance here — leaves the parked
extruder's commanded velocity nonzero. The dispatcher used to read the park
off that end derivative and concluded the machine was mid-motion, so the
next resume across an idle gap aborted klippy with `anchor_underrun`
instead of re-anchoring forward.
"""

import pytest

from tools.sim import configs

pytestmark = pytest.mark.needs_elf


def _cfg(world):
    return configs.heaters_config(
        world.h7_pty, str(world.gcode_dir), max_jerk=0
    )


def test_extruding_moves_resume_across_host_round_trips(sim_world):
    # Each gcode_ok is a host round trip: the stream drains to rest and the
    # playhead runs past the committed end long before the next move lands,
    # which is the resume the anchor has to re-anchor rather than fault.
    world = sim_world(_cfg, dual_mcu=False)
    world.gcode_ok("SET_KINEMATIC_POSITION X=20 Y=20 Z=1")
    world.gcode_ok("G1 X25 Y25 E7.5")
    world.gcode_ok("M400", timeout=60)
    world.gcode_ok("SET_PRESSURE_ADVANCE EXTRUDER=extruder ADVANCE=0.025")
    world.gcode_ok("G1 X30 Y30 E8.0")
    world.gcode_ok("G1 X25 Y25")
    world.gcode_ok("M400", timeout=60)
    world.gcode_ok("G1 X20 Y20 E0.5")
    world.gcode_ok("M400", timeout=60)
    assert world.shutdown_line() is None
