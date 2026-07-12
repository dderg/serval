"""M400 after G4 must wait the dwell out on the MCU clock.

The engine drains straight through a dwell (it queues time, not motion), so
a drain-only wait returns immediately and anything sequenced against the
dwell races it — the bench failure was SERVO_SYNC re-enabling torque while
its scheduled disable was still pending, canceling the release outright.

The assertion is clock-domain exact and independent of how fast the sim's
virtual clock runs: after M400 returns, the host's estimate of the MCU
clock must have reached the queued print-time frontier (which includes the
dwell), give or take the standing scheduling lead.
"""

import pytest

from tools.sim import configs

pytestmark = pytest.mark.needs_elf

DWELL_S = 3.0
LEAD_MARGIN_S = 1.0


def toolhead_status(world) -> dict:
    return world.status({"toolhead": None})["toolhead"]


def test_m400_waits_out_a_dwell_on_the_mcu_clock(sim_world):
    world = sim_world(
        lambda w: configs.minimal_config(w.h7_pty, str(w.gcode_dir)),
        dual_mcu=False,
    )
    world.gcode_ok("SET_KINEMATIC_POSITION X=125 Y=125 Z=125")
    world.gcode_ok("G1 X130 F3000")
    world.gcode_ok("M400")

    world.gcode_ok("G4 P%d" % int(DWELL_S * 1000))
    frontier = toolhead_status(world)["print_time"]
    world.gcode_ok("M400", timeout=DWELL_S + 30.0)
    est_after = toolhead_status(world)["estimated_print_time"]

    assert est_after >= frontier - LEAD_MARGIN_S, (
        "M400 returned %.3fs of MCU clock before the dwell's queued "
        "frontier — the dwell was discarded instead of waited out"
        % (frontier - est_after)
    )
