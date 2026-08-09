"""A probe trip that exhausts its travel without triggering (the z_tilt
"range too large" failure) must leave the host toolhead at the true stop
position, not the pre-probe height — a stale height makes the next G28's
hop logic skip the retract and drag the nozzle through the bed.
"""

import pytest

from tools.sim import configs

pytestmark = pytest.mark.needs_elf


def test_failed_probe_adopts_true_stop_position(sim_world):
    world = sim_world(
        lambda w: configs.probe_config(w.h7_pty, str(w.gcode_dir), "virtual"),
        dual_mcu=False,
    )
    world.gcode_ok("G28", timeout=180)
    assert world.toolhead_z() == pytest.approx(6.5, abs=0.1)

    world.gcode_ok("G1 Z10 F600", timeout=60)
    world.gcode_ok("M400", timeout=60)
    # Rename the physical rest point (8.5mm above the endstop wall) to
    # gcode Z=1: the probe window (position_min=-5) now runs out 2.5mm
    # above the wall, reproducing the tilted-bed no-trigger failure.
    world.gcode_ok("SET_KINEMATIC_POSITION Z=1", timeout=60)

    world.mark_log()
    resp = world.gcode("PROBE", timeout=90)
    assert "did not trigger" in str(resp.get("error", "")), resp

    pos = world.toolhead_position()
    assert pos[2] == pytest.approx(-5.0, abs=0.1), (
        "after a no-trigger probe the toolhead must report the true stop"
        " position (bottom of the probe window), not the pre-probe height:"
        f" {pos}"
    )
    homed = world.status()["toolhead"]["homed_axes"]
    assert "z" in homed, (
        "the stop position is reconciled from executed motion — Z must"
        f" stay homed so hop/retract logic keeps working: {homed!r}"
    )
    assert "trip_aborted_position_adopted" in world.events_text()

    world.mark_log()
    world.gcode_ok("G28 Z", timeout=120)
    assert world.toolhead_z() == pytest.approx(6.5, abs=0.1)
    assert world.shutdown_line() is None
