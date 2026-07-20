"""PROBE_CALIBRATE end-to-end: auto probe, manual TESTZ session, z_offset.

Also covers the klicky/dockable pattern of wrapping PROBE_CALIBRATE in a
gcode_macro via rename_existing, which requires the command to exist at
connect time.
"""

import re

import pytest

from tools.sim import configs

pytestmark = pytest.mark.needs_elf

X_OFFSET = 24.0
Y_OFFSET = 5.0

KLICKY_MACRO = """
[gcode_macro PROBE_CALIBRATE]
rename_existing: _PROBE_CALIBRATE
gcode:
    _PROBE_CALIBRATE {rawparams}
"""


def _cfg(extra=""):
    def make(world):
        return (
            configs.probe_config(world.h7_pty, str(world.gcode_dir), "default")
            + extra
        )

    return make


def _z_offset_from_log(out):
    lines = [line for line in out.splitlines() if "probe: z_offset:" in line]
    assert lines, "no z_offset result line"
    m = re.search(r"probe: z_offset: (-?\d+\.\d+)", lines[-1])
    assert m
    return float(m.group(1))


def test_probe_calibrate_requires_homing(sim_world):
    world = sim_world(_cfg(), dual_mcu=False)
    resp = world.gcode("PROBE_CALIBRATE", timeout=60)
    assert "Must home before probe" in str(resp.get("error", ""))


def test_probe_calibrate_accept_flow(sim_world):
    world = sim_world(_cfg(), dual_mcu=False)
    world.gcode_ok("G28", timeout=180)
    start_pos = world.toolhead_position()

    world.mark_log()
    world.gcode_ok("PROBE_CALIBRATE", timeout=120)
    world.expect_log("Starting manual Z probe")

    pos = world.toolhead_position()
    assert pos[0] == pytest.approx(start_pos[0] + X_OFFSET, abs=0.1)
    assert pos[1] == pytest.approx(start_pos[1] + Y_OFFSET, abs=0.1)
    probe_z = pos[2] - 5.0
    assert probe_z == pytest.approx(1.5, abs=0.25)

    world.gcode_ok("TESTZ Z=-5.2", timeout=60)
    assert world.toolhead_z() == pytest.approx(pos[2] - 5.2, abs=0.05)

    world.mark_log()
    world.gcode_ok("ACCEPT", timeout=60)
    out = world.expect_log("probe: z_offset:")
    assert _z_offset_from_log(out) == pytest.approx(0.2, abs=0.1)

    world.mark_log()
    world.gcode_ok("TESTZ Z=-1", timeout=30)
    world.expect_log("Unknown command")
    assert world.shutdown_line() is None


def test_probe_calibrate_abort_flow(sim_world):
    world = sim_world(_cfg(), dual_mcu=False)
    world.gcode_ok("G28", timeout=180)

    world.mark_log()
    world.gcode_ok("PROBE_CALIBRATE", timeout=120)
    world.expect_log("Starting manual Z probe")
    world.gcode_ok("ABORT", timeout=60)

    assert "probe: z_offset:" not in world.log_tail()
    world.mark_log()
    world.gcode_ok("ACCEPT", timeout=30)
    world.expect_log("Unknown command")

    resp = world.gcode("PROBE_CALIBRATE", timeout=120)
    assert "error" not in resp, "PROBE_CALIBRATE must restart after ABORT"
    world.gcode_ok("ABORT", timeout=60)
    assert world.shutdown_line() is None


def test_probe_calibrate_rename_existing_macro(sim_world):
    world = sim_world(_cfg(KLICKY_MACRO), dual_mcu=False)
    world.gcode_ok("G28", timeout=180)

    world.mark_log()
    world.gcode_ok("PROBE_CALIBRATE", timeout=120)
    world.expect_log("Starting manual Z probe")
    world.gcode_ok("ABORT", timeout=60)
    assert world.shutdown_line() is None
