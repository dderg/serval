"""[probe] / virtual-endstop validation against real firmware.

The auto-endstop walls in libsim_intercept.c stand in for physical
switches: X steps trip gpio200, Y gpio201, Z gpio202+gpio203.
"""

import re

import pytest

from tools.sim import configs

pytestmark = pytest.mark.needs_elf


def _cfg(variant):
    def make(world):
        return configs.probe_config(
            world.h7_pty,
            str(world.gcode_dir),
            variant,
            world.f4_pty if variant == "remote" else None,
        )

    return make


@pytest.mark.parametrize("variant", sorted(configs.PROBE_BOOT_ERRORS))
def test_bad_probe_config_rejected_at_boot(sim_world, variant):
    world = sim_world(
        _cfg(variant),
        spawn_mcus=False,
        expect_boot_error=configs.PROBE_BOOT_ERRORS[variant],
    )
    assert "Printer is ready" not in world.klippy_log_text()


def _last_probe_z(log_text):
    probe_lines = [line for line in log_text.splitlines() if " is z=" in line]
    m = re.search(r"is z=(-?\d+\.?\d*)", probe_lines[-1])
    assert m, "no probe result line"
    return float(m.group(1))


def _assert_probe_flow(world, variant):
    world.mark_log()
    world.gcode_ok("QUERY_PROBE")
    world.expect_log("probe: open")

    resp = world.gcode("PROBE", timeout=60)
    assert "Must home before probe" in str(resp.get("error", ""))

    world.mark_log()
    world.gcode_ok("G28", timeout=180)
    if variant in ("virtual", "safe-z", "points"):
        world.expect_log("homing: Z trigger=1.5000")

    z = world.toolhead_z()
    expected_z = {"safe-z": 10.0, "gpio-z": 5.0}.get(variant, 6.5)
    assert z == pytest.approx(expected_z, abs=0.1)
    if variant == "safe-z":
        pos = world.toolhead_position()
        assert pos[0] == pytest.approx(125.0, abs=0.5)
        assert pos[1] == pytest.approx(125.0, abs=0.5)

    world.mark_log()
    world.gcode_ok("PROBE", timeout=90)
    probe_out = world.expect_log(" is z=")
    probe_lines = [line for line in probe_out.splitlines() if " is z=" in line]
    m = re.search(r"is z=(-?\d+\.?\d*)", probe_lines[-1])
    expected_probe_z = 0.0 if variant == "gpio-z" else 1.5
    assert m and float(m.group(1)) == pytest.approx(expected_probe_z, abs=0.25)

    world.mark_log()
    world.gcode_ok("PROBE_ACCURACY SAMPLES=3", timeout=180)
    acc_out = world.expect_log("probe accuracy results")
    acc_lines = [
        line
        for line in acc_out.splitlines()
        if "probe accuracy results" in line
    ]
    m = re.search(r"range (\d+\.?\d*)", acc_lines[-1])
    assert m and float(m.group(1)) < 0.25

    world.mark_log()
    world.gcode_ok("QUERY_PROBE")
    world.expect_log("probe: open")


@pytest.mark.parametrize(
    "variant",
    ["virtual", "safe-z", "gpio-z"],
)
def test_probe_homing_and_probing(sim_world, variant):
    world = sim_world(_cfg(variant), dual_mcu=False)
    _assert_probe_flow(world, variant)
    assert world.shutdown_line() is None


def test_probe_multi_point_tools(sim_world):
    world = sim_world(_cfg("points"), dual_mcu=False)
    _assert_probe_flow(world, "points")

    world.mark_log()
    world.gcode_ok("SCREWS_TILT_CALCULATE", timeout=300)
    out = world.expect_log("back")
    assert "front left" in out

    world.mark_log()
    resp = world.gcode("BED_MESH_CALIBRATE", timeout=600)
    assert "activating a mesh is not supported" in str(resp.get("error", "")), (
        "BED_MESH_CALIBRATE probing should reach mesh activation, which the planner rejects"
    )

    world.mark_log()
    world.gcode_ok("FORCE_MOVE STEPPER=z DISTANCE=0.5 VELOCITY=5", timeout=60)
    world.gcode_ok("PROBE", timeout=90)
    shifted_z = _last_probe_z(world.expect_log(" is z="))
    assert abs(shifted_z - 1.5) == pytest.approx(0.5, abs=0.1)

    world.mark_log()
    world.gcode_ok("Z_TILT_ADJUST", timeout=300)
    world.expect_log("Making the following Z adjustments")

    world.mark_log()
    world.gcode_ok("PROBE", timeout=90)
    rebased_z = _last_probe_z(world.expect_log(" is z="))
    assert rebased_z == pytest.approx(1.5, abs=0.1), (
        "probe height must be frame-consistent after the common-mode rebase"
    )
    assert world.shutdown_line() is None


def test_probe_remote_mcu_trsync(sim_world):
    """Endstop trsync on a different MCU than the steppers."""
    world = sim_world(_cfg("remote"), dual_mcu=True)
    world.mark_log()
    world.gcode_ok("G28 Z", timeout=120)
    world.expect_log("set Z=3.2500")
    out = world.klippy_log_text()
    assert (
        "remote trsync terminal report" in out
        or "sim_remote_endstop: firing" in out
    )
    m = re.search(r"trip_to_stop_travel=(-?\d+\.\d+)", out)
    assert m, "no trip_to_stop_travel in homing log"
    assert -0.01 <= float(m.group(1)) < 0.5
    assert world.toolhead_z() == pytest.approx(8.25, abs=0.1)
    assert world.shutdown_line() is None
