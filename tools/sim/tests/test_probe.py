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


_TRIP_RESOLUTION_XFAIL = pytest.mark.xfail(
    reason="G28 trip-time resolution fails under the virtual clock: the "
    "vtime pacer ties virtual time to the (deprioritized) motion tick "
    "thread, so under load the MCU clock crawls relative to real time "
    "while klippy's clocksync still estimates ~50MHz — the trigger clock "
    "then maps outside the retained motion-history window ('query host "
    "time precedes retained motion history'). Deterministic repro; needs "
    "a dedicated clocksync/vtime session.",
)


@pytest.mark.parametrize("variant", ["virtual", "safe-z", "gpio-z"])
@_TRIP_RESOLUTION_XFAIL
def test_probe_homing_and_probing(sim_world, variant):
    world = sim_world(_cfg(variant), dual_mcu=False)
    _assert_probe_flow(world, variant)
    assert world.shutdown_line() is None


@_TRIP_RESOLUTION_XFAIL
def test_probe_multi_point_tools(sim_world):
    world = sim_world(_cfg("points"), dual_mcu=False)
    _assert_probe_flow(world, "points")

    world.mark_log()
    world.gcode_ok("SCREWS_TILT_ADJUST", timeout=300)
    out = world.expect_log("front left")
    assert "back" in out

    world.mark_log()
    world.gcode_ok("BED_MESH_CALIBRATE", timeout=600)
    world.expect_log("Mesh Bed Leveling Complete")

    world.mark_log()
    resp = world.gcode("Z_TILT_ADJUST", timeout=300)
    assert "per-motor Z adjustment is not yet implemented" in str(
        resp.get("error", "")
    )
    world.expect_log("Z adjustments needed")
    assert world.shutdown_line() is None


@_TRIP_RESOLUTION_XFAIL
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
    assert 0.0 <= float(m.group(1)) < 0.5
    assert world.toolhead_z() == pytest.approx(8.25, abs=0.1)
    assert world.shutdown_line() is None
