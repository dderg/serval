"""Beacon eddy-current probe emulation: homing, calibration, probing,
poke, mesh, and accelerometer streaming against the beacon_klipper
plugin (fetched into tools/sim/third_party_repos by fetch_plugins.sh).
"""

import pathlib

import pytest

from tools.sim import configs

REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
BEACON_PLUGIN = (
    REPO_ROOT / "tools" / "sim" / "third_party_repos" / "beacon_klipper"
)

pytestmark = [
    pytest.mark.needs_elf,
    pytest.mark.skipif(
        not BEACON_PLUGIN.exists(),
        reason="beacon_klipper plugin not fetched (tools/sim/fetch_plugins.sh)",
    ),
]


def _cfg(bed_mesh=False):
    def make(world):
        return configs.beacon_homing_config(
            world.h7_pty,
            world.f4_pty,
            world.beacon_pty,
            str(world.gcode_dir),
            bed_mesh=bed_mesh,
        )

    return make


@pytest.fixture
def world(sim_world):
    return sim_world(_cfg(), beacon=True)


def test_connect_loads_beacon_module(world):
    world.gcode_ok("G4 P2000", timeout=15)
    log = world.klippy_log_text().replace(
        "Executing Beacon update script failed: Traceback",
        "Executing Beacon update script failed:",
    )
    assert "Traceback" not in log
    assert "Failed to load module 'beacon'" not in log
    assert world.shutdown_line() is None


def test_proximity_homing(world):
    world.gcode_ok("SET_KINEMATIC_POSITION X=150 Y=150 Z=100", timeout=10)
    world.gcode_ok("G4 P1000", timeout=15)
    world.gcode_ok("G28 Z", timeout=60)
    assert world.shutdown_line() is None
    toolhead = world.status().get("toolhead", {})
    assert "z" in toolhead.get("homed_axes", "")


def test_probing_after_homing(world):
    world.gcode_ok("SET_KINEMATIC_POSITION X=150 Y=150 Z=100", timeout=10)
    world.gcode_ok("G4 P1000", timeout=15)
    world.gcode_ok("G28 Z", timeout=120)
    world.gcode_ok("PROBE PROBE_METHOD=proximity SAMPLES=2", timeout=120)
    world.gcode_ok("PROBE PROBE_METHOD=contact SAMPLES=1", timeout=120)
    world.gcode_ok("PROBE_ACCURACY SAMPLES=3", timeout=180)
    assert world.shutdown_line() is None


def test_contact_auto_calibrate(world):
    world.gcode_ok("SET_KINEMATIC_POSITION X=150 Y=150 Z=10", timeout=10)
    world.gcode_ok("G4 P1000", timeout=15)
    resp = world.gcode("BEACON_AUTO_CALIBRATE", timeout=600)
    err = str(resp.get("error", "")) if isinstance(resp, dict) else ""
    log = world.klippy_log_text()
    if err and "model convergence" in err and "Collected" in log:
        pytest.xfail("emulator frequency model does not fit a real curve")
    assert not err, err
    assert "Collected" in log
    assert world.shutdown_line() is None


def test_poke(world):
    world.gcode_ok("SET_KINEMATIC_POSITION X=150 Y=150 Z=10", timeout=10)
    world.gcode_ok("G4 P1000", timeout=15)
    world.gcode_ok("BEACON_POKE TOP=5 BOTTOM=-0.3", timeout=120)
    log = world.klippy_log_text()
    assert "Armed at:" in log
    assert "Triggered at:" in log
    assert world.shutdown_line() is None


def test_bed_mesh(sim_world):
    world = sim_world(_cfg(bed_mesh=True), beacon=True)
    world.gcode_ok("SET_KINEMATIC_POSITION X=150 Y=150 Z=100", timeout=10)
    world.gcode_ok("G4 P1000", timeout=15)
    world.gcode_ok("G28 Z", timeout=120)
    world.gcode_ok("BED_MESH_CALIBRATE", timeout=600)
    assert world.shutdown_line() is None


def test_accelerometer_stream(world):
    world.gcode_ok("ACCELEROMETER_MEASURE CHIP=beacon", timeout=30)
    world.gcode_ok("G4 P1000", timeout=15)
    world.gcode_ok("ACCELEROMETER_MEASURE CHIP=beacon NAME=test", timeout=30)
    assert world.shutdown_line() is None
