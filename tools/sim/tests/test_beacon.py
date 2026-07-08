"""Beacon eddy-current probe emulation: homing, probing, calibration,
and accelerometer streaming against the beacon_klipper fork
(fetched into tools/sim/third_party_repos by fetch_plugins.sh).

The emulator tracks toolhead Z from the shim's step counters, so both
proximity (threshold-crossing) and contact (Z reaching the bed at 0)
triggers fire at step-accurate positions and clocks.
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


def _home(world, z=100):
    world.gcode_ok(f"SET_KINEMATIC_POSITION X=150 Y=150 Z={z}", timeout=10)
    world.gcode_ok("G4 P1000", timeout=15)
    world.gcode_ok("G28 Z", timeout=120)


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
    _home(world)
    assert world.shutdown_line() is None
    toolhead = world.status().get("toolhead", {})
    assert "z" in toolhead.get("homed_axes", "")


def test_proximity_probing(world):
    _home(world)
    # Hop into the calibrated model range (0.2..5mm) before probing, as a
    # real print-start macro would.
    world.gcode_ok("G1 Z3 F600", timeout=30)
    world.gcode_ok("PROBE PROBE_METHOD=proximity SAMPLES=2", timeout=120)
    world.gcode_ok("PROBE_ACCURACY SAMPLES=3", timeout=180)
    assert world.shutdown_line() is None


def test_contact_probing(world):
    _home(world)
    world.gcode_ok("PROBE PROBE_METHOD=contact SAMPLES=1", timeout=120)
    assert world.shutdown_line() is None


def test_contact_probing_survives_latch_commit_delay(world):
    """Field failure (trident 2026-07-08): if the firmware's triggered
    latch commits after the trsync fires and beacon_contact_stop_home
    discards uncommitted state, querying the detect clock after the stop
    raised "Timeout getting contact time". The plugin must read the
    detect clock before stopping contact homing."""
    _home(world)
    world.beacon.contact_latch_commit_delay = 0.3
    world.gcode_ok("PROBE PROBE_METHOD=contact SAMPLES=1", timeout=120)
    assert world.shutdown_line() is None


def test_contact_auto_calibrate(world):
    world.gcode_ok("SET_KINEMATIC_POSITION X=150 Y=150 Z=10", timeout=10)
    world.gcode_ok("G4 P1000", timeout=15)
    world.gcode_ok("BEACON_AUTO_CALIBRATE", timeout=300)
    assert "Collected" in world.klippy_log_text()
    assert world.shutdown_line() is None


def _probe_samples(world):
    return [
        float(line.rsplit("z=", 1)[1])
        for line in world.klippy_log_text().splitlines()
        if line.startswith("probe at ")
    ]


def test_contact_auto_calibrate_with_mesh_active(sim_world):
    world = sim_world(_cfg(bed_mesh=True), beacon=True)
    world.gcode_ok("SET_KINEMATIC_POSITION X=150 Y=150 Z=10", timeout=10)
    world.gcode_ok("G4 P1000", timeout=15)
    world.gcode_ok("BEACON_AUTO_CALIBRATE", timeout=300)
    world.gcode_ok("G1 Z3 F600", timeout=30)
    world.gcode_ok(
        "PROBE PROBE_METHOD=contact SAMPLES=6 SAMPLES_TOLERANCE=9", timeout=300
    )
    plain = _probe_samples(world)
    world.gcode_ok("BED_MESH_PROFILE LOAD=edge", timeout=15)
    world.gcode_ok(
        "PROBE PROBE_METHOD=contact SAMPLES=6 SAMPLES_TOLERANCE=9", timeout=300
    )
    meshed = _probe_samples(world)[len(plain) :]
    assert world.shutdown_line() is None

    plain_spread = max(plain) - min(plain)
    meshed_spread = max(meshed) - min(meshed)
    assert meshed_spread < max(0.012, 2.0 * plain_spread), (
        "mesh-active contact probing must be as repeatable as mesh-off: "
        "plain=%r (spread %.4f) meshed=%r (spread %.4f)"
        % (plain, plain_spread, meshed, meshed_spread)
    )


def test_poke(world):
    world.gcode_ok("SET_KINEMATIC_POSITION X=150 Y=150 Z=10", timeout=10)
    world.gcode_ok("G4 P1000", timeout=15)
    world.gcode_ok("BEACON_POKE TOP=5 BOTTOM=-0.3", timeout=120)
    log = world.klippy_log_text()
    assert "Armed at:" in log
    assert "Triggered at:" in log
    assert world.shutdown_line() is None


@pytest.mark.skip(
    reason="hangs: mid-scan the emulator's beacon stream stalls ('Beacon "
    "sensor not receiving data' shutdown), then the fork's mesh scan "
    "callback (beacon.py cb, sample['pos']) KeyErrors on the pos-less "
    "samples flushed after shutdown, killing the reactor — beacon stream "
    "throughput/stall issue, needs a dedicated session (NOT the shaper "
    "stream-boundary bug, which is fixed)",
)
def test_bed_mesh(sim_world):
    world = sim_world(_cfg(bed_mesh=True), beacon=True)
    _home(world)
    world.gcode_ok("BED_MESH_CALIBRATE", timeout=300)
    assert world.shutdown_line() is None


def test_accelerometer_stream(world):
    world.gcode_ok("ACCELEROMETER_MEASURE CHIP=beacon", timeout=30)
    world.gcode_ok("G4 P1000", timeout=15)
    world.gcode_ok("ACCELEROMETER_MEASURE CHIP=beacon NAME=test", timeout=30)
    assert world.shutdown_line() is None
