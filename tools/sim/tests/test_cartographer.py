"""Cartographer (scanner.py) emulation: scan homing, probing, touch,
and mesh scanning against the cartographer-klipper kalico-seam fork
(fetched into tools/sim/third_party_repos by fetch_plugins.sh).

The emulator tracks toolhead Z from the shim's step counters, so both
SCAN (count-threshold) and TOUCH (Z reaching the bed at 0) triggers
fire at step-accurate positions and clocks.
"""

import pathlib

import pytest

from tools.sim import configs

REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
CARTOGRAPHER_PLUGIN = (
    REPO_ROOT / "tools" / "sim" / "third_party_repos" / "cartographer_klipper"
)

pytestmark = [
    pytest.mark.needs_elf,
    pytest.mark.skipif(
        not CARTOGRAPHER_PLUGIN.exists(),
        reason="cartographer plugin not fetched (tools/sim/fetch_plugins.sh)",
    ),
]


def _cfg(mode="scan"):
    def make(world):
        return configs.cartographer_homing_config(
            world.h7_pty,
            world.f4_pty,
            world.cartographer_pty,
            str(world.gcode_dir),
            mode=mode,
        )

    return make


@pytest.fixture
def world(sim_world):
    return sim_world(_cfg(), cartographer=True)


def _home(world, z=100):
    world.gcode_ok(f"SET_KINEMATIC_POSITION X=150 Y=150 Z={z}", timeout=10)
    world.gcode_ok("G4 P1000", timeout=15)
    world.gcode_ok("G28 Z", timeout=120)


def test_connect_loads_scanner_module(world):
    world.gcode_ok("G4 P2000", timeout=15)
    log = world.klippy_log_text()
    assert "Traceback" not in log
    assert "Failed to load module 'scanner'" not in log
    assert world.shutdown_line() is None


def test_scan_homing(world):
    _home(world)
    assert world.shutdown_line() is None
    toolhead = world.status().get("toolhead", {})
    assert "z" in toolhead.get("homed_axes", "")


def test_scan_probing(world):
    _home(world)
    # Hop into the calibrated model range (0.2..5mm) before probing, as a
    # real print-start macro would.
    world.gcode_ok("G1 Z3 F600", timeout=30)
    world.gcode_ok("PROBE SAMPLES=2", timeout=120)
    world.gcode_ok("PROBE_ACCURACY SAMPLES=3", timeout=180)
    assert world.shutdown_line() is None


def test_scan_probe_reports_sane_z(world):
    _home(world)
    world.gcode_ok("G1 Z3 F600", timeout=30)
    world.gcode_ok("PROBE SAMPLES=2", timeout=120)
    probes = [
        line
        for line in world.klippy_log_text().splitlines()
        if line.startswith("probe at ")
    ]
    assert probes, "no probe result line in klippy.log"
    # "probe at x,y,Z is z=D": Z is the toolhead position, D the measured
    # sensor distance. With the emulator's bed at physical 0 the probe
    # result Z + trigger_distance - D must land near trigger_distance
    # (2.0); the slack is the saved polynomial's fit error against the
    # stub's analytic frequency model, twice (once at the homing trigger,
    # once at the probing height).
    toolhead_z = float(probes[-1].split(" is z=")[0].rsplit(",", 1)[1])
    dist = float(probes[-1].split(" is z=")[1].split()[0])
    bed_z = toolhead_z + 2.0 - dist
    assert 1.75 < bed_z < 2.25, probes[-1]


def test_touch_probe(sim_world):
    world = sim_world(_cfg(mode="touch"), cartographer=True)
    _home(world)
    world.gcode_ok("CARTOGRAPHER_TOUCH", timeout=300)
    assert world.shutdown_line() is None


def test_bed_mesh_scan(sim_world):
    world = sim_world(_cfg(), cartographer=True)
    _home(world)
    world.gcode_ok("G1 Z3 F600", timeout=30)
    world.gcode_ok("BED_MESH_CALIBRATE", timeout=600)
    assert "Mesh calibration complete" in world.klippy_log_text()
    assert world.shutdown_line() is None
