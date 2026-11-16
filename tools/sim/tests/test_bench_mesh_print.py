"""The bench printer.cfg prints the first layers with its "wavy" bed mesh
loaded, straight through and with a PAUSE/RESUME on the way.

Bench finding F1 (sweep of 0b90eeaf3): this print aborted klippy in
`kalico-shaper` with a follower seam weld over a 14.5 us segment exceeding
the acceleration budget, because the mesh-warped z leader was fitted with
velocity steps at its seams and the extruder's advance turned them into
position steps.
"""

import pathlib
import time

import pytest

from tools.sim import configs

pytestmark = pytest.mark.needs_elf

GCODE = (
    pathlib.Path(__file__).resolve().parents[1]
    / "gcode"
    / "voron_first_layers.gcode"
)
PRINT_TIMEOUT_S = 1800
PAUSE_AFTER_PRINT_S = 20.0
INTERRUPTIONS = ("straight", "pause_resume")
PRINT_POLL_S = 0.5


def _wait_print_time(world, print_s: float) -> None:
    deadline = world.vclock.deadline(PRINT_TIMEOUT_S)
    with world.vclock.running_free():
        while not deadline.expired():
            stats = world.status({"print_stats": None}).get("print_stats", {})
            if (stats.get("print_duration") or 0.0) >= print_s:
                return
            time.sleep(PRINT_POLL_S)
    raise AssertionError(f"the print never reached {print_s} s")


@pytest.mark.parametrize("interruption", INTERRUPTIONS)
def test_first_layers_over_the_wavy_mesh(sim_world, interruption):
    world = sim_world(
        lambda w: configs.bench_mesh_print_config(w.h7_pty, str(w.gcode_dir)),
        dual_mcu=False,
    )
    world.gcode_ok("SET_KINEMATIC_POSITION X=0 Y=0 Z=0", timeout=10)
    world.gcode_ok("BED_MESH_PROFILE LOAD=wavy")
    if interruption == "straight":
        world.print_file(GCODE, timeout=PRINT_TIMEOUT_S)
    else:
        (world.gcode_dir / GCODE.name).write_bytes(GCODE.read_bytes())
        world.gcode_ok(f"SDCARD_PRINT_FILE FILENAME={GCODE.name}")
        _wait_print_time(world, PAUSE_AFTER_PRINT_S)
        world.gcode_ok("PAUSE", timeout=PRINT_TIMEOUT_S)
        world.gcode_ok("RESUME", timeout=PRINT_TIMEOUT_S)
        world.wait_print_done(timeout=PRINT_TIMEOUT_S)
    world.gcode_ok("M400", timeout=PRINT_TIMEOUT_S)

    assert world.shutdown_line() is None, world.log_tail()[-3000:]
    stdout = (world.log_dir / "klippy.stdout").read_text(errors="replace")
    assert "panicked" not in stdout, stdout[-3000:]
