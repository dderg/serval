"""Prints whose follower-source or motor-command fit panicked klippy's
shaper (#885, #890), on the bench printer.cfg with sim pins.

#885: bell X/Y kernels under a Z+E move, and a reciprocal advance whose
read velocity touches zero at a ratio break, left the extruder's source
without an enclosure to prove its fit against. #890: mode_inverse after the
X/Y kernels stepped the motor command's velocity where a kernel edge passed
a seam of its source fit.
"""

import pathlib

import pytest

from tools.sim import configs

pytestmark = pytest.mark.needs_elf

PRINT_TIMEOUT_S = 120
FATAL_EVENTS = (
    "panicked",
    "stream worker fatal",
    "stream_worker_fatal",
    "runtime_fault",
)
EPOCHS = (
    pathlib.Path(__file__).resolve().parents[3]
    / "tools/bench/gcode/epochs.gcode"
)
EPOCHS_PREFIX_LINES = 28

CASES = {
    "bell_z_e_move": (
        configs.BenchXy.BELL,
        configs.BenchAdvance.LINEAR,
        "X=40 Y=150 Z=10",
        "G90\nM83\nG1 X40 Y150 Z10 F6000\nG1 X240 Y150 Z50 E38.6 F54000\n",
    ),
    "mode_inverse_corner": (
        configs.BenchXy.MODE_INVERSE,
        configs.BenchAdvance.LINEAR,
        "X=0 Y=0 Z=0",
        "G1 X0 Y0\nG1 X10 Y0\nG1 X10.872 Y0.038\n",
    ),
}


def _epochs_prefix():
    lines = EPOCHS.read_text().splitlines()
    return "\n".join(lines[:EPOCHS_PREFIX_LINES]) + "\n"


def _run(sim_world, xy, advance, start, gcode):
    world = sim_world(
        lambda w: configs.bench_print_config(
            w.h7_pty, str(w.gcode_dir), xy=xy, advance=advance
        ),
        dual_mcu=False,
    )
    world.gcode_ok(f"SET_KINEMATIC_POSITION {start}")
    path = world.gcode_dir / "follower_source_fits.gcode"
    path.write_text(gcode)
    world.print_file(path, timeout=PRINT_TIMEOUT_S)
    world.gcode_ok("M400", timeout=60)
    stdout = (world.log_dir / "klippy.stdout").read_text(errors="replace")
    events = world.events_text()
    for fatal in FATAL_EVENTS:
        assert fatal not in stdout and fatal not in events, stdout[-3000:]
    assert world.shutdown_line() is None, world.log_tail()[-3000:]


@pytest.mark.parametrize("case", CASES)
def test_shaped_print_plans(sim_world, case):
    _run(sim_world, *CASES[case])


def test_recipr_advance_through_epochs(sim_world):
    _run(
        sim_world,
        configs.BenchXy.KERNELS,
        configs.BenchAdvance.RECIPR,
        "X=60 Y=60 Z=0",
        _epochs_prefix(),
    )
