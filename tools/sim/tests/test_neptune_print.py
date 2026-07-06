"""Neptune-bench print profile on real firmware: arc_fit + extruder follower
with the bench's pressure-advance/smoothing chain, replaying the first layers
of the Voron cube slice that aborted the bench pump (pump_piece_in_past)."""

import pathlib

import pytest

from tools.sim import configs

pytestmark = pytest.mark.needs_elf

GCODE = (
    pathlib.Path(__file__).resolve().parents[1]
    / "gcode"
    / "voron_first_layers.gcode"
)


def test_voron_first_layers_print(sim_world):
    world = sim_world(
        lambda w: configs.neptune_print_config(w.h7_pty, str(w.gcode_dir)),
        dual_mcu=False,
    )
    world.gcode_ok("SET_KINEMATIC_POSITION X=0 Y=0 Z=0", timeout=10)
    print_time = world.print_file(GCODE, timeout=1800)
    assert print_time > 0

    events = world.events_text()
    for fatal in ("pump_piece_in_past", "runtime_fault", "diag.rust_fault"):
        assert fatal not in events, (
            f"{fatal} during print:\n{world.log_tail()[-3000:]}"
        )
    stalls = events.count("axis_stalled")
    underruns = events.count("anchor_underrun")
    print(
        f"print_time={print_time:.1f}s axis_stalled={stalls} anchor_underrun={underruns}"
    )
    assert world.shutdown_line() is None
