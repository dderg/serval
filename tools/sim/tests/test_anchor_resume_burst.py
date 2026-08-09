"""The M400-drain -> dense-burst seam that killed klippy on the Voron 0.

Bench shape (repro_z14.gcode, 2026-08-02): G28, approach the layer, M400,
then ~5770 dense absolute XY moves. The M400 drain parks the machine, so the
burst's first commit is a fresh anchor, and a fresh anchor grants the whole
runway the resumed stream then has to survive the producer's next hiccup.
The producer's first step after a refill from empty is its most expensive
one, and on a flat 250 ms lead the segment behind it landed 245 ms past the
playhead -> anchor_underrun -> stream_worker_fatal.

The anchor is transport-agnostic, so this runs in piece mode: the
stepcompress sim MCU cannot retire a dense micro-segment burst in real time
on the virtual clock and times out before the seam is reached.

Streamed via SD so klippy's lookahead fills the way it does on the bench.
"""

from __future__ import annotations

import pathlib

import pytest

from tools.sim import configs

pytestmark = pytest.mark.needs_elf

SEGMENT_MM = 0.5
BURST_SEGMENTS = 900
FEEDRATE_MM_MIN = 4800


def _write_resume_burst_gcode(path: pathlib.Path) -> None:
    lines = [
        "G90",
        "SET_KINEMATIC_POSITION X=125 Y=125 Z=20",
        "G1 X60 Y60 F3000",
        "G1 Z14.4 F300",
        "M400",
    ]
    x, y = 60.0, 60.0
    step = SEGMENT_MM
    for i in range(BURST_SEGMENTS):
        x += step
        y += step if i % 2 == 0 else -step
        if x > 180.0:
            x, step = 60.0, SEGMENT_MM
        lines.append(f"G1 X{x:.4f} Y{y:.4f} F{FEEDRATE_MM_MIN}")
    lines.append("M400")
    path.write_text("\n".join(lines) + "\n")


def test_dense_burst_straight_after_a_drain_does_not_underrun(sim_world):
    world = sim_world(
        lambda w: configs.minimal_config(w.h7_pty, str(w.gcode_dir)),
        dual_mcu=False,
    )
    gpath = world.gcode_dir / "anchor_resume_burst.gcode"
    _write_resume_burst_gcode(gpath)
    world.print_file(gpath, timeout=600)

    events = world.events_text()
    resumes = events.count('"anchor_idle_resume"')
    anchors = events.count('"anchor_decision"')
    print(f"\n[resume-burst] fresh anchors={anchors} idle resumes={resumes}")
    assert world.shutdown_line() is None, world.log_tail()
    assert events.count('"anchor_underrun"') == 0, events[-4000:]
    assert events.count('"anchor_low_margin"') == 0, events[-4000:]
    assert anchors >= 2, (
        "the M400 never parked the stream, so the burst did not resume onto a"
        f" fresh anchor — the seam under test was not exercised ({events[-2000:]})"
    )
    assert world.toolhead_position()[2] == pytest.approx(14.4, abs=0.01)
