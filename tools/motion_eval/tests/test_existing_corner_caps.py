"""Existing corexy configs with max_corner_accel 21000 and 28000 at
max_accel 20000 (voron0's smoothers) start, plan a corpus case, and keep
every executed X/Y peak within the cap: what the moves alone ask of an
axis is at most max_accel, below the cap."""

import pathlib
import tempfile

import numpy as np
import pytest

from tools.motion_eval import evaluate, run, sweep

FAST_CFG = sweep.HERE / "configs" / "fast.cfg"
MAX_ACCEL = 20000
CASE = "voron_top_layers"
DT = 25e-6


def _config(max_corner_accel):
    return (
        "[include %s]\n\n"
        "[printer]\nmax_accel: %d\n\n"
        "[input_shaper]\nshaper_freq_x: 112.8\nshaper_freq_y: 90.2\n\n"
        "[corner_deviation]\nmax_corner_accel: %d\n"
        % (FAST_CFG, MAX_ACCEL, max_corner_accel)
    )


@pytest.mark.parametrize("max_corner_accel", [21000, 28000])
def test_existing_axis_cap_plans_within_the_cap(max_corner_accel):
    text = (sweep.CORPUS / (CASE + ".gcode")).read_text()
    with tempfile.TemporaryDirectory() as tmp:
        config = pathlib.Path(tmp) / "printer.cfg"
        config.write_text(_config(max_corner_accel))
        raw = run.run_klippy(config, text, sweep.case_start(text))
    moves = raw.capture["moves"]
    assert moves
    assert max(m["max_accel"] for m in moves) == MAX_ACCEL
    _, _, _, _, _, motion = evaluate.executed_motion(raw.capture, DT)
    for axis in "xy":
        assert np.abs(motion[axis].acc).max() <= max_corner_accel
