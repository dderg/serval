from __future__ import annotations

import pathlib
import tempfile

import pytest

from tools.motion_eval import live, run, sweep

needs_live = pytest.mark.skipif(
    not live.LINUX_MCU, reason="KLIPPER_LINUX_MCU not set"
)
CASES = ("corners_turn90", "circles", "voron_top_layers")
LIMITERS = {"default": "", "corner_deviation": "[corner_deviation]\n"}


def _config(tmp, limiter):
    path = pathlib.Path(tmp) / "printer.cfg"
    path.write_text(
        "[include %s]\n\n%s" % (live.OWNER_LINUX_CFG, LIMITERS[limiter])
    )
    return path


@needs_live
@pytest.mark.parametrize("limiter", sorted(LIMITERS))
@pytest.mark.parametrize("case", CASES)
def test_live_steps_match_batch(case, limiter):
    text = (sweep.CORPUS / (case + ".gcode")).read_text()
    start = sweep.case_start(text)
    with tempfile.TemporaryDirectory() as tmp:
        config = _config(tmp, limiter)
        batch = run.run_klippy(
            config, text, start, dictionary=live.linux_dictionary()
        )
        real = live.run_live(config, text, start)
    assert real.state == "ready"
    assert real.print_wall_s > live.STATS_PERIOD_S
    assert real.status_errors == []
    assert real.status_polls >= real.print_wall_s / (2.0 * live.POLL_PERIOD_S)
    trapq = live.compare_trapq(batch.capture["moves"], real.capture["moves"])
    assert trapq["same"], trapq
    for name, s in live.compare_streams(batch.capture, real.capture).items():
        assert s["same_positions"], (name, s)
        if s["steps"] > 1:
            assert s["max_time_error_s"] <= s["allowed_s"], (name, s)
    for name, e in live.compare_execution(real.capture, real.executed).items():
        assert e["executed"] == e["commanded"], (name, e)
        assert e["executed_net"] == e["commanded_net"], (name, e)
