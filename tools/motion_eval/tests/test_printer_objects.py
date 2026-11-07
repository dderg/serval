from __future__ import annotations

import json
import os
import pathlib
import subprocess
import sys
import tempfile

import pytest

from klippy.extras.heaters import MAX_HEAT_TIME
from tools.motion_eval import mcu_dict, run, sweep
from tools.motion_eval.live import PI5_SLOWDOWN

LIMITERS = {"default": "", "corner_deviation": "[corner_deviation]\n"}


SHAPER_SWITCHES = """G1 X110 Y110 F6000
SET_INPUT_SHAPER SHAPER_TYPE=smooth_ei SHAPER_FREQ_X=30 SHAPER_FREQ_Y=25
G1 X120 Y100
SET_INPUT_SHAPER SHAPER_TYPE=smooth_mzv SHAPER_FREQ_X=200 SHAPER_FREQ_Y=130
G1 X100 Y120
SET_INPUT_SHAPER SHAPER_TYPE=mzv SHAPER_FREQ_X=40 SHAPER_FREQ_Y=35
G1 X110 Y110
"""
STALL_BUDGET_S = 0.5 * MAX_HEAT_TIME / PI5_SLOWDOWN


def probe_run(section, gcode="G1 X110 Y110 F6000\n"):
    """Run klippy batch on fast.cfg (input shaper included) plus section,
    and return what object_probe found after klippy:ready."""
    with tempfile.TemporaryDirectory() as tmp:
        tmp = pathlib.Path(tmp)
        config = tmp / "printer.cfg"
        config.write_text(
            "[include %s]\n\n%s" % (sweep.CONFIGS["fast"]["cfg"], section)
        )
        gcode_text, gcode = gcode, tmp / "in.gcode"
        gcode.write_text(run.wrap_gcode(gcode_text, (100, 100, 10)))
        result = tmp / "result.json"
        log = tmp / "klippy.log"
        res = subprocess.run(
            [
                sys.executable,
                "-m",
                "tools.motion_eval.object_probe",
                str(result),
                str(config),
                "-i",
                str(gcode),
                "-o",
                str(tmp / "out"),
                "-d",
                str(mcu_dict.build(mcu_dict.DEFAULT_MCU_CONFIG)),
                "-l",
                str(log),
            ],
            cwd=run.REPO_ROOT,
            env=dict(os.environ, PYTHONPATH=str(run.REPO_ROOT)),
            capture_output=True,
            text=True,
        )
        assert res.returncode == 0 and result.exists(), (
            res.stderr[-3000:] + log.read_text()[-3000:]
        )
        results = json.loads(result.read_text())
    assert results["failures"] is not None, "klippy:ready never fired"
    return results


@pytest.mark.parametrize("limiter", sorted(LIMITERS))
def test_live_only_object_hooks_work(limiter):
    assert probe_run(LIMITERS[limiter])["failures"] == []


@pytest.mark.parametrize("limiter", sorted(LIMITERS))
def test_shaper_changes_do_not_stall_the_reactor(limiter):
    """A heater must see a new PWM value within MAX_HEAT_TIME; on a host
    PI5_SLOWDOWN times slower, half of that is the most one reactor
    callback may take here."""
    results = probe_run(LIMITERS[limiter], SHAPER_SWITCHES)
    assert results["failures"] == []
    assert results["max_stall_s"] <= STALL_BUDGET_S
