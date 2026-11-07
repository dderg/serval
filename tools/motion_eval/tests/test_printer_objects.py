from __future__ import annotations

import json
import os
import pathlib
import subprocess
import sys
import tempfile

import pytest

from tools.motion_eval import mcu_dict, run, sweep

LIMITERS = {"default": "", "corner_deviation": "[corner_deviation]\n"}


def probe_failures(section):
    """Run klippy batch on fast.cfg (input shaper included) plus section,
    and collect what object_probe found after klippy:ready."""
    with tempfile.TemporaryDirectory() as tmp:
        tmp = pathlib.Path(tmp)
        config = tmp / "printer.cfg"
        config.write_text(
            "[include %s]\n\n%s" % (sweep.CONFIGS["fast"]["cfg"], section)
        )
        gcode = tmp / "in.gcode"
        gcode.write_text(run.wrap_gcode("G1 X110 Y110 F6000\n", (100, 100, 10)))
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
        failures = json.loads(result.read_text())["failures"]
    assert failures is not None, "klippy:ready never fired"
    return failures


@pytest.mark.parametrize("limiter", sorted(LIMITERS))
def test_live_only_object_hooks_work(limiter):
    assert probe_failures(LIMITERS[limiter]) == []
