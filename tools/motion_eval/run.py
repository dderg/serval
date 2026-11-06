"""Run a printer.cfg + gcode through klippy batch mode and collect results."""

from __future__ import annotations

import dataclasses
import json
import pathlib
import subprocess
import sys
import tempfile

from . import mcu_dict, stepdecode

REPO_ROOT = mcu_dict.REPO_ROOT
MAIN_MCU = "mcu"
LOG_TAIL_CHARS = 6000


class KlippyRunError(RuntimeError):
    pass


@dataclasses.dataclass
class RawRun:
    capture: dict
    traces: dict


def wrap_gcode(gcode: str, start_xyz) -> str:
    x, y, z = start_xyz
    return "SET_KINEMATIC_POSITION X=%r Y=%r Z=%r\n%s\nM400\n" % (
        x,
        y,
        z,
        gcode,
    )


def run_klippy(
    config_path, gcode: str, start_xyz, mcu_config=mcu_dict.DEFAULT_MCU_CONFIG
) -> RawRun:
    dictionary = mcu_dict.build(mcu_config)
    with tempfile.TemporaryDirectory(prefix="motion_eval_") as tmp:
        tmpdir = pathlib.Path(tmp)
        gcode_path = tmpdir / "input.gcode"
        gcode_path.write_text(wrap_gcode(gcode, start_xyz))
        output_path = tmpdir / "mcu.out"
        log_path = tmpdir / "klippy.log"
        capture_path = tmpdir / "capture.json"
        cmd = [
            sys.executable,
            "-m",
            "tools.motion_eval.capture",
            str(capture_path),
            str(pathlib.Path(config_path).resolve()),
            "-i",
            str(gcode_path),
            "-o",
            str(output_path),
            "-d",
            str(dictionary),
            "-l",
            str(log_path),
        ]
        res = subprocess.run(cmd, cwd=REPO_ROOT, capture_output=True, text=True)
        if res.returncode or not capture_path.exists():
            log = log_path.read_text() if log_path.exists() else ""
            raise KlippyRunError(
                "klippy batch run failed (exit %d)\n--- stderr ---\n%s\n"
                "--- klippy.log tail ---\n%s"
                % (
                    res.returncode,
                    res.stderr[-LOG_TAIL_CHARS:],
                    log[-LOG_TAIL_CHARS:],
                )
            )
        capture = json.loads(capture_path.read_text())
        steppers = capture["steppers"]
        foreign = sorted(n for n, s in steppers.items() if s["mcu"] != MAIN_MCU)
        if foreign:
            raise KlippyRunError(
                "steppers on secondary MCUs are not decoded: %s" % foreign
            )
        invert_by_oid = {s["oid"]: s["invert_dir"] for s in steppers.values()}
        by_oid = stepdecode.decode(dictionary, output_path, invert_by_oid)
    traces = {name: by_oid[s["oid"]] for name, s in steppers.items()}
    return RawRun(capture=capture, traces=traces)
