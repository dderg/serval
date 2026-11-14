"""Slicer SET_VELOCITY_LIMIT parameters kv2 does not apply.

OrcaSlicer's jerk settings emit SQUARE_CORNER_VELOCITY= on every feature
change; under [corner_deviation] the parameter has no meaning (the limiter
sets corner speeds). Its accel_to_decel setting emits ACCEL_TO_DECEL=,
which upstream no longer converts to MINIMUM_CRUISE_RATIO. Both are
ignored with a warning once per print (print_stats reset), and every other
parameter of the line still applies."""

from __future__ import annotations

import json
import os
import pathlib
import re
import subprocess
import sys
import tempfile

import pytest

from tools.motion_eval import mcu_dict, run, sweep

START = (100.0, 100.0, 10.0)
LIMITERS = {"default": "", "corner_deviation": "[corner_deviation]\n"}
MOVES = """G1 X110 Y100 F12000
G1 X110 Y110
G1 X100 Y110
G1 X100 Y100
"""
OK = "ok"
WARNING_PREFIX = "// WARNING"
SCV_LINES = (
    "SET_VELOCITY_LIMIT ACCEL=5000 SQUARE_CORNER_VELOCITY=8\n",
    "SET_VELOCITY_LIMIT SQUARE_CORNER_VELOCITY=20\n",
    "SET_VELOCITY_LIMIT ACCEL=8000 SQUARE_CORNER_VELOCITY=6\n",
    "SET_VELOCITY_LIMIT SQUARE_CORNER_VELOCITY=1\n",
)
REPORT_LIMITS = "SET_VELOCITY_LIMIT ; report the limits\n"
BLOCKS = 40


def batch(section, gcode):
    """Run klippy batch (motion_eval capture) on fast.cfg plus section and
    a virtual_sdcard; return the capture and the log text."""
    with tempfile.TemporaryDirectory() as tmp:
        tmp = pathlib.Path(tmp)
        config = tmp / "printer.cfg"
        sd = tmp / "sd"
        sd.mkdir()
        config.write_text(
            "[include %s]\n\n[virtual_sdcard]\npath: %s\n\n%s"
            % (sweep.CONFIGS["fast"]["cfg"], sd, section)
        )
        gcode_path = tmp / "in.gcode"
        gcode_path.write_text(run.wrap_gcode(gcode, START))
        capture = tmp / "capture.json"
        log = tmp / "klippy.log"
        res = subprocess.run(
            [
                sys.executable,
                "-m",
                "tools.motion_eval.capture",
                str(capture),
                str(config),
                "-i",
                str(gcode_path),
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
        log_text = log.read_text() if log.exists() else ""
        assert res.returncode == 0 and capture.exists(), (
            res.stderr[-3000:] + log_text[-3000:]
        )
        return json.loads(capture.read_text()), log_text


def strip_param(gcode, name):
    """What a user would do by hand: drop NAME=value, then drop the
    SET_VELOCITY_LIMIT lines left without parameters."""
    gcode = re.sub(r" %s=[0-9.]+" % name, "", gcode)
    return re.sub(r"(?m)^SET_VELOCITY_LIMIT[ \t]*\n", "", gcode)


def warnings(capture, name):
    return [
        line
        for line in capture["console"]
        if line.startswith(WARNING_PREFIX) and name in line
    ]


def other_responses(capture):
    """Console output except the per-command acknowledgements (a stripped
    file has fewer commands) and the warnings."""
    return [
        line
        for line in capture["console"]
        if line != OK and not line.startswith(WARNING_PREFIX)
    ]


def log_warnings(log, name):
    return [line for line in log.splitlines() if name in line]


def assert_same_motion(cap, ref):
    assert len(cap["moves"]) == len(ref["moves"])
    assert cap["moves"] == ref["moves"]
    assert cap["flushes"] == ref["flushes"]


def scv_gcode():
    return "".join(
        SCV_LINES[i % len(SCV_LINES)] + MOVES + REPORT_LIMITS
        for i in range(BLOCKS)
    )


def test_corner_deviation_ignores_square_corner_velocity_once():
    gcode = scv_gcode()
    cap, log = batch(LIMITERS["corner_deviation"], gcode)
    ref, _ = batch(
        LIMITERS["corner_deviation"],
        strip_param(gcode, "SQUARE_CORNER_VELOCITY"),
    )
    assert_same_motion(cap, ref)
    assert {m["accel"] for m in cap["moves"]} == {5000.0, 8000.0}
    assert other_responses(cap) == other_responses(ref)
    (warning,) = warnings(cap, "SQUARE_CORNER_VELOCITY")
    assert "max_corner_accel" in warning
    assert len(log_warnings(log, "SQUARE_CORNER_VELOCITY")) == 1
    assert not warnings(ref, "SQUARE_CORNER_VELOCITY")


def test_corner_deviation_applies_the_other_limits_of_the_line():
    gcode = (
        "SET_VELOCITY_LIMIT VELOCITY=150 ACCEL=4000 MINIMUM_CRUISE_RATIO=0.3"
        " SQUARE_CORNER_VELOCITY=8\n" + MOVES + REPORT_LIMITS
    )
    cap, _ = batch(LIMITERS["corner_deviation"], gcode)
    ref, _ = batch(
        LIMITERS["corner_deviation"],
        strip_param(gcode, "SQUARE_CORNER_VELOCITY"),
    )
    assert_same_motion(cap, ref)
    assert max(m["cruise_v"] for m in cap["moves"]) == 150.0
    assert {m["accel"] for m in cap["moves"]} == {4000.0}
    assert other_responses(cap) == other_responses(ref)


def test_square_corner_velocity_in_printer_config_is_still_an_error():
    with pytest.raises(AssertionError, match="replaces square_corner_velocity"):
        batch(
            "[printer]\nsquare_corner_velocity: 5\n\n[corner_deviation]\n",
            MOVES,
        )


def test_square_corner_velocity_still_applies_without_corner_deviation():
    slow = "SET_VELOCITY_LIMIT SQUARE_CORNER_VELOCITY=1\n" + MOVES
    fast = "SET_VELOCITY_LIMIT SQUARE_CORNER_VELOCITY=20\n" + MOVES
    cap_slow, log = batch(LIMITERS["default"], slow)
    cap_fast, _ = batch(LIMITERS["default"], fast)
    slow_v = [m["start_v"] for m in cap_slow["moves"]]
    fast_v = [m["start_v"] for m in cap_fast["moves"]]
    assert slow_v != fast_v
    assert all(s <= f for s, f in zip(slow_v, fast_v))
    assert not warnings(cap_slow, "SQUARE_CORNER_VELOCITY")
    assert not log_warnings(log, "SQUARE_CORNER_VELOCITY")


@pytest.mark.parametrize("limiter", sorted(LIMITERS))
def test_accel_to_decel_is_ignored_with_one_warning(limiter):
    gcode = "".join(
        "SET_VELOCITY_LIMIT ACCEL=%d ACCEL_TO_DECEL=%d\n" % accels + MOVES
        for accels in ((8000, 2000), (6000, 1000)) * (BLOCKS // 2)
    )
    cap, log = batch(LIMITERS[limiter], gcode)
    ref, _ = batch(LIMITERS[limiter], strip_param(gcode, "ACCEL_TO_DECEL"))
    assert_same_motion(cap, ref)
    assert {m["accel"] for m in cap["moves"]} == {6000.0, 8000.0}
    (warning,) = warnings(cap, "ACCEL_TO_DECEL")
    assert "MINIMUM_CRUISE_RATIO" in warning
    assert len(log_warnings(log, "ACCEL_TO_DECEL")) == 1


def test_each_print_warns_again():
    line = (
        "SET_VELOCITY_LIMIT ACCEL=8000 ACCEL_TO_DECEL=6400"
        " SQUARE_CORNER_VELOCITY=6\n"
    )
    gcode = line + MOVES + line + "SDCARD_RESET_FILE\n" + line + MOVES + line
    cap, log = batch(LIMITERS["corner_deviation"], gcode)
    for name in ("SQUARE_CORNER_VELOCITY", "ACCEL_TO_DECEL"):
        assert len(warnings(cap, name)) == 2
        assert len(log_warnings(log, name)) == 2
