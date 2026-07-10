import importlib.util
import json
import os
import struct

import numpy as np
import pytest

_SCRIPT = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "scripts",
    "servo_gain_report.py",
)
_spec = importlib.util.spec_from_file_location(
    "servo_gain_report_script", _SCRIPT
)
sgr = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(sgr)


def _touch(directory, name):
    path = os.path.join(directory, name)
    with open(path, "w"):
        pass
    return path


def test_named_steps_pick_newest_capture_per_step(tmp_path):
    d = str(tmp_path)
    _touch(d, "cal_p2000_s1250_i1000_20260611_210000.scap")
    newest = _touch(d, "cal_p2000_s1250_i1000_20260611_220000.scap")
    files = sgr.find_named_steps(d, ["cal_p2000_s1250_i1000"])
    assert files == [((2000, 1250, 1000), newest)]


def test_named_steps_exclude_stale_steps_from_other_runs(tmp_path):
    d = str(tmp_path)
    _touch(d, "cal_p2880_s1800_i694_20260610_180000.scap")
    kept = _touch(d, "cal_p2400_s1500_i833_20260611_220000.scap")
    files = sgr.find_named_steps(d, ["cal_p2400_s1500_i833"])
    assert files == [((2400, 1500, 833), kept)]


def test_named_steps_sort_by_speed_gain(tmp_path):
    d = str(tmp_path)
    slow = _touch(d, "cal_p1600_s1000_i1250_20260611_220000.scap")
    fast = _touch(d, "cal_p2400_s1500_i833_20260611_220010.scap")
    files = sgr.find_named_steps(
        d, ["cal_p2400_s1500_i833", "cal_p1600_s1000_i1250"]
    )
    assert files == [((1600, 1000, 1250), slow), ((2400, 1500, 833), fast)]


def test_named_step_without_capture_fails_loudly(tmp_path):
    with pytest.raises(SystemExit, match="cal_p2000_s1250_i1000"):
        sgr.find_named_steps(str(tmp_path), ["cal_p2000_s1250_i1000"])


def test_explicit_files_and_steps_are_mutually_exclusive(tmp_path):
    scap = _touch(str(tmp_path), "cal_p2000_s1250_i1000_20260611_220000.scap")
    with pytest.raises(SystemExit, match="not both"):
        sgr.main([scap, "--steps", "cal_p2000_s1250_i1000"])


def _write_cruise_capture(tmp_path, filename, invert):
    """1 kHz capture with one long constant-speed move; counts are written in
    the drive frame (negated when invert is set), the way a real inverted
    servo reports them."""
    n = 6000
    fs = 1000.0
    t = np.arange(n) / fs
    moving = np.zeros(n, dtype=bool)
    moving[500:5000] = True
    ferr = np.where(moving, 200.0 * np.sin(2 * np.pi * 80.0 * t), 0.0)
    target = np.cumsum(np.where(moving, 100, 0)).astype(np.int64)
    sign = -1 if invert else 1
    header = {
        "version": 2,
        "cycle_ns": 1_000_000,
        "record_size": 21,
        "started_utc": "2026-06-10T12:00:00Z",
        "started_mono_ns": 0,
        "drives": [
            {
                "name": "x",
                "counts_per_mm": 3276.8,
                "rotation_distance": 40.0,
                "invert": invert,
            }
        ],
        "channels": [
            {"name": "cycle_index", "dtype": "u64", "offset": 0},
            {"name": "flags", "dtype": "u8", "offset": 8},
            {"name": "target_counts", "dtype": "i32", "offset": 9},
            {"name": "position_actual", "dtype": "i32", "offset": 13},
            {"name": "following_error", "dtype": "i32", "offset": 17},
        ],
    }
    path = os.path.join(str(tmp_path), filename)
    with open(path, "wb") as f:
        f.write((json.dumps(header) + "\n").encode())
        for i in range(n):
            fe = int(round(ferr[i]))
            tgt = int(target[i])
            f.write(
                struct.pack(
                    "<QBiii",
                    i,
                    3 if moving[i] else 1,
                    sign * tgt,
                    sign * (tgt - fe),
                    sign * fe,
                )
            )
    return path


def test_inverted_drive_metrics_flip_into_kinematic_frame(tmp_path):
    normal = sgr.drive_metrics(
        _write_cruise_capture(tmp_path, "normal.scap", invert=False), "x"
    )
    inverted = sgr.drive_metrics(
        _write_cruise_capture(tmp_path, "inverted.scap", invert=True), "x"
    )
    np.testing.assert_allclose(inverted["cruise_ferr"], normal["cruise_ferr"])
    assert inverted["cruise_mm_s"] == pytest.approx(normal["cruise_mm_s"])
    assert inverted["overshoot_max_um"] == pytest.approx(
        normal["overshoot_max_um"]
    )
