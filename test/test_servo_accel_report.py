import importlib.util
import json
import os
import struct

import numpy as np
import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCRIPTS = os.path.join(os.path.dirname(_HERE), "scripts")


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


sar = _load(
    "servo_accel_report_script", os.path.join(_SCRIPTS, "servo_accel_report.py")
)
sc = _load(
    "servo_capture_script_accel", os.path.join(_SCRIPTS, "servo_capture.py")
)

FLAG_TORQUE_ENABLED = 1
FLAG_MOTION_ACTIVE = 2

CHANNELS = [
    {"name": "cycle_index", "dtype": "u64", "offset": 0},
    {"name": "flags", "dtype": "u8", "offset": 8},
    {"name": "target_counts", "dtype": "i32", "offset": 9},
    {"name": "position_demand", "dtype": "i32", "offset": 13},
    {"name": "position_actual", "dtype": "i32", "offset": 17},
    {"name": "following_error", "dtype": "i32", "offset": 21},
    {"name": "torque_actual", "dtype": "i16", "offset": 25},
    {"name": "statusword", "dtype": "u16", "offset": 27},
    {"name": "error_code", "dtype": "u16", "offset": 29},
]


def _touch(directory, name):
    path = os.path.join(directory, name)
    with open(path, "w"):
        pass
    return path


def synth_torque_capture(tmp_path, torque, moving, cycle_ns=1_000_000):
    n = len(torque)
    header = {
        "version": 1,
        "cycle_ns": cycle_ns,
        "record_size": 31,
        "started_utc": "2026-07-05T00:00:00Z",
        "started_mono_ns": 0,
        "drives": [{"name": "x", "counts_per_mm": 1000.0}],
        "channels": CHANNELS,
    }
    path = os.path.join(str(tmp_path), "tq.scap")
    with open(path, "wb") as f:
        f.write((json.dumps(header) + "\n").encode())
        for i in range(n):
            flag = FLAG_TORQUE_ENABLED | (
                FLAG_MOTION_ACTIVE if moving[i] else 0
            )
            f.write(
                struct.pack(
                    "<QBiiiihHH", i, flag, 0, 0, 0, 0, int(torque[i]), 0x0627, 0
                )
            )
    return path


def _summary(tmp_path, torque, moving, cycle_ns=1_000_000):
    path = synth_torque_capture(tmp_path, torque, moving, cycle_ns)
    _, data, _ = sc.load_capture(path)
    fs = 1e9 / cycle_ns
    return sc.torque_summary(data, torque_limit=900, fs=fs)


def test_clean_trace_reports_no_rail(tmp_path):
    n = 500
    moving = np.ones(n, dtype=bool)
    torque = np.full(n, 400, dtype=np.int16)
    s = _summary(tmp_path, torque, moving)
    assert not s["rail_detected"]
    assert s["peak"] == 400
    assert s["rail_samples"] == 0


def test_clipped_trace_detects_rail(tmp_path):
    n = 1000
    moving = np.ones(n, dtype=bool)
    torque = np.full(n, 500, dtype=np.int16)
    torque[100:250] = 1493  # 150 samples clipped at the rail
    s = _summary(tmp_path, torque, moving)
    assert s["rail_detected"]
    assert s["peak"] == 1493
    assert s["peak_pct_rated"] == pytest.approx(149.3)
    assert s["rail_level"] == round(1493 * 0.97)
    assert s["rail_samples"] == 150
    assert s["rail_pct_moving"] == pytest.approx(15.0)
    # 1 kHz -> 150 samples = 150 ms, one contiguous burst
    assert s["rail_ms"] == pytest.approx(150.0)
    assert s["longest_burst_ms"] == pytest.approx(150.0)


def test_rail_counts_only_moving_samples(tmp_path):
    n = 400
    moving = np.zeros(n, dtype=bool)
    moving[:200] = True
    torque = np.full(n, 500, dtype=np.int16)
    torque[300:350] = 1500  # peak lives in a non-moving region
    s = _summary(tmp_path, torque, moving)
    assert s["peak"] == 1500
    assert s["rail_samples"] == 0
    assert s["rail_pct_moving"] == 0.0


def test_longest_burst_is_max_run(tmp_path):
    n = 600
    moving = np.ones(n, dtype=bool)
    torque = np.full(n, 500, dtype=np.int16)
    torque[10:30] = 1500  # 20-sample burst
    torque[100:140] = 1500  # 40-sample burst
    s = _summary(tmp_path, torque, moving)
    assert s["rail_samples"] == 60
    assert s["longest_burst_ms"] == pytest.approx(40.0)


def test_rail_ms_scales_with_sample_rate(tmp_path):
    n = 400
    moving = np.ones(n, dtype=bool)
    torque = np.full(n, 500, dtype=np.int16)
    torque[0:40] = 1500
    s = _summary(tmp_path, torque, moving, cycle_ns=250_000)  # 4 kHz
    assert s["rail_samples"] == 40
    assert s["rail_ms"] == pytest.approx(10.0)  # 40 / 4000 s


def test_accel_from_name():
    assert sar.accel_from_name("accel_a20000_20260705_120000.scap") == 20000
    assert sar.accel_from_name("cal_p1_s2_i3_20260705_120000.scap") is None


def test_named_steps_pick_newest_per_step(tmp_path):
    d = str(tmp_path)
    _touch(d, "accel_a10000_20260705_210000.scap")
    newest = _touch(d, "accel_a10000_20260705_220000.scap")
    files = sar.find_named_steps(d, ["accel_a10000"])
    assert files == [(10000, newest)]


def test_named_steps_sorted_by_accel(tmp_path):
    d = str(tmp_path)
    hi = _touch(d, "accel_a20000_20260705_220010.scap")
    lo = _touch(d, "accel_a10000_20260705_220000.scap")
    files = sar.find_named_steps(d, ["accel_a20000", "accel_a10000"])
    assert files == [(10000, lo), (20000, hi)]


def test_named_step_without_capture_fails_loudly(tmp_path):
    with pytest.raises(SystemExit, match="accel_a10000"):
        sar.find_named_steps(str(tmp_path), ["accel_a10000"])


def test_no_captures_found_fails_loudly(tmp_path):
    with pytest.raises(SystemExit, match="no sweep captures found"):
        sar.main(["--captures-dir", str(tmp_path), "--tag", "nope"])


def test_recommend_highest_clean_accel():
    steps = [
        (5000, {"rail_detected": False}),
        (10000, {"rail_detected": False}),
        (15000, {"rail_detected": True}),
        (20000, {"rail_detected": True}),
    ]
    accel, note = sar.recommend(steps)
    assert accel == 10000
    assert "15000" in note


def test_recommend_all_railed():
    steps = [(5000, {"rail_detected": True}), (10000, {"rail_detected": True})]
    accel, note = sar.recommend(steps)
    assert accel is None
    assert "lower the accel" in note


def test_explicit_file_without_accel_field_fails_loudly(tmp_path):
    scap = _touch(str(tmp_path), "track_20260705_220000.scap")
    with pytest.raises(SystemExit, match="accel field"):
        sar.main([scap])
