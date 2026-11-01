from __future__ import annotations

import json
import struct
from datetime import datetime, timedelta, timezone
from types import SimpleNamespace

import capture_events
import mcu_execution
import numpy as np
import pytest
import step_tap

FREQ = 84e6
STEPS = 500
INTERVAL = 84_000
LATE_STEP = 200
LATE_TICKS = 252
COINCIDENT_OFFSET_TICKS = 100
COINCIDENT_LATE_NS = COINCIDENT_OFFSET_TICKS / FREQ * 1e9
MOTORS = [SimpleNamespace(role=r, channel=i) for i, r in enumerate("abze")]
EMPTY = np.empty(0, np.int64)


def _record(kind: int, oid: int, payload: bytes) -> bytes:
    return step_tap.HEADER.pack(kind, 0, 0, oid, 0) + payload.ljust(16, b"\0")


def _lane(
    oid: int, axis: int, steps: int, interval: int, start: int = 0
) -> bytes:
    return (
        _record(step_tap.KIND_LANE, oid, struct.pack("<IId", axis, 0, 0.0125))
        + _record(step_tap.KIND_RESET_STEP_CLOCK, oid, struct.pack("<I", start))
        + _record(step_tap.KIND_SET_NEXT_STEP_DIR, oid, bytes([1]))
        + _record(
            step_tap.KIND_QUEUE_STEP,
            oid,
            struct.pack("<IHh", interval, steps, 0),
        )
    )


def _write_tap(run_dir, *lanes: bytes) -> None:
    header = _record(step_tap.KIND_MCU, 0, struct.pack("<IId", 0, 0, FREQ))
    (run_dir / "step_tap_mcu0_p1_s0.bin").write_bytes(
        step_tap.MAGIC + header + b"".join(lanes)
    )


def _commanded(steps: int, interval: int, start: int = 0) -> np.ndarray:
    return start + interval * np.arange(1, steps + 1, dtype=np.int64)


def _channel(ticks, dir_ticks=EMPTY, dir_levels=(), initial_dir=1):
    return capture_events.Channel(
        np.asarray(ticks, np.int64),
        np.asarray(dir_ticks, np.int64),
        np.asarray(dir_levels, np.int8),
        initial_dir,
    )


def _events(*channels: capture_events.Channel) -> capture_events.EventCapture:
    padded = list(channels) + [_channel(EMPTY)] * (4 - len(channels))
    return capture_events.EventCapture(padded, 0, 0, 0, 0, [0] * 4)


def _single_lane_run(run_dir) -> np.ndarray:
    _write_tap(run_dir, _lane(5, 0, STEPS, INTERVAL))
    return _commanded(STEPS, INTERVAL)


def test_a_late_step_stands_out_above_the_fitted_clock_map(tmp_path):
    commanded = _single_lane_run(tmp_path)
    captured = np.round(commanded * (1 + 50e-6) + 1_234).astype(np.int64)
    captured[LATE_STEP] += LATE_TICKS
    report = mcu_execution.report(tmp_path, _events(_channel(captured)), MOTORS)
    assert report["motors"]["a"]["steps"] == STEPS
    assert report["steps_late_over_1000ns"] == 1
    assert report["lateness_ns"]["max"] == pytest.approx(3_000, abs=30)
    assert report["lateness_ns"]["p50"] < 30


def test_a_commanded_and_captured_count_mismatch_is_fatal(tmp_path):
    commanded = _single_lane_run(tmp_path)
    with pytest.raises(mcu_execution.ExecutionError, match="commanded steps"):
        mcu_execution.report(
            tmp_path, _events(_channel(commanded[:-1])), MOTORS
        )


def test_a_captured_dir_flip_the_tap_never_commanded_is_fatal(tmp_path):
    commanded = _single_lane_run(tmp_path)
    flip = commanded[STEPS // 2] - INTERVAL // 2
    channel = _channel(commanded, [flip], [0])
    with pytest.raises(mcu_execution.ExecutionError, match="DIR disagrees"):
        mcu_execution.report(tmp_path, _events(channel), MOTORS)


def test_a_constant_dir_inversion_is_accepted(tmp_path):
    commanded = _single_lane_run(tmp_path)
    channel = _channel(commanded, initial_dir=0)
    report = mcu_execution.report(tmp_path, _events(channel), MOTORS)
    assert report["motors"]["a"]["steps"] == STEPS


def test_two_sessions_in_one_run_are_fatal(tmp_path):
    commanded = _single_lane_run(tmp_path)
    (tmp_path / "step_tap_mcu0_p1_s1.bin").write_bytes(
        (tmp_path / "step_tap_mcu0_p1_s0.bin").read_bytes()
    )
    with pytest.raises(mcu_execution.ExecutionError, match="session"):
        mcu_execution.report(tmp_path, _events(_channel(commanded)), MOTORS)


def test_no_tap_means_no_report(tmp_path):
    assert mcu_execution.report(tmp_path, None, MOTORS) is None


def _mcu_event(when: datetime, behind_ticks: int, min_next: int) -> dict:
    return {
        "_time": when.astimezone(timezone.utc)
        .isoformat()
        .replace("+00:00", "Z"),
        "event": mcu_execution.STEP_LOAD_LATE_EVENT,
        "arg0": (-behind_ticks) & 0xFFFFFFFF,
        "arg1": min_next,
    }


def test_step_load_late_events_match_the_step_at_min_next_plus_behind(tmp_path):
    commanded = _single_lane_run(tmp_path)
    captured = commanded.copy()
    captured[LATE_STEP] += LATE_TICKS
    start = datetime.now()
    (tmp_path / "run.json").write_text(
        json.dumps(
            {
                "host_start": start.isoformat(),
                "host_end": (start + timedelta(seconds=10)).isoformat(),
            }
        )
    )
    behind = 500
    late_clock = int(commanded[LATE_STEP])
    events = [
        _mcu_event(start + timedelta(seconds=1), behind, late_clock + behind),
        _mcu_event(start - timedelta(seconds=5), behind, late_clock + behind),
    ]
    (tmp_path / "events").mkdir()
    (tmp_path / "events" / "mcu.jsonl").write_text(
        "\n".join(json.dumps(e) for e in events) + "\n"
    )
    report = mcu_execution.report(tmp_path, _events(_channel(captured)), MOTORS)
    assert report["step_load_late_events"] == 1
    assert report["step_load_late_matched"] == 1
    example = report["step_load_late_examples"][0]
    assert example["behind_ns"] == pytest.approx(behind / FREQ * 1e9)
    [match] = example["matched"]
    assert match["motor"] == "a"
    assert match["measured_lateness_ns"] == pytest.approx(3_000, abs=30)


def test_piecewise_fit_tracks_slow_drift_but_not_single_outliers():
    x = np.linspace(0, 20 * FREQ, 20_000)
    y = x / FREQ + 1e-6 * np.sin(x / FREQ / 3)
    y[5_000] += 5e-6
    evaluate, inliers = mcu_execution.piecewise_fit(x, y, 1.0 * FREQ)
    residual = (y - evaluate(x)) * 1e9
    assert not inliers[5_000]
    assert residual[5_000] > 4_900
    assert np.abs(np.delete(residual, 5_000)).max() < 20


def test_piecewise_fit_bridges_stretches_with_no_eligible_steps():
    x = np.linspace(0, 20 * FREQ, 20_000)
    y = x / FREQ * (1 + 30e-6) + 0.5
    eligible = (x < 5 * FREQ) | (x > 15 * FREQ)
    evaluate, _ = mcu_execution.piecewise_fit(x, y, 1.0 * FREQ, eligible)
    assert np.abs((y - evaluate(x)) * 1e9).max() < 1


def test_a_step_exactly_one_window_away_is_isolated():
    window = 252
    a = np.array([1_000, 10_000], np.int64)
    b = np.array([1_000 + window, 10_000 + window - 1], np.int64)
    alone_a, alone_b = mcu_execution.isolated_steps([a, b], window)
    assert alone_a.tolist() == [True, False]
    assert alone_b.tolist() == [True, False]


def test_a_majority_of_coincident_late_steps_does_not_drag_the_clock_map(
    tmp_path,
):
    late_lanes = 3
    shared = STEPS // 2
    _write_tap(
        tmp_path,
        _lane(1, 0, STEPS, INTERVAL),
        *[
            _lane(2 + axis, axis, shared, 2 * INTERVAL, COINCIDENT_OFFSET_TICKS)
            for axis in range(1, 1 + late_lanes)
        ],
    )
    a = _commanded(STEPS, INTERVAL)
    late = _commanded(shared, 2 * INTERVAL, 2 * COINCIDENT_OFFSET_TICKS)
    assert late_lanes * shared > STEPS, (
        "the coincident late steps are the majority"
    )
    report = mcu_execution.report(
        tmp_path,
        _events(_channel(a), *[_channel(late)] * late_lanes),
        MOTORS,
    )
    motors = report["motors"]
    assert motors["a"]["coincident_steps"] == shared
    assert motors["a"]["isolated_lateness_ns"]["p50"] == pytest.approx(
        0, abs=30
    )
    assert motors["a"]["coincident_lateness_ns"]["p50"] == pytest.approx(
        0, abs=30
    )
    for role in "bze":
        assert motors[role]["coincident_steps"] == shared
        assert motors[role]["coincident_lateness_ns"]["p50"] == pytest.approx(
            COINCIDENT_LATE_NS, abs=30
        )
