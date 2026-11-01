from __future__ import annotations

import struct
from types import SimpleNamespace

import capture_events
import mcu_execution
import numpy as np
import pytest
import step_tap

FREQ = 84e6
OID = 5
STEPS = 500
INTERVAL = 84_000
LATE_STEP = 200
LATE_TICKS = 252
MOTORS = [SimpleNamespace(role=r, channel=i) for i, r in enumerate("abze")]


def _record(kind: int, now: int, payload: bytes, oid: int = OID) -> bytes:
    return step_tap.HEADER.pack(kind, 0, 0, oid, now) + payload.ljust(16, b"\0")


def _write_tap(run_dir, steps: int) -> np.ndarray:
    data = (
        step_tap.MAGIC
        + _record(step_tap.KIND_MCU, 0, struct.pack("<IId", 0, 0, FREQ), 0)
        + _record(step_tap.KIND_LANE, 0, struct.pack("<IId", 0, 0, 0.0125))
        + _record(step_tap.KIND_RESET_STEP_CLOCK, 0, struct.pack("<I", 0))
        + _record(step_tap.KIND_SET_NEXT_STEP_DIR, 0, bytes([1]))
        + _record(
            step_tap.KIND_QUEUE_STEP, 0, struct.pack("<IHh", INTERVAL, steps, 0)
        )
    )
    (run_dir / "step_tap_mcu0.bin").write_bytes(data)
    return INTERVAL * np.arange(1, steps + 1, dtype=np.int64)


def _events(a_ticks: np.ndarray) -> capture_events.EventCapture:
    empty = np.empty(0, np.int64)
    channels = [
        capture_events.Channel(a_ticks, empty, np.empty(0, np.int8), 1),
        *[
            capture_events.Channel(empty, empty, np.empty(0, np.int8), 1)
            for _ in range(3)
        ],
    ]
    return capture_events.EventCapture(channels, 0, 0, 0, 0, [0] * 4)


def test_a_late_step_stands_out_above_the_fitted_clock_map(tmp_path):
    commanded = _write_tap(tmp_path, STEPS)
    captured = np.round(commanded * (1 + 50e-6) + 1_234).astype(np.int64)
    captured[LATE_STEP] += LATE_TICKS
    report = mcu_execution.report(tmp_path, _events(captured), MOTORS)
    assert report["motors"]["a"]["steps"] == STEPS
    assert report["steps_late_over_1000ns"] == 1
    assert report["lateness_ns"]["max"] == pytest.approx(3_000, abs=30)
    assert report["lateness_ns"]["p50"] < 30


def test_a_commanded_and_captured_count_mismatch_is_fatal(tmp_path):
    commanded = _write_tap(tmp_path, STEPS)
    with pytest.raises(mcu_execution.ExecutionError, match="commanded steps"):
        mcu_execution.report(tmp_path, _events(commanded[:-1]), MOTORS)


def test_no_tap_means_no_report(tmp_path):
    assert mcu_execution.report(tmp_path, None, MOTORS) is None


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


def test_coincident_steps_do_not_drag_the_clock_map(tmp_path):
    def lane(oid: int, axis: int, steps: int, interval: int) -> bytes:
        return (
            _record(
                step_tap.KIND_LANE, 0, struct.pack("<IId", axis, 0, 0.0125), oid
            )
            + _record(
                step_tap.KIND_RESET_STEP_CLOCK, 0, struct.pack("<I", 0), oid
            )
            + _record(step_tap.KIND_SET_NEXT_STEP_DIR, 0, bytes([1]), oid)
            + _record(
                step_tap.KIND_QUEUE_STEP,
                0,
                struct.pack("<IHh", interval, steps, 0),
                oid,
            )
        )

    (tmp_path / "step_tap_mcu0.bin").write_bytes(
        step_tap.MAGIC
        + _record(step_tap.KIND_MCU, 0, struct.pack("<IId", 0, 0, FREQ), 0)
        + lane(1, 0, STEPS, INTERVAL)
        + lane(2, 1, STEPS // 2, 2 * INTERVAL)
    )
    a = INTERVAL * np.arange(1, STEPS + 1, dtype=np.int64)
    b = 2 * INTERVAL * np.arange(1, STEPS // 2 + 1, dtype=np.int64) + 100
    empty = np.empty(0, np.int64)
    channels = [
        capture_events.Channel(ticks, empty, np.empty(0, np.int8), 1)
        for ticks in (a, b, empty, empty)
    ]
    events = capture_events.EventCapture(channels, 0, 0, 0, 0, [0] * 4)
    report = mcu_execution.report(tmp_path, events, MOTORS)
    assert report["motors"]["a"]["coincident_steps"] == STEPS // 2
    assert report["motors"]["a"]["isolated_lateness_ns"]["max"] < 30
    assert report["motors"]["b"]["coincident_lateness_ns"][
        "p50"
    ] == pytest.approx(1_190, abs=30)
