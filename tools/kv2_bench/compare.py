"""Executed vs planned: captured steps against the batch reference and the
live queue_step tap.

Three step streams per motor, all with exact counts and DIR signs:
- reference: klippy batch mode on this box (reference.py), F401 clock ticks;
- tap: the queue_step schedule the live host sent (pi_agent.py), F401 ticks;
- capture: every STEP rising edge on the F407 (event mode), F407 ticks.

Steps pair by index per motor, which is only valid when the sign sequences
are identical, so that is checked first and a mismatch fails. Timelines:
the tap and the capture share one continuous timeline (one linear map, the
two crystals' ratio); the reference restarts its print_time whenever the
toolhead went idle, as the live host does at other instants, so reference
timelines are split into epochs at those restarts. An offset jump between
paired streams is accepted as an epoch boundary only where the reference
rests (no step on any motor for REST_S); a jump mid-motion fails.

Timing bound for capture vs reference: every queue_step step lies in
[ideal - MAX_STEPCOMPRESS_ERROR, ideal] (stepcompress.c minmax_point), for
the live host and for the batch run alike, so their difference is within
+-MAX_STEPCOMPRESS_ERROR; the MCU adds its execution latency above its
floor, measured on the same run as capture vs tap.
"""

from __future__ import annotations

import dataclasses
import json
from pathlib import Path

import numpy as np

from .reference import Reference, Steps
from .serval_tools import capture_events

CHANNELS = ("stepper_x", "stepper_y", "stepper_z", "extruder")
MAX_STEPCOMPRESS_ERROR_S = 25e-6
CAPTURE_TICK_HZ = 84_000_000
INSTRUMENT_SLACK_S = 0.1e-6
REST_S = 0.020
JUMP_S = 100e-6


class CompareError(RuntimeError):
    pass


def load_capture(run_dir: Path) -> dict[str, Steps]:
    events = capture_events.load(run_dir)
    if events.truncated_tail_bytes:
        raise CompareError(
            f"event capture has {events.truncated_tail_bytes} truncated bytes"
        )
    return {
        name: Steps(channel.step_ticks.astype(np.int64), channel.step_signs())
        for name, channel in zip(CHANNELS, events.channels)
    }


def load_tap(run_dir: Path) -> dict[str, Steps]:
    clocks: dict[str, list[np.ndarray]] = {name: [] for name in CHANNELS}
    signs: dict[str, list[np.ndarray]] = {name: [] for name in CHANNELS}
    for line in (run_dir / "tap.jsonl").read_text().splitlines():
        message = json.loads(line)
        if "tap" not in message:
            continue
        name, params = message["tap"], message["params"]
        data = params["data"]
        step_clock = params["first_clock"] - data[0][0]
        batch = []
        for interval, count, add, add2, shift in data:
            if add2 or shift:
                raise CompareError(f"{name}: high-precision queue_step in tap")
            n = abs(count)
            if not n:
                continue
            k = np.arange(n, dtype=np.int64)
            entry = step_clock + (k + 1) * interval + add * k * (k + 1) // 2
            step_clock = int(entry[-1])
            batch.append(entry)
            signs[name].append(np.full(n, 1 if count > 0 else -1, np.int8))
        if not batch:
            continue
        if clocks[name] and batch[0][0] <= clocks[name][-1][-1]:
            raise CompareError(
                f"{name}: tap batch starting at clock {batch[0][0]} overlaps "
                f"the previous one ending at {clocks[name][-1][-1]}"
            )
        clocks[name].extend(batch)
    return {
        name: Steps(
            np.concatenate(clocks[name])
            if clocks[name]
            else np.zeros(0, np.int64),
            np.concatenate(signs[name])
            if signs[name]
            else np.zeros(0, np.int8),
        )
        for name in CHANNELS
    }


def first_divergence(a: np.ndarray, b: np.ndarray) -> int | None:
    n = min(len(a), len(b))
    differ = np.flatnonzero(a[:n] != b[:n])
    if len(differ):
        return int(differ[0])
    return None if len(a) == len(b) else n


def sequences(planned: dict[str, Steps], executed: dict[str, Steps]) -> dict:
    per_motor = {}
    for name in CHANNELS:
        p, e = planned[name], executed[name]
        divergence = first_divergence(p.signs, e.signs)
        per_motor[name] = {
            "planned_steps": len(p.signs),
            "executed_steps": len(e.signs),
            "planned_net": p.net,
            "executed_net": e.net,
            "net_match": p.net == e.net,
            "identical_sequence": divergence is None,
            "first_divergence_index": divergence,
        }
    return {
        "per_motor": per_motor,
        "net_steps_match": all(m["net_match"] for m in per_motor.values()),
        "all_steps_paired": all(
            m["identical_sequence"] for m in per_motor.values()
        ),
    }


@dataclasses.dataclass
class Paired:
    x: np.ndarray
    y: np.ndarray
    motor: np.ndarray
    index: np.ndarray


def pair(planned: dict[str, Steps], executed: dict[str, Steps]) -> Paired:
    xs, ys, motors, indices = [], [], [], []
    for number, name in enumerate(CHANNELS):
        n = len(planned[name].clocks)
        xs.append(planned[name].clocks)
        ys.append(executed[name].clocks)
        motors.append(np.full(n, number, np.int8))
        indices.append(np.arange(n, dtype=np.int64))
    x = np.concatenate(xs)
    order = np.argsort(x, kind="stable")
    return Paired(
        x[order],
        np.concatenate(ys)[order],
        np.concatenate(motors)[order],
        np.concatenate(indices)[order],
    )


def epochs(p: Paired, freq: float) -> tuple[np.ndarray, list[dict]]:
    d = p.y - p.x
    jumps = np.flatnonzero(np.abs(np.diff(d)) > JUMP_S * freq) + 1
    gaps = p.x[jumps] - p.x[jumps - 1]
    mid_motion = [
        {
            "at_reference_s": float(p.x[i] / freq),
            "jump_us": float((d[i] - d[i - 1]) / freq * 1e6),
            "reference_gap_ms": float(g / freq * 1e3),
        }
        for i, g in zip(jumps, gaps)
        if g < REST_S * freq
    ]
    ids = np.zeros(len(d), np.int64)
    ids[jumps] = 1
    return np.cumsum(ids), mid_motion


def fit(
    p: Paired, ids: np.ndarray, slope: float | None
) -> tuple[float, np.ndarray]:
    """y = slope * x + offset[epoch] by least squares, slope shared."""
    x = p.x.astype(np.float64)
    y = p.y.astype(np.float64)
    count = np.bincount(ids)
    x_mean = np.bincount(ids, x) / count
    y_mean = np.bincount(ids, y) / count
    xc = x - x_mean[ids]
    yc = y - y_mean[ids]
    if slope is None:
        slope = float(np.dot(xc, yc) / np.dot(xc, xc))
    return slope, yc - slope * xc


def residual_stats(residual_s: np.ndarray, motor: np.ndarray) -> dict:
    out = {}
    for number, name in enumerate(CHANNELS):
        r = np.abs(residual_s[motor == number]) * 1e6
        if not len(r):
            continue
        out[name] = {
            "steps": int(len(r)),
            "abs_max_us": float(r.max()),
            "p99_us": float(np.percentile(r, 99)),
            "p50_us": float(np.percentile(r, 50)),
        }
    return out


def timeline(
    planned: dict[str, Steps],
    executed: dict[str, Steps],
    freq: float,
    slope: float | None,
    single_epoch: bool,
) -> tuple[dict, Paired, np.ndarray]:
    p = pair(planned, executed)
    ids, mid_motion = epochs(p, freq)
    if single_epoch:
        ids = np.zeros_like(ids)
    fitted, residual = fit(p, ids, slope)
    residual_s = residual / freq
    worst = int(np.argmax(np.abs(residual_s)))
    return (
        {
            "epochs": int(ids.max() + 1) if len(ids) else 0,
            "jumps_mid_motion": mid_motion,
            "slope_ppm": (fitted - 1.0) * 1e6,
            "per_motor": residual_stats(residual_s, p.motor),
            "abs_max_us": float(np.abs(residual_s).max() * 1e6),
            "worst": {
                "motor": CHANNELS[p.motor[worst]],
                "index": int(p.index[worst]),
                "reference_s": float(p.x[worst] / freq),
                "residual_us": float(residual_s[worst] * 1e6),
            },
        },
        p,
        residual_s,
    )


def mcu_lateness(residual_s: np.ndarray) -> dict:
    lateness = (residual_s - residual_s.min()) * 1e6
    return {
        "max_us": float(lateness.max()),
        "p999_us": float(np.percentile(lateness, 99.9)),
        "p99_us": float(np.percentile(lateness, 99)),
        "p50_us": float(np.percentile(lateness, 50)),
    }


def schedule_identity(
    planned: dict[str, Steps], executed: dict[str, Steps], freq: float
) -> dict:
    """Two queue_step schedules in one clock domain: per epoch, the offset
    most steps share, the share of steps exactly at it and the largest
    departure from it (compression choosing other steps)."""
    p = pair(planned, executed)
    ids, mid_motion = epochs(p, freq)
    d = p.y - p.x
    departure = np.zeros(len(d), np.int64)
    for epoch in range(int(ids.max()) + 1 if len(ids) else 0):
        mask = ids == epoch
        values, counts = np.unique(d[mask], return_counts=True)
        departure[mask] = d[mask] - values[np.argmax(counts)]
    worst = int(np.argmax(np.abs(departure))) if len(d) else 0
    return {
        "epochs": int(ids.max() + 1) if len(ids) else 0,
        "jumps_mid_motion": mid_motion,
        "steps_identical": float(np.mean(departure == 0)) if len(d) else 1.0,
        "abs_max_us": float(np.abs(departure).max() / freq * 1e6)
        if len(d)
        else 0.0,
        "worst_reference_s": float(p.x[worst] / freq) if len(d) else None,
    }


def analyze(run_dir: Path, reference_dir: Path) -> dict:
    reference = Reference.load(reference_dir)
    freq = reference.clock_freq
    if freq != CAPTURE_TICK_HZ:
        raise CompareError(
            f"reference clock {freq} Hz, capture ticks at 84 MHz"
        )
    captured = load_capture(run_dir)
    result: dict = {
        "reference": str(reference_dir),
        "reference_meta": reference.meta,
    }
    seq = sequences(reference.steppers, captured)
    result["capture_vs_reference"] = {"sequences": seq}
    lateness = None
    if (run_dir / "tap.jsonl").exists():
        tap = load_tap(run_dir)
        tap_seq = sequences(tap, captured)
        tap_ref_seq = sequences(reference.steppers, tap)
        result["capture_vs_tap"] = {"sequences": tap_seq}
        result["tap_vs_reference"] = {"sequences": tap_ref_seq}
        if tap_seq["all_steps_paired"]:
            stats, _, residual = timeline(tap, captured, freq, None, True)
            lateness = mcu_lateness(residual)
            result["capture_vs_tap"]["timing"] = stats
            result["capture_vs_tap"]["mcu_lateness"] = lateness
        if tap_ref_seq["all_steps_paired"]:
            result["tap_vs_reference"]["identity"] = schedule_identity(
                reference.steppers, tap, freq
            )
    if seq["all_steps_paired"]:
        stats, _, _ = timeline(reference.steppers, captured, freq, None, False)
        result["capture_vs_reference"]["timing"] = stats
        if lateness is not None:
            bound_us = (
                MAX_STEPCOMPRESS_ERROR_S + INSTRUMENT_SLACK_S
            ) * 1e6 + lateness["max_us"]
            stats["bound_us"] = bound_us
            stats["bound_source"] = (
                "MAX_STEPCOMPRESS_ERROR 25 us (klippy/stepper.py) + 0.1 us "
                "instrument + this run's measured MCU lateness max "
                "(capture vs tap)"
            )
            stats["within_bound"] = (
                stats["abs_max_us"] <= bound_us
                and not stats["jumps_mid_motion"]
            )
    checks = {
        "net_steps_match": seq["net_steps_match"],
        "all_steps_paired": seq["all_steps_paired"],
    }
    timing = result["capture_vs_reference"].get("timing", {})
    if "within_bound" in timing:
        checks["timing_within_bound"] = timing["within_bound"]
    if "capture_vs_tap" in result:
        checks["mcu_executed_every_commanded_step"] = result["capture_vs_tap"][
            "sequences"
        ]["all_steps_paired"]
    result["checks"] = checks
    result["ok"] = all(checks.values())
    (run_dir / "analysis.json").write_text(json.dumps(result, indent=2))
    return result
