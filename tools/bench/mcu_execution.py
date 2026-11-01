"""MCU execution error: captured step edges against the commanded schedule.

Needs an event-mode capture and the host step tap (step_tap_mcu<N>.bin, see
step_tap.py). Commanded and captured steps pair by index per motor; a count
mismatch is fatal. One robust piecewise-linear map from the mcu clock to
capture time (knots every KNOT_SECONDS) absorbs both crystals and the
constant pipeline delay, so lateness is reported relative to its p0.1 floor.
The map is fitted on isolated steps only: when two motors' steps are
commanded within COINCIDENCE_SECONDS the mcu fires them one after another,
so the second is genuinely late and must not drag the clock map.
"""

from __future__ import annotations

import json
from datetime import datetime, timezone
from pathlib import Path

import capture_events
import numpy as np
import step_tap

KNOT_SECONDS = 2.0
COINCIDENCE_SECONDS = 3e-6
GAP_STIFFNESS = 1.0
FIT_ROUNDS = 3
INLIER_FLOOR_S = 100e-9
LATENESS_FLOOR_PERCENTILE = 0.1
LATE_THRESHOLDS_NS = (1_000, 5_000)
STEP_LOAD_LATE_EVENT = "motion.step_load_late"
REPORTED_LOAD_LATE = 20
NS = 1e9


class ExecutionError(RuntimeError):
    pass


def _hat_basis(
    x: np.ndarray, knots: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    index = np.clip(np.searchsorted(knots, x, "right") - 1, 0, knots.size - 2)
    weight = (x - knots[index]) / (knots[index + 1] - knots[index])
    return index, weight


def _solve(index, weight, y, size) -> np.ndarray:
    left = 1.0 - weight
    normal = np.zeros((size, size))
    np.add.at(normal, (index, index), left * left)
    np.add.at(normal, (index + 1, index + 1), weight * weight)
    np.add.at(normal, (index, index + 1), left * weight)
    np.add.at(normal, (index + 1, index), left * weight)
    if size >= 3:
        second_difference = np.diff(np.eye(size), n=2, axis=0)
        normal += GAP_STIFFNESS * second_difference.T @ second_difference
    rhs = np.zeros(size)
    np.add.at(rhs, index, left * y)
    np.add.at(rhs, index + 1, weight * y)
    return np.linalg.solve(normal, rhs)


def piecewise_fit(
    x: np.ndarray,
    y: np.ndarray,
    spacing: float,
    eligible: np.ndarray | None = None,
):
    knots = np.arange(x.min(), x.max() + spacing, spacing, dtype=np.float64)
    if knots.size < 2:
        knots = np.array([x.min(), x.min() + spacing])
    index, weight = _hat_basis(x, knots)
    eligible = np.ones(x.size, bool) if eligible is None else eligible
    keep = eligible.copy()
    for _ in range(FIT_ROUNDS):
        values = _solve(index[keep], weight[keep], y[keep], knots.size)
        model = values[index] * (1.0 - weight) + values[index + 1] * weight
        residual = y - model
        median = np.median(residual[keep])
        spread = 1.4826 * np.median(np.abs(residual[keep] - median))
        keep = eligible & (
            np.abs(residual - median) < max(5.0 * spread, INLIER_FLOOR_S)
        )

    def evaluate(points: np.ndarray) -> np.ndarray:
        i, w = _hat_basis(points, knots)
        return values[i] * (1.0 - w) + values[i + 1] * w

    return evaluate, keep


def isolated_steps(clocks: list[np.ndarray], window: float) -> list[np.ndarray]:
    isolated = []
    for index, own in enumerate(clocks):
        alone = np.ones(own.size, bool)
        for other_index, other in enumerate(clocks):
            if other_index == index or other.size == 0:
                continue
            at = np.clip(np.searchsorted(other, own), 1, max(other.size - 1, 1))
            nearest = np.minimum(
                np.abs(other[np.minimum(at, other.size - 1)] - own),
                np.abs(other[at - 1] - own),
            )
            alone &= nearest >= window
        isolated.append(alone)
    return isolated


def _stats_ns(values_ns: np.ndarray) -> dict:
    if values_ns.size == 0:
        return {}
    return {
        "p50": float(np.percentile(values_ns, 50)),
        "p99": float(np.percentile(values_ns, 99)),
        "p99_9": float(np.percentile(values_ns, 99.9)),
        "max": float(values_ns.max()),
    }


def _run_window(run_dir: Path) -> tuple[datetime, datetime]:
    run = json.loads((run_dir / "run.json").read_text())
    return tuple(
        datetime.fromisoformat(run[key]).astimezone(timezone.utc)
        for key in ("host_start", "host_end")
    )


def _step_load_late_events(run_dir: Path) -> list[dict]:
    log = run_dir / "events" / "mcu.jsonl"
    if not log.exists():
        return []
    start, end = _run_window(run_dir)
    found = []
    for line in log.read_text().splitlines():
        if STEP_LOAD_LATE_EVENT not in line:
            continue
        event = json.loads(line)
        when = datetime.fromisoformat(event["_time"].replace("Z", "+00:00"))
        if start <= when <= end:
            found.append(event)
    return found


def _signed32(value: int) -> int:
    return value - (1 << 32) if value >= 1 << 31 else value


def report(
    run_dir: Path, events: capture_events.EventCapture | None, motors
) -> dict | None:
    taps = sorted(run_dir.glob("step_tap_mcu*.bin"))
    if not taps:
        return None
    if events is None:
        raise ExecutionError(
            "a step tap needs an event-mode capture to compare against"
        )
    if len(taps) != 1:
        raise ExecutionError(
            f"expected one step tap, found {[t.name for t in taps]}"
        )
    tap = step_tap.load(taps[0])
    channel_of = {m.role: m.channel for m in motors}
    roles = ("a", "b", "z", "e")
    paired = []
    for lane in tap.lanes.values():
        if lane.motor_in_axis != 0 or lane.axis >= len(roles):
            raise ExecutionError(
                f"oid {lane.oid}: axis {lane.axis} motor {lane.motor_in_axis} is not a captured motor"
            )
        role = roles[lane.axis]
        channel = events.channels[channel_of[role]]
        commanded = lane.step_clocks()
        captured = channel.step_ticks
        if commanded.size != captured.size:
            raise ExecutionError(
                f"{role} (oid {lane.oid}): {commanded.size} commanded steps, "
                f"{captured.size} captured"
            )
        if commanded.size == 0:
            continue
        dirs = np.where(lane.step_dirs() != 0, 1, -1)
        signs = channel.step_signs()
        if not (np.all(dirs == signs) or np.all(dirs == -signs)):
            raise ExecutionError(
                f"{role}: captured DIR disagrees with the commanded dir"
            )
        paired.append(
            (role, commanded, captured / capture_events.EVENT_TICK_HZ, channel)
        )
    if not paired:
        raise ExecutionError("the step tap commanded no steps")
    isolated = isolated_steps(
        [c for _, c, _, _ in paired], COINCIDENCE_SECONDS * tap.clock_freq
    )
    if not any(alone.any() for alone in isolated):
        raise ExecutionError(
            "every step has another motor's step within "
            f"{COINCIDENCE_SECONDS * NS:.0f} ns; no isolated step to fit the clock map"
        )
    x = np.concatenate([c for _, c, _, _ in paired]).astype(np.float64)
    y = np.concatenate([t for _, _, t, _ in paired])
    evaluate, inliers = piecewise_fit(
        x, y, KNOT_SECONDS * tap.clock_freq, np.concatenate(isolated)
    )
    lateness_ns = (y - evaluate(x)) * NS
    floor = np.percentile(lateness_ns[inliers], LATENESS_FLOOR_PERCENTILE)
    relative_ns = lateness_ns - floor
    motors_report = {}
    offset = 0
    lows = {}
    for (role, commanded, times, channel), alone in zip(paired, isolated):
        rel = relative_ns[offset : offset + commanded.size]
        offset += commanded.size
        lows[role] = (commanded & 0xFFFFFFFF, rel)
        interval_error = (
            np.diff(times) - np.diff(evaluate(commanded.astype(np.float64)))
        ) * NS
        after = np.searchsorted(channel.step_ticks, channel.dir_ticks)
        has_step = after < channel.step_ticks.size
        dir_setup = (
            channel.step_ticks[after[has_step]] - channel.dir_ticks[has_step]
        )
        motors_report[role] = {
            "steps": int(commanded.size),
            "lateness_ns": _stats_ns(rel),
            "isolated_lateness_ns": _stats_ns(rel[alone]),
            "coincident_steps": int(np.count_nonzero(~alone)),
            "coincident_lateness_ns": _stats_ns(rel[~alone]),
            **{
                f"steps_late_over_{t}ns": int(np.count_nonzero(rel > t))
                for t in LATE_THRESHOLDS_NS
            },
            "interval_error_abs_ns": _stats_ns(np.abs(interval_error)),
            "dir_edges": int(channel.dir_ticks.size),
            "dir_setup_min_ns": float(
                dir_setup.min() / capture_events.EVENT_TICK_HZ * NS
            )
            if dir_setup.size
            else None,
        }
    load_late = []
    for event in _step_load_late_events(run_dir):
        behind = _signed32(int(event["arg0"]))
        commanded_low = (int(event["arg1"]) + behind) & 0xFFFFFFFF
        matches = [
            {"motor": role, "measured_lateness_ns": float(rel[index])}
            for role, (low, rel) in lows.items()
            for index in np.flatnonzero(low == commanded_low)
        ]
        load_late.append(
            {"behind_ns": -behind / tap.clock_freq * NS, "matched": matches}
        )
    return {
        "mcu_id": tap.mcu_id,
        "clock_freq": tap.clock_freq,
        "fit_knot_seconds": KNOT_SECONDS,
        "fit_inlier_fraction": float(inliers.mean()),
        "lateness_floor_percentile": LATENESS_FLOOR_PERCENTILE,
        "lateness_ns": _stats_ns(relative_ns),
        **{
            f"steps_late_over_{t}ns": int(np.count_nonzero(relative_ns > t))
            for t in LATE_THRESHOLDS_NS
        },
        "motors": motors_report,
        "step_load_late_events": len(load_late),
        "step_load_late_matched": sum(1 for e in load_late if e["matched"]),
        "step_load_late_examples": load_late[:REPORTED_LOAD_LATE],
        "projections": len(tap.projections),
    }
