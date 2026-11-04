"""Nozzle flow through the corners of a bench print, from captured steps.

`analyze.py --corner-flow` calls `report`. Every extruding corner, short
segment curve and ratio step of the print is measured three ways with
tools/flow_metric.py, the definition tools/corner_flow.py uses:

- offline: the snapshot plan's continuous x, y and e tracks;
- reference: the analyzer's reference steps for a, b and e, the plan
  quantized onto the motors' step lattices exactly as the printer steps it;
- capture: the captured a, b and e steps, mapped to stream time by the
  epoch's alignment.

Steps sit on the lattice: a motor steps when its position crosses a lattice
line, so each step event is an exact (time, position) point of the motion
and the tracks are rebuilt by joining them linearly. reference minus
offline is the step quantization; capture minus reference is what the
printer executed differently from its own command.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from tools import flow_metric
from tools.corner_flow import PrinterConfig

CORNER_TURN_DEG = 20.0
CURVE_TURN_DEG = 1.0
CURVE_SEGMENT_MM = 2.0
CURVE_MIN_VERTICES = 3
EXTRUDING_RATIO = 1e-6
VELOCITY_WINDOW_S = 2e-3
ACCEL_WINDOW_S = 5e-3
EVENT_MARGIN_S = 1.0
PLAN_AXES = [0, 1, 3]
CORNER_METRICS = ("apex", "peak", "dip", "width_mm", "blob_nl")
STEP_METRICS = ("moved_nl", "net_nl", "switch_mm")
XY_ROLES = ("a", "b")


@dataclass
class Feature:
    kind: str
    apex_xy: tuple[float, float]
    turn_deg: float
    speed_mm_s: float
    window_mm: float
    ratio: float
    ratio_after: float


def _segments(block: list[dict]):
    for previous, waypoint in zip(block, block[1:]):
        dx = waypoint["position"][0] - previous["position"][0]
        dy = waypoint["position"][1] - previous["position"][1]
        de = waypoint["position"][3] - previous["position"][3]
        length = math.hypot(dx, dy)
        yield length, (dx, dy), de, waypoint["feedrate"]


def features(block: list[dict]) -> list[Feature]:
    """The block's extruding corners (a turn of at least CORNER_TURN_DEG
    between two extruding moves), curves (runs of short extruding segments
    each turning a little) and ratio steps (two extruding moves laying
    different beads, turning or not)."""
    segments = list(_segments(block))
    found: list[Feature] = []
    curve: list[int] = []

    def close_curve() -> None:
        if len(curve) >= CURVE_MIN_VERTICES:
            middle = curve[len(curve) // 2]
            run = sum(segments[i][0] for i in range(curve[0], curve[-1] + 2))
            length, _, de, speed = segments[middle]
            found.append(
                Feature(
                    "curve",
                    tuple(block[middle + 1]["position"][:2]),
                    sum(_turn(segments, i) for i in curve),
                    speed,
                    run / 2.0,
                    de / length,
                    de / length,
                )
            )
        curve.clear()

    for index in range(len(segments) - 1):
        length_in, _, de_in, speed = segments[index]
        length_out, _, de_out, _ = segments[index + 1]
        extruding = (
            length_in > 0
            and length_out > 0
            and de_in / length_in > EXTRUDING_RATIO
            and de_out / length_out > EXTRUDING_RATIO
        )
        turn = _turn(segments, index) if extruding else 0.0
        window = min(
            flow_metric.CORNER_WINDOW_MM, 0.5 * min(length_in, length_out)
        )
        apex = tuple(block[index + 1]["position"][:2])
        ratio_in = de_in / length_in if extruding else 0.0
        ratio_out = de_out / length_out if extruding else 0.0
        if extruding and flow_metric.is_ratio_step(ratio_in, ratio_out):
            close_curve()
            found.append(
                Feature("step", apex, turn, speed, window, ratio_in, ratio_out)
            )
        elif extruding and turn >= CORNER_TURN_DEG:
            close_curve()
            found.append(
                Feature("corner", apex, turn, speed, window, ratio_in, ratio_in)
            )
        elif (
            extruding
            and turn >= CURVE_TURN_DEG
            and max(length_in, length_out) <= CURVE_SEGMENT_MM
        ):
            curve.append(index)
        else:
            close_curve()
    close_curve()
    return found


def _turn(segments, index: int) -> float:
    (_, (ax, ay), _, _), (_, (bx, by), _, _) = (
        segments[index],
        segments[index + 1],
    )
    cos = (ax * bx + ay * by) / (math.hypot(ax, ay) * math.hypot(bx, by))
    return math.degrees(math.acos(max(-1.0, min(1.0, cos))))


def _local(epoch) -> tuple[np.ndarray, np.ndarray]:
    """An epoch's sample times from its own start, and its axis rows."""
    t_start, t_end, axes_pva = epoch
    return np.linspace(0.0, t_end - t_start, axes_pva.shape[1]), axes_pva


def _same_time(times: np.ndarray) -> np.ndarray:
    return times


def _track(
    times: np.ndarray, levels: np.ndarray, step_mm: float, t: np.ndarray
):
    """A motor's position on `t` from its step events: each step lands on
    its lattice line at its time; joined linearly, held outside."""
    return np.interp(t, times, levels * step_mm)


def _step_source(events, to_stream, select, offset, motors, weights, t):
    """The nozzle's x, y and e on the block-local times `t`, rebuilt from
    the a, b and e step events `select` picks, `to_stream` mapping their
    times onto the stream the block starts at `offset` in."""
    tracks = {}
    for role in (*XY_ROLES, "e"):
        steps = events[role].between(*select)
        tracks[role] = _track(
            to_stream(steps.time) - offset,
            steps.level.astype(np.float64),
            motors[role].step_mm,
            t,
        )
    matrix = np.array([weights[role][:2] for role in XY_ROLES])
    xy = np.linalg.solve(matrix, np.vstack([tracks[r] for r in XY_ROLES])).T
    return xy, tracks["e"]


def _extruder_peaks(velocity: np.ndarray, accel: np.ndarray, span) -> dict:
    return {
        "ve_max": float(velocity[span].max()),
        "ve_min": float(velocity[span].min()),
        "ae_max": float(np.abs(accel[span]).max()),
    }


def _smoothed_gradient(values: np.ndarray, dt: float, window_s: float):
    width = max(1, int(round(window_s / dt)))
    kernel = np.ones(width) / width
    return np.convolve(np.gradient(values, dt), kernel, mode="same")


def feature_spans(t, xy, speed, feats) -> list[np.ndarray]:
    """The samples each feature's window covers while the nozzle moves,
    found on the plan so every source is read over the same instants."""
    trace_s = np.concatenate(
        [[0.0], np.cumsum(np.hypot(*np.diff(xy, axis=0).T))]
    )
    moving = speed > flow_metric.MOVING_SPEED_MM_S
    return [
        moving & (np.abs(trace_s - trace_s[apex]) <= f.window_mm)
        for apex, f in zip(
            flow_metric.apex_indices(xy, [f.apex_xy for f in feats]), feats
        )
    ]


def _feature_report(trace, apex, feat: Feature, nominal, bead_area) -> dict:
    if feat.kind == "step":
        return flow_metric.step_report(
            trace,
            apex,
            (feat.ratio / nominal, feat.ratio_after / nominal),
            feat.window_mm,
            bead_area,
        )
    return flow_metric.corner_report(trace, apex, feat.window_mm, bead_area)


def _source_report(
    t, xy, e, commanded, feats, spans, melt_k, nominal, bead_area
):
    trace = flow_metric.flow_trace(t, xy, e, commanded, melt_k, nominal)
    apexes = flow_metric.apex_indices(xy, [f.apex_xy for f in feats])
    dt = t[1] - t[0]
    velocity = _smoothed_gradient(e, dt, VELOCITY_WINDOW_S)
    accel = _smoothed_gradient(velocity, dt, ACCEL_WINDOW_S)
    return [
        {
            **_feature_report(trace, apex, f, nominal, bead_area),
            **_extruder_peaks(velocity, accel, span),
        }
        for apex, f, span in zip(apexes, feats, spans)
    ]


def _epoch_for(epochs, start_s: float, end_s: float):
    for epoch in epochs:
        if (
            epoch.reference.end_s >= start_s
            and epoch.reference.start_s <= end_s
        ):
            return epoch
    raise RuntimeError(
        f"no capture epoch pairs the stream span {start_s}..{end_s}"
    )


def report(
    inputs,
    detail: dict,
    blocks: list[list[dict]],
    axis_weights: dict,
    stream_epochs,
) -> dict:
    """Per extruding feature of the print: the flow deviation (fraction of
    the nominal bead) or, at a ratio step, the material it moves, and the
    extruder peaks, from the plan, the reference steps and the captured
    steps. `blocks` are the analyzer's planned epochs, one per reference
    block; `stream_epochs(config_text, axes, sample_period_s)` samples the
    print planned as one stream across each of them."""
    config = PrinterConfig.parse(inputs.config.config_text)
    melt_k = config.advance_k()
    filament = config.getfloat("extruder", "filament_diameter", 1.75)
    filament_area = math.pi * (filament / 2.0) ** 2
    reference = detail["reference"]
    epochs = detail["epochs"]
    capture_events = detail["capture_events"]
    motors = {m.role: m for m in inputs.motors}
    shaped_epochs, unshaped_epochs = (
        stream_epochs(text, PLAN_AXES, flow_metric.SAMPLE_DT_S)
        for text in (inputs.config.config_text, config.unshaped().text())
    )
    rows = []
    for block, reference_block, shaped_epoch, unshaped_epoch in zip(
        blocks, reference.blocks, shaped_epochs, unshaped_epochs, strict=True
    ):
        feats = features(block)
        if not feats:
            continue
        duration = reference_block.duration_s
        t, shaped = _local(shaped_epoch)
        t_raw, raw = _local(unshaped_epoch)
        xy_plan = np.column_stack([shaped[0][:, 0], shaped[1][:, 0]])
        vertices = flow_metric.path_vertices(
            tuple(waypoint["position"][:2]) for waypoint in block
        )
        commanded = flow_metric.commanded_bead(t, xy_plan, t_raw, raw, vertices)
        spans = feature_spans(
            t, xy_plan, np.hypot(shaped[0][:, 1], shaped[1][:, 1]), feats
        )
        offset = reference_block.stream_offset_s
        alignment = _epoch_for(epochs, offset, offset + duration).alignment
        span = (offset - EVENT_MARGIN_S, offset + duration + EVENT_MARGIN_S)
        sources = {
            "offline": (xy_plan, shaped[2][:, 0]),
            "reference": _step_source(
                reference.events,
                _same_time,
                span,
                offset,
                motors,
                axis_weights,
                t,
            ),
            "capture": _step_source(
                capture_events,
                alignment.to_stream,
                tuple(alignment.to_capture(np.array(span))),
                offset,
                motors,
                axis_weights,
                t,
            ),
        }
        nominal = feats[0].ratio
        bead_area = nominal * filament_area
        reports = {
            name: _source_report(
                t, xy, e, commanded, feats, spans, melt_k, nominal, bead_area
            )
            for name, (xy, e) in sources.items()
        }
        for index, feat in enumerate(feats):
            rows.append(
                {
                    "kind": feat.kind,
                    "apex_xy": list(feat.apex_xy),
                    "turn_deg": feat.turn_deg,
                    "speed_mm_s": feat.speed_mm_s,
                    "window_mm": feat.window_mm,
                    **{name: reports[name][index] for name in reports},
                }
            )
    return {
        "melt_k_s": melt_k,
        "features": rows,
        "max_abs": _max_abs(rows),
    }


def _max_abs(rows: list[dict]) -> dict:
    """Over every feature and metric: the largest quantization (reference
    minus offline) and execution (capture minus reference) differences."""
    out = {}
    for pair in (
        ("reference", "offline"),
        ("capture", "reference"),
        ("capture", "offline"),
    ):
        key = f"{pair[0]}_minus_{pair[1]}"
        out[key] = {
            metric: max(
                (
                    abs(row[pair[0]][metric] - row[pair[1]][metric])
                    for row in rows
                    if metric in row[pair[0]]
                ),
                default=0.0,
            )
            for metric in CORNER_METRICS + STEP_METRICS
        }
    return out
