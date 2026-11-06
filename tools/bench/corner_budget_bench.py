"""The corner budget on the steps the printer took.

`analyze.py --corner-budget` calls `report`. Every extruding vertex the
config's corner budget holds (`corner_vertex_budgets`, the path-accuracy
audit's reading of it) is missed by three XY paths:

- plan: the stream plan's shaped x and y;
- reference: the analyzer's reference a and b steps, the plan quantized
  onto the motors' step lattices exactly as the printer steps it;
- capture: the captured a and b steps, mapped to stream time by the
  epoch's alignment.

Each vertex is read where the stream plan comes closest to it (the
path-accuracy audit's cursor, `plan_t`), over the stretch of the plan
within WINDOW_MM of path either side, on the same instants for every
source. A motor's path is rebuilt by joining its steps linearly: a step
fires as the motion crosses half way to the next lattice line and a
reversal between two steps leaves the join a further step off, so the
rebuilt XY strays from the motion by at most the corexy map of
QUANTIZATION_STEPS on each motor. The captured miss must stay within the
vertex's budget, the shaped carriers' `max_path_deviation` and that
quantization.
"""

from __future__ import annotations

import itertools

import numpy as np

PLAN_DT_S = 1e-4
FINE_DT_S = 1e-5
WINDOW_MM = 1.0
QUANTIZATION_STEPS = 2.0
XY_ROLES = ("a", "b")
SOURCES = ("plan", "reference", "capture")


def quantization_mm(motors: dict, axis_weights: dict) -> float:
    """The farthest the XY rebuilt from a and b steps strays from the motion
    they sample: QUANTIZATION_STEPS on each motor, mapped back through
    corexy."""
    matrix = np.array([axis_weights[role][:2] for role in XY_ROLES])
    steps = QUANTIZATION_STEPS * np.array(
        [motors[role].step_mm for role in XY_ROLES]
    )
    return max(
        float(np.hypot(*np.linalg.solve(matrix, steps * np.array(signs))))
        for signs in itertools.product((-1.0, 1.0), repeat=2)
    )


def _motor_mm(events, step_mm: float, t: np.ndarray) -> np.ndarray:
    """A motor's position on `t` (its events' clock) from its steps joined
    linearly, held before the first and after the last."""
    if len(events.time) == 0:
        raise RuntimeError("a corner motor never stepped in the run")
    lo = max(int(np.searchsorted(events.time, t[0], "left")) - 1, 0)
    hi = int(np.searchsorted(events.time, t[-1], "right")) + 1
    return np.interp(
        t, events.time[lo:hi], events.level[lo:hi].astype(np.float64)
    ) * step_mm


def _steps_xy(events, motors, axis_weights, t_events) -> np.ndarray:
    matrix = np.array([axis_weights[role][:2] for role in XY_ROLES])
    tracks = np.vstack(
        [
            _motor_mm(events[role], motors[role].step_mm, t_events)
            for role in XY_ROLES
        ]
    )
    return np.linalg.solve(matrix, tracks).T


def _miss(xy: np.ndarray, vertex: np.ndarray) -> float:
    """The closest the chords joining `xy` pass to `vertex`."""
    a, b = xy[:-1], xy[1:]
    ab = b - a
    length_sq = np.einsum("ij,ij->i", ab, ab)
    along = np.where(
        length_sq > 0.0,
        np.einsum("ij,ij->i", vertex - a, ab) / np.maximum(length_sq, 1e-300),
        0.0,
    ).clip(0.0, 1.0)
    nearest = a + along[:, None] * ab
    return float(np.hypot(*(nearest - vertex).T).min())


def _window(t: np.ndarray, xy: np.ndarray, apex: float) -> tuple[float, float]:
    """The plan's instants within WINDOW_MM of path either side of `apex`."""
    centre = int(np.clip(np.searchsorted(t, apex), 0, len(t) - 1))
    bounds = []
    for step in (-1, 1):
        index, travelled = centre, 0.0
        while 0 < index < len(t) - 1 and travelled < WINDOW_MM:
            travelled += float(np.hypot(*(xy[index + step] - xy[index])))
            index += step
        bounds.append(t[index])
    return bounds[0], bounds[1]


def _block_of(blocks, plan_t: float):
    for block in blocks:
        if block["t_start"] <= plan_t <= block["t_end"]:
            return block
    raise RuntimeError(f"no planned epoch holds the vertex reached at t={plan_t}")


def report(inputs, detail: dict, planned, axis_weights, stream_epochs, engine):
    """Per held extruding vertex of the print: its budget and the plan's,
    the reference steps' and the captured steps' XY miss, and whether the
    capture stays within budget + carrier slack + step quantization."""
    budgets = engine.corner_vertex_budgets(
        planned.waypoints, inputs.config.config_text, inputs.step_benchmarks
    )
    motors = {m.role: m for m in inputs.motors}
    quantization = quantization_mm(motors, axis_weights)
    slack = budgets["carrier_slack_mm"]
    reference = detail["reference"]
    epochs = detail["epochs"]
    capture_events = detail["capture_events"]
    blocks = []
    for (t_start, t_end, rows), reference_block in zip(
        stream_epochs(inputs.config.config_text, [0, 1], PLAN_DT_S),
        reference.blocks,
        strict=True,
    ):
        offset = reference_block.stream_offset_s
        blocks.append(
            {
                "t_start": t_start,
                "t_end": t_end,
                "t": np.linspace(0.0, t_end - t_start, rows.shape[1]),
                "xy": np.column_stack([rows[0][:, 0], rows[1][:, 0]]),
                "offset": offset,
                "alignment": next(
                    epoch.alignment
                    for epoch in epochs
                    if epoch.reference.end_s >= offset
                    and epoch.reference.start_s
                    <= offset + reference_block.duration_s
                ),
            }
        )
    rows = []
    for vertex in budgets["vertices"]:
        if not (vertex["held"] and vertex["extrudes"]):
            continue
        point = np.array(vertex["position"][:2])
        block = _block_of(blocks, vertex["plan_t"])
        t, xy = block["t"], block["xy"]
        lo, hi = _window(t, xy, vertex["plan_t"] - block["t_start"])
        fine = np.arange(lo, hi + FINE_DT_S, FINE_DT_S)
        stream_t = fine + block["offset"]
        misses = {
            "plan": vertex["plan_miss_mm"],
            "reference": _miss(
                _steps_xy(reference.events, motors, axis_weights, stream_t),
                point,
            ),
            "capture": _miss(
                _steps_xy(
                    capture_events,
                    motors,
                    axis_weights,
                    block["alignment"].to_capture(stream_t),
                ),
                point,
            ),
        }
        bound = vertex["budget_mm"] + slack + quantization
        rows.append(
            {
                "xy": point.tolist(),
                "turn_deg": vertex["turn_deg"],
                "corner": vertex["corner"],
                "budget_mm": vertex["budget_mm"],
                **{f"{name}_miss_mm": misses[name] for name in SOURCES},
                "within": misses["capture"] <= bound,
            }
        )
    over = [row for row in rows if not row["within"]]
    return {
        "budget": list(budgets["budget"]),
        "carrier_slack_mm": slack,
        "quantization_mm": quantization,
        "vertices": len(rows),
        "over": over,
        "worst": {
            f"{name}_share": max(
                (row[f"{name}_miss_mm"] / row["budget_mm"] for row in rows),
                default=0.0,
            )
            for name in SOURCES
        },
        "worst_capture_minus_plan_mm": max(
            (row["capture_miss_mm"] - row["plan_miss_mm"] for row in rows),
            default=0.0,
        ),
        "ok": not over,
        "rows": rows,
    }
