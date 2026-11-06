"""The corner budget on the steps the printer took.

`analyze.py --corner-budget` calls `report`. Every extruding vertex the
config's corner budget holds (`corner_vertex_budgets`, the path-accuracy
audit's reading of it) is missed by three XY paths:

- plan: the stream plan's shaped x and y;
- reference: the analyzer's reference a and b steps, the plan quantized
  onto the motors' step lattices exactly as the printer steps it;
- capture: the captured a and b steps, mapped to stream time by the
  epoch's alignment.

A step lands on its lattice line at its time, so a motor's path is
rebuilt by joining its steps linearly; between two steps the motor stays
within one step of that line. The rebuilt XY therefore strays from the
motion it samples by at most the step quantization: the corexy map of one
step on each motor. The captured miss must stay within the vertex's
budget, the shaped carriers' `max_path_deviation` and that quantization.
"""

from __future__ import annotations

import itertools

import numpy as np

PLAN_DT_S = 1e-4
FINE_DT_S = 1e-5
APEX_SEARCH_MM = 2.0
SEARCH_CHUNK = 20_000
XY_ROLES = ("a", "b")
SOURCES = ("plan", "reference", "capture")


def quantization_mm(motors: dict, axis_weights: dict) -> float:
    """The farthest the XY rebuilt from a and b steps strays from the motion
    they sample: one step on each motor, mapped back through corexy."""
    matrix = np.array([axis_weights[role][:2] for role in XY_ROLES])
    steps = np.array([motors[role].step_mm for role in XY_ROLES])
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


class _Cursor:
    """Walks the plan's samples forward through the blocks: each vertex is
    found after the one before it, as the nozzle reaches them in order."""

    def __init__(self, blocks):
        self.blocks = blocks
        self.block = 0
        self.index = 0

    def find(self, vertex: np.ndarray):
        while self.block < len(self.blocks):
            t, xy = self.blocks[self.block][:2]
            start = self.index
            while start < len(t):
                stop = min(start + SEARCH_CHUNK, len(t))
                distance = np.hypot(*(xy[start:stop] - vertex).T)
                near = np.flatnonzero(distance < APEX_SEARCH_MM)
                if near.size:
                    first = start + int(near[0])
                    last = first
                    while (
                        last + 1 < len(t)
                        and np.hypot(*(xy[last + 1] - vertex)) < APEX_SEARCH_MM
                    ):
                        last += 1
                    self.index = last + 1
                    return self.block, first, last
                start = stop
            self.block += 1
            self.index = 0
        raise RuntimeError(f"the plan never nears the vertex {vertex.tolist()}")


def report(inputs, detail: dict, planned, axis_weights, stream_epochs, engine):
    """Per held extruding vertex of the print: its budget and the plan's,
    the reference steps' and the captured steps' XY miss, and whether the
    capture stays within budget + carrier slack + step quantization."""
    budgets = engine.corner_vertex_budgets(
        planned.waypoints, inputs.config.config_text
    )
    motors = {m.role: m for m in inputs.motors}
    quantization = quantization_mm(motors, axis_weights)
    slack = budgets["carrier_slack_mm"]
    reference = detail["reference"]
    epochs = detail["epochs"]
    capture_events = detail["capture_events"]
    plan_blocks = []
    for (t_start, t_end, rows), reference_block in zip(
        stream_epochs(inputs.config.config_text, [0, 1], PLAN_DT_S),
        reference.blocks,
        strict=True,
    ):
        t = np.linspace(0.0, t_end - t_start, rows.shape[1])
        offset = reference_block.stream_offset_s
        alignment = next(
            epoch.alignment
            for epoch in epochs
            if epoch.reference.end_s >= offset
            and epoch.reference.start_s <= offset + reference_block.duration_s
        )
        plan_blocks.append(
            (t, np.column_stack([rows[0][:, 0], rows[1][:, 0]]), offset, alignment)
        )
    cursor = _Cursor(plan_blocks)
    rows = []
    for vertex in budgets["vertices"]:
        if not (vertex["held"] and vertex["extrudes"]):
            continue
        point = np.array(vertex["position"][:2])
        block, first, last = cursor.find(point)
        t, xy, offset, alignment = plan_blocks[block]
        lo, hi = max(first - 1, 0), min(last + 1, len(t) - 1)
        fine = np.arange(t[lo], t[hi], FINE_DT_S)
        plan_xy = np.column_stack(
            [np.interp(fine, t, xy[:, axis]) for axis in (0, 1)]
        )
        stream_t = fine + offset
        misses = {
            "plan": _miss(plan_xy, point),
            "reference": _miss(
                _steps_xy(reference.events, motors, axis_weights, stream_t),
                point,
            ),
            "capture": _miss(
                _steps_xy(
                    capture_events,
                    motors,
                    axis_weights,
                    alignment.to_capture(stream_t),
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
