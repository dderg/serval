"""Nozzle flow through corners: one definition for the offline planner
(tools/corner_flow.py) and captured bench steps (tools/bench/corner_flow_bench.py).

The commanded e track is passed through a first-order melt model
(``e_cmd = e_out + K * d(e_out)/dt``, K the configured linear pressure
advance), and the material leaving the nozzle is measured per millimetre of
the shaped nozzle path. The deviation is that flow minus the bead the G-code
commands at the same place of the unshaped plan, over the nominal bead:
0 lays the commanded bead exactly. Where the ratio steps, the bead is read
instead as the material laid across the switch and the path the switch
takes.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

SAMPLE_DT_S = 10e-6
PATH_STEP_MM = 0.02
MOVING_SPEED_MM_S = 5.0
RAW_REST_SPEED_MM_S = 1e-3
WIDTH_THRESHOLD = 0.05
CORNER_WINDOW_MM = 15.0
APEX_SEARCH_MM = 2.0
DWELL_TOLERANCE_MM = 1e-6
RATIO_STEP_MIN = 0.05
SWITCH_SPAN = (0.05, 0.95)
NL_PER_MM3 = 1000.0


def melted(t: np.ndarray, e_cmd: np.ndarray, melt_k: float) -> np.ndarray:
    if melt_k <= 0.0:
        return e_cmd
    dt = t[1] - t[0]
    alpha = math.exp(-dt / melt_k)
    slope = np.diff(e_cmd) / dt
    lag = np.zeros_like(e_cmd)
    for index, drive in enumerate(melt_k * slope * (alpha - 1.0), start=1):
        lag[index] = alpha * lag[index - 1] + drive
    return e_cmd + lag


def commanded_ratio(t: np.ndarray, raw: np.ndarray, t_at: np.ndarray):
    """The extrusion per mm the G-code commands at each time, read off the
    unshaped plan: the follower lays each move's ratio over the shaped path
    the nozzle covers during that move. Where the unshaped toolhead rests,
    the nearest moving instant's ratio stands."""
    speed = np.hypot(raw[0][:, 1], raw[1][:, 1])
    moving = speed > 1e-6
    ratio = np.zeros_like(speed)
    ratio[moving] = raw[2][moving, 1] / speed[moving]
    indices = np.flatnonzero(moving)
    nearest = np.clip(np.searchsorted(indices, np.arange(len(t))), 1, None)
    nearest = np.minimum(nearest, len(indices) - 1)
    previous = indices[nearest - 1]
    following = indices[nearest]
    here = np.arange(len(t))
    chosen = np.where(here - previous <= following - here, previous, following)
    ratio = ratio[chosen]
    return np.interp(t_at, t, ratio)


def is_ratio_step(before: float, after: float) -> bool:
    return abs(after - before) >= RATIO_STEP_MIN * max(abs(before), abs(after))


def path_vertices(points) -> list[tuple[float, float]]:
    """The toolhead's start and every point it moves on to, a point it does
    not leave (an extrude-only move, a dwell) counted once."""
    vertices: list[tuple[float, float]] = []
    for x, y in points:
        if not vertices or vertices[-1] != (x, y):
            vertices.append((x, y))
    return vertices


def vertex_dwells(xy: np.ndarray, vertices) -> list[tuple[int, int] | None]:
    """Where the path `xy` passes each of `vertices`, in order: the first
    and last sample of its closest approach, one sample where it sweeps by
    and its whole stay where it rests. Each vertex is searched within
    APEX_SEARCH_MM, and no further than halfway to either neighbour so two
    vertices never claim the same samples; None where the path stays out."""
    points = np.asarray(vertices, dtype=float)
    halfway = np.hypot(*np.diff(points, axis=0).T) / 2.0
    assert np.all(halfway > 0.0), "consecutive vertices coincide"
    radii = np.minimum(
        np.concatenate([[APEX_SEARCH_MM], halfway]),
        np.concatenate([halfway, [APEX_SEARCH_MM]]),
    )
    radii = np.minimum(radii, APEX_SEARCH_MM)
    found: list[tuple[int, int] | None] = []
    start = 0
    for (cx, cy), radius in zip(points, radii):
        distance = np.hypot(xy[start:, 0] - cx, xy[start:, 1] - cy)
        near = np.flatnonzero(distance < radius)
        if near.size == 0:
            found.append(None)
            continue
        entry = near[0]
        exits = np.flatnonzero(distance[entry:] >= radius)
        run = distance[entry : entry + exits[0] if exits.size else None]
        closest = np.flatnonzero(run <= run.min() + DWELL_TOLERANCE_MM)
        found.append((start + entry + closest[0], start + entry + closest[-1]))
        start += entry + run.size
    return found


def unshaped_times(t, xy, t_raw, xy_raw, vertices) -> np.ndarray:
    """The unshaped plan's time at each shaped sample. The two are planned
    apart, shaping changing the speeds the planner allows, so the lag
    between them drifts along the path; they are paired where both pass a
    vertex (arriving and leaving where they rest there) and at their ends,
    and timed linearly between, so each ratio step falls at its vertex on
    both."""
    pairs = [
        (shaped, unshaped)
        for shaped, unshaped in zip(
            vertex_dwells(xy, vertices), vertex_dwells(xy_raw, vertices)
        )
        if shaped and unshaped
    ]
    shaped_at = [t[0], *(t[i] for shaped, _ in pairs for i in shaped), t[-1]]
    raw_at = [
        t_raw[0],
        *(t_raw[i] for _, unshaped in pairs for i in unshaped),
        t_raw[-1],
    ]
    return np.interp(t, shaped_at, raw_at)


def apex_indices(xy: np.ndarray, corners) -> list[int]:
    found = []
    start = 0
    for cx, cy in corners:
        distance = np.hypot(xy[start:, 0] - cx, xy[start:, 1] - cy)
        near = np.flatnonzero(distance < APEX_SEARCH_MM)
        if near.size == 0:
            raise RuntimeError(f"the path never nears ({cx}, {cy})")
        run_end = near[0]
        while run_end + 1 < distance.size and distance[run_end + 1] < (
            APEX_SEARCH_MM
        ):
            run_end += 1
        apex = near[0] + int(np.argmin(distance[near[0] : run_end + 1]))
        found.append(start + apex)
        start += run_end + 1
    return found


@dataclass
class CommandedBead:
    """What the G-code asks for at each shaped sample: the extrusion ratio
    and whether the unshaped toolhead is moving (a toolhead at rest lays no
    bead, so material moved there is left out)."""

    ratio: np.ndarray
    raw_moving: np.ndarray


def commanded_bead(t, xy, t_raw, raw, vertices) -> CommandedBead:
    """`vertices` are the program's (`path_vertices`)."""
    xy_raw = np.column_stack([raw[0][:, 0], raw[1][:, 0]])
    at = unshaped_times(t, xy, t_raw, xy_raw, vertices)
    raw_speed = np.hypot(raw[0][:, 1], raw[1][:, 1])
    return CommandedBead(
        ratio=commanded_ratio(t_raw, raw, at),
        raw_moving=np.interp(at, t_raw, raw_speed) > RAW_REST_SPEED_MM_S,
    )


@dataclass
class FlowTrace:
    s: np.ndarray
    middle: np.ndarray
    laid: np.ndarray
    deviation: np.ndarray


def flow_trace(
    t: np.ndarray,
    xy: np.ndarray,
    e_cmd: np.ndarray,
    commanded: CommandedBead,
    melt_k: float,
    nominal_ratio: float,
) -> FlowTrace:
    """Nozzle flow per mm of the path `xy` traces, and that flow against the
    commanded bead, both over the nominal bead, on a PATH_STEP_MM grid."""
    step = np.hypot(np.diff(xy[:, 0]), np.diff(xy[:, 1]))
    s = np.concatenate([[0.0], np.cumsum(step)])
    advancing = np.concatenate([[True], step > 0])
    e_out = melted(t, e_cmd, melt_k)
    laying = advancing[1:] & commanded.raw_moving[1:]
    laid = np.concatenate([[0.0], np.where(laying, np.diff(e_out), 0.0)])
    e_out = e_out[0] + np.cumsum(laid)
    grid = np.arange(0.0, s[-1], PATH_STEP_MM)
    e_grid = np.interp(grid, s[advancing], e_out[advancing])
    middle = grid[:-1] + 0.5 * PATH_STEP_MM
    r_grid = np.interp(middle, s, commanded.ratio)
    laid_per_mm = np.diff(e_grid) / PATH_STEP_MM / nominal_ratio
    return FlowTrace(
        s=s,
        middle=middle,
        laid=laid_per_mm,
        deviation=laid_per_mm - r_grid / nominal_ratio,
    )


def corner_report(
    trace: FlowTrace, apex: int, window_mm: float, bead_area_mm2: float
) -> dict:
    s_apex = trace.s[apex]
    near = np.abs(trace.middle - s_apex) <= window_mm
    local = trace.deviation[near]
    return {
        "s_mm": float(s_apex),
        "apex": float(np.interp(s_apex, trace.middle, trace.deviation)),
        "peak": float(local.max()),
        "dip": float(local.min()),
        "width_mm": float(
            np.count_nonzero(np.abs(local) > WIDTH_THRESHOLD) * PATH_STEP_MM
        ),
        "blob_nl": float(
            np.clip(local, 0.0, None).sum()
            * PATH_STEP_MM
            * bead_area_mm2
            * NL_PER_MM3
        ),
    }


def step_report(
    trace: FlowTrace,
    apex: int,
    ratios: tuple[float, float],
    window_mm: float,
    bead_area_mm2: float,
) -> dict:
    """A ratio step from `ratios[0]` to `ratios[1]` (over the nominal bead)
    at the sample `apex`, as the G-code lays it: sharp, where the nozzle
    passes the vertex. Read within `window_mm` either side:

    - `moved_nl`: the material laid across the switch, the mean of the
      excess over the old ratio before it and the shortfall from the new one
      after it (a bead switched sharply moves none);
    - `net_nl`: those two summed, what the step gains or loses;
    - `switch_mm`: the path the laid bead takes from 5% to 95% of the step.
    """
    before, after = ratios
    s_apex = trace.s[apex]
    early = (trace.middle >= s_apex - window_mm) & (trace.middle < s_apex)
    late = (trace.middle >= s_apex) & (trace.middle <= s_apex + window_mm)
    early_mm = (trace.laid[early] - before).sum() * PATH_STEP_MM
    late_mm = (trace.laid[late] - after).sum() * PATH_STEP_MM
    nl_per_mm = bead_area_mm2 * NL_PER_MM3
    window = early | late
    progress = (trace.laid[window] - before) / (after - before)
    s = trace.middle[window]
    started = s[progress > SWITCH_SPAN[0]]
    unfinished = s[progress < SWITCH_SPAN[1]]
    switch_mm = (
        float(unfinished.max() - started.min())
        if started.size and unfinished.size
        else 0.0
    )
    return {
        "s_mm": float(s_apex),
        "moved_nl": 0.5 * (abs(early_mm) + abs(late_mm)) * nl_per_mm,
        "net_nl": (early_mm + late_mm) * nl_per_mm,
        "switch_mm": max(switch_mm, 0.0),
    }
