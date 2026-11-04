"""Nozzle flow through corners: one definition for the offline planner
(tools/corner_flow.py) and captured bench steps (tools/bench/corner_flow_bench.py).

The commanded e track is passed through a first-order melt model
(``e_cmd = e_out + K * d(e_out)/dt``, K the configured linear pressure
advance), and the material leaving the nozzle is measured per millimetre of
the shaped nozzle path. The deviation is that flow minus the bead the G-code
commands at the same instant of the unshaped plan, over the nominal bead:
0 lays the commanded bead exactly.
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


def timeline_offset(t, xy, t_raw, raw) -> float:
    """How much later the shaped plan runs than the unshaped one: shaping
    holds the start for its kernels' reach. Found as the shift that best
    lays the unshaped toolhead path onto the shaped one."""
    stride = max(len(t) // 4000, 1)
    t_probe, xy_probe = t[::stride], xy[::stride]

    def mismatch(shift: float) -> float:
        x = np.interp(t_probe - shift, t_raw, raw[0][:, 0])
        y = np.interp(t_probe - shift, t_raw, raw[1][:, 0])
        return float(np.mean(np.hypot(x - xy_probe[:, 0], y - xy_probe[:, 1])))

    coarse = min(np.arange(-0.1, 0.1, 5e-4), key=mismatch)
    return float(
        min(np.arange(coarse - 5e-4, coarse + 5e-4, 1e-5), key=mismatch)
    )


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


def commanded_bead(t, xy, t_raw, raw) -> CommandedBead:
    offset = timeline_offset(t, xy, t_raw, raw)
    raw_speed = np.hypot(raw[0][:, 1], raw[1][:, 1])
    return CommandedBead(
        ratio=commanded_ratio(t_raw + offset, raw, t),
        raw_moving=np.interp(t, t_raw + offset, raw_speed)
        > RAW_REST_SPEED_MM_S,
    )


@dataclass
class FlowTrace:
    s: np.ndarray
    middle: np.ndarray
    deviation: np.ndarray


def flow_trace(
    t: np.ndarray,
    xy: np.ndarray,
    e_cmd: np.ndarray,
    commanded: CommandedBead,
    melt_k: float,
    nominal_ratio: float,
) -> FlowTrace:
    """Nozzle flow per mm of the path `xy` traces against the commanded
    bead, over the nominal bead, on a PATH_STEP_MM grid."""
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
    deviation = (np.diff(e_grid) / PATH_STEP_MM - r_grid) / nominal_ratio
    return FlowTrace(s=s, middle=middle, deviation=deviation)


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
            * 1000.0
        ),
    }
