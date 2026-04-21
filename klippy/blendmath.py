# klippy/blendmath.py
# Shared math utilities for corner blending and shaper analysis.
#
# Provides vector operations and shaper-related utilities used by
# the motion planner (blendplanner.py) and shape implementations
# (blendquintic.py, etc.).
#
# Copyright (C) 2026
# This file may be distributed under the terms of the GNU GPLv3 license.
from __future__ import annotations

import math
from typing import Optional, Tuple

from klippy import blendshaper

Vec3 = Tuple[float, float, float]


def vdot(a: Vec3, b: Vec3) -> float:
    """Scalar dot product of two 3-vectors."""
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def vcross(a: Vec3, b: Vec3) -> Vec3:
    """Cross product of two 3-vectors."""
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def vnorm(a: Vec3) -> float:
    """Euclidean length of a 3-vector."""
    return math.sqrt(vdot(a, a))


def vscale(a: Vec3, s: float) -> Vec3:
    """Multiply a 3-vector by a scalar."""
    return (a[0] * s, a[1] * s, a[2] * s)


def vadd(a: Vec3, b: Vec3) -> Vec3:
    """Component-wise sum of two 3-vectors."""
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def vsub(a: Vec3, b: Vec3) -> Vec3:
    """Component-wise difference of two 3-vectors (a - b)."""
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def vnormalize(a: Vec3) -> Vec3:
    """Return a unit vector in the direction of a; raise ValueError if zero."""
    n = vnorm(a)
    if n == 0.0:
        raise ValueError("cannot normalize zero vector")
    return vscale(a, 1.0 / n)


def _sigma_T_max_from_toolhead(toolhead):
    """Return the max RMS spread (seconds) across axis input shaper impulse
    patterns, or 0.0 if no shaper is loaded or all axes are unshaped.

    sigma_T is a property of the shaper impulse sequence alone — independent
    of target_smoothing — so this is safe to use for suppression decisions
    regardless of the target_smoothing=0 sentinel behaviour in
    _extract_shapers().
    """
    if toolhead is None:
        return 0.0
    printer = getattr(toolhead, "printer", None)
    if printer is None:
        return 0.0
    is_obj = printer.lookup_object("input_shaper", None)
    if is_obj is None:
        return 0.0
    from klippy.extras import shaper_defs as _shaper_defs
    factory = {s.name: s.init_func for s in _shaper_defs.INPUT_SHAPERS}
    sigma_max = 0.0
    for axis_shaper in is_obj.get_shapers():
        params = axis_shaper.params
        freq = float(params.shaper_freq)
        stype = params.shaper_type
        damp = float(params.damping_ratio)
        if freq > 0.0 and stype in factory:
            A, T = factory[stype](freq, damp)
            w_sum = sum(A)
            if w_sum > 0.0:
                t_bar = sum(a * t for a, t in zip(A, T)) / w_sum
                var = sum(
                    a * (t - t_bar) ** 2 for a, t in zip(A, T)
                ) / w_sum
                sigma = math.sqrt(max(0.0, var))
                if sigma > sigma_max:
                    sigma_max = sigma
    return sigma_max




def _extract_shapers(toolhead):
    """Pull per-axis shaper snapshots off a Kalico toolhead.

    Returns an empty list if `toolhead` is None or no `input_shaper`
    module is loaded. Unshaped axes (shaper_freq == 0) are included
    with A_axis = 0 so the caller sees them and can still reason
    about missing axes.
    """
    if toolhead is None:
        return []
    printer = getattr(toolhead, "printer", None)
    if printer is None:
        return []
    is_obj = printer.lookup_object("input_shaper", None)
    if is_obj is None:
        return []

    # Lazy-import ShaperCalibrate to avoid a hard dependency when
    # blendmath is imported in a non-Kalico context (e.g. pytest).
    from klippy.extras.shaper_calibrate import ShaperCalibrate
    from klippy.extras import shaper_defs

    # Pick up user-configured target_smoothing if the input_shaper
    # object carries one — lets operators trade quality vs corner
    # speed via the [input_shaper] target_smoothing config. A mock
    # or older object without the attribute falls back to the
    # ShaperCalibrate default (0.12 mm).
    #
    # Sentinel: target_smoothing == 0 disables the shaper-derived
    # velocity cap entirely (returns [] so compute_shaper_bounds
    # produces inf bounds). Use SET_INPUT_SHAPER TARGET_SMOOTHING=0
    # to isolate arc-planner cost from residual shaper-cap cost.
    target = getattr(is_obj, "target_smoothing", None)
    if target is not None and target <= 0.0:
        return []
    sc = ShaperCalibrate(printer=None, target_smoothing=target)
    shaper_factory = {s.name: s.init_func for s in shaper_defs.INPUT_SHAPERS}

    snaps = []
    for axis_shaper in is_obj.get_shapers():
        params = axis_shaper.params
        freq = float(params.shaper_freq)
        shaper_type = params.shaper_type
        damping_ratio = float(params.damping_ratio)
        if freq > 0.0 and shaper_type in shaper_factory:
            impulses = shaper_factory[shaper_type](freq, damping_ratio)
            A_axis = float(sc.find_shaper_max_accel(impulses))
        else:
            A_axis = 0.0
        snaps.append(blendshaper.AxisShaperSnapshot(
            axis=axis_shaper.axis,
            shaper_type=shaper_type,
            shaper_freq=freq,
            damping_ratio=damping_ratio,
            A_axis=A_axis,
        ))
    return snaps




def interpolate_extruder(
    polyline,
    d_consumed: float,
    e_per_mm_prev: float,
    e_per_mm_next: float,
) -> list:
    """Attach an E coordinate to each polyline point.

    The blend arc replaces the final `d_consumed` mm of the previous move and
    the first `d_consumed` mm of the next move. Total E through the arc is
    conserved: sum across the polyline equals
    `d_consumed * (e_per_mm_prev + e_per_mm_next)`. E is distributed uniformly
    over the polyline's arc-length parameterization.
    """
    if not polyline:
        return []

    total_e = d_consumed * (e_per_mm_prev + e_per_mm_next)

    # Arc length along the polyline (piecewise-linear approximation).
    seg_lens = []
    total_len = 0.0
    for p0, p1 in zip(polyline, polyline[1:]):
        seg_len = vnorm(vsub(p1, p0))
        seg_lens.append(seg_len)
        total_len += seg_len

    if total_len == 0.0:
        # Degenerate polyline (single point or collapsed).
        return [(p[0], p[1], p[2], 0.0) for p in polyline]

    out = [(polyline[0][0], polyline[0][1], polyline[0][2], 0.0)]
    e = 0.0
    for seg_len, p1 in zip(seg_lens, polyline[1:]):
        e += total_e * seg_len / total_len
        out.append((p1[0], p1[1], p1[2], e))
    return out
