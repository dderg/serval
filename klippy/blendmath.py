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

# Sine-of-half-angle below which we treat a junction as collinear.
# Matches blendquintic.COLLINEAR_EPS; kept local to avoid an import.
COLLINEAR_EPS = 1e-6


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
        # Smooth-family axes don't carry shaper_freq/shaper_type; their
        # impulse-spread sigma_T is zero by construction, so skip them.
        freq = float(getattr(params, "shaper_freq", 0.0) or 0.0)
        stype = getattr(params, "shaper_type", "") or ""
        damp = float(getattr(params, "damping_ratio", 0.0) or 0.0)
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


def _scv_equivalent_junction_v(
    cos_half: float,
    sin_half: float,
    corner_deviation: float,
    sigma_T_max: float,
    a_max: float,
) -> float:
    """Klipper junction-deviation velocity cap equivalent to mainline SCV
    at a shaper with RMS impulse spread sigma_T_max, evaluated at a corner
    with the given half-angle geometry.

    Derivation:
      - SCV-equivalent at 90° matching corner_deviation under shaper smear:
            v_scv90 = cd / (sqrt(2) * sigma_T)
      - Klipper's JD formula (jd = SCV^2 * (sqrt(2) - 1) / a_max):
            jd_eq = v_scv90^2 * (sqrt(2) - 1) / a_max
      - Per-corner radius and velocity:
            R_scv = jd_eq * cos(theta/2) / (1 - cos(theta/2))
            v_j   = sqrt(R_scv * a_max)

    Returns +inf for collinear (no cap needed) or when any input is
    non-positive (no cap derivable).
    """
    one_minus_cos = 1.0 - cos_half
    if sin_half <= COLLINEAR_EPS or one_minus_cos <= 1e-12:
        return float("inf")
    if sigma_T_max <= 0.0 or corner_deviation <= 0.0 or a_max <= 0.0:
        return float("inf")
    v_scv90 = corner_deviation / (math.sqrt(2.0) * sigma_T_max)
    jd_eq = v_scv90 * v_scv90 * (math.sqrt(2.0) - 1.0) / a_max
    R_scv = jd_eq * cos_half / one_minus_cos
    return math.sqrt(R_scv * a_max)


def suppressed_junction_v(
    prev_move,
    next_move,
    corner_deviation: float,
    toolhead,
) -> Optional[float]:
    """SCV-equivalent junction velocity to apply when blend_from_moves
    returns None at a non-collinear corner.

    Companion to blend_from_moves: when the arc is suppressed (either by
    the shaper-aware or velocity-aware rule), the fork's calc_junction
    has no built-in JD cap — so without this cap the toolhead would hit
    sharp corners at full commanded velocity, causing step skipping.

    Returns:
        None  — truly collinear junction (no cap needed), or no shaper
                 loaded (no cap derivable; mainline-Kalico calc_junction
                 quarter-tan cap still applies as a lax safety net).
        float — velocity cap to pass to prev.limit_next_junction_speed().
    """
    if toolhead is None:
        return None
    prev_dir: Vec3 = (
        prev_move.axes_r[0], prev_move.axes_r[1], prev_move.axes_r[2],
    )
    next_dir: Vec3 = (
        next_move.axes_r[0], next_move.axes_r[1], next_move.axes_r[2],
    )
    dp = max(-1.0, min(1.0, vdot(prev_dir, next_dir)))
    cos_half = math.sqrt(max(0.0, (1.0 + dp) * 0.5))
    sin_half = math.sqrt(max(0.0, (1.0 - dp) * 0.5))
    if sin_half < COLLINEAR_EPS:
        return None
    sigma_T = _sigma_T_max_from_toolhead(toolhead)
    if sigma_T <= 0.0:
        return None
    a_max = min(prev_move.accel, next_move.accel)
    v_j = _scv_equivalent_junction_v(
        cos_half, sin_half, corner_deviation, sigma_T, a_max,
    )
    if not math.isfinite(v_j):
        return None
    return v_j


def should_suppress_quintic(
    prev_move,
    next_move,
    corner_deviation: float,
    shape,
    toolhead,
) -> bool:
    """Decide whether to skip the quintic blend and run sharp-V under
    the input shaper instead. Two-clause rule:
      1. Shaper-smeared sharp-V deviation <= corner_deviation (path tolerance).
      2. Sharp-V recovery time (2·v·sin_half/a_max) <= blend traversal time
         at v_cap_fn(arc_length/2) (time).
    Suppress only when BOTH hold.

    Returns True  => drop the blend; caller runs sharp-V at a
                     suppressed_junction_v cap.
    Returns False => keep the blend (either tolerance or time fails).

    Derivation: docs/superpowers/plans/plan4-derivations/quintic_suppression.md.
    """
    # Defensive: no shape — nothing to suppress or keep.
    if shape is None:
        return True
    # Extruder-only moves have no XYZ geometry; nothing to blend.
    if (prev_move.axes_d[0] == 0.0 and prev_move.axes_d[1] == 0.0
            and prev_move.axes_d[2] == 0.0):
        return True
    if (next_move.axes_d[0] == 0.0 and next_move.axes_d[1] == 0.0
            and next_move.axes_d[2] == 0.0):
        return True
    theta = getattr(shape, "theta", 0.0)
    sin_half = math.sin(0.5 * theta)
    if sin_half < COLLINEAR_EPS:
        return True
    sigma_T = _sigma_T_max_from_toolhead(toolhead)
    if sigma_T <= 0.0:
        # No impulse shaper loaded -> no equivalent sharp-V claim; keep
        # the blend. (Matches suppressed_junction_v's None branch.)
        return False
    v_prev = math.sqrt(max(0.0, prev_move.max_cruise_v2))
    v_next = math.sqrt(max(0.0, next_move.max_cruise_v2))
    v = min(v_prev, v_next)
    if v <= 0.0:
        return True                                       # v -> 0 sanity
    a_max = min(prev_move.accel, next_move.accel)
    if a_max <= 0.0:
        return False
    # Clause 1: path tolerance.
    dev_sharpV = 2.0 * v * sin_half * sigma_T
    if dev_sharpV > corner_deviation:
        return False
    # Clause 2: time.
    t_sharpV = 2.0 * v * sin_half / a_max
    arc_len = getattr(shape, "arc_length", 0.0)
    if arc_len <= 0.0:
        return True                                       # degenerate
    v_mid = shape.v_cap_fn(0.5 * arc_len)
    if not math.isfinite(v_mid) or v_mid <= 0.0:
        return False
    t_blend = arc_len / v_mid
    return t_sharpV <= t_blend


def _compute_A_axis_smooth_is(shaper_type: str, shaper_freq: float,
                              damping_ratio: float,
                              target_smoothing: float = 0.12) -> float:
    """A_axis for a Smooth-IS kernel, in the same units as FIR A_axis.

    Closed-form: A_axis = 2 * target_smoothing / sigma_T^2, where
    sigma_T^2 is the second central moment of the kernel's
    compactly-supported polynomial w(tau). Delegated to
    ShaperCalibrate.find_smoother_max_accel.

    damping_ratio is accepted for signature parity with the FIR path
    but has no effect — SIS kernels are fixed-shape.

    Returns 0.0 for unknown shaper_type or shaper_freq <= 0 (the
    _extract_shapers sentinel for 'no shaper contribution').

    Derivation: docs/superpowers/plans/plan4-derivations/A_axis_smooth_is.md.
    """
    from klippy.extras import shaper_defs
    from klippy.extras.shaper_calibrate import ShaperCalibrate

    if shaper_freq <= 0.0:
        return 0.0
    factory = {s.name: s.init_func for s in shaper_defs.INPUT_SMOOTHERS}
    if shaper_type not in factory:
        return 0.0
    smoother = factory[shaper_type](shaper_freq, damping_ratio)
    sc = ShaperCalibrate(printer=None, target_smoothing=target_smoothing)
    return float(sc.find_smoother_max_accel(smoother, target_smoothing))


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
        # After the BE-v2 smooth-shapers port, axes can carry either
        # TypedInputShaperParams (impulse) or TypedInputSmootherParams
        # (smooth). Both families now contribute an analytical A_axis to
        # the shaper-derived velocity cap.
        #
        # TypedInputShaperParams uses shaper_type / shaper_freq.
        # TypedInputSmootherParams uses smoother_type / smoother_freq.
        # Try the shaper_* names first (FIR); fall back to smoother_* (SIS).
        shaper_type = (getattr(params, "shaper_type", None)
                       or getattr(params, "smoother_type", "")
                       or "")
        freq_raw = (getattr(params, "shaper_freq", None)
                    if getattr(params, "shaper_type", None) is not None
                    else getattr(params, "smoother_freq", 0.0))
        freq = float(freq_raw or 0.0)
        damping_ratio = float(getattr(params, "damping_ratio", 0.0) or 0.0)
        if freq <= 0.0 or not shaper_type:
            A_axis = 0.0
        elif shaper_type in shaper_factory:
            # FIR: use ShaperCalibrate.find_shaper_max_accel.
            impulses = shaper_factory[shaper_type](freq, damping_ratio)
            A_axis = float(sc.find_shaper_max_accel(impulses))
        elif shaper_type in {s.name for s in shaper_defs.INPUT_SMOOTHERS}:
            # Cardinal B-spline chain: analytical A_axis from the kernel.
            A_axis = _compute_A_axis_smooth_is(shaper_type, freq, damping_ratio,
                                               target_smoothing=target or 0.12)
        else:
            A_axis = 0.0
        # Plan 5 Pillar 1 D4: carry ‖h_axis‖₁ for the saturation cap.
        # Prefer AxisInputSmoother.G_axis (populated by
        # recompute_fused_kernel); fall back to input_shaper.get_axis_G
        # for axes that don't live on a smoother object. Both paths
        # return 1.0 when no feedforward inverse is wired.
        axis_char = axis_shaper.get_axis()
        inverse_G = float(getattr(axis_shaper, "G_axis", 1.0) or 1.0)
        if inverse_G == 1.0:
            get_axis_G = getattr(is_obj, "get_axis_G", None)
            if callable(get_axis_G):
                inverse_G = float(get_axis_G(axis_char) or 1.0)
        snaps.append(blendshaper.AxisShaperSnapshot(
            axis=axis_char,
            shaper_type=shaper_type,
            shaper_freq=freq,
            damping_ratio=damping_ratio,
            A_axis=A_axis,
            inverse_G=inverse_G,
        ))
    return snaps




def interpolate_extruder(
    polyline,
    d_consumed: float,
    e_per_mm_prev: float,
    e_per_mm_next: float,
) -> list:
    """Attach an E coordinate to each polyline point.

    The blend curve replaces the final `d_consumed` mm of the previous move and
    the first `d_consumed` mm of the next move. Total E through the blend is
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
