# klippy/blendmath.py
# Corner-blending geometry module.
#
# Pure-math primitives: given two adjacent linear moves and a
# chord-tolerance parameter, returns a G1 tangent circular arc that
# smooths the corner, along with the maximum velocity it may be
# traversed at and a fine-segmented polyline approximation.
#
# See docs/superpowers/specs/2026-04-16-blend-geometry-module-design.md
#
# Copyright (C) 2026
# This file may be distributed under the terms of the GNU GPLv3 license.
from __future__ import annotations

import math
from dataclasses import dataclass, replace
from typing import Optional, Tuple

from klippy import blendshaper

Vec3 = Tuple[float, float, float]

COLLINEAR_EPS = 1e-6
REVERSAL_EPS = 1e-6


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


@dataclass(frozen=True)
class BlendArc:
    """Tangent-arc blend geometry between two adjacent moves.

    Coordinates: ``entry_pt``, ``exit_pt``, and ``center`` are in a
    corner-local frame where the corner vertex is at the origin. Callers
    must translate by the vertex position to obtain world coordinates.
    ``entry_tangent``, ``exit_tangent``, and ``plane_normal`` are
    direction vectors and frame-independent.

    For degenerate corners (R = 0, returned for U-turns), entry_pt /
    exit_pt / center are all (0, 0, 0) and plane_normal is (0, 0, 0).
    """

    R: float
    theta: float
    d_consumed: float
    v_cap: float
    center: Vec3
    entry_pt: Vec3
    exit_pt: Vec3
    entry_tangent: Vec3
    exit_tangent: Vec3
    plane_normal: Vec3


def blend_geometry(
    prev_dir: Vec3,
    next_dir: Vec3,
    L_prev: float,
    L_next: float,
    corner_deviation: float,
    a_max: float,
    j_eff: float,
) -> Optional[BlendArc]:
    """Compute the tangent-arc blend for a corner, or None if no blend needed.

    ``prev_dir`` and ``next_dir`` must be unit vectors.
    """
    # Deflection angle theta: 0 = collinear, pi = U-turn.
    # With head-to-tail unit directions:
    #   cos(theta) = prev_dir . next_dir
    #   cos(theta/2) = sqrt((1 + prev_dir.next_dir) / 2)
    #   sin(theta/2) = sqrt((1 - prev_dir.next_dir) / 2)
    dp = vdot(prev_dir, next_dir)
    # Clamp for numerical safety; dp should lie in [-1, 1] for unit vectors.
    dp = max(-1.0, min(1.0, dp))
    cos_half = math.sqrt(max(0.0, (1.0 + dp) * 0.5))
    sin_half = math.sqrt(max(0.0, (1.0 - dp) * 0.5))

    if sin_half < COLLINEAR_EPS:
        # Collinear: no blend required.
        return None

    if cos_half < REVERSAL_EPS:
        # U-turn: no tangent arc exists. Caller must stop at the junction.
        return BlendArc(
            R=0.0,
            theta=math.pi,
            d_consumed=0.0,
            v_cap=0.0,
            center=(0.0, 0.0, 0.0),
            entry_pt=(0.0, 0.0, 0.0),
            exit_pt=(0.0, 0.0, 0.0),
            entry_tangent=prev_dir,
            exit_tangent=next_dir,
            plane_normal=(0.0, 0.0, 0.0),
        )

    # Deflection angle (rad).
    theta = 2.0 * math.atan2(sin_half, cos_half)

    # Tolerance-driven radius:
    R_tol = corner_deviation * cos_half / (1.0 - cos_half)

    # Midpoint / adjacent-segment cap (half-segment rule): each blend claims
    # at most half the adjacent segment so two neighbouring corners meet at
    # most at the segment midpoint. cot(theta/2) = cos_half / sin_half.
    # d_consumed = R * tan(theta/2) <= 0.5 * min(L_prev, L_next) by construction.
    R_mid = 0.5 * min(L_prev, L_next) * cos_half / sin_half

    R = min(R_tol, R_mid)

    # Centripetal cap: v² <= a_max · R. During steady-state arc cruise
    # tangential accel is identically zero (constant v_cent), so the
    # full acceleration budget is available for centripetal. LinuxCNC's
    # original Pythagorean split reserved 50% tangential (a_n ≤ √3/2 ·
    # a_max) to handle simultaneous decel-and-turn in one segment; our
    # pipeline instead hands decel/accel to the truncated-linear
    # segments on either side of the arc, so reserving that headroom
    # inside the arc is unused.
    v_centripetal = math.sqrt(a_max * R) if R > 0.0 else 0.0

    # Jerk floor: R >= v^(3/2) / sqrt(j_eff)  =>  v <= (R * sqrt(j_eff))^(2/3)
    if R > 0.0 and j_eff > 0.0:
        v_jerk = (R * math.sqrt(j_eff)) ** (2.0 / 3.0)
    else:
        v_jerk = 0.0

    v_cap = min(v_centripetal, v_jerk)

    # Tangent points on the adjacent rays. prev_dir points *toward* the
    # vertex, so the entry tangent point sits at -d * prev_dir (upstream).
    # next_dir points *away from* the vertex, so exit sits at +d * next_dir.
    d = R * sin_half / cos_half
    entry_pt = vscale(prev_dir, -d)
    exit_pt = vscale(next_dir, d)

    # Plane normal (ambiguous sign for collinear / reversal; safe here since
    # those cases already returned). Choose prev x next for consistent
    # right-handed orientation.
    raw_normal = vcross(prev_dir, next_dir)
    raw_norm_n = vnorm(raw_normal)
    if raw_norm_n == 0.0:
        plane_normal: Vec3 = (0.0, 0.0, 0.0)
    else:
        plane_normal = vscale(raw_normal, 1.0 / raw_norm_n)

    # Arc center: perpendicular to prev_dir at entry_pt, offset by R toward
    # the interior of the corner. The interior direction is
    # normalize(next_dir - prev_dir * cos_theta) -- but it's simpler to
    # compute via the inward perpendicular n_prev = plane_normal x prev_dir
    # (with the sign chosen so that stepping from entry_pt by +R*n_prev
    # lands on the arc center).
    n_prev = vcross(plane_normal, prev_dir)
    # Choose sign so n_prev points from entry_pt toward the corner interior.
    # The interior is on the next_dir side; dot with next_dir should be >= 0.
    if vdot(n_prev, next_dir) < 0.0:
        n_prev = vscale(n_prev, -1.0)
    center = vadd(entry_pt, vscale(n_prev, R))

    return BlendArc(
        R=R,
        theta=theta,
        d_consumed=d,
        v_cap=v_cap,
        center=center,
        entry_pt=entry_pt,
        exit_pt=exit_pt,
        entry_tangent=prev_dir,
        exit_tangent=next_dir,
        plane_normal=plane_normal,
    )


def segment_arc(arc: BlendArc, max_chord_err: float = 1e-2) -> list:
    """Return a polyline approximating the arc, with chord error <= max_chord_err."""
    if max_chord_err <= 0.0:
        raise ValueError("max_chord_err must be positive")
    if arc.R <= 0.0:
        # Degenerate: single point at the (coincident) entry.
        return [arc.entry_pt]

    # Step angle such that chord deviation from the arc is <= max_chord_err.
    # chord error e = R * (1 - cos(dphi/2))  =>  dphi = 2 * acos(1 - e/R).
    e_over_r = max_chord_err / arc.R
    if e_over_r >= 1.0:
        # Absurd tolerance: one segment is enough.
        return [arc.entry_pt, arc.exit_pt]
    dphi_max = 2.0 * math.acos(1.0 - e_over_r)

    num_segments = max(1, math.ceil(arc.theta / dphi_max))
    dphi = arc.theta / num_segments

    # Direction of rotation: from (entry_pt - center) toward (exit_pt - center).
    # Rodrigues' rotation around arc.plane_normal by angle phi, applied to
    # the radial vector from center.
    r0 = vsub(arc.entry_pt, arc.center)
    axis = arc.plane_normal

    points: list = [arc.entry_pt]
    for i in range(1, num_segments):
        phi = dphi * i
        r = _rotate(r0, axis, phi)
        points.append(vadd(arc.center, r))
    points.append(arc.exit_pt)
    return points


def _rotate(v: Vec3, axis: Vec3, angle: float) -> Vec3:
    """Rotate vector v around unit axis by angle (radians). Rodrigues."""
    c = math.cos(angle)
    s = math.sin(angle)
    ax_dot_v = vdot(axis, v)
    ax_cross_v = vcross(axis, v)
    return (
        v[0] * c + ax_cross_v[0] * s + axis[0] * ax_dot_v * (1.0 - c),
        v[1] * c + ax_cross_v[1] * s + axis[1] * ax_dot_v * (1.0 - c),
        v[2] * c + ax_cross_v[2] * s + axis[2] * ax_dot_v * (1.0 - c),
    )


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
    target = getattr(is_obj, "target_smoothing", None)
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


def blend_from_moves(
    prev_move,
    next_move,
    corner_deviation: float,
    j_eff: float = float("inf"),
    toolhead=None,
) -> Optional[BlendArc]:
    """Adapter: compute a blend arc from a pair of Kalico Move-like objects.

    Skips the blend if either move is non-kinematic (E-only). The
    effective a_max is the stricter of the two moves' accel values.

    If `toolhead` is given, derives `j_eff` and an additional per-axis
    entry-step velocity cap from the toolhead's input shaper module.
    In that case any explicit `j_eff` argument is ignored.

    If `toolhead` is None (default), `blend_geometry` is called once
    with the given `j_eff` (default +inf) — preserves the pre-shaper
    behavior used by existing tests.
    """
    if toolhead is not None and j_eff != float("inf"):
        raise ValueError(
            "blend_from_moves: j_eff and toolhead are mutually exclusive "
            "(toolhead derives j_eff from shaper state; passing both is ambiguous)"
        )
    if not getattr(prev_move, "is_kinematic_move", True):
        return None
    if not getattr(next_move, "is_kinematic_move", True):
        return None

    prev_dir: Vec3 = (
        prev_move.axes_r[0],
        prev_move.axes_r[1],
        prev_move.axes_r[2],
    )
    next_dir: Vec3 = (
        next_move.axes_r[0],
        next_move.axes_r[1],
        next_move.axes_r[2],
    )
    a_max = min(prev_move.accel, next_move.accel)

    if toolhead is None:
        return blend_geometry(
            prev_dir=prev_dir, next_dir=next_dir,
            L_prev=prev_move.move_d, L_next=next_move.move_d,
            corner_deviation=corner_deviation,
            a_max=a_max, j_eff=j_eff,
        )

    shapers = _extract_shapers(toolhead)

    # First pass: no jerk constraint — we need R, entry_pt, center,
    # plane_normal to compute per-axis bounds.
    arc_0 = blend_geometry(
        prev_dir=prev_dir, next_dir=next_dir,
        L_prev=prev_move.move_d, L_next=next_move.move_d,
        corner_deviation=corner_deviation,
        a_max=a_max, j_eff=float("inf"),
    )
    if arc_0 is None or arc_0.R == 0.0 or not shapers:
        return arc_0

    n_hat = vnormalize(vsub(arc_0.center, arc_0.entry_pt))
    bounds = blendshaper.compute_shaper_bounds(
        shapers=shapers,
        R=arc_0.R,
        n_hat=n_hat,
        p_hat=arc_0.plane_normal,
    )

    # Second pass: with the derived j_eff.
    arc = blend_geometry(
        prev_dir=prev_dir, next_dir=next_dir,
        L_prev=prev_move.move_d, L_next=next_move.move_d,
        corner_deviation=corner_deviation,
        a_max=a_max, j_eff=bounds.j_eff,
    )
    if arc is None or arc.R == 0.0:
        return arc

    # Re-evaluate Bound (b) against the final R / n_hat (Bound (b) is
    # mildly R-dependent; second evaluation is near-free and keeps the
    # bound honest on corners where the second-pass R differs from R_0).
    n_hat_final = vnormalize(vsub(arc.center, arc.entry_pt))
    bounds_final = blendshaper.compute_shaper_bounds(
        shapers=shapers,
        R=arc.R,
        n_hat=n_hat_final,
        p_hat=arc.plane_normal,
    )
    v_cap = min(arc.v_cap, bounds_final.v_step_cap)
    # BlendArc is frozen; return a copy with the capped v_cap.
    return replace(arc, v_cap=v_cap)


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
