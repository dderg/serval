# klippy/blendquintic.py
# Quintic Bezier corner-blending geometry module.
#
# Pure-math primitives: given two adjacent linear moves and a
# chord-tolerance parameter, returns a G2 symmetric 6-point quintic
# Bezier blend that smooths the corner, along with the maximum velocity
# it may be traversed at and a fine-segmented polyline approximation.
#
# Analogous to klippy/blendmath.py (which handles G1 arcs).
#
# See docs/superpowers/specs/2026-04-19-subspec-6d-quintic-hermite-design.md
#
# Copyright (C) 2026
# This file may be distributed under the terms of the GNU GPLv3 license.
from __future__ import annotations

import math
from dataclasses import dataclass, replace
from typing import List, Optional, Tuple

from klippy import blendshaper
from klippy.blendmath import (
    _extract_shapers,
    vdot,
    vcross,
    vnorm,
    vscale,
    vsub,
    vnormalize,
)

Vec3 = Tuple[float, float, float]

COLLINEAR_EPS = 1e-6
REVERSAL_EPS = 1e-6


def _lerp(a: Vec3, b: Vec3, t: float) -> Vec3:
    """Linear interpolation between two 3-vectors."""
    return (
        a[0] + (b[0] - a[0]) * t,
        a[1] + (b[1] - a[1]) * t,
        a[2] + (b[2] - a[2]) * t,
    )


def _quintic_eval(Q, t: float) -> Vec3:
    """Evaluate a quintic Bezier at parameter t via De Casteljau.

    Q is an indexable of 6 control points Q0..Q5. Returns the
    position on the curve at parameter t in [0, 1].
    """
    p = [Q[i] for i in range(6)]
    for level in range(5, 0, -1):
        for i in range(level):
            p[i] = _lerp(p[i], p[i + 1], t)
    return p[0]


def _bezier_eval_general(P, t: float) -> Vec3:
    """De Casteljau for a Bezier curve of any degree. P is a list of
    n+1 control points (tuples). Returns the point at parameter t."""
    p = [P[i] for i in range(len(P))]
    level = len(p) - 1
    while level > 0:
        for i in range(level):
            p[i] = _lerp(p[i], p[i + 1], t)
        level -= 1
    return p[0]


def _quintic_first_deriv(Q, t: float) -> Vec3:
    """Evaluate B'(t) for a quintic Bezier at parameter t.

    The derivative of a degree-5 Bezier with control points Q0..Q5
    is a degree-4 Bezier with control points 5*(Q[i+1] - Q[i]).
    """
    D = [
        (
            5.0 * (Q[i + 1][0] - Q[i][0]),
            5.0 * (Q[i + 1][1] - Q[i][1]),
            5.0 * (Q[i + 1][2] - Q[i][2]),
        )
        for i in range(5)
    ]
    return _bezier_eval_general(D, t)


def _quintic_second_deriv(Q, t: float) -> Vec3:
    """Evaluate B''(t) for a quintic Bezier at parameter t.

    The second derivative is a degree-3 Bezier with control points
    20 * (Q[i+2] - 2*Q[i+1] + Q[i]).
    """
    DD = [
        (
            20.0 * (Q[i + 2][0] - 2.0 * Q[i + 1][0] + Q[i][0]),
            20.0 * (Q[i + 2][1] - 2.0 * Q[i + 1][1] + Q[i][1]),
            20.0 * (Q[i + 2][2] - 2.0 * Q[i + 1][2] + Q[i][2]),
        )
        for i in range(4)
    ]
    return _bezier_eval_general(DD, t)


def _deviation_coeff(r: float) -> float:
    """The chord-deviation prefactor (1 + 15*r)/16."""
    return (1.0 + 15.0 * r) / 16.0


def _deviation_closed_form(d: float, r: float, sin_half: float) -> float:
    """Chord deviation of a symmetric quintic blend, closed form.

    At the midpoint t=0.5:
        |B(0.5) - V| = ((1 + 15*r) / 16) * d * sin(theta/2)
    """
    return _deviation_coeff(r) * d * sin_half


def _d_from_deviation(eps: float, r: float, sin_half: float) -> float:
    """Inverse: tangent length d required to achieve chord deviation eps.

    d = 16 * eps / ((1 + 15*r) * sin(theta/2))
    """
    denom = (1.0 + 15.0 * r) * sin_half
    if denom <= 0.0:
        raise ValueError("_d_from_deviation: non-positive denominator")
    return 16.0 * eps / denom


def _curvature_at(Q, t: float) -> float:
    """Curvature of the quintic at parameter t.

    kappa(t) = |B'(t) x B''(t)| / |B'(t)|^3
    Returns 0.0 if |B'(t)| is near zero (degenerate endpoint with
    coincident control points — expected at t=0 and t=1 for symmetric
    quintic blends).
    """
    d1 = _quintic_first_deriv(Q, t)
    d2 = _quintic_second_deriv(Q, t)
    d1_norm = vnorm(d1)
    if d1_norm < 1e-12:
        return 0.0
    cx = d1[1] * d2[2] - d1[2] * d2[1]
    cy = d1[2] * d2[0] - d1[0] * d2[2]
    cz = d1[0] * d2[1] - d1[1] * d2[0]
    cross_norm = math.sqrt(cx * cx + cy * cy + cz * cz)
    return cross_norm / (d1_norm ** 3)


_PEAK_KAPPA_SAMPLES = 100  # dense-sample count for peak-curvature search


def _peak_curvature(Q) -> Tuple[float, float]:
    """Return (kappa_max, t_peak) along the quintic.

    Dense sampling at _PEAK_KAPPA_SAMPLES points; returns the maximum
    curvature along the blend and the parameter value where it occurs.
    Endpoints always have kappa = 0 for a symmetric blend, so they are
    included but will not normally win.

    For a fully-degenerate curve (all samples give kappa = 0) the
    returned t_peak is 0.5 as a neutral-midpoint sentinel. Callers
    must gate on kappa_peak > 0.0 before using t_peak.
    """
    best_k = 0.0
    best_t = 0.5
    for i in range(_PEAK_KAPPA_SAMPLES):
        t = i / (_PEAK_KAPPA_SAMPLES - 1)
        k = _curvature_at(Q, t)
        if k > best_k:
            best_k = k
            best_t = t
    return best_k, best_t


# Quadratic fit coefficients for the minimum-traversal-time shape
# parameter r as a function of the deflection angle theta (radians).
# Derived from the 151-angle x 3-deviation subagent sweep dated
# 2026-04-19; see subspec-6d design spec section "Shape parameter
# r(theta)". Worst-case traversal-time penalty vs the per-angle
# optimum is 0.21% at theta ~ 10 deg (near the validity edge).
_R_FIT_C0 = 0.5085
_R_FIT_C1 = -0.03785
_R_FIT_C2 = 0.05715

_R_CLAMP_MIN = 0.50
_R_CLAMP_MAX = 0.86


def _shape_ratio(theta: float) -> float:
    """Shape parameter r for the quintic blend at deflection theta.

    Quadratic fit (radians) clamped to the [0.50, 0.86] safety range.
    Returns the r value used to place the inner control-point pair at
    +/- r*d along each tangent ray.
    """
    r = _R_FIT_C0 + _R_FIT_C1 * theta + _R_FIT_C2 * theta * theta
    if r < _R_CLAMP_MIN:
        return _R_CLAMP_MIN
    if r > _R_CLAMP_MAX:
        return _R_CLAMP_MAX
    return r


@dataclass(frozen=True)
class QuinticBlend:
    """Symmetric quintic Bezier blend between two adjacent moves.

    Coordinates: ``Q`` control points and ``entry_pt``/``exit_pt``
    (derivable as ``Q[0]`` and ``Q[5]``) are in a corner-local frame
    where the corner vertex is at the origin. Callers must translate
    by the vertex position to obtain world coordinates.

    For degenerate corners (returned for U-turns), ``kappa_peak`` is
    0.0, ``v_cap`` is 0.0, and ``Q`` is six copies of (0, 0, 0).

    Fields:
        Q:              6 control points (Q0..Q5)
        theta:          deflection angle (rad), 0 = straight, pi = U-turn
        r:              shape parameter used (in [0.50, 0.86])
        d_consumed:     tangent length along each ray (mm)
        kappa_peak:     maximum curvature along the blend (1/mm)
        t_peak:         parameter value in [0, 1] at which kappa_peak
                        occurs; off-center for the r values used in
                        practice. 0.5 when the blend is degenerate.
        v_cap:          maximum traversal velocity (mm/s)
        entry_tangent:  unit vector, same as prev_dir into corner
        exit_tangent:   unit vector, same as next_dir out of corner
        plane_normal:   unit vector orthogonal to the blend plane, or
                        (0, 0, 0) for degenerate (U-turn / collinear)
                        corners where no single plane is defined
    """
    Q: Tuple[Vec3, Vec3, Vec3, Vec3, Vec3, Vec3]
    theta: float
    r: float
    d_consumed: float
    kappa_peak: float
    t_peak: float
    v_cap: float
    entry_tangent: Vec3
    exit_tangent: Vec3
    plane_normal: Vec3


def quintic_geometry(
    prev_dir: Vec3,
    next_dir: Vec3,
    L_prev: float,
    L_next: float,
    corner_deviation: float,
    a_max: float,
) -> Optional[QuinticBlend]:
    """Compute the quintic Bezier blend for a corner, or None if no
    blend is needed. Centripetal bound only — shaper and rotation-jerk
    bounds are added by `quintic_geometry_with_shaper` / `blend_from_
    moves_quintic`.

    ``prev_dir`` and ``next_dir`` must be unit vectors. Angle
    convention: deflection theta, where 0 = collinear, pi = U-turn.
    """
    dp = vdot(prev_dir, next_dir)
    dp = max(-1.0, min(1.0, dp))
    cos_half = math.sqrt(max(0.0, (1.0 + dp) * 0.5))
    sin_half = math.sqrt(max(0.0, (1.0 - dp) * 0.5))

    if sin_half < COLLINEAR_EPS:
        return None

    if cos_half < REVERSAL_EPS:
        return QuinticBlend(
            Q=((0.0, 0.0, 0.0),) * 6,
            theta=math.pi,
            r=_R_CLAMP_MIN,
            d_consumed=0.0,
            kappa_peak=0.0,
            t_peak=0.5,
            v_cap=0.0,
            entry_tangent=prev_dir,
            exit_tangent=next_dir,
            plane_normal=(0.0, 0.0, 0.0),
        )

    theta = 2.0 * math.atan2(sin_half, cos_half)
    r = _shape_ratio(theta)

    d_tol = _d_from_deviation(corner_deviation, r, sin_half)
    d_mid = 0.5 * min(L_prev, L_next)
    d = min(d_tol, d_mid)

    # Build the 6 control points relative to the corner vertex (V at origin).
    # Q0 = -d * prev_dir, Q5 = +d * next_dir.
    Q = (
        vscale(prev_dir, -d),
        vscale(prev_dir, -r * d),
        vscale(prev_dir, -r * d),
        vscale(next_dir, r * d),
        vscale(next_dir, r * d),
        vscale(next_dir, d),
    )

    kappa_peak, t_peak = _peak_curvature(Q)

    # Plane normal: right-handed, consistent with blendmath.
    raw_normal = vcross(prev_dir, next_dir)
    raw_norm_n = vnorm(raw_normal)
    if raw_norm_n == 0.0:
        plane_normal: Vec3 = (0.0, 0.0, 0.0)
    else:
        plane_normal = vscale(raw_normal, 1.0 / raw_norm_n)

    # Centripetal bound.
    if kappa_peak > 0.0:
        v_cent = math.sqrt(a_max / kappa_peak)
    else:
        v_cent = 0.0
    v_cap = v_cent

    return QuinticBlend(
        Q=Q,
        theta=theta,
        r=r,
        d_consumed=d,
        kappa_peak=kappa_peak,
        t_peak=t_peak,
        v_cap=v_cap,
        entry_tangent=prev_dir,
        exit_tangent=next_dir,
        plane_normal=plane_normal,
    )


_SHAPER_SAMPLE_TS = (0.25, 0.5, 0.75)


def _point_frame(Q, t: float) -> Tuple[float, Vec3, Vec3]:
    """Return (R_loc, tangent_hat, normal_hat) at parameter t.

    R_loc = 1 / kappa(t). If the local curvature is near zero (endpoints
    or a nearly-flat stretch), R_loc is +inf — use sparingly. tangent
    and normal are unit vectors in 3D.
    """
    d1 = _quintic_first_deriv(Q, t)
    d2 = _quintic_second_deriv(Q, t)
    d1_norm = vnorm(d1)
    if d1_norm < 1e-12:
        return float("inf"), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
    tangent = vscale(d1, 1.0 / d1_norm)
    # Kappa = |d1 x d2| / |d1|^3
    cross = vcross(d1, d2)
    cross_norm = vnorm(cross)
    if cross_norm < 1e-12:
        return float("inf"), tangent, (0.0, 0.0, 0.0)
    kappa = cross_norm / (d1_norm ** 3)
    R_loc = 1.0 / kappa
    # Principal normal direction: component of d2 perpendicular to tangent.
    # N = (d2 - (d2 . tangent) * tangent) normalized.
    dot_d2_t = vdot(d2, tangent)
    perp = vsub(d2, vscale(tangent, dot_d2_t))
    perp_norm = vnorm(perp)
    if perp_norm < 1e-12:
        return R_loc, tangent, (0.0, 0.0, 0.0)
    normal = vscale(perp, 1.0 / perp_norm)
    return R_loc, tangent, normal


def _three_point_shaper_cap(blend: QuinticBlend, shapers) -> float:
    """Minimum of the shaper entry-step velocity cap evaluated at
    t in {0.25, 0.5, 0.75}. Returns +inf if no shapers or no bound."""
    if not shapers:
        return float("inf")
    p_hat = blend.plane_normal
    cap = float("inf")
    for t in _SHAPER_SAMPLE_TS:
        R_loc, _tangent, normal = _point_frame(blend.Q, t)
        if R_loc == float("inf"):
            continue
        bounds = blendshaper.compute_shaper_bounds(
            shapers=shapers,
            R=R_loc,
            n_hat=normal,
            p_hat=p_hat,
        )
        if bounds.v_step_cap < cap:
            cap = bounds.v_step_cap
    return cap


def _dense_shaper_cap(blend: QuinticBlend, shapers, samples: int = 101) -> float:
    """Reference: dense-sample minimum shaper cap along the blend.

    Used by tests to verify the three-point approximation is close to
    the true minimum. NOT called in production planning paths.
    """
    if not shapers:
        return float("inf")
    cap = float("inf")
    p_hat = blend.plane_normal
    for i in range(samples):
        t = i / (samples - 1)
        R_loc, _tangent, normal = _point_frame(blend.Q, t)
        if R_loc == float("inf"):
            continue
        bounds = blendshaper.compute_shaper_bounds(
            shapers=shapers,
            R=R_loc,
            n_hat=normal,
            p_hat=p_hat,
        )
        if bounds.v_step_cap < cap:
            cap = bounds.v_step_cap
    return cap


def quintic_geometry_with_shaper(
    base: Optional[QuinticBlend],
    shapers,
    j_eff: float,
) -> Optional[QuinticBlend]:
    """Apply shaper + rotation-jerk bounds on top of a base `QuinticBlend`.

    Tightens v_cap by:
      - the three-point shaper entry-step bound, and
      - the rotation-jerk bound v_jerk = (R_peak * sqrt(j_eff))^(2/3)
        evaluated at the peak-curvature point.

    Pass j_eff = +inf to disable the rotation-jerk bound (useful for
    non-shaper callers and tests).
    """
    if base is None:
        return None
    if base.d_consumed == 0.0:
        return base

    v_shaper = _three_point_shaper_cap(base, shapers)

    if base.kappa_peak > 0.0 and j_eff > 0.0 and j_eff != float("inf"):
        R_peak = 1.0 / base.kappa_peak
        v_jerk = (R_peak * math.sqrt(j_eff)) ** (2.0 / 3.0)
    else:
        v_jerk = float("inf")

    v_cap = min(base.v_cap, v_shaper, v_jerk)
    return replace(base, v_cap=v_cap)


def blend_from_moves_quintic(
    prev_move,
    next_move,
    corner_deviation: float,
    j_eff: float = float("inf"),
    toolhead=None,
) -> Optional[QuinticBlend]:
    """Adapter: compute a quintic blend from a pair of Kalico Move-like
    objects. Mirrors `blendmath.blend_from_moves`.

    Skips the blend if either move is non-kinematic (E-only). The
    effective a_max is the stricter of the two moves' accel values.

    If `toolhead` is given, derives `j_eff` and the shaper velocity
    bound from the toolhead's input shaper module. In that case any
    explicit `j_eff` argument is ignored.
    """
    if toolhead is not None and j_eff != float("inf"):
        raise ValueError(
            "blend_from_moves_quintic: j_eff and toolhead are mutually "
            "exclusive (toolhead derives j_eff from shaper state; "
            "passing both is ambiguous)"
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

    base = quintic_geometry(
        prev_dir=prev_dir,
        next_dir=next_dir,
        L_prev=prev_move.move_d,
        L_next=next_move.move_d,
        corner_deviation=corner_deviation,
        a_max=a_max,
    )
    if base is None or base.d_consumed == 0.0:
        return base

    if toolhead is None:
        return quintic_geometry_with_shaper(
            base=base, shapers=[], j_eff=j_eff,
        )

    shapers = _extract_shapers(toolhead)

    # First pass: derive j_eff from shaper state using the peak-curvature
    # radius as the arc-like input (blendshaper.compute_shaper_bounds
    # expects an R; use R = 1/kappa_peak).
    if base.kappa_peak > 0.0 and shapers:
        R_peak = 1.0 / base.kappa_peak
        # Use the normal at the peak-curvature point for j_eff derivation.
        # For the r values in production use (clamped to >= 0.5), t_peak is
        # materially off-center — using the midpoint normal here would pair
        # the peak R with a different point's normal and mis-size j_eff.
        _, _, n_peak = _point_frame(base.Q, base.t_peak)
        bounds = blendshaper.compute_shaper_bounds(
            shapers=shapers,
            R=R_peak,
            n_hat=n_peak,
            p_hat=base.plane_normal,
        )
        derived_j = bounds.j_eff
    else:
        derived_j = float("inf")

    return quintic_geometry_with_shaper(
        base=base, shapers=shapers, j_eff=derived_j,
    )


_SUBDIVIDE_MAX_DEPTH = 12


def _perp_distance(p: Vec3, a: Vec3, b: Vec3) -> float:
    """Perpendicular distance from p to the infinite line through a and b."""
    ab = vsub(b, a)
    ab_len = vnorm(ab)
    if ab_len < 1e-12:
        return vnorm(vsub(p, a))
    ap = vsub(p, a)
    cross = vcross(ab, ap)
    return vnorm(cross) / ab_len


def _quintic_flatness(Q) -> float:
    """Max perpendicular distance of Q1..Q4 from the chord Q0-Q5."""
    chord_a = Q[0]
    chord_b = Q[5]
    return max(
        _perp_distance(Q[1], chord_a, chord_b),
        _perp_distance(Q[2], chord_a, chord_b),
        _perp_distance(Q[3], chord_a, chord_b),
        _perp_distance(Q[4], chord_a, chord_b),
    )


def _quintic_split(Q):
    """Split quintic at t=0.5 via De Casteljau, return (left, right)
    control-point tuples each with 6 points."""
    # Level 0 -> 5 by repeated lerp at t=0.5. Capture outer points at each level.
    p = [Q[i] for i in range(6)]
    left = [p[0]]
    right = [p[5]]
    for level in range(5, 0, -1):
        new_p = [_lerp(p[i], p[i + 1], 0.5) for i in range(level)]
        left.append(new_p[0])
        right.append(new_p[-1])
        p = new_p
    # left has 6 points (Q0 to midpoint); right is captured in reverse order.
    right.reverse()
    return tuple(left), tuple(right)


def segment_quintic(
    blend: QuinticBlend,
    max_chord_err: float = 1e-2,
) -> List[Vec3]:
    """Return a polyline approximating the quintic blend with max chord
    error <= max_chord_err. Adaptive De Casteljau subdivision."""
    if max_chord_err <= 0.0:
        raise ValueError("max_chord_err must be positive")
    if blend.d_consumed == 0.0:
        return [blend.Q[0]]

    out: List[Vec3] = [blend.Q[0]]

    def _recurse(Q, depth):
        if depth >= _SUBDIVIDE_MAX_DEPTH or _quintic_flatness(Q) <= max_chord_err:
            out.append(Q[5])
            return
        left, right = _quintic_split(Q)
        _recurse(left, depth + 1)
        _recurse(right, depth + 1)

    _recurse(blend.Q, 0)
    return out


def interpolate_extruder_quintic(
    polyline: List[Vec3],
    d_consumed: float,
    e_per_mm_prev: float,
    e_per_mm_next: float,
) -> List[Tuple[float, float, float, float]]:
    """Attach an E coordinate to each polyline point.

    The quintic blend replaces the final `d_consumed` mm of the prior
    move and the first `d_consumed` mm of the next move. Total E through
    the blend is conserved: sum across the polyline equals
    `d_consumed * (e_per_mm_prev + e_per_mm_next)`. E is distributed
    uniformly over the polyline's arc-length (piecewise-linear).
    """
    if not polyline:
        return []
    total_e = d_consumed * (e_per_mm_prev + e_per_mm_next)

    seg_lens = []
    total_len = 0.0
    for p0, p1 in zip(polyline, polyline[1:]):
        seg_len = vnorm(vsub(p1, p0))
        seg_lens.append(seg_len)
        total_len += seg_len

    if total_len == 0.0:
        return [(p[0], p[1], p[2], 0.0) for p in polyline]

    out: List[Tuple[float, float, float, float]] = [
        (polyline[0][0], polyline[0][1], polyline[0][2], 0.0),
    ]
    e = 0.0
    for seg_len, p1 in zip(seg_lens, polyline[1:]):
        e += total_e * seg_len / total_len
        out.append((p1[0], p1[1], p1[2], e))
    return out
