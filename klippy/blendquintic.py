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
    vadd,
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
        v_cap:          maximum traversal velocity (mm/s)
        entry_tangent:  unit vector, same as prev_dir into corner
        exit_tangent:   unit vector, same as next_dir out of corner
        plane_normal:   unit vector orthogonal to the blend plane
    """
    Q: Tuple[Vec3, Vec3, Vec3, Vec3, Vec3, Vec3]
    theta: float
    r: float
    d_consumed: float
    kappa_peak: float
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

    kappa_peak, _ = _peak_curvature(Q)

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
        v_cap=v_cap,
        entry_tangent=prev_dir,
        exit_tangent=next_dir,
        plane_normal=plane_normal,
    )
