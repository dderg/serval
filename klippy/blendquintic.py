# klippy/blendquintic.py
# Copyright (C) 2026
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# Quintic Hermite Bezier corner-blending primitive.
#
# Implements the SmoothShape protocol. Arc-length parameterised via a
# cached 8-Gauss-Legendre s -> t map built at from_moves time.
#
# Math verified via audit of blend-arc-quintic-archive; the five
# correct pieces (De Casteljau, curvature, chord deviation, r(theta),
# rotation-jerk) port verbatim. The three-point shaper cap from the
# archive is replaced with dense sampling (archive had a silent ~15%
# overshoot at the worst angles).
from __future__ import annotations

import math
from typing import Optional, Tuple

from . import blendshape

Vec3 = Tuple[float, float, float]

COLLINEAR_EPS = 1e-6
REVERSAL_EPS = 1e-6
_SUBDIVIDE_MAX_DEPTH = 12
_DEFAULT_CHORD_TOL = 1e-3   # 1 um; tighter than archive's 10 um to reduce segment-boundary kappa steps


def _lerp(a: Vec3, b: Vec3, t: float) -> Vec3:
    return (
        a[0] + (b[0] - a[0]) * t,
        a[1] + (b[1] - a[1]) * t,
        a[2] + (b[2] - a[2]) * t,
    )


def _quintic_eval(Q, t: float) -> Vec3:
    """De Casteljau evaluation of a 6-control-point quintic Bezier."""
    p = [Q[i] for i in range(6)]
    for level in range(5, 0, -1):
        for i in range(level):
            p[i] = _lerp(p[i], p[i + 1], t)
    return p[0]


def _quintic_first_deriv(Q, t: float) -> Vec3:
    """B'(t) for a quintic Bezier. Degree-4 Bezier with control points
    5*(Q[i+1] - Q[i])."""
    D = [
        (
            5.0 * (Q[i + 1][0] - Q[i][0]),
            5.0 * (Q[i + 1][1] - Q[i][1]),
            5.0 * (Q[i + 1][2] - Q[i][2]),
        )
        for i in range(5)
    ]
    # De Casteljau on the degree-4 control points.
    p = [D[i] for i in range(5)]
    for level in range(4, 0, -1):
        for i in range(level):
            p[i] = _lerp(p[i], p[i + 1], t)
    return p[0]


def _quintic_second_deriv(Q, t: float) -> Vec3:
    """B''(t). Degree-3 Bezier with control points 20*(Q[i+2]-2*Q[i+1]+Q[i])."""
    D2 = [
        (
            20.0 * (Q[i + 2][0] - 2.0 * Q[i + 1][0] + Q[i][0]),
            20.0 * (Q[i + 2][1] - 2.0 * Q[i + 1][1] + Q[i][1]),
            20.0 * (Q[i + 2][2] - 2.0 * Q[i + 1][2] + Q[i][2]),
        )
        for i in range(4)
    ]
    p = [D2[i] for i in range(4)]
    for level in range(3, 0, -1):
        for i in range(level):
            p[i] = _lerp(p[i], p[i + 1], t)
    return p[0]


def _quintic_third_deriv(Q, t: float) -> Vec3:
    """B'''(t). Degree-2 Bezier with control points 60*(Q[i+3]-3*Q[i+2]+3*Q[i+1]-Q[i])."""
    D3 = [
        (
            60.0 * (Q[i + 3][0] - 3.0 * Q[i + 2][0] + 3.0 * Q[i + 1][0] - Q[i][0]),
            60.0 * (Q[i + 3][1] - 3.0 * Q[i + 2][1] + 3.0 * Q[i + 1][1] - Q[i][1]),
            60.0 * (Q[i + 3][2] - 3.0 * Q[i + 2][2] + 3.0 * Q[i + 1][2] - Q[i][2]),
        )
        for i in range(3)
    ]
    p = [D3[i] for i in range(3)]
    for level in range(2, 0, -1):
        for i in range(level):
            p[i] = _lerp(p[i], p[i + 1], t)
    return p[0]


def _quintic_split(Q):
    """Split a quintic Bezier at t=0.5 via De Casteljau; returns (left, right)."""
    # Run De Casteljau at t=0.5, collecting left/right control nets.
    p = [Q[i] for i in range(6)]
    left = [p[0]]
    right_tail = [p[5]]
    for level in range(5, 0, -1):
        new_p = []
        for i in range(level):
            m = _lerp(p[i], p[i + 1], 0.5)
            new_p.append(m)
        left.append(new_p[0])
        right_tail.append(new_p[-1])
        p = new_p
    right = list(reversed(right_tail))
    return tuple(left), tuple(right)


def _perp_distance(p: Vec3, a: Vec3, b: Vec3) -> float:
    """Perpendicular distance from point p to the line through a,b."""
    ab = (b[0] - a[0], b[1] - a[1], b[2] - a[2])
    ap = (p[0] - a[0], p[1] - a[1], p[2] - a[2])
    cross = (
        ap[1] * ab[2] - ap[2] * ab[1],
        ap[2] * ab[0] - ap[0] * ab[2],
        ap[0] * ab[1] - ap[1] * ab[0],
    )
    ab_len = math.sqrt(ab[0] ** 2 + ab[1] ** 2 + ab[2] ** 2)
    if ab_len < 1e-12:
        return math.sqrt(ap[0] ** 2 + ap[1] ** 2 + ap[2] ** 2)
    cross_len = math.sqrt(cross[0] ** 2 + cross[1] ** 2 + cross[2] ** 2)
    return cross_len / ab_len


def _quintic_flatness(Q) -> float:
    """Max perpendicular distance of Q1..Q4 from the chord Q0-Q5.

    Classical upper bound on curve-to-chord distance via the convex-hull
    property. Used as the adaptive-subdivision termination metric.
    """
    chord_a = Q[0]
    chord_b = Q[5]
    return max(
        _perp_distance(Q[1], chord_a, chord_b),
        _perp_distance(Q[2], chord_a, chord_b),
        _perp_distance(Q[3], chord_a, chord_b),
        _perp_distance(Q[4], chord_a, chord_b),
    )


def _curvature_at_t(Q, t: float) -> float:
    """Curvature at parameter t. For 2D (z=0), reduces to
    kappa = |B'_x * B''_y - B'_y * B''_x| / |B'|^3.
    For 3D, kappa = |B' x B''| / |B'|^3.
    """
    d1 = _quintic_first_deriv(Q, t)
    d2 = _quintic_second_deriv(Q, t)
    cross = (
        d1[1] * d2[2] - d1[2] * d2[1],
        d1[2] * d2[0] - d1[0] * d2[2],
        d1[0] * d2[1] - d1[1] * d2[0],
    )
    num = math.sqrt(cross[0] ** 2 + cross[1] ** 2 + cross[2] ** 2)
    den = (d1[0] ** 2 + d1[1] ** 2 + d1[2] ** 2) ** 1.5
    if den < 1e-30:
        return 0.0
    return num / den


def _point_frame(Q, t: float) -> tuple[Vec3, Vec3, Vec3]:
    """Return (position, unit tangent, unit normal) at parameter t.

    Normal is the 2D planar normal in the xy-plane (rot90 of tangent);
    for 3D paths the formula would use the Frenet frame but MO is 2D.
    """
    p = _quintic_eval(Q, t)
    d1 = _quintic_first_deriv(Q, t)
    d1n = math.sqrt(d1[0] ** 2 + d1[1] ** 2 + d1[2] ** 2)
    if d1n < 1e-30:
        return p, (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)
    tan = (d1[0] / d1n, d1[1] / d1n, d1[2] / d1n)
    # 2D normal: rotate tangent 90 deg CCW in xy-plane.
    nrm = (-tan[1], tan[0], 0.0)
    return p, tan, nrm


def _peak_curvature(Q, n_samples: int = 100) -> tuple[float, float]:
    """Dense-sampling peak-curvature evaluator.

    Returns (t_peak, kappa_peak). n_samples=100 gives ~5 sig-fig agreement
    with 20001-sample reference per archive audit.
    """
    best_t = 0.5
    best_k = 0.0
    for i in range(n_samples + 1):
        t = i / n_samples
        k = _curvature_at_t(Q, t)
        if k > best_k:
            best_k = k
            best_t = t
    return best_t, best_k


class QuinticShape:
    """Quintic Hermite Bezier corner blend. Implements SmoothShape."""

    d_consumed: float
    theta: float
    arc_length: float

    def __init__(self) -> None:
        raise NotImplementedError(
            "QuinticShape is constructed via QuinticShape.from_moves(...)"
        )

    @classmethod
    def from_moves(
        cls,
        prev_move,
        next_move,
        corner_deviation: float,
        limits: blendshape.KinematicLimits,
    ) -> Optional["QuinticShape"]:
        """Construct a quintic blend for the corner between prev_move and
        next_move. Returns None for degenerate corners (collinear,
        near-reversal, chord budget infeasible). Caller (planner) falls
        back to sharp-V when None is returned.
        """
        if prev_move is None or next_move is None:
            return None
        # Full implementation lands in task 12.
        return None

    # Protocol methods stubbed to allow isinstance checks; each one is
    # filled in by the tasks below.
    def position_at(self, s: float) -> Vec3:
        raise NotImplementedError

    def tangent_at(self, s: float) -> Vec3:
        raise NotImplementedError

    def curvature_at(self, s: float) -> float:
        raise NotImplementedError

    def dkappa_ds(self, s: float) -> float:
        raise NotImplementedError

    def v_cap_fn(self, s: float) -> float:
        raise NotImplementedError

    def polyline(self, chord_tol: float = _DEFAULT_CHORD_TOL) -> list[Vec3]:
        raise NotImplementedError
