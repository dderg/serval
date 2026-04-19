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
