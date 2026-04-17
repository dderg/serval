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
from dataclasses import dataclass
from typing import Optional, Tuple

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
    """Tangent-arc blend geometry between two adjacent moves."""

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
    """Compute the tangent-arc blend for a corner, or None if no blend needed."""
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

    # Midpoint / adjacent-segment cap. cot(theta/2) = cos_half / sin_half.
    R_mid = min(L_prev, L_next) * cos_half / sin_half

    R = min(R_tol, R_mid)

    # Placeholder v_cap; refined in later tasks.
    v_cap = float("inf")

    return BlendArc(
        R=R,
        theta=theta,
        d_consumed=R * sin_half / cos_half,
        v_cap=v_cap,
        center=(0.0, 0.0, 0.0),
        entry_pt=(0.0, 0.0, 0.0),
        exit_pt=(0.0, 0.0, 0.0),
        entry_tangent=prev_dir,
        exit_tangent=next_dir,
        plane_normal=(0.0, 0.0, 0.0),
    )
