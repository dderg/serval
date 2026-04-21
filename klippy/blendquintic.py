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

from typing import Optional, Tuple

from . import blendshape

Vec3 = Tuple[float, float, float]

COLLINEAR_EPS = 1e-6
REVERSAL_EPS = 1e-6
_SUBDIVIDE_MAX_DEPTH = 12
_DEFAULT_CHORD_TOL = 1e-3   # 1 um; tighter than archive's 10 um to reduce segment-boundary kappa steps


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
