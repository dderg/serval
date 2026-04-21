# klippy/blendshape.py
# Copyright (C) 2026
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# Shape-agnostic types for curvature-continuous corner blends.
#
# SmoothShape is a Protocol: any curve implementation (quintic Bezier,
# Pythagorean-Hodograph spline, Euler-spiral clothoid, ...) that exposes
# the listed surface is a SmoothShape. The planner talks to this
# protocol; concrete shapes never leak implementation details (control
# points, Fresnel tables, speed polynomials).
#
# KinematicLimits is the flat dataclass shape factories take in place of
# the whole toolhead object — decouples shape construction from the
# full kinematics/extruder stack.
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional, Protocol, runtime_checkable, Tuple

Vec3 = Tuple[float, float, float]


@dataclass
class ExtruderLimits:
    """First-class extruder constraints (pillar 3).

    Plan 1 leaves this as None everywhere; plan 4 threads it through.
    """
    accel_max: float   # mm/s^2 on the filament
    rpm_max: float     # drive-pulley angular velocity


@dataclass
class KinematicLimits:
    """Flat dataclass passed into shape factories. Replaces handing the
    whole toolhead object in. Built once per planner run."""
    a_max: float
    v_max: float
    jerk_max: Optional[float]   # j_eff for rotation-jerk cap; None disables
    extruder_caps: Optional[ExtruderLimits]   # None until plan 4 (pillar 3)
    # Per-axis shaper snapshots for dense per-s shaper cap in v_cap_fn.
    # Empty list disables the shaper cap.
    # Populated by the planner via _extract_shapers; None until Task 12.
    shapers: Optional[list] = field(default=None)


@runtime_checkable
class SmoothShape(Protocol):
    """Curvature-continuous corner blend between two adjacent moves.

    Arc-length parameterised; s in [0, arc_length]. Protocol is
    implementation-opaque — consumers see only this surface.

    Velocity-limit convention: `v_cap_fn(s)` returns the velocity
    limit curve V_lim(s) from centripetal + shaper + (optional) jerk
    bounds. Pillar 3 (plan 4) wraps this with an extruder cap as a
    separate stage, not here.
    """

    d_consumed: float   # tangent length consumed per incoming edge [mm]
    theta: float        # deflection angle [rad]
    arc_length: float   # total length of the blend [mm]

    def position_at(self, s: float) -> Vec3: ...
    def tangent_at(self, s: float) -> Vec3: ...
    def curvature_at(self, s: float) -> float: ...
    def dkappa_ds(self, s: float) -> float: ...
    def v_cap_fn(self, s: float) -> float: ...
    def polyline(self, chord_tol: float) -> list[Vec3]: ...
