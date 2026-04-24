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
    """First-class extruder constraints (pillar 3, Plan 3).

    Post-PA stepper output is bounded by (a_E_max, v_E_max). smooth_time
    is the PA smoothing window and feeds the cap formula via
    K_h = (15/8) / smooth_time.

    Built by `klippy/extras/extruder.py::extruder_limits_snapshot()` and
    read by `klippy/blendextruder.py::cap_move()`.
    """
    a_E_max: float       # mm/s^2 on filament (from config: max_extruder_accel)
    v_E_max: float       # mm/s on filament (config: max_extruder_rpm * rotation_distance / 60)
    smooth_time: float   # seconds — current pressure_advance_smooth_time


@dataclass
class KinematicLimits:
    """Flat dataclass passed into shape factories. Replaces handing the
    whole toolhead object in. Built once per planner run."""
    a_max: float
    v_max: float
    jerk_max: Optional[float]   # j_eff for rotation-jerk cap; None disables
    extruder_caps: Optional[ExtruderLimits]   # plan 3 wires; plan 5 consumes
    # Per-axis shaper snapshots for dense per-s shaper cap in v_cap_fn.
    # Empty list disables the shaper cap.
    # Populated by the planner via extract_shapers; None until Task 12.
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
