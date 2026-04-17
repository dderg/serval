# klippy/blendshaper.py
# Shaper-derived jerk bound module for corner blending.
#
# Given a toolhead's per-axis input-shaper configuration and a
# blend-arc corner geometry, computes the effective jerk ceiling
# (j_eff) passed to blendmath.blend_geometry plus a per-axis
# entry-step velocity cap (v_step_cap) applied post-hoc.
#
# Pure math: zero Kalico imports. All per-axis shaper state is
# carried in AxisShaperSnapshot records created by the adapter.
#
# See docs/superpowers/specs/2026-04-17-j-eff-derivation-design.md
#
# Copyright (C) 2026
# This file may be distributed under the terms of the GNU GPLv3 license.
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Optional, Tuple

Vec3 = Tuple[float, float, float]

PROJECTION_EPS = 1e-9


@dataclass(frozen=True)
class AxisShaperSnapshot:
    axis: str
    shaper_type: Optional[str]
    shaper_freq: float
    damping_ratio: float
    A_axis: float


@dataclass(frozen=True)
class ShaperBounds:
    j_eff: float
    v_step_cap: float
