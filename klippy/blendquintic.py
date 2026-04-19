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
