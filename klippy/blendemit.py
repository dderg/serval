# klippy/blendemit.py
# Copyright (C) 2026
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# Shape-agnostic emission helpers for the corner blender.
#
# The planner calls into a single seam so new shapes (clothoid, etc.)
# slot in here without touching CornerBlender.
#
# See docs/superpowers/specs/2026-04-19-subspec-6e-shape-selection-design.md
from __future__ import annotations

from . import blendmath, blendquintic


def segment(blend, max_chord_err):
    """Return a polyline approximating `blend` with chord error
    <= max_chord_err. Dispatches on the blend's dataclass type.
    """
    if isinstance(blend, blendquintic.QuinticBlend):
        return blendquintic.segment_quintic(blend, max_chord_err)
    return blendmath.segment_arc(blend, max_chord_err)
