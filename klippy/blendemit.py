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

import math

from . import blendmath, blendquintic


def segment(blend, max_chord_err):
    """Return a polyline approximating `blend` with chord error
    <= max_chord_err. Dispatches on the blend's dataclass type.
    """
    if isinstance(blend, blendquintic.QuinticBlend):
        return blendquintic.segment_quintic(blend, max_chord_err)
    return blendmath.segment_arc(blend, max_chord_err)


def per_segment_v_cap(blend, max_chord_err, a_max):
    """Return (polyline, v_caps) where v_caps[i] is the centripetal
    velocity cap for the i-th polyline segment (between polyline[i]
    and polyline[i+1]).

    For an arc: curvature is constant (1/R) along the whole blend, so
    every segment's cap equals the blend-wide v_cap — the existing
    flat-cap behaviour is preserved identically.

    For a quintic: curvature varies (bathtub shape peaking near
    t = r and 1-r). Each segment cap is derived from the max of the
    per-vertex curvatures at its two endpoints (conservative). The
    segments straddling the peak get the tight cap; segments near
    the low-curvature endpoints get generous caps and rely on
    look-ahead to ramp tangentially through the blend.

    The blend-wide v_cap (shaper + rotation-jerk bounds) is an upper
    ceiling applied to every per-segment cap — those bounds are
    shape-global, not segment-local.
    """
    poly = segment(blend, max_chord_err)
    if len(poly) < 2:
        return poly, []

    v_global = blend.v_cap

    if isinstance(blend, blendquintic.QuinticBlend):
        pts_with_t = blendquintic.segment_quintic_with_t(blend, max_chord_err)
        # Sanity: segment_quintic and segment_quintic_with_t walk the
        # same recursion so they must produce identical point counts.
        # Guard defensively.
        if len(pts_with_t) != len(poly):
            # Fall back to flat cap rather than mis-align segment caps.
            return poly, [v_global] * (len(poly) - 1)
        kappas = [
            blendquintic._curvature_at(blend.Q, t) for _, t in pts_with_t
        ]
        v_caps = []
        for i in range(len(poly) - 1):
            k_seg = max(kappas[i], kappas[i + 1])
            if k_seg > 0.0 and a_max > 0.0:
                v_cent = math.sqrt(a_max / k_seg)
            else:
                v_cent = float("inf")
            v_caps.append(min(v_global, v_cent))
        return poly, v_caps

    # Arc: constant curvature. Per-segment cap equals blend.v_cap
    # (blend.v_cap already folds in the centripetal + shaper + jerk
    # bounds; deriving v_cent from a_max/kappa here would re-do the
    # centripetal derivation without the shaper/jerk fold).
    return poly, [v_global] * (len(poly) - 1)
