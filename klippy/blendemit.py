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
    """Return (polyline, v_caps) where v_caps[i] is the velocity cap
    for the i-th polyline segment (between polyline[i] and polyline[i+1]).

    The blend's single `v_cap` field bundles three upper bounds:
      (1) centripetal:     sqrt(a_max / kappa_peak)
      (2) shaper step:     axis-specific dv per kHz of shaper content
      (3) rotation jerk:   (R_peak * sqrt(j_eff))^(2/3)
    Bounds (2) and (3) are shape-global — applying them per-segment
    at the same value everywhere is correct. Bound (1), however, is a
    worst-case along the polyline; applying it flat is the bug we are
    fixing. Per-segment centripetal is `sqrt(a_max / k_seg)` where
    k_seg is the local curvature across that segment.

    Arc: curvature is constant (1/R), so per-segment centripetal
    matches the blend-wide value and every segment cap equals blend.v_cap
    — identical to the historical flat-cap behaviour.

    Quintic: we separate the shape-global ceiling from the per-segment
    centripetal. If blend.v_cap < v_cent_peak, a non-centripetal bound
    (shaper / jerk) is binding and stays as a global ceiling. If
    blend.v_cap == v_cent_peak (centripetal is the sole binding term)
    there is no global ceiling and segments near the endpoints run
    free up to the move's max_velocity / look-ahead limits.
    """
    poly = segment(blend, max_chord_err)
    if len(poly) < 2:
        return poly, []

    v_global_blend = blend.v_cap

    if isinstance(blend, blendquintic.QuinticBlend):
        # Peak-curvature centripetal, i.e. the piece of blend.v_cap
        # that varies along the polyline. Anything tighter than this
        # in blend.v_cap must be a shape-global bound.
        if blend.kappa_peak > 0.0 and a_max > 0.0:
            v_cent_peak = math.sqrt(a_max / blend.kappa_peak)
        else:
            v_cent_peak = float("inf")
        # Non-centripetal ceiling: if blend.v_cap came in below
        # v_cent_peak, a shaper/jerk bound is binding — carry it
        # through as a global ceiling. Otherwise centripetal is the
        # tightest and there is no additional per-segment ceiling
        # beyond a_max/k_seg.
        if v_global_blend < v_cent_peak:
            v_global = v_global_blend
        else:
            v_global = float("inf")

        pts_with_t = blendquintic.segment_quintic_with_t(blend, max_chord_err)
        # Sanity: segment_quintic and segment_quintic_with_t walk the
        # same recursion so they must produce identical point counts.
        if len(pts_with_t) != len(poly):
            return poly, [v_global_blend] * (len(poly) - 1)
        # Sample curvature at three t-values across each segment:
        # endpoints and the parameter midpoint. The endpoint-only
        # approximation can miss peaks when subdivision lands a vertex
        # exactly at the peak-curvature t (the quintic's peak is at
        # t ~ r which adaptive De Casteljau hits at coarse polylines),
        # degrading per-segment sampling back to a flat-cap result.
        # The extra midpoint evaluation is O(1) per segment and ensures
        # we see the true per-segment maximum even with few polyline
        # vertices.
        kappas = [
            blendquintic._curvature_at(blend.Q, t) for _, t in pts_with_t
        ]
        v_caps = []
        for i in range(len(poly) - 1):
            t_lo = pts_with_t[i][1]
            t_hi = pts_with_t[i + 1][1]
            t_mid = 0.5 * (t_lo + t_hi)
            k_mid = blendquintic._curvature_at(blend.Q, t_mid)
            k_seg = max(kappas[i], kappas[i + 1], k_mid)
            if k_seg > 0.0 and a_max > 0.0:
                v_cent = math.sqrt(a_max / k_seg)
            else:
                v_cent = float("inf")
            v_caps.append(min(v_global, v_cent))
        return poly, v_caps

    # Arc: constant curvature. Per-segment cap equals blend.v_cap
    # (blend.v_cap already folds in the centripetal + shaper + jerk
    # bounds and the centripetal is flat anyway).
    return poly, [v_global_blend] * (len(poly) - 1)
