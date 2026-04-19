# klippy/blendplanner.py
# Copyright (C) 2026
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# Corner-blending planner integration.
# See docs/superpowers/specs/2026-04-17-planner-integration-design.md
from __future__ import annotations

import math

from . import blendemit, blendmath, blendquintic


def _copy_caller_state(src, dst):
    """Transfer caller-mutable Move state from src to the truncated dst.

    Pins caller-intent fields verbatim (timing_callbacks, next_junction_v2,
    max_cruise_v2, accel) so that M204 / SET_VELOCITY_LIMIT
    / register_lookahead_callback mutations applied upstream to src survive
    the emit-time construction of dst. Recomputes length-derived fields
    (delta_v2, smooth_delta_v2, min_move_t) from dst's NEW move_d and the
    pinned accel.

    The accel pin is a direct assignment (not via dst.limit_speed) because
    limit_speed takes min(self.accel, accel); if an intervening M204 had
    lowered toolhead.max_accel between src construction and emit, Move.__init__'s
    snapshot of the new (lower) value would win over src.accel.
    """
    dst.timing_callbacks = list(src.timing_callbacks)
    dst.next_junction_v2 = src.next_junction_v2
    dst.max_cruise_v2 = src.max_cruise_v2
    dst.accel = src.accel
    dst.delta_v2 = 2.0 * dst.move_d * dst.accel
    ratio = src.smooth_delta_v2 / src.delta_v2 if src.delta_v2 > 0.0 else 1.0
    dst.smooth_delta_v2 = min(
        dst.delta_v2, 2.0 * dst.move_d * dst.accel * ratio
    )
    dst.min_move_t = dst.move_d / math.sqrt(dst.max_cruise_v2)



class CornerBlender:
    """Second filter stage in the blend pipeline.

    Buffers one move; on the next arriving move computes a corner blend
    (arc or quintic, chosen by deflection angle) and emits
    [trunc_prev, blend_polyline_moves...] while buffering the
    truncated-next-head as the new candidate prev.
    """

    def __init__(self, toolhead, *, move_cls, max_chord_err=None):
        self._toolhead = toolhead
        self._move_cls = move_cls
        self.max_chord_err = max_chord_err
        self._prev = None
        self.polyline_moves_emitted = 0
        self.blends_emitted = 0

    def feed(self, move):
        if not move.is_kinematic_move:
            return self.flush() + [move]
        if self._prev is None:
            self._prev = move
            return []
        # _select_blend passes toolhead=... into both shape adapters so
        # their shaper-aware caps are derived from live input-shaper
        # state per corner rather than a module constant.
        blend = self._select_blend(self._prev, move)
        if blend is None:
            # Collinear: prepass should have caught. Emit prev, buffer next.
            emitted = [self._prev]
            self._prev = move
            return emitted
        if blend.d_consumed == 0.0 or blend.v_cap == 0.0:
            # U-turn / degenerate: force a stop at the junction.
            self._prev.limit_next_junction_speed(0.0)
            emitted = [self._prev]
            self._prev = move
            return emitted
        trunc_prev, arc_moves, trunc_next_head = self._emit_blend(
            self._prev, move, blend
        )
        self._prev = trunc_next_head
        self.blends_emitted += 1
        self.polyline_moves_emitted += len(arc_moves)
        return [trunc_prev] + arc_moves

    def _resolve_chord_err(self):
        """Return the polyline chord tolerance to use for the current blend.

        If self.max_chord_err was set at construction time, that value wins.
        Otherwise auto-scale as max(20e-3, 0.2 * toolhead.corner_deviation).
        """
        if self.max_chord_err is not None:
            return self.max_chord_err
        # 20 microns absolute floor; 20% of corner_deviation for a sensible
        # auto-scale at loose tolerances.
        return max(20e-3, 0.2 * self._toolhead.corner_deviation)

    def _select_blend(self, prev, nxt):
        """Pick arc or quintic per the deflection-angle rule.

        alpha < low -> arc; low <= alpha <= high -> quintic;
        alpha > high -> arc. Thresholds come from the toolhead config
        (shape_switchover_low_deg / shape_switchover_high_deg).
        """
        prev_dir = prev.axes_r[:3]
        next_dir = nxt.axes_r[:3]
        dot = (
            prev_dir[0] * next_dir[0]
            + prev_dir[1] * next_dir[1]
            + prev_dir[2] * next_dir[2]
        )
        # Clamp for numerical safety before acos.
        if dot > 1.0:
            dot = 1.0
        elif dot < -1.0:
            dot = -1.0
        alpha_deg = math.degrees(math.acos(dot))
        low = self._toolhead.shape_switchover_low_deg
        high = self._toolhead.shape_switchover_high_deg
        if low <= alpha_deg <= high:
            return blendquintic.blend_from_moves_quintic(
                prev, nxt,
                self._toolhead.corner_deviation,
                toolhead=self._toolhead,
            )
        return blendmath.blend_from_moves(
            prev, nxt,
            self._toolhead.corner_deviation,
            toolhead=self._toolhead,
        )

    def _emit_blend(self, prev, nxt, blend):
        """Construct [trunc_prev, blend_moves...] and the trunc_next_head.

        Returns (trunc_prev, arc_moves_list, trunc_next_head). The
        arc_moves_list name is historical; it holds polyline moves for
        whichever shape (arc or quintic) the selector picked.
        """
        th = self._toolhead
        move_cls = self._move_cls

        prev_dir = prev.axes_r[:3]
        next_dir = nxt.axes_r[:3]
        vertex = prev.end_pos[:3]

        # --- 1. Truncated prev ---
        prev_cruise_v = math.sqrt(prev.max_cruise_v2)
        trunc_prev_end_xyz = tuple(
            vertex[i] - blend.d_consumed * prev_dir[i] for i in range(3)
        )
        # E carried proportional to the truncated fraction of prev.move_d.
        frac_prev = 1.0 - blend.d_consumed / prev.move_d
        trunc_prev_end_e = prev.start_pos[3] + frac_prev * prev.axes_d[3]
        trunc_prev_end = (
            trunc_prev_end_xyz[0], trunc_prev_end_xyz[1],
            trunc_prev_end_xyz[2], trunc_prev_end_e,
        )
        trunc_prev = move_cls(th, prev.start_pos, trunc_prev_end, prev_cruise_v)
        _copy_caller_state(prev, trunc_prev)

        # --- 2. Arc polyline ---
        chord_err = self._resolve_chord_err()
        arc_accel = min(prev.accel, nxt.accel)
        # Per-segment velocity caps from local curvature. For an arc the
        # list is flat (equal to blend.v_cap everywhere) — identical to
        # the historical flat-cap behaviour. For a quintic the endpoints
        # get generous caps and the peak-curvature region keeps the tight
        # centripetal bound; look-ahead ramps tangentially between them.
        polyline_local, seg_v_caps = blendemit.per_segment_v_cap(
            blend, chord_err, arc_accel,
        )
        polyline_world = [
            (p[0] + vertex[0], p[1] + vertex[1], p[2] + vertex[2])
            for p in polyline_local
        ]
        points_4d = blendmath.interpolate_extruder(
            polyline_world, blend.d_consumed,
            prev.axes_r[3], nxt.axes_r[3],
        )
        # Offset the interpolate_extruder E (starts at 0) by trunc_prev_end_e
        # so each polyline point's absolute E continues the global count.
        points_4d = [
            (p[0], p[1], p[2], p[3] + trunc_prev_end_e) for p in points_4d
        ]
        # Move.__init__ clamps to toolhead.max_velocity, so the widest cap we
        # can construct a Move with is min(prev/nxt cruise ceilings). Per-
        # segment caps then apply on top via limit_speed / max_cruise_v2.
        neighbour_cap_v2 = min(prev.max_cruise_v2, nxt.max_cruise_v2)
        neighbour_cap_v = math.sqrt(neighbour_cap_v2)
        arc_moves = []
        for (p0, p1), seg_v_cap in zip(
            zip(points_4d, points_4d[1:]), seg_v_caps
        ):
            seg_cap_v2 = min(neighbour_cap_v2, seg_v_cap ** 2)
            seg_cap_v = math.sqrt(seg_cap_v2)
            am = move_cls(th, p0, p1, neighbour_cap_v)
            am.max_cruise_v2 = seg_cap_v2
            am.limit_speed(seg_cap_v, arc_accel)
            # Look-ahead smoothing (smooth_delta_v2) is deliberately left
            # untouched: for quintics it unlocks the tangential ramp that
            # speeds up at low-curvature endpoints and decelerates into
            # the peak; for arcs seg_v_cap is flat so the smoothed ramp
            # is also flat in practice.
            am.min_move_t = am.move_d / seg_cap_v
            arc_moves.append(am)

        # --- 3. Truncated next head ---
        trunc_next_head_start_xyz = tuple(
            vertex[i] + blend.d_consumed * next_dir[i] for i in range(3)
        )
        # E at the truncated-next-head start: offset from nxt.start_pos by the
        # consumed head fraction. Symmetric with trunc_prev's E formula.
        frac_consumed_next = blend.d_consumed / nxt.move_d
        trunc_next_head_start_e = nxt.start_pos[3] + frac_consumed_next * nxt.axes_d[3]
        trunc_next_head_start = (
            trunc_next_head_start_xyz[0], trunc_next_head_start_xyz[1],
            trunc_next_head_start_xyz[2], trunc_next_head_start_e,
        )
        next_cruise_v = math.sqrt(nxt.max_cruise_v2)
        trunc_next_head = move_cls(
            th, trunc_next_head_start, nxt.end_pos, next_cruise_v
        )
        _copy_caller_state(nxt, trunc_next_head)

        # Aggregate-safety re-check. check_move runs before lookahead.add_move
        # in ToolHead.move, so emitted arc-polyline Moves bypass it otherwise.
        # One representative is sufficient: all arc moves share accel, v_cap,
        # and per-mm E rate; spatially the polyline is localized near the
        # corner vertex so envelope checks evaluate at roughly the same
        # coordinates across all points.
        if arc_moves:
            representative = arc_moves[0]
            th.kin.check_move(representative)
            if representative.axes_d[3]:
                th.extruder.check_move(representative)

        return trunc_prev, arc_moves, trunc_next_head

    def flush(self):
        if self._prev is None:
            return []
        emitted = [self._prev]
        self._prev = None
        return emitted

    def reset(self):
        self._prev = None

    def peek_buffered(self):
        return [self._prev] if self._prev is not None else []
