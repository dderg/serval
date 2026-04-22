# klippy/blendplanner.py
# Copyright (C) 2026
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# Corner-blending planner integration.
# See docs/superpowers/specs/2026-04-17-planner-integration-design.md
from __future__ import annotations

import math

from . import blendmath, blendquintic, blendshape


def _extract_extruder_caps(toolhead):
    """Pull the ExtruderLimits off the toolhead's extruder if configured.

    Returns None when the extruder cap is disabled or no extruder is
    present — the downstream shape-build code treats None as 'no cap'.

    Plan 3 wires this; Plan 5 (pillar 2 unified v(s)) will consume it
    as part of the continuous v(s) evaluation along the curve. For
    now the per-move cap is applied at Move-level in Move.limit_speed
    (see Task 11).
    """
    extruder = getattr(toolhead, "extruder", None)
    if extruder is None:
        return None
    snap_fn = getattr(extruder, "extruder_limits_snapshot", None)
    if snap_fn is None:
        # PrinterExtruder delegates to its primary ExtruderStepper.
        steppers = getattr(extruder, "extruder_steppers", None)
        if steppers:
            snap_fn = getattr(steppers[0], "extruder_limits_snapshot", None)
    if snap_fn is None:
        return None
    snapshot = snap_fn()
    if snapshot is None:
        return None
    _, limits = snapshot
    return limits


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

    Buffers one move; on the next arriving move computes a tangent-arc
    blend and emits [trunc_prev, arc_polyline_moves...] while buffering
    the truncated-next-head as the new candidate prev.
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
        th = self._toolhead
        limits = blendshape.KinematicLimits(
            a_max=th.max_accel,
            v_max=th.max_velocity,
            jerk_max=None,       # plan 1: jerk cap disabled; plan 5 wires it
            extruder_caps=_extract_extruder_caps(th),  # plan 3; consumed by plan 5
            shapers=blendmath._extract_shapers(th),
        )
        shape = blendquintic.QuinticShape.from_moves(
            self._prev, move,
            th.corner_deviation,
            limits,
        )
        if shape is None or blendmath.should_suppress_quintic(
                self._prev, move, th.corner_deviation, shape, th):
            # Drop the blend and fall into the sharp-V suppressed path.
            # Reasons include:
            #   (a) shape is None: collinear corners, near-reversals, or
            #       moves too short to accommodate the blend.
            #   (b) should_suppress_quintic fired on a successfully-formed
            #       shape: two-clause D3 rule determined sharp-V + shaper
            #       is at least as good as the blend (path-tolerance and
            #       time both satisfied).
            return self._suppress_and_advance(move)
        trunc_prev, blend_moves, trunc_next_head = self._emit_blend(
            self._prev, move, shape
        )
        self._prev = trunc_next_head
        self.blends_emitted += 1
        self.polyline_moves_emitted += len(blend_moves)
        return [trunc_prev] + blend_moves

    def _suppress_and_advance(self, move):
        """Fallback path when the blend is dropped — either from_moves
        returned None OR the D3 quintic suppression rule fired on an
        otherwise-valid shape. Caps prev's next-junction velocity via
        suppressed_junction_v when a shaper is loaded; otherwise uses
        the near-reversal heuristic as a safety net.

        suppressed_junction_v derives an SCV-equivalent cap from the
        active shaper's sigma_T; it is shape-agnostic (works for both
        the shape=None case and the suppression case).
        """
        th = self._toolhead
        v_j = blendmath.suppressed_junction_v(
            self._prev, move, th.corner_deviation, th
        )
        if v_j is not None and math.isfinite(v_j):
            self._prev.limit_next_junction_speed(v_j)
        else:
            # No shaper loaded (or v_j undefined). Fall back to the
            # near-reversal hard-stop heuristic so the toolhead
            # doesn't round pi-radian reversals at cruise velocity.
            dp = sum(
                self._prev.axes_r[i] * move.axes_r[i] for i in range(3)
            )
            if dp <= -0.5:
                self._prev.limit_next_junction_speed(0.0)
        emitted = [self._prev]
        self._prev = move
        return emitted

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

    def _emit_blend(self, prev, nxt, shape):
        """Construct [trunc_prev, polyline_moves...] and the trunc_next_head.

        `shape` is a QuinticShape (SmoothShape protocol). Returns
        (trunc_prev, blend_moves_list, trunc_next_head).
        """
        th = self._toolhead
        move_cls = self._move_cls

        prev_dir = prev.axes_r[:3]
        next_dir = nxt.axes_r[:3]
        vertex = prev.end_pos[:3]

        # --- 1. Truncated prev ---
        prev_cruise_v = math.sqrt(prev.max_cruise_v2)
        trunc_prev_end_xyz = tuple(
            vertex[i] - shape.d_consumed * prev_dir[i] for i in range(3)
        )
        # E carried proportional to the truncated fraction of prev.move_d.
        frac_prev = 1.0 - shape.d_consumed / prev.move_d
        trunc_prev_end_e = prev.start_pos[3] + frac_prev * prev.axes_d[3]
        trunc_prev_end = (
            trunc_prev_end_xyz[0], trunc_prev_end_xyz[1],
            trunc_prev_end_xyz[2], trunc_prev_end_e,
        )
        trunc_prev = move_cls(th, prev.start_pos, trunc_prev_end, prev_cruise_v)
        _copy_caller_state(prev, trunc_prev)

        # --- 2. Quintic polyline ---
        # shape.polyline() returns world-space Vec3 points (control points are
        # built in world coordinates in from_moves). No vertex offset needed.
        chord_err = self._resolve_chord_err()
        polyline_world = shape.polyline(chord_err)
        points_4d = blendmath.interpolate_extruder(
            polyline_world, shape.d_consumed,
            prev.axes_r[3], nxt.axes_r[3],
        )
        # Offset the interpolate_extruder E (starts at 0) by trunc_prev_end_e
        # so each polyline point's absolute E continues the global count.
        points_4d = [
            (p[0], p[1], p[2], p[3] + trunc_prev_end_e) for p in points_4d
        ]
        # Plan 1 scalar v_cap: midpoint of the blend arc-length.
        # Pillar 2 plan replaces this with per-segment v(s) integration.
        shape_mid_v = shape.v_cap_fn(shape.arc_length / 2.0)
        arc_cap_v2 = min(prev.max_cruise_v2, nxt.max_cruise_v2, shape_mid_v ** 2)
        arc_cap_v = math.sqrt(arc_cap_v2)
        arc_accel = min(prev.accel, nxt.accel)
        blend_moves = []
        for p0, p1 in zip(points_4d, points_4d[1:]):
            am = move_cls(th, p0, p1, arc_cap_v)
            am.max_cruise_v2 = arc_cap_v2
            am.limit_speed(arc_cap_v, arc_accel)
            am.min_move_t = am.move_d / arc_cap_v
            blend_moves.append(am)

        # --- 3. Truncated next head ---
        trunc_next_head_start_xyz = tuple(
            vertex[i] + shape.d_consumed * next_dir[i] for i in range(3)
        )
        # E at the truncated-next-head start: offset from nxt.start_pos by the
        # consumed head fraction. Symmetric with trunc_prev's E formula.
        frac_consumed_next = shape.d_consumed / nxt.move_d
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
        # in ToolHead.move, so emitted polyline Moves bypass it otherwise.
        # One representative is sufficient: all blend moves share accel, v_cap,
        # and per-mm E rate; spatially the polyline is localized near the
        # corner vertex so envelope checks evaluate at roughly the same
        # coordinates across all points.
        if blend_moves:
            representative = blend_moves[0]
            th.kin.check_move(representative)
            if representative.axes_d[3]:
                th.extruder.check_move(representative)

        return trunc_prev, blend_moves, trunc_next_head

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
