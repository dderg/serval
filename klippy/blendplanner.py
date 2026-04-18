# klippy/blendplanner.py
# Copyright (C) 2026
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# Corner-blending planner integration.
# See docs/superpowers/specs/2026-04-17-planner-integration-design.md
from __future__ import annotations

import math

from . import blendmath


def _copy_caller_state(src, dst):
    """Transfer caller-mutable Move state from src to the truncated dst.

    Pins caller-intent fields verbatim (timing_callbacks, next_junction_v2,
    max_cruise_v2, junction_deviation, accel) so that M204 / SET_VELOCITY_LIMIT
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
    dst.junction_deviation = src.junction_deviation
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
        arc = blendmath.blend_from_moves(
            self._prev, move,
            self._toolhead.corner_deviation,
            toolhead=self._toolhead,
        )
        if arc is None:
            # Collinear: prepass should have caught. Emit prev, buffer next.
            emitted = [self._prev]
            self._prev = move
            return emitted
        if arc.R == 0.0 or arc.v_cap == 0.0:
            # U-turn / degenerate: force a stop at the junction.
            self._prev.limit_next_junction_speed(0.0)
            emitted = [self._prev]
            self._prev = move
            return emitted
        # Blend steps 6–8 (arc emission) come in Task 8.
        emitted = [self._prev]
        self._prev = move
        return emitted

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
