# klippy/blendplanner.py
# Copyright (C) 2026
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# Corner-blending planner integration.
# See docs/superpowers/specs/2026-04-17-planner-integration-design.md
from __future__ import annotations

import math

from . import blendmath


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
        return []

    def flush(self):
        return []

    def reset(self):
        self._prev = None

    def peek_buffered(self):
        return [self._prev] if self._prev is not None else []
