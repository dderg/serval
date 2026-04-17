# klippy/blendprepass.py
# Copyright (C) 2026
# This file may be distributed under the terms of the GNU GPLv3 license.
from __future__ import annotations


class CollinearCollapser:
    """Naive-CAM collinearity prepass. See
    docs/superpowers/specs/2026-04-17-naive-cam-prepass-design.md for rationale.
    """

    def __init__(self, toolhead, move_cls):
        self._toolhead = toolhead
        self._move_cls = move_cls
        self._chain = []
        self.tolerance = 25e-3
        self.max_chain = 100
        self.epm_rel = 1e-2
        self.f_rel = 1e-6
        self.min_seg_len = 1e-9
        self.t_eps = 1e-9

    def feed(self, move):
        return []

    def flush(self):
        return []

    def reset(self):
        self._chain = []
