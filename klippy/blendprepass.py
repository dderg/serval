# klippy/blendprepass.py
# Copyright (C) 2026
# This file may be distributed under the terms of the GNU GPLv3 license.
from __future__ import annotations

import logging


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
        if move.move_d < self.min_seg_len:
            return [move]
        if not move.is_kinematic_move:
            return self._flush_chain() + [move]
        if not self._chain:
            self._chain = [move]
            return []
        if not self._merge_gate_passes(move):
            emitted = self._flush_chain()
            self._chain = [move]
            return emitted
        self._chain.append(move)
        return []

    def _merge_gate_passes(self, candidate):
        anchor = self._chain[0]
        # Gate (a): cruise velocity equality
        max_cv2 = max(candidate.max_cruise_v2, anchor.max_cruise_v2)
        if abs(candidate.max_cruise_v2 - anchor.max_cruise_v2) > self.f_rel * max_cv2:
            return False
        # Gate (b): E-per-XYZ-mm equality (signed; retract<->extrude reversal fails)
        ae = candidate.axes_r[3]
        be = anchor.axes_r[3]
        if abs(ae - be) > self.epm_rel * max(abs(ae), abs(be), 1e-9):
            return False
        # Gates (c) and (d) come in later tasks.
        return True

    def flush(self):
        if not self._chain:
            return []
        return self._flush_chain()

    def reset(self):
        self._chain = []

    def _flush_chain(self):
        try:
            if len(self._chain) == 1:
                result = self._chain
            else:
                result = [self._build_merged_move(self._chain)]
        except Exception:
            logging.warning(
                "blendprepass: chain cleared after build error (len=%d)",
                len(self._chain),
            )
            raise
        finally:
            self._chain = []
        return result

    def _build_merged_move(self, chain):
        # Real implementation arrives in Task 5; placeholder raises so any
        # unexpected multi-move chain in earlier tasks is visible.
        raise NotImplementedError("merged move construction not yet implemented")
