# klippy/blendprepass.py
# Copyright (C) 2026
# This file may be distributed under the terms of the GNU GPLv3 license.
from __future__ import annotations

import logging
import math

from . import blendmath


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
        if len(self._chain) >= self.max_chain:
            emitted = self._flush_chain()
            self._chain = [move]
            return emitted
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
        # Gate (c): perpendicular deviation of every buffered intermediate endpoint
        # from the anchor-to-candidate chord stays within tolerance.
        A = anchor.start_pos[:3]
        B = candidate.end_pos[:3]
        AB = blendmath.vsub(B, A)
        ab_len = blendmath.vnorm(AB)
        if ab_len < self.min_seg_len:
            return False
        for p_move in self._chain:
            P = p_move.end_pos[:3]
            AP = blendmath.vsub(P, A)
            perp_dist = blendmath.vnorm(blendmath.vcross(AP, AB)) / ab_len
            if perp_dist > self.tolerance:
                return False
        # Gate (d): projection bounds — every intermediate endpoint must lie
        # on the AB segment interior (0 <= t <= 1, with eps slack for float noise).
        ab_dot_ab = blendmath.vdot(AB, AB)
        for p_move in self._chain:
            P = p_move.end_pos[:3]
            AP = blendmath.vsub(P, A)
            t = blendmath.vdot(AP, AB) / ab_dot_ab
            if not (-self.t_eps <= t <= 1.0 + self.t_eps):
                return False
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
        start_pos = chain[0].start_pos
        end_pos = chain[-1].end_pos
        cruise_v = math.sqrt(chain[0].max_cruise_v2)
        merged = self._move_cls(self._toolhead, start_pos, end_pos, cruise_v)
        # Pin head-of-chain values so SET_VELOCITY_LIMIT / M204 mid-chain does
        # not leak into the merged Move via Move.__init__'s toolhead snapshot.
        merged.max_cruise_v2 = chain[0].max_cruise_v2
        merged.min_move_t = merged.move_d / cruise_v
        merged.junction_deviation = chain[0].junction_deviation
        # Narrowest accel observed (may have been lowered by a constituent's
        # kin.check_move via limit_speed). limit_speed additionally applies
        # toolhead.max_accel_NEW if M204 was issued mid-chain.
        merged.limit_speed(cruise_v, min(m.accel for m in chain))
        # Preserve chain tail's next-junction cap and all constituent callbacks.
        # Under flush-on-get_last (adapter in Task 12), callbacks only land on
        # chain[-1]; the list-comprehension is defense in depth.
        merged.next_junction_v2 = chain[-1].next_junction_v2
        merged.timing_callbacks = [
            cb for m in chain for cb in m.timing_callbacks
        ]
        return merged
