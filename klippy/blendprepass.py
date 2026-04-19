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
            return self.flush() + [move]
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

    def peek_buffered(self):
        """Read-only view of the currently buffered chain.

        Returns a fresh list copy so callers that mutate the result do not
        corrupt internal state. Part of the filter protocol consumed by
        BlendPipelineLookAheadQueue (sub-spec #4).
        """
        return list(self._chain)

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
        # Aggregate-safety re-check. Per-constituent kin.check_move already
        # validated each segment; this catches aggregate limits such as
        # max_extrude_only_distance that can only be evaluated on the merge.
        if merged.is_kinematic_move:
            self._toolhead.kin.check_move(merged)
        if merged.axes_d[3]:
            self._toolhead.extruder.check_move(merged)
        return merged


class BlendPipelineLookAheadQueue:
    """Generic ordered filter-chain adapter in front of a LookAheadQueue.

    Each filter exposes feed(move) -> list[Move], flush() -> list[Move],
    reset() -> None, peek_buffered() -> list[Move]. On add_move, the
    incoming Move is piped through every filter in order; the survivors
    reach the inner LookAheadQueue. On flush, a two-pass drain flows
    each filter's flush() output through later filters' feed() before
    delivering to the inner queue, then flush()es the inner queue.

    get_last() does NOT drain filters - it peeks via peek_buffered() so
    that callers mutating the returned Move (timing_callbacks,
    limit_next_junction_speed) do not force a premature un-blended
    emission. The emit-time path (_build_merged_move in the prepass,
    _emit_blend in the blender) transfers caller-mutated state onto the
    actually-queued Move so the mutation survives.
    """

    def __init__(self, filters, lookahead):
        self._filters = list(filters)
        self._lookahead = lookahead

    def add_move(self, move):
        acc = [move]
        for f in self._filters:
            acc = [out for m in acc for out in f.feed(m)]
        for m in acc:
            self._lookahead.add_move(m)

    def flush(self, lazy=False):
        acc = []
        for f in self._filters:
            # Invariant: earlier filters may flush moves that must still be
            # seen by later filters' feed() before the later filter itself
            # flushes. First pipe any previous-flush residue through this
            # filter's feed, then append this filter's own flush output.
            acc = [out for m in acc for out in f.feed(m)]
            acc += f.flush()
        for m in acc:
            self._lookahead.add_move(m)
        self._lookahead.flush(lazy=lazy)

    def reset(self):
        for f in self._filters:
            f.reset()
        self._lookahead.reset()

    def set_flush_time(self, flush_time):
        self._lookahead.set_flush_time(flush_time)

    def get_last(self):
        for f in reversed(self._filters):
            buf = f.peek_buffered()
            if buf:
                return buf[-1]
        return self._lookahead.get_last()

    @property
    def queue(self):
        result = []
        for f in self._filters:
            result += f.peek_buffered()
        result += list(self._lookahead.queue)
        return result
