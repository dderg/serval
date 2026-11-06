# Turning servo samples into abutting endpoint lane runs
#
# Copyright (C) 2026  Kalico contributors
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from __future__ import annotations

import dataclasses

from . import wire

Q16 = 1 << 16


class EthercatError(Exception):
    pass


def split_run(run: wire.LaneRun, max_len: int) -> list[wire.LaneRun]:
    """Split a lane run into consecutive runs of at most max_len samples.
    REANCHOR stays on the first piece and TAIL on the last."""
    pieces = []
    for begin in range(0, len(run.samples), max_len):
        samples = run.samples[begin : begin + max_len]
        flags = run.flags & ~(wire.FLAG_REANCHOR | wire.FLAG_TAIL)
        if not begin:
            flags |= run.flags & wire.FLAG_REANCHOR
        if begin + len(samples) == len(run.samples):
            flags |= run.flags & wire.FLAG_TAIL
        pieces.append(
            dataclasses.replace(
                run,
                flags=flags,
                start_index=run.start_index + begin,
                samples=samples,
            )
        )
    return pieces


class ServoLane:
    """One servo's anchor epoch: converts sampled motion to drive units.

    The endpoint latches each epoch's drive origin from the target it is
    holding, so a run's pos_counts are counts relative to the position held
    when the epoch started. Re-anchoring is only valid while stationary.
    """

    def __init__(self, servo, slot_idx, cycle_ns):
        self.servo = servo
        self.slot_idx = slot_idx
        self.cycle_ns = cycle_ns
        self.held_counts = None
        self.origin_mm_q16 = None
        self.last_motion_index = None

    def reanchor(self):
        """Start a new epoch at the next run; only valid while holding."""
        self.held_counts = None

    def build_run(self, start_index, pos, vel, acc, count) -> wire.LaneRun:
        servo = self.servo
        flags = 0
        if self.held_counts is None:
            if vel[0] or acc[0]:
                raise EthercatError(
                    "%s: cannot anchor at index %d while moving"
                    " (velocity %r, accel %r)"
                    % (servo.get_name(), start_index, vel[0], acc[0])
                )
            self.held_counts = servo.drive_counts(pos[0])
            self.origin_mm_q16 = round_half_away(pos[0] * Q16)
            flags |= wire.FLAG_REANCHOR
        held = self.held_counts
        samples = [
            wire.SetpointSample(
                servo.drive_counts(pos[i]) - held,
                servo.drive_velocity(vel[i]),
                0,
                acc[i],
            )
            for i in range(count)
        ]
        moving = [i for i in range(count) if vel[i] or acc[i]]
        if moving:
            self.last_motion_index = start_index + moving[-1]
        if not moving or moving[-1] != count - 1:
            flags |= wire.FLAG_TAIL
        return wire.LaneRun(
            axis_idx=servo.axis_index,
            slot_idx=self.slot_idx,
            flags=flags,
            origin_mm_q16=self.origin_mm_q16,
            start_index=start_index,
            interval_ticks=self.cycle_ns,
            samples=samples,
        )


def round_half_away(value):
    if value >= 0.0:
        return int(value + 0.5)
    return int(value - 0.5)


class SlotRing:
    """Host-side mirror of one endpoint slot ring's admission rules.

    now_index is the endpoint's current play index, or None in batch mode
    where nothing plays in real time. Every violation raises: the endpoint
    would latch the same fault, and padding or clamping would hide motion
    that was lost.
    """

    def __init__(self, slot_idx, cycle_ns, ring_depth_cycles, min_lead_cycles):
        self.slot_idx = slot_idx
        self.cycle_ns = cycle_ns
        self.ring_depth_cycles = ring_depth_cycles
        self.min_lead_cycles = min_lead_cycles
        self.next_index = None
        self.origin_mm_q16 = None
        self.tail = True

    def admit(self, run: wire.LaneRun, now_index=None):
        name = "slot %d" % (self.slot_idx,)
        if run.interval_ticks != self.cycle_ns:
            raise EthercatError(
                "%s: run interval %d ns != cycle %d ns"
                % (name, run.interval_ticks, self.cycle_ns)
            )
        if now_index is not None and self.next_index is not None:
            if now_index >= self.next_index and not self.tail:
                raise EthercatError(
                    "%s: underrun, ring drained at index %d while moving"
                    " (play index %d)" % (name, self.next_index, now_index)
                )
        if run.flags & wire.FLAG_REANCHOR:
            self.origin_mm_q16 = run.origin_mm_q16
        elif self.next_index is None:
            raise EthercatError("%s: run before the ring was anchored" % name)
        elif run.start_index != self.next_index:
            raise EthercatError(
                "%s: run starts at index %d, ring continues at %d"
                % (name, run.start_index, self.next_index)
            )
        elif run.origin_mm_q16 != self.origin_mm_q16:
            raise EthercatError(
                "%s: origin changed from %d to %d inside an anchor epoch"
                % (name, self.origin_mm_q16, run.origin_mm_q16)
            )
        if now_index is not None:
            lead = run.start_index - now_index
            if lead < self.min_lead_cycles:
                raise EthercatError(
                    "%s: run late, starts at index %d with %d cycles of lead"
                    " (%d required)"
                    % (name, run.start_index, lead, self.min_lead_cycles)
                )
            if run.end_index - now_index > self.ring_depth_cycles:
                raise EthercatError(
                    "%s: ring full, run ends %d cycles ahead of play index"
                    " (ring holds %d)"
                    % (name, run.end_index - now_index, self.ring_depth_cycles)
                )
        self.next_index = run.end_index
        self.tail = bool(run.flags & wire.FLAG_TAIL)
