"""Lane run building and the endpoint ring's admission rules."""

from __future__ import annotations

import pytest

from klippy.plugins.ethercat import wire
from klippy.plugins.ethercat.stream import (
    EthercatError,
    ServoLane,
    SlotRing,
    split_run,
)

CYCLE_NS = 250000
RING_DEPTH = 4096
MIN_LEAD = 40


def run(start, count, flags=0, origin=0, moving=True, interval=CYCLE_NS):
    vel = 10 if moving else 0
    samples = [wire.SetpointSample(i, vel, 0, 0.0) for i in range(count)]
    if not moving:
        flags |= wire.FLAG_TAIL
    return wire.LaneRun(0, 0, flags, origin, start, interval, samples)


def ring():
    return SlotRing(0, CYCLE_NS, RING_DEPTH, MIN_LEAD)


def anchored(now=None):
    r = ring()
    r.admit(run(1000, 100, wire.FLAG_REANCHOR), now)
    return r


def test_abutting_runs_are_admitted():
    r = anchored()
    r.admit(run(1100, 50))
    r.admit(run(1150, 1))
    assert r.next_index == 1151


@pytest.mark.parametrize("start", [1099, 1101])
def test_hole_or_overlap_is_rejected(start):
    r = anchored()
    with pytest.raises(EthercatError, match="ring continues at 1100"):
        r.admit(run(start, 10))


def test_run_before_anchor_is_rejected():
    with pytest.raises(EthercatError, match="anchored"):
        ring().admit(run(0, 10))


def test_origin_shift_inside_epoch_is_rejected():
    r = anchored()
    with pytest.raises(EthercatError, match="origin changed"):
        r.admit(run(1100, 10, origin=1))


def test_interval_mismatch_is_rejected():
    with pytest.raises(EthercatError, match="interval"):
        ring().admit(run(0, 10, wire.FLAG_REANCHOR, interval=CYCLE_NS * 2))


def test_late_run_is_rejected():
    r = anchored(now=1000 - MIN_LEAD)
    with pytest.raises(EthercatError, match="late"):
        r.admit(run(1100, 10), now_index=1100 - MIN_LEAD + 1)


def test_ring_drained_while_moving_is_underrun():
    r = anchored(now=1000 - MIN_LEAD)
    with pytest.raises(EthercatError, match="underrun"):
        r.admit(run(5000, 10, wire.FLAG_REANCHOR), now_index=1100)


def test_ring_drained_after_tail_reanchors():
    r = ring()
    r.admit(run(1000, 100, wire.FLAG_REANCHOR, moving=False), 1000 - MIN_LEAD)
    r.admit(run(5000, 10, wire.FLAG_REANCHOR), now_index=5000 - MIN_LEAD)
    assert r.next_index == 5010


def test_ring_full_is_rejected():
    r = anchored(now=1000 - MIN_LEAD)
    with pytest.raises(EthercatError, match="ring full"):
        r.admit(run(1100, RING_DEPTH), now_index=1000)


def test_split_run_keeps_flags_at_the_ends():
    whole = run(0, 600, wire.FLAG_REANCHOR, moving=False)
    pieces = split_run(whole, wire.MAX_FILL_CYCLES)
    assert [p.start_index for p in pieces] == [0, 256, 512]
    assert [len(p.samples) for p in pieces] == [256, 256, 88]
    assert [p.flags for p in pieces] == [wire.FLAG_REANCHOR, 0, wire.FLAG_TAIL]
    assert sum((p.samples for p in pieces), []) == whole.samples
    r = SlotRing(0, CYCLE_NS, RING_DEPTH, 0)
    for p in pieces:
        r.admit(p)


class FakeServo:
    axis_index = 3
    step = 0.001

    def get_name(self):
        return "stepper_fake"

    def drive_counts(self, pos):
        return round(pos / self.step)

    def drive_velocity(self, vel):
        return round(vel / self.step)


def test_lane_counts_are_relative_to_the_held_position():
    lane = ServoLane(FakeServo(), 2, CYCLE_NS)
    first = lane.build_run(10, [5.0, 5.0], [0.0, 0.0], [0.0, 0.0], 2)
    assert first.flags == wire.FLAG_REANCHOR | wire.FLAG_TAIL
    assert first.origin_mm_q16 == 5 << 16
    assert [s.pos_counts for s in first.samples] == [0, 0]
    second = lane.build_run(12, [5.001, 5.003], [1.0, 2.0], [3.0, 4.0], 2)
    assert second.flags == 0
    assert [s.pos_counts for s in second.samples] == [1, 3]
    assert [s.vel_ff for s in second.samples] == [1000, 2000]
    assert (second.axis_idx, second.slot_idx) == (3, 2)


def test_lane_refuses_to_anchor_while_moving():
    lane = ServoLane(FakeServo(), 0, CYCLE_NS)
    with pytest.raises(EthercatError, match="while moving"):
        lane.build_run(0, [1.0], [0.5], [0.0], 1)
