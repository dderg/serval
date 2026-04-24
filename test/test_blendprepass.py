# test/test_blendprepass.py
import math
import random

import pytest

from klippy import blendprepass


class _FakeCheckMove:
    def __init__(self, exc=None):
        self.calls = []
        self._exc = exc

    def check_move(self, move):
        self.calls.append(move)
        if self._exc is not None:
            raise self._exc


class _FakeToolhead:
    def __init__(self, **overrides):
        self.max_velocity = overrides.get("max_velocity", 500.0)
        self.max_accel = overrides.get("max_accel", 10000.0)
        self.max_jerk = overrides.get("max_jerk", 100000.0)
        self.kin = _FakeCheckMove()
        self.extruder = _FakeCheckMove()


class _FakeMove:
    """Reimplements klippy.toolhead.Move.__init__ without pulling pyserial."""

    def __init__(self, toolhead, start_pos, end_pos, speed):
        self.toolhead = toolhead
        self.start_pos = tuple(start_pos)
        self.end_pos = tuple(end_pos)
        self.accel = toolhead.max_accel
        self.j_max = toolhead.max_jerk
        self.timing_callbacks = []
        velocity = min(speed, toolhead.max_velocity)
        self.is_kinematic_move = True
        axes_d = [end_pos[i] - start_pos[i] for i in (0, 1, 2, 3)]
        self.axes_d = axes_d
        move_d = math.sqrt(sum(d * d for d in axes_d[:3]))
        if move_d < 0.000000001:
            self.end_pos = (
                start_pos[0],
                start_pos[1],
                start_pos[2],
                end_pos[3],
            )
            axes_d[0] = axes_d[1] = axes_d[2] = 0.0
            move_d = abs(axes_d[3])
            inv_move_d = 1.0 / move_d if move_d else 0.0
            self.accel = 99999999.9
            velocity = speed
            self.is_kinematic_move = False
        else:
            inv_move_d = 1.0 / move_d
        self.move_d = move_d
        self.axes_r = [d * inv_move_d for d in axes_d]
        self.min_move_t = move_d / velocity if velocity else 0.0
        self.max_start_v2 = 0.0
        self.max_cruise_v2 = velocity ** 2
        self.next_junction_v2 = 999999999.9

    def limit_speed(self, speed, accel):
        speed2 = speed ** 2
        if speed2 < self.max_cruise_v2:
            self.max_cruise_v2 = speed2
            self.min_move_t = self.move_d / speed if speed else 0.0
        self.accel = min(self.accel, accel)


def _collapser(toolhead=None):
    th = toolhead or _FakeToolhead()
    return blendprepass.CollinearCollapser(th, move_cls=_FakeMove)


def test_construct_and_flush_empty():
    c = _collapser()
    assert c.flush() == []


def test_feed_zero_length_move_passes_through():
    c = _collapser()
    th = c._toolhead
    # Construct a zero-length move directly; Move.__init__ flags it non-kinematic
    # but also gives it move_d=0 which is the step-1 branch we want to exercise.
    zero = _FakeMove(th, (0, 0, 0, 0), (0, 0, 0, 0), speed=100.0)
    assert zero.move_d == 0.0
    out = c.feed(zero)
    assert out == [zero]
    assert c._chain == []


def test_feed_non_kinematic_flushes_and_passes():
    c = _collapser()
    th = c._toolhead
    # Build a non-empty chain first
    m1 = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    c.feed(m1)
    assert c._chain == [m1]
    # E-only move: XYZ identical, E delta present => is_kinematic_move=False
    eonly = _FakeMove(th, (10, 0, 0, 0.5), (10, 0, 0, 1.5), speed=100.0)
    assert eonly.is_kinematic_move is False
    out = c.feed(eonly)
    assert out == [m1, eonly]
    assert c._chain == []


def test_feed_non_kinematic_on_empty_chain_does_not_crash():
    # Regression: a non-kinematic move arriving while the chain is empty
    # (e.g. a pause/G10 E-only retract before any kinematic move has been
    # buffered) must pass through without hitting _build_merged_move([]).
    c = _collapser()
    th = c._toolhead
    eonly = _FakeMove(th, (10, 0, 0, 0.5), (10, 0, 0, 1.5), speed=100.0)
    assert eonly.is_kinematic_move is False
    assert c._chain == []
    out = c.feed(eonly)
    assert out == [eonly]
    assert c._chain == []


def test_first_kinematic_move_starts_chain():
    c = _collapser()
    th = c._toolhead
    m = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    out = c.feed(m)
    assert out == []
    assert c._chain == [m]


def test_flush_singleton_returns_move_unchanged():
    c = _collapser()
    th = c._toolhead
    m = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    c.feed(m)
    out = c.flush()
    assert out == [m]  # single-element chain: identity, not a built merge
    assert c._chain == []


def test_reset_discards_chain():
    c = _collapser()
    th = c._toolhead
    m = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    c.feed(m)
    c.reset()
    assert c._chain == []
    assert c.flush() == []


def test_speed_change_breaks_chain():
    c = _collapser()
    th = c._toolhead
    m1 = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    # Speed differs by 1% (> f_rel=1e-6)
    m2 = _FakeMove(th, (10, 0, 0, 0.5), (20, 0, 0, 1.0), speed=101.0)
    assert c.feed(m1) == []
    out = c.feed(m2)
    # Gate (a) rejects; chain flushes as singleton, m2 starts new chain.
    assert out == [m1]
    assert c._chain == [m2]


def test_flow_change_breaks_chain():
    c = _collapser()
    th = c._toolhead
    # Same speed, same direction; different E-per-mm (> epm_rel=1%)
    m1 = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    m2 = _FakeMove(th, (10, 0, 0, 0.5), (20, 0, 0, 1.1), speed=100.0)
    assert c.feed(m1) == []
    out = c.feed(m2)
    assert out == [m1]
    assert c._chain == [m2]


def test_flow_within_tolerance_does_not_break():
    c = _collapser()
    th = c._toolhead
    # 0.5 mm E vs 0.5005 mm E over same 10 mm XYZ -> 0.1% diff (< epm_rel)
    m1 = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    m2 = _FakeMove(th, (10, 0, 0, 0.5), (20, 0, 0, 1.0005), speed=100.0)
    out = c.feed(m1)
    assert out == []
    # Gate (a) passes: speeds equal. Gate (b) passes: 0.1% < 1%.
    out = c.feed(m2)
    assert out == []
    assert c._chain == [m1, m2]


def test_two_collinear_moves_merge():
    c = _collapser()
    th = c._toolhead
    m1 = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    m2 = _FakeMove(th, (10, 0, 0, 0.5), (20, 0, 0, 1.0), speed=100.0)
    c.feed(m1)
    c.feed(m2)
    out = c.flush()
    assert len(out) == 1
    merged = out[0]
    assert merged is not m1 and merged is not m2
    assert merged.start_pos == (0, 0, 0, 0)
    assert merged.end_pos[:3] == (20.0, 0.0, 0.0)
    assert merged.axes_d[3] == pytest.approx(1.0, abs=1e-12)
    assert merged.move_d == pytest.approx(20.0, abs=1e-12)


def test_non_collinear_moves_do_not_merge():
    c = _collapser()
    th = c._toolhead
    m1 = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    # 1 mm perpendicular offset: well beyond 25 µm tolerance
    m2 = _FakeMove(th, (10, 0, 0, 0.5), (20, 1.0, 0, 1.0), speed=100.0)
    assert c.feed(m1) == []
    out = c.feed(m2)
    assert out == [m1]
    assert c._chain == [m2]


def test_within_tolerance_offset_merges():
    c = _collapser()
    th = c._toolhead
    # 20 µm perpendicular offset from the A-to-C chord — within 25 µm tolerance.
    m1 = _FakeMove(th, (0, 0, 0, 0), (10, 20e-3, 0, 0.5), speed=100.0)
    m2 = _FakeMove(th, (10, 20e-3, 0, 0.5), (20, 0, 0, 1.0), speed=100.0)
    # Chord A(0,0,0)->C(20,0,0). Intermediate B=(10, 20e-3, 0). Perpendicular
    # distance from B to chord = 20 µm.
    assert c.feed(m1) == []
    assert c.feed(m2) == []
    assert c._chain == [m1, m2]


def test_uturn_rejected_by_projection_bounds():
    c = _collapser()
    th = c._toolhead
    # A=(0,0,0) -> B=(10,0,0), candidate ends at (0,0,0): AB length 0 -> rejected.
    m1 = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    m2 = _FakeMove(th, (10, 0, 0, 0.5), (0, 0, 0, 1.0), speed=100.0)
    assert c.feed(m1) == []
    out = c.feed(m2)
    assert out == [m1]
    assert c._chain == [m2]


def test_overshoot_retrace_rejected():
    c = _collapser()
    th = c._toolhead
    # Anchor A=(0,0,0); chain moves to B=(12,0,0) then candidate to C=(10,0,0).
    # Projection t of B onto AC chord = 12/10 = 1.2 -> out of [0,1], reject.
    m1 = _FakeMove(th, (0, 0, 0, 0), (12, 0, 0, 0.6), speed=100.0)
    m2 = _FakeMove(th, (12, 0, 0, 0.6), (10, 0, 0, 0.5), speed=100.0)
    assert c.feed(m1) == []
    out = c.feed(m2)
    assert out == [m1]
    assert c._chain == [m2]


def test_legitimate_extension_passes_projection():
    c = _collapser()
    th = c._toolhead
    m1 = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    m2 = _FakeMove(th, (10, 0, 0, 0.5), (20, 0, 0, 1.0), speed=100.0)
    assert c.feed(m1) == []
    assert c.feed(m2) == []
    # Both buffered; gate (d) allowed the extension.
    assert len(c._chain) == 2


def _build_collinear_chain(toolhead, n, seg_len=1.0, e_per_mm=0.05, speed=100.0):
    moves = []
    for i in range(n):
        start = (i * seg_len, 0, 0, i * seg_len * e_per_mm)
        end = ((i + 1) * seg_len, 0, 0, (i + 1) * seg_len * e_per_mm)
        moves.append(_FakeMove(toolhead, start, end, speed=speed))
    return moves


def test_chain_cap_flushes_at_max():
    c = _collapser()
    th = c._toolhead
    moves = _build_collinear_chain(th, c.max_chain + 1)
    for m in moves[:-1]:
        assert c.feed(m) == []
    assert len(c._chain) == c.max_chain
    out = c.feed(moves[-1])
    assert len(out) == 1
    merged = out[0]
    assert merged.start_pos == (0, 0, 0, 0)
    assert merged.end_pos[:3] == pytest.approx((100.0, 0.0, 0.0), abs=1e-9)
    assert c._chain == [moves[-1]]


def test_merged_pins_max_cruise_v2_to_chain_head():
    # Start with headroom so m1/m2 can both be constructed at speed=100.
    th = _FakeToolhead(max_velocity=1000.0)
    c = blendprepass.CollinearCollapser(th, move_cls=_FakeMove)
    m1 = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    m2 = _FakeMove(th, (10, 0, 0, 0.5), (20, 0, 0, 1.0), speed=100.0)
    c.feed(m1)
    c.feed(m2)
    # Simulate SET_VELOCITY_LIMIT dropping max_velocity below the chain's
    # cruise speed between feed() and flush().
    th.max_velocity = 50.0
    out = c.flush()
    merged = out[0]
    # Without pinning, Move.__init__ would clamp cruise to 50, giving
    # max_cruise_v2 = 2500. With pinning, chain[0].max_cruise_v2 = 10000.
    assert merged.max_cruise_v2 == pytest.approx(m1.max_cruise_v2, rel=1e-12)
    # And min_move_t must reflect the pinned cruise, not the clamped one.
    # merged.move_d = 20; cruise_v = sqrt(10000) = 100; min_move_t = 0.2
    assert merged.min_move_t == pytest.approx(20.0 / 100.0, rel=1e-12)


def test_merged_preserves_minimum_accel_across_chain():
    c = _collapser()
    th = c._toolhead
    m1 = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    m2 = _FakeMove(th, (10, 0, 0, 0.5), (20, 0, 0, 1.0), speed=100.0)
    # Simulate kinematics having applied limit_speed to m2.
    m2.limit_speed(100.0, 3000.0)
    assert m2.accel == 3000.0
    c.feed(m1)
    c.feed(m2)
    out = c.flush()
    merged = out[0]
    assert merged.accel == pytest.approx(3000.0, rel=1e-12)


def test_merged_preserves_next_junction_v2_from_chain_tail():
    c = _collapser()
    th = c._toolhead
    m1 = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    m2 = _FakeMove(th, (10, 0, 0, 0.5), (20, 0, 0, 1.0), speed=100.0)
    m2.next_junction_v2 = 12345.0  # limit_next_junction_speed was called on tail
    c.feed(m1)
    c.feed(m2)
    out = c.flush()
    assert out[0].next_junction_v2 == pytest.approx(12345.0, rel=1e-12)


def test_merged_concatenates_timing_callbacks():
    c = _collapser()
    th = c._toolhead

    def cb1(t):
        return None

    def cb2(t):
        return None

    m1 = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    m2 = _FakeMove(th, (10, 0, 0, 0.5), (20, 0, 0, 1.0), speed=100.0)
    # Under flush-on-get_last, callbacks should only ever land on chain[-1].
    # Defense in depth: also preserve if an earlier constituent carried them.
    m1.timing_callbacks.append(cb1)
    m2.timing_callbacks.append(cb2)
    c.feed(m1)
    c.feed(m2)
    out = c.flush()
    assert out[0].timing_callbacks == [cb1, cb2]


def test_post_merge_kin_check_move_runs():
    c = _collapser()
    th = c._toolhead
    m1 = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    m2 = _FakeMove(th, (10, 0, 0, 0.5), (20, 0, 0, 1.0), speed=100.0)
    c.feed(m1)
    c.feed(m2)
    out = c.flush()
    # Exactly one post-merge check: on the merged Move itself.
    assert len(th.kin.calls) == 1
    assert th.kin.calls[0] is out[0]


def test_post_merge_extruder_check_runs_only_when_e_delta_nonzero():
    c = _collapser()
    th = c._toolhead
    # Pure-travel chain: axes_d[3] == 0 on both, merged axes_d[3] == 0
    m1 = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0), speed=100.0)
    m2 = _FakeMove(th, (10, 0, 0, 0), (20, 0, 0, 0), speed=100.0)
    c.feed(m1)
    c.feed(m2)
    c.flush()
    assert th.extruder.calls == []

    # With extrusion:
    th2 = _FakeToolhead()
    c2 = blendprepass.CollinearCollapser(th2, move_cls=_FakeMove)
    m3 = _FakeMove(th2, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    m4 = _FakeMove(th2, (10, 0, 0, 0.5), (20, 0, 0, 1.0), speed=100.0)
    c2.feed(m3)
    c2.feed(m4)
    c2.flush()
    assert len(th2.extruder.calls) == 1


def test_post_merge_check_skipped_for_singleton_chain():
    # Singletons skip _build_merged_move entirely (pass through identity),
    # so no post-merge check fires. This preserves per-move check_move behavior.
    c = _collapser()
    th = c._toolhead
    m = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    c.feed(m)
    c.flush()
    assert th.kin.calls == []
    assert th.extruder.calls == []


class _RaisingKin:
    def check_move(self, move):
        raise RuntimeError("kin limit violation")


def test_exception_in_merged_check_clears_chain(caplog):
    th = _FakeToolhead()
    th.kin = _RaisingKin()
    c = blendprepass.CollinearCollapser(th, move_cls=_FakeMove)
    m1 = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    m2 = _FakeMove(th, (10, 0, 0, 0.5), (20, 0, 0, 1.0), speed=100.0)
    c.feed(m1)
    c.feed(m2)
    with caplog.at_level("WARNING"):
        with pytest.raises(RuntimeError, match="kin limit violation"):
            c.flush()
    assert c._chain == []
    assert any("blendprepass: chain cleared" in r.message for r in caplog.records)


def test_feed_after_exception_starts_clean():
    th = _FakeToolhead()
    th.kin = _RaisingKin()
    c = blendprepass.CollinearCollapser(th, move_cls=_FakeMove)
    m1 = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    m2 = _FakeMove(th, (10, 0, 0, 0.5), (20, 0, 0, 1.0), speed=100.0)
    c.feed(m1)
    c.feed(m2)
    with pytest.raises(RuntimeError):
        c.flush()
    # After exception, chain is empty; next feed starts a fresh chain of size 1.
    m3 = _FakeMove(th, (20, 0, 0, 1.0), (30, 0, 0, 1.5), speed=100.0)
    assert c.feed(m3) == []
    assert c._chain == [m3]


class _FakeInnerQueue:
    def __init__(self):
        self.queue = []
        self.flush_calls = []
        self.reset_calls = 0
        self.set_flush_time_calls = []

    def add_move(self, move):
        self.queue.append(move)

    def flush(self, lazy=False):
        self.flush_calls.append(lazy)

    def reset(self):
        self.reset_calls += 1
        self.queue = []

    def set_flush_time(self, t):
        self.set_flush_time_calls.append(t)

    def get_last(self):
        return self.queue[-1] if self.queue else None


def test_adapter_add_move_routes_through_prepass():
    th = _FakeToolhead()
    c = blendprepass.CollinearCollapser(th, move_cls=_FakeMove)
    inner = _FakeInnerQueue()
    adapter = blendprepass.BlendPipelineLookAheadQueue([c], inner)

    m1 = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    m2 = _FakeMove(th, (10, 0, 0, 0.5), (20, 0, 0, 1.0), speed=100.0)
    adapter.add_move(m1)
    adapter.add_move(m2)
    # Buffered in prepass; inner queue still empty.
    assert inner.queue == []
    assert c._chain == [m1, m2]


def test_adapter_flush_drains_and_forwards_lazy_flag():
    th = _FakeToolhead()
    c = blendprepass.CollinearCollapser(th, move_cls=_FakeMove)
    inner = _FakeInnerQueue()
    adapter = blendprepass.BlendPipelineLookAheadQueue([c], inner)

    m1 = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    m2 = _FakeMove(th, (10, 0, 0, 0.5), (20, 0, 0, 1.0), speed=100.0)
    adapter.add_move(m1)
    adapter.add_move(m2)
    adapter.flush(lazy=True)
    # Chain emitted into inner queue, then inner flushed with lazy=True.
    assert len(inner.queue) == 1
    assert inner.flush_calls == [True]
    assert c._chain == []


def test_adapter_reset_discards_chain_and_resets_inner():
    th = _FakeToolhead()
    c = blendprepass.CollinearCollapser(th, move_cls=_FakeMove)
    inner = _FakeInnerQueue()
    adapter = blendprepass.BlendPipelineLookAheadQueue([c], inner)

    m = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    adapter.add_move(m)
    adapter.reset()
    assert c._chain == []
    assert inner.reset_calls == 1


def test_adapter_set_flush_time_passes_through():
    th = _FakeToolhead()
    c = blendprepass.CollinearCollapser(th, move_cls=_FakeMove)
    inner = _FakeInnerQueue()
    adapter = blendprepass.BlendPipelineLookAheadQueue([c], inner)

    adapter.set_flush_time(2.0)
    assert inner.set_flush_time_calls == [2.0]


def test_adapter_get_last_peeks_without_flushing_prepass():
    th = _FakeToolhead()
    c = blendprepass.CollinearCollapser(th, move_cls=_FakeMove)
    inner = _FakeInnerQueue()
    adapter = blendprepass.BlendPipelineLookAheadQueue([c], inner)

    m1 = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    m2 = _FakeMove(th, (10, 0, 0, 0.5), (20, 0, 0, 1.0), speed=100.0)
    adapter.add_move(m1)
    adapter.add_move(m2)
    # Before get_last, chain is buffered, inner queue empty.
    assert inner.queue == []
    last = adapter.get_last()
    # get_last peeks; the prepass chain is STILL buffered (not flushed).
    assert inner.queue == []
    assert c._chain == [m1, m2]
    # Returned move is the tail of the buffered chain.
    assert last is m2


def test_adapter_queue_property_reports_buffered_moves():
    th = _FakeToolhead()
    c = blendprepass.CollinearCollapser(th, move_cls=_FakeMove)
    inner = _FakeInnerQueue()
    adapter = blendprepass.BlendPipelineLookAheadQueue([c], inner)

    # Empty state: queue is empty.
    assert not adapter.queue

    m1 = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    m2 = _FakeMove(th, (10, 0, 0, 0.5), (20, 0, 0, 1.0), speed=100.0)
    adapter.add_move(m1)
    adapter.add_move(m2)
    # Buffered in prepass but inner still empty — queue property reflects
    # buffered pending work.
    assert adapter.queue
    assert len(adapter.queue) == 2


@pytest.mark.parametrize("seed", range(50))
def test_random_collinear_chain_merges(seed):
    rng = random.Random(seed)
    th = _FakeToolhead()
    c = blendprepass.CollinearCollapser(th, move_cls=_FakeMove)
    # Random unit direction in a plane (z=0 for simplicity of the noise model)
    theta = rng.uniform(0.0, 2 * math.pi)
    ux, uy = math.cos(theta), math.sin(theta)
    # Perpendicular direction in the plane
    px, py = -uy, ux
    n = rng.randint(2, 100)
    anchor = (0.0, 0.0, 0.0, 0.0)
    cursor = anchor
    for _ in range(n):
        seg_len = rng.uniform(0.01, 10.0)
        noise = rng.uniform(-20e-6, 20e-6)  # 20 µm well under 25 µm tolerance
        nx = cursor[0] + ux * seg_len + px * noise
        ny = cursor[1] + uy * seg_len + py * noise
        e_delta = seg_len * 0.05
        end = (nx, ny, 0.0, cursor[3] + e_delta)
        m = _FakeMove(th, cursor, end, speed=100.0)
        c.feed(m)
        cursor = end
    out = c.flush()
    assert len(out) == 1, f"seed {seed}: expected 1 merged move, got {len(out)}"


@pytest.mark.parametrize("seed", range(50))
def test_random_chain_splits_at_violation(seed):
    rng = random.Random(seed)
    th = _FakeToolhead()
    c = blendprepass.CollinearCollapser(th, move_cls=_FakeMove)
    n_before = rng.randint(2, 50)
    moves = _build_collinear_chain(th, n_before)
    for m in moves:
        c.feed(m)
    # Now an offset-violating move: 50 µm perpendicular offset > 25 µm tolerance.
    last_end = moves[-1].end_pos
    violator_end = (last_end[0] + 1.0, last_end[1] + 50e-3, 0.0, last_end[3] + 0.05)
    violator = _FakeMove(th, last_end, violator_end, speed=100.0)
    out = c.feed(violator)
    # First output: the merged prior chain.
    assert len(out) == 1
    # Violator started a fresh chain.
    assert c._chain == [violator]


@pytest.mark.parametrize("seed", range(50))
def test_total_displacement_preserved(seed):
    rng = random.Random(seed)
    th = _FakeToolhead()
    c = blendprepass.CollinearCollapser(th, move_cls=_FakeMove)
    n = rng.randint(2, 50)
    moves = _build_collinear_chain(th, n)
    for m in moves:
        c.feed(m)
    out = c.flush()
    merged = out[0]
    for i in range(4):
        expected = sum(m.axes_d[i] for m in moves)
        assert merged.axes_d[i] == pytest.approx(expected, abs=1e-9)


def test_merged_has_fresh_lookahead_invariants():
    # Spec test #15: merged Move must carry fresh lookahead state, not
    # inherited/leaked values from any constituent.
    c = _collapser()
    th = c._toolhead
    m1 = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    m2 = _FakeMove(th, (10, 0, 0, 0.5), (20, 0, 0, 1.0), speed=100.0)
    # Simulate lookahead having partially planned a constituent before merge.
    m1.max_start_v2 = 1234.5
    c.feed(m1)
    c.feed(m2)
    out = c.flush()
    merged = out[0]
    assert merged.max_start_v2 == 0.0


def test_post_merge_kin_check_can_reject_aggregate():
    # Spec test #23: the post-merge check_move re-run catches limits that
    # constituents individually pass but the aggregate violates.
    class _AggregateRejectKin:
        # Rejects any move with XYZ travel over 15 mm — constituents at
        # 10 mm each pass; merged at 20 mm fails.
        def check_move(self, move):
            if move.move_d > 15.0:
                raise RuntimeError("aggregate XYZ travel exceeded")

    th = _FakeToolhead()
    th.kin = _AggregateRejectKin()
    c = blendprepass.CollinearCollapser(th, move_cls=_FakeMove)
    m1 = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    m2 = _FakeMove(th, (10, 0, 0, 0.5), (20, 0, 0, 1.0), speed=100.0)
    # Constituents individually have move_d=10, pass.
    th.kin.check_move(m1)
    th.kin.check_move(m2)
    # Merge must fail post-build.
    c.feed(m1)
    c.feed(m2)
    with pytest.raises(RuntimeError, match="aggregate XYZ travel exceeded"):
        c.flush()


def test_gate_d_t_eps_tolerance_allows_ulp_noise():
    # Spec test #24: gate (d) projection allows float-noise slack.
    # A legitimate extension of a collinear chain should not be rejected
    # by a sub-ulp t value > 1.0.
    c = _collapser()
    th = c._toolhead
    # Build a chain where the intermediate endpoint is at coordinate that,
    # after subtractive-cancellation in `t = (AP·AB)/(AB·AB)`, is bit-equal
    # to 1.0. Using 10 mm + 10 mm segments makes t = 1.0 exact for the
    # first intermediate if we use the chord A(0,0,0) -> B(20,0,0).
    # Constructing a scenario where ulp noise makes t slightly > 1:
    m1 = _FakeMove(th, (0, 0, 0, 0), (10.0, 0.0, 0.0, 0.5), speed=100.0)
    m2 = _FakeMove(th, (10.0, 0.0, 0.0, 0.5), (20.0, 0.0, 0.0, 1.0), speed=100.0)
    assert c.feed(m1) == []
    assert c.feed(m2) == []
    assert len(c._chain) == 2

    # Now construct a t value clearly outside slack: intermediate at
    # t = 1 + 100*ulp, which at coord 10 on a chord of 20 is ~2e-12 over.
    # We need an offset large enough to definitely exceed t_eps=1e-9, so
    # push the intermediate slightly past the chord end.
    c.reset()
    m1 = _FakeMove(th, (0, 0, 0, 0), (10.0 + 1e-6, 0.0, 0.0, 0.5), speed=100.0)
    m2 = _FakeMove(th, (10.0 + 1e-6, 0.0, 0.0, 0.5),
                   (10.0, 0.0, 0.0, 1.0), speed=100.0)
    # Chord A(0,0,0)->B(10,0,0). Intermediate at (10+1e-6, 0, 0) has
    # t = (10+1e-6)/10 = 1 + 1e-7, outside t_eps = 1e-9 slack -> reject.
    assert c.feed(m1) == []
    out = c.feed(m2)
    assert out == [m1]  # chain flushed as singleton
    assert c._chain == [m2]


def test_chain_cap_consecutive_flushes_stay_collinear():
    # Spec test #25: two successive chain-cap flushes should produce two
    # merged moves whose axes_r directions match exactly (collinear). This
    # pins the invariant that a vase-mode print spanning >100 segments
    # does NOT hit a cornering penalty at the cap boundary.
    c = _collapser()
    th = c._toolhead
    # Build 200 perfectly collinear moves of length 0.5 mm each along +X.
    moves = _build_collinear_chain(th, 200, seg_len=0.5)
    emitted = []
    for m in moves:
        emitted.extend(c.feed(m))
    emitted.extend(c.flush())
    # Exactly two merged moves (first 100 chunk + second 100 chunk).
    assert len(emitted) == 2
    first, second = emitted
    # Both span 50 mm (100 * 0.5) along +X.
    assert first.move_d == pytest.approx(50.0, abs=1e-9)
    assert second.move_d == pytest.approx(50.0, abs=1e-9)
    # axes_r direction vectors must match bit-for-bit (collinear invariant).
    for i in range(3):
        assert first.axes_r[i] == pytest.approx(second.axes_r[i], abs=1e-12)
    # Chord continuity: first ends where second begins.
    assert first.end_pos[:3] == second.start_pos[:3]


def test_peek_buffered_returns_chain_copy():
    c = _collapser()
    th = c._toolhead
    assert c.peek_buffered() == []
    m1 = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    m2 = _FakeMove(th, (10, 0, 0, 0.5), (20, 0, 0, 1.0), speed=100.0)
    c.feed(m1)
    c.feed(m2)
    buf = c.peek_buffered()
    assert buf == [m1, m2]
    # Mutation of the returned list must not affect internal state.
    buf.append("garbage")
    assert c._chain == [m1, m2]
    # Subsequent feed must still work.
    m3 = _FakeMove(th, (20, 0, 0, 1.0), (30, 0, 0, 1.5), speed=100.0)
    assert c.feed(m3) == []
    assert c._chain == [m1, m2, m3]
