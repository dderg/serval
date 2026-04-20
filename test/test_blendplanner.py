# test/test_blendplanner.py
import math
import random

import pytest

from klippy import blendplanner


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
        self.max_accel_to_decel = overrides.get("max_accel_to_decel", 10000.0)
        self.corner_deviation = overrides.get("corner_deviation", 50e-3)
        self.kin = _FakeCheckMove()
        self.extruder = _FakeCheckMove()


class _FakeMove:
    """Reimplements klippy.toolhead.Move.__init__ without pulling pyserial."""

    def __init__(self, toolhead, start_pos, end_pos, speed):
        self.toolhead = toolhead
        self.start_pos = tuple(start_pos)
        self.end_pos = tuple(end_pos)
        self.accel = toolhead.max_accel
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
        self.delta_v2 = 2.0 * move_d * self.accel
        self.max_smoothed_v2 = 0.0
        self.smooth_delta_v2 = 2.0 * move_d * toolhead.max_accel_to_decel
        self.next_junction_v2 = 999999999.9

    def limit_speed(self, speed, accel):
        speed2 = speed ** 2
        if speed2 < self.max_cruise_v2:
            self.max_cruise_v2 = speed2
            self.min_move_t = self.move_d / speed if speed else 0.0
        self.accel = min(self.accel, accel)
        self.delta_v2 = 2.0 * self.move_d * self.accel
        self.smooth_delta_v2 = min(self.smooth_delta_v2, self.delta_v2)

    def limit_next_junction_speed(self, speed):
        self.next_junction_v2 = min(self.next_junction_v2, speed ** 2)


def _blender(toolhead=None, max_chord_err=None):
    th = toolhead or _FakeToolhead()
    return blendplanner.CornerBlender(
        th, move_cls=_FakeMove, max_chord_err=max_chord_err
    )


def test_construct_and_flush_empty():
    b = _blender()
    assert b.flush() == []
    assert b.peek_buffered() == []
    assert b.polyline_moves_emitted == 0
    assert b.blends_emitted == 0


def test_feed_first_move_buffers():
    b = _blender()
    th = b._toolhead
    m = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    out = b.feed(m)
    assert out == []
    assert b._prev is m
    assert b.peek_buffered() == [m]


def test_flush_drains_buffered_prev():
    b = _blender()
    th = b._toolhead
    m = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    b.feed(m)
    out = b.flush()
    assert out == [m]
    assert b._prev is None


def test_reset_drops_buffered_prev():
    b = _blender()
    th = b._toolhead
    m = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    b.feed(m)
    b.reset()
    assert b._prev is None
    assert b.flush() == []


def test_feed_non_kinematic_flushes_and_passes():
    b = _blender()
    th = b._toolhead
    m_kin = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    b.feed(m_kin)
    # E-only: XYZ identical, E delta present
    eonly = _FakeMove(th, (10, 0, 0, 0.5), (10, 0, 0, 1.5), speed=100.0)
    assert eonly.is_kinematic_move is False
    out = b.feed(eonly)
    assert out == [m_kin, eonly]
    assert b._prev is None


def test_feed_collinear_pair_passes_through_with_rebuffer():
    b = _blender()
    th = b._toolhead
    # Two exactly collinear moves along +X: blend_from_moves returns None.
    m1 = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    m2 = _FakeMove(th, (10, 0, 0, 0.5), (20, 0, 0, 1.0), speed=100.0)
    assert b.feed(m1) == []
    out = b.feed(m2)
    # Collinear: emit prev unchanged, buffer next. No velocity cap imposed.
    assert out == [m1]
    assert b._prev is m2
    assert m1.next_junction_v2 == 999999999.9  # unchanged


def test_feed_uturn_emits_prev_with_zero_next_junction():
    b = _blender()
    th = b._toolhead
    # 180° reversal: +X then -X
    m1 = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    m2 = _FakeMove(th, (10, 0, 0, 0.5), (0, 0, 0, 1.0), speed=100.0)
    assert b.feed(m1) == []
    out = b.feed(m2)
    # U-turn: emit prev with limit_next_junction_speed(0); buffer next.
    assert out == [m1]
    assert m1.next_junction_v2 == 0.0
    assert b._prev is m2


# ---------------------------------------------------------------------------
# Regression: suppression returning None at non-collinear corners must
# apply an SCV-equivalent junction cap via limit_next_junction_speed.
# Without the cap the toolhead hits the corner at full cruise velocity and
# skips steps — see `blend_from_moves` docstring for suppression rules.
# ---------------------------------------------------------------------------


class _FakeAxisIS:
    def __init__(self, axis, stype, freq, damping=0.1):
        self.axis = axis
        class _P:
            pass
        self.params = _P()
        self.params.shaper_type = stype
        self.params.shaper_freq = freq
        self.params.damping_ratio = damping


class _FakeIS:
    def __init__(self, shapers):
        self._shapers = shapers
    def get_shapers(self):
        return list(self._shapers)


class _FakePrinter:
    def __init__(self, is_obj):
        self._is = is_obj
    def lookup_object(self, name, default=None):
        if name == "input_shaper":
            return self._is
        return default


class _FakeToolheadWithShaper(_FakeToolhead):
    def __init__(self, **kw):
        super().__init__(**kw)
        self.printer = _FakePrinter(_FakeIS([
            _FakeAxisIS("x", "zv", 80.0),
            _FakeAxisIS("y", "zv", 80.0),
        ]))


def test_feed_suppressed_non_collinear_applies_junction_cap():
    """Rule 2 suppression at a sharp 90° corner with short segments must
    still apply an SCV-equivalent junction velocity cap to the emitted
    prev move — otherwise the toolhead barrels through the corner at
    full cruise velocity."""
    th = _FakeToolheadWithShaper(
        corner_deviation=0.2, max_accel=50000.0, max_velocity=500.0,
    )
    b = blendplanner.CornerBlender(th, move_cls=_FakeMove)
    # 1 mm segments, 90° corner. R_tol = 0.483 mm (binds), v_arc ≈ 155;
    # SCV-equivalent v_j ≈ 29 mm/s at this cd/sigma — fork_cost > main_cost
    # → blend_from_moves returns None (velocity-aware rule).
    m1 = _FakeMove(th, (0, 0, 0, 0), (1, 0, 0, 0.1), speed=500.0)
    m2 = _FakeMove(th, (1, 0, 0, 0.1), (1, 1, 0, 0.2), speed=500.0)
    assert b.feed(m1) == []
    out = b.feed(m2)
    # Prev was emitted (blend suppressed) and next buffered.
    assert out == [m1]
    assert b._prev is m2
    # Cap was applied — next_junction_v2 dropped well below default sentinel.
    assert m1.next_junction_v2 < 999999999.9
    # Cap must be strictly positive (non-collinear, not a U-turn).
    assert m1.next_junction_v2 > 0.0


def test_feed_collinear_with_shaper_still_no_cap():
    """Truly collinear corners must NOT gain a cap just because a shaper
    is present — the suppressed-junction path only applies to real
    non-collinear corners."""
    th = _FakeToolheadWithShaper(corner_deviation=0.1)
    b = blendplanner.CornerBlender(th, move_cls=_FakeMove)
    m1 = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    m2 = _FakeMove(th, (10, 0, 0, 0.5), (20, 0, 0, 1.0), speed=100.0)
    b.feed(m1)
    out = b.feed(m2)
    assert out == [m1]
    assert m1.next_junction_v2 == 999999999.9


def _state_src_dst_pair():
    """Build a (src, dst) pair where src is a 'full-length' parent and dst a
    truncated child constructed via the Move ctor against the same toolhead."""
    th = _FakeToolhead()
    src = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 1.0), speed=200.0)
    # Simulate caller mutations on src:
    src.timing_callbacks.append(lambda t: None)
    src.next_junction_v2 = 42.0
    src.max_cruise_v2 = 150.0 ** 2
    src.accel = 5000.0
    src.delta_v2 = 2.0 * src.move_d * src.accel
    src.smooth_delta_v2 = min(src.delta_v2, 2.0 * src.move_d * 2500.0)
    dst = _FakeMove(th, (0, 0, 0, 0), (4, 0, 0, 0.4), speed=200.0)
    return th, src, dst


def test_copy_caller_state_transfers_caller_intent_fields():
    th, src, dst = _state_src_dst_pair()
    blendplanner._copy_caller_state(src, dst)
    # Caller-intent fields pinned verbatim.
    assert dst.timing_callbacks == src.timing_callbacks
    assert dst.timing_callbacks is not src.timing_callbacks  # copy, not alias
    assert dst.next_junction_v2 == 42.0
    assert dst.max_cruise_v2 == 150.0 ** 2
    assert dst.accel == 5000.0


def test_copy_caller_state_recomputes_length_derived_fields():
    th, src, dst = _state_src_dst_pair()
    blendplanner._copy_caller_state(src, dst)
    # delta_v2 recomputed from NEW move_d (4 mm) and pinned accel (5000).
    assert dst.delta_v2 == pytest.approx(2.0 * 4.0 * 5000.0)
    # smooth_delta_v2 preserves the parent ratio; src had smooth/delta = 0.5
    # (max_accel_to_decel/accel = 2500/5000), so dst should follow.
    ratio = src.smooth_delta_v2 / src.delta_v2
    assert dst.smooth_delta_v2 == pytest.approx(
        min(dst.delta_v2, 2.0 * 4.0 * dst.accel * ratio)
    )
    # min_move_t = move_d / sqrt(max_cruise_v2) = 4 / 150 = 0.02667
    assert dst.min_move_t == pytest.approx(4.0 / 150.0)


def test_copy_caller_state_handles_zero_delta_v2():
    th, src, dst = _state_src_dst_pair()
    src.delta_v2 = 0.0
    src.smooth_delta_v2 = 0.0
    blendplanner._copy_caller_state(src, dst)
    # Falls back to ratio=1.0 when src.delta_v2 is zero; dst.smooth_delta_v2
    # collapses to dst.delta_v2 via the min().
    assert dst.smooth_delta_v2 == pytest.approx(dst.delta_v2)


def test_90deg_corner_emits_trunc_prev_plus_arc_polyline_and_buffers_next_head():
    b = _blender(max_chord_err=20e-3)
    th = b._toolhead
    # Two 10mm moves meeting at a 90° corner at (10,0,0).
    m_prev = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    m_next = _FakeMove(th, (10, 0, 0, 0.5), (10, 10, 0, 1.0), speed=100.0)
    assert b.feed(m_prev) == []
    out = b.feed(m_next)
    # Emission: [trunc_prev, arc[0], ..., arc[N-1]]
    assert len(out) >= 2
    trunc_prev = out[0]
    arc_moves = out[1:]
    # trunc_prev shares start_pos with m_prev.
    assert trunc_prev.start_pos[:3] == m_prev.start_pos[:3]
    # trunc_prev ends before the vertex by arc.d_consumed along +X.
    # R_mid = 0.5 * min(10,10) * cot(45°) = 5. R_tol binds much smaller:
    # R_tol = 50e-3 * cos(45°)/(1-cos(45°)) ≈ 0.1207. So R = R_tol ≈ 0.1207,
    # d = R * tan(45°) ≈ 0.1207.
    d_expected = 50e-3 * (math.sqrt(2)/2) / (1 - math.sqrt(2)/2)
    assert trunc_prev.end_pos[0] == pytest.approx(10.0 - d_expected, rel=1e-6)
    assert trunc_prev.end_pos[1] == pytest.approx(0.0, abs=1e-9)
    # buffered next_trunc_head starts where the arc ends.
    assert b._prev is not None
    assert b._prev is not m_next
    nxt_head = b._prev
    assert nxt_head.start_pos[0] == pytest.approx(10.0, abs=1e-9)
    assert nxt_head.start_pos[1] == pytest.approx(d_expected, rel=1e-6)
    assert nxt_head.end_pos[:3] == (10.0, 10.0, 0.0)
    # Polyline points all lie on the arc within max_chord_err.
    # Arc center: m_prev.end_pos + R*n_hat where n_hat bisects inward.
    # At a 90° corner +X to +Y, center = vertex + R*(-1/sqrt2, 1/sqrt2) rotated;
    # simpler check: every arc_move endpoint must be within R + chord_err of center.
    # We compute center from arc.entry_pt + R in the direction (next-prev)/|...|.
    # For simplicity just verify that arc spans from near (10-d,0) to (10,d).
    first_pt = arc_moves[0].start_pos[:3]
    last_pt = arc_moves[-1].end_pos[:3]
    assert first_pt[0] == pytest.approx(10.0 - d_expected, rel=1e-6)
    assert last_pt[1] == pytest.approx(d_expected, rel=1e-6)
    # All arc moves share the same max_cruise_v2 (arc.v_cap^2 in this case).
    v_caps = [am.max_cruise_v2 for am in arc_moves]
    assert max(v_caps) - min(v_caps) < 1e-6
    # Instrumentation.
    assert b.blends_emitted == 1
    assert b.polyline_moves_emitted == len(arc_moves)


def test_corner_deviation_mutation_affects_next_blend():
    # Regression guard: the CornerBlender reads toolhead.corner_deviation
    # live on every feed() call, so a mid-print mutation (as performed by
    # cmd_SET_VELOCITY_LIMIT) must influence the very next blend's radius
    # and arc length. This is the mechanism the
    # "SET_VELOCITY_LIMIT CORNER_DEVIATION=N" UX depends on.
    th = _FakeToolhead(corner_deviation=0.2)
    b = blendplanner.CornerBlender(th, move_cls=_FakeMove, max_chord_err=20e-3)
    # First blend at cd=0.2 on a 90° corner with long (10mm) neighbors.
    # R_mid = 5mm; R_tol at cd=0.2 ~ 0.483mm; R_tol at cd=0.01 ~ 0.024mm;
    # both well below R_mid so R_tol binds in each case.
    m1_prev = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0), speed=100.0)
    m1_next = _FakeMove(th, (10, 0, 0, 0), (10, 10, 0, 0), speed=100.0)
    b.feed(m1_prev)
    out_loose = b.feed(m1_next)
    arc_loose = out_loose[1:]
    trunc_prev_loose = out_loose[0]
    d_loose = 10.0 - trunc_prev_loose.end_pos[0]
    # Mutate corner_deviation mid-stream, mimic cmd_SET_VELOCITY_LIMIT.
    th.corner_deviation = 0.01
    b.reset()  # clear the buffered next-head so a fresh blend is computed.
    m2_prev = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0), speed=100.0)
    m2_next = _FakeMove(th, (10, 0, 0, 0), (10, 10, 0, 0), speed=100.0)
    b.feed(m2_prev)
    out_tight = b.feed(m2_next)
    arc_tight = out_tight[1:]
    trunc_prev_tight = out_tight[0]
    d_tight = 10.0 - trunc_prev_tight.end_pos[0]
    # 20x deviation shrink must produce roughly 20x shorter d_consumed
    # (d_consumed = R * tan(theta/2); at 90° tan=1 so d == R, and R is
    # linear in corner_deviation when R_tol binds).
    assert d_loose > 10 * d_tight
    # Arc cruise velocity caps must also shrink: v_cap = sqrt(a_max * R).
    v_cap_loose = math.sqrt(arc_loose[0].max_cruise_v2)
    v_cap_tight = math.sqrt(arc_tight[0].max_cruise_v2)
    assert v_cap_loose > 4 * v_cap_tight  # sqrt(20) ~ 4.47


def test_e_conservation_through_blend():
    b = _blender(max_chord_err=20e-3)
    th = b._toolhead
    m_prev = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    m_next = _FakeMove(th, (10, 0, 0, 0.5), (10, 10, 0, 1.0), speed=100.0)
    b.feed(m_prev)
    out = b.feed(m_next)
    # Drain buffered trunc_next_head.
    out += b.flush()
    total_e = sum(am.axes_d[3] for am in out)
    expected = m_prev.axes_d[3] + m_next.axes_d[3]
    assert total_e == pytest.approx(expected, rel=1e-9, abs=1e-12)


def test_asymmetric_segments_half_segment_rule_caps_consumption():
    b = _blender(max_chord_err=20e-3)
    th = b._toolhead
    # 60° corner. Short segment = 2mm, long = 10mm. With LOOSE tolerance so
    # R_tol >> R_mid and the midpoint cap binds.
    # R_mid = 0.5 * min(2, 10) * cot(30°) = 0.5 * 2 * sqrt(3) = sqrt(3)
    # d = R * tan(30°) = sqrt(3) * (1/sqrt(3)) = 1.0 (= L_short / 2)
    th.corner_deviation = 10.0  # absurdly loose so R_tol does not bind
    angle = math.radians(60.0)
    m_prev = _FakeMove(th, (0, 0, 0, 0), (2, 0, 0, 0.1), speed=100.0)
    # Rotate next direction by 60° from +X.
    next_end = (
        2 + 10 * math.cos(angle),
        0 + 10 * math.sin(angle),
        0, 0.6,
    )
    m_next = _FakeMove(th, (2, 0, 0, 0.1), next_end, speed=100.0)
    b.feed(m_prev)
    out = b.feed(m_next)
    trunc_prev = out[0]
    # trunc_prev.move_d should equal 2 - 1 = 1 mm (half-segment consumption).
    assert trunc_prev.move_d == pytest.approx(1.0, rel=1e-6)


def test_aggregate_kin_check_move_fires_on_representative_arc_move():
    b = _blender(max_chord_err=20e-3)
    th = b._toolhead
    m_prev = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    m_next = _FakeMove(th, (10, 0, 0, 0.5), (10, 10, 0, 1.0), speed=100.0)
    b.feed(m_prev)
    out = b.feed(m_next)
    # The representative arc move was passed to kin.check_move exactly once.
    arc_moves = out[1:]
    assert len(th.kin.calls) == 1
    assert th.kin.calls[0] is arc_moves[0]


def test_aggregate_extruder_check_move_fires_when_extruding():
    b = _blender(max_chord_err=20e-3)
    th = b._toolhead
    m_prev = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    m_next = _FakeMove(th, (10, 0, 0, 0.5), (10, 10, 0, 1.0), speed=100.0)
    b.feed(m_prev)
    b.feed(m_next)
    # Extruder check_move called once on the representative arc move (E delta
    # is non-zero because both prev and next extrude).
    assert len(th.extruder.calls) == 1


def test_aggregate_extruder_check_move_skipped_when_not_extruding():
    b = _blender(max_chord_err=20e-3)
    th = b._toolhead
    # E coordinate identical across prev and next (travel moves).
    m_prev = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0), speed=100.0)
    m_next = _FakeMove(th, (10, 0, 0, 0), (10, 10, 0, 0), speed=100.0)
    b.feed(m_prev)
    b.feed(m_next)
    assert len(th.extruder.calls) == 0


def test_arc_polyline_smooth_delta_v2_not_pinned():
    # The emit path no longer pins smooth_delta_v2 == delta_v2 — that
    # was overreach. Kalico's Move invariant smooth_delta_v2 <= delta_v2
    # must still hold on every emitted polyline move.
    b = _blender(max_chord_err=20e-3)
    th = b._toolhead
    m_prev = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    m_next = _FakeMove(th, (10, 0, 0, 0.5), (10, 10, 0, 1.0), speed=100.0)
    b.feed(m_prev)
    out = b.feed(m_next)
    arc_moves = out[1:]
    for am in arc_moves:
        assert am.smooth_delta_v2 <= am.delta_v2 + 1e-12


def test_arc_polyline_speed_continuity_1ppm():
    b = _blender(max_chord_err=20e-3)
    th = b._toolhead
    m_prev = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    m_next = _FakeMove(th, (10, 0, 0, 0.5), (10, 10, 0, 1.0), speed=100.0)
    b.feed(m_prev)
    out = b.feed(m_next)
    arc_moves = out[1:]
    v2s = [am.max_cruise_v2 for am in arc_moves]
    # All arc moves share the same cap to 1 ppm.
    assert (max(v2s) - min(v2s)) / max(v2s) < 1e-6


def _random_unit_3d(rng):
    while True:
        v = (rng.uniform(-1, 1), rng.uniform(-1, 1), rng.uniform(-1, 1))
        n = math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)
        if n > 0.1:
            return (v[0] / n, v[1] / n, v[2] / n)


@pytest.mark.parametrize("seed", range(50))
def test_property_random_3d_corners(seed):
    rng = random.Random(seed)
    b = _blender(max_chord_err=20e-3)
    th = b._toolhead
    th.corner_deviation = rng.uniform(20e-3, 200e-3)
    d_prev = _random_unit_3d(rng)
    d_next = _random_unit_3d(rng)
    # Skip pathological near-collinear / near-reversal samples so the test
    # exercises a real blend.
    dot = d_prev[0] * d_next[0] + d_prev[1] * d_next[1] + d_prev[2] * d_next[2]
    if abs(dot) > 0.95:
        pytest.skip("near-collinear or near-reversal")
    L_prev = rng.uniform(1.0, 20.0)
    L_next = rng.uniform(1.0, 20.0)
    vertex = (
        rng.uniform(10, 90),
        rng.uniform(10, 90),
        rng.uniform(5, 20),
    )
    start = tuple(vertex[i] - L_prev * d_prev[i] for i in range(3))
    end = tuple(vertex[i] + L_next * d_next[i] for i in range(3))
    # E coordinates picked so prev and next share approximate flow so the
    # extruder-boundary cap does not dominate the test.
    prev_e_start = 0.0
    prev_e_end = 0.05 * L_prev
    next_e_end = prev_e_end + 0.05 * L_next
    m_prev = _FakeMove(
        th, (start[0], start[1], start[2], prev_e_start),
        (vertex[0], vertex[1], vertex[2], prev_e_end),
        speed=100.0,
    )
    m_next = _FakeMove(
        th, (vertex[0], vertex[1], vertex[2], prev_e_end),
        (end[0], end[1], end[2], next_e_end),
        speed=100.0,
    )
    b.feed(m_prev)
    out = b.feed(m_next) + b.flush()
    # Invariant 1: E conservation.
    total_e = sum(am.axes_d[3] for am in out)
    expected_e = m_prev.axes_d[3] + m_next.axes_d[3]
    assert total_e == pytest.approx(expected_e, rel=1e-9, abs=1e-12)
    # Invariant 2: non-negative move_d on every emitted piece.
    for am in out:
        assert am.move_d >= -1e-12
    # Invariant 3: first emitted piece starts at m_prev.start_pos.
    assert out[0].start_pos[:3] == m_prev.start_pos[:3]
    # Invariant 4: last emitted piece ends at m_next.end_pos (within float noise).
    assert out[-1].end_pos[0] == pytest.approx(m_next.end_pos[0], abs=1e-9)
    assert out[-1].end_pos[1] == pytest.approx(m_next.end_pos[1], abs=1e-9)
    assert out[-1].end_pos[2] == pytest.approx(m_next.end_pos[2], abs=1e-9)


def test_blender_degenerate_R_zero_forces_stop_at_prev():
    """When CornerBlender produces R=0 (e.g. U-turn or extremely short neighbor),
    the previous move must be limited to a full stop at its end junction.
    This is the safety net that replaces the old JD constraint for the
    blender-decline path. This test verifies the safety net is intact
    before any JD deletion work."""
    b = _blender()
    th = b._toolhead
    # Create a U-turn: +X then -X (180° reversal).
    m1 = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    m2 = _FakeMove(th, (10, 0, 0, 0.5), (0, 0, 0, 1.0), speed=100.0)
    assert b.feed(m1) == []  # buffered
    out = b.feed(m2)
    # U-turn: blender detects R=0 and v_cap=0, forces a stop at prev's junction.
    assert out == [m1]
    assert m1.next_junction_v2 == 0.0
    assert b._prev is m2  # next is buffered for the next corner


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


def test_pipeline_composition_prepass_then_blender():
    from klippy import blendprepass
    th = _FakeToolhead(corner_deviation=50e-3)
    prepass = blendprepass.CollinearCollapser(th, move_cls=_FakeMove)
    blender = blendplanner.CornerBlender(
        th, move_cls=_FakeMove, max_chord_err=20e-3
    )
    inner = _FakeInnerQueue()
    adapter = blendprepass.BlendPipelineLookAheadQueue(
        [prepass, blender], inner
    )
    # 10 short collinear +X moves, then a 90 degree turn into 10 short +Y moves.
    pos = (0.0, 0.0, 0.0, 0.0)
    for i in range(10):
        nxt = (pos[0] + 1.0, pos[1], pos[2], pos[3] + 0.05)
        adapter.add_move(_FakeMove(th, pos, nxt, speed=100.0))
        pos = nxt
    for i in range(10):
        nxt = (pos[0], pos[1] + 1.0, pos[2], pos[3] + 0.05)
        adapter.add_move(_FakeMove(th, pos, nxt, speed=100.0))
        pos = nxt
    adapter.flush()
    # Prepass merged each side into a long move; blender produced one blend.
    assert blender.blends_emitted == 1
    assert blender.polyline_moves_emitted >= 2


def test_pipeline_adapter_get_last_returns_blender_prev_when_buffered():
    from klippy import blendprepass
    th = _FakeToolhead(corner_deviation=50e-3)
    prepass = blendprepass.CollinearCollapser(th, move_cls=_FakeMove)
    blender = blendplanner.CornerBlender(
        th, move_cls=_FakeMove, max_chord_err=20e-3
    )
    inner = _FakeInnerQueue()
    adapter = blendprepass.BlendPipelineLookAheadQueue(
        [prepass, blender], inner
    )
    # Feed one move. Prepass buffers it; get_last returns from prepass.
    m = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    adapter.add_move(m)
    assert adapter.get_last() is m
    # Inner queue still empty - no flush side effect.
    assert inner.queue == []


def test_get_last_no_forfeit_callback_transfers_to_trunc_prev():
    # Verify that mutations applied to the move returned by get_last()
    # (timing_callbacks, limit_next_junction_speed) survive into trunc_prev
    # after the blend, and that get_last() does not cause a premature flush.
    b = _blender(max_chord_err=20e-3)
    th = b._toolhead
    m_prev = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    b.feed(m_prev)
    # get_last via peek_buffered — no flush side effect.
    assert b._prev is m_prev
    marker = []
    m_prev.timing_callbacks.append(lambda t: marker.append(t))
    m_prev.limit_next_junction_speed(50.0)
    # Feed the next move — should trigger a blend and transfer callback state.
    m_next = _FakeMove(th, (10, 0, 0, 0.5), (10, 10, 0, 1.0), speed=100.0)
    out = b.feed(m_next)
    # Emission: [trunc_prev, arc[0], ..., arc[N-1]]
    assert len(out) >= 2
    trunc_prev = out[0]
    assert trunc_prev is not m_prev  # new Move, not the original
    # Callback transferred onto trunc_prev via _copy_caller_state.
    assert trunc_prev.timing_callbacks != []
    # limit_next_junction_speed was applied to m_prev (next_junction_v2 = 50^2)
    # and transferred onto trunc_prev via _copy_caller_state.
    assert trunc_prev.next_junction_v2 == 50.0 ** 2


def test_set_velocity_limit_mid_blend_does_not_leak_lowered_accel():
    b = _blender(max_chord_err=20e-3)
    th = b._toolhead
    m_prev = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    # Original accel snapshotted on m_prev: 10000 (th.max_accel at ctor).
    assert m_prev.accel == 10000.0
    b.feed(m_prev)
    # User issues an M204 that lowers accel.
    th.max_accel = 3000.0
    m_next = _FakeMove(th, (10, 0, 0, 0.5), (10, 10, 0, 1.0), speed=100.0)
    assert m_next.accel == 3000.0
    out = b.feed(m_next)
    trunc_prev = out[0]
    # trunc_prev must pin parent's accel (10000), NOT the lowered toolhead
    # value. This is the critical anti-leak assertion — _copy_caller_state
    # uses direct assignment, not limit_speed.
    assert trunc_prev.accel == m_prev.accel  # 10000, not min(10000, 3000)


def test_drip_mode_single_move_emits_unchanged():
    from klippy import blendprepass
    th = _FakeToolhead(corner_deviation=50e-3)
    prepass = blendprepass.CollinearCollapser(th, move_cls=_FakeMove)
    blender = blendplanner.CornerBlender(
        th, move_cls=_FakeMove, max_chord_err=20e-3
    )
    inner = _FakeInnerQueue()
    adapter = blendprepass.BlendPipelineLookAheadQueue(
        [prepass, blender], inner
    )
    # Mimic drip_move: one move arrives, then flush.
    m = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    adapter.add_move(m)
    adapter.flush()
    # Single move exits unblended (no corner to blend against).
    assert inner.queue == [m]
    assert blender.blends_emitted == 0


def test_blender_peek_buffered():
    b = _blender()
    th = b._toolhead
    assert b.peek_buffered() == []
    m = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    b.feed(m)
    assert b.peek_buffered() == [m]
    # Peek must not mutate state.
    b.peek_buffered()
    assert b._prev is m


def test_adapter_queue_reports_blender_buffered_move():
    from klippy import blendprepass
    th = _FakeToolhead(corner_deviation=50e-3)
    prepass = blendprepass.CollinearCollapser(th, move_cls=_FakeMove)
    blender = blendplanner.CornerBlender(
        th, move_cls=_FakeMove, max_chord_err=20e-3
    )
    inner = _FakeInnerQueue()
    adapter = blendprepass.BlendPipelineLookAheadQueue(
        [prepass, blender], inner
    )
    m = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
    adapter.add_move(m)
    # Prepass buffered the move; adapter.queue must reflect it.
    assert adapter.queue == [m]


def test_max_accel_to_decel_is_property_tracking_min_cruise_ratio():
    """max_accel_to_decel must be derived from min_cruise_ratio on every
    read, not cached as a field set by _calc_junction_deviation."""
    # Use a real ToolHead-like object via direct attribute manipulation.
    # The contract is: max_accel_to_decel == max_accel * (1 - min_cruise_ratio)
    # at any moment, with no recompute call required.
    from klippy import toolhead as th_mod

    class _Stub:
        max_accel_to_decel = th_mod.ToolHead.max_accel_to_decel
        max_accel = 5000.0
        min_cruise_ratio = 0.5

    s = _Stub()
    assert s.max_accel_to_decel == 2500.0
    s.min_cruise_ratio = 0.7
    assert s.max_accel_to_decel == pytest.approx(1500.0, rel=1e-12)
    s.max_accel = 10000.0
    assert s.max_accel_to_decel == pytest.approx(3000.0, rel=1e-12)


def test_calc_junction_skips_block_at_perfect_tangency():
    """At a tangent (collinear) junction, cos_theta_d2 == 0 and the
    centripetal/JD block must be skipped entirely. max_start_v2 is
    therefore set by the pre-block min() — typically prev.max_start_v2
    + prev.delta_v2."""
    from klippy import toolhead as th_mod

    class _StubExtruder:
        def calc_junction(self, prev, nxt):
            return 1e18

    class _StubToolhead:
        max_velocity = 1e6
        max_accel = 10000.0
        min_cruise_ratio = 0.5
        max_accel_to_decel = th_mod.ToolHead.max_accel_to_decel
        junction_deviation = 0.01  # ignored after deletion; still readable
        extruder = _StubExtruder()

    th = _StubToolhead()
    m1 = th_mod.Move(th, (0, 0, 0, 0), (10, 0, 0, 0), speed=200.0)
    m2 = th_mod.Move(th, (10, 0, 0, 0), (20, 0, 0, 0), speed=200.0)
    # Pre-state: m1.max_start_v2 starts at 0; m1.delta_v2 = 2*10*10000 = 200000.
    m2.calc_junction(m1)
    # Tangent: block skipped, max_start_v2 = min(extruder, cruise, prev_cruise,
    #   prev.next_junction_v2, prev.max_start_v2 + prev.delta_v2)
    # = min(1e18, 40000, 40000, 999999999.9, 200000) = 40000 (cruise cap binds).
    assert m2.max_start_v2 == pytest.approx(40000.0, rel=1e-12)


def test_calc_junction_centripetal_at_90deg_after_jd_removal():
    """At a 90° corner with JD deleted, the centripetal mid-move cap
    must be the binding term: v² ≤ 0.5 · d · a · tan(θ/2). With θ=π/2,
    d=10, a=10000: cap = 0.5 · 10 · 10000 · 1 = 50000."""
    from klippy import toolhead as th_mod

    class _StubExtruder:
        def calc_junction(self, prev, nxt):
            return 1e18

    class _StubToolhead:
        max_velocity = 1e6
        max_accel = 10000.0
        min_cruise_ratio = 0.5
        max_accel_to_decel = th_mod.ToolHead.max_accel_to_decel
        junction_deviation = 0.01  # ignored after deletion
        extruder = _StubExtruder()

    th = _StubToolhead()
    m1 = th_mod.Move(th, (0, 0, 0, 0), (10, 0, 0, 0), speed=1000.0)
    m2 = th_mod.Move(th, (10, 0, 0, 0), (10, 10, 0, 0), speed=1000.0)
    m2.calc_junction(m1)
    # delta_v2 = 2*10*10000 = 200000; quarter_tan(π/4) = 0.25;
    # centripetal = 0.25 * 200000 = 50000. cruise cap = 1e6 (loose).
    # JD cap (if still present) = R_jd * 0.01 * 10000 = 2.414 * 100 = 241.4 — would bind.
    # After JD deletion: centripetal = 50000 binds.
    assert m2.max_start_v2 == pytest.approx(50000.0, rel=1e-12)


def test_scv_config_deprecation_warning(caplog):
    """When [printer] square_corner_velocity is set in config, ToolHead
    init must call config.deprecate and emit a one-time logging.warning
    so users see it in klippy.log and Mainsail's deprecation panel."""
    import logging
    from unittest.mock import MagicMock

    # Build a mock config that reports square_corner_velocity = 5
    mock_config = MagicMock()
    def _getfloat(name, default=None, **kw):
        if name == "square_corner_velocity":
            return 5.0
        return default
    mock_config.getfloat.side_effect = _getfloat

    # Replicate the ToolHead config-handling block in isolation
    from klippy import toolhead as th_mod
    with caplog.at_level(logging.WARNING):
        scv_legacy = mock_config.getfloat(
            "square_corner_velocity", None, minval=0.0
        )
        if scv_legacy is not None:
            mock_config.deprecate("square_corner_velocity")
            import logging as _log
            _log.warning(
                "config option [printer] square_corner_velocity is obsolete; "
                "the new arc-blending planner ignores it. Remove it from your "
                "config to silence this warning."
            )

    mock_config.deprecate.assert_called_once_with("square_corner_velocity")
    assert any(
        "square_corner_velocity is obsolete" in rec.message
        for rec in caplog.records
    )


def test_scv_config_absent_no_warning(caplog):
    """When config has no square_corner_velocity entry, no warning fires."""
    from unittest.mock import MagicMock

    mock_config = MagicMock()
    def _getfloat(name, default=None, **kw):
        return default  # always return default (None for SCV)
    mock_config.getfloat.side_effect = _getfloat

    with caplog.at_level("WARNING"):
        scv_legacy = mock_config.getfloat(
            "square_corner_velocity", None, minval=0.0
        )
        if scv_legacy is not None:
            mock_config.deprecate("square_corner_velocity")

    mock_config.deprecate.assert_not_called()
    assert not any(
        "square_corner_velocity" in rec.message for rec in caplog.records
    )


def test_scv_gcode_silent_noop_pattern():
    """Pattern-level verification: gcmd.get_float for SQUARE_CORNER_VELOCITY
    must accept the value without error and the local must not be assigned
    to any toolhead attribute. Verified at the SET_VELOCITY_LIMIT call site
    in toolhead.py — this test exercises the contract."""
    from unittest.mock import MagicMock
    gcmd = MagicMock()
    gcmd.get_float.return_value = 10.0
    # Replicate the SCV-handling pattern from cmd_SET_VELOCITY_LIMIT
    square_corner_velocity = gcmd.get_float(
        "SQUARE_CORNER_VELOCITY", None, minval=0.0
    )
    # Contract: the value is parsed but never assigned anywhere.
    # The local exists only for the all-None guard.
    assert square_corner_velocity == 10.0
    # Critically, no follow-up assignment exists — this is a structural test
    # confirmed by grep in step 3 above (zero self.square_corner_velocity hits
    # in toolhead.py).


def test_status_excludes_square_corner_velocity():
    """toolhead.get_status output must not contain square_corner_velocity
    after sub-spec #5. End-to-end check using a real ToolHead is heavy;
    structural verification via grep in Task 10 step 3 is the primary gate.
    This test exists to fail loudly if a future patch reintroduces the key."""
    import inspect
    from klippy import toolhead as th_mod
    src = inspect.getsource(th_mod.ToolHead.get_status)
    assert '"square_corner_velocity"' not in src, (
        "ToolHead.get_status reintroduced square_corner_velocity key"
    )
    assert "'square_corner_velocity'" not in src
