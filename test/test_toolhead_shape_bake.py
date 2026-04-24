"""Plan 9 A3 — shape-everywhere tests."""
import pytest
import os
import sys

_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from klippy import blendmath
from klippy import toolhead as th_mod


class _Printer:
    def __init__(self):
        self._objs = {}
    def lookup_object(self, name, default=None):
        return self._objs.get(name, default)


class _FakeToolhead:
    """Minimal toolhead stub for Move construction in A3 tests."""
    def __init__(self):
        self.printer = _Printer()
        self.max_velocity = 500.0
        self.max_accel = 5000.0
        self.max_accel_to_decel = 5000.0
        self.square_corner_velocity = 5.0
        self.junction_deviation = 0.05
        self.max_jerk = 100000.0
        # Plan 9 A2d → A3: extruder metadata consumed by PA-compose
        self.extruder = _DummyExtruder()
        self.trapq = None
        # Plan 9 A3 T2: cached shaper snapshot read by Move.__init__.
        # Default to empty; tests that load an input_shaper call
        # _refresh_shapers_snapshot() after populating printer objects.
        self.shapers_snapshot = []
    def _refresh_shapers_snapshot(self):
        self.shapers_snapshot = blendmath.extract_shapers(self)
    def note_kinematic_activity(self, *a, **kw):
        pass


class _DummyExtruder:
    def get_status(self, eventtime=None):
        return {"pressure_advance": 0.0,
                "pressure_advance_model": "linear",
                "pressure_advance_smooth_time": 0.0}
    def calc_junction(self, *a, **kw):
        return 1e99


def _make_move(tool, start, end, speed=100.0):
    return th_mod.Move(tool, list(start), list(end), speed)


def test_build_unshaped_payload_returns_three_tuple():
    tool = _FakeToolhead()
    move = _make_move(tool, [0., 0., 0., 0.], [20., 0., 0., 0.])
    move.set_junction(100.0**2, 100.0**2, 100.0**2)
    payload = move.build_unshaped_payload()
    assert isinstance(payload, tuple) and len(payload) == 3
    phase_t_ends, total_t, coeff_tuple = payload
    assert len(phase_t_ends) >= 1
    assert total_t == phase_t_ends[-1]
    # Interleaved x/y/z/e layout, 15 coeffs per phase
    assert len(coeff_tuple) == len(phase_t_ends) * 15 * 4


def test_finalize_shape_stub_matches_a2d_payload_layout():
    """Task 1 stub: finalize_shape is a pass-through. The resulting
    quintic_trapq_payload 9-tuple must be structurally identical to
    the A2d output: same phase_t_ends, same total_t, same coeff layout.
    """
    tool = _FakeToolhead()
    move = _make_move(tool, [0., 0., 0., 0.], [20., 0., 0., 0.])
    move.set_junction(100.0**2, 100.0**2, 100.0**2)
    assert move.quintic_trapq_payload is not None
    (phase_t_ends, total_t, arc_length, v_cap_min, start_pos_xyz,
     coeff_tuple, legacy_accel_end, legacy_decel_start,
     legacy_total_t) = move.quintic_trapq_payload
    # Consistency: the unshaped and baked polynomials match (stub pass-through)
    u_phase_t_ends, u_total_t, u_coeffs = move._unshaped_payload
    assert phase_t_ends == u_phase_t_ends
    assert total_t == u_total_t
    assert coeff_tuple == u_coeffs
    assert arc_length == pytest.approx(20.0)
    # Legacy compat fields preserved
    assert legacy_total_t == pytest.approx(
        move.accel_t + move.cruise_t + move.decel_t)


# ---------------------------------------------------------------------------
# Task 2 — shaper snapshot capture
# ---------------------------------------------------------------------------


def test_move_captures_empty_shapers_snapshot_when_no_input_shaper():
    tool = _FakeToolhead()
    move = _make_move(tool, [0., 0., 0., 0.], [20., 0., 0., 0.])
    # No input_shaper module loaded → empty list
    assert move._shapers_snapshot == []


def test_move_captures_shaper_snapshot_when_input_shaper_loaded():
    tool = _FakeToolhead()
    class _FakeAxisShaper:
        class params:
            shaper_type = "mzv"
            shaper_freq = 42.0
            damping_ratio = 0.1
        def get_axis(self):
            return "x"
    class _FakeIS:
        def get_shapers(self):
            return [_FakeAxisShaper(), _FakeAxisShaper()]
    tool.printer._objs["input_shaper"] = _FakeIS()
    # Mirror the real lifecycle: SET_INPUT_SHAPER / connect refreshes the
    # toolhead cache. Move.__init__ then reads the cache directly — O(1).
    tool._refresh_shapers_snapshot()
    move = _make_move(tool, [0., 0., 0., 0.], [20., 0., 0., 0.])
    # Snapshot must be captured (exact format is blendmath's — here we
    # just assert non-empty)
    assert len(move._shapers_snapshot) == 2


def test_move_reads_toolhead_shapers_snapshot_cache_not_extract():
    """Regression guard: Move.__init__ must read toolhead.shapers_snapshot
    verbatim rather than calling extract_shapers per move. The
    code-quality fix on A3T2 moved the expensive find_shaper_max_accel
    bisection off the per-move hot path and onto SET_INPUT_SHAPER.
    """
    tool = _FakeToolhead()
    # Seed the cache with a sentinel object; if Move.__init__ ever reverts
    # to calling extract_shapers(toolhead) we'll get back [] instead.
    sentinel = object()
    tool.shapers_snapshot = [sentinel]
    move = _make_move(tool, [0., 0., 0., 0.], [20., 0., 0., 0.])
    assert move._shapers_snapshot == [sentinel]
    assert move._shapers_snapshot[0] is sentinel


def test_refresh_shapers_snapshot_picks_up_config_changes():
    """Simulate SET_INPUT_SHAPER: configure input_shaper on the printer,
    call _refresh_shapers_snapshot, then construct a Move — the move
    must see the updated snapshot. This verifies the invalidation path
    SET_INPUT_SHAPER exercises (input_shaper._flush_for_shaper_update
    calls toolhead._refresh_shapers_snapshot)."""
    tool = _FakeToolhead()
    # Baseline: no shaper loaded
    m1 = _make_move(tool, [0., 0., 0., 0.], [20., 0., 0., 0.])
    assert m1._shapers_snapshot == []
    # "SET_INPUT_SHAPER": load an input_shaper, then refresh the cache.
    class _FakeAxisShaper:
        class params:
            shaper_type = "mzv"
            shaper_freq = 42.0
            damping_ratio = 0.1
        def get_axis(self):
            return "x"
    class _FakeIS:
        def get_shapers(self):
            return [_FakeAxisShaper()]
    tool.printer._objs["input_shaper"] = _FakeIS()
    tool._refresh_shapers_snapshot()
    m2 = _make_move(tool, [20., 0., 0., 0.], [40., 0., 0., 0.])
    assert len(m2._shapers_snapshot) == 1
    # The baseline move's snapshot is unchanged (frozen at construction).
    assert m1._shapers_snapshot == []


# ---------------------------------------------------------------------------
# Task 3 — real shape-bake in finalize_shape
# ---------------------------------------------------------------------------


def _make_toolhead_with_mzv():
    """Return a _FakeToolhead with a 3-axis MZV 42 Hz shaper configured."""
    tool = _FakeToolhead()
    class _FakeAxisShaper:
        class params:
            shaper_type = "mzv"
            shaper_freq = 42.0
            damping_ratio = 0.1
        def get_axis(self):
            return "x"
    class _FakeIS:
        def get_shapers(self):
            return [_FakeAxisShaper()] * 3
    tool.printer._objs["input_shaper"] = _FakeIS()
    tool._refresh_shapers_snapshot()
    return tool


def test_finalize_shape_applies_mzv_when_shaper_configured():
    """With a configured MZV shaper, finalize_shape must produce a
    polynomial different from the unshaped (the shape is actually
    applied)."""
    tool = _make_toolhead_with_mzv()
    move = _make_move(tool, [0., 0., 0., 0.], [50., 0., 0., 0.])
    move.set_junction(150.0**2, 150.0**2, 150.0**2)
    # Unshaped is captured; baked differs because MZV convolution
    # extends the move in time.
    u_phase_t_ends, u_total_t, u_coeffs = move._unshaped_payload
    baked = move.quintic_trapq_payload
    b_phase_t_ends, b_total_t = baked[0], baked[1]
    b_coeffs = baked[5]
    assert b_total_t >= u_total_t  # MZV stretches by the shaper window
    # Coefficient buffers must differ (shape applied, not pass-through)
    assert b_coeffs != u_coeffs


def test_finalize_shape_passthrough_when_no_shaper():
    """No input_shaper → pass-through (matches Task 1 stub behavior)."""
    tool = _FakeToolhead()
    move = _make_move(tool, [0., 0., 0., 0.], [50., 0., 0., 0.])
    move.set_junction(150.0**2, 150.0**2, 150.0**2)
    u_phase_t_ends, u_total_t, u_coeffs = move._unshaped_payload
    baked = move.quintic_trapq_payload
    assert baked[0] == u_phase_t_ends
    assert baked[1] == u_total_t
    assert baked[5] == u_coeffs


def test_finalize_shape_offsets_prev_neighbour_polynomial():
    """When a prev_unshaped is supplied with its own start_pos,
    finalize_shape should shift it into cur_start_pos frame before
    composing. We verify by building two setups that only differ in
    neighbour start_pos — the baked polynomial must differ because
    the XY offset changes cross-boundary coefficients."""
    tool = _make_toolhead_with_mzv()

    # Two moves; second will shape-bake with first as prev.
    m1 = _make_move(tool, [0., 0., 0., 0.], [20., 0., 0., 0.])
    m1.set_junction(150.0**2, 150.0**2, 150.0**2)
    m2 = _make_move(tool, [20., 0., 0., 0.], [40., 0., 0., 0.])
    m2.set_junction(150.0**2, 150.0**2, 150.0**2)

    # Bake m2 with m1 as prev — correct (continuous): m1 starts at (0,0,0)
    # so dx = 0 - 20 = -20 shift applied to m1's c[0].
    m2.finalize_shape(
        prev_unshaped=m1._unshaped_payload,
        prev_start_pos_xyz=(0., 0., 0.),
    )
    coeffs_with_offset = m2.quintic_trapq_payload[5]

    # Bake m2 with m1's unshaped but WRONG start_pos (pretend m1
    # started at m2's start) — no offset applied (dx=0).
    m2.finalize_shape(
        prev_unshaped=m1._unshaped_payload,
        prev_start_pos_xyz=(20., 0., 0.),
    )
    coeffs_no_offset = m2.quintic_trapq_payload[5]

    assert coeffs_with_offset != coeffs_no_offset


# ---------------------------------------------------------------------------
# Task 4 — LookAheadQueue deferred-last state fields
# ---------------------------------------------------------------------------


def test_lookahead_queue_has_deferred_last_state():
    """LookAheadQueue exposes the deferred-last tuple for the Plan 9 A3
    shape-bake pass."""
    tool = _FakeToolhead()
    laq = th_mod.LookAheadQueue(tool)
    assert laq._pending_last is None


def test_lookahead_queue_reset_clears_deferred_last_state():
    tool = _FakeToolhead()
    laq = th_mod.LookAheadQueue(tool)
    # Any non-None tuple works; the internal shape is not contractual at
    # the reset layer.
    laq._pending_last = (object(), object(), (1., 2., 3.))
    laq.reset()
    assert laq._pending_last is None


# ---------------------------------------------------------------------------
# Task 5 — LookAheadQueue deferred-last shape-bake pass
# ---------------------------------------------------------------------------


def _make_laq_with_spy(tool):
    """Return (LookAheadQueue, emitted_list) with a spy on _process_moves."""
    laq = th_mod.LookAheadQueue(tool)
    emitted = []
    tool._process_moves = lambda moves: emitted.extend(moves)
    return laq, emitted


def _inject_and_drain(laq, moves):
    """Inject pre-set-junction moves directly into the LookAheadQueue and
    drain with lazy=False. Bypasses the reverse-pass velocity math so tests
    exercise the shape-bake pass without depending on junction convergence."""
    for m in moves:
        if m.is_kinematic_move:
            m.set_junction(0.0, m.max_cruise_v2 * 0.5, 0.0)
        laq.queue.append(m)
    laq.flush(lazy=False)


def test_flush_drain_emits_all_moves():
    """lazy=False (drain) emits all moves, all get shape-baked payloads."""
    tool = _FakeToolhead()
    laq, emitted = _make_laq_with_spy(tool)
    m1 = _make_move(tool, [0., 0., 0., 0.], [10., 0., 0., 0.])
    m2 = _make_move(tool, [10., 0., 0., 0.], [20., 0., 0., 0.])
    m3 = _make_move(tool, [20., 0., 0., 0.], [30., 0., 0., 0.])
    _inject_and_drain(laq, [m1, m2, m3])
    assert len(emitted) == 3
    assert emitted == [m1, m2, m3]
    assert laq._pending_last is None
    for m in (m1, m2, m3):
        assert m.quintic_trapq_payload is not None


def test_flush_lazy_false_drains_all():
    """lazy=False emits all moves and leaves no pending state."""
    tool = _FakeToolhead()
    laq, emitted = _make_laq_with_spy(tool)
    m1 = _make_move(tool, [0., 0., 0., 0.], [10., 0., 0., 0.])
    m2 = _make_move(tool, [10., 0., 0., 0.], [20., 0., 0., 0.])
    _inject_and_drain(laq, [m1, m2])
    assert len(emitted) == 2
    assert laq._pending_last is None


def test_flush_single_move_lazy_false():
    """A single-move drain flush emits the one move immediately."""
    tool = _FakeToolhead()
    laq, emitted = _make_laq_with_spy(tool)
    m1 = _make_move(tool, [0., 0., 0., 0.], [10., 0., 0., 0.])
    _inject_and_drain(laq, [m1])
    assert len(emitted) == 1
    assert emitted[0] is m1
    assert laq._pending_last is None


def test_flush_chain_prev_pending_emitted_before_new_batch():
    """A pending move from a prior flush is emitted at the head of the
    next flush's batch, finalized with the first new move as its next
    neighbour.

    We plant a pending move directly to simulate the output of a prior lazy
    flush, then drain a new batch. The pending move must appear first."""
    tool = _FakeToolhead()
    laq, emitted = _make_laq_with_spy(tool)
    m1 = _make_move(tool, [0., 0., 0., 0.], [10., 0., 0., 0.])
    m2 = _make_move(tool, [10., 0., 0., 0.], [20., 0., 0., 0.])
    m3 = _make_move(tool, [20., 0., 0., 0.], [30., 0., 0., 0.])
    # Simulate "m1 was the pending last of a prior flush"
    m1.set_junction(0.0, m1.max_cruise_v2 * 0.5, 0.0)
    laq._pending_last = (m1, None, None)
    # Drain [m2, m3]. m1 should appear first.
    _inject_and_drain(laq, [m2, m3])
    assert emitted == [m1, m2, m3]
    assert laq._pending_last is None


def test_flush_chain_two_drain_flushes():
    """Two successive drain flushes carry moves in the correct order with
    no re-ordering or duplication."""
    tool = _FakeToolhead()
    laq, emitted = _make_laq_with_spy(tool)
    m1 = _make_move(tool, [0., 0., 0., 0.], [10., 0., 0., 0.])
    m2 = _make_move(tool, [10., 0., 0., 0.], [20., 0., 0., 0.])
    m3 = _make_move(tool, [20., 0., 0., 0.], [30., 0., 0., 0.])
    _inject_and_drain(laq, [m1, m2])
    assert emitted == [m1, m2]
    _inject_and_drain(laq, [m3])
    assert emitted == [m1, m2, m3]
    assert laq._pending_last is None


def test_flush_pure_e_move_passes_through_unchanged():
    """Pure-E (non-kinematic) moves are not shape-bake targets — they pass
    through _process_moves unchanged and quintic_trapq_payload stays None."""
    tool = _FakeToolhead()
    laq, emitted = _make_laq_with_spy(tool)
    m_e = _make_move(tool, [10., 10., 10., 0.], [10., 10., 10., 5.])
    assert m_e.is_kinematic_move is False
    laq.queue.append(m_e)
    laq.flush(lazy=False)
    assert len(emitted) == 1
    assert emitted[0] is m_e
    # Pure-E move: set_junction is never called, so quintic_trapq_payload
    # is never set. Either absent or None — either way, not shape-baked.
    assert getattr(m_e, "quintic_trapq_payload", None) is None


def test_flush_lazy_true_single_move_becomes_pending():
    """With a 1-move queue and lazy=True, the reverse pass may return
    early (flush_count=0) OR flush it (flush_count=1). In the flush_count=1
    case, T5 must hold the move as the pending-last. We inject the move
    with set_junction already called to ensure the reverse pass confirms it,
    then force flush(lazy=True) by bypassing add_move.

    We verify the post-condition: either nothing was emitted (the move is
    pending) or the pending is None (lazy returned early — also acceptable
    because the move is still in the queue for the next flush).
    The key invariant: if _pending_last is set, its first element IS the
    move we added."""
    tool = _FakeToolhead()
    laq, emitted = _make_laq_with_spy(tool)
    m1 = _make_move(tool, [0., 0., 0., 0.], [10., 0., 0., 0.])
    m1.set_junction(0.0, m1.max_cruise_v2 * 0.5, 0.0)
    laq.queue.append(m1)
    laq.flush(lazy=True)
    if laq._pending_last is not None:
        assert laq._pending_last[0] is m1
        assert len(emitted) == 0


def test_drain_pending_on_empty_queue_lazy_false():
    """Regression: lazy=False must drain a pending move even when the
    queue is empty.

    Scenario: a prior lazy flush held `m_last` as the pending-last and
    emptied the queue. A subsequent lazy=False flush with no new moves
    must still finalize the pending with next=None and emit it. Otherwise
    the move is silently dropped at drain points (wait_moves,
    flush_step_generation, drip_move, etc.)."""
    tool = _FakeToolhead()
    laq, emitted = _make_laq_with_spy(tool)
    m1 = _make_move(tool, [0., 0., 0., 0.], [10., 0., 0., 0.])
    # Simulate "m1 was held as pending by a prior lazy flush".
    m1.set_junction(0.0, m1.max_cruise_v2 * 0.5, 0.0)
    laq._pending_last = (m1, None, None)
    # Queue is empty.
    assert len(laq.queue) == 0
    # Drain with lazy=False.
    laq.flush(lazy=False)
    # m1 must have been emitted and pending state cleared.
    assert emitted == [m1]
    assert laq._pending_last is None


def test_empty_queue_lazy_true_does_not_drain_pending():
    """Dual of test_drain_pending_on_empty_queue_lazy_false: lazy=True
    with an empty queue must NOT drain the pending — we might still get
    a follow-up move that provides the "next" neighbour. The pending
    stays held."""
    tool = _FakeToolhead()
    laq, emitted = _make_laq_with_spy(tool)
    m1 = _make_move(tool, [0., 0., 0., 0.], [10., 0., 0., 0.])
    m1.set_junction(0.0, m1.max_cruise_v2 * 0.5, 0.0)
    laq._pending_last = (m1, None, None)
    laq.flush(lazy=True)
    # Pending is still held; nothing emitted.
    assert emitted == []
    assert laq._pending_last is not None
    assert laq._pending_last[0] is m1


def test_drain_non_kinematic_pending_on_empty_queue():
    """Edge: if the pending somehow holds a non-bake-target (defensive
    path — shouldn't happen in production since only plain kinematic
    Moves get held), lazy=False still emits it without calling
    finalize_shape."""
    tool = _FakeToolhead()
    laq, emitted = _make_laq_with_spy(tool)
    # A pure-E move as pending — synthetic defensive scenario
    m_e = _make_move(tool, [0., 0., 0., 0.], [0., 0., 0., 5.])
    assert m_e.is_kinematic_move is False
    laq._pending_last = (m_e, None, None)
    laq.flush(lazy=False)
    assert emitted == [m_e]
    assert laq._pending_last is None


# ---------------------------------------------------------------------------
# Task 6 — A3 integration tests
# ---------------------------------------------------------------------------


def test_a3_e2e_mzv_shaper_bakes_payload_through_lookahead():
    """End-to-end A3 path: build toolhead with MZV shaper, queue a
    sequence of kinematic moves through LookAheadQueue, flush, and
    verify the resulting quintic_trapq_payload is shape-baked (the
    polynomial's coefficient buffer differs from the unshaped
    counterpart, proving the shaper convolution was applied).

    This is the primary A3 integration assertion: plain Moves that exit
    the LookAheadQueue flush pass must have their polynomial transformed
    by the shaper kernel, not pass-through.
    """
    tool = _make_toolhead_with_mzv()
    laq, emitted = _make_laq_with_spy(tool)

    m1 = _make_move(tool, [0., 0., 0., 0.], [30., 0., 0., 0.], speed=200.0)
    m2 = _make_move(tool, [30., 0., 0., 0.], [60., 0., 0., 0.], speed=200.0)
    m3 = _make_move(tool, [60., 0., 0., 0.], [90., 0., 0., 0.], speed=200.0)
    _inject_and_drain(laq, [m1, m2, m3])

    assert len(emitted) == 3, "all three moves must be emitted"
    for label, mv in (("m1", m1), ("m2", m2), ("m3", m3)):
        assert mv.quintic_trapq_payload is not None, (
            "%s: quintic_trapq_payload not set" % label
        )
        baked_coeffs = mv.quintic_trapq_payload[5]
        unshaped_coeffs = mv._unshaped_payload[2]
        assert baked_coeffs != unshaped_coeffs, (
            "%s: baked_coeffs == unshaped_coeffs — shape was NOT applied "
            "(MZV convolution should change the polynomial)" % label
        )


def test_a3_i3_coverage_gap_neighbour_changes_coeff_tuple():
    """I3 coverage-gap test (from the T5 reviewer): prove that a move's
    quintic_trapq_payload[5] (coeff_tuple) depends on its queue-neighbour
    presence.

    Scenario A: queue [m1, m2, m3] with MZV, lazy flush + drain.
        m1 is baked with m2 as its next neighbour (queue-internal).
        Capture m1's coeff_tuple → coeffs_with_neighbour.

    Scenario B: build fresh copies of the same moves; queue [m1_fresh]
        alone; lazy=False drain. m1 is baked with next=None (no neighbour).
        Capture m1's coeff_tuple → coeffs_alone.

    Assert coeffs_with_neighbour != coeffs_alone — proves the deferred-
    last pattern correctly propagates neighbour context, not just a zero-
    pad for every move.
    """
    # Scenario A — m1 baked with m2 as next neighbour
    tool_a = _make_toolhead_with_mzv()
    laq_a, emitted_a = _make_laq_with_spy(tool_a)
    m1_a = _make_move(tool_a, [0., 0., 0., 0.], [20., 0., 0., 0.], speed=200.0)
    m2_a = _make_move(tool_a, [20., 0., 0., 0.], [40., 0., 0., 0.], speed=200.0)
    m3_a = _make_move(tool_a, [40., 0., 0., 0.], [60., 0., 0., 0.], speed=200.0)
    _inject_and_drain(laq_a, [m1_a, m2_a, m3_a])
    assert m1_a in emitted_a, "m1 must be emitted in Scenario A"
    coeffs_with_neighbour = m1_a.quintic_trapq_payload[5]

    # Scenario B — m1_fresh alone, no neighbours
    tool_b = _make_toolhead_with_mzv()
    laq_b, emitted_b = _make_laq_with_spy(tool_b)
    m1_b = _make_move(tool_b, [0., 0., 0., 0.], [20., 0., 0., 0.], speed=200.0)
    _inject_and_drain(laq_b, [m1_b])
    assert m1_b in emitted_b, "m1 must be emitted in Scenario B"
    coeffs_alone = m1_b.quintic_trapq_payload[5]

    assert coeffs_with_neighbour != coeffs_alone, (
        "m1's coeff_tuple must differ when baked with a next-neighbour "
        "(Scenario A) vs. baked alone with next=None (Scenario B). "
        "If they are equal, the deferred-last neighbour context is not "
        "being used in the shaper convolution."
    )


def test_a3_ztilt_regression_1000mms_shape_baked():
    """Z-tilt regression scenario (structural test): queue a kinematic move
    at 1000 mm/s with an MZV shaper at ~42 Hz (Trident config), and verify
    the resulting polynomial is shape-baked (differs from unshaped). This is
    a structural correctness test — it proves the A3 hot path runs correctly
    for the specific velocity/shaper regime where the z_tilt stepper-slip
    regression manifested on the physical Trident printer.

    Hardware validation (confirming the stepper slip is gone) is out of scope
    for automated tests and must be done on the printer. This test verifies
    that a move at the regression velocity exits the planner with a shaped
    polynomial, which is the necessary precondition for the hardware fix.
    """
    tool = _make_toolhead_with_mzv()  # MZV 42 Hz — same as Trident config
    # Use high max_velocity to allow 1000 mm/s cruise
    tool.max_velocity = 1000.0
    tool.max_accel = 10000.0
    tool.max_accel_to_decel = 10000.0
    # Refresh shaper snapshot so the toolhead cache reflects updated params
    # (snapshot was captured at construction via shapers_snapshot already set)

    laq, emitted = _make_laq_with_spy(tool)

    # 100 mm kinematic move at 1000 mm/s — matches the z_tilt regression case
    m = _make_move(tool, [0., 0., 0., 0.], [100., 0., 0., 0.], speed=1000.0)
    # Manually call set_junction with aggressive jerk profile
    m.set_junction(0.0, 1000.0 ** 2, 0.0)
    laq.queue.append(m)
    laq.flush(lazy=False)

    assert len(emitted) == 1, "the move must be emitted"
    assert emitted[0] is m
    assert m.quintic_trapq_payload is not None, (
        "quintic_trapq_payload must be set after flush at 1000 mm/s"
    )
    # The baked polynomial must differ from the unshaped polynomial — proving
    # the MZV kernel was applied to the high-speed move.
    baked_coeffs = m.quintic_trapq_payload[5]
    unshaped_coeffs = m._unshaped_payload[2]
    assert baked_coeffs != unshaped_coeffs, (
        "baked_coeffs == unshaped_coeffs at 1000 mm/s — MZV shaper was NOT "
        "applied (z_tilt regression: this is the exact path that was "
        "emitting un-shaped polynomials, causing stepper slip)"
    )
