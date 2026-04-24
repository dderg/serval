"""Plan 9 A3-followup — full production pipeline integration test.

All existing A3 tests inject moves directly into ``LookAheadQueue.queue``,
bypassing the production filter stack.  This file provides tests that exercise
the REAL entry path:

    BlendPipelineLookAheadQueue  →  [CollinearCollapser, CornerBlender]
                                 →  inner LookAheadQueue
                                 →  _process_moves spy

== BUG SURFACED ==

``test_full_pipeline_with_mzv_shaper_exercises_all_stages`` surfaces a
production bug: ``LookAheadQueue.flush`` calls ``move.reachable_v_from_v_end``
and ``move.j_max`` on every move in the queue, but ``QuinticBlendMove``
(emitted by ``CornerBlender``) does not have these attributes.

In the real production stack, ``CornerBlender`` finalises and emits
``QuinticBlendMove`` objects through ``BlendPipelineLookAheadQueue.add_move``
→ ``inner_LookAheadQueue.add_move(m)`` → appended to ``inner.queue``.  When
``inner.flush()`` subsequently runs its reverse-pass loop, it crashes on any
``QuinticBlendMove`` in the queue.

The correct fix is to add ``reachable_v_from_v_end`` and ``j_max`` (or an
equivalent guard) to ``QuinticBlendMove``.  For ``QuinticBlendMove`` the
kinematics are pre-baked by TOPP (v_in/v_out/cruise_v are locked in by
Option-Z at emit time) so the reverse-pass cannot usefully change the profile;
the method should return ``v_end`` unchanged (the blend is already velocity-
safe from TOPP) or a sufficiently large value that the existing constraints
win.  A ``j_max`` that is large (``float("inf")`` or ``toolhead.max_jerk``)
is also safe since the TOPP pass already saturated the jerk-based
centripetal cap.

**Verdict: DONE_WITH_CONCERNS** — test file committed; production bug
documented in the test; requires a follow-up fix in ``QuinticBlendMove``.
"""
from __future__ import annotations

import math

import pytest

from klippy import blendmath, blendplanner, blendprepass
from klippy.toolhead import LookAheadQueue, Move


# ---------------------------------------------------------------------------
# Harness: fake toolhead with full pipeline wired up.
# ---------------------------------------------------------------------------


class _FakePrinter:
    """Minimal printer surface for Move.set_junction error paths and
    blendmath.extract_shapers."""

    command_error = Exception

    def __init__(self, is_obj):
        self._is = is_obj

    def lookup_object(self, name, default=None):
        if name == "input_shaper":
            return self._is
        return default


class _FakeAxisShaper:
    """Mirrors AxisInputShaper's blendmath-visible surface."""

    def __init__(self, axis, shaper_type, freq, damping=0.1):
        self._axis = axis

        class _P:
            pass

        self.params = _P()
        self.params.shaper_type = shaper_type
        self.params.shaper_freq = freq
        self.params.damping_ratio = damping

    def get_axis(self):
        return self._axis

    def get_type(self):
        return self.params.shaper_type


class _FakeIS:
    def __init__(self, shapers):
        self._shapers = shapers

    def get_shapers(self):
        return list(self._shapers)


class _FakeKin:
    def check_move(self, m):
        pass


class _FakeExtruder:
    def check_move(self, m):
        pass

    def calc_junction(self, *_a):
        return 1e18

    def get_status(self, eventtime=None):
        return {
            "pressure_advance": 0.0,
            "pressure_advance_model": "linear",
            "pressure_advance_smooth_time": 0.0,
        }


class _PipelineToolhead:
    """Toolhead stub that wires the full production filter pipeline.

    Attributes match what CollinearCollapser / CornerBlender / LookAheadQueue
    / Move all read.  ``shapers_snapshot`` is populated once at construction
    via ``blendmath.extract_shapers`` so ``Move.__init__`` picks it up.

    The public ``laq`` is a real ``BlendPipelineLookAheadQueue``; call
    ``laq.add_move(m)`` to exercise the filter stack.
    ``captured`` accumulates every move batch delivered to ``_process_moves``.
    """

    def __init__(self, shaper_type="mzv", freq=42.0):
        self.max_velocity = 500.0
        self.max_accel = 10000.0
        self.max_accel_to_decel = 10000.0
        self.max_jerk = 100000.0
        self.corner_deviation = 0.1      # mm — same as blendplanner test fixtures
        self.extruder_cap_snapshot = None

        # Wire input_shaper via printer so blendmath.extract_shapers finds it.
        is_obj = _FakeIS([
            _FakeAxisShaper("x", shaper_type, freq),
            _FakeAxisShaper("y", shaper_type, freq),
        ])
        self.printer = _FakePrinter(is_obj)
        self.kin = _FakeKin()
        self.extruder = _FakeExtruder()

        # Populate the shaper snapshot cache on the toolhead so
        # Move.__init__ picks it up O(1) without calling extract_shapers.
        self.shapers_snapshot = blendmath.extract_shapers(self)

        # Build the full production pipeline.
        inner_queue = LookAheadQueue(self)
        prepass = blendprepass.CollinearCollapser(self, move_cls=Move)
        blender = blendplanner.CornerBlender(self, move_cls=Move)
        self.laq = blendprepass.BlendPipelineLookAheadQueue(
            [prepass, blender], inner_queue
        )
        # References for introspection in tests.
        self._prepass = prepass
        self._blender = blender
        self._inner_laq = inner_queue

        self.captured = []

    def _process_moves(self, moves):
        self.captured.extend(moves)


# ---------------------------------------------------------------------------
# Integration test — exercises all filter stages.
# ---------------------------------------------------------------------------


def test_full_pipeline_with_mzv_shaper_exercises_all_stages():
    """Integration test: moves enter via BlendPipelineLookAheadQueue.add_move,
    traverse CollinearCollapser → CornerBlender → LookAheadQueue.

    ** THIS TEST SURFACES A PRODUCTION BUG **

    ``LookAheadQueue.flush`` calls ``move.reachable_v_from_v_end`` and reads
    ``move.j_max`` on every move in its queue.  ``QuinticBlendMove`` (emitted
    by ``CornerBlender`` and forwarded to the inner queue) does not have
    ``reachable_v_from_v_end`` or ``j_max``.  The flush crashes with
    ``AttributeError: 'QuinticBlendMove' object has no attribute
    'reachable_v_from_v_end'``.

    The test is left in *failing* state intentionally: it is the regression
    test that must be unblocked once ``QuinticBlendMove`` grows
    ``reachable_v_from_v_end`` (returning ``v_end`` — TOPP has already
    saturated velocity) and ``j_max`` (``float("inf")`` is correct since the
    jerk cap was applied at blend-emit time).

    Failure modes this test guards against once the bug is fixed:
    - BlendPipelineLookAheadQueue bypassed → blender.blends_emitted stays 0.
    - CornerBlender bypassed → no QuinticBlendMove in captured.
    - Inner LookAheadQueue shape-bake pass bypassed → plain Move payloads
      have baked_coeffs == unshaped_coeffs.
    - Shaper not picked up → toolhead.shapers_snapshot empty.
    """
    th = _PipelineToolhead(shaper_type="mzv", freq=42.0)

    # Pre-check: shaper snapshot must be non-empty.
    assert len(th.shapers_snapshot) > 0, (
        "shapers_snapshot is empty — blendmath.extract_shapers did not find "
        "the MZV shaper on the toolhead.printer.  Shaper config not picked up."
    )

    # Three-move L-shape: A-B is a 90° corner; B-C is another 90° corner.
    th_speed = 200.0
    side = 20.0
    m_a = Move(th, [0.0, 0.0, 0.0, 0.0], [side, 0.0, 0.0, 0.0], th_speed)
    m_b = Move(th, [side, 0.0, 0.0, 0.0], [side, side, 0.0, 0.0], th_speed)
    m_c = Move(th, [side, side, 0.0, 0.0], [0.0, side, 0.0, 0.0], th_speed)

    # Moves enter via the production entry point — not via queue injection.
    th.laq.add_move(m_a)
    th.laq.add_move(m_b)
    th.laq.add_move(m_c)

    # This flush currently raises:
    #   AttributeError: 'QuinticBlendMove' object has no attribute
    #   'reachable_v_from_v_end'
    # because LookAheadQueue.flush's reverse-pass loop calls that method on
    # every move in the inner queue, and QuinticBlendMove is now in that queue.
    #
    # Expected fix: add reachable_v_from_v_end(v_end) → v_end and j_max to
    # QuinticBlendMove.  Then remove the xfail mark and add the assertions
    # below.
    th.laq.flush(lazy=False)

    # --- Post-bug-fix assertions (currently unreachable) --------------------

    # Assertion 2: CornerBlender ran and emitted at least one blend.
    assert th._blender.blends_emitted >= 1, (
        "CornerBlender.blends_emitted == 0 after a 90° corner — the filter "
        "was bypassed or CornerBlender.feed never ran through "
        "BlendPipelineLookAheadQueue."
    )

    # Assertion 3: inner LookAheadQueue flush delivered moves.
    assert len(th.captured) > 0, (
        "_process_moves was never called — inner LookAheadQueue.flush did not "
        "run, or BlendPipelineLookAheadQueue.flush never reached the inner queue."
    )

    # Partition: plain kinematic Moves vs QuinticBlendMove instances.
    plain_moves = [m for m in th.captured if isinstance(m, Move)]
    quintic_moves = [
        m for m in th.captured
        if isinstance(m, blendplanner.QuinticBlendMove)
    ]

    # At least one QuinticBlendMove confirms CornerBlender produced a blend.
    assert len(quintic_moves) >= 1, (
        "No QuinticBlendMove in captured output — CornerBlender may have "
        "suppressed all corners (check corner_deviation / segment length) or "
        "the filter was bypassed."
    )

    # At least one plain kinematic Move in captured output.
    assert len(plain_moves) >= 1, (
        "Expected at least one plain kinematic Move in captured output."
    )

    # Assertion 4: shape-bake applied to plain kinematic Moves.
    shaped_count = 0
    for m in plain_moves:
        assert m.quintic_trapq_payload is not None, (
            "plain Move has quintic_trapq_payload=None after flush."
        )
        baked_coeffs = m.quintic_trapq_payload[5]
        unshaped_coeffs = m._unshaped_payload[2]
        if baked_coeffs != unshaped_coeffs:
            shaped_count += 1

    assert shaped_count >= 1, (
        "All plain kinematic Moves have baked_coeffs == unshaped_coeffs — "
        "the MZV shaper was NOT applied in the LookAheadQueue shape-bake pass."
    )

    # QuinticBlendMove: confirm baked payload is set and well-formed.
    for qm in quintic_moves:
        assert qm.quintic_trapq_payload is not None
        payload = qm.quintic_trapq_payload
        assert len(payload) == 9
        assert payload[1] > 0.0   # total_t positive
        assert payload[2] > 0.0   # arc_length positive


# ---------------------------------------------------------------------------
# Tests that do NOT require a fix — they test the layers that already work.
# ---------------------------------------------------------------------------


def test_pipeline_shapers_snapshot_nonempty_means_moves_see_mzv():
    """Regression guard: toolhead.shapers_snapshot must be non-empty when an
    MZV shaper is configured, and every Move constructed after that must
    inherit a non-empty _shapers_snapshot.

    This would fail if _PipelineToolhead forgot to call
    blendmath.extract_shapers or if Move.__init__ no longer reads
    toolhead.shapers_snapshot.
    """
    th = _PipelineToolhead(shaper_type="mzv", freq=42.0)
    assert len(th.shapers_snapshot) > 0, (
        "toolhead.shapers_snapshot is empty with MZV configured"
    )
    m = Move(th, [0.0, 0.0, 0.0, 0.0], [10.0, 0.0, 0.0, 0.0], 200.0)
    assert len(m._shapers_snapshot) > 0, (
        "Move._shapers_snapshot is empty — Move.__init__ not reading "
        "toolhead.shapers_snapshot"
    )
    from klippy.blendshaper import AxisShaperSnapshot
    for snap in m._shapers_snapshot:
        assert isinstance(snap, AxisShaperSnapshot), (
            "Expected AxisShaperSnapshot, got %r" % type(snap)
        )


def test_pipeline_corner_blender_fires_on_ninety_degree_corner():
    """Structural: CornerBlender sees a 90-degree corner and emits exactly
    one QuinticBlendMove into whatever its downstream receives.

    This exercises CornerBlender.feed with real Move objects (not
    _FakeMove stubs) and confirms the blend-planning path runs without
    crashing up to the point where the QuinticBlendMove would enter the
    inner LookAheadQueue.

    We call CornerBlender directly here (not through the full
    BlendPipelineLookAheadQueue) to avoid the production bug in
    LookAheadQueue.flush (see test_full_pipeline_with_mzv_shaper_exercises_all_stages).
    """
    th = _PipelineToolhead(shaper_type="mzv", freq=42.0)

    speed = 200.0
    side = 20.0
    m_prev = Move(th, [0.0, 0.0, 0.0, 0.0], [side, 0.0, 0.0, 0.0], speed)
    m_next = Move(th, [side, 0.0, 0.0, 0.0], [side, side, 0.0, 0.0], speed)

    cb = blendplanner.CornerBlender(th, move_cls=Move)
    out_feed_1 = cb.feed(m_prev)
    assert out_feed_1 == [], (
        "First feed must buffer m_prev, emitting nothing"
    )
    out_feed_2 = cb.feed(m_next)
    # After the second move, CornerBlender emits [trunc_prev].
    # The QuinticBlendMove is deferred in _pending_quintic.
    assert len(out_feed_2) >= 1, (
        "Expected at least trunc_prev emitted on second feed"
    )
    for m in out_feed_2:
        assert isinstance(m, Move), (
            "Feed output before flush must be plain Move (trunc_prev), "
            "got %r" % type(m)
        )

    # Flushing drains the pending QuinticBlendMove.
    out_flush = cb.flush()
    assert cb.blends_emitted == 1, (
        "Expected exactly 1 blend for one 90° corner, got %d"
        % cb.blends_emitted
    )
    # Flush returns [quintic_move, trunc_next_head] or just [quintic_move]
    # depending on whether there is still a _prev buffered.
    quintic_moves = [m for m in out_flush
                     if isinstance(m, blendplanner.QuinticBlendMove)]
    assert len(quintic_moves) == 1, (
        "Expected 1 QuinticBlendMove from flush, got %d" % len(quintic_moves)
    )
    qm = quintic_moves[0]
    assert qm.quintic_trapq_payload is not None, (
        "QuinticBlendMove.quintic_trapq_payload must be set"
    )
    payload = qm.quintic_trapq_payload
    assert len(payload) == 9
    assert payload[1] > 0.0   # total_t
    assert payload[2] > 0.0   # arc_length


def test_pipeline_collinear_collapser_merges_before_corner_blender():
    """Structural: CollinearCollapser merges three collinear +X moves into
    one before CornerBlender sees them.

    We call CollinearCollapser directly and check that the chain of three
    moves flushes to exactly one merged Move.  This confirms CollinearCollapser
    is correctly wired with real Move objects (the merge path calls
    kin.check_move and extruder.check_move on the merged move).
    """
    th = _PipelineToolhead(shaper_type="mzv", freq=42.0)

    speed = 200.0
    m1 = Move(th, [0.0,  0.0, 0.0, 0.0], [10.0, 0.0, 0.0, 0.0], speed)
    m2 = Move(th, [10.0, 0.0, 0.0, 0.0], [20.0, 0.0, 0.0, 0.0], speed)
    m3 = Move(th, [20.0, 0.0, 0.0, 0.0], [30.0, 0.0, 0.0, 0.0], speed)

    collapser = blendprepass.CollinearCollapser(th, move_cls=Move)
    assert collapser.feed(m1) == []
    assert collapser.feed(m2) == []
    assert collapser.feed(m3) == []

    # A non-collinear move triggers the flush+chain-reset, emitting the
    # merged move.
    m4 = Move(th, [30.0, 0.0, 0.0, 0.0], [30.0, 30.0, 0.0, 0.0], speed)
    flushed = collapser.feed(m4)
    # Three collinear moves → one merged move.
    assert len(flushed) == 1, (
        "Expected 1 merged move from collinear collapse, got %d" % len(flushed)
    )
    merged = flushed[0]
    assert isinstance(merged, Move), (
        "Merged result must be a Move, got %r" % type(merged)
    )
    # The merged move spans the full +X extent.
    assert merged.start_pos[:3] == pytest.approx([0.0, 0.0, 0.0], abs=1e-9)
    assert merged.end_pos[:3] == pytest.approx([30.0, 0.0, 0.0], abs=1e-9)


def test_pipeline_quintic_blend_move_missing_reverse_pass_attributes():
    """Documents the production bug: QuinticBlendMove lacks the attributes
    that LookAheadQueue.flush's reverse-pass loop requires.

    This test verifies the bug is present (it should pass until the bug is
    fixed).  Once QuinticBlendMove grows reachable_v_from_v_end and j_max,
    this test should be inverted / deleted and
    test_full_pipeline_with_mzv_shaper_exercises_all_stages un-xfail'd.

    Bug: LookAheadQueue.flush calls move.reachable_v_from_v_end() and reads
    move.j_max on every move in the queue.  CornerBlender emits
    QuinticBlendMove objects that enter the inner LookAheadQueue via
    BlendPipelineLookAheadQueue.add_move → inner.add_move.  When flush runs
    its reverse-pass loop, it crashes on QuinticBlendMove.
    """
    th = _PipelineToolhead(shaper_type="mzv", freq=42.0)

    # Build a QuinticBlendMove via CornerBlender (the normal production path).
    speed = 200.0
    side = 20.0
    m_prev = Move(th, [0.0, 0.0, 0.0, 0.0], [side, 0.0, 0.0, 0.0], speed)
    m_next = Move(th, [side, 0.0, 0.0, 0.0], [side, side, 0.0, 0.0], speed)

    cb = blendplanner.CornerBlender(th, move_cls=Move)
    cb.feed(m_prev)
    cb.feed(m_next)
    quintics = [m for m in cb.flush()
                if isinstance(m, blendplanner.QuinticBlendMove)]
    assert len(quintics) == 1, "Expected 1 QuinticBlendMove from CornerBlender"
    qm = quintics[0]

    # Confirm the missing attributes.
    assert not hasattr(qm, "reachable_v_from_v_end"), (
        "QuinticBlendMove now has reachable_v_from_v_end — "
        "the production bug is fixed.  Delete this test and "
        "un-xfail test_full_pipeline_with_mzv_shaper_exercises_all_stages."
    )
    assert not hasattr(qm, "j_max"), (
        "QuinticBlendMove now has j_max — "
        "the production bug is fixed.  Delete this test and "
        "un-xfail test_full_pipeline_with_mzv_shaper_exercises_all_stages."
    )
