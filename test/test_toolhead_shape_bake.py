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
