"""Plan 9 A3 — shape-everywhere tests."""
import pytest
import os
import sys

_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

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
    move = _make_move(tool, [0., 0., 0., 0.], [20., 0., 0., 0.])
    # Snapshot must be captured (exact format is blendmath's — here we
    # just assert non-empty)
    assert len(move._shapers_snapshot) == 2
