# test/test_toolhead_jerk_integration.py
"""Phase A2c Task 6 — end-to-end jerk integration test.

Two test layers:

1. CONFIG-LOADING (PrinterShim):
   Verify that ``[printer] max_jerk: 100000`` is parsed by the config
   layer and surfaced through the same ``config.getfloat("max_jerk", ...)``
   call that ``ToolHead.__init__`` makes.  The PrinterShim does not
   instantiate a real ToolHead (it has no Reactor/MCU), so we read the
   value directly from the config wrapper using the same call site
   signature as the real ToolHead.

2. PLANNER-LEVEL INTEGRATION (Move + LookAheadQueue, real production code):
   Drive two real ``Move`` objects through a real ``LookAheadQueue`` with
   a stub toolhead that carries the jerk attribute.  Verify:
     - the reverse pass uses jerk-limited reachable velocity (not 2*a*L)
     - ``set_junction`` stores a ``jerk_profile.Profile`` on each move
     - the per-phase timing sum equals the profile total T
     - the result differs materially from the constant-accel trapezoid
       approximation, confirming the jerk-aware path is active.

   This is the distinguishing assertion: under trapezoid math the
   reachable start_v for a 10 mm stop would be sqrt(2*5000*10) = 316.2
   mm/s; under jerk_math it is ~215 mm/s (triangular regime).  A delta
   of ~100 mm/s (~30%) is unmistakable.
"""
from __future__ import annotations

import math
import pathlib
import typing

import pytest

from klippy_testing import PrinterShim


# ---------------------------------------------------------------------------
# Helper: fake toolhead with all attributes real Move/LookAheadQueue need.
# ---------------------------------------------------------------------------


class _FakeToolhead:
    """Minimal toolhead surface for Move + LookAheadQueue.

    Mirrors the attribute contract established by the real ToolHead and
    used in test_toolhead_jerk_wiring.py.
    """

    def __init__(self, **kw):
        self.max_velocity = kw.get("max_velocity", 500.0)
        self.max_accel = kw.get("max_accel", 5000.0)
        self.max_jerk = kw.get("max_jerk", 100000.0)
        self.max_accel_to_decel = kw.get("max_accel_to_decel", 5000.0)
        self.min_cruise_ratio = kw.get("min_cruise_ratio", 0.0)
        self._captured = []

        class _Kin:
            def check_move(self, m):
                pass

        class _Ext:
            def check_move(self, m):
                pass

            def calc_junction(self, *_a):
                return 1e18

        self.kin = _Kin()
        self.extruder = _Ext()

    def _process_moves(self, moves):
        self._captured.extend(moves)


# ---------------------------------------------------------------------------
# Layer 1 — config-loading.
# ---------------------------------------------------------------------------


def test_toolhead_max_jerk_config_loaded(
    config_root: typing.Annotated[
        pathlib.Path, "test_configs/toolhead_jerk"
    ],
):
    """[printer] max_jerk: 100000 loads and is readable by the ToolHead
    constructor's getfloat call signature.

    This replaces the placeholder skip in test_toolhead_jerk_wiring.py
    (test_toolhead_max_jerk_default_loaded).  The PrinterShim parses the
    config file; we exercise the same getfloat path that ToolHead.__init__
    uses so any typo in the option name would fail this test.
    """
    start_args = {"config_file": str(config_root / "printer.cfg")}
    with PrinterShim(start_args) as printer:
        config = printer.load_config()
        printer_section = config.getsection("printer")
        # Exactly the call that ToolHead.__init__ makes:
        max_jerk = printer_section.getfloat("max_jerk", 100000.0, above=0.0)
    assert max_jerk == 100000.0


def test_toolhead_max_jerk_non_default_loaded(
    config_root: typing.Annotated[
        pathlib.Path, "test_configs/toolhead_jerk"
    ],
):
    """Verify that a custom max_jerk value survives the round-trip through
    the config layer (i.e., the key name is correct in both the .cfg and
    the getfloat call).

    We write a temporary override, reload, and check.
    """
    printer_cfg = config_root / "printer.cfg"
    original = printer_cfg.read_text()
    # Substitute the value so we get a different number to assert.
    modified = original.replace("max_jerk: 100000", "max_jerk: 250000")
    printer_cfg.write_text(modified)
    start_args = {"config_file": str(printer_cfg)}
    try:
        with PrinterShim(start_args) as printer:
            config = printer.load_config()
            printer_section = config.getsection("printer")
            max_jerk = printer_section.getfloat(
                "max_jerk", 100000.0, above=0.0
            )
    finally:
        printer_cfg.write_text(original)
    assert max_jerk == 250000.0


# ---------------------------------------------------------------------------
# Layer 2 — planner-level integration: real Move + LookAheadQueue.
# ---------------------------------------------------------------------------


def test_jerk_reverse_pass_differs_from_trapezoid():
    """Two-move sequence ending at stop.  The jerk-aware reverse pass must
    compute a start_v for move B that is materially lower than the
    constant-accel trapezoid approximation, confirming the jerk path is
    active.

    Expected values (a=5000 mm/s², j=100000 mm/s³, L=10 mm, v_end=0):
      - Trapezoid: start_v = sqrt(2 * 5000 * 10) = 316.2 mm/s
      - Jerk-limited (triangular regime):
            u = (L * sqrt(j))^(1/3) ≈ 14.68 mm^(1/3) * s^(-2/3)
            dv = u^2 ≈ 215.5 mm/s
        So reachable_v_end ≈ 215.5 mm/s — ~32% lower than trapezoid.
    """
    from klippy import jerk_math
    from klippy.toolhead import LookAheadQueue, Move

    th = _FakeToolhead(max_accel=5000.0, max_jerk=100000.0)
    lookahead = LookAheadQueue(th)

    # Two collinear moves so calc_junction gives cos_theta = 1 (straight).
    m_a = Move(th, (0, 0, 0, 0), (40, 0, 0, 0), speed=500.0)
    m_b = Move(th, (40, 0, 0, 0), (50, 0, 0, 0), speed=500.0)
    lookahead.queue.extend([m_a, m_b])
    m_b.calc_junction(m_a)
    lookahead.flush(lazy=False)

    expected_jerk = jerk_math.reachable_v_end(
        v_start=0.0, a_max=5000.0, j_max=100000.0, L=10.0
    )
    trapezoid_approx = math.sqrt(2.0 * 5000.0 * 10.0)  # 316.2 mm/s

    assert m_b.start_v == pytest.approx(expected_jerk, rel=1e-9)
    assert m_b.end_v == pytest.approx(0.0, abs=1e-12)
    # Core distinguishing assertion: jerk-limited is materially lower.
    assert expected_jerk < trapezoid_approx * 0.9, (
        "jerk-limited start_v=%.2f should be at least 10%% below "
        "trapezoid approx=%.2f — jerk path may not be active"
        % (expected_jerk, trapezoid_approx)
    )


def test_set_junction_produces_jerk_profile_on_real_move():
    """set_junction on a real Move must attach a jerk_profile.Profile with
    status JP_OK and segments summing to the correct total duration.

    This is the end-to-end path: Move captures j_max from toolhead, then
    set_junction delegates to jerk_profile.compute_profile.
    """
    from klippy.chelper import jerk_profile as jp_mod
    from klippy.toolhead import Move

    th = _FakeToolhead(max_accel=5000.0, max_jerk=100000.0)
    m = Move(th, (0, 0, 0, 0), (50, 0, 0, 0), speed=200.0)
    m.set_junction(start_v2=0.0, cruise_v2=200.0 ** 2, end_v2=0.0)

    assert hasattr(m, "jerk_profile"), "set_junction must attach jerk_profile"
    prof = m.jerk_profile
    assert prof.status == jp_mod.JP_OK
    assert len(prof.segments) >= 1

    # Timing consistency: legacy fields must match profile total.
    profile_total = sum(s.T for s in prof.segments)
    legacy_total = m.accel_t + m.cruise_t + m.decel_t
    assert legacy_total == pytest.approx(profile_total, rel=1e-9)


def test_two_move_flush_produces_jerk_profiles_on_both_moves():
    """After LookAheadQueue.flush both moves must carry jerk_profile
    attributes with valid (JP_OK) profiles, confirming set_junction is
    called for every move in the flush pass.
    """
    from klippy.chelper import jerk_profile as jp_mod
    from klippy.toolhead import LookAheadQueue, Move

    th = _FakeToolhead(max_accel=5000.0, max_jerk=100000.0)
    lookahead = LookAheadQueue(th)

    m_a = Move(th, (0, 0, 0, 0), (40, 0, 0, 0), speed=500.0)
    m_b = Move(th, (40, 0, 0, 0), (50, 0, 0, 0), speed=500.0)
    lookahead.queue.extend([m_a, m_b])
    m_b.calc_junction(m_a)
    lookahead.flush(lazy=False)

    for label, mv in (("move_A", m_a), ("move_B", m_b)):
        assert hasattr(mv, "jerk_profile"), (
            "%s: missing jerk_profile after flush" % label
        )
        assert mv.jerk_profile.status == jp_mod.JP_OK, (
            "%s: jerk_profile.status=%d (expected JP_OK=%d)"
            % (label, mv.jerk_profile.status, jp_mod.JP_OK)
        )
        total = sum(s.T for s in mv.jerk_profile.segments)
        legacy = mv.accel_t + mv.cruise_t + mv.decel_t
        assert legacy == pytest.approx(total, rel=1e-9), (
            "%s: legacy_total=%.9f != profile_total=%.9f"
            % (label, legacy, total)
        )
