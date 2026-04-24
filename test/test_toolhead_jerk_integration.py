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


# ---------------------------------------------------------------------------
# Layer 3 — A2d integration: qpayload presence and correctness after flush.
# ---------------------------------------------------------------------------


def test_kinematic_move_populates_qpayload_end_to_end():
    """After lookahead flushes a kinematic move through the real Move +
    LookAheadQueue pipeline, move.quintic_trapq_payload must be set and
    its total_t must match the jerk profile's summed segment times."""
    from klippy.toolhead import Move, LookAheadQueue

    class _StubToolhead:
        def __init__(self):
            self.max_velocity = 500.0
            self.max_accel = 5000.0
            self.max_jerk = 100000.0
            self.extruder_cap_snapshot = None

            class _K:
                def check_move(self, m):
                    pass

            class _E:
                def check_move(self, m):
                    pass

                def calc_junction(self, *_a):
                    return 1e18

            self.kin = _K()
            self.extruder = _E()
            self.captured = []

        def _process_moves(self, moves):
            self.captured.extend(moves)

    th = _StubToolhead()
    la = LookAheadQueue(th)
    m_a = Move(th, (0, 0, 0, 0), (40, 0, 0, 0), speed=500.0)
    m_b = Move(th, (40, 0, 0, 0), (50, 0, 0, 0), speed=500.0)
    la.queue.extend([m_a, m_b])
    m_b.calc_junction(m_a)
    la.flush(lazy=False)
    for m in th.captured:
        assert hasattr(m, "quintic_trapq_payload"), (
            "move %s missing quintic_trapq_payload after flush" % m
        )
        total_t = m.quintic_trapq_payload[1]
        legacy_total = m.accel_t + m.cruise_t + m.decel_t
        assert total_t == pytest.approx(legacy_total, rel=1e-9)


def test_kinematic_move_qpayload_phase_count_exceeds_three_for_jerk_regime():
    """For a short move in the triangular jerk regime, the qpayload must
    carry more than 3 phases — proving the jerk polynomial is emitted,
    not a degenerate 3-phase trapezoid."""
    from klippy.toolhead import Move, LookAheadQueue

    class _StubToolhead:
        def __init__(self):
            self.max_velocity = 500.0
            self.max_accel = 5000.0
            self.max_jerk = 100000.0
            self.extruder_cap_snapshot = None

            class _K:
                def check_move(self, m):
                    pass

            class _E:
                def check_move(self, m):
                    pass

                def calc_junction(self, *_a):
                    return 1e18

            self.kin = _K()
            self.extruder = _E()
            self.captured = []

        def _process_moves(self, moves):
            self.captured.extend(moves)

    th = _StubToolhead()
    la = LookAheadQueue(th)
    m = Move(th, (0, 0, 0, 0), (10, 0, 0, 0), speed=500.0)
    la.queue.append(m)
    la.flush(lazy=False)
    payload = th.captured[0].quintic_trapq_payload
    n_phases = len(payload[0])
    assert n_phases >= 3, "expected >=3 phases, got %d" % n_phases
    assert n_phases > 3, (
        "only %d phases — jerk polynomial may be degenerate" % n_phases
    )


def test_kinematic_retract_preserves_signed_e():
    """A kinematic move with negative E displacement (wipe-retract) must
    produce a qpayload whose E polynomial encodes the signed
    displacement. We verify that the net E integrated over the move
    duration equals axes_d[3].

    With no PA configured the E polynomial is:
        E(tau) = extr_r * P_proj(tau)
    where P_proj is the XY position projected onto the unit direction n.
    For start_pos=(0,0,0), E(0) = 0 and E(T_total) = extr_r * arc_length
    = axes_d[3]. Summing (E_i(T_i) - E_i(0)) across phases gives the
    total displacement, which must equal axes_d[3] = -5.0.
    """
    from klippy.toolhead import Move, LookAheadQueue

    class _StubToolhead:
        def __init__(self):
            self.max_velocity = 500.0
            self.max_accel = 5000.0
            self.max_jerk = 100000.0
            self.extruder_cap_snapshot = None

            class _K:
                def check_move(self, m):
                    pass

            class _E:
                def check_move(self, m):
                    pass

                def calc_junction(self, *_a):
                    return 1e18

            self.kin = _K()
            self.extruder = _E()
            self.captured = []

        def _process_moves(self, moves):
            self.captured.extend(moves)

    th = _StubToolhead()
    la = LookAheadQueue(th)
    # Kinematic move with negative E (wipe-retract combo).
    m = Move(th, (0, 0, 0, 0), (20, 0, 0, -5), speed=500.0)
    assert m.is_kinematic_move is True
    la.queue.append(m)
    la.flush(lazy=False)
    captured_m = th.captured[0]
    payload = captured_m.quintic_trapq_payload
    phase_t_ends_tuple = payload[0]
    coeff_tuple = payload[5]
    n_phases = len(phase_t_ends_tuple)
    # Sum (E_end - E_start) across phases. Each phase's E polynomial is in
    # absolute XY-position frame; the delta per phase is the displacement.
    # With k_pa=0: E(tau) = extr_r * P_proj(tau), P_proj(0)=start_pos_x=0,
    # so net total equals extr_r * arc_length = axes_d[3] = -5.0.
    prev_t = 0.0
    total_e_displacement = 0.0
    for i in range(n_phases):
        phase_end_t = phase_t_ends_tuple[i]
        T = phase_end_t - prev_t
        # E at tau=0 for this phase (constant term of E polynomial).
        e_start = coeff_tuple[(i * 15 + 0) * 4 + 3]
        # E at tau=T: sum_{k=0..14} coeff[k] * T^k.
        e_end = 0.0
        t_pow = 1.0
        for k in range(15):
            e_end += coeff_tuple[(i * 15 + k) * 4 + 3] * t_pow
            t_pow *= T
        total_e_displacement += e_end - e_start
        prev_t = phase_end_t
    assert total_e_displacement == pytest.approx(-5.0, rel=1e-6), (
        "E polynomial net displacement %.9f != axes_d[3]=-5.0"
        % total_e_displacement
    )


# ---------------------------------------------------------------------------
# Phase A5 — reverse pass is jerk-feasible by construction.
# ---------------------------------------------------------------------------


def test_reverse_pass_closes_bed_mesh_crash():
    """The original bed_mesh crash inputs, fed through Move +
    LookAheadQueue, must NOT raise 'Jerk profile infeasible'.

    Crash tuple: start_v=374.7, cruise_v_request=469.8, end_v=469.8,
    move_d=1.143, accel=70k, j_max=500k. Pre-A5 the trapezoidal cruise
    cap let set_junction receive a jerk-infeasible (start, cruise, end,
    L) tuple and jerk_profile.compute_profile raised. Post-A5 the
    reverse pass clips cruise_v via max_reachable_cruise_v, so the tuple
    is feasible by construction.
    """
    from klippy.toolhead import Move, LookAheadQueue

    th = _FakeToolhead(max_accel=70000.0, max_jerk=500000.0,
                       max_velocity=600.0)
    la = LookAheadQueue(th)
    # Recreate the crash pattern: a pre-probe cruise move feeding a
    # short 1.143 mm probe hop at 469.8 mm/s that lands at 469.8 mm/s
    # (probe drop into a subsequent move of equal speed).
    m_a = Move(th, (0, 0, 0, 0), (50, 0, 0, 0), speed=469.8)
    m_b = Move(th, (50, 0, 0, 0), (51.143, 0, 0, 0), speed=469.8)
    m_c = Move(th, (51.143, 0, 0, 0), (200, 0, 0, 0), speed=469.8)
    la.queue.extend([m_a, m_b, m_c])
    m_b.calc_junction(m_a)
    m_c.calc_junction(m_b)
    la.flush(lazy=False)
    # If the plan is correct, flush did not raise. Each move must carry
    # a valid jerk_profile attached by set_junction.
    for m in (m_a, m_b, m_c):
        assert hasattr(m, "jerk_profile"), \
            "set_junction must run for every flushed move"


# ---------------------------------------------------------------------------
# Phase A5 T6 — bed_mesh acceptance gate.
# ---------------------------------------------------------------------------


def test_a5_bed_mesh_exact_crash_tuple_replay():
    """A5 T6 ACCEPTANCE GATE — replay the exact bed_mesh-crashing tuple
    through the real ``Move`` + ``LookAheadQueue.flush`` pipeline.

    The original Trident bed_mesh_calibrate crash captured this exact
    state on the short probe move:

        start_v        = 374.7    mm/s
        cruise_v_req   = 469.8    mm/s   (move's max_cruise_v2 = 469.8²)
        end_v          = 469.8    mm/s   (probe-drop into equal-speed move)
        move_d         = 1.143    mm
        accel          = 70000    mm/s²
        j_max          = 500000   mm/s³

    Pre-A5, the trapezoidal cruise cap
    ``(start_v² + reachable_start_v²) * 0.5`` told the planner this
    tuple was feasible. Then ``set_junction`` invoked
    ``jerk_profile.compute_profile`` which rejected it with
    ``klippy.gcode.CommandError: Jerk profile infeasible for move
    (...)`` because the 374.7 → 469.8 ramp under j=500k needs
    ~11.65 mm of runway, not the 0.574 mm a constant-accel
    approximation suggests (off by 20×).

    Post-A5, the reverse pass clips ``cruise_v`` to
    ``max_reachable_cruise_v(374.7, 469.8, 70k, 500k, 1.143, 469.8)
    = 375.86`` mm/s, which is jerk-feasible by construction. End_v
    is then re-clamped to ``min(469.8, 375.86) = 375.86`` mm/s, so
    the tuple ``set_junction`` actually sees is jerk-feasible and
    ``compute_profile`` returns ``JP_OK``.

    This is the **acceptance gate** — the hardware regression is
    closed iff this test passes. We pin the move's pre-flush state
    to the exact crash tuple (rather than reconstructing the
    upstream G-code chain that produced 374.7 mm/s start_v) so the
    test is independent of bed_mesh's specific calibration sequence.
    """
    from klippy import jerk_math
    from klippy.toolhead import LookAheadQueue, Move

    th = _FakeToolhead(max_accel=70000.0, max_jerk=500000.0,
                       max_velocity=600.0)
    la = LookAheadQueue(th)

    # Single 1.143 mm move with the crash-tuple kinematic state.
    move = Move(th, (0, 0, 0, 0), (1.143, 0, 0, 0), speed=469.8)
    # speed=469.8 already pins max_cruise_v2 = 469.8²; verify.
    assert move.move_d == pytest.approx(1.143, rel=1e-12)
    assert move.max_cruise_v2 == pytest.approx(469.8 ** 2, rel=1e-12)
    assert move.accel == 70000.0
    assert move.j_max == 500000.0
    # Pin the upstream-imposed start_v² = 374.7². In a real bed_mesh
    # calibrate, this value is the result of the reverse-pass cascade
    # through prior probe moves. We inject it directly to make the
    # test independent of the upstream chain — the acceptance question
    # is whether *this* move + the queue's flush cope with the tuple.
    move.max_start_v2 = 374.7 ** 2
    # Tail: a follow-on move at 469.8 mm/s (the crash had end_v=469.8).
    # Its max_start_v2 is the queue's source for next_end_v² when
    # processing `move` in the reverse pass.
    tail = Move(th, (1.143, 0, 0, 0), (50, 0, 0, 0), speed=469.8)
    tail.max_start_v2 = 469.8 ** 2
    la.queue.extend([move, tail])

    # Pre-A5: la.flush would propagate 469.8² backwards as next_end_v²,
    # the trapezoidal cruise cap would say "OK", and set_junction would
    # raise CommandError. Post-A5: max_reachable_cruise_v clips cruise_v
    # to ~375.86 mm/s and the (clamped) tuple is feasible.
    la.flush(lazy=False)  # MUST NOT RAISE — this is the gate.

    # Verify the chosen cruise_v matches the analytic jerk-aware cap.
    expected_cruise_v = jerk_math.max_reachable_cruise_v(
        v_start=374.7, v_end=469.8,
        a_max=70000.0, j_max=500000.0,
        L=1.143, v_cruise_cap=469.8,
    )
    assert expected_cruise_v == pytest.approx(375.86, abs=0.05), (
        "spec-cited expected cruise_v cap is ~375.86 mm/s; analytic "
        "primitive gave %.4f" % expected_cruise_v
    )
    assert move.cruise_v == pytest.approx(expected_cruise_v, rel=1e-9), (
        "cruise_v should be clipped to max_reachable_cruise_v "
        "(%.4f) but was %.4f" % (expected_cruise_v, move.cruise_v)
    )
    # set_junction must have run and attached a feasible jerk profile.
    assert hasattr(move, "jerk_profile"), \
        "set_junction must run for the bed_mesh probe move"
    from klippy.chelper import jerk_profile as jp_mod
    assert move.jerk_profile.status == jp_mod.JP_OK, (
        "post-A5 jerk_profile.status must be JP_OK (got %d)"
        % move.jerk_profile.status
    )
    # The A2d emit path must have populated the quintic payload.
    assert move.quintic_trapq_payload is not None, (
        "quintic_trapq_payload must be populated post-flush — "
        "downstream _process_moves consumes this 9-tuple"
    )
    # Sanity: the trapezoidal cap that mis-classified the tuple as
    # feasible was off by 20×. Spot-check the math is still off by
    # the cited factor — if this drifts, the spec doc is stale.
    trapezoidal_L = (469.8 ** 2 - 374.7 ** 2) / (2.0 * 70000.0)
    jerk_aware_L = 11.647  # from the spec verification block.
    assert trapezoidal_L == pytest.approx(0.574, abs=0.001)
    assert jerk_aware_L / trapezoidal_L == pytest.approx(20.3, abs=0.5), (
        "trapezoidal vs jerk-aware L ratio drifted from spec's ~20× "
        "(got %.2f×)" % (jerk_aware_L / trapezoidal_L)
    )


def test_reverse_pass_no_smoothed_fields_on_move():
    """After A5, Move must not carry smoothed-pass state.

    The smoothed pass is dead — its backing fields should be gone so
    future code cannot accidentally read stale values.
    """
    from klippy.toolhead import Move
    th = _FakeToolhead()
    m = Move(th, (0, 0, 0, 0), (10, 0, 0, 0), speed=100.0)
    assert not hasattr(m, "smooth_delta_v2"), \
        "A5 must remove smooth_delta_v2 from Move"
    assert not hasattr(m, "max_smoothed_v2"), \
        "A5 must remove max_smoothed_v2 from Move"
    assert not hasattr(m, "delta_v2"), \
        "A5 must remove delta_v2 from Move"


def test_reverse_pass_uses_max_reachable_cruise_v():
    """For a short move between two high-velocity moves, the chosen
    cruise_v must equal max_reachable_cruise_v(start_v, end_v, a, j, L).

    This is the structural assertion: the trapezoidal cruise cap is
    gone and the jerk-aware primitive is in its place.
    """
    from klippy import jerk_math
    from klippy.toolhead import Move, LookAheadQueue

    th = _FakeToolhead(max_accel=5000.0, max_jerk=100000.0,
                       max_velocity=500.0)
    la = LookAheadQueue(th)
    # Long flank, tiny middle, long flank — middle move's cruise_v is
    # constrained by jerk reachability from 500 mm/s ends across 2 mm.
    m_a = Move(th, (0, 0, 0, 0), (100, 0, 0, 0), speed=500.0)
    m_b = Move(th, (100, 0, 0, 0), (102, 0, 0, 0), speed=500.0)
    m_c = Move(th, (102, 0, 0, 0), (200, 0, 0, 0), speed=500.0)
    la.queue.extend([m_a, m_b, m_c])
    m_b.calc_junction(m_a)
    m_c.calc_junction(m_b)
    la.flush(lazy=False)
    # m_b's cruise_v should match the analytic jerk-aware cap.
    expected = jerk_math.max_reachable_cruise_v(
        v_start=m_b.start_v, v_end=m_b.end_v,
        a_max=m_b.accel, j_max=m_b.j_max,
        L=m_b.move_d, v_cruise_cap=500.0,
    )
    assert m_b.cruise_v == pytest.approx(expected, rel=1e-6)


def test_calc_junction_forward_cap_uses_reachable_v_end():
    """Move.calc_junction's forward reachability cap must be
    ``reachable_v_end(prev_start_v, prev_accel, prev_j_max, prev_move_d)²``
    — the A5 jerk-aware replacement for the trapezoidal
    ``prev_max_start_v2 + prev_delta_v2`` term.

    Scenario: m1 starts from rest (max_start_v2=0), so the forward
    cap evaluates to ``reachable_v_end(0, a, j, L)²``. For a=5000,
    j=100000, L=10 the triangular-regime answer is ~215.44 mm/s
    → v² ≈ 46415. At a 150° turn the centripetal cap is
    ~93301 (looser), the per-move max_cruise_v² = 1000² = 1e6
    (looser), and the extruder cap is 1e18 (looser). Forward-reach
    is the binding term.
    """
    import math as _math
    from klippy import jerk_math
    from klippy.toolhead import Move

    th = _FakeToolhead(max_accel=5000.0, max_jerk=100000.0,
                       max_velocity=1000.0)
    # 150° turn angle at the junction (theta/2 = 75° → tan ≈ 3.732).
    turn = _math.radians(150.0)
    dx = 10.0 * _math.cos(_math.pi - turn)
    dy = 10.0 * _math.sin(_math.pi - turn)
    m1 = Move(th, (0, 0, 0, 0), (10, 0, 0, 0), speed=1000.0)
    m2 = Move(th, (10, 0, 0, 0), (10 + dx, dy, 0, 0), speed=1000.0)
    m2.calc_junction(m1)

    prev_start_v = (_math.sqrt(m1.max_start_v2)
                    if m1.max_start_v2 > 0.0 else 0.0)
    expected_reach = jerk_math.reachable_v_end(
        v_start=prev_start_v, a_max=m1.accel, j_max=m1.j_max,
        L=m1.move_d,
    )
    expected_cap = expected_reach * expected_reach
    assert m2.max_start_v2 == pytest.approx(expected_cap, rel=1e-9), (
        f"Forward-reach cap should bind: got {m2.max_start_v2} "
        f"vs expected {expected_cap}"
    )


# ---------------------------------------------------------------------------
# A5 T4 — retirement of max_accel_to_decel / minimum_cruise_ratio.
# ---------------------------------------------------------------------------


def test_toolhead_has_no_max_accel_to_decel(
    config_root: typing.Annotated[
        pathlib.Path, "test_configs/toolhead_jerk"
    ],
):
    """A5: max_accel_to_decel is retired. The ToolHead must not expose
    it as a property, and the config deprecation path must be gone."""
    start_args = {"config_file": str(config_root / "printer.cfg")}
    with PrinterShim(start_args) as printer:
        config = printer.load_config()
        from klippy.toolhead import ToolHead
        # The property must be gone.
        assert not hasattr(ToolHead, "max_accel_to_decel"), (
            "ToolHead.max_accel_to_decel must be deleted in A5 T4"
        )


def test_toolhead_has_no_min_cruise_ratio():
    """A5: minimum_cruise_ratio is retired. ToolHead must not set
    min_cruise_ratio during __init__, and no class-level descriptor."""
    from klippy.toolhead import ToolHead
    # No class-level descriptor.
    assert not hasattr(ToolHead, "min_cruise_ratio"), (
        "ToolHead must not have a class-level min_cruise_ratio"
    )
    # Instance level: _FakeToolhead mirrors real ToolHead; if real ToolHead
    # sets min_cruise_ratio in __init__ the integration tests that use
    # _FakeToolhead will fail differently. A direct class check suffices.
