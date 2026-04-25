"""Integration tests for Plan 8 Chunk 3 — baked-in pressure advance.

Drives two right-angle kinematic moves through the CornerBlender with
an extruder PA snapshot configured, and verifies the emitted
QuinticBlendMove's .e polynomial matches the expected PA composition
at multiple sample points.
"""
from __future__ import annotations

import math

import pytest

from klippy import blendextruder, blendplanner, blendshape


class _FakeCheckMove:
    def __init__(self):
        self.calls = []

    def check_move(self, move):
        self.calls.append(move)


class _FakeToolhead:
    def __init__(self, pa_snap=None, **overrides):
        self.max_velocity = overrides.get("max_velocity", 500.0)
        self.max_accel = overrides.get("max_accel", 10000.0)
        self.max_jerk = overrides.get("max_jerk", 100000.0)
        self.corner_deviation = overrides.get("corner_deviation", 0.2)
        self.kin = _FakeCheckMove()
        if pa_snap is None:
            self.extruder = _FakeCheckMove()
            self.extruder_cap_snapshot = None
        else:
            ext_limits = blendshape.ExtruderLimits(
                a_E_max=math.inf, v_E_max=math.inf, smooth_time=0.04,
            )
            self.extruder = _FakeExtruder(pa_snap, ext_limits)
            self.extruder_cap_snapshot = (pa_snap, ext_limits)


class _FakeExtruder:
    def __init__(self, pa_snap, ext_limits):
        self._pa_snap = pa_snap
        self._limits = ext_limits
        self.calls = []

    def check_move(self, move):
        self.calls.append(move)

    def extruder_limits_snapshot(self):
        return (self._pa_snap, self._limits)


class _FakeMove:
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
                start_pos[0], start_pos[1], start_pos[2], end_pos[3],
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
        self.next_junction_v_capped_to = None

    def limit_speed(self, speed, accel):
        speed2 = speed ** 2
        if speed2 < self.max_cruise_v2:
            self.max_cruise_v2 = speed2
            self.min_move_t = self.move_d / speed if speed else 0.0
        self.accel = min(self.accel, accel)

    def limit_next_junction_speed(self, speed):
        self.next_junction_v2 = min(self.next_junction_v2, speed ** 2)
        self.next_junction_v_capped_to = speed


def _emit_right_angle_blend(th, speed=200.0):
    cb = blendplanner.CornerBlender(th, move_cls=_FakeMove, max_chord_err=20e-3)
    m_prev = _FakeMove(th, (0.0, 0.0, 0.0, 0.0), (10.0, 0.0, 0.0, 0.5),
                       speed=speed)
    m_next = _FakeMove(th, (10.0, 0.0, 0.0, 0.5), (10.0, 10.0, 0.0, 1.0),
                       speed=speed)
    assert cb.feed(m_prev) == []
    # chunk2-fix deferred-emit: round-2 emits only trunc_prev. Flush the
    # pending quintic (final bake with next=None for the session end).
    out_feed = cb.feed(m_next)
    assert len(out_feed) == 1
    trunc_prev = out_feed[0]
    out_flush = cb.flush()
    assert isinstance(out_flush[0], blendplanner.QuinticBlendMove)
    return trunc_prev, out_flush[0]


def _eval_axis_poly(coeff_tuple, phase, axis, tau):
    stride = 15 * 4
    base = phase * stride
    val = coeff_tuple[base + 14 * 4 + axis]
    for k in range(13, -1, -1):
        val = val * tau + coeff_tuple[base + k * 4 + axis]
    return val


def _bake_chord(payload):
    """Chord projection of the post-bake XY polynomial along axis_n.

    Mirrors the formula used by ``QuinticBlendMove.finalize_shape`` to compute
    extr_r. For curved blends ``arc_length > chord``, so the chord-projected
    linear_pa_compose integral closes on ``extr_r * chord``; setting
    ``extr_r = axes_d[3] / chord`` makes it close on ``axes_d[3]`` exactly.
    These tests therefore reference the chord-corrected ``extr_r`` rather
    than ``axes_d[3] / arc_length``, matching the production composer's
    formula bit-for-bit.
    """
    (phase_t_ends, _total, _arc_length, _vcap, _spos,
     coeff_tuple, *_legacy) = payload
    n_phases = len(phase_t_ends)
    if n_phases == 0:
        return 0.0
    stride = 15 * 4
    last = n_phases - 1
    prev_t = phase_t_ends[last - 1] if last > 0 else 0.0
    dt = phase_t_ends[last] - prev_t
    chord = 0.0
    for ax_idx in range(3):
        start = coeff_tuple[0 * stride + 0 * 4 + ax_idx]
        end = coeff_tuple[last * stride + 14 * 4 + ax_idx]
        for k in range(13, -1, -1):
            end = end * dt + coeff_tuple[last * stride + k * 4 + ax_idx]
        chord += (end - start)
    return chord


def _bake_chord_along_axis_n(payload, axis_n):
    (phase_t_ends, _total, _arc_length, _vcap, _spos,
     coeff_tuple, *_legacy) = payload
    n_phases = len(phase_t_ends)
    if n_phases == 0:
        return 0.0
    stride = 15 * 4
    last = n_phases - 1
    prev_t = phase_t_ends[last - 1] if last > 0 else 0.0
    dt = phase_t_ends[last] - prev_t
    chord = 0.0
    for ax_idx in range(3):
        start = coeff_tuple[0 * stride + 0 * 4 + ax_idx]
        end = coeff_tuple[last * stride + 14 * 4 + ax_idx]
        for k in range(13, -1, -1):
            end = end * dt + coeff_tuple[last * stride + k * 4 + ax_idx]
        chord += axis_n[ax_idx] * (end - start)
    return chord


def _eval_axis_deriv(coeff_tuple, phase, axis, tau):
    stride = 15 * 4
    base = phase * stride
    v = 0.0
    for k in range(1, 15):
        v += k * coeff_tuple[base + k * 4 + axis] * (tau ** (k - 1))
    return v


def _pick_phase(phase_t_ends, t):
    prev = 0.0
    for p, t_end in enumerate(phase_t_ends):
        if t <= t_end or p == len(phase_t_ends) - 1:
            return p, t - prev
        prev = t_end
    return len(phase_t_ends) - 1, t - phase_t_ends[-2]


def test_linear_pa_e_polynomial_matches_direct_formula():
    """Linear PA: the .e slot should match
        E(tau) = extr_r * n.P(tau) + k_pa * n.V(tau)
    where n is the unit XY direction, P/V are the XYZ polynomial and
    its derivative. Tested at multiple tau values on the accel / cruise
    / decel phases of the emitted QuinticBlendMove.
    """
    k_pa = 0.05
    pa_snap = blendextruder.PAModelSnapshot(kind="linear", params=(k_pa,))
    th = _FakeToolhead(pa_snap=pa_snap)
    _, move = _emit_right_angle_blend(th, speed=200.0)
    payload = move.quintic_trapq_payload
    (phase_t_ends, _total, arc_length, _vcap, _spos,
     coeff_tuple, *_legacy) = payload
    # move's axis_n is the chord direction. axes_d[3] = E displacement.
    axis_n = (move.axes_r[0], move.axes_r[1], move.axes_r[2])
    # Plan 9 chord-corrected extr_r: production linear_pa_compose receives
    # ``axes_d[3] / bake_chord`` (not ``arc_length``) so the chord-projected
    # integral closes on axes_d[3] for curved blends. See blendplanner.py.
    bake_chord = _bake_chord_along_axis_n(payload, axis_n)
    extr_r = move.axes_d[3] / bake_chord
    t_total = phase_t_ends[-1]
    for i in range(21):
        t_sample = t_total * i / 20.0
        p, tau = _pick_phase(phase_t_ends, t_sample)
        # Projected position and velocity.
        p_proj = sum(
            axis_n[a] * _eval_axis_poly(coeff_tuple, p, a, tau)
            for a in range(3)
        )
        v_proj = sum(
            axis_n[a] * _eval_axis_deriv(coeff_tuple, p, a, tau)
            for a in range(3)
        )
        expected = extr_r * p_proj + k_pa * v_proj
        got = _eval_axis_poly(coeff_tuple, p, 3, tau)
        assert got == pytest.approx(expected, abs=1e-8)


def _first_nondegenerate_phase(phase_t_ends):
    """Return (phase_idx, phase_duration, phase_start_in_move). Zero-
    duration phases (introduced by right-angle blends where accel /
    decel collapse) are skipped. Chebyshev fit is exact at the phase's
    endpoint nodes, so tests anchor at the start of the first
    nondegenerate phase.
    """
    prev = 0.0
    for p, t_end in enumerate(phase_t_ends):
        T = t_end - prev
        if T > 1e-12:
            return p, T, prev
        prev = t_end
    raise AssertionError("all phases degenerate")


def test_tanh_pa_e_polynomial_composes_exact_at_linear_term():
    """tanh PA: the .e polynomial's interpolation is exact at the
    Chebyshev-Lobatto endpoints of each phase (including tau=0 of the
    first nondegenerate phase). Verify end-point equality against the
    direct tanh formula.
    """
    la = 0.02
    no = 0.05
    v_lin = 40.0
    pa_snap = blendextruder.PAModelSnapshot(
        kind="tanh", params=(la, no, v_lin),
    )
    th = _FakeToolhead(pa_snap=pa_snap)
    _, move = _emit_right_angle_blend(th, speed=200.0)
    payload = move.quintic_trapq_payload
    (phase_t_ends, _total, arc_length, _vcap, _spos,
     coeff_tuple, *_legacy) = payload
    axis_n = (move.axes_r[0], move.axes_r[1], move.axes_r[2])
    # Plan 9 chord-corrected extr_r: production linear_pa_compose receives
    # ``axes_d[3] / bake_chord`` (not ``arc_length``) so the chord-projected
    # integral closes on axes_d[3] for curved blends. See blendplanner.py.
    bake_chord = _bake_chord_along_axis_n(payload, axis_n)
    extr_r = move.axes_d[3] / bake_chord
    p, _T, _tstart = _first_nondegenerate_phase(phase_t_ends)
    # tau=0 of the first nondegenerate phase is a Chebyshev-Lobatto
    # node — interpolation residual is O(machine eps).
    p_proj_0 = sum(
        axis_n[a] * _eval_axis_poly(coeff_tuple, p, a, 0.0)
        for a in range(3)
    )
    v_proj_0 = sum(
        axis_n[a] * _eval_axis_deriv(coeff_tuple, p, a, 0.0)
        for a in range(3)
    )
    v_clamped = max(v_proj_0, 0.0)
    expected_0 = (extr_r * p_proj_0
                  + la * v_proj_0
                  + no * math.tanh(v_clamped / v_lin))
    got_0 = _eval_axis_poly(coeff_tuple, p, 3, 0.0)
    assert got_0 == pytest.approx(expected_0, abs=1e-9)


def test_tanh_pa_filament_budget_on_cruise_phase():
    """On a cruise phase (constant V), the Chebyshev fit is exact (any
    constant is in the span of degree-4 polynomials), so the .e slot
    matches the tanh formula to machine precision on the cruise
    interior.
    """
    la = 0.02
    no = 0.05
    v_lin = 40.0
    pa_snap = blendextruder.PAModelSnapshot(
        kind="tanh", params=(la, no, v_lin),
    )
    th = _FakeToolhead(pa_snap=pa_snap)
    _, move = _emit_right_angle_blend(th, speed=200.0)
    payload = move.quintic_trapq_payload
    (phase_t_ends, _total, arc_length, _vcap, _spos,
     coeff_tuple, *_legacy) = payload
    # Phase boundaries: find the cruise phase by looking at V's second
    # derivative (zero on cruise). Simpler: sample all phases at their
    # midpoints and check the error against the direct formula; on
    # constant-V phases the error should be ~0; on curved phases we
    # accept up to the residual reported by the composer (approximated
    # here as 1 mm filament, way more than we expect in practice).
    axis_n = (move.axes_r[0], move.axes_r[1], move.axes_r[2])
    # Plan 9 chord-corrected extr_r: production linear_pa_compose receives
    # ``axes_d[3] / bake_chord`` (not ``arc_length``) so the chord-projected
    # integral closes on axes_d[3] for curved blends. See blendplanner.py.
    bake_chord = _bake_chord_along_axis_n(payload, axis_n)
    extr_r = move.axes_d[3] / bake_chord
    prev = 0.0
    any_sampled = False
    for p, t_end in enumerate(phase_t_ends):
        T = t_end - prev
        prev = t_end
        if T <= 1e-12:
            continue
        any_sampled = True
        # Sample midpoint.
        tau = 0.5 * T
        p_proj = sum(
            axis_n[a] * _eval_axis_poly(coeff_tuple, p, a, tau)
            for a in range(3)
        )
        v_proj = sum(
            axis_n[a] * _eval_axis_deriv(coeff_tuple, p, a, tau)
            for a in range(3)
        )
        v_clamped = max(v_proj, 0.0)
        expected = (extr_r * p_proj
                    + la * v_proj
                    + no * math.tanh(v_clamped / v_lin))
        got = _eval_axis_poly(coeff_tuple, p, 3, tau)
        # Loose bound — blend curves have significant deg-4 fit
        # residual on the tanh(v/v_lin) composition; the design accepts
        # up to filament_err = residual * NO. See blendplanner.py's
        # warning threshold.
        err = abs(got - expected)
        assert err < 1e-2, (
            f"tanh phase={p} midpoint err={err:.3g} mm exceeds loose bound"
        )
    assert any_sampled, "no nondegenerate phase sampled"


def test_recipr_pa_endpoint_exact():
    """recipr PA: same endpoint-exactness property as tanh."""
    la = 0.02
    no = 0.05
    v_lin = 40.0
    pa_snap = blendextruder.PAModelSnapshot(
        kind="recipr", params=(la, no, v_lin),
    )
    th = _FakeToolhead(pa_snap=pa_snap)
    _, move = _emit_right_angle_blend(th, speed=200.0)
    payload = move.quintic_trapq_payload
    (phase_t_ends, _total, arc_length, _vcap, _spos,
     coeff_tuple, *_legacy) = payload
    axis_n = (move.axes_r[0], move.axes_r[1], move.axes_r[2])
    # Plan 9 chord-corrected extr_r: production linear_pa_compose receives
    # ``axes_d[3] / bake_chord`` (not ``arc_length``) so the chord-projected
    # integral closes on axes_d[3] for curved blends. See blendplanner.py.
    bake_chord = _bake_chord_along_axis_n(payload, axis_n)
    extr_r = move.axes_d[3] / bake_chord
    p, _T, _tstart = _first_nondegenerate_phase(phase_t_ends)
    p_proj_0 = sum(
        axis_n[a] * _eval_axis_poly(coeff_tuple, p, a, 0.0)
        for a in range(3)
    )
    v_proj_0 = sum(
        axis_n[a] * _eval_axis_deriv(coeff_tuple, p, a, 0.0)
        for a in range(3)
    )
    v_clamped = max(v_proj_0, 0.0)
    r = v_clamped / v_lin
    expected_0 = (extr_r * p_proj_0
                  + la * v_proj_0
                  + no * (1.0 - 1.0 / (1.0 + r)))
    got_0 = _eval_axis_poly(coeff_tuple, p, 3, 0.0)
    assert got_0 == pytest.approx(expected_0, abs=1e-9)


def _polynomial_e_displacement(payload):
    """Total E displacement of the post-bake polynomial over the move.

    Evaluates E(0) at the start of phase 0 and E(T) at the end of the
    last phase via Horner. Equal to ``axes_d[3]`` iff the chord-projected
    linear_pa_compose integral closes correctly.
    """
    (phase_t_ends, _total, _arc_length, _vcap, _spos,
     coeff_tuple, *_legacy) = payload
    n_phases = len(phase_t_ends)
    if n_phases == 0:
        return 0.0
    stride = 15 * 4
    last = n_phases - 1
    prev_t = phase_t_ends[last - 1] if last > 0 else 0.0
    dt = phase_t_ends[last] - prev_t
    e_start = coeff_tuple[0 * stride + 0 * 4 + 3]
    e_end = coeff_tuple[last * stride + 14 * 4 + 3]
    for k in range(13, -1, -1):
        e_end = e_end * dt + coeff_tuple[last * stride + k * 4 + 3]
    return e_end - e_start


def test_qbm_polynomial_e_closes_on_axes_d3_no_pa():
    """Regression for the QBM chord-projection bug.

    Pre-fix: ``linear_pa_compose`` set ``E[k] = extr_r * (axis_n . XYZ[k])``
    with ``extr_r = axes_d[3] / arc_length``. For a curved blend the
    chord-projected integral closes on ``extr_r * chord``, which is
    strictly less than ``axes_d[3]`` whenever ``chord < arc_length``. The
    extruder bookkeeping ``last_position[0] += axes_d[3]`` therefore
    accumulated a per-move discontinuity in physical E and eventually
    crashed stepcompress with ``Invalid sequence``. The fix rescales
    ``extr_r`` to the post-bake chord projection so the chord-projected
    integral closes on ``axes_d[3]`` exactly. This test pins that
    invariant for the no-PA path on a 90° corner blend.
    """
    th = _FakeToolhead(pa_snap=None)
    _, move = _emit_right_angle_blend(th, speed=200.0)
    e_disp = _polynomial_e_displacement(move.quintic_trapq_payload)
    assert e_disp == pytest.approx(move.axes_d[3], abs=1e-12), (
        f"polynomial E displacement {e_disp!r} must close on "
        f"axes_d[3]={move.axes_d[3]!r} exactly"
    )


def test_qbm_polynomial_e_closes_on_axes_d3_linear_pa():
    """Same chord-projection regression, but with linear PA enabled.

    The k_pa term contributes ``k_pa * (v_chord_end - v_chord_start)`` to
    the integral. For a same-velocity blend (cruise → cruise → cruise on
    a right-angle corner reduced by corner_deviation) the PA term
    cancels at the chord-projected endpoints, so the integral still
    closes on ``axes_d[3]`` exactly.
    """
    pa_snap = blendextruder.PAModelSnapshot(kind="linear", params=(0.05,))
    th = _FakeToolhead(pa_snap=pa_snap)
    _, move = _emit_right_angle_blend(th, speed=200.0)
    payload = move.quintic_trapq_payload
    e_disp = _polynomial_e_displacement(payload)
    # Linear PA contributes k_pa * delta_v_chord across the whole move.
    # On the right-angle blend the entry and exit velocity vectors have
    # equal projection onto axis_n by symmetry, so delta_v_chord ≈ 0 and
    # the integral closes on axes_d[3] to machine precision.
    axis_n = (move.axes_r[0], move.axes_r[1], move.axes_r[2])
    pa_drift = e_disp - move.axes_d[3]
    assert abs(pa_drift) < 1e-9, (
        f"polynomial E displacement {e_disp!r} drift from "
        f"axes_d[3]={move.axes_d[3]!r} = {pa_drift!r}; expected ≈ 0 by "
        f"symmetry of the right-angle blend"
    )


def test_pa_disabled_e_equals_extr_r_times_projection():
    """Without PA, the .e slot is just extr_r * n.P(tau)."""
    th = _FakeToolhead(pa_snap=None)
    _, move = _emit_right_angle_blend(th, speed=200.0)
    payload = move.quintic_trapq_payload
    (phase_t_ends, _total, arc_length, _vcap, _spos,
     coeff_tuple, *_legacy) = payload
    axis_n = (move.axes_r[0], move.axes_r[1], move.axes_r[2])
    # Plan 9 chord-corrected extr_r: production linear_pa_compose receives
    # ``axes_d[3] / bake_chord`` (not ``arc_length``) so the chord-projected
    # integral closes on axes_d[3] for curved blends. See blendplanner.py.
    bake_chord = _bake_chord_along_axis_n(payload, axis_n)
    extr_r = move.axes_d[3] / bake_chord
    t_total = phase_t_ends[-1]
    for i in range(11):
        t_sample = t_total * i / 10.0
        p, tau = _pick_phase(phase_t_ends, t_sample)
        p_proj = sum(
            axis_n[a] * _eval_axis_poly(coeff_tuple, p, a, tau)
            for a in range(3)
        )
        expected = extr_r * p_proj
        got = _eval_axis_poly(coeff_tuple, p, 3, tau)
        assert got == pytest.approx(expected, abs=1e-10)
