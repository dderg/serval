# test/test_plan5_integration.py
#
# Plan 8 Chunk 2 Task 14 — integration tests for the baked-in shaper path.
#
# Chains the corner-blending planner through to the baked quintic polynomial
# and verifies:
#   - One QuinticBlendMove per corner (D7 emit contract).
#   - The baked payload (Chunk 2 variable-length phases) round-trips through
#     trapq_append_quintic intact.
#   - The baked polynomial matches a Python-side reference convolution of
#     the pre-bake (unshaped) trajectory with the bs-family kernel, to
#     within the 100 µm passband spec.
#   - Phase boundaries of the baked polynomial are C^0 continuous.
#
# The post-hoc shaper cascade + fused feedforward inverse this file used
# to exercise retired in Plan 8 Chunk 2 Task 13. The reference path here
# is the explicit Python convolution of the unshaped polynomial against
# the bs kernel — see _reference_convolution() below.

from __future__ import annotations

import math

import numpy as np
import pytest

from klippy import blendplanner, blendquintic, blendshape, chelper, topp
from klippy.chelper.linear_quintic import append_trapezoid_as_quintic
from klippy.extras import shaper_defs


# ---------------------------------------------------------------------------
# Inline fixtures — mirror test_blendplanner.py's Move/Toolhead stubs so this
# file is importable standalone.
# ---------------------------------------------------------------------------


class _FakeCheckMove:
    def __init__(self):
        self.calls = []

    def check_move(self, move):
        self.calls.append(move)


class _FakeToolhead:
    def __init__(self, **overrides):
        self.max_velocity = overrides.get("max_velocity", 500.0)
        self.max_accel = overrides.get("max_accel", 10000.0)
        self.max_jerk = overrides.get("max_jerk", 100000.0)
        self.corner_deviation = overrides.get("corner_deviation", 50e-3)
        self.kin = _FakeCheckMove()
        self.extruder = _FakeCheckMove()


class _FakeMove:
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


class _FakeAxisIS:
    """Mirrors klippy.extras.input_shaper.AxisInputSmoother's blendmath-
    visible surface for bs-family shapers. Plan 8 Chunk 2 reads
    shaper_type / shaper_freq from params directly."""
    def __init__(self, axis, stype, freq, damping=0.0):
        self._axis = axis

        class _P:
            pass

        self.params = _P()
        # bs-family: smoother_type / smoother_freq convention — but we
        # populate the FIR-style names too so blendmath.extract_shapers
        # reads the right fields on either branch.
        self.params.shaper_type = stype
        self.params.shaper_freq = freq
        self.params.smoother_type = stype
        self.params.smoother_freq = freq
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


class _FakePrinter:
    def __init__(self, is_obj):
        self._is = is_obj

    def lookup_object(self, name, default=None):
        if name == "input_shaper":
            return self._is
        return default


# ---------------------------------------------------------------------------
# Baked-polynomial evaluator + unshaped-trajectory reference.
# ---------------------------------------------------------------------------


def _unpack_payload(payload):
    """Return (phase_t_ends, total_t_baked, arc_length, v_cap_min,
    start_pos_xyz, coeff_tuple, legacy_triple).

    Plan 8 Chunk 2 payload layout (9 fields):
        (phase_t_ends_tuple, total_t_baked,
         arc_length, v_cap_min, start_pos_xyz, coeff_tuple,
         legacy_t_accel_end, legacy_t_decel_start, legacy_total_t)
    """
    (phase_t_ends_tuple, total_t_baked,
     arc_length, v_cap_min, start_pos_xyz, coeff_tuple,
     legacy_t_accel_end, legacy_t_decel_start, legacy_total_t) = payload
    return (
        tuple(phase_t_ends_tuple), total_t_baked,
        arc_length, v_cap_min, start_pos_xyz, coeff_tuple,
        (legacy_t_accel_end, legacy_t_decel_start, legacy_total_t),
    )


def _baked_position_fn(payload):
    """Return a callable t → (x, y, z) evaluating the baked per-phase
    polynomials as trapq.c::move_get_coord does (Horner in phase-local t).
    """
    (phase_t_ends, total_t_baked, _, _, start_pos_xyz, coeff_tuple, _
     ) = _unpack_payload(payload)
    n_phases = len(phase_t_ends)
    # Unpack coeff_tuple into phases[p][axis][k] layout.
    # Plan 8 Chunk 3: 4-axis stride (x, y, z, e). Only XY axes (0..2) are
    # extracted here — the .e slot is exercised by linear_pa_compose tests.
    phases = []
    buf = list(coeff_tuple)
    for p in range(n_phases):
        base = p * 15 * 4
        axes_coeffs = [[0.0] * 15 for _ in range(3)]
        for k in range(15):
            for ax in range(3):
                axes_coeffs[ax][k] = buf[base + k * 4 + ax]
        phases.append(axes_coeffs)

    def eval_at(t):
        # Locate the containing phase. Evaluate the phase polynomial in
        # phase-local time even at t=0 — the composer's zero-pad convention
        # means c[0] of phase 0 already carries the pad-integrated value,
        # which is NOT equal to struct move's start_pos.
        phase_start = 0.0
        chosen_phase = None
        delta_t = 0.0
        for p in range(n_phases):
            phase_end = phase_t_ends[p]
            if t <= phase_end or p == n_phases - 1:
                chosen_phase = phases[p]
                delta_t = t - phase_start
                break
            phase_start = phase_end
        if chosen_phase is None:
            chosen_phase = phases[-1]
            delta_t = phase_t_ends[-1] - (
                phase_t_ends[-2] if n_phases > 1 else 0.0
            )
        out = [0.0, 0.0, 0.0]
        for ax in range(3):
            coeffs = chosen_phase[ax]
            v = coeffs[14]
            for k in range(13, -1, -1):
                v = v * delta_t + coeffs[k]
            out[ax] = v
        return tuple(out)

    return eval_at, total_t_baked


def _unshaped_position_fn(
        shape, v_in, v_out, cruise_v, a_max, s_accel_end, s_decel_start):
    """Build the UNSHAPED 3-phase polynomial directly from the QuinticShape
    (same call the planner makes before passing through bake_shaper_polynomial)
    and return a t → (x, y, z) evaluator plus its total_t.

    This is the "reference" trajectory that the bake path must approximate
    via kernel convolution on passband frequencies.
    """
    (accel_polys, cruise_polys, decel_polys, t_accel_end, t_decel_start,
     total_t, _arc_length) = shape.compose_phase_polynomials(
        v_in=v_in, v_out=v_out, cruise_v=cruise_v, a_max=a_max,
        s_accel_end=s_accel_end, s_decel_start=s_decel_start,
    )

    def eval_at(t):
        if t <= 0.0:
            # Evaluate accel phase at delta_t = 0 → constant-term (start_pos).
            delta_t = 0.0
            phase = accel_polys
        elif t <= t_accel_end:
            delta_t = t
            phase = accel_polys
        elif t <= t_decel_start:
            delta_t = t - t_accel_end
            phase = cruise_polys
        elif t <= total_t:
            delta_t = t - t_decel_start
            phase = decel_polys
        else:
            # Stationary hold at end pos — evaluate decel at its endpoint.
            delta_t = total_t - t_decel_start
            phase = decel_polys
        out = [0.0, 0.0, 0.0]
        for ax in range(3):
            coeffs = phase[ax]
            v = coeffs[14] if len(coeffs) > 14 else 0.0
            for k in range(min(13, len(coeffs) - 1), -1, -1):
                v = v * delta_t + coeffs[k]
            out[ax] = v
        return tuple(out)

    return eval_at, total_t


def _bs_kernel_eval(bs_variant, freq, damping, tau_array):
    """Return w(tau) for the bs-family kernel at the given variant/freq.
    Normalized (integrates to 1 over its support [-t_sm/2, t_sm/2])."""
    for ism in shaper_defs.INPUT_SMOOTHERS:
        if ism.name == bs_variant:
            break
    else:
        raise ValueError("unknown bs variant: %s" % bs_variant)
    C_pieces, t_sm = ism.init_func(freq, damping, True)
    vals = np.asarray(shaper_defs.bspline_eval(C_pieces, tau_array, t_sm))
    return vals, t_sm


def _reference_convolution(
        traj_fn, bs_variant, freq, t_eval, pad_start, pad_end,
        total_t_unshaped, n_quad=513):
    """Compute (k ⊛ x)(t_eval) using Simpson's rule over [-t_sm/2, t_sm/2].

    Matches bs_compose.c convention: outside the unshaped [0, total_t_unshaped]
    window the trajectory is ZERO-PADDED (see bs_compose.c line 317: "zero-
    pad outside the move"). This is the per-move contribution convention —
    the full-stream shaped trajectory sums contributions across adjacent
    moves, but for a single-move unit test this is the exact invariant the
    composer computes.
    """
    _, t_sm = _bs_kernel_eval(bs_variant, freq, 0.0, np.array([0.0]))
    hst = 0.5 * t_sm
    nodes = n_quad if n_quad % 2 == 1 else n_quad + 1
    step = (2.0 * hst) / (nodes - 1)
    taus = np.linspace(-hst, +hst, nodes)
    k_vals, _ = _bs_kernel_eval(bs_variant, freq, 0.0, taus)

    def padded(t, axis):
        # bs_compose.c zero-pad convention: outside [0, move_t] the
        # single-move contribution is 0 (pad_start / pad_end unused here).
        if t <= 0.0 or t >= total_t_unshaped:
            return 0.0
        return traj_fn(t)[axis]

    simp = np.ones(nodes)
    simp[1:-1:2] = 4.0
    simp[2:-1:2] = 2.0
    out = np.zeros(3)
    for ax in range(3):
        integrand = np.array([
            k_vals[i] * padded(t_eval - taus[i], ax) for i in range(nodes)
        ])
        out[ax] = (step / 3.0) * np.sum(simp * integrand)
    return tuple(out)


# ---------------------------------------------------------------------------
# Shared fixtures: toolhead + blend emission.
# ---------------------------------------------------------------------------


def _make_toolhead_with_bs_shaper(bs_variant, freq, max_accel, corner_deviation):
    th = _FakeToolhead(
        max_accel=max_accel,
        max_accel_to_decel=max_accel,
        corner_deviation=corner_deviation,
        max_velocity=500.0,
    )
    th.printer = _FakePrinter(_FakeIS([
        _FakeAxisIS("x", bs_variant, freq),
        _FakeAxisIS("y", bs_variant, freq),
    ]))
    return th


def _emit_right_angle_blend(th, speed=200.0):
    """Drive two 10 mm moves meeting at a 90° corner through CornerBlender.

    Returns (trunc_prev, quintic_move, trunc_next_head). The chunk2-fix
    deferred-emit wiring keeps the quintic buffered until the next move
    arrives, so we drive `flush()` to drain it with next=None (matches
    the session-end / print-stops-here path).
    """
    cb = blendplanner.CornerBlender(th, move_cls=_FakeMove, max_chord_err=20e-3)
    m_prev = _FakeMove(th, (0.0, 0.0, 0.0, 0.0), (10.0, 0.0, 0.0, 0.5),
                       speed=speed)
    m_next = _FakeMove(th, (10.0, 0.0, 0.0, 0.5), (10.0, 10.0, 0.0, 1.0),
                       speed=speed)
    assert cb.feed(m_prev) == []
    out_feed = cb.feed(m_next)
    # Round 2: [trunc_prev] (quintic is pending-deferred).
    trunc_prev = out_feed[0]
    trunc_next_head = cb._prev
    # Flush drains the pending quintic with next=None and then _prev.
    out_flush = cb.flush()
    # First element is the finalized quintic; second is trunc_next_head.
    assert isinstance(out_flush[0], blendplanner.QuinticBlendMove)
    quintic_move = out_flush[0]
    return trunc_prev, quintic_move, trunc_next_head


def _push_quintic_to_trapq(ffi_main, ffi_lib, tq, print_time, payload):
    """Mirror ToolHead._process_moves's quintic-emit block (Plan 8 Chunk 2
    variable-length phases)."""
    (phase_t_ends, total_t_baked, arc_length, v_cap_min, start_pos_xyz,
     coeff_tuple, _legacy) = _unpack_payload(payload)
    n_phases = len(phase_t_ends)
    coeff_buf = ffi_main.new(
        f"double[{n_phases * 15 * 4}]", list(coeff_tuple)
    )
    phase_t_ends_buf = ffi_main.new(
        f"double[{n_phases}]", list(phase_t_ends),
    )
    ffi_lib.trapq_append_quintic(
        tq, print_time,
        n_phases, phase_t_ends_buf,
        total_t_baked, arc_length, v_cap_min,
        0,  # shape_disabled=False: planner blend is always shaped
        start_pos_xyz[0], start_pos_xyz[1], start_pos_xyz[2],
        coeff_buf,
    )
    return total_t_baked


# ---------------------------------------------------------------------------
# Integration tests.
# ---------------------------------------------------------------------------


class TestPlan5CascadeIntegration:
    """End-to-end pipeline: plan corner → blend → baked polynomial."""

    FREQ = 40.0
    CORNER_DEVIATION = 0.2
    MAX_ACCEL = 10000.0
    SPEED = 200.0

    # --- Structural: single QuinticBlendMove per corner ----------------------

    def test_single_quintic_blend_per_corner(self):
        """D7 emit contract: one QuinticBlendMove per blend, not a polyline.

        The Plan 8 Chunk 2 payload has 9 fields (variable phase count +
        legacy trapezoid timings). Verify structure + finite coeffs.
        """
        th = _make_toolhead_with_bs_shaper(
            "bs3", self.FREQ, self.MAX_ACCEL, self.CORNER_DEVIATION,
        )
        _, quintic_move, _ = _emit_right_angle_blend(th, speed=self.SPEED)
        payload = quintic_move.quintic_trapq_payload
        assert len(payload) == 9, "payload must be 9-tuple (Chunk 2 layout)"
        (phase_t_ends, total_t_baked, arc_length, v_cap_min, start_pos_xyz,
         coeff_tuple, legacy_triple) = _unpack_payload(payload)
        assert total_t_baked > 0.0
        assert arc_length > 0.0
        assert v_cap_min > 0.0
        n_phases = len(phase_t_ends)
        assert 1 <= n_phases <= 32, "n_phases out of MOVE_MAX_PIECES range"
        # phase_t_ends is monotone non-decreasing, last equals total_t_baked.
        assert all(phase_t_ends[i] <= phase_t_ends[i + 1]
                   for i in range(n_phases - 1))
        assert phase_t_ends[-1] == pytest.approx(total_t_baked, rel=1e-12)
        assert len(coeff_tuple) == n_phases * 15 * 4
        for c in coeff_tuple:
            assert math.isfinite(c), "non-finite coefficient in payload"
        # Legacy trapezoid-in-s timings sanity-check.
        (t_ae, t_ds, legacy_total) = legacy_triple
        assert 0.0 <= t_ae <= t_ds <= legacy_total

    # --- Integration: blend → trapq_append_quintic → extract -----------------

    def test_blend_routes_through_trapq_append_quintic(self):
        """CornerBlender emits a QuinticBlendMove → trapq_append_quintic
        stores it. Verified via trapq_extract_old after trapq_finalize_moves.
        """
        th = _make_toolhead_with_bs_shaper(
            "bs3", self.FREQ, self.MAX_ACCEL, self.CORNER_DEVIATION,
        )
        _, quintic_move, _ = _emit_right_angle_blend(th, speed=self.SPEED)
        ffi_main, ffi_lib = chelper.get_ffi()
        tq = ffi_main.gc(ffi_lib.trapq_alloc(), ffi_lib.trapq_free)
        t_print = 1.0
        total_t = _push_quintic_to_trapq(
            ffi_main, ffi_lib, tq, t_print,
            quintic_move.quintic_trapq_payload,
        )
        ffi_lib.trapq_finalize_moves(tq, t_print + total_t + 1.0, 0.0)
        pm = ffi_main.new("struct pull_move[4]")
        n = ffi_lib.trapq_extract_old(
            tq, pm, 4, t_print - 0.001, t_print + total_t + 1.0,
        )
        assert n >= 1
        found = None
        for i in range(n):
            if pm[i].start_v > 0.0 and pm[i].move_t > 0.0:
                found = pm[i]
                break
        assert found is not None, "no motion-carrying quintic entry"
        assert found.move_t == pytest.approx(total_t, rel=1e-9)
        assert found.print_time == pytest.approx(t_print, rel=1e-9)

    # --- Baked polynomial matches Python reference convolution ---------------

    @pytest.mark.parametrize("bs_variant,max_err_um", [
        ("bs1", 100.0),
        ("bs3", 100.0),
        ("bs5", 100.0),
    ])
    def test_shaper_cascade_matches_planned_within_passband(
            self, bs_variant, max_err_um):
        """The baked-in polynomial (blendplanner.bake_shaper_polynomial)
        must match a Python-side reference convolution of the UNSHAPED
        quintic with the same bs kernel, within the 100 µm passband spec.

        The reference convolution uses Simpson's rule over [-t_sm/2,
        t_sm/2] with pad-with-endpoint semantics outside the unshaped
        duration. Since the bs kernel integrates to 1, a constant-pad
        convolves to the same constant — matches the natural interpretation
        of the baked polynomial outside its own support.
        """
        th = _make_toolhead_with_bs_shaper(
            bs_variant, self.FREQ, self.MAX_ACCEL, self.CORNER_DEVIATION,
        )
        _, quintic_move, _ = _emit_right_angle_blend(th, speed=self.SPEED)
        payload = quintic_move.quintic_trapq_payload

        # Baked polynomial evaluator.
        baked_fn, total_t_baked = _baked_position_fn(payload)

        # Unshaped reference: re-run compose_phase_polynomials with the
        # same TOPP-derived timings the planner used. We reconstruct them
        # from the QuinticBlendMove attributes rather than re-doing TOPP.
        shape = quintic_move.shape
        # QuinticBlendMove stores start/end v2 and cruise_v on itself via
        # set_junction; the composer needs v_in/v_out/cruise_v + s_accel/s_decel.
        # The legacy trapezoid-in-s triple in the payload gives the timings
        # directly, and shape carries arc_length + a_max (via shape._limits).
        legacy_t_accel_end = payload[-3]
        legacy_t_decel_start = payload[-2]
        legacy_total_t = payload[-1]
        # Recover cruise_v / v_in / v_out from the legacy triple + arc_length.
        # This is exactly what set_junction stores on QuinticBlendMove when
        # called by the outer lookahead, but without depending on that call
        # having happened we peel them off the stored payload timing.
        arc_length = payload[2]
        cruise_t = max(0.0, legacy_t_decel_start - legacy_t_accel_end)
        # Reuse the planner's baked-v_cap_min as a floor: cruise_v is at
        # most v_cap_min, or v_in/v_out at the boundary.
        v_cap_min = payload[3]
        # Inverted trapezoid-in-s: find cruise_v from timings + arc_length.
        a_max = shape._limits.a_max
        # s_accel = 0.5 * (v_in + cruise_v) * t_accel (affine in s);
        # short of redoing TOPP, use the QuinticShape.v_cap_fn(0) = v_in
        # and v_cap_fn(arc_length) = v_out consistent with the planner.
        # For right-angle 90° corners under a centripetal-dominated shape
        # v_in == v_out == v_cap_min (symmetric blend).
        v_in = v_out = min(v_cap_min, math.sqrt(quintic_move.max_start_v2))
        # Solve cruise_v from (v_in + cruise_v) * t_accel/2 + cruise_v * cruise_t
        #   + (cruise_v + v_out) * t_decel/2 = arc_length
        # With symmetric ends v_in=v_out and t_accel=t_decel:
        t_accel = legacy_t_accel_end
        t_decel = legacy_total_t - legacy_t_decel_start
        if t_accel == pytest.approx(t_decel, abs=1e-12):
            # Symmetric. cruise_v solves:
            #   v_in * t_accel + cruise_v * (t_accel + cruise_t) = arc_length
            denom = t_accel + cruise_t
            if denom > 0:
                cruise_v = (arc_length - v_in * t_accel) / denom
            else:
                cruise_v = v_in
        else:
            cruise_v = arc_length / legacy_total_t
        s_accel_end = 0.5 * (v_in + cruise_v) * t_accel
        s_decel_start = arc_length - 0.5 * (cruise_v + v_out) * t_decel

        unshaped_fn, total_t_unshaped = _unshaped_position_fn(
            shape, v_in=v_in, v_out=v_out, cruise_v=cruise_v, a_max=a_max,
            s_accel_end=s_accel_end, s_decel_start=s_decel_start,
        )

        # Reference convolution at t_eval samples inside [0, total_t_unshaped].
        # Extend t_eval range up to total_t_baked: the baked polynomial is
        # defined there and should carry the shaper's kernel tail.
        pad_start = unshaped_fn(0.0)
        pad_end = unshaped_fn(total_t_unshaped)

        n_samples = 25
        max_err = 0.0
        max_err_t = 0.0
        max_err_axis = ""
        # Sample inside the baked duration. For FIR the baked polynomial
        # extends beyond total_t_unshaped — convolve with padding there too.
        eval_end = total_t_baked
        for i in range(n_samples + 1):
            t = eval_end * i / n_samples
            ref = _reference_convolution(
                unshaped_fn, bs_variant, self.FREQ, t,
                pad_start, pad_end, total_t_unshaped,
            )
            baked = baked_fn(t)
            for ax_char, ax in (("x", 0), ("y", 1)):
                err = abs(baked[ax] - ref[ax])
                if err > max_err:
                    max_err = err
                    max_err_t = t
                    max_err_axis = ax_char
        max_err_actual_um = max_err * 1000.0
        assert max_err_actual_um < max_err_um, (
            "%s bake-vs-convolve identity: max err %.2f um (axis=%s) "
            "> %.1f um at t=%.4fs (total_t_baked=%.4fs, "
            "total_t_unshaped=%.4fs)"
            % (bs_variant, max_err_actual_um, max_err_axis, max_err_um,
               max_err_t, total_t_baked, total_t_unshaped)
        )

    # --- Phase-boundary continuity (C^0) -------------------------------------

    def test_phase_boundaries_C0_continuous(self):
        """No step discontinuity in the baked polynomial at any phase
        boundary. Plan 8 Chunk 2 produces variable-phase payloads (the
        bs composer can introduce up to 2N+1 sub-phases for an N-phase
        input). Each internal breakpoint must be C^0 continuous.
        """
        th = _make_toolhead_with_bs_shaper(
            "bs3", self.FREQ, self.MAX_ACCEL, self.CORNER_DEVIATION,
        )
        _, quintic_move, _ = _emit_right_angle_blend(th, speed=self.SPEED)
        payload = quintic_move.quintic_trapq_payload
        baked_fn, total_t_baked = _baked_position_fn(payload)
        (phase_t_ends, _, _, _, _, _, _) = _unpack_payload(payload)

        eps = 1e-9
        for p, t_boundary in enumerate(phase_t_ends[:-1]):
            if t_boundary <= 0.0 or t_boundary >= total_t_baked:
                continue
            p_lo = baked_fn(t_boundary - eps)
            p_hi = baked_fn(t_boundary + eps)
            for ax in range(3):
                assert abs(p_hi[ax] - p_lo[ax]) < 1e-6, (
                    "discontinuity at phase %d boundary (t=%.6f) axis %d: "
                    "lo=%.12f hi=%.12f" % (p, t_boundary, ax, p_lo[ax],
                                            p_hi[ax])
                )

    # --- Linear regression gate (unchanged by Plan 8 Chunk 2) ----------------

    def test_linear_move_through_pipeline_is_FP_precise(self):
        """A pure straight line (no blend) routed through trapq_append still
        reconstructs the closed-form linear trajectory to within FP precision.
        """
        th = _make_toolhead_with_bs_shaper(
            "bs3", self.FREQ, self.MAX_ACCEL, self.CORNER_DEVIATION,
        )
        cb = blendplanner.CornerBlender(th, move_cls=_FakeMove)
        m = _FakeMove(th, (0, 0, 0, 0), (10, 0, 0, 0.5), speed=100.0)
        _ = cb.feed(m)
        emitted = cb.flush()
        assert emitted == [m]
        assert not hasattr(m, "quintic_trapq_payload") or \
            getattr(m, "quintic_trapq_payload", None) is None
        ffi_main, ffi_lib = chelper.get_ffi()
        tq = ffi_main.gc(ffi_lib.trapq_alloc(), ffi_lib.trapq_free)
        accel_t = 0.05
        cruise_t = 0.05
        decel_t = 0.05
        start_v = 0.0
        cruise_v = 100.0
        accel = cruise_v / accel_t
        total_t = accel_t + cruise_t + decel_t
        append_trapezoid_as_quintic(
            tq, 1.0,
            accel_t, cruise_t, decel_t,
            0.0, 0.0, 0.0,
            1.0, 0.0, 0.0,
            start_v, cruise_v, accel,
        )
        ffi_lib.trapq_finalize_moves(tq, 1.0 + total_t + 1.0, 0.0)
        pm = ffi_main.new("struct pull_move[8]")
        n = ffi_lib.trapq_extract_old(
            tq, pm, 8, 0.5, 2.0 + total_t,
        )
        assert n >= 1
        found = None
        for i in range(n):
            if pm[i].start_v > 0.0 and pm[i].move_t > 0.0:
                found = pm[i]
                break
        assert found is not None
        assert found.move_t == pytest.approx(total_t, rel=1e-12)
        accel_d = start_v * accel_t + 0.5 * accel * accel_t * accel_t
        cruise_d = cruise_v * cruise_t
        decel_d = cruise_v * decel_t - 0.5 * accel * decel_t * decel_t
        chord = accel_d + cruise_d + decel_d
        assert found.start_v == pytest.approx(chord / total_t, rel=1e-12)
        assert found.x_r == pytest.approx(1.0, abs=1e-12)
        assert found.y_r == pytest.approx(0.0, abs=1e-12)
        assert found.accel == pytest.approx(0.0, abs=0.0)


# ---------------------------------------------------------------------------
# Phase A5 T2 C1 — Move.calc_junction must tolerate a QuinticBlendMove as
# prev_move, including reading prev_move.j_max for the jerk-aware forward
# reachability cap. Regression for the omitted attribute that the reviewer
# found after T2 landed.
# ---------------------------------------------------------------------------


def test_quintic_blend_move_carries_j_max_parity_with_move():
    """A real QuinticBlendMove emitted by CornerBlender must carry
    ``j_max`` (= toolhead.max_jerk). ``Move.calc_junction`` reads this
    when the prev_move is a QBM; without parity the flush crashes with
    AttributeError.
    """
    from klippy.toolhead import Move as _RealMove

    th = _make_toolhead_with_bs_shaper(
        "bs3", 40.0, 10000.0, 0.2,
    )
    _, quintic_move, _ = _emit_right_angle_blend(th, speed=200.0)
    # Parity check: the attribute exists and matches toolhead.max_jerk.
    assert hasattr(quintic_move, "j_max"), (
        "QuinticBlendMove must carry j_max (A5 attribute-contract parity "
        "with Move — Move.calc_junction reads prev_move.j_max)"
    )
    assert quintic_move.j_max == th.max_jerk

    # End-to-end contract: a plain Move constructed downstream of the QBM
    # must survive calc_junction(qbm) without AttributeError AND its
    # max_start_v2 must be bounded by the jerk-aware forward-reach cap
    # computed from the QBM's (max_start_v2, accel, j_max, move_d).
    import math as _math
    from klippy import jerk_math as _jm

    class _RealisticToolhead:
        # Minimal surface Move + calc_junction need. Uses the QBM's
        # toolhead.max_jerk so the plain Move's own j_max matches.
        max_velocity = th.max_velocity
        max_accel = th.max_accel
        max_jerk = th.max_jerk

        class _Ext:
            def calc_junction(self, *_a):
                return 1e18
        extruder = _Ext()

    rth = _RealisticToolhead()
    # Place the plain Move collinear with the QBM's exit direction so the
    # centripetal cap is loose (cos_theta ≈ 1) and the forward-reach cap
    # is the binding term.
    qbm_axes_r = quintic_move.axes_r
    exit_dir = (qbm_axes_r[0], qbm_axes_r[1], qbm_axes_r[2])
    exit_pos = quintic_move.end_pos
    end_pos = (
        exit_pos[0] + 10.0 * exit_dir[0],
        exit_pos[1] + 10.0 * exit_dir[1],
        exit_pos[2] + 10.0 * exit_dir[2],
        exit_pos[3],
    )
    # The real Move requires (x,y,z,e) tuples; ensure the chord is > 0.
    m = _RealMove(rth, exit_pos, end_pos, speed=200.0)
    # Must not raise: this is the regression.
    m.calc_junction(quintic_move)
    # Upper bound: the jerk-aware forward-reach cap is computed from the
    # QBM attributes. max_start_v2 must be at most that value.
    prev_start_v = (_math.sqrt(quintic_move.max_start_v2)
                    if quintic_move.max_start_v2 > 0.0 else 0.0)
    forward_reach = _jm.reachable_v_end(
        v_start=prev_start_v,
        a_max=quintic_move.accel, j_max=quintic_move.j_max,
        L=quintic_move.move_d,
    )
    assert m.max_start_v2 <= forward_reach * forward_reach + 1e-9, (
        "Move.max_start_v2 must respect the QBM-sourced forward-reach cap"
    )
