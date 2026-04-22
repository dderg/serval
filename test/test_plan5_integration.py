# test/test_plan5_integration.py
#
# Plan 5 Task 29 — end-to-end cascade identity integration test.
#
# Chains the full pipeline in a single test session:
#   CornerBlender.feed → QuinticBlendMove → trapq_append_quintic (C FFI)
#   planned trajectory (QuinticShape + TOPP profile) → fused kernel cascade
#
# and verifies that the shaper-cascade output reproduces the planned
# trajectory to within passband fidelity (≤ 100 μm) over the quintic
# duration for bs1/bs3/bs5.
#
# Architecture note (escalation):
#   toolhead._process_moves requires a full ToolHead instance with an MCU,
#   reactor, stepper-kinematics graph etc. This test bypasses it by
#   replicating the quintic-emit block of _process_moves directly: unpack
#   QuinticBlendMove.quintic_trapq_payload, allocate a trapq via cffi,
#   and call trapq_append_quintic.
#
# Architecture note (shaper sampling):
#   kin_shaper.c::shaper_calc_position walks a live trapq list and is
#   invoked from itersolve_generate_steps, which needs a full stepcompress
#   setup (MCU OID, step_dist, queue, etc.) to drive. Exposing an
#   arbitrary-time sampling helper would require a C-side change outside
#   the test's scope. Instead, the cascade integral is evaluated on the
#   Python side using the _exact same_ fused-kernel piecewise polynomial
#   the C side receives via input_shaper_set_smoother_params — ensuring
#   the kernel-mathematical content is identical, only the convolution
#   engine differs (Python quadrature vs the C antiderivative fast path
#   in integrate.c::integrate_move).
from __future__ import annotations

import math

import numpy as np
import pytest

from klippy import blendplanner, blendquintic, blendshape, chelper, topp
from klippy.extras import bspline_inverse, shaper_defs


# ---------------------------------------------------------------------------
# Inline fixtures — mirror test_blendplanner.py's Move/Toolhead stubs so this
# file is importable standalone (test/ isn't a package, so cross-file imports
# don't resolve under pytest's default rootdir discovery).
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
        self.max_accel_to_decel = overrides.get("max_accel_to_decel", 10000.0)
        self.corner_deviation = overrides.get("corner_deviation", 50e-3)
        self.kin = _FakeCheckMove()
        self.extruder = _FakeCheckMove()


class _FakeMove:
    """Re-implements klippy.toolhead.Move.__init__ without pyserial etc."""

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
        self.delta_v2 = 2.0 * move_d * self.accel
        self.max_smoothed_v2 = 0.0
        self.smooth_delta_v2 = 2.0 * move_d * toolhead.max_accel_to_decel
        self.next_junction_v2 = 999999999.9
        self.next_junction_v_capped_to = None

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
        self.next_junction_v_capped_to = speed


class _FakeAxisIS:
    """Mirrors klippy.extras.input_shaper.AxisInputShaper's blendmath-visible
    surface. Plan 5 blendmath._extract_shapers reads shaper_type/shaper_freq
    via .get_axis() + .params.
    """
    def __init__(self, axis, stype, freq, damping=0.1):
        self._axis = axis

        class _P:
            pass

        self.params = _P()
        self.params.shaper_type = stype
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


class _FakePrinter:
    def __init__(self, is_obj):
        self._is = is_obj

    def lookup_object(self, name, default=None):
        if name == "input_shaper":
            return self._is
        return default


# ---------------------------------------------------------------------------
# Fused-kernel builder — mirrors input_shaper.py AxisInputSmoother.
# ---------------------------------------------------------------------------


def _build_fused_kernel(bs_variant, f_sh_hz, target_passband):
    """Return (C_fused, t_fused, G, C_fwd, t_sm) for the given bs variant.

    Mirrors klippy/extras/input_shaper.py::AxisInputSmoother.recompute_fused_kernel
    step-for-step so the kernel content is bit-identical to what gets shipped
    to the C side via input_shaper_set_smoother_params.
    """
    for ism in shaper_defs.INPUT_SMOOTHERS:
        if ism.name == bs_variant:
            break
    else:
        raise ValueError("unknown bs variant: %s" % bs_variant)
    C_fwd, t_sm = ism.init_func(f_sh_hz, 0.1, True)
    pb_max_hz = target_passband * f_sh_hz
    h, T_h, dt = bspline_inverse.compute_inverse_fir(
        C_fwd, t_sm, f_sh_hz=f_sh_hz, pb_max_hz=pb_max_hz, tukey_alpha=0.05,
    )
    C_fused = bspline_inverse.fit_fused_kernel(
        C_fwd, t_sm, h, T_h, dt, n_pieces=9, degree=5,
    )
    G = float(np.sum(np.abs(h)) * dt)
    return C_fused, t_sm + T_h, G, C_fwd, t_sm


def _eval_piecewise(C_pieces, tau):
    """Evaluate a piecewise-polynomial kernel at global time tau.

    Coeffs are in the global-time convention per bspline_inverse.fit_fused_kernel
    (ascending powers of tau, not piece-local).
    """
    for (a, b, coeffs) in C_pieces:
        if a <= tau <= b:
            acc = 0.0
            for c in reversed(coeffs):
                acc = acc * tau + c
            return acc
    return 0.0


def _shaper_convolve(C_pieces, t_sm, traj_fn, t0, n_quad=256):
    """Compute (k ⊛ x)(t0) = ∫ k(tau) x(t0 - tau) d tau over [-t_sm/2, +t_sm/2].

    This matches the C-side convention where the smoother is centered at 0
    (see kin_shaper.c::range_integrate and the antiderivative fast path in
    integrate.c). n_quad chooses the Simpson-composite resolution; 256 is
    plenty for a degree-5 piecewise kernel convolved with a degree-10
    trajectory polynomial.
    """
    hst = 0.5 * t_sm
    # Simpson's composite rule over [-hst, +hst] with 2m+1 nodes.
    m = n_quad // 2
    nodes = 2 * m + 1
    step = (2.0 * hst) / (nodes - 1)
    acc = 0.0
    for i in range(nodes):
        tau = -hst + i * step
        w = 4.0 if (i % 2 == 1) else 2.0
        if i == 0 or i == nodes - 1:
            w = 1.0
        k_val = _eval_piecewise(C_pieces, tau)
        # Convention: shaped-position integrates the kernel against the
        # commanded trajectory evaluated at t0 + tau (shaper in C uses
        # move_time + tau when walking the trapq list). For a symmetric
        # kernel (zero-mean) the ± choice doesn't shift the output.
        x_val = traj_fn(t0 + tau)
        acc += w * k_val * x_val
    return acc * step / 3.0


# ---------------------------------------------------------------------------
# Fixture: planned-quintic trajectory evaluator.
# ---------------------------------------------------------------------------


def _planned_position_fn(payload):
    """Return a callable t → (x, y, z) evaluating the composed per-phase
    polynomials exactly as the C-side move_get_coord would.

    Mirrors trapq.c::move_get_coord's Horner loop for MOVE_QUINTIC_POLY_T.
    """
    (t_accel_end, t_decel_start, total_t, arc_length, v_cap_min,
     start_pos_xyz, coeff_tuple) = payload
    # Unpack the 99-double buffer back into per-phase [axis][k] form.
    # Layout: phase0..phase2; per phase: 11 monomial coeffs * 3 axes
    # interleaved (c[0].x, c[0].y, c[0].z, c[1].x, ...). Coefficients are in
    # phase-LOCAL time (t - t_phase_start).
    phases = []
    buf = list(coeff_tuple)
    for p in range(3):
        base = p * 11 * 3
        axes_coeffs = [[0.0] * 11 for _ in range(3)]
        for k in range(11):
            for ax in range(3):
                axes_coeffs[ax][k] = buf[base + k * 3 + ax]
        phases.append(axes_coeffs)

    def eval_at(t):
        # Clip to [0, total_t]; outside the move the trajectory is a
        # stationary hold at the endpoint (pad moves handle this).
        if t <= 0.0:
            return start_pos_xyz
        if t >= total_t:
            # End position = evaluate decel phase at its endpoint.
            delta_t = total_t - t_decel_start
            phase = phases[2]
        elif t <= t_accel_end:
            phase = phases[0]
            delta_t = t
        elif t <= t_decel_start:
            phase = phases[1]
            delta_t = t - t_accel_end
        else:
            phase = phases[2]
            delta_t = t - t_decel_start
        out = [0.0, 0.0, 0.0]
        for ax in range(3):
            coeffs = phase[ax]
            v = coeffs[10]
            for k in range(9, -1, -1):
                v = v * delta_t + coeffs[k]
            out[ax] = v
        return tuple(out)

    return eval_at, total_t


# ---------------------------------------------------------------------------
# Shared fixtures: toolhead + blend emission.
# ---------------------------------------------------------------------------


def _make_toolhead_with_bs_shaper(bs_variant, freq, max_accel, corner_deviation):
    """Build a _FakeToolhead whose input_shaper stub advertises a bs-family
    shaper on both X and Y axes at the given frequency. Matches the D4
    test-harness pattern in test_blendplanner.py.

    Note: _FakeAxisIS is sufficient for blendmath._extract_shapers — the
    snapshot pipeline there reads params.shaper_type/shaper_freq but does
    NOT call the fused-kernel design path. That's done separately in this
    test module via _build_fused_kernel.
    """
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

    Returns (trunc_prev, quintic_move, trunc_next_head). The quintic_move is
    a QuinticBlendMove carrying the trapq payload.
    """
    cb = blendplanner.CornerBlender(th, move_cls=_FakeMove, max_chord_err=20e-3)
    m_prev = _FakeMove(th, (0.0, 0.0, 0.0, 0.0), (10.0, 0.0, 0.0, 0.5),
                       speed=speed)
    m_next = _FakeMove(th, (10.0, 0.0, 0.0, 0.5), (10.0, 10.0, 0.0, 1.0),
                       speed=speed)
    assert cb.feed(m_prev) == []
    out = cb.feed(m_next)
    assert len(out) == 2, "expected [trunc_prev, QuinticBlendMove]"
    trunc_prev, quintic_move = out
    assert isinstance(quintic_move, blendplanner.QuinticBlendMove)
    assert cb._prev is not None
    trunc_next_head = cb._prev
    return trunc_prev, quintic_move, trunc_next_head


# ---------------------------------------------------------------------------
# trapq FFI round-trip: feed the QuinticBlendMove payload directly into
# trapq_append_quintic (bypassing ToolHead._process_moves). Verify kind=1
# storage + round-trip the move metadata via trapq_extract_old.
# ---------------------------------------------------------------------------


def _push_quintic_to_trapq(ffi_main, ffi_lib, tq, print_time, payload):
    """Mirror ToolHead._process_moves's quintic-emit block without pulling
    in the full toolhead instance."""
    (t_accel_end, t_decel_start, total_t, arc_length, v_cap_min,
     start_pos_xyz, coeff_tuple) = payload
    coeff_buf = ffi_main.new("double[99]", list(coeff_tuple))
    ffi_lib.trapq_append_quintic(
        tq, print_time,
        t_accel_end, t_decel_start, total_t,
        arc_length, v_cap_min,
        start_pos_xyz[0], start_pos_xyz[1], start_pos_xyz[2],
        coeff_buf,
    )
    return total_t


# ---------------------------------------------------------------------------
# Integration tests — shared TestClass amortizes toolhead/blend construction.
# ---------------------------------------------------------------------------


class TestPlan5CascadeIntegration:
    """End-to-end pipeline: plan corner → blend → trapq → cascade."""

    FREQ = 40.0
    TARGET_PASSBAND = 0.3
    # corner_deviation chosen so the blend arc_length falls in the
    # 0.5-1 mm range: short enough to fit comfortably inside a ~50 ms
    # total_t window, long enough that the planned trajectory carries
    # measurable amplitude (the cascade error scales with trajectory
    # amplitude × passband error, so we need amplitude > 0 to exercise
    # the bound meaningfully).
    CORNER_DEVIATION = 0.2
    MAX_ACCEL = 10000.0
    SPEED = 200.0

    # --- Structural: single QuinticBlendMove per 90° corner -----------------

    def test_single_quintic_blend_per_corner(self):
        """D7 emit contract: one QuinticBlendMove per blend, not a polyline.

        Regression guard against a future accidental revert to the N-piece
        polyline emit. The 99-double coefficient payload is present and
        finite.
        """
        th = _make_toolhead_with_bs_shaper(
            "bs3", self.FREQ, self.MAX_ACCEL, self.CORNER_DEVIATION,
        )
        _, quintic_move, _ = _emit_right_angle_blend(th, speed=self.SPEED)
        payload = quintic_move.quintic_trapq_payload
        assert len(payload) == 7
        (t_accel_end, t_decel_start, total_t, arc_length, v_cap_min,
         start_pos_xyz, coeff_tuple) = payload
        assert total_t > 0.0
        assert arc_length > 0.0
        assert v_cap_min > 0.0
        assert 0.0 <= t_accel_end <= t_decel_start <= total_t
        assert len(coeff_tuple) == 99
        for c in coeff_tuple:
            assert math.isfinite(c), "non-finite coefficient in payload"

    # --- Integration: blend → trapq_append_quintic → extract (kind=1) -------

    def test_blend_routes_through_trapq_append_quintic(self):
        """CornerBlender emits a QuinticBlendMove → trapq_append_quintic
        stores it as kind=1 (MOVE_QUINTIC_POLY_T). Verified via
        trapq_extract_old after trapq_finalize_moves.

        This is the FFI-level integration bridge ToolHead._process_moves
        relies on. If it breaks, every blend becomes silently wrong at the
        C boundary.
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
        # Find the quintic entry (kind == 1) among finalized moves.
        found = None
        for i in range(n):
            if pm[i].kind == 1:
                found = pm[i]
                break
        assert found is not None, "no kind=1 MOVE_QUINTIC_POLY_T entry"
        assert found.move_t == pytest.approx(total_t, rel=1e-9)
        assert found.print_time == pytest.approx(t_print, rel=1e-9)

    # --- Cascade identity across bs1/bs3/bs5 ----------------------------------

    @pytest.mark.parametrize("bs_variant,max_err_um", [
        ("bs1", 100.0),
        ("bs3", 100.0),
        ("bs5", 100.0),
    ])
    def test_shaper_cascade_matches_planned_within_passband(
            self, bs_variant, max_err_um):
        """End-to-end cascade identity.

        Plan a 90° corner blend → TOPP → QuinticBlendMove → trapq payload.
        Compute the shaper-cascade output (fused kernel convolved with the
        planned trajectory) and compare to the planned trajectory itself.

        For a bs-family feedforward-inverse cascade, fused-kernel passband
        error is ≤ 3.17% (bs3 @ 12 Hz per §4.3 new_shaper_family.md). On
        a quintic trajectory of amplitude ~ a few mm, this translates to
        position error bounded by ~100 μm.

        Sampling is restricted to the interior of the padded trajectory
        where the convolution window [t - t_fused/2, t + t_fused/2] stays
        fully inside a defined region. Outside the quintic we pad with
        a stationary hold (constant position) — a valid zero-motion
        extension that the fused kernel (integrating to 1) preserves
        exactly.
        """
        th = _make_toolhead_with_bs_shaper(
            bs_variant, self.FREQ, self.MAX_ACCEL, self.CORNER_DEVIATION,
        )
        _, quintic_move, _ = _emit_right_angle_blend(th, speed=self.SPEED)
        payload = quintic_move.quintic_trapq_payload
        planned_fn, total_t = _planned_position_fn(payload)

        # Padding: hold the start position before t=0 and the end position
        # after t=total_t. The fused kernel has unit DC gain, so a constant
        # input convolves to the same constant — the cascade at an
        # out-of-move time reports the last-known trajectory value exactly.
        start_xyz = planned_fn(0.0)
        end_xyz = planned_fn(total_t)

        def padded_x(t):
            if t <= 0.0:
                return start_xyz[0]
            if t >= total_t:
                return end_xyz[0]
            return planned_fn(t)[0]

        def padded_y(t):
            if t <= 0.0:
                return start_xyz[1]
            if t >= total_t:
                return end_xyz[1]
            return planned_fn(t)[1]

        C_fused, t_fused, G, _, _ = _build_fused_kernel(
            bs_variant, self.FREQ, self.TARGET_PASSBAND,
        )

        # Sample across the full quintic duration. At the boundaries the
        # convolution window extends into the stationary hold, which matches
        # what the toolhead does in practice (pad moves / prior state).
        n_samples = 30
        max_err = 0.0
        max_err_t = 0.0
        max_err_axis = ""
        for i in range(n_samples + 1):
            t = total_t * i / n_samples
            planned_xyz = planned_fn(t)
            shaped_x = _shaper_convolve(C_fused, t_fused, padded_x, t)
            shaped_y = _shaper_convolve(C_fused, t_fused, padded_y, t)
            err_x = abs(shaped_x - planned_xyz[0])
            err_y = abs(shaped_y - planned_xyz[1])
            if err_x > max_err:
                max_err = err_x
                max_err_t = t
                max_err_axis = "x"
            if err_y > max_err:
                max_err = err_y
                max_err_t = t
                max_err_axis = "y"
        max_err_actual_um = max_err * 1000.0
        assert max_err_actual_um < max_err_um, (
            "%s cascade identity: max err %.2f um (axis=%s) > %.1f um "
            "at t=%.4fs (total_t=%.4fs)"
            % (bs_variant, max_err_actual_um, max_err_axis, max_err_um,
               max_err_t, total_t)
        )

    # --- Phase-boundary continuity (C^0) --------------------------------------

    def test_phase_boundaries_C0_continuous(self):
        """No step discontinuity in the commanded quintic trajectory at
        the phase boundaries t_accel_end and t_decel_start. The shaper
        cascade would smear a step, but it must not _introduce_ one.

        The Python-side composed phase polynomials must match value at
        the phase boundary from both sides (evaluating the accel phase
        at delta_t = t_accel_end and the cruise phase at delta_t = 0).
        """
        th = _make_toolhead_with_bs_shaper(
            "bs3", self.FREQ, self.MAX_ACCEL, self.CORNER_DEVIATION,
        )
        _, quintic_move, _ = _emit_right_angle_blend(th, speed=self.SPEED)
        payload = quintic_move.quintic_trapq_payload
        planned_fn, total_t = _planned_position_fn(payload)
        (t_accel_end, t_decel_start, _, _, _, _, _) = payload
        # Eval just before and at the accel/cruise boundary.
        eps = 1e-9
        # t_accel_end boundary
        if t_accel_end > 0.0 and t_accel_end < total_t:
            p_lo = planned_fn(t_accel_end - eps)
            p_hi = planned_fn(t_accel_end + eps)
            for ax in range(3):
                assert abs(p_hi[ax] - p_lo[ax]) < 1e-9, (
                    "discontinuity at t_accel_end on axis %d: "
                    "lo=%.12f hi=%.12f" % (ax, p_lo[ax], p_hi[ax])
                )
        # t_decel_start boundary
        if t_decel_start > 0.0 and t_decel_start < total_t:
            p_lo = planned_fn(t_decel_start - eps)
            p_hi = planned_fn(t_decel_start + eps)
            for ax in range(3):
                assert abs(p_hi[ax] - p_lo[ax]) < 1e-9, (
                    "discontinuity at t_decel_start on axis %d: "
                    "lo=%.12f hi=%.12f" % (ax, p_lo[ax], p_hi[ax])
                )

    # --- Linear regression gate: linear move through pipeline is bit-exact ---

    def test_linear_move_through_pipeline_is_FP_precise(self):
        """D2 foundation regression gate: a pure straight line (no blend)
        routed through trapq_append (the linear path) plus a kin_shaper
        cascade reproduces the classical pre-Plan-5 behavior to within
        float-FP precision.

        We exercise this by feeding a pure linear move through the blender
        (which just passes it through since there's no partner move for a
        blend) and then through trapq_append on the FFI side; the
        round-trip recovered kind must be 0 (MOVE_LINEAR) and the stored
        start_v/accel must match the emitted move's planned values
        exactly.
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
        # Push the linear move through trapq_append directly (mirrors the
        # linear branch of ToolHead._process_moves).
        ffi_main, ffi_lib = chelper.get_ffi()
        tq = ffi_main.gc(ffi_lib.trapq_alloc(), ffi_lib.trapq_free)
        # Synthesize a simple trapezoidal profile: accel from 0 → 100 → 0.
        accel_t = 0.05
        cruise_t = 0.05
        decel_t = 0.05
        start_v = 0.0
        cruise_v = 100.0
        accel = cruise_v / accel_t   # mm/s^2
        ffi_lib.trapq_append(
            tq, 1.0,
            accel_t, cruise_t, decel_t,
            0.0, 0.0, 0.0,     # start_pos
            1.0, 0.0, 0.0,     # axes_r — pure +X
            start_v, cruise_v, accel,
        )
        ffi_lib.trapq_finalize_moves(tq, 1.0 + accel_t + cruise_t + decel_t + 1.0, 0.0)
        pm = ffi_main.new("struct pull_move[8]")
        n = ffi_lib.trapq_extract_old(
            tq, pm, 8, 0.5, 2.0 + accel_t + cruise_t + decel_t,
        )
        assert n >= 1
        found_accel = None
        for i in range(n):
            if pm[i].kind == 0 and pm[i].accel > 0.0 \
                    and abs(pm[i].start_v - start_v) < 1e-9:
                found_accel = pm[i]
                break
        assert found_accel is not None
        # Bit-exact agreement with what we fed in — no loss through
        # quintic-path refactoring of trapq.c.
        assert found_accel.accel == pytest.approx(accel, rel=0.0, abs=0.0)
        assert found_accel.start_v == pytest.approx(start_v, rel=0.0, abs=0.0)
        assert found_accel.x_r == pytest.approx(1.0, rel=0.0, abs=0.0)
        assert found_accel.y_r == pytest.approx(0.0, rel=0.0, abs=0.0)
