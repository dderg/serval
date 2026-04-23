# klippy/blendplanner.py
# Copyright (C) 2026
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# Corner-blending planner integration.
# See docs/superpowers/specs/2026-04-17-planner-integration-design.md
from __future__ import annotations

import math

from . import blendmath, blendquintic, blendshape, topp
from .chelper import bs_compose as _bs_compose
from .chelper import fir_compose as _fir_compose
from .chelper import linear_pa_compose as _linear_pa_compose
from .extras import shaper_defs as _shaper_defs


def _extract_extruder_caps(toolhead):
    """Pull the ExtruderLimits off the toolhead's extruder if configured.

    Returns None when the extruder cap is disabled or no extruder is
    present — the downstream shape-build code treats None as 'no cap'.

    Plan 5 D7 consumes this as the per-s v_extr(s) contribution inside
    v_cap_fn (see `klippy/blendquintic.py::_v_extr_of_k`). Before D7
    the same data fed blendextruder.cap_move as a per-move cap;
    that call-site is retired in D7 — flow-rate capping now lives
    entirely inside the unified v_cap_fn.
    """
    # Prefer the cached snapshot on the toolhead (ToolHead.extruder_cap_snapshot,
    # refreshed by SET_PRESSURE_ADVANCE etc.); fall back to re-querying the
    # extruder for test shims that don't wire the cache.
    cached = getattr(toolhead, "extruder_cap_snapshot", None)
    if cached is not None:
        _, limits = cached
        return limits
    extruder = getattr(toolhead, "extruder", None)
    if extruder is None:
        return None
    snap_fn = getattr(extruder, "extruder_limits_snapshot", None)
    if snap_fn is None:
        # PrinterExtruder delegates to its primary ExtruderStepper.
        steppers = getattr(extruder, "extruder_steppers", None)
        if steppers:
            snap_fn = getattr(steppers[0], "extruder_limits_snapshot", None)
    if snap_fn is None:
        return None
    snapshot = snap_fn()
    if snapshot is None:
        return None
    _, limits = snapshot
    return limits


def _extract_pa_snapshot(toolhead):
    """Pull the PAModelSnapshot off the toolhead's extruder, if configured.

    Plan 5 D7: v_cap_fn needs the live PA model to compute the per-s
    extruder-flow cap. Returns None when PA isn't enabled.
    """
    cached = getattr(toolhead, "extruder_cap_snapshot", None)
    if cached is not None:
        pa_snap, _ = cached
        return pa_snap
    extruder = getattr(toolhead, "extruder", None)
    if extruder is None:
        return None
    snap_fn = getattr(extruder, "extruder_limits_snapshot", None)
    if snap_fn is None:
        steppers = getattr(extruder, "extruder_steppers", None)
        if steppers:
            snap_fn = getattr(steppers[0], "extruder_limits_snapshot", None)
    if snap_fn is None:
        return None
    snapshot = snap_fn()
    if snapshot is None:
        return None
    pa_snap, _ = snapshot
    return pa_snap


_BS_ORDERS = {f"bs{i}": i for i in range(1, 6)}


def _resolve_linear_k_pa(toolhead):
    """Return the linear-PA coefficient (k_pa) currently configured on the
    toolhead's extruder, or 0.0 if PA is disabled or the configured model
    is non-linear.

    Plan 8 Chunk 3 Stage A: only the 'linear' PA model is composed at plan
    time. Non-linear models ('tanh', 'recipr') will be handled by Stage B
    via piecewise Chebyshev fits; until then their k_pa contribution is
    treated as zero (the .e slot still carries extr_r * P_proj, the
    nominal filament motion without PA kick).
    """
    pa_snap = _extract_pa_snapshot(toolhead)
    if pa_snap is None:
        return 0.0
    if pa_snap.kind != "linear":
        return 0.0
    if not pa_snap.params:
        return 0.0
    return float(pa_snap.params[0])


def _bake_shaper_polynomial(
    accel_polys, cruise_polys, decel_polys,
    t_accel_end, t_decel_start, total_t,
    shapers,
    shape_disabled=False,
):
    """Apply the configured input-shaping kernel at plan time.

    Takes the raw (accel, cruise, decel) quintic phase polynomials produced
    by QuinticShape.compose_phase_polynomials plus their phase timings,
    and — depending on the axis shaper snapshots — returns a baked
    piecewise-polynomial representation:

      - ``shape_disabled`` set (homing / force / manual / pure-E): skip
        baking entirely. Returns the legacy 3-phase layout.
      - No shaper configured (no shapers, disabled, or mismatched per-
        axis types): pass-through. Returns the legacy 3-phase layout.
      - FIR shaper (zv / mzv): all axes must share the same shaper_type,
        freq, damping — run fir_compose once, output shape extends
        beyond total_t by the last impulse delay.
      - Smooth-IS (bs1..bs5): same homogeneous-kernel requirement —
        run bs_compose.

    Returns
    -------
    (phase_t_ends, total_t_baked, coeff_buf_flat)
        phase_t_ends : tuple[float] of length n_phases (absolute move-local).
        total_t_baked: phase_t_ends[-1] (may be longer than input total_t
                       for FIR due to kernel support extension).
        coeff_buf_flat: tuple[float] of length n_phases * 15 * 4 in the
                        interleaved layout the C trapq_append_quintic wants
                        (Plan 8 Chunk 3: x, y, z, e per coeff). The .e
                        slot is left zero here — the linear-PA composer
                        (Chunk 3 Task 2) populates it from the baked XY
                        polynomial after this returns.

    Heterogeneous per-axis shapers are not supported in Chunk 2; when
    detected we fall back to pass-through. Chunk 3 (per-axis polynomial
    baking) will remove that restriction.
    """
    # Helper that packs the legacy 3-phase output as a flat coeff buffer.
    # Plan 8 Chunk 3: 4-axis stride — .e left zero, populated downstream.
    def _legacy_passthrough():
        phase_t_ends = (t_accel_end, t_decel_start, total_t)
        flat = []
        for phase in (accel_polys, cruise_polys, decel_polys):
            for k in range(15):
                flat.append(phase[0][k])
                flat.append(phase[1][k])
                flat.append(phase[2][k])
                flat.append(0.0)  # .e slot
        return phase_t_ends, total_t, tuple(flat)

    if shape_disabled:
        # Emit site requested unshaped output — skip the whole bake path.
        return _legacy_passthrough()
    if not shapers:
        return _legacy_passthrough()
    # Filter snapshots that actually carry a shaper.
    active = [s for s in shapers if s.shaper_freq > 0.0 and s.shaper_type]
    if not active:
        return _legacy_passthrough()
    # Chunk 2 simplification: all active axes must share the same shaper
    # config. Heterogeneous = fall back to pass-through.
    first = active[0]
    for s in active[1:]:
        if (s.shaper_type != first.shaper_type
                or abs(s.shaper_freq - first.shaper_freq) > 1e-9
                or abs(s.damping_ratio - first.damping_ratio) > 1e-9):
            return _legacy_passthrough()
    shaper_type = first.shaper_type
    freq = first.shaper_freq
    damping = first.damping_ratio

    # Build the input (3-phase) flat buffer.
    in_phase_t_ends = [t_accel_end, t_decel_start, total_t]
    # Collapse zero-duration phases: bs / fir composers tolerate them but
    # emitting fewer input phases keeps the breakpoint grid tighter. We
    # still pass the 3-phase layout as-is here; the composer's break-dedup
    # handles zero-length phases.
    in_coeffs = []
    for phase in (accel_polys, cruise_polys, decel_polys):
        for k in range(15):
            in_coeffs.append(phase[0][k])
            in_coeffs.append(phase[1][k])
            in_coeffs.append(phase[2][k])
            in_coeffs.append(0.0)  # .e slot — populated by linear_pa_compose

    try:
        if shaper_type in _BS_ORDERS:
            order = _BS_ORDERS[shaper_type]
            phase_t_ends, out_coeffs = _bs_compose.bs_compose(
                in_phase_t_ends, in_coeffs,
                bs_order=order,
                shaper_freq=freq,
                damping_ratio=damping,
            )
        elif shaper_type == "zv":
            A, T = _shaper_defs.get_zv_shaper(freq, damping)
            phase_t_ends, out_coeffs = _fir_compose.fir_compose(
                in_phase_t_ends, in_coeffs,
                impulse_amplitudes=A, impulse_delays=T,
            )
        elif shaper_type == "mzv":
            A, T = _shaper_defs.get_mzv_shaper(freq, damping)
            phase_t_ends, out_coeffs = _fir_compose.fir_compose(
                in_phase_t_ends, in_coeffs,
                impulse_amplitudes=A, impulse_delays=T,
            )
        else:
            return _legacy_passthrough()
    except ValueError:
        # Composer bailed (overflow or bad args). Fall back to the
        # unshaped polynomial — safer than dropping the move.
        return _legacy_passthrough()

    if not phase_t_ends:
        return _legacy_passthrough()
    return tuple(phase_t_ends), phase_t_ends[-1], tuple(out_coeffs)


def _copy_caller_state(src, dst):
    """Transfer caller-mutable Move state from src to the truncated dst.

    Pins caller-intent fields verbatim (timing_callbacks, next_junction_v2,
    max_cruise_v2, accel) so that M204 / SET_VELOCITY_LIMIT
    / register_lookahead_callback mutations applied upstream to src survive
    the emit-time construction of dst. Recomputes length-derived fields
    (delta_v2, smooth_delta_v2, min_move_t) from dst's NEW move_d and the
    pinned accel.

    The accel pin is a direct assignment (not via dst.limit_speed) because
    limit_speed takes min(self.accel, accel); if an intervening M204 had
    lowered toolhead.max_accel between src construction and emit, Move.__init__'s
    snapshot of the new (lower) value would win over src.accel.
    """
    dst.timing_callbacks = list(src.timing_callbacks)
    dst.next_junction_v2 = src.next_junction_v2
    dst.max_cruise_v2 = src.max_cruise_v2
    dst.accel = src.accel
    dst.delta_v2 = 2.0 * dst.move_d * dst.accel
    ratio = src.smooth_delta_v2 / src.delta_v2 if src.delta_v2 > 0.0 else 1.0
    dst.smooth_delta_v2 = min(
        dst.delta_v2, 2.0 * dst.move_d * dst.accel * ratio
    )
    dst.min_move_t = dst.move_d / math.sqrt(dst.max_cruise_v2)


class QuinticBlendMove:
    """Move-like wrapper for a direct-quintic blend emission.

    Holds the per-phase position-in-t polynomial coefficients produced by
    QuinticShape.compose_phase_polynomials, plus enough Move-compatible
    state (start_pos, end_pos, axes_r, move_d, max_cruise_v2, accel, etc.)
    for the LookAheadQueue / Move lifecycle.

    toolhead.ToolHead._process_moves detects `quintic_trapq_payload` on a
    Move and routes it to trapq_append_quintic instead of the default
    trapq_append call. Plan 5 D7 flipped CornerBlender._emit_blend to emit
    a single QuinticBlendMove per blend, replacing the N-piece polyline
    loop. The TOPP-derived trapezoid-in-s profile (cruise_v, s_accel_end,
    s_decel_start) composes with the quintic position(u) to produce the
    per-phase polynomial coefficients.

    The `quintic_trapq_payload` attribute is a tuple
    (phase_t_ends_tuple, total_t_baked, arc_length, v_cap_min,
     start_pos_xyz, coeff_tuple, legacy_t_accel_end, legacy_t_decel_start,
     legacy_total_t). The first 6 fields feed trapq_append_quintic
     directly; the trailing legacy trio stays around for callers like
     set_junction that still want the underlying trapezoid-in-s phase
     timings (which may differ from the baked polynomial's phase
     structure when an FIR shaper extends the move duration).
    """

    def __init__(self, toolhead, shape, start_pos_4d, end_pos_4d,
                 v_in, v_out, cruise_v, s_accel_end, s_decel_start,
                 a_max, v_cap_min, accel=None):
        self.toolhead = toolhead
        self.shape = shape
        self.start_pos = tuple(start_pos_4d)
        self.end_pos = tuple(end_pos_4d)
        axes_d = [end_pos_4d[i] - start_pos_4d[i] for i in range(4)]
        self.axes_d = axes_d
        # Chord-length axes_r so the LookAheadQueue's set_junction sees a
        # sensible "direction" for the blend. The true per-axis motion
        # comes from the quintic polynomial, not axes_r, but outer
        # lookahead uses it for axis-unit dot products.
        move_d = math.sqrt(sum(d * d for d in axes_d[:3]))
        inv = 1.0 / move_d if move_d else 0.0
        self.axes_r = tuple(d * inv for d in axes_d)
        self.move_d = shape.arc_length if shape.arc_length > 0.0 else move_d
        self.accel = accel if accel is not None else a_max
        self.timing_callbacks = []
        self.is_kinematic_move = True
        self.max_cruise_v2 = cruise_v * cruise_v
        self.max_start_v2 = v_in * v_in
        self.max_smoothed_v2 = v_in * v_in
        (accel_polys, cruise_polys, decel_polys, t_accel_end, t_decel_start,
         total_t, arc_length) = shape.compose_phase_polynomials(
            v_in=v_in, v_out=v_out, cruise_v=cruise_v, a_max=a_max,
            s_accel_end=s_accel_end, s_decel_start=s_decel_start,
        )
        # Plan 8 Chunk 2 — bake the configured input-shaping kernel into
        # the polynomial at plan time (Tasks 7–9). For unshaped axes or
        # disabled shapers this is a pass-through; for bs1..bs5 it calls
        # bs_compose, and for zv / mzv it calls fir_compose. The baked
        # payload may expand from the legacy 3-phase layout to up to
        # MOVE_MAX_PIECES phases.
        shape_limits = getattr(shape, "_limits", None)
        shaper_list = (
            getattr(shape_limits, "shapers", None) if shape_limits is not None
            else None
        ) or []
        phase_t_ends_tuple, total_t_baked, coeff_tuple = _bake_shaper_polynomial(
            accel_polys, cruise_polys, decel_polys,
            t_accel_end, t_decel_start, total_t,
            shaper_list or [],
        )
        # Plan 8 Chunk 3 Task 3 — bake linear PA into the .e slot of the
        # baked polynomial. For non-linear PA the .e slot stays zero
        # (Stage B's nonlinear_pa_compose lands later). For linear or
        # zero PA we run the composer; the .e content is ready for the
        # Stage C extruder-stepper rewrite to consume directly. Until
        # Stage C lands, the .e slot is harmlessly populated but unread
        # (extruder still convolves PA on its own trapq).
        n_phases_baked = len(phase_t_ends_tuple)
        # axes_d[3] is signed E displacement; arc_length is the curve length
        # of the XY blend. extr_r = E-displacement per XY-arc-mm (signed).
        if arc_length > 0.0:
            extr_r = axes_d[3] / arc_length
        else:
            extr_r = 0.0
        # XY direction n: chord-direction unit vector. For straight moves
        # this matches the actual XY motion; for curved blends it's the
        # legacy approximation (kin_extruder.c made the same choice).
        axis_n = (self.axes_r[0], self.axes_r[1], self.axes_r[2])
        k_pa = _resolve_linear_k_pa(toolhead)
        coeff_tuple = tuple(_linear_pa_compose.linear_pa_compose(
            n_phases_baked, list(coeff_tuple),
            axis_n=axis_n, extr_r=extr_r, k_pa=k_pa,
        ))
        start_pos_xyz = (start_pos_4d[0], start_pos_4d[1], start_pos_4d[2])
        if not (v_cap_min and math.isfinite(v_cap_min)) or v_cap_min <= 0.0:
            v_cap_min = cruise_v if cruise_v > 0.0 else v_in
        self.v_cap_min = v_cap_min
        # New payload layout carries a variable-length phase_t_ends tuple
        # plus the flat coeff buffer. Consumer (toolhead._process_moves)
        # reads n_phases = len(phase_t_ends_tuple). Total duration is
        # phase_t_ends_tuple[-1] (= total_t_baked), which may exceed the
        # input total_t for FIR shapers that extend the move by max_tau.
        self.quintic_trapq_payload = (
            phase_t_ends_tuple, total_t_baked,
            arc_length, v_cap_min, start_pos_xyz, coeff_tuple,
            # Legacy 3-phase timings for consumers that still need them
            # (set_junction, extruder.move timing, min_move_t). These
            # always reflect the INPUT (unshaped) timings; the baked
            # polynomial spans phase_t_ends_tuple[-1], which can be
            # longer.
            t_accel_end, t_decel_start, total_t,
        )
        self.min_move_t = total_t if total_t > 0.0 else (
            self.move_d / cruise_v if cruise_v > 0.0 else 0.0
        )
        self.delta_v2 = 2.0 * self.move_d * self.accel
        self.smooth_delta_v2 = self.delta_v2
        self.next_junction_v2 = v_cap_min * v_cap_min if v_cap_min else 0.0
        self.next_junction_v_capped_to = None

    def limit_speed(self, speed, accel):
        v2 = speed * speed
        if v2 < self.max_cruise_v2:
            self.max_cruise_v2 = v2
        self.accel = min(self.accel, accel)
        self.delta_v2 = 2.0 * self.move_d * self.accel
        self.smooth_delta_v2 = min(self.smooth_delta_v2, self.delta_v2)

    def limit_next_junction_speed(self, speed):
        v2 = speed * speed
        self.next_junction_v2 = min(self.next_junction_v2, v2)
        self.next_junction_v_capped_to = speed

    def calc_junction(self, prev_move):
        # The blend's v_in is pointwise-safe via TOPP + v_cap_fn (D7
        # Option Z); centripetal and extruder flow caps are already
        # composed into v_cap_fn at emit time. Skip both here and just
        # run the upstream cascade so the outer lookahead can still
        # tighten max_start_v2 / max_smoothed_v2 when the predecessor
        # turns out to be rate-limited.
        if not self.is_kinematic_move or not prev_move.is_kinematic_move:
            return
        max_start_v2 = min(
            self.max_start_v2,
            self.max_cruise_v2,
            prev_move.max_cruise_v2,
            prev_move.next_junction_v2,
            prev_move.max_start_v2 + prev_move.delta_v2,
        )
        self.max_start_v2 = max_start_v2
        self.max_smoothed_v2 = min(
            max_start_v2,
            prev_move.max_smoothed_v2 + prev_move.smooth_delta_v2,
        )

    def set_junction(self, start_v2, cruise_v2, end_v2):
        # Under D7 Option Z the outer lookahead converges to exactly the
        # v_in / cruise_v / v_out that TOPP composed into the quintic
        # phase polynomials, so the (start_v2, cruise_v2, end_v2) passed
        # in should equal the blender's baked-in values. The pre-composed
        # phase timings (t_accel_end, t_decel_start, total_t) are the
        # authoritative ones — trapq_append_quintic steps directly from
        # them in _process_moves regardless of what we store here.
        # Populate the Move-shaped fields (start_v, cruise_v, end_v,
        # accel_t, cruise_t, decel_t) that downstream consumers such as
        # extruder.move expect to find on every move.
        # New payload layout: (phase_t_ends_tuple, total_t_baked,
        # arc_length, v_cap_min, start_pos_xyz, coeff_tuple,
        # t_accel_end, t_decel_start, total_t). The legacy 3-phase
        # (t_accel_end, t_decel_start, total_t) trio is retained at the
        # tail for consumers like set_junction that still want the
        # trapezoid-in-s-timing shape even after the polynomial has been
        # extended by a FIR kernel.
        payload = self.quintic_trapq_payload
        t_accel_end, t_decel_start, total_t = payload[-3], payload[-2], payload[-1]
        self.start_v = math.sqrt(start_v2) if start_v2 > 0.0 else 0.0
        self.cruise_v = math.sqrt(cruise_v2) if cruise_v2 > 0.0 else 0.0
        self.end_v = math.sqrt(end_v2) if end_v2 > 0.0 else 0.0
        self.accel_t = t_accel_end
        self.cruise_t = max(0.0, t_decel_start - t_accel_end)
        self.decel_t = max(0.0, total_t - t_decel_start)


class CornerBlender:
    """Second filter stage in the blend pipeline.

    Buffers one move; on the next arriving move computes a quintic corner
    blend and emits [trunc_prev, QuinticBlendMove] while buffering the
    truncated-next-head as the new candidate prev.

    Plan 5 D7 — `_emit_blend` now emits a **single** QuinticBlendMove per
    corner. The per-phase position-in-t polynomials are composed at emit
    time (Python side) from a trapezoid-in-s velocity profile produced by
    TOPP (klippy/topp.py). The formerly-separate polyline loop and the
    blendextruder.cap_move per-move pass are both retired:

      - The polyline-to-trapq pipeline is replaced by trapq_append_quintic
        (C-side FFI landed in D2); one trapq entry per blend.
      - The extruder flow cap is absorbed into v_cap_fn(s) as the v_extr(s)
        branch so TOPP sees it directly — no separate cap_move call
        remains in toolhead.move.

    The `polyline_moves_emitted` instrumentation counter is kept for
    backwards compatibility with dashboards and older tests (it now
    counts emitted QuinticBlendMove instances rather than polyline
    sub-moves — always equal to `blends_emitted`).
    """

    def __init__(self, toolhead, *, move_cls, max_chord_err=None):
        self._toolhead = toolhead
        self._move_cls = move_cls
        # max_chord_err is retained as a constructor kwarg for backwards
        # compatibility with older callers / tests. After D7 the
        # polyline-chord tolerance no longer gates emit — the whole blend
        # is a single trapq entry — so the value is ignored.
        self.max_chord_err = max_chord_err
        self._prev = None
        self.polyline_moves_emitted = 0
        self.blends_emitted = 0

    def feed(self, move):
        if not move.is_kinematic_move:
            return self.flush() + [move]
        if self._prev is None:
            self._prev = move
            return []
        th = self._toolhead
        limits = blendshape.KinematicLimits(
            a_max=th.max_accel,
            v_max=th.max_velocity,
            jerk_max=None,       # plan 1: jerk cap disabled; plan 5 wires it
            extruder_caps=_extract_extruder_caps(th),  # Plan 5 D7 consumes per-s
            shapers=blendmath._extract_shapers(th),
        )
        shape = blendquintic.QuinticShape.from_moves(
            self._prev, move,
            th.corner_deviation,
            limits,
        )
        if shape is None or blendmath.should_suppress_quintic(
                self._prev, move, th.corner_deviation, shape, th):
            # Drop the blend and fall into the sharp-V suppressed path.
            # Reasons include:
            #   (a) shape is None: collinear corners, near-reversals, or
            #       moves too short to accommodate the blend.
            #   (b) should_suppress_quintic fired on a successfully-formed
            #       shape: two-clause D3 rule determined sharp-V + shaper
            #       is at least as good as the blend (path-tolerance and
            #       time both satisfied).
            return self._suppress_and_advance(move)
        trunc_prev, quintic_move, trunc_next_head = self._emit_blend(
            self._prev, move, shape
        )
        self._prev = trunc_next_head
        self.blends_emitted += 1
        # Post-D7: one QuinticBlendMove per blend, so the legacy
        # polyline-moves counter advances by 1.
        self.polyline_moves_emitted += 1
        return [trunc_prev, quintic_move]

    def _suppress_and_advance(self, move):
        """Fallback path when the blend is dropped — either from_moves
        returned None OR the D3 quintic suppression rule fired on an
        otherwise-valid shape. Caps prev's next-junction velocity via
        suppressed_junction_v when a shaper is loaded; otherwise uses
        the near-reversal heuristic as a safety net.

        suppressed_junction_v derives an SCV-equivalent cap from the
        active shaper's sigma_T; it is shape-agnostic (works for both
        the shape=None case and the suppression case).
        """
        th = self._toolhead
        v_j = blendmath.suppressed_junction_v(
            self._prev, move, th.corner_deviation, th
        )
        if v_j is not None and math.isfinite(v_j):
            self._prev.limit_next_junction_speed(v_j)
        else:
            # No shaper loaded (or v_j undefined). Fall back to the
            # near-reversal hard-stop heuristic so the toolhead
            # doesn't round pi-radian reversals at cruise velocity.
            dp = sum(
                self._prev.axes_r[i] * move.axes_r[i] for i in range(3)
            )
            if dp <= -0.5:
                self._prev.limit_next_junction_speed(0.0)
        emitted = [self._prev]
        self._prev = move
        return emitted

    def _emit_blend(self, prev, nxt, shape):
        """Construct [trunc_prev, QuinticBlendMove] and the trunc_next_head.

        `shape` is a QuinticShape (SmoothShape protocol). Plan 5 D7:
          1. Compose v_cap_fn(s) that captures all 5 cap sources
             (centripetal-saturation, rotation-jerk, shaper-bandwidth,
             user v_max, and the per-s extruder-flow cap).
          2. Sample v_cap_min() for the Option Z upstream junction cap.
          3. Run TOPP on the composed v_cap to get a trapezoid-in-s
             profile.
          4. Emit a single QuinticBlendMove with the per-phase position-
             in-t polynomials built from the TOPP profile.

        Returns (trunc_prev, quintic_move, trunc_next_head).
        """
        th = self._toolhead
        move_cls = self._move_cls

        prev_dir = prev.axes_r[:3]
        next_dir = nxt.axes_r[:3]
        vertex = prev.end_pos[:3]

        # --- 1. Truncated prev ---
        prev_cruise_v = math.sqrt(prev.max_cruise_v2)
        trunc_prev_end_xyz = tuple(
            vertex[i] - shape.d_consumed * prev_dir[i] for i in range(3)
        )
        # E carried proportional to the truncated fraction of prev.move_d.
        frac_prev = 1.0 - shape.d_consumed / prev.move_d
        trunc_prev_end_e = prev.start_pos[3] + frac_prev * prev.axes_d[3]
        trunc_prev_end = (
            trunc_prev_end_xyz[0], trunc_prev_end_xyz[1],
            trunc_prev_end_xyz[2], trunc_prev_end_e,
        )
        trunc_prev = move_cls(th, prev.start_pos, trunc_prev_end, prev_cruise_v)
        _copy_caller_state(prev, trunc_prev)

        # --- 2. Truncated next head (built before quintic emit so we can
        #        pass its start_pos through to the QuinticBlendMove boundary). ---
        trunc_next_head_start_xyz = tuple(
            vertex[i] + shape.d_consumed * next_dir[i] for i in range(3)
        )
        # E at the truncated-next-head start: offset from nxt.start_pos by the
        # consumed head fraction. Symmetric with trunc_prev's E formula.
        frac_consumed_next = shape.d_consumed / nxt.move_d
        trunc_next_head_start_e = (nxt.start_pos[3]
                                   + frac_consumed_next * nxt.axes_d[3])
        trunc_next_head_start = (
            trunc_next_head_start_xyz[0], trunc_next_head_start_xyz[1],
            trunc_next_head_start_xyz[2], trunc_next_head_start_e,
        )
        next_cruise_v = math.sqrt(nxt.max_cruise_v2)
        trunc_next_head = move_cls(
            th, trunc_next_head_start, nxt.end_pos, next_cruise_v
        )
        _copy_caller_state(nxt, trunc_next_head)

        # --- 3. Unified v_cap closure (all 5 cap sources composed per-s). ---
        prev_flow_k = prev.axes_r[3] if len(prev.axes_r) >= 4 else 0.0
        nxt_flow_k = nxt.axes_r[3] if len(nxt.axes_r) >= 4 else 0.0
        pa_snap = _extract_pa_snapshot(th)

        def v_cap_closure(s):
            return shape.v_cap_fn(s, prev_flow_k, nxt_flow_k, pa_snap)

        # Option Z: the blend's true min cap.  Used both as the
        # max_cruise_v2 ceiling for the emitted move and as the
        # next_junction_v2 clamp fed upstream so the lookahead converges
        # to a compatible junction velocity.
        v_cap_min = shape.v_cap_min(prev_flow_k, nxt_flow_k, pa_snap)
        if not math.isfinite(v_cap_min) or v_cap_min <= 0.0:
            v_cap_min = min(prev_cruise_v, next_cruise_v)

        # --- 4. Run TOPP for the trapezoid-in-s velocity profile. ---
        a_max = min(prev.accel, nxt.accel)
        if a_max <= 0.0:
            a_max = th.max_accel
        # Endpoint velocities come from the outer lookahead's upstream
        # pass. The Option Z contract hands the lookahead v_cap_min, so
        # the prev/nxt cruise velocities should already be compatible.
        v_in = min(prev_cruise_v, v_cap_min)
        v_out = min(next_cruise_v, v_cap_min)

        try:
            cruise_v, s_accel_end, s_decel_start = topp.topp_trapezoid(
                v_cap_closure, shape.arc_length,
                v_in=v_in, v_out=v_out, a_max=a_max,
            )
        except topp.TOPPError:
            # Fallback: if the boundary is infeasible (rare; lookahead
            # feed is wrong), collapse to a flat-v_cap_min profile.
            cruise_v = v_cap_min
            s_accel_end = 0.0
            s_decel_start = shape.arc_length
            v_in = v_out = cruise_v

        # --- 5. Emit a single QuinticBlendMove. ---
        start_pos_4d = (
            trunc_prev_end[0], trunc_prev_end[1], trunc_prev_end[2],
            trunc_prev_end_e,
        )
        end_pos_4d = (
            trunc_next_head_start[0], trunc_next_head_start[1],
            trunc_next_head_start[2], trunc_next_head_start_e,
        )
        quintic_move = QuinticBlendMove(
            toolhead=th, shape=shape,
            start_pos_4d=start_pos_4d, end_pos_4d=end_pos_4d,
            v_in=v_in, v_out=v_out,
            cruise_v=cruise_v,
            s_accel_end=s_accel_end, s_decel_start=s_decel_start,
            a_max=a_max, v_cap_min=v_cap_min, accel=a_max,
        )

        # Aggregate-safety re-check. check_move runs before lookahead.add_move
        # in ToolHead.move, so the emitted QuinticBlendMove bypasses it
        # otherwise. One representative is sufficient for the corner
        # envelope check.
        th.kin.check_move(quintic_move)
        if quintic_move.axes_d[3]:
            th.extruder.check_move(quintic_move)

        return trunc_prev, quintic_move, trunc_next_head

    def flush(self):
        if self._prev is None:
            return []
        emitted = [self._prev]
        self._prev = None
        return emitted

    def reset(self):
        self._prev = None

    def peek_buffered(self):
        return [self._prev] if self._prev is not None else []
