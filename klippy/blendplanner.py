# klippy/blendplanner.py
# Copyright (C) 2026
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# Corner-blending planner integration.
# See docs/superpowers/specs/2026-04-17-planner-integration-design.md
from __future__ import annotations

import math

import logging

from . import blendmath, blendquintic, blendshape, jerk_math, topp
from .chelper import bs_compose as _bs_compose
from .chelper import fir_compose as _fir_compose
from .chelper import linear_pa_compose as _linear_pa_compose
from .chelper import nonlinear_pa_compose as _nonlinear_pa_compose
from .chelper import smooth_compose as _smooth_compose
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

# Legacy smooth-IS kernel init functions, keyed by shaper_type. Restored
# alongside the bs family — empirically some printers prefer the smooth-IS
# single-piece polynomials at equivalent support width (see branch notes).
_SMOOTH_IS_INIT_FUNCS = {
    "smooth_zv": _shaper_defs.get_smooth_zv_smoother,
    "smooth_mzv": _shaper_defs.get_smooth_mzv_smoother,
    "smooth_ei": _shaper_defs.get_smooth_ei_smoother,
    "smooth_2hump_ei": _shaper_defs.get_smooth_2hump_ei_smoother,
    "smooth_zvd_ei": _shaper_defs.get_smooth_zvd_ei_smoother,
    "smooth_si": _shaper_defs.get_smooth_si_smoother,
}


# Filament-error threshold above which nonlinear_pa_compose emits a
# warning (per Phase 0 research §6). 1 µm = 1e-3 mm.
_PA_FIT_FILAMENT_WARN_MM = 1e-3


def _resolve_pa_dispatch(toolhead):
    """Return a dispatch tuple describing how to compose E polynomial PA.

    Returns one of:
      ("none", 0.0)                            — PA disabled entirely.
      ("linear", k_pa)                         — linear PA with coeff k_pa.
      ("nonlinear", model, LA, NO, v_lin)      — tanh / recipr PA.

    Plan 8 Chunk 3 Stage B: 'linear' routes to linear_pa_compose (exact
    polynomial arithmetic). 'nonlinear' routes to nonlinear_pa_compose
    with the configured model, linear_advance, nonlinear_offset, and
    linearization_velocity.
    """
    pa_snap = _extract_pa_snapshot(toolhead)
    if pa_snap is None:
        return ("none", 0.0)
    if pa_snap.kind == "linear":
        if not pa_snap.params:
            return ("none", 0.0)
        k_pa = float(pa_snap.params[0])
        if k_pa <= 0.0:
            return ("none", 0.0)
        return ("linear", k_pa)
    if pa_snap.kind in ("tanh", "recipr"):
        if len(pa_snap.params) < 3:
            return ("none", 0.0)
        la, no, v_lin = (float(x) for x in pa_snap.params[:3])
        if la <= 0.0 and no <= 0.0:
            return ("none", 0.0)
        if no > 0.0 and v_lin <= 0.0:
            # Misconfigured snapshot; bail out safely.
            return ("none", 0.0)
        return ("nonlinear", pa_snap.kind, la, no, v_lin)
    return ("none", 0.0)


def _build_unshaped_payload(
    accel_polys, cruise_polys, decel_polys,
    t_accel_end, t_decel_start, total_t,
):
    """Pack the raw (unshaped) 3-phase quintic polynomial as the
    interleaved-axis flat coeff buffer the composers consume.

    Returns (phase_t_ends, total_t, coeff_buf_flat) with .e left zero.
    """
    phase_t_ends = (t_accel_end, t_decel_start, total_t)
    flat = []
    for phase in (accel_polys, cruise_polys, decel_polys):
        for k in range(15):
            flat.append(phase[0][k])
            flat.append(phase[1][k])
            flat.append(phase[2][k])
            flat.append(0.0)  # .e slot
    return phase_t_ends, total_t, tuple(flat)


def offset_unshaped_for_neighbour(unshaped_payload, origin_delta_xyz):
    """Return a neighbour unshaped-payload with its XY polynomial shifted
    by `origin_delta_xyz` so the resulting motion is continuous with
    the current move's reference frame.

    Each QuinticBlendMove's polynomial starts at position 0 at
    t_local=0 (absolute position comes from start_pos_xyz in trapq).
    When used as a neighbour to a different move, the neighbour's
    absolute start position differs from the current move's by
    (neighbour_start_xyz - cur_start_xyz); we add that delta to c[0]
    of every phase / every XY axis so the stream is continuous when
    the composer integrates the kernel across the boundary.

    The .e slot is intentionally NOT offset — the E polynomial is
    populated downstream by linear_pa_compose (which operates on the
    already-baked XY polynomial) and never reaches the neighbour path
    here.
    """
    t_ends, total_t, coeffs = unshaped_payload
    n_phases = len(t_ends)
    flat = list(coeffs)
    dx, dy, dz = origin_delta_xyz
    for p in range(n_phases):
        flat[(p * 15 + 0) * 4 + 0] += dx
        flat[(p * 15 + 0) * 4 + 1] += dy
        flat[(p * 15 + 0) * 4 + 2] += dz
    return t_ends, total_t, tuple(flat)


def bake_shaper_polynomial(
    unshaped_phase_t_ends, unshaped_total_t, unshaped_coeffs,
    shapers,
    shape_disabled=False,
    prev_unshaped=None,
    next_unshaped=None,
):
    """Apply the configured input-shaping kernel at plan time.

    Consumes the pre-packed unshaped polynomial (phase_t_ends, total_t,
    flat coeff buffer — same layout as produced by
    `_build_unshaped_payload`) plus the axis shaper snapshots, and
    returns the baked piecewise polynomial:

      - ``shape_disabled`` set (homing / force / manual / pure-E): skip
        baking. Returns the unshaped polynomial verbatim.
      - No shaper configured: same pass-through.
      - FIR shaper (zv / mzv): all axes must share shaper_type / freq /
        damping — run fir_compose with the supplied neighbour
        polynomials.
      - Cardinal B-spline chain (bs1..bs5): same homogeneous-kernel
        requirement — run bs_compose with neighbour polynomials.
      - Smooth-IS family (smooth_zv / smooth_mzv / smooth_ei /
        smooth_2hump_ei / smooth_zvd_ei / smooth_si): single-piece
        polynomial kernel from the pre-Plan-5 Butyugin design. Routes
        through smooth_compose with the init_func-supplied kernel
        pieces.

    Neighbour polynomials (`prev_unshaped` / `next_unshaped`) are tuples
    of the same shape as the unshaped payload — `(phase_t_ends,
    total_t, coeff_buf)` — or None when no neighbour is available (first
    / last move of a session, or a non-blend neighbour). The composer
    integrates the kernel across move boundaries using these; without
    them the kernel window zero-pads at the boundary, which is correct
    only when the print actually stops there.

    Returns
    -------
    (phase_t_ends, total_t_baked, coeff_buf_flat)
    """
    if shape_disabled or not shapers:
        return unshaped_phase_t_ends, unshaped_total_t, unshaped_coeffs
    active = [s for s in shapers if s.shaper_freq > 0.0 and s.shaper_type]
    if not active:
        return unshaped_phase_t_ends, unshaped_total_t, unshaped_coeffs
    first = active[0]
    for s in active[1:]:
        if (s.shaper_type != first.shaper_type
                or abs(s.shaper_freq - first.shaper_freq) > 1e-9
                or abs(s.damping_ratio - first.damping_ratio) > 1e-9):
            return unshaped_phase_t_ends, unshaped_total_t, unshaped_coeffs
    shaper_type = first.shaper_type
    freq = first.shaper_freq
    damping = first.damping_ratio

    in_phase_t_ends = list(unshaped_phase_t_ends)
    in_coeffs = list(unshaped_coeffs)

    # Unpack neighbour polynomials into composer kwargs. Either neighbour
    # may be None (zero-pad that side).
    neighbour_kwargs = {}
    if prev_unshaped is not None:
        prev_t_ends, prev_T, prev_coeffs = prev_unshaped
        neighbour_kwargs["prev_phase_t_ends"] = list(prev_t_ends)
        neighbour_kwargs["prev_coeffs"] = list(prev_coeffs)
        neighbour_kwargs["prev_T_move"] = float(prev_T)
    if next_unshaped is not None:
        next_t_ends, next_T, next_coeffs = next_unshaped
        neighbour_kwargs["next_phase_t_ends"] = list(next_t_ends)
        neighbour_kwargs["next_coeffs"] = list(next_coeffs)
        neighbour_kwargs["next_T_move"] = float(next_T)

    try:
        if shaper_type in _BS_ORDERS:
            order = _BS_ORDERS[shaper_type]
            phase_t_ends, out_coeffs = _bs_compose.bs_compose(
                in_phase_t_ends, in_coeffs,
                bs_order=order,
                shaper_freq=freq,
                damping_ratio=damping,
                **neighbour_kwargs,
            )
        elif shaper_type in _SMOOTH_IS_INIT_FUNCS:
            init_func = _SMOOTH_IS_INIT_FUNCS[shaper_type]
            # Smooth-IS kernels accept (shaper_freq, damping_ratio,
            # normalize_coeffs=True). damping_ratio is ignored by the
            # current smooth-IS set (kernels are fixed-shape at zeta=0.1)
            # but still passed through for signature parity.
            kernel_pieces, t_sm = init_func(freq, damping, True)
            if not kernel_pieces or t_sm <= 0.0:
                return unshaped_phase_t_ends, unshaped_total_t, unshaped_coeffs
            phase_t_ends, out_coeffs = _smooth_compose.smooth_compose(
                in_phase_t_ends, in_coeffs,
                kernel_pieces=kernel_pieces,
                t_sm=t_sm,
                **neighbour_kwargs,
            )
        elif shaper_type == "zv":
            A, T = _shaper_defs.get_zv_shaper(freq, damping)
            phase_t_ends, out_coeffs = _fir_compose.fir_compose(
                in_phase_t_ends, in_coeffs,
                impulse_amplitudes=A, impulse_delays=T,
                **neighbour_kwargs,
            )
        elif shaper_type == "mzv":
            A, T = _shaper_defs.get_mzv_shaper(freq, damping)
            phase_t_ends, out_coeffs = _fir_compose.fir_compose(
                in_phase_t_ends, in_coeffs,
                impulse_amplitudes=A, impulse_delays=T,
                **neighbour_kwargs,
            )
        else:
            return unshaped_phase_t_ends, unshaped_total_t, unshaped_coeffs
    except ValueError:
        # Composer bailed (overflow or bad args). Fall back to the
        # unshaped polynomial — safer than dropping the move.
        return unshaped_phase_t_ends, unshaped_total_t, unshaped_coeffs

    if not phase_t_ends:
        return unshaped_phase_t_ends, unshaped_total_t, unshaped_coeffs
    return tuple(phase_t_ends), phase_t_ends[-1], tuple(out_coeffs)


def _copy_caller_state(src, dst):
    """Transfer caller-mutable Move state from src to the truncated dst.

    Pins caller-intent fields verbatim (timing_callbacks, next_junction_v2,
    max_cruise_v2, accel) so that M204 / SET_VELOCITY_LIMIT
    / register_lookahead_callback mutations applied upstream to src survive
    the emit-time construction of dst. Recomputes length-derived
    min_move_t from dst's new move_d and pinned max_cruise_v2.

    The accel pin is a direct assignment (not via dst.limit_speed) because
    limit_speed takes min(self.accel, accel); if an intervening M204 had
    lowered toolhead.max_accel between src construction and emit,
    Move.__init__'s snapshot of the new (lower) value would win over
    src.accel.

    Plan 9 A5 T3: delta_v2 / smooth_delta_v2 are no longer copied —
    those fields are retired from the Move / QuinticBlendMove attribute
    contract. The forward-reach cap that consumed them is now jerk-aware
    inside calc_junction (reads prev_move.{accel, j_max, move_d} directly).
    """
    dst.timing_callbacks = list(src.timing_callbacks)
    dst.next_junction_v2 = src.next_junction_v2
    dst.max_cruise_v2 = src.max_cruise_v2
    dst.accel = src.accel
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
     directly; the trailing legacy trio is the unshaped trapezoid-in-s
     phase timings (which may differ from the baked polynomial's phase
     structure when an FIR shaper extends the move duration) kept for
     debugging / instrumentation.
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
        # Plan 9 A5: j_max parity with plain Move. A downstream plain
        # Move's calc_junction reads prev_move.j_max for the jerk-aware
        # forward reachability cap; without this, flushes where a plain
        # Move follows a QuinticBlendMove would AttributeError. T3 will
        # rewrite QBM further, but the j_max parity is part of the
        # stable attribute contract shared with Move.
        self.j_max = toolhead.max_jerk
        self.timing_callbacks = []
        self.is_kinematic_move = True
        self.max_cruise_v2 = cruise_v * cruise_v
        self.max_start_v2 = v_in * v_in
        # Plan 9 A5 T3: smoothed-pass state (max_smoothed_v2,
        # smooth_delta_v2) and the trapezoidal forward cap (delta_v2)
        # are retired. QBM's attribute surface tracks Move's new
        # contract: max_cruise_v2, max_start_v2, accel, j_max, move_d,
        # next_junction_v2.
        (accel_polys, cruise_polys, decel_polys, t_accel_end, t_decel_start,
         total_t, arc_length) = shape.compose_phase_polynomials(
            v_in=v_in, v_out=v_out, cruise_v=cruise_v, a_max=a_max,
            s_accel_end=s_accel_end, s_decel_start=s_decel_start,
        )
        # Remember the unshaped polynomial so (a) CornerBlender can hand
        # it to the NEXT quintic's composer as "prev", and (b) this
        # quintic's own bake can be re-driven with neighbour info.
        self._unshaped_payload = _build_unshaped_payload(
            accel_polys, cruise_polys, decel_polys,
            t_accel_end, t_decel_start, total_t,
        )
        self._arc_length = arc_length
        self._v_in = v_in
        self._v_out = v_out
        self._cruise_v = cruise_v
        self._a_max = a_max
        self._v_cap_min_raw = v_cap_min
        self._start_pos_4d = tuple(start_pos_4d)
        self._unshaped_timings = (t_accel_end, t_decel_start, total_t)
        # Store the shaper list at construction time so finalize_shape
        # (which may be called later from CornerBlender once the next
        # neighbour is known) sees the SAME snapshot as this move was
        # planned against. Re-querying shape._limits.shapers at finalize
        # time would risk a race with SET_INPUT_SHAPER between plan and
        # emit.
        shape_limits = getattr(shape, "_limits", None)
        shaper_list = (
            getattr(shape_limits, "shapers", None)
            if shape_limits is not None else None
        ) or []
        self._shapers_snapshot = list(shaper_list)
        self._pa_dispatch_cached = _resolve_pa_dispatch(toolhead)
        # Initial bake with no neighbour info. CornerBlender will
        # re-drive finalize_shape(prev, next) before the move is
        # released downstream; the initial bake is a safety net for any
        # emit path that bypasses the deferral (tests that build a
        # QuinticBlendMove directly, flush at session end, etc.).
        self.finalize_shape(prev_unshaped=None, next_unshaped=None)

    def finalize_shape(self, prev_unshaped=None, next_unshaped=None,
                       prev_start_pos_xyz=None, next_start_pos_xyz=None):
        """(Re-)compose the shaper bake with neighbour polynomials.

        Called twice for most moves: once from __init__ with both
        neighbours None (safety-net), and again from
        CornerBlender.feed once the next-move's unshaped polynomial is
        known. The second call overwrites the quintic_trapq_payload and
        related attributes; downstream consumers (toolhead._process_moves)
        read the final payload after the CornerBlender emits the move.

        `prev_start_pos_xyz` / `next_start_pos_xyz` are the neighbour
        moves' absolute start_pos (XY only used) so this move can shift
        the neighbour polynomials into its own reference frame before
        the composer integrates across the boundary. When omitted the
        neighbour polynomial is used as-is (assumes continuous frames
        — correct for tests that hand-craft the offsets).
        """
        unshaped_t_ends, unshaped_total_t, unshaped_coeffs = self._unshaped_payload
        cur_start_xyz = (
            self._start_pos_4d[0], self._start_pos_4d[1], self._start_pos_4d[2],
        )
        if prev_unshaped is not None and prev_start_pos_xyz is not None:
            dx = prev_start_pos_xyz[0] - cur_start_xyz[0]
            dy = prev_start_pos_xyz[1] - cur_start_xyz[1]
            dz = prev_start_pos_xyz[2] - cur_start_xyz[2]
            prev_unshaped = offset_unshaped_for_neighbour(
                prev_unshaped, (dx, dy, dz)
            )
        if next_unshaped is not None and next_start_pos_xyz is not None:
            dx = next_start_pos_xyz[0] - cur_start_xyz[0]
            dy = next_start_pos_xyz[1] - cur_start_xyz[1]
            dz = next_start_pos_xyz[2] - cur_start_xyz[2]
            next_unshaped = offset_unshaped_for_neighbour(
                next_unshaped, (dx, dy, dz)
            )
        phase_t_ends_tuple, total_t_baked, coeff_tuple = bake_shaper_polynomial(
            unshaped_t_ends, unshaped_total_t, unshaped_coeffs,
            self._shapers_snapshot,
            prev_unshaped=prev_unshaped,
            next_unshaped=next_unshaped,
        )
        n_phases_baked = len(phase_t_ends_tuple)
        arc_length = self._arc_length
        axes_d = self.axes_d
        # axes_d[3] is signed E displacement; arc_length is the curve length
        # of the XY blend. extr_r = E-displacement per XY-arc-mm (signed).
        if arc_length > 0.0:
            extr_r = axes_d[3] / arc_length
        else:
            extr_r = 0.0
        axis_n = (self.axes_r[0], self.axes_r[1], self.axes_r[2])
        pa_dispatch = self._pa_dispatch_cached
        if pa_dispatch[0] == "linear":
            k_pa = pa_dispatch[1]
            coeff_tuple = tuple(_linear_pa_compose.linear_pa_compose(
                n_phases_baked, list(coeff_tuple),
                axis_n=axis_n, extr_r=extr_r, k_pa=k_pa,
            ))
        elif pa_dispatch[0] == "nonlinear":
            _, model, la, no, v_lin = pa_dispatch
            coeff_list, residual = _nonlinear_pa_compose.nonlinear_pa_compose(
                n_phases_baked, list(phase_t_ends_tuple),
                list(coeff_tuple),
                axis_n=axis_n, extr_r=extr_r,
                linear_advance=la,
                nonlinear_offset=no,
                linearization_velocity=v_lin,
                model=model,
            )
            coeff_tuple = tuple(coeff_list)
            # residual is already in filament-mm: nonlinear_pa_compose
            # returns max |truth - approx| where both sides include the
            # nonlinear_offset factor. No extra scaling needed here.
            filament_err = residual
            if filament_err > _PA_FIT_FILAMENT_WARN_MM:
                logging.warning(
                    "nonlinear_pa_compose fit error %.3g mm exceeds "
                    "%.3g mm filament budget (model=%s, NO=%.4f, "
                    "v_lin=%.2f)",
                    filament_err, _PA_FIT_FILAMENT_WARN_MM,
                    model, no, v_lin,
                )
        else:
            coeff_tuple = tuple(_linear_pa_compose.linear_pa_compose(
                n_phases_baked, list(coeff_tuple),
                axis_n=axis_n, extr_r=extr_r, k_pa=0.0,
            ))
        start_pos_xyz = (
            self._start_pos_4d[0], self._start_pos_4d[1],
            self._start_pos_4d[2],
        )
        v_cap_min = self._v_cap_min_raw
        if not (v_cap_min and math.isfinite(v_cap_min)) or v_cap_min <= 0.0:
            v_cap_min = self._cruise_v if self._cruise_v > 0.0 else self._v_in
        self.v_cap_min = v_cap_min
        t_accel_end, t_decel_start, total_t = self._unshaped_timings
        self.quintic_trapq_payload = (
            phase_t_ends_tuple, total_t_baked,
            arc_length, v_cap_min, start_pos_xyz, coeff_tuple,
            # Legacy 3-phase timings — always reflect the unshaped
            # timings; the baked polynomial spans phase_t_ends_tuple[-1].
            t_accel_end, t_decel_start, total_t,
        )
        self.min_move_t = total_t if total_t > 0.0 else (
            self.move_d / self._cruise_v if self._cruise_v > 0.0 else 0.0
        )
        # Plan 9 A5 T3: delta_v2 / smooth_delta_v2 retired — the forward
        # reachability cap is now jerk-aware (calc_junction reads
        # prev_move.{accel, j_max, move_d, max_start_v2} directly).
        self.next_junction_v2 = (
            v_cap_min * v_cap_min if v_cap_min else 0.0
        )
        self.next_junction_v_capped_to = None
        # Move-shape fields populated directly from the TOPP-baked endpoint
        # velocities / phase timings — the LookAheadQueue reverse pass skips
        # QBMs (Option-Z: blend velocities are immutable once composed), so
        # set_junction is never invoked and these fields must be populated
        # here. Consumers (extruder.move, motion_report) still read them.
        self.start_v = self._v_in
        self.cruise_v = self._cruise_v
        self.end_v = self._v_out
        self.accel_t = t_accel_end
        self.cruise_t = max(0.0, t_decel_start - t_accel_end)
        self.decel_t = max(0.0, total_t - t_decel_start)

    def limit_speed(self, speed, accel):
        # Plan 9 A5 T3: delta_v2 / smooth_delta_v2 retired — the forward
        # reachability cap is jerk-aware now (Move.calc_junction reads
        # prev_move.{accel, j_max, move_d}).
        v2 = speed * speed
        if v2 < self.max_cruise_v2:
            self.max_cruise_v2 = v2
        self.accel = min(self.accel, accel)

    def limit_next_junction_speed(self, speed):
        v2 = speed * speed
        self.next_junction_v2 = min(self.next_junction_v2, v2)
        self.next_junction_v_capped_to = speed

    def calc_junction(self, prev_move):
        # Plan 9 A5 T3: same treatment as Move.calc_junction — the
        # constant-accel forward cap (prev_move.delta_v2) is replaced
        # with jerk-aware forward reachability via
        # jerk_math.reachable_v_end. The smoothed-pass propagation
        # (max_smoothed_v2) is gone with A5.
        #
        # The blend's v_in is pointwise-safe via TOPP + v_cap_fn (D7
        # Option Z); centripetal and extruder flow caps are already
        # composed into v_cap_fn at emit time. Skip both here and just
        # run the upstream cascade so the outer lookahead can still
        # tighten max_start_v2 when the predecessor turns out to be
        # rate-limited.
        if not self.is_kinematic_move or not prev_move.is_kinematic_move:
            return
        prev_start_v = (math.sqrt(prev_move.max_start_v2)
                        if prev_move.max_start_v2 > 0.0 else 0.0)
        # prev_move is either a plain Move or another QuinticBlendMove;
        # both carry j_max (Move.__init__ snapshots it from the toolhead,
        # QuinticBlendMove.__init__ sets it explicitly as of T2 fixup).
        # All test stubs have been migrated to provide the attribute (T5).
        prev_j_max = prev_move.j_max
        prev_forward_reach = jerk_math.reachable_v_end(
            v_start=prev_start_v,
            a_max=prev_move.accel,
            j_max=prev_j_max,
            L=prev_move.move_d,
        )
        max_start_v2 = min(
            self.max_start_v2,
            self.max_cruise_v2,
            prev_move.max_cruise_v2,
            prev_move.next_junction_v2,
            prev_forward_reach * prev_forward_reach,
        )
        self.max_start_v2 = max_start_v2


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
        # One-move emit deferral so the composer can integrate the kernel
        # across move boundaries using neighbour polynomials. When a new
        # QuinticBlendMove forms, the previously-pending quintic finally
        # learns its "next" and is re-baked before being released
        # downstream. See chunk2-fix boundary-artifact report.
        self._pending_quintic = None
        # prev passed to pending's bake: (unshaped_payload, start_pos_xyz)
        # or None when there is no prior quintic neighbour.
        self._pending_prev = None
        self._pending_leading = []  # linear moves preceding pending_quintic
        self.polyline_moves_emitted = 0
        self.blends_emitted = 0

    def _finalize_pending(self, next_unshaped, next_start_pos_xyz):
        """Drain the pending-emit buffer with the pending quintic's next
        neighbour now known. Returns the list of released moves in
        emit-time order.
        """
        if self._pending_quintic is None:
            released = list(self._pending_leading)
            self._pending_leading = []
            return released
        prev_payload = None
        prev_start = None
        if self._pending_prev is not None:
            prev_payload, prev_start = self._pending_prev
        self._pending_quintic.finalize_shape(
            prev_unshaped=prev_payload,
            next_unshaped=next_unshaped,
            prev_start_pos_xyz=prev_start,
            next_start_pos_xyz=next_start_pos_xyz,
        )
        released = list(self._pending_leading) + [self._pending_quintic]
        self._pending_leading = []
        self._pending_quintic = None
        self._pending_prev = None
        return released

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
            shapers=blendmath.extract_shapers(th),
        )
        shape = blendquintic.QuinticShape.from_moves(
            self._prev, move,
            th.corner_deviation,
            limits,
        )
        if shape is None or blendmath.should_suppress_quintic(
                self._prev, move, th.corner_deviation, shape, th):
            return self._suppress_and_advance(move)
        trunc_prev, quintic_move, trunc_next_head = self._emit_blend(
            self._prev, move, shape
        )
        self._prev = trunc_next_head
        self.blends_emitted += 1
        # Post-D7: one QuinticBlendMove per blend, so the legacy
        # polyline-moves counter advances by 1.
        self.polyline_moves_emitted += 1
        # Capture the old pending's (unshaped, start_pos_xyz) before
        # finalize_pending clears it — the new quintic will use it as
        # its prev neighbour.
        old_pending_snapshot = None
        if self._pending_quintic is not None:
            old_pending_snapshot = (
                self._pending_quintic._unshaped_payload,
                (self._pending_quintic._start_pos_4d[0],
                 self._pending_quintic._start_pos_4d[1],
                 self._pending_quintic._start_pos_4d[2]),
            )
        # The pending quintic (if any) now knows its next neighbour —
        # it's the freshly-built quintic_move's unshaped polynomial.
        released = self._finalize_pending(
            next_unshaped=quintic_move._unshaped_payload,
            next_start_pos_xyz=(
                quintic_move._start_pos_4d[0],
                quintic_move._start_pos_4d[1],
                quintic_move._start_pos_4d[2],
            ),
        )
        # Emit order: previously-buffered leading + finalized pending +
        # trunc_prev (which leads the newly-pending quintic). trunc_prev
        # is a plain linear Move so it needs no shape finalization.
        released.append(trunc_prev)
        # Record the new pending quintic. Its prev neighbour is the
        # just-finalized pending quintic's unshaped polynomial.
        self._pending_quintic = quintic_move
        self._pending_prev = old_pending_snapshot
        self._pending_leading = []
        return released

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
        emitted_prev = self._prev
        self._prev = move
        # No new quintic forms here, so any pending quintic's next
        # neighbour is necessarily a linear move — which does not
        # participate in the across-boundary bake. Finalize the
        # pending quintic with next=None (zero-pad) and emit in time
        # order: [pending_quintic_finalized, emitted_prev].
        released = self._finalize_pending(
            next_unshaped=None, next_start_pos_xyz=None,
        )
        released.append(emitted_prev)
        return released

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
        # Drain the pending quintic (if any) with next=None — the print
        # stops here so the composer's zero-pad on the "next" side is
        # correct.
        released = self._finalize_pending(
            next_unshaped=None, next_start_pos_xyz=None,
        )
        if self._prev is not None:
            released.append(self._prev)
            self._prev = None
        return released

    def reset(self):
        self._prev = None
        self._pending_quintic = None
        self._pending_prev = None
        self._pending_leading = []

    def peek_buffered(self):
        buf = list(self._pending_leading)
        if self._pending_quintic is not None:
            buf.append(self._pending_quintic)
        if self._prev is not None:
            buf.append(self._prev)
        return buf
