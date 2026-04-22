# klippy/blendplanner.py
# Copyright (C) 2026
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# Corner-blending planner integration.
# See docs/superpowers/specs/2026-04-17-planner-integration-design.md
from __future__ import annotations

import math

from . import blendmath, blendquintic, blendshape, topp


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
    (t_accel_end, t_decel_start, total_t, arc_length, v_cap_min,
     start_pos_xyz, coeff_buf) ready to feed trapq_append_quintic.
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
        # Pack coeff_buf for trapq_append_quintic (99 doubles).
        coeff_buf = []
        for phase in (accel_polys, cruise_polys, decel_polys):
            for k in range(11):
                coeff_buf.append(phase[0][k])
                coeff_buf.append(phase[1][k])
                coeff_buf.append(phase[2][k])
        start_pos_xyz = (start_pos_4d[0], start_pos_4d[1], start_pos_4d[2])
        if not (v_cap_min and math.isfinite(v_cap_min)) or v_cap_min <= 0.0:
            v_cap_min = cruise_v if cruise_v > 0.0 else v_in
        self.v_cap_min = v_cap_min
        self.quintic_trapq_payload = (
            t_accel_end, t_decel_start, total_t, arc_length, v_cap_min,
            start_pos_xyz, tuple(coeff_buf),
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
