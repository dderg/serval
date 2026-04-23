"""Neighbour-aware boundary-integration tests for bs_compose / fir_compose.

chunk2-fix — the composers accept optional prev / next move polynomials
so the kernel integrand crosses move boundaries for continuous prints.
Without neighbour data, a constant-velocity move at 100 mm/s with a
bs3 @ 40 Hz shaper loses ~14 mm at the final 5 ms (the kernel window
extends past move-end into what would be the next move, but zero-pad
truncates it).

This file validates:
  - Middle move of a 3-move constant-v sequence is exact at the
    boundaries.
  - End-of-session move (next=None) reproduces today's zero-padded
    behaviour (matches the print actually stopping).
  - fir mzv on an accel→cruise sequence is exact across the
    transition.
"""
from __future__ import annotations

import math

import numpy as np
import pytest

from klippy.chelper.bs_compose import bs_compose
from klippy.chelper.fir_compose import fir_compose
from klippy.extras import shaper_defs


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _pack_const_v_move(move_t, v, axis=0):
    """Single-phase constant-velocity move x(t) = v * t on chosen axis."""
    coeffs = [0.0] * (1 * 15 * 4)
    coeffs[(0 * 15 + 1) * 4 + axis] = v
    return [move_t], coeffs


def _pack_const_v_move_offset(move_t, v, x0, axis=0):
    """Single-phase constant-velocity move x(t) = x0 + v*t on chosen axis.

    Used for neighbour moves whose absolute-t frame is shifted but whose
    polynomial should describe their LOCAL motion starting at x0 at
    t_local = 0.
    """
    coeffs = [0.0] * (1 * 15 * 4)
    coeffs[(0 * 15 + 0) * 4 + axis] = x0
    coeffs[(0 * 15 + 1) * 4 + axis] = v
    return [move_t], coeffs


def _pack_accel_move(move_t, v0, a, x0=0.0, axis=0):
    """x(t) = x0 + v0*t + 0.5*a*t^2."""
    coeffs = [0.0] * (1 * 15 * 4)
    coeffs[(0 * 15 + 0) * 4 + axis] = x0
    coeffs[(0 * 15 + 1) * 4 + axis] = v0
    coeffs[(0 * 15 + 2) * 4 + axis] = 0.5 * a
    return [move_t], coeffs


def _eval_output(out_t_ends, out_coeffs, t, axis=0):
    """Evaluate the composed piecewise polynomial at time t."""
    start = 0.0
    phase_idx = len(out_t_ends) - 1
    for p, t_end in enumerate(out_t_ends):
        if t <= t_end + 1e-12:
            phase_idx = p
            break
    if phase_idx > 0:
        start = out_t_ends[phase_idx - 1]
    dt = t - start
    val = 0.0
    for k in range(14, -1, -1):
        val = val * dt + out_coeffs[(phase_idx * 15 + k) * 4 + axis]
    return val


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


def test_bs3_middle_of_constant_v_sequence_is_exact_at_boundaries():
    """bs3 @ 40 Hz on 3 identical 100 mm/s constant-v moves.

    Each move's polynomial describes its own motion in local coords;
    for CONTINUOUS motion across moves, we express the neighbour
    polynomials with the position offset (origin) already shifted into
    the current move's frame. Prev's position at u_local=0 is
    `-v*prev_T_move` (it started earlier by prev_T_move seconds and
    travels v*prev_T_move less than current's frame). Next's position
    at u_local=0 is `v*cur_T_move` (it started later, already has
    current's travel accumulated in the absolute frame).

    With the continuity offset applied, the convolution reproduces
    v*t (current-frame position) exactly at every sample — including
    within T_sm/2 of both boundaries where the pre-fix zero-pad
    produced ~mm-scale errors.
    """
    shaper_freq = 40.0
    m = 3
    v = 100.0  # mm/s
    move_t = 0.1  # 10 cm of travel per move

    # Prev's polynomial in current's frame: starts at absolute-u =
    # -prev_T_move at position -v*prev_T_move, reaches position 0 at
    # absolute-u = 0. In prev's LOCAL frame that is:
    #   x_prev(u_local) = v*u_local - v*prev_T_move
    prev_t_ends, prev_coeffs = _pack_const_v_move_offset(
        move_t, v, x0=-v * move_t
    )
    cur_t_ends, cur_coeffs = _pack_const_v_move(move_t, v)
    # Next's polynomial in current's frame: starts at absolute-u =
    # +move_t at position v*move_t, in its LOCAL frame that is:
    #   x_next(u_local) = v*u_local + v*move_t
    next_t_ends, next_coeffs = _pack_const_v_move_offset(
        move_t, v, x0=v * move_t
    )

    out_t_ends, out_coeffs = bs_compose(
        cur_t_ends, cur_coeffs,
        bs_order=m, shaper_freq=shaper_freq,
        prev_phase_t_ends=prev_t_ends,
        prev_coeffs=prev_coeffs,
        prev_T_move=move_t,
        next_phase_t_ends=next_t_ends,
        next_coeffs=next_coeffs,
        next_T_move=move_t,
    )
    t_sm = shaper_defs._F_M_TABLE[m] / shaper_freq
    # Boundary-tight samples where pre-fix zero-pad showed ~mm errors.
    for t in [1e-4, 1e-3, 5e-3,
              t_sm / 4, t_sm / 2,
              move_t - t_sm / 2, move_t - 5e-3,
              move_t - 1e-3, move_t - 1e-4]:
        y = _eval_output(out_t_ends, out_coeffs, t, axis=0)
        expected = v * t
        assert abs(y - expected) < 1e-6, (
            f"bs3 middle-move at t={t:.6f}: got {y:.9f}, "
            f"expected {expected:.9f}, err={y - expected:.3e}"
        )


def test_bs3_no_next_neighbour_zero_pads_at_end():
    """bs3 on a single 100 mm/s constant-v move with NULL next.

    This matches today's (pre-fix) behaviour and is CORRECT when the
    print actually stops at the move boundary. Within T_sm/2 of
    move-end the output deviates from v*t because the kernel tail
    integrates over u > move_t where position is (correctly) zero.
    """
    shaper_freq = 40.0
    m = 3
    v = 100.0
    move_t = 0.1
    t_sm = shaper_defs._F_M_TABLE[m] / shaper_freq
    cur_t_ends, cur_coeffs = _pack_const_v_move(move_t, v)

    # With no neighbours (same as single-move behaviour).
    out_t_ends, out_coeffs = bs_compose(
        cur_t_ends, cur_coeffs,
        bs_order=m, shaper_freq=shaper_freq,
    )
    # Interior (past t_sm/2 from either boundary) still matches v*t.
    t_interior = 0.5 * move_t
    y_int = _eval_output(out_t_ends, out_coeffs, t_interior, axis=0)
    assert abs(y_int - v * t_interior) < 1e-6
    # Within t_sm/2 of move-end, the result deviates because the kernel
    # tail truncates. This confirms that when next=None (session end)
    # the zero-pad does produce the expected boundary artifact.
    t_near_end = move_t - 1e-3
    y_end = _eval_output(out_t_ends, out_coeffs, t_near_end, axis=0)
    # The error should be O(v * t_sm) — concretely a large deviation.
    # We assert the deviation is NON-negligible to confirm that the
    # "no neighbour" code path still zero-pads.
    err = abs(y_end - v * t_near_end)
    assert err > 1e-3, (
        f"bs3 no-next at t={t_near_end}: expected non-trivial deviation "
        f"(>1e-3 mm), got err={err:.3e}"
    )


def test_bs3_with_next_only_matches_at_end():
    """Session start emulation: no prev, but a next neighbour. Then
    the end-of-move region should match v*t (the kernel tail into the
    next move is integrated correctly); the start still zero-pads.
    Next polynomial is offset by +v*move_t so motion is continuous in
    current's frame.
    """
    shaper_freq = 40.0
    m = 3
    v = 100.0
    move_t = 0.1
    cur_t_ends, cur_coeffs = _pack_const_v_move(move_t, v)
    next_t_ends, next_coeffs = _pack_const_v_move_offset(
        move_t, v, x0=v * move_t
    )

    out_t_ends, out_coeffs = bs_compose(
        cur_t_ends, cur_coeffs,
        bs_order=m, shaper_freq=shaper_freq,
        next_phase_t_ends=next_t_ends,
        next_coeffs=next_coeffs,
        next_T_move=move_t,
    )
    t_near_end = move_t - 1e-3
    y_end = _eval_output(out_t_ends, out_coeffs, t_near_end, axis=0)
    assert abs(y_end - v * t_near_end) < 1e-6, (
        f"bs3 with next-only at t={t_near_end}: expected exact, "
        f"got err={y_end - v * t_near_end:.3e}"
    )


def test_mzv_cruise_across_boundary_is_exact():
    """mzv shaper on two adjacent cruise moves with the prev / next
    polynomials offset so motion is continuous in current's frame.

    Expected output: v * (t - t_c) with t_c the mzv centroid delay —
    same as the single-move interior test, now exact at boundaries
    where the pre-fix zero-pad produced large deviations.
    """
    shaper_freq = 40.0
    damping_ratio = 0.1
    A, T = shaper_defs.get_mzv_shaper(shaper_freq, damping_ratio)
    t_c = shaper_defs.get_shaper_offset(A, T)
    max_tau = T[-1]
    move_t = 0.2
    v = 100.0

    # Prev offset: its polynomial in current's frame gives v*u at
    # u in [-move_t, 0], so in prev-local coords x(u_local) = v*u_local
    # - v*move_t (starts at -v*move_t, ends at 0).
    prev_t_ends, prev_coeffs = _pack_const_v_move_offset(
        move_t, v, x0=-v * move_t
    )
    cur_t_ends, cur_coeffs = _pack_const_v_move(move_t, v)
    # Next offset: v*u_local + v*move_t so at u = move_t+ it starts at
    # v*move_t (matching current's end).
    next_t_ends, next_coeffs = _pack_const_v_move_offset(
        move_t, v, x0=v * move_t
    )

    out_t_ends, out_coeffs = fir_compose(
        cur_t_ends, cur_coeffs,
        impulse_amplitudes=A, impulse_delays=T,
        prev_phase_t_ends=prev_t_ends,
        prev_coeffs=prev_coeffs,
        prev_T_move=move_t,
        next_phase_t_ends=next_t_ends,
        next_coeffs=next_coeffs,
        next_T_move=move_t,
    )
    # With continuous x(u) = v*u across boundaries, the FIR sum is
    # sum_i a_i * v * (t - tau_i) = v*(t - t_c). Sample at boundary-
    # tight points (within max_tau of both t=0 and t=move_t).
    for t in list(np.linspace(1e-3, max_tau + 1e-3, 5)) + \
             list(np.linspace(move_t - max_tau - 1e-3, move_t - 1e-3, 5)):
        y = _eval_output(out_t_ends, out_coeffs, float(t), axis=0)
        expected = v * (t - t_c)
        assert abs(y - expected) < 1e-6, (
            f"mzv cruise boundary at t={t:.5f}: got {y}, "
            f"expected {expected}, err={y - expected:.3e}"
        )


def test_mzv_accel_then_cruise_boundary_exact():
    """Accel move followed by cruise move with mzv shaper. The prev
    (accel) polynomial is offset so motion is continuous in current's
    (cruise) frame. Expected: the impulse-delayed sum integrates the
    continuous stream x(u) = (accel shape for u<0) joined to (v_end * u
    for u >= 0) at u=0 with value 0.
    """
    shaper_freq = 40.0
    damping_ratio = 0.1
    A, T = shaper_defs.get_mzv_shaper(shaper_freq, damping_ratio)
    total = sum(A) or 1.0
    A_n = [a / total for a in A]
    max_tau = T[-1]

    accel_t = 0.05
    cruise_t = 0.15
    v0 = 50.0
    a = 800.0

    v_end = v0 + a * accel_t
    x_end_accel = v0 * accel_t + 0.5 * a * accel_t * accel_t

    # Prev (accel) polynomial in current's frame: shift so its value
    # at u = 0 (local = accel_t) is 0. Raw accel polynomial at local =
    # accel_t is x_end_accel; we subtract x_end_accel so the offset
    # polynomial ends at 0.
    prev_t_ends, prev_coeffs = _pack_accel_move(
        accel_t, v0, a, x0=-x_end_accel
    )
    cur_t_ends, cur_coeffs = _pack_const_v_move(cruise_t, v_end)

    out_t_ends, out_coeffs = fir_compose(
        cur_t_ends, cur_coeffs,
        impulse_amplitudes=A, impulse_delays=T,
        prev_phase_t_ends=prev_t_ends,
        prev_coeffs=prev_coeffs,
        prev_T_move=accel_t,
    )

    for t in np.linspace(0.0, max_tau, 5):
        if t < 1e-6:
            t = 1e-6
        y = _eval_output(out_t_ends, out_coeffs, float(t), axis=0)
        expected = 0.0
        for i, (a_i, tau_i) in enumerate(zip(A_n, T)):
            u = t - tau_i
            if 0.0 <= u <= cruise_t:
                expected += a_i * (v_end * u)
            elif -accel_t <= u < 0.0:
                # Offset-accel polynomial: local = u + accel_t,
                # value = v0*local + 0.5*a*local^2 - x_end_accel.
                u_local = u + accel_t
                expected += a_i * (
                    v0 * u_local + 0.5 * a * u_local * u_local
                    - x_end_accel
                )
        assert abs(y - expected) < 1e-8, (
            f"mzv accel→cruise at t={t:.5f}: got {y}, "
            f"expected {expected}, err={y - expected:.3e}"
        )


def test_bs3_single_move_path_still_works():
    """Regression: with no prev/next supplied the composer produces
    the same output as the original single-move path."""
    shaper_freq = 40.0
    m = 3
    v = 100.0
    move_t = 0.5
    t_ends, coeffs = _pack_const_v_move(move_t, v)
    out_t_ends, out_coeffs = bs_compose(
        t_ends, coeffs, bs_order=m, shaper_freq=shaper_freq,
    )
    # Interior should match v*t.
    t_sm = shaper_defs._F_M_TABLE[m] / shaper_freq
    for t in np.linspace(t_sm, move_t - t_sm, 5):
        y = _eval_output(out_t_ends, out_coeffs, float(t), axis=0)
        assert abs(y - v * t) < 1e-6


def test_mzv_single_move_path_still_works():
    """Regression: fir_compose with no prev/next yields the original
    output shape."""
    shaper_freq = 40.0
    damping_ratio = 0.1
    A, T = shaper_defs.get_mzv_shaper(shaper_freq, damping_ratio)
    t_c = shaper_defs.get_shaper_offset(A, T)
    max_tau = T[-1]
    move_t = 0.5
    v = 100.0
    t_ends, coeffs = _pack_const_v_move(move_t, v)
    out_t_ends, out_coeffs = fir_compose(
        t_ends, coeffs, A, T,
    )
    for t in np.linspace(max_tau + 1e-4, move_t - 1e-4, 5):
        y = _eval_output(out_t_ends, out_coeffs, float(t), axis=0)
        expected = v * (t - t_c)
        assert abs(y - expected) < 1e-6
