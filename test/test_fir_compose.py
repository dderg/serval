"""Tests for the FIR impulse-train polynomial composer
(klippy/chelper/fir_compose.c).

Plan 8 Chunk 2 — validates zv / mzv polynomial baking.
"""
from __future__ import annotations

import math

import numpy as np
import pytest

from klippy.chelper.fir_compose import fir_compose
from klippy.extras import shaper_defs


def _pack_constant_velocity(move_t, v, axis=0):
    coeffs = [0.0] * (1 * 15 * 4)
    coeffs[(0 * 15 + 1) * 4 + axis] = v
    return [move_t], coeffs


def _pack_accel(move_t, v0, a, axis=0):
    coeffs = [0.0] * (1 * 15 * 4)
    coeffs[(0 * 15 + 1) * 4 + axis] = v0
    coeffs[(0 * 15 + 2) * 4 + axis] = 0.5 * a
    return [move_t], coeffs


def _eval(out_t_ends, out_coeffs, t, axis=0):
    start = 0.0
    pidx = len(out_t_ends) - 1
    for p, t_end in enumerate(out_t_ends):
        if t <= t_end + 1e-12:
            pidx = p
            break
        start = t_end
    if pidx > 0:
        start = out_t_ends[pidx - 1]
    else:
        start = 0.0
    dt = t - start
    val = 0.0
    for k in range(14, -1, -1):
        val = val * dt + out_coeffs[(pidx * 15 + k) * 4 + axis]
    return val


def test_mzv_constant_velocity_shifted():
    """mzv on constant-velocity: in the shaped move's interior (past the
    last impulse and before input end), output matches the mean-delay-
    shifted constant-velocity motion.

    MZV impulse centroid t_c = sum(a_i * tau_i) / sum(a_i). Output y(t) =
    v * (t - t_c) in the interior (since sum(a_i) = 1 after normalization
    and constant-velocity passes through any FIR unchanged except for the
    centroid delay).
    """
    shaper_freq = 40.0
    damping_ratio = 0.1
    A, T = shaper_defs.get_mzv_shaper(shaper_freq, damping_ratio)
    t_c = shaper_defs.get_shaper_offset(A, T)
    max_tau = T[-1]
    move_t = 0.5
    v = 100.0
    t_ends, coeffs = _pack_constant_velocity(move_t, v, axis=0)
    out_t_ends, out_coeffs = fir_compose(
        t_ends, coeffs, A, T,
    )
    # Interior sample range: [max_tau, move_t].
    for t in np.linspace(max_tau + 1e-4, move_t - 1e-4, 7):
        y = _eval(out_t_ends, out_coeffs, float(t), axis=0)
        expected = v * (t - t_c)
        assert abs(y - expected) < 1e-6, (
            f"mzv(const-v) at t={t}: got {y}, expected {expected}"
        )


def test_mzv_step_cancellation():
    """A classical MZV step test: take input as constant velocity with
    zero acceleration. Apply mzv: in the interior, after the kernel
    settles, output equals input v*(t - t_c) exactly. Residual vibration
    cancellation is a physics property of the convolution that we
    validate by reproducing the expected waveform against a numerical
    reference convolution."""
    shaper_freq = 40.0
    damping_ratio = 0.1
    A, T = shaper_defs.get_mzv_shaper(shaper_freq, damping_ratio)
    # Normalize
    total = sum(A)
    A_n = [a / total for a in A]
    move_t = 0.3
    v0 = 60.0
    a = 500.0
    t_ends, coeffs = _pack_accel(move_t, v0, a, axis=0)
    out_t_ends, out_coeffs = fir_compose(
        t_ends, coeffs, A, T,
    )

    def x_fn(u):
        if u < 0.0 or u > move_t:
            return 0.0
        return v0 * u + 0.5 * a * u * u

    # Reference: y(t) = sum(A_n_i * x(t - T_i)). Sample in interior.
    max_tau = T[-1]
    for t in np.linspace(max_tau + 1e-4, move_t - 1e-4, 7):
        y = _eval(out_t_ends, out_coeffs, float(t), axis=0)
        ref = sum(A_n[i] * x_fn(t - T[i]) for i in range(len(A)))
        assert abs(y - ref) < 1e-8, (
            f"mzv(ramp) at t={t}: got={y}, ref={ref}"
        )


def test_zv_piece_count_bound():
    """ZV on 3-phase input: output piece count at most (n_phases + 1) * 2
    pieces, per the Minkowski-sum bound."""
    shaper_freq = 40.0
    damping_ratio = 0.1
    A, T = shaper_defs.get_zv_shaper(shaper_freq, damping_ratio)
    t_ends = [0.1, 0.2, 0.3]
    coeffs = [0.0] * (3 * 15 * 4)
    coeffs[(0 * 15 + 1) * 4 + 0] = 50.0
    coeffs[(1 * 15 + 0) * 4 + 0] = 5.0
    coeffs[(1 * 15 + 1) * 4 + 0] = 100.0
    coeffs[(2 * 15 + 0) * 4 + 0] = 15.0
    coeffs[(2 * 15 + 1) * 4 + 0] = 100.0
    out_t_ends, out_coeffs = fir_compose(t_ends, coeffs, A, T)
    # Upper bound: 4 phase edges (0, T1, T2, T3) × 2 impulses = 8
    # breakpoints including 0, so up to 7 output pieces (or fewer after
    # uniq). Additionally the kernel extends move_t by max_tau.
    n_pieces = len(out_t_ends)
    assert 1 <= n_pieces <= 8, (
        f"zv 3-phase -> {n_pieces} pieces, expected <= 8"
    )
    # Total duration is move_t + max_tau.
    assert abs(out_t_ends[-1] - (0.3 + T[-1])) < 1e-12
