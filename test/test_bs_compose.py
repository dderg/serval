"""Tests for the bs-kernel polynomial composer (klippy/chelper/bs_compose.c).

Plan 8 Chunk 2 — validates that the analytical composer produces the same
shape as a numerical reference convolution.
"""
from __future__ import annotations

import math

import numpy as np
import pytest

from klippy.chelper.bs_compose import bs_compose
from klippy.extras import shaper_defs


# ----- helpers ------------------------------------------------------------


def _pack_constant_velocity(move_t, v, axis=0):
    """Return (phase_t_ends, coeff_buf) for a single-phase constant-velocity
    move with position x(t) = v * t on the chosen axis (0=x, 1=y, 2=z)."""
    n_in = 1
    coeffs = [0.0] * (n_in * 15 * 4)
    # phase 0, c[0].axis = 0, c[1].axis = v.
    coeffs[(0 * 15 + 1) * 4 + axis] = v
    return [move_t], coeffs


def _pack_zero(move_t, n_phases=3):
    """Return (phase_t_ends, coeff_buf) for a zero-motion move with the
    given number of phases (evenly spaced)."""
    t_ends = [move_t * (p + 1) / n_phases for p in range(n_phases)]
    coeffs = [0.0] * (n_phases * 15 * 4)
    return t_ends, coeffs


def _pack_accel_ramp(move_t, v0, a, axis=0):
    """Constant-acceleration ramp: x(t) = v0*t + 0.5*a*t^2 on one axis."""
    coeffs = [0.0] * (1 * 15 * 4)
    coeffs[(0 * 15 + 1) * 4 + axis] = v0
    coeffs[(0 * 15 + 2) * 4 + axis] = 0.5 * a
    return [move_t], coeffs


def _eval_output(out_t_ends, out_coeffs, t, axis=0):
    """Evaluate the composed piecewise polynomial at time t (absolute move-
    local). Returns position on the selected axis."""
    # Identify phase.
    start = 0.0
    phase_idx = None
    for p, t_end in enumerate(out_t_ends):
        if t <= t_end + 1e-12:
            phase_idx = p
            break
        start = t_end
    if phase_idx is None:
        phase_idx = len(out_t_ends) - 1
        start = out_t_ends[-2] if len(out_t_ends) >= 2 else 0.0
    # Phase-local time.
    dt = t - start
    # Horner.
    val = 0.0
    for k in range(14, -1, -1):
        val = val * dt + out_coeffs[(phase_idx * 15 + k) * 4 + axis]
    return val


def _numerical_conv(x_fn, w_fn, t, t_sm, n_samples=4001):
    """Numerical reference: (x * w)(t) via trapezoidal integration over
    tau in [-t_sm/2, t_sm/2]. x_fn is a callable returning the input at
    any real u (zero outside the move); w_fn evaluates the kernel at tau."""
    tau = np.linspace(-0.5 * t_sm, 0.5 * t_sm, n_samples)
    dtau = tau[1] - tau[0]
    vals = np.array([x_fn(t - ti) * w_fn(ti) for ti in tau])
    # np.trapz was removed in NumPy 2.0; fall back to np.trapezoid.
    trap = getattr(np, "trapezoid", None) or np.trapz
    return trap(vals, dx=dtau)


def _bs_kernel_fn(m, t_sm):
    """Return a Python callable evaluating the bs_m kernel at tau."""
    pieces, _ = shaper_defs._get_bs_smoother(m, 1.0 / t_sm * shaper_defs._F_M_TABLE[m],
                                             None, True)
    def w(tau):
        for (a, b, coeffs) in pieces:
            if a - 1e-15 <= tau <= b + 1e-15:
                r = 0.0
                for c in reversed(coeffs):
                    r = r * tau + c
                return r
        return 0.0
    return w


# ----- tests --------------------------------------------------------------


def test_bs1_constant_velocity_centroid_delay():
    """bs1 on constant-velocity: in the interior of the move (past the
    first T_sm/2 and before the last T_sm/2), the output must match the
    input (a constant-velocity motion passes through any zero-mean
    smoothing kernel unchanged). Near boundaries the zero-pad truncates
    the kernel and output deviates — those points are not checked."""
    shaper_freq = 40.0  # Hz
    t_sm = shaper_defs._F_M_TABLE[1] / shaper_freq
    move_t = 0.5  # well longer than t_sm
    v = 100.0  # mm/s
    t_ends, coeffs = _pack_constant_velocity(move_t, v, axis=0)
    out_t_ends, out_coeffs = bs_compose(
        t_ends, coeffs, bs_order=1, shaper_freq=shaper_freq,
    )
    assert len(out_t_ends) >= 2
    # Interior sample: 0.25 s (well past t_sm/2 ~ 0.02 s).
    for t in [0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40]:
        y = _eval_output(out_t_ends, out_coeffs, t, axis=0)
        expected = v * t
        # bs kernel is zero-mean → constant-velocity passes through exactly.
        assert abs(y - expected) < 1e-6, (
            f"bs1(const-v) at t={t}: got {y}, expected {expected}"
        )


def test_bs3_accel_ramp_matches_numerical_convolution():
    """bs3 on an accel ramp: sampled positions in the interior match a
    numerical reference convolution to sub-micron precision."""
    shaper_freq = 40.0
    m = 3
    t_sm = shaper_defs._F_M_TABLE[m] / shaper_freq
    move_t = 0.4
    v0 = 50.0
    a = 800.0
    t_ends, coeffs = _pack_accel_ramp(move_t, v0, a, axis=0)
    out_t_ends, out_coeffs = bs_compose(
        t_ends, coeffs, bs_order=m, shaper_freq=shaper_freq,
    )
    w = _bs_kernel_fn(m, t_sm)

    def x_fn(u):
        if u < 0.0 or u > move_t:
            return 0.0
        return v0 * u + 0.5 * a * u * u

    # Sample deep in the interior where boundary truncation is zero.
    # Interior band: [t_sm/2, move_t - t_sm/2].
    lo = 0.5 * t_sm + 1e-3
    hi = move_t - 0.5 * t_sm - 1e-3
    for frac in np.linspace(0.1, 0.9, 9):
        t = lo + frac * (hi - lo)
        got = _eval_output(out_t_ends, out_coeffs, t, axis=0)
        ref = _numerical_conv(x_fn, w, t, t_sm, n_samples=4001)
        assert abs(got - ref) < 1e-5, (
            f"bs{m}(ramp) at t={t}: got={got}, ref={ref}, err={got - ref}"
        )


def test_zero_input_zero_output():
    """Zero input gives zero output on every axis across every phase."""
    shaper_freq = 40.0
    t_ends, coeffs = _pack_zero(0.3, n_phases=3)
    for m in (1, 2, 3, 4, 5):
        out_t_ends, out_coeffs = bs_compose(
            t_ends, coeffs, bs_order=m, shaper_freq=shaper_freq,
        )
        assert all(abs(c) < 1e-15 for c in out_coeffs), (
            f"bs{m} zero-input produced non-zero output"
        )


def test_bs5_piece_count_within_budget():
    """bs5 on a 3-phase input with all phases non-degenerate produces at
    most 28 output pieces (matches research-doc bound). 32-slot budget."""
    shaper_freq = 40.0  # chosen so t_sm is smaller than any phase duration
    move_t = 0.6
    # 3 non-degenerate phases: accel (0..0.2), cruise (0.2..0.4), decel
    # (0.4..0.6), each with some distinct polynomial content.
    t_ends = [0.2, 0.4, 0.6]
    coeffs = [0.0] * (3 * 15 * 4)
    # accel: x(t) = v0*t + 0.5*a*t^2 in phase-local (origin = 0).
    coeffs[(0 * 15 + 1) * 4 + 0] = 50.0
    coeffs[(0 * 15 + 2) * 4 + 0] = 400.0
    # cruise: phase-local origin 0.2; at t_local=0 position = 50*0.2 +
    # 0.5*800*0.04 = 10 + 16 = 26. x(t_local) = 26 + 130 * t_local.
    coeffs[(1 * 15 + 0) * 4 + 0] = 26.0
    coeffs[(1 * 15 + 1) * 4 + 0] = 130.0
    # decel: phase-local origin 0.4; x(t_local) = 52 + 130*t_local - 0.5*400*t_local^2.
    coeffs[(2 * 15 + 0) * 4 + 0] = 52.0
    coeffs[(2 * 15 + 1) * 4 + 0] = 130.0
    coeffs[(2 * 15 + 2) * 4 + 0] = -200.0
    out_t_ends, out_coeffs = bs_compose(
        t_ends, coeffs, bs_order=5, shaper_freq=shaper_freq,
    )
    # Research bound: 4 phase edges × 7 kernel edges = 28, minus
    # duplicates. The emitter caps at 28.
    assert 1 <= len(out_t_ends) <= 28, (
        f"bs5 produced {len(out_t_ends)} pieces, expected ≤ 28"
    )
    # Last t_end equals move_t.
    assert abs(out_t_ends[-1] - move_t) < 1e-12
