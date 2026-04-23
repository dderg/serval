"""Python wrapper around the FIR impulse-train polynomial composer
(fir_compose.c).

Plan 8 Chunk 2: bake zv / mzv input-shaping into the planner-emitted
polynomial. Amplitudes are normalized here so the output DC gain is 1.

Neighbour-aware: optional ``prev_*`` / ``next_*`` arguments supply
adjacent moves' UNSHAPED polynomials for across-boundary kernel
integration. When omitted the corresponding side zero-pads (matches the
print actually starting / stopping at the move boundary).
"""
from __future__ import annotations

import math
from typing import List, Optional, Sequence, Tuple

from klippy.chelper import get_ffi

FIR_MAX_OUT_PHASES = 32


def fir_compose(
    input_phase_t_ends: Sequence[float],
    input_coeffs: Sequence[float],
    impulse_amplitudes: Sequence[float],
    impulse_delays: Sequence[float],
    out_capacity: int = FIR_MAX_OUT_PHASES,
    normalize: bool = True,
    prev_phase_t_ends: Optional[Sequence[float]] = None,
    prev_coeffs: Optional[Sequence[float]] = None,
    prev_T_move: float = 0.0,
    next_phase_t_ends: Optional[Sequence[float]] = None,
    next_coeffs: Optional[Sequence[float]] = None,
    next_T_move: float = 0.0,
) -> Tuple[List[float], List[float]]:
    """Bake an FIR impulse train into a piecewise-polynomial move."""
    ffi, lib = get_ffi()
    if len(impulse_amplitudes) != len(impulse_delays):
        raise ValueError("amplitudes/delays length mismatch")
    n_imp = len(impulse_amplitudes)
    amps = list(impulse_amplitudes)
    if normalize:
        total = sum(amps)
        if not math.isfinite(total) or total == 0.0:
            raise ValueError("amplitudes sum is zero/non-finite")
        amps = [a / total for a in amps]
    n_in = len(input_phase_t_ends)
    expected = n_in * 15 * 4
    if len(input_coeffs) != expected:
        raise ValueError(
            f"input_coeffs length {len(input_coeffs)} != expected {expected}"
        )
    in_t_ends_buf = ffi.new("double[]", list(input_phase_t_ends))
    in_coeffs_buf = ffi.new("double[]", list(input_coeffs))
    amps_buf = ffi.new("double[]", amps)
    delays_buf = ffi.new("double[]", list(impulse_delays))

    have_prev = (
        prev_phase_t_ends is not None
        and prev_coeffs is not None
        and prev_T_move and prev_T_move > 0.0
    )
    have_next = (
        next_phase_t_ends is not None
        and next_coeffs is not None
        and next_T_move and next_T_move > 0.0
    )
    null_double = ffi.cast("const double *", 0)
    if have_prev:
        n_prev = len(prev_phase_t_ends)
        if len(prev_coeffs) != n_prev * 15 * 4:
            raise ValueError(
                f"prev_coeffs length {len(prev_coeffs)} != "
                f"expected {n_prev * 15 * 4}"
            )
        prev_t_buf = ffi.new("double[]", list(prev_phase_t_ends))
        prev_c_buf = ffi.new("double[]", list(prev_coeffs))
    else:
        n_prev = 0
        prev_t_buf = null_double
        prev_c_buf = null_double
    if have_next:
        n_next = len(next_phase_t_ends)
        if len(next_coeffs) != n_next * 15 * 4:
            raise ValueError(
                f"next_coeffs length {len(next_coeffs)} != "
                f"expected {n_next * 15 * 4}"
            )
        next_t_buf = ffi.new("double[]", list(next_phase_t_ends))
        next_c_buf = ffi.new("double[]", list(next_coeffs))
    else:
        n_next = 0
        next_t_buf = null_double
        next_c_buf = null_double

    out_t_ends_buf = ffi.new("double[]", out_capacity)
    out_coeffs_buf = ffi.new("double[]", out_capacity * 15 * 4)
    n_out = lib.fir_compose(
        int(n_prev), prev_t_buf, prev_c_buf, float(prev_T_move or 0.0),
        n_in, in_t_ends_buf, in_coeffs_buf,
        int(n_next), next_t_buf, next_c_buf, float(next_T_move or 0.0),
        n_imp, amps_buf, delays_buf,
        out_capacity,
        out_t_ends_buf, out_coeffs_buf,
    )
    if n_out < 0:
        raise ValueError("fir_compose failed (overflow or bad args)")
    out_t_ends = [out_t_ends_buf[i] for i in range(n_out)]
    out_coeffs = [out_coeffs_buf[i] for i in range(n_out * 15 * 4)]
    return out_t_ends, out_coeffs
