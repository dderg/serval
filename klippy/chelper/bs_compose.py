"""Python wrapper around the bs-kernel polynomial composer (bs_compose.c).

Plan 8 Chunk 2: convolve a move's per-phase quintic-in-t polynomials with a
cardinal B-spline kernel bs_m (m in 1..5) at plan time, producing a
piecewise polynomial in the same 15-coefficient slot layout that
trapq_append_quintic expects.

Neighbour-aware: optional ``prev_*`` / ``next_*`` arguments let the
composer integrate the kernel across move boundaries using the UNSHAPED
polynomial of the adjacent moves. When omitted (None) the corresponding
side zero-pads, which matches reality only when the print actually starts
/ stops at the move boundary.
"""
from __future__ import annotations

from typing import List, Optional, Sequence, Tuple

from klippy.chelper import get_ffi

# Maximum output phases. Must stay in sync with trapq.h MOVE_MAX_PIECES.
BS_MAX_OUT_PHASES = 32


def bs_compose(
    input_phase_t_ends: Sequence[float],
    input_coeffs: Sequence[float],
    bs_order: int,
    shaper_freq: float,
    damping_ratio: float = 0.0,
    out_capacity: int = BS_MAX_OUT_PHASES,
    prev_phase_t_ends: Optional[Sequence[float]] = None,
    prev_coeffs: Optional[Sequence[float]] = None,
    prev_T_move: float = 0.0,
    next_phase_t_ends: Optional[Sequence[float]] = None,
    next_coeffs: Optional[Sequence[float]] = None,
    next_T_move: float = 0.0,
) -> Tuple[List[float], List[float]]:
    """Compose the bs-kernel convolution over a quintic piecewise polynomial.

    Parameters
    ----------
    input_phase_t_ends : sequence[float]
        Absolute move-local end time of each input phase. Length = n_in.
    input_coeffs : sequence[float]
        n_in * 15 * 4 doubles, per phase (Plan 8 Chunk 3):
            c[0].x, c[0].y, c[0].z, c[0].e, c[1].x, ... c[14].e
        The .e slot is ignored on input and zeroed on output.
    bs_order : int
        Cardinal B-spline order 1..5.
    shaper_freq : float
        Shaper frequency in Hz; > 0.
    damping_ratio : float, optional
        Accepted for signature parity. Ignored (bs kernel is damping-
        independent).
    out_capacity : int, optional
        Output phase capacity. Defaults to BS_MAX_OUT_PHASES.
    prev_phase_t_ends, prev_coeffs, prev_T_move : optional
        Previous move's UNSHAPED phase polynomial. If omitted, outside
        u < 0 zero-pads (correct at print start).
    next_phase_t_ends, next_coeffs, next_T_move : optional
        Next move's UNSHAPED phase polynomial. If omitted, outside
        u > move_t zero-pads (correct at print end).

    Returns
    -------
    (out_phase_t_ends, out_coeffs)
    """
    ffi, lib = get_ffi()
    n_in = len(input_phase_t_ends)
    if n_in <= 0:
        raise ValueError("empty input phases")
    expected = n_in * 15 * 4
    if len(input_coeffs) != expected:
        raise ValueError(
            f"input_coeffs length {len(input_coeffs)} != expected {expected}"
        )
    in_t_ends_buf = ffi.new("double[]", list(input_phase_t_ends))
    in_coeffs_buf = ffi.new("double[]", list(input_coeffs))

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
    n_out = lib.bs_compose(
        int(n_prev), prev_t_buf, prev_c_buf, float(prev_T_move or 0.0),
        n_in, in_t_ends_buf, in_coeffs_buf,
        int(n_next), next_t_buf, next_c_buf, float(next_T_move or 0.0),
        int(bs_order), float(shaper_freq), float(damping_ratio),
        out_capacity,
        out_t_ends_buf, out_coeffs_buf,
    )
    if n_out < 0:
        raise ValueError("bs_compose failed (overflow or bad args)")
    out_t_ends = [out_t_ends_buf[i] for i in range(n_out)]
    out_coeffs = [out_coeffs_buf[i] for i in range(n_out * 15 * 4)]
    return out_t_ends, out_coeffs
