"""Python wrapper around the generic piecewise-polynomial composer
(smooth_compose.c).

Restores the pre-Plan-5 smooth-IS kernel family (smooth_zv, smooth_mzv,
smooth_ei, smooth_2hump_ei, smooth_zvd_ei, smooth_si) alongside the
cardinal B-spline chain (bs1..bs5). The two kernel families share the
baked-planner path; they differ only in kernel shape:

  - bs_compose builds a degree-m cardinal B-spline chain internally
    from a single integer ``bs_order``.
  - smooth_compose takes the kernel as an externally-supplied piecewise
    polynomial over [-t_sm/2, +t_sm/2]. The caller is responsible for
    computing the piece coefficients (typically via
    ``klippy/extras/shaper_defs.py::INPUT_SMOOTHERS[...].init_func``).

Neighbour-aware in the same way as bs_compose: optional ``prev_*`` /
``next_*`` arguments let the composer integrate the kernel across move
boundaries using the UNSHAPED polynomial of the adjacent moves. Omitted
sides zero-pad.
"""
from __future__ import annotations

from typing import List, Optional, Sequence, Tuple

from klippy.chelper import get_ffi

# Maximum output phases. Must stay in sync with trapq.h MOVE_MAX_PIECES.
SMOOTH_MAX_OUT_PHASES = 32

# Kernel-piece buffer geometry. Must match SMOOTH_KERNEL_MAX_NC /
# SMOOTH_KERNEL_MAX_PIECES in klippy/chelper/smooth_compose.h.
SMOOTH_KERNEL_MAX_NC = 9
SMOOTH_KERNEL_MAX_PIECES = 16


def _pack_kernel_pieces(
    piece_coeffs_list: Sequence[Tuple[float, float, Sequence[float]]],
) -> Tuple[List[float], List[float], List[float]]:
    """Convert the Python piece tuples (t_start, t_end, coeffs_ascending)
    into the three flat arrays the C ABI consumes:
      (piece_starts, piece_ends, coeff_rows_flat).

    coeff_rows_flat is laid out as ``n_pieces * SMOOTH_KERNEL_MAX_NC``
    ascending-power coefficients per piece, zero-padded on the right so
    every piece occupies exactly SMOOTH_KERNEL_MAX_NC slots.
    """
    n = len(piece_coeffs_list)
    if n <= 0:
        raise ValueError("empty kernel_pieces")
    if n > SMOOTH_KERNEL_MAX_PIECES:
        raise ValueError(
            f"kernel_pieces count {n} exceeds SMOOTH_KERNEL_MAX_PIECES="
            f"{SMOOTH_KERNEL_MAX_PIECES}"
        )
    starts = [0.0] * n
    ends = [0.0] * n
    flat_coeffs = [0.0] * (n * SMOOTH_KERNEL_MAX_NC)
    for i, (t_start, t_end, coeffs) in enumerate(piece_coeffs_list):
        starts[i] = float(t_start)
        ends[i] = float(t_end)
        if len(coeffs) > SMOOTH_KERNEL_MAX_NC:
            raise ValueError(
                f"kernel piece {i} has {len(coeffs)} coefficients, exceeds "
                f"SMOOTH_KERNEL_MAX_NC={SMOOTH_KERNEL_MAX_NC}"
            )
        for k, c in enumerate(coeffs):
            flat_coeffs[i * SMOOTH_KERNEL_MAX_NC + k] = float(c)
    return starts, ends, flat_coeffs


def smooth_compose(
    input_phase_t_ends: Sequence[float],
    input_coeffs: Sequence[float],
    kernel_pieces: Sequence[Tuple[float, float, Sequence[float]]],
    t_sm: float,
    out_capacity: int = SMOOTH_MAX_OUT_PHASES,
    prev_phase_t_ends: Optional[Sequence[float]] = None,
    prev_coeffs: Optional[Sequence[float]] = None,
    prev_T_move: float = 0.0,
    next_phase_t_ends: Optional[Sequence[float]] = None,
    next_coeffs: Optional[Sequence[float]] = None,
    next_T_move: float = 0.0,
) -> Tuple[List[float], List[float]]:
    """Compose a kernel convolution over a quintic piecewise polynomial.

    Parameters
    ----------
    input_phase_t_ends : sequence[float]
        Absolute move-local end time of each input phase. Length = n_in.
    input_coeffs : sequence[float]
        n_in * 15 * 4 doubles, per phase, interleaved-axis
        (c[0].x, c[0].y, c[0].z, c[0].e, c[1].x, ...). The .e slot is
        ignored on input and zeroed on output.
    kernel_pieces : sequence[(t_start, t_end, coeffs_ascending)]
        Piecewise polynomial kernel over [-t_sm/2, +t_sm/2]. Pieces
        must be contiguous, sorted, and cover the declared window.
        coeffs_ascending is ASCENDING power-basis; per-piece degree
        up to SMOOTH_KERNEL_MAX_NC - 1 = 8.
    t_sm : float
        Total kernel support window (seconds). Must match the span of
        the supplied pieces.
    out_capacity : int, optional
        Output phase capacity. Defaults to SMOOTH_MAX_OUT_PHASES.
    prev_phase_t_ends, prev_coeffs, prev_T_move : optional
        Previous move's UNSHAPED phase polynomial for across-boundary
        kernel integration. If omitted, outside u < 0 zero-pads
        (correct at print start).
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
    if t_sm <= 0.0:
        raise ValueError("t_sm must be positive")

    piece_starts, piece_ends, piece_coeffs_flat = _pack_kernel_pieces(
        kernel_pieces
    )
    n_pieces = len(piece_starts)

    in_t_ends_buf = ffi.new("double[]", list(input_phase_t_ends))
    in_coeffs_buf = ffi.new("double[]", list(input_coeffs))
    kp_starts_buf = ffi.new("double[]", piece_starts)
    kp_ends_buf = ffi.new("double[]", piece_ends)
    kp_coeffs_buf = ffi.new("double[]", piece_coeffs_flat)

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
    n_out = lib.smooth_compose(
        int(n_prev), prev_t_buf, prev_c_buf, float(prev_T_move or 0.0),
        n_in, in_t_ends_buf, in_coeffs_buf,
        int(n_next), next_t_buf, next_c_buf, float(next_T_move or 0.0),
        int(n_pieces), kp_starts_buf, kp_ends_buf, kp_coeffs_buf,
        float(t_sm),
        out_capacity,
        out_t_ends_buf, out_coeffs_buf,
    )
    if n_out < 0:
        raise ValueError("smooth_compose failed (overflow or bad args)")
    out_t_ends = [out_t_ends_buf[i] for i in range(n_out)]
    out_coeffs = [out_coeffs_buf[i] for i in range(n_out * 15 * 4)]
    return out_t_ends, out_coeffs
