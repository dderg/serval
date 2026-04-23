"""Python wrapper around the bs-kernel polynomial composer (bs_compose.c).

Plan 8 Chunk 2: convolve a move's per-phase quintic-in-t polynomials with a
cardinal B-spline kernel bs_m (m in 1..5) at plan time, producing a
piecewise polynomial in the same 15-coefficient slot layout that
trapq_append_quintic expects.
"""
from __future__ import annotations

from typing import Iterable, List, Sequence, Tuple

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
) -> Tuple[List[float], List[float]]:
    """Compose the bs-kernel convolution over a quintic piecewise polynomial.

    Parameters
    ----------
    input_phase_t_ends : sequence[float]
        Absolute move-local end time of each input phase. Length = n_in.
    input_coeffs : sequence[float]
        n_in * 15 * 4 doubles, per phase (Plan 8 Chunk 3):
            c[0].x, c[0].y, c[0].z, c[0].e, c[1].x, ... c[14].e
        The .e slot is ignored on input and zeroed on output — the
        downstream linear-PA composer fills it from the baked XY polynomial.
    bs_order : int
        Cardinal B-spline order 1..5.
    shaper_freq : float
        Shaper frequency in Hz; > 0.
    damping_ratio : float, optional
        Accepted for signature parity. Ignored (bs kernel is damping-
        independent).
    out_capacity : int, optional
        Output phase capacity. Defaults to BS_MAX_OUT_PHASES.

    Returns
    -------
    (out_phase_t_ends, out_coeffs)
        out_phase_t_ends: list of n_out doubles.
        out_coeffs: list of n_out * 15 * 4 doubles, same layout as input.

    Raises
    ------
    ValueError if the composer fails (overflow, bad inputs).
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
    out_t_ends_buf = ffi.new("double[]", out_capacity)
    out_coeffs_buf = ffi.new("double[]", out_capacity * 15 * 4)
    n_out = lib.bs_compose(
        n_in,
        in_t_ends_buf, in_coeffs_buf,
        int(bs_order), float(shaper_freq), float(damping_ratio),
        out_capacity,
        out_t_ends_buf, out_coeffs_buf,
    )
    if n_out < 0:
        raise ValueError("bs_compose failed (overflow or bad args)")
    out_t_ends = [out_t_ends_buf[i] for i in range(n_out)]
    out_coeffs = [out_coeffs_buf[i] for i in range(n_out * 15 * 4)]
    return out_t_ends, out_coeffs
