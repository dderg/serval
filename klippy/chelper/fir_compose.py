"""Python wrapper around the FIR impulse-train polynomial composer
(fir_compose.c).

Plan 8 Chunk 2: bake zv / mzv input-shaping into the planner-emitted
polynomial. Amplitudes are normalized here so the output DC gain is 1.
"""
from __future__ import annotations

import math
from typing import List, Sequence, Tuple

from klippy.chelper import get_ffi

FIR_MAX_OUT_PHASES = 32


def fir_compose(
    input_phase_t_ends: Sequence[float],
    input_coeffs: Sequence[float],
    impulse_amplitudes: Sequence[float],
    impulse_delays: Sequence[float],
    out_capacity: int = FIR_MAX_OUT_PHASES,
    normalize: bool = True,
) -> Tuple[List[float], List[float]]:
    """Bake an FIR impulse train into a piecewise-polynomial move.

    Parameters
    ----------
    input_phase_t_ends : sequence[float]
        Absolute move-local end time per input phase.
    input_coeffs : sequence[float]
        n_in * 15 * 4 doubles, interleaved per-axis (Plan 8 Chunk 3:
        x, y, z, e). The .e slot is ignored on input and zeroed on output.
    impulse_amplitudes : sequence[float]
        Shaper amplitudes a_i. When `normalize` is True they are scaled to
        sum to 1.0 (standard convention for zero-vibration shapers).
    impulse_delays : sequence[float]
        Shaper delays tau_i. Must be >= 0.
    out_capacity : int, optional
        Output phase capacity.
    normalize : bool, optional
        When True (default) normalize amplitudes so sum == 1.

    Returns
    -------
    (out_phase_t_ends, out_coeffs)
    """
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
    out_t_ends_buf = ffi.new("double[]", out_capacity)
    out_coeffs_buf = ffi.new("double[]", out_capacity * 15 * 4)
    n_out = lib.fir_compose(
        n_in,
        in_t_ends_buf, in_coeffs_buf,
        n_imp, amps_buf, delays_buf,
        out_capacity,
        out_t_ends_buf, out_coeffs_buf,
    )
    if n_out < 0:
        raise ValueError("fir_compose failed (overflow or bad args)")
    out_t_ends = [out_t_ends_buf[i] for i in range(n_out)]
    out_coeffs = [out_coeffs_buf[i] for i in range(n_out * 15 * 4)]
    return out_t_ends, out_coeffs
