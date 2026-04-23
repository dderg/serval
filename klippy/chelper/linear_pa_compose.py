"""Python wrapper around the linear pressure-advance polynomial composer
(linear_pa_compose.c).

Plan 8 Chunk 3 Task 2: bake linear pressure advance into the planner-
emitted polynomial. The composer reads the .x/.y/.z slots of a baked
4-axis coeff_buf (the output of bs_compose / fir_compose / pass-through)
and writes the .e slot in place per the math derivation in the header.
"""
from __future__ import annotations

from typing import List, Sequence, Tuple

from klippy.chelper import get_ffi


def linear_pa_compose(
    n_phases: int,
    coeff_buf: Sequence[float],
    axis_n: Tuple[float, float, float],
    extr_r: float,
    k_pa: float,
) -> List[float]:
    """Compose linear PA into the .e slot of an n_phases * 15 * 4 buffer.

    Parameters
    ----------
    n_phases : int
        Number of phases in the buffer.
    coeff_buf : sequence[float]
        Length n_phases * 15 * 4. Layout per coefficient:
        (.x, .y, .z, .e). The .x/.y/.z slots are read; .e is overwritten
        in the returned copy.
    axis_n : (n_x, n_y, n_z)
        Unit XY direction along motion (used to project XYZ position into
        a 1-D arc-length polynomial).
    extr_r : float
        Extruder ratio: filament-mm per XY-arc-mm (signed).
    k_pa : float
        Linear pressure-advance coefficient.

    Returns
    -------
    list[float]
        A length-(n_phases * 15 * 4) list with the .e slot filled. The
        .x/.y/.z slots are preserved bit-identically.
    """
    ffi, lib = get_ffi()
    expected = n_phases * 15 * 4
    if len(coeff_buf) != expected:
        raise ValueError(
            f"coeff_buf length {len(coeff_buf)} != expected {expected}"
        )
    buf = ffi.new("double[]", list(coeff_buf))
    lib.linear_pa_compose(
        int(n_phases), buf,
        float(axis_n[0]), float(axis_n[1]), float(axis_n[2]),
        float(extr_r), float(k_pa),
    )
    return [buf[i] for i in range(expected)]
