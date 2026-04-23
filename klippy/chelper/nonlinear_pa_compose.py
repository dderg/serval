"""Python wrapper around the non-linear pressure-advance polynomial
composer (klippy/chelper/nonlinear_pa_compose.c).

Plan 8 Chunk 3 Task 6: bake tanh / recipr pressure advance into the
planner-emitted polynomial via piecewise degree-4 Chebyshev fits on
phase-local tau. The composer also absorbs the exact linear-PA terms
(extr_r * P_proj + linear_advance * V_proj) so it's a drop-in
replacement for linear_pa_compose when the model is non-linear.

Model dispatch:
    "none" or None  -> NLPA_MODEL_NONE (linear-only)
    "tanh"          -> NLPA_MODEL_TANH
    "recipr"        -> NLPA_MODEL_RECIPR
"""
from __future__ import annotations

from typing import List, Optional, Sequence, Tuple

from klippy.chelper import get_ffi

NLPA_MODEL_NONE = 0
NLPA_MODEL_TANH = 1
NLPA_MODEL_RECIPR = 2

_MODEL_KIND = {
    None: NLPA_MODEL_NONE,
    "none": NLPA_MODEL_NONE,
    "linear": NLPA_MODEL_NONE,  # linear handled by linear_pa_compose path
    "tanh": NLPA_MODEL_TANH,
    "recipr": NLPA_MODEL_RECIPR,
}


def nonlinear_pa_compose(
    n_phases: int,
    phase_t_ends: Sequence[float],
    coeff_buf: Sequence[float],
    axis_n: Tuple[float, float, float],
    extr_r: float,
    linear_advance: float,
    nonlinear_offset: float,
    linearization_velocity: float,
    model: Optional[str] = "tanh",
) -> Tuple[List[float], float]:
    """Compose tanh / recipr PA into the .e slot of an n_phases * 15 * 4
    buffer in place.

    Returns (coeff_buf_list, max_residual). The residual is the raw
    (unscaled) Chebyshev fit error; filament error = residual *
    nonlinear_offset.
    """
    ffi, lib = get_ffi()
    expected = n_phases * 15 * 4
    if len(coeff_buf) != expected:
        raise ValueError(
            f"coeff_buf length {len(coeff_buf)} != expected {expected}"
        )
    if len(phase_t_ends) != n_phases:
        raise ValueError(
            f"phase_t_ends length {len(phase_t_ends)} != n_phases {n_phases}"
        )
    if model is None:
        kind = NLPA_MODEL_NONE
    else:
        try:
            kind = _MODEL_KIND[model]
        except KeyError as exc:
            raise ValueError("unknown PA model %r" % model) from exc
    buf = ffi.new("double[]", list(coeff_buf))
    ts = ffi.new("double[]", list(phase_t_ends))
    residual = ffi.new("double[1]")
    lib.nonlinear_pa_compose(
        int(n_phases), ts, buf,
        float(axis_n[0]), float(axis_n[1]), float(axis_n[2]),
        float(extr_r),
        float(linear_advance),
        float(nonlinear_offset),
        float(linearization_velocity),
        int(kind),
        residual,
    )
    return [buf[i] for i in range(expected)], float(residual[0])
