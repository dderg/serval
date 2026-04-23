"""Python wrapper around the degree-4 Chebyshev piecewise fitter
(klippy/chelper/cheb_fit.c).

Plan 8 Chunk 3 Task 5: provides the per-interval closed-form Chebyshev
interpolation used by nonlinear_pa_compose to bake tanh / recipr
pressure-advance into the planner polynomial.

Layered API:
  * ``cheb_nodes(v_lo, v_hi)`` — the 5 Chebyshev-second-kind node
    locations on [v_lo, v_hi] in increasing-v order.
  * ``cheb_fit_interval(samples)`` — fit a single sub-interval, returning
    monomial coefficients m[0..4] evaluated in normalized
    ``t = 2*(v - v_lo)/(v_hi - v_lo) - 1``.
  * ``cheb_fit_piecewise(v_lo, v_hi, breaks, samples_per_piece)`` — fit a
    list of sub-intervals in one FFI call.
  * ``cheb_eval_mono(mono, v_lo, v_hi, v)`` — evaluate a monomial piece
    at a v-value.
  * ``cheb_fit_function(f, v_lo, v_hi, breaks=())`` — convenience
    wrapper that samples f at the Chebyshev nodes of each sub-interval
    and returns the fit. Includes an estimated max-abs residual sampled
    on a dense grid.
"""
from __future__ import annotations

from typing import Callable, List, Sequence, Tuple

from klippy.chelper import get_ffi

CHEB_FIT_DEGREE = 4
CHEB_FIT_COEFFS = 5


def cheb_nodes(v_lo: float, v_hi: float) -> List[float]:
    """Return the 5 Chebyshev-second-kind nodes on [v_lo, v_hi] in
    increasing-v order (including both endpoints).
    """
    ffi, lib = get_ffi()
    buf = ffi.new("double[%d]" % CHEB_FIT_COEFFS)
    lib.cheb_fit_degree4_nodes(float(v_lo), float(v_hi), buf)
    return [buf[i] for i in range(CHEB_FIT_COEFFS)]


def cheb_fit_interval(samples: Sequence[float]) -> List[float]:
    """Fit one sub-interval. Returns 5 monomial coefficients m[0..4].

    The polynomial is evaluated in normalized coordinate
    ``t = 2*(v - v_lo)/(v_hi - v_lo) - 1`` as ``sum_k m[k] * t**k``.
    """
    if len(samples) != CHEB_FIT_COEFFS:
        raise ValueError(
            "expected %d samples, got %d" % (CHEB_FIT_COEFFS, len(samples))
        )
    ffi, lib = get_ffi()
    in_buf = ffi.new("double[]", list(samples))
    out_buf = ffi.new("double[%d]" % CHEB_FIT_COEFFS)
    lib.cheb_fit_degree4_interval(in_buf, ffi.NULL, out_buf)
    return [out_buf[i] for i in range(CHEB_FIT_COEFFS)]


def cheb_fit_piecewise(
    v_lo: float, v_hi: float,
    breaks: Sequence[float],
    samples_per_piece: Sequence[Sequence[float]],
) -> Tuple[List[List[float]], List[float]]:
    """Fit a piecewise Chebyshev polynomial over [v_lo, v_hi].

    Parameters
    ----------
    v_lo, v_hi : endpoints.
    breaks : interior breakpoints, strictly inside (v_lo, v_hi).
    samples_per_piece : one list of 5 samples per piece.

    Returns
    -------
    (mono_coeffs_per_piece, piece_v_bounds)
        mono_coeffs_per_piece : list of 5-element lists.
        piece_v_bounds : [v_lo, *breaks, v_hi] (length n_pieces + 1).
    """
    n_pieces = len(breaks) + 1
    if len(samples_per_piece) != n_pieces:
        raise ValueError(
            "need %d sample-lists for %d pieces, got %d"
            % (n_pieces, n_pieces, len(samples_per_piece))
        )
    flat_samples = []
    for piece in samples_per_piece:
        if len(piece) != CHEB_FIT_COEFFS:
            raise ValueError(
                "each piece needs %d samples" % CHEB_FIT_COEFFS
            )
        flat_samples.extend(float(s) for s in piece)
    ffi, lib = get_ffi()
    breaks_buf = (
        ffi.new("double[]", list(breaks)) if breaks else ffi.NULL
    )
    samples_buf = ffi.new("double[]", flat_samples)
    out_mono = ffi.new("double[%d]" % (n_pieces * CHEB_FIT_COEFFS))
    out_bounds = ffi.new("double[%d]" % (n_pieces + 1))
    rc = lib.cheb_fit_degree4_piecewise(
        float(v_lo), float(v_hi),
        len(breaks), breaks_buf,
        samples_buf, out_mono, out_bounds,
    )
    if rc != 0:
        raise ValueError(
            "cheb_fit_degree4_piecewise returned %d (bad breakpoints?)" % rc
        )
    mono = [
        [out_mono[i * CHEB_FIT_COEFFS + k] for k in range(CHEB_FIT_COEFFS)]
        for i in range(n_pieces)
    ]
    bounds = [out_bounds[i] for i in range(n_pieces + 1)]
    return mono, bounds


def cheb_eval_mono(
    mono: Sequence[float],
    v_lo: float, v_hi: float,
    v: float,
) -> float:
    """Evaluate the monomial form at v in [v_lo, v_hi]."""
    ffi, lib = get_ffi()
    buf = ffi.new("double[]", list(mono))
    return lib.cheb_fit_degree4_eval_mono(
        buf, float(v_lo), float(v_hi), float(v)
    )


def cheb_fit_function(
    f: Callable[[float], float],
    v_lo: float,
    v_hi: float,
    breaks: Sequence[float] = (),
    residual_samples: int = 65,
) -> Tuple[List[List[float]], List[float], float]:
    """Evaluate f at the Chebyshev nodes of each sub-interval, fit
    piecewise, and return the per-piece monomials plus a coarse estimate
    of the max-abs residual on a dense grid.

    Returns (mono_per_piece, piece_v_bounds, residual_est).
    """
    breaks = list(breaks)
    piece_bounds = [v_lo] + breaks + [v_hi]
    samples_per_piece = []
    for i in range(len(breaks) + 1):
        lo, hi = piece_bounds[i], piece_bounds[i + 1]
        nodes = cheb_nodes(lo, hi)
        samples_per_piece.append([float(f(v)) for v in nodes])
    mono, bounds = cheb_fit_piecewise(
        v_lo, v_hi, breaks, samples_per_piece
    )
    # Dense-grid residual estimate across the full range.
    if residual_samples < 2:
        return mono, bounds, 0.0
    residual = 0.0
    for k in range(residual_samples):
        v = v_lo + (v_hi - v_lo) * k / (residual_samples - 1)
        # Pick piece containing v.
        piece_idx = 0
        for p in range(len(breaks)):
            if v >= breaks[p]:
                piece_idx = p + 1
        lo, hi = bounds[piece_idx], bounds[piece_idx + 1]
        approx = cheb_eval_mono(mono[piece_idx], lo, hi, v)
        err = abs(f(v) - approx)
        if err > residual:
            residual = err
    return mono, bounds, residual
