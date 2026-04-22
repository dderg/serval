# Definitions of the supported input shapers
#
# Copyright (C) 2020-2023  Dmitry Butyugin <dmbutyugin@google.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import collections
import math

DEFAULT_DAMPING_RATIO = 0.1

InputShaperCfg = collections.namedtuple(
    "InputShaperCfg", ("name", "init_func", "min_freq")
)
InputSmootherCfg = collections.namedtuple(
    "InputSmootherCfg", ("name", "init_func", "min_freq")
)


def get_none_shaper():
    return ([], [])


def get_zv_shaper(shaper_freq, damping_ratio):
    df = math.sqrt(1.0 - damping_ratio**2)
    K = math.exp(-damping_ratio * math.pi / df)
    t_d = 1.0 / (shaper_freq * df)
    A = [1.0, K]
    T = [0.0, 0.5 * t_d]
    return (A, T)


def get_mzv_shaper(shaper_freq, damping_ratio):
    df = math.sqrt(1.0 - damping_ratio**2)
    K = math.exp(-0.75 * damping_ratio * math.pi / df)
    t_d = 1.0 / (shaper_freq * df)

    a1 = 1.0 - 1.0 / math.sqrt(2.0)
    a2 = (math.sqrt(2.0) - 1.0) * K
    a3 = a1 * K * K

    A = [a1, a2, a3]
    T = [0.0, 0.375 * t_d, 0.75 * t_d]
    return (A, T)


def get_shaper_offset(A, T):
    if not A:
        return 0.0
    return sum([a * t for a, t in zip(A, T)]) / sum(A)


# Legacy single-polynomial kernel coefficients -> one-piece piecewise form.
# Kept as a helper for consumers (e.g. the pressure-advance extruder smoother
# at klippy/kinematics/extruder.py:24, and the [input_shaper] custom
# `coeffs_{x,y}` config path) that still describe a kernel as flat
# power-basis coefficients over the centered window [-t_sm/2, +t_sm/2].
# Returns (C_pieces, t_sm) in the same format as the bs*-family init_func
# results so downstream code can treat both uniformly.
#
# Input `coeffs` is ASCENDING power-basis: coeffs[i] is the coefficient of
# t^i in w(t) = sum_i coeffs[i] * t^i. This matches the legacy pre-Plan-5
# C init_smoother convention, which the existing PA-smoother literal
# [15/8, 0, -15, 0, 30] at kinematics/extruder.py:24 relies on (that literal
# encodes w_raw(t) = 15/8 - 15*t^2 + 30*t^4, a 4th-order smoothing window
# that vanishes at t = +-t_sm/2 once normalized by 1/t_sm^(i+1)).
#
# With normalize_coeffs=True each coefficient is divided by t_sm^(i+1). The
# C-side init_smoother additionally rescales to unit integral so the
# pre-norm magnitude here does not have to match the unit-integral target
# exactly.
def init_smoother(coeffs, smooth_time, normalize_coeffs):
    n = len(coeffs)
    if n == 0 or smooth_time <= 0.0:
        return ([], smooth_time if smooth_time > 0.0 else 0.0)
    if normalize_coeffs:
        inv_t_sm = 1.0 / smooth_time
        piece_coeffs = [0.0] * n
        scale = inv_t_sm
        for i in range(n):
            piece_coeffs[i] = coeffs[i] * scale
            scale *= inv_t_sm
    else:
        piece_coeffs = list(coeffs)
    hst = 0.5 * smooth_time
    return ([(-hst, hst, piece_coeffs)], smooth_time)


def get_none_smoother():
    # Identity / disabled smoother. Zero pieces, zero width.
    return ([], 0.0)


# ---------------------------------------------------------------------------
# Cardinal B-spline chain family (bs1..bs5)
#
# Replacement for the legacy smooth_* family. Each variant is the (m+1)-fold
# self-convolution of a unit rectangle of width T_sm/(m+1), rescaled to
# support [-T_sm/2, +T_sm/2] with unit integral.
#
# Closed-form per-piece coefficients follow Curry-Schoenberg (1966) Theorem 2
# divided-difference form. F_m = T_sm * f_sh constants are pre-computed for
# damping ratio zeta=0.1 and 5% residual vibration target; values match
# docs/superpowers/plans/plan5-derivations/new_shaper_family.md §2.
# ---------------------------------------------------------------------------

_F_M_TABLE = {
    1: 1.5553,
    2: 1.9462,
    3: 2.2519,
    4: 2.5061,
    5: 2.7252,
}


def _cardinal_bspline_pieces(m):
    """Return [(a_canonical, b_canonical, coeffs_ascending), ...] for the
    cardinal B-spline of order m on canonical support [0, m+1].

    The kernel value on sub-interval [i, i+1] is the polynomial:
        N_{m+1}(tau) = (1 / m!) * sum_{k=0..i} (-1)^k * C(m+1, k) * (tau - k)^m
    Expanded into the power basis tau^j, j = 0..m.
    """
    pieces = []
    fac_m = float(math.factorial(m))
    for i in range(m + 1):
        coeffs = [0.0] * (m + 1)
        for k in range(i + 1):
            sign = (-1.0) ** k
            binom = math.comb(m + 1, k)
            for j in range(m + 1):
                coeffs[j] += (sign * binom * math.comb(m, j)
                              * ((-k) ** (m - j))) / fac_m
        pieces.append((float(i), float(i + 1), coeffs))
    return pieces


def _rescale_piece(piece, t_sm, m):
    """Rescale a canonical-[0, m+1] piece to real-time [-t_sm/2, +t_sm/2]
    with unit integral.

    Map canonical tau -> real t via tau = s*(t - shift), where
    s = (m+1) / t_sm and shift = -t_sm/2. Substitute and re-expand.

    The canonical kernel already integrates to 1 over canonical support
    (cardinal B-splines have unit integral). Changing variable introduces
    a Jacobian factor of s = (m+1)/t_sm so the density integrates to 1
    over the real support [-t_sm/2, +t_sm/2].
    """
    a, b, coeffs = piece
    s = (m + 1) / t_sm
    shift = -0.5 * t_sm
    n = len(coeffs)

    # Substitute tau = s*(t - shift) -> tau = s*t + b0 where b0 = -s*shift.
    # Then (s*t + b0)^j expands via binomial into ascending powers of t.
    b0 = -s * shift
    # new_coeffs[k] = sum_j coeffs[j] * C(j, k) * s^k * b0^(j-k)
    new_coeffs = [0.0] * n
    for j in range(n):
        if coeffs[j] == 0.0:
            continue
        for k in range(j + 1):
            new_coeffs[k] += (coeffs[j] * math.comb(j, k)
                              * (s ** k) * (b0 ** (j - k)))
    # Jacobian: dtau = s * dt, so density scales by s.
    new_coeffs = [c * s for c in new_coeffs]
    # New piece bounds (map canonical [a, b] -> real).
    # t = tau/s + shift
    new_a = a / s + shift
    new_b = b / s + shift
    return (new_a, new_b, new_coeffs)


def _get_bs_smoother(m, shaper_freq,
                     damping_ratio_unused=None,
                     normalize_coeffs=True):
    """Return (C_pieces, t_sm) for cardinal B-spline of order m at shaper_freq.

    damping_ratio/normalize_coeffs are accepted for call-signature parity
    with the legacy smooth_* init_func entries. The cardinal B-spline has
    a closed-form kernel independent of damping ratio; normalize_coeffs
    is ignored (the kernel is always unit-integral by construction).
    """
    if shaper_freq <= 0.0:
        return ([], 0.0)
    F_m = _F_M_TABLE[m]
    t_sm = F_m / shaper_freq
    canonical = _cardinal_bspline_pieces(m)
    pieces = [_rescale_piece(p, t_sm, m) for p in canonical]
    return (pieces, t_sm)


def _get_bs1_smoother(f, z=None, n=True):
    return _get_bs_smoother(1, f, z, n)


def _get_bs2_smoother(f, z=None, n=True):
    return _get_bs_smoother(2, f, z, n)


def _get_bs3_smoother(f, z=None, n=True):
    return _get_bs_smoother(3, f, z, n)


def _get_bs4_smoother(f, z=None, n=True):
    return _get_bs_smoother(4, f, z, n)


def _get_bs5_smoother(f, z=None, n=True):
    return _get_bs_smoother(5, f, z, n)


def bspline_eval(C_pieces, grid, t_sm=None):
    """Evaluate a piecewise-polynomial kernel at the given sample points.

    C_pieces : list of (t_start, t_end, coeffs_ascending)
    grid     : iterable of t-values
    t_sm     : unused; accepted for symmetry with legacy callers.

    Returns a list/numpy-array of kernel values. Points outside any piece
    return 0.0.
    """
    try:
        import numpy as np

        grid_arr = np.asarray(grid, dtype=float)
        out = np.zeros_like(grid_arr)
        for (a, b, coeffs) in C_pieces:
            if not coeffs:
                continue
            # Inclusive on the left; last piece inclusive on the right so
            # endpoint samples stay covered.
            mask = (grid_arr >= a) & (grid_arr <= b)
            if not mask.any():
                continue
            # Horner evaluation of ascending-power coeffs.
            vals = np.zeros(int(mask.sum()))
            tsel = grid_arr[mask]
            for c in reversed(coeffs):
                vals = vals * tsel + c
            out[mask] = vals
        return out
    except ImportError:
        out = []
        for t in grid:
            v = 0.0
            for (a, b, coeffs) in C_pieces:
                if a <= t <= b and coeffs:
                    acc = 0.0
                    for c in reversed(coeffs):
                        acc = acc * t + c
                    v = acc
                    break
            out.append(v)
        return out


def get_smoother_offset(C_pieces, t_sm, normalized=True):
    """First central moment / zeroth central moment of the kernel.

    C_pieces may be either:
      - New piecewise form: list of (t_start, t_end, coeffs_ascending).
      - Empty list (identity / disabled smoother): returns 0.
    """
    if not C_pieces or t_sm <= 0.0:
        return 0.0
    int_t0 = 0.0
    int_t1 = 0.0
    for (a, b, coeffs) in C_pieces:
        for k, c in enumerate(coeffs):
            # int_{a}^{b} c * tau^k dtau
            int_t0 += c * (b ** (k + 1) - a ** (k + 1)) / (k + 1)
            # int_{a}^{b} c * tau^(k+1) dtau
            int_t1 += c * (b ** (k + 2) - a ** (k + 2)) / (k + 2)
    if int_t0 == 0.0:
        return 0.0
    return int_t1 / int_t0


# min_freq for each shaper is chosen to have projected max_accel ~= 1500
# This Kalico fork retains only zv and mzv as impulse shapers; the full
# EI family is replaced by the smooth variants below.
INPUT_SHAPERS = [
    InputShaperCfg("zv", get_zv_shaper, min_freq=21.0),
    InputShaperCfg("mzv", get_mzv_shaper, min_freq=23.0),
]

# Cardinal B-spline chain family (Plan 5 "Magnum Opus" replacement for the
# legacy smooth_* family). Single integer parameter m = 1..5 interpolates
# from narrow/fast/weak (bs1) to wide/slow/strong (bs5). All variants share
# a closed-form kernel with no passband spectral zeros, which lets every
# variant carry a finite-support FIR inverse (future Plan 5 D3).
INPUT_SMOOTHERS = [
    InputSmootherCfg("bs1", _get_bs1_smoother, min_freq=18.0),
    InputSmootherCfg("bs2", _get_bs2_smoother, min_freq=20.0),
    InputSmootherCfg("bs3", _get_bs3_smoother, min_freq=21.0),
    InputSmootherCfg("bs4", _get_bs4_smoother, min_freq=23.0),
    InputSmootherCfg("bs5", _get_bs5_smoother, min_freq=25.0),
]


# Migration table: old smooth_* name -> closest bs* replacement.
# See docs/superpowers/specs/2026-04-22-plan5-direct-quintic-pillar1-design.md
# §D6 for rationale.
RETIRED_SMOOTHER_MIGRATION = {
    "smooth_zv": "bs1",
    "smooth_mzv": "bs2",
    "smooth_ei": "bs3",
    "smooth_2hump_ei": "bs4",
    "smooth_zvd_ei": "bs5",
    "smooth_si": "bs3",
}
