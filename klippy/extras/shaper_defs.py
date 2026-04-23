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
# Legacy smooth-IS family (pre-Plan-5, restored alongside the bs family).
#
# Each smoother below is a SINGLE polynomial piece on the centered window
# [-t_sm/2, +t_sm/2]. Coefficients were optimized by Dmitry Butyugin (2020-
# 2023) in the Maxima algebra system at zeta=0.1 for a 5% residual-vibration
# target; see `docs/superpowers/plans/plan5-derivations/shaper_family.md`
# and the upstream Klipper commit history for the closed-form derivation.
#
# The raw coefficient lists below are stored DESCENDING (highest degree
# first), matching the historical convention used by the pre-Plan-5
# C `init_smoother`. They are reversed to ASCENDING in-situ before being
# passed to the shared `init_smoother` helper defined above.
# ---------------------------------------------------------------------------


def _smooth_init(coeffs_descending, smooth_time, normalize_coeffs):
    """Convert descending-coeff legacy smooth-IS kernel to piecewise form.

    The shared `init_smoother` above consumes ASCENDING-power coefficients
    and produces a one-piece (t_start, t_end, coeffs) tuple. Reverse here
    so the historical descending literals paste in unchanged.
    """
    return init_smoother(
        list(reversed(coeffs_descending)), smooth_time, normalize_coeffs
    )


def get_smooth_zv_smoother(
    shaper_freq, damping_ratio_unused=None, normalize_coeffs=True
):
    coeffs = [
        -118.4265334338076,
        5.861885495127615,
        29.52796003014231,
        -1.465471373781904,
        0.01966833207740377,
    ]
    return _smooth_init(coeffs, 0.8025 / shaper_freq, normalize_coeffs)


def get_smooth_mzv_smoother(
    shaper_freq, damping_ratio_unused=None, normalize_coeffs=True
):
    coeffs = [
        -1906.717580206364,
        125.8892756660212,
        698.0200035767849,
        -37.75923018121473,
        -62.18762409216703,
        1.57172781617736,
        1.713117990217123,
    ]
    return _smooth_init(coeffs, 0.95625 / shaper_freq, normalize_coeffs)


def get_smooth_ei_smoother(
    shaper_freq, damping_ratio_unused=None, normalize_coeffs=True
):
    coeffs = [
        -1797.048868963208,
        120.5310596109878,
        669.6653197989012,
        -35.71975707450795,
        -62.49388325512682,
        1.396748042940248,
        1.848276903900512,
    ]
    return _smooth_init(coeffs, 1.06625 / shaper_freq, normalize_coeffs)


def get_smooth_2hump_ei_smoother(
    shaper_freq, damping_ratio_unused=None, normalize_coeffs=True
):
    coeffs = [
        -22525.88434486782,
        2524.826047114184,
        10554.22832043971,
        -1051.778387878068,
        -1475.914693073253,
        121.2177946817349,
        57.95603221424528,
        -4.018706414213658,
        0.8375784787864095,
    ]
    return _smooth_init(coeffs, 1.14875 / shaper_freq, normalize_coeffs)


def get_smooth_si_smoother(
    shaper_freq, damping_ratio_unused=None, normalize_coeffs=True
):
    coeffs = [
        -6186.76006449789,
        1206.747198930197,
        2579.985143622855,
        -476.8554763069169,
        -295.546608490564,
        52.69679971161049,
        4.234582468800491,
        -2.226157642004671,
        1.267781046297883,
    ]
    return _smooth_init(coeffs, 1.245 / shaper_freq, normalize_coeffs)


def get_smooth_zvd_ei_smoother(
    shaper_freq, damping_ratio_unused=None, normalize_coeffs=True
):
    coeffs = [
        -18835.07746719777,
        1914.349309746547,
        8786.608981369287,
        -807.3061869131075,
        -1209.429748155012,
        96.48879052981883,
        43.1595785340444,
        -3.577268915175282,
        1.083220648523371,
    ]
    return _smooth_init(coeffs, 1.475 / shaper_freq, normalize_coeffs)


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

# Smoother catalog: cardinal B-spline chain (bs1..bs5) and the legacy
# smooth-IS family. Both kernel families compose through
# `smooth_compose`/`bs_compose` in the planner — bs variants carry a
# closed-form kernel with no passband spectral zeros (FIR-invertible),
# while the smooth-IS variants are the single-piece polynomials from the
# original Butyugin design, preferred by some printers for cleaner
# visual output at equivalent support width.
INPUT_SMOOTHERS = [
    InputSmootherCfg("bs1", _get_bs1_smoother, min_freq=18.0),
    InputSmootherCfg("bs2", _get_bs2_smoother, min_freq=20.0),
    InputSmootherCfg("bs3", _get_bs3_smoother, min_freq=21.0),
    InputSmootherCfg("bs4", _get_bs4_smoother, min_freq=23.0),
    InputSmootherCfg("bs5", _get_bs5_smoother, min_freq=25.0),
    InputSmootherCfg("smooth_zv", get_smooth_zv_smoother, min_freq=18.0),
    InputSmootherCfg("smooth_mzv", get_smooth_mzv_smoother, min_freq=20.0),
    InputSmootherCfg("smooth_ei", get_smooth_ei_smoother, min_freq=21.0),
    InputSmootherCfg(
        "smooth_2hump_ei", get_smooth_2hump_ei_smoother, min_freq=21.5
    ),
    InputSmootherCfg(
        "smooth_zvd_ei", get_smooth_zvd_ei_smoother, min_freq=26.0
    ),
    InputSmootherCfg("smooth_si", get_smooth_si_smoother, min_freq=21.5),
]
