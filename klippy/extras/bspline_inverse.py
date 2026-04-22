# FIR companion kernel for the cardinal B-spline smoother family (bs1..bs5).
#
# Plan 5 (Magnum Opus) Pillar 1, Task 4. Computes a finite-support inverse
# h(tau) such that the cascade (h * w)(t) approximates delta(t) inside the
# motion passband. Construction follows
# docs/superpowers/plans/plan5-derivations/new_shaper_family.md section 10:
#
#   1. Sample the forward kernel w symmetrically about zero at grid dt.
#   2. Normalize to unit discrete integral, zero-pad to an FFT-friendly
#      power-of-two length, and compute W = FFT(w) * dt.
#   3. Build a bandlimited inverse mask in the frequency domain:
#        - Pure 1/W on [0, pb_max_hz].
#        - Cosine-taper rolloff over (pb_max_hz, f_sh).
#        - Hard zero above f_sh.
#      f_sh is passed explicitly (not inferred from pb_max_hz) to avoid the
#      pb_max = 0.3 * f_sh coupling from being load-bearing. pb_max_hz
#      defaults to 0.3 * f_sh_hz when omitted, matching the section 4.3
#      reference table. The Tikhonov-only variant from
#      fir_companion_kernel.md section 1 is documented there as failing on
#      this kernel family (out-of-band |W| near zero gives H huge HF gain
#      and G = ||h||_1 on the order of 10^2); the hard bandlimit is what
#      keeps G near 2.
#   4. IFFT back, truncate to T_h = T_h_ratio * T_sm (odd length, center
#      tap at tau = 0), Tukey-window, and renormalize so integral of h = 1.
#
# Task 5 extends the module with the Path C fused kernel fit: numerically
# convolve h with w, least-squares fit the sampled cascade to 9 equal-width
# degree-5 pieces, and publish closed-form passband-error diagnostics for
# regression testing. See
# docs/superpowers/plans/plan5-derivations/fused_kernel_storage_resolution.md
# for derivation and verification table.
#
# Copyright (C) 2026  Danila Dergachev
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math

import numpy as np

from klippy.extras import shaper_defs


def _tukey_window(n, alpha):
    """Tukey window of length n with taper fraction alpha per side."""
    if n <= 1:
        return np.ones(max(n, 0))
    if alpha <= 0.0:
        return np.ones(n)
    L = int(alpha * (n - 1) / 2)
    wnd = np.ones(n)
    if L <= 0:
        return wnd
    idx = np.arange(L)
    ramp = 0.5 * (1.0 + np.cos(np.pi * (idx / L - 1.0)))
    wnd[:L] = ramp
    wnd[n - L:] = ramp[::-1]
    return wnd


def compute_inverse_fir(C_pieces, t_sm, f_sh_hz, pb_max_hz=None,
                        dt=1e-5, T_h_ratio=2.0, tukey_alpha=0.25,
                        eps_rel=3e-3):
    """FIR inverse of a piecewise-polynomial forward kernel.

    Cosine-taper + hard-bandlimit IFFT design per new_shaper_family.md
    section 10. Returns (h, T_h, dt) with h an odd-length 1-D numpy array
    of FIR taps centered at tau = 0.

    Args:
        C_pieces: list of (t_start, t_end, coeffs) as produced by
            shaper_defs.INPUT_SMOOTHERS[*].init_func.
        t_sm: forward kernel support width.
        f_sh_hz: shaper frequency (Hz). Hard spectral zero above this;
            this is the outer edge of the cosine-taper transition band.
        pb_max_hz: passband upper bound (Hz). Defaults to 0.3 * f_sh_hz
            to match the section 4.3 reference table convention. Pass
            explicitly for diagnostics against the wider pb_max = 0.5 * f_sh
            band.
        dt: tap spacing (default 10 us).
        T_h_ratio: T_h / T_sm (default 2.0).
        tukey_alpha: Tukey taper fraction (default 0.25).
        eps_rel: safety floor on |W| as a fraction of max|W|, guarding
            against literal zeros in the discrete spectrum (default 3e-3).
            Not a Tikhonov weight; the pure-Tikhonov form
            H = conj(W)/(|W|^2 + eps^2) was superseded by the hard
            bandlimit because the B-spline spectrum falls off too fast
            for Tikhonov alone to control HF gain.
    """
    if not C_pieces or t_sm <= 0.0:
        raise ValueError("compute_inverse_fir requires a non-empty kernel "
                         "with t_sm > 0")
    if f_sh_hz <= 0.0:
        raise ValueError("compute_inverse_fir requires f_sh_hz > 0")
    if pb_max_hz is None:
        pb_max_hz = 0.3 * f_sh_hz
    if pb_max_hz <= 0.0 or pb_max_hz >= f_sh_hz:
        raise ValueError("compute_inverse_fir requires "
                         "0 < pb_max_hz < f_sh_hz")
    # 1. Sample forward kernel on a symmetric odd-length grid.
    hst = 0.5 * t_sm
    n_w = int(2 * hst / dt) + 1
    if n_w % 2 == 0:
        n_w += 1
    t_w = (np.arange(n_w) - n_w // 2) * dt
    w = np.asarray(shaper_defs.bspline_eval(C_pieces, t_w, t_sm), dtype=float)
    w_area = float(np.sum(w) * dt)
    if w_area <= 0.0:
        raise ValueError("forward kernel has non-positive discrete integral")
    w = w / w_area
    # 2. FFT-friendly zero-pad (factor 32 per section 10 reproducer).
    T_h = T_h_ratio * t_sm
    L_min = 32.0 * max(t_sm, T_h)
    N_fft = int(2 ** math.ceil(math.log2(L_min / dt)))
    w_pad = np.zeros(N_fft)
    start = N_fft // 2 - n_w // 2
    w_pad[start:start + n_w] = w
    W = np.fft.fft(np.fft.ifftshift(w_pad)) * dt
    # 3. Bandlimited inverse mask: 1/W inside passband, cosine-taper in
    # transition band, hard zero above f_sh.
    f_sh = f_sh_hz
    freqs = np.fft.fftfreq(N_fft, dt)
    fa = np.abs(freqs)
    # eps_rel floors |W| so the 1/W division is safe even when the sinc^m+1
    # spectrum lands on a sample within machine-epsilon of an exact zero.
    eps_floor = eps_rel * float(np.max(np.abs(W)))
    W_safe = W.copy()
    near_zero = np.abs(W_safe) < eps_floor
    if near_zero.any():
        # Preserve phase; clamp magnitude to eps_floor.
        phase = np.where(np.abs(W_safe) > 0,
                         W_safe / np.where(np.abs(W_safe) > 0,
                                           np.abs(W_safe), 1.0),
                         1.0 + 0j)
        W_safe = np.where(near_zero, eps_floor * phase, W_safe)
    H = np.zeros(N_fft, dtype=complex)
    in_pb = fa <= pb_max_hz
    in_taper = (fa > pb_max_hz) & (fa < f_sh)
    H[in_pb] = 1.0 / W_safe[in_pb]
    if in_taper.any():
        u = (fa[in_taper] - pb_max_hz) / (f_sh - pb_max_hz)
        H[in_taper] = (1.0 / W_safe[in_taper]) * 0.5 * (1.0 + np.cos(np.pi * u))
    # 4. IFFT -> truncate to T_h -> Tukey-window -> renormalize.
    h_full = np.fft.fftshift(np.fft.ifft(H)).real / dt
    n_h = int(T_h / dt)
    if n_h % 2 == 0:
        n_h += 1
    lo = N_fft // 2 - n_h // 2
    h = h_full[lo:lo + n_h].copy()
    h *= _tukey_window(n_h, tukey_alpha)
    h_area = float(np.sum(h) * dt)
    if h_area == 0.0:
        raise ValueError("inverse kernel integral vanished after windowing")
    h /= h_area
    return h, T_h, dt


# ---------------------------------------------------------------------------
# Task 5 — Path C fused kernel fit.
#
# Numerically convolve the FIR inverse h with the piecewise forward kernel w
# to produce the fused cascade k_fused = h * w, then least-squares fit the
# sampled k_fused to 9 equal-width × degree-5 piecewise polynomial pieces for
# C-side storage. See
# docs/superpowers/plans/plan5-derivations/fused_kernel_storage_resolution.md
# for derivation and the verification table this module's tests regress
# against.
# ---------------------------------------------------------------------------


def _convolve_h_w(C_pieces, t_sm, h_taps, dt):
    """Numerically convolve h ⊛ w on a symmetric grid.

    Returns (k_samples, grid) where k_samples has length len(h)+len(w)-1 and
    grid spans [-(T_sm+T_h)/2, +(T_sm+T_h)/2] at spacing dt. Both h and the
    sampled w are centered at tau = 0 so the convolution output is centered
    at tau = 0 as well.
    """
    hst = 0.5 * t_sm
    n_w = int(2 * hst / dt) + 1
    if n_w % 2 == 0:
        n_w += 1
    t_w = (np.arange(n_w) - n_w // 2) * dt
    w = np.asarray(shaper_defs.bspline_eval(C_pieces, t_w, t_sm), dtype=float)
    # Normalize w to unit discrete integral, matching compute_inverse_fir's
    # internal convention so the cascade target is FT(w) * FT(h) = 1.
    w_area = float(np.sum(w) * dt)
    if w_area <= 0.0:
        raise ValueError("forward kernel has non-positive discrete integral")
    w = w / w_area
    k = np.convolve(h_taps, w, mode='full') * dt
    n_k = len(k)
    grid = (np.arange(n_k) - (n_k - 1) / 2.0) * dt
    return k, grid


def cascade_passband_error(C_pieces, t_sm, h_taps, dt, pb_max_hz,
                           n_eval=500):
    """Max |FT(k_fused)(f) - 1| over (0, pb_max_hz].

    k_fused = h_taps ⊛ w where w is sampled from C_pieces at spacing dt.
    The FT is computed by direct Riemann-sum evaluation at n_eval linearly
    spaced frequencies so the check is independent of FFT grid alignment.

    Args:
        C_pieces: piecewise-polynomial forward kernel.
        t_sm: forward kernel support.
        h_taps: FIR inverse tap array from compute_inverse_fir.
        dt: tap spacing.
        pb_max_hz: upper bound of the passband to check.
        n_eval: number of evaluation frequencies.
    """
    k, grid = _convolve_h_w(C_pieces, t_sm, h_taps, dt)
    freqs = np.linspace(pb_max_hz / n_eval, pb_max_hz, n_eval)
    max_err = 0.0
    for f in freqs:
        omega = 2.0 * math.pi * f
        K = float(np.sum(k * np.cos(omega * grid)) * dt) + 1j * float(
            np.sum(k * (-np.sin(omega * grid))) * dt)
        err = abs(abs(K) - 1.0)
        if err > max_err:
            max_err = err
    return max_err


def fit_fused_kernel(C_pieces, t_sm, h_taps, t_h, dt,
                     n_pieces=9, degree=5):
    """Least-squares fit k_fused = h ⊛ w to equal-width piecewise polynomial.

    Numerically convolves h_taps with the piecewise forward kernel on a dense
    dt grid and fits each of n_pieces equal-width segments to an
    ascending-power-basis polynomial of the given degree. Convention matches
    shaper_defs.init_smoother: coefficient c_i multiplies tau^i in the
    GLOBAL time variable (not a piece-local variable).

    Args:
        C_pieces: piecewise forward kernel from
            shaper_defs.INPUT_SMOOTHERS[*].init_func.
        t_sm: forward kernel support width (seconds).
        h_taps: FIR inverse from compute_inverse_fir.
        t_h: inverse kernel support width (seconds).
        dt: tap spacing (must match compute_inverse_fir's dt).
        n_pieces: equal-width piece count (default 9, per Path C derivation).
        degree: polynomial degree per piece (default 5).

    Returns:
        List of (t_start, t_end, coeffs_ascending) tuples with
        len(coeffs_ascending) == degree + 1. Polynomial value on the piece
        is sum_i coeffs_ascending[i] * tau^i with tau in global coordinates.
    """
    if n_pieces <= 0:
        raise ValueError("n_pieces must be >= 1")
    if degree < 0:
        raise ValueError("degree must be >= 0")
    k_samples, grid = _convolve_h_w(C_pieces, t_sm, h_taps, dt)
    t_support = t_sm + t_h
    half_support = 0.5 * t_support
    # Clip to the fit support window; the numerical convolution's outer
    # samples are exactly zero (support of h ⊛ w is [-T/2, +T/2] where
    # T = T_sm + T_h) but grid length may round to an odd sample count.
    in_support = (grid >= -half_support - 1e-12) & \
                 (grid <= +half_support + 1e-12)
    t_all = grid[in_support]
    k_all = k_samples[in_support]
    piece_width = t_support / float(n_pieces)
    edges = -half_support + piece_width * np.arange(n_pieces + 1)
    # Snap the last edge so floating-point drift doesn't drop the final
    # sample out of the last piece.
    edges[-1] = +half_support
    pieces = []
    for p in range(n_pieces):
        lo = edges[p]
        hi = edges[p + 1]
        if p < n_pieces - 1:
            mask = (t_all >= lo) & (t_all < hi)
        else:
            mask = (t_all >= lo) & (t_all <= hi)
        t_local = t_all[mask]
        k_local = k_all[mask]
        if len(t_local) < degree + 1:
            raise ValueError(
                "fit_fused_kernel: piece %d has %d samples, "
                "need at least %d for degree-%d fit" %
                (p, len(t_local), degree + 1, degree))
        coeffs = np.polynomial.polynomial.polyfit(
            t_local, k_local, degree)
        pieces.append((float(lo), float(hi), [float(c) for c in coeffs]))
    return pieces


def _symmetric_monomial_exp_integral(k, hw, omega):
    """∫_{-hw}^{+hw} u^k exp(-i omega u) du for the symmetric piece variable u.

    Evaluated by analytically computing the real/imaginary parts from the
    cosine/sine integrals of u^k on a symmetric interval:

      ∫_{-hw}^{+hw} u^k cos(omega u) du  is 0 for odd k; for even k it is
        2 · ∫_0^{hw} u^k cos(omega u) du.
      ∫_{-hw}^{+hw} u^k sin(omega u) du  is 0 for even k; for odd k it is
        2 · ∫_0^{hw} u^k sin(omega u) du.

    Each one-sided integral is computed by the standard stable recursion

      C_k(hw, ω) = (hw^k · sin(ω hw) - k · S_{k-1}(hw, ω)) / ω
      S_k(hw, ω) = (-hw^k · cos(ω hw) + k · C_{k-1}(hw, ω)) / ω + k/ω · ...

    with omega = 0 handled as the elementary monomial limit.
    """
    if omega == 0.0:
        # ∫_{-hw}^{+hw} u^k du = (hw^{k+1} - (-hw)^{k+1}) / (k+1)
        if k % 2 == 1:
            return 0.0 + 0.0j
        return complex(2.0 * (hw ** (k + 1)) / (k + 1), 0.0)
    # Precompute one-sided integrals
    #   C_j = int_0^{hw} u^j cos(w u) du
    #   S_j = int_0^{hw} u^j sin(w u) du
    # via the recurrences:
    #   C_0 = sin(w hw) / w
    #   S_0 = (1 - cos(w hw)) / w
    #   C_j = (hw^j · sin(w hw)) / w - (j / w) · S_{j-1}
    #   S_j = (-hw^j · cos(w hw)) / w + (j / w) · C_{j-1} + j_0_term
    # Deriving by parts: d/du (u^j sin(wu)/w) = j u^{j-1} sin(wu)/w
    #                                          + u^j cos(wu)
    # So C_j = [u^j sin(wu)/w]_0^{hw} - (j/w) ∫ u^{j-1} sin(wu) du
    #        = hw^j sin(w hw)/w - (j/w) · S_{j-1}
    # And by parts: d/du (-u^j cos(wu)/w) = j u^{j-1} (-cos(wu)/w)
    #                                       + u^j sin(wu)
    # So S_j = [-u^j cos(wu)/w]_0^{hw} + (j/w) ∫ u^{j-1} cos(wu) du
    #        = -hw^j cos(w hw)/w + [0·cos(0)/w if j>0 else 0] + (j/w) C_{j-1}
    # The boundary at u=0: -0^j cos(0)/w = 0 for j >= 1; for j = 0 it equals
    # -1/w, which gives S_0 = (1 - cos(w hw))/w. Covered by the j=0 init.
    s_wh = math.sin(omega * hw)
    c_wh = math.cos(omega * hw)
    C = [0.0] * (k + 1)
    S = [0.0] * (k + 1)
    C[0] = s_wh / omega
    S[0] = (1.0 - c_wh) / omega
    hw_pow = 1.0  # hw^j; start at hw^0 = 1
    for j in range(1, k + 1):
        hw_pow *= hw  # now hw_pow = hw^j
        C[j] = hw_pow * s_wh / omega - (j / omega) * S[j - 1]
        S[j] = -hw_pow * c_wh / omega + (j / omega) * C[j - 1]
    # ∫_{-hw}^{+hw} u^k cos(w u) du = 2 C_k if k even else 0
    # ∫_{-hw}^{+hw} u^k sin(w u) du = 2 S_k if k odd else 0
    if k % 2 == 0:
        real = 2.0 * C[k]
        imag = 0.0
    else:
        real = 0.0
        imag = 2.0 * S[k]
    # Full integrand e^{-iωu} = cos(ωu) - i sin(ωu)
    # ⇒ ∫ u^k e^{-iωu} du = (cos integral) - i (sin integral)
    return complex(real, -imag)


def _poly_times_exp_integral(k, t_start, t_end, omega):
    """Closed-form ∫_{t_start}^{t_end} tau^k exp(-i omega tau) dtau.

    Implemented by shifting the integration variable to the piece midpoint
    u = tau - t_mid so |u| is bounded by W/2 (where W is the piece width),
    expanding (u + t_mid)^k via the binomial theorem into monomials in u,
    and then using _symmetric_monomial_exp_integral on each symmetric
    moment. The midpoint shift avoids catastrophic cancellation that
    plagues the direct integration-by-parts form at small omega × t_mid^k,
    which blows up quickly with k when t_mid has magnitude ≫ W/2 (typical
    for the outer pieces of a fused kernel centered at 0).
    """
    t_mid = 0.5 * (t_start + t_end)
    hw = 0.5 * (t_end - t_start)
    # Accumulate: (u + t_mid)^k = sum_{j=0..k} C(k, j) · t_mid^(k-j) · u^j
    acc = 0.0 + 0.0j
    for j in range(k + 1):
        binom = math.comb(k, j)
        acc += binom * (t_mid ** (k - j)) * \
               _symmetric_monomial_exp_integral(j, hw, omega)
    # Overall factor e^{-i omega t_mid}
    phase = np.exp(-1j * omega * t_mid)
    return phase * acc


def fit_fused_kernel_passband_error(fused_pieces, pb_max_hz, n_eval=500):
    """Closed-form FT of a fitted piecewise k_fused, checked on the passband.

    Evaluates max |FT(k_piecewise)(f) - 1| on (0, pb_max_hz] by analytical
    integration of the polynomial-times-exp kernel per piece. Consumers
    should prefer this over cascade_passband_error for regressing the stored
    fit: the C side will see exactly this piecewise form.

    Args:
        fused_pieces: list of (t_start, t_end, coeffs_ascending) tuples as
            returned by fit_fused_kernel. Coefficients are ascending powers
            of the GLOBAL time variable.
        pb_max_hz: passband upper bound.
        n_eval: evaluation frequency count (default 500).
    """
    freqs = np.linspace(pb_max_hz / n_eval, pb_max_hz, n_eval)
    max_err = 0.0
    for f in freqs:
        omega = 2.0 * math.pi * f
        K = 0.0 + 0.0j
        for (t_start, t_end, coeffs) in fused_pieces:
            for kk, c in enumerate(coeffs):
                K += c * _poly_times_exp_integral(kk, t_start, t_end, omega)
        err = abs(abs(K) - 1.0)
        if err > max_err:
            max_err = err
    return max_err
