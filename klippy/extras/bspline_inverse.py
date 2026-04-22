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
#      f_sh is recovered from pb_max_hz via the section 4.3 convention
#      pb_max = 0.3 * f_sh. The Tikhonov-only variant from
#      fir_companion_kernel.md section 1 is documented there as failing on
#      this kernel family (out-of-band |W| near zero gives H huge HF gain
#      and G = ||h||_1 on the order of 10^2); the hard bandlimit is what
#      keeps G near 2.
#   4. IFFT back, truncate to T_h = T_h_ratio * T_sm (odd length, center
#      tap at tau = 0), Tukey-window, and renormalize so integral of h = 1.
#
# Consumed by Task 5 (Path C least-squares fit of the fused cascade h * w
# to a 9-piece degree-5 polynomial for C-side storage). Not yet wired into
# input_shaper.py.
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


def compute_inverse_fir(C_pieces, t_sm, pb_max_hz, dt=1e-5,
                        T_h_ratio=2.0, tukey_alpha=0.25,
                        eps_rel=3e-3):
    """FIR inverse of a piecewise-polynomial forward kernel.

    Cosine-taper + hard-bandlimit IFFT design per new_shaper_family.md
    section 10. Returns (h, T_h, dt) with h an odd-length 1-D numpy array
    of FIR taps centered at tau = 0.

    Args:
        C_pieces: list of (t_start, t_end, coeffs) as produced by
            shaper_defs.INPUT_SMOOTHERS[*].init_func.
        t_sm: forward kernel support width.
        pb_max_hz: passband upper bound. f_sh is recovered as
            pb_max_hz / 0.3 to match the section 4.3 reference table.
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
    f_sh = pb_max_hz / 0.3
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
