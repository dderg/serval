#ifndef FIR_COMPOSE_H
#define FIR_COMPOSE_H

/* Plan 8 Chunk 2 — FIR impulse-train polynomial composer (zv / mzv).
 *
 * Given a move's per-axis phase polynomials (as produced by
 * QuinticShape.compose_phase_polynomials — position(t) per phase in phase-
 * local time, up to 15 coeffs per phase per axis), apply an input-shaping
 * FIR kernel of N impulses:
 *
 *     y(t) = sum_{i=0..N-1} a_i * x(t - tau_i)
 *
 * The shaped move emits N time-shifted copies of the input piecewise
 * polynomial, each scaled by a_i, summed on a common time grid built from
 * the Minkowski sum { phase_boundary + tau_i : p, i }.
 *
 * Mathematical properties:
 *   - Output polynomial degree per piece: same as input (no degree bump).
 *   - Output number of pieces: up to (n_input_phases + 1) * N, bounded by
 *     out_capacity.
 *   - Impulse amplitudes are NOT renormalized here; the caller is
 *     responsible for passing a pre-normalized amplitude vector such that
 *     sum(a_i) == 1. (For get_zv_shaper / get_mzv_shaper the raw
 *     amplitudes returned from shaper_defs are NOT unit-sum; this
 *     composer treats them verbatim, so the Python wrapper normalizes
 *     before calling.)
 *
 * Boundary handling (Chunk 2 limitation, matches bs_compose):
 *   Time outside the input move [0, move_t] is zero-padded — the shifted
 *   copy at delay tau_i contributes zero for t - tau_i < 0 or
 *   t - tau_i > move_t. This slightly truncates the first/last kernel
 *   support of a print sequence. See bs_compose.h for the rationale and
 *   planned follow-up.
 *
 * Returns number of output phases on success (>= 1), or -1 on overflow.
 */
int fir_compose(
    int n_input_phases,
    const double *input_phase_t_ends,   /* length n_input_phases */
    const double *input_coeffs,         /* n_input_phases * 15 * 3 doubles */
    int n_impulses,                     /* 2 (zv) or 3 (mzv) */
    const double *impulse_amplitudes,   /* length n_impulses */
    const double *impulse_delays,       /* length n_impulses */
    int out_capacity,                   /* caller buffer size (phases) */
    double *out_phase_t_ends,           /* length out_capacity */
    double *out_coeffs                  /* out_capacity * 15 * 3 doubles */
);

#endif /* FIR_COMPOSE_H */
