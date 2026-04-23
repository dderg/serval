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
 *   - Output number of pieces: up to (n_input_phases + 1) * N + neighbour
 *     phase-boundary crossings, bounded by out_capacity.
 *   - Impulse amplitudes are NOT renormalized here; the caller is
 *     responsible for passing a pre-normalized amplitude vector such that
 *     sum(a_i) == 1.
 *
 * Neighbour polynomials:
 *   The composer accepts optional prev / next move polynomials for
 *   kernel-window integration across move boundaries. Pass the UNSHAPED
 *   neighbour polynomials (direct output of compose_phase_polynomials).
 *   Passing already-shaped neighbours convolves against baked motion
 *   (double-baking — wrong). When NULL the corresponding side zero-pads.
 *
 * Returns number of output phases on success (>= 1), or -1 on overflow.
 */
int fir_compose(
    /* Previous move (or NULL / 0 / 0.0 for "no prev"). */
    int prev_n_phases,
    const double *prev_phase_t_ends,    /* length prev_n_phases, or NULL */
    const double *prev_coeffs,          /* prev_n_phases * 15 * 4, or NULL */
    double prev_T_move,                 /* duration of previous move */
    /* Current move. */
    int n_input_phases,
    const double *input_phase_t_ends,   /* length n_input_phases */
    const double *input_coeffs,         /* n_input_phases * 15 * 4 doubles */
    /* Next move (or NULL / 0 / 0.0). */
    int next_n_phases,
    const double *next_phase_t_ends,    /* length next_n_phases, or NULL */
    const double *next_coeffs,          /* next_n_phases * 15 * 4, or NULL */
    double next_T_move,                 /* duration of next move */
    /* Shaper impulse train. */
    int n_impulses,                     /* 2 (zv) or 3 (mzv) */
    const double *impulse_amplitudes,   /* length n_impulses */
    const double *impulse_delays,       /* length n_impulses */
    /* Output. */
    int out_capacity,                   /* caller buffer size (phases) */
    double *out_phase_t_ends,           /* length out_capacity */
    double *out_coeffs                  /* out_capacity * 15 * 4 doubles */
);

#endif /* FIR_COMPOSE_H */
