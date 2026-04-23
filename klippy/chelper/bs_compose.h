#ifndef BS_COMPOSE_H
#define BS_COMPOSE_H

/* Plan 8 Chunk 2 — Analytical bs-kernel polynomial composer.
 *
 * Given a move's per-axis phase polynomials (as produced by
 * QuinticShape.compose_phase_polynomials — position(t) per phase in phase-
 * local time, up to 15 coeffs per phase per axis), convolve each with the
 * cardinal B-spline kernel bs_m (m in 1..5, see
 * klippy/extras/shaper_defs.py:_cardinal_bspline_pieces) and write the
 * resulting piecewise polynomial into the output buffer, ready to feed
 * trapq_append_quintic.
 *
 * Math derivation: docs/superpowers/plans/plan8-research/
 *                  bs_polynomial_composer.md
 *
 * Output pieces:
 *   - Up to (P_prev + P_cur + P_next + 1) * (m + 2) sub-intervals in the
 *     neighbour-aware path (bs5 3-phase moves: (9 + 1) * 7 = 70 worst,
 *     but clipped to [0, move_t] typically yields far fewer). Capped at
 *     MOVE_MAX_PIECES by the caller-supplied out_capacity.
 *   - Each output piece is a polynomial of degree (m + 9) in phase-local
 *     time, fitted into the 15-coefficient slot. Coefficients above
 *     (m + 9) are written as zero.
 *
 * Neighbour polynomials:
 *   The composer accepts optional previous and next move polynomials to
 *   integrate the kernel across move boundaries. For continuous prints
 *   this is required to recover the exact shape at [0, T_sm/2] and
 *   [T_move - T_sm/2, T_move] of every non-boundary move. When NULL
 *   the corresponding side zero-pads (correct when the actual print
 *   starts / stops at the move boundary).
 *
 *   Important: pass the UNSHAPED neighbour polynomials (the direct output
 *   of QuinticShape.compose_phase_polynomials). Passing already-shaped
 *   neighbours convolves against baked motion — double-baking — which is
 *   wrong.
 *
 * Returns number of output phases on success (>= 1), or -1 on overflow
 * (out_capacity exceeded — caller should widen MOVE_MAX_PIECES or pick
 * a smaller bs order).
 */
int bs_compose(
    /* Previous move (or NULL arrays / 0 / 0.0 for "no prev"). */
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
    /* Kernel. */
    int bs_order,                       /* 1..5 */
    double shaper_freq,                 /* Hz, > 0 */
    double damping_ratio,               /* ignored — bs kernel is damping-independent */
    /* Output. */
    int out_capacity,                   /* caller buffer size (phases) */
    double *out_phase_t_ends,           /* length out_capacity */
    double *out_coeffs                  /* out_capacity * 15 * 4 doubles; .e zeroed */
);

#endif /* BS_COMPOSE_H */
