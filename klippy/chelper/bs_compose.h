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
 *   - Up to P_in * (m + 2) sub-intervals (bs5 worst case with 3 input
 *     phases: 3 * 7 = 21, plus boundary fragments — capped at
 *     MOVE_MAX_PIECES by the caller-supplied out_capacity).
 *   - Each output piece is a polynomial of degree (m + 9) in phase-local
 *     time, fitted into the 15-coefficient slot. Coefficients above
 *     (m + 9) are written as zero.
 *
 * Boundary simplification (Chunk 2 limitation):
 *   The composer operates on a SINGLE move in isolation. Time outside
 *   [0, move_t] is treated as zero-padded (the convolution integrates
 *   only the part of the kernel support that overlaps [0, move_t]).
 *   This gives a slightly wrong shape within (T_sm/2) of the FIRST and
 *   LAST moves of a print — a transient startup/shutdown artifact.
 *   Most prints spend almost all their time mid-sequence where the
 *   approximation is exact (the kernel tails into the current move's
 *   own boundary-velocity plateaus, which the zero-pad truncates but
 *   the plateau self-continuation of the polynomial covers).
 *   A follow-up task will add proper neighbour-aware baking using the
 *   lookahead flush window (docs/.../lookahead_window.md: 250 ms is
 *   plenty for T_sm up to ~90 ms).
 *
 * Returns number of output phases on success (>= 1), or -1 on overflow
 * (out_capacity exceeded — caller should widen MOVE_MAX_PIECES or pick
 * a smaller bs order).
 */
int bs_compose(
    int n_input_phases,
    const double *input_phase_t_ends,   /* length n_input_phases */
    const double *input_coeffs,         /* n_input_phases * 15 * 4 doubles */
    int bs_order,                       /* 1..5 */
    double shaper_freq,                 /* Hz, > 0 */
    double damping_ratio,               /* ignored — bs kernel is damping-independent */
    int out_capacity,                   /* caller buffer size (phases) */
    double *out_phase_t_ends,           /* length out_capacity */
    double *out_coeffs                  /* out_capacity * 15 * 4 doubles; .e zeroed */
);

#endif /* BS_COMPOSE_H */
