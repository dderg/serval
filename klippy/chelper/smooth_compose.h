#ifndef SMOOTH_COMPOSE_H
#define SMOOTH_COMPOSE_H

/* Generic piecewise-polynomial kernel composer.
 *
 * Restoration of the pre-Plan-5 smooth-IS family (smooth_zv, smooth_mzv,
 * smooth_ei, smooth_2hump_ei, smooth_zvd_ei, smooth_si) alongside the
 * cardinal B-spline chain (bs1..bs5). Both families now share the same
 * baked-planner path; the smooth-IS variants differ from bs only in
 * kernel shape (single polynomial piece vs. (m+1)-piece cardinal
 * B-spline chain).
 *
 * Unlike bs_compose, which builds its kernel internally from a single
 * integer `bs_order`, this composer accepts the kernel as an externally-
 * supplied piecewise polynomial over [-T_sm/2, +T_sm/2]. The caller is
 * responsible for computing piece coefficients (typically via
 * `klippy/extras/shaper_defs.py::INPUT_SMOOTHERS[...].init_func`).
 *
 * Precision caveat for smooth-IS kernels
 * --------------------------------------
 * The smooth-IS unit-integral-normalized kernels carry power-basis
 * coefficients that grow as 1/t_sm^(k+1); at typical shaper_freq = 40 Hz
 * this gives O(1e14) per-piece coefficients with heavy cross-
 * cancellation across the degree-6 polynomial. Folded through the
 * composer's 15-slot output-polynomial algebra, this leaves O(1e-3)
 * residual drift in the output coefficients that should analytically
 * be zero. Evaluated at phase-local t ~ 0.4 s on a 500 mm/s cruise
 * the drift is O(0.5 mm).
 *
 * In practice QuinticBlendMove spans are 5-50 ms so the drift is
 * sub-nanometer at operational move durations. Long straight cruises
 * bypass this composer entirely (trapq_append handles them directly).
 *
 * A future stability refactor (tracked as a follow-up) should move the
 * smooth-IS kernel representation to a Bernstein or centered-Chebyshev
 * basis to avoid the heavy power-basis cancellation at large shaper
 * frequencies.
 *
 * Kernel layout (flat arrays, length `kernel_n_pieces`):
 *   kernel_piece_starts[i], kernel_piece_ends[i] — absolute-tau edges
 *     of piece i. Pieces must be contiguous, sorted, and cover
 *     [-t_sm/2, +t_sm/2] exactly.
 *   kernel_piece_coeffs[i * SMOOTH_KERNEL_MAX_NC + k] — coefficient of
 *     tau^k for piece i, in ASCENDING power basis (matches
 *     shaper_defs.py piece format). Trailing slots above the actual
 *     piece degree must be zero.
 *
 * Other arguments + output layout mirror bs_compose.
 *
 * Returns number of output phases on success (>= 1), or -1 on overflow
 * / bad args.
 */

/* Max coefficients per kernel piece. Sized for smooth-IS kernels that
 * run up to degree 8 (smooth_2hump_ei / smooth_si / smooth_zvd_ei). */
#define SMOOTH_KERNEL_MAX_NC 9
/* Max kernel pieces. smooth-IS kernels have 1 piece; bs5 has 6. Keep a
 * larger pad for any future piecewise shape. */
#define SMOOTH_KERNEL_MAX_PIECES 16

int smooth_compose(
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
    /* Kernel pieces (ascending-power, one flat buffer). */
    int kernel_n_pieces,
    const double *kernel_piece_starts,  /* length kernel_n_pieces */
    const double *kernel_piece_ends,    /* length kernel_n_pieces */
    const double *kernel_piece_coeffs,  /* kernel_n_pieces * SMOOTH_KERNEL_MAX_NC */
    double t_sm,                        /* total kernel support, > 0 */
    /* Output. */
    int out_capacity,                   /* caller buffer size (phases) */
    double *out_phase_t_ends,           /* length out_capacity */
    double *out_coeffs                  /* out_capacity * 15 * 4 doubles; .e zeroed */
);

#endif /* SMOOTH_COMPOSE_H */
