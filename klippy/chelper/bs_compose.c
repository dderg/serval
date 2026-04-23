/* Plan 8 Chunk 2 — Analytical bs-kernel polynomial composer.
 *
 * See bs_compose.h for the public contract and docs/superpowers/plans/
 * plan8-research/bs_polynomial_composer.md for the mathematical derivation.
 *
 * Algorithm summary:
 *
 *   y(t) = integral over tau of x(t - tau) * w(tau) dtau
 *
 * Substitute u = t - tau (tau = t - u, dtau = -du):
 *
 *   y(t) = integral over u of x(u) * w(t - u) du
 *
 * x(u) is the input piecewise polynomial (up to 3 phases, each degree 10,
 * defined in absolute move-local time). w(tau) is the bs_m kernel, a
 * piecewise polynomial of degree (m-1) on (m+1) equal sub-intervals
 * spanning [-T_sm/2, +T_sm/2].
 *
 * The Minkowski sum of x's breakpoints and w's breakpoints in t yields
 * the breakpoints of y. Between any two consecutive breakpoints, for
 * each kernel piece i the u-support [t - b_i, t - a_i] lies entirely
 * within one input phase (of prev/current/next), so y is a single
 * polynomial in t over the sub-interval.
 *
 * Neighbour-aware boundary handling:
 *   The composer accepts optional prev / next move polynomials. When
 *   supplied, kernel-window overlap with u < 0 integrates the prev
 *   move's polynomial (origin-shifted to t in [-T_prev, 0)), and
 *   overlap with u > T_cur integrates the next move's polynomial
 *   (origin-shifted to t in [T_cur, T_cur + T_next)). When NULL, the
 *   corresponding overlap is zero-padded — correct when the actual
 *   print starts / stops at the move boundary.
 *
 * Implementation notes:
 *   - Input phase polynomials are converted from phase-local to
 *     absolute-move-local t by a Pascal shift per phase per axis.
 *     This is done once up front and cached in `abs_poly`. The prev
 *     / next polynomials are shifted into the same absolute-t frame
 *     as the current move by subtracting/adding T_prev / T_cur.
 *   - Kernel is evaluated in absolute-tau basis (as returned by the
 *     shaper_defs rescale path), but we rebuild it here in C from the
 *     closed-form cardinal B-spline formula so the composer has no
 *     FFI dependency on the Python side.
 *   - Output polynomial per sub-interval is kept in absolute-t monomial
 *     basis during assembly, then Pascal-shifted to piece-local at the
 *     end (the trapq polynomial layout is phase-local).
 *
 * Degree bookkeeping:
 *   - Input phase degree: 10 (coefficients c[0..10], with c[11..14] = 0
 *     in the current caller).
 *   - Kernel piece degree: m (the kernel is the (m+1)-fold self-convolution
 *     of a rectangle; piecewise-polynomial of degree m). Coefficients
 *     w_{i, 0..m}.
 *   - Per-piece integrand degree in u: 10 + m.
 *   - Output piece degree in t: 10 + m. For bs5 that is 15 — exactly the
 *     15-coeff slot width. Per the research note the top degree drops from
 *     the per-piece upper bound (2m + 10) to (m + 10) after kernel-piece
 *     cancellation, but here we simply size the arithmetic to the safe
 *     per-piece bound and rely on the cancellation to populate the highest
 *     slots with near-zero — then truncate at slot 14.
 *
 *   Empirical degree note: for bs1 (m = 1) the output on a constant-
 *   velocity input reproduces v * t exactly (degree 1). This is a useful
 *   regression guard that the composer handles the simplest case.
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "bs_compose.h"

#define BS_MAX_ORDER 5
/* Input polynomial max degree, inclusive. compose_phase_polynomials pads
 * to 15 coefficients; the natural quintic degree maxes at 10. */
#define BS_INPUT_MAX_DEG 10
#define BS_INPUT_NC (BS_INPUT_MAX_DEG + 1)   /* 11 coefficients used */
/* Kernel has m+1 pieces, each degree m (coefficients 0..m), so up to
 * 6 pieces / degree 5 / 6 coefficients per piece. */
#define BS_KERNEL_MAX_PIECES (BS_MAX_ORDER + 1)   /* bs5 -> 6 pieces */
#define BS_KERNEL_NC (BS_MAX_ORDER + 1)           /* coeffs per piece, m + 1 */
/* Output per-piece polynomial degree m + 9, i.e. up to 14 for bs5; leaves
 * room to 15 = MOVE_QUINTIC_POLY_COEFFS. */
#define BS_OUTPUT_NC 15
/* Max breakpoints. With neighbour-aware baking we enumerate breakpoints
 * from up to 3 moves (prev / current / next), each contributing at most
 * (n_phases + 1) edges Minkowski-summed against (m + 2) kernel edges.
 * Worst case: 3 moves * 4 edges * 7 kernel edges = 84 raw; bump the slack
 * to 256 for safety (neighbour tests may feed more phases). */
#define BS_MAX_BREAKS 256
/* Input phases per "move" (prev/current/next). The planner currently
 * emits up to 3 phases per move, but Plan 8 Chunk 2 composed outputs can
 * themselves serve as the next input — those can be up to MOVE_MAX_PIECES.
 * Keep headroom. */
#define BS_INPUT_MAX_PHASES 32
#define BS_TOTAL_MAX_PHASES (3 * BS_INPUT_MAX_PHASES)

/* F_m table (duplicated from klippy/extras/shaper_defs.py:_F_M_TABLE) so
 * the C composer is self-contained. Values derived at zeta=0.1, V=0.05
 * residual-vibration target. */
static const double F_M_TABLE[BS_MAX_ORDER + 1] = {
    0.0,    /* m = 0 unused */
    1.5553, /* bs1 */
    1.9462, /* bs2 */
    2.2519, /* bs3 */
    2.5061, /* bs4 */
    2.7252, /* bs5 */
};

/* ------------------------------------------------------------------ */
/* Pascal-retarget a polynomial between two origins.
 *   in : f(tau_in) = sum_k in[k] * tau_in^k,       tau_in  = t - origin_in
 *   out: f(tau_out) = sum_j out[j] * tau_out^j,    tau_out = t - origin_out
 * With D = origin_out - origin_in:
 *   tau_in = tau_out + D
 *   tau_in^k = sum_j C(k, j) * D^(k - j) * tau_out^j
 *   out[j]  = sum_{k >= j} in[k] * C(k, j) * D^(k - j)
 */
static void pascal_retarget(
    const double *in, double origin_in,
    double origin_out, int nc, double *out)
{
    double D = origin_out - origin_in;
    for (int j = 0; j < nc; ++j) {
        double acc = 0.0;
        double dpow = 1.0;  /* D^(k - j), starting at k = j */
        for (int k = j; k < nc; ++k) {
            double binom = 1.0;
            for (int r = 0; r < j; ++r)
                binom = binom * (double)(k - r) / (double)(r + 1);
            acc += in[k] * binom * dpow;
            dpow *= D;
        }
        out[j] = acc;
    }
}

/* Compute (-k)^(m - j) safely for small integer k, m, j. Explicit branches
 * because pow(0, 0) is 1 but some libcs fumble for negative base. */
static double int_pow_signed(int base, int exp)
{
    if (exp == 0)
        return 1.0;
    double r = 1.0;
    for (int i = 0; i < exp; ++i)
        r *= (double)base;
    return r;
}

/* Integer binomial C(n, k). */
static double binomial(int n, int k)
{
    if (k < 0 || k > n)
        return 0.0;
    if (k > n - k)
        k = n - k;
    double r = 1.0;
    for (int i = 0; i < k; ++i)
        r = r * (double)(n - i) / (double)(i + 1);
    return r;
}

/* Build the bs_m kernel in absolute-tau basis.
 * Output: n_pieces = m + 1; each piece i has support [tau_edges[i],
 * tau_edges[i+1]] and coefficients kernel_coeffs[i][0..m-1] in ascending
 * powers of absolute tau.
 */
static void build_bs_kernel(
    int m, double t_sm,
    double *tau_edges,        /* length m + 2 */
    double kernel_coeffs[BS_KERNEL_MAX_PIECES][BS_KERNEL_NC],
    int *n_pieces_out)
{
    double fac_m = 1.0;
    for (int i = 2; i <= m; ++i)
        fac_m *= (double)i;
    double canonical[BS_KERNEL_MAX_PIECES][BS_KERNEL_NC + 1];
    memset(canonical, 0, sizeof(canonical));
    for (int i = 0; i <= m; ++i) {
        for (int k = 0; k <= i; ++k) {
            double sign = (k & 1) ? -1.0 : 1.0;
            double binom_mp1_k = binomial(m + 1, k);
            for (int j = 0; j <= m; ++j) {
                double coef = sign * binom_mp1_k
                              * binomial(m, j)
                              * int_pow_signed(-k, m - j) / fac_m;
                canonical[i][j] += coef;
            }
        }
    }
    double s = (double)(m + 1) / t_sm;
    double shift = -0.5 * t_sm;
    double b0 = -s * shift;  /* = (m+1)/2 */
    for (int i = 0; i <= m; ++i) {
        double new_coeffs[BS_KERNEL_NC + 1];
        memset(new_coeffs, 0, sizeof(new_coeffs));
        for (int j = 0; j <= m; ++j) {
            if (canonical[i][j] == 0.0)
                continue;
            double s_pow = 1.0;
            double b0_pows[BS_KERNEL_NC + 1];
            b0_pows[j] = 1.0;
            for (int k = j - 1; k >= 0; --k)
                b0_pows[k] = b0_pows[k + 1] * b0;
            for (int k = 0; k <= j; ++k) {
                new_coeffs[k] += canonical[i][j] * binomial(j, k)
                                 * s_pow * b0_pows[k];
                s_pow *= s;
            }
        }
        for (int k = 0; k < BS_KERNEL_NC; ++k)
            kernel_coeffs[i][k] = s * new_coeffs[k];
    }
    double h = t_sm / (double)(m + 1);
    for (int i = 0; i <= m + 1; ++i)
        tau_edges[i] = -0.5 * t_sm + (double)i * h;
    *n_pieces_out = m + 1;
}

/* Comparison for qsort of doubles. */
static int cmp_dbl(const void *a, const void *b)
{
    double da = *(const double *)a;
    double db = *(const double *)b;
    if (da < db)
        return -1;
    if (da > db)
        return 1;
    return 0;
}

/* ------------------------------------------------------------------ */
/* Build the absolute-t polynomial array for a "concatenated" phase stream
 * covering prev / current / next moves (each optional). Input phase-local
 * polynomials are Pascal-retargeted into an absolute-t frame where t = 0
 * coincides with the START of the current move:
 *
 *   - prev  spans u in [-T_prev, 0)   with phase boundaries at T_prev_bnd
 *     shifted by -T_prev.
 *   - cur   spans u in [0, T_cur]     with phase boundaries unchanged.
 *   - next  spans u in [T_cur, T_cur + T_next] with phase boundaries
 *     shifted by +T_cur.
 *
 * Returns n_total_phases and writes:
 *   phase_starts[0..n_total_phases]   — n_total_phases + 1 boundary values
 *                                        in absolute-t; phase p spans
 *                                        [phase_starts[p], phase_starts[p+1]].
 *   abs_poly[p][k]                    — coefficient of t^k for phase p.
 */
static int flatten_axis_polys(
    int prev_n, const double *prev_ends, const double *prev_axis,
    double prev_T, int cur_n, const double *cur_ends,
    const double *cur_axis, int next_n, const double *next_ends,
    const double *next_axis, double next_T,
    double *phase_starts,
    double abs_poly[BS_TOTAL_MAX_PHASES][BS_INPUT_NC])
{
    int p_out = 0;
    /* Absolute shift for prev: its internal time 0 sits at t = -prev_T. */
    if (prev_n > 0 && prev_axis != NULL) {
        double shift = -prev_T;
        for (int p = 0; p < prev_n; ++p) {
            double start_rel = (p == 0) ? 0.0 : prev_ends[p - 1];
            double end_rel = prev_ends[p];
            phase_starts[p_out] = shift + start_rel;
            double in_buf[BS_OUTPUT_NC];
            for (int k = 0; k < BS_OUTPUT_NC; ++k)
                in_buf[k] = prev_axis[p * BS_OUTPUT_NC + k];
            double shifted[BS_INPUT_NC];
            /* Phase-local origin sits at shift + start_rel in absolute-t. */
            pascal_retarget(in_buf, shift + start_rel, 0.0, BS_INPUT_NC, shifted);
            for (int k = 0; k < BS_INPUT_NC; ++k)
                abs_poly[p_out][k] = shifted[k];
            p_out++;
            (void)end_rel;
        }
    }
    /* Current move. */
    for (int p = 0; p < cur_n; ++p) {
        double start_rel = (p == 0) ? 0.0 : cur_ends[p - 1];
        phase_starts[p_out] = start_rel;
        double in_buf[BS_OUTPUT_NC];
        for (int k = 0; k < BS_OUTPUT_NC; ++k)
            in_buf[k] = cur_axis[p * BS_OUTPUT_NC + k];
        double shifted[BS_INPUT_NC];
        pascal_retarget(in_buf, start_rel, 0.0, BS_INPUT_NC, shifted);
        for (int k = 0; k < BS_INPUT_NC; ++k)
            abs_poly[p_out][k] = shifted[k];
        p_out++;
    }
    /* Next move. */
    if (next_n > 0 && next_axis != NULL) {
        double shift = cur_ends[cur_n - 1];
        for (int p = 0; p < next_n; ++p) {
            double start_rel = (p == 0) ? 0.0 : next_ends[p - 1];
            phase_starts[p_out] = shift + start_rel;
            double in_buf[BS_OUTPUT_NC];
            for (int k = 0; k < BS_OUTPUT_NC; ++k)
                in_buf[k] = next_axis[p * BS_OUTPUT_NC + k];
            double shifted[BS_INPUT_NC];
            pascal_retarget(in_buf, shift + start_rel, 0.0, BS_INPUT_NC, shifted);
            for (int k = 0; k < BS_INPUT_NC; ++k)
                abs_poly[p_out][k] = shifted[k];
            p_out++;
        }
        (void)next_T;
    }
    /* Final phase-end sentinel. */
    double final_end;
    if (next_n > 0 && next_axis != NULL)
        final_end = cur_ends[cur_n - 1] + next_ends[next_n - 1];
    else
        final_end = cur_ends[cur_n - 1];
    phase_starts[p_out] = final_end;
    return p_out;
}

/* ------------------------------------------------------------------ */
/* Core single-axis composer. Called 3x from bs_compose (once per axis).
 *
 * Inputs:
 *   n_total      : number of flattened phases (prev + cur + next)
 *   phase_starts : length n_total + 1, absolute-t bounds of each phase
 *   abs_poly     : per-phase polynomial in absolute-t monomial basis
 *   m            : bs order 1..5
 *   t_sm         : kernel support width
 *   u_min, u_max : absolute-t integration domain (outside is zero-padded).
 *                  Typically u_min = -prev_T (or 0 if no prev) and
 *                  u_max = cur_T + next_T (or cur_T if no next).
 *   cur_move_t   : duration of the current move (for output phase shift)
 *   breaks       : sorted unique output breakpoints in absolute-t,
 *                  covering [0, cur_move_t]
 *   n_breaks     : length of breaks
 *
 * Outputs:
 *   out_coeffs   : per-phase 15 coefficients, phase-local [n_out * 15]
 */
static int compose_axis(
    int n_total,
    const double *phase_starts,
    double abs_poly[BS_TOTAL_MAX_PHASES][BS_INPUT_NC],
    int m,
    double t_sm,
    double u_min,
    double u_max,
    const double *breaks,
    int n_breaks,
    double *out_coeffs)
{
    /* Build the kernel. */
    double tau_edges[BS_KERNEL_MAX_PIECES + 1];
    double kernel_coeffs[BS_KERNEL_MAX_PIECES][BS_KERNEL_NC];
    int n_kpieces = 0;
    build_bs_kernel(m, t_sm, tau_edges, kernel_coeffs, &n_kpieces);

    int n_out = n_breaks - 1;
    for (int s_idx = 0; s_idx < n_out; ++s_idx) {
        double t_a = breaks[s_idx];
        double t_b = breaks[s_idx + 1];
        double t_mid = 0.5 * (t_a + t_b);
        double y_abs[BS_OUTPUT_NC];
        memset(y_abs, 0, sizeof(y_abs));

        for (int i = 0; i < n_kpieces; ++i) {
            double a_i = tau_edges[i];
            double b_i = tau_edges[i + 1];
            /* Unclipped u-range for this kernel piece at t_mid. */
            double u_hi_mid = t_mid - a_i;
            double u_lo_mid = t_mid - b_i;
            /* Clip to [u_min, u_max] (zero-pad outside the integration
             * domain). */
            double u_hi_cl = u_hi_mid;
            double u_lo_cl = u_lo_mid;
            if (u_hi_cl > u_max) u_hi_cl = u_max;
            if (u_lo_cl < u_min) u_lo_cl = u_min;
            if (u_hi_cl <= u_lo_cl)
                continue;
            /* Identify which input phase contains u_mid_clip. Within a
             * sub-interval bounded by the full break grid (all prev/
             * cur/next phase boundaries shifted by each kernel edge),
             * the clipped u-range lies in exactly one flattened phase. */
            double u_mid_clip = 0.5 * (u_hi_cl + u_lo_cl);
            int p = 0;
            while (p < n_total - 1 && u_mid_clip > phase_starts[p + 1])
                p++;
            /* Guard against round-off landing u_mid_clip just outside. */
            if (u_mid_clip < phase_starts[0] - 1e-15)
                continue;
            if (u_mid_clip > phase_starts[n_total] + 1e-15)
                continue;
            /* Build integration limits as linear polys in t (degree 1
             * when the edge is unclipped, degree 0 when clipped by the
             * domain bound). */
            double uh_p[2], ul_p[2];
            int uh_clipped_hi = (u_hi_mid > u_max);
            int ul_clipped_lo = (u_lo_mid < u_min);
            if (uh_clipped_hi) {
                uh_p[0] = u_max;
                uh_p[1] = 0.0;
            } else {
                uh_p[0] = -a_i;
                uh_p[1] = 1.0;
            }
            if (ul_clipped_lo) {
                ul_p[0] = u_min;
                ul_p[1] = 0.0;
            } else {
                ul_p[0] = -b_i;
                ul_p[1] = 1.0;
            }

            /* Pre-compute powers of uh_p and ul_p as polys in t up to
             * degree (BS_INPUT_MAX_DEG + m + 1). */
            int max_n = BS_INPUT_MAX_DEG + m;
            int max_pow = max_n + 1;
            double uh_pow[BS_INPUT_NC + BS_KERNEL_NC + 1][BS_OUTPUT_NC];
            double ul_pow[BS_INPUT_NC + BS_KERNEL_NC + 1][BS_OUTPUT_NC];
            memset(uh_pow, 0, sizeof(uh_pow));
            memset(ul_pow, 0, sizeof(ul_pow));
            uh_pow[0][0] = 1.0;
            ul_pow[0][0] = 1.0;
            for (int pw = 1; pw <= max_pow; ++pw) {
                for (int k = 0; k < BS_OUTPUT_NC; ++k) {
                    double a0 = uh_pow[pw - 1][k];
                    if (a0 == 0.0)
                        continue;
                    if (uh_p[0] != 0.0 && k < BS_OUTPUT_NC)
                        uh_pow[pw][k] += a0 * uh_p[0];
                    if (uh_p[1] != 0.0 && k + 1 < BS_OUTPUT_NC)
                        uh_pow[pw][k + 1] += a0 * uh_p[1];
                }
                for (int k = 0; k < BS_OUTPUT_NC; ++k) {
                    double a0 = ul_pow[pw - 1][k];
                    if (a0 == 0.0)
                        continue;
                    if (ul_p[0] != 0.0 && k < BS_OUTPUT_NC)
                        ul_pow[pw][k] += a0 * ul_p[0];
                    if (ul_p[1] != 0.0 && k + 1 < BS_OUTPUT_NC)
                        ul_pow[pw][k + 1] += a0 * ul_p[1];
                }
            }

            double X[BS_INPUT_NC];
            for (int k = 0; k < BS_INPUT_NC; ++k)
                X[k] = abs_poly[p][k];
            double w[BS_KERNEL_NC];
            for (int j = 0; j < BS_KERNEL_NC; ++j)
                w[j] = kernel_coeffs[i][j];

            int integrand_deg = BS_INPUT_MAX_DEG + m;
            for (int n = 0; n <= integrand_deg; ++n) {
                double P_n[BS_KERNEL_NC];
                memset(P_n, 0, sizeof(P_n));
                for (int k = 0; k <= n && k < BS_INPUT_NC; ++k) {
                    int l = n - k;
                    if (l < 0 || l > m)
                        continue;
                    double Xk = X[k];
                    if (Xk == 0.0)
                        continue;
                    double sign_l = (l & 1) ? -1.0 : 1.0;
                    for (int j = l; j <= m; ++j) {
                        double w_ij = w[j];
                        if (w_ij == 0.0)
                            continue;
                        int t_deg = j - l;
                        double term = Xk * w_ij * binomial(j, l) * sign_l;
                        P_n[t_deg] += term;
                    }
                }
                double inv_np1 = 1.0 / (double)(n + 1);
                for (int a = 0; a <= m; ++a) {
                    double pa = P_n[a];
                    if (pa == 0.0)
                        continue;
                    double scale = pa * inv_np1;
                    for (int b = 0; b < BS_OUTPUT_NC; ++b) {
                        double diff = uh_pow[n + 1][b] - ul_pow[n + 1][b];
                        if (diff == 0.0)
                            continue;
                        int c = a + b;
                        if (c >= BS_OUTPUT_NC)
                            continue;
                        y_abs[c] += scale * diff;
                    }
                }
            }
        }

        /* Shift from absolute-t to phase-local (origin = breaks[s_idx]). */
        double y_local[BS_OUTPUT_NC];
        pascal_retarget(y_abs, 0.0, breaks[s_idx],
                        BS_OUTPUT_NC, y_local);
        for (int k = 0; k < BS_OUTPUT_NC; ++k)
            out_coeffs[s_idx * BS_OUTPUT_NC + k] = y_local[k];
    }

    return 0;
}

/* ------------------------------------------------------------------ */
/* Public entry. See header for contract. */
int bs_compose(
    int prev_n_phases,
    const double *prev_phase_t_ends,
    const double *prev_coeffs,
    double prev_T_move,
    int n_input_phases,
    const double *input_phase_t_ends,
    const double *input_coeffs,
    int next_n_phases,
    const double *next_phase_t_ends,
    const double *next_coeffs,
    double next_T_move,
    int bs_order,
    double shaper_freq,
    double damping_ratio,
    int out_capacity,
    double *out_phase_t_ends,
    double *out_coeffs)
{
    (void)damping_ratio;  /* bs kernel is damping-independent */

    if (n_input_phases <= 0 || bs_order < 1 || bs_order > BS_MAX_ORDER)
        return -1;
    if (shaper_freq <= 0.0)
        return -1;
    double move_t = input_phase_t_ends[n_input_phases - 1];
    if (move_t <= 0.0)
        return -1;
    double t_sm = F_M_TABLE[bs_order] / shaper_freq;
    double h = t_sm / (double)(bs_order + 1);

    /* Resolve neighbour presence. Treat NULL arrays or non-positive
     * T_move as "no neighbour" (zero-pad outside). */
    int have_prev = (prev_n_phases > 0 && prev_phase_t_ends != NULL
                     && prev_coeffs != NULL && prev_T_move > 0.0);
    int have_next = (next_n_phases > 0 && next_phase_t_ends != NULL
                     && next_coeffs != NULL && next_T_move > 0.0);
    if (have_prev && prev_n_phases > BS_INPUT_MAX_PHASES)
        return -1;
    if (have_next && next_n_phases > BS_INPUT_MAX_PHASES)
        return -1;
    if (n_input_phases > BS_INPUT_MAX_PHASES)
        return -1;

    double u_min = have_prev ? -prev_T_move : 0.0;
    double u_max = have_next ? (move_t + next_T_move) : move_t;

    /* 1. Enumerate breakpoints in [0, move_t]: the output grid. The
     *    integrand changes whenever a kernel edge crosses any (prev /
     *    cur / next) phase boundary. For kernel edge i and phase boundary
     *    u = T_bnd, the break is t = T_bnd + tau_edge_i. We then clip
     *    to [0, move_t] and keep the open-interior breaks. */
    double raw_breaks[BS_MAX_BREAKS];
    int n_raw = 0;
    raw_breaks[n_raw++] = 0.0;
    raw_breaks[n_raw++] = move_t;
    /* Collect all flattened phase boundaries in absolute-t. */
    double phase_bnds[BS_MAX_BREAKS];
    int n_bnd = 0;
    if (have_prev) {
        phase_bnds[n_bnd++] = -prev_T_move;   /* prev start */
        for (int p = 0; p < prev_n_phases; ++p)
            phase_bnds[n_bnd++] = -prev_T_move + prev_phase_t_ends[p];
    }
    /* Always include current move start (which may coincide with prev end). */
    phase_bnds[n_bnd++] = 0.0;
    for (int p = 0; p < n_input_phases; ++p)
        phase_bnds[n_bnd++] = input_phase_t_ends[p];
    if (have_next) {
        for (int p = 0; p < next_n_phases; ++p)
            phase_bnds[n_bnd++] = move_t + next_phase_t_ends[p];
    }
    for (int p = 0; p < n_bnd; ++p) {
        for (int i = 0; i <= bs_order + 1; ++i) {
            double t_break = phase_bnds[p]
                             + (-0.5 * t_sm + (double)i * h);
            if (t_break > 0.0 && t_break < move_t) {
                if (n_raw >= BS_MAX_BREAKS)
                    return -1;
                raw_breaks[n_raw++] = t_break;
            }
        }
    }
    qsort(raw_breaks, n_raw, sizeof(double), cmp_dbl);
    double tol = 1e-12 * (move_t + 1.0);
    double uniq_breaks[BS_MAX_BREAKS];
    int n_uniq = 0;
    for (int i = 0; i < n_raw; ++i) {
        if (n_uniq == 0 || raw_breaks[i] - uniq_breaks[n_uniq - 1] > tol) {
            uniq_breaks[n_uniq++] = raw_breaks[i];
        }
    }
    int n_out_phases = n_uniq - 1;
    if (n_out_phases <= 0 || n_out_phases > out_capacity)
        return -1;

    /* 2. Per-axis compose. The coeff buffer layout is interleaved
     *    (x, y, z, e) per phase per coefficient index. We compose XY
     *    (axes 0..2); the .e slot is zeroed (populated downstream by
     *    linear_pa_compose). */
    for (int s = 0; s < n_out_phases; ++s) {
        for (int k = 0; k < BS_OUTPUT_NC; ++k) {
            out_coeffs[(s * BS_OUTPUT_NC + k) * 4 + 3] = 0.0;
        }
    }
    for (int axis = 0; axis < 3; ++axis) {
        double in_prev_axis[BS_INPUT_MAX_PHASES * BS_OUTPUT_NC];
        double in_cur_axis[BS_INPUT_MAX_PHASES * BS_OUTPUT_NC];
        double in_next_axis[BS_INPUT_MAX_PHASES * BS_OUTPUT_NC];
        if (have_prev) {
            for (int p = 0; p < prev_n_phases; ++p) {
                for (int k = 0; k < BS_OUTPUT_NC; ++k) {
                    in_prev_axis[p * BS_OUTPUT_NC + k] =
                        prev_coeffs[(p * BS_OUTPUT_NC + k) * 4 + axis];
                }
            }
        }
        for (int p = 0; p < n_input_phases; ++p) {
            for (int k = 0; k < BS_OUTPUT_NC; ++k) {
                in_cur_axis[p * BS_OUTPUT_NC + k] =
                    input_coeffs[(p * BS_OUTPUT_NC + k) * 4 + axis];
            }
        }
        if (have_next) {
            for (int p = 0; p < next_n_phases; ++p) {
                for (int k = 0; k < BS_OUTPUT_NC; ++k) {
                    in_next_axis[p * BS_OUTPUT_NC + k] =
                        next_coeffs[(p * BS_OUTPUT_NC + k) * 4 + axis];
                }
            }
        }

        double abs_poly[BS_TOTAL_MAX_PHASES][BS_INPUT_NC];
        double phase_starts[BS_TOTAL_MAX_PHASES + 1];
        int n_total = flatten_axis_polys(
            have_prev ? prev_n_phases : 0,
            have_prev ? prev_phase_t_ends : NULL,
            have_prev ? in_prev_axis : NULL,
            have_prev ? prev_T_move : 0.0,
            n_input_phases, input_phase_t_ends, in_cur_axis,
            have_next ? next_n_phases : 0,
            have_next ? next_phase_t_ends : NULL,
            have_next ? in_next_axis : NULL,
            have_next ? next_T_move : 0.0,
            phase_starts, abs_poly
        );
        if (n_total <= 0)
            return -1;

        double out_axis[BS_MAX_BREAKS * BS_OUTPUT_NC];
        int rc = compose_axis(
            n_total, phase_starts, abs_poly,
            bs_order, t_sm, u_min, u_max,
            uniq_breaks, n_uniq,
            out_axis
        );
        if (rc != 0)
            return -1;
        for (int s = 0; s < n_out_phases; ++s) {
            for (int k = 0; k < BS_OUTPUT_NC; ++k) {
                out_coeffs[(s * BS_OUTPUT_NC + k) * 4 + axis] =
                    out_axis[s * BS_OUTPUT_NC + k];
            }
        }
    }

    for (int s = 0; s < n_out_phases; ++s)
        out_phase_t_ends[s] = uniq_breaks[s + 1];

    return n_out_phases;
}
