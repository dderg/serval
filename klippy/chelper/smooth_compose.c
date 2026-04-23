/* Generic piecewise-polynomial kernel composer.
 *
 * See smooth_compose.h for the public contract. The algorithm is
 * identical to bs_compose.c — Minkowski-sum breakpoint enumeration,
 * per-piece polynomial integration on each sub-interval, Pascal
 * retargeting from absolute-t back to phase-local — with the kernel
 * pieces supplied by the caller instead of being computed from a
 * single `bs_order` integer.
 *
 * Primary caller: the smooth-IS family (smooth_zv / smooth_mzv /
 * smooth_ei / smooth_2hump_ei / smooth_zvd_ei / smooth_si), which
 * ships a single-piece kernel of degree up to 8 over the centered
 * window [-t_sm/2, +t_sm/2]. The composer is written to accept any
 * piecewise-polynomial kernel with up to SMOOTH_KERNEL_MAX_NC - 1 = 8
 * degree per piece, so future kernel shapes can reuse it.
 *
 * The bs composer (bs_compose.c) builds its kernel internally and
 * stays on the degree-5 path to keep its coefficient arithmetic
 * minimal; this composer trades a slightly larger inner loop for
 * kernel generality.
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "compiler.h" // __visible
#include "smooth_compose.h"

/* Input polynomial max degree, inclusive. compose_phase_polynomials pads
 * to 15 coefficients; the natural quintic degree maxes at 10. */
#define SC_INPUT_MAX_DEG 10
#define SC_INPUT_NC (SC_INPUT_MAX_DEG + 1)    /* 11 coefficients used */
/* Output per-piece polynomial: input * kernel integrated -> degree up to
 * SC_INPUT_MAX_DEG + max_kernel_deg = 10 + 8 = 18 for smooth_2hump_ei on
 * a fully-populated quintic phase. We truncate at 15 slots
 * (MOVE_QUINTIC_POLY_COEFFS); callers that feed typical QuinticBlendMove
 * outputs have actual input degree 5, so effective output degree is 13,
 * comfortably within the slot width. */
#define SC_OUTPUT_NC 15
/* Worst-case breakpoint count (see bs_compose.c for the same bound). */
#define SC_MAX_BREAKS 256
/* Input phases per "move" (prev / current / next). */
#define SC_INPUT_MAX_PHASES 32
#define SC_TOTAL_MAX_PHASES (3 * SC_INPUT_MAX_PHASES)
/* Powers buffer: n runs up to SC_INPUT_MAX_DEG + max_kdeg so its length
 * must cover max_n + 1 = 20. */
#define SC_MAX_N_PLUS_ONE (SC_INPUT_MAX_DEG + SMOOTH_KERNEL_MAX_NC + 1)

/* ------------------------------------------------------------------ */
/* Pascal-retarget a polynomial between two origins. See bs_compose.c
 * for the derivation; this is a verbatim copy. */
static void pascal_retarget_sc(
    const double *in, double origin_in,
    double origin_out, int nc, double *out)
{
    double D = origin_out - origin_in;
    for (int j = 0; j < nc; ++j) {
        double acc = 0.0;
        double dpow = 1.0;
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

/* Integer binomial C(n, k). */
static double binomial_sc(int n, int k)
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

/* Comparison for qsort of doubles. */
static int cmp_dbl_sc(const void *a, const void *b)
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
/* Flatten prev / cur / next per-axis phase polynomials into a single
 * absolute-t phase array. Verbatim port from bs_compose.c. */
static int flatten_axis_polys_sc(
    int prev_n, const double *prev_ends, const double *prev_axis,
    double prev_T, int cur_n, const double *cur_ends,
    const double *cur_axis, int next_n, const double *next_ends,
    const double *next_axis, double next_T,
    double *phase_starts,
    double abs_poly[SC_TOTAL_MAX_PHASES][SC_INPUT_NC])
{
    int p_out = 0;
    if (prev_n > 0 && prev_axis != NULL) {
        double shift = -prev_T;
        for (int p = 0; p < prev_n; ++p) {
            double start_rel = (p == 0) ? 0.0 : prev_ends[p - 1];
            double end_rel = prev_ends[p];
            phase_starts[p_out] = shift + start_rel;
            double in_buf[SC_OUTPUT_NC];
            for (int k = 0; k < SC_OUTPUT_NC; ++k)
                in_buf[k] = prev_axis[p * SC_OUTPUT_NC + k];
            double shifted[SC_INPUT_NC];
            pascal_retarget_sc(in_buf, shift + start_rel, 0.0,
                               SC_INPUT_NC, shifted);
            for (int k = 0; k < SC_INPUT_NC; ++k)
                abs_poly[p_out][k] = shifted[k];
            p_out++;
            (void)end_rel;
        }
    }
    for (int p = 0; p < cur_n; ++p) {
        double start_rel = (p == 0) ? 0.0 : cur_ends[p - 1];
        phase_starts[p_out] = start_rel;
        double in_buf[SC_OUTPUT_NC];
        for (int k = 0; k < SC_OUTPUT_NC; ++k)
            in_buf[k] = cur_axis[p * SC_OUTPUT_NC + k];
        double shifted[SC_INPUT_NC];
        pascal_retarget_sc(in_buf, start_rel, 0.0, SC_INPUT_NC, shifted);
        for (int k = 0; k < SC_INPUT_NC; ++k)
            abs_poly[p_out][k] = shifted[k];
        p_out++;
    }
    if (next_n > 0 && next_axis != NULL) {
        double shift = cur_ends[cur_n - 1];
        for (int p = 0; p < next_n; ++p) {
            double start_rel = (p == 0) ? 0.0 : next_ends[p - 1];
            phase_starts[p_out] = shift + start_rel;
            double in_buf[SC_OUTPUT_NC];
            for (int k = 0; k < SC_OUTPUT_NC; ++k)
                in_buf[k] = next_axis[p * SC_OUTPUT_NC + k];
            double shifted[SC_INPUT_NC];
            pascal_retarget_sc(in_buf, shift + start_rel, 0.0,
                               SC_INPUT_NC, shifted);
            for (int k = 0; k < SC_INPUT_NC; ++k)
                abs_poly[p_out][k] = shifted[k];
            p_out++;
        }
        (void)next_T;
    }
    double final_end;
    if (next_n > 0 && next_axis != NULL)
        final_end = cur_ends[cur_n - 1] + next_ends[next_n - 1];
    else
        final_end = cur_ends[cur_n - 1];
    phase_starts[p_out] = final_end;
    return p_out;
}

/* ------------------------------------------------------------------ */
/* Single-axis composer.
 *
 * Uses the caller-supplied kernel pieces directly. `max_kdeg` is the
 * largest kernel-piece degree across all pieces (used to size the
 * inner integrand loop and the powers buffer).
 */
static int compose_axis_sc(
    int n_total,
    const double *phase_starts,
    double abs_poly[SC_TOTAL_MAX_PHASES][SC_INPUT_NC],
    int n_kpieces,
    const double *tau_edges,          /* length n_kpieces + 1 */
    const double kernel_coeffs[SMOOTH_KERNEL_MAX_PIECES][SMOOTH_KERNEL_MAX_NC],
    int max_kdeg,
    double u_min,
    double u_max,
    const double *breaks,
    int n_breaks,
    double *out_coeffs)
{
    int n_out = n_breaks - 1;
    int integrand_deg = SC_INPUT_MAX_DEG + max_kdeg;
    int max_pow = integrand_deg + 1;
    if (max_pow + 1 > SC_MAX_N_PLUS_ONE)
        return -1;

    for (int s_idx = 0; s_idx < n_out; ++s_idx) {
        double t_a = breaks[s_idx];
        double t_b = breaks[s_idx + 1];
        double t_mid = 0.5 * (t_a + t_b);
        double y_abs[SC_OUTPUT_NC];
        memset(y_abs, 0, sizeof(y_abs));

        for (int i = 0; i < n_kpieces; ++i) {
            double a_i = tau_edges[i];
            double b_i = tau_edges[i + 1];
            double u_hi_mid = t_mid - a_i;
            double u_lo_mid = t_mid - b_i;
            double u_hi_cl = u_hi_mid;
            double u_lo_cl = u_lo_mid;
            if (u_hi_cl > u_max) u_hi_cl = u_max;
            if (u_lo_cl < u_min) u_lo_cl = u_min;
            if (u_hi_cl <= u_lo_cl)
                continue;
            double u_mid_clip = 0.5 * (u_hi_cl + u_lo_cl);
            int p = 0;
            while (p < n_total - 1 && u_mid_clip > phase_starts[p + 1])
                p++;
            if (u_mid_clip < phase_starts[0] - 1e-15)
                continue;
            if (u_mid_clip > phase_starts[n_total] + 1e-15)
                continue;

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

            /* Powers of uh_p and ul_p as polys in t. */
            double uh_pow[SC_MAX_N_PLUS_ONE][SC_OUTPUT_NC];
            double ul_pow[SC_MAX_N_PLUS_ONE][SC_OUTPUT_NC];
            memset(uh_pow, 0, sizeof(uh_pow));
            memset(ul_pow, 0, sizeof(ul_pow));
            uh_pow[0][0] = 1.0;
            ul_pow[0][0] = 1.0;
            for (int pw = 1; pw <= max_pow; ++pw) {
                for (int k = 0; k < SC_OUTPUT_NC; ++k) {
                    double a0 = uh_pow[pw - 1][k];
                    if (a0 == 0.0)
                        continue;
                    if (uh_p[0] != 0.0 && k < SC_OUTPUT_NC)
                        uh_pow[pw][k] += a0 * uh_p[0];
                    if (uh_p[1] != 0.0 && k + 1 < SC_OUTPUT_NC)
                        uh_pow[pw][k + 1] += a0 * uh_p[1];
                }
                for (int k = 0; k < SC_OUTPUT_NC; ++k) {
                    double a0 = ul_pow[pw - 1][k];
                    if (a0 == 0.0)
                        continue;
                    if (ul_p[0] != 0.0 && k < SC_OUTPUT_NC)
                        ul_pow[pw][k] += a0 * ul_p[0];
                    if (ul_p[1] != 0.0 && k + 1 < SC_OUTPUT_NC)
                        ul_pow[pw][k + 1] += a0 * ul_p[1];
                }
            }

            double X[SC_INPUT_NC];
            for (int k = 0; k < SC_INPUT_NC; ++k)
                X[k] = abs_poly[p][k];
            /* Integrand coefficient collection.
             *
             *   f(u, t) = X(u) * w(t - u)
             *   X(u) = sum_k X[k] * u^k
             *   w(tau) with tau = t - u expands binomially in u:
             *     (t - u)^j = sum_{l=0..j} C(j, l) * t^(j-l) * (-u)^l
             *   so each (X[k] * u^k) * (w[j] * (t - u)^j) contributes
             *     X[k] * w[j] * C(j, l) * (-1)^l * u^(k + l) * t^(j - l)
             *   Integrating over u from u_lo to u_hi gives
             *     [u^(n + 1) / (n + 1)] where n = k + l, with u_hi and
             *     u_lo each being polynomials in t so raising to power
             *     (n + 1) stays polynomial.
             */
            int local_kdeg = max_kdeg;  /* loop bound is safe upper */
            int this_integrand_deg = SC_INPUT_MAX_DEG + local_kdeg;
            for (int n = 0; n <= this_integrand_deg; ++n) {
                double P_n[SMOOTH_KERNEL_MAX_NC];
                memset(P_n, 0, sizeof(P_n));
                for (int k = 0; k <= n && k < SC_INPUT_NC; ++k) {
                    int l = n - k;
                    if (l < 0 || l > local_kdeg)
                        continue;
                    double Xk = X[k];
                    if (Xk == 0.0)
                        continue;
                    double sign_l = (l & 1) ? -1.0 : 1.0;
                    for (int j = l; j <= local_kdeg; ++j) {
                        double w_ij = kernel_coeffs[i][j];
                        if (w_ij == 0.0)
                            continue;
                        int t_deg = j - l;
                        double term = Xk * w_ij * binomial_sc(j, l) * sign_l;
                        P_n[t_deg] += term;
                    }
                }
                double inv_np1 = 1.0 / (double)(n + 1);
                for (int a = 0; a <= local_kdeg; ++a) {
                    double pa = P_n[a];
                    if (pa == 0.0)
                        continue;
                    double scale = pa * inv_np1;
                    for (int b = 0; b < SC_OUTPUT_NC; ++b) {
                        double diff = uh_pow[n + 1][b] - ul_pow[n + 1][b];
                        if (diff == 0.0)
                            continue;
                        int c = a + b;
                        if (c >= SC_OUTPUT_NC)
                            continue;
                        y_abs[c] += scale * diff;
                    }
                }
            }
        }

        /* Shift from absolute-t to phase-local (origin = breaks[s_idx]). */
        double y_local[SC_OUTPUT_NC];
        pascal_retarget_sc(y_abs, 0.0, breaks[s_idx],
                           SC_OUTPUT_NC, y_local);
        for (int k = 0; k < SC_OUTPUT_NC; ++k)
            out_coeffs[s_idx * SC_OUTPUT_NC + k] = y_local[k];
    }

    return 0;
}

/* ------------------------------------------------------------------ */
/* Public entry. See header for contract. */
int __visible
smooth_compose(
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
    int kernel_n_pieces,
    const double *kernel_piece_starts,
    const double *kernel_piece_ends,
    const double *kernel_piece_coeffs,
    double t_sm,
    int out_capacity,
    double *out_phase_t_ends,
    double *out_coeffs)
{
    if (n_input_phases <= 0)
        return -1;
    if (kernel_n_pieces <= 0 || kernel_n_pieces > SMOOTH_KERNEL_MAX_PIECES)
        return -1;
    if (t_sm <= 0.0)
        return -1;
    double move_t = input_phase_t_ends[n_input_phases - 1];
    if (move_t <= 0.0)
        return -1;

    int have_prev = (prev_n_phases > 0 && prev_phase_t_ends != NULL
                     && prev_coeffs != NULL && prev_T_move > 0.0);
    int have_next = (next_n_phases > 0 && next_phase_t_ends != NULL
                     && next_coeffs != NULL && next_T_move > 0.0);
    if (have_prev && prev_n_phases > SC_INPUT_MAX_PHASES)
        return -1;
    if (have_next && next_n_phases > SC_INPUT_MAX_PHASES)
        return -1;
    if (n_input_phases > SC_INPUT_MAX_PHASES)
        return -1;

    /* Copy external kernel into internal fixed-size buffer and compute
     * max kernel-piece degree + piece edges. */
    double tau_edges[SMOOTH_KERNEL_MAX_PIECES + 1];
    double kernel_coeffs[SMOOTH_KERNEL_MAX_PIECES][SMOOTH_KERNEL_MAX_NC];
    memset(kernel_coeffs, 0, sizeof(kernel_coeffs));
    int max_kdeg = 0;
    for (int i = 0; i < kernel_n_pieces; ++i) {
        tau_edges[i] = kernel_piece_starts[i];
        /* Consistency check: previous piece's end == this piece's start
         * (contiguous kernel). Allow a small tolerance. */
        if (i > 0) {
            double gap = kernel_piece_starts[i] - kernel_piece_ends[i - 1];
            if (gap > 1e-12 || gap < -1e-12)
                return -1;
        }
        for (int k = 0; k < SMOOTH_KERNEL_MAX_NC; ++k) {
            double c = kernel_piece_coeffs[i * SMOOTH_KERNEL_MAX_NC + k];
            kernel_coeffs[i][k] = c;
            if (c != 0.0 && k > max_kdeg)
                max_kdeg = k;
        }
    }
    tau_edges[kernel_n_pieces] = kernel_piece_ends[kernel_n_pieces - 1];

    /* Sanity-check kernel support matches the declared t_sm. */
    double support = tau_edges[kernel_n_pieces] - tau_edges[0];
    if (support <= 0.0)
        return -1;
    /* Tolerate tiny float discrepancy between piece edges and t_sm. */
    if (fabs(support - t_sm) > 1e-9 * t_sm + 1e-12)
        return -1;

    double u_min = have_prev ? -prev_T_move : 0.0;
    double u_max = have_next ? (move_t + next_T_move) : move_t;

    /* Enumerate breakpoints in [0, move_t]. Each (phase boundary,
     * kernel edge) pair contributes a candidate. */
    double raw_breaks[SC_MAX_BREAKS];
    int n_raw = 0;
    raw_breaks[n_raw++] = 0.0;
    raw_breaks[n_raw++] = move_t;
    double phase_bnds[SC_MAX_BREAKS];
    int n_bnd = 0;
    if (have_prev) {
        phase_bnds[n_bnd++] = -prev_T_move;
        for (int p = 0; p < prev_n_phases; ++p)
            phase_bnds[n_bnd++] = -prev_T_move + prev_phase_t_ends[p];
    }
    phase_bnds[n_bnd++] = 0.0;
    for (int p = 0; p < n_input_phases; ++p)
        phase_bnds[n_bnd++] = input_phase_t_ends[p];
    if (have_next) {
        for (int p = 0; p < next_n_phases; ++p)
            phase_bnds[n_bnd++] = move_t + next_phase_t_ends[p];
    }
    for (int p = 0; p < n_bnd; ++p) {
        for (int i = 0; i <= kernel_n_pieces; ++i) {
            double t_break = phase_bnds[p] + tau_edges[i];
            if (t_break > 0.0 && t_break < move_t) {
                if (n_raw >= SC_MAX_BREAKS)
                    return -1;
                raw_breaks[n_raw++] = t_break;
            }
        }
    }
    qsort(raw_breaks, n_raw, sizeof(double), cmp_dbl_sc);
    double tol = 1e-12 * (move_t + 1.0);
    double uniq_breaks[SC_MAX_BREAKS];
    int n_uniq = 0;
    for (int i = 0; i < n_raw; ++i) {
        if (n_uniq == 0 || raw_breaks[i] - uniq_breaks[n_uniq - 1] > tol) {
            uniq_breaks[n_uniq++] = raw_breaks[i];
        }
    }
    int n_out_phases = n_uniq - 1;
    if (n_out_phases <= 0 || n_out_phases > out_capacity)
        return -1;

    /* Zero the E axis (populated downstream by linear_pa_compose). */
    for (int s = 0; s < n_out_phases; ++s) {
        for (int k = 0; k < SC_OUTPUT_NC; ++k) {
            out_coeffs[(s * SC_OUTPUT_NC + k) * 4 + 3] = 0.0;
        }
    }
    for (int axis = 0; axis < 3; ++axis) {
        double in_prev_axis[SC_INPUT_MAX_PHASES * SC_OUTPUT_NC];
        double in_cur_axis[SC_INPUT_MAX_PHASES * SC_OUTPUT_NC];
        double in_next_axis[SC_INPUT_MAX_PHASES * SC_OUTPUT_NC];
        if (have_prev) {
            for (int p = 0; p < prev_n_phases; ++p) {
                for (int k = 0; k < SC_OUTPUT_NC; ++k) {
                    in_prev_axis[p * SC_OUTPUT_NC + k] =
                        prev_coeffs[(p * SC_OUTPUT_NC + k) * 4 + axis];
                }
            }
        }
        for (int p = 0; p < n_input_phases; ++p) {
            for (int k = 0; k < SC_OUTPUT_NC; ++k) {
                in_cur_axis[p * SC_OUTPUT_NC + k] =
                    input_coeffs[(p * SC_OUTPUT_NC + k) * 4 + axis];
            }
        }
        if (have_next) {
            for (int p = 0; p < next_n_phases; ++p) {
                for (int k = 0; k < SC_OUTPUT_NC; ++k) {
                    in_next_axis[p * SC_OUTPUT_NC + k] =
                        next_coeffs[(p * SC_OUTPUT_NC + k) * 4 + axis];
                }
            }
        }

        double abs_poly[SC_TOTAL_MAX_PHASES][SC_INPUT_NC];
        double phase_starts[SC_TOTAL_MAX_PHASES + 1];
        int n_total = flatten_axis_polys_sc(
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

        double out_axis[SC_MAX_BREAKS * SC_OUTPUT_NC];
        int rc = compose_axis_sc(
            n_total, phase_starts, abs_poly,
            kernel_n_pieces, tau_edges, kernel_coeffs, max_kdeg,
            u_min, u_max,
            uniq_breaks, n_uniq,
            out_axis
        );
        if (rc != 0)
            return -1;
        for (int s = 0; s < n_out_phases; ++s) {
            for (int k = 0; k < SC_OUTPUT_NC; ++k) {
                out_coeffs[(s * SC_OUTPUT_NC + k) * 4 + axis] =
                    out_axis[s * SC_OUTPUT_NC + k];
            }
        }
    }

    for (int s = 0; s < n_out_phases; ++s)
        out_phase_t_ends[s] = uniq_breaks[s + 1];

    return n_out_phases;
}
