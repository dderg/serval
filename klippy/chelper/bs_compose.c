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
 * within one input phase, so y is a single polynomial in t over the
 * sub-interval.
 *
 * Move-boundary handling: for this Chunk 2 we zero-pad outside
 * [0, move_t] — any kernel overlap with u < 0 or u > move_t contributes
 * zero. This is the documented approximation (see header).
 *
 * Implementation notes:
 *   - Input phase polynomials are converted from phase-local to
 *     absolute-move-local t by a Pascal shift per phase per axis.
 *     This is done once up front and cached in `abs_poly`.
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
/* Max breakpoints. For bs5: (n_phases + 1 internal edges) * (m + 2 kernel
 * edges). With n_phases <= 3 we get up to 4 * 7 = 28 breakpoints.
 * Tests may feed more phases for stress, so keep room. */
#define BS_MAX_BREAKS 128

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
 *
 * This replicates _cardinal_bspline_pieces + _rescale_piece from
 * shaper_defs.py exactly. We build the canonical kernel on [0, m+1] first,
 * then map canonical tau = s*(t - shift) with s = (m+1)/T_sm,
 * shift = -T_sm/2. The Jacobian factor s applied at the end gives unit
 * integral over the real support.
 */
static void build_bs_kernel(
    int m, double t_sm,
    double *tau_edges,        /* length m + 2 */
    double kernel_coeffs[BS_KERNEL_MAX_PIECES][BS_KERNEL_NC],
    int *n_pieces_out)
{
    /* Canonical pieces: sub-interval [i, i+1] has coefficients
     *   N(tau) = (1/m!) * sum_{k=0..i} (-1)^k * C(m+1, k) *
     *                                  sum_{j=0..m} C(m, j) * (-k)^(m-j) * tau^j
     */
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
    /* Rescale each canonical piece [i, i+1] to real time [-T_sm/2 + i*h,
     * -T_sm/2 + (i+1)*h] with h = T_sm / (m+1). Map tau_canon = s*(t - shift)
     * with s = (m+1)/T_sm, shift = -T_sm/2.
     * In absolute-t basis this gives:
     *   tau_canon^j = (s*t + b0)^j where b0 = -s * shift = (m+1)/2.
     * Expand via binomial into ascending t powers, then apply the density
     * Jacobian factor s.
     */
    double s = (double)(m + 1) / t_sm;
    double shift = -0.5 * t_sm;
    double b0 = -s * shift;  /* = (m+1)/2 */
    for (int i = 0; i <= m; ++i) {
        double new_coeffs[BS_KERNEL_NC + 1];
        memset(new_coeffs, 0, sizeof(new_coeffs));
        for (int j = 0; j <= m; ++j) {
            if (canonical[i][j] == 0.0)
                continue;
            /* (s*t + b0)^j = sum_k C(j,k) * s^k * b0^(j-k) * t^k */
            double s_pow = 1.0;
            double b0_pow_jk = 1.0;
            /* Pre-compute b0^(j - k) for k = 0..j */
            double b0_pows[BS_KERNEL_NC + 1];
            b0_pows[j] = 1.0;
            for (int k = j - 1; k >= 0; --k)
                b0_pows[k] = b0_pows[k + 1] * b0;
            /* b0_pows[k] = b0^(j - k). */
            for (int k = 0; k <= j; ++k) {
                new_coeffs[k] += canonical[i][j] * binomial(j, k)
                                 * s_pow * b0_pows[k];
                s_pow *= s;
            }
            (void)b0_pow_jk;
        }
        /* Jacobian factor s applied to all coefficients. */
        for (int k = 0; k < BS_KERNEL_NC; ++k)
            kernel_coeffs[i][k] = s * new_coeffs[k];
        /* Piece bounds: canonical [i, i+1] -> real [-T_sm/2 + i*h,
         * -T_sm/2 + (i+1)*h]. */
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
/* Core single-axis composer. Called 3x from bs_compose (once per axis).
 *
 * Inputs:
 *   n_in        : number of input phases
 *   phase_ends  : absolute move-local end time of each phase [n_in]
 *   phase_coeffs: per-phase 15 coefficients, phase-local basis [n_in * 15]
 *   m           : bs order 1..5
 *   t_sm        : kernel support width (F_m / shaper_freq)
 *   move_t      : total move duration
 *   breaks      : sorted unique breakpoints (absolute t) [n_breaks]
 *   n_breaks    : number of breakpoints (= n output phases + 1, with
 *                 breaks[0] = 0, breaks[n_breaks-1] = move_t)
 *
 * Outputs:
 *   out_coeffs  : per-phase 15 coefficients, phase-local [n_out * 15]
 *                 where n_out = n_breaks - 1.
 *
 * Returns 0 on success, -1 on internal error.
 */
static int compose_axis(
    int n_in,
    const double *phase_ends,
    const double *phase_coeffs,
    int m,
    double t_sm,
    double move_t,
    const double *breaks,
    int n_breaks,
    double *out_coeffs)
{
    /* 1. Convert each input phase polynomial to absolute-t monomial basis.
     *    phase p has phase-local origin phase_starts[p] = (p == 0 ? 0
     *    : phase_ends[p-1]). */
    double abs_poly[BS_MAX_BREAKS][BS_INPUT_NC];
    memset(abs_poly, 0, sizeof(abs_poly));
    for (int p = 0; p < n_in; ++p) {
        double origin = (p == 0) ? 0.0 : phase_ends[p - 1];
        /* phase_coeffs[p * 15 + k] -> only [0..10] are potentially non-zero
         * for the pre-Chunk-3 callers. We still honour all 15 in case a
         * test shoves degree-14 input (bs5 round-trip etc.). */
        double in_buf[BS_OUTPUT_NC];
        for (int k = 0; k < BS_OUTPUT_NC; ++k)
            in_buf[k] = phase_coeffs[p * BS_OUTPUT_NC + k];
        /* Reduce to BS_INPUT_NC effective coefficients; higher-order are
         * treated as zero. If any non-zero degree > 10 comes in the
         * composer still handles it but the arithmetic below truncates.
         * In practice the callers always have deg <= 10 before bs. */
        double shifted[BS_INPUT_NC];
        /* Convert phase-local (origin = phase_origin) to absolute-t
         * (origin = 0). */
        pascal_retarget(in_buf, origin, 0.0, BS_INPUT_NC, shifted);
        for (int k = 0; k < BS_INPUT_NC; ++k)
            abs_poly[p][k] = shifted[k];
    }

    /* 2. Build the kernel. */
    double tau_edges[BS_KERNEL_MAX_PIECES + 1];
    double kernel_coeffs[BS_KERNEL_MAX_PIECES][BS_KERNEL_NC];
    int n_kpieces = 0;
    build_bs_kernel(m, t_sm, tau_edges, kernel_coeffs, &n_kpieces);

    /* 3. For each output sub-interval [t_a, t_b] from consecutive
     *    breakpoints, compute the output polynomial y(t) in absolute-t
     *    basis by summing the contributions of each (kernel piece,
     *    overlapping input phase) pair, with integration limits clipped
     *    to the kernel-piece/phase overlap and to [0, move_t].
     *
     *    For a given kernel piece i with absolute-tau support [a_i, b_i]
     *    (a_i = tau_edges[i], b_i = tau_edges[i+1]):
     *        u = t - tau
     *        u in [t - b_i, t - a_i]
     *    We must further restrict u to the input phase range [T_p, T_{p+1}]
     *    and to [0, move_t] (zero-pad outside the move).
     *
     *    Under the sub-interval-constancy guarantee (from Minkowski break
     *    enumeration) the u-limits per (i, p) are:
     *        u_hi(t) = clip(t - a_i, T_p, T_{p+1}, 0, move_t, chosen constant side)
     *        u_lo(t) = clip(t - b_i, T_p, T_{p+1}, 0, move_t, chosen constant side)
     *    But because we enumerate breakpoints for every kernel edge and
     *    every phase edge (and move edges 0, move_t), within a sub-
     *    interval the "active" limit form is affine in t with slope 1
     *    (when unclipped) or constant (when clipped by a phase/move edge).
     *
     *    We probe at t_mid to determine which case each kernel piece is
     *    in, then build I_i(t) symbolically.
     */
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
            /* Clip to [0, move_t] (zero-pad outside the move). */
            double u_hi_cl = u_hi_mid;
            double u_lo_cl = u_lo_mid;
            if (u_hi_cl > move_t) u_hi_cl = move_t;
            if (u_lo_cl < 0.0)    u_lo_cl = 0.0;
            if (u_hi_cl <= u_lo_cl)
                continue;  /* kernel piece entirely outside the move */
            /* Determine which input phase contains u_mid in the clipped
             * range. Within a breakpoint-bounded sub-interval the clipped
             * u-range lies entirely in one phase by construction. */
            double u_mid_clip = 0.5 * (u_hi_cl + u_lo_cl);
            int p = 0;
            while (p < n_in - 1 && u_mid_clip > phase_ends[p])
                p++;
            /* Build the integrand P_i(u; t) = x_p(u) * w_i(t - u), as a
             * polynomial in u with coefficients that are polynomials in t.
             *
             * Strategy: expand w_i(t - u) as a polynomial in u with
             * coefficients depending on t, multiply by x_p(u), then
             * integrate u^n from u_lo(t) to u_hi(t).
             *
             * w_i(tau) = sum_j w_ij * tau^j    (tau = t - u)
             *          = sum_j w_ij * sum_l C(j,l) * t^(j-l) * (-u)^l
             *          = sum_j sum_l w_ij * C(j,l) * (-1)^l * t^(j-l) * u^l
             *
             * x_p(u) = sum_k X_k * u^k            (abs_poly[p][k])
             *
             * Product coefficient of u^n (with n = k + l):
             *   P_n(t) = sum_{k+l=n} X_k * sum_{j>=l} w_ij * C(j,l) * (-1)^l * t^(j-l)
             *
             * Integration: integral u^n du from ul(t) to uh(t)
             *            = (uh^(n+1) - ul^(n+1)) / (n+1).
             * uh(t), ul(t) are affine in t (clipped case) or a constant (fully
             * clipped). We treat both uniformly by representing uh(t), ul(t)
             * as polynomials in t (degree 1 for unclipped, degree 0 for
             * clipped).
             */
            /* uh(t), ul(t) as linear polys in t: uh_p[0] + uh_p[1]*t. */
            double uh_p[2], ul_p[2];
            /* Unclipped: uh = t - a_i -> [-a_i, 1]; ul = t - b_i -> [-b_i, 1] */
            int uh_clipped_hi = (u_hi_mid > move_t);
            int ul_clipped_lo = (u_lo_mid < 0.0);
            /* Also phase-bound clipping: if u_mid was clipped by a phase
             * boundary rather than a move boundary, we need to detect that.
             * For the current chunk we've only guaranteed that within the
             * sub-interval neither u_hi nor u_lo crosses a breakpoint — so
             * once we've clipped to [0, move_t], the surviving range lies
             * in a single phase [T_p, T_{p+1}]. No extra phase clipping. */
            if (uh_clipped_hi) {
                uh_p[0] = move_t;
                uh_p[1] = 0.0;
            } else {
                uh_p[0] = -a_i;
                uh_p[1] = 1.0;
            }
            if (ul_clipped_lo) {
                ul_p[0] = 0.0;
                ul_p[1] = 0.0;
            } else {
                ul_p[0] = -b_i;
                ul_p[1] = 1.0;
            }

            /* Pre-compute powers of uh_p and ul_p (as polys in t) up to
             * degree (BS_INPUT_NC + m) — safe upper bound. The power
             * `uh_p^(n+1)` in t has degree at most (n+1). Integrand in u
             * has degree BS_INPUT_MAX_DEG + m. */
            int max_n = BS_INPUT_MAX_DEG + m;
            int max_pow = max_n + 1;  /* n+1 */
            /* Storage: uh_pow[pw][k] = coefficient of t^k in uh_p^pw. */
            double uh_pow[BS_INPUT_NC + BS_KERNEL_NC + 1][BS_OUTPUT_NC];
            double ul_pow[BS_INPUT_NC + BS_KERNEL_NC + 1][BS_OUTPUT_NC];
            memset(uh_pow, 0, sizeof(uh_pow));
            memset(ul_pow, 0, sizeof(ul_pow));
            /* pw = 0: constant 1. */
            uh_pow[0][0] = 1.0;
            ul_pow[0][0] = 1.0;
            for (int pw = 1; pw <= max_pow; ++pw) {
                /* uh_pow[pw] = uh_pow[pw-1] * uh_p (convolve coefficients). */
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

            /* Build P_n(t) as polys in t of degree at most m:
             *   P_n(t) = sum_{k=0..min(n, 10)} X_k * sum_{j=max(n-k, 0)..m}
             *            w_{ij} * C(j, n-k) * (-1)^(n-k) * t^(j - (n - k))
             */
            double X[BS_INPUT_NC];
            for (int k = 0; k < BS_INPUT_NC; ++k)
                X[k] = abs_poly[p][k];
            double w[BS_KERNEL_NC];
            for (int j = 0; j < BS_KERNEL_NC; ++j)
                w[j] = kernel_coeffs[i][j];

            /* Integrand in u has degree BS_INPUT_MAX_DEG + m. */
            int integrand_deg = BS_INPUT_MAX_DEG + m;
            /* For each n = 0..integrand_deg, compute P_n(t) and
             * accumulate into y_abs as P_n(t) * (uh^(n+1) - ul^(n+1)) /
             * (n + 1). */
            for (int n = 0; n <= integrand_deg; ++n) {
                double P_n[BS_KERNEL_NC];
                memset(P_n, 0, sizeof(P_n));
                for (int k = 0; k <= n && k < BS_INPUT_NC; ++k) {
                    int l = n - k;
                    if (l < 0 || l > m)  /* kernel piece has coeffs 0..m */
                        continue;
                    double Xk = X[k];
                    if (Xk == 0.0)
                        continue;
                    double sign_l = (l & 1) ? -1.0 : 1.0;
                    /* Iterate over j in [l, m]. Each contributes to
                     * t^(j - l) in P_n. */
                    for (int j = l; j <= m; ++j) {
                        double w_ij = w[j];
                        if (w_ij == 0.0)
                            continue;
                        int t_deg = j - l;
                        double term = Xk * w_ij * binomial(j, l) * sign_l;
                        P_n[t_deg] += term;
                    }
                }
                /* Now accumulate P_n(t) * (uh^(n+1) - ul^(n+1)) / (n+1)
                 * into y_abs (poly in t). */
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

        /* 4. Shift y_abs from absolute-t basis to phase-local basis
         *    (origin = breaks[s_idx]). Store in output. */
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
    int n_input_phases,
    const double *input_phase_t_ends,
    const double *input_coeffs,
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

    /* 1. Enumerate breakpoints: kernel edges tau_i = -T_sm/2 + i*h shifted
     *    by each phase boundary (0, T_1, ..., T_{n_in}). Plus move edges
     *    0 and move_t. Clip to [0, move_t] and uniq-sort. */
    double raw_breaks[BS_MAX_BREAKS];
    int n_raw = 0;
    raw_breaks[n_raw++] = 0.0;
    raw_breaks[n_raw++] = move_t;
    /* Phase boundaries in absolute move-local time. */
    double phase_boundaries[BS_MAX_BREAKS];
    int n_pb = n_input_phases + 1;
    phase_boundaries[0] = 0.0;
    for (int p = 0; p < n_input_phases; ++p)
        phase_boundaries[p + 1] = input_phase_t_ends[p];
    /* For each kernel edge i in [0..m+1], breakpoint t satisfies
     *   u_{phase-boundary} = t - (-T_sm/2 + i*h) = T_p
     *   -> t = T_p + (-T_sm/2 + i*h)
     */
    for (int p = 0; p < n_pb; ++p) {
        for (int i = 0; i <= bs_order + 1; ++i) {
            double t_break = phase_boundaries[p]
                             + (-0.5 * t_sm + (double)i * h);
            if (t_break > 0.0 && t_break < move_t) {
                if (n_raw >= BS_MAX_BREAKS)
                    return -1;
                raw_breaks[n_raw++] = t_break;
            }
        }
    }
    qsort(raw_breaks, n_raw, sizeof(double), cmp_dbl);
    /* Uniq with absolute tolerance ~1e-12 * move_t. */
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

    /* 2. Per-axis compose. input_coeffs layout (Plan 8 Chunk 3):
     *    per phase: 15 * 4 doubles interleaved
     *        (c[0].x, c[0].y, c[0].z, c[0].e, c[1].x, ... c[14].e)
     *    Output same layout. We compose only the XY axes (0..2) — the .e
     *    slot is zeroed here and populated downstream by linear_pa_compose
     *    from the (now baked) XY polynomial. Pure-E content arrives via a
     *    separate emit path and never reaches this composer.
     */
    /* Zero the .e slots in the output buffer up front. */
    for (int s = 0; s < n_out_phases; ++s) {
        for (int k = 0; k < BS_OUTPUT_NC; ++k) {
            out_coeffs[(s * BS_OUTPUT_NC + k) * 4 + 3] = 0.0;
        }
    }
    for (int axis = 0; axis < 3; ++axis) {
        double in_axis[BS_MAX_BREAKS * BS_OUTPUT_NC];
        double out_axis[BS_MAX_BREAKS * BS_OUTPUT_NC];
        /* Deinterleave this axis. */
        for (int p = 0; p < n_input_phases; ++p) {
            for (int k = 0; k < BS_OUTPUT_NC; ++k) {
                in_axis[p * BS_OUTPUT_NC + k] =
                    input_coeffs[(p * BS_OUTPUT_NC + k) * 4 + axis];
            }
        }
        int rc = compose_axis(
            n_input_phases, input_phase_t_ends, in_axis,
            bs_order, t_sm, move_t,
            uniq_breaks, n_uniq,
            out_axis
        );
        if (rc != 0)
            return -1;
        /* Re-interleave. */
        for (int s = 0; s < n_out_phases; ++s) {
            for (int k = 0; k < BS_OUTPUT_NC; ++k) {
                out_coeffs[(s * BS_OUTPUT_NC + k) * 4 + axis] =
                    out_axis[s * BS_OUTPUT_NC + k];
            }
        }
    }

    /* 3. Output phase t_ends = uniq_breaks[1..n_uniq-1] (skip the 0). */
    for (int s = 0; s < n_out_phases; ++s)
        out_phase_t_ends[s] = uniq_breaks[s + 1];

    return n_out_phases;
}
