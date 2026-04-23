/* Plan 8 Chunk 2 — FIR impulse-train polynomial composer (zv / mzv).
 *
 * See fir_compose.h for the public contract.
 *
 * Algorithm:
 *
 *   y(t) = sum_{i} a_i * x(t - tau_i)
 *
 * where x is the input piecewise polynomial (n_in phases on
 * [0, move_t]) and each time-shifted copy x(t - tau_i) is a piecewise
 * polynomial on [tau_i, move_t + tau_i] with phase boundaries
 * { T_p + tau_i : p = 0..n_in }.
 *
 * Steps:
 *   1. Collect breakpoints B = sort(unique({ T_p + tau_i }) intersected
 *      with [0, move_t + max_tau] clipped further if desired. For an
 *      input-shaping FIR on a planner move, the downstream trapq entry
 *      must still cover the span [0, move_t + max_tau] because the
 *      kernel extends the move duration by max_tau. We therefore emit
 *      the full span, lengthening move_t accordingly. (Caller handles
 *      the time-budget adjustment.)
 *   2. For each output sub-interval [t_a, t_b]:
 *        - For each impulse i:
 *            u = t - tau_i;
 *            identify phase p such that T_p <= u < T_{p+1} at t_mid;
 *            x_p is a poly in phase-local time (delta = u - T_p =
 *            t - tau_i - T_p); rewrite as a poly in sub-interval-local t
 *            by Pascal shift (origin = t_a) after accounting for the
 *            constant offset (tau_i + T_p - t_a).
 *            Scale by a_i and accumulate.
 *        - Store the resulting poly in phase-local basis with origin t_a.
 */

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "fir_compose.h"

#define FIR_MAX_IMPULSES 8
#define FIR_OUTPUT_NC 15
#define FIR_MAX_BREAKS 128

/* Sort doubles (qsort callback). */
static int cmp_dbl_fir(const void *a, const void *b)
{
    double da = *(const double *)a;
    double db = *(const double *)b;
    if (da < db)
        return -1;
    if (da > db)
        return 1;
    return 0;
}

/* Pascal shift: translate a poly from local origin o1 to local origin o2.
 *   f(t - o1) = sum_k c_in[k] * (t - o1)^k
 * We want coefficients of (t - o2):
 *   (t - o1)^k = (t - o2 + (o2 - o1))^k = sum_j C(k,j) * (o2 - o1)^(k-j) * (t - o2)^j
 * -> c_out[j] = sum_{k>=j} c_in[k] * C(k, j) * (o2 - o1)^(k - j)
 */
static void pascal_retarget(
    const double *in, double origin_in,
    double origin_out, int nc, double *out)
{
    double delta = origin_out - origin_in;
    for (int j = 0; j < nc; ++j) {
        double acc = 0.0;
        double dpow = 1.0;  /* delta^(k - j), starting at k = j */
        for (int k = j; k < nc; ++k) {
            double binom = 1.0;
            for (int r = 0; r < j; ++r)
                binom = binom * (double)(k - r) / (double)(r + 1);
            acc += in[k] * binom * dpow;
            dpow *= delta;
        }
        out[j] = acc;
    }
}

int fir_compose(
    int n_input_phases,
    const double *input_phase_t_ends,
    const double *input_coeffs,
    int n_impulses,
    const double *impulse_amplitudes,
    const double *impulse_delays,
    int out_capacity,
    double *out_phase_t_ends,
    double *out_coeffs)
{
    if (n_input_phases <= 0 || n_impulses <= 0 || n_impulses > FIR_MAX_IMPULSES)
        return -1;
    double move_t = input_phase_t_ends[n_input_phases - 1];
    if (move_t <= 0.0)
        return -1;
    double max_tau = 0.0;
    for (int i = 0; i < n_impulses; ++i) {
        if (impulse_delays[i] > max_tau)
            max_tau = impulse_delays[i];
        if (impulse_delays[i] < 0.0)
            return -1;
    }
    double out_span = move_t + max_tau;

    /* Build phase starts (absolute move-local): phase_starts[0] = 0,
     * phase_starts[p] = T_{p-1} = input_phase_t_ends[p-1]. */
    double phase_starts[FIR_MAX_BREAKS];
    phase_starts[0] = 0.0;
    for (int p = 0; p < n_input_phases; ++p)
        phase_starts[p + 1] = input_phase_t_ends[p];

    /* 1. Enumerate breakpoints: T_p + tau_i for each phase boundary p and
     *    impulse i, plus 0 and out_span. */
    double raw_breaks[FIR_MAX_BREAKS];
    int n_raw = 0;
    raw_breaks[n_raw++] = 0.0;
    raw_breaks[n_raw++] = out_span;
    for (int p = 0; p <= n_input_phases; ++p) {
        for (int i = 0; i < n_impulses; ++i) {
            double t_break = phase_starts[p] + impulse_delays[i];
            if (t_break > 0.0 && t_break < out_span) {
                if (n_raw >= FIR_MAX_BREAKS)
                    return -1;
                raw_breaks[n_raw++] = t_break;
            }
        }
    }
    qsort(raw_breaks, n_raw, sizeof(double), cmp_dbl_fir);
    double tol = 1e-12 * (out_span + 1.0);
    double uniq_breaks[FIR_MAX_BREAKS];
    int n_uniq = 0;
    for (int i = 0; i < n_raw; ++i) {
        if (n_uniq == 0 || raw_breaks[i] - uniq_breaks[n_uniq - 1] > tol)
            uniq_breaks[n_uniq++] = raw_breaks[i];
    }
    int n_out_phases = n_uniq - 1;
    if (n_out_phases <= 0 || n_out_phases > out_capacity)
        return -1;

    /* 2. Compose per sub-interval. */
    for (int s = 0; s < n_out_phases; ++s) {
        double t_a = uniq_breaks[s];
        double t_b = uniq_breaks[s + 1];
        double t_mid = 0.5 * (t_a + t_b);

        /* Zero accumulator. */
        double acc[FIR_OUTPUT_NC * 3];
        memset(acc, 0, sizeof(acc));

        for (int i = 0; i < n_impulses; ++i) {
            double a_i = impulse_amplitudes[i];
            if (a_i == 0.0)
                continue;
            double tau_i = impulse_delays[i];
            /* Shifted time u = t - tau_i. At t = t_mid, u_mid = t_mid - tau_i. */
            double u_mid = t_mid - tau_i;
            if (u_mid <= 0.0 || u_mid >= move_t)
                continue;  /* zero-pad outside the input move */
            /* Identify phase p such that phase_starts[p] <= u_mid <
             * input_phase_t_ends[p]. */
            int p = 0;
            while (p < n_input_phases - 1 && u_mid > input_phase_t_ends[p])
                p++;
            double phase_origin = phase_starts[p];
            /* x_p is a poly in (u - phase_origin). In absolute input-t,
             * that polynomial has origin = phase_origin. The shifted copy
             * x(t - tau_i) at absolute output-t has origin = phase_origin
             * + tau_i. Pascal-retarget to output-phase-local origin t_a. */
            for (int axis = 0; axis < 3; ++axis) {
                double coeffs_in[FIR_OUTPUT_NC];
                for (int k = 0; k < FIR_OUTPUT_NC; ++k)
                    coeffs_in[k] =
                        input_coeffs[(p * FIR_OUTPUT_NC + k) * 3 + axis];
                double coeffs_out[FIR_OUTPUT_NC];
                pascal_retarget(coeffs_in,
                                phase_origin + tau_i,
                                t_a,
                                FIR_OUTPUT_NC, coeffs_out);
                for (int k = 0; k < FIR_OUTPUT_NC; ++k)
                    acc[k * 3 + axis] += a_i * coeffs_out[k];
            }
        }

        /* Write accumulator into out_coeffs at phase s. */
        for (int k = 0; k < FIR_OUTPUT_NC; ++k) {
            for (int axis = 0; axis < 3; ++axis) {
                out_coeffs[(s * FIR_OUTPUT_NC + k) * 3 + axis] =
                    acc[k * 3 + axis];
            }
        }
    }

    /* 3. Output phase t_ends. */
    for (int s = 0; s < n_out_phases; ++s)
        out_phase_t_ends[s] = uniq_breaks[s + 1];

    return n_out_phases;
}
