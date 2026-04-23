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
 * Neighbour-aware boundary handling:
 *   For each impulse i the shifted input at time t uses x(t - tau_i).
 *   When t - tau_i < 0 the composer consults the prev move (if supplied)
 *   whose absolute-t frame is shifted by -T_prev so its end sits at
 *   u = 0. When t - tau_i > T_cur the composer consults the next move
 *   (if supplied) shifted by +T_cur. NULL neighbours zero-pad — the
 *   pre-fix behaviour, correct only when the print actually stops at
 *   the move boundary.
 *
 *   For zv / mzv shapers tau_i >= 0, so only the prev-side lookup
 *   actually fires; the next-side is plumbed for signature symmetry
 *   with bs_compose. For an acausal shaper with tau_i < 0 the next
 *   side activates automatically.
 */

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "compiler.h" // __visible
#include "fir_compose.h"

#define FIR_MAX_IMPULSES 8
#define FIR_OUTPUT_NC 15
#define FIR_MAX_BREAKS 256
#define FIR_INPUT_MAX_PHASES 32

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

/* Pascal shift: translate a poly from local origin o1 to local origin o2. */
static void pascal_retarget(
    const double *in, double origin_in,
    double origin_out, int nc, double *out)
{
    double delta = origin_out - origin_in;
    for (int j = 0; j < nc; ++j) {
        double acc = 0.0;
        double dpow = 1.0;
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

int __visible
fir_compose(
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
    int n_impulses,
    const double *impulse_amplitudes,
    const double *impulse_delays,
    int out_capacity,
    double *out_phase_t_ends,
    double *out_coeffs)
{
    if (n_input_phases <= 0 || n_impulses <= 0 || n_impulses > FIR_MAX_IMPULSES)
        return -1;
    if (n_input_phases > FIR_INPUT_MAX_PHASES)
        return -1;
    double move_t = input_phase_t_ends[n_input_phases - 1];
    if (move_t <= 0.0)
        return -1;
    double max_tau = 0.0;
    double min_tau = 0.0;
    for (int i = 0; i < n_impulses; ++i) {
        if (impulse_delays[i] > max_tau)
            max_tau = impulse_delays[i];
        if (impulse_delays[i] < min_tau)
            min_tau = impulse_delays[i];
    }
    double out_span = move_t + max_tau;

    int have_prev = (prev_n_phases > 0 && prev_phase_t_ends != NULL
                     && prev_coeffs != NULL && prev_T_move > 0.0);
    int have_next = (next_n_phases > 0 && next_phase_t_ends != NULL
                     && next_coeffs != NULL && next_T_move > 0.0);
    if (have_prev && prev_n_phases > FIR_INPUT_MAX_PHASES)
        return -1;
    if (have_next && next_n_phases > FIR_INPUT_MAX_PHASES)
        return -1;

    /* Build phase starts (absolute move-local) for current. */
    double phase_starts[FIR_INPUT_MAX_PHASES + 1];
    phase_starts[0] = 0.0;
    for (int p = 0; p < n_input_phases; ++p)
        phase_starts[p + 1] = input_phase_t_ends[p];
    /* Prev starts in absolute-t (where prev end coincides with u = 0). */
    double prev_phase_starts[FIR_INPUT_MAX_PHASES + 1];
    if (have_prev) {
        prev_phase_starts[0] = -prev_T_move;
        for (int p = 0; p < prev_n_phases; ++p)
            prev_phase_starts[p + 1] = -prev_T_move + prev_phase_t_ends[p];
    }
    /* Next starts in absolute-t (shifted by +move_t). */
    double next_phase_starts[FIR_INPUT_MAX_PHASES + 1];
    if (have_next) {
        next_phase_starts[0] = move_t;
        for (int p = 0; p < next_n_phases; ++p)
            next_phase_starts[p + 1] = move_t + next_phase_t_ends[p];
    }

    /* 1. Enumerate breakpoints: a break occurs at absolute output-t = t
     *    whenever u = t - tau_i crosses a phase boundary of any of
     *    prev / cur / next. Clip to [0, out_span]. */
    double raw_breaks[FIR_MAX_BREAKS];
    int n_raw = 0;
    raw_breaks[n_raw++] = 0.0;
    raw_breaks[n_raw++] = out_span;
    for (int i = 0; i < n_impulses; ++i) {
        double tau_i = impulse_delays[i];
        /* Current move boundaries. */
        for (int p = 0; p <= n_input_phases; ++p) {
            double t_break = phase_starts[p] + tau_i;
            if (t_break > 0.0 && t_break < out_span) {
                if (n_raw >= FIR_MAX_BREAKS)
                    return -1;
                raw_breaks[n_raw++] = t_break;
            }
        }
        if (have_prev) {
            for (int p = 0; p <= prev_n_phases; ++p) {
                double t_break = prev_phase_starts[p] + tau_i;
                if (t_break > 0.0 && t_break < out_span) {
                    if (n_raw >= FIR_MAX_BREAKS)
                        return -1;
                    raw_breaks[n_raw++] = t_break;
                }
            }
        }
        if (have_next) {
            for (int p = 0; p <= next_n_phases; ++p) {
                double t_break = next_phase_starts[p] + tau_i;
                if (t_break > 0.0 && t_break < out_span) {
                    if (n_raw >= FIR_MAX_BREAKS)
                        return -1;
                    raw_breaks[n_raw++] = t_break;
                }
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

    /* 2. Compose per sub-interval. For each impulse, locate u_mid in the
     *    appropriate move (prev / cur / next) and accumulate the Pascal-
     *    shifted contribution. Zero-pad if u_mid falls outside all
     *    supplied move ranges. */
    for (int s = 0; s < n_out_phases; ++s) {
        double t_a = uniq_breaks[s];
        double t_b = uniq_breaks[s + 1];
        double t_mid = 0.5 * (t_a + t_b);

        double acc[FIR_OUTPUT_NC * 4];
        memset(acc, 0, sizeof(acc));

        for (int i = 0; i < n_impulses; ++i) {
            double a_i = impulse_amplitudes[i];
            if (a_i == 0.0)
                continue;
            double tau_i = impulse_delays[i];
            double u_mid = t_mid - tau_i;

            /* Identify which move and which phase. */
            int which = -1;  /* 0=prev, 1=cur, 2=next */
            int p = 0;
            double phase_origin = 0.0;
            if (u_mid >= 0.0 && u_mid <= move_t) {
                which = 1;
                while (p < n_input_phases - 1 && u_mid > input_phase_t_ends[p])
                    p++;
                phase_origin = phase_starts[p];
            } else if (u_mid < 0.0 && have_prev
                       && u_mid >= -prev_T_move) {
                which = 0;
                p = 0;
                while (p < prev_n_phases - 1
                       && u_mid > prev_phase_starts[p + 1])
                    p++;
                phase_origin = prev_phase_starts[p];
            } else if (u_mid > move_t && have_next
                       && u_mid <= move_t + next_T_move) {
                which = 2;
                p = 0;
                while (p < next_n_phases - 1
                       && u_mid > next_phase_starts[p + 1])
                    p++;
                phase_origin = next_phase_starts[p];
            } else {
                continue;  /* zero-pad */
            }

            const double *src_coeffs = NULL;
            if (which == 0) src_coeffs = prev_coeffs;
            else if (which == 1) src_coeffs = input_coeffs;
            else if (which == 2) src_coeffs = next_coeffs;

            /* The phase polynomial is expressed in phase-local time
             * (origin = phase_origin in the absolute-u frame). The shifted
             * copy x(t - tau_i) at absolute-t has origin at phase_origin
             * + tau_i in the absolute-t frame. Pascal-retarget that to
             * the output sub-interval origin t_a. */
            for (int axis = 0; axis < 3; ++axis) {
                double coeffs_in[FIR_OUTPUT_NC];
                for (int k = 0; k < FIR_OUTPUT_NC; ++k)
                    coeffs_in[k] =
                        src_coeffs[(p * FIR_OUTPUT_NC + k) * 4 + axis];
                double coeffs_out[FIR_OUTPUT_NC];
                pascal_retarget(coeffs_in,
                                phase_origin + tau_i,
                                t_a,
                                FIR_OUTPUT_NC, coeffs_out);
                for (int k = 0; k < FIR_OUTPUT_NC; ++k)
                    acc[k * 4 + axis] += a_i * coeffs_out[k];
            }
        }

        for (int k = 0; k < FIR_OUTPUT_NC; ++k) {
            for (int axis = 0; axis < 4; ++axis) {
                out_coeffs[(s * FIR_OUTPUT_NC + k) * 4 + axis] =
                    acc[k * 4 + axis];
            }
        }
    }

    for (int s = 0; s < n_out_phases; ++s)
        out_phase_t_ends[s] = uniq_breaks[s + 1];

    (void)min_tau;  /* retained for potential acausal overlap refinements */
    return n_out_phases;
}
