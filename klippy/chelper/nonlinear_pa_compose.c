/* Plan 8 Chunk 3 Task 6 — non-linear pressure-advance polynomial composer.
 *
 * See nonlinear_pa_compose.h for the math derivation. Implementation:
 *
 *   1. Run the linear_pa_compose formula (exact polynomial arithmetic)
 *      for extr_r * P_proj + linear_advance * V_proj.
 *   2. For tanh / recipr, sample the non-linear residual
 *          g(tau) = nonlinear_offset * f(V_proj(tau) / linearization_velocity)
 *      at the 7 Chebyshev-second-kind nodes on phase-local
 *      tau in [0, t_end - t_prev_end].
 *   3. Interpolate a degree-6 polynomial in tau through those samples
 *      (via the DCT-I closed form, in tau-units directly).
 *   4. Add the resulting polynomial to .e[0..6].
 *
 * Degree rationale (Chunk 3 fix, 2026-04-23)
 * ------------------------------------------
 * The original implementation used a degree-4 Chebyshev fit per phase.
 * Empirical: on a sharp-corner blend with nonlinear_offset = 0.05 the
 * residual hit ~26 µm filament — 26× over the 1 µm budget. Raising the
 * fit degree to 6 captures the tanh knee substantially better while
 * keeping the output polynomial within the 15-coefficient slot (.e[0..6],
 * alongside the exact linear terms that already occupy .e[0..14]).
 *
 * We do NOT subdivide the phase adaptively: per-XY-phase subdivision
 * would require widening MOVE_MAX_PIECES (currently 32, already stressed
 * to ~28 by bs5 output), which is a larger structural change reserved
 * for a future refactor. For now, degree 6 is the practical compromise
 * (per Phase 0 research pa_piecewise_fit.md: deg-6 vs deg-4 is
 * significantly better at the tanh knee). When residual still exceeds
 * budget the caller receives the value via out_max_residual and logs a
 * once-per-move warning.
 *
 * If a workload reliably exceeds 1 µm with degree 6, the follow-up is
 * either: (a) bump MOVE_QUINTIC_POLY_COEFFS from 15 to ~20 and enable
 * true piecewise subdivision here, or (b) accept the artifact for
 * aggressive-corner PA and document the ceiling.
 */

#include "compiler.h" // __visible
#include "nonlinear_pa_compose.h"

#include <math.h>
#include <stddef.h>

#define NLPA_NC 15
#define NLPA_AXES 4
#define NLPA_E_OFFSET 3

/* Degree-6 interpolation constants. Fit polynomial has 7 coefficients
 * (.e[0..6]); remaining .e[7..14] stay available for the exact linear
 * PA terms (which can populate up through .e[14] via extr_r * P on a
 * bs5-shaped phase of degree 14). */
#define NLPA_DEG 6
#define NLPA_NFIT 7 /* deg + 1 */

#ifndef NLPA_PI
#define NLPA_PI 3.14159265358979323846
#endif

/* Residual grid: samples per phase to pick up the mid-interval worst-
 * case error between interpolation nodes. With 7 interpolation nodes
 * we sample 33 points — ≥4 between every adjacent node pair. */
#define NLPA_RESID_SAMPLES 33

/* Evaluate the derivative of the polynomial (per-phase) at tau.
 *   P'(tau) = sum_{k>=1} k * c[k] * tau^{k-1}
 * Implemented as Horner on the derivative-coefficient sequence. */
static inline double
eval_axis_poly_deriv(const double *phase, int axis_slot, double tau)
{
    /* Derivative has degree NLPA_NC - 2; coefficients are k * c[k] for
     * k = 1 .. NLPA_NC - 1. */
    double r = (double)(NLPA_NC - 1) * phase[(NLPA_NC - 1) * NLPA_AXES + axis_slot];
    int k;
    for (k = NLPA_NC - 2; k >= 1; --k)
        r = r * tau + (double)k * phase[k * NLPA_AXES + axis_slot];
    return r;
}

/* Compute V_proj(tau) = n . d/dtau P_xyz(tau) for a given phase. */
static inline double
v_proj_at(const double *phase, double nx, double ny, double nz, double tau)
{
    double vx = eval_axis_poly_deriv(phase, 0, tau);
    double vy = eval_axis_poly_deriv(phase, 1, tau);
    double vz = eval_axis_poly_deriv(phase, 2, tau);
    return nx * vx + ny * vy + nz * vz;
}

/* Non-linear PA response function, clamped to non-negative v.
 *   tanh:   tanh(v / v_lin)
 *   recipr: 1 - 1 / (1 + v / v_lin)
 * Both are 0 at v = 0, monotone increasing for v > 0, and asymptote
 * to 1 as v -> infinity. */
static inline double
pa_f_nonlin(double v, double v_lin, int model_kind)
{
    if (v < 0.0)
        v = 0.0;
    if (v_lin <= 0.0)
        return 0.0;
    double r = v / v_lin;
    switch (model_kind) {
    case NLPA_MODEL_TANH:
        return tanh(r);
    case NLPA_MODEL_RECIPR:
        return 1.0 - 1.0 / (1.0 + r);
    default:
        return 0.0;
    }
}

/* Interpolate g_samples (length NLPA_NFIT, at Chebyshev-second-kind
 * nodes in tau on [0, T]) onto a degree-NLPA_DEG monomial basis in
 * RAW tau (not normalized).
 *
 * Approach:
 *   1. DCT-I on samples to obtain Chebyshev-of-T_k coefficients in
 *      normalized s = 2*tau/T - 1.
 *   2. Expand T_0..T_6 to monomials in s (explicit closed form).
 *   3. Substitute s = a*tau + b (a = 2/T, b = -1) into the s-monomial
 *      via binomial expansion.
 *
 * out_c has length NLPA_NFIT (= 7), indexed by tau-monomial degree. */
static void
fit_deg_samples_to_tau_mono(
    double T,
    const double *g_samples, /* length NLPA_NFIT */
    double *out_c)           /* length NLPA_NFIT */
{
    /* DCT-I in s. Samples arrive in increasing-tau order (= increasing
     * x order); DCT wants decreasing-x (j=0 -> x=1). Reverse index. */
    double cheb_c[NLPA_NFIT];
    double f_dct[NLPA_NFIT];
    int j, k;
    for (j = 0; j < NLPA_NFIT; ++j)
        f_dct[j] = g_samples[NLPA_DEG - j];
    for (k = 0; k < NLPA_NFIT; ++k) {
        double acc = 0.5 * (f_dct[0] + ((k & 1) ? -1.0 : 1.0) * f_dct[NLPA_DEG]);
        for (j = 1; j < NLPA_DEG; ++j)
            acc += f_dct[j]
                 * cos((double)j * (double)k * NLPA_PI / (double)NLPA_DEG);
        acc *= (2.0 / (double)NLPA_DEG);
        cheb_c[k] = acc;
    }
    /* Chebyshev-Lobatto interpolation: last coefficient must be halved
     * (DCT-I "double endpoints" convention). */
    cheb_c[NLPA_DEG] *= 0.5;
    /* Chebyshev-series -> monomial in s. Using T_0..T_6:
     *   T_0 = 1
     *   T_1 = x
     *   T_2 = 2 x^2 - 1
     *   T_3 = 4 x^3 - 3 x
     *   T_4 = 8 x^4 - 8 x^2 + 1
     *   T_5 = 16 x^5 - 20 x^3 + 5 x
     *   T_6 = 32 x^6 - 48 x^4 + 18 x^2 - 1
     *
     * Series p(s) = (c_0/2)*T_0 + sum_{k>=1} c_k T_k(s).
     *
     * Collect coefficients of s^0 .. s^6: */
    double s_mono[NLPA_NFIT];
    s_mono[0] = 0.5 * cheb_c[0] - cheb_c[2] + cheb_c[4] - cheb_c[6];
    s_mono[1] = cheb_c[1] - 3.0 * cheb_c[3] + 5.0 * cheb_c[5];
    s_mono[2] = 2.0 * cheb_c[2] - 8.0 * cheb_c[4] + 18.0 * cheb_c[6];
    s_mono[3] = 4.0 * cheb_c[3] - 20.0 * cheb_c[5];
    s_mono[4] = 8.0 * cheb_c[4] - 48.0 * cheb_c[6];
    s_mono[5] = 16.0 * cheb_c[5];
    s_mono[6] = 32.0 * cheb_c[6];
    /* Convert s-monomial to tau-monomial via s = a*tau + b, a = 2/T,
     * b = -1. Then s^k = sum_{j=0..k} C(k,j) * a^j * b^(k-j) * tau^j.
     * Do this via direct accumulation (k up to 6). */
    double a = (T > 0.0) ? (2.0 / T) : 0.0;
    double b = -1.0;
    /* Precompute powers. */
    double a_pow[NLPA_NFIT];
    double b_pow[NLPA_NFIT];
    a_pow[0] = 1.0;
    b_pow[0] = 1.0;
    int i;
    for (i = 1; i < NLPA_NFIT; ++i) {
        a_pow[i] = a_pow[i - 1] * a;
        b_pow[i] = b_pow[i - 1] * b;
    }
    /* Binomial coefficients for (k,j), 0 <= j <= k <= 6. Table is
     * tiny; hand-roll for clarity. */
    static const double binom[NLPA_NFIT][NLPA_NFIT] = {
        {1, 0, 0, 0, 0, 0, 0},
        {1, 1, 0, 0, 0, 0, 0},
        {1, 2, 1, 0, 0, 0, 0},
        {1, 3, 3, 1, 0, 0, 0},
        {1, 4, 6, 4, 1, 0, 0},
        {1, 5, 10, 10, 5, 1, 0},
        {1, 6, 15, 20, 15, 6, 1},
    };
    /* Zero output. */
    int kk;
    for (kk = 0; kk < NLPA_NFIT; ++kk)
        out_c[kk] = 0.0;
    /* For each s^k term, distribute into tau-monomials via binomial. */
    for (k = 0; k < NLPA_NFIT; ++k) {
        double sk = s_mono[k];
        if (sk == 0.0)
            continue;
        int jj;
        for (jj = 0; jj <= k; ++jj) {
            double coeff = sk * binom[k][jj] * a_pow[jj] * b_pow[k - jj];
            out_c[jj] += coeff;
        }
    }
}

void __visible
nonlinear_pa_compose(
    int n_phases,
    const double *phase_t_ends,
    double *coeff_buf,
    double axis_n_x,
    double axis_n_y,
    double axis_n_z,
    double extr_r,
    double linear_advance,
    double nonlinear_offset,
    double linearization_velocity,
    int model_kind,
    double *out_max_residual)
{
    if (out_max_residual)
        *out_max_residual = 0.0;
    if (n_phases <= 0 || coeff_buf == NULL || phase_t_ends == NULL)
        return;

    int nl_enabled = (model_kind == NLPA_MODEL_TANH
                      || model_kind == NLPA_MODEL_RECIPR)
                     && nonlinear_offset != 0.0
                     && linearization_velocity > 0.0;

    int p, k;
    double prev_t_end = 0.0;
    double max_resid = 0.0;

    for (p = 0; p < n_phases; ++p) {
        double *phase = coeff_buf + p * NLPA_NC * NLPA_AXES;

        /* First: exact polynomial term
         *   E_linear(tau) = extr_r * P_proj(tau) + linear_advance * V_proj(tau)
         * Mirror linear_pa_compose with k_pa = linear_advance. */
        double c_proj[NLPA_NC];
        for (k = 0; k < NLPA_NC; ++k) {
            double cx = phase[k * NLPA_AXES + 0];
            double cy = phase[k * NLPA_AXES + 1];
            double cz = phase[k * NLPA_AXES + 2];
            c_proj[k] = axis_n_x * cx + axis_n_y * cy + axis_n_z * cz;
        }
        /* Zero the entire .e column before writing — keeps stale content
         * from earlier baking (e.g. a prior PA pass) from leaking through. */
        for (k = 0; k < NLPA_NC; ++k)
            phase[k * NLPA_AXES + NLPA_E_OFFSET] = 0.0;
        /* Linear terms. */
        for (k = 0; k < NLPA_NC - 1; ++k) {
            phase[k * NLPA_AXES + NLPA_E_OFFSET] =
                extr_r * c_proj[k]
                + linear_advance * (double)(k + 1) * c_proj[k + 1];
        }
        phase[(NLPA_NC - 1) * NLPA_AXES + NLPA_E_OFFSET] =
            extr_r * c_proj[NLPA_NC - 1];

        /* Non-linear term (Chebyshev-in-tau interpolation, degree 6). */
        if (nl_enabled) {
            double t_end = phase_t_ends[p];
            double T = t_end - prev_t_end;
            if (T > 0.0) {
                /* Chebyshev-second-kind nodes on [0, T] in increasing
                 * tau. tau_i = T/2 * (1 + cos((n-i) * pi / n)). */
                double tau_nodes[NLPA_NFIT];
                double g_samples[NLPA_NFIT];
                int i;
                for (i = 0; i < NLPA_NFIT; ++i) {
                    double x = cos((double)(NLPA_DEG - i) * NLPA_PI
                                   / (double)NLPA_DEG);
                    tau_nodes[i] = 0.5 * T * (1.0 + x);
                    double v = v_proj_at(phase, axis_n_x, axis_n_y, axis_n_z,
                                         tau_nodes[i]);
                    g_samples[i] = nonlinear_offset
                                 * pa_f_nonlin(v, linearization_velocity,
                                               model_kind);
                }
                /* Mitigation for v=0 DC kick: subtract g(0) before fit,
                 * then re-add as an exact constant. Since f(0) = 0 for
                 * both tanh and recipr (both are 0 at v=0), g(0) = 0,
                 * but the interpolated polynomial may not be. Enforce
                 * g_hat(0) = 0 by shifting the constant term. */
                double g0 = g_samples[0]; /* = 0 in practice, but keep
                                             general for negative-v edge. */
                for (i = 0; i < NLPA_NFIT; ++i)
                    g_samples[i] -= g0;
                double g_mono[NLPA_NFIT];
                fit_deg_samples_to_tau_mono(T, g_samples, g_mono);
                /* Exact constant restoration: the constant term in tau
                 * should equal g(0) = g0 exactly. Override. */
                g_mono[0] += g0;
                /* Add into .e[0..NLPA_DEG]. */
                for (k = 0; k < NLPA_NFIT; ++k)
                    phase[k * NLPA_AXES + NLPA_E_OFFSET] += g_mono[k];

                /* Residual estimate on dense grid (compare original
                 * g(tau) (pre-shift) against g_hat(tau) + g0). */
                if (out_max_residual) {
                    double local_max = 0.0;
                    for (i = 0; i < NLPA_RESID_SAMPLES; ++i) {
                        double tau = T * (double)i
                                   / (double)(NLPA_RESID_SAMPLES - 1);
                        double v = v_proj_at(phase, axis_n_x, axis_n_y,
                                             axis_n_z, tau);
                        double truth = nonlinear_offset
                                     * pa_f_nonlin(v, linearization_velocity,
                                                   model_kind);
                        /* Evaluate g_mono(tau) via Horner. */
                        double approx = g_mono[NLPA_DEG];
                        int kk;
                        for (kk = NLPA_DEG - 1; kk >= 0; --kk)
                            approx = approx * tau + g_mono[kk];
                        double err = fabs(truth - approx);
                        if (err > local_max)
                            local_max = err;
                    }
                    if (local_max > max_resid)
                        max_resid = local_max;
                }
            }
        }

        prev_t_end = phase_t_ends[p];
    }

    if (out_max_residual)
        *out_max_residual = max_resid;
}
