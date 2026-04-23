/* Plan 8 Chunk 3 Task 6 — non-linear pressure-advance polynomial composer.
 *
 * See nonlinear_pa_compose.h for the math derivation. Implementation:
 *
 *   1. Run the linear_pa_compose formula (exact polynomial arithmetic)
 *      for extr_r * P_proj + linear_advance * V_proj.
 *   2. For tanh / recipr, sample the non-linear residual
 *          g(tau) = nonlinear_offset * f(V_proj(tau) / linearization_velocity)
 *      at the 5 Chebyshev-second-kind nodes on phase-local
 *      tau in [0, t_end - t_prev_end].
 *   3. Interpolate a degree-4 polynomial in tau through those samples
 *      (via the same DCT-I closed form as cheb_fit_degree4_interval,
 *      but directly in tau-units rather than normalized t).
 *   4. Add the resulting polynomial to .e[0..4].
 *
 * The approximation error is the difference between the true non-linear
 * response and its degree-4 interpolation at the 5 nodes. Per the Phase 0
 * research, this meets the 1 µm filament threshold across typical per-
 * move tau-spans at nonlinear_offset <= 0.1. The caller inspects the
 * out_max_residual (sampled on a dense grid per phase) and decides
 * whether to warn.
 */

#include "nonlinear_pa_compose.h"

#include <math.h>
#include <stddef.h>

#define NLPA_NC 15
#define NLPA_AXES 4
#define NLPA_E_OFFSET 3

/* Degree-4 interpolation constants — mirror cheb_fit.c. */
#define NLPA_DEG 4
#define NLPA_NFIT 5 /* deg + 1 */

#ifndef NLPA_PI
#define NLPA_PI 3.14159265358979323846
#endif

/* Residual grid: samples per phase to pick up the mid-interval worst-
 * case error between interpolation nodes. 17 = 4 intervals * 4 samples
 * + endpoints; overkill but cheap. */
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

/* Interpolate g_samples (length 5, at Chebyshev-2nd-kind nodes in tau
 * on [0, T]) onto a degree-4 monomial basis in RAW tau (not normalized).
 *
 * We first fit in normalized s = 2*tau/T - 1 via the DCT-I (same code
 * path as cheb_fit_degree4_interval), then expand monomials in s to
 * monomials in tau via s = (2/T) * tau - 1. The binomial expansion of
 * (2/T)^k * (tau - T/2)^k is messy; simpler: expand piece by piece.
 *
 * We return c[0..4] in tau-units such that sum_k c[k] * tau^k equals
 * the fit. */
static void
fit_deg4_samples_to_tau_mono(
    double T,
    const double *g_samples, /* length 5 */
    double *out_c)           /* length 5 */
{
    /* DCT-I in s (same as cheb_fit.c). */
    double cheb_c[NLPA_NFIT];
    /* Reorder samples: samples are increasing-v (= increasing-tau here),
     * DCT expects increasing-j which maps to DECREASING-t (cos(j*pi/n)).
     *   f_dct[j] = samples[n - j] */
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
     * (DCT-I "double endpoints" convention). See cheb_fit.c for
     * derivation. */
    cheb_c[NLPA_DEG] *= 0.5;
    /* Chebyshev-series -> monomial in s = 2*tau/T - 1. Same identity as
     * cheb_to_mono in cheb_fit.c (c_0 halved). */
    double s_mono[NLPA_NFIT];
    s_mono[0] = 0.5 * cheb_c[0] - cheb_c[2] + cheb_c[4];
    s_mono[1] = cheb_c[1] - 3.0 * cheb_c[3];
    s_mono[2] = 2.0 * cheb_c[2] - 8.0 * cheb_c[4];
    s_mono[3] = 4.0 * cheb_c[3];
    s_mono[4] = 8.0 * cheb_c[4];
    /* Convert s-monomial to tau-monomial via s = (2/T) * tau - 1.
     * p(s) = sum_i s_mono[i] * s^i
     * s^i = sum_{j=0..i} C(i,j) * (2/T)^j * (-1)^{i-j} * tau^j
     * Do the substitution explicitly for degree <= 4.
     *
     * Let a = 2/T, b = -1. Then s = a*tau + b and
     *   s^0 = 1
     *   s^1 = a*tau + b
     *   s^2 = a^2*tau^2 + 2*a*b*tau + b^2
     *   s^3 = a^3*tau^3 + 3*a^2*b*tau^2 + 3*a*b^2*tau + b^3
     *   s^4 = a^4*tau^4 + 4*a^3*b*tau^3 + 6*a^2*b^2*tau^2
     *         + 4*a*b^3*tau + b^4 */
    double a = (T > 0.0) ? (2.0 / T) : 0.0;
    double b = -1.0;
    double a2 = a * a, a3 = a2 * a, a4 = a3 * a;
    double b2 = b * b, b3 = b2 * b, b4 = b3 * b;
    /* Coefficient of tau^k contributions: */
    out_c[0] = s_mono[0]
             + s_mono[1] * b
             + s_mono[2] * b2
             + s_mono[3] * b3
             + s_mono[4] * b4;
    out_c[1] = s_mono[1] * a
             + s_mono[2] * 2.0 * a * b
             + s_mono[3] * 3.0 * a * b2
             + s_mono[4] * 4.0 * a * b3;
    out_c[2] = s_mono[2] * a2
             + s_mono[3] * 3.0 * a2 * b
             + s_mono[4] * 6.0 * a2 * b2;
    out_c[3] = s_mono[3] * a3
             + s_mono[4] * 4.0 * a3 * b;
    out_c[4] = s_mono[4] * a4;
}

void
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

        /* Non-linear term (Chebyshev-in-tau interpolation). */
        if (nl_enabled) {
            double t_end = phase_t_ends[p];
            double T = t_end - prev_t_end;
            if (T > 0.0) {
                /* Chebyshev-second-kind nodes on [0, T] in increasing tau. */
                double tau_nodes[NLPA_NFIT];
                double g_samples[NLPA_NFIT];
                int i;
                for (i = 0; i < NLPA_NFIT; ++i) {
                    /* tau_i = T/2 * (1 - cos((n - i) * pi / n))
                     *       = T/2 * (1 + cos(i * pi / n)) reversed; with
                     * our increasing-i convention this yields increasing
                     * tau. */
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
                fit_deg4_samples_to_tau_mono(T, g_samples, g_mono);
                /* Exact constant restoration: the constant term in tau
                 * should equal g(0) = g0 exactly. Override. */
                g_mono[0] += g0;
                /* Add into .e[0..4] — Chebyshev fit is degree 4 in tau. */
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
                        /* Evaluate g_mono(tau) and add g0. */
                        double approx = g_mono[NLPA_DEG];
                        int kk;
                        for (kk = NLPA_DEG - 1; kk >= 0; --kk)
                            approx = approx * tau + g_mono[kk];
                        /* g_mono[0] already has g0 added above. */
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
