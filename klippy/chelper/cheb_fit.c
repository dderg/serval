/* Plan 8 Chunk 3 Task 5 — degree-4 Chebyshev piecewise polynomial fitter.
 *
 * See cheb_fit.h for the API. Implementation notes:
 *
 * Chebyshev-second-kind nodes on [-1, 1] are the n+1 Gauss-Lobatto-
 * Chebyshev points
 *     x_j = cos(j * pi / n),    j = 0 .. n
 * which include the endpoints ±1. We emit nodes in INCREASING-v order,
 * i.e. x in INCREASING-x order, so we reverse the raw j index:
 *     x_j_inc = cos((n - j) * pi / n),   j = 0 .. n.
 *
 * Interpolating the polynomial at these nodes admits a closed-form
 * Chebyshev series via the Clenshaw-Curtis / DCT-I relation:
 *
 *     c_k = (2 / n) * [ (1/2) * (f_0 + (-1)^k * f_n)
 *                       + sum_{j=1..n-1} f_j * cos(j * k * pi / n) ]
 *
 * where f_j are samples at x_j = cos(j * pi / n). Here we order samples
 * in increasing-v, so we swap j <-> (n - j) before feeding the DCT:
 *
 *     f[j]_dct = samples[n - j]
 *
 * c_k is then the coefficient of T_k(x) for the Chebyshev series. c_0
 * is halved per the DCT-I convention (Chebyshev series is c_0/2 + sum_{k>0} c_k T_k).
 *
 * Conversion to monomial form uses the explicit polynomial expansions
 * of T_0..T_4:
 *     T_0 = 1
 *     T_1 = x
 *     T_2 = 2 x^2 - 1
 *     T_3 = 4 x^3 - 3 x
 *     T_4 = 8 x^4 - 8 x^2 + 1
 *
 * Numerical stability at degree 4 is not an issue; the DCT is well
 * conditioned and the monomial conversion matrix has entries in [-8, 8].
 */

#include "cheb_fit.h"
#include <math.h>
#include <stddef.h>

#define CF_N 4 /* degree */
#define CF_NC 5 /* coeff count */

#ifndef CF_PI
#define CF_PI 3.14159265358979323846
#endif

void
cheb_fit_degree4_nodes(double v_lo, double v_hi, double *out_nodes)
{
    int i;
    double half_span = 0.5 * (v_hi - v_lo);
    double mid = 0.5 * (v_lo + v_hi);
    /* x_inc[i] = cos((n-i)*pi/n), i = 0..n. i=0 -> cos(pi) = -1 -> v_lo;
     * i=n -> cos(0) = 1 -> v_hi. Monotone increasing in v. */
    for (i = 0; i < CF_NC; ++i) {
        double x = cos((double)(CF_N - i) * CF_PI / (double)CF_N);
        out_nodes[i] = mid + half_span * x;
    }
}

/* Compute Chebyshev-of-T_k coefficients c[0..n] via DCT-I on samples
 * ordered in increasing-v. */
static void
cheb_dct1(const double *samples_inc, double *out_cheb)
{
    /* samples_inc[0] = f(x = -1) = f(v_lo); samples_inc[n] = f(x = 1). */
    /* DCT expects samples at x_j = cos(j*pi/n), which is decreasing in j:
     *   j=0 -> x=1 (v_hi)
     *   j=n -> x=-1 (v_lo)
     * so f_dct[j] = samples_inc[n - j]. */
    double f[CF_NC];
    int j;
    for (j = 0; j < CF_NC; ++j)
        f[j] = samples_inc[CF_N - j];

    int k;
    for (k = 0; k < CF_NC; ++k) {
        double acc = 0.5 * (f[0] + ((k & 1) ? -1.0 : 1.0) * f[CF_N]);
        for (j = 1; j < CF_N; ++j)
            acc += f[j] * cos((double)j * (double)k * CF_PI / (double)CF_N);
        acc *= (2.0 / (double)CF_N);
        out_cheb[k] = acc;
    }
    /* DCT-I "double endpoints" convention for interpolation via the
     * Chebyshev-Lobatto (2nd-kind) nodes requires an additional halving
     * of BOTH c_0 and c_n. c_0 is already handled implicitly by the
     * Chebyshev series convention (c_0/2 + sum_{k>=1} c_k T_k); the
     * last-coefficient halving must be applied here so interpolation
     * through the n+1 nodes is exact. */
    out_cheb[CF_N] *= 0.5;
}

/* Convert Chebyshev-series coefficients (c_0 is halved per DCT-I
 * convention; series is c_0/2 + sum_{k>=1} c_k T_k) to plain monomial
 * coefficients m[0..n] for p(x) = sum_k m[k] * x^k. Degree 4 only.
 *
 * Expansions (c_0 is the DCT output with the halving convention):
 *   c_0/2 * 1              -> m[0] += c_0/2
 *   c_1 * x                -> m[1] += c_1
 *   c_2 * (2 x^2 - 1)      -> m[0] -= c_2, m[2] += 2 c_2
 *   c_3 * (4 x^3 - 3 x)    -> m[1] -= 3 c_3, m[3] += 4 c_3
 *   c_4 * (8 x^4 - 8 x^2 + 1)  -> m[0] += c_4, m[2] -= 8 c_4, m[4] += 8 c_4
 */
static void
cheb_to_mono(const double *c, double *m)
{
    m[0] = 0.5 * c[0] - c[2] + c[4];
    m[1] = c[1] - 3.0 * c[3];
    m[2] = 2.0 * c[2] - 8.0 * c[4];
    m[3] = 4.0 * c[3];
    m[4] = 8.0 * c[4];
}

double
cheb_fit_degree4_interval(
    const double *samples,
    double *out_cheb_coeffs,
    double *out_mono_coeffs)
{
    double c[CF_NC];
    cheb_dct1(samples, c);
    if (out_cheb_coeffs) {
        int k;
        for (k = 0; k < CF_NC; ++k)
            out_cheb_coeffs[k] = c[k];
    }
    if (out_mono_coeffs)
        cheb_to_mono(c, out_mono_coeffs);
    /* Interpolation at nodes: residual is zero by construction. */
    return 0.0;
}

int
cheb_fit_degree4_piecewise(
    double v_lo, double v_hi,
    int n_breaks,
    const double *breaks,
    const double *samples,
    double *out_mono_coeffs,
    double *out_piece_v_bounds)
{
    if (v_hi <= v_lo || n_breaks < 0)
        return 1;
    /* Validate breakpoints: strictly increasing, strictly inside (v_lo,
     * v_hi). */
    double prev = v_lo;
    int i;
    for (i = 0; i < n_breaks; ++i) {
        if (breaks[i] <= prev || breaks[i] >= v_hi)
            return 2;
        prev = breaks[i];
    }
    int n_pieces = n_breaks + 1;
    /* Populate piece bounds first. */
    if (out_piece_v_bounds) {
        out_piece_v_bounds[0] = v_lo;
        for (i = 0; i < n_breaks; ++i)
            out_piece_v_bounds[i + 1] = breaks[i];
        out_piece_v_bounds[n_pieces] = v_hi;
    }
    for (i = 0; i < n_pieces; ++i) {
        cheb_fit_degree4_interval(
            samples + i * CF_NC, NULL,
            out_mono_coeffs ? (out_mono_coeffs + i * CF_NC) : NULL);
    }
    return 0;
}

double
cheb_fit_degree4_eval_mono(
    const double *mono_coeffs,
    double v_lo, double v_hi,
    double v)
{
    if (v_hi <= v_lo)
        return mono_coeffs[0];
    double t = 2.0 * (v - v_lo) / (v_hi - v_lo) - 1.0;
    /* Horner descending: m[4] down to m[0]. */
    double r = mono_coeffs[4];
    r = r * t + mono_coeffs[3];
    r = r * t + mono_coeffs[2];
    r = r * t + mono_coeffs[1];
    r = r * t + mono_coeffs[0];
    return r;
}
