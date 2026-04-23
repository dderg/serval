#ifndef CHEB_FIT_H
#define CHEB_FIT_H

/* Plan 8 Chunk 3 Task 5 — degree-4 Chebyshev piecewise polynomial fitter.
 *
 * Given a real-valued function f(v) sampled at a caller-chosen degree+1
 * set of Chebyshev nodes of the second kind on each sub-interval
 * [v_lo_i, v_hi_i], return the Chebyshev polynomial coefficients that
 * interpolate f at those nodes on each sub-interval. The caller evaluates
 * the resulting polynomial in normalized coordinate
 *   t = 2*(v - v_lo)/(v_hi - v_lo) - 1    (t in [-1, 1])
 * via
 *   p(t) = c[0]*T0(t) + c[1]*T1(t) + ... + c[n]*Tn(t)
 * or, equivalently, convert once to monomial form.
 *
 * Degree is fixed at 4 (5 coefficients per sub-interval). See
 * `docs/superpowers/plans/plan8-research/pa_piecewise_fit.md` for the
 * derivation — deg-4 is the sweet spot across tanh / recipr models.
 *
 * Mitigation per research §5.1 (v=0 DC kick): callers that want an
 * exact f(0) can fit f(v) - f(0) instead of f(v), then add f(0) back
 * post-composition. This file does not do that baking; it's a pure
 * fitter.
 */

#define CHEB_FIT_DEGREE 4
#define CHEB_FIT_COEFFS 5 /* CHEB_FIT_DEGREE + 1 */

/* Fit a single sub-interval.
 *
 *   samples[i] = f(v_lo + 0.5 * (1 + cos((n-i)*pi/n)) * (v_hi - v_lo))
 * for i = 0 .. n, where n = CHEB_FIT_DEGREE. These are the Chebyshev
 * nodes of the second kind mapped to [v_lo, v_hi] in increasing-v
 * order. The caller supplies the samples; this routine does not invoke
 * f itself (keeps it callable from any language).
 *
 * Outputs:
 *   out_cheb_coeffs: Chebyshev coefficients c[0..4] (length 5).
 *   out_mono_coeffs: monomial coefficients m[0..4] for the polynomial
 *                    expressed in normalized t = 2*(v - v_lo)/(v_hi -
 *                    v_lo) - 1, so that
 *                      p(v) = m[0] + m[1]*t + m[2]*t^2 + m[3]*t^3 + m[4]*t^4
 *                    Either or both may be NULL.
 *
 * Returns the maximum-abs Chebyshev residual at the interpolation nodes
 * (always 0 at nodes since we're interpolating, but kept as a hook for
 * future least-squares variants that fit more nodes than coefficients).
 */
double cheb_fit_degree4_interval(
    const double *samples,       /* length CHEB_FIT_COEFFS = 5 */
    double *out_cheb_coeffs,     /* length 5, may be NULL */
    double *out_mono_coeffs);    /* length 5, may be NULL */

/* Piecewise fit over [v_lo, v_hi] partitioned by breakpoints.
 *
 *   n_breaks: number of interior breakpoints (0 => single piece).
 *   breaks: strictly increasing list, each strictly inside (v_lo, v_hi).
 *   samples: (n_breaks + 1) * CHEB_FIT_COEFFS doubles; samples[piece * 5
 *   + i] is f evaluated at the i'th Chebyshev node of piece `piece`.
 *   out_mono_coeffs: (n_breaks + 1) * CHEB_FIT_COEFFS doubles; monomial
 *                    coefficients per piece.
 *   out_piece_v_bounds: (n_breaks + 2) doubles; [v_lo, breaks..., v_hi]
 *                       for the caller's convenience (can be NULL).
 *
 * Returns 0 on success, nonzero on malformed breakpoint list.
 */
int cheb_fit_degree4_piecewise(
    double v_lo, double v_hi,
    int n_breaks,
    const double *breaks,
    const double *samples,
    double *out_mono_coeffs,
    double *out_piece_v_bounds);

/* Evaluate the monomial form returned by cheb_fit_degree4_interval
 * at v in [v_lo, v_hi]: maps v -> normalized t, then Horner. */
double cheb_fit_degree4_eval_mono(
    const double *mono_coeffs,   /* length 5 */
    double v_lo, double v_hi,
    double v);

/* Populate the 5 Chebyshev-of-2nd-kind node locations inside [v_lo, v_hi]
 * in increasing-v order:
 *
 *   out_nodes[i] = v_lo + 0.5 * (1 - cos(i * pi / n)) * (v_hi - v_lo)
 *
 * for i = 0..4, where n = CHEB_FIT_DEGREE = 4. These go to the caller
 * which evaluates f at them and hands the samples back to the fitter.
 */
void cheb_fit_degree4_nodes(double v_lo, double v_hi, double *out_nodes);

#endif /* CHEB_FIT_H */
