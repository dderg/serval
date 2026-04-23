#ifndef NONLINEAR_PA_COMPOSE_H
#define NONLINEAR_PA_COMPOSE_H

/* Plan 8 Chunk 3 Task 6 — non-linear pressure-advance polynomial composer.
 *
 * Given a baked piecewise XY polynomial (produced by bs_compose /
 * fir_compose / pass-through; occupying .x/.y/.z slots of a 4-axis
 * coeff_buf) and a non-linear PA model (tanh or recipr), compose the
 * E (extruder) polynomial into the .e slot in place.
 *
 * Math
 * ----
 * The PA contribution for tanh / recipr models is
 *
 *   E(tau) = extr_r * P_proj(tau)
 *          + linear_advance * V_proj(tau)
 *          + nonlinear_offset * f(V_proj(tau) / linearization_velocity)
 *
 * where f is tanh(x) or 1 - 1/(1+x). The first two terms are exact
 * polynomial arithmetic (same as linear_pa_compose). Only the non-linear
 * term is approximated — composed by evaluating
 *
 *     g(tau) = nonlinear_offset * f(V_proj(tau) / linearization_velocity)
 *
 * at the 5 Chebyshev-second-kind nodes in phase-local tau (one piece per
 * XY-polynomial phase is what the planner emits in Chunk 3), interpolating
 * a degree-4 polynomial through those samples, and adding it to the .e
 * slot. This is the "tau-direct" strategy in the implementation plan:
 * simpler than explicit Chebyshev-in-v composition and gives comparable
 * error per the Phase 0 research.
 *
 * Model kinds:
 *   0 = disabled / linear (call linear_pa_compose instead; the nonlinear
 *       path still runs, it just produces zeros for the offset term)
 *   1 = tanh
 *   2 = recipr
 *
 * Residual
 * --------
 * The `out_max_residual` out-param reports the max-abs fit error on a
 * densely sampled residual grid (per phase; max across phases). Caller
 * multiplies by nonlinear_offset to get filament error and decides
 * whether to accept or fall back.
 */

/* Model-kind sentinels. Keep in sync with the Python dispatch. */
#define NLPA_MODEL_NONE   0
#define NLPA_MODEL_TANH   1
#define NLPA_MODEL_RECIPR 2

/* Compose tanh / recipr PA into the .e slot of a baked XY coeff_buf.
 *
 *   n_phases        number of piecewise phases in coeff_buf.
 *   phase_t_ends    absolute move-local end times of each phase
 *                   (length n_phases). Used to derive phase durations
 *                   for node placement in phase-local tau.
 *   coeff_buf       in/out, n_phases * 15 * 4 doubles (x, y, z, e).
 *                   The .x/.y/.z slots are read; .e is overwritten.
 *   axis_n_*        unit XY direction (projection for scalar P/V).
 *   extr_r          filament-mm per XY-arc-mm (signed).
 *   linear_advance  linear PA coefficient (the exact polynomial term).
 *   nonlinear_offset non-linear PA scale factor (typical 0.02-0.1 mm).
 *                   If 0, the non-linear branch is short-circuited to
 *                   "linear only" (matches linear_pa_compose semantics
 *                   with k_pa = linear_advance).
 *   linearization_velocity  x = v / this; must be > 0 if
 *                           nonlinear_offset is > 0 (guarded by caller).
 *   model_kind      one of NLPA_MODEL_*. NONE falls back to linear only.
 *   out_max_residual may be NULL. If non-NULL, receives the max-abs fit
 *                    residual (not yet multiplied by nonlinear_offset).
 *                    Callers translate via filament_err = residual *
 *                    nonlinear_offset.
 */
void nonlinear_pa_compose(
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
    double *out_max_residual);

#endif /* NONLINEAR_PA_COMPOSE_H */
