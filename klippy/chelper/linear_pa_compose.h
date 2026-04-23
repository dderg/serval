#ifndef LINEAR_PA_COMPOSE_H
#define LINEAR_PA_COMPOSE_H

/* Plan 8 Chunk 3 Task 2 — Linear pressure-advance polynomial composer.
 *
 * Given a baked piecewise XY polynomial (the planner output of bs_compose
 * / fir_compose / pass-through, filling .x/.y/.z slots of the 4-axis
 * coeff_buf), compose the E (extruder) polynomial into the .e slot of
 * the same buffer in place.
 *
 * Math
 * ----
 * For each phase, the XY position is a polynomial in phase-local time tau:
 *
 *   P_xy(tau) = (P_x(tau), P_y(tau), P_z(tau))
 *   P_a(tau)  = sum_k c[k].a * tau^k     (a in {x, y, z})
 *
 * Project onto the unit XY direction n = (n_x, n_y, n_z) to get a scalar
 * 1-D position polynomial along motion:
 *
 *   P_proj(tau) = n_x * P_x(tau) + n_y * P_y(tau) + n_z * P_z(tau)
 *
 * Per-coefficient: c_proj[k] = n_x * c[k].x + n_y * c[k].y + n_z * c[k].z.
 *
 * Linear pressure advance adds a velocity-proportional kick to nominal
 * filament position:
 *
 *   E(tau) = extr_r * P_proj(tau) + k_pa * V_proj(tau)
 *
 * where
 *   - extr_r is the extruder ratio (filament mm per XY-arc mm; signed),
 *   - V_proj(tau) = d/dtau P_proj(tau) is the projected XY velocity,
 *   - k_pa is the linear-PA coefficient.
 *
 * For a polynomial of degree N, the derivative coefficients are
 *   v[k] = (k + 1) * c[k + 1]    for k = 0 .. N - 1
 * with v[N] = 0. So
 *
 *   E_c[k] = extr_r * c_proj[k] + k_pa * (k + 1) * c_proj[k + 1]
 *                                          for k = 0 .. NC - 2
 *   E_c[NC - 1] = extr_r * c_proj[NC - 1]
 *
 * where NC = MOVE_QUINTIC_POLY_COEFFS = 15. The composition is exact
 * polynomial arithmetic for straight segments (where the projection
 * recovers true 1-D arc length). For curved blends the projection is
 * an approximation: the chord direction n underestimates the true arc
 * length by ~1 - cos(θ/2) where θ is the corner angle. The legacy
 * kin_extruder.c PA path made the same approximation, so behavior is
 * preserved.
 *
 * Buffer layout (input AND output, in place):
 *   coeff_buf[phase * 15 * 4 + k * 4 + axis]
 *   axis: 0=x, 1=y, 2=z, 3=e
 * The function reads .x/.y/.z and writes .e per coefficient per phase.
 *
 * Special cases
 *   - k_pa == 0: E is the unshifted scaled XY projection.
 *   - extr_r == 0: E is purely the PA kick (rare; mostly unused).
 *   - Both == 0: every .e slot stays zero (the composer still runs to
 *     guarantee no stale .e content survives from earlier baking).
 */
void linear_pa_compose(
    int n_phases,
    double *coeff_buf,                /* in/out, n_phases * 15 * 4 doubles */
    double axis_n_x,                  /* unit XY direction component */
    double axis_n_y,
    double axis_n_z,
    double extr_r,                    /* filament-mm per XY-arc-mm (signed) */
    double k_pa);                     /* linear PA coefficient */

#endif /* LINEAR_PA_COMPOSE_H */
