/* Plan 8 Chunk 3 Task 2 — Linear pressure-advance polynomial composer.
 *
 * See linear_pa_compose.h for the math derivation and buffer layout.
 */

#include "linear_pa_compose.h"

/* Mirror trapq.h's MOVE_QUINTIC_POLY_COEFFS to keep this file standalone
 * (no need to pull the full trapq header into the compile graph). The two
 * constants are linked by buffer-layout convention; if MOVE_QUINTIC_POLY_COEFFS
 * ever changes, update LPA_NC here too. */
#define LPA_NC 15
#define LPA_AXES 4
#define LPA_E_OFFSET 3

void
linear_pa_compose(
    int n_phases,
    double *coeff_buf,
    double axis_n_x,
    double axis_n_y,
    double axis_n_z,
    double extr_r,
    double k_pa)
{
    if (n_phases <= 0 || coeff_buf == 0)
        return;
    int p, k;
    for (p = 0; p < n_phases; ++p) {
        double *phase = coeff_buf + p * LPA_NC * LPA_AXES;
        /* First pass: compute c_proj[k] = n . c[k]_xyz into a local
         * scratch array. */
        double c_proj[LPA_NC];
        for (k = 0; k < LPA_NC; ++k) {
            double cx = phase[k * LPA_AXES + 0];
            double cy = phase[k * LPA_AXES + 1];
            double cz = phase[k * LPA_AXES + 2];
            c_proj[k] = axis_n_x * cx + axis_n_y * cy + axis_n_z * cz;
        }
        /* Second pass: write .e from c_proj and its derivative.
         *   E[k]      = extr_r * c_proj[k] + k_pa * (k + 1) * c_proj[k + 1]
         *   E[NC - 1] = extr_r * c_proj[NC - 1]
         */
        for (k = 0; k < LPA_NC - 1; ++k) {
            phase[k * LPA_AXES + LPA_E_OFFSET] =
                extr_r * c_proj[k]
                + k_pa * (double)(k + 1) * c_proj[k + 1];
        }
        phase[(LPA_NC - 1) * LPA_AXES + LPA_E_OFFSET] =
            extr_r * c_proj[LPA_NC - 1];
    }
}
