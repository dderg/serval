#ifndef LINEAR_QUINTIC_H
#define LINEAR_QUINTIC_H

// Fill a 180-double coefficient buffer representing a linear
// accel/cruise/decel trapezoid as a degenerate quintic. Buffer layout:
// coeff_buf[phase * 60 + coeff * 4 + axis]. phase in {0=accel, 1=cruise,
// 2=decel}, coeff in [0..14], axis in {0=x, 1=y, 2=z, 3=e}. For degenerate
// quintic: c[0] = start_pos, c[1] = axes_r * v_start_of_phase, c[2] =
// axes_r * half_accel_of_phase, c[3..14] = 0. The .e slot is left zero;
// the linear-PA composer (linear_pa_compose.c, Plan 8 Chunk 3) populates
// it from the XY polynomial at plan emit time.
void build_linear_as_quintic_coeffs(
    double accel_t, double cruise_t, double decel_t,
    double start_v, double cruise_v, double accel,
    double axes_r_x, double axes_r_y, double axes_r_z,
    double start_pos_x, double start_pos_y, double start_pos_z,
    double coeff_buf[180]);

#endif
