#ifndef LINEAR_QUINTIC_H
#define LINEAR_QUINTIC_H

// Fill a 99-double coefficient buffer representing a linear
// accel/cruise/decel trapezoid as a degenerate quintic. Buffer layout:
// coeff_buf[phase * 33 + coeff * 3 + axis]. phase in {0=accel, 1=cruise,
// 2=decel}, coeff in [0..10], axis in {0=x, 1=y, 2=z}. For degenerate
// quintic: c[0] = start_pos, c[1] = axes_r * v_start_of_phase, c[2] =
// axes_r * half_accel_of_phase, c[3..10] = 0.
void build_linear_as_quintic_coeffs(
    double accel_t, double cruise_t, double decel_t,
    double start_v, double cruise_v, double accel,
    double axes_r_x, double axes_r_y, double axes_r_z,
    double start_pos_x, double start_pos_y, double start_pos_z,
    double coeff_buf[99]);

#endif
