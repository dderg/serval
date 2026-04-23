#include "linear_quintic.h"

static inline void
fill_phase(double *buf_phase, double v, double a,
           double pos_x, double pos_y, double pos_z,
           double rx, double ry, double rz)
{
    // c[0] = start_pos_axis
    buf_phase[0 * 3 + 0] = pos_x;
    buf_phase[0 * 3 + 1] = pos_y;
    buf_phase[0 * 3 + 2] = pos_z;
    // c[1] = axes_r * v
    buf_phase[1 * 3 + 0] = rx * v;
    buf_phase[1 * 3 + 1] = ry * v;
    buf_phase[1 * 3 + 2] = rz * v;
    // c[2] = axes_r * (a / 2)
    double half_a = 0.5 * a;
    buf_phase[2 * 3 + 0] = rx * half_a;
    buf_phase[2 * 3 + 1] = ry * half_a;
    buf_phase[2 * 3 + 2] = rz * half_a;
    // c[3..10] = 0
    for (int i = 3; i < 11; i++) {
        buf_phase[i * 3 + 0] = 0.0;
        buf_phase[i * 3 + 1] = 0.0;
        buf_phase[i * 3 + 2] = 0.0;
    }
}

void
build_linear_as_quintic_coeffs(
    double accel_t, double cruise_t, double decel_t,
    double start_v, double cruise_v, double accel,
    double axes_r_x, double axes_r_y, double axes_r_z,
    double start_pos_x, double start_pos_y, double start_pos_z,
    double coeff_buf[99])
{
    // Accel phase: start_v + accel * t, pos starts at (start_pos_*).
    fill_phase(&coeff_buf[0 * 33], start_v, accel,
               start_pos_x, start_pos_y, start_pos_z,
               axes_r_x, axes_r_y, axes_r_z);
    // Cruise phase: constant cruise_v, pos starts where accel ended.
    double accel_disp = start_v * accel_t + 0.5 * accel * accel_t * accel_t;
    double pos_after_accel_x = start_pos_x + axes_r_x * accel_disp;
    double pos_after_accel_y = start_pos_y + axes_r_y * accel_disp;
    double pos_after_accel_z = start_pos_z + axes_r_z * accel_disp;
    fill_phase(&coeff_buf[1 * 33], cruise_v, 0.0,
               pos_after_accel_x, pos_after_accel_y, pos_after_accel_z,
               axes_r_x, axes_r_y, axes_r_z);
    // Decel phase: starts at cruise_v, accel = -accel (deceleration).
    double pos_after_cruise_x = pos_after_accel_x + axes_r_x * cruise_v * cruise_t;
    double pos_after_cruise_y = pos_after_accel_y + axes_r_y * cruise_v * cruise_t;
    double pos_after_cruise_z = pos_after_accel_z + axes_r_z * cruise_v * cruise_t;
    fill_phase(&coeff_buf[2 * 33], cruise_v, -accel,
               pos_after_cruise_x, pos_after_cruise_y, pos_after_cruise_z,
               axes_r_x, axes_r_y, axes_r_z);
}
