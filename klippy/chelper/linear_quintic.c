#include "compiler.h" // __visible
#include "jerk_profile.h" // struct jerk_profile_result, JP_OK
#include "linear_quintic.h"
#include "trapq.h" // MOVE_MAX_PIECES

// Plan 8 Chunk 3: per-phase stride MOVE_QUINTIC_POLY_COEFFS (15) * 4 axes
// (x, y, z, e) = 60 doubles. The .e slot is left zero by this builder; the
// linear-PA composer (linear_pa_compose.c) fills it from the XY polynomial
// at plan emit time. For pure-E (extrude-only) moves the extruder emit
// site populates .e directly via append_extruder_only_as_quintic.
#define LINEAR_QUINTIC_PHASE_STRIDE 60
#define LINEAR_QUINTIC_COEFFS 15

static inline void
fill_phase(double *buf_phase, double v, double a,
           double pos_x, double pos_y, double pos_z,
           double rx, double ry, double rz)
{
    // c[0] = start_pos_axis (xyz), .e = 0
    buf_phase[0 * 4 + 0] = pos_x;
    buf_phase[0 * 4 + 1] = pos_y;
    buf_phase[0 * 4 + 2] = pos_z;
    buf_phase[0 * 4 + 3] = 0.0;
    // c[1] = axes_r * v (xyz), .e = 0
    buf_phase[1 * 4 + 0] = rx * v;
    buf_phase[1 * 4 + 1] = ry * v;
    buf_phase[1 * 4 + 2] = rz * v;
    buf_phase[1 * 4 + 3] = 0.0;
    // c[2] = axes_r * (a / 2) (xyz), .e = 0
    double half_a = 0.5 * a;
    buf_phase[2 * 4 + 0] = rx * half_a;
    buf_phase[2 * 4 + 1] = ry * half_a;
    buf_phase[2 * 4 + 2] = rz * half_a;
    buf_phase[2 * 4 + 3] = 0.0;
    // c[3..14] = 0
    for (int i = 3; i < LINEAR_QUINTIC_COEFFS; i++) {
        buf_phase[i * 4 + 0] = 0.0;
        buf_phase[i * 4 + 1] = 0.0;
        buf_phase[i * 4 + 2] = 0.0;
        buf_phase[i * 4 + 3] = 0.0;
    }
}

void __visible
build_linear_as_quintic_coeffs(
    double accel_t, double cruise_t, double decel_t,
    double start_v, double cruise_v, double accel,
    double axes_r_x, double axes_r_y, double axes_r_z,
    double start_pos_x, double start_pos_y, double start_pos_z,
    double coeff_buf[180])
{
    // Accel phase: start_v + accel * t, pos starts at (start_pos_*).
    fill_phase(&coeff_buf[0 * LINEAR_QUINTIC_PHASE_STRIDE], start_v, accel,
               start_pos_x, start_pos_y, start_pos_z,
               axes_r_x, axes_r_y, axes_r_z);
    // Cruise phase: constant cruise_v, pos starts where accel ended.
    double accel_disp = start_v * accel_t + 0.5 * accel * accel_t * accel_t;
    double pos_after_accel_x = start_pos_x + axes_r_x * accel_disp;
    double pos_after_accel_y = start_pos_y + axes_r_y * accel_disp;
    double pos_after_accel_z = start_pos_z + axes_r_z * accel_disp;
    fill_phase(&coeff_buf[1 * LINEAR_QUINTIC_PHASE_STRIDE], cruise_v, 0.0,
               pos_after_accel_x, pos_after_accel_y, pos_after_accel_z,
               axes_r_x, axes_r_y, axes_r_z);
    // Decel phase: starts at cruise_v, accel = -accel (deceleration).
    double pos_after_cruise_x = pos_after_accel_x + axes_r_x * cruise_v * cruise_t;
    double pos_after_cruise_y = pos_after_accel_y + axes_r_y * cruise_v * cruise_t;
    double pos_after_cruise_z = pos_after_accel_z + axes_r_z * cruise_v * cruise_t;
    fill_phase(&coeff_buf[2 * LINEAR_QUINTIC_PHASE_STRIDE], cruise_v, -accel,
               pos_after_cruise_x, pos_after_cruise_y, pos_after_cruise_z,
               axes_r_x, axes_r_y, axes_r_z);
}

/* Translate a 1-D jerk_profile_result into the multi-axis quintic-trapq slot
 * layout. Writes up to MOVE_MAX_PIECES phases into coeff_buf[MOVE_MAX_PIECES*15*4];
 * unused phases are left untouched (caller is expected to zero the buffer).
 *
 * Returns the number of phases emitted, or -1 if the profile is not JP_OK.
 *
 * axes_r: direction ratios (rx, ry, rz). For a unit-norm move vector, |(rx,ry,rz)|=1.
 * start_pos: absolute start position (axis-E / axis 3 is set to 0 by caller).
 * phase_t_ends_out: absolute (cumulative) phase end times, length must be
 * MOVE_MAX_PIECES.
 *
 * Plan 9 Phase A2a. Mirrors build_linear_as_quintic_coeffs for jerk profiles.
 */
__visible int
build_jerk_profile_as_quintic_coeffs(
    const struct jerk_profile_result *prof,
    double rx, double ry, double rz,
    double start_pos_x, double start_pos_y, double start_pos_z,
    double *phase_t_ends_out,
    double *coeff_buf /* [MOVE_MAX_PIECES * 15 * 4] */)
{
    (void)prof; (void)rx; (void)ry; (void)rz;
    (void)start_pos_x; (void)start_pos_y; (void)start_pos_z;
    (void)phase_t_ends_out; (void)coeff_buf;
    return -1;
}
