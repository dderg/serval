#include <stddef.h> // NULL
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
    double *coeff_buf)
{
    if (prof == NULL || prof->status != JP_OK)
        return -1;
    /* Zero coeff_buf (caller may not have zeroed). */
    for (int i = 0; i < MOVE_MAX_PIECES * 15 * 4; i++)
        coeff_buf[i] = 0.0;
    /* Note: seg->coeffs[0] is absolute-in-1D (set by build_accel_side in
     * jerk_profile.c, which threads *p_cursor as an absolute scalar starting
     * at 0.0 set by jerk_profile_compute). So axis-wise c0 is simply
     * start_pos_<axis> + r_<axis> * seg->coeffs[0]. No per-phase running
     * offset needed. */
    double cum_t = 0.0;
    int out_phase = 0;
    for (int s = 0; s < prof->n_segments; s++) {
        const struct jerk_profile_segment *seg = &prof->segments[s];
        if (seg->T <= 1e-12)
            continue;       /* Skip zero-duration segments. */
        if (out_phase >= MOVE_MAX_PIECES)
            return -1;      /* Too many phases to fit. */
        /* Per-axis polynomial coefficients: ax_c[k] = axis_ratio * seg.coeffs[k]. */
        double *phase_base = coeff_buf + out_phase * 15 * 4;
        for (int k = 0; k < 6; k++) {
            double c_1d = seg->coeffs[k];
            phase_base[k * 4 + 0] = rx * c_1d;
            phase_base[k * 4 + 1] = ry * c_1d;
            phase_base[k * 4 + 2] = rz * c_1d;
            phase_base[k * 4 + 3] = 0.0; /* Axis E not handled here — A5 scope. */
        }
        /* Override c0 with absolute start_pos + axis-ratio * 1-D segment start. */
        phase_base[0 * 4 + 0] = start_pos_x + rx * seg->coeffs[0];
        phase_base[0 * 4 + 1] = start_pos_y + ry * seg->coeffs[0];
        phase_base[0 * 4 + 2] = start_pos_z + rz * seg->coeffs[0];
        cum_t += seg->T;
        phase_t_ends_out[out_phase] = cum_t;
        out_phase++;
    }
    return out_phase;
}
