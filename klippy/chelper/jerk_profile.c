/* Plan 9 Phase A1: jerk-limited polynomial profile generator.
 *
 * Implementation mirrors docs/superpowers/plans/plan9-derivations/
 * jerk_profile_ref.py (the pre-verified Python reference).
 */
#include <math.h>
#include <string.h>

#include "compiler.h" // __visible
#include "jerk_profile.h"

static const double JP_EPS = 1e-12;

__visible void
jerk_profile_accel_side_timings(double v_start, double v_end,
                                double a_max, double j_max,
                                double *out_t_j, double *out_t_a,
                                double *out_a_peak, double *out_dist)
{
    double dv = v_end - v_start;
    if (dv < 0.0)
        dv = -dv;
    if (dv < JP_EPS) {
        *out_t_j = 0.0;
        *out_t_a = 0.0;
        *out_a_peak = 0.0;
        *out_dist = 0.0;
        return;
    }
    double dv_tri = (a_max * a_max) / j_max;
    double t_j, t_a, a_p;
    if (dv >= dv_tri) {
        t_j = a_max / j_max;
        t_a = (dv - dv_tri) / a_max;
        a_p = a_max;
    } else {
        a_p = sqrt(j_max * dv);
        t_j = a_p / j_max;
        t_a = 0.0;
    }
    double T = 2.0 * t_j + t_a;
    double d = 0.5 * (v_start + v_end) * T;
    *out_t_j = t_j;
    *out_t_a = t_a;
    *out_a_peak = a_p;
    *out_dist = d;
}

/* Distance covered by a one-sided accel group (v_start -> v_end), no jerk
 * limit care here — we just compute (v_start+v_end)/2 * T where T is the
 * group's total duration under (a_max, j_max). Reused inside find_v_hat. */
static double
accel_side_distance(double v_start, double v_end, double a_max, double j_max)
{
    double t_j, t_a, a_p, d;
    jerk_profile_accel_side_timings(v_start, v_end, a_max, j_max,
                                    &t_j, &t_a, &a_p, &d);
    return d;
}

__visible double
jerk_profile_find_v_hat(double v0, double v1, double v_peak,
                        double a_max, double j_max, double L)
{
    double v_lo = (v0 > v1) ? v0 : v1;
    double v_hi = v_peak;
    /* If full-peak is already feasible (caller mis-used us), return v_peak. */
    double d_full = accel_side_distance(v0, v_peak, a_max, j_max)
                  + accel_side_distance(v_peak, v1, a_max, j_max);
    if (d_full <= L + JP_EPS)
        return v_peak;
    /* Target: residual(v_hat) = d_acc(v0 -> v_hat) + d_dec(v_hat -> v1) - L.
     * Monotonically increasing in v_hat over [v_lo, v_hi]. Bisect. */
    for (int iter = 0; iter < 80; iter++) {
        double v_mid = 0.5 * (v_lo + v_hi);
        double d_mid = accel_side_distance(v0, v_mid, a_max, j_max)
                     + accel_side_distance(v_mid, v1, a_max, j_max);
        if (d_mid > L)
            v_hi = v_mid;
        else
            v_lo = v_mid;
        if ((v_hi - v_lo) < 1e-12 * (v_hi + 1.0))
            break;
    }
    return 0.5 * (v_lo + v_hi);
}

__visible int
jerk_profile_compute(double v0, double v1, double v_peak,
                     double a_max, double j_max, double L,
                     struct jerk_profile_result *out)
{
    (void)v0; (void)v1; (void)v_peak;
    (void)a_max; (void)j_max; (void)L;
    memset(out, 0, sizeof(*out));
    out->status = JP_BAD_INPUT;
    return JP_BAD_INPUT;
}
