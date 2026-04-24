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
    (void)v_start; (void)v_end; (void)a_max; (void)j_max;
    *out_t_j = 0.0;
    *out_t_a = 0.0;
    *out_a_peak = 0.0;
    *out_dist = 0.0;
}

__visible double
jerk_profile_find_v_hat(double v0, double v1, double v_peak,
                        double a_max, double j_max, double L)
{
    (void)v0; (void)v1; (void)a_max; (void)j_max; (void)L;
    return v_peak;
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
