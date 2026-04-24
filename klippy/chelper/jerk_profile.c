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

/* Append up to three segments (J+, A+, J-) describing the one-sided speed
 * change v_start -> v_end under (a_max, j_max). Segment 1 starts at state
 * (p0, v_start, 0). On return, *p_cursor, *v_cursor, *a_cursor are updated
 * to the state at the *end* of the last emitted segment. n_segments is
 * incremented. Returns the accel peak (a_p). If dv == 0, emits nothing and
 * returns 0.
 */
static double
build_accel_side(double v_start, double v_end, double a_max, double j_max,
                 struct jerk_profile_segment *segs, int *n_segments,
                 double *p_cursor, double *v_cursor, double *a_cursor)
{
    double t_j, t_a, a_p, dist;
    jerk_profile_accel_side_timings(v_start, v_end, a_max, j_max,
                                    &t_j, &t_a, &a_p, &dist);
    if (t_j < JP_EPS && t_a < JP_EPS)
        return 0.0;
    double sign = (v_end >= v_start) ? +1.0 : -1.0;
    double j = sign * j_max;
    /* Segment 1: J+ (jerk-up, accel rising 0 -> sign*a_p). */
    struct jerk_profile_segment *s = &segs[(*n_segments)++];
    s->type = (sign > 0) ? JP_SEG_JERK_UP_ACC : JP_SEG_JERK_DOWN_DEC;
    s->T = t_j;
    s->coeffs[0] = *p_cursor;
    s->coeffs[1] = *v_cursor;
    s->coeffs[2] = 0.0;
    s->coeffs[3] = j / 6.0;
    s->coeffs[4] = 0.0;
    s->coeffs[5] = 0.0;
    s->p0 = *p_cursor; s->v0 = *v_cursor; s->a0 = 0.0; s->j = j;
    /* Advance cursor to end of segment 1. */
    double p1 = *p_cursor + *v_cursor * t_j + (j / 6.0) * t_j * t_j * t_j;
    double v1s = *v_cursor + 0.5 * j * t_j * t_j;
    double a1 = j * t_j;    /* == sign * a_p */
    *p_cursor = p1; *v_cursor = v1s; *a_cursor = a1;
    /* Segment 2: A+ (const-accel) only if t_a > 0. */
    if (t_a > JP_EPS) {
        s = &segs[(*n_segments)++];
        s->type = (sign > 0) ? JP_SEG_CONST_ACC : JP_SEG_CONST_DEC;
        s->T = t_a;
        s->coeffs[0] = *p_cursor;
        s->coeffs[1] = *v_cursor;
        s->coeffs[2] = 0.5 * a1;
        s->coeffs[3] = 0.0;
        s->coeffs[4] = 0.0;
        s->coeffs[5] = 0.0;
        s->p0 = *p_cursor; s->v0 = *v_cursor; s->a0 = a1; s->j = 0.0;
        double p2 = *p_cursor + *v_cursor * t_a + 0.5 * a1 * t_a * t_a;
        double v2 = *v_cursor + a1 * t_a;
        *p_cursor = p2; *v_cursor = v2;
    }
    /* Segment 3: J- (jerk-down, accel falling sign*a_p -> 0). */
    s = &segs[(*n_segments)++];
    s->type = (sign > 0) ? JP_SEG_JERK_DOWN_ACC : JP_SEG_JERK_UP_DEC;
    s->T = t_j;
    s->coeffs[0] = *p_cursor;
    s->coeffs[1] = *v_cursor;
    s->coeffs[2] = 0.5 * a1;
    s->coeffs[3] = -j / 6.0;
    s->coeffs[4] = 0.0;
    s->coeffs[5] = 0.0;
    s->p0 = *p_cursor; s->v0 = *v_cursor; s->a0 = a1; s->j = -j;
    double p3 = *p_cursor + *v_cursor * t_j + 0.5 * a1 * t_j * t_j
              + (-j / 6.0) * t_j * t_j * t_j;
    double v3 = *v_cursor + a1 * t_j + 0.5 * (-j) * t_j * t_j;
    /* a should return to 0. */
    *p_cursor = p3; *v_cursor = v3; *a_cursor = 0.0;
    return a_p;
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
    memset(out, 0, sizeof(*out));
    /* Input validation. */
    if (!(v0 >= 0.0) || !(v1 >= 0.0) || !(v_peak > 0.0)
        || !(a_max > 0.0) || !(j_max > 0.0) || !(L > 0.0)
        || v0 > v_peak + JP_EPS || v1 > v_peak + JP_EPS) {
        out->status = JP_BAD_INPUT;
        return JP_BAD_INPUT;
    }
    /* Feasibility: d_floor = accel(v0 -> max(v0,v1)) + accel(max(v0,v1) -> v1)
     * (one side ramps up, the other down, by a trivial min-distance path). */
    double v_mid = (v0 > v1) ? v0 : v1;
    double d_floor = accel_side_distance(v0, v_mid, a_max, j_max)
                   + accel_side_distance(v_mid, v1, a_max, j_max);
    if (L + JP_EPS < d_floor) {
        out->status = JP_INFEASIBLE;
        out->v_hat = v_mid;
        return JP_INFEASIBLE;
    }
    /* Does full-peak fit? */
    double d_full = accel_side_distance(v0, v_peak, a_max, j_max)
                  + accel_side_distance(v_peak, v1, a_max, j_max);
    double v_hat;
    int have_cruise = 0;
    double cruise_T = 0.0;
    if (L + JP_EPS >= d_full) {
        v_hat = v_peak;
        cruise_T = (L - d_full) / v_peak;
        have_cruise = (cruise_T > JP_EPS);
    } else {
        v_hat = jerk_profile_find_v_hat(v0, v1, v_peak, a_max, j_max, L);
    }
    out->v_hat = v_hat;
    /* Build accel side (v0 -> v_hat). */
    double p_cur = 0.0, v_cur = v0, a_cur = 0.0;
    double a_acc = build_accel_side(v0, v_hat, a_max, j_max,
                                    out->segments, &out->n_segments,
                                    &p_cur, &v_cur, &a_cur);
    out->a_acc = a_acc;
    /* Cruise (if any). */
    if (have_cruise) {
        struct jerk_profile_segment *s = &out->segments[out->n_segments++];
        s->type = JP_SEG_CRUISE;
        s->T = cruise_T;
        s->coeffs[0] = p_cur;
        s->coeffs[1] = v_cur;
        s->coeffs[2] = 0.0;
        s->coeffs[3] = 0.0;
        s->coeffs[4] = 0.0;
        s->coeffs[5] = 0.0;
        s->p0 = p_cur; s->v0 = v_cur; s->a0 = 0.0; s->j = 0.0;
        p_cur += v_cur * cruise_T;
        /* v, a unchanged. */
    }
    /* Build decel side (v_hat -> v1). */
    double a_dec = build_accel_side(v_hat, v1, a_max, j_max,
                                    out->segments, &out->n_segments,
                                    &p_cur, &v_cur, &a_cur);
    out->a_dec = a_dec;
    out->status = JP_OK;
    return JP_OK;
}
