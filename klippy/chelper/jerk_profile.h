/* Plan 9 Phase A1: jerk-limited polynomial profile generator.
 *
 * Given a 1-D single-move spec (v0, v1, v_peak, a_max, j_max, L), produce a
 * piecewise polynomial description of the time-optimal jerk-limited motion.
 *
 * Output layout: up to 7 segments, each with a duration T and ascending-order
 * polynomial coefficients c0..c5 (so p(t) = c0 + c1*t + c2*t^2 + ... + c5*t^5).
 * Degree per segment: jerk phase = 3, const-accel phase = 2, cruise = 1.
 * Coefficients above the polynomial degree are set to 0.0 for safety.
 */
#ifndef JERK_PROFILE_H
#define JERK_PROFILE_H

#ifdef __cplusplus
extern "C" {
#endif

#define JERK_PROFILE_MAX_SEGMENTS 7
#define JERK_PROFILE_MAX_COEFFS 6

/* Segment type tags, matching the reference implementation. */
enum jerk_profile_seg_type {
    JP_SEG_NONE = 0,
    JP_SEG_JERK_UP_ACC   = 1,   /* 'J+':  accel rising 0 -> a_acc         */
    JP_SEG_CONST_ACC     = 2,   /* 'A+':  constant accel at a_acc         */
    JP_SEG_JERK_DOWN_ACC = 3,   /* 'J-':  accel falling a_acc -> 0        */
    JP_SEG_CRUISE        = 4,   /* 'C':   constant velocity v_hat         */
    JP_SEG_JERK_DOWN_DEC = 5,   /* 'J-d': accel falling 0 -> -a_dec       */
    JP_SEG_CONST_DEC     = 6,   /* 'A-':  constant decel at -a_dec        */
    JP_SEG_JERK_UP_DEC   = 7,   /* 'J+d': accel rising -a_dec -> 0        */
};

/* Result status codes. */
enum jerk_profile_status {
    JP_OK           = 0,
    JP_INFEASIBLE   = 1,   /* L < d_floor: cannot achieve v1 from v0 within limits */
    JP_BAD_INPUT    = 2,   /* NaN / negative / nonsense inputs                      */
};

struct jerk_profile_segment {
    int type;                                  /* enum jerk_profile_seg_type */
    double T;                                  /* segment duration (s)        */
    double coeffs[JERK_PROFILE_MAX_COEFFS];    /* ascending: c0, c1, ..., c5  */
    /* Diagnostic state at segment start (not required for replay but handy). */
    double p0;
    double v0;
    double a0;
    double j;
};

struct jerk_profile_result {
    int status;                                /* enum jerk_profile_status    */
    int n_segments;
    struct jerk_profile_segment segments[JERK_PROFILE_MAX_SEGMENTS];
    /* Diagnostics. */
    double a_acc;
    double a_dec;
    double v_hat;
};

/* Main entry point. Inputs must be: v0 >= 0, v1 >= 0, v_peak >= max(v0, v1),
 * a_max > 0, j_max > 0, L > 0. Returns JP_OK on success, error code otherwise.
 */
int jerk_profile_compute(
    double v0,
    double v1,
    double v_peak,
    double a_max,
    double j_max,
    double L,
    struct jerk_profile_result *out);

/* Sub-primitives exposed for testing. */
void jerk_profile_accel_side_timings(
    double v_start,
    double v_end,
    double a_max,
    double j_max,
    double *out_t_j,
    double *out_t_a,
    double *out_a_peak,
    double *out_dist);

double jerk_profile_find_v_hat(
    double v0,
    double v1,
    double v_peak,
    double a_max,
    double j_max,
    double L);

#ifdef __cplusplus
}
#endif

#endif /* JERK_PROFILE_H */
