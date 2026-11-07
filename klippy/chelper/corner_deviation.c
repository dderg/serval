// Junction speed of [corner_deviation] (klippy/extras/corner_deviation.py
// builds the tables and documents the limits)
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <math.h> // atan2, hypot, sqrt
#include <stdint.h> // uint64_t
#include "compiler.h" // __visible

#pragma GCC optimize ("fp-contract=off")

#define CD_STEPS 256

struct cd_step {
    double tau, dv[2], share[2];
};

struct cd_limiter {
    int n_alpha, n_theta;
    double alpha_step, theta_step;
    double *floor, *w_floor;
    double variance, window, max_corner_accel, short_turn;
    int has_corner_bound;
    double density_peak[2];
    double min_direction_change, min_xy_share;
    uint64_t limited, step_limited, xy_stops;
    struct cd_step steps[CD_STEPS];
    int steps_len, steps_head;
};

static int
clamped_index(double x, int last)
{
    int i = (int)x;
    return i < last ? i : last;
}

static double
curve_speed(const struct cd_limiter *l, double prev_length
            , double move_length, double turn, double accel, double room)
{
    double curvature = 2. * fabs(turn) / (prev_length + move_length);
    double curve_v2 = 2. * accel * room / (l->variance * curvature);
    double longest = move_length > prev_length ? move_length : prev_length;
    if (longest < sqrt(curve_v2) * l->window)
        curve_v2 -= .5 * accel * longest;
    return sqrt(curve_v2 > 0. ? curve_v2 : 0.);
}

static double
corner_bound(struct cd_limiter *l, const double up[2], const double um[2]
             , const double du[2], double accel, double speed
             , const double window_dv[2]
             , const double window_share[2])
{
    double bound = speed;
    int axis;
    for (axis = 0; axis < 2; axis++) {
        double density_peak = l->density_peak[axis];
        if (!density_peak || !du[axis])
            continue;
        double linear = du[axis] * density_peak;
        double share = fabs(um[axis]) > fabs(up[axis])
                       ? fabs(um[axis]) : fabs(up[axis]);
        if (window_share[axis] > share)
            share = window_share[axis];
        double room = (l->max_corner_accel - accel * share
                       - density_peak * window_dv[axis]);
        if (room <= 0.) {
            bound = 0.;
            continue;
        }
        double own = room / linear;
        if (own < bound)
            bound = own;
    }
    if (bound < speed)
        l->step_limited++;
    return bound;
}

static int
window_steps(struct cd_limiter *l, double tau, double window_dv[2]
             , double window_share[2])
{
    double elapsed = tau;
    int i;
    for (i = 0; i < l->steps_len; i++) {
        if (elapsed > 2. * l->window)
            return 1;
        const struct cd_step *s = &l->steps[
            (l->steps_head - 1 - i + CD_STEPS) % CD_STEPS];
        int axis;
        for (axis = 0; axis < 2; axis++) {
            window_dv[axis] += s->dv[axis];
            if (s->share[axis] > window_share[axis])
                window_share[axis] = s->share[axis];
        }
        elapsed += s->tau;
    }
    return elapsed > 2. * l->window || l->steps_len < CD_STEPS;
}

static void
note_step(struct cd_limiter *l, double tau, double v2, double upx
          , double upy, double umx, double umy)
{
    struct cd_step *s = &l->steps[l->steps_head];
    double v = sqrt(v2);
    s->tau = tau;
    s->dv[0] = v * fabs(umx - upx);
    s->dv[1] = v * fabs(umy - upy);
    s->share[0] = fabs(upx);
    s->share[1] = fabs(upy);
    l->steps_head = (l->steps_head + 1) % CD_STEPS;
    if (l->steps_len < CD_STEPS)
        l->steps_len++;
}

static double
plan_junction(struct cd_limiter *l, double upx, double upy
              , double umx, double umy, double prev_d, double move_d
              , double prev_accel, double move_accel, double cap_v2
              , const double window_dv[2], const double window_share[2])
{
    double dux = fabs(umx - upx), duy = fabs(umy - upy);
    if (hypot(dux, duy) < l->min_direction_change)
        return cap_v2;
    double prev_xy = hypot(upx, upy), move_xy = hypot(umx, umy);
    if ((move_xy < prev_xy ? move_xy : prev_xy) < l->min_xy_share) {
        l->xy_stops++;
        return 0.;
    }
    double accel = move_accel > prev_accel ? move_accel : prev_accel;
    double turn = atan2(upx * umy - upy * umx, upx * umx + upy * umy);
    double alpha = atan2(upy, upx), cell_turn = turn;
    if (alpha < 0.) {
        alpha = -alpha;
        cell_turn = -cell_turn;
    }
    if (alpha > .5 * M_PI) {
        alpha = M_PI - alpha;
        cell_turn = -cell_turn;
    }
    int cell = (clamped_index(alpha / l->alpha_step, l->n_alpha - 1)
                * l->n_theta
                + clamped_index((cell_turn + M_PI) / l->theta_step
                                , l->n_theta - 1));
    double w = l->w_floor[cell], room = l->floor[cell];
    double speed = 0.;
    if (room > 0.) {
        speed = accel * w;
        if (turn) {
            double curve = curve_speed(l, prev_d * prev_xy, move_d * move_xy
                                       , turn, accel, room);
            if (curve < speed)
                speed = curve;
        }
    }
    double reach = speed * l->window;
    if (speed > 0. && fabs(turn) > l->short_turn
        && (prev_d * prev_xy < reach || move_d * move_xy < reach))
        speed = 0.;
    if (l->has_corner_bound) {
        double up[2] = {upx, upy}, um[2] = {umx, umy}, du[2] = {dux, duy};
        speed = corner_bound(l, up, um, du, accel, speed, window_dv
                             , window_share);
    }
    double v2 = speed * speed;
    if (cap_v2 <= v2)
        return cap_v2;
    l->limited++;
    return v2;
}

double __visible
cd_junction_v2(struct cd_limiter *l, double upx, double upy
               , double umx, double umy, double prev_d, double move_d
               , double prev_accel, double move_accel, double prev_max_v2
               , double cap_v2)
{
    double window_dv[2] = {0., 0.}, window_share[2] = {0., 0.};
    if (!l->has_corner_bound)
        return plan_junction(l, upx, upy, umx, umy, prev_d, move_d
                             , prev_accel, move_accel, cap_v2
                             , window_dv, window_share);
    double tau = prev_d * hypot(upx, upy) / sqrt(prev_max_v2);
    double v2 = 0.;
    if (window_steps(l, tau, window_dv, window_share))
        v2 = plan_junction(l, upx, upy, umx, umy, prev_d, move_d
                           , prev_accel, move_accel, cap_v2
                           , window_dv, window_share);
    note_step(l, tau, v2, upx, upy, umx, umy);
    return v2;
}
