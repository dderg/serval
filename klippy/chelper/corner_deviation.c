// Junction speed of [corner_deviation] (klippy/extras/corner_deviation.py
// builds the tables and documents the limits)
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <math.h> // atan2, hypot, sqrt
#include <stdint.h> // uint64_t
#include "compiler.h" // __visible

#pragma GCC optimize ("fp-contract=off")

struct cd_limiter {
    int n_alpha, n_theta, n_w;
    double alpha_step, theta_step;
    double *w, *g, *floor, *w_floor;
    double variance, window, max_deviation, max_corner_accel;
    int has_cap, has_corner_bound;
    double density_peak[2];
    double min_direction_change, min_xy_share;
    uint64_t limited, stopped_by_cap, step_limited, xy_stops;
};

static double
max_w_within(const struct cd_limiter *l, const double *g, double level)
{
    const double *w = l->w;
    if (level < g[0])
        return 0.;
    int lo = 0, hi = l->n_w - 1;
    if (g[hi] <= level)
        return w[hi];
    while (hi - lo > 1) {
        int mid = (lo + hi) / 2;
        if (g[mid] <= level)
            lo = mid;
        else
            hi = mid;
    }
    double frac = g[hi] > g[lo] ? (level - g[lo]) / (g[hi] - g[lo]) : 0.;
    return w[lo] + frac * (w[hi] - w[lo]);
}

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
             , const double du[2], double prev_d, double move_d, double accel
             , double speed)
{
    double rate = 1. / (move_d < prev_d ? move_d : prev_d);
    double bound = speed;
    int axis;
    for (axis = 0; axis < 2; axis++) {
        double density_peak = l->density_peak[axis];
        if (!density_peak || !du[axis])
            continue;
        double linear = du[axis] * density_peak;
        double share = fabs(um[axis]) > fabs(up[axis])
                       ? fabs(um[axis]) : fabs(up[axis]);
        double room = l->max_corner_accel - accel * share;
        if (room <= 0.) {
            bound = 0.;
            continue;
        }
        double curve = 2. * room / (linear + sqrt(
            linear * linear + 4. * du[axis] * rate * room));
        if (curve < bound)
            bound = curve;
    }
    if (bound < speed)
        l->step_limited++;
    return bound;
}

double __visible
cd_junction_v2(struct cd_limiter *l, double upx, double upy
               , double umx, double umy, double prev_d, double move_d
               , double prev_accel, double move_accel, double cap_v2)
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
    const double *g = &l->g[cell * l->n_w];
    double w = l->w_floor[cell], room = l->floor[cell];
    if (l->has_cap) {
        double level = l->max_deviation / accel;
        if (level < g[0]) {
            l->stopped_by_cap++;
            w = room = 0.;
        } else {
            double capped_w = max_w_within(l, g, level);
            if (capped_w < w)
                w = capped_w;
            if (level < room)
                room = level;
        }
    }
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
    if (l->has_corner_bound) {
        double up[2] = {upx, upy}, um[2] = {umx, umy}, du[2] = {dux, duy};
        speed = corner_bound(l, up, um, du, prev_d, move_d, accel, speed);
    }
    double v2 = speed * speed;
    if (cap_v2 <= v2)
        return cap_v2;
    l->limited++;
    return v2;
}
