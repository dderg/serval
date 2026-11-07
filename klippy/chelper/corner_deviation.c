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
    double tau, v, dv[2], share[2];
};

#define CD_SHAPE_NODES 32

struct cd_shape {
    int n;
    double u0, h;
    double segmax[CD_SHAPE_NODES];
};

struct cd_window {
    int n;
    double first_gap, gap[CD_STEPS], dv[CD_STEPS][2], share[2];
};

struct cd_limiter {
    int n_alpha, n_theta;
    double alpha_step, theta_step;
    double *floor, *w_floor;
    double variance, window, max_corner_accel, short_turn;
    int has_corner_bound;
    double density_peak[2];
    struct cd_shape shape[2];
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
later_at(const struct cd_shape *sh, const double *later, double u)
{
    int i = (int)((u - sh->u0) / sh->h);
    if (i >= sh->n)
        return 0.;
    return later[i < 0 ? 0 : i];
}

static void
earlier_steps(const struct cd_shape *sh, const struct cd_window *win
              , int axis, double best[CD_SHAPE_NODES])
{
    double next[CD_SHAPE_NODES];
    int i, k;
    for (i = 0; i < sh->n; i++)
        best[i] = 0.;
    for (k = win->n - 1; k >= 0; k--) {
        double running = 0.;
        for (i = sh->n - 1; i >= 0; i--) {
            double here = (win->dv[k][axis] * sh->segmax[i]
                           + later_at(sh, best, sh->u0 + i * sh->h
                                      + win->gap[k]));
            if (here > running)
                running = here;
            next[i] = running;
        }
        for (i = 0; i < sh->n; i++)
            best[i] = next[i];
    }
}

static double
corner_bound(struct cd_limiter *l, const double up[2], const double um[2]
             , const double du[2], double accel, double speed
             , const struct cd_window *win)
{
    double bound = speed, best[CD_SHAPE_NODES];
    int axis, i;
    for (axis = 0; axis < 2; axis++) {
        const struct cd_shape *sh = &l->shape[axis];
        if (!sh->n || !du[axis])
            continue;
        double share = fabs(um[axis]) > fabs(up[axis])
                       ? fabs(um[axis]) : fabs(up[axis]);
        if (win->share[axis] > share)
            share = win->share[axis];
        double room = l->max_corner_accel - accel * share;
        earlier_steps(sh, win, axis, best);
        for (i = 0; i < sh->n; i++) {
            double own = room - later_at(sh, best, sh->u0 + i * sh->h
                                         + win->first_gap);
            if (own <= 0.) {
                bound = 0.;
                break;
            }
            own /= du[axis] * sh->segmax[i];
            if (own < bound)
                bound = own;
        }
    }
    if (bound < speed)
        l->step_limited++;
    return bound;
}

static double
window_span(const struct cd_limiter *l)
{
    double span = 0.;
    int axis;
    for (axis = 0; axis < 2; axis++) {
        const struct cd_shape *sh = &l->shape[axis];
        if (sh->n && sh->n * sh->h > span)
            span = sh->n * sh->h;
    }
    return span;
}

static int
window_steps(struct cd_limiter *l, double tau, struct cd_window *win)
{
    double span = window_span(l), elapsed = tau;
    int i;
    win->n = 0;
    win->first_gap = tau;
    win->share[0] = win->share[1] = 0.;
    for (i = 0; i < l->steps_len; i++) {
        if (elapsed > span)
            return 1;
        const struct cd_step *st = &l->steps[
            (l->steps_head - 1 - i + CD_STEPS) % CD_STEPS];
        int axis;
        win->gap[win->n] = st->tau;
        for (axis = 0; axis < 2; axis++) {
            win->dv[win->n][axis] = st->dv[axis];
            if (st->share[axis] > win->share[axis])
                win->share[axis] = st->share[axis];
        }
        win->n++;
        elapsed += st->tau;
    }
    return elapsed > span || l->steps_len < CD_STEPS;
}

static double
traverse_time(double length, double va, double vb, double accel
              , double vmax)
{
    double peak = sqrt(.5 * (va * va + vb * vb) + accel * length);
    if (vmax < peak)
        peak = vmax;
    return peak > 0. ? length / peak : 0.;
}

static void
note_step(struct cd_limiter *l, double tau, double v2, double upx
          , double upy, double umx, double umy)
{
    struct cd_step *s = &l->steps[l->steps_head];
    double v = sqrt(v2);
    s->tau = tau;
    s->v = v;
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
              , double va, double prev_max_v, struct cd_window *win)
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
        double vb = cap_v2 < speed * speed ? sqrt(cap_v2) : speed;
        double tau = traverse_time(prev_d * prev_xy, va, vb, prev_accel
                                   , prev_max_v);
        if (window_steps(l, tau, win))
            speed = corner_bound(l, up, um, du, accel, speed, win);
        else
            speed = 0.;
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
               , int chained, double cap_v2)
{
    struct cd_window win;
    win.n = 0;
    win.share[0] = win.share[1] = 0.;
    double prev_max_v = sqrt(prev_max_v2), va = prev_max_v;
    if (chained && l->steps_len)
        va = l->steps[(l->steps_head - 1 + CD_STEPS) % CD_STEPS].v;
    double v2 = plan_junction(l, upx, upy, umx, umy, prev_d, move_d
                              , prev_accel, move_accel, cap_v2, va
                              , prev_max_v, &win);
    if (l->has_corner_bound)
        note_step(l, traverse_time(prev_d * hypot(upx, upy), va, sqrt(v2)
                                   , prev_accel, prev_max_v)
                  , v2, upx, upy, umx, umy);
    return v2;
}
