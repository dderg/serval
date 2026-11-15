// Junction speed of [corner_deviation] (klippy/extras/corner_deviation.py
// builds the tables and documents the limits)
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <math.h> // atan2, hypot, sqrt
#include <stdint.h> // uint64_t
#include "compiler.h" // __visible

#pragma GCC optimize ("fp-contract=off")

#define CD_STEPS 256
#define CD_SEGMENTS 32
#define CD_NODES (CD_SEGMENTS + 1)
#define CD_CDF_SUB 16
#define CD_CDF_NODES ((3 * CD_SEGMENTS + 2) * CD_CDF_SUB + 1)
#define CD_ROWS 4
#define CD_SOLVER_STEPS 6
#define CD_SOLVER_TOLERANCE 0.005

// Kernel masses: the part common to both axis kernels, min(w_x, w_y),
// and each axis' excess over it
enum { CD_COMMON, CD_EXCESS_X, CD_EXCESS_Y, CD_MASSES };

// Stretches of time around the junction, in order, with what the legs'
// acceleration can be there: (-inf, -d] any move of the window before
// the junction, any sign; [-d, 0] the move before it, decelerating or
// cruising (d: the longest it can decelerate into the junction); [0, g]
// the move after it, accelerating or cruising (g: the earliest it can
// start to decelerate); [g, tau] that move, any sign (tau: its shortest
// duration); [tau, inf) any later move in any direction at the largest
// acceleration of the window
enum {
    CD_WINDOW, CD_PREV_DOWN, CD_NEXT_UP, CD_NEXT_FREE, CD_FUTURE
    , CD_REGIONS, CD_BOUNDARIES = CD_REGIONS - 1
};

struct cd_step {
    double tau, v, accel, du[2], leg[2];
};

struct cd_shape {
    int n;
    double u0, h;
    double wmax[2][CD_SEGMENTS], wmin[2][CD_SEGMENTS];
    double cdf_pad, cdf_h, cdf_margin;
    double cdf[CD_MASSES][CD_CDF_NODES];
    double total[CD_MASSES];
};

struct cd_limiter {
    int n_alpha, n_theta;
    double alpha_step, theta_step;
    double *floor, *w_floor;
    double variance, window, max_corner_accel, short_turn;
    int has_corner_bound, n_rows;
    double rows[CD_ROWS][2];
    struct cd_shape shape;
    double min_direction_change, min_xy_share;
    uint64_t limited, step_limited, xy_stops;
    struct cd_step steps[CD_STEPS];
    int steps_len, steps_head;
};

struct cd_window {
    int n;
    double gap[CD_STEPS];
    const struct cd_step *step[CD_STEPS];
};

// Per row and sign of the executed acceleration: the junction's own burst
// per unit speed and the earlier steps' bursts on each segment, and the
// legs' acceleration per kernel mass: the whole kernel at the weight of
// the first stretch (base), and at each later stretch's start the rise
// and fall of the weight, times the kernel mass past that boundary
struct cd_row_bound {
    double own[CD_SEGMENTS], earlier[CD_SEGMENTS];
    double base, margin;
    double rise[CD_BOUNDARIES][CD_MASSES], fall[CD_BOUNDARIES][CD_MASSES];
};

typedef double cd_masses[CD_MASSES][CD_NODES];

struct cd_junction {
    double prev_accel, move_accel, forward_v2, move_d, prev_tau, move_tau;
    cd_masses at, next_end;
    int n_bounds;
    struct cd_row_bound bounds[2 * CD_ROWS];
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
positive(double x)
{
    return x > 0. ? x : 0.;
}

static double
lesser(double a, double b)
{
    return a < b ? a : b;
}

// A leg of axis coefficients (cx, cy) as weights of the three masses,
// for accelerations of sign `sign` (0: either sign)
static void
leg_weight(double out[CD_MASSES], double cx, double cy, double sign)
{
    if (sign) {
        out[CD_COMMON] = positive(sign * (cx + cy));
        out[CD_EXCESS_X] = positive(sign * cx);
        out[CD_EXCESS_Y] = positive(sign * cy);
        return;
    }
    out[CD_COMMON] = fabs(cx + cy);
    out[CD_EXCESS_X] = fabs(cx);
    out[CD_EXCESS_Y] = fabs(cy);
}

static void
widen_weight(double w[CD_MASSES], const double other[CD_MASSES])
{
    int m;
    for (m = 0; m < CD_MASSES; m++)
        if (other[m] > w[m])
            w[m] = other[m];
}

// Upper bound on each segment of the burst of a step with axis
// coefficients (cx, cy)
static void
burst_segmax(const struct cd_shape *sh, double cx, double cy
             , double out[CD_SEGMENTS])
{
    const double *wx = cx > 0. ? sh->wmax[0] : sh->wmin[0];
    const double *wy = cy > 0. ? sh->wmax[1] : sh->wmin[1];
    int i;
    for (i = 0; i < sh->n; i++)
        out[i] = positive(cx * wx[i] + cy * wy[i]);
}

// Segments a time offset spans (the segment that offset falls in)
static int
segments_in(const struct cd_shape *sh, double offset)
{
    double s = floor(offset / sh->h);
    if (s > sh->n)
        return sh->n;
    return s < -sh->n ? -sh->n : (int)s;
}

// later[i + shift], 0 past the window's end, later[0] before its start
static double
shifted(const struct cd_shape *sh, const double *later, int i, int shift)
{
    int k = i + shift;
    if (k >= sh->n)
        return 0.;
    return later[k < 0 ? 0 : k];
}

// Largest sum, per segment of the junction's window, of the earlier
// steps' bursts over every placement that keeps at least their gaps
static void
earlier_steps(const struct cd_shape *sh, const struct cd_window *win
              , const double row[2], double polarity
              , double out[CD_SEGMENTS])
{
    double best[CD_SEGMENTS], gap = 0.;
    int i, k, any = 0;
    for (k = win->n - 1; k >= 0; k--) {
        const struct cd_step *st = win->step[k];
        double scale = polarity * st->v;
        double cx = scale * row[0] * st->du[0];
        double cy = scale * row[1] * st->du[1];
        if (k < win->n - 1)
            gap += win->gap[k + 1];
        if (cx <= 0. && cy <= 0.)
            continue;
        double here[CD_SEGMENTS], running = 0.;
        int shift = segments_in(sh, gap);
        burst_segmax(sh, cx, cy, here);
        for (i = sh->n - 1; i >= 0; i--) {
            if (any)
                here[i] += shifted(sh, best, i, shift);
            if (here[i] > running)
                running = here[i];
            here[i] = running;
        }
        for (i = 0; i < sh->n; i++)
            best[i] = here[i];
        any = 1;
        gap = 0.;
    }
    if (!any) {
        for (i = 0; i < sh->n; i++)
            out[i] = 0.;
        return;
    }
    int shift = segments_in(sh, gap + win->gap[0]);
    for (i = 0; i < sh->n; i++)
        out[i] = shifted(sh, best, i, shift);
}

// Kernel masses past a boundary at time s after the junction, at each
// node of the window (the cdf at node time - s; the table's spacing
// divides the window's, so every node shares one interpolation weight)
static void
masses_past(const struct cd_shape *sh, double s, cd_masses out)
{
    double shift = -s;
    if (shift > sh->cdf_pad)
        shift = sh->cdf_pad;
    if (shift < -sh->cdf_pad)
        shift = -sh->cdf_pad;
    double x = (shift + sh->cdf_pad) / sh->cdf_h;
    int k = (int)x, m, i;
    if (k > CD_CDF_NODES - 2 - sh->n * CD_CDF_SUB)
        k = CD_CDF_NODES - 2 - sh->n * CD_CDF_SUB;
    double f = x - k;
    for (m = 0; m < CD_MASSES; m++) {
        const double *c = &sh->cdf[m][k];
        for (i = 0; i <= sh->n; i++) {
            const double *p = &c[i * CD_CDF_SUB];
            out[m][i] = p[0] + f * (p[1] - p[0]);
        }
    }
}

// Longest a move ending at speed v can decelerate into its end (its
// peak speed being at most that of an accel-decel triangle from the
// fastest start forward_v2 allows), and not before it starts
static double
prev_decel_time(const struct cd_junction *j, double v)
{
    double peak = sqrt(.5 * (j->forward_v2 + v * v));
    return lesser(positive((peak - v) / j->prev_accel), j->prev_tau);
}

// Shortest time after the junction at which the next move can start to
// decelerate (to any end speed, so its accel-decel triangle is lowest
// when it stops), and not after its shortest duration
static double
next_decel_time(const struct cd_junction *j, double v)
{
    double reach_v2 = 2. * j->move_accel * j->move_d;
    if (reach_v2 < v * v)
        reach_v2 = v * v;
    double t = positive((sqrt(.5 * (reach_v2 + v * v)) - v) / j->move_accel);
    return lesser(t, j->move_tau);
}

// Largest junction speed s whose burst fits under max_corner_accel with
// the legs as they can be at a junction passed at speed v (0 if it fits
// at no speed). Each boundary's mass grows with time, so on a segment a
// rising weight is taken at its end and a falling one at its start. The
// legs' model of speed v also holds at every slower junction speed: the
// stretches whose acceleration sign it restricts only grow.
static double
fitting_speed(const struct cd_limiter *l, const struct cd_junction *j
              , double v)
{
    const struct cd_shape *sh = &l->shape;
    cd_masses prev_decel, next_decel;
    const cd_masses *mass[CD_BOUNDARIES] = {
        &prev_decel, &j->at, &next_decel, &j->next_end};
    double best = INFINITY;
    int b, i, k, m;
    masses_past(sh, -prev_decel_time(j, v), prev_decel);
    masses_past(sh, next_decel_time(j, v), next_decel);
    for (b = 0; b < j->n_bounds; b++) {
        const struct cd_row_bound *rb = &j->bounds[b];
        double legs[CD_SEGMENTS];
        for (i = 0; i < sh->n; i++)
            legs[i] = rb->base + rb->margin;
        for (k = 0; k < CD_BOUNDARIES; k++)
            for (m = 0; m < CD_MASSES; m++) {
                double rise = rb->rise[k][m], fall = rb->fall[k][m];
                const double *past = (*mass[k])[m];
                if (rise)
                    for (i = 0; i < sh->n; i++)
                        legs[i] += rise * past[i + 1];
                if (fall)
                    for (i = 0; i < sh->n; i++)
                        legs[i] -= fall * past[i];
            }
        for (i = 0; i < sh->n; i++) {
            if (!rb->own[i])
                continue;
            double room = l->max_corner_accel - rb->earlier[i] - legs[i];
            if (room <= 0.)
                return 0.;
            if (room < best * rb->own[i])
                best = room / rb->own[i];
        }
    }
    return best;
}

static void
set_legs(const struct cd_shape *sh, struct cd_row_bound *rb
         , double weight[CD_REGIONS][CD_MASSES])
{
    int r, m;
    rb->base = rb->margin = 0.;
    for (m = 0; m < CD_MASSES; m++)
        rb->base += weight[CD_WINDOW][m] * sh->total[m];
    for (r = 1; r < CD_REGIONS; r++)
        for (m = 0; m < CD_MASSES; m++) {
            double jump = weight[r][m] - weight[r - 1][m];
            rb->rise[r - 1][m] = positive(jump);
            rb->fall[r - 1][m] = positive(-jump);
            rb->margin += fabs(jump) * sh->cdf_margin;
        }
}

static void
setup_bounds(const struct cd_limiter *l, struct cd_junction *j
             , const struct cd_window *win, const double up[2]
             , const double um[2])
{
    const struct cd_shape *sh = &l->shape;
    double du[2] = {um[0] - up[0], um[1] - up[1]};
    double prev_leg[2] = {j->prev_accel * up[0], j->prev_accel * up[1]};
    double move_leg[2] = {j->move_accel * um[0], j->move_accel * um[1]};
    double accel = j->prev_accel > j->move_accel ? j->prev_accel
                   : j->move_accel;
    int r, p, k;
    for (k = 0; k < win->n; k++)
        if (win->step[k]->accel > accel)
            accel = win->step[k]->accel;
    j->n_bounds = 0;
    for (r = 0; r < l->n_rows; r++) {
        const double *row = l->rows[r];
        double weight[CD_REGIONS][CD_MASSES], step_leg[CD_MASSES];
        double pc[2] = {row[0] * prev_leg[0], row[1] * prev_leg[1]};
        double nc[2] = {row[0] * move_leg[0], row[1] * move_leg[1]};
        leg_weight(weight[CD_WINDOW], pc[0], pc[1], 0.);
        for (k = 0; k < win->n; k++) {
            leg_weight(step_leg, row[0] * win->step[k]->leg[0]
                       , row[1] * win->step[k]->leg[1], 0.);
            widen_weight(weight[CD_WINDOW], step_leg);
        }
        leg_weight(weight[CD_NEXT_FREE], nc[0], nc[1], 0.);
        weight[CD_FUTURE][CD_COMMON] = accel * hypot(row[0], row[1]);
        weight[CD_FUTURE][CD_EXCESS_X] = accel * fabs(row[0]);
        weight[CD_FUTURE][CD_EXCESS_Y] = accel * fabs(row[1]);
        for (p = 0; p < 2; p++) {
            double polarity = p ? -1. : 1.;
            struct cd_row_bound *rb = &j->bounds[j->n_bounds];
            double cx = polarity * row[0] * du[0];
            double cy = polarity * row[1] * du[1];
            if (cx <= 0. && cy <= 0.)
                continue;
            burst_segmax(sh, cx, cy, rb->own);
            earlier_steps(sh, win, row, polarity, rb->earlier);
            leg_weight(weight[CD_PREV_DOWN], pc[0], pc[1], -polarity);
            leg_weight(weight[CD_NEXT_UP], nc[0], nc[1], polarity);
            set_legs(sh, rb, weight);
            j->n_bounds++;
        }
    }
    masses_past(sh, 0., j->at);
    masses_past(sh, j->move_tau, j->next_end);
}

// Largest junction speed up to speed whose executed acceleration on
// every row stays within max_corner_accel. The speed that fits with the
// legs of a faster junction is safe, and the fitting speed does not grow
// with the junction speed: the search keeps a safe speed and a faster
// unsafe one, tries the secant root between them, and ends once the safe
// speed fits only itself.
static double
corner_bound(struct cd_limiter *l, struct cd_junction *j, double speed)
{
    double fits = fitting_speed(l, j, speed);
    if (fits >= speed)
        return speed;
    l->step_limited++;
    double safe = fits, unsafe = speed, unsafe_gap = fits - speed;
    double safe_gap = fitting_speed(l, j, safe) - safe;
    int k;
    for (k = 0; k < CD_SOLVER_STEPS; k++) {
        double tolerance = CD_SOLVER_TOLERANCE * unsafe;
        if (unsafe - safe <= tolerance || safe_gap <= tolerance)
            break;
        double next = .5 * (safe + unsafe);
        if (safe_gap > 0. && safe_gap - unsafe_gap > 0.)
            next = safe + safe_gap * (unsafe - safe) / (safe_gap - unsafe_gap);
        if (!(next > safe && next < unsafe))
            next = .5 * (safe + unsafe);
        fits = fitting_speed(l, j, next);
        if (fits >= next) {
            safe = next;
            safe_gap = fits - next;
        } else {
            unsafe = next;
            unsafe_gap = fits - next;
            if (fits > safe) {
                safe = fits;
                safe_gap = fitting_speed(l, j, safe) - safe;
            }
        }
    }
    return safe;
}

static int
window_steps(struct cd_limiter *l, double tau, struct cd_window *win)
{
    double span = l->shape.n * l->shape.h, elapsed = tau;
    int i;
    win->n = 0;
    for (i = 0; i < l->steps_len; i++) {
        if (elapsed > span)
            return 1;
        const struct cd_step *st = &l->steps[
            (l->steps_head - 1 - i + CD_STEPS) % CD_STEPS];
        win->gap[win->n] = win->n ? win->step[win->n - 1]->tau : tau;
        win->step[win->n] = st;
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
          , double upy, double umx, double umy, double prev_accel)
{
    struct cd_step *s = &l->steps[l->steps_head];
    s->tau = tau;
    s->v = sqrt(v2);
    s->accel = prev_accel;
    s->du[0] = umx - upx;
    s->du[1] = umy - upy;
    s->leg[0] = prev_accel * upx;
    s->leg[1] = prev_accel * upy;
    l->steps_head = (l->steps_head + 1) % CD_STEPS;
    if (l->steps_len < CD_STEPS)
        l->steps_len++;
}

static double
table_speed(struct cd_limiter *l, double upx, double upy, double umx
            , double umy, double prev_d, double move_d, double accel)
{
    double prev_xy = hypot(upx, upy), move_xy = hypot(umx, umy);
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
    return speed;
}

static double
bounded_speed(struct cd_limiter *l, double upx, double upy, double umx
              , double umy, double prev_d, double move_d, double prev_accel
              , double move_accel, double prev_max_v, double move_max_v2
              , double prev_start_v2, double speed)
{
    struct cd_window win;
    struct cd_junction j;
    double up[2] = {upx, upy}, um[2] = {umx, umy};
    double prev_xy = hypot(upx, upy), move_xy = hypot(umx, umy);
    double move_max_v = sqrt(move_max_v2);
    j.prev_accel = prev_accel;
    j.move_accel = move_accel;
    j.forward_v2 = prev_start_v2 + 2. * prev_accel * prev_d;
    j.move_d = move_d;
    j.prev_tau = traverse_time(prev_d * prev_xy, sqrt(prev_start_v2), speed
                               , prev_accel, prev_max_v);
    j.move_tau = traverse_time(move_d * move_xy, speed, move_max_v
                               , move_accel, move_max_v);
    if (!window_steps(l, j.prev_tau, &win))
        return 0.;
    setup_bounds(l, &j, &win, up, um);
    return corner_bound(l, &j, speed);
}

double __visible
cd_junction_v2(struct cd_limiter *l, double upx, double upy
               , double umx, double umy, double prev_d, double move_d
               , double prev_accel, double move_accel, double prev_max_v2
               , double move_max_v2, double prev_start_v2, double cap_v2)
{
    double prev_xy = hypot(upx, upy), move_xy = hypot(umx, umy);
    double prev_max_v = sqrt(prev_max_v2), v2 = cap_v2;
    if ((move_xy < prev_xy ? move_xy : prev_xy) < l->min_xy_share) {
        l->xy_stops++;
        v2 = 0.;
    } else if (hypot(umx - upx, umy - upy) >= l->min_direction_change) {
        double accel = move_accel > prev_accel ? move_accel : prev_accel;
        double speed = table_speed(l, upx, upy, umx, umy, prev_d, move_d
                                   , accel);
        double cap = sqrt(cap_v2);
        if (speed > cap)
            speed = cap;
        if (l->has_corner_bound && speed > 0.)
            speed = bounded_speed(l, upx, upy, umx, umy, prev_d, move_d
                                  , prev_accel, move_accel, prev_max_v
                                  , move_max_v2, prev_start_v2, speed);
        if (speed < cap) {
            v2 = speed * speed;
            l->limited++;
        }
    }
    if (l->has_corner_bound)
        note_step(l, traverse_time(prev_d * prev_xy, sqrt(prev_start_v2)
                                   , sqrt(v2), prev_accel, prev_max_v)
                  , v2, upx, upy, umx, umy, prev_accel);
    return v2;
}
