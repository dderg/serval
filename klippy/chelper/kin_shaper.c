// Kinematic input shapers to minimize motion vibrations in XY plane
//
// Copyright (C) 2019-2020  Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2020  Dmitry Butyugin <dmbutyugin@google.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <math.h> // sqrt, exp
#include <stddef.h> // offsetof
#include <stdlib.h> // malloc
#include <string.h> // memset
#include "compiler.h" // __visible
#include "integrate.h" // integrate_weighted
#include "itersolve.h" // struct stepper_kinematics
#include "kin_shaper.h" // struct shaper_pulses
#include "trapq.h" // struct move


/****************************************************************
 * Shaper initialization
 ****************************************************************/

// Shift pulses around 'mid-point' t=0 so that the input shaper is an identity
// transformation for constant-speed motion (i.e. input_shaper(v * T) = v * T)
static void
shift_pulses(struct shaper_pulses *sp)
{
    int i;
    double ts = 0.;
    for (i = 0; i < sp->num_pulses; ++i)
        ts += sp->pulses[i].a * sp->pulses[i].t;
    for (i = 0; i < sp->num_pulses; ++i)
        sp->pulses[i].t -= ts;
}

int
init_shaper(int n, double a[], double t[], struct shaper_pulses *sp)
{
    if (n < 0 || n > ARRAY_SIZE(sp->pulses)) {
        sp->num_pulses = 0;
        return -1;
    }
    int i;
    double sum_a = 0.;
    for (i = 0; i < n; ++i)
        sum_a += a[i];
    double inv_a = 1. / sum_a;
    // Reverse pulses vs their traditional definition
    for (i = 0; i < n; ++i) {
        sp->pulses[n-i-1].a = a[i] * inv_a;
        sp->pulses[n-i-1].t = -t[i];
    }
    sp->num_pulses = n;
    shift_pulses(sp);
    return 0;
}


/****************************************************************
 * Generic position calculation via shaper convolution
 ****************************************************************/

static inline double
get_axis_position(const struct move *m, int axis, double move_time)
{
    double axis_r = m->axes_r.axis[axis - 'x'];
    double start_pos = m->start_pos.axis[axis - 'x'];
    double move_dist = move_get_distance(m, move_time);
    return start_pos + axis_r * move_dist;
}

static inline double
get_axis_position_across_moves(const struct move *m, int axis, double time)
{
    while (likely(time < 0.)) {
        m = list_prev_entry(m, node);
        time += m->move_t;
    }
    while (likely(time > m->move_t)) {
        time -= m->move_t;
        m = list_next_entry(m, node);
    }
    return get_axis_position(m, axis, time);
}

// Calculate the position from the convolution of the shaper with input signal
inline double
shaper_calc_position(const struct move *m, int axis, double move_time
                     , const struct shaper_pulses *sp)
{
    double res = 0.;
    int num_pulses = sp->num_pulses, i;
    for (i = 0; i < num_pulses; ++i) {
        double t = sp->pulses[i].t, a = sp->pulses[i].a;
        res += a * get_axis_position_across_moves(m, axis, move_time + t);
    }
    return res;
}

/****************************************************************
 * Generic position calculation via smoother integration
 ****************************************************************/

// Calculate the definitive integral over a range of moves
static double
range_integrate(const struct move *m, int axis, double move_time
                , const struct smoother *sm)
{
    move_time += sm->t_offs;
    while (unlikely(move_time < 0.)) {
        m = list_prev_entry(m, node);
        move_time += m->move_t;
    }
    while (unlikely(move_time > m->move_t)) {
        move_time -= m->move_t;
        m = list_next_entry(m, node);
    }
    // Calculate integral for the current move
    double start = move_time - sm->hst, end = move_time + sm->hst;
    double t0 = move_time;
    if (unlikely(start >= 0. && end <= m->move_t))
        return integrate_move(m, axis, m->start_pos.axis[axis - 'x'],
                              t0, &sm->pm_diff);
    smoother_antiderivatives left =
        likely(start < 0.) ? calc_antiderivatives(sm, t0) : sm->p_hst;
    smoother_antiderivatives right =
        likely(end > m->move_t) ? calc_antiderivatives(sm, t0 - m->move_t)
                                : sm->m_hst;
    smoother_antiderivatives diff = diff_antiderivatives(&right, &left);
    double res = integrate_move(m, axis, m->start_pos.axis[axis - 'x'],
                                t0, &diff);
    // Integrate over previous moves
    const struct move *prev = m;
    while (likely(start < 0.)) {
        prev = list_prev_entry(prev, node);
        start += prev->move_t;
        t0 += prev->move_t;
        smoother_antiderivatives r = left;
        left = likely(start < 0.) ? calc_antiderivatives(sm, t0)
                                  : sm->p_hst;
        diff = diff_antiderivatives(&r, &left);
        res += integrate_move(prev, axis, prev->start_pos.axis[axis - 'x'],
                              t0, &diff);
    }
    // Integrate over future moves
    t0 = move_time;
    while (likely(end > m->move_t)) {
        end -= m->move_t;
        t0 -= m->move_t;
        m = list_next_entry(m, node);
        smoother_antiderivatives l = right;
        right = likely(end > m->move_t) ? calc_antiderivatives(sm,
                                                               t0 - m->move_t)
                                        : sm->m_hst;
        diff = diff_antiderivatives(&right, &l);
        res += integrate_move(m, axis, m->start_pos.axis[axis - 'x'],
                              t0, &diff);
    }
    return res;
}

// Calculate average position using the specified smoother
static inline double
smoother_calc_position(const struct move *m, int axis, double move_time
                       , const struct smoother *sm)
{
    return range_integrate(m, axis, move_time, sm);
}

/****************************************************************
 * Kinematics-related shaper code
 ****************************************************************/

#define DUMMY_T 500.0

struct input_shaper {
    struct stepper_kinematics sk;
    struct stepper_kinematics *orig_sk;
    struct move m;
    struct shaper_pulses sp_x, sp_y;
    struct smoother sm_x, sm_y;
};

// Optimized calc_position when only x axis is needed
static double
shaper_x_calc_position(struct stepper_kinematics *sk, struct move *m
                       , double move_time)
{
    struct input_shaper *is = container_of(sk, struct input_shaper, sk);
    if (!is->sp_x.num_pulses && !is->sm_x.hst)
        return is->orig_sk->calc_position_cb(is->orig_sk, m, move_time);
    is->m.start_pos.x = is->sp_x.num_pulses
        ?   shaper_calc_position(m, 'x', move_time, &is->sp_x)
        : smoother_calc_position(m, 'x', move_time, &is->sm_x);
    return is->orig_sk->calc_position_cb(is->orig_sk, &is->m, DUMMY_T);
}

// Optimized calc_position when only y axis is needed
static double
shaper_y_calc_position(struct stepper_kinematics *sk, struct move *m
                       , double move_time)
{
    struct input_shaper *is = container_of(sk, struct input_shaper, sk);
    if (!is->sp_y.num_pulses && !is->sm_y.hst)
        return is->orig_sk->calc_position_cb(is->orig_sk, m, move_time);
    is->m.start_pos.y = is->sp_y.num_pulses
        ?   shaper_calc_position(m, 'y', move_time, &is->sp_y)
        : smoother_calc_position(m, 'y', move_time, &is->sm_y);
    return is->orig_sk->calc_position_cb(is->orig_sk, &is->m, DUMMY_T);
}

// General calc_position for both x and y axes
static double
shaper_xy_calc_position(struct stepper_kinematics *sk, struct move *m
                        , double move_time)
{
    struct input_shaper *is = container_of(sk, struct input_shaper, sk);
    if (!is->sp_x.num_pulses && !is->sp_y.num_pulses
            && !is->sm_x.hst && !is->sm_y.hst)
        return is->orig_sk->calc_position_cb(is->orig_sk, m, move_time);
    is->m.start_pos = move_get_coord(m, move_time);
    if (is->sp_x.num_pulses || is->sm_x.hst)
        is->m.start_pos.x = is->sp_x.num_pulses
            ?   shaper_calc_position(m, 'x', move_time, &is->sp_x)
            : smoother_calc_position(m, 'x', move_time, &is->sm_x);
    if (is->sp_y.num_pulses || is->sm_y.hst)
        is->m.start_pos.y = is->sp_y.num_pulses
            ?   shaper_calc_position(m, 'y', move_time, &is->sp_y)
            : smoother_calc_position(m, 'y', move_time, &is->sm_y);
    return is->orig_sk->calc_position_cb(is->orig_sk, &is->m, DUMMY_T);
}

// A callback that forwards post_cb call to the original kinematics
static void
shaper_commanded_pos_post_fixup(struct stepper_kinematics *sk)
{
    struct input_shaper *is = container_of(sk, struct input_shaper, sk);
    is->orig_sk->commanded_pos = sk->commanded_pos;
    is->orig_sk->post_cb(is->orig_sk);
    sk->commanded_pos = is->orig_sk->commanded_pos;
}

static void
shaper_note_generation_time(struct input_shaper *is)
{
    double pre_active = 0., post_active = 0.;
    if ((is->sk.active_flags & AF_X) && is->sp_x.num_pulses) {
        pre_active = is->sp_x.pulses[is->sp_x.num_pulses-1].t;
        post_active = -is->sp_x.pulses[0].t;
    } else if ((is->sk.active_flags & AF_X) && is->sm_x.hst) {
        pre_active = is->sm_x.hst + is->sm_x.t_offs;
        if (pre_active < 0.) pre_active = 0.;
        post_active = is->sm_x.hst - is->sm_x.t_offs;
        if (post_active < 0.) post_active = 0.;
    }
    if ((is->sk.active_flags & AF_Y) && is->sp_y.num_pulses) {
        pre_active = is->sp_y.pulses[is->sp_y.num_pulses-1].t > pre_active
            ? is->sp_y.pulses[is->sp_y.num_pulses-1].t : pre_active;
        post_active = -is->sp_y.pulses[0].t > post_active
            ? -is->sp_y.pulses[0].t : post_active;
    } else if ((is->sk.active_flags & AF_Y) && is->sm_y.hst) {
        pre_active = is->sm_y.hst + is->sm_y.t_offs > pre_active
            ? is->sm_y.hst + is->sm_y.t_offs : pre_active;
        post_active = is->sm_y.hst - is->sm_y.t_offs > post_active
            ? is->sm_y.hst - is->sm_y.t_offs : post_active;
    }
    is->sk.gen_steps_pre_active = pre_active;
    is->sk.gen_steps_post_active = post_active;
}

void __visible
input_shaper_update_sk(struct stepper_kinematics *sk)
{
    struct input_shaper *is = container_of(sk, struct input_shaper, sk);
    if ((is->orig_sk->active_flags & (AF_X | AF_Y)) == (AF_X | AF_Y))
        is->sk.calc_position_cb = shaper_xy_calc_position;
    else if (is->orig_sk->active_flags & AF_X)
        is->sk.calc_position_cb = shaper_x_calc_position;
    else if (is->orig_sk->active_flags & AF_Y)
        is->sk.calc_position_cb = shaper_y_calc_position;
    is->sk.active_flags = is->orig_sk->active_flags;
    shaper_note_generation_time(is);
}

int __visible
input_shaper_set_sk(struct stepper_kinematics *sk
                    , struct stepper_kinematics *orig_sk)
{
    struct input_shaper *is = container_of(sk, struct input_shaper, sk);
    if (orig_sk->active_flags == AF_X)
        is->sk.calc_position_cb = shaper_x_calc_position;
    else if (orig_sk->active_flags == AF_Y)
        is->sk.calc_position_cb = shaper_y_calc_position;
    else if (orig_sk->active_flags & (AF_X | AF_Y))
        is->sk.calc_position_cb = shaper_xy_calc_position;
    else
        return -1;
    is->sk.active_flags = orig_sk->active_flags;
    is->orig_sk = orig_sk;
    is->sk.commanded_pos = orig_sk->commanded_pos;
    is->sk.last_flush_time = orig_sk->last_flush_time;
    is->sk.last_move_time = orig_sk->last_move_time;
    if (orig_sk->post_cb) {
        is->sk.post_cb = shaper_commanded_pos_post_fixup;
    }
    return 0;
}

int __visible
input_shaper_set_shaper_params(struct stepper_kinematics *sk, char axis
                               , int n, double a[], double t[])
{
    if (axis != 'x' && axis != 'y')
        return -1;
    struct input_shaper *is = container_of(sk, struct input_shaper, sk);
    struct shaper_pulses *sp = axis == 'x' ? &is->sp_x : &is->sp_y;
    struct smoother *sm = axis == 'x' ? &is->sm_x : &is->sm_y;
    int status = 0;
    // Ignore input shaper update if the axis is not active
    if (is->orig_sk->active_flags & (axis == 'x' ? AF_X : AF_Y)) {
        status = init_shaper(n, a, t, sp);
        memset(sm, 0, sizeof(*sm));
        shaper_note_generation_time(is);
    }
    return status;
}

int __visible
input_shaper_set_smoother_params(struct stepper_kinematics *sk, char axis
                                 , int n, double a[], double t_sm)
{
    if (axis != 'x' && axis != 'y')
        return -1;
    struct input_shaper *is = container_of(sk, struct input_shaper, sk);
    struct shaper_pulses *sp = axis == 'x' ? &is->sp_x : &is->sp_y;
    struct smoother *sm = axis == 'x' ? &is->sm_x : &is->sm_y;
    int status = 0;
    if (is->orig_sk->active_flags & (axis == 'x' ? AF_X : AF_Y)) {
        status = init_smoother(n, a, t_sm, sm);
        sp->num_pulses = 0;
        shaper_note_generation_time(is);
    }
    return status;
}

struct stepper_kinematics * __visible
input_shaper_alloc(void)
{
    struct input_shaper *is = malloc(sizeof(*is));
    memset(is, 0, sizeof(*is));
    is->m.move_t = 2. * DUMMY_T;
    return &is->sk;
}


/****************************************************************
 * Shaped position, velocity and acceleration (servo setpoints)
 ****************************************************************/

static inline void
axis_pva(const struct move *m, int axis, double move_time, double pva[3])
{
    double axis_r = m->axes_r.axis[axis - 'x'];
    double accel = 2. * m->half_accel;
    pva[0] = get_axis_position(m, axis, move_time);
    pva[1] = axis_r * (m->start_v + accel * move_time);
    pva[2] = axis_r * accel;
}

static void
shaper_calc_pva(const struct move *m, int axis, double move_time
                , const struct shaper_pulses *sp, double pva[3])
{
    pva[0] = pva[1] = pva[2] = 0.;
    int i;
    for (i = 0; i < sp->num_pulses; ++i) {
        double t = move_time + sp->pulses[i].t, a = sp->pulses[i].a;
        const struct move *pm = m;
        while (t < 0.) {
            pm = list_prev_entry(pm, node);
            t += pm->move_t;
        }
        while (t > pm->move_t) {
            t -= pm->move_t;
            pm = list_next_entry(pm, node);
        }
        double p[3];
        axis_pva(pm, axis, t, p);
        pva[0] += a * p[0];
        pva[1] += a * p[1];
        pva[2] += a * p[2];
    }
}

static inline double
axis_velocity_jump(const struct move *prev, const struct move *next, int axis)
{
    double v_end = prev->axes_r.axis[axis - 'x']
        * (prev->start_v + 2. * prev->half_accel * prev->move_t);
    return next->axes_r.axis[axis - 'x'] * next->start_v - v_end;
}

// Derivative of the it0 antiderivative: the smoother weight at s
static double
smoother_density(const struct smoother *sm, double s)
{
    double q = 0., dq = 0.;
    int i;
    for (i = 0; i < sm->n; ++i) {
        dq = dq * s + q;
        q = q * s + sm->c0[i];
    }
    return q + s * dq;
}

static inline void
smoother_add_move_pva(const struct move *m, int axis, double t0
                      , const smoother_antiderivatives *ad, double pva[3])
{
    pva[0] += integrate_move(m, axis, m->start_pos.axis[axis - 'x'], t0, ad);
    pva[1] += integrate_velocity(m, axis, t0, ad);
    pva[2] += 2. * m->half_accel * m->axes_r.axis[axis - 'x'] * ad->it0;
}

// range_integrate() of position, velocity and acceleration. A velocity
// jump at a move boundary inside the kernel window contributes the jump
// times the kernel weight at the boundary to the acceleration.
static void
smoother_calc_pva(const struct move *m, int axis, double move_time
                  , const struct smoother *sm, double pva[3])
{
    pva[0] = pva[1] = pva[2] = 0.;
    move_time += sm->t_offs;
    while (unlikely(move_time < 0.)) {
        m = list_prev_entry(m, node);
        move_time += m->move_t;
    }
    while (unlikely(move_time > m->move_t)) {
        move_time -= m->move_t;
        m = list_next_entry(m, node);
    }
    double start = move_time - sm->hst, end = move_time + sm->hst;
    double t0 = move_time;
    if (unlikely(start >= 0. && end <= m->move_t)) {
        smoother_add_move_pva(m, axis, t0, &sm->pm_diff, pva);
        return;
    }
    smoother_antiderivatives left =
        likely(start < 0.) ? calc_antiderivatives(sm, t0) : sm->p_hst;
    smoother_antiderivatives right =
        likely(end > m->move_t) ? calc_antiderivatives(sm, t0 - m->move_t)
                                : sm->m_hst;
    smoother_antiderivatives diff = diff_antiderivatives(&right, &left);
    smoother_add_move_pva(m, axis, t0, &diff, pva);
    const struct move *prev = m;
    while (likely(start < 0.)) {
        const struct move *later = prev;
        prev = list_prev_entry(prev, node);
        pva[2] += axis_velocity_jump(prev, later, axis)
            * smoother_density(sm, t0);
        start += prev->move_t;
        t0 += prev->move_t;
        smoother_antiderivatives r = left;
        left = likely(start < 0.) ? calc_antiderivatives(sm, t0)
                                  : sm->p_hst;
        diff = diff_antiderivatives(&r, &left);
        smoother_add_move_pva(prev, axis, t0, &diff, pva);
    }
    t0 = move_time;
    while (likely(end > m->move_t)) {
        const struct move *earlier = m;
        end -= m->move_t;
        t0 -= m->move_t;
        m = list_next_entry(m, node);
        pva[2] += axis_velocity_jump(earlier, m, axis)
            * smoother_density(sm, t0);
        smoother_antiderivatives l = right;
        right = likely(end > m->move_t) ? calc_antiderivatives(sm,
                                                               t0 - m->move_t)
                                        : sm->m_hst;
        diff = diff_antiderivatives(&right, &l);
        smoother_add_move_pva(m, axis, t0, &diff, pva);
    }
}

static void
shaped_axis_pva(const struct move *m, int axis, double move_time
                , const struct shaper_pulses *sp, const struct smoother *sm
                , double pva[3])
{
    if (sp->num_pulses)
        shaper_calc_pva(m, axis, move_time, sp, pva);
    else if (sm->hst)
        smoother_calc_pva(m, axis, move_time, sm, pva);
    else
        axis_pva(m, axis, move_time, pva);
}

int
input_shaper_calc_pva(struct stepper_kinematics *sk, const struct move *m
                      , double move_time, struct coord *pva)
{
    if (sk->calc_position_cb != shaper_x_calc_position
        && sk->calc_position_cb != shaper_y_calc_position
        && sk->calc_position_cb != shaper_xy_calc_position)
        return -1;
    struct input_shaper *is = container_of(sk, struct input_shaper, sk);
    double axes[3][3];
    shaped_axis_pva(m, 'x', move_time, &is->sp_x, &is->sm_x, axes[0]);
    shaped_axis_pva(m, 'y', move_time, &is->sp_y, &is->sm_y, axes[1]);
    axis_pva(m, 'z', move_time, axes[2]);
    int axis, order;
    for (axis = 0; axis < 3; ++axis)
        for (order = 0; order < 3; ++order)
            pva[order].axis[axis] = axes[axis][order];
    return 0;
}
