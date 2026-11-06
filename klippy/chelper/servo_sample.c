// Sample a stepper's commanded motion on a uniform time grid
//
// Copyright (C) 2026  Kalico contributors
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <math.h> // fabs
#include <stddef.h> // offsetof
#include <stdlib.h> // calloc
#include "compiler.h" // __visible
#include "itersolve.h" // struct stepper_kinematics
#include "kin_shaper.h" // input_shaper_calc_pva
#include "trapq.h" // struct move

#define SHAPED_POSITION_REL_TOLERANCE 1e-9

enum {
    SS_OK = 0,
    SS_NO_KINEMATICS = -1,
    SS_BEFORE_TRAPQ_HISTORY = -2,
    SS_POSITION_MISMATCH = -3,
    SS_NOT_FINITE = -4,
};

struct servo_sampler {
    struct stepper_kinematics *sk;
    double offset;
    struct coord row;
};

struct servo_sample_error {
    double time, position, linear_position;
};

struct servo_sampler * __visible
servo_sampler_alloc(void)
{
    return calloc(1, sizeof(struct servo_sampler));
}

void __visible
servo_sampler_free(struct servo_sampler *ss)
{
    free(ss);
}

// The stepper position must be offset + row . (x, y, z) for every
// toolhead coordinate; the caller verifies that before setting it.
void __visible
servo_sampler_set_kinematics(struct servo_sampler *ss
                             , struct stepper_kinematics *sk, double offset
                             , double row_x, double row_y, double row_z)
{
    ss->sk = sk;
    ss->offset = offset;
    ss->row = (struct coord) { .x = row_x, .y = row_y, .z = row_z };
}

static inline double
row_dot(const struct coord *row, const struct coord *c)
{
    return row->x * c->x + row->y * c->y + row->z * c->z;
}

static void
trapq_move_pva(const struct move *m, double move_time, struct coord *pva)
{
    double accel = 2. * m->half_accel;
    double velocity = m->start_v + accel * move_time;
    pva[0] = move_get_coord(m, move_time);
    int i;
    for (i = 0; i < 3; ++i) {
        pva[1].axis[i] = m->axes_r.axis[i] * velocity;
        pva[2].axis[i] = m->axes_r.axis[i] * accel;
    }
}

// The move leaves this servo where it is: the toolhead stands still, or
// moves only along directions the servo does not follow (a Y homing move
// for an X servo)
static inline int
move_is_stationary(const struct servo_sampler *ss, const struct move *m)
{
    return (!m->start_v && !m->half_accel) || !row_dot(&ss->row, &m->axes_r);
}

// End of the newest finalized move that moves this servo: a sample whose
// shaper window reaches back before it would need moves the trapq no longer
// has
static double
finalized_motion_end(const struct servo_sampler *ss)
{
    struct move *m;
    list_for_each_entry(m, &ss->sk->tq->history, node) {
        if (!move_is_stationary(ss, m))
            return m->print_time + m->move_t;
    }
    return -INFINITY;
}

static int
sample_one(struct servo_sampler *ss, const struct move *first
           , const struct move *m, double finalized_end, double t
           , double *pos, double *vel, double *acc
           , struct servo_sample_error *err)
{
    struct stepper_kinematics *sk = ss->sk;
    struct move *tail = list_last_entry(&sk->tq->moves, struct move, node);
    double first_end = first->print_time + first->move_t;
    double window_start = t - sk->gen_steps_pre_active;
    if (window_start < finalized_end) {
        err->time = t;
        return SS_BEFORE_TRAPQ_HISTORY;
    }
    if (first == tail) {
        *pos = sk->commanded_pos;
        *vel = *acc = 0.;
        return SS_OK;
    }
    if (move_is_stationary(ss, first)
        && t + sk->gen_steps_post_active < first_end) {
        *pos = itersolve_calc_position_from_coord(
            sk, first->start_pos.x, first->start_pos.y, first->start_pos.z);
        *vel = *acc = 0.;
        return SS_OK;
    }
    if (window_start < first->print_time) {
        err->time = t;
        return SS_BEFORE_TRAPQ_HISTORY;
    }
    double move_time = t - m->print_time;
    struct move *mm = (struct move *)m;
    double p = sk->calc_position_cb(sk, mm, move_time);
    struct coord pva[3];
    if (input_shaper_calc_pva(sk, m, move_time, pva) < 0)
        trapq_move_pva(m, move_time, pva);
    double lp = ss->offset + row_dot(&ss->row, &pva[0]);
    double v = row_dot(&ss->row, &pva[1]), a = row_dot(&ss->row, &pva[2]);
    err->time = t;
    err->position = p;
    err->linear_position = lp;
    if (!isfinite(p) || !isfinite(lp) || !isfinite(v) || !isfinite(a))
        return SS_NOT_FINITE;
    if (fabs(p - lp) > SHAPED_POSITION_REL_TOLERANCE * fmax(1., fabs(p)))
        return SS_POSITION_MISMATCH;
    *pos = p;
    *vel = v;
    *acc = a;
    return SS_OK;
}

// Fill pos/vel/acc with the stepper's commanded motion at
// start_time + i * interval for i in [0, count)
int32_t __visible
servo_sampler_generate(struct servo_sampler *ss, double start_time
                       , double interval, int count, double *pos
                       , double *vel, double *acc
                       , struct servo_sample_error *err)
{
    struct stepper_kinematics *sk = ss->sk;
    if (!sk || !sk->tq)
        return SS_NO_KINEMATICS;
    trapq_check_sentinels(sk->tq);
    struct move *head = list_first_entry(&sk->tq->moves, struct move, node);
    struct move *first = list_next_entry(head, node);
    struct move *m = first;
    double finalized_end = finalized_motion_end(ss);
    int i;
    for (i = 0; i < count; ++i) {
        double t = start_time + i * interval;
        while (t >= m->print_time + m->move_t)
            m = list_next_entry(m, node);
        int ret = sample_one(ss, first, m, finalized_end, t, &pos[i]
                             , &vel[i], &acc[i], err);
        if (ret)
            return ret;
    }
    if (count)
        sk->commanded_pos = pos[count - 1];
    return SS_OK;
}

// First time whose sample would not reach back into finalized motion (the
// remnant of a move cut short, eg by a homing trip); -INFINITY if there is
// none
double __visible
servo_sampler_earliest_sample(struct servo_sampler *ss)
{
    struct stepper_kinematics *sk = ss->sk;
    trapq_check_sentinels(sk->tq);
    return finalized_motion_end(ss) + sk->gen_steps_pre_active;
}
