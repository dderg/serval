// Extruder stepper pulse time generation
//
// Copyright (C) 2018-2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <math.h> // tanh
#include <stddef.h> // offsetof
#include <stdlib.h> // malloc
#include <string.h> // memset
#include "compiler.h" // __visible
#include "itersolve.h" // struct stepper_kinematics
#include "integrate.h" // struct smoother
#include "pyhelper.h" // errorf
#include "trapq.h" // move_get_distance

// Plan 8 Chunk 2 Task 13: kin_shaper.c retired along with the post-hoc
// step-generator shaper cascade. The previous extruder shaper-pulse
// path (init_shaper / shaper_calc_position / shaper_pa_range_integrate)
// becomes dead on this fork — XY shaping is baked directly into the
// planner polynomial by blendplanner._bake_shaper_polynomial, so the
// extruder reads an already-shaped trajectory via move_get_coord and
// smoothing + PA through the integrate.h kernel alone.

// Without pressure advance, the extruder stepper position is:
//     extruder_position(t) = nominal_position(t)
// When pressure advance is enabled, additional filament is pushed
// into the extruder during acceleration (and retracted during
// deceleration). The formula is:
//     pa_position(t) = (nominal_position(t)
//                       + pressure_advance * nominal_velocity(t))
// The nominal position and velocity are then smoothed using a weighted average:
//     smooth_position(t) = (
//         definitive_integral(nominal_position(x+t_offs) * smoother(t-x) * dx,
//                             from=t-smooth_time/2, to=t+smooth_time/2)
//     smooth_velocity(t) = (
//         definitive_integral(nominal_velocity(x+t_offs) * smoother(t-x) * dx,
//                             from=t-smooth_time/2, to=t+smooth_time/2)
// and the final pressure advance value calculated as
//     smooth_pa_position(t) = smooth_position(t) + pa_func(smooth_velocity(t))
// where pa_func(v) = pressure_advance * v for linear velocity model or a more
// complicated function for non-linear pressure advance models.

// Calculate the definitive integral of extruder for a given move. PA fires
// whenever the XY path is moving: any X or Y polynomial coefficient c[1..10]
// beyond the constant term is non-zero in any phase.
static inline void
pa_move_integrate(const struct move *m, int axis
                  , double t0, const smoother_antiderivatives *ad
                  , double *pa_velocity_integral)
{
    int can_pressure_advance = 0;
    int n = m->n_phases;
    for (int p = 0; p < n && !can_pressure_advance; ++p) {
        const struct move_quintic_phase *ph = &m->phases[p];
        for (int k = 1; k < MOVE_QUINTIC_POLY_COEFFS; ++k) {
            if (ph->c[k].x > 0.0 || ph->c[k].y > 0.0) {
                can_pressure_advance = 1;
                break;
            }
        }
    }

    // Calculate definitive integral
    if (can_pressure_advance)
        *pa_velocity_integral += integrate_velocity(m, axis, t0, ad);
}

// Calculate the definitive integral of the extruder over a range of moves
static void
pa_range_integrate(const struct move *m, int axis, double move_time
                   , const struct smoother *sm
                   , double *pa_velocity_integral)
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
    *pa_velocity_integral = 0.;
    if (unlikely(start >= 0. && end <= m->move_t)) {
        pa_move_integrate(m, axis, t0, &sm->pm_diff,
                          pa_velocity_integral);
        return;
    }
    smoother_antiderivatives left =
        likely(start < 0.) ? calc_antiderivatives(sm, t0) : sm->p_hst;
    smoother_antiderivatives right =
        likely(end > m->move_t) ? calc_antiderivatives(sm, t0 - m->move_t)
                                : sm->m_hst;
    smoother_antiderivatives diff = diff_antiderivatives(&right, &left);
    pa_move_integrate(m, axis, t0, &diff,
                      pa_velocity_integral);
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
        pa_move_integrate(prev, axis, t0, &diff,
                          pa_velocity_integral);
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
        pa_move_integrate(m, axis, t0, &diff,
                          pa_velocity_integral);
    }
}

struct pressure_advance_params {
    union {
        struct {
            double pressure_advance;
        };
        struct {
            double linear_advance, nonlinear_offset, linearization_velocity;
        };
        double params[3];
    };
};

typedef double (*pressure_advance_func)(
        double, double, struct pressure_advance_params *pa_params);

struct extruder_stepper {
    struct stepper_kinematics sk;
    struct smoother sm[3];
    struct pressure_advance_params pa_params;
    pressure_advance_func pa_func;
    double time_offset;
};

double __visible
pressure_advance_linear_model_func(double position, double pa_velocity
                                   , struct pressure_advance_params *pa_params)
{
    return position + pa_velocity * pa_params->pressure_advance;
}

double __visible
pressure_advance_tanh_model_func(double position, double pa_velocity
                                 , struct pressure_advance_params *pa_params)
{
    position += pa_params->linear_advance * pa_velocity;
    if (pa_params->nonlinear_offset) {
        double rel_velocity = pa_velocity / pa_params->linearization_velocity;
        position += pa_params->nonlinear_offset * tanh(rel_velocity);
    }
    return position;
}

double __visible
pressure_advance_recipr_model_func(double position, double pa_velocity
                                   , struct pressure_advance_params *pa_params)
{
    position += pa_params->linear_advance * pa_velocity;
    if (pa_params->nonlinear_offset) {
        double rel_velocity = pa_velocity / pa_params->linearization_velocity;
        position += pa_params->nonlinear_offset * (1. - 1. / (1. + rel_velocity));
    }
    return position;
}

static double
extruder_calc_position(struct stepper_kinematics *sk, struct move *m
                       , double move_time)
{
    struct extruder_stepper *es = container_of(sk, struct extruder_stepper, sk);
    move_time += es->time_offset;
    while (unlikely(move_time < 0.)) {
        m = list_prev_entry(m, node);
        move_time += m->move_t;
    }
    while (unlikely(move_time >= m->move_t)) {
        move_time -= m->move_t;
        m = list_next_entry(m, node);
    }
    int i;
    struct coord e_pos, pa_vel;
    // Post-hoc shaping retired (Plan 8 Chunk 2 Task 13). XY content is
    // already baked into the planner polynomial, so e_pos == base_pos
    // on every axis and PA integration reads the shaped trajectory
    // through pa_range_integrate alone.
    struct coord base_pos = move_get_coord(m, move_time);
    for (i = 0; i < 3; ++i) {
        int axis = 'x' + i;
        const struct smoother* sm = &es->sm[i];
        e_pos.axis[i] = base_pos.axis[i];
        if (!sm->hst) {
            pa_vel.axis[i] = 0.;
        } else {
            pa_range_integrate(m, axis, move_time, sm,
                               &pa_vel.axis[i]);
        }
    }
    double position = e_pos.x + e_pos.y + e_pos.z;
    double pa_velocity = pa_vel.x + pa_vel.y + pa_vel.z;
    if (pa_velocity > 0.)
        return es->pa_func(position, pa_velocity, &es->pa_params);
    return position;
}

static void
extruder_note_generation_time(struct extruder_stepper *es)
{
    double pre_active = 0., post_active = 0.;
    int i;
    for (i = 0; i < 3; ++i) {
        const struct smoother* sm = &es->sm[i];
        double pre_active_axis = sm->hst + sm->t_offs + es->time_offset;
        double post_active_axis = sm->hst - sm->t_offs - es->time_offset;
        if (pre_active_axis > pre_active)
            pre_active = pre_active_axis;
        if (post_active_axis > post_active)
            post_active = post_active_axis;
    }
    es->sk.gen_steps_pre_active = pre_active;
    es->sk.gen_steps_post_active = post_active;
}

void __visible
extruder_set_pressure_advance(struct stepper_kinematics *sk
                              , int n_params, double params[]
                              , double time_offset)
{
    struct extruder_stepper *es = container_of(sk, struct extruder_stepper, sk);
    es->time_offset = time_offset;
    memset(&es->pa_params, 0, sizeof(es->pa_params));
    extruder_note_generation_time(es);
    if (n_params < 0 || n_params > ARRAY_SIZE(es->pa_params.params))
        return;
    memcpy(&es->pa_params, params, n_params * sizeof(params[0]));
}

void __visible
extruder_set_pressure_advance_model_func(struct stepper_kinematics *sk
                                         , pressure_advance_func func)
{
    struct extruder_stepper *es = container_of(sk, struct extruder_stepper, sk);
    memset(&es->pa_params, 0, sizeof(es->pa_params));
    es->pa_func = func;
}

double __visible
extruder_get_step_gen_window(struct stepper_kinematics *sk)
{
    struct extruder_stepper *es = container_of(sk, struct extruder_stepper, sk);
    return es->sk.gen_steps_pre_active > es->sk.gen_steps_post_active
         ? es->sk.gen_steps_pre_active : es->sk.gen_steps_post_active;
}

struct stepper_kinematics * __visible
extruder_stepper_alloc(void)
{
    struct extruder_stepper *es = malloc(sizeof(*es));
    memset(es, 0, sizeof(*es));
    es->sk.calc_position_cb = extruder_calc_position;
    es->pa_func = pressure_advance_tanh_model_func;
    es->sk.active_flags = AF_X | AF_Y | AF_Z;
    return &es->sk;
}
