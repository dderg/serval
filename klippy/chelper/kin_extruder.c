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
#include "pyhelper.h" // errorf
#include "trapq.h" // move_get_coord

// Plan 8 Chunk 3 Task 8 — kin_extruder.c reduced to a thin step-gen
// wrapper. Pressure advance and input shaping are now both baked into
// the planner-emitted polynomial at plan time (see
// klippy/blendplanner.py and klippy/chelper/{linear,nonlinear}_pa_compose.c).
// The extruder stepper reads its axis-e polynomial directly via
// move_get_coord(m, t).e; no convolution, no smoothing kernels, no
// per-axis shaper arrays.
//
// The PA-model function pointers below are retained only because
// kinematics/extruder.py references them by symbol for legacy code
// paths (SET_PRESSURE_ADVANCE plumbing + regressions that check the
// snapshot). The step-generator itself no longer calls them.

struct extruder_stepper {
    struct stepper_kinematics sk;
    double time_offset;
};

// Legacy pressure-advance "model func" prototypes — kept as no-op
// symbols so Python FFI lookups on extruder.py's get_func() don't
// fail. They were the convolution-time f(v) functions; Plan 8 Chunk 3
// composes the entire PA response into the planner polynomial, so
// these are never actually called during step generation. A caller
// that invokes them manually will get a plausible scalar answer —
// same math as before.

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
    /* Plan 8 Chunk 3 Task 8: the move's .e polynomial carries the
     * PA-baked extruder position. Return it directly. */
    struct coord c = move_get_coord(m, move_time);
    return c.e;
}

void __visible
extruder_set_pressure_advance(struct stepper_kinematics *sk
                              , int n_params, double params[]
                              , double time_offset)
{
    struct extruder_stepper *es = container_of(sk, struct extruder_stepper, sk);
    es->time_offset = time_offset;
    /* Plan 8 Chunk 3: PA parameters are applied at planner emit time
     * via {linear,nonlinear}_pa_compose. The per-stepper copy used to
     * matter for the step-gen convolution; no longer. We still accept
     * the call (keeps the SET_PRESSURE_ADVANCE FFI wiring alive) and
     * carry time_offset through so the step-gen clock offset still
     * works for operator fine-tune. */
    (void)n_params;
    (void)params;
}

void __visible
extruder_set_pressure_advance_model_func(struct stepper_kinematics *sk
                                         , void *func)
{
    /* Plan 8 Chunk 3: no per-stepper model func — dispatch lives in
     * the planner emit site. Accept and ignore for FFI compatibility. */
    (void)sk;
    (void)func;
}

double __visible
extruder_get_step_gen_window(struct stepper_kinematics *sk)
{
    /* No smoothing / shaping at step-gen time anymore — window is 0. */
    (void)sk;
    return 0.0;
}

struct stepper_kinematics * __visible
extruder_stepper_alloc(void)
{
    struct extruder_stepper *es = malloc(sizeof(*es));
    memset(es, 0, sizeof(*es));
    es->sk.calc_position_cb = extruder_calc_position;
    es->sk.active_flags = AF_X | AF_Y | AF_Z;
    return &es->sk;
}
