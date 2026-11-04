// Self-benchmark of the classic step dispatch path.
//
// Copyright (C) 2026  Serval contributors
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <string.h> // memset
#include "autoconf.h" // CONFIG_MCU_SIM
#include "basecmd.h" // foreach_oid
#include "board/gpio.h" // gpio_out_bench_sink
#include "board/irq.h" // irq_save
#include "board/misc.h" // timer_read_time
#include "command.h" // DECL_COMMAND
#include "sched.h" // sched_timer_dispatch
#include "stepper.h" // stepper_setup_dispatch

#define BENCH_MOTORS 4
#define BENCH_STEPS_PER_MOTOR 32
#define BENCH_REPEATS 3
#define BENCH_STEP_INTERVAL_US 10
#define BENCH_PULSE_US 2
#define BENCH_START_DELAY_US 100

static struct stepper bench_steppers[BENCH_MOTORS];

static uint_fast8_t
bench_owns(struct timer *t)
{
    for (int i = 0; i < BENCH_MOTORS; i++)
        if (t == &bench_steppers[i].time)
            return 1;
    return 0;
}

static uint_fast8_t
bench_active(void)
{
    for (int i = 0; i < BENCH_MOTORS; i++)
        if (bench_steppers[i].count)
            return 1;
    return 0;
}

static void
require_idle_steppers(void)
{
    uint8_t oid;
    struct stepper *s;
    foreach_oid(oid, s, command_config_stepper) {
        if (s->count)
            shutdown("Step benchmark requested while a stepper is moving");
    }
}

static void
bench_arm(struct stepper *s, uint_fast8_t both_edge, uint32_t first_step)
{
    memset(s, 0, sizeof(*s));
    s->step_pin = gpio_out_bench_sink();
    s->dir_pin = gpio_out_bench_sink();
    stepper_setup_dispatch(s, both_edge,
                           both_edge ? 0 : timer_from_us(BENCH_PULSE_US));
    s->interval = timer_from_us(BENCH_STEP_INTERVAL_US);
    s->count = s->flags & SF_SINGLE_SCHED ? BENCH_STEPS_PER_MOTOR
                                          : 2 * BENCH_STEPS_PER_MOTOR;
    s->next_step_time = s->time.waketime = first_step;
}

static uint32_t
bench_ticks_per_step(uint_fast8_t both_edge)
{
    uint32_t interval = timer_from_us(BENCH_STEP_INTERVAL_US);
    uint32_t near = timer_from_us(1) / 4;
    uint32_t phases[BENCH_MOTORS] = {
        0, near, interval / 2, interval / 2 + near
    };
    irqstatus_t flag = irq_save();
    uint32_t start = timer_read_time() + timer_from_us(BENCH_START_DELAY_US);
    for (int i = 0; i < BENCH_MOTORS; i++) {
        bench_arm(&bench_steppers[i], both_edge, start + phases[i]);
        sched_add_timer(&bench_steppers[i].time);
    }
    sched_writable_begin();
    uint32_t busy = 0;
    while (bench_active()) {
        struct timer *head = sched_peek_timer();
        if (timer_is_before(timer_read_time(), head->waketime))
            continue;
        uint_fast8_t measured = bench_owns(head);
        uint32_t before = timer_read_time();
        sched_timer_dispatch();
        if (measured)
            busy += timer_read_time() - before;
    }
    sched_writable_end();
    timer_kick();
    irq_restore(flag);
    return busy / (BENCH_MOTORS * BENCH_STEPS_PER_MOTOR);
}

static uint32_t
bench_worst_ticks_per_step(uint_fast8_t both_edge)
{
    uint32_t worst = 0;
    for (int i = 0; i < BENCH_REPEATS; i++) {
        uint32_t ticks = bench_ticks_per_step(both_edge);
        if (ticks > worst)
            worst = ticks;
    }
    return worst;
}

noinline void
stepper_bench_measure(uint32_t *pulsed, uint32_t *both_edge)
{
    *pulsed = bench_worst_ticks_per_step(0);
    *both_edge = bench_worst_ticks_per_step(1);
}

static struct {
    uint32_t pulsed, both_edge;
} bench_result;

void
command_get_step_benchmark(uint32_t *args)
{
    if (!CONFIG_MCU_SIM && !bench_result.pulsed) {
        require_idle_steppers();
        stepper_bench_measure(&bench_result.pulsed, &bench_result.both_edge);
    }
    sendf("step_benchmark pulsed_ticks_per_step=%u"
          " both_edge_ticks_per_step=%u simulated=%c",
          bench_result.pulsed, bench_result.both_edge, CONFIG_MCU_SIM);
}
DECL_COMMAND(command_get_step_benchmark, "get_step_benchmark");
