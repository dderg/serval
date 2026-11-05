// Self-benchmark of the classic step dispatch path: the timer IRQ time one
// isolated motor's step costs, with the instruction cache evicted between
// steps as the main loop's tasks evict it.
//
// Copyright (C) 2026  Serval contributors
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <string.h> // memset
#include "autoconf.h" // CONFIG_MCU_SIM
#include "basecmd.h" // foreach_oid
#include "board/gpio.h" // gpio_out_bench_sink
#include "board/irq.h" // irq_poll
#include "board/misc.h" // timer_read_time
#include "command.h" // DECL_COMMAND
#include "compiler.h" // __stringify
#include "sched.h" // sched_add_timer
#include "stepper.h" // stepper_setup_dispatch

#define BENCH_STEPS 32
#define BENCH_REPEATS 3
#define BENCH_STEP_INTERVAL_US 25
#define BENCH_PULSE_US 2
#define BENCH_START_DELAY_US 100
#if CONFIG_MACH_STM32H7
#define BENCH_ICACHE_EVICT_NOPS 16384
#else
#define BENCH_ICACHE_EVICT_NOPS 1024
#endif

static struct stepper bench_stepper;

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

static noinline void
bench_evict_instruction_cache(void)
{
    asm volatile(".rept " __stringify(BENCH_ICACHE_EVICT_NOPS) "\n nop\n .endr"
                 ::: "memory");
}

static void
bench_arm(uint_fast8_t both_edge, uint32_t first_step)
{
    struct stepper *s = &bench_stepper;
    memset(s, 0, sizeof(*s));
    s->step_pin = gpio_out_bench_sink();
    s->dir_pin = gpio_out_bench_sink();
    stepper_setup_dispatch(s, both_edge,
                           both_edge ? 0 : timer_from_us(BENCH_PULSE_US));
    s->interval = timer_from_us(BENCH_STEP_INTERVAL_US);
    s->count = s->flags & SF_SINGLE_SCHED ? BENCH_STEPS : 2 * BENCH_STEPS;
    s->next_step_time = s->time.waketime = first_step;
}

static uint32_t
bench_ticks_per_step(uint_fast8_t both_edge)
{
    bench_arm(both_edge,
              timer_read_time() + timer_from_us(BENCH_START_DELAY_US));
    uint32_t busy_before = timer_dispatch_busy_ticks();
    sched_add_timer(&bench_stepper.time);
    while (bench_stepper.count) {
        bench_evict_instruction_cache();
        irq_poll();
    }
    return (timer_dispatch_busy_ticks() - busy_before) / BENCH_STEPS;
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
