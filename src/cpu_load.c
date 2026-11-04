// Where the CPU went, per window: asleep, in the timer interrupt, in the
// part of it non-step timers took, and how many step timer events it
// dispatched; the host derives the step and non-step shares.
//
// Copyright (C) 2026  Serval contributors
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "board/irq.h" // irq_wait_idle_ticks
#include "board/misc.h" // timer_read_time
#include "command.h" // sendf
#include "sched.h" // DECL_TASK

#define CPU_LOAD_WINDOW_US 250000

static struct {
    uint32_t start, idle, timer_irq, nonstep_timer, step_events;
} window;

void
cpu_load_task(void)
{
    uint32_t now = timer_read_time();
    if (timer_is_before(now, window.start + timer_from_us(CPU_LOAD_WINDOW_US)))
        return;
    uint32_t idle = irq_wait_idle_ticks();
    uint32_t timer_irq = timer_dispatch_busy_ticks();
    uint32_t nonstep_timer = sched_nonstep_timer_ticks();
    uint32_t step_events = sched_step_events();
    sendf("cpu_load window_ticks=%u idle_ticks=%u timer_irq_ticks=%u"
          " nonstep_timer_ticks=%u step_events=%u",
          now - window.start, idle - window.idle,
          timer_irq - window.timer_irq, nonstep_timer - window.nonstep_timer,
          step_events - window.step_events);
    window.start = now;
    window.idle = idle;
    window.timer_irq = timer_irq;
    window.nonstep_timer = nonstep_timer;
    window.step_events = step_events;
}
DECL_TASK(cpu_load_task);
