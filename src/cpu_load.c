// Where the CPU went, per window: asleep, in the timer interrupt, and in
// its non-step callbacks; the host derives the step and non-step shares.
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
    uint32_t start, idle, timer_irq, callback;
} window;

void
cpu_load_task(void)
{
    uint32_t now = timer_read_time();
    if (timer_is_before(now, window.start + timer_from_us(CPU_LOAD_WINDOW_US)))
        return;
    uint32_t idle = irq_wait_idle_ticks();
    uint32_t timer_irq = timer_dispatch_busy_ticks();
    uint32_t callback = sched_callback_busy_ticks();
    sendf("cpu_load window_ticks=%u idle_ticks=%u timer_irq_ticks=%u"
          " callback_ticks=%u",
          now - window.start, idle - window.idle,
          timer_irq - window.timer_irq, callback - window.callback);
    window.start = now;
    window.idle = idle;
    window.timer_irq = timer_irq;
    window.callback = callback;
}
DECL_TASK(cpu_load_task);
