// Where the CPU went, per window: asleep, in the timer interrupt, in its
// non-step callbacks, and how many steps it dispatched in each mode; the
// host derives the step and non-step shares.
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
    uint32_t start, idle, timer_irq, callback, pulsed_edges, both_edge_steps;
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
    uint32_t pulsed_edges = sched_pulsed_step_edges();
    uint32_t both_edge_steps = sched_both_edge_steps();
    sendf("cpu_load window_ticks=%u idle_ticks=%u timer_irq_ticks=%u"
          " callback_ticks=%u pulsed_step_edges=%u both_edge_steps=%u",
          now - window.start, idle - window.idle,
          timer_irq - window.timer_irq, callback - window.callback,
          pulsed_edges - window.pulsed_edges,
          both_edge_steps - window.both_edge_steps);
    window.start = now;
    window.idle = idle;
    window.timer_irq = timer_irq;
    window.callback = callback;
    window.pulsed_edges = pulsed_edges;
    window.both_edge_steps = both_edge_steps;
}
DECL_TASK(cpu_load_task);
