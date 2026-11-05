// Generic interrupt based timer helper functions
//
// Copyright (C) 2017  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "autoconf.h" // CONFIG_CLOCK_FREQ
#include "board/irq.h" // irq_disable
#include "board/misc.h" // timer_from_us
#include "board/timer_irq.h" // timer_dispatch_many
#include "command.h" // shutdown
#include "sched.h" // sched_timer_dispatch

DECL_CONSTANT("CLOCK_FREQ", CONFIG_CLOCK_FREQ);

uint32_t
timer_from_us(uint32_t us)
{
    return us * (CONFIG_CLOCK_FREQ / 1000000);
}

// Return true if time1 is before time2.  Always use this function to
// compare times as regular C comparisons can fail if the counter
// rolls over.
//
// used, externally_visible: under -fwhole-program LTO the C callers inline
// this one-liner, leaving no standalone symbol; the kalico Rust staticlib
// calls it via the C ABI and needs the out-of-line copy. Mirrors
// src/generic/armcm_timer.c's timer_is_before (the SysTick backend).
__attribute__((used, externally_visible))
uint8_t
timer_is_before(uint32_t time1, uint32_t time2)
{
    return (int32_t)(time1 - time2) < 0;
}

static uint32_t timer_repeat_until;
#define TIMER_REPEAT_TICKS timer_from_us(100)

#if CONFIG_MACH_STM32G0 && !CONFIG_MCU_SIM
#define TIMER_IRQ_LEAD_TICKS 64
#else
#define TIMER_IRQ_LEAD_TICKS 0
#endif
#define TIMER_COMPARE_MARGIN_TICKS timer_from_us(2)
#define TIMER_MIN_TRY_TICKS (TIMER_IRQ_LEAD_TICKS + TIMER_COMPARE_MARGIN_TICKS)
#define TIMER_DEFER_REPEAT_TICKS timer_from_us(5)

static inline void
timer_spin_until(uint32_t due)
{
    while (timer_is_before(timer_read_time(), due))
        ;
}

uint32_t
timer_dispatch_many(void)
{
    uint32_t tru = timer_repeat_until;
    for (;;) {
        uint32_t due = sched_peek_timer()->waketime;
        int32_t ahead = due - timer_read_time();
        if (ahead > (int32_t)TIMER_MIN_TRY_TICKS)
            return due - TIMER_IRQ_LEAD_TICKS;
        timer_spin_until(due);
        uint32_t next = sched_timer_dispatch();

        uint32_t now = timer_read_time();
        int32_t diff = next - now;
        if (diff > (int32_t)TIMER_MIN_TRY_TICKS)
            return next - TIMER_IRQ_LEAD_TICKS;

        if (unlikely(timer_is_before(tru, now))) {
            if (diff < (int32_t)(-timer_from_us(1000)))
                try_shutdown("Rescheduled timer in the past");
            if (sched_check_set_tasks_busy()) {
                timer_repeat_until = now + TIMER_REPEAT_TICKS;
                return now + TIMER_DEFER_REPEAT_TICKS;
            }
            timer_repeat_until = tru = now + TIMER_REPEAT_TICKS;
        }

        irq_enable();
        while (unlikely(diff > (int32_t)TIMER_IRQ_LEAD_TICKS))
            diff = next - timer_read_time();
        irq_disable();
    }
}

// Make sure timer_repeat_until doesn't wrap 32bit comparisons
void
timer_task(void)
{
    uint32_t now = timer_read_time();
    irq_disable();
    if (timer_is_before(timer_repeat_until, now))
        timer_repeat_until = now;
    irq_enable();
}
DECL_TASK(timer_task);
