// Definitions for irq enable/disable on ARM Cortex-M processors
//
// Copyright (C) 2017-2018  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "board/internal.h" // __CORTEX_M
#include "board/misc.h" // timer_read_time
#include "irq.h" // irqstatus_t
#include "sched.h" // DECL_SHUTDOWN

void
irq_disable(void)
{
    asm volatile("cpsid i" ::: "memory");
}

void
irq_enable(void)
{
    asm volatile("cpsie i" ::: "memory");
}

irqstatus_t
irq_save(void)
{
    irqstatus_t flag;
    asm volatile("mrs %0, primask" : "=r" (flag) :: "memory");
    irq_disable();
    return flag;
}

void
irq_restore(irqstatus_t flag)
{
    asm volatile("msr primask, %0" :: "r" (flag) : "memory");
}

static uint32_t idle_ticks;

static inline void
irq_sleep_until_pending(void)
{
    if (__CORTEX_M == 7)
        while (!(SCB->ICSR & (SCB_ICSR_ISRPENDING_Msk
                              | SCB_ICSR_PENDSTSET_Msk)))
            ;
    else
        asm volatile("wfi" ::: "memory");
}

void
irq_wait(void)
{
    uint32_t slept = timer_read_time();
    irq_sleep_until_pending();
    idle_ticks += timer_read_time() - slept;
    asm volatile("cpsie i\n    cpsid i\n" ::: "memory");
}

uint32_t
irq_wait_idle_ticks(void)
{
    return idle_ticks;
}

void
irq_poll(void)
{
}

// Clear the active irq if a shutdown happened in an irq handler
void
clear_active_irq(void)
{
    uint32_t psr;
    asm volatile("mrs %0, psr" : "=r" (psr));
    if (!(psr & 0x1ff))
        // Shutdown did not occur in an irq - nothing to do.
        return;
    // Clear active irq status
    psr = 1<<24; // T-bit
    uint32_t temp;
    asm volatile(
        "  push { %1 }\n"
        "  adr %0, 1f\n"
        "  push { %0 }\n"
        "  push { r0, r1, r2, r3, r4, lr }\n"
        "  bx %2\n"
        ".balign 4\n"
        "1:\n"
        : "=&r"(temp) : "r"(psr), "r"(0xfffffff9) : "r12", "cc");
}
DECL_SHUTDOWN(clear_active_irq);
