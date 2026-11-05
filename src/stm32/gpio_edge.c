// Timed pin edges through the EXTI controller
//
// Copyright (C) 2026  Serval contributors
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "autoconf.h" // CONFIG_MACH_STM32G0
#include "board/irq.h" // irq_save
#include "board/misc.h" // timer_read_time
#include "command.h" // shutdown
#include "generic/armcm_boot.h" // armcm_enable_irq
#include "generic/gpio_edge.h" // gpio_edge_claim
#include "internal.h" // GPIO2PORT
#include "sched.h" // DECL_SHUTDOWN

#define EXTI_LINES 16
#define EXTI_NVIC_PRIO 1

#if CONFIG_MACH_STM32G0
#define EXTI_RISING (EXTI->RTSR1)
#define EXTI_FALLING (EXTI->FTSR1)
#define EXTI_MASK (EXTI->IMR1)
#define EXTI_ROUTE_BITS 8
#define EXTI_ROUTE (EXTI->EXTICR)
#elif CONFIG_MACH_STM32H7
#define EXTI_RISING (EXTI->RTSR1)
#define EXTI_FALLING (EXTI->FTSR1)
#define EXTI_MASK (EXTI_D1->IMR1)
#define EXTI_PENDING (EXTI_D1->PR1)
#define EXTI_ROUTE_BITS 4
#define EXTI_ROUTE (SYSCFG->EXTICR)
#define EXTI_ROUTE_CLOCK SYSCFG_BASE
#elif CONFIG_MACH_STM32F1
#define EXTI_RISING (EXTI->RTSR)
#define EXTI_FALLING (EXTI->FTSR)
#define EXTI_MASK (EXTI->IMR)
#define EXTI_PENDING (EXTI->PR)
#define EXTI_ROUTE_BITS 4
#define EXTI_ROUTE (AFIO->EXTICR)
#define EXTI_ROUTE_CLOCK AFIO_BASE
#else
#define EXTI_RISING (EXTI->RTSR)
#define EXTI_FALLING (EXTI->FTSR)
#define EXTI_MASK (EXTI->IMR)
#define EXTI_PENDING (EXTI->PR)
#define EXTI_ROUTE_BITS 4
#define EXTI_ROUTE (SYSCFG->EXTICR)
#define EXTI_ROUTE_CLOCK SYSCFG_BASE
#endif

#define EXTI_LINES_PER_ROUTE (32 / EXTI_ROUTE_BITS)

static struct gpio_edge *exti_edges[EXTI_LINES];
static uint32_t exti_claimed;

static inline uint32_t
exti_pending(void)
{
#if CONFIG_MACH_STM32G0
    return EXTI->RPR1 | EXTI->FPR1;
#else
    return EXTI_PENDING;
#endif
}

static inline void
exti_clear(uint32_t lines)
{
#if CONFIG_MACH_STM32G0
    EXTI->RPR1 = lines;
    EXTI->FPR1 = lines;
#else
    EXTI_PENDING = lines;
#endif
}

static void
exti_dispatch(uint32_t entry_time, uint32_t lines)
{
    uint32_t pending = exti_pending() & lines;
    exti_clear(pending);
    while (pending) {
        uint32_t line = __builtin_ctz(pending);
        pending &= pending - 1;
        struct gpio_edge *edge = exti_edges[line];
        if (!edge)
            shutdown("EXTI line fired without a claimed edge");
        edge->on_edge_irq(edge, entry_time);
    }
}

#if CONFIG_MACH_STM32G0
void
EXTI0_1_IRQHandler(void)
{
    exti_dispatch(timer_read_time(), 0x0003);
}

void
EXTI2_3_IRQHandler(void)
{
    exti_dispatch(timer_read_time(), 0x000c);
}

void
EXTI4_15_IRQHandler(void)
{
    exti_dispatch(timer_read_time(), 0xfff0);
}

static void
exti_enable_vector(uint32_t line)
{
    if (line < 2)
        armcm_enable_irq(EXTI0_1_IRQHandler, EXTI0_1_IRQn, EXTI_NVIC_PRIO);
    else if (line < 4)
        armcm_enable_irq(EXTI2_3_IRQHandler, EXTI2_3_IRQn, EXTI_NVIC_PRIO);
    else
        armcm_enable_irq(EXTI4_15_IRQHandler, EXTI4_15_IRQn, EXTI_NVIC_PRIO);
}
#else
void
EXTI0_IRQHandler(void)
{
    exti_dispatch(timer_read_time(), 0x0001);
}

void
EXTI1_IRQHandler(void)
{
    exti_dispatch(timer_read_time(), 0x0002);
}

void
EXTI2_IRQHandler(void)
{
    exti_dispatch(timer_read_time(), 0x0004);
}

void
EXTI3_IRQHandler(void)
{
    exti_dispatch(timer_read_time(), 0x0008);
}

void
EXTI4_IRQHandler(void)
{
    exti_dispatch(timer_read_time(), 0x0010);
}

void
EXTI9_5_IRQHandler(void)
{
    exti_dispatch(timer_read_time(), 0x03e0);
}

void
EXTI15_10_IRQHandler(void)
{
    exti_dispatch(timer_read_time(), 0xfc00);
}

static void
exti_enable_vector(uint32_t line)
{
    switch (line) {
    case 0:
        armcm_enable_irq(EXTI0_IRQHandler, EXTI0_IRQn, EXTI_NVIC_PRIO);
        break;
    case 1:
        armcm_enable_irq(EXTI1_IRQHandler, EXTI1_IRQn, EXTI_NVIC_PRIO);
        break;
    case 2:
        armcm_enable_irq(EXTI2_IRQHandler, EXTI2_IRQn, EXTI_NVIC_PRIO);
        break;
    case 3:
        armcm_enable_irq(EXTI3_IRQHandler, EXTI3_IRQn, EXTI_NVIC_PRIO);
        break;
    case 4:
        armcm_enable_irq(EXTI4_IRQHandler, EXTI4_IRQn, EXTI_NVIC_PRIO);
        break;
    case 5 ... 9:
        armcm_enable_irq(EXTI9_5_IRQHandler, EXTI9_5_IRQn, EXTI_NVIC_PRIO);
        break;
    default:
        armcm_enable_irq(EXTI15_10_IRQHandler, EXTI15_10_IRQn
                         , EXTI_NVIC_PRIO);
        break;
    }
}
#endif

static void
exti_route(uint32_t line, uint32_t port)
{
#ifdef EXTI_ROUTE_CLOCK
    if (!is_enabled_pclock(EXTI_ROUTE_CLOCK))
        enable_pclock(EXTI_ROUTE_CLOCK);
#endif
    uint32_t shift = (line % EXTI_LINES_PER_ROUTE) * EXTI_ROUTE_BITS;
    uint32_t field = ((1u << EXTI_ROUTE_BITS) - 1) << shift;
    volatile uint32_t *route = &EXTI_ROUTE[line / EXTI_LINES_PER_ROUTE];
    *route = (*route & ~field) | (port << shift);
}

uint_fast8_t
gpio_edge_claim(struct gpio_edge *edge, uint32_t pin)
{
    uint32_t line = pin % EXTI_LINES, bit = 1 << line;
    if (exti_claimed & bit)
        return 0;
    irqstatus_t flag = irq_save();
    exti_claimed |= bit;
    exti_edges[line] = edge;
    edge->line = line;
    EXTI_MASK &= ~bit;
    exti_route(line, GPIO2PORT(pin));
    irq_restore(flag);
    exti_enable_vector(line);
    return 1;
}

void
gpio_edge_arm(struct gpio_edge *edge, uint_fast8_t rising)
{
    uint32_t bit = 1 << edge->line;
    irqstatus_t flag = irq_save();
    if (rising) {
        EXTI_FALLING &= ~bit;
        EXTI_RISING |= bit;
    } else {
        EXTI_RISING &= ~bit;
        EXTI_FALLING |= bit;
    }
    exti_clear(bit);
    EXTI_MASK |= bit;
    irq_restore(flag);
}

void
gpio_edge_disarm(struct gpio_edge *edge)
{
    uint32_t bit = 1 << edge->line;
    irqstatus_t flag = irq_save();
    EXTI_MASK &= ~bit;
    exti_clear(bit);
    irq_restore(flag);
}

uint_fast8_t
gpio_edge_pending(struct gpio_edge *edge)
{
    return !!(exti_pending() & EXTI_MASK & (1 << edge->line));
}

void
gpio_edge_shutdown(void)
{
    EXTI_MASK &= ~exti_claimed;
    exti_clear(exti_claimed);
}
DECL_SHUTDOWN(gpio_edge_shutdown);
