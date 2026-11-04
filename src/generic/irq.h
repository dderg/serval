#ifndef __GENERIC_IRQ_H
#define __GENERIC_IRQ_H

#include <stdint.h> // uint32_t

typedef unsigned long irqstatus_t;

void irq_disable(void);
void irq_enable(void);
irqstatus_t irq_save(void);
void irq_restore(irqstatus_t flag);
void irq_wait(void);
uint32_t irq_wait_idle_ticks(void);
void irq_poll(void);

#endif // irq.h
