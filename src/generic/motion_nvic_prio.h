#ifndef __MOTION_NVIC_PRIO_H
#define __MOTION_NVIC_PRIO_H

// LOAD-BEARING INVARIANT: TIM5 (producer) and the step-output timer (consumer)
// MUST stay EQUAL to each other (both MOTION_NVIC_PRIO). PRIGROUP=0 means
// equal-priority interrupts cannot nest, which is what keeps the step_queue SPSC
// and the kalico_kick_step_output compare-register poke non-racing (the producer
// and consumer can never interleave). If a future change ever splits their
// priorities, the volatile SPSC must become true-atomic first. Both timers read
// this one constant, so they move together by construction.
#define MOTION_NVIC_PRIO 2

#define SCHED_NVIC_PRIO 2

#endif
