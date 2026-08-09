// SPSC step queue per motor axis. Producer = TIM5 ISR (Rust); consumer =
// per-axis Klipper timer (Rust, via SysTick dispatch). Storage C-owned per the
// B2/B3 invariant in docs/rewrite/mcu-c-rust-boundary.md. The struct
// layout mirrors Rust #[repr(C)] — keep in sync (static_asserts below).

#ifndef __STEP_QUEUE_H
#define __STEP_QUEUE_H

#include <stdint.h>
#include <stddef.h>
#include "autoconf.h" // CONFIG_MOTION_SAMPLE_RATE_HZ

#ifndef CONFIG_MOTION_SAMPLE_RATE_HZ
#error "step_queue.h requires CONFIG_MOTION_SAMPLE_RATE_HZ (runtime motion targets only)"
#endif

#define N_AXIS_STEP_QUEUES     4

// Per-axis step throughput target: the 500 kHz single-edge ceiling set by
// STEP_MIN_EDGE_DWT's 1 us edge spacing. The sizing below mirrors the Rust
// derivation in rust/runtime/build.rs — keep in sync.
#define RUNTIME_TARGET_STEP_RATE_HZ 500000

// ceil(target / sample_rate) clamped to [16, 256]: the per-sample burst
// capacity, mirroring runtime::sub_sample_timing::MAX_STEPS_PER_SAMPLE;
// runtime_set_axis_step_budget rejects anything larger.
#define RUNTIME_STEPS_PER_SAMPLE_RAW \
    ((RUNTIME_TARGET_STEP_RATE_HZ + CONFIG_MOTION_SAMPLE_RATE_HZ - 1) \
     / CONFIG_MOTION_SAMPLE_RATE_HZ)
#define RUNTIME_MAX_STEPS_PER_SAMPLE \
    (RUNTIME_STEPS_PER_SAMPLE_RAW < 16 ? 16 \
     : RUNTIME_STEPS_PER_SAMPLE_RAW > 256 ? 256 \
     : RUNTIME_STEPS_PER_SAMPLE_RAW)

// Two worst-case producer bursts, rounded up to a power of two.
#define STEP_QUEUE_DEPTH_MIN (2 * RUNTIME_MAX_STEPS_PER_SAMPLE)
#define STEP_QUEUE_DEPTH \
    (STEP_QUEUE_DEPTH_MIN <= 32 ? 32 \
     : STEP_QUEUE_DEPTH_MIN <= 64 ? 64 \
     : STEP_QUEUE_DEPTH_MIN <= 128 ? 128 \
     : STEP_QUEUE_DEPTH_MIN <= 256 ? 256 : 512)
#define STEP_QUEUE_DEPTH_MASK  (STEP_QUEUE_DEPTH - 1)

typedef struct {
    uint32_t cycle_abs;   // low 32 bits of DWT CYCCNT; wrap-aware compare only
    int8_t   dir;
    uint8_t  stepper_sel; // 0 = all steppers of the motor; n = only stepper n-1
    uint8_t  _pad[2];
} StepEntry;

typedef struct {
    volatile uint16_t tail;
    volatile uint16_t head;
    uint8_t  _pad[4];
    StepEntry buf[STEP_QUEUE_DEPTH];
} StepQueue;

extern StepQueue step_queues[N_AXIS_STEP_QUEUES];

_Static_assert(sizeof(StepEntry) == 8, "StepEntry layout drift");
_Static_assert(sizeof(StepQueue) == 8 + 8 * (size_t)STEP_QUEUE_DEPTH,
               "StepQueue layout drift");
_Static_assert(STEP_QUEUE_DEPTH >= 2 * RUNTIME_MAX_STEPS_PER_SAMPLE,
               "queue must absorb two producer sample bursts");
_Static_assert(offsetof(StepQueue, buf) == 8, "StepQueue.buf offset drift");
_Static_assert((STEP_QUEUE_DEPTH & STEP_QUEUE_DEPTH_MASK) == 0,
               "STEP_QUEUE_DEPTH must be power of 2");

#endif // __STEP_QUEUE_H
