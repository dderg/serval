// Host-process tick driver: a pthread calls kalico_runtime_tick_sample at the
// motion sample rate, mirroring the H7 TIM5_IRQHandler. MACH_LINUX only.

#include "generic/runtime_tick.h"

#include <dlfcn.h>
#include <errno.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>

#include "autoconf.h" // CONFIG_CLOCK_FREQ
#include "kalico_runtime.h"
#include "sched.h"
#include "step_queue.h" // StepQueue, step_queues[], N_AXIS_STEP_QUEUES

extern void *runtime_handle;
extern void runtime_endstop_sample_pins(void); // src/runtime_tick.c

// On H7 this is the IWDG flag; the Linux build has no IWDG, so default ok=1.
volatile uint8_t runtime_liveness_ok = 1;

// Maps the H7's CONFIG_KALICO_SIM cycle counter onto the host-derived one.
volatile uint32_t runtime_sim_cyccnt = 0;

// No TIM5 exception frame on the host; Rust -311 externs resolve to 0.
__attribute__((used, externally_visible))
uint32_t runtime_tim5_stacked_pc(void) { return 0; }
__attribute__((used, externally_visible))
uint32_t runtime_tim5_stacked_exc(void) { return 0; }

// MUST equal the rate the engine derives sample_period from: a mismatch makes
// the per-tick inter-arrival gap differ from sample_period and trips the
// runtime's TickIntervalExceeded guard on the first active tick. Stock Pi
// kernels floor at ~1 kHz (clock_nanosleep); higher rates need PREEMPT_RT +
// SCHED_FIFO (-r).
#define HOST_TICK_HZ ((unsigned long)CONFIG_KALICO_MOTION_SAMPLE_RATE_HZ)
#define HOST_TICK_NS (1000000000UL / HOST_TICK_HZ)

static atomic_int host_tick_enabled = 0;
static atomic_int host_tick_thread_started = 0;
static pthread_t host_tick_thread;
static struct timespec host_tick_t0;

static uint64_t
host_monotonic_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    uint64_t s  = (uint64_t)(ts.tv_sec  - host_tick_t0.tv_sec);
    int64_t  ns = (int64_t)ts.tv_nsec - host_tick_t0.tv_nsec;
    return s * 1000000000ULL + (uint64_t)(ns + (ns < 0 ? 1000000000 : 0));
}

extern uint32_t timer_read_time(void); // src/linux/timer.c
extern uint64_t timer_read_time_u64(void); // src/linux/timer.c

__attribute__((used)) uint32_t
runtime_cyccnt_read(void)
{
    // Klipper's own clock, so the engine's `now` shares the reference frame of
    // the set_clock_est values klippy sends the bridge. A different t0 here
    // would put t_start and `now` in different frames.
    return timer_read_time();
}

__attribute__((used)) uint64_t
runtime_host_widened_clock_now(void)
{
    return timer_read_time_u64();
}

#if CONFIG_KALICO_SIM
// Matches printer_real/config after pin-overrides.toml: X(motor0)=PG4→gpio18,
// Y(motor1)=PF11→gpio7, Z(motor2)=PG0→gpio15.
static const int step_gpio_lines[N_AXIS_STEP_QUEUES] = { 18, 7, 15, -1 };

static void (*sim_notify_step)(int chip, int line, int32_t n_steps);
#endif

// One sample period expressed in MCU clock cycles.
// CONFIG_CLOCK_FREQ = 50 MHz for MACH_LINUX; HOST_TICK_HZ = 1000 Hz.
// Result: 50 000 cycles, matching sample_period_cycles in the Rust engine.
#define HOST_TICK_CYCLES ((uint32_t)(CONFIG_CLOCK_FREQ / HOST_TICK_HZ))

static void *
host_tick_main(void *arg)
{
    (void)arg;

#if CONFIG_KALICO_SIM
    sim_notify_step = dlsym(RTLD_DEFAULT, "sim_intercept_notify_step");
#endif

    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);

    // Last MCU clock value seen at tick-thread wake. Zero = not yet set.
    uint32_t last_tick_clk = 0;

    while (1) {
        next.tv_nsec += HOST_TICK_NS;
        while (next.tv_nsec >= 1000000000L) {
            next.tv_nsec -= 1000000000L;
            next.tv_sec  += 1;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);

        if (!atomic_load_explicit(&host_tick_enabled, memory_order_acquire))
            continue;
        if (!runtime_handle)
            continue;

        // On MACH_LINUX, the MCU main loop runs on a separate CPU core and
        // can advance virtual time by N*sample_period in one ppoll-timer step
        // while this thread sleeps between ticks. When vtime jumps, the tick
        // thread fires all skipped ticks at the same virtual `now`, and the
        // runtime's inter-arrival gap guard (last_tick_now diff) sees a gap
        // of N*sample_period instead of 1, tripping TickIntervalExceeded.
        //
        // Detect the jump here (actual MCU clock advanced by > 2 periods
        // since the last tick) and reset the gap baseline before the engine
        // sees the first tick after the jump.
        uint32_t now_clk = timer_read_time();
        if (last_tick_clk != 0) {
            uint32_t gap_clk = now_clk - last_tick_clk;
            if (gap_clk > 2 * HOST_TICK_CYCLES)
                kalico_runtime_clear_last_tick_now(runtime_handle);
        }
        last_tick_clk = now_clk;

#if !CONFIG_KALICO_SIM
        runtime_endstop_sample_pins();
#endif

        (void)runtime_cyccnt_read();
        kalico_runtime_tick_sample(runtime_handle);

        // Must drain every tick or the queue overflows (StepQueueOverflow).
        // Sim notifies the auto-endstop shim to count pulses; raw STEP/DIR GPIO
        // output on a real Linux MCU is not wired here (SPI phase-stepping is).
        for (int axis = 0; axis < N_AXIS_STEP_QUEUES; axis++) {
            StepQueue *q = &step_queues[axis];
            while (q->head != q->tail) {
#if CONFIG_KALICO_SIM
                uint16_t idx = q->head & (STEP_QUEUE_DEPTH - 1);
                int8_t dir = q->buf[idx].dir;
#endif
                q->head++;
#if CONFIG_KALICO_SIM
                if (sim_notify_step && step_gpio_lines[axis] >= 0)
                    sim_notify_step(0, step_gpio_lines[axis],
                                    dir ? -1 : 1);
#endif
            }
        }
    }
    return NULL;
}

extern void *runtime_handle;

__attribute__((used)) void
runtime_tick_init(void)
{
    if (atomic_exchange(&host_tick_thread_started, 1))
        return;
    clock_gettime(CLOCK_MONOTONIC, &host_tick_t0);

    pthread_attr_t attr;
    pthread_attr_init(&attr);

    // Request SCHED_FIFO for the tick thread to mirror TIM5 ISR preemption
    // semantics on MACH_LINUX. Without it, the CFS scheduler may let the MCU
    // main loop advance virtual time by >1 sample period before the tick
    // thread wakes, triggering TickIntervalExceeded (gap > 2*sample_period).
    //
    // Requires CAP_SYS_NICE (--privileged in Docker or cap_sys_nice on bare
    // Linux). Failure is non-fatal: CFS ticks are noisier but still correct
    // on lightly loaded hosts. The error is printed so sim runs expose it.
    struct sched_param sp;
    sp.sched_priority = (sched_get_priority_max(SCHED_FIFO)
                         + sched_get_priority_min(SCHED_FIFO)) / 2;
    if (pthread_attr_setschedpolicy(&attr, SCHED_FIFO) == 0
        && pthread_attr_setschedparam(&attr, &sp) == 0)
        pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    else
        fprintf(stderr, "kalico_host_tick: SCHED_FIFO attr setup failed"
                " — tick jitter possible\n");

    int rc = pthread_create(&host_tick_thread, &attr, host_tick_main, NULL);
    if (rc == EPERM)
        // SCHED_FIFO requires CAP_SYS_NICE; retry at CFS with no priority
        // attributes so the tick still starts on unprivileged hosts.
        rc = pthread_create(&host_tick_thread, NULL, host_tick_main, NULL);
    pthread_attr_destroy(&attr);
    if (rc != 0) {
        fprintf(stderr, "kalico_host_tick: pthread_create failed: %d\n", rc);
        // Reset so a later runtime_tick_init can retry.
        atomic_store(&host_tick_thread_started, 0);
    }
}

extern uint32_t stats_send_time_high; // src/basecmd.c
extern uint32_t stats_send_time;      // src/basecmd.c (exposed 2026-05-11)

__attribute__((used)) void
runtime_tick_enable(void)
{
    // Seed widen state with command_get_uptime's exact arithmetic. Replicating
    // it is mandatory: otherwise the engine's WidenState lags klippy's
    // last_clock by 2^32 and the first segment's t_start is unreachable from
    // the engine's `now` (curve evaluated at u=0 → zero step pulses).
    if (runtime_handle) {
        uint32_t low = timer_read_time();
        uint32_t high = stats_send_time_high + (low < stats_send_time);
        uint64_t baseline = ((uint64_t)high) << 32 | (uint64_t)low;
        runtime_handle_seed_widen(runtime_handle, baseline);
        // On the MCU the engine resolves step_queues via a C extern; on the
        // host it must be installed explicitly.
        kalico_runtime_install_step_queues(runtime_handle,
                                           (uint8_t *)step_queues);
    }
    atomic_store_explicit(&host_tick_enabled, 1, memory_order_release);
}

__attribute__((used)) void
runtime_tick_disable(void)
{
    atomic_store_explicit(&host_tick_enabled, 0, memory_order_release);
}

// The pthread loop drains step_queues inline, so the kick is a no-op here.
// Stubs satisfy the extern references; `used` keeps them through --gc-sections.
static uint32_t host_step_out_target;

__attribute__((used)) void
step_output_timer_arm(uint32_t cycle_abs)
{
    host_step_out_target = cycle_abs;
}

__attribute__((used)) uint32_t
step_output_timer_armed_target(void)
{
    return host_step_out_target;
}

__attribute__((used)) uint8_t
step_output_timer_is_running(void)
{
    return 0;
}
