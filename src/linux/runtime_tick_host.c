#include "generic/runtime_tick.h"

#include <dlfcn.h>
#include <errno.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/syscall.h>
#include <time.h>
#include <unistd.h>

#include "autoconf.h"
#include "runtime.h"
#include "sched.h"
#include "step_queue.h"

extern void *runtime_handle;

// On H7 this is the IWDG flag; the Linux build has no IWDG, so default ok=1.
volatile uint8_t runtime_liveness_ok = 1;

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
#define HOST_TICK_HZ ((unsigned long)CONFIG_MOTION_SAMPLE_RATE_HZ)
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

extern uint32_t timer_read_time(void);
extern uint64_t timer_read_time_u64(void);

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

#if CONFIG_MCU_SIM
static const int step_gpio_lines[N_AXIS_STEP_QUEUES] = { 18, 7, 15, 20 };

static void (*sim_notify_step)(int chip, int line, int32_t n_steps,
                               uint64_t sched_vtime_ns, uint64_t sched_cycle);

// libvtime pacer API: while registered, virtual time cannot advance past
// this thread's floor, so no motion sample period is ever skipped.
static int (*vtime_pacer_register)(uint64_t period_ns);
static void (*vtime_pacer_advance)(int slot, uint64_t target_ns,
                                   uint64_t period_ns);
static int vtime_pacer_slot = -1;
#endif

#define TICK_TRACE_DEPTH 16
static uint32_t tick_trace_times[TICK_TRACE_DEPTH];
static uint32_t tick_trace_seq;

__attribute__((used)) void
runtime_tick_trace_dump(void)
{
    fprintf(stderr, "[tick-trace] seq=%u recent reads (oldest first):\n",
            tick_trace_seq);
    for (unsigned i = 0; i < TICK_TRACE_DEPTH; i++) {
        unsigned idx = (tick_trace_seq + i) % TICK_TRACE_DEPTH;
        fprintf(stderr, "[tick-trace]   %u\n", tick_trace_times[idx]);
    }
}

static void *
host_tick_main(void *arg)
{
    (void)arg;

#if CONFIG_MCU_SIM
    // This thread paces virtual time (its pacer floor bounds the vtime
    // driver), so it must not be demoted: every starved tick is a span
    // where the whole simulated world stands still against real time.
    sim_notify_step = dlsym(RTLD_DEFAULT, "sim_intercept_notify_step");
    vtime_pacer_register = dlsym(RTLD_DEFAULT, "vtime_pacer_register");
    vtime_pacer_advance = dlsym(RTLD_DEFAULT, "vtime_pacer_advance");
    if (vtime_pacer_register && vtime_pacer_advance)
        vtime_pacer_slot = vtime_pacer_register(HOST_TICK_NS);
#else
    // Real Linux MCU: this tick is the motion ISR — inherit the process
    // scheduler (SCHED_FIFO with -r) rather than self-demoting, or it can't
    // hold cadence under load and trips TickIntervalExceeded.
#endif

    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);

#if CONFIG_MCU_SIM
    // Raw syscall: dlsym(RTLD_NEXT) resolves to the libvtime preload (it is
    // ahead of libc), which silently made "real_ms" a copy of virtual time.
    struct timespec real_prev, vt_prev;
    syscall(SYS_clock_gettime, CLOCK_MONOTONIC, &real_prev);
    clock_gettime(CLOCK_MONOTONIC, &vt_prev);
    uint64_t ticks_done = 0, ticks_prev = 0;
    uint64_t max_tick_gap_real_ns = 0;
    struct timespec real_last_tick = real_prev;
#endif

    while (1) {
        next.tv_nsec += HOST_TICK_NS;
        while (next.tv_nsec >= 1000000000L) {
            next.tv_nsec -= 1000000000L;
            next.tv_sec  += 1;
        }
#if CONFIG_MCU_SIM
        if (vtime_pacer_slot >= 0) {
            uint64_t target = (uint64_t)next.tv_sec * 1000000000ULL
                              + (uint64_t)next.tv_nsec;
            vtime_pacer_advance(vtime_pacer_slot, target, HOST_TICK_NS);
        } else {
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
        }
#else
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
#endif

#if CONFIG_MCU_SIM
        ticks_done++;
        struct timespec real_now_ts;
        syscall(SYS_clock_gettime, CLOCK_MONOTONIC, &real_now_ts);
        uint64_t gap = (uint64_t)(real_now_ts.tv_sec - real_last_tick.tv_sec)
                           * 1000000000ULL
                       + (uint64_t)(real_now_ts.tv_nsec
                                    - real_last_tick.tv_nsec);
        real_last_tick = real_now_ts;
        if (gap > max_tick_gap_real_ns)
            max_tick_gap_real_ns = gap;
        if (ticks_done - ticks_prev >= HOST_TICK_HZ) {
            struct timespec vt_now_ts;
            clock_gettime(CLOCK_MONOTONIC, &vt_now_ts);
            uint64_t real_el =
                (uint64_t)(real_now_ts.tv_sec - real_prev.tv_sec) * 1000000000ULL
                + (uint64_t)(real_now_ts.tv_nsec - real_prev.tv_nsec);
            uint64_t vt_el =
                (uint64_t)(vt_now_ts.tv_sec - vt_prev.tv_sec) * 1000000000ULL
                + (uint64_t)(vt_now_ts.tv_nsec - vt_prev.tv_nsec);
            fprintf(stderr,
                    "[tick-rate] ticks=%llu vt_ms=%llu real_ms=%llu"
                    " max_gap_real_us=%llu\n",
                    (unsigned long long)(ticks_done - ticks_prev),
                    (unsigned long long)(vt_el / 1000000),
                    (unsigned long long)(real_el / 1000000),
                    (unsigned long long)(max_tick_gap_real_ns / 1000));
            ticks_prev = ticks_done;
            real_prev = real_now_ts;
            vt_prev = vt_now_ts;
            max_tick_gap_real_ns = 0;
        }
#endif
        if (!atomic_load_explicit(&host_tick_enabled, memory_order_acquire))
            continue;
        if (!runtime_handle)
            continue;

        tick_trace_times[tick_trace_seq % TICK_TRACE_DEPTH] =
            runtime_cyccnt_read();
        tick_trace_seq++;
        runtime_tick_sample(runtime_handle);

        // Must drain every tick or the queue overflows (StepQueueOverflow).
        // The shim wants each step's SCHEDULED sub-sample time, not this
        // drain tick: the sim outputs a whole sample burst at once, so the
        // dispatch lag lives only in each entry's cycle_abs. Reconstruct the
        // absolute virtual time from cycle_abs against the current 64-bit
        // clock and this tick's virtual clock.
#if CONFIG_MCU_SIM
        struct timespec sim_vt;
        clock_gettime(CLOCK_MONOTONIC, &sim_vt);
        uint64_t sim_vt_ns = (uint64_t)sim_vt.tv_sec * 1000000000ULL
                             + (uint64_t)sim_vt.tv_nsec;
        uint64_t sim_now_cyc = timer_read_time_u64();
        const int64_t sim_ns_per_tick = 1000000000LL / CONFIG_CLOCK_FREQ;
#endif
        for (int axis = 0; axis < N_AXIS_STEP_QUEUES; axis++) {
            StepQueue *q = &step_queues[axis];
            while (q->head != q->tail) {
#if CONFIG_MCU_SIM
                uint16_t idx = q->head & (STEP_QUEUE_DEPTH - 1);
                int8_t signed_step_delta = q->buf[idx].dir;
                uint32_t sched_lo = q->buf[idx].cycle_abs;
                int32_t sched_delta = (int32_t)(sched_lo - (uint32_t)sim_now_cyc);
                uint64_t sched_vtime_ns =
                    sim_vt_ns + (uint64_t)((int64_t)sched_delta * sim_ns_per_tick);
#endif
                q->head++;
#if CONFIG_MCU_SIM
                if (sim_notify_step && step_gpio_lines[axis] >= 0) {
                    uint64_t sched_cycle =
                        (uint64_t)((int64_t)sim_now_cyc + sched_delta);
                    sim_notify_step(0, step_gpio_lines[axis],
                                    signed_step_delta, sched_vtime_ns,
                                    sched_cycle);
                }
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
    int rc = pthread_create(&host_tick_thread, &attr, host_tick_main, NULL);
    pthread_attr_destroy(&attr);
    if (rc != 0) {
        fprintf(stderr, "kalico_host_tick: pthread_create failed: %d\n", rc);
        atomic_store(&host_tick_thread_started, 0);
    }
}

extern uint32_t stats_send_time_high;
extern uint32_t stats_send_time;

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
        runtime_install_step_queues(runtime_handle,
                                           (uint8_t *)step_queues);
    }
    atomic_store_explicit(&host_tick_enabled, 1, memory_order_release);
}

__attribute__((used)) void
runtime_tick_disable(void)
{
    atomic_store_explicit(&host_tick_enabled, 0, memory_order_release);
}

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
