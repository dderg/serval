#include "generic/runtime_tick.h"

#include <stdint.h>
#include <stdio.h>

#include "autoconf.h"
#include "board/misc.h"
#include "command.h"
#include "runtime.h"
#include "sched.h"

extern void *runtime_handle;
extern uint64_t timer_read_time_u64(void);
extern uint32_t stats_send_time_high;
extern uint32_t stats_send_time;

volatile uint8_t runtime_liveness_ok = 1;

__attribute__((used, externally_visible))
uint32_t runtime_tim5_stacked_pc(void) { return 0; }
__attribute__((used, externally_visible))
uint32_t runtime_tim5_stacked_exc(void) { return 0; }

#define HOST_TICK_CYCLES (CONFIG_CLOCK_FREQ / CONFIG_MOTION_SAMPLE_RATE_HZ)
#define TICK_TRACE_DEPTH 16

static struct timer host_tick_timer;
static uint8_t host_tick_enabled;
static uint32_t tick_trace_times[TICK_TRACE_DEPTH];
static uint32_t tick_trace_seq;

__attribute__((used)) uint32_t
runtime_cyccnt_read(void)
{
    return timer_read_time();
}

__attribute__((used)) uint64_t
runtime_host_widened_clock_now(void)
{
    return timer_read_time_u64();
}

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

static uint_fast8_t
host_tick_event(struct timer *timer)
{
    tick_trace_times[tick_trace_seq++ % TICK_TRACE_DEPTH] = timer_read_time();
    runtime_tick_sample(runtime_handle);
    timer->waketime += HOST_TICK_CYCLES;
    return SF_RESCHEDULE;
}

__attribute__((used)) void
runtime_tick_init(void)
{
    host_tick_timer.func = host_tick_event;
}

__attribute__((used)) void
runtime_tick_enable(void)
{
    if (host_tick_enabled)
        return;
    if (!runtime_handle)
        shutdown("runtime tick without runtime");
    uint32_t low = timer_read_time();
    uint32_t high = stats_send_time_high + (low < stats_send_time);
    uint64_t baseline = ((uint64_t)high << 32) | low;
    runtime_handle_seed_widen(runtime_handle, baseline);
    host_tick_timer.waketime = low + HOST_TICK_CYCLES;
    host_tick_enabled = 1;
    sched_add_timer(&host_tick_timer);
}

__attribute__((used)) void
runtime_tick_disable(void)
{
    if (!host_tick_enabled)
        return;
    sched_del_timer(&host_tick_timer);
    host_tick_enabled = 0;
}
