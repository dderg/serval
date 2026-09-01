#include <string.h>
#if defined(__linux__) || defined(__APPLE__)
#include <stdio.h>
#include <time.h>
#endif
#include "autoconf.h"
#include "board/gpio.h"
#include "board/internal.h"
#include "board/irq.h"
#include "board/misc.h"
#include "command.h"
#include "sched.h"
#include "runtime.h"
#include "mcu_transport_dispatch.h"
#include "event_log.h"
#include "generic/runtime_tick.h"
#include "generic/fault_handler.h"

#if CONFIG_MOTION_RUNTIME
// Read from Rust via `extern "C" { static runtime_clock_freq: u32; }`;
// used,externally_visible keeps it through -fwhole-program LTO.
const uint32_t runtime_clock_freq __attribute__((used, externally_visible))
    = CONFIG_CLOCK_FREQ;

const uint32_t runtime_sample_rate_hz __attribute__((used, externally_visible))
    = CONFIG_MOTION_SAMPLE_RATE_HZ;
#endif


#if CONFIG_MOTION_RUNTIME
extern volatile uint8_t runtime_liveness_ok;  // defined in src/stm32/watchdog.c

#define ENGINE_STATUS_RUNNING 1
#define ENGINE_STATUS_FAULT   3

// Foreground-only; NEVER call from ISR.
__attribute__((used, externally_visible))
uint64_t
runtime_host_now_us(void)
{
    uint32_t cycles = timer_read_time();
    return ((uint64_t)cycles) / (CONFIG_CLOCK_FREQ / 1000000U);
}
#endif

// (tag, stage, value) packed: bits[31:24]=tag, [23:16]=stage, [15:0]=value.
volatile uint32_t runtime_diag_last_packed __attribute__((used, externally_visible));

// Survives NVIC_SystemReset via .persistent_diag (NOLOAD, outside bss); next
// boot checks magic == RT_DIAG_MAGIC.
#define RT_DIAG_MAGIC 0xD1A6BABE

struct rt_diag_persistent {
    uint32_t magic;
    uint32_t last_packed;
    uint32_t last_us;
    uint32_t fault_count;
};

volatile struct rt_diag_persistent rt_diag_persistent
    __attribute__((section(".persistent_diag"), used, externally_visible));

volatile uint32_t runtime_diag_prior_boot_snapshot
    __attribute__((used, externally_visible));

volatile uint32_t runtime_diag_prior_magic_raw
    __attribute__((used, externally_visible));
volatile uint32_t runtime_diag_prior_packed_raw
    __attribute__((used, externally_visible));

__attribute__((used, externally_visible))
void
runtime_diag_progress(uint32_t tag, uint32_t stage, uint32_t value)
{
    uint32_t packed = ((tag & 0xFFu) << 24)
                    | ((stage & 0xFFu) << 16)
                    | (value & 0xFFFFu);
    runtime_diag_last_packed = packed;
    rt_diag_persistent.magic = RT_DIAG_MAGIC;
    rt_diag_persistent.last_packed = packed;
    rt_diag_persistent.last_us = timer_read_time();
}

// Advances regardless of engine state, unlike the ISR-published widened_now,
// so this is the only clock a classic-stepping build can widen. ISR-safe: the
// epoch pair is sampled before the low word and under the same irq guard
// stats_update publishes it with, so no caller can splice a pre-wrap low word
// onto a post-wrap epoch.
__attribute__((used, externally_visible))
uint64_t
runtime_widened_host_clock(void)
{
    extern uint32_t stats_send_time;
    extern uint32_t stats_send_time_high;
    irqstatus_t flag = irq_save();
    uint32_t last = stats_send_time;
    uint32_t high = stats_send_time_high;
    uint32_t cur = timer_read_time();
    irq_restore(flag);
    return ((uint64_t)(high + (cur < last)) << 32) | (uint64_t)cur;
}

#if CONFIG_MOTION_RUNTIME
// used,externally_visible: the Rust staticlib calls these; LTO would otherwise
// DCE the standalone symbols.
__attribute__((used, externally_visible))
uint32_t
runtime_irq_save(void)
{
    return (uint32_t)irq_save();
}

__attribute__((used, externally_visible))
void
runtime_irq_restore(uint32_t flags)
{
    irq_restore((irqstatus_t)flags);
}
#endif

#if CONFIG_MOTION_RUNTIME

void* runtime_handle = 0;            // exposed (non-static) for runtime_tick_h7.c
static struct task_wake runtime_drain_wake;
static struct timer runtime_drain_timer;

static uint32_t last_status_emit_time = 0;
static uint8_t prev_engine_status = 0;

static uint32_t last_seen_tick_counter = 0;
static uint32_t last_progress_time = 0;

static uint8_t last_seen_status = 255;

// Reschedule from now (not +=1ms) to avoid a "timer in past" shutdown if the
// foreground stalls for >1 ms.
static uint_fast8_t
runtime_drain_event(struct timer *t)
{
    sched_wake_task(&runtime_drain_wake);
    t->waketime = timer_read_time() + timer_from_us(1000);
    return SF_RESCHEDULE;
}

void
runtime_init(void)
{
    extern volatile uint32_t runtime_diag_prior_magic_raw;
    extern volatile uint32_t runtime_diag_prior_packed_raw;
    runtime_diag_prior_magic_raw = rt_diag_persistent.magic;
    runtime_diag_prior_packed_raw = rt_diag_persistent.last_packed;
    if (rt_diag_persistent.magic == RT_DIAG_MAGIC
        && rt_diag_persistent.last_packed != 0) {
        runtime_diag_prior_boot_snapshot = rt_diag_persistent.last_packed;
    }

    runtime_diag_progress(0xB0, 0, 0);

#define RUNTIME_INIT_STUB 0  /* DIAG: 1 stubs runtime_init for crash bisect */
#if RUNTIME_INIT_STUB
    runtime_diag_progress(0xBF, 0, 0xCAFE);
    return;
#endif

    runtime_diag_progress(0xB1, 0, 0);
    runtime_handle = runtime_handle_create();
    if (!runtime_handle) {
        runtime_diag_progress(0xB1, 1, 0xFFFF);
        return;
    }
    runtime_diag_progress(0xB2, 0, 0);
    last_seen_tick_counter = runtime_handle_tick_counter(runtime_handle);
    last_progress_time = timer_read_time();
    last_seen_status = runtime_handle_status(runtime_handle);
    runtime_diag_progress(0xB3, 0, 0);

    runtime_diag_progress(0xB4, 0, 0);
    runtime_tick_init();
    runtime_diag_progress(0xB5, 0, 0);

    runtime_drain_timer.func = runtime_drain_event;
    runtime_drain_timer.waketime = timer_read_time() + timer_from_us(1000);
    sched_add_timer(&runtime_drain_timer);

    last_status_emit_time = timer_read_time();
}
DECL_INIT(runtime_init);

#define LIVENESS_THRESHOLD_MS 25
#define LIVENESS_THRESHOLD_TICKS  \
    ((LIVENESS_THRESHOLD_MS) * (CONFIG_CLOCK_FREQ / 1000))

#define FAST_STATUS_MAX_AXES 8
// A phase lane's sample-run ring holds single-digit runs, and the host may not
// send past what this heartbeat has retired. The credit therefore has to reach
// the host in a fraction of the ring's playback time, not the 10 ms a status
// display would be happy with.
#define FAST_STATUS_RETIREMENT_MIN_TICKS \
    ((uint32_t)((CONFIG_CLOCK_FREQ) / 500))

#define AXIS_STALL_DETECT_TICKS ((uint32_t)(CONFIG_CLOCK_FREQ) * 2u)
#define AXIS_STALL_REPORT_PERIOD_TICKS ((uint32_t)(CONFIG_CLOCK_FREQ) * 5u)

static int32_t
saturate_ticks_to_ms(int64_t ticks)
{
    int64_t ms = ticks / (int64_t)(CONFIG_CLOCK_FREQ / 1000);
    if (ms > INT32_MAX) return INT32_MAX;
    if (ms < INT32_MIN) return INT32_MIN;
    return (int32_t)ms;
}

// Observability only: an axis with runs pending whose retired counter has been
// frozen for AXIS_STALL_DETECT_TICKS gets its front sample-run window reported
// against the current clock. A far-future start/end here is the silent-hold
// signature; a long dwell also reports and is benign.
static void
report_stalled_axes(int32_t nr, const uint32_t *retired_change_time,
                    uint32_t *stall_report_time, uint32_t cur_time)
{
    for (int32_t i = 0; i < nr; i++) {
        if ((cur_time - retired_change_time[i]) < AXIS_STALL_DETECT_TICKS)
            continue;
        if ((cur_time - stall_report_time[i]) < AXIS_STALL_REPORT_PERIOD_TICKS)
            continue;
        uint64_t start = 0, end = 0;
        uint32_t occupancy = 0;
        irqstatus_t flag = irq_save();
        int32_t armed = runtime_axis_head_window(runtime_handle, (uint32_t)i,
                                                 &start, &end, &occupancy);
        uint64_t now64 = runtime_now_ticks(runtime_handle);
        irq_restore(flag);
        if (armed <= 0 && occupancy == 0)
            continue;
        stall_report_time[i] = cur_time;
        uint32_t stalled_ms = (uint32_t)((uint64_t)(cur_time
                                                    - retired_change_time[i])
                                         / (CONFIG_CLOCK_FREQ / 1000));
        event_log_emit(EVENT_LOG_LEVEL_WARN, EVENT_LOG_SUBSYS_MOTION,
                       EVENT_LOG_EVENT_MOTION_AXIS_STALLED, 0,
                       (((uint32_t)i) << 16) | (occupancy & 0xFFFFu),
                       stalled_ms);
        if (armed == 1)
            event_log_emit(EVENT_LOG_LEVEL_WARN, EVENT_LOG_SUBSYS_MOTION,
                           EVENT_LOG_EVENT_MOTION_AXIS_STALLED_HEAD, 0,
                           (uint32_t)saturate_ticks_to_ms(
                               (int64_t)(start - now64)),
                           (uint32_t)saturate_ticks_to_ms(
                               (int64_t)(end - now64)));
    }
}

void
runtime_drain(void)
{
    if (!runtime_handle) return;
    if (!sched_check_wake(&runtime_drain_wake)) return;

    diag_task_heartbeat(diag_slot_rt_drain_calls(),
                        diag_slot_rt_drain_last_tick(),
                        diag_slot_rt_drain_max_gap(),
                        timer_from_us(50000),
                        0); // no event tag — idle gaps are normal

    // Liveness acts only on RUNNING; other states refresh the anchor so a
    // transition INTO RUNNING doesn't trip on a stale anchor.
    uint32_t cur_counter = runtime_handle_tick_counter(runtime_handle);
    uint32_t cur_time = timer_read_time();
    uint8_t cur_status = runtime_handle_status(runtime_handle);
    if (cur_status == ENGINE_STATUS_RUNNING) {
        if (cur_counter != last_seen_tick_counter) {
            last_seen_tick_counter = cur_counter;
            last_progress_time = cur_time;
        } else if ((cur_time - last_progress_time) > LIVENESS_THRESHOLD_TICKS) {
            runtime_liveness_ok = 0;
        }
    } else {
        last_progress_time = cur_time;
        last_seen_tick_counter = cur_counter;
    }

    if (cur_status == ENGINE_STATUS_FAULT) {
        runtime_liveness_ok = 0;
        if (prev_engine_status != ENGINE_STATUS_FAULT) {
            int32_t fault_code = runtime_handle_last_error(runtime_handle);
            uint32_t fault_detail = runtime_handle_fault_detail(runtime_handle);
            uint32_t tick_blocker_pc = runtime_handle_tick_blocker_pc(runtime_handle);
            mcu_transport_emit_fault_event((uint16_t)fault_code, fault_detail,
                                           tick_blocker_pc);
        }
    }

    // shutdown() is safe in foreground (DECL_TASK) but NOT from ISR.
    static int32_t last_acted_error;
    int32_t cur_error = runtime_handle_last_error(runtime_handle);
    if (cur_error != 0 && cur_error != last_acted_error) {
        last_acted_error = cur_error;
        uint32_t fdetail = runtime_handle_fault_detail(runtime_handle);
        uint32_t tick_blocker_pc = runtime_handle_tick_blocker_pc(runtime_handle);
        mcu_transport_emit_fault_event((uint16_t)cur_error, fdetail,
                                       tick_blocker_pc);
        // Persist before shutdown resets the USB stack.
        diag_ring_push(DIAG_EV_RUST_FAULT, (uint32_t)cur_error, fdetail);
        runtime_liveness_ok = 0;
#ifdef __linux__
        {
            extern void runtime_tick_trace_dump(void);
            runtime_tick_trace_dump();
        }
#endif
        shutdown("kalico runtime fault");
    }

    if (cur_status != prev_engine_status) {
        diag_record_engine_xition(prev_engine_status, cur_status, cur_counter);
    }
    prev_engine_status = cur_status;
    if (cur_status != last_seen_status) {
        last_seen_status = cur_status;
    }

    {
        static uint32_t last_retired_seen[FAST_STATUS_MAX_AXES];
        static uint32_t retired_change_time[FAST_STATUS_MAX_AXES];
        static uint32_t stall_report_time[FAST_STATUS_MAX_AXES];
        uint32_t retired[FAST_STATUS_MAX_AXES];
        uint64_t playback[FAST_STATUS_MAX_AXES];
        uint8_t st = 0;
        uint16_t fc = 0;
        int32_t nr = runtime_get_heartbeat(runtime_handle, &st, &fc,
                                                  retired, playback,
                                                  FAST_STATUS_MAX_AXES);
        static uint8_t pending_advance;
        if (nr > 0) {
            for (int32_t i = 0; i < nr; i++) {
                if (retired[i] != last_retired_seen[i]) {
                    pending_advance = 1;
                    last_retired_seen[i] = retired[i];
                    retired_change_time[i] = cur_time;
                }
            }
            uint32_t elapsed = cur_time - last_status_emit_time;
            if (pending_advance
                && elapsed >= FAST_STATUS_RETIREMENT_MIN_TICKS) {
                send_status_heartbeat();
                last_status_emit_time = cur_time;
                pending_advance = 0;
            }
            report_stalled_axes(nr, retired_change_time, stall_report_time,
                                cur_time);
        }
    }

    event_log_drain();
}
DECL_TASK(runtime_drain);


void
runtime_tick_shutdown(void)
{
    runtime_tick_disable();
}
DECL_SHUTDOWN(runtime_tick_shutdown);

void
runtime_status_drain(void)
{
    if (!runtime_handle) return;
    uint32_t now = timer_read_time();
    const uint32_t status_period_ticks = CONFIG_CLOCK_FREQ / 10;
    if ((int32_t)(now - last_status_emit_time) < (int32_t)status_period_ticks)
        return;
    last_status_emit_time = now;
    send_status_heartbeat();

    diag_task_heartbeat(diag_slot_rt_status_calls(),
                        diag_slot_rt_status_last_tick(),
                        diag_slot_rt_status_max_gap(),
                        timer_from_us(200000),
                        0); // no event tag — emit gap shows up as missing emits

#if defined(__linux__) || defined(__APPLE__)
    uint8_t status = runtime_handle_status(runtime_handle);
    int32_t c0 = runtime_get_stepper_count(runtime_handle, 0);
    int32_t c1 = runtime_get_stepper_count(runtime_handle, 1);
    int32_t c2 = runtime_get_stepper_count(runtime_handle, 2);
    extern uint32_t runtime_get_xdirect_write_count(void);
    uint32_t spi_writes = runtime_get_xdirect_write_count();
    extern uint64_t console_rx_bytes, console_tx_bytes, console_tx_drops;
    fprintf(stderr,
        "[sim-progress] status=%u counts=[%d,%d,%d]"
        " spi_writes=%u rx=%llu tx=%llu tx_drops=%llu\n",
        status, c0, c1, c2, spi_writes,
        (unsigned long long)console_rx_bytes,
        (unsigned long long)console_tx_bytes,
        (unsigned long long)console_tx_drops);
    fflush(stderr);
#endif
}
DECL_TASK(runtime_status_drain);

#else

// MOTION_RUNTIME=n build: no MCU-side motion engine. The Kalico envelope
// stays alive — the status heartbeat and the structured log drain remain the
// host's health and diagnostics feed.

void
runtime_diag_boot_snapshot_init(void)
{
    runtime_diag_prior_magic_raw = rt_diag_persistent.magic;
    runtime_diag_prior_packed_raw = rt_diag_persistent.last_packed;
    if (rt_diag_persistent.magic == RT_DIAG_MAGIC
        && rt_diag_persistent.last_packed != 0)
        runtime_diag_prior_boot_snapshot = rt_diag_persistent.last_packed;
    runtime_diag_progress(0xB0, 0, 0);
}
DECL_INIT(runtime_diag_boot_snapshot_init);

static uint32_t last_status_emit_time;

void
runtime_status_drain(void)
{
    uint32_t now = timer_read_time();
    const uint32_t status_period_ticks = CONFIG_CLOCK_FREQ / 10;
    if ((int32_t)(now - last_status_emit_time)
        >= (int32_t)status_period_ticks) {
        last_status_emit_time = now;
        send_status_heartbeat();
    }
    event_log_drain();
}
DECL_TASK(runtime_status_drain);

#endif
