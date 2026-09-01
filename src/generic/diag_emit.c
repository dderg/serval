#include <stdint.h>
#include "autoconf.h"
#include "board/internal.h"
#include "board/irq.h"
#include "event_log.h"
#include "fault_handler_internal.h"

static struct diag_event dump_ring[DIAG_RING_LEN];

static uint8_t
diag_ring_tag_level(uint8_t tag)
{
    switch (tag) {
    case DIAG_EV_RUST_FAULT:
        return EVENT_LOG_LEVEL_ERROR;
    case DIAG_EV_TIM5_LONG:
    case DIAG_EV_OTG_LONG:
    case DIAG_EV_TX_DROP_KAL:
    case DIAG_EV_TX_DROP_KLP:
        return EVENT_LOG_LEVEL_WARN;
    default:
        return EVENT_LOG_LEVEL_DEBUG;
    }
}

// Call only after the host's mcu-log hook is up (post-connect, from stepper.c);
// a boot-time emit would be dropped before the hook exists.
void
kalico_diag_emit_prior_crash(void)
{
    uint8_t iwdg = 0;
#if CONFIG_MACH_STM32H7
    iwdg = (reset_cause_snapshot & RCC_RSR_IWDG1RSTF) ? 1u : 0u;
#elif CONFIG_MACH_STM32F4
    iwdg = (reset_cause_snapshot & RCC_CSR_IWDGRSTF) ? 1u : 0u;
#endif
    uint8_t had_fault = (fault_rec.magic == FAULT_MAGIC) ? 1u : 0u;
    // klippy's connect-reset overwrites the RCC cause with SFTRST, so a real
    // foreground freeze survives only via prior_run_froze (in BKPSRAM); do not
    // drop it from this condition.
    // A "Timer too close" run must also replay the deep forensics (ring,
    // block_source, tim5_ia) — it is a clean shutdown, not an iwdg/fault.
    uint8_t abnormal = iwdg || had_fault || prior_run_froze
                       || prior_snap.ttc_count != 0;

    event_log_emit(abnormal ? EVENT_LOG_LEVEL_WARN : EVENT_LOG_LEVEL_DEBUG,
                    EVENT_LOG_SUBSYS_RUNTIME, EVENT_LOG_EVENT_RUNTIME_MCU_RESET,
                    0, reset_cause_snapshot, live_snap.iwdg_reset_count);

    if (had_fault) {
        event_log_emit(EVENT_LOG_LEVEL_ERROR, EVENT_LOG_SUBSYS_RUNTIME,
                        EVENT_LOG_EVENT_RUNTIME_HARD_FAULT,
                        (uint16_t)fault_rec.exc_kind, fault_rec.pc, fault_rec.lr);
        event_log_emit(EVENT_LOG_LEVEL_ERROR, EVENT_LOG_SUBSYS_RUNTIME,
                        EVENT_LOG_EVENT_RUNTIME_FAULT_STATUS, 0,
                        fault_rec.cfsr, fault_rec.hfsr);
    }

    if (prior_snap.worst_fg_stall_ticks) {
        event_log_emit(EVENT_LOG_LEVEL_WARN, EVENT_LOG_SUBSYS_RUNTIME,
                        EVENT_LOG_EVENT_RUNTIME_FG_FREEZE, 0,
                        prior_snap.worst_fg_stall_pc,
                        prior_snap.worst_fg_stall_ticks);
    }

    if (prior_snap.worst_task_cyc) {
        event_log_emit(EVENT_LOG_LEVEL_WARN, EVENT_LOG_SUBSYS_RUNTIME,
                        EVENT_LOG_EVENT_RUNTIME_FG_TASK, 0,
                        prior_snap.worst_task_func,
                        prior_snap.worst_task_cyc);
    }

    if (prior_snap.worst_msg_cyc || prior_snap.demux_msgs_max) {
        event_log_emit(EVENT_LOG_LEVEL_WARN, EVENT_LOG_SUBSYS_RUNTIME,
                        EVENT_LOG_EVENT_RUNTIME_FG_MSG, 0,
                        prior_snap.worst_msg_kind,
                        prior_snap.worst_msg_cyc);
        event_log_emit(EVENT_LOG_LEVEL_WARN, EVENT_LOG_SUBSYS_RUNTIME,
                        EVENT_LOG_EVENT_RUNTIME_FG_MSG_HEAD, 0,
                        prior_snap.worst_msg_head,
                        prior_snap.cur_msg_head);
        event_log_emit(EVENT_LOG_LEVEL_WARN, EVENT_LOG_SUBSYS_RUNTIME,
                        EVENT_LOG_EVENT_RUNTIME_FG_DEMUX, 0,
                        prior_snap.demux_backlog_max,
                        prior_snap.demux_msgs_max);
    }

    if (prior_snap.ttc_count) {
        event_log_emit(EVENT_LOG_LEVEL_WARN, EVENT_LOG_SUBSYS_RUNTIME,
                        EVENT_LOG_EVENT_RUNTIME_TIMER_TOO_CLOSE,
                        (uint16_t)(prior_snap.ttc_count > 0xFFFFu
                                   ? 0xFFFFu : prior_snap.ttc_count),
                        prior_snap.ttc_caller,
                        prior_snap.ttc_func);
        event_log_emit(EVENT_LOG_LEVEL_WARN, EVENT_LOG_SUBSYS_RUNTIME,
                        EVENT_LOG_EVENT_RUNTIME_TIMER_TOO_CLOSE_LATE, 0,
                        prior_snap.ttc_late,
                        prior_snap.ttc_count);
    }

    if (prior_snap.rearm_count) {
        event_log_emit(EVENT_LOG_LEVEL_WARN, EVENT_LOG_SUBSYS_MOTION,
                        EVENT_LOG_EVENT_MOTION_STEP_REARM, 0,
                        prior_snap.rearm_count,
                        prior_snap.rearm_min_margin);
        event_log_emit(EVENT_LOG_LEVEL_WARN, EVENT_LOG_SUBSYS_MOTION,
                        EVENT_LOG_EVENT_MOTION_STEP_REARM_TIGHT, 0,
                        prior_snap.rearm_armed,
                        prior_snap.rearm_below_floor);
    }

    if (abnormal) {
        extern volatile uint32_t runtime_diag_prior_packed_raw;
        uint32_t fc = had_fault ? fault_rec.fault_count : 0u;
        event_log_emit(EVENT_LOG_LEVEL_WARN, EVENT_LOG_SUBSYS_RUNTIME,
                        EVENT_LOG_EVENT_RUNTIME_RT_PROGRESS, 0,
                        runtime_diag_prior_packed_raw, fc);

        event_log_emit(EVENT_LOG_LEVEL_WARN, EVENT_LOG_SUBSYS_RUNTIME,
                        EVENT_LOG_EVENT_RUNTIME_LAST_DISPATCH, 0,
                        saved_prior_last_dispatch_func,
                        saved_prior_last_dispatch_addr);

        if (prior_diag_present) {
            event_log_emit(EVENT_LOG_LEVEL_WARN, EVENT_LOG_SUBSYS_RUNTIME,
                            EVENT_LOG_EVENT_RUNTIME_ISR_PHASE, 0,
                            prior_diag.rt_isr_phase, prior_diag.ring_overflow);
            event_log_emit(EVENT_LOG_LEVEL_WARN, EVENT_LOG_SUBSYS_RUNTIME,
                            EVENT_LOG_EVENT_RUNTIME_BLOCK_SOURCE, 0,
                            prior_diag.usb_burst_max_cyc, 0);
            event_log_emit(EVENT_LOG_LEVEL_WARN, EVENT_LOG_SUBSYS_RUNTIME,
                            EVENT_LOG_EVENT_RUNTIME_TIM5_IA, 0,
                            prior_diag.tim5_ia_min_cyc,
                            prior_diag.tim5_ia_max_cyc);

            uint32_t head = prior_diag.ring_head & DIAG_RING_MASK;
            for (uint32_t i = 0; i < DIAG_RING_LEN; i++) {
                uint32_t idx = (head + i) & DIAG_RING_MASK;
                uint8_t tag = prior_ring[idx].tag;
                if (tag == DIAG_EV_NONE)
                    continue;
                event_log_emit(diag_ring_tag_level(tag), EVENT_LOG_SUBSYS_DIAG,
                                tag, 0, prior_ring[idx].a, prior_ring[idx].b);
            }
        }
    }
}

void
kalico_diag_emit_live(void)
{
    // ISRs push to diag_ring concurrently; copy it under one irq_save so the
    // snapshot is consistent.
    irqstatus_t flag = irq_save();
    uint32_t head          = diag.ring_head & DIAG_RING_MASK;
    uint32_t ring_seq      = diag.ring_seq;
    uint32_t ring_overflow = diag.ring_overflow;
    for (uint32_t i = 0; i < DIAG_RING_LEN; i++) {
        dump_ring[i].tag = diag_ring[i].tag;
        dump_ring[i].a   = diag_ring[i].a;
        dump_ring[i].b   = diag_ring[i].b;
    }
    irq_restore(flag);

    uint32_t now = timer_read_time();
    uint32_t uptime_us = boot_tick_initialized
        ? (uint32_t)((uint64_t)(now - boot_first_tick) * 1000000u / CONFIG_CLOCK_FREQ)
        : 0u;
    event_log_emit(EVENT_LOG_LEVEL_DEBUG, EVENT_LOG_SUBSYS_RUNTIME,
                    EVENT_LOG_EVENT_RUNTIME_DIAG_DUMP, 0, uptime_us, ring_seq);

    event_log_emit(EVENT_LOG_LEVEL_DEBUG, EVENT_LOG_SUBSYS_RUNTIME,
                    EVENT_LOG_EVENT_RUNTIME_ISR_PHASE, 0,
                    diag.rt_isr_phase, ring_overflow);
    event_log_emit(EVENT_LOG_LEVEL_DEBUG, EVENT_LOG_SUBSYS_RUNTIME,
                    EVENT_LOG_EVENT_RUNTIME_BLOCK_SOURCE, 0,
                    diag.usb_burst_max_cyc, 0);
    event_log_emit(EVENT_LOG_LEVEL_DEBUG, EVENT_LOG_SUBSYS_RUNTIME,
                    EVENT_LOG_EVENT_RUNTIME_TIM5_IA, 0,
                    diag.tim5_ia_min_cyc, diag.tim5_ia_max_cyc);

    if (live_snap.worst_fg_stall_ticks) {
        event_log_emit(EVENT_LOG_LEVEL_DEBUG, EVENT_LOG_SUBSYS_RUNTIME,
                        EVENT_LOG_EVENT_RUNTIME_FG_FREEZE, 0,
                        live_snap.worst_fg_stall_pc,
                        live_snap.worst_fg_stall_ticks);
    }

    if (live_snap.worst_task_cyc) {
        event_log_emit(EVENT_LOG_LEVEL_DEBUG, EVENT_LOG_SUBSYS_RUNTIME,
                        EVENT_LOG_EVENT_RUNTIME_FG_TASK, 0,
                        live_snap.worst_task_func,
                        live_snap.worst_task_cyc);
    }

    if (live_snap.worst_msg_cyc || live_snap.demux_msgs_max) {
        event_log_emit(EVENT_LOG_LEVEL_DEBUG, EVENT_LOG_SUBSYS_RUNTIME,
                        EVENT_LOG_EVENT_RUNTIME_FG_MSG, 0,
                        live_snap.worst_msg_kind,
                        live_snap.worst_msg_cyc);
        event_log_emit(EVENT_LOG_LEVEL_DEBUG, EVENT_LOG_SUBSYS_RUNTIME,
                        EVENT_LOG_EVENT_RUNTIME_FG_MSG_HEAD, 0,
                        live_snap.worst_msg_head,
                        live_snap.cur_msg_head);
        event_log_emit(EVENT_LOG_LEVEL_DEBUG, EVENT_LOG_SUBSYS_RUNTIME,
                        EVENT_LOG_EVENT_RUNTIME_FG_DEMUX, 0,
                        live_snap.demux_backlog_max,
                        live_snap.demux_msgs_max);
    }

    if (live_snap.ttc_count) {
        event_log_emit(EVENT_LOG_LEVEL_DEBUG, EVENT_LOG_SUBSYS_RUNTIME,
                        EVENT_LOG_EVENT_RUNTIME_TIMER_TOO_CLOSE,
                        (uint16_t)(live_snap.ttc_count > 0xFFFFu
                                   ? 0xFFFFu : live_snap.ttc_count),
                        live_snap.ttc_caller,
                        live_snap.ttc_func);
        event_log_emit(EVENT_LOG_LEVEL_DEBUG, EVENT_LOG_SUBSYS_RUNTIME,
                        EVENT_LOG_EVENT_RUNTIME_TIMER_TOO_CLOSE_LATE, 0,
                        live_snap.ttc_late,
                        live_snap.ttc_count);
    }

    if (live_snap.rearm_count) {
        event_log_emit(EVENT_LOG_LEVEL_DEBUG, EVENT_LOG_SUBSYS_MOTION,
                        EVENT_LOG_EVENT_MOTION_STEP_REARM, 0,
                        live_snap.rearm_count,
                        live_snap.rearm_min_margin);
        event_log_emit(EVENT_LOG_LEVEL_DEBUG, EVENT_LOG_SUBSYS_MOTION,
                        EVENT_LOG_EVENT_MOTION_STEP_REARM_TIGHT, 0,
                        live_snap.rearm_armed,
                        live_snap.rearm_below_floor);
    }

    for (uint32_t i = 0; i < DIAG_RING_LEN; i++) {
        uint32_t idx = (head + i) & DIAG_RING_MASK;
        uint8_t tag = dump_ring[idx].tag;
        if (tag == DIAG_EV_NONE)
            continue;
        event_log_emit(diag_ring_tag_level(tag), EVENT_LOG_SUBSYS_DIAG,
                        tag, 0, dump_ring[idx].a, dump_ring[idx].b);
    }
}
