#include <stdint.h>
#include <string.h>
#include "autoconf.h"
#include "board/internal.h"
#include "command.h"
#include "sched.h"
#include "fault_handler_internal.h"

extern volatile uint8_t runtime_liveness_ok;
#if CONFIG_MOTION_RUNTIME
extern void *runtime_handle;
extern uint32_t runtime_handle_tick_counter(void *handle);
extern uint8_t  runtime_handle_status(void *handle);
#endif

struct rt_diag_persistent {
    uint32_t magic;
    uint32_t last_packed;
    uint32_t last_us;
    uint32_t fault_count;
};
extern volatile struct rt_diag_persistent rt_diag_persistent;

static uint32_t preboot_cur_task_func;
static uint32_t preboot_cur_msg_kind;

// A task/msg marker left open across a reset would be closed by the first
// task hook against the fresh clock epoch — a wrapped duration that clamps to
// DIAG_STALL_CAP_CYC and poisons the boot replay's worst slots. Save the
// markers for the replay and clear them before any task hook runs.
static void
discard_preboot_progress_markers(void)
{
    if (live_snap.magic != LIVE_MAGIC)
        return;
    preboot_cur_task_func = live_snap.cur_task_func;
    preboot_cur_msg_kind = live_snap.cur_msg_kind;
    live_snap.cur_task_func = 0;
    live_snap.cur_msg_kind = 0;
    diag_cache_clean();
}

#if CONFIG_MACH_STM32H7
__attribute__((section(".bkp_bss"), used))
#else
__attribute__((section(".persistent_diag"), used))
#endif
volatile struct live_snapshot live_snap;

void
fault_handler_init(void)
{
#if (__CORTEX_M >= 3)
    SCB->SHCSR |= SCB_SHCSR_USGFAULTENA_Msk
                | SCB_SHCSR_BUSFAULTENA_Msk
                | SCB_SHCSR_MEMFAULTENA_Msk;
    SCB->CCR |= SCB_CCR_DIV_0_TRP_Msk;
    // Do not enable UNALIGN_TRP: unaligned half-word/word loads are common here.
#endif
#if CONFIG_MACH_STM32H7
    RCC->AHB4ENR |= RCC_AHB4ENR_BKPRAMEN;
    PWR->CR1 |= PWR_CR1_DBP;
    PWR->CR2 |= PWR_CR2_BREN;
    {
        volatile int spin = 0;
        while (!(PWR->CR2 & PWR_CR2_BRRDY) && spin < 100000) spin++;
    }
#endif
    discard_preboot_progress_markers();
}
DECL_INIT(fault_handler_init);

#include "board/misc.h"

uint32_t boot_first_tick;
static uint32_t last_emit_tick;
static uint32_t emits_done;
uint32_t reset_cause_snapshot;
static uint32_t reset_cause_raw;
static uint32_t prior_live_present_at_boot;
static uint32_t saved_prior_live;
static uint32_t saved_prior_engine;
static uint32_t saved_prior_tick;
static uint32_t saved_prior_last_run_tick;
static uint32_t saved_prior_samples;
uint32_t prior_run_froze;
uint32_t saved_prior_last_dispatch_func;
uint32_t saved_prior_last_dispatch_addr;
// Full copy of the crashed run's live_snap, taken at boot before the per-run
// fields are zeroed; all "prior run" reporting reads this, never live_snap.
struct live_snapshot prior_snap;

struct diag_counters prior_diag;
struct diag_event    prior_ring[DIAG_RING_LEN];
uint32_t             prior_diag_present;

#if CONFIG_MACH_STM32H7
#include "board/internal.h"
#endif

static uint32_t
read_reset_cause(void)
{
#if CONFIG_MACH_STM32H7
    return RCC->RSR;
#elif CONFIG_MACH_STM32F4
    return RCC->CSR;
#else
    return 0;
#endif
}

static void
clear_reset_cause(void)
{
#if CONFIG_MACH_STM32H7
    RCC->RSR |= RCC_RSR_RMVF;
#elif CONFIG_MACH_STM32F4
    RCC->CSR |= RCC_CSR_RMVF;
#endif
}

static void
fault_handler_report_boot_init(uint32_t now)
{
    boot_first_tick = now;
    boot_tick_initialized = 1;
    last_emit_tick = now - timer_from_us(2000000);
    reset_cause_snapshot = read_reset_cause();
    reset_cause_raw = reset_cause_snapshot;
    clear_reset_cause();
    if (live_snap.magic == LIVE_MAGIC) {
        prior_live_present_at_boot = 1;
        memcpy(&prior_snap, (const void *)&live_snap, sizeof(prior_snap));
        prior_snap.cur_task_func = preboot_cur_task_func;
        prior_snap.cur_msg_kind = preboot_cur_msg_kind;
        saved_prior_live          = live_snap.live;
        saved_prior_engine        = live_snap.engine_status;
        saved_prior_tick          = live_snap.tick_counter;
        saved_prior_last_run_tick = live_snap.last_engine_running_tick;
        saved_prior_samples       = live_snap.samples_taken;
        prior_run_froze           = live_snap.this_run_froze;
        saved_prior_last_dispatch_func = live_snap.last_dispatch_func;
        saved_prior_last_dispatch_addr = live_snap.last_dispatch_addr;
    } else {
        live_snap.iwdg_reset_count = 0;
    }
    // Per-run stats: replay the prior run's values (prior_snap) at boot,
    // then start this run from zero so each boot reports its own run.
    live_snap.worst_fg_stall_ticks = 0;
    live_snap.worst_fg_stall_pc    = 0;
    live_snap.worst_fg_stall_exc   = 0;
    live_snap.last_dispatch_func   = 0;
    live_snap.last_dispatch_addr   = 0;
    live_snap.this_run_froze       = 0;
    live_snap.cur_task_func        = 0;
    live_snap.cur_task_start       = 0;
    live_snap.worst_task_func      = 0;
    live_snap.worst_task_cyc       = 0;
    live_snap.cur_msg_kind         = 0;
    live_snap.cur_msg_start        = 0;
    live_snap.cur_msg_head         = 0;
    live_snap.worst_msg_kind       = 0;
    live_snap.worst_msg_cyc        = 0;
    live_snap.worst_msg_head       = 0;
    live_snap.demux_backlog_max    = 0;
    live_snap.demux_msgs_max       = 0;
    live_snap.ttc_caller           = 0;
    live_snap.ttc_func             = 0;
    live_snap.ttc_late             = 0;
    live_snap.ttc_count            = 0;
    live_snap.rearm_count          = 0;
    live_snap.rearm_min_margin     = (uint32_t)INT32_MAX;
    live_snap.rearm_min_oid        = 0;
    live_snap.rearm_min_waketime   = 0;
    live_snap.rearm_min_last_reset = 0;
    live_snap.rearm_min_discards   = 0;
    live_snap.rearm_armed          = 0;
    live_snap.rearm_below_floor    = 0;
    live_snap.worst_timer_func     = 0;
    live_snap.worst_timer_cyc      = 0;
    live_snap.step_spin_count      = 0;
    live_snap.step_spin_worst_cyc  = 0;
    live_snap.step_spin_stale_count = 0;
    live_snap.step_spin_stale_max  = 0;
    live_snap.step_spin_stale_first = 0;
#if CONFIG_MACH_STM32H7
    if (reset_cause_raw & RCC_RSR_IWDG1RSTF)
        live_snap.iwdg_reset_count++;
#elif CONFIG_MACH_STM32F4
    if (reset_cause_raw & RCC_CSR_IWDGRSTF)
        live_snap.iwdg_reset_count++;
#endif
    live_snap.samples_taken = 0;

    if (diag.magic == DIAG_MAGIC) {
        prior_diag_present = 1;
        prior_diag.magic                = diag.magic;
        prior_diag.tim5_irq_count       = diag.tim5_irq_count;
        prior_diag.tim5_irq_cycles_total = diag.tim5_irq_cycles_total;
        prior_diag.tim5_irq_cycles_max  = diag.tim5_irq_cycles_max;
        prior_diag.otg_irq_count        = diag.otg_irq_count;
        prior_diag.otg_irq_cycles_total = diag.otg_irq_cycles_total;
        prior_diag.otg_irq_cycles_max   = diag.otg_irq_cycles_max;
        prior_diag.rt_tick_count        = diag.rt_tick_count;
        prior_diag.rt_tick_cycles_max   = diag.rt_tick_cycles_max;
        prior_diag.rt_tick_cycles_total = diag.rt_tick_cycles_total;
        prior_diag.rt_eval_n            = diag.rt_eval_n;
        prior_diag.rt_eval_cycles_max   = diag.rt_eval_cycles_max;
        prior_diag.rt_eval_cycles_total = diag.rt_eval_cycles_total;
        prior_diag.rt_dvel_n            = diag.rt_dvel_n;
        prior_diag.rt_dvel_cycles_max   = diag.rt_dvel_cycles_max;
        prior_diag.rt_dvel_cycles_total = diag.rt_dvel_cycles_total;
        prior_diag.walk_cycles_max      = diag.walk_cycles_max;
        prior_diag.walk_n               = diag.walk_n;
        prior_diag.monomial_cycles_max  = diag.monomial_cycles_max;
        prior_diag.monomial_n           = diag.monomial_n;
        prior_diag.rt_isr_phase         = diag.rt_isr_phase;
        for (uint32_t axis = 0; axis < 3; axis++) {
            prior_diag.rt_curve_degree[axis]    = diag.rt_curve_degree[axis];
            prior_diag.rt_curve_cps_len[axis]   = diag.rt_curve_cps_len[axis];
            prior_diag.rt_curve_knots_len[axis] = diag.rt_curve_knots_len[axis];
        }
        for (uint32_t i = 0; i < DIAG_HIST_NBUCKETS; i++) {
            prior_diag.tim5_irq_buckets[i] = diag.tim5_irq_buckets[i];
            prior_diag.rt_tick_buckets[i]  = diag.rt_tick_buckets[i];
        }
        prior_diag.usb_out_calls        = diag.usb_out_calls;
        prior_diag.usb_out_max_gap_ticks = diag.usb_out_max_gap_ticks;
        prior_diag.usb_in_calls         = diag.usb_in_calls;
        prior_diag.usb_in_max_gap_ticks  = diag.usb_in_max_gap_ticks;
        prior_diag.runtime_drain_calls   = diag.runtime_drain_calls;
        prior_diag.runtime_drain_max_gap_ticks = diag.runtime_drain_max_gap_ticks;
        prior_diag.runtime_status_calls   = diag.runtime_status_calls;
        prior_diag.runtime_status_max_gap_ticks = diag.runtime_status_max_gap_ticks;
        prior_diag.tx_drops_kalico        = diag.tx_drops_kalico;
        prior_diag.tx_drops_klipper       = diag.tx_drops_klipper;
        prior_diag.tx_drops_transport_last_len = diag.tx_drops_transport_last_len;
        prior_diag.tx_drops_klipper_last_max = diag.tx_drops_klipper_last_max;
        prior_diag.ring_head            = diag.ring_head;
        prior_diag.ring_seq             = diag.ring_seq;
        prior_diag.ring_overflow        = diag.ring_overflow;
        prior_diag.boot_count           = diag.boot_count;
        prior_diag.systick_max_cyc        = diag.systick_max_cyc;
        prior_diag.stepout_max_cyc        = diag.stepout_max_cyc;
        prior_diag.stepout_burst_max_cyc  = diag.stepout_burst_max_cyc;
        prior_diag.usb_burst_max_cyc      = diag.usb_burst_max_cyc;
        prior_diag.tim5_ia_min_cyc        = diag.tim5_ia_min_cyc;
        prior_diag.tim5_ia_max_cyc        = diag.tim5_ia_max_cyc;
        prior_diag.tim5_ia_last_cyc       = diag.tim5_ia_last_cyc;
        prior_diag.usb_in_busy_n          = diag.usb_in_busy_n;
        prior_diag.usb_gintsts_sticky     = diag.usb_gintsts_sticky;
        prior_diag.usb_gintsts_now        = diag.usb_gintsts_now;
        prior_diag.usb_gintmsk_now        = diag.usb_gintmsk_now;
        prior_diag.usb_in_diepctl         = diag.usb_in_diepctl;
        prior_diag.usb_in_diepint         = diag.usb_in_diepint;
        prior_diag.usb_in_dtxfsts         = diag.usb_in_dtxfsts;
        prior_diag.usb_out_doepctl        = diag.usb_out_doepctl;
        prior_diag.usb_out_doepint        = diag.usb_out_doepint;
        prior_diag.out_unarmed_worst_cyc  = diag.out_unarmed_worst_cyc;
        prior_diag.out_unarmed_worst_end  = diag.out_unarmed_worst_end;
#if CONFIG_MOTION_RUNTIME
        {
            extern void kalico_stepout_late_get(uint32_t *out_max_late,
                                                uint32_t *out_late_count,
                                                uint32_t *out_max_drained);
            kalico_stepout_late_get(&prior_diag.stepout_late_max_cyc,
                                    &prior_diag.stepout_late_count,
                                    &prior_diag.stepout_late_max_drained);
        }
#endif
        for (uint32_t i = 0; i < DIAG_RING_LEN; i++) {
            prior_ring[i].tag       = diag_ring[i].tag;
            prior_ring[i]._pad0     = diag_ring[i]._pad0;
            prior_ring[i].seq       = diag_ring[i].seq;
            prior_ring[i].timestamp = diag_ring[i].timestamp;
            prior_ring[i].a         = diag_ring[i].a;
            prior_ring[i].b         = diag_ring[i].b;
        }
    }
    memset((void *)&diag, 0, sizeof(diag));
    diag.magic = DIAG_MAGIC;
    diag.boot_count = prior_diag_present ? (prior_diag.boot_count + 1) : 1;
    for (uint32_t i = 0; i < DIAG_RING_LEN; i++) {
        diag_ring[i].tag = DIAG_EV_NONE;
        diag_ring[i].seq = 0;
        diag_ring[i].timestamp = 0;
        diag_ring[i].a = 0;
        diag_ring[i].b = 0;
    }
    if (prior_diag_present) {
        output("prior_diag_at_init boot %u tim5_n %u otg_n %u out_n %u in_n %u"
               " drain_n %u stat_n %u ring_seq %u ring_overflow %u"
               " drops_kal %u drops_klp %u",
               prior_diag.boot_count,
               prior_diag.tim5_irq_count,
               prior_diag.otg_irq_count,
               prior_diag.usb_out_calls,
               prior_diag.usb_in_calls,
               prior_diag.runtime_drain_calls,
               prior_diag.runtime_status_calls,
               prior_diag.ring_seq,
               prior_diag.ring_overflow,
               prior_diag.tx_drops_kalico,
               prior_diag.tx_drops_klipper);
    }
    diag_cache_clean();
}

static void
fault_handler_report_liveness_update(uint32_t now)
{
    uint32_t live_now = runtime_liveness_ok;
    uint8_t engine_now = 0xFF;
    uint32_t tick_now = 0;
#if CONFIG_MOTION_RUNTIME
    if (runtime_handle) {
        tick_now = runtime_handle_tick_counter(runtime_handle);
        engine_now = runtime_handle_status(runtime_handle);
    }
#endif
    if (live_snap.magic != LIVE_MAGIC)
        live_snap.boot_count = 0;
    live_snap.live = live_now;
    live_snap.engine_status = (uint32_t)engine_now;
    live_snap.tick_counter = tick_now;
    live_snap.sample_time = now;
    live_snap.samples_taken++;
    if (engine_now == 1)
        live_snap.last_engine_running_tick = tick_now;
    live_snap.magic = LIVE_MAGIC;
}

static void
fault_handler_report_emit(uint32_t now)
{
    if (emits_done >= 3)
        return;
    uint32_t elapsed = now - last_emit_tick;
    if (elapsed < timer_from_us(2000000))
        return;
    last_emit_tick = now;
    uint32_t since_boot_us = (uint32_t)((uint64_t)(now - boot_first_tick)
                                        * 1000000u
                                        / CONFIG_CLOCK_FREQ);
    // Free-form %u, not name=%u: the decoder needs this to build #msg for
    // klippy.log; structured name=%u would break that path.
    output("boot_diag emit %u since_us %u rcc %u prior %u live %u engine %u tick %u",
           emits_done, since_boot_us, reset_cause_raw,
           (uint32_t)(fault_rec.magic == FAULT_MAGIC),
           live_snap.live, live_snap.engine_status, live_snap.tick_counter);
    if (prior_live_present_at_boot) {
        output("prior_live live %u engine %u tick %u last_run_tick %u samples %u",
               saved_prior_live, saved_prior_engine,
               saved_prior_tick, saved_prior_last_run_tick,
               saved_prior_samples);
    }
    output("fg_freeze stall_ticks %u pc %u exc %u iwdg %u last_disp_func %u last_disp_addr %u",
           prior_snap.worst_fg_stall_ticks,
           prior_snap.worst_fg_stall_pc,
           prior_snap.worst_fg_stall_exc,
           live_snap.iwdg_reset_count,
           prior_snap.last_dispatch_func,
           prior_snap.last_dispatch_addr);
    output("fg_task worst_func %u worst_cyc %u cur_func %u",
           prior_snap.worst_task_func,
           prior_snap.worst_task_cyc,
           prior_snap.cur_task_func);
    output("fg_msg worst_kind %u worst_cyc %u cur_kind %u backlog_max %u msgs_max %u",
           prior_snap.worst_msg_kind,
           prior_snap.worst_msg_cyc,
           prior_snap.cur_msg_kind,
           prior_snap.demux_backlog_max,
           prior_snap.demux_msgs_max);
    output("fg_msg_head worst_head %u cur_head %u",
           prior_snap.worst_msg_head,
           prior_snap.cur_msg_head);
    output("timer_too_close caller %u func %u late_cyc %u count %u",
           prior_snap.ttc_caller,
           prior_snap.ttc_func,
           prior_snap.ttc_late,
           prior_snap.ttc_count);
    output("step_rearm count %u min_margin_cyc %i armed %u below_floor %u"
           " oid %u waketime %u last_reset %u discards %u",
           prior_snap.rearm_count,
           (int32_t)prior_snap.rearm_min_margin,
           prior_snap.rearm_armed,
           prior_snap.rearm_below_floor,
           prior_snap.rearm_min_oid,
           prior_snap.rearm_min_waketime,
           prior_snap.rearm_min_last_reset,
           prior_snap.rearm_min_discards);
    output("sched_timer_worst func %u cyc %u",
           prior_snap.worst_timer_func,
           prior_snap.worst_timer_cyc);
    output("step_spin count %u worst_cyc %u stale_count %u stale_max %u"
           " stale_first %u",
           prior_snap.step_spin_count,
           prior_snap.step_spin_worst_cyc,
           prior_snap.step_spin_stale_count,
           prior_snap.step_spin_stale_max,
           prior_snap.step_spin_stale_first);
    if (fault_rec.magic == FAULT_MAGIC) {
        output("prior_fault kind %u count %u pc %u lr %u psr %u"
               " r0 %u r1 %u r2 %u r3 %u r12 %u",
               fault_rec.exc_kind, fault_rec.fault_count,
               fault_rec.pc, fault_rec.lr, fault_rec.psr,
               fault_rec.r0, fault_rec.r1, fault_rec.r2,
               fault_rec.r3, fault_rec.r12);
        output("prior_fault_status cfsr %u hfsr %u bfar %u mmfar %u"
               " shcsr %u exc_return %u",
               fault_rec.cfsr, fault_rec.hfsr,
               fault_rec.bfar, fault_rec.mmfar,
               fault_rec.shcsr, fault_rec.exc_return);
    }
    output("rt_diag_prior magic=%u packed=%u us=%u faults=%u",
           rt_diag_persistent.magic,
           rt_diag_persistent.last_packed,
           rt_diag_persistent.last_us,
           rt_diag_persistent.fault_count);
    extern volatile uint32_t sched_bad_add_caller;
    extern volatile uint32_t sched_bad_add_value;
    extern volatile uint32_t sched_bad_add_stack0;
    extern volatile uint32_t sched_bad_add_stack1;
    extern volatile uint32_t sched_bad_add_stack2;
    extern volatile uint32_t sched_bad_add_blocked_count;
    output("sched_bad_add caller %u value %u blocked %u"
           " sp0 %u sp1 %u sp2 %u",
           sched_bad_add_caller, sched_bad_add_value,
           sched_bad_add_blocked_count,
           sched_bad_add_stack0,
           sched_bad_add_stack1,
           sched_bad_add_stack2);

    if (prior_diag_present) {
        output("prior_diag_summary boot %u tim5_n %u tim5_max_cyc %u"
               " tim5_total_lo %u tim5_total_hi %u",
               prior_diag.boot_count,
               prior_diag.tim5_irq_count,
               prior_diag.tim5_irq_cycles_max,
               (uint32_t)(prior_diag.tim5_irq_cycles_total & 0xFFFFFFFFu),
               (uint32_t)(prior_diag.tim5_irq_cycles_total >> 32));
        output("prior_diag_summary_rt rt_n %u rt_max_cyc %u"
               " rt_total_lo %u rt_total_hi %u",
               prior_diag.rt_tick_count,
               prior_diag.rt_tick_cycles_max,
               (uint32_t)(prior_diag.rt_tick_cycles_total & 0xFFFFFFFFu),
               (uint32_t)(prior_diag.rt_tick_cycles_total >> 32));
        output("prior_diag_summary_eval n %u max %u total_lo %u total_hi %u",
               prior_diag.rt_eval_n, prior_diag.rt_eval_cycles_max,
               (uint32_t)(prior_diag.rt_eval_cycles_total & 0xFFFFFFFFu),
               (uint32_t)(prior_diag.rt_eval_cycles_total >> 32));
        output("prior_diag_summary_dvel n %u max %u total_lo %u total_hi %u",
               prior_diag.rt_dvel_n, prior_diag.rt_dvel_cycles_max,
               (uint32_t)(prior_diag.rt_dvel_cycles_total & 0xFFFFFFFFu),
               (uint32_t)(prior_diag.rt_dvel_cycles_total >> 32));
        output("prior_diag_phase walk_max %u walk_n %u mono_max %u mono_n %u"
               " isr_phase %u",
               prior_diag.walk_cycles_max, prior_diag.walk_n,
               prior_diag.monomial_cycles_max, prior_diag.monomial_n,
               prior_diag.rt_isr_phase);
        output("prior_diag_summary_curve x_deg %u x_cps %u x_knots %u"
               " y_deg %u y_cps %u y_knots %u z_deg %u z_cps %u z_knots %u",
               (uint32_t)prior_diag.rt_curve_degree[0],
               (uint32_t)prior_diag.rt_curve_cps_len[0],
               (uint32_t)prior_diag.rt_curve_knots_len[0],
               (uint32_t)prior_diag.rt_curve_degree[1],
               (uint32_t)prior_diag.rt_curve_cps_len[1],
               (uint32_t)prior_diag.rt_curve_knots_len[1],
               (uint32_t)prior_diag.rt_curve_degree[2],
               (uint32_t)prior_diag.rt_curve_cps_len[2],
               (uint32_t)prior_diag.rt_curve_knots_len[2]);
        output("prior_diag_summary_otg otg_n %u otg_max_cyc %u"
               " otg_total_lo %u otg_total_hi %u",
               prior_diag.otg_irq_count,
               prior_diag.otg_irq_cycles_max,
               (uint32_t)(prior_diag.otg_irq_cycles_total & 0xFFFFFFFFu),
               (uint32_t)(prior_diag.otg_irq_cycles_total >> 32));
        output("prior_diag_summary_block systick %u stepout %u"
               " stepout_burst %u usb_burst %u",
               prior_diag.systick_max_cyc,
               prior_diag.stepout_max_cyc,
               prior_diag.stepout_burst_max_cyc,
               prior_diag.usb_burst_max_cyc);
        output("prior_diag_summary_tim5ia min %u max %u last %u period %u",
               prior_diag.tim5_ia_min_cyc,
               prior_diag.tim5_ia_max_cyc,
               prior_diag.tim5_ia_last_cyc,
               (uint32_t)(CONFIG_CLOCK_FREQ / CONFIG_MOTION_SAMPLE_RATE_HZ));
        output("prior_diag_summary_usb in_busy %u gintsts_sticky %u gintsts %u"
               " gintmsk %u in_diepctl %u in_diepint %u in_dtxfsts %u"
               " out_doepctl %u out_doepint %u",
               prior_diag.usb_in_busy_n,
               prior_diag.usb_gintsts_sticky,
               prior_diag.usb_gintsts_now,
               prior_diag.usb_gintmsk_now,
               prior_diag.usb_in_diepctl,
               prior_diag.usb_in_diepint,
               prior_diag.usb_in_dtxfsts,
               prior_diag.usb_out_doepctl,
               prior_diag.usb_out_doepint);
        output("prior_diag_out_unarmed worst_cyc %u end_tick %u",
               prior_diag.out_unarmed_worst_cyc,
               prior_diag.out_unarmed_worst_end);
        output("prior_diag_tasks out_n %u out_max_gap %u in_n %u in_max_gap %u"
               " drain_n %u drain_max_gap %u stat_n %u stat_max_gap %u",
               prior_diag.usb_out_calls,
               prior_diag.usb_out_max_gap_ticks,
               prior_diag.usb_in_calls,
               prior_diag.usb_in_max_gap_ticks,
               prior_diag.runtime_drain_calls,
               prior_diag.runtime_drain_max_gap_ticks,
               prior_diag.runtime_status_calls,
               prior_diag.runtime_status_max_gap_ticks);
        output("prior_diag_drops kalico %u last_len %u klipper %u last_max %u"
               " ring_seq %u ring_overflow %u",
               prior_diag.tx_drops_kalico,
               prior_diag.tx_drops_transport_last_len,
               prior_diag.tx_drops_klipper,
               prior_diag.tx_drops_klipper_last_max,
               prior_diag.ring_seq,
               prior_diag.ring_overflow);
        // Histogram split across two outputs to stay within MESSAGE_MAX=64 B;
        // merging them overflows the wire message.
        output("prior_diag_hist_irq_lo b0 %u b1 %u b2 %u b3 %u b4 %u b5 %u b6 %u b7 %u",
               prior_diag.tim5_irq_buckets[0], prior_diag.tim5_irq_buckets[1],
               prior_diag.tim5_irq_buckets[2], prior_diag.tim5_irq_buckets[3],
               prior_diag.tim5_irq_buckets[4], prior_diag.tim5_irq_buckets[5],
               prior_diag.tim5_irq_buckets[6], prior_diag.tim5_irq_buckets[7]);
        output("prior_diag_hist_irq_hi b8 %u b9 %u b10 %u b11 %u b12 %u b13 %u b14 %u b15 %u",
               prior_diag.tim5_irq_buckets[8], prior_diag.tim5_irq_buckets[9],
               prior_diag.tim5_irq_buckets[10], prior_diag.tim5_irq_buckets[11],
               prior_diag.tim5_irq_buckets[12], prior_diag.tim5_irq_buckets[13],
               prior_diag.tim5_irq_buckets[14], prior_diag.tim5_irq_buckets[15]);
        output("prior_diag_hist_rt_lo b0 %u b1 %u b2 %u b3 %u b4 %u b5 %u b6 %u b7 %u",
               prior_diag.rt_tick_buckets[0], prior_diag.rt_tick_buckets[1],
               prior_diag.rt_tick_buckets[2], prior_diag.rt_tick_buckets[3],
               prior_diag.rt_tick_buckets[4], prior_diag.rt_tick_buckets[5],
               prior_diag.rt_tick_buckets[6], prior_diag.rt_tick_buckets[7]);
        output("prior_diag_hist_rt_hi b8 %u b9 %u b10 %u b11 %u b12 %u b13 %u b14 %u b15 %u",
               prior_diag.rt_tick_buckets[8], prior_diag.rt_tick_buckets[9],
               prior_diag.rt_tick_buckets[10], prior_diag.rt_tick_buckets[11],
               prior_diag.rt_tick_buckets[12], prior_diag.rt_tick_buckets[13],
               prior_diag.rt_tick_buckets[14], prior_diag.rt_tick_buckets[15]);
    }

    emits_done++;
}

void
fault_handler_report_task(void)
{
    uint32_t now = timer_read_time();
    if (!boot_tick_initialized) {
        fault_handler_report_boot_init(now);
        return;
    }
    fault_handler_report_liveness_update(now);
    fault_handler_report_emit(now);
}
DECL_TASK(fault_handler_report_task);
