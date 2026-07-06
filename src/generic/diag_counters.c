#include <stdint.h>
#include "autoconf.h"
#include "fault_handler_internal.h"

extern void diag_ring_push(uint8_t tag, uint32_t a, uint32_t b);

#if CONFIG_MACH_STM32H7
__attribute__((section(".bkp_bss"), used))
#else
__attribute__((section(".persistent_diag"), used))
#endif
volatile struct diag_counters diag;

#define FG_FREEZE_REPORT_THRESHOLD 8

uint32_t boot_tick_initialized;

void
diag_tim5_account(uint32_t enter_cycles, uint32_t exit_cycles)
{
    uint32_t dur = exit_cycles - enter_cycles;
    diag.tim5_irq_count++;

    static uint32_t prev_enter;
    static uint8_t  have_prev;
    if (have_prev) {
        uint32_t ia = enter_cycles - prev_enter;
        diag.tim5_ia_last_cyc = ia;
        if (ia > diag.tim5_ia_max_cyc)
            diag.tim5_ia_max_cyc = ia;
        if (diag.tim5_ia_min_cyc == 0 || ia < diag.tim5_ia_min_cyc)
            diag.tim5_ia_min_cyc = ia;
    }
    prev_enter = enter_cycles;
    have_prev = 1;

    diag.tim5_irq_cycles_total += dur;
    if (dur > diag.tim5_irq_cycles_max)
        diag.tim5_irq_cycles_max = dur;
    uint32_t bucket = dur >> DIAG_HIST_SHIFT;
    if (bucket >= DIAG_HIST_NBUCKETS)
        bucket = DIAG_HIST_NBUCKETS - 1;
    diag.tim5_irq_buckets[bucket]++;
    if (dur > 26000u)
        diag_ring_push(DIAG_EV_TIM5_LONG, dur, enter_cycles);

    static uint32_t fg_hb_prev;
    static uint32_t fg_stall_ticks;
    static uint8_t  fg_init;
    static uint8_t  fg_seen_advance;
    uint32_t hb = live_snap.samples_taken;
    if (!fg_init) {
        fg_hb_prev = hb;
        fg_init = 1;
    } else if (hb != fg_hb_prev) {
        fg_hb_prev = hb;
        fg_stall_ticks = 0;
        fg_seen_advance = 1;
    } else if (fg_seen_advance) {
        fg_stall_ticks++;
        if (fg_stall_ticks >= FG_FREEZE_REPORT_THRESHOLD)
            live_snap.this_run_froze = 1;
        if (fg_stall_ticks > live_snap.worst_fg_stall_ticks) {
            extern uint32_t runtime_tim5_stacked_pc(void);
            extern uint32_t runtime_tim5_stacked_exc(void);
            live_snap.worst_fg_stall_ticks = fg_stall_ticks;
            live_snap.worst_fg_stall_pc    = runtime_tim5_stacked_pc();
            live_snap.worst_fg_stall_exc   = runtime_tim5_stacked_exc();
        }
    }

    // A task/message that never returns is invisible to the foreground enter
    // hooks (they only close completed work), so promote the in-progress task
    // and message growing durations into their worst slots here each tick.
    // Not before boot init: a cur_task/cur_msg left open by a reset-command
    // reboot persists in this RAM, and timing it against the fresh (near-zero)
    // clock caps worst_msg_cyc with garbage before report_task zeroes it.
    if (!boot_tick_initialized)
        return;
    uint32_t mon_now = timer_read_time();
    if (live_snap.cur_task_func)
        diag_update_worst(&live_snap.worst_task_cyc, &live_snap.worst_task_func,
                          mon_now - live_snap.cur_task_start,
                          live_snap.cur_task_func);
    if (live_snap.cur_msg_kind)
        diag_update_worst_msg(mon_now - live_snap.cur_msg_start,
                              live_snap.cur_msg_kind, live_snap.cur_msg_head);
}

__attribute__((used, externally_visible))
void
diag_rt_eval_account(uint32_t cycles)
{
    diag.rt_eval_n++;
    diag.rt_eval_cycles_total += cycles;
    if (cycles > diag.rt_eval_cycles_max)
        diag.rt_eval_cycles_max = cycles;
}

__attribute__((used, externally_visible))
void
diag_rt_curve_meta(uint32_t axis_idx, uint32_t degree,
                   uint32_t cps_len, uint32_t knots_len)
{
    if (axis_idx >= 3) return;
    diag.rt_curve_degree[axis_idx]    = (uint8_t)(degree & 0xFFu);
    diag.rt_curve_cps_len[axis_idx]   = (uint16_t)(cps_len & 0xFFFFu);
    diag.rt_curve_knots_len[axis_idx] = (uint16_t)(knots_len & 0xFFFFu);
}

__attribute__((used, externally_visible))
void
diag_rt_dvel_account(uint32_t cycles)
{
    diag.rt_dvel_n++;
    diag.rt_dvel_cycles_total += cycles;
    if (cycles > diag.rt_dvel_cycles_max)
        diag.rt_dvel_cycles_max = cycles;
}

__attribute__((used, externally_visible))
void
diag_walk_account(uint32_t cycles)
{
    diag.walk_n++;
    if (cycles > diag.walk_cycles_max)
        diag.walk_cycles_max = cycles;
}

__attribute__((used, externally_visible))
void
diag_monomial_account(uint32_t cycles)
{
    diag.monomial_n++;
    if (cycles > diag.monomial_cycles_max)
        diag.monomial_cycles_max = cycles;
}

__attribute__((used, externally_visible))
void
runtime_set_isr_phase(uint32_t phase)
{
    diag.rt_isr_phase = phase;
}

void
diag_runtime_tick_account(uint32_t cycles)
{
    diag.rt_tick_count++;
    diag.rt_tick_cycles_total += cycles;
    if (cycles > diag.rt_tick_cycles_max)
        diag.rt_tick_cycles_max = cycles;
    uint32_t bucket = cycles >> DIAG_HIST_SHIFT;
    if (bucket >= DIAG_HIST_NBUCKETS)
        bucket = DIAG_HIST_NBUCKETS - 1;
    diag.rt_tick_buckets[bucket]++;
}

void diag_usb_burst_track(uint32_t enter_cycles, uint32_t exit_cycles);

void
diag_otg_account(uint32_t enter_cycles, uint32_t exit_cycles)
{
    uint32_t dur = exit_cycles - enter_cycles;
    diag.otg_irq_count++;
    diag.otg_irq_cycles_total += dur;
    if (dur > diag.otg_irq_cycles_max)
        diag.otg_irq_cycles_max = dur;
    if (dur > 26000u)
        diag_ring_push(DIAG_EV_OTG_LONG, dur, enter_cycles);
    diag_usb_burst_track(enter_cycles, exit_cycles);
}

#define DIAG_BURST_GAP_CYC 13000u

static inline void
diag_burst_fold(volatile uint32_t *max_out,
                uint32_t *start, uint32_t *last_exit,
                uint32_t enter_cycles, uint32_t exit_cycles)
{
    uint32_t gap = enter_cycles - *last_exit;
    if (*last_exit == 0 || gap > DIAG_BURST_GAP_CYC) {
        *start = enter_cycles;
    }
    *last_exit = exit_cycles;
    uint32_t span = exit_cycles - *start;
    if (span > *max_out)
        *max_out = span;
}

void
diag_systick_account(uint32_t enter_cycles, uint32_t exit_cycles)
{
    uint32_t dur = exit_cycles - enter_cycles;
    if (dur > diag.systick_max_cyc)
        diag.systick_max_cyc = dur;
}

void
diag_stepout_account(uint32_t enter_cycles, uint32_t exit_cycles)
{
    static uint32_t burst_start;
    static uint32_t burst_last_exit;
    uint32_t dur = exit_cycles - enter_cycles;
    if (dur > diag.stepout_max_cyc)
        diag.stepout_max_cyc = dur;
    diag_burst_fold(&diag.stepout_burst_max_cyc,
                    &burst_start, &burst_last_exit,
                    enter_cycles, exit_cycles);
}

void
diag_usb_burst_track(uint32_t enter_cycles, uint32_t exit_cycles)
{
    static uint32_t burst_start;
    static uint32_t burst_last_exit;
    diag_burst_fold(&diag.usb_burst_max_cyc,
                    &burst_start, &burst_last_exit,
                    enter_cycles, exit_cycles);
}

volatile uint32_t *diag_slot_usb_out_calls(void)        { return &diag.usb_out_calls; }
volatile uint32_t *diag_slot_usb_out_last_tick(void)    { return &diag.usb_out_last_tick; }
volatile uint32_t *diag_slot_usb_out_max_gap(void)      { return &diag.usb_out_max_gap_ticks; }
volatile uint32_t *diag_slot_usb_in_calls(void)         { return &diag.usb_in_calls; }
volatile uint32_t *diag_slot_usb_in_last_tick(void)     { return &diag.usb_in_last_tick; }
volatile uint32_t *diag_slot_usb_in_max_gap(void)       { return &diag.usb_in_max_gap_ticks; }
volatile uint32_t *diag_slot_rt_drain_calls(void)       { return &diag.runtime_drain_calls; }
volatile uint32_t *diag_slot_rt_drain_last_tick(void)   { return &diag.runtime_drain_last_tick; }
volatile uint32_t *diag_slot_rt_drain_max_gap(void)     { return &diag.runtime_drain_max_gap_ticks; }
volatile uint32_t *diag_slot_rt_status_calls(void)      { return &diag.runtime_status_calls; }
volatile uint32_t *diag_slot_rt_status_last_tick(void)  { return &diag.runtime_status_last_tick; }
volatile uint32_t *diag_slot_rt_status_max_gap(void)    { return &diag.runtime_status_max_gap_ticks; }

volatile uint32_t *diag_slot_otg_rxflvl(void)         { return &diag.otg_rxflvl_fires; }
volatile uint32_t *diag_slot_otg_iepint(void)         { return &diag.otg_iepint_fires; }
volatile uint32_t *diag_slot_otg_other(void)          { return &diag.otg_otherflag_fires; }
volatile uint32_t *diag_slot_otg_other_sts(void)      { return &diag.otg_otherflag_last_sts; }
volatile uint32_t *diag_slot_notify_bulk_out(void)    { return &diag.notify_bulk_out_calls; }
volatile uint32_t *diag_slot_task_invoke(void)        { return &diag.task_invoke_count; }
volatile uint32_t *diag_slot_read_zero(void)          { return &diag.usb_read_zero_returns; }
volatile uint32_t *diag_slot_read_data(void)          { return &diag.usb_read_data_returns; }

// An armed idle OUT endpoint keeps DOEPCTL.EPENA set, so unarmed time only
// accrues between packet reception and the foreground's consume-and-rearm —
// exactly the window where the host's writes back up. Latch the worst episode
// and when it ended so a host-side write timeout can be matched against it.
#define USB_OUT_DOEPCTL_EPENA_BIT (1u << 31)

static void
diag_track_out_unarmed(uint32_t out_doepctl)
{
    extern uint32_t timer_read_time(void);
    static uint32_t unarmed_since;
    static uint8_t unarmed;
    uint32_t now = timer_read_time();
    if (!(out_doepctl & USB_OUT_DOEPCTL_EPENA_BIT)) {
        if (!unarmed) {
            unarmed = 1;
            unarmed_since = now;
        }
        uint32_t dur = now - unarmed_since;
        if (dur > diag.out_unarmed_worst_cyc) {
            diag.out_unarmed_worst_cyc = dur;
            diag.out_unarmed_worst_end = now;
        }
    } else {
        unarmed = 0;
    }
}

void
diag_usb_poll(uint32_t gintsts, uint32_t gintmsk, uint32_t in_diepctl,
              uint32_t in_diepint, uint32_t in_dtxfsts, uint32_t out_doepctl,
              uint32_t out_doepint)
{
    diag.usb_gintsts_sticky |= gintsts;
    diag.usb_gintsts_now = gintsts;
    diag.usb_gintmsk_now = gintmsk;
    diag.usb_in_diepctl = in_diepctl;
    diag.usb_in_diepint = in_diepint;
    diag.usb_in_dtxfsts = in_dtxfsts;
    diag.usb_out_doepctl = out_doepctl;
    diag.usb_out_doepint = out_doepint;
    diag_track_out_unarmed(out_doepctl);
}

volatile uint32_t *diag_slot_enable_rx(void)        { return &diag.enable_rx_n; }
volatile uint32_t *diag_slot_enable_rx_rearm(void)  { return &diag.enable_rx_rearmed_n; }
volatile uint32_t *diag_slot_peek_empty(void)       { return &diag.peek_empty_n; }
volatile uint32_t *diag_slot_peek_data(void)        { return &diag.peek_data_n; }
