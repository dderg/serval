#ifndef __GENERIC_FAULT_HANDLER_INTERNAL_H
#define __GENERIC_FAULT_HANDLER_INTERNAL_H

#include <stdint.h>
#include "autoconf.h"
#include "board/internal.h"

extern uint32_t timer_read_time(void);

#define FAULT_MAGIC 0x46415541u

struct fault_record {
    uint32_t magic;
    uint32_t exc_kind;
    uint32_t r0, r1, r2, r3, r12, lr, pc, psr;
    uint32_t cfsr, hfsr, dfsr, bfar, mmfar, afsr;
    uint32_t exc_return;
    uint32_t shcsr;
    uint32_t fault_count;
};

extern volatile struct fault_record fault_rec;

// Bump on any live_snapshot layout change so a reflash (RAM survives, old magic
// still matches) can't seed the new fields with stale bytes — a mismatch forces
// the cold-init zero pass.
#define LIVE_MAGIC 0x4C495648u

struct live_snapshot {
    uint32_t magic;
    uint32_t live;
    uint32_t engine_status;
    uint32_t tick_counter;
    uint32_t sample_time;
    uint32_t boot_count;
    uint32_t last_engine_running_tick;
    uint32_t samples_taken;
    uint32_t worst_fg_stall_ticks;
    uint32_t worst_fg_stall_pc;
    uint32_t worst_fg_stall_exc;
    uint32_t iwdg_reset_count;
    uint32_t last_dispatch_func;
    uint32_t last_dispatch_addr;
    uint32_t this_run_froze;
    uint32_t cur_task_func;
    uint32_t cur_task_start;
    uint32_t worst_task_func;
    uint32_t worst_task_cyc;
    uint32_t cur_msg_kind;
    uint32_t cur_msg_start;
    uint32_t cur_msg_head;
    uint32_t worst_msg_kind;
    uint32_t worst_msg_cyc;
    uint32_t worst_msg_head;
    uint32_t demux_backlog_max;
    uint32_t demux_msgs_max;
    uint32_t ttc_caller;
    uint32_t ttc_func;
    uint32_t ttc_late;
    uint32_t ttc_count;
    uint32_t rearm_count;
    uint32_t rearm_min_margin;
    uint32_t rearm_armed;
    uint32_t rearm_below_floor;
    uint32_t worst_timer_func;
    uint32_t worst_timer_cyc;
    uint32_t step_spin_count;
    uint32_t step_spin_worst_cyc;
    uint32_t step_spin_stale_count;
    uint32_t step_spin_stale_max;
    uint32_t step_spin_stale_first;
};

extern volatile struct live_snapshot live_snap;
// Full copy of the crashed run's live_snap, taken at boot before the per-run
// fields are zeroed; all "prior run" reporting reads this, never live_snap.
extern struct live_snapshot prior_snap;

// H7 BKPSRAM is D-cache-backed: writes need SCB_CleanDCache_by_Addr; a bare
// __DSB() drains only the store buffer, not the cache lines, so crash records
// would be lost across reset.
#if CONFIG_MACH_STM32H7
static inline void
diag_cache_clean(void)
{
    extern uint8_t _bkp_bss_start, _bkp_bss_end;
    uint32_t addr = (uint32_t)&_bkp_bss_start;
    uint32_t size = (uint32_t)(&_bkp_bss_end - &_bkp_bss_start);
    SCB_CleanDCache_by_Addr((uint32_t*)addr, (int32_t)size);
    __DSB();
}
#else
static inline void diag_cache_clean(void) { __DSB(); }
#endif

// Durations past ~2 s mean "stuck"; cap so the 32-bit cycle counter can't wrap
// across a multi-second hang into a meaningless huge value.
#define DIAG_STALL_CAP_CYC (CONFIG_CLOCK_FREQ * 2u)

static inline void
diag_update_worst(volatile uint32_t *worst_cyc, volatile uint32_t *worst_id,
                  uint32_t dur, uint32_t id)
{
    if (dur > DIAG_STALL_CAP_CYC)
        dur = DIAG_STALL_CAP_CYC;
    if (dur > *worst_cyc) {
        *worst_cyc = dur;
        *worst_id = id;
    }
}

static inline void
diag_update_worst_msg(uint32_t dur, uint32_t kind, uint32_t head)
{
    if (dur > DIAG_STALL_CAP_CYC)
        dur = DIAG_STALL_CAP_CYC;
    if (dur > live_snap.worst_msg_cyc) {
        live_snap.worst_msg_cyc = dur;
        live_snap.worst_msg_kind = kind;
        live_snap.worst_msg_head = head;
    }
}

#define DIAG_MAGIC      0x4449414Eu
#define DIAG_RING_LEN   32
#define DIAG_RING_MASK  (DIAG_RING_LEN - 1)
_Static_assert((DIAG_RING_LEN & DIAG_RING_MASK) == 0,
               "DIAG_RING_LEN must be a power of two for DIAG_RING_MASK");

enum {
    DIAG_EV_NONE          = 0,
    DIAG_EV_TIM5_LONG     = 1,
    DIAG_EV_OTG_LONG      = 2,
    DIAG_EV_USB_OUT_GAP   = 3,
    DIAG_EV_USB_IN_GAP    = 4,
    DIAG_EV_TX_DROP_KAL   = 5,
    DIAG_EV_TX_DROP_KLP   = 6,
    DIAG_EV_ENGINE_XITION = 7,
    DIAG_EV_RUST_FAULT    = 8,
};

struct diag_event {
    uint8_t  tag;
    uint8_t  _pad0;
    uint16_t seq;
    uint32_t timestamp;
    uint32_t a;
    uint32_t b;
};

extern volatile struct diag_event diag_ring[DIAG_RING_LEN];
extern struct diag_event prior_ring[DIAG_RING_LEN];

#define DIAG_HIST_NBUCKETS 16
#define DIAG_HIST_SHIFT    12

struct diag_counters {
    uint32_t magic;

    uint32_t tim5_irq_count;
    uint64_t tim5_irq_cycles_total;
    uint32_t tim5_irq_cycles_max;
    uint32_t otg_irq_count;
    uint64_t otg_irq_cycles_total;
    uint32_t otg_irq_cycles_max;

    uint32_t tim5_irq_buckets[DIAG_HIST_NBUCKETS];
    uint32_t rt_tick_count;
    uint32_t rt_tick_cycles_max;
    uint64_t rt_tick_cycles_total;
    uint32_t rt_tick_buckets[DIAG_HIST_NBUCKETS];

    uint32_t rt_eval_n;
    uint32_t rt_eval_cycles_max;
    uint64_t rt_eval_cycles_total;
    uint32_t rt_dvel_n;
    uint32_t rt_dvel_cycles_max;
    uint64_t rt_dvel_cycles_total;

    uint32_t walk_cycles_max;
    uint32_t walk_n;
    uint32_t monomial_cycles_max;
    uint32_t monomial_n;

    uint32_t rt_isr_phase;

    uint8_t  rt_curve_degree[3];
    uint16_t rt_curve_cps_len[3];
    uint16_t rt_curve_knots_len[3];

    uint32_t usb_out_calls;
    uint32_t usb_out_last_tick;
    uint32_t usb_out_max_gap_ticks;
    uint32_t usb_in_calls;
    uint32_t usb_in_last_tick;
    uint32_t usb_in_max_gap_ticks;
    uint32_t runtime_drain_calls;
    uint32_t runtime_drain_last_tick;
    uint32_t runtime_drain_max_gap_ticks;
    uint32_t runtime_status_calls;
    uint32_t runtime_status_last_tick;
    uint32_t runtime_status_max_gap_ticks;

    uint32_t tx_drops_kalico;
    uint32_t tx_drops_klipper;
    uint32_t tx_drops_transport_last_len;
    uint32_t tx_drops_klipper_last_max;

    uint32_t ring_head;
    uint32_t ring_seq;
    uint32_t ring_overflow;

    uint32_t boot_count;

    uint32_t otg_rxflvl_fires;
    uint32_t otg_iepint_fires;
    uint32_t otg_otherflag_fires;
    uint32_t otg_otherflag_last_sts;

    uint32_t notify_bulk_out_calls;
    uint32_t task_invoke_count;
    uint32_t usb_read_zero_returns;
    uint32_t usb_read_data_returns;

    uint32_t otg_gintmsk_now;
    uint32_t otg_gintsts_now;

    uint32_t out_ep_doepctl;
    uint32_t out_ep_doeptsiz;
    uint32_t out_ep_doepint;
    uint32_t enable_rx_n;
    uint32_t enable_rx_rearmed_n;
    uint32_t peek_empty_n;
    uint32_t peek_data_n;

    uint32_t systick_max_cyc;
    uint32_t stepout_max_cyc;
    uint32_t stepout_burst_max_cyc;
    uint32_t usb_burst_max_cyc;

    uint32_t tim5_ia_min_cyc;
    uint32_t tim5_ia_max_cyc;
    uint32_t tim5_ia_last_cyc;

    uint32_t usb_in_busy_n;
    uint32_t usb_gintsts_sticky;
    uint32_t usb_gintsts_now;
    uint32_t usb_gintmsk_now;
    uint32_t usb_in_diepctl;
    uint32_t usb_in_diepint;
    uint32_t usb_in_dtxfsts;
    uint32_t usb_out_doepctl;
    uint32_t usb_out_doepint;
    uint32_t out_unarmed_worst_cyc;
    uint32_t out_unarmed_worst_end;

    uint32_t stepout_late_max_cyc;
    uint32_t stepout_late_count;
    uint32_t stepout_late_max_drained;
};

extern volatile struct diag_counters diag;
extern struct diag_counters prior_diag;
extern uint32_t prior_diag_present;

extern uint32_t boot_tick_initialized;
extern uint32_t boot_first_tick;
extern uint32_t reset_cause_snapshot;
extern uint32_t prior_run_froze;
extern uint32_t saved_prior_last_dispatch_func;
extern uint32_t saved_prior_last_dispatch_addr;

#endif
