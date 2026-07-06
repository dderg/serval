#include <stdint.h>
#include "autoconf.h"
#include "board/irq.h"
#include "fault_handler_internal.h"

#if CONFIG_MACH_STM32H7
__attribute__((section(".bkp_bss"), used))
#else
__attribute__((section(".persistent_diag"), used))
#endif
volatile struct diag_event diag_ring[DIAG_RING_LEN];
static volatile uint8_t diag_ring_frozen;

__attribute__((used, externally_visible))
void
diag_ring_push(uint8_t tag, uint32_t a, uint32_t b)
{
    extern uint32_t timer_read_time(void);
    if (diag_ring_frozen)
        return;
    irqstatus_t flag = irq_save();
    uint32_t head = diag.ring_head & DIAG_RING_MASK;
    uint32_t next = (head + 1) & DIAG_RING_MASK;
    diag_ring[head].tag = tag;
    diag_ring[head]._pad0 = 0;
    diag_ring[head].seq = (uint16_t)(diag.ring_seq & 0xFFFF);
    diag_ring[head].timestamp = timer_read_time();
    diag_ring[head].a = a;
    diag_ring[head].b = b;
    diag.ring_head = next;
    diag.ring_seq++;
    if (diag.ring_seq > DIAG_RING_LEN
        && (diag.ring_seq - DIAG_RING_LEN) > diag.ring_overflow)
        diag.ring_overflow = diag.ring_seq - DIAG_RING_LEN;
    if (tag == DIAG_EV_RUST_FAULT)
        diag_ring_frozen = 1;
    diag_cache_clean();
    irq_restore(flag);
}

void
diag_task_heartbeat(volatile uint32_t *calls,
                    volatile uint32_t *last_tick,
                    volatile uint32_t *max_gap,
                    uint32_t threshold_ticks,
                    uint8_t event_tag)
{
    extern uint32_t timer_read_time(void);
    uint32_t now = timer_read_time();
    uint32_t prev = *last_tick;
    *calls = *calls + 1;
    *last_tick = now;
    if (prev != 0) {
        uint32_t gap = now - prev;
        if (gap > *max_gap)
            *max_gap = gap;
        if (event_tag && gap > threshold_ticks)
            diag_ring_push(event_tag, gap, prev);
    }
}

void
diag_note_usb_in_busy(void)
{
    diag.usb_in_busy_n++;
}

void
diag_note_dispatch(uint32_t func, uint32_t addr)
{
    live_snap.last_dispatch_func = func;
    live_snap.last_dispatch_addr = addr;
}

static void
diag_close_task(uint32_t now)
{
    if (live_snap.cur_task_func)
        diag_update_worst(&live_snap.worst_task_cyc, &live_snap.worst_task_func,
                          now - live_snap.cur_task_start, live_snap.cur_task_func);
}

// Called by mcu_demux_pump around each dispatched command/frame. kind is a
// nonzero tag (0 = no message in progress): 0x100|channel for a kalico frame,
// 0x200|cmd for a Klipper command, 0x300 for a demux error. Pairs with the
// per-task timing to split "one slow command" (worst_msg ~ worst_task) from
// "backlog of many" (worst_task >> worst_msg).
__attribute__((used, externally_visible))
void
diag_note_msg_enter(uint32_t kind, uint32_t head)
{
    uint32_t now = timer_read_time();
    if (live_snap.cur_msg_kind)
        diag_update_worst_msg(now - live_snap.cur_msg_start,
                              live_snap.cur_msg_kind, live_snap.cur_msg_head);
    live_snap.cur_msg_start = now;
    live_snap.cur_msg_kind = kind;
    live_snap.cur_msg_head = head;
}

__attribute__((used, externally_visible))
void
diag_note_msg_exit(void)
{
    if (live_snap.cur_msg_kind)
        diag_update_worst_msg(timer_read_time() - live_snap.cur_msg_start,
                              live_snap.cur_msg_kind, live_snap.cur_msg_head);
    live_snap.cur_msg_kind = 0;
}

// Called by sched_add_timer just before the "Timer too close" try_shutdown;
// latches the first offender (caller PC, timer callback, lateness) into the
// reset-surviving snapshot so the crash replay can name the timer.
__attribute__((used, externally_visible))
void
diag_note_timer_too_close(uint32_t caller, uint32_t func, uint32_t late)
{
    if (!live_snap.ttc_count) {
        live_snap.ttc_caller = caller;
        live_snap.ttc_func   = func;
        live_snap.ttc_late   = late;
    }
    live_snap.ttc_count++;
    diag_cache_clean();
}

// Called by sched_main after a shutdown longjmp: the aborted work never
// reached diag_note_msg_exit, so an in-progress message/task marker would let
// the TIM5 growing-duration monitor inflate the worst slots forever.
__attribute__((used, externally_visible))
void
diag_note_shutdown_reset(void)
{
    live_snap.cur_msg_kind = 0;
    live_snap.cur_task_func = 0;
}

__attribute__((used, externally_visible))
void
diag_note_demux(uint32_t backlog, uint32_t msgs)
{
    if (backlog > live_snap.demux_backlog_max)
        live_snap.demux_backlog_max = backlog;
    if (msgs > live_snap.demux_msgs_max)
        live_snap.demux_msgs_max = msgs;
}

// Called by the generated ctr_run_taskfuncs before each DECL_TASK. Times the
// previous task and publishes the one about to run, so a foreground stall names
// the offending task. start is stored before func: the TIM5-ISR monitor reads
// func first, so seeing a published func guarantees a matching start.
__attribute__((used, externally_visible))
void
diag_note_task_enter(uint32_t func)
{
    uint32_t now = timer_read_time();
    diag_close_task(now);
    live_snap.cur_task_start = now;
    live_snap.cur_task_func = func;
}

__attribute__((used, externally_visible))
void
diag_note_task_loop_end(void)
{
    diag_close_task(timer_read_time());
    live_snap.cur_task_func = 0;
}

void
diag_record_tx_drop_kalico(uint32_t len, uint32_t tpos)
{
    diag.tx_drops_kalico++;
    diag.tx_drops_transport_last_len = len;
    diag_ring_push(DIAG_EV_TX_DROP_KAL, len, tpos);
}

void
diag_record_tx_drop_klipper(uint32_t max_size, uint32_t tpos)
{
    diag.tx_drops_klipper++;
    diag.tx_drops_klipper_last_max = max_size;
    diag_ring_push(DIAG_EV_TX_DROP_KLP, max_size, tpos);
}

void
diag_record_engine_xition(uint8_t prev, uint8_t cur, uint32_t samples_taken)
{
    diag_ring_push(DIAG_EV_ENGINE_XITION,
                   ((uint32_t)prev << 8) | (uint32_t)cur,
                   samples_taken);
}
