// Classic host-scheduled stepper step execution.
//
// Copyright (C) 2016-2025  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "autoconf.h" // CONFIG_*
#include "basecmd.h" // move_alloc
#include "board/gpio.h" // gpio_out_toggle_noirq
#include "board/irq.h" // irq_disable
#include "board/misc.h"
#include "event_log.h" // event_log_emit
#include "command.h" // DECL_COMMAND
#include "compiler.h" // likely
#include "sched.h" // sched_add_timer
#include "stepper.h" // struct stepper

DECL_CONSTANT("STEPPER_STEP_BOTH_EDGE", 1);

#if HAVE_EDGE_OPTIMIZATION
 DECL_CONSTANT("STEPPER_OPTIMIZED_EDGE", EDGE_STEP_TICKS);
#endif
#if HAVE_AVR_OPTIMIZATION
 DECL_CONSTANT("STEPPER_OPTIMIZED_UNSTEP", AVR_STEP_TICKS);
#endif

static struct task_wake barrier_ack_wake;

#if CONFIG_HIGH_PREC_STEP
static inline void
add_interval(uint32_t *time, struct stepper *s)
{
    uint32_t interval = s->interval + s->int_low_acc;
    *time += interval >> s->shift;
    s->int_low_acc = interval - ((interval >> s->shift) << s->shift);
}

static inline void
inc_interval(struct stepper *s)
{
    s->interval += s->add;
    s->add += s->add2;
}
#endif

static uint_fast8_t
stepper_next_is_barrier(struct stepper *s)
{
    if (move_queue_empty(&s->mq))
        return 0;
    struct move_node *mn = move_queue_first(&s->mq);
    struct stepper_move *m = container_of(mn, struct stepper_move, node);
    return m->flags & MF_BARRIER;
}

static uint_fast8_t
stepper_load_next(struct stepper *s)
{
    while (stepper_next_is_barrier(s)) {
        struct move_node *mn = move_queue_pop(&s->mq);
        move_queue_push(mn, &s->completed_barriers);
        sched_wake_task(&barrier_ack_wake);
    }
    if (move_queue_empty(&s->mq)) {
        s->count = 0;
        return SF_DONE;
    }

    struct move_node *mn = move_queue_pop(&s->mq);
    struct stepper_move *m = container_of(mn, struct stepper_move, node);
    uint32_t move_first_interval = m->interval;
#if CONFIG_HIGH_PREC_STEP
    uint_fast8_t high_precision = m->flags & SF_HIGH_PREC_STEP;
    uint32_t move_interval = high_precision ? m->next_interval : m->interval;
    int32_t move_add = m->add;
    int32_t move_add2;
    uint_fast8_t move_shift;
    uint16_t move_int_low_acc;
    if (high_precision) {
        move_add2 = m->add2;
        move_shift = m->shift;
        move_int_low_acc = m->int_low_acc;
    }
#else
    uint32_t move_interval = m->interval;
    int_fast16_t move_add = m->add;
#endif
    uint_fast16_t move_count = m->count;
    uint_fast8_t need_dir_change = m->flags & MF_DIR;
    move_free(m);

    // Add all steps to s->position (stepper_get_position() can calc mid-move)
    s->position = (need_dir_change ? -s->position : s->position) + move_count;

    // Load next move into 'struct stepper'
    s->add = move_add;
#if CONFIG_HIGH_PREC_STEP
    if (high_precision) {
        s->add2 = move_add2;
        s->shift = move_shift;
        s->int_low_acc = move_int_low_acc;
        s->interval = move_interval;
        s->flags |= SF_HIGH_PREC_STEP;
    } else {
        s->interval = move_interval + move_add;
        s->flags &= ~SF_HIGH_PREC_STEP;
    }
#else
    s->interval = move_interval + move_add;
#endif
    if (HAVE_OPTIMIZED_PATH && s->flags & SF_OPTIMIZED_PATH) {
        // Using optimized stepper_event_edge() or stepper_event_avr()
        s->time.waketime += move_first_interval;
        if (HAVE_AVR_OPTIMIZATION)
            s->flags = (move_add ? s->flags | SF_HAVE_ADD
                        : s->flags & ~SF_HAVE_ADD);
        s->count = move_count;
    } else {
        // Using fully scheduled stepper_event_full() code (the scheduler
        // may be called twice for each step)
        uint_fast8_t was_active = !!s->count;
        uint32_t min_next_time = s->time.waketime;
        s->next_step_time += move_first_interval;
        s->time.waketime = s->next_step_time;
        s->count = (s->flags & SF_SINGLE_SCHED ? move_count
                    : (uint32_t)move_count * 2);
        if (was_active && timer_is_before(s->next_step_time, min_next_time)) {
            // Actively stepping and next step event close to the last unstep
            int32_t diff = s->next_step_time - min_next_time;
            event_log_emit(EVENT_LOG_LEVEL_WARN, EVENT_LOG_SUBSYS_MOTION,
                           EVENT_LOG_EVENT_MOTION_STEP_LOAD_LATE, 0,
                           (uint32_t)diff, min_next_time);
            if (diff < (int32_t)-timer_from_us(1000))
                shutdown("Stepper too far in past");
            s->time.waketime = min_next_time;
        }
        if (was_active && need_dir_change) {
            // Must ensure minimum time between step change and dir change.
            // The settle window is measured from the fresh clock, never from
            // min_next_time: that waketime is the reset-step-clock anchor
            // when a halted stepper re-arms mid-volley, and a stale-future
            // anchor would hold the ISR until the clock caught it (the
            // session-stable ~144 ms "Rescheduled timer in the past" at the
            // post-trip retract). Bounded to step_pulse_ticks by construction.
            if (s->flags & SF_SINGLE_SCHED) {
                uint32_t now = timer_read_time();
                uint32_t spin_until = now + s->step_pulse_ticks;
                uint32_t stale_ahead = 0;
                if (timer_is_before(spin_until, min_next_time))
                    stale_ahead = min_next_time - spin_until;
                while (timer_is_before(timer_read_time(), spin_until))
                    ;
                extern void diag_note_step_spin(uint32_t dur_cyc,
                                                uint32_t stale_ahead);
                diag_note_step_spin(timer_read_time() - now, stale_ahead);
            }
            gpio_out_toggle_noirq(s->dir_pin);
#if CONFIG_MCU_SIM
            // The sim's virtual clock races arbitrarily far ahead of a
            // scheduled event (see src/linux/timer.c), so re-arming the dir
            // settle off it catapults the stepper past the host's step chain
            // and makes every later load report a phantom "too far in past".
            // Settle against the pending unstep, as stepper_event_full() does.
            uint32_t dir_settle_from = min_next_time;
#else
            uint32_t dir_settle_from = timer_read_time();
#endif
            min_next_time = dir_settle_from + s->step_pulse_ticks;
            if (timer_is_before(s->time.waketime, min_next_time))
                s->time.waketime = min_next_time;
            return SF_RESCHEDULE;
        }
    }

    // Set new direction (if needed)
    if (need_dir_change)
        gpio_out_toggle_noirq(s->dir_pin);
    return SF_RESCHEDULE;
}

// Optimized step function to step on each step pin edge
static uint_fast8_t
stepper_event_edge(struct timer *t)
{
    struct stepper *s = container_of(t, struct stepper, time);
    gpio_out_toggle_noirq(s->step_pin);
    uint32_t count = s->count - 1;
    if (likely(count)) {
#if CONFIG_HIGH_PREC_STEP
        if (s->flags & SF_HIGH_PREC_STEP) {
            add_interval(&s->time.waketime, s);
            inc_interval(s);
        } else {
            s->time.waketime += s->interval;
            s->interval += s->add;
        }
#else
        s->time.waketime += s->interval;
        s->interval += s->add;
#endif
        return SF_RESCHEDULE;
    }
    return stepper_load_next(s);
}

// AVR optimized step function
static uint_fast8_t
stepper_event_avr(struct timer *t)
{
    struct stepper *s = container_of(t, struct stepper, time);
    gpio_out_toggle_noirq(s->step_pin);
    uint16_t *pcount = (void*)&s->count, count = *pcount - 1;
    if (likely(count)) {
        *pcount = count;
        s->time.waketime += s->interval;
        gpio_out_toggle_noirq(s->step_pin);
        if (s->flags & SF_HAVE_ADD)
            s->interval += s->add;
        return SF_RESCHEDULE;
    }
    if (stepper_next_is_barrier(s)) {
        gpio_out_toggle_noirq(s->step_pin);
        return stepper_load_next(s);
    }
    uint_fast8_t ret = stepper_load_next(s);
    gpio_out_toggle_noirq(s->step_pin);
    return ret;
}

// Regular "fully scheduled" step function
uint_fast8_t
stepper_event_full(struct timer *t)
{
    struct stepper *s = container_of(t, struct stepper, time);
    gpio_out_toggle_noirq(s->step_pin);
#if CONFIG_MCU_SIM
    uint32_t min_next_time = s->time.waketime + s->step_pulse_ticks;
#else
    uint32_t curtime = timer_read_time();
    uint32_t min_next_time = curtime + s->step_pulse_ticks;
#endif
    uint32_t count = s->count - 1;
    if (likely(count & 1 && !(s->flags & SF_SINGLE_SCHED)))
        // Schedule unstep event
        goto reschedule_min;
    if (likely(count)) {
#if CONFIG_HIGH_PREC_STEP
        if (s->flags & SF_HIGH_PREC_STEP) {
            add_interval(&s->next_step_time, s);
            inc_interval(s);
        } else {
            s->next_step_time += s->interval;
            s->interval += s->add;
        }
#else
        s->next_step_time += s->interval;
        s->interval += s->add;
#endif
        if (unlikely(timer_is_before(s->next_step_time, min_next_time)))
            // The next step event is too close - push it back
            goto reschedule_min;
        s->count = count;
        s->time.waketime = s->next_step_time;
        return SF_RESCHEDULE;
    }
    s->time.waketime = min_next_time;
    return stepper_load_next(s);
reschedule_min:
    s->count = count;
    s->time.waketime = min_next_time;
    return SF_RESCHEDULE;
}

// Optimized entry point for step function (may be inlined into sched.c code)
uint_fast8_t
stepper_event(struct timer *t)
{
    if (HAVE_EDGE_OPTIMIZATION)
        return stepper_event_edge(t);
    if (HAVE_AVR_OPTIMIZATION)
        return stepper_event_avr(t);
    return stepper_event_full(t);
}

// Record late idle re-arms before the scheduler applies its canonical
// near-time policy.
static void
enqueue_move(struct stepper *s, struct stepper_move *m, uint8_t oid)
{
    irq_disable();
    uint8_t flags = s->flags;
    if (!!(flags & SF_LAST_DIR) != !!(flags & SF_NEXT_DIR)) {
        flags ^= SF_LAST_DIR;
        m->flags |= MF_DIR;
    }
    if (s->count) {
        s->flags = flags;
        move_queue_push(&m->node, &s->mq);
    } else if (flags & SF_NEED_RESET) {
        move_free(m);
    } else {
        s->flags = flags;
        move_queue_push(&m->node, &s->mq);
        stepper_load_next(s);
        extern void diag_note_step_rearm(int32_t margin);
        int32_t margin = (int32_t)(s->time.waketime - timer_read_time());
        diag_note_step_rearm(margin);
        if (unlikely(margin < 0))
            event_log_emit(EVENT_LOG_LEVEL_WARN, EVENT_LOG_SUBSYS_MOTION,
                           EVENT_LOG_EVENT_MOTION_STEP_REARM_LATE, oid,
                           (uint32_t)margin, s->time.waketime);
        sched_add_timer(&s->time);
    }
    irq_enable();
}

// Schedule a set of steps with a given timing
void
command_queue_step(uint32_t *args)
{
    struct stepper *s = stepper_oid_lookup(args[0]);
    struct stepper_move *m = move_alloc();
    m->interval = args[1];
    m->count = args[2];
    if (!m->count)
        shutdown("Invalid count parameter");
    m->add = args[3];
    m->flags = 0;

    enqueue_move(s, m, args[0]);
}
DECL_COMMAND(command_queue_step,
             "queue_step oid=%c interval=%u count=%hu add=%hi");
#if CONFIG_HIGH_PREC_STEP
void
command_queue_step_hp(uint32_t *args)
{
    struct stepper *s = stepper_oid_lookup(args[0]);
    struct stepper_move *m = move_alloc();
    m->count = args[2];
    if (!m->count || m->count >= 0x8000)
        shutdown("Invalid count parameter");
    uint32_t interval = args[1];
    int32_t add = args[3];
    int32_t add2 = args[4];
    int8_t shift = args[5];
    if (shift <= 0) {
        interval <<= -shift;
        add = add >= 0 ? add << -shift : -(-add << -shift);
        add2 = add2 >= 0 ? add2 << -shift : -(-add2 << -shift);
        m->next_interval = interval + add;
        m->add = add + add2;
        m->add2 = add2;
        m->int_low_acc = 0;
        m->interval = interval;
        m->shift = 0;
    } else {
        m->next_interval = interval + add;
        m->add = add + add2;
        m->add2 = add2;
        m->int_low_acc = 1 << (shift - 1);
        interval += m->int_low_acc;
        m->interval = interval >> shift;
        m->int_low_acc = interval - ((interval >> shift) << shift);
        m->shift = shift;
    }
    m->flags = SF_HIGH_PREC_STEP;
    enqueue_move(s, m, args[0]);
}
DECL_COMMAND(command_queue_step_hp,
             "queue_step_hp oid=%c interval=%u count=%hu add=%hi "
             "add2=%hi shift=%hi");
#endif

void
command_stepcompress_barrier(uint32_t *args)
{
    struct stepper *s = stepper_oid_lookup(args[0]);
    struct stepper_move *m = move_alloc();
    m->interval = args[1];
    m->count = 0;
    m->add = 0;
    m->flags = MF_BARRIER;

    irq_disable();
    if (s->count) {
        move_queue_push(&m->node, &s->mq);
    } else {
        if (!move_queue_empty(&s->mq))
            shutdown("Stepper inactive with queued moves");
        move_queue_push(&m->node, &s->completed_barriers);
        sched_wake_task(&barrier_ack_wake);
    }
    irq_enable();
}
DECL_COMMAND(command_stepcompress_barrier,
             "stepcompress_barrier oid=%c seq=%u");

void
stepcompress_barrier_ack_task(void)
{
    if (!sched_check_wake(&barrier_ack_wake))
        return;
    uint8_t oid;
    struct stepper *s;
    foreach_oid(oid, s, command_config_stepper) {
        for (;;) {
            irq_disable();
            if (move_queue_empty(&s->completed_barriers)) {
                irq_enable();
                break;
            }
            struct move_node *mn = move_queue_pop(&s->completed_barriers);
            struct stepper_move *m = container_of(mn, struct stepper_move,
                                                  node);
            uint32_t seq = m->interval;
            move_free(m);
            irq_enable();
            sendf("stepcompress_barrier_ack oid=%c barrier_seq=%u", oid, seq);
        }
    }
}
DECL_TASK(stepcompress_barrier_ack_task);

// Set the direction of the next queued step
void
command_set_next_step_dir(uint32_t *args)
{
    struct stepper *s = stepper_oid_lookup(args[0]);
    uint8_t nextdir = args[1] ? SF_NEXT_DIR : 0;
    irq_disable();
    s->flags = (s->flags & ~SF_NEXT_DIR) | nextdir;
    irq_enable();
}
DECL_COMMAND(command_set_next_step_dir, "set_next_step_dir oid=%c dir=%c");

// Set an absolute time that the next step will be relative to
void
command_reset_step_clock(uint32_t *args)
{
    struct stepper *s = stepper_oid_lookup(args[0]);
    uint32_t waketime = args[1];
    irq_disable();
    if (s->count)
        shutdown("Can't reset time when stepper active");
    s->next_step_time = s->time.waketime = waketime;
    s->flags &= ~SF_NEED_RESET;
    irq_enable();
}
DECL_COMMAND(command_reset_step_clock, "reset_step_clock oid=%c clock=%u");

// Return the current stepper position.  Caller must disable irqs.
uint32_t
stepper_get_position(struct stepper *s)
{
    uint32_t position = s->position;
    // If stepper is mid-move, subtract out steps not yet taken
    if (s->flags & SF_SINGLE_SCHED)
        position -= s->count;
    else
        position -= s->count / 2;
    // The top bit of s->position is an optimized reverse direction flag
    if (position & 0x80000000)
        return -position;
    return position;
}

// Report the current position of the stepper
void
command_stepper_get_position(uint32_t *args)
{
    uint8_t oid = args[0];
    struct stepper *s = stepper_oid_lookup(oid);
    irq_disable();
    uint32_t position = stepper_get_position(s);
    irq_enable();
    sendf("stepper_position oid=%c pos=%i", oid, position - POSITION_BIAS);
}
DECL_COMMAND(command_stepper_get_position, "stepper_get_position oid=%c");

// Seed the absolute step counter so the host's reconcile can compare the
// mcu's executed position against its own step-stream bookkeeping.
//
// The top bit of s->position is the optimized reverse-direction flag: while
// it is set the counter is stored negated and load_next adds step counts
// downward. Seeding must re-encode into whichever flavour is live, or every
// later step lands with the sign flipped.
void
command_stepcompress_set_position(uint32_t *args)
{
    struct stepper *s = stepper_oid_lookup(args[0]);
    irq_disable();
    if (s->count)
        shutdown("Can't set position when stepper active");
    uint32_t position = args[1] + POSITION_BIAS;
    s->position = s->position & 0x80000000 ? -position : position;
    irq_enable();
}
DECL_COMMAND(command_stepcompress_set_position,
             "stepcompress_set_position oid=%c pos=%i");

// Abandon every queued step and park the outputs. Caller must disable irqs.
//
// Barriers are retirement receipts, not motion: the host blocks its own
// retirement bookkeeping until each one comes back, so a discarded barrier
// would wedge the host rather than merely lose steps. Discarding a barrier
// therefore completes it — the steps it fenced are gone either way.
void
stepper_classic_halt(struct stepper *s)
{
    sched_del_timer(&s->time);
    s->next_step_time = s->time.waketime = 0;
    s->position = -stepper_get_position(s);
    s->count = 0;
    // Drop the fixed-point step accumulator wholesale: a resumed stream
    // re-anchors with reset_step_clock and the first load overwrites every
    // field, but a stale add/shift/int_low_acc surviving the halt would let
    // a stray event between halt and re-anchor walk a dead step chain.
    s->interval = 0;
#if CONFIG_HIGH_PREC_STEP
    s->add = 0;
    s->add2 = 0;
    s->shift = 0;
    s->int_low_acc = 0;
#else
    s->add = 0;
#endif
    s->flags = (s->flags & (SF_OPTIMIZED_PATH | SF_SINGLE_SCHED))
        | SF_NEED_RESET;
    while (!move_queue_empty(&s->mq)) {
        struct move_node *mn = move_queue_pop(&s->mq);
        struct stepper_move *m = container_of(mn, struct stepper_move, node);
        if (m->flags & MF_BARRIER)
            move_queue_push(mn, &s->completed_barriers);
        else
            move_free(m);
    }
    sched_wake_task(&barrier_ack_wake);
    gpio_out_write(s->dir_pin, 0);
    gpio_out_write(s->step_pin, s->step_idle_level);
}

uint8_t
stepper_classic_halt_all(uint32_t *stream_end_clock)
{
    uint8_t oid, found = 0;
    struct stepper *s;
    foreach_oid(oid, s, command_config_stepper) {
        uint32_t waketime = s->time.waketime;
        if (waketime && !(s->flags & SF_NEED_RESET)) {
            uint32_t stream_end = s->count ? waketime - 1 : waketime;
            if (!found || timer_is_before(*stream_end_clock, stream_end))
                *stream_end_clock = stream_end;
            found = 1;
        }
        event_log_emit(EVENT_LOG_LEVEL_DEBUG, EVENT_LOG_SUBSYS_MOTION,
                       EVENT_LOG_EVENT_MOTION_STEP_HALT, oid,
                       s->flags, s->count);
        stepper_classic_halt(s);
    }
    return found;
}
