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
    uint32_t move_interval = m->interval;
    uint_fast16_t move_count = m->count;
    int_fast16_t move_add = m->add;
    uint_fast8_t need_dir_change = m->flags & MF_DIR;
    move_free(m);

    // Add all steps to s->position (stepper_get_position() can calc mid-move)
    s->position = (need_dir_change ? -s->position : s->position) + move_count;

    // Load next move into 'struct stepper'
    s->add = move_add;
    s->interval = move_interval + move_add;
    if (HAVE_OPTIMIZED_PATH && s->flags & SF_OPTIMIZED_PATH) {
        // Using optimized stepper_event_edge() or stepper_event_avr()
        s->time.waketime += move_interval;
        if (HAVE_AVR_OPTIMIZATION)
            s->flags = (move_add ? s->flags | SF_HAVE_ADD
                        : s->flags & ~SF_HAVE_ADD);
        s->count = move_count;
    } else {
        // Using fully scheduled stepper_event_full() code (the scheduler
        // may be called twice for each step)
        uint_fast8_t was_active = !!s->count;
        uint32_t min_next_time = s->time.waketime;
        s->next_step_time += move_interval;
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
            // Must ensure minimum time between step change and dir change
            if (s->flags & SF_SINGLE_SCHED)
                while (timer_is_before(timer_read_time(), min_next_time))
                    ;
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
        s->count = count;
        s->time.waketime += s->interval;
        s->interval += s->add;
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
        s->next_step_time += s->interval;
        s->interval += s->add;
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
                           EVENT_LOG_EVENT_MOTION_STEP_REARM_LATE, 0,
                           (uint32_t)margin, s->time.waketime);
        sched_add_timer(&s->time);
    }
    irq_enable();
}
DECL_COMMAND(command_queue_step,
             "queue_step oid=%c interval=%u count=%hu add=%hi");

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
        irq_disable();
        struct move_node *mn = move_queue_first(&s->completed_barriers);
        move_queue_clear(&s->completed_barriers);
        irq_enable();
        while (mn) {
            struct stepper_move *m = container_of(mn, struct stepper_move,
                                                  node);
            mn = mn->next;
            uint32_t seq = m->interval;
            irq_disable();
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
