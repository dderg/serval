// Basic scheduling functions and startup/shutdown code.
//
// Copyright (C) 2016-2024  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <setjmp.h> // setjmp
#include "autoconf.h" // CONFIG_*
#include "basecmd.h" // stats_update
#include "board/io.h" // readb
#include "board/irq.h" // irq_save
#include "compiler.h" // noinline
#include "board/misc.h" // timer_from_us
#include "board/pgm.h" // READP
#include "command.h" // shutdown
#if CONFIG_INLINE_STEPPER_HACK
#include "stepper.h" // stepper_event
#endif
#include "sched.h" // sched_check_periodic

static uint_fast8_t periodic_event(struct timer *t);
static uint_fast8_t sentinel_event(struct timer *t);
static uint_fast8_t deleted_event(struct timer *t);

extern uint_fast8_t timer_wrap_event(struct timer *t);

static struct sched_protected_state {
    struct timer periodic_timer;
    struct timer sentinel_timer;
    struct timer deleted_timer;
    struct timer wrap_timer;
    struct timer *timer_list;
    struct timer *last_insert;
} SchedState SCHED_PROTECTED = {
    .periodic_timer = { .func = periodic_event,
                        .next = &SchedState.sentinel_timer },
    .sentinel_timer = { .func = sentinel_event,
                        .waketime = 0x80000000 },
    .deleted_timer  = { .func = deleted_event },
    .wrap_timer     = { .func = timer_wrap_event,
                        .waketime = 0xffffff },
    .timer_list  = &SchedState.periodic_timer,
    .last_insert = &SchedState.periodic_timer,
};

static struct {
    int8_t tasks_status, tasks_busy;
    uint8_t shutdown_status, shutdown_reason;
} SchedFlags;

struct timer *
sched_get_wrap_timer(void)
{
    return &SchedState.wrap_timer;
}


/****************************************************************
 * Timers
 ****************************************************************/

// The periodic_timer simplifies the timer code by ensuring there is
// always a timer on the timer list and that there is always a timer
// not far in the future.
static uint_fast8_t
periodic_event(struct timer *t)
{
    // Make sure the stats task runs periodically
    sched_wake_tasks();
    SchedState.periodic_timer.waketime += timer_from_us(100000);
    SchedState.sentinel_timer.waketime =
        SchedState.periodic_timer.waketime + 0x80000000;
    return SF_RESCHEDULE;
}

// The sentinel timer is always the last timer on timer_list - its
// presence allows the code to avoid checking for NULL while
// traversing timer_list.  Since sentinel_timer.waketime is always
// equal to (periodic_timer.waketime + 0x80000000) any added timer
// must always have a waketime less than one of these two timers.
static uint_fast8_t
sentinel_event(struct timer *t)
{
    shutdown("sentinel timer called");
}

#define SCHED_INSERT_MAX_WALK 1024u

volatile uint32_t sched_insert_corrupt_pos;
volatile uint32_t sched_insert_corrupt_count;

// Find position for a timer in timer_list and insert it
static void __always_inline
insert_timer(struct timer *pos, struct timer *t, uint32_t waketime)
{
    struct timer *prev;
    uint32_t walk = 0;
    for (;;) {
        prev = pos;
        pos = pos->next;
        if (timer_is_before(waketime, pos->waketime))
            break;
        // Fail loud, do NOT spin: a runaway walk means the timer list is
        // corrupt. Spinning here runs under the SysTick irq_disable, masking
        // TIM5 and starving the IWDG pet -> ~0.5s freeze -> watchdog reset ->
        // USB-CDC drop -> klippy abort (the silent failure we chased). Shut
        // down cleanly instead so the cause is named and the host stays up.
        if (unlikely(++walk > SCHED_INSERT_MAX_WALK)) {
            sched_insert_corrupt_pos = (uint32_t)pos;
            sched_insert_corrupt_count++;
            sched_writable_end();
            try_shutdown("insert_timer: scheduler timer list corrupt");
        }
    }
    t->next = pos;
    prev->next = t;
}

void
sched_walk_chain(uint32_t addrs[SCHED_CHAIN_WALK_N],
                 uint32_t nexts[SCHED_CHAIN_WALK_N],
                 uint32_t *steps)
{
    for (uint32_t i = 0; i < SCHED_CHAIN_WALK_N; i++) {
        addrs[i] = 0;
        nexts[i] = 0;
    }
    *steps = 0;
    struct timer *pos = &SchedState.periodic_timer;
    for (uint32_t i = 0; i < SCHED_CHAIN_WALK_N; i++) {
        addrs[i] = (uint32_t)pos;
        struct timer *nx = pos->next;
        nexts[i] = (uint32_t)nx;
        *steps = i + 1;
        if (nx == &SchedState.sentinel_timer)
            return;
        uint32_t nu = (uint32_t)nx;
        if (nu == 0 || (nu & 3u) != 0u
            || nu < 0x20000010u || nu >= 0x20020000u)
            return;
        pos = nx;
    }
}

volatile uint32_t sched_bad_add_caller __attribute__((used));
volatile uint32_t sched_bad_add_value  __attribute__((used));

static inline int
addr_looks_bogus_for_timer(uint32_t p)
{
    (void)p;
    return 0;
}

volatile uint32_t sched_bad_add_stack0 __attribute__((used));
volatile uint32_t sched_bad_add_stack1 __attribute__((used));
volatile uint32_t sched_bad_add_stack2 __attribute__((used));
volatile uint32_t sched_bad_add_blocked_count __attribute__((used));

static void
sched_insert_timer(struct timer *add, uint32_t caller)
{
    if (addr_looks_bogus_for_timer((uint32_t)add)) {
        sched_bad_add_blocked_count++;
        if (!sched_bad_add_caller) {
            sched_bad_add_caller = caller;
            sched_bad_add_value  = (uint32_t)add;
            register uint32_t *sp asm("sp");
            sched_bad_add_stack0 = sp[0];
            sched_bad_add_stack1 = sp[1];
            sched_bad_add_stack2 = sp[2];
        }
        return;
    }
    uint32_t waketime = add->waketime;
    irqstatus_t flag = irq_save();
    sched_writable_begin();
    struct timer *tl = SchedState.timer_list;
    if (unlikely(timer_is_before(waketime, tl->waketime))) {
        // This timer is before all other scheduled timers
        uint32_t now = timer_read_time();
        if (timer_is_before(waketime, now)) {
            extern void diag_note_timer_too_close(uint32_t caller,
                                                  uint32_t func,
                                                  uint32_t late);
            diag_note_timer_too_close(caller,
                                      (uint32_t)(uintptr_t)add->func,
                                      now - waketime);
            try_shutdown("Timer too close");
        }
        if (tl == &SchedState.deleted_timer)
            add->next = SchedState.deleted_timer.next;
        else
            add->next = tl;
        SchedState.deleted_timer.waketime = waketime;
        SchedState.deleted_timer.next = add;
        SchedState.timer_list = &SchedState.deleted_timer;
        timer_kick();
    } else {
        insert_timer(tl, add, waketime);
    }
    sched_writable_end();
    irq_restore(flag);
}

// Schedule a function call at a supplied time.
void
sched_add_timer(struct timer *add)
{
    sched_insert_timer(add, (uint32_t)(uintptr_t)__builtin_return_address(0));
}

// The deleted timer is used when deleting an active timer.
static uint_fast8_t
deleted_event(struct timer *t)
{
    return SF_DONE;
}

// Remove a timer that may be live.
void
sched_del_timer(struct timer *del)
{
    irqstatus_t flag = irq_save();
    sched_writable_begin();
    if (SchedState.timer_list == del) {
        // Deleting the next active timer - replace with deleted_timer
        SchedState.deleted_timer.waketime = del->waketime;
        SchedState.deleted_timer.next = del->next;
        SchedState.timer_list = &SchedState.deleted_timer;
    } else {
        // Find and remove from timer list (if present)
        struct timer *pos;
        for (pos = SchedState.timer_list; pos->next; pos = pos->next) {
            if (pos->next == del) {
                pos->next = del->next;
                break;
            }
        }
    }
    if (SchedState.last_insert == del)
        SchedState.last_insert = &SchedState.periodic_timer;
    sched_writable_end();
    irq_restore(flag);
}

static volatile uint32_t sched_dispatch_history_addr[SCHED_DISPATCH_HISTORY_N];
static volatile uint32_t sched_dispatch_history_func[SCHED_DISPATCH_HISTORY_N];
static volatile uint32_t sched_dispatch_history_idx;

#define DISPATCH_HISTORY_MASK (SCHED_DISPATCH_HISTORY_N - 1)
_Static_assert((SCHED_DISPATCH_HISTORY_N & DISPATCH_HISTORY_MASK) == 0,
               "dispatch history slot is a mask of the running index");

void
sched_get_dispatch_history(uint32_t *idx,
                           uint32_t addrs[SCHED_DISPATCH_HISTORY_N],
                           uint32_t funcs[SCHED_DISPATCH_HISTORY_N])
{
    *idx = sched_dispatch_history_idx;
    for (int i = 0; i < SCHED_DISPATCH_HISTORY_N; i++) {
        addrs[i] = sched_dispatch_history_addr[i];
        funcs[i] = sched_dispatch_history_func[i];
    }
}

// `used, externally_visible`: Rust-only caller; attribute prevents --gc-sections
// from dropping the section before the Rust archive reference resolves.
__attribute__((used, externally_visible))
uint32_t
sched_last_dispatched_func(void)
{
    uint32_t idx = sched_dispatch_history_idx;
    if (idx == 0)
        return 0;
    return sched_dispatch_history_func[(idx - 1) & DISPATCH_HISTORY_MASK];
}

extern void diag_note_timer_duration(uint32_t dur_cyc, uint32_t func);

static __always_inline unsigned int
reschedule_head(struct timer *t, uint_fast8_t res, struct timer *next,
                uint32_t next_waketime)
{
    if (unlikely(res == SF_DONE)) {
        SchedState.timer_list = next;
        if (SchedState.last_insert == t)
            SchedState.last_insert = next;
        return next_waketime;
    }
    uint32_t updated_waketime = t->waketime;
    if (timer_is_before(updated_waketime, next_waketime))
        return updated_waketime;
    SchedState.timer_list = next;
    struct timer *pos = SchedState.last_insert;
    if (timer_is_before(updated_waketime, pos->waketime))
        pos = next;
    insert_timer(pos, t, updated_waketime);
    SchedState.last_insert = t;
    return next_waketime;
}

#if CONFIG_INLINE_STEPPER_HACK
#if CONFIG_MCU_SIM
#define STEP_COINCIDENCE_TICKS 0
#else
#define STEP_COINCIDENCE_TICKS (CONFIG_CLOCK_FREQ / 1000000)
#endif

static __always_inline uint_fast8_t
timer_is_step(struct timer *t)
{
    return !t->func || t->func == stepper_event_full;
}

static __always_inline uint_fast8_t
step_joins_batch(struct timer *t, uint32_t waketime, uint32_t horizon)
{
    return !timer_is_before(horizon, waketime) && timer_is_step(t);
}

static __always_inline struct timer *
fire_follower_step_edges(struct timer *t, uint32_t scheduled, uint32_t horizon)
{
    int32_t leader_lag = timer_read_time() - scheduled;
    if (leader_lag < 0)
        shutdown("Step timer dispatched before its waketime");
    struct timer *last;
    do {
        uint32_t due = t->waketime + leader_lag;
        while (timer_is_before(timer_read_time(), due))
            ;
        stepper_step_edge(t, stepper_timer_on_edge_path(t));
        last = t;
        t = t->next;
    } while (step_joins_batch(t, t->waketime, horizon));
    return last;
}

static void
reinsert_detached_timer(struct timer *t)
{
    uint32_t waketime = t->waketime;
    struct timer *head = SchedState.timer_list;
    if (timer_is_before(waketime, head->waketime)) {
        t->next = head;
        SchedState.timer_list = t;
        return;
    }
    struct timer *pos = SchedState.last_insert;
    if (timer_is_before(waketime, pos->waketime))
        pos = head;
    insert_timer(pos, t, waketime);
    SchedState.last_insert = t;
}

static noinline void
dispatch_coincident_steps(struct timer *leader, uint32_t scheduled,
                          uint32_t horizon)
{
    struct timer *last = fire_follower_step_edges(leader->next, scheduled,
                                                  horizon);
    struct timer *rest = last->next;
    SchedState.timer_list = rest;
    SchedState.last_insert = rest;
    struct timer *t = leader;
    for (;;) {
        struct timer *next = t->next;
        if (stepper_step_finish(t, stepper_timer_on_edge_path(t))
            == SF_RESCHEDULE)
            reinsert_detached_timer(t);
        if (t == last)
            return;
        t = next;
    }
}

static __always_inline unsigned int
dispatch_step_timer(struct timer *t, uint_fast8_t on_edge_path)
{
    uint32_t scheduled = t->waketime;
    stepper_step_edge(t, on_edge_path);
    struct timer *next = t->next;
    uint32_t next_waketime = next->waketime;
    uint32_t horizon = scheduled + STEP_COINCIDENCE_TICKS;
    if (unlikely(step_joins_batch(next, next_waketime, horizon))) {
        dispatch_coincident_steps(t, scheduled, horizon);
        return SchedState.timer_list->waketime;
    }
    uint_fast8_t res = stepper_step_finish(t, on_edge_path);
    return reschedule_head(t, res, next, next_waketime);
}
#endif

static volatile uint32_t callback_busy_ticks;

static noinline unsigned int
dispatch_callback_timer(struct timer *t)
{
    uint32_t start = timer_read_time();
    uint_fast8_t res = t->func(t);
    unsigned int next_waketime = reschedule_head(t, res, t->next,
                                                 t->next->waketime);
    callback_busy_ticks += timer_read_time() - start;
    return next_waketime;
}

uint32_t
sched_callback_busy_ticks(void)
{
    return callback_busy_ticks;
}

// Invoke the next timer - called from board hardware irq code.
unsigned int
sched_timer_dispatch(void)
{
    struct timer *t = SchedState.timer_list;
    uint32_t hidx = sched_dispatch_history_idx;
    uint32_t slot = hidx & DISPATCH_HISTORY_MASK;
    sched_dispatch_history_addr[slot] = (uint32_t)t;
    sched_dispatch_history_func[slot] = (uint32_t)t->func;
    sched_dispatch_history_idx = hidx + 1;
    extern void diag_note_dispatch(uint32_t func, uint32_t addr);
    diag_note_dispatch((uint32_t)t->func, (uint32_t)t);

    uint32_t diag_func = (uint32_t)(uintptr_t)t->func;
#if CONFIG_INLINE_STEPPER_HACK
    if (!diag_func)
        diag_func = (uint32_t)(uintptr_t)&stepper_event_full;
#endif
    uint32_t diag_start = timer_read_time();
    unsigned int next_waketime;
#if CONFIG_INLINE_STEPPER_HACK
    if (likely(timer_is_step(t))) {
        if (stepper_timer_on_edge_path(t))
            next_waketime = dispatch_step_timer(t, 1);
        else
            next_waketime = dispatch_step_timer(t, 0);
    } else {
        next_waketime = dispatch_callback_timer(t);
    }
#else
    next_waketime = dispatch_callback_timer(t);
#endif
    diag_note_timer_duration(timer_read_time() - diag_start, diag_func);
    return next_waketime;
}

struct timer *
sched_peek_timer(void)
{
    return SchedState.timer_list;
}

// Remove all user timers
void
sched_timer_reset(void)
{
    sched_writable_begin();
    SchedState.timer_list = &SchedState.deleted_timer;
    SchedState.deleted_timer.waketime = SchedState.periodic_timer.waketime;
    SchedState.deleted_timer.next = SchedState.last_insert
        = &SchedState.periodic_timer;
    SchedState.periodic_timer.next = &SchedState.sentinel_timer;
    sched_writable_end();
    timer_kick();
}


/****************************************************************
 * Tasks
 ****************************************************************/

#define TS_IDLE      -1
#define TS_REQUESTED 0
#define TS_RUNNING   1

// Note that at least one task is ready to run
void
sched_wake_tasks(void)
{
    SchedFlags.tasks_status = TS_REQUESTED;
}

// Check if tasks busy (called from low-level timer dispatch code)
uint8_t
sched_check_set_tasks_busy(void)
{
    // Return busy if tasks never idle between two consecutive calls
    if (SchedFlags.tasks_busy >= TS_REQUESTED)
        return 1;
    SchedFlags.tasks_busy = SchedFlags.tasks_status;
    return 0;
}

// Note that a task is ready to run
void
sched_wake_task(struct task_wake *w)
{
    sched_wake_tasks();
    writeb(&w->wake, 1);
}

// Check if a task is ready to run (as indicated by sched_wake_task)
uint8_t
sched_check_wake(struct task_wake *w)
{
    if (!readb(&w->wake))
        return 0;
    writeb(&w->wake, 0);
    return 1;
}

// Main task dispatch loop
static void
run_tasks(void)
{
    uint32_t start = timer_read_time();
    for (;;) {
        // Check if can sleep
        irq_poll();
        if (SchedFlags.tasks_status != TS_REQUESTED) {
            start -= timer_read_time();
            irq_disable();
            if (SchedFlags.tasks_status != TS_REQUESTED) {
                // Sleep processor (only run timers) until tasks woken
                SchedFlags.tasks_status = SchedFlags.tasks_busy = TS_IDLE;
                do {
                    irq_wait();
                } while (SchedFlags.tasks_status != TS_REQUESTED);
            }
            irq_enable();
            start += timer_read_time();
        }
        SchedFlags.tasks_status = TS_RUNNING;

        // Run all tasks
        extern void ctr_run_taskfuncs(void);
        ctr_run_taskfuncs();

        // Update statistics
        uint32_t cur = timer_read_time();
        stats_update(start, cur);
        start = cur;
    }
}


/****************************************************************
 * Shutdown processing
 ****************************************************************/

// Return true if the machine is in an emergency stop state
uint8_t
sched_is_shutdown(void)
{
    return !!SchedFlags.shutdown_status;
}

// Transition out of shutdown state
void
sched_clear_shutdown(void)
{
    if (!SchedFlags.shutdown_status)
        shutdown("Shutdown cleared when not shutdown");
    if (SchedFlags.shutdown_status == 2)
        // Ignore attempt to clear shutdown if still processing shutdown
        return;
    SchedFlags.shutdown_status = 0;
}

// Invoke all shutdown functions (as declared by DECL_SHUTDOWN)
static void
run_shutdown(int reason)
{
    irq_disable();
    uint32_t cur = timer_read_time();
    if (!SchedFlags.shutdown_status)
        SchedFlags.shutdown_reason = reason;
    SchedFlags.shutdown_status = 2;
    sched_timer_reset();
    extern void ctr_run_shutdownfuncs(void);
    ctr_run_shutdownfuncs();
    SchedFlags.shutdown_status = 1;
    irq_enable();

    sendf("shutdown clock=%u static_string_id=%hu", cur
          , SchedFlags.shutdown_reason);
}

// Report the last shutdown reason code
void
sched_report_shutdown(void)
{
    sendf("is_shutdown static_string_id=%hu", SchedFlags.shutdown_reason);
}

// Shutdown the machine if not already in the process of shutting down
void __always_inline
sched_try_shutdown(uint_fast8_t reason)
{
    if (!SchedFlags.shutdown_status)
        sched_shutdown(reason);
}

static jmp_buf shutdown_jmp;

// Force the machine to immediately run the shutdown handlers
void
sched_shutdown(uint_fast8_t reason)
{
    irq_disable();
    longjmp(shutdown_jmp, reason);
}


/****************************************************************
 * Startup
 ****************************************************************/

// Main loop of program
void
sched_main(void)
{
    extern void ctr_run_initfuncs(void);
    ctr_run_initfuncs();

    sendf("starting");

    irq_disable();
    int ret = setjmp(shutdown_jmp);
    if (ret) {
        // try_shutdown can longjmp from inside a sched_writable_begin()
        // window (e.g. sched_add_timer's "Timer too close" check), so the
        // matching end() is skipped and the depth counter is left
        // non-zero. Reset before running shutdown handlers so the
        // protection re-engages cleanly.
        sched_writable_reset();
        // The longjmp also skips any pending diag_note_msg_exit; clear the
        // in-progress markers so the TIM5 growing-duration monitor can't
        // keep inflating worst_msg_cyc/worst_task_cyc across the shutdown.
        extern void diag_note_shutdown_reset(void);
        diag_note_shutdown_reset();
        run_shutdown(ret);
    }
    irq_enable();

    run_tasks();
}
