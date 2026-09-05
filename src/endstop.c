#include "autoconf.h"
#include "basecmd.h"
#include "board/gpio.h"
#include "board/irq.h"
#include "board/misc.h"
#include "command.h"
#include "sched.h"
#include "trsync.h"
#include "mcu_transport_dispatch.h"
#include "stepper.h"

#define ENDSTOP_UNBOUND 0xFF

extern uint64_t runtime_widened_host_clock(void);

struct endstop {
    struct timer time;
    uint32_t rest_ticks;
    uint32_t pin_id;
    uint32_t last_clear_clock;
    struct gpio_in pin;
    uint64_t trip_clock;
    struct trsync *ts;
    uint8_t trigger_reason;
    uint8_t endstop_id;
    uint8_t invert;
    uint8_t armed;
    uint8_t trip_pending;
    uint8_t trip_processing;
    uint8_t tripped;
    uint8_t motor;
    uint8_t stepper;
    uint8_t group;
};

static struct task_wake endstop_trip_wake;

static uint_fast8_t
endstop_event(struct timer *t)
{
    struct endstop *e = container_of(t, struct endstop, time);
    uint8_t raw = gpio_in_read(e->pin) ? 1 : 0;
    uint8_t active = raw ^ e->invert;
    uint32_t obs_clock = timer_read_time();
    if (active && e->armed) {
        uint64_t now64 = runtime_widened_host_clock();
        uint32_t gap = obs_clock - e->last_clear_clock;
        uint32_t mid32 = e->last_clear_clock + gap / 2;
        int32_t mid_delta = (int32_t)(mid32 - (uint32_t)now64);
        e->trip_clock = now64 + (int64_t)mid_delta;
        if (e->group && e->motor != ENDSTOP_UNBOUND) {
            uint8_t binding_count = runtime_motor_binding_count(e->motor);
            if (binding_count) {
                if (e->stepper >= binding_count)
                    shutdown("bad endstop binding");
                stepper_suppress_set(e->motor, e->stepper);
            }
            e->trip_clock = now64;
        }
        e->armed = 0;
        e->trip_pending = 1;
        e->tripped = 1;
        if (e->ts) {
            if (!e->group)
                classic_stop_gate_at(now64);
            trsync_do_trigger(e->ts, e->trigger_reason);
        }
        sched_wake_task(&endstop_trip_wake);
        return SF_DONE;
    }
    e->last_clear_clock = obs_clock;
    e->time.waketime += e->rest_ticks;
    return SF_RESCHEDULE;
}

void
command_config_endstop(uint32_t *args)
{
    struct endstop *e = oid_alloc(args[0], command_config_endstop, sizeof(*e));
    e->endstop_id = args[1];
    e->pin_id = args[2];
    e->pin = gpio_in_setup(args[2], args[3]);
    e->invert = args[4] ? 1 : 0;
    e->armed = 0;
    e->trip_pending = 0;
    e->trip_processing = 0;
    e->tripped = 0;
    e->trip_clock = 0;

    uint8_t motor = args[5];
    uint8_t stepper = args[6];
    uint8_t motor_unbound = motor == ENDSTOP_UNBOUND;
    uint8_t stepper_unbound = stepper == ENDSTOP_UNBOUND;
    if (motor_unbound != stepper_unbound)
        shutdown("bad endstop binding");
    if (!motor_unbound
        && (motor >= RUNTIME_MOTOR_COUNT
            || stepper >= RUNTIME_MAX_STEPPERS_PER_MOTOR))
        shutdown("bad endstop binding");
    e->motor = motor;
    e->stepper = stepper;
    e->group = args[7] ? 1 : 0;
    e->ts = NULL;
    e->trigger_reason = 0;
    e->time.func = endstop_event;

    uint8_t oid;
    struct endstop *other;
    foreach_oid(oid, other, command_config_endstop) {
        if (other != e && other->pin_id == e->pin_id)
            shutdown("endstop: duplicate pin");
    }
}
DECL_COMMAND(command_config_endstop,
             "config_endstop oid=%c endstop_id=%c pin=%u pull_up=%c invert=%c"
             " motor=%c stepper=%c group=%c");

void
command_query_endstop(uint32_t *args)
{
    struct endstop *e = oid_lookup(args[0], command_config_endstop);
    sched_del_timer(&e->time);
    e->rest_ticks = args[1];
    if (!e->rest_ticks) {
        e->armed = 0;
        return;
    }
    e->tripped = 0;
    e->armed = 1;
    e->last_clear_clock = timer_read_time();
    e->time.waketime = e->last_clear_clock + e->rest_ticks;
    sched_add_timer(&e->time);
}
DECL_COMMAND(command_query_endstop,
             "query_endstop oid=%c rest_ticks=%u");

void
command_endstop_query_state(uint32_t *args)
{
    struct endstop *e = oid_lookup(args[0], command_config_endstop);
    uint8_t raw = gpio_in_read(e->pin) ? 1 : 0;
    sendf("endstop_state oid=%c armed=%c pin_value=%c tripped=%c"
          " trip_clock=%u",
          args[0], e->armed, raw, e->tripped, (uint32_t)e->trip_clock);
}
DECL_COMMAND(command_endstop_query_state, "endstop_query_state oid=%c");

void
command_endstop_arm_trsync(uint32_t *args)
{
    struct endstop *e = oid_lookup(args[0], command_config_endstop);
    e->ts = trsync_oid_lookup(args[1]);
    e->trigger_reason = args[2];
}
DECL_COMMAND(command_endstop_arm_trsync,
             "endstop_arm_trsync oid=%c trsync_oid=%c trigger_reason=%c");

void
command_endstop_clear_trsync(uint32_t *args)
{
    struct endstop *e = oid_lookup(args[0], command_config_endstop);
    e->ts = NULL;
    e->trigger_reason = 0;
}
DECL_COMMAND(command_endstop_clear_trsync, "endstop_clear_trsync oid=%c");

void
endstop_trip_task(void)
{
    if (!sched_check_wake(&endstop_trip_wake))
        return;
    uint8_t oid;
    struct endstop *e;
    uint8_t any_processing = 0;
    irqstatus_t flag = irq_save();
    foreach_oid(oid, e, command_config_endstop) {
        if (!e->trip_pending)
            continue;
        e->trip_pending = 0;
        e->trip_processing = 1;
        any_processing = 1;
    }
    irq_restore(flag);
    if (!any_processing)
        return;
    uint8_t needs_stop = 0;
    foreach_oid(oid, e, command_config_endstop) {
        if (e->trip_processing && !e->group)
            needs_stop = 1;
    }
    if (needs_stop) {
        uint64_t discard_clock;
        (void)handle_stop_inner(&discard_clock);
    }
    foreach_oid(oid, e, command_config_endstop) {
        if (!e->trip_processing)
            continue;
        e->trip_processing = 0;
        mcu_transport_emit_endstop_trip(e->endstop_id, e->trip_clock);
    }
}
DECL_TASK(endstop_trip_task);
