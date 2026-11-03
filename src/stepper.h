#ifndef __STEPPER_H
#define __STEPPER_H

#include <stdint.h>
#include "autoconf.h" // CONFIG_MOTION_RUNTIME
#include "basecmd.h" // struct move_queue_head
#include "board/gpio.h" // struct gpio_out
#include "board/misc.h" // timer_read_time
#include "compiler.h" // DIV_ROUND_UP
#include "sched.h" // struct timer
#include "trsync.h" // struct trsync_signal

#define RUNTIME_MOTOR_COUNT 4
#define RUNTIME_MAX_STEPPERS_PER_MOTOR 4

#if CONFIG_MOTION_RUNTIME
uint8_t runtime_motor_binding_count(uint8_t motor_idx);
#else
static inline uint8_t
runtime_motor_binding_count(uint8_t motor_idx)
{
    (void)motor_idx;
    return 0;
}
#endif

struct stepper {
    struct timer time;
    uint32_t interval;
#if CONFIG_HIGH_PREC_STEP
    int32_t add;
    int32_t add2;
    uint_fast8_t shift;
    uint16_t int_low_acc;
#else
    int16_t add;
#endif
    uint32_t count;
    uint32_t next_step_time;
    uint32_t position;
    struct move_queue_head mq;
    struct move_queue_head completed_barriers;
    uint8_t flags : 8;
    struct gpio_out step_pin, dir_pin;
    uint32_t step_pulse_ticks;
    uint8_t step_both_edge, step_idle_level;
    struct trsync_signal stop_signal;
    uint32_t last_reset_clock;
    uint16_t need_reset_discards;
};

void command_config_stepper(uint32_t *args);
struct stepper *stepper_oid_lookup(uint8_t oid);
uint64_t stepper_halt_oid(uint8_t oid);

struct stepper_move {
    struct move_node node;
    uint32_t interval;
#if CONFIG_HIGH_PREC_STEP
    uint32_t next_interval;
    int32_t add;
    int32_t add2;
    uint_fast8_t shift;
    uint16_t int_low_acc;
#else
    int16_t add;
#endif
    uint16_t count;
    uint8_t flags;
};

enum { MF_DIR=1<<0, MF_BARRIER=1<<1 };

enum { POSITION_BIAS=0x40000000 };

enum {
    SF_LAST_DIR=1<<0, SF_NEXT_DIR=1<<1, SF_RESET_FENCED=1<<2, SF_NEED_RESET=1<<3,
    SF_SINGLE_SCHED=1<<4, SF_OPTIMIZED_PATH=1<<5, SF_HAVE_ADD=1<<6,
#if CONFIG_HIGH_PREC_STEP
    SF_HIGH_PREC_STEP=1<<7
#endif
};

#if CONFIG_INLINE_STEPPER_HACK && CONFIG_WANT_STEPPER_OPTIMIZED_BOTH_EDGE
 #define HAVE_OPTIMIZED_PATH 1
 #define HAVE_EDGE_OPTIMIZATION 1
 #define HAVE_AVR_OPTIMIZATION 0
#elif CONFIG_INLINE_STEPPER_HACK && CONFIG_MACH_AVR
 #define HAVE_OPTIMIZED_PATH 1
 #define HAVE_EDGE_OPTIMIZATION 0
 #define HAVE_AVR_OPTIMIZATION 1
#else
 #define HAVE_OPTIMIZED_PATH 0
 #define HAVE_EDGE_OPTIMIZATION 0
 #define HAVE_AVR_OPTIMIZATION 0
#endif

// Edge optimization only enabled when fastest rate notably slower than 100ns
#define EDGE_STEP_TICKS DIV_ROUND_UP(CONFIG_CLOCK_FREQ, 8000000)
#define AVR_STEP_TICKS 40 // minimum instructions between step gpio pulses

uint_fast8_t stepper_event_full(struct timer *t);
uint_fast8_t stepper_load_next(struct stepper *s);
uint32_t stepper_get_position(struct stepper *s);
int32_t stepper_classic_wire_position(struct stepper *s);
void stepper_classic_halt(struct stepper *s);
uint8_t stepper_classic_halt_all(uint32_t *stream_end_clock);

#if CONFIG_HIGH_PREC_STEP
static __always_inline void
stepper_add_interval(uint32_t *time, struct stepper *s)
{
    uint32_t interval = s->interval + s->int_low_acc;
    *time += interval >> s->shift;
    s->int_low_acc = interval - ((interval >> s->shift) << s->shift);
}

static __always_inline void
stepper_inc_interval(struct stepper *s)
{
    s->interval += s->add;
    s->add += s->add2;
}
#endif

static __always_inline void
stepper_full_edge(struct stepper *s)
{
    gpio_out_toggle_noirq(s->step_pin);
#if CONFIG_MCU_SIM
    s->time.waketime += s->step_pulse_ticks;
#else
    s->time.waketime = timer_read_time() + s->step_pulse_ticks;
#endif
}

static __always_inline uint_fast8_t
stepper_edge_finish(struct stepper *s)
{
    uint32_t count = s->count - 1;
    if (likely(count)) {
        s->count = count;
#if CONFIG_HIGH_PREC_STEP
        if (s->flags & SF_HIGH_PREC_STEP) {
            stepper_add_interval(&s->time.waketime, s);
            stepper_inc_interval(s);
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

static __always_inline uint_fast8_t
stepper_full_finish(struct stepper *s)
{
    uint32_t min_next_time = s->time.waketime;
    uint32_t count = s->count - 1;
    if (likely(count & 1 && !(s->flags & SF_SINGLE_SCHED)))
        goto reschedule_min;
    if (likely(count)) {
#if CONFIG_HIGH_PREC_STEP
        if (s->flags & SF_HIGH_PREC_STEP) {
            stepper_add_interval(&s->next_step_time, s);
            stepper_inc_interval(s);
        } else {
            s->next_step_time += s->interval;
            s->interval += s->add;
        }
#else
        s->next_step_time += s->interval;
        s->interval += s->add;
#endif
        if (unlikely(timer_is_before(s->next_step_time, min_next_time)))
            goto reschedule_min;
        s->count = count;
        s->time.waketime = s->next_step_time;
        return SF_RESCHEDULE;
    }
    return stepper_load_next(s);
reschedule_min:
    s->count = count;
    return SF_RESCHEDULE;
}

static __always_inline uint_fast8_t
stepper_timer_on_edge_path(struct timer *t)
{
    return HAVE_EDGE_OPTIMIZATION && !t->func;
}

static __always_inline void
stepper_step_edge(struct timer *t, uint_fast8_t on_edge_path)
{
    struct stepper *s = container_of(t, struct stepper, time);
    if (on_edge_path)
        gpio_out_toggle_noirq(s->step_pin);
    else
        stepper_full_edge(s);
    barrier();
}

static __always_inline uint_fast8_t
stepper_step_finish(struct timer *t, uint_fast8_t on_edge_path)
{
    struct stepper *s = container_of(t, struct stepper, time);
    if (on_edge_path)
        return stepper_edge_finish(s);
    return stepper_full_finish(s);
}

#endif // stepper.h
