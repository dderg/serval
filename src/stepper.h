#ifndef __STEPPER_H
#define __STEPPER_H

#include <stdint.h> // uint8_t
#include "autoconf.h" // CONFIG_CLASSIC_STEPPING
#include "basecmd.h" // struct move_queue_head
#include "board/gpio.h" // struct gpio_out
#include "compiler.h" // DIV_ROUND_UP
#include "sched.h" // struct timer
#include "trsync.h" // struct trsync_signal

struct stepper {
#if CONFIG_CLASSIC_STEPPING
    struct timer time;
    uint32_t interval;
    int16_t add;
    uint32_t count;
    uint32_t next_step_time;
    uint32_t position;
    struct move_queue_head mq;
    struct move_queue_head completed_barriers;
    uint8_t flags : 8;
#endif
    struct gpio_out step_pin, dir_pin;
    uint32_t step_pulse_ticks;
    uint8_t step_both_edge, step_idle_level;
    struct trsync_signal stop_signal;
};

void command_config_stepper(uint32_t *args);
struct stepper *stepper_oid_lookup(uint8_t oid);

#if CONFIG_CLASSIC_STEPPING

struct stepper_move {
    struct move_node node;
    uint32_t interval;
    int16_t add;
    uint16_t count;
    uint8_t flags;
};

enum { MF_DIR=1<<0, MF_BARRIER=1<<1 };

enum { POSITION_BIAS=0x40000000 };

enum {
    SF_LAST_DIR=1<<0, SF_NEXT_DIR=1<<1, SF_NEED_RESET=1<<3,
    SF_SINGLE_SCHED=1<<4, SF_OPTIMIZED_PATH=1<<5, SF_HAVE_ADD=1<<6
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

uint_fast8_t stepper_event(struct timer *t);
uint_fast8_t stepper_event_full(struct timer *t);
uint32_t stepper_get_position(struct stepper *s);
void stepper_classic_halt(struct stepper *s);
uint8_t stepper_classic_halt_all(uint32_t *stream_end_clock);

#endif // CONFIG_CLASSIC_STEPPING

#endif // stepper.h
