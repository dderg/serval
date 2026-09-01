// Handling of stepper drivers.
//
// Copyright (C) 2016-2025  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "autoconf.h" // CONFIG_*
#include "basecmd.h" // oid_alloc
#include "board/gpio.h" // gpio_out_write
#include "board/irq.h" // irq_disable / irq_enable
#include "board/misc.h" // timer_from_us, timer_read_time, timer_is_before
#include "command.h" // DECL_COMMAND, command_decode_ptr
#include "sched.h" // DECL_SHUTDOWN
#include "trsync.h" // trsync_add_signal
#if CONFIG_MOTION_RUNTIME
#include "runtime.h" // StepperBindingRust
#endif
#include "event_log.h" // event_log_emit (mcu structured-log ready marker)
#include "generic/fault_handler.h" // kalico_diag_emit_prior_crash (Stage 5)
#include "stepper.h"

#if CONFIG_MOTION_RUNTIME
// 1 us single-edge floor per step, in CONFIG_CLOCK_FREQ ticks: the physical
// cadence ceiling the configure-time step-width guard is derived against.
#define STEP_MIN_EDGE_DWT ((CONFIG_CLOCK_FREQ) / 1000000u)
#endif

volatile uint32_t config_stepper_oids_seen
    __attribute__((used, externally_visible));

void
command_config_stepper(uint32_t *args)
{
    {
        uint8_t oid = args[0] & 0xFFu;
        if (oid < 32)
            config_stepper_oids_seen |= (1u << oid);
    }
    {
        extern void runtime_diag_progress(uint32_t tag, uint32_t stage,
                                          uint32_t value);
        uint32_t exp_lo = (uint32_t)((uintptr_t)command_config_stepper & 0xFFu);
        runtime_diag_progress(0xCD, args[0] & 0xFFu, exp_lo);
    }
    struct stepper *s = oid_alloc(args[0], command_config_stepper, sizeof(*s));
    int_fast8_t invert_step = (int_fast8_t)args[3];
    s->step_both_edge = invert_step < 0;
    s->step_idle_level = invert_step > 0;
    s->step_pulse_ticks = args[4];
    s->step_pin = gpio_out_setup(args[1], s->step_idle_level);
    s->dir_pin = gpio_out_setup(args[2], 0);
    s->position = -POSITION_BIAS;
    if (s->step_both_edge)
        s->flags |= SF_SINGLE_SCHED;
    move_queue_setup(&s->mq, sizeof(struct stepper_move));
    move_queue_setup(&s->completed_barriers, sizeof(struct stepper_move));
    if (HAVE_EDGE_OPTIMIZATION) {
        if (s->step_both_edge && s->step_pulse_ticks <= EDGE_STEP_TICKS)
            s->flags |= SF_OPTIMIZED_PATH;
        else
            s->time.func = stepper_event_full;
    } else if (HAVE_AVR_OPTIMIZATION) {
        if (!s->step_both_edge && s->step_pulse_ticks <= AVR_STEP_TICKS)
            s->flags |= SF_SINGLE_SCHED | SF_OPTIMIZED_PATH;
        else
            s->time.func = stepper_event_full;
    } else if (!CONFIG_INLINE_STEPPER_HACK) {
        s->time.func = stepper_event_full;
    }
#if !CONFIG_MOTION_RUNTIME
    // The config phase runs after the host's identify/attach handshake has
    // installed the mcu-log hook; emitting at boot races the host and the
    // frame is lost. A MOTION_RUNTIME build hangs this off
    // kalico_configure_axis.
    static uint8_t event_log_ready_emitted;
    if (!event_log_ready_emitted) {
        event_log_ready_emitted = 1;
        event_log_emit(EVENT_LOG_LEVEL_DEBUG, EVENT_LOG_SUBSYS_RUNTIME,
                       EVENT_LOG_EVENT_RUNTIME_MCU_READY, 0, 0, 0);
        kalico_diag_emit_prior_crash();
    }
#endif
}
DECL_COMMAND(command_config_stepper, "config_stepper oid=%c step_pin=%c"
             " dir_pin=%c invert_step=%c step_pulse_ticks=%u");

struct stepper *
stepper_oid_lookup(uint8_t oid)
{
    return oid_lookup(oid, command_config_stepper);
}

static void
stepper_stop(struct trsync_signal *tss, uint8_t reason)
{
    struct stepper *s = container_of(tss, struct stepper, stop_signal);
    stepper_classic_halt(s);
#if CONFIG_SAMPLE_STEPPING
    extern void sample_stepping_halt(void);
    sample_stepping_halt();
#endif
}

void
command_stepper_stop_on_trigger(uint32_t *args)
{
    struct stepper *s = stepper_oid_lookup(args[0]);
    struct trsync *ts = trsync_oid_lookup(args[1]);
    trsync_add_signal(ts, &s->stop_signal, stepper_stop);
}
DECL_COMMAND(command_stepper_stop_on_trigger,
             "stepper_stop_on_trigger oid=%c trsync_oid=%c");

void
command_diag_stepper_buzz(uint32_t *args)
{
    uint8_t oid = args[0] & 0xFFu;
    uint8_t dir = args[1] & 0x01u;
    uint32_t step_count = args[2];
    uint32_t period_ticks = args[3];
    if (step_count > 2000) step_count = 2000;
    uint32_t min_period = timer_from_us(100);
    if (period_ticks < min_period) period_ticks = min_period;

    struct stepper *s = stepper_oid_lookup(oid);
    gpio_out_write(s->dir_pin, dir);

    // Settle direction before first step edge (TMC datasheet typically
    // requires >= 20 ns; 1 µs is generous).
    uint32_t settle_deadline = timer_read_time() + timer_from_us(1);
    while (timer_is_before(timer_read_time(), settle_deadline))
        ;

    uint32_t deadline = timer_read_time();
    for (uint32_t i = 0; i < step_count; i++) {
        gpio_out_toggle(s->step_pin);
        if (!s->step_both_edge) {
            uint32_t fall_deadline = timer_read_time() + s->step_pulse_ticks;
            while (timer_is_before(timer_read_time(), fall_deadline))
                ;
            gpio_out_toggle(s->step_pin);
        }
        deadline += period_ticks;
        while (timer_is_before(timer_read_time(), deadline))
            ;
    }

    sendf("diag_stepper_buzz_response oid=%c step_count=%u",
          oid, step_count);
}
DECL_COMMAND(command_diag_stepper_buzz,
             "diag_stepper_buzz oid=%c dir=%c step_count=%u"
             " period_ticks=%u");

void
stepper_shutdown(void)
{
    uint8_t i;
    struct stepper *s;
    foreach_oid(i, s, command_config_stepper) {
        stepper_stop(&s->stop_signal, 0);
    }
    stepper_suppress_clear_all();
}
DECL_SHUTDOWN(stepper_shutdown);

#if CONFIG_MOTION_RUNTIME

static volatile uint8_t runtime_motor_suppress_mask[RUNTIME_MOTOR_COUNT];

struct runtime_motor_stepper {
    struct stepper *stepper;
    uint8_t invert_dir;
};

static struct runtime_motor_stepper runtime_motor_steppers[RUNTIME_MOTOR_COUNT]
                                                          [RUNTIME_MAX_STEPPERS_PER_MOTOR];
static uint8_t runtime_motor_stepper_count[RUNTIME_MOTOR_COUNT];

__attribute__((used, externally_visible))
uint8_t
runtime_motor_binding_count(uint8_t motor_idx)
{
    if (motor_idx >= RUNTIME_MOTOR_COUNT) return 0;
    return runtime_motor_stepper_count[motor_idx];
}

void
stepper_suppress_set(uint8_t motor, uint8_t stepper)
{
    if (motor >= RUNTIME_MOTOR_COUNT
        || stepper >= RUNTIME_MAX_STEPPERS_PER_MOTOR)
        shutdown("suppress index");
    runtime_motor_suppress_mask[motor] |= (uint8_t)(1u << stepper);
}

__attribute__((used, externally_visible))
uint8_t
stepper_suppress_mask(uint8_t motor)
{
    if (motor >= RUNTIME_MOTOR_COUNT)
        shutdown("suppress index");
    return runtime_motor_suppress_mask[motor];
}

void
stepper_suppress_clear_all(void)
{
    for (uint8_t i = 0; i < RUNTIME_MOTOR_COUNT; i++)
        runtime_motor_suppress_mask[i] = 0;
}

extern void *runtime_handle;

void
command_kalico_configure_axis(uint32_t *args)
{
    uint8_t axis_idx        = args[0];
    uint8_t mode            = args[1];
    uint32_t mstep_bits     = args[2];
    uint32_t extrusion_bits = args[3];
    uint8_t stepper_count   = args[4];
    uint16_t blob_len       = (uint16_t)args[5];
    const uint8_t *blob     = command_decode_ptr(args[6]);

    if (axis_idx >= RUNTIME_MOTOR_COUNT)
        shutdown("configure_axis axis_idx out of range");
    if (mode > 1)
        shutdown("configure_axis mode invalid");
    if (stepper_count > RUNTIME_MAX_STEPPERS_PER_MOTOR)
        shutdown("configure_axis too many steppers per axis");
    if (blob_len != (uint16_t)stepper_count * 4)
        shutdown("configure_axis blob length mismatch");
    if (!runtime_handle)
        shutdown("configure_axis before runtime init");

    struct {
        struct stepper *stepper;
        uint8_t invert_dir;
        uint8_t tmc_cs_oid;
    } staged[RUNTIME_MAX_STEPPERS_PER_MOTOR] = {{0}};

    extern void *command_config_spi(uint32_t *);
    for (uint8_t i = 0; i < stepper_count; i++) {
        uint8_t stepper_oid = blob[i*4 + 0];
        uint8_t dir_invert  = blob[i*4 + 1];
        uint8_t tmc_cs_oid  = blob[i*4 + 2];
        uint8_t flags       = blob[i*4 + 3];
        if (flags != 0)
            shutdown("configure_axis reserved stepper flags must be zero");
        if (dir_invert > 1)
            shutdown("configure_axis dir_invert must be 0 or 1");
        struct stepper *s = oid_lookup(stepper_oid, command_config_stepper);
        if (tmc_cs_oid != 0xFF) {
            (void)oid_lookup(tmc_cs_oid, command_config_spi);
        }
        staged[i].stepper = s;
        staged[i].invert_dir = dir_invert;
        staged[i].tmc_cs_oid = tmc_cs_oid;
    }

    struct StepperBindingRust bindings[RUNTIME_MAX_STEPPERS_PER_MOTOR];
    for (uint8_t i = 0; i < stepper_count; i++) {
        bindings[i].stepper_oid = blob[i*4 + 0];
        bindings[i].tmc_cs_oid = staged[i].tmc_cs_oid;
        bindings[i]._pad[0] = 0;
        bindings[i]._pad[1] = 0;
    }
    int32_t rc = runtime_configure_axis(
        runtime_handle, axis_idx, mode, mstep_bits,
        stepper_count > 0 ? bindings : 0,
        stepper_count);
    if (rc != 0)
        shutdown("configure_axis rejected by runtime");

    uint8_t motor_both_edge = stepper_count ? staged[0].stepper->step_both_edge
                                            : 1;
    uint32_t motor_pulse_ticks = 0;
    for (uint8_t i = 0; i < stepper_count; i++) {
        if (staged[i].stepper->step_both_edge != motor_both_edge)
            shutdown("configure_axis mixed step edge modes on one axis");
        if (staged[i].stepper->step_pulse_ticks > motor_pulse_ticks)
            motor_pulse_ticks = staged[i].stepper->step_pulse_ticks;
    }

    runtime_motor_stepper_count[axis_idx] = stepper_count;
    for (uint8_t i = 0; i < stepper_count; i++) {
        runtime_motor_steppers[axis_idx][i].stepper = staged[i].stepper;
        runtime_motor_steppers[axis_idx][i].invert_dir = staged[i].invert_dir;
    }
    runtime_motor_suppress_mask[axis_idx] = 0;
    (void)extrusion_bits;

    // Guard the motor's physical step cadence: half the sample window (the
    // step ISR is shared across axes) must fit at least one step — the 1us
    // edge floor, plus the pulse-width busy-wait when the driver only steps
    // on rising edges (no dedge).
    uint32_t sample_ticks = CONFIG_CLOCK_FREQ / CONFIG_MOTION_SAMPLE_RATE_HZ;
    uint32_t per_step_ticks = STEP_MIN_EDGE_DWT
        + (motor_both_edge ? 0 : motor_pulse_ticks);
    if ((sample_ticks / 2) / per_step_ticks < 1)
        shutdown("configure_axis step pulse wider than the sample budget");

    extern void runtime_tick_enable(void);
    runtime_tick_enable();

    // Emit only after the first configure_axis: the config phase runs after the
    // host's identify/attach handshake installs the mcu-log hook. Emitting at
    // MCU boot / first drain races ahead of the host connecting; the frame is
    // lost.
    static uint8_t event_log_ready_emitted;
    if (!event_log_ready_emitted) {
        event_log_ready_emitted = 1;
        event_log_emit(EVENT_LOG_LEVEL_DEBUG, EVENT_LOG_SUBSYS_RUNTIME,
                        EVENT_LOG_EVENT_RUNTIME_MCU_READY, 0, 0, 0);
        kalico_diag_emit_prior_crash();
    }
}
DECL_COMMAND(command_kalico_configure_axis,
             "kalico_configure_axis axis_idx=%c mode=%c microstep_distance=%u"
             " extrusion_per_xy_mm=%u stepper_count=%c steppers=%*s");

void
command_runtime_reset(uint32_t *args)
{
    (void)args;
    if (!runtime_handle)
        shutdown("runtime reset before runtime init");
    irqstatus_t flag = irq_save();
    int32_t rc = runtime_reset(runtime_handle);
    irq_restore(flag);
    if (rc != 0)
        shutdown("runtime reset rejected");
}
DECL_COMMAND(command_runtime_reset, "runtime_reset");

#endif

void
command_runtime_diag_dump(uint32_t *args)
{
    (void)args;
    kalico_diag_emit_live();
}
DECL_COMMAND(command_runtime_diag_dump, "runtime_diag_dump");

#if CONFIG_MOTION_RUNTIME

void
command_kalico_phase_stepping_enable_spi(uint32_t *args)
{
    (void)args;
    extern void phase_stepping_enable_writes(void);
    phase_stepping_enable_writes();
}
DECL_COMMAND(command_kalico_phase_stepping_enable_spi,
             "kalico_phase_stepping_enable_spi");

void
command_kalico_phase_stepping_disable_spi(uint32_t *args)
{
    (void)args;
    extern void phase_stepping_disable_writes(void);
    phase_stepping_disable_writes();
}
DECL_COMMAND(command_kalico_phase_stepping_disable_spi,
             "kalico_phase_stepping_disable_spi");

// A mode switch hands the axis between the classic executor and the
// runtime's sample walker. Both drive the same motor, so on entry into
// Phase mode the runtime adopts the classic executor's step count -
// otherwise the host's later transport seed shifts the phase readout
// out from under the freshly aligned coils.
static void
runtime_adopt_classic_count(uint8_t axis_idx)
{
    if (axis_idx >= RUNTIME_MOTOR_COUNT
        || !runtime_motor_stepper_count[axis_idx])
        return;
    struct runtime_motor_stepper *rms = &runtime_motor_steppers[axis_idx][0];
    int32_t wire = stepper_classic_wire_position(rms->stepper);
    int32_t count = rms->invert_dir ? -wire : wire;
    if (runtime_seed_axis_count(runtime_handle, axis_idx, count) != 0)
        shutdown("kalico_set_axis_mode count seed rejected");
}

void
command_kalico_set_axis_mode(uint32_t *args)
{
    if (!runtime_handle)
        shutdown("kalico_set_axis_mode before runtime init");
    uint8_t axis_idx = args[0];
    uint8_t mode = args[1];
    if (axis_idx >= RUNTIME_MOTOR_COUNT)
        shutdown("kalico_set_axis_mode axis_idx out of range");
    int32_t rc = runtime_set_axis_mode(runtime_handle, axis_idx, mode);
    if (rc == -2)
        shutdown("kalico_set_axis_mode rejected: sample playback active");
    if (rc != 0)
        shutdown("kalico_set_axis_mode rejected: bad axis or mode");
    if (mode == 1)
        runtime_adopt_classic_count(axis_idx);
}
DECL_COMMAND(command_kalico_set_axis_mode,
             "kalico_set_axis_mode axis_idx=%c mode=%c");

void
command_kalico_set_stepper_offset(uint32_t *args)
{
    if (!runtime_handle)
        shutdown("kalico_set_stepper_offset before runtime init");
    uint8_t stepper_idx = args[0];
    int32_t delta = (int32_t)args[1];
    uint16_t max_per_sample = args[2];
    int32_t rc = runtime_set_stepper_offset(
        runtime_handle, stepper_idx, delta, max_per_sample);
    if (rc != 0)
        shutdown("kalico_set_stepper_offset rejected (bad parameters)");
}
DECL_COMMAND(command_kalico_set_stepper_offset,
             "kalico_set_stepper_offset stepper_idx=%c delta_microsteps=%i"
             " max_microsteps_per_sample=%hu");


void
command_kalico_phase_jog_to(uint32_t *args)
{
    if (!runtime_handle)
        shutdown("kalico_phase_jog_to before runtime init");
    uint8_t stepper_oid = args[0];
    uint16_t target_phase = args[1];
    uint16_t max_per_sample = args[2];
    int32_t rc = runtime_phase_jog_to(
        runtime_handle, stepper_oid, target_phase, max_per_sample);
    if (rc != 0)
        shutdown("kalico_phase_jog_to rejected (bad args or not in phase mode)");
}
DECL_COMMAND(command_kalico_phase_jog_to,
             "kalico_phase_jog_to oid=%c target_phase=%hu"
             " max_microsteps_per_sample=%hu");

void
command_kalico_phase_align_to(uint32_t *args)
{
    if (!runtime_handle)
        shutdown("kalico_phase_align_to before runtime init");
    uint8_t stepper_oid = args[0];
    uint16_t target_phase = args[1];
    int32_t rc = runtime_phase_align_to(
        runtime_handle, stepper_oid, target_phase);
    if (rc == -2)
        shutdown("kalico_phase_align_to rejected: sample playback active");
    if (rc != 0)
        shutdown("kalico_phase_align_to rejected: unknown stepper oid"
                 " or bad target_phase");
}
DECL_COMMAND(command_kalico_phase_align_to,
             "kalico_phase_align_to oid=%c target_phase=%hu");

void
command_kalico_get_phase_state(uint32_t *args)
{
    if (!runtime_handle)
        shutdown("kalico_get_phase_state before runtime init");
    uint8_t stepper_oid = args[0];
    uint8_t axis_idx = 0, mode = 0, settled = 0;
    uint16_t phase = 0;
    int32_t rc = runtime_get_phase_state(
        runtime_handle, stepper_oid, &axis_idx, &mode, &phase, &settled);
    if (rc != 0)
        shutdown("kalico_get_phase_state unknown stepper oid");
    sendf("motion_phase_state oid=%c axis_idx=%c mode=%c phase=%hu settled=%c",
          stepper_oid, axis_idx, mode, phase, settled);
}
DECL_COMMAND(command_kalico_get_phase_state,
             "kalico_get_phase_state oid=%c");

#endif
