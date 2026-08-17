// Sample-stream transport commands for phase-stepped lanes.
//
// The argstrings come from src/sample_wire.h, which mirrors
// rust/runtime/src/sample_wire.rs — a runtime test parses the header and
// asserts the two agree, so neither side can drift.
//
// Every command is a thin decode-and-forward: the Rust executor owns abutment,
// interpolation and fault latching. A rejected run latches a distinct fault
// code, which runtime_drain turns into a shutdown on the next foreground pass.
#include <stdint.h>
#include "autoconf.h"
#include "board/irq.h"
#include "command.h"
#include "runtime.h"
#include "sample_wire.h"
#include "sched.h"
#include "stepper.h"
#include "trsync.h"

extern void *runtime_handle;

int32_t runtime_sample_anchor(struct Runtime *rt, uint8_t oid, uint32_t clock,
                              int32_t position);
int32_t runtime_sample_run(struct Runtime *rt, uint8_t oid,
                           uint32_t interval_ticks, uint8_t count,
                           const uint8_t *data, uint16_t data_len);
int32_t runtime_sample_overlay(struct Runtime *rt, uint8_t oid, uint32_t clock,
                               uint32_t interval_ticks, uint8_t count,
                               const uint8_t *data, uint16_t data_len);
int32_t runtime_sample_query(struct Runtime *rt, uint8_t oid,
                             uint64_t *out_clock, int32_t *out_position);
int32_t runtime_sample_halt(struct Runtime *rt, uint64_t halt_clock);

void
command_sample_anchor(uint32_t *args)
{
    if (!runtime_handle)
        shutdown("sample_anchor without a motion runtime");
    irqstatus_t flag = irq_save();
    runtime_sample_anchor(runtime_handle, args[0] & 0xFFu, args[1],
                          (int32_t)args[2]);
    irq_restore(flag);
}
DECL_COMMAND(command_sample_anchor, SAMPLE_ANCHOR_ARGS);

void
command_sample_run(uint32_t *args)
{
    if (!runtime_handle)
        shutdown("sample_run without a motion runtime");
    uint8_t count = args[2] & 0xFFu;
    uint8_t data_len = args[3];
    const uint8_t *data = command_decode_ptr(args[4]);
    if (count > SAMPLE_RUN_COUNT_MAX || data_len > SAMPLE_RUN_DATA_MAX)
        shutdown("sample_run exceeds the wire cap");
    irqstatus_t flag = irq_save();
    runtime_sample_run(runtime_handle, args[0] & 0xFFu, args[1], count, data,
                       data_len);
    irq_restore(flag);
}
DECL_COMMAND(command_sample_run, SAMPLE_RUN_ARGS);

void
command_sample_overlay(uint32_t *args)
{
    if (!runtime_handle)
        shutdown("sample_overlay without a motion runtime");
    uint8_t count = args[3] & 0xFFu;
    uint8_t data_len = args[4];
    const uint8_t *data = command_decode_ptr(args[5]);
    if (count > SAMPLE_RUN_COUNT_MAX || data_len > SAMPLE_RUN_DATA_MAX)
        shutdown("sample_overlay exceeds the wire cap");
    irqstatus_t flag = irq_save();
    runtime_sample_overlay(runtime_handle, args[0] & 0xFFu, args[1], args[2],
                           count, data, data_len);
    irq_restore(flag);
}
DECL_COMMAND(command_sample_overlay, SAMPLE_OVERLAY_ARGS);

void
command_sample_get_position(uint32_t *args)
{
    uint8_t oid = args[0] & 0xFFu;
    if (!runtime_handle)
        shutdown("sample_get_position without a motion runtime");
    uint64_t clock = 0;
    int32_t position = 0;
    irqstatus_t flag = irq_save();
    int32_t rc = runtime_sample_query(runtime_handle, oid, &clock, &position);
    irq_restore(flag);
    if (rc != 0)
        shutdown("sample_get_position for an unbound oid");
    sendf(SAMPLE_POSITION_ARGS, oid, (uint32_t)clock, position);
}
DECL_COMMAND(command_sample_get_position, SAMPLE_GET_POSITION_ARGS);

// trsync trip. Publishes the halt clock through SharedState atomics; the next
// motion tick freezes each lane at the position that clock interpolates to,
// exactly as stepper_classic_halt freezes the classic step chain. Called from
// the trip's IRQ context, so it reads the ISR-published widened clock rather
// than the foreground-only stats clock, and touches no engine state.
void
sample_stepping_halt(void)
{
    if (!runtime_handle)
        return;
    runtime_sample_halt(runtime_handle, runtime_now_ticks(runtime_handle));
}
