# Same-MCU Homing Self-Gate

## Goal

When an armed endstop trips, let the MCU that owns it **brake its own motion
immediately**, instead of waiting for the host to detect the trip and send a
`Stop` round-trip back. This is the documented "same-MCU local-siren fast-path"
deferred in
[`2026-05-31-trsync-cross-mcu-homing-design.md`](2026-05-31-trsync-cross-mcu-homing-design.md)
(its "Out of scope / future" section), now that the cross-MCU relay is
confirmed working on hardware.

The behavior is **unconditional**: every MCU gates its own curve evaluator
whenever one of its *locally-armed* endstops trips. No host flag, no topology
negotiation. This is safe because an endstop is only ever armed during a
homing/probing move (`query_endstop` with a nonzero `rest_ticks`), so a trip can
never fire mid-print.

## Background: the current stop path

1. **Detect (IRQ).** `src/endstop.c::endstop_event` runs in the sched-timer IRQ.
   On an armed pin going active it latches `trip_clock`
   (`runtime_now_ticks`), disarms, sets `trip_pending`, and wakes
   `endstop_trip_task`. **It does not stop motion.** The emit is deferred to the
   foreground task because `mcu_transport_send_frame` uses a shared `tx_buf`
   and the USB transmit cursor, neither safe from IRQ (`endstop.c:26-29`).
2. **Report (task).** `endstop_trip_task` ships `kalico_endstop_tripped`
   (`trip_clock`) to the host.
3. **Stop (host round-trip).** The host (`bridge.rs::dispatch_endstop_trip`)
   spawns `homing-trip-handler`, flushes the pump, and `broadcast_stop` sends a
   blocking `Stop` to every participating stepper MCU. Each `Stop` runs
   `handle_stop` → `runtime_gate_pieces` (freeze the curve evaluator) and
   replies with `discard_clock`.

For same-MCU homing (endstop pin and homed steppers on one board — the Trident
X/Y-on-H7 case) the physical brake waits for the full host round-trip in step 3.
That latency is pure overshoot past the trigger point.

## Design

### 1. Extract `handle_stop_inner` and call it on a local trip

Split the gating core out of `handle_stop` in `src/mcu_transport_dispatch.c`. The only
part `handle_stop` keeps for itself is the host-facing `send_stop_response` — a
self-triggered stop has no host request to reply to, and sending an unsolicited
`StopResponse` would desync the control channel (the host correlates responses
by id). `handle_stop_inner` is the existing `handle_stop` body, verbatim, minus
the response.

`handle_stop_inner` is **not** `static` — `endstop_trip_task` lives in
`endstop.c`, which already includes `mcu_transport_dispatch.h`, so its prototype goes
there (beside `mcu_transport_emit_endstop_trip`):

```c
// mcu_transport_dispatch.h
int32_t handle_stop_inner(uint64_t *discard_clock);
```

```c
// mcu_transport_dispatch.c
int32_t handle_stop_inner(uint64_t *discard_clock) {
    int32_t rc = RUNTIME_ERR_NOT_INIT;
    uint64_t dc = 0;
    if (runtime_handle) {
        irqstatus_t flag = irq_save();
        rc = runtime_gate_pieces(runtime_handle);
        dc = runtime_now_ticks(runtime_handle);
        irq_restore(flag);
    }
    *discard_clock = dc;
    return rc;
}

static void handle_stop(uint32_t correlation_id) {
    uint64_t discard_clock;
    int32_t rc = handle_stop_inner(&discard_clock);
    send_stop_response(correlation_id, rc, discard_clock);
}
```

`irq_save`/`irq_restore` is required: `gate_pieces` mutates the engine through a
raw pointer with no internal lock, and the engine tick runs in the higher-
priority TIM5 ISR which would otherwise preempt the mutation. This is exactly
what `handle_stop` does today.

In `endstop_trip_task`, brake locally (`handle_stop_inner`), then emit the trip
event(s) as today. The `discard_clock` out-param is ignored on the self-gate
path — there is no host to report it to.

```c
void endstop_trip_task(void) {
    if (!sched_check_wake(&endstop_trip_wake))
        return;
    uint64_t discard_clock;
    handle_stop_inner(&discard_clock);   // brake locally; out-param unused here
    uint8_t oid;
    struct endstop *e;
    foreach_oid(oid, e, command_config_endstop) {
        if (!e->trip_pending)
            continue;
        e->trip_pending = 0;
        mcu_transport_emit_endstop_trip(e->endstop_id, e->trip_clock);
    }
}
```

The brake gates the whole engine (not a single axis), matching the existing
`broadcast_stop` semantics, which already gates each participant's whole engine.

### 2. The double stop just stops again

After the self-gate, the host *will* send its own `Stop` to every participant —
including the MCU that already braked. That second stop simply runs
`handle_stop_inner` again: `gate_pieces` is idempotent (`discard_pending` +
set-flag), so re-gating an already-gated engine is a harmless no-op. The host
gets its `StopResponse` as always. No special-casing, no remembered state.

> **Note (out of scope here):** with the self-gate, the host's `discard_clock`
> (from its own late `Stop`) no longer marks the instant the toolhead actually
> braked — it brakes earlier, at ≈`trip_clock`. The overshoot / slide-past-stop
> accounting that consumes `discard_clock` is being reworked by a separate
> mechanism and is intentionally **not** addressed here. This spec leaves the
> host's `dispatch_endstop_trip` / position-reconstruction path untouched.

### 3. Lift the CLAUDE.md constraint

`CLAUDE.md` currently says (line 27-28):

> We deliberately do not optimize for same mcu endstop+motor homing. All homing
> works as if mcu with the endstop is a different one than the one that drives
> the motors. it makes testing easier at this stage of development.

This change introduces exactly that optimization. Replace the note with a short
statement that same-MCU trips brake locally (the self-gate), while the host
relay still fans the stop out cross-MCU — both paths converge on the same
`gate_pieces` freeze.

## What stays unchanged

- **Host side**: `broadcast_stop`, `dispatch_endstop_trip`, position
  reconstruction, `home_axis_start`, `ResumeStream`/ungate. The host still sends
  `Stop` to every participant including the self-gated MCU; that `Stop` is now a
  harmless re-gate there.
- **Wire protocol**: `kalico_endstop_tripped`, `Stop`/`StopResponse`,
  `endstop_query_state` — no format changes.
- **Cross-MCU path**: a remote source MCU self-gates its own (idle) engine
  harmlessly; remote sink MCUs still stop via the host's `broadcast_stop`.

## Error handling

No new error surface. `handle_stop_inner` preserves the existing `rc`
(`RUNTIME_ERR_NOT_INIT` when the runtime handle is absent; `RUNTIME_OK`
otherwise). The self-gate path ignores `rc` (there is no host to report to); the
host `Stop` path reports it exactly as before.

## Testing

- **Rust (`cargo nextest run`)**: re-gating an already-gated engine is a
  harmless no-op (exercise via the `c-api` `piece_gate.rs` harness: gate,
  gate again, assert still gated and no error / no corruption).
- **Existing suites**: `motion-engine` `homing/tests.rs` and `pump` tests
  continue to pass unchanged (host path untouched).
- **mcu-sim**: a same-MCU home (endstop + homed stepper on one emulated MCU)
  where the trip event is delivered to the host with deliberate latency; assert
  the curve evaluator freezes at trip time (motion stops before the host's
  `Stop` lands).

## Files touched

| File | Change |
|------|--------|
| `src/mcu_transport_dispatch.c` | extract `handle_stop_inner` (non-`static`, current `handle_stop` body minus the response); `handle_stop` calls it then `send_stop_response`. |
| `src/mcu_transport_dispatch.h` | declare `handle_stop_inner` (so `endstop.c` can call it). |
| `src/endstop.c` | `endstop_trip_task` calls `handle_stop_inner` before emitting the trip event(s). |
| `CLAUDE.md` | replace the "do not optimize same-MCU homing" note with the self-gate behavior. |
| (tests) | `c-api` idempotent re-gate coverage; mcu-sim same-MCU home. |

## Out of scope

- Overshoot / slide-past-stop accounting (`discard_clock` consumer) — handled by
  a separate mechanism; the host path is untouched here.
- IRQ-immediate gating (braking inside `endstop_event` rather than the
  foreground task) — a few µs tighter, but the task-level brake is simpler.
- Per-axis gating — `gate_pieces` freezes the whole engine, matching existing
  `broadcast_stop` behavior; multi-endstop simultaneous homing already stops
  together today.
- Any change to the cross-MCU relay or the drip-cohort deadman.
