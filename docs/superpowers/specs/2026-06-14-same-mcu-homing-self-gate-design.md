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
   (`kalico_runtime_now_ticks`), disarms, sets `trip_pending`, and wakes
   `endstop_trip_task`. **It does not stop motion.** The emit is deferred to the
   foreground task because `kalico_transport_send_frame` uses a shared `tx_buf`
   and the USB transmit cursor, neither safe from IRQ (`endstop.c:26-29`).
2. **Report (task).** `endstop_trip_task` ships `kalico_endstop_tripped`
   (`trip_clock`) to the host.
3. **Stop (host round-trip).** The host (`bridge.rs::dispatch_endstop_trip`)
   spawns `homing-trip-handler`, flushes the pump, and `broadcast_stop` sends a
   blocking `Stop` to every participating stepper MCU. Each `Stop` runs
   `handle_stop` → `kalico_runtime_gate_pieces` (freeze the curve evaluator) and
   replies with `discard_clock = kalico_runtime_now_ticks`.
4. **Reconcile.** The host reconstructs the **trip** position at `trip_clock`
   and the **final** position at `discard_clock` from motion history.

For same-MCU homing (endstop pin and homed steppers on one board — the Trident
X/Y-on-H7 case) the physical brake waits for the full host round-trip in step 3.
That latency is pure overshoot past the trigger point.

### Why `discard_clock` is load-bearing

`klippy/extras/homing.py:80` sets the post-home position to:

```python
trigger_height + (final_pos[axis] - trip_pos[axis])
```

`final_pos − trip_pos` is the **overshoot**, added to the trigger height and
fed to `toolhead.set_position` (homing.py:305). It is `planned(discard_clock) −
planned(trip_clock)`. Today this is correct because the MCU follows the planned
trajectory right up until the host's `Stop`, so `discard_clock` equals the clock
at which motion actually froze.

If the MCU brakes early (this change) but the host still reads `discard_clock`
from its *later* `Stop`, the overshoot is overstated by `speed × round-trip` and
the homed position is wrong by that much — a systematic error. So the brake
clock must stay honest (see design point 2).

## Design

### 1. Extract `handle_stop_inner` and call it on a local trip

Split the gating core out of `handle_stop` in `src/kalico_dispatch.c`. The only
part `handle_stop` keeps for itself is the host-facing `send_stop_response` — a
self-triggered stop has no host request to reply to, and sending an unsolicited
`StopResponse` would desync the control channel (the host correlates responses
by id).

`handle_stop_inner` is **not** `static` — `endstop_trip_task` lives in
`endstop.c`, which already includes `kalico_dispatch.h`, so its prototype goes
there (beside `kalico_native_emit_endstop_trip`):

```c
// kalico_dispatch.h
int32_t handle_stop_inner(uint64_t *discard_clock);
```

```c
static uint64_t s_gate_clock;   // brake clock, captured on the first gate

int32_t handle_stop_inner(uint64_t *discard_clock) {
    int32_t rc = KALICO_ERR_NOT_INIT;
    if (runtime_handle) {
        irqstatus_t flag = irq_save();
        if (!kalico_runtime_pieces_gated(runtime_handle)) {
            rc = kalico_runtime_gate_pieces(runtime_handle);
            s_gate_clock = kalico_runtime_now_ticks(runtime_handle);
        } else {
            rc = KALICO_OK;
        }
        irq_restore(flag);
    }
    *discard_clock = s_gate_clock;
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
priority TIM5 ISR which would otherwise preempt the mutation. This matches what
`handle_stop` does today.

In `endstop_trip_task`, brake locally first (`handle_stop_inner`), then emit the
trip event(s) as today:

```c
void endstop_trip_task(void) {
    if (!sched_check_wake(&endstop_trip_wake))
        return;
    uint64_t discard_clock;
    handle_stop_inner(&discard_clock);   // brake locally; return value unused here
    uint8_t oid;
    struct endstop *e;
    foreach_oid(oid, e, command_config_endstop) {
        if (!e->trip_pending)
            continue;
        e->trip_pending = 0;
        kalico_native_emit_endstop_trip(e->endstop_id, e->trip_clock);
    }
}
```

The brake gates the whole engine (not a single axis), matching the existing
`broadcast_stop` semantics, which already gates each participant's whole engine.

### 2. Keep the brake clock honest

`handle_stop_inner` captures `s_gate_clock` the **first** time it gates and
returns that stored clock on every subsequent call. So when the host's later
`Stop` arrives and runs `handle_stop` → `handle_stop_inner`, it finds pieces
already gated and replies with the **real brake time** instead of a fresh
`now_ticks`. The host's reconstruction and the wire protocol are unchanged;
`discard_clock` simply now always means "the clock motion actually froze,"
whether the freeze was local or host-driven.

No explicit reset is needed across homing cycles: `ResumeStream` ungates after
homing (`pieces_gated` → false), so the next trip's first gate recaptures a
fresh `s_gate_clock`.

### 3. The double stop is idempotent

After the self-gate, the host *will* send its own `Stop` to every participant —
including the MCU that already braked. That second stop must be a safe no-op:

- **First stop (self-gate):** `pieces_gated` false → gate, capture
  `s_gate_clock = now`.
- **Second stop (host `Stop`):** `pieces_gated` true → `else` branch returns
  `KALICO_OK` and the *stored* `s_gate_clock`. No re-gate, no `gate_pieces`
  call, no clock overwrite. The host gets the `result=0` it expects and the
  honest brake clock as `discard_clock`.

There is no arrival-order hazard: the self-gate runs to completion inside
`endstop_trip_task` before the main loop processes any incoming `Stop`, so
"already gated" is always true when the host's `Stop` lands. `gate_pieces` is
itself idempotent (`discard_pending` + set-flag), so even a direct double-gate
would be harmless — the `pieces_gated` guard exists to preserve the *clock*, not
to protect `gate_pieces`.

This is exercised in the Rust tests below (gate, advance clock, gate again →
`discard_clock` unchanged).

### 4. Expose `pieces_gated` over the FFI

The engine already has `pieces_gated()` (`engine.rs:264`, used internally in
`runtime_ffi.rs`), but there is no C-callable export. Add one beside
`kalico_runtime_gate_pieces` in `rust/kalico-c-api/src/runtime_ffi.rs`:

```rust
#[unsafe(no_mangle)]
pub unsafe extern "C" fn kalico_runtime_pieces_gated(rt: *mut KalicoRuntime) -> bool { ... }
```

and declare it in `rust/kalico-c-api/include/kalico_runtime.h`. Null/`!INIT_DONE`
guards return `false` (matching the other accessors' conservative defaults).

### 5. Lift the CLAUDE.md constraint

`CLAUDE.md` currently says (line 27-28):

> We deliberately do not optimize for same mcu endstop+motor homing. All homing
> works as if mcu with the endstop is a different one than the one that drives
> the motors. it makes testing easier at this stage of development.

This change introduces exactly that optimization. Replace the note with a short
statement that same-MCU trips brake locally (the self-gate), while the host
relay still fans the stop out cross-MCU — both paths converge on the same
`gate_pieces` freeze.

## What stays unchanged

- **Host side**: `broadcast_stop`, `dispatch_endstop_trip`, trip/final position
  reconstruction, `home_axis_start`, `ResumeStream`/ungate. The host still sends
  `Stop` to every participant including the self-gated MCU; that `Stop` is now
  idempotent there and returns the honest stored clock.
- **Wire protocol**: `kalico_endstop_tripped`, `Stop`/`StopResponse`,
  `endstop_query_state` — no format changes.
- **Cross-MCU path**: a remote source MCU self-gates its own (idle) engine
  harmlessly; remote sink MCUs still stop via the host's `broadcast_stop`.

## Error handling

No new error surface. `handle_stop_inner` preserves the existing `rc`
(`KALICO_ERR_NOT_INIT` when the runtime handle is absent; `KALICO_OK`
otherwise). The self-gate path ignores `rc` (there is no host to report to);
the host `Stop` path reports it exactly as before.

## Testing

- **Rust (`cargo nextest run`)**:
  - `kalico_runtime_pieces_gated` returns the engine state (false before gate,
    true after, false after ungate).
  - An already-gated `handle_stop_inner`/`Stop` returns the stored first-gate
    clock, not a fresh `now_ticks` (exercise via the `kalico-c-api`
    `piece_gate.rs` harness: gate, advance the clock, gate again, assert the
    returned `discard_clock` is unchanged).
- **Existing suites**: `motion-bridge` `homing/tests.rs` and `pump` tests
  continue to pass unchanged (host path untouched).
- **kalico-sim**: a same-MCU home (endstop + homed stepper on one emulated MCU)
  where the trip event is delivered to the host with deliberate latency; assert
  the curve evaluator freezes at trip time (motion stops before the host's
  `Stop` lands) and the reported overshoot reflects the true brake, not the
  host round-trip.

## Files touched

| File | Change |
|------|--------|
| `src/kalico_dispatch.c` | extract `handle_stop_inner` (non-`static`: gate + capture/return `s_gate_clock`); `handle_stop` calls it then `send_stop_response`; add `s_gate_clock` static. |
| `src/kalico_dispatch.h` | declare `handle_stop_inner` (so `endstop.c` can call it). |
| `src/endstop.c` | `endstop_trip_task` calls `handle_stop_inner` after emitting the trip event. |
| `rust/kalico-c-api/src/runtime_ffi.rs` | add `kalico_runtime_pieces_gated` export. |
| `rust/kalico-c-api/include/kalico_runtime.h` | declare `kalico_runtime_pieces_gated`. |
| `CLAUDE.md` | replace the "do not optimize same-MCU homing" note with the self-gate behavior. |
| (tests) | `kalico-c-api` gate/clock coverage; kalico-sim same-MCU home. |

## Out of scope

- IRQ-immediate gating (braking inside `endstop_event` rather than the
  foreground task) — a few µs tighter, but the task-level brake is simpler and
  the freeze clock is captured honestly either way.
- Per-axis gating — `gate_pieces` freezes the whole engine, matching existing
  `broadcast_stop` behavior; multi-endstop simultaneous homing already stops
  together today.
- Any change to the cross-MCU relay or the drip-cohort deadman.
