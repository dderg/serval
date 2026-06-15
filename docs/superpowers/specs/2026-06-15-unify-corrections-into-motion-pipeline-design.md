# Unify per-motor corrections into the main motion pipeline — design

**Goal:** Make a per-motor "correction" (motors_sync's buzz, MOTOR_ADJUST,
force_move's single-stepper move) a *normal piece on the main motion ring*
carrying a per-piece motor bitmask, and delete the entire bespoke correction
path. Corrections then ride the same async `submit → planner channel → pump
thread` dispatch as regular moves — non-blocking, feedback-paced — which is the
real fix for the SYNC crashes.

## Why — the separate path is what blocked the reactor

Regular motion is non-blocking: `submit_move` drops a move on a channel
(`planner.submit_move → sender.send(PlannerMsg::Move)`, planner.rs:207-209) and
returns; a background **pump thread** (`pump.rs::run_pump`) does the wire send,
paced by `DrainSync` feedback. The reactor never waits on the wire.

The correction path does **not** use that machinery. `stream_correction_entries`
(bridge.rs) loops inline on the reactor thread inside `py.detach`, blocking on
`wait_room` (ring-drain) and on each `kalico_call_on_channel` wire ack. While a
buzz streams, the reactor greenlet is frozen inside that Rust loop — it can't
submit motors_sync's next measurement move or service clocksync. The regular
moves then dispatch late (`seg0_deficit`), the planner overruns
(`replan_overrun`), and the MCU halts the main stream (`-142 STREAM_HALTED`) or
the USB transport drops. Every band-aid (`get_last_move_time` anchoring, real
`dwell`, `wait_moves` before each buzz) added *more* synchronous waiting to the
same blocked thread; one variant also overflowed the correction ring (`-309
RING_FULL`) because far-future-anchored pieces never retired.

Unifying onto the main ring removes the separate synchronous streamer entirely:
a correction becomes a piece the pump dispatches asynchronously, exactly like a
move. This also *strengthens* the CLAUDE.md "uniform cubic, no source-type
special-cases" rule — there stops being a correction primitive at all.

## Design

### 1. The motor mask

Every piece gains a `motor_mask: u8` — a bitmask, one bit per motor of the axis.

- `MAX_MOTORS_PER_AXIS = 8` — a single named constant (so the one place to widen
  to `u16` later is obvious). The `u8` mask width derives from it.
- **`mask == 0` → normal full-axis move:** all motors step, every stepper's
  `position_count` advances, and the axis trajectory accumulator `p_prev`
  advances. This is what every G5/G5.1 move emits today.
- **`mask != 0` → overlay:** only the set motors step and advance their
  `position_count`; **`p_prev` is not advanced.** This is a correction.

The single field encodes both "which motors" and "counts toward position?",
because a *kinematic* move always drives every motor of its axis together — a
motor *subset* is only ever an overlay. Investigation confirmed **no
subset-driving move should count** (followers are separate full axes;
dual_carriage steps all motors of its axis; homing moves are full moves), so
`subset ⇒ don't advance p_prev` has zero exceptions.

### 2. `PieceEntry` layout — the mask is free

`rust/runtime/src/piece_ring.rs` `PieceEntry` is 32 bytes:
`start_time: u64` (0-7), `coeffs: [f32;4]` (8-23), `duration: f32` (24-27),
`_reserved: u32` (28-31, "must be zero"). The mask takes one byte of
`_reserved` — rename to `motor_mask: u8` + `_reserved: [u8;3]`. `#[repr(C)]`
size/alignment unchanged; the wire `to_le_bytes`/`from_le_bytes` and the 32-byte
chunk framing are unchanged. Both MCUs are reflashed together, so the
field-meaning change of those bytes needs no cross-version compat.

### 3. MCU evaluator — one path, mask-driven

Merge `dispatch_correction.rs` into the main `dispatch_stepper.rs` evaluation so
there is one evaluator. Per piece, keyed off `motor_mask`:

- **Step output:** `stepper_sel` is derived from the mask. `mask == 0` →
  `STEPPER_SEL_ALL`. A single-bit mask → that motor's `stepper_sel = idx+1`
  (today's correction behavior). (Multi-bit masks: see Open Questions — the
  current `StepEntry.stepper_sel` encodes "all or one"; a general bitmask needs
  either per-motor step entries or a widened `stepper_sel`. The motors_sync /
  MOTOR_ADJUST / force_move callers only ever target one motor, so single-bit is
  the live requirement; multi-bit can be deferred or emitted as per-motor
  entries.)
- **Position count:** advance `position_count` for exactly the masked steppers
  (`mask == 0` → all, via the existing `commit_position_count` loop; subset →
  only those, via the existing `commit_motor_position_count`).
- **Axis accumulator:** advance `p_prev`/`v_prev` (engine.rs:452) **only when
  `mask == 0`.** A correction leaves `p_prev` untouched, so it is invisible to
  homing / `get_position` / `SET_KINEMATIC_POSITION` / `query_motor_state` (all
  read `p_prev`, not per-stepper `position_count`).
- **Phase-mode motors:** the existing `phase_offset_target` advance
  (dispatch_correction.rs:220) and `ramp_phase_offset` (dispatch_stepper.rs:314)
  are preserved for masked phase-mode motors. Dormant on non-phase benches.

The hot-path cost is one `motor_mask` load + a branch — the per-piece overhead
we consciously accept for the unification.

### 4. Planner — a direct-axis-space submit

Regular moves enter via `submit_move(dx,dy,dz,de,feedrate)` → kinematics →
per-axis cubic. A correction is a raw per-axis displacement on specific motors,
not an XYZ move, so the planner gains one new entry point:

`submit_axis_overlay(mcu_id, axis_idx, motor_mask, pieces)` — takes the cubic
`ProfilePiece`s the existing `plan_correction_sequence` / `plan_correction_profile`
already produce (correction.rs), stamps `motor_mask` onto each `PieceEntry`, and
sends them down the **same** planner channel → pump path as moves
(`PlannerMsg`). No kinematics, no `p_prev` math on the host — the mask tells the
MCU not to touch `p_prev`. Everything downstream (ring, wire `PushPieces`, pump,
`DrainSync`, drain-wait) is the existing move path.

### 5. Host — async submission, drain-based completion

`PyMotionBridge::submit_correction_sequence` / `adjust_motor` become thin async
calls: build the cubic pieces, call `submit_axis_overlay` (enqueue + return).
No inline streaming, no `wait_room`, no `py.detach` blocking loop.
`Motion.submit_correction` / `submit_motor_adjust` drop the
`get_last_move_time`/`dwell`/`wait_moves` timeline machinery entirely. Callers
that need to wait for the buzz to finish (motors_sync before measuring) use
`toolhead.wait_moves()` — which drains via the pump's `DrainSync` and **yields
the reactor** (poll loop with `reactor.pause`), so it never freezes the loop.

### 6. What gets deleted

- `stream_correction_entries` and the inline streaming loop (bridge.rs).
- `PushCorrectionPieces` / `PushCorrectionPiecesResponse` wire messages.
- The separate correction ring: `CORRECTION_RING_DEPTH`,
  `stepping_state::correction_ring`, `correction_armed`,
  `correction_last_step_count`, `dispatch_correction.rs` (folded into the main
  evaluator).
- The separate correction `DrainSync` (`correction_drain`), `reset_axis`,
  `wait_room`, `room`.
- The heartbeat `correction_retired_counts` plumbing added earlier
  (messages.rs, engine.rs `correction_retired_counts`, runtime_ffi,
  kalico_dispatch.c / runtime_tick.c correction-retired wiring).
- `Motion._stream_correction_on_timeline`'s timeline machinery + the
  `motors_sync`/`motor_adjust` band-aids.

## Position-of-record safety

Confirmed: the host reads the per-axis `p_prev` accumulator (engine.rs:452,
sourced from the trajectory, exposed via `motor_state()` →
`kalico_runtime_query_motor_state` → `position_query.rs`), never an individual
stepper's `position_count`. Corrections never advance `p_prev`, so a per-motor
overlay cannot shift homing or any position query. The per-stepper
`position_count` divergence (and `phase_offset` for phase mode) is exactly the
desync offset motors_sync intends, and it persists on the targeted motor only.

## Open questions to resolve first (in the plan)

1. **Multi-bit masks vs `stepper_sel`. [DECIDED — single-bit + fail-loud.]**
   `StepEntry.stepper_sel` today encodes "all (0) or one (idx+1)". Live callers
   (motors_sync, MOTOR_ADJUST, force_move) only ever target one motor. So:
   `mask == 0` → `STEPPER_SEL_ALL`; mask with exactly one bit set → that motor's
   `stepper_sel = idx+1`; **mask with >1 bit set → loud error** at the submit
   boundary (`KALICO_ERR_…`), not silently mishandled. If we ever need multi-bit
   overlays we revisit then (YAGNI). No `stepper_sel` widening, no per-bit step
   entries in this work.
2. **`_reserved` zero-check.** Anything that asserts `_reserved == 0` on decode
   must be relaxed to ignore the mask byte. Audit `piece_ring.rs` and the
   protocol decode.
3. **Drain identity for overlays.** Confirm a masked piece on `(mcu, axis)`
   retires through the same `DrainSync` counter as a normal move on that axis so
   `wait_moves()` correctly waits for buzz completion.

## Testing

- Runtime unit: a `mask == 0` piece advances all steppers' `position_count` and
  `p_prev`; a single-bit-mask piece advances only that stepper's
  `position_count` and leaves `p_prev` unchanged; `stepper_sel` derived
  correctly for both.
- Runtime unit: multi-bit mask is rejected loudly (if we choose single-bit).
- `PieceEntry` round-trips the mask through `to_le_bytes`/`from_le_bytes`; size
  stays 32 bytes.
- Bridge/host: `submit_correction_sequence` returns without blocking (no inline
  wire calls); the pieces reach the pump with the mask stamped.
- Python: `Motion.submit_correction` no longer calls `wait_moves`/`dwell`; the
  buzz is enqueued and the caller drains via `wait_moves`.
- Bench: SYNC runs to completion without `seg0_deficit`/`-142`/`-309`; energize →
  buzz → de-energize no longer overlaps (the original goal).

## Non-goals

- Multi-bit motor masks (deferred unless a caller needs them).
- Changing kinematics, the G5/G5.1 reduce stage, or phase-stepping behavior.
- Reworking the pump's dispatch algorithm — corrections reuse it as-is.
