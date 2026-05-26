# Implementation Plan: Atomic Multi-MCU Segment Commit

**Spec:** `2026-05-26-atomic-multi-mcu-segment-commit.md`

## Build order

Firmware first (MCU Rust + C), then host (motion-bridge). Each task is
independently testable. Tasks within a phase can be parallelized where noted.

---

## Phase 1: Firmware — pending slot and protocol primitives

### Task 1: Pending segment slot in FgState

**Files:** `rust/runtime/src/state.rs`

Add `pending_segment: Option<Segment>` to `FgState`. Initialize as `None`.
Add `pending_segment_handles: [CurveHandle; 4]` to track curve handles
registered for the pending segment (needed for abort reclamation).

No behavioral change yet — push_segment still enqueues directly.

### Task 2: Wire protocol constants

**Files:** `rust/kalico-host-rt/src/kalico_native.rs` (or equivalent protocol
constants file), `src/kalico_dispatch.h`

Add message kind constants:
- `KALICO_MSG_COMMIT_SEGMENT` (request)
- `KALICO_MSG_COMMIT_SEGMENT_RESPONSE`
- `KALICO_MSG_ABORT_PENDING` (request)
- `KALICO_MSG_ABORT_PENDING_RESPONSE`

Add error codes:
- `KALICO_ERR_PENDING_SLOT_OCCUPIED`
- `KALICO_ERR_NO_PENDING_SEGMENT`
- `KALICO_ERR_SEGMENT_ID_MISMATCH`

Add fault code:
- `KALICO_FAULT_LATE_ARM = 0x0010`

### Task 3: Modify push_segment — store in pending slot

**Files:** `rust/kalico-c-api/src/runtime_ffi.rs`

Modify `push_segment_impl`:
- Check pending slot is empty. If occupied, return `ERR_PENDING_SLOT_OCCUPIED`.
- Build the `Segment` struct as before, but store it in
  `fg.pending_segment = Some(seg)` instead of `fg.queue_producer.enqueue(seg)`.
- Store curve handles in `fg.pending_segment_handles`.
- Register handles in the retirement table (existing code, unchanged).
- Do NOT call the TIM5 re-enable protocol (that moves to commit).
- Do NOT advance `accepted_segment_id_seen` / `accepted_segment_id` on
  SharedState — move those to commit. Return `accepted_id` in the response
  as before (it's the push ACK, not the commit ACK).

Wait — actually `accepted_segment_id` should be set on push (it means
"received into pending"), per spec. Keep the store. Add a separate
`committed_segment_id` atomic to SharedState (Task 1) that commit sets.

### Task 4: Implement commit_segment FFI

**Files:** `rust/kalico-c-api/src/runtime_ffi.rs`, `rust/runtime/src/state.rs`

New `runtime_handle_commit_segment(rt, segment_id, t_start_clock, duration_clocks, out_segment_id) -> i32`:

- Check pending slot is non-empty. If empty, return `ERR_NO_PENDING_SEGMENT`.
- Check `segment_id` matches pending segment's ID. If not, return
  `ERR_SEGMENT_ID_MISMATCH`.
- Compute timing: if `t_start_clock != 0`, use directly (cold start) and
  `t_end = t_start_clock + duration_clocks`. If `t_start_clock == 0`, chain:
  `t_start = fg.last_committed_t_end`, `t_end = t_start + duration_clocks`.
- Take the segment from pending slot (`fg.pending_segment.take()`).
- Stamp `t_start` and `t_end` onto the segment.
- Enqueue to SPSC: `fg.queue_producer.enqueue(seg)`.
- Update `fg.last_committed_t_end = t_end`.
- Update `shared.committed_segment_id`.
- Run the TIM5 re-enable protocol (moved from push_segment_impl).
- Return `KALICO_OK`.

### Task 5: Implement abort_pending FFI

**Files:** `rust/kalico-c-api/src/runtime_ffi.rs`

New `runtime_handle_abort_pending(rt, segment_id, out_aborted_id) -> i32`:

- If pending slot is empty: set `*out_aborted_id = 0`, return `KALICO_OK`.
- If pending slot's ID != `segment_id`: return `ERR_SEGMENT_ID_MISMATCH`.
- Reclaim curve handles from `fg.pending_segment_handles` via the retirement
  table (unregister + release slots).
- Clear pending slot: `fg.pending_segment = None`.
- Set `*out_aborted_id = segment_id`.
- Return `KALICO_OK`.

### Task 6: C-side dispatch handlers

**Files:** `src/kalico_dispatch.c`

Add `handle_commit_segment(correlation_id, body, body_len)`:
- Parse: `segment_id (u32)`, `t_start_clock (u64)`, `duration_clocks (u64)` =
  20 bytes.
- Call `runtime_handle_commit_segment(runtime_handle, ...)`.
- Send `CommitSegmentResponse`.

Add `handle_abort_pending(correlation_id, body, body_len)`:
- Parse: `segment_id (u32)` = 4 bytes.
- Call `runtime_handle_abort_pending(runtime_handle, ...)`.
- Send `AbortPendingResponse`.

Wire into the dispatch switch on the new message kind constants.

### Task 7: Remove silent rebase, add LATE_ARM fault

**Files:** `rust/runtime/src/tick.rs`

In `isr_sample_tick`, the arm path (around line 1283):
- Remove the `if lateness > 0 { seg.t_start = now; seg.t_end += lateness; }`
  block.
- Add: if `seg.t_start + JITTER_TOLERANCE_CYCLES < now`, set
  `shared.last_error` to `KALICO_FAULT_LATE_ARM`, set `runtime_status` to
  `Fault`. Return without arming.
- Define `JITTER_TOLERANCE_CYCLES` as `3 * sample_period_cycles` (3 ISR ticks).

### Task 8: Capability bit

**Files:** `rust/runtime/src/state.rs` (or identify response builder),
`src/kalico_dispatch.c` (identify handler)

Add `SEGMENT_COMMIT_CAPABLE` (bit 1) to the capabilities bitmap in
`IdentifyResponse`. Set it when the firmware supports the two-phase protocol
(i.e., always in this build — it's a compile-time constant).

---

## Phase 2: Host — dispatch restructuring

### Task 9: Producer wire calls

**Files:** `rust/kalico-host-rt/src/producer.rs` (or equivalent)

Add `commit_segment(io, segment_id, t_start_clock, duration_clocks, timeout) -> Result<CommitSegmentInfo>`.

Add `abort_pending(io, segment_id, timeout) -> Result<AbortPendingInfo>`.

Both follow the existing `push_segment` pattern: build wire frame, send via
`kalico_call`, parse response.

### Task 10: Restructure dispatch closure

**Files:** `rust/motion-bridge/src/bridge.rs`

The dispatch closure currently processes one segment at a time:
```
for each mcu_plan:
    for each sub:
        alloc → load → push
```

Restructure to:
```
for each logical segment:
    // Phase 1: push all MCUs
    for each mcu:
        build plan (idle if no displacement)
        for each sub in plan:
            alloc → load_curve → push_segment
        collect push ACK
    if any push failed:
        abort_pending on all MCUs that ACK'd
        release orphaned curve slots
        return error

    // Phase 2: commit all MCUs
    compute timing (cold start or chain)
    for each mcu:
        commit_segment(mcu, segment_id, t_start_clock, duration_clocks)
        collect commit ACK
    if any commit failed:
        emergency_stop all MCUs
        return error
```

### Task 11: Idle segment dispatch

**Files:** `rust/motion-bridge/src/dispatch.rs`, `rust/motion-bridge/src/bridge.rs`

Remove the `is_trivially_constant` skip logic and the `all_constant → continue`
branch from `build_push_params`. Every MCU in the motion group always gets a
plan. MCUs with all-constant axes get a plan with zero `curves_to_load` (no
`load_curve` needed), but `push_segment` is still sent with all handles =
UNUSED.

### Task 12: Cold-start and chaining timing

**Files:** `rust/motion-bridge/src/bridge.rs`

In the dispatch closure's Phase 2:
- Track `is_cold_start` (first segment, or previous segment drained).
- Cold start: `t_start_clock = clock_sync.translate(host_now + lead_s)` per
  MCU. `lead_s = sum(per_mcu_commit_rtt_estimate) + margin`.
- Chaining: `t_start_clock = 0`. MCU handles the rest.
- `duration_clocks = (seg.t_end - seg.t_start) * mcu_freq` per MCU.

Remove the existing `mcu_base_clock` / `schedule_state` / `now_plus_lead`
computation from the push path (timing no longer lives there).

### Task 13: Feature detection

**Files:** `rust/motion-bridge/src/bridge.rs`

At `init_planner` time, after `attach_serial` completes:
- Check `SEGMENT_COMMIT_CAPABLE` bit on all bridge MCUs.
- If all have it: use two-phase dispatch.
- If any lack it: fall back to legacy single-phase dispatch (existing code
  path, preserved as-is). Log a warning.
- Store the decision in a flag on `PyMotionBridge` that the dispatch closure
  reads.

### Task 14: Error handling

**Files:** `rust/motion-bridge/src/bridge.rs`

- Push failure: send `abort_pending` to all MCUs that ACK'd push. Release any
  orphaned curve slots on MCUs where `load_curve` succeeded but `push_segment`
  was never sent. Return `DispatchError`.
- Commit failure: send `emergency_stop` to all MCUs. Return unrecoverable
  `DispatchError`.

---

## Phase 3: Validation

### Task 15: Unit tests — firmware

- `push_segment` stores in pending, does not enqueue.
- `commit_segment` moves pending to queue.
- `commit_segment` with empty pending returns error.
- `commit_segment` with wrong ID returns error.
- `push_segment` with occupied pending returns error.
- `abort_pending` clears slot, reclaims curves.
- `abort_pending` with wrong ID returns error.
- `abort_pending` on empty slot is no-op.
- Late arm faults instead of rebasing.

### Task 16: Unit tests — host

- Dispatch closure pushes all MCUs before committing.
- Idle MCU receives push with UNUSED handles.
- Push failure triggers abort on all.
- Commit failure triggers emergency_stop.
- Cold-start timing is in the future.
- Chaining uses `t_start_clock = 0`.

### Task 17: Integration test — bench

- Flash both MCUs. G28 X Y Z completes.
- Z jog works.
- Z homing works (Beacon proximity).
- No `SLOT POOL FULL` during homing.
- No `LATE_ARM` during normal operation.
- bridge_diag.log shows push+commit for both MCUs on every segment.

---

## Dependencies and risks

| Risk | Mitigation |
|------|-----------|
| Pending slot adds ~56 bytes to FgState (F446 RAM at 100%) | Offset by removing fields made redundant (the TIM5 re-enable state that moved to commit). Profile after Task 3. |
| `is_trivially_constant` removal sends more segments to F446 | The idle segments have no `load_curve` cost and retire in one tick. Net load is lower than the current spurious Z-curve dispatch. |
| Legacy fallback (Task 13) preserves old bugs | Acceptable — the fallback is only for mixed-capability setups during transition. New firmware will always have the capability bit. |
| SPSC queue depth may be smaller than curve pool on some configs | Task 10 should assert `spsc_queue_depth >= curve_pool_n` at init time or log a warning. |
