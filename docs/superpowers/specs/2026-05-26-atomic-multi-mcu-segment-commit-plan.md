# Implementation Plan: Atomic Multi-MCU Segment Commit

**Spec:** `2026-05-26-atomic-multi-mcu-segment-commit.md`

## Build order

Firmware first (MCU Rust + C), then host (motion-bridge). Each task is
independently testable. Tasks within a phase can be parallelized where noted.

---

## Phase 0: Config prerequisite

### Task 0: Reduce H7 pool depth, increase max_pieces_per_curve

**Files:** `.config.h7.bak` on the Pi

Reduce `CONFIG_RUNTIME_CURVE_POOL_N` from 16 to 4 (matching F446). Reallocate
the freed RAM to `CONFIG_RUNTIME_MAX_CONTROL_POINTS` /
`CONFIG_RUNTIME_MAX_KNOT_VECTOR_LEN` so that `max_pieces_per_curve` grows.

**Why this is Phase 0:** The single-pending-slot invariant means each sub-plan
within a logical segment must go through its own push+commit cycle. If the H7
still needs 11 sub-plans per segment, each one is a full multi-MCU round-trip.
Reducing pool depth and increasing piece capacity cuts sub-plans to 1-2 per
segment, making the two-phase protocol practical. Without this, the dispatch
loop becomes `11 × (push_all + commit_all)` per homing move, which is
unacceptably slow.

---

## Phase 1: Firmware — pending slot and protocol primitives

### Task 1: New fields in FgState and SharedState

**Files:** `rust/runtime/src/state.rs`

Add to `FgState`:
- `staged_segment: Option<Segment>` — the foreground-owned pending slot.
  Named `staged_segment` (NOT `pending_segment`, which already exists on
  `IsrState` as the ISR's deferred-parking slot — different ownership, different
  purpose).
- `staged_segment_handles: [CurveHandle; 4]` — curve handles loaded for the
  staged segment (needed for abort reclamation).
- `last_committed_t_end: u64` — the `t_end` of the most recently committed
  segment. Used for chaining (`t_start_clock == 0`). Initialize to 0. Reset
  on flush.

Add to `SharedState`:
- `committed_segment_id: AtomicU32` — the latest committed segment ID.
  Initialize to 0. Reset on flush.

Initialize all new fields in `FgState::new()` / `SharedState::new()`.

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

### Task 3: Modify push_segment — store in staged slot

**Files:** `rust/kalico-c-api/src/runtime_ffi.rs`

Modify `push_segment_impl`:
- Check `fg.staged_segment` is `None`. If `Some`, return
  `ERR_PENDING_SLOT_OCCUPIED`.
- Build the `Segment` struct as before.
- Store in `fg.staged_segment = Some(seg)`.
- Store curve handles in `fg.staged_segment_handles`.
- Do NOT register handles in the retirement table (moved to Task 4 — commit).
- Do NOT call the TIM5 re-enable protocol (moved to Task 4 — commit).
- Do NOT drive the stream state machine transitions (`StreamOpening` →
  `StreamOpenPriming`, `Armed` → `Running`) — moved to Task 4.
- Set `shared.accepted_segment_id` (push ACK cursor — "received into pending").
- Return `accepted_id` in the response as before.

### Task 4: Implement commit_segment FFI

**Files:** `rust/kalico-c-api/src/runtime_ffi.rs`

New `runtime_handle_commit_segment(rt, segment_id, t_start_clock,
duration_clocks, out_segment_id) -> i32`:

- Check `fg.staged_segment` is `Some`. If `None`, return
  `ERR_NO_PENDING_SEGMENT`.
- Check `segment_id` matches staged segment's ID. If not, return
  `ERR_SEGMENT_ID_MISMATCH`.
- Compute timing:
  - If `t_start_clock != 0` (cold start): `t_start = t_start_clock`,
    `t_end = t_start_clock + duration_clocks`.
  - If `t_start_clock == 0` (chaining): `t_start = fg.last_committed_t_end`,
    `t_end = t_start + duration_clocks`.
- Take the segment: `let mut seg = fg.staged_segment.take().unwrap()`.
- Stamp `seg.t_start = t_start`, `seg.t_end = t_end`.
- Register curve handles from `fg.staged_segment_handles` in the retirement
  table (moved from push).
- Enqueue to SPSC: `fg.queue_producer.enqueue(seg)`.
- Update `fg.last_committed_t_end = t_end`.
- Update `shared.committed_segment_id`.
- Drive stream state machine transitions (moved from push):
  `StreamOpening` → `StreamOpenPriming` (capture `first_priming_segment_t_start`
  from the now-known `t_start`), `Armed` → `Running`.
- Run the TIM5 re-enable protocol (moved from push).
- Return `KALICO_OK`.

### Task 5: Implement abort_pending FFI

**Files:** `rust/kalico-c-api/src/runtime_ffi.rs`

New `runtime_handle_abort_pending(rt, segment_id, out_aborted_id) -> i32`:

- If `fg.staged_segment` is `None`: set `*out_aborted_id = 0`, return
  `KALICO_OK` (idempotent).
- If staged segment's ID != `segment_id`: return `ERR_SEGMENT_ID_MISMATCH`.
- Reclaim curve handles from `fg.staged_segment_handles` by calling
  `pool.confirm_retired(handle)` directly on each handle (NOT via the
  retirement table — handles are not registered there until commit, so there
  is nothing to unregister).
- Clear staged slot: `fg.staged_segment = None`.
- Set `*out_aborted_id = segment_id`.
- Return `KALICO_OK`.

### Task 6: C-side dispatch handlers

**Files:** `src/kalico_dispatch.c`

Add `handle_commit_segment(correlation_id, body, body_len)`:
- Validate `body_len == 20` (segment_id:u32 + t_start_clock:u64 +
  duration_clocks:u64).
- Parse fields (little-endian).
- Call `runtime_handle_commit_segment(runtime_handle, ...)`.
- Send `CommitSegmentResponse` (result:i32 + segment_id:u32 = 8 bytes).

Add `handle_abort_pending(correlation_id, body, body_len)`:
- Validate `body_len == 4` (segment_id:u32).
- Parse segment_id.
- Call `runtime_handle_abort_pending(runtime_handle, ...)`.
- Send `AbortPendingResponse` (result:i32 + segment_id:u32 = 8 bytes).

Wire into the dispatch switch on the new message kind constants.

### Task 7: Remove silent rebase, add LATE_ARM fault

**Files:** `rust/runtime/src/tick.rs`

In `isr_sample_tick`, the arm path (search for the `lateness > 0` rebase
block, currently `seg.t_start = now; seg.t_end = seg.t_end.saturating_add(lateness)`):

- Remove the rebase block entirely.
- Add: if `seg.t_start.saturating_add(JITTER_TOLERANCE_CYCLES) < now`, set
  `shared.last_error` to `KALICO_FAULT_LATE_ARM`, set `runtime_status` to
  `Fault`. Return without arming.
- Define `JITTER_TOLERANCE_CYCLES` as `3 * sample_period_cycles` (3 ISR ticks).

### Task 8: Capability bit + flush update

**Files:** `rust/runtime/src/state.rs` (or identify response builder),
`src/kalico_dispatch.c` (identify handler), `rust/runtime/src/stream.rs`

- Add `SEGMENT_COMMIT_CAPABLE` (bit 1) to the capabilities bitmap in
  `IdentifyResponse`.
- Update `stream::flush` (and `runtime_force_idle` path) to clear
  `fg.staged_segment`, release any staged curve handles via
  `pool.confirm_retired`, and reset `fg.last_committed_t_end = 0` and
  `shared.committed_segment_id = 0`. This prevents a stale staged segment
  from blocking the next push after a fault-recovery flush.

---

## Phase 2: Host — dispatch restructuring

### Task 9: Producer wire calls

**Files:** `rust/kalico-host-rt/src/producer.rs` (or equivalent)

Add `commit_segment(io, segment_id, t_start_clock, duration_clocks, timeout)
-> Result<CommitSegmentInfo>`.

Add `abort_pending(io, segment_id, timeout) -> Result<AbortPendingInfo>`.

Both follow the existing `push_segment` pattern: build wire frame, send via
`kalico_call`, parse response.

### Task 10: Restructure dispatch closure

**Files:** `rust/motion-bridge/src/bridge.rs`

The dispatch closure currently processes one segment at a time with sub-plans
serialized per-MCU. Restructure to two-phase with sub-plan-level commit:

```
for each logical segment:
    plans = build_push_params(...)  // one plan per MCU (may have N sub-plans)
    max_subs = max(plan.sub_count for plan in plans)

    for sub_idx in 0..max_subs:
        // Phase 1: push sub_idx to all MCUs
        for each mcu:
            if sub_idx < mcu.plan.sub_count:
                alloc slots → load_curve → push_segment
            else:
                push idle segment (UNUSED handles, no load_curve)
            collect push ACK
        if any push failed:
            abort_pending on all MCUs that ACK'd
            release orphaned curve slots
            return error

        // Phase 2: commit sub_idx on all MCUs
        compute timing (cold start for first sub, chain for rest)
        for each mcu:
            commit_segment(mcu, segment_id, t_start_clock, duration_clocks)
            collect commit ACK
        if any commit failed:
            emergency_stop all MCUs
            return error
```

Each sub-plan is a separate segment ID — the host assigns sequential IDs
across sub-plans. MCUs that have fewer sub-plans than the max receive idle
segments for the remaining sub-indices. This preserves the lockstep invariant
at sub-plan granularity.

Note: with Task 0 (pool depth reduction), `max_subs` should be 1-2 for
typical moves, making the inner loop trivial.

### Task 11: Idle segment dispatch

**Files:** `rust/motion-bridge/src/dispatch.rs`, `rust/motion-bridge/src/bridge.rs`

Remove the `is_trivially_constant` skip logic and the `all_constant → continue`
branch from `build_push_params`. Every MCU in the motion group always gets a
plan. MCUs with all-constant axes get a plan with zero `curves_to_load`.

In the dispatch closure (Task 10), MCUs with no curves for a given sub-plan
get `push_segment` with all handles = UNUSED, no `load_curve` calls.

Verify that `Segment::compute_consumers_remaining` returns 0 for all-UNUSED
handles (it already does — confirm with a unit test in Task 15).

### Task 12: Cold-start and chaining timing

**Files:** `rust/motion-bridge/src/bridge.rs`

In the dispatch closure's Phase 2:
- Track `is_cold_start` (first segment, or previous segment drained, or
  continuity break detected via `schedule_state`).
- Cold start: per MCU, `t_start_clock = clock_sync.translate(host_now + lead_s)`.
  `lead_s = N_mcus * per_mcu_commit_rtt_estimate + margin` (sum, not max —
  commits are sequential).
- Chaining: `t_start_clock = 0`. MCU handles the rest.
- `duration_clocks = ((seg.t_end - seg.t_start) * mcu_freq).round() as u64`
  per MCU. Computed once per logical segment, reused across sub-plans (duration
  per sub-plan is the sub-plan's own time window, not the full segment's).

Remove the existing `mcu_base_clock` / `schedule_state` / `now_plus_lead`
computation from the push path (timing no longer lives there).

### Task 13: Credit counter restructuring

**Files:** `rust/motion-bridge/src/bridge.rs`, `rust/kalico-host-rt/src/credit.rs`

The existing `CreditCounter` consumes a credit on `push_segment` (when the
segment enters the SPSC queue). Under the two-phase protocol, the segment
enters the SPSC queue on commit, not push. Restructure:

- Credit is consumed on `commit_segment`, not `push_segment`.
- The staged segment (in the pending slot) does not consume a credit — it is
  not yet in the SPSC queue.
- Credit is freed on segment retirement (`kalico_credit_freed`), unchanged.

This means the credit counter tracks SPSC queue occupancy only, which is
correct — the pending slot is a single-element staging area outside the queue.

### Task 14: Feature detection and error handling

**Files:** `rust/motion-bridge/src/bridge.rs`

Feature detection:
- At `init_planner` time, check `SEGMENT_COMMIT_CAPABLE` bit on all bridge
  MCUs.
- If all have it: use two-phase dispatch.
- If any lack it: fall back to legacy single-phase dispatch (existing code
  path, preserved as-is). Log a warning.

Error handling:
- Push failure: send `abort_pending` to all MCUs that ACK'd push. Release any
  orphaned curve slots on MCUs where `load_curve` succeeded but `push_segment`
  was never sent (call `slot_pool.release(slot)` directly). Return
  `DispatchError`.
- Commit failure: send `emergency_stop` to all MCUs via existing
  `bridge_send`. Return unrecoverable `DispatchError`.

---

## Phase 3: Validation

### Task 15: Unit tests — firmware

- `push_segment` stores in `staged_segment`, does not enqueue to SPSC.
- `commit_segment` moves staged to SPSC queue, stamps timing.
- `commit_segment` with empty staged slot returns `ERR_NO_PENDING_SEGMENT`.
- `commit_segment` with wrong ID returns `ERR_SEGMENT_ID_MISMATCH`.
- `push_segment` with occupied staged slot returns `ERR_PENDING_SLOT_OCCUPIED`.
- `abort_pending` clears staged slot, reclaims curves via `pool.confirm_retired`.
- `abort_pending` with wrong ID returns `ERR_SEGMENT_ID_MISMATCH`.
- `abort_pending` on empty slot is no-op (returns ID 0).
- Late arm faults instead of rebasing.
- Flush clears staged slot, resets `last_committed_t_end` and
  `committed_segment_id`.
- `compute_consumers_remaining` returns 0 for all-UNUSED handles (idle
  segment retires in one tick).
- Two consecutive commits with `t_start_clock = 0` produce contiguous timing
  (`seg2.t_start == seg1.t_end`).
- Chaining after cold-start preserves duration.

### Task 16: Unit tests — host

- Dispatch closure pushes all MCUs before committing.
- Idle MCU receives push with UNUSED handles.
- Push failure triggers `abort_pending` on all MCUs that ACK'd.
- Commit failure triggers `emergency_stop`.
- Cold-start timing is in the future per MCU.
- Chaining uses `t_start_clock = 0`.
- Credit consumed on commit, not push.
- Sub-plan splitting: each sub is a separate push+commit cycle.

### Task 17: Integration test — bench

- Flash both MCUs. G28 X Y Z completes.
- Z jog works.
- Z homing works (Beacon proximity).
- No `SLOT POOL FULL` during homing.
- No `LATE_ARM` during normal operation.
- `bridge_diag.log` shows push+commit for both MCUs on every segment.
- Verify pipeline depth: `committed_segment_id - retired_through_segment_id`
  never exceeds `min(curve_pool_n, spsc_queue_depth) - 1`.

### Task 18: Fault injection test — bench

- Temporarily reduce SPSC queue depth to 1 to trigger push rejection.
  Verify `AbortPending` fires and the next segment proceeds normally.
- Inject a stale segment ID in commit to trigger `ERR_SEGMENT_ID_MISMATCH`.
  Verify emergency_stop fires.

---

## Dependencies

```
Task 0 (config) ── no code dependency, can be done anytime before Task 17
Task 1 ← Task 3, Task 4, Task 5, Task 8
Task 2 ← Task 3, Task 4, Task 5, Task 6
Task 3 ← Task 4 (push must stage before commit can move)
Task 4 ← Task 6 (FFI before C handler)
Task 5 ← Task 6
Task 7 ── independent (ISR change, no dependency on pending slot)
Task 8 ── depends on Task 1 (flush clears staged slot)
Task 9 ← Task 10 (wire calls before dispatch restructuring)
Task 10 ← Task 11, Task 12, Task 13, Task 14
Tasks 15-18 ← all of above
```

## Risks

| Risk | Mitigation |
|------|-----------|
| F446 RAM at 100% — ~84 bytes net new | Profile after Task 1. If tight, reduce `SharedState` diagnostic fields added in this session (push_t_start_lo/hi, push_widened_lo/hi — 16 bytes reclaimable). |
| Sub-plan count still >1 after Task 0 | Task 10's inner loop handles it correctly. Worst case is slower dispatch, not incorrect behavior. |
| Legacy fallback preserves old bugs | Acceptable — only for mixed-capability setups during transition. |
| SPSC queue depth < curve pool on some configs | Task 10 should assert `spsc_queue_depth >= curve_pool_n` at init and log warning if violated. |
