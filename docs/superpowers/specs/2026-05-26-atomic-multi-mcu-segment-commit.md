# Atomic Multi-MCU Segment Commit

**Date:** 2026-05-26  
**Status:** Draft  
**Branch:** fix-z-motion

## Problem

The dispatch closure pushes segments to MCUs sequentially: all sub-plans for
MCU 0 (H7), then all sub-plans for MCU 1 (F446). Each MCU's engine starts
executing as soon as a segment's `t_start_clock` arrives — there is no barrier
between "segment received" and "segment executing."

When MCU 0's curve pool fills (e.g., 11 sub-plans for a Z homing move's XY
shaper output exceeds 16 slots), MCU 1's push is blocked behind MCU 0's
retirement. MCU 0 starts executing while MCU 1 has received nothing. The axes
desynchronize.

Additionally, the existing ISR silently rebases late segments forward
(`seg.t_start = now; seg.t_end += lateness`). In a multi-MCU setup this
produces desynchronized trajectories: one axis starts on time, another starts
late, the toolhead path deviates, and nothing reports it.

Observed failure: Z homing move dispatched to both H7 (XY hold) and F446 (Z
descent). H7 received 7 of 11 sub-plans, then its pool filled. F446 never
received its single sub-plan. Z never moved. The user had to pull the plug.

## Goal

A logical segment either commits on all participating MCUs or on none.
No MCU executes a segment until every MCU has confirmed receipt.

## Architecture

### Invariant

The host MUST NOT push segment N+1 to any MCU until segment N has been
committed on all MCUs or aborted on all MCUs.

Each MCU has exactly one pending slot. The pending slot is either empty or
holds one uncommitted segment. There is no pending queue, no batch buffer, no
out-of-order commit.

### Segment ID

Segment IDs are assigned by the host. They are globally unique — all MCUs
receive the same segment ID for the same logical segment. IDs are monotonically
increasing, starting at 1 (ID 0 is reserved as "no segment"). The host is the
sole assigner. Gaps in the ID sequence are permitted (an aborted segment
consumes an ID that is never committed).

### Two-phase segment dispatch

**Phase 1 — Push.** For each logical segment from the planner:

1. Build per-MCU plans (existing `build_push_params`).
2. For MCUs with no displacement on their axes: build an **idle segment** —
   `push_segment` with all handles = `UNUSED_SENTINEL`, no `load_curve` calls.
3. For each MCU: `load_curve` (where needed), then `push_segment`.
   Each MCU ACKs with `PushSegmentResponse`.
4. Collect all ACKs. If any MCU rejects or times out: send `AbortPending` to
   all MCUs that ACK'd (to clear their pending slots and reclaim any loaded
   curve slots). Report the fault. Do not proceed to Phase 2.

**Phase 2 — Commit.** Once all MCUs have ACK'd segment N:

5. Compute timing (see §Timing below).
6. Send `CommitSegment` to all MCUs. Each MCU ACKs with
   `CommitSegmentResponse`.
7. Collect all commit ACKs. If any MCU rejects or times out, the situation is
   unrecoverable (some MCUs may have already enqueued the segment). The host
   issues `emergency_stop` to all MCUs. This is an e-stop-equivalent fault.

### Segment ID cursors

Three monotonic cursors track each segment's lifecycle per MCU:

- **`accepted_segment_id`** — set on `PushSegmentResponse`. The MCU has the
  segment data in its pending slot.
- **`committed_segment_id`** — set on `CommitSegmentResponse`. The segment is
  in the SPSC queue, available for ISR dequeue.
- **`retired_through_segment_id`** — set on segment retirement
  (`kalico_credit_freed`). The segment has been fully evaluated and its
  resources freed.

With the single-pending-slot invariant, `accepted` is at most 1 ahead of
`committed`. Monotonicity is `id > last_accepted_id` (not strict sequential —
gaps from aborted segments are permitted).

### Timing: "what" in the push, "when" in the commit

`PushSegment` carries the segment data: curve handles, kinematics, segment ID.
No absolute clock values. The `t_start` and `t_end` fields in the existing
wire format are zeroed and ignored. The segment describes *what* to execute.

`CommitSegment` carries the timing:

- `t_start_clock: u64` — nonzero for cold start (absolute MCU clock); zero for
  continuous streaming (chain from previous segment's `t_end`).
- `duration_clocks: u64` — segment duration in MCU clock cycles. Same value in
  both cold-start and chaining modes. The MCU always computes
  `t_end = t_start + duration_clocks`.

The segment duration is invariant — it is a property of the trajectory, not of
wall-clock time. Using `duration_clocks` instead of an absolute `t_end_clock`
eliminates dual-semantic ambiguity.

**Cold-start `t_start_clock` computation.** The host computes `t_start_clock`
per MCU after all pushes complete:

```
for each mcu:
    host_now = monotonic_clock()
    t_start_clock[mcu] = clock_sync[mcu].translate(host_now + lead_host_s)
```

`lead_host_s` is in host-time seconds. It must cover the time to send
`CommitSegment` to ALL MCUs sequentially:
`lead_host_s = sum(per_mcu_commit_rtt) + clock_sync_error_bound + safety_margin`.
For the MVP with 2-3 MCUs over USB: `lead_host_s ≈ 5ms`. Each MCU's
`t_start_clock` is independently translated via its own clock sync regression,
so clock sync error is handled per-MCU.

### Idle segments

Every MCU receives every logical segment, even when it has no curves to
evaluate.

- The commit protocol is uniform — no "does this MCU participate?" branching.
- Pipeline depth is symmetric across MCUs (bounded by the shallowest pool).
- The `is_trivially_constant` skip logic is removed from the dispatch path.
- MCUs always advance their segment counter in lockstep.

An idle segment is a regular segment with all curve handles = UNUSED. Same
commit flow, same slot lifecycle, same retirement. From the MCU's perspective
there is no such thing as an "idle segment" — it is a segment where every axis
evaluates to nothing.

**Idle segment retirement.** An idle segment has `consumers_remaining = 0` at
construction (no axis has a real curve handle). The ISR dequeues it, sees no
consumers, and retires it immediately — it occupies the current-segment slot
for one ISR tick, advances the segment counter, and fires `credit_freed`. This
is the correct behavior: the idle segment's only purpose is to advance the
lockstep counter and free the pipeline slot promptly. No minimum-lifetime
guarantee is needed.

### Steady-state pipeline

In continuous streaming, the pipeline depth D is bounded by:

```
D = min(shallowest_mcu_curve_pool_n, shallowest_mcu_spsc_queue_depth) - 1
```

(The `-1` accounts for the segment currently being executed.) The host
maintains up to D committed segments in each MCU's SPSC queue. The pipeline
operates as:

1. MCU retires segment N → `credit_freed` → slot opens.
2. Host pushes segment N+D+1 to all MCUs (filling the freed slot).
3. Host commits segment N+D+1.
4. MCU is already executing N+1, segments N+2 through N+D+1 are queued.
5. Segments chain by `t_end → t_start` automatically.

The commit is sent well ahead of execution — the newly committed segment sits
behind D-1 already-committed segments. The commit is not on the critical path
as long as segment duration >> commit round-trip time, which holds for all
practical print speeds.

### MCU-side guards

The MCU enforces the protocol with four hard guards:

1. **Push with pending slot occupied → reject.** The host violated the
   invariant (pushed N+1 before committing or aborting N).
   `PushSegmentResponse` returns `ERR_PENDING_SLOT_OCCUPIED`.

2. **Commit with empty pending slot → reject.** No preceding push.
   `CommitSegmentResponse` returns `ERR_NO_PENDING_SEGMENT`.

3. **Commit with mismatched segment ID → reject.** The commit's `segment_id`
   does not match the pending slot's ID. Indicates a host accounting bug or
   memory corruption. `CommitSegmentResponse` returns
   `ERR_SEGMENT_ID_MISMATCH`.

4. **Late arm → hard fault.** If `t_start_clock < now - JITTER_TOLERANCE` at
   ISR arm time, the engine transitions to Fault with fault code `LATE_ARM`
   (value `0x0010`). `JITTER_TOLERANCE` is a small number of ISR ticks (e.g.,
   2-3 ticks at the sample rate) to absorb normal scheduling jitter. Any
   lateness beyond that stops the machine. The existing silent-rebase logic is
   removed entirely. The `LATE_ARM` fault is reported asynchronously via the
   `kalico_status_v6` frame's `last_fault` field (same mechanism as existing
   engine faults). The host's status poller detects it and issues
   `emergency_stop`.

Note: excessively early `t_start_clock` (e.g., `now + 10 seconds`) is not
guarded at the MCU level — it causes a long stall but is not unsafe. The host
is responsible for computing reasonable lead times.

### Abort protocol

`AbortPending` clears the pending slot on a single MCU without committing.
Used by the host when Phase 1 fails on one MCU and the others need their
pending slots cleared before the next segment can be pushed.

The request carries `segment_id` as a safety interlock — the MCU verifies it
matches the pending slot's ID before aborting. If the IDs don't match (host
accounting bug, reorder), the MCU returns `ERR_SEGMENT_ID_MISMATCH` and does
not clear the slot.

- If the pending slot holds a segment with matching ID: clears it, reclaims
  any curve pool slots that were loaded for that segment (via the retirement
  table), returns `AbortPendingResponse` with the aborted segment ID.
- If the pending slot is empty: no-op, returns success with segment ID 0
  (regardless of the requested ID — idempotent).

The host sends `AbortPending` to ALL MCUs on any Phase 1 failure, ensuring
every MCU's pending slot is clean before the next push cycle.

### Curve slot reclamation on failure

If `load_curve` succeeds on MCU 0 but `push_segment` or `load_curve` fails on
MCU 1:

- The host sends `AbortPending` to MCU 0. The abort handler reclaims the curve
  slots loaded for the failed segment via the retirement table (same mechanism
  used for normal segment retirement).
- Curve slots are only associated with a segment after `push_segment` succeeds
  (the push registers them in the retirement table). If `push_segment` was
  never sent (because `load_curve` failed on the same MCU), the host must
  explicitly release the loaded slots via the existing slot pool release path.

### Pool depth consequences

The effective in-flight depth is bounded by
`min(curve_pool_n, spsc_queue_depth) - 1` on the shallowest MCU. If the F446
has `CURVE_POOL_N=4` and the H7 has `CURVE_POOL_N=16`, the system can only
sustain 3 committed segments in flight (F446 is the bottleneck). The H7's
extra 12 slots are wasted capacity.

**Recommendation:** reduce H7 `CURVE_POOL_N` to match F446 (or a small
multiple), and reallocate the freed RAM to `max_pieces_per_curve`. More pieces
per curve means fewer sub-plans per segment (fewer wire round-trips), which
directly reduces the dispatch time for Phase 1. This is the change that
eliminates the root cause: the Z homing move needed 11 sub-plans only because
`max_pieces_per_curve` was too small relative to the shaped curve's piece
count. With deeper curves and shallower pools, the same move fits in 1-2
sub-plans.

### Feature detection

`CommitSegment` and `AbortPending` are new wire messages. Old firmware does not
recognize them. This is a breaking change that requires coordinated host +
firmware deployment. The host detects MCU capability via the
`IdentifyResponse` capabilities bitmap (existing mechanism): a new
`SEGMENT_COMMIT_CAPABLE` bit (bit 1) indicates the MCU supports the two-phase
protocol. If any bridge-attached MCU lacks this bit, the host falls back to the
legacy single-phase path (direct enqueue on push, no commit, no abort) and logs
a warning. Mixed-capability setups are not supported for synchronized motion —
all MCUs in a motion group must support the two-phase protocol.

### Failure modes

| Scenario | Behavior |
|----------|----------|
| All MCUs ACK push + commit | Execution proceeds |
| One MCU rejects push (pending slot occupied) | Host sends AbortPending to all, faults |
| One MCU transport timeout on push | Host sends AbortPending to all that ACK'd, faults |
| `load_curve` fails on one MCU | Host sends AbortPending to all that ACK'd, releases orphaned curve slots, faults |
| One MCU rejects commit (empty pending / id mismatch) | Unrecoverable — other MCUs may have committed. Host issues emergency_stop |
| One MCU transport timeout on commit | Unrecoverable — host issues emergency_stop |
| Late arm on any MCU (`t_start < now - jitter`) | MCU hard faults with `LATE_ARM` (0x0010), reported via `kalico_status_v6` `last_fault` |
| One MCU hangs after commit | Other MCUs execute up to D committed segments (D = pipeline depth), then stall. Host detects via status polling and faults |

## Wire protocol

### New message: `CommitSegment`

```
CommitSegment {
    segment_id: u32,       // matches the id from PushSegment (global)
    t_start_clock: u64,    // absolute MCU clock (cold start), or 0 (chain)
    duration_clocks: u64,  // segment duration in MCU clock cycles
}
```

Response:

```
CommitSegmentResponse {
    result: i32,       // 0 = OK, negative = error code
    segment_id: u32,   // echo back for correlation
}
```

Error codes: `ERR_NO_PENDING_SEGMENT`, `ERR_SEGMENT_ID_MISMATCH`.

### New message: `AbortPending`

```
AbortPending {
    segment_id: u32,   // expected pending segment ID (safety interlock)
}
```

Response:

```
AbortPendingResponse {
    result: i32,       // 0 = OK, negative = error code
    segment_id: u32,   // ID of the aborted segment, or 0 if slot was empty
}
```

Error codes: `ERR_SEGMENT_ID_MISMATCH` (pending slot holds a different ID).

### Modified behavior: `PushSegment`

`push_segment` no longer enqueues the segment to the SPSC queue, and no longer
carries meaningful timing. Instead:

- Checks pending slot is empty. If occupied, returns
  `ERR_PENDING_SLOT_OCCUPIED` (guard 1).
- Validates the segment (existing checks: `id > last_accepted_id`, valid
  kinematics, etc.). Gaps in the ID sequence are permitted.
- Stores curve handles, kinematics, and segment ID into the pending slot.
- Returns `PushSegmentResponse` with `accepted_id`.
- The segment is NOT visible to the ISR until `CommitSegment` arrives.

The `t_start` and `t_end` fields in the existing wire format are zeroed and
ignored (no wire format change, avoids protocol version bump).

### Modified behavior: `CommitSegment`

- Checks pending slot is non-empty (guard 2). If empty, returns
  `ERR_NO_PENDING_SEGMENT`.
- Checks segment ID matches (guard 3). If mismatched, returns
  `ERR_SEGMENT_ID_MISMATCH`.
- Computes absolute timing: if `t_start_clock != 0`, uses it directly (cold
  start). If `t_start_clock == 0`, sets
  `t_start = previous_segment_t_end` (chaining). In both cases,
  `t_end = t_start + duration_clocks`.
- Moves the segment from the pending slot to the SPSC queue.
- Clears the pending slot.
- Runs the existing re-enable protocol (TIM5 arm on Idle→Running transition).
- Returns `CommitSegmentResponse`.

### Modified behavior: `AbortPending`

- Checks `segment_id` matches pending slot (if non-empty). If mismatched,
  returns `ERR_SEGMENT_ID_MISMATCH`.
- If pending slot is non-empty and ID matches: reclaims loaded curve pool
  slots via the retirement table, clears the pending slot, returns the aborted
  segment ID.
- If pending slot is empty: no-op, returns segment ID 0.

## Implementation scope

### Firmware (MCU) — `src/` and `rust/runtime/`

1. Add pending segment slot to `FgState` (foreground-owned, not ISR-visible).
2. Modify `runtime_handle_push_segment`: store in pending slot instead of SPSC
   enqueue. Reject if pending slot occupied (`ERR_PENDING_SLOT_OCCUPIED`).
3. New `runtime_handle_commit_segment` FFI: validate pending slot and segment
   ID, stamp timing, move to SPSC queue, run re-enable protocol.
4. New `runtime_handle_abort_pending` FFI: validate segment ID, clear pending
   slot, reclaim curve slots.
5. New kalico-native dispatch handlers in `src/kalico_dispatch.c`:
   `handle_commit_segment` and `handle_abort_pending` — parse wire frames,
   call FFI, send responses.
6. Wire protocol registration (message kind constants, response kinds).
7. Remove silent-rebase logic from ISR arm path. Replace with `LATE_ARM`
   fault (0x0010), reported via `kalico_status_v6` `last_fault` field.
8. Add `SEGMENT_COMMIT_CAPABLE` bit (bit 1) to `IdentifyResponse` capabilities
   bitmap.

### Host (motion-bridge) — `rust/motion-bridge/`

1. Restructure dispatch closure: for each logical segment, push all MCUs
   first (including idle segments), collect all push ACKs, then commit all
   MCUs, collect all commit ACKs.
2. Add `producer::commit_segment` and `producer::abort_pending` calls.
3. Idle segment support: `push_segment` with all handles UNUSED for
   non-participating MCUs. Remove `is_trivially_constant` skip logic and
   `all_constant` skip.
4. Cold-start timing: compute `t_start_clock` per MCU via clock sync after all
   pushes complete, using `lead_host_s = sum(per_mcu_commit_rtt) + margin`.
5. Continuous-streaming timing: send `t_start_clock = 0` for chained segments.
6. Error handling: on push failure, `AbortPending` to all MCUs that ACK'd,
   release orphaned curve slots, report fault. On commit failure,
   `emergency_stop`.
7. Feature detection: check `SEGMENT_COMMIT_CAPABLE` bit on all bridge MCUs
   at `init_planner` time. Fall back to legacy single-phase dispatch if any
   MCU lacks it. Log warning for mixed-capability setups.

### What doesn't change

- ISR dequeue + evaluation path (still dequeues from SPSC, same
  `isr_sample_tick`).
- Curve loading (`load_curve` / `LoadCurveCubic`).
- Clock sync regression.
- Per-axis step timer (`kalico_per_axis_step_event`).
