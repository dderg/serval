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

Observed failure: Z homing move dispatched to both H7 (XY hold) and F446 (Z
descent). H7 received 7 of 11 sub-plans, then its pool filled. F446 never
received its single sub-plan. Z never moved. The user had to pull the plug.

## Goal

A logical segment either commits on all participating MCUs or on none.
No MCU executes a segment until every MCU has confirmed receipt.

## Architecture

### Two-phase segment dispatch

**Phase 1 — Push.** For each logical segment from the planner:

1. Build per-MCU plans (existing `build_push_params`).
2. For MCUs with no displacement on their axes: build an **idle segment** —
   `push_segment` with all handles = `UNUSED_SENTINEL`, no `load_curve` calls.
   The MCU receives the segment's time window but evaluates nothing.
3. Push to ALL MCUs: `load_curve` (where needed) + `push_segment` for each.
   Each MCU ACKs with `PushSegmentResponse` (already exists).
4. Collect all ACKs. If any MCU rejects or times out, abort — do not proceed
   to Phase 2. Report the fault.

**Phase 2 — Commit.** Once all MCUs have ACK'd segment N:

5. Send `CommitSegment { segment_id: N }` to all MCUs.
6. Each MCU transitions segment N from pending to armed (available for ISR
   dequeue).

The MCU's ISR is unchanged — it still dequeues from the SPSC queue. The only
change is that `push_segment` no longer enqueues directly; it parks the
segment in a foreground-owned pending slot. `commit_segment` moves it to the
queue.

### Idle segments

Every MCU receives every logical segment, even when it has no curves to
evaluate. Benefits:

- The commit protocol is uniform — no "does this MCU participate?" branching.
- Pipeline depth is symmetric across MCUs (bounded by the shallowest pool).
- The `is_trivially_constant` skip logic is removed from the dispatch path.
- MCUs always advance their segment counter in lockstep.

An idle segment costs one `push_segment` round-trip (no `load_curve`). The
MCU's engine sees all handles = UNUSED, evaluates nothing, and retires the
segment immediately after its time window passes.

### Pool depth consequences

The effective in-flight depth is bounded by the shallowest MCU. If the F446
has `CURVE_POOL_N=4` and the H7 has `CURVE_POOL_N=16`, the system can only
have 4 segments in flight. The H7's extra 12 slots are wasted.

**Recommendation:** reduce H7 pool depth to match F446 (or a small multiple),
and reallocate the freed RAM to `max_pieces_per_curve`. This reduces sub-plan
count (fewer wire round-trips per segment) and eliminates the pool exhaustion
that caused the observed failure.

### Failure modes

| Scenario | Behavior |
|----------|----------|
| All MCUs ACK push | Commit sent, execution proceeds |
| One MCU rejects push | No commit sent, all MCUs hold pending segment, host reports fault |
| One MCU transport timeout on push | No commit sent, host reports fault |
| One MCU hangs after commit | Other MCUs execute one committed segment, then stall waiting for next commit (safe) |
| Commit arrives at different times | Safe — execution gates on `t_start_clock`, not commit arrival. Commit just arms; timing is in the clock domain |

### Timing

The commit adds one round-trip per segment (broadcast `CommitSegment` +
responses). At USB full-speed latency (~1ms per MCU), this is ~2-3ms for a
2-3 MCU setup. The `t_start_clock` lead time (currently 250ms) absorbs this
easily.

For throughput at speed: the host pipelines segment N+1's Phase 1 (push) while
segment N is executing. The commit latency is hidden as long as the pipeline
stays ahead of execution — same constraint as today, just with the commit
round-trip added to the per-segment dispatch budget.

## Wire protocol

### New message: `CommitSegment`

```
CommitSegment {
    segment_id: u32,   // matches the id from PushSegment
}
```

Response:

```
CommitSegmentResponse {
    result: i32,       // 0 = OK, negative = error
    segment_id: u32,   // echo back for correlation
}
```

### Modified behavior: `PushSegment`

`push_segment` no longer enqueues the segment to the SPSC queue. Instead:

- Validates the segment (existing checks).
- Loads curve handles into the engine's pending slot (foreground-owned).
- Returns `PushSegmentResponse` with `accepted_id` (unchanged wire format).
- The segment is NOT visible to the ISR until `CommitSegment` arrives.

### Modified behavior: `CommitSegment`

- Looks up the pending segment by `segment_id`.
- Moves it from the pending slot to the SPSC queue.
- Returns `CommitSegmentResponse`.
- On error (no pending segment, id mismatch): returns error code, does not
  enqueue.

## Implementation scope

### Firmware (MCU) — `src/` and `rust/runtime/`

1. Add pending segment slot to `FgState` (foreground-owned, not ISR-visible).
2. Modify `runtime_handle_push_segment` to store in pending slot instead of
   SPSC enqueue.
3. New `runtime_handle_commit_segment` FFI: move pending → SPSC queue,
   run the existing re-enable protocol (TIM5 arm on Idle→Running transition).
4. New kalico-native dispatch handler in `src/kalico_dispatch.c`:
   `handle_commit_segment` — parse wire frame, call FFI, send response.
5. Wire protocol registration (message kind constant, response kind).

### Host (motion-bridge) — `rust/motion-bridge/`

1. Restructure dispatch closure: push all MCUs for a segment, collect ACKs,
   then commit all.
2. Add `producer::commit_segment` call (new kalico-native message send).
3. Idle segment support: `push_segment` with all handles UNUSED for
   non-participating MCUs. Remove `is_trivially_constant` skip logic.
4. Error handling: if any MCU fails push or commit, report fault, do not
   proceed.

### What doesn't change

- ISR evaluation path (still dequeues from SPSC, same `isr_sample_tick`).
- Curve loading (`load_curve` / `LoadCurveCubic`).
- Clock sync and `t_start_clock` computation.
- Credit/slot retirement (`kalico_credit_freed`).
- Per-axis step timer (`kalico_per_axis_step_event`).
