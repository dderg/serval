---
title: 'Relocate segment-dispatch logic out of an opaque cross-file closure'
type: 'refactor'
created: '2026-07-01'
status: 'done'
route: 'one-shot'
context: []
---

## Relocate segment-dispatch logic out of an opaque cross-file closure

## Intent

**Problem:** To follow one committed `ShapedSegment` from `StreamState::commit()` to the pump, a reader had to jump into an opaque `Arc<dyn Fn>` (`DispatchFn`) that was invoked in `stream_planner.rs` but actually defined ~200 lines away in `bridge.rs`, with no named function marking where the real logic lived.

**Approach:** Extracted the closure's body verbatim into a named free function `dispatch_segment(ctx: &SegmentDispatchCtx, seg: &ShapedSegment)` co-located with its only caller in `stream_planner.rs`; `bridge.rs` now just builds the `SegmentDispatchCtx` (the same `Arc`-cloned state the closure used to capture) and passes a one-line closure calling it. Pure relocation — no algorithm, signature, channel, or threading changes.

## Suggested Review Order

**Where the logic now lives**

- New named home for the dispatch logic, previously an anonymous closure body in `bridge.rs`.
  [`stream_planner.rs:415`](../../rust/motion-engine/src/stream_planner.rs#L415)

- The context struct that replaces closure-capture — same `Arc`-cloned fields as before, just named.
  [`stream_planner.rs:401`](../../rust/motion-engine/src/stream_planner.rs#L401)

**Where it's now constructed**

- `bridge.rs` builds the context once and hands a one-line closure to `stream_planner`, instead of the ~100-line inline closure this replaces.
  [`bridge.rs:3322`](../../rust/motion-engine/src/bridge.rs#L3322)

**Peripherals**

- `dispatch_committed` (unchanged) — the existing caller `dispatch_segment` is now readable next to.
  [`stream_planner.rs:517`](../../rust/motion-engine/src/stream_planner.rs#L517)
