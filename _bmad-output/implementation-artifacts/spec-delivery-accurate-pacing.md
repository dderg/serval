---
title: 'Delivery-accurate backpressure pacing: gate on the dispatched-to-pump frontier'
type: 'bugfix'
created: '2026-06-23'
status: 'in-review'
baseline_commit: '623a07c0b10c9b42ee6cc26b504b11f3a78b0dc9'
context:
  - '{project-root}/CLAUDE.md'
  - '{project-root}/_bmad-output/implementation-artifacts/investigations/piece-start-in-past-clock-rebase-investigation.md'
  - '{project-root}/_bmad-output/implementation-artifacts/spec-submission-aware-backpressure.md'
  - '{project-root}/_bmad-output/implementation-artifacts/spec-incremental-stream-planning.md'
---

<frozen-after-approval reason="human-owned intent — do not modify unless human renegotiates">

## Intent

**Problem:** Prints still crash with `-308 PieceStartInPast` (session `k-1782180588-39437`, ~6.1 ms axis-3 deficit, watchdog reset). Investigation H13–H15: the dispatched trajectory frontier freezes ~1.07 s and starves both MCUs. Two interlocking faults. (1) The host gate paces on the **submitted** frontier `_mcu_pending_end_time` (`motion.py:570`, advanced on *submit*), not the **dispatched-to-pump** frontier. At the fatal cycle it read 2.31 s of lead while only 0.72 s was delivered — the 1.59 s over-read is the submitted-but-uncommitted backlog — so it throttled the feeder *while the MCU starved*, and the over-read grew with the freeze. (2) Commit selection (`stream.rs:325-347`) can yield `commit_count=0` on a small buffer with no clean seam clearing the brake setback (batch 33: `n=4 barrier=3 commit_count=0`); it returns an empty dispatch and the frontier stalls — and with the feeder paused, no moves arrive to grow the buffer and unstick it.

**Approach:** Pace on real delivery. (1) Gate on the engine's already-exposed committed/dispatched lead (`last_move_time`), excluding the uncommitted-intake term, so during a freeze the gate sees the true draining lead and keeps feeding. (2) When commit would freeze (`commit_count=0`) and delivered lead is thin, force-advance the frontier instead of holding a small buffer wholly uncommitted. Feeding grows the buffer past the setback, which also unsticks the commit naturally.

## Boundaries & Constraints

**Always:** Preserve the fail-loud `DRAIN_TIMEOUT` raise in `_check_pause` (never spin forever). Keep `_mcu_pending_end_time` advancing on submit for `commanded_pos`/`_sync_print_time`/`toolhead:sync_print_time` — only its use *as the gate frontier* changes. Keep ≥ one brake-setback of open tail uncommitted under healthy lead (the incremental-planning terminal-independence invariant). The intake-inclusive signal survives as a diagnostic.

**Ask First:** Any change that braces the gate across the engine-stream clock vs MCU print-clock boundary differently than the existing anchor reconciliation — commit `ab538756d` deliberately moved the gate *off* the engine signal once; re-introducing it must not regress the clock-domain handling.

**Never:** Do not re-raise `lead_secs`/`MAX_LEAD_SECS` or grow the MCU ring (band-aids that mask the signal bug). Do not advance/pad a late start time (CLAUDE.md fail-loud). Do not bake a brake-to-rest mid-stream when the buffer can still grow under a feeding producer — force-advance is only the thin-lead, producer-stalling case where decelerating is the physically correct outcome.

## I/O & Edge-Case Matrix

| Scenario | Input / State | Expected Output / Behavior | Error Handling |
|----------|--------------|---------------------------|----------------|
| Healthy streaming | dispatched lead in (low, high) band, buffer grows past setback | Gate does not throttle; commit holds ≥ setback uncommitted; trajectory unchanged | N/A |
| Frozen frontier + intake backlog | dispatched frontier static, large uncommitted-intake tail | Gate reads draining dispatched lead (≈0.72 s), stays below high-water, keeps feeding | N/A |
| Small no-clean-seam buffer, thin lead | `commit_count=0`, delivered lead below thin threshold | Force-advance dispatched frontier (no freeze); frontier `t_end` strictly advances | N/A |
| Producer truly stalled | no new moves, lead draining to zero | Force-drain to rest (existing path) keeps frontier alive; MCU not scheduled in past | DRAIN_TIMEOUT → raise |

</frozen-after-approval>

## Code Map

- `klippy/motion.py:565-601` -- `_check_pause`: gate computes `buffer_time = _mcu_pending_end_time - est` (submitted frontier A). Change to read the engine's dispatched lead (B). Keep the watermark loop, `feed_throttle_enter/exit` events, and `DRAIN_TIMEOUT`.
- `klippy/motion.py:551-556` -- `_bump_pending_end_time`: advances A on submit. Keep for position/print-time sync; do NOT repurpose as the gate frontier.
- `rust/motion-engine/src/bridge.rs:3821-3844` -- `queued_motion_secs`: returns `(t0 + last_move_time − host_now) + uncommitted`. The `(t0 + last_move_time − host_now)` term IS the dispatched-to-pump lead (B); `+ uncommitted` is the A−B over-read. Expose the B-only lead for the gate; keep the intake-inclusive value as a diagnostic.
- `rust/motion-engine/src/bridge.rs:3809-3818` -- `get_last_move_time`: committed frontier `t_end` (B), stream clock — the frontier the gate should pace on.
- `rust/motion-engine/src/stream.rs:325-347` -- commit-count selection: loop `for i in 1..=barrier` with `if total_arc − arc_to_seam < setback break`; yields `chosen=0` on a small buffer. Add the thin-lead force-advance.
- `rust/motion-engine/src/stream.rs:582-592` -- `brake_to_rest_setback`: the held-back braking distance (terminal-independence invariant).
- `rust/motion-engine/src/stream_planner.rs:~615-626` -- idle-drain watermark / `force` trigger fed into `commit`. Extend the force condition to also fire on thin delivered lead, not only wall-clock idle.
- `rust/motion-engine/src/stream_planner.rs:363-366` -- `dispatch_committed`: advances `last_move_time_bits`; a no-op when `segs` is empty — this is exactly the frozen frontier.

## Tasks & Acceptance

**Execution:**
- [x] `rust/motion-engine/src/bridge.rs` -- Expose the committed/dispatched lead in host-clock seconds excluding the uncommitted-intake term (split `queued_motion_secs` or add `dispatched_lead_secs`); keep the intake-inclusive value available for diagnostics only. -- Gate must pace on B, not B+intake.
- [x] `klippy/motion.py` -- In `_check_pause`, compute `buffer_time` from the engine's dispatched lead (B) instead of `_mcu_pending_end_time − est`; preserve the high/low watermark loop, `feed_throttle_*` events, and the `DRAIN_TIMEOUT` raise. -- Stop throttling the feeder during a freeze.
- [x] `rust/motion-engine/src/stream_planner.rs` + `stream.rs` -- When commit selection yields `commit_count=0` and delivered lead is thin, force-advance the dispatched frontier (route through the existing `force`/idle-drain path) so it never freezes; leave healthy-lead behavior (hold ≥ setback) untouched. -- Fix the `commit_count=0` regression.
- [x] `rust/motion-engine/src/bridge.rs` tests + `stream.rs` tests -- Unit-test the B-only lead accessor (excludes intake) and the thin-lead force-advance (`commit_count` 0 → frontier advances); cover the healthy-lead unchanged path. -- Per I/O matrix.

**Acceptance Criteria:**
- Given a static dispatched frontier with a large uncommitted-intake backlog, when `_check_pause` evaluates, then `buffer_time` reflects the draining dispatched lead and the feeder is NOT throttled.
- Given a small no-clean-seam buffer with thin delivered lead, when `commit` runs, then the dispatched frontier `t_end` strictly advances (no `commit_count=0` freeze).
- Given healthy streaming lead, when `commit` runs, then it still holds ≥ one brake setback uncommitted and produces no mid-stream brake (no throughput regression).

## Spec Change Log

## Design Notes

The dispatched lead already lives inside `queued_motion_secs` — `(t0 + last_move_time − host_now)`; the fix is to stop adding `uncommitted` to the *gate* signal (it stays a diagnostic). Clock domains differ: the gate's `est` is `mcu.estimated_print_time` while the engine lead is anchored via `dispatch_anchor.t0()`. Commit `ab538756d` moved the gate off the engine signal once — reconcile through the existing anchor, do not hand-roll a second clock bridge.

Face-2 throughput rule: force-advance under thin lead is not a regression — if the producer has stalled and the whole buffer sits within a braking distance, decelerating is physically required (you cannot cruise into an empty buffer). The regression to avoid is braking when the buffer can still grow under a feeding producer; with face 1 fixed this path is the rare true-starvation safety net, not steady state.

## Verification

**Commands:**
- `cd rust && cargo nextest run -p motion-engine` -- expected: new bridge + stream unit tests pass, suite green.
- `./scripts/ci.sh quick` -- expected: fully green (ruff, rust-test, rust-clippy -D warnings, rust-fmt, watchdog-canary).
- `./scripts/ci.sh py` -- expected: green (change touches `klippy/motion.py`).

**Manual checks (if no CLI):**
- Bench print (EtherCAT, the `-308` repro): no `PieceStartInPast` fault; `dispatch_committed` t_end advances continuously (no ~1 s gap); `feed_throttle_enter` does not fire while the committed lead is draining. Verify via `query-logs`/`mcu-diagnostics`.
