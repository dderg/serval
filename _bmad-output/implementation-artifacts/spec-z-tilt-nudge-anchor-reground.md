---
title: 'z_tilt post-nudge anchor re-grounding'
type: 'bugfix'
created: '2026-06-27'
status: 'done'
baseline_commit: '3839d08afb1dd9076370f7225c45ed85d167ac37'
context:
  - '{project-root}/_bmad-output/implementation-artifacts/investigations/post-nudge-slow-move-investigation.md'
---

<frozen-after-approval reason="human-owned intent — do not modify unless human renegotiates">

## Intent

**Problem:** After `z_tilt_adjust` runs per-motor nudge moves, the host gcode-feed throttle (`_check_pause`, klippy/motion.py) gates on `engine.queued_motion_secs()` = `dispatch_anchor.t0 + last_move_time - host_now + uncommitted`. The nudge path (`run_nudge`, stream_planner.rs) advances `last_move_time`, but the anchor is never re-grounded because `motion_drain_finalize` (bridge.rs) is an empty no-op. The signal reports a phantom ~26s backlog and the throttle stalls motion ~20s between probe points (bench-confirmed, session k-1782577178-22340).

**Approach:** Restore grounding at the post-drain chokepoint: give `Anchor` a `reground` method and implement `motion_drain_finalize` to snap `t0` so the queued-motion signal collapses to ~0 once the channel has drained. Emit a loud structured event when grounding absorbs a real phantom, so the failure can never again hide silently.

## Boundaries & Constraints

**Always:** Re-ground only inside `motion_drain_finalize`, which runs solely after `motion_drain_poll()==true` (channel empty, playhead caught up — klippy/motion.py:558-568). Preserve the by-design floating-ahead lookahead during streaming (anchor.rs:24-30) — `queued_motion_secs` reading `buffer_time` mid-stream is correct and must not change. Match surrounding code style; no narration comments.

**Ask First:** Escalating the tripwire from a warn-level event to an `invoke_shutdown`/error. Touching the streaming hot path or `anchor_segment`'s underrun logic.

**Never:** Do not collapse or remove the frontier lookahead. Do not change the throttle source in `_check_pause`. Do not address the broader typed `HostSeconds`/`EngineSeconds` boundary or the "nudge with no following `wait_moves`" hot-path stall (FORCE_MOVE gcode, angle.py, motors_sync.py) — explicit follow-up, out of scope. Do not commit, push, or flash hardware.

## I/O & Edge-Case Matrix

| Scenario | Input / State | Expected Output / Behavior | Error Handling |
|----------|--------------|---------------------------|----------------|
| Post-nudge phantom | `last_move_time` ~26s ahead of `host_now` under stale `t0`; channel drained; `motion_drain_finalize` called | `t0` regrounded → `queued_motion_secs()` ≈ 0; warn `motion`/`anchor_phantom_grounded` with `gap_s` ≈ 26 emitted | N/A |
| True quiescence, no phantom | `last_move_time` already ≈ `host_now`; finalize called | `t0` set so `queued_motion_secs()` ≈ 0; gap below tolerance → no warn | N/A |
| Mid-stream | Active print | finalize never runs (only post `motion_drain_poll()==true`) — signal untouched | by construction |

</frozen-after-approval>

## Code Map

- `rust/motion-engine/src/anchor.rs` -- `Anchor { t0, last_t_end, lead_secs }`; only `new`/`anchor_segment`/`t0` exist. Add `reground`.
- `rust/motion-engine/src/anchor/tests.rs` -- anchor unit tests (separate-file convention). Add reground tests.
- `rust/motion-engine/src/bridge.rs:3664` -- `motion_drain_finalize(&self)` no-op to implement. Mirror the lock/compute pattern of `queued_motion_secs` (`:3921-3943`): `planner.last_move_time()`, `router.host_now_secs()`, `dispatch_anchor` (fields `:655/:661/:668`).
- `klippy/motion.py:558-568` -- `_wait_mcu_drained` caller (context only; no change).

## Tasks & Acceptance

**Execution:**
- [x] `rust/motion-engine/src/anchor.rs` -- add `pub fn reground(&mut self, host_now: f64, frontier_u: f64)` setting `self.t0 = Some(host_now - frontier_u)` and `self.last_t_end = frontier_u`; add a module const for the phantom-warn tolerance (e.g. `PHANTOM_GROUND_WARN_SECS = 1.0`).
- [x] `rust/motion-engine/src/bridge.rs` -- implement `motion_drain_finalize`: read `last_move_time` + `host_now`; compute the pre-grounding lead (same arithmetic as `queued_motion_secs`); if it exceeds the tolerance, `tracing::warn!(subsystem="motion", event="anchor_phantom_grounded", gap_s=…, …)`; then `dispatch_anchor.reground(host_now, last_move_time)`. No signature change.
- [x] `rust/motion-engine/src/anchor/tests.rs` -- unit-test the matrix: phantom collapses to ~0 after `reground`; idempotent/no-op when already grounded.

**Acceptance Criteria:**
- Given a nudge advanced `last_move_time` to a ~26s phantom lead and the channel has drained, when `motion_drain_finalize` runs, then `queued_motion_secs()` returns ~0 (≤ epsilon) and a warn `motion`/`anchor_phantom_grounded` event records the gap.
- Given the anchor is already grounded at quiescence, when `motion_drain_finalize` runs, then the signal stays ~0 and no phantom warn is emitted.
- Given `cargo nextest run -p motion-engine`, then the new reground tests pass and the crate suite stays green.

## Design Notes

`reground` is the inverse of the floating offset: `queued_motion_secs = t0 + last_move_time - host_now + uncommitted`; setting `t0 = host_now - last_move_time` makes the lead term 0, and at a true drain `uncommitted` is already 0. `last_move_time` stays monotonic — only `t0` moves. The tripwire converts the previously-silent stall into a `fail-loudly` signal without risking a print: a stale throttle reading must not shutdown a print, so warn-level only.

## Verification

**Commands:**
- `cd rust && cargo nextest run -p motion-engine` -- expected: green; new `anchor::tests` reground cases pass.
- `cd rust && cargo clippy -p motion-engine -- -D warnings` -- expected: clean.

## Suggested Review Order

- Entry point — the post-drain chokepoint that re-grounds; read the early-return + warn + reground sequence.
  [`bridge.rs:3664`](../../rust/motion-engine/src/bridge.rs#L3664)

- The grounding primitive — inverts the floating offset so the queued-motion lead collapses to 0.
  [`anchor.rs:75`](../../rust/motion-engine/src/anchor.rs#L75)

- The tripwire tolerance constant (warn-level, not shutdown).
  [`anchor.rs:3`](../../rust/motion-engine/src/anchor.rs#L3)

- Tests — phantom collapse + idempotent-at-quiescence, reusing the existing `grounded_queued_secs` helper.
  [`tests.rs:136`](../../rust/motion-engine/src/anchor/tests.rs#L136)
