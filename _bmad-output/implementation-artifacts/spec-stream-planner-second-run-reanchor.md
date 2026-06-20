---
title: 'Fix stream_planner second-run starvation: re-anchor on insufficient lead, not only full idle'
type: 'bugfix'
created: '2026-06-19'
status: 'draft'
context: ['{project-root}/_bmad-output/implementation-artifacts/investigations/neptune-second-run-starvation-investigation.md']
---

<frozen-after-approval reason="human-owned intent — do not modify unless human renegotiates">

## Intent

**Problem:** Running the same SD print twice deterministically crashes the second run with a fail-loud abort (`DispatchError::SegmentLate` → "planner stream starvation … scheduled 0.086s in the past"). The live planner `stream_planner.rs` only re-anchors the stream clock when the machine is *fully* idle (`esc > t_committed()`). When a second run arrives near-back-to-back — run 1 nearly drained, so the remaining lead `t_committed − esc` is smaller than the ~100–200 ms replan/shape latency — it appends at the stale committed horizon and the first segment lands in the MCU's past.

**Approach:** Re-anchor when the *remaining lead is insufficient*, not only when fully idle: fire the existing idle re-anchor (`advance_idle(esc + LEAD)`) when `esc + LEAD > t_committed()`. This is the behavior already hardened in `planner.rs` (the sota-motion path; commits `67334c98c`, `acd743fbd`, `917e7eeab`) which the newer `stream_planner.rs` reimplemented too strictly. Extract the decision into a pure helper so it is unit-testable without wall-clock timing.

## Boundaries & Constraints

**Always:** Re-anchor only when `state.is_empty()` (committed buffer drained, at rest, `entry_v == 0.0`) — an idle/inter-run resume. `advance_idle(esc + LEAD)` ties the new stream time to the live playhead plus a fresh full lead, so the first segment dispatches ≥ `host_now + LEAD` after solve latency. `advance_idle` must never rewind (it is monotonic, stream.rs:128). Match the working semantics in `planner.rs:597-614`.

**Ask First:** If the fix requires resetting the per-MCU `Anchor.t0` (anchor.rs) or injecting a controllable clock into `run_loop` to make it testable — i.e. the one-line condition change proves insufficient in verification. Surface before expanding scope.

**Never:** Do NOT silently re-anchor a *continuous* (non-empty, mid-stream) stream that is genuinely behind — real planner starvation must still abort loudly at `anchor.rs:39` (CLAUDE.md fail-loud contract). Do NOT touch the `planner.rs` path, the `Anchor` starvation/timeline-reset logic, or the `Flush`/`Dwell`/`Reset` handlers. Do NOT widen LEAD or add a blanket re-anchor that masks throughput regressions.

## I/O & Edge-Case Matrix

| Scenario | Input / State (`esc`, `t_committed`, `LEAD=0.25`, `is_empty`) | Expected Output / Behavior | Error Handling |
|----------|--------------|---------------------------|----------------|
| Fully idle (long gap) | esc=2.5, t_c=2.0, empty | Re-anchor → advance_idle(2.75) | N/A |
| Near-drain back-to-back (the bug) | esc=1.82, t_c=2.0, empty | Re-anchor → advance_idle(2.07); seg0 lands ahead | N/A |
| Healthy buffered stream | esc=0.5, t_c=2.0, empty | No re-anchor (2.0 > 0.75); append at horizon | N/A |
| Mid-stream, behind | any esc, t_c, **not empty** | No re-anchor; if genuinely late, abort `SegmentLate` | Fail loud at anchor.rs |
| Exact boundary | esc + LEAD == t_committed, empty | No re-anchor (strict `>`); advance_idle no-op anyway | N/A |

</frozen-after-approval>

## Code Map

- `rust/motion-engine/src/stream_planner.rs` (~L360-368, `StreamMsg::Move` arm in `run_loop`) -- the defective re-anchor guard; the change site.
- `rust/motion-engine/src/stream.rs` (`advance_idle` L122-131, `t_committed` L144, `is_empty` L134, `entry_v` L149) -- StreamState API the guard uses; `advance_idle` is monotonic and asserts drained+rest.
- `rust/motion-engine/src/anchor.rs` (`anchor_segment` L27-65) -- per-MCU dispatch-time anchor; self-heals via `timeline_reset`/fresh `t0` once seg lands far enough ahead. NOT modified — reference for why bumping the stream clock suffices.
- `rust/motion-engine/src/planner.rs` (L597-614) -- the working reference implementation (different StreamState model; do not copy literally).
- `rust/motion-engine/src/stream_planner/tests.rs` -- handle-level test harness (`Capture`, `StreamPlannerHandle::spawn`, `submit_move`, `flush`, `snapshot`).
- `rust/motion-engine/src/stream/tests.rs` (L120) -- existing `advance_idle_reanchors_committed_time_after_a_gap` pattern.

## Tasks & Acceptance

**Execution:**
- [ ] `rust/motion-engine/src/stream_planner.rs` -- Extract the idle-resume decision into a pure free function `fn idle_resume_target(esc: f64, t_committed: f64) -> Option<f64>` returning `Some(esc + LEAD)` when `esc + LEAD > t_committed`, else `None`. In the `StreamMsg::Move` arm, replace the inline `if state.is_empty() && esc > state.t_committed() + 1e-6 { state.advance_idle(esc + LEAD); }` with: compute `esc`, then `if state.is_empty() { if let Some(t) = idle_resume_target(esc, state.t_committed()) { state.advance_idle(t); } }`. Keep the `is_empty()` guard at the call site (fail-loud boundary).
- [ ] `rust/motion-engine/src/stream_planner/tests.rs` -- Add unit tests for `idle_resume_target` covering every I/O Matrix row (deterministic, no timing). Add one handle-level regression test `second_run_after_near_drain_reanchors`: submit a run, `flush()`, then with a tuned `std::thread::sleep` placing `esc` in `(t_committed − LEAD, t_committed)`, submit a second run + `flush()`, and assert via `snapshot()` that the second run's first `t_start` is re-anchored to ≈ `esc + LEAD` (strictly greater than run 1's last `t_end` by ≈ the gap), not appended at run 1's horizon. If the timing test is flaky, mark it `#[ignore]` with a comment and rely on the pure-function tests (raise via **Ask First** if so).

**Acceptance Criteria:**
- Given two identical SD prints run near-back-to-back (run 1 nearly drained), when the second run's first move is processed, then the stream clock re-anchors to `esc + LEAD` and the first segment dispatches ahead of `host_now` (no `SegmentLate` abort).
- Given a healthy mid-print stream where the planner is genuinely behind (`!state.is_empty()`), when a late segment dispatches, then `anchor.rs` still raises `SegmentLate` and the planner aborts loudly (fail-loud preserved).
- Given `esc + LEAD <= t_committed` with an empty buffer, when a move arrives, then no re-anchor occurs and the move appends at the existing horizon.

## Design Notes

Why `esc + LEAD > t_committed` (not `esc > t_committed`): `esc` is the wall-clock playhead position in stream-time. The old guard only fired once the playhead had fully overrun the committed horizon. But a segment planned at `t_committed` is only `t_committed − esc` ahead of the playhead; if that margin is below `LEAD` (and below the ~100–200 ms solve that runs before dispatch), seg0 lands in the past. Firing whenever `esc + LEAD > t_committed` guarantees every after-rest move starts with a full fresh lead. Since the dispatch `Anchor.t0` already embeds a `+LEAD` cushion, the resulting `scheduled_host = t0 + (esc + LEAD)` stays ≥ `host_now + LEAD`, absorbing solve latency; this scales correctly for both tiny and multi-second gaps. The `is_empty()` guard is the discriminator that keeps genuine mid-stream starvation aborting.

## Verification

**Commands:**
- `cd rust && cargo nextest run -p motion-engine -E 'test(idle_resume) + test(second_run_after_near_drain) + test(reanchor)'` -- expected: new tests pass.
- `cd rust && cargo nextest run -p motion-engine` -- expected: no regressions in stream_planner/stream/anchor suites.
- `./scripts/ci.sh rust-clippy` -- expected: clean (`-D warnings`).
- `./scripts/ci.sh rust-fmt` -- expected: clean.

**Manual checks:**
- After flashing the branch to the Neptune bench, run test3.gcode twice; confirm no `stream_planner_fatal` in the structured logs (query-logs) and the second run completes. (Bench/manual — outside CI; user runs gcode.)
