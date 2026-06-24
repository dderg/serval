---
title: 'Truly-bounded host motion pipeline (end-to-end backpressure)'
type: 'refactor'
created: '2026-06-24'
status: 'in-progress'
baseline_commit: 'dbc42a4cfd7cfd673bfc668e4132753cc4e89f70'
context:
  - '{project-root}/_bmad-output/implementation-artifacts/investigations/neptune-crash-short-residual-investigation.md'
---

<frozen-after-approval reason="human-owned intent — do not modify unless human renegotiates">

## Intent

**Problem:** Host backpressure is not bounded end-to-end. The planner→pump hop is an **unbounded** `std::sync::mpsc::channel::<PumpMsg>()` (`bridge.rs:2774`) and `submit_move` raises a **fatal `RuntimeError`** on input-channel-full instead of pausing the reader. So the gcode reader dumps an entire file unthrottled; any file whose move count outruns the 8192 input-channel cap crashes the host ("input channel full — backpressure gate bypassed"), and pacing leans on a separate time gate (`_check_pause`/`queued_motion_secs`) that can't see in-channel moves.

**Approach:** Make every hop a bounded queue that backpressures its producer, so the reader self-clocks to MCU consumption — with the pump's `MAX_LEAD_SECS` (~2s just-in-time) as the terminal time-bound — then retire the now-redundant `_check_pause` time gate. All waiting is **cooperative** (`reactor.pause`), never a hard OS-thread block.

## Boundaries & Constraints

**Always:** Every blocking wait must (a) yield the reactor via `reactor.pause`, (b) carry a fail-loud drain deadline (no silent hang), (c) break promptly on shutdown/estop. Control messages (`Shutdown`/`Flush`/`Heartbeat`/`DripDisarm`) must never block behind piece-data backpressure. The terminal time-bound is the pump's `MAX_LEAD_SECS`, not channel counts. Committed trajectory output is byte-identical — this is flow-control restructuring only.

**Ask First:** Retiring `_check_pause`/`queued_motion_secs` (step 4) — only after end-to-end backpressure is verified. Changing `MAX_LEAD_SECS` or any channel capacity. Any design where piece-data backpressure could block estop/flush/shutdown.

**Never:** Hard-blocking the reactor (blocking the OS thread inside `submit_move`). Addressing the EtherCAT `-308` starvation (separate: RT scheduling / NIC drivers). Changing planner or pump algorithms or the trajectory. Dropping, padding, or reordering moves. Silent recovery — deadlines fail loud.

## I/O & Edge-Case Matrix

| Scenario | Input / State | Expected Output / Behavior | Error Handling |
|----------|--------------|---------------------------|----------------|
| Reader outruns planner | submit faster than planner pulls | `submit_move` (Py wrapper) yields+retries; reader held mid-`run_script`; no overflow | N/A |
| Planner not draining | input channel stays full past `DRAIN_TIMEOUT` | distinct, attributable fail-loud error ("planner stalled") | raise |
| Pump at lead horizon | piece-data channel full (pump ~2s ahead) | planner dispatch blocks (yield-to-shutdown) → input fills → reader pauses | N/A |
| Shutdown/estop mid-wait | reader or planner parked in a wait | wait breaks immediately; control messages still delivered; no deadlock | N/A |
| File > input-cap of moves | whole file offered fast | metered to ~2s lead; gradual progress; no `RuntimeError` | N/A |

</frozen-after-approval>

## Code Map

- `rust/motion-engine/src/stream_planner.rs:184,217,222` -- input channel `bounded(INPUT_CHANNEL_CAP=8192)`, `submit_move`→`try_submit_move`, `pending_channel_moves()`.
- `rust/motion-engine/src/stream_planner.rs:778,781` -- planner-thread `commit(false)` + `dispatch_committed` (the producer into the pump channel).
- `rust/motion-engine/src/bridge.rs:2774` -- **UNBOUNDED** `std::sync::mpsc::channel::<PumpMsg>()` — the hop to split (control vs data) and bound.
- `rust/motion-engine/src/bridge.rs:3088-3204` -- `DispatchFn` closure; `pump_tx.send(PumpMsg::Enqueue(m))` (non-blocking today).
- `rust/motion-engine/src/bridge.rs:3374` -- `submit_move` PyO3 (return would-block instead of fatal `ChannelFull`).
- `rust/motion-engine/src/pump.rs:485,335` -- `run_pump(rx: Receiver<PumpMsg>)`, `MAX_LEAD_SECS=2.0`; pump must stop draining the data channel at the lead horizon so the bound backpressures.
- `klippy/motion_engine.py:402,486` -- `submit_move` wrapper (add cooperative wait), `queued_motion_secs`.
- `klippy/motion.py:445,656,132` -- `toolhead.move`, `_check_pause` (retire in step 4), `buffer_time_high/low`.
- `klippy/extras/virtual_sdcard.py` -- gcode reader; no change (held synchronously via the `run_script`→`reactor.pause` call stack).

## Tasks & Acceptance

**Execution (build order — each step independently testable offline):**

*Step 1 — input-channel backpressure (closes the overflow crash):*
- [x] `rust/motion-engine/src/stream_planner.rs` + `bridge.rs` -- expose `INPUT_CHANNEL_CAP` (pub) and an `input_channel_capacity()` PyO3 accessor. The handle already does non-blocking `try_send` (`Err(ChannelFull)` = would-block), now an unreachable fail-loud backstop.
- [x] `klippy/motion.py` (`_check_pause`) + `motion_engine.py` -- add a channel-count gate beside the time gate: throttle when `pending_channel_moves >= channel_high` (= 3/4 cap), drain to `channel_low` (= 1/2 cap), with a shutdown break and the existing `DRAIN_TIMEOUT` fail-loud. Single-producer reader makes the post-submit gate overflow-safe; no `submit_move` contract change.
- [x] `test/test_motion_backpressure.py` + `rust/motion-engine/src/stream_planner/tests.rs` -- offline: count-gate throttle/drain/deadline/shutdown-break (Python); channel depth tracks occupancy and refuses overflow at capacity (Rust).

*Steps 2–4 — bounded planner→pump hop + retire the gate (pending bench-verify of step 1):*
- [ ] `rust/motion-engine/src/bridge.rs` -- split `PumpMsg` into a control channel (unbounded, never blocks) and a piece-data (`Enqueue`) channel.
- [ ] `rust/motion-engine/src/bridge.rs` + `pump.rs` -- bound the piece-data channel; planner dispatch blocks as a yield-to-shutdown loop; pump stops draining at `MAX_LEAD_SECS` so the bound backpressures the planner.
- [ ] `rust/motion-engine/src/pump/tests.rs` -- offline: slow mock pump → planner blocks → input fills → reader pauses; shutdown unblocks; delivered lead bounded ~`MAX_LEAD_SECS`.
- [ ] `klippy/motion.py` -- retire the `_check_pause` time loop once end-to-end backpressure is verified (Ask-First); preserve the `_drip_active`/homing exemption.

**Acceptance Criteria:**
- Given a file whose move count exceeds the input-channel cap, when streamed, then it completes with no "input channel full" `RuntimeError` and motion stays ~`MAX_LEAD_SECS` ahead (progress advances gradually, not an instant jump).
- Given a stalled planner or pump, when any backpressure wait exceeds `DRAIN_TIMEOUT`, then a distinct, attributable error is raised — never a silent hang.
- Given an estop/shutdown during any backpressure wait, then the wait breaks promptly and control messages are delivered (no deadlock behind piece-data).
- Given the full motion suite, when run, then committed trajectory output is unchanged versus baseline.

## Spec Change Log

- **2026-06-24 — Step 1 approach refined during implementation.** Original task text put the cooperative wait in the `motion_engine.py submit_move` wrapper as a `try_send`-return loop. Implementation instead added a **channel-count gate inside `motion.py:_check_pause`** (beside the existing time gate), because: (a) the move is built and `move_seq` consumed *before* the send, so retrying the whole `submit_move` would skip a line number; (b) the gcode reader is the single producer (reactor thread + `gcode_mutex`) and the planner only drains, so a post-submit `pending < cap` gate is race-free and overflow-safe; (c) `_check_pause` already owns the reactor, `DRAIN_TIMEOUT`, `self.printer` (shutdown), and the `feed_throttle`/`backpressure_view` logging. Frozen Intent/Boundaries/I/O matrix unchanged. Also fixed a pre-existing breakage: the instrumentation commit added `pending_channel_moves()`/`dispatched_lead_secs()`/`uncommitted_intake_secs()` calls to `_check_pause` without updating `FakeEngine`, leaving 4 backpressure tests red.

## Design Notes

The gate is a patch for one unbounded hop: because `bridge.rs:2774` never backpressures, pump/MCU fullness can't reach the reader, so `queued_motion_secs` (committed-frontier-vs-`host_now`) is currently the *only* end-to-end pacing. Bounding that hop makes the chain self-clock, retiring the gate.

Count↔time caveat: bounding a channel by *move/piece count* yields a buffer whose *time* depth varies with segment size (1024 tiny-facet pieces ≈ 250ms; 1024 long moves ≈ minutes). The time-bound is therefore enforced at the **pump** (`MAX_LEAD_SECS` just-in-time send), not by counts — the pump must *stop draining* its data channel at the lead horizon so the bound propagates back as backpressure. Step 1 alone closes the overflow crash; steps 2–4 deliver the self-clocking pipeline and let the gate go.

## Verification

**Commands:**
- `cargo nextest run -p motion-engine` -- expected: new offline backpressure/deadline/shutdown tests pass; all existing stream/pump tests green.
- `cargo test --doc -p motion-engine` -- expected: green if doc examples touched.
- `./scripts/ci.sh quick && ./scripts/ci.sh py` -- expected: fully green (touches `klippy/`).

**Manual checks:**
- Bench (`query-logs`): a file exceeding the input-cap streams with gradual `sdcard`/`backpressure_view` progress, `channel_pending` bounded, and no "input channel full" RuntimeError; `work_handler_done outcome=complete` only after motion plays out.
