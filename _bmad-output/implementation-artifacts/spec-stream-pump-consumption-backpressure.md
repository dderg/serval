---
title: 'Stream→pump time-based consumption backpressure'
type: 'bugfix'
created: '2026-06-21'
status: 'done'
context:
  - '{project-root}/_bmad-output/implementation-artifacts/investigations/gcode-queue-hang-investigation.md'
  - '{project-root}/docs/rewrite/mcu-c-rust-boundary.md'
baseline_commit: 'f432e585dedde85a3f64d60924743e644cb00271'
---

<frozen-after-approval reason="human-owned intent — do not modify unless human renegotiates">

## Intent

**Problem:** Long gcode queues hang because the submit→stream-planner→pump channel chain is fully unbounded (`rust/motion-engine/src/stream_planner.rs:62` `crossbeam_channel::unbounded`; `rust/motion-engine/src/bridge.rs:2786` `std::sync::mpsc::channel`). The host outruns the MCU piece ring and races arbitrarily far ahead in time; the pump's only credit-release signal (heartbeat `retired_counts`) lives entirely inside the pump (`rust/motion-engine/src/pump.rs:515-541`) with no return path to the planner, so under TX saturation the pump deadlocks at `StallFull` and motion stops. Short queues never fill the ring, so they never hit this.

**Approach:** Add a **time-based** consumption gate at the stream-planner→pump boundary. The pump derives a per-axis *freed-time frontier* (host-time up to which that axis's pieces have been retired) from the heartbeat `retired_counts` plus retained pushed-piece end-times, and publishes it back to the planner. The planner takes the **min across axes** (the bottleneck axis — coordinated axes are freed in sync) and refuses to dispatch a segment whose `t_start` exceeds `frontier + LOOKAHEAD`, where `LOOKAHEAD` is a fixed, tunable host-side constant (5.0s initial). The gate is mode-agnostic — no special handling for drip or any other pump mode. A coarse bounded submit channel propagates the planner's park back to `submit_move` so Python stops ingesting. Fail loudly on frontier/accounting errors.

## Boundaries & Constraints

**Always:**
- Credit model is **time-based**, keyed to retired/freed time, not piece counts. The frontier is the **min across axes** of per-axis freed-time (the lagging/bottleneck axis); coordinated axes freed in sync makes this stable.
- `LOOKAHEAD` is a fixed, tunable host-side constant (initial 5.0s), mode-agnostic. It bounds how far ahead of *consumed* the host may race. 5.0s is large enough that the pump always has schedulable pieces within its own (mode-specific) scheduling horizon and never starves. Tune later if throughput/latency measurements warrant.
- Steady-state throughput must not degrade: when motion keeps up, the planner must not block or add round-trips; the gate is a non-blocking time comparison in the common path.
- The planner must keep servicing control messages (`Flush`, `Shutdown`, `Reset`, `StreamOpen`, `HomeDrip`, `Nudge`) while parked waiting for the frontier to advance — no deadlock.
- The pump must retain enough per-axis pushed-piece end-host-time history to map `retired count → freed time` (pieces are popped on push at `pump.rs:684-692`, so a per-axis FIFO of pushed end-times is required).
- Fail loudly on frontier regression (freed time moving backward), frontier/retired-count mismatch, or unbounded growth of the end-time history — never silently recover.

**Ask First:**
- Whether `submit_move` may hard-block the Python reactor or must yield cooperatively (`klippy/motion.py:move()`). Default proposal: hard-block on the bounded submit channel; revisit if reactor callbacks (heater PID, temp) starve on a repro.
- The submit-channel bound size (proposed: a small multiple of the segments fitting in `LOOKAHEAD`).

**Never:**
- Do not pace based on MCU TX-buffer dynamics (the deprioritized H4 thread).
- Do not touch heartbeat `retired_counts` generation, MCU transport, or `send_frame`/5s RPC.
- Do not change the MCU heartbeat protocol (host derives freed-time from existing counts + retained end-times).
- Do not add a new crate dependency (use `crossbeam-channel` already in `rust/motion-engine/Cargo.toml`).
- Do not gate on piece-count ring room (`enqueued - retired < ring_depth`) — time is the credit currency.

## I/O & Edge-Case Matrix

| Scenario | Input / State | Expected Output / Behavior | Error Handling |
|----------|--------------|---------------------------|----------------|
| Steady state | continuous G1 stream, motion keeps up | frontier advances; `t_start ≤ frontier + LOOKAHEAD` always holds; planner dispatches without blocking; no added latency vs unbounded baseline | N/A |
| Long queue, host races ahead | segment `t_start > frontier + LOOKAHEAD` | planner parks dispatch until the bottleneck axis's freed-time advances (heartbeat arrives); submit channel fills → `submit_move` blocks → Python paces | N/A |
| Heartbeat stalled (H4, residual) | frontier never advances | host stops ingesting (bounded submit channel); motion stalls — known residual risk, not fixed here; failure is bounded and observable | N/A |
| Control msg while parked | `Flush`/`Shutdown` arrives while parked | planner drains and handles it promptly | N/A |
| First segment / no retirements yet | no frontier published yet | planner uses an initial frontier (e.g. ack_now or −∞) so the first segment dispatches | N/A |
| Frontier regression | heartbeat reports a freed-time earlier than the previous frontier | N/A | fail loudly (panic/abort) |
| Shutdown while parked | `StreamMsg::Shutdown` while parked | planner exits promptly | N/A |

</frozen-after-approval>

## Code Map

- `rust/motion-engine/src/pump.rs:15-43` -- `AxisQueue` (`pushed`/`retired`/`ring_depth`); add per-axis freed-time frontier tracking + a pushed-end-time FIFO.
- `rust/motion-engine/src/pump.rs:515-541` -- `PumpMsg::Heartbeat` handler; on `q.retired` advance, recompute the axis freed-time frontier from the end-time FIFO and publish to the planner credit channel.
- `rust/motion-engine/src/pump.rs:574-579,205,291` -- `horizon_of`, `DRIP_WINDOW_SECS`, `MAX_LEAD_SECS`; the pump's mode-specific scheduling horizon (the gate is agnostic to it).
- `rust/motion-engine/src/pump.rs:684-692` -- push site where pieces are popped; the end-time FIFO must be appended here.
- `rust/motion-engine/src/stream_planner.rs:40-86,242-347` -- `StreamPlannerHandle`/submit channel (`unbounded`→bounded) and `run_loop`; add the freed-time frontier ledger fed by the pump and the `t_start ≤ frontier + LOOKAHEAD` gate around dispatch, while still draining control messages.
- `rust/motion-engine/src/bridge.rs:2786,2864,3100-3203` -- pump input channel creation + stored clone + dispatch closure (`enqueue_segment` then `pump_tx.send(PumpMsg::Enqueue)`); wire the pump→planner credit channel and enforce the gate before the send.
- `rust/motion-engine/src/pump.rs:228-236` -- `PumpMsg` enum (credit flows on a separate channel; confirm no new variant needed during impl).
- `rust/motion-engine/src/stream_planner/tests.rs`, `rust/motion-engine/tests/pump_loop.rs` -- add backpressure tests.
- `klippy/motion.py:356-389`, `klippy/motion_engine.py:386-388` -- `submit_move` call site; verify the now-may-block contract is reactor-safe.

## Tasks & Acceptance

**Execution:**
- [x] `rust/motion-engine/src/pump.rs` -- in `AxisQueue`, retain a per-axis FIFO of pushed-piece end-host-times (appended at the push site `684-692`); in the `PumpMsg::Heartbeat` handler (`515-541`), when `q.retired` advances, recompute the axis freed-time frontier (end-time of the retired-th pushed piece) and publish it to a credit channel back to the planner -- give the planner the consumed-time signal it currently lacks.
- [x] `rust/motion-engine/src/stream_planner.rs` -- replace `unbounded()` submit channel with `crossbeam_channel::bounded(N)`; add a freed-time frontier ledger fed by the pump credit channel (min across axes); in `run_loop`, gate dispatch on `seg.t_start ≤ frontier + LOOKAHEAD` (`LOOKAHEAD` a fixed tunable const, 5.0s) while still draining control messages -- end-to-end time-based backpressure keyed to pump consumption.
- [x] `rust/motion-engine/src/bridge.rs` -- create the pump→planner credit channel at `2786`, pass ends into both threads, and enforce the gate in the dispatch closure (`3100-3203`) before `pump_tx.send(PumpMsg::Enqueue(m))` -- connect the two threads.
- [x] `rust/motion-engine/src/stream_planner/tests.rs` -- unit test: dispatch parks when `t_start > frontier + LOOKAHEAD` and resumes on a frontier advance; a `Flush`/`Shutdown` sent while parked is serviced without deadlock -- verify gating + no-deadlock invariant.
- [x] `rust/motion-engine/tests/pump_loop.rs` -- integration test: an enqueue stream beyond `LOOKAHEAD` keeps planner/pump queues bounded, the pump resumes on heartbeat, and submit back-pressures; assert the frontier is the min across axes -- verify the hang is fixed.
- [x] `klippy/motion.py` / `klippy/motion_engine.py` -- confirm a now-may-block `submit_move` is safe on the reactor (no code change unless a yield point is needed) -- Python-side contract check.

**Acceptance Criteria:**
- Given a gcode stream longer than the MCU ring time capacity, when streamed via the gcode input, then motion completes without hanging and the submit/planner/pump queues stay bounded (no unbounded memory growth).
- Given a segment whose `t_start` exceeds `frontier + LOOKAHEAD`, when the planner is about to dispatch it, then it parks until a heartbeat advances the bottleneck axis's freed-time, then resumes; motion never stalls permanently while heartbeats flow.
- Given coordinated multi-axis motion, when the frontier is computed, then it equals the min across axes of per-axis freed-time (bottleneck axis), not a sum or per-axis independent gate.
- Given the planner is parked on the gate, when a `Flush` or `Shutdown` arrives, then the planner services it promptly without deadlock.
- Given a freed-time regression (frontier moves backward) or retired/end-time accounting mismatch, when detected, then the engine fails loudly (panic/abort) rather than silently continuing.
- Given steady-state streaming where motion keeps up, when measuring planner dispatch latency, then no measurable blocking/round-trip is added vs the unbounded baseline (manual throughput spot-check per the project throughput rule).

## Design Notes

The pump is the sole consumption truth: `AxisQueue.retired` advances only in `PumpMsg::Heartbeat` (`pump.rs:515-541`); nothing outside the pump thread sees retired/freed-time today. The pump already thinks in time via `horizon_of` (`ack_now + lead_secs`, `pump.rs:574-579`) — this design extends that currency upstream. The freed-time frontier per axis = end-host-time of the `retired`-th pushed piece; since pieces are FIFO-retired, this is the host-time up to which the MCU has completed that axis. Coordinated axes are freed in sync, so the **min across axes** is a stable, meaningful "what is consumed" reference — picking the lagging axis matches the physical constraint that a coordinated move can't complete until its slowest axis does. `LOOKAHEAD` is a fixed, mode-agnostic host-side constant (5.0s): it bounds how far past *consumed* the host may race, and is large enough that the pump always has schedulable pieces within its own (mode-specific) scheduling horizon and never starves. Bounding the submit channel lets the planner's park propagate to `submit_move`, pacing Python. No steady-state latency: the gate is a non-blocking time comparison when `t_start ≤ frontier + LOOKAHEAD`.

Golden example: `LOOKAHEAD = 5.0s`. Bottleneck axis freed up to `t = 4.0s`. Next segment `t_start = 9.5s` → `9.5 > 4.0 + 5.0 = 9.0` → planner parks. Heartbeat retires one more piece on that axis → frontier advances to `4.6s` → `9.5 ≤ 9.6` → dispatch.

Residual risk: if MCU heartbeats stop arriving entirely (the deprioritized H4 TX-drop scenario), the frontier never advances and motion still stalls — but the host now also stops ingesting, so the failure is bounded and observable instead of an unbounded hang. Revisit H4 instrumentation if this fix does not resolve the repro.

## Verification

**Commands:**
- `./scripts/ci.sh quick` -- expected: fully green (ruff, rust-test, rust-clippy, rust-fmt, watchdog-canary).
- `cargo nextest run -p motion-engine -E 'test(backpressure)'` and the new `stream_planner`/`pump_loop` tests -- expected: pass.
- `cargo nextest run -p motion-engine` -- expected: no regressions in existing `pump_stalls_on_ring_full_resumes_on_heartbeat` etc.

**Manual checks:**
- Run the long-gcode repro (>ring capacity moves) via the `mcu-sim` or `renode-simulation` skill (MCU boundary integration is NOT in CI); confirm motion completes and queues stay bounded. Capture `events/*.jsonl` per the investigation's Reproduction Plan.

## Suggested Review Order

**Backpressure Flow**

- Start with the bounded move/control split and seeded frontier ledger.
  [`stream_planner.rs:59`](../../rust/motion-engine/src/stream_planner.rs#L59)

- Review the parked loop that stops draining moves but still handles control.
  [`stream_planner.rs:374`](../../rust/motion-engine/src/stream_planner.rs#L374)

- Confirm reset/shutdown fail loudly instead of dropping gated committed motion.
  [`stream_planner.rs:541`](../../rust/motion-engine/src/stream_planner.rs#L541)

**Pump Credit**

- See retained pushed end-times and per-axis freed-time state.
  [`pump.rs:17`](../../rust/motion-engine/src/pump.rs#L17)

- Check heartbeat retirement mapping and fail-loud accounting guards.
  [`pump.rs:527`](../../rust/motion-engine/src/pump.rs#L527)

- Confirm pushed pieces append end-host-times before advancing pushed count.
  [`pump.rs:751`](../../rust/motion-engine/src/pump.rs#L751)

**Bridge Wiring**

- Inspect pump-to-planner credit channel and initial host-time frontier.
  [`bridge.rs:2786`](../../rust/motion-engine/src/bridge.rs#L2786)

- Confirm lost frontier receiver is fatal, not silently ignored.
  [`bridge.rs:2866`](../../rust/motion-engine/src/bridge.rs#L2866)

- Review gate-before-anchor dispatch to avoid mutating parked retries.
  [`bridge.rs:3158`](../../rust/motion-engine/src/bridge.rs#L3158)

- Follow configured axis keys into the stream planner frontier ledger.
  [`bridge.rs:3362`](../../rust/motion-engine/src/bridge.rs#L3362)

**Anchor**

- Check non-mutating host-start projection used by pre-anchor gating.
  [`anchor.rs:67`](../../rust/motion-engine/src/anchor.rs#L67)

**Tests**

- Planner test covers park, control servicing, and frontier resume.
  [`stream_planner/tests.rs:260`](../../rust/motion-engine/src/stream_planner/tests.rs#L260)

- Planner test verifies bottleneck frontier is min across configured axes.
  [`stream_planner/tests.rs:322`](../../rust/motion-engine/src/stream_planner/tests.rs#L322)

- Pump integration test verifies retired pieces publish freed host time.
  [`pump_loop.rs:103`](../../rust/motion-engine/tests/pump_loop.rs#L103)
