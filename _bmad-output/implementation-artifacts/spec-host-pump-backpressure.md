---
title: 'Host pump-backlog backpressure: pace gcode ingestion against MCU execution'
type: 'bugfix'
created: '2026-06-21'
status: 'done'
baseline_commit: '20397fbdfd55a8ebcde8705b12d22952e59b9159'
context: ['{project-root}/CLAUDE.md', '{project-root}/_bmad-output/planning-artifacts/sprint-change-proposal-2026-06-21.md']
---

<frozen-after-approval reason="human-owned intent — do not modify unless human renegotiates">

## Intent

**Problem:** Printing any gcode file larger than a few moves freezes. The host reads and submits moves to the new stream planner faster than the printer can execute them — nothing couples ingestion rate to MCU execution progress (mainline Klipper's buffer-time `_check_pause` throttle was never ported to `motion.py`). The unbounded planner channel + per-push whole-buffer replan + MCU-ring overrun then make the system stall.

**Approach:** Gate host gcode ingestion on **pump backlog depth** — the count of pieces the pump holds unpushed (it accumulates exactly when the MCU ring is full and the pump waits for the MCU to retire moves). After each submit, the host pauses the move greenlet (`reactor.pause`) while backlog is above a high watermark, resuming below a low watermark. Pausing in the move path throttles every gcode source (sdcard, network, macros) uniformly, and yields the reactor so MCU comms advance and the backlog drains.

## Boundaries & Constraints

**Always:** Backpressure is a cooperative `reactor.pause` in the host reactor, never a blocking enqueue. Keep PyO3 signatures and the emit backend unchanged. Fail loud: if backlog does not drain within `DRAIN_TIMEOUT`, raise (MCU stalled) — never spin forever. The throttle reads a single aggregate atomic the pump maintains.

**Ask First:** Changing the watermark *unit* away from pump unpushed-piece count (e.g. to enqueued-minus-retired or a time estimate). Adding a bounded/blocking planner channel (deferred to backstop work).

**Never:** Block `submit_move` / the planner channel to apply backpressure (deadlocks the single-threaded reactor). Pause during drip/homing moves (own completion gating). Change MCU wire format, `PieceEntry`, or the pump's scheduling logic. Add input shaping.

## I/O & Edge-Case Matrix

| Scenario | Input / State | Expected Output / Behavior | Error Handling |
|----------|--------------|---------------------------|----------------|
| Large file, fast ingest | gcode >> a few moves | host pauses in `move()` when backlog > high, resumes < low; file prints to completion; planner buffer + pump backlog stay bounded | N/A |
| Steady state | continuous extrusion | backlog oscillates between low/high; MCU ring never empties (no stutter), never grows unbounded | N/A |
| Small file | a few moves | backlog never reaches high → no pause; prints as before | N/A |
| MCU not retiring | backlog stuck above high | `reactor.pause` loop times out after `DRAIN_TIMEOUT` → raise `command_error` | raise |
| Drip / homing | `drip_move` | no backpressure pause (drip completion gates it) | N/A |
| Sim / no MCU | `self.mcu is None` | `_check_pause` is a no-op | N/A |

</frozen-after-approval>

## Code Map

- `rust/motion-engine/src/pump.rs` -- `run_pump` (374), `AxisQueue.pieces` (16); add `backlog: Arc<AtomicU64>` param, store `Σ q.pieces.len()` at the bottom of the outer loop (after apply + the `'send` loop, ~665/720).
- `rust/motion-engine/src/pump/{tests,drip_tests,sched_tests,wire_sink_tests}.rs` -- ~10 `run_pump(...)` call sites; pass a fresh `Arc::new(AtomicU64::new(0))`.
- `rust/motion-engine/src/bridge.rs` -- add `pump_backlog: Arc<AtomicU64>` field (mirror `dispatched_segments` at 621/877); clone into the `run_pump` spawn (2819); add FFI `fn pump_backlog(&self) -> u64` near `get_last_move_time` (3773).
- `klippy/motion_engine.py` -- add `pump_backlog` wrapper method (coerce `None`→0 for the stub); leave OUT of `_STUB_MOTION_METHODS`.
- `klippy/motion.py` -- `_check_pause()` throttle; read `pump_backlog_high`/`pump_backlog_low` config in `__init__` (~128); call `_check_pause` at the end of `move` (356), `move_curve` (391), `dwell` (469); guard drip via a flag set in `drip_move` (464); surface backlog in `stats()` (755). `DRAIN_TIMEOUT` (13) reused.
- `test/test_*.py` / `rust/motion-engine/src/pump/tests.rs` -- new tests (backlog accounting; throttle pause/resume + timeout).

## Tasks & Acceptance

**Execution:**
- [x] `rust/motion-engine/src/pump.rs` -- add `backlog: Arc<AtomicU64>` param; recompute and `store` total unpushed pieces (Acquire/Release) once per outer-loop iteration.
- [x] `rust/motion-engine/src/pump/*tests*.rs` -- update `run_pump` call sites with the new arg. (also `bridge/tests.rs`, `tests/ethercat_transport.rs`, `tests/pump_loop.rs`)
- [x] `rust/motion-engine/src/bridge.rs` -- add field + init + spawn-clone + FFI `pump_backlog()`.
- [x] `klippy/motion_engine.py` -- expose `pump_backlog()` on the wrapper.
- [x] `klippy/motion.py` -- add `_check_pause()` (double-watermark loop, `DRAIN_TIMEOUT` fail-loud, drip/no-mcu guards), config reads, call sites, `stats()` backlog.
- [x] tests -- pump backlog-accounting unit test (`pump/tests.rs`); `_check_pause` throttle + timeout test (`test/test_motion_backpressure.py`); updated `test/test_motion_resync.py` fake.

**Acceptance Criteria:**
- Given a multi-thousand-move file, when printed, then it completes without freeze and the planner buffer + pump backlog stay bounded.
- Given the pump holds backlog above the high watermark and the MCU keeps retiring, when `_check_pause` runs, then it pauses and returns once backlog falls below the low watermark.
- Given a stalled MCU (no retirement), when backlog stays above high for `DRAIN_TIMEOUT`, then `_check_pause` raises a clear `command_error`.
- Given a drip/homing move or `self.mcu is None`, when issued, then no backpressure pause occurs.
- Given the pump pushes all pieces to the ring, when it idles, then `pump_backlog()` reads ~0.

## Design Notes

Why pump-depth, not a time watermark: the pump already holds a `VecDeque` of unpushed pieces per `AxisQueue`; pieces sit there precisely when the ring is full, so unpushed depth is a direct, execution-coupled readout — no duration math, no commit-gated `get_last_move_time` (which only advances on dispatch, never seeing the flood). The ring fills first (stays full → never starves), then backlog accrues → we throttle, bounding total depth.

`_check_pause`: no-op when `self.mcu is None` or during drip; loop `while engine.pump_backlog() > pump_backlog_high: reactor.pause(now + 0.010)` with a `DRAIN_TIMEOUT` deadline that raises `command_error` if backlog never drains. High gates entry; `pump_backlog_low` sizes the steady-state cushion.

Defaults: `pump_backlog_high` ≈ 200, `pump_backlog_low` ≈ 100 pieces — config-overridable; final values are a bench-tuning task (keep ring full, backlog bounded). The bounded-channel fail-loud backstop in `stream_planner.rs` is deferred (`deferred-work.md`).

## Verification

**Commands:**
- `cargo nextest run -p motion-engine` -- expected: pump + bridge suites green incl. new backlog test.
- `make -f Makefile.rust motion-engine` -- expected: cdylib builds; `klippy/_motion_engine.so` updated.
- `./scripts/ci.sh quick` -- expected: green (ruff, rust-test, clippy `-D warnings`, fmt).
- `./scripts/ci.sh py` -- expected: green (klippy touched).

**Manual checks:**
- Run a large slicer gcode through the offline simulator / bench: prints to completion, no freeze; observe `stats()` backlog oscillating between watermarks, MCU ring neither empty nor unbounded.

## Suggested Review Order

**Host throttle (design intent — start here)**

- The double-watermark pause loop: reads backlog once per iteration, drains high→low, fails loud on stuck MCU.
  [`motion.py:570`](../../klippy/motion.py#L570)

- Call sites — every gcode-issuing path is throttled uniformly (move / move_curve / dwell).
  [`motion.py:398`](../../klippy/motion.py#L398)

- Config watermarks + fail-loud cross-field validation (catches the default-bypasses-maxval gap).
  [`motion.py:130`](../../klippy/motion.py#L130)
  [`motion.py:563`](../../klippy/motion.py#L563)

**Backpressure signal (Rust pump → FFI)**

- The signal: total unpushed pieces, re-stored once per pump loop iteration (refreshes on every heartbeat/enqueue).
  [`pump.rs:724`](../../rust/motion-engine/src/pump.rs#L724)

- The shared atomic threaded into `run_pump`.
  [`pump.rs:383`](../../rust/motion-engine/src/pump.rs#L383)

- Engine field + FFI read exposed to Python.
  [`bridge.rs:622`](../../rust/motion-engine/src/bridge.rs#L622)
  [`bridge.rs:3789`](../../rust/motion-engine/src/bridge.rs#L3789)

- Wrapper method (coerces stub `None`→0) and the stub's explicit 0 (prevents `stats()` TypeError).
  [`motion_engine.py:473`](../../klippy/motion_engine.py#L473)
  [`motion_engine.py:106`](../../klippy/motion_engine.py#L106)

**Observability + tests (supporting)**

- Real backlog surfaced in `stats()`.
  [`motion.py:796`](../../klippy/motion.py#L796)

- Pump backlog accounting: accrues when ring-full, drains to 0 when pushed.
  [`tests.rs:557`](../../rust/motion-engine/src/pump/tests.rs#L557)

- Host throttle behaviour: no-pause-below-high, drain-to-low, timeout, drip/no-mcu skips, inverted-watermark reject.
  [`test_motion_backpressure.py:1`](../../test/test_motion_backpressure.py#L1)
