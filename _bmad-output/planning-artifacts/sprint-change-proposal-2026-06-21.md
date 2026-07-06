# Sprint Change Proposal — Host backpressure / gcode-ingestion flow control

**Date:** 2026-06-21
**Author:** dderg
**Trigger spec:** `spec-motion-11-pipeline-production-cutover.md` (status: in-progress)
**Scope classification:** Moderate (amends an in-progress spec's I/O surface; host + optional planner change)

---

## Section 1 — Issue Summary

**Problem statement.** Printing any gcode file larger than a few moves freezes — "nothing happens." The host reads and submits moves to the new stream planner faster than the printer can physically execute them, with no mechanism coupling ingestion rate to MCU execution progress. The system makes no visible forward progress.

**How it was discovered.** Attempting to print a real (non-trivial) gcode file on the bench.

**Root cause.** The host never paces gcode ingestion against MCU execution. Mainline Klipper throttles in the toolhead via a buffer-time watermark — when planned-ahead time exceeds `BUFFER_TIME_HIGH` (~2 s) the move path calls `reactor.pause()` until it drains to `BUFFER_TIME_LOW` (~1 s); because moves are submitted synchronously from the same greenlet that reads the file, that pause is what stops the reader. **That mechanism was never ported to `motion.py`.** A grep over `klippy/` finds no `BUFFER_TIME`, `_check_pause`, or `need_check_pause`. The only `reactor.pause` calls in `motion.py` are in the drain paths (`wait_moves_and_mcu`, `_drain_to_mcu_execution`), not the steady-state `move()`/`move_curve()` path, which submits and returns immediately.

**Evidence.**
- `klippy/motion.py:356-389` (`move`) and `:391-428` (`move_curve`): submit then `_sync_print_time()`; no watermark, no pause.
- `klippy/extras/virtual_sdcard.py:277-321` (`work_handler`): reads/dispatches in a tight loop, yielding (`reactor.pause(NOW)`) only once per 8 KB chunk — nothing throttles per-move.
- `rust/motion-engine/src/stream_planner.rs:62`: the planner channel is `unbounded()`; `submit_move` (`:91-95`) never blocks the caller.
- `rust/motion-engine/src/stream.rs:187-189`: `commit()` re-runs `fit_chain` + `plan_velocity_warm_start` over the **entire** uncommitted `VecDeque` on every push → O(n²) as the buffer balloons.
- Commit-gated signal trap: `motion.py:383-387` bumps `_mcu_pending_end_time` by the delta of `engine.get_last_move_time()`, but that value (`stream_planner.rs:165-167` → `last_move_time_bits`) only updates on **dispatch/commit** (`:221`), not on submit. During streaming `commit(false)` defers, so the delta is ~0 — a watermark built on `_mcu_pending_end_time` would not even observe the flood.

---

## Section 2 — Impact Analysis

**Spec impact.**
- `spec-motion-11-pipeline-production-cutover.md` (in-progress) owns the `submit_move` seam and the streaming planner. Its I/O & Edge-Case Matrix covers "Late segment" and "Flush/Dwell/program end" but has **no row for sustained throughput / host backpressure**. This is a genuine gap in the frozen intent — needs a Spec Change Log amendment (the throttle does not violate the frozen "keep the planner thread architecture" or "never stop between collinear moves" constraints).

**Code impact.**
- `klippy/motion.py` — primary change: add the pump-backlog throttle to the move submission path.
- `rust/motion-engine/src/pump.rs` + bridge — expose an aggregate unpushed-piece count for the host to read.
- `rust/motion-engine/src/stream_planner.rs` — optional: bound the channel as a fail-loud backstop.
- No change required to the emit backend, MCU wire format, or PyO3 signatures (preserves the spec-11 reuse boundary).

**Architectural impact.** Restores the mainline-equivalent backpressure invariant: *host ingestion is paced by MCU execution progress via cooperative reactor pausing*. Confirms a hard design rule for the rewrite: **backpressure must be a cooperative `reactor.pause` in the single-threaded host reactor, never a blocking enqueue** — blocking `submit_move` (called under `py.detach`) would stall the reactor, freezing MCU comms so `estimated_print_time` never advances and the queue never drains (permanent deadlock).

---

## Section 3 — Recommended Approach

**Direct adjustment** (no rollback, no MVP cut). Gate host gcode ingestion on **pump backlog depth** — pause submitting when the pump holds too much unpushed work, resume when it drains. No buffer-time estimate; the signal is execution-coupled by construction.

**Why pump-depth, not a time watermark.** The pump (`pump.rs:16-37`) already holds a `pieces: VecDeque` of unpushed pieces per `AxisQueue`, alongside `ring_depth`/`pushed`/`retired` (→ `in_flight`, `available()`). Pieces accumulate there *exactly when the MCU ring is full and the pump is waiting for the MCU to retire moves*. So unpushed depth is a direct readout of "how far ahead of execution are we," requiring no nominal-duration math and no commit-gated `get_last_move_time`. It satisfies the two real requirements directly: the ring never goes empty (keep feeding while depth is low) and never grows uncontrollably (stop when depth is high).

1. **Backlog signal.** The pump maintains an aggregate `AtomicU32` total-unpushed-piece count (sum of per-`AxisQueue` `pieces.len()`), updated on enqueue and push. Expose it through the bridge (`pump_backlog()` / similar) so `motion.py` can read it with a cheap atomic load. (Optionally also expose `in_flight` vs `ring_depth` if a fuller-ring signal proves better in tuning.)

2. **Throttle in the move path.** After submit:
   ```
   while engine.pump_backlog() > PUMP_DEPTH_HIGH:
       reactor.pause(monotonic + POLL_INTERVAL)
   ```
   `PUMP_DEPTH_HIGH`/`PUMP_DEPTH_LOW` are piece-count thresholds (two thresholds only to avoid pause/resume thrashing). Defaults sized to keep roughly one ring-depth of lookahead — enough that the ring never starves, small enough that backlog stays bounded. Pausing in the move greenlet throttles every gcode source uniformly (sdcard, network, macros).

3. **Fail loud, never hang.** If backlog does not drain while paused (MCU stalled / not retiring), time out (reuse the `DRAIN_TIMEOUT` pattern) and raise — never spin forever. Bound the planner channel/buffer as a backstop that **errors** (not blocks) if ever hit; reaching it means the throttle was bypassed (a bug), consistent with the project's fail-loud value.

4. **Exclusions.** Skip the throttle for drip/homing moves (own completion gating) and when `mcu is None` (sim).

5. **Consequential win.** With ingestion gated, the planner buffer stays bounded, so the per-push whole-buffer replan (`stream.rs:187`) becomes O(bounded window) instead of O(growing file) — removes the quadratic stall as a side effect; not a separate fix.

**Rejected alternatives.** (a) Bounded *blocking* channel as the primary throttle — deadlocks the reactor (see Architectural impact). (b) Buffer-time watermark on a host-side nominal-duration estimate — workable but needs duration math and a frontier signal; pump-depth is simpler and more directly tied to execution.

---

## Section 4 — Detailed Change Proposals

**`rust/motion-engine/src/pump.rs` + bridge**
- Maintain an aggregate `AtomicU32` total-unpushed-piece count, incremented on enqueue and decremented on push. Expose it through the bridge as `pump_backlog()` (and via `motion_engine.py`).

**`klippy/motion.py`**
- Add `_check_pause()` — poll `engine.pump_backlog()`, `reactor.pause` while above `PUMP_DEPTH_HIGH` until below `PUMP_DEPTH_LOW`, with `DRAIN_TIMEOUT` fail-loud. Call it at the end of `move` (`:356`)/`move_curve` (`:391`) (and after `dwell`'s bump, `:469`). Skip when `mcu is None`; do not call from `drip_move`.
- Add `pump_depth_high`/`pump_depth_low` config reads with sensible defaults; surface real backlog in `stats()` (`:755` currently hardcodes `buffer_time=0.000`).

**`rust/motion-engine/src/stream_planner.rs` (backstop, optional this pass)**
- Replace `unbounded()` (`:62`) with a bounded channel, and make `submit_move` return a fail-loud error if full (never block) — a backstop in case the host throttle is ever bypassed.

**`spec-motion-11-pipeline-production-cutover.md`**
- Spec Change Log entry (non-frozen): record the host backpressure requirement and add an I/O-matrix row — *"Sustained throughput: input faster than execution → host paces ingestion via buffer-time watermark; no freeze, no unbounded growth; fail loud on stuck drain."*

---

## Section 5 — Implementation Handoff

**Scope:** Moderate — amends an in-progress frozen spec's I/O surface + host code change (+ optional Rust backstop).

**Recipients:** Developer (host motion.py change + spec amendment); optional Rust pass for the channel backstop.

**Success criteria.**
- A multi-thousand-move gcode file prints to completion without freeze; host memory and planner buffer stay bounded.
- Steady-state buffered time oscillates between the low and high watermarks; MCU never underruns mid-print (no stutter) and never floods.
- A stalled MCU during a pause raises a clear timeout error rather than hanging.
- `./scripts/ci.sh quick` green; `cargo nextest run -p motion-engine` green.
