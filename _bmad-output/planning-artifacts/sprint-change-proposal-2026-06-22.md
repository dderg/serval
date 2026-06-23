# Sprint Change Proposal — Motion backpressure: gate submission on a submission-aware queued-time signal

**Date:** 2026-06-22
**Author:** dderg
**Supersedes:** `sprint-change-proposal-2026-06-21.md` (the pump-backlog throttle)
**Trigger investigation:** `_bmad-output/implementation-artifacts/investigations/print-completes-early-investigation.md` (Follow-up #3, High confidence)
**Scope classification:** Moderate (host `motion.py` + engine bridge/planner signal; no MCU wire-format change)

---

## Section 1 — Issue Summary

**Problem.** Mainsail's "requested position" jumps far ahead of actual motion (for a short print, straight to the final point) and the print reports **complete** long before motion finishes; the host also over-fills, eventually scheduling pieces in the MCU's past (`-142` → `fault_event` → `send_frame_fatal`, mid-layer-2 crash).

**Root cause (confirmed).** `commanded_pos` / `gcode_position` are set at **submit** (klippy/motion.py:402; gcode_move.py:124-145). Submission is throttled only by `engine.pump_backlog()` — the **pump-queue depth**, the *last* stage. Between the gcode interpreter and the pump sit **two unbounded queues** (planner input channel `stream_planner.rs:78`; pump channel `bridge.rs:2810`). The host dumps ring-sized bursts into them faster than motion drains; the reported position reflects submission, so it races to the end. The 2026-06-21 throttle picked the wrong gauge: a downstream depth cannot bound an upstream submission frontier.

**Why the previous fix missed it.** `pump_backlog` only rises once the large MCU ring fills; by then the interpreter has already submitted (and `commanded_pos` jumped) a ring's worth. Confirmed empirically: `submit_move_enter` arrives in bursts of 351 then 866 (session k-1782142874-30191), not paced.

**Mainline reference (git `main` klippy/toolhead.py).** Bounded ~0.25 s lookahead (`LOOKAHEAD_FLUSH_TIME`), flushed **synchronously** so `print_time` tracks submission to within 0.25 s; `_check_pause` then `reactor.pause`s the gcode greenlet while `print_time − est > BUFFER_TIME_HIGH (2.0)` down to `BUFFER_TIME_LOW (1.0)` (toolhead.py:228-237, 615-626, 517-546). Our config already has `buffer_time_high=2.0` / `buffer_time_low=1.0` — **unused**.

---

## Section 2 — Impact Analysis

**Code impact.**
- `klippy/motion.py` — replace the `pump_backlog` gate in `_check_pause` with a buffer-time gate on a submission-aware engine signal; wire the existing `buffer_time_high/low` config.
- `rust/motion-engine` (bridge + stream_planner) — expose **`queued_motion_secs()`**: the real dispatched frontier plus a planner-tracked **uncommitted-intake** tally, so the signal reflects *submission*, not just dispatch (closes the 2026-06-21 "commit-gated signal trap").
- `rust/motion-engine/src/pump.rs` — `MAX_LEAD_SECS` band-aid (raised 1.0→2.0 to mask `-142`) reverts once real backpressure lands.
- No change to MCU wire format, emit backend, or PyO3 move-submission signatures.

**Architectural impact.** Establishes the invariant mainline has and we lost: *there is a single bounded lead window (in motion-time) between the gcode interpreter and the live MCU playhead, enforced cooperatively at the submit path.* The unbounded async queues are allowed to exist (planner stays on its own thread for parallel optimization) **only because** the cooperative gate stops the interpreter from filling them past the window.

**Carry-over hard rules (from 2026-06-21, still binding).**
- Backpressure is a **cooperative `reactor.pause`** in the host greenlet — **never** a blocking `submit_move` (runs under `py.detach`; blocking it stalls the reactor → `estimated_print_time` freezes → permanent deadlock).
- **Fail loud, never hang**: `DRAIN_TIMEOUT` on the pause loop; a bounded-channel/ring backstop that **errors** (not blocks) if the gate is ever bypassed.

---

## Section 3 — Recommended Approach & Decisions

**Direct adjustment.** Gate submission on **buffer-time over a submission-aware frontier**, mainline-style. The four decisions the trigger asked to settle:

**D1 — Where the lead-window boundary lives.** At the host move-submission path (`_check_pause`, called from `move`/`move_curve`/`dwell`), via cooperative `reactor.pause` — exactly mainline's location. Not in `submit_move`, not a blocking channel.

**D2 — How the engine reports queued-seconds (the crux).** New bridge call `queued_motion_secs()` returns
`(get_last_move_time − est_anchor) + uncommitted_intake_secs`, where:
- the **committed/dispatched** part is the planner's *real optimized* frontier (`get_last_move_time`) — accurate, no phantom-buffer over-estimate;
- the **uncommitted intake** part is a planner-thread tally of nominal `min_move_t` for moves received but not yet committed (incremented on intake/push, reconciled on commit).
This is what closes the 2026-06-21 signal trap: the frontier now moves on **submit**, not only on commit. The nominal estimate is used **only** for the small uncommitted tail; because the gate keeps that tail bounded, its inaccuracy is negligible — unlike the abandoned host-side all-nominal `_mcu_pending_end_time`, which over-estimated the *whole* buffer and under-fed (starved) the MCU.
Host computes `buffer_time = queued_motion_secs()` and gates on `buffer_time_high/low`.

**D3 — How to bound the intake without deadlocking the reactor.** The cooperative gate bounds it *by construction*: after each submit, `_check_pause` pauses the interpreter while `buffer_time > buffer_time_high`, so it can never get more than ~one move past the window before yielding. The unbounded channels stay unbounded in the happy path (no blocking enqueue → no deadlock); add a **bounded backstop that errors** (planner channel cap and/or ring) reached only if the gate is bypassed — a fail-loud bug signal, per project rule.

**D4 — pump_backlog and the MAX_LEAD band-aid.**
- **pump_backlog:** demote from steady-state throttle to **diagnostics only** (keep it in `stats()`); optionally retain a *high* ceiling as a secondary fail-loud backstop. It is no longer the gate.
- **MAX_LEAD_SECS:** revert the 1.0→2.0 band-aid once buffer-time backpressure lands; with the host paced, the over-fill that produced `-142` can't occur, and re-tightening restores fail-loud visibility.

**Rejected alternatives.** (a) Host-side all-nominal buffer-time (`_mcu_pending_end_time`) — the phantom buffer; over-estimates, under-feeds. (b) pump-depth gauge — the 2026-06-21 approach; downstream, doesn't bound submission. (c) Synchronous lookahead like mainline — abandons the parallel planner thread (a deliberate rewrite goal); unnecessary if the engine reports submission-aware queued-time.

---

## Section 4 — Detailed Change Proposals (for bmad-quick-dev)

**`rust/motion-engine` (stream_planner + bridge)**
- Planner thread maintains `uncommitted_intake_secs` (atomic / shared): `+= move.min_move_t` (or feedrate-derived nominal) on intake; on each commit, subtract the nominal of the moves that became committed (their real time is now in `get_last_move_time`). Reset on flush/reset/stream_open.
- Bridge `queued_motion_secs(mcu) -> f64` = `(get_last_move_time − est_anchor) + uncommitted_intake_secs`, clamped ≥ 0. Expose via `motion_engine.py`.
- (Backstop, this pass or next) bound the planner input channel; `submit_move` returns a fail-loud error (never blocks) if full.

**`klippy/motion.py`**
- Rewrite `_check_pause`: gate on `buffer_time = engine.queued_motion_secs(...)`; `reactor.pause` while `> buffer_time_high`, resume `< buffer_time_low`; keep the wall-clock reactor-yield and `DRAIN_TIMEOUT` fail-loud. Skip when `mcu is None` and for drip/homing.
- Use the existing `buffer_time_high (2.0)`/`buffer_time_low (1.0)` config (drop the `pump_backlog_high/low` gate; keep `pump_backlog` in `stats()` as diagnostics).

**`rust/motion-engine/src/pump.rs`**
- Revert `MAX_LEAD_SECS` to its pre-band-aid value.

**Docs / investigation**
- Update the investigation: H1 resolution; note the design lives here.

---

## Section 5 — Implementation Handoff

**Scope:** Moderate — host `motion.py` + engine signal (bridge/planner); Rust backstop; band-aid revert. No wire-format/PyO3-signature change.

**Recipients:** Developer via `bmad-quick-dev`.

**Success criteria (validated on the Neptune bench).**
- Mainsail "requested position" advances smoothly, staying ~1–2 s ahead of actual motion (not jumping to the end); a short print no longer reports complete early.
- Steady-state `buffer_time` oscillates between low/high watermarks; MCU never underruns (no stutter) and never over-fills (no `-142`/`send_frame_fatal`).
- A real multi-layer extrusion print runs past layer 2 to completion; `note_complete` fires near true end.
- A stalled MCU raises a clear `DRAIN_TIMEOUT` error, never hangs.
- `./scripts/ci.sh quick` green; `cargo nextest run -p motion-engine` green; `./scripts/ci.sh py` if `klippy/` changed.
