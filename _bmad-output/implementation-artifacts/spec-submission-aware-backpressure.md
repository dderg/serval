---
title: 'Submission-aware motion backpressure: gate gcode on queued-motion-seconds'
type: 'bugfix'
created: '2026-06-22'
status: 'done'
baseline_commit: 'ac22888301d1d37bbbfbcf572e3ccf03ac00870d'
context:
  - '{project-root}/CLAUDE.md'
  - '{project-root}/_bmad-output/planning-artifacts/sprint-change-proposal-2026-06-22.md'
  - '{project-root}/_bmad-output/implementation-artifacts/investigations/print-completes-early-investigation.md'
---

<frozen-after-approval reason="human-owned intent — do not modify unless human renegotiates">

## Intent

**Problem:** Host submission is throttled only by `engine.pump_backlog()` (pump-queue depth, the *last* pipeline stage), which is blind to MCU-buffered motion and stays low until the large MCU ring fills — so the gcode interpreter dumps a ring's worth (or a whole small print) in bursts. Because `commanded_pos`/`gcode_position` are set at *submit* (`motion.py:402`), Mainsail's requested position jumps to the end and `note_complete` fires early; the over-fill then schedules pieces in the MCU's past (`-142` → `fault_event` → `send_frame_fatal`, mid-layer-2 crash). `buffer_time_high/low` exist in config but are only logged, never used to gate.

**Approach:** Gate submission on motion-time-queued-ahead, mainline-style. Expose a *submission-aware* engine signal `queued_motion_secs = (get_last_move_time − est_anchor) + uncommitted_intake_secs` — the planner's real optimized frontier plus a planner-thread nominal tally of received-but-not-yet-committed moves, so the signal advances on **submit**, not only commit. Rewrite `_check_pause` to cooperatively pause the move greenlet while the signal exceeds `buffer_time_high`, resuming below `buffer_time_low`. Revert the `MAX_LEAD_SECS` band-aid and demote `pump_backlog` to diagnostics.

## Boundaries & Constraints

**Always:**
- Backpressure is a cooperative `reactor.pause` in the host move greenlet — **never** a blocking `submit_move`/enqueue (runs under `py.detach`; blocking it freezes `estimated_print_time` → permanent reactor deadlock).
- Fail loud, never hang: keep the `DRAIN_TIMEOUT` deadline on the pause loop (raise `command_error` if the signal never drains → MCU stalled); the planner input channel becomes a **bounded backstop** whose `submit_move` send **errors** (never blocks) when full — a gate-bypass bug signal.
- `uncommitted_intake_secs` is a single-writer (planner-thread) tally: `+= nominal_t` on intake, `-= committed moves' nominal_t` on commit, reset to 0 on flush/reset/stream_open. The bridge FFI only reads it.
- Keep PyO3 move-submission signatures, the MCU wire format/`PieceEntry`, and the emit backend unchanged.

**Ask First:**
- Removing `_mcu_pending_end_time` plumbing (the rejected host-side phantom buffer) — it may feed non-gate logic; this pass only stops *gating* on it.
- Sizing the bounded-channel cap low enough that it could trip in the happy path — it must only trip on a gate bypass.

**Never:**
- Gate on `pump_backlog` (downstream, blind to MCU-ring motion) or on host-side all-nominal `_mcu_pending_end_time` (over-estimates the *whole* buffer → under-feeds/starves the MCU).
- Adopt synchronous mainline-style lookahead — it abandons the parallel planner thread (a deliberate rewrite goal).
- Pause during drip/homing moves or when `self.mcu is None`.
- Change pump scheduling logic or add input shaping.

## I/O & Edge-Case Matrix

| Scenario | Input / State | Expected Output / Behavior | Error Handling |
|----------|--------------|---------------------------|----------------|
| Small print (< one ring) | whole file ingestible at once | host pauses in `move()` once `queued_motion_secs > buffer_time_high`; requested position stays ~1–2 s ahead of motion; `note_complete` near true end | N/A |
| Steady-state extrusion | continuous gcode | signal oscillates between `buffer_time_low`/`high`; MCU ring never underruns (no stutter) nor over-fills (no `-142`) | N/A |
| Tiny print | a few moves | signal never reaches high → no pause; prints as before | N/A |
| MCU not retiring | signal stuck above `buffer_time_low` | pause loop hits `DRAIN_TIMEOUT` → raise `command_error` | raise |
| Gate bypassed (bug) | planner channel fills to cap | `submit_move` send returns a fail-loud error, never blocks | error |
| Drip / homing or `mcu is None` | `drip_move` / sim | `_check_pause` is a no-op | N/A |
| Buffer fully committed | no uncommitted moves | `uncommitted_intake_secs == 0`; `queued_motion_secs = frontier − est` | N/A |

</frozen-after-approval>

## Code Map

- `klippy/motion.py` -- `_check_pause` (573–621) rewrite: gate on `queued_motion_secs(est)` vs `buffer_time_high`/`low`; config (130–142) drop `pump_backlog_high/low` + `_validate_pump_watermarks`, keep `buffer_time_*`; call sites `move` (404), `move_curve` (441), `dwell` (491); drip flag set in `drip_move` (480); `stats()` (825–846) keep `pump_backlog` as diagnostics, switch `buffer_time` to the real signal.
- `klippy/motion_engine.py` -- add `queued_motion_secs(est_anchor)` wrapper (~477, coerce `None`→0.0); `_StubEngine` returns 0.0 (keep OUT of `_STUB_MOTION_METHODS`).
- `rust/motion-engine/src/stream_planner.rs` -- new `uncommitted_intake_secs: Arc<AtomicU64>` (f64-bits, single-writer) created+cloned in `spawn` (79–84); `+= nominal_t` in `handle_move_arrival` (380–407); `-=` committed nominal in the commit/dispatch path (~275 / `state.commit` 570); reset in `StreamOpen`/`Reset`/`Flush` (453–459, 422); channel `unbounded()` (78) → `bounded(CAP)`, `submit_move` send → `try_send` erroring on `Full`.
- `rust/motion-engine/src/bridge.rs` -- mirror the `pump_backlog` plumbing: field (~636), init (~900), clone into `init_planner` spawn (3336–3366); FFI `queued_motion_secs(&self, est_anchor: f64) -> f64` near `get_last_move_time` (3811) = `((last_move_time − est_anchor) + uncommitted).max(0.0)`.
- `rust/motion-engine/src/pump.rs` -- revert `MAX_LEAD_SECS` 2.0 → 1.0 (300); band-aid was commit `48ad66dd4`.

## Tasks & Acceptance

**Execution:** (ordered by dependency — engine signal first, host gate second, revert last)
- [x] `rust/motion-engine/src/stream_planner.rs` -- add the `uncommitted_intake_secs` atomic; accrue `nominal_t` on intake, subtract committed `nominal_t` on commit, reset on flush/reset/stream_open. `nominal_t` = segment length / `feedrate_mm_s` (input nominal, not the optimized time).
- [x] `rust/motion-engine/src/{stream_planner,bridge}.rs` -- bound the planner input channel; `submit_move` send returns a fail-loud error on `Full` (never blocks).
- [x] `rust/motion-engine/src/bridge.rs` -- mirror `pump_backlog` `Arc<AtomicU64>` field/init/spawn-clone; FFI `queued_motion_secs(est_anchor) -> f64` = `((last_move_time − est_anchor) + uncommitted).max(0.0)`.
- [x] `klippy/motion_engine.py` -- `queued_motion_secs(est_anchor)` wrapper (`None`→0.0) + stub 0.0.
- [x] `klippy/motion.py` -- rewrite `_check_pause` to gate on `queued_motion_secs(est)` vs `buffer_time_high/low`; drop the `pump_backlog` watermark config + validator; keep `DRAIN_TIMEOUT` + drip/no-mcu guards; `stats()` keeps `pump_backlog` (diagnostics) and reports the real `buffer_time`.
- [x] `rust/motion-engine/src/pump.rs` -- revert `MAX_LEAD_SECS` to 1.0; re-run the pump suite to confirm the lead-window tests still pass.
- [x] tests -- Rust (`stream_planner` tests): `uncommitted_intake_secs` accrual-on-intake / subtract-on-commit / reset-to-0; bounded-channel fail-loud-when-full. Python (`test/test_motion_backpressure.py`): `_check_pause` pause/resume across the buffer-time watermarks, `DRAIN_TIMEOUT` raise, drip/no-mcu skip.
- [x] `_bmad-output/implementation-artifacts/investigations/print-completes-early-investigation.md` -- mark H1 Resolved; note the design lives in this spec.

**Acceptance Criteria:**
- Given a multi-thousand-move file, when printed on the bench, then the requested position stays ~1–2 s ahead of motion (not jumping to the end) and `note_complete` fires near the true end, with no `-142`/`send_frame_fatal`.
- Given continuous extrusion, when steady-state, then `queued_motion_secs` oscillates between `buffer_time_low`/`high` and the MCU neither underruns nor over-fills.
- Given a stalled MCU, when the signal stays above `buffer_time_low` for `DRAIN_TIMEOUT`, then `_check_pause` raises a clear `command_error` and never hangs.
- Given a drip/homing move or `self.mcu is None`, when issued, then no backpressure pause occurs.
- Given all moves committed and the buffer empty, when queried, then `uncommitted_intake_secs` reads 0.

## Design Notes

**Why the uncommitted tally (closes the 2026-06-21 "commit-gated signal trap").** `get_last_move_time` advances only on commit, so a frontier-only signal stays flat through a submission burst and the gate never closes. The nominal tally (`length/feedrate`) moves the signal on *submit*. It is used **only** for the small uncommitted tail; the gate keeps that tail bounded, so its inaccuracy is negligible — unlike the rejected all-nominal `_mcu_pending_end_time`, which over-estimated the *whole* buffer and starved the MCU.

**`est_anchor` from Python, not an mcu handle.** The planner frontier is a single global print-time on the same clock as `mcu.estimated_print_time(now)` (per investigation Follow-up #3), so Rust just returns the clamped duration. Host call: `buffer_time = engine.queued_motion_secs(self.mcu.estimated_print_time(now))`.

**Single-writer atomic.** Only the planner thread mutates the tally (intake + commit both run in `run_loop`); the FFI only reads. Load/store (Acquire/Release) on f64-bits suffices — no CAS — mirroring `last_move_time_bits`. The bounded channel is a bug-detector, not a throttle: size `CAP` well above one gate-window of moves so it trips only if `_check_pause` is bypassed (fail-loud), never in steady state.

## Verification

**Commands:**
- `cargo nextest run -p motion-engine` -- expected: green incl. new intake-tally + bounded-channel tests.
- `make -f Makefile.rust motion-engine` -- expected: cdylib builds; `klippy/_motion_engine.so` updated.
- `./scripts/ci.sh quick` -- expected: green (ruff, rust-test, clippy `-D warnings`, fmt, watchdog-canary).
- `./scripts/ci.sh py` -- expected: green (klippy touched).

**Manual checks (Neptune bench):**
- A real multi-layer extrusion print runs past layer 2 to completion; Mainsail's requested position tracks motion within ~1–2 s; `stats()` `buffer_time` oscillates between the watermarks; no `-142`.

## Suggested Review Order

**The signal (engine, start here)**

- Entry point — the FFI formula `((frontier − est) + uncommitted).max(0)`; everything else feeds this.
  [`bridge.rs:3823`](../../rust/motion-engine/src/bridge.rs#L3823)
- The crux — `uncommitted_intake_secs` makes the signal track *submit*, not just commit; f64-bits, single-writer.
  [`stream_planner.rs:241`](../../rust/motion-engine/src/stream_planner.rs#L241)
- Accrue on intake (fail-loud on non-positive feedrate).
  [`stream_planner.rs:260`](../../rust/motion-engine/src/stream_planner.rs#L260)
- Subtract on commit — pop exactly `buffered_before − buffered()`; underflow fails loud.
  [`stream_planner.rs:272`](../../rust/motion-engine/src/stream_planner.rs#L272)
- The commit call site that drives the subtraction.
  [`stream_planner.rs:673`](../../rust/motion-engine/src/stream_planner.rs#L673)

**The host gate**

- Gate on `queued_motion_secs(est)` vs `buffer_time_high/low`; cooperative pause, `DRAIN_TIMEOUT` fail-loud.
  [`motion.py:565`](../../klippy/motion.py#L565)
- Extracted reactor-yield helper (name carries the intent; no narration comment).
  [`motion.py:558`](../../klippy/motion.py#L558)
- The previously-inert `buffer_time_high/low` config now wired to the gate.
  [`motion.py:130`](../../klippy/motion.py#L130)
- `stats()` reports the real signal; `pump_backlog` demoted to diagnostics.
  [`motion.py:805`](../../klippy/motion.py#L805)
- Wrapper (`None`→0.0) + stub 0.0.
  [`motion_engine.py:476`](../../klippy/motion_engine.py#L476)

**Fail-loud backstop & band-aid revert**

- Bounded planner channel — `submit_move` errors (never blocks) when full.
  [`stream_planner.rs:223`](../../rust/motion-engine/src/stream_planner.rs#L223)
- Revert the `MAX_LEAD_SECS` 2.0→1.0 band-aid (`48ad66dd4`).
  [`pump.rs:300`](../../rust/motion-engine/src/pump.rs#L300)

**Tests (supporting)**

- Tally survives the real partial-commit + head-trim path; exact-zero after flush.
  [`tests.rs:391`](../../rust/motion-engine/src/stream_planner/tests.rs#L391)
- Full channel errors instead of blocking.
  [`tests.rs:442`](../../rust/motion-engine/src/stream_planner/tests.rs#L442)
- Host gate: pause/drain-to-low and `DRAIN_TIMEOUT` raise.
  [`test_motion_backpressure.py:85`](../../test/test_motion_backpressure.py#L85)
