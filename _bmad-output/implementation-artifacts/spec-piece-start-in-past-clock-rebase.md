---
title: 'Sever the contaminating clock-anchor writer (fix -308 PieceStartInPast)'
type: 'bugfix'
created: '2026-06-22'
status: 'done'
context: []
baseline_commit: 'ab538756d6dc935e8b303275862daa979352f099'
---

<frozen-after-approval reason="human-owned intent — do not modify unless human renegotiates">

## Intent

**Problem:** Prints crash with MCU fault **-308 PieceStartInPast**. The engine router keeps one clock anchor per MCU, but two writers update it every clocksync sample with conflicting conventions: the projection-correct callback (`time_avg + min_half_rtt`, `clock_avg`) and the legacy `serial.set_clock_est` path (`time_avg + TRANSMIT_EXTRA`, `clock_avg − 3·pred_stddev`). The legacy path drags a serialqueue *transmit-safety* bias (~1720 µs, the `3σ` term) into the motion anchor; the two projections differ by ~2 ms — 10× the MCU's 200 µs budget. When it writes last and a piece is then dispatched, the `start_time` lands in the MCU's past → -308 → reset.

**Approach:** Feed the engine anchor from exactly one writer. Remove the contaminating per-sample `self.serial.set_clock_est(...)` call inside `_handle_clock`; the callback already reseeds the same anchor every sample with the smooth, projection-correct values, so removal leaves no gap and no stale pieces. In this fork `serial.set_clock_est` only ever fed the engine (early-returns when `_motion_engine is None`, never touches the C serialqueue), so its transmit-bias values have no other consumer.

## Boundaries & Constraints

**Always:** The engine clock anchor is fed by exactly one writer per clocksync sample — the projection-correct callback (`time_avg + min_half_rtt`, `int(clock_avg)`). Keep all three safety layers intact: smooth single-writer anchor (prevent) + `anchor.rs` re-anchor-forward on underrun (stutter, not crash) + the MCU 200 µs budget (final tripwire).

**Ask First:** If removing the `−3σ` / `TRANSMIT_EXTRA` feed turns out to degrade the engine's command *transmit* timing — i.e. the engine genuinely needs a transmit-scheduling clock distinct from the projection anchor — HALT and give it a **distinct** slot. Never re-contaminate the motion anchor to restore transmit timing.

**Never:** Do not widen the MCU `MAX_START_IN_PAST_SECS` (200 µs) budget — that masks the instability and violates fail-loud. Do not add Rust-side smoothing to hide the jitter. Do not alter the clocksync regression math (`_handle_clock` DECAY/outlier logic) or the Rust `set_clock_est_rebased` projection.

## I/O & Edge-Case Matrix

| Scenario | Input / State | Expected Output / Behavior | Error Handling |
|----------|--------------|---------------------------|----------------|
| Steady-state sample, engine mode | `_handle_clock` regression update | Engine anchor written exactly once, via the callback: offset `time_avg + min_half_rtt`, last_clock `int(clock_avg)`. No `serial.set_clock_est` engine write. | N/A |
| File-output / sim seed | `connect_file` → `serial.set_clock_est(freq, monotonic, 0, 0)` | Engine still receives its synthetic seed (this call path is unchanged). | N/A |
| Non-engine MCU | `_handle_clock` with `_motion_engine is None` | Unchanged: `serial.set_clock_est` already no-ops here; nothing new happens. | N/A |
| Piece dispatched after a sample | `start_time` projected from the single smooth anchor | `now − start_time` stays within the 200 µs MCU budget; no -308. | MCU 200 µs tripwire remains the final guard (untouched). |

</frozen-after-approval>

## Code Map

- `klippy/clocksync.py` -- `_handle_clock` per-sample regression. Both writers live here: the contaminating `self.serial.set_clock_est(...)` (Path A, lines ~172-177) and the projection-correct callback dispatch (Path B, lines ~185-194). **The fix site.** Also defines the now-dead `TRANSMIT_EXTRA` constant (line 12).
- `klippy/serialhdl.py:460` -- `set_clock_est`; engine-only feed, early-returns when `_motion_engine is None`, never calls `serialqueue_set_clock_est`. Confirms Path A is pure engine contamination. Read-only; unchanged.
- `klippy/mcu.py:1279` -- `_engine_clock_est_cb` (Path B), registered via `_clocksync.set_clock_est_callback`. The surviving writer. Read-only.
- `rust/host-rt/src/passthrough_queue/router.rs:332` -- `set_clock_est_rebased`; the single mcu0 anchor both paths wrote (`clock_freq`/`clock_offset`/`last_clock`). Read-only context.
- `rust/runtime/src/motion_core.rs:114` -- `MAX_START_IN_PAST_SECS = 200e-6` tripwire that raises -308. Read-only; must stay untouched.

## Tasks & Acceptance

**Execution:**
- [x] `klippy/clocksync.py` -- In `_handle_clock`, delete the `self.serial.set_clock_est(new_freq, self.time_avg + TRANSMIT_EXTRA, int(self.clock_avg - 3.0 * pred_stddev), clock)` call so the engine anchor is fed only by the projection-correct callback below it. Remove the now-orphaned narration comment and the now-unused `TRANSMIT_EXTRA` module constant (verify no other reference remains). Leave the `connect_file` seed (`serial.set_clock_est(freq, monotonic, 0, 0)`) and the callback dispatch intact.

**Acceptance Criteria:**
- Given engine mode, when `_handle_clock` processes a clocksync sample, then the engine anchor is written exactly once per sample with offset `time_avg + min_half_rtt` and last_clock `int(clock_avg)` — no `TRANSMIT_EXTRA` / `−3σ` values reach the anchor.
- Given a multi-minute print on the Neptune bench (curvature-profile), when it runs, then the print completes with zero -308 PieceStartInPast faults and zero `junction_jump_anomalous` events, and the structured log shows a single `set_clock_est_rebased` record per MCU per sample (was two).
- Given a file-output / sim MCU, when it connects, then the engine still receives its initial synthetic clock seed (`connect_file` path unchanged).

## Design Notes

The call is *removed*, not *fixed*: its transmit-biased arguments exist to schedule serialqueue command transmits early, but the engine replaced the serialqueue and exposes a single clock record (`set_clock_est_rebased`) used for projection — there is no transmit-scheduling slot for those values to feed, so they only ever clobbered the projection anchor. The `clocksync.py:183` comment already warned `TRANSMIT_EXTRA` "must not contaminate the router anchor"; this makes the code honor it. The callback fires on registration (immediate seed) and every subsequent sample, so removal leaves no reseeding gap.

## Verification

**Commands:**
- `./scripts/ci.sh quick` -- expected: green (ruff + rust gates).
- `./scripts/ci.sh py` -- expected: green; clocksync/mcu Python tests pass (klippy/ touched).

**Manual checks (bench, per investigation):**
- Run a multi-minute print on Neptune (`ethercatpi5.local`) on curvature-profile. Via the `query-logs` skill: confirm one `set_clock_est_rebased` per MCU per ~985 ms sample (not two), zero `junction_jump_anomalous`, and the print completes without -308. Larger files (which previously "crashed right away") complete.
