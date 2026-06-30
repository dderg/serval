---
title: 'MCU clock synchronization — per-MCU locked reference clock + drift fault'
type: 'feature'
created: '2026-06-30'
status: 'done'
baseline_commit: '3f46874438e4868b0114de6f4adcb0c4e203b927'
context:
  - '{project-root}/_bmad-output/specs/spec-clock-sync/SPEC.md'
  - '{project-root}/docs/rewrite/mcu-c-rust-boundary.md'
---

<frozen-after-approval reason="human-owned intent — do not modify unless human renegotiates">

## Intent

**Problem:** MCU clocks drift relative to each other, desyncing axes split across MCUs. Today the host re-anchors the next piece's start time by the drift duration (`anchor.rs` underrun branch shifts `t0`), producing discrete motion: every sync gaps or overlaps pieces, and an overlap trips the fatal `-308 PieceStartInPast`. Drift is already measured (`ClockSyncEstimator`) but only used to re-project start times, not to rate-match. (Full rationale: linked SPEC.)

**Approach:** Give each MCU a **locked reference clock mapping** `(ref_freq, ref_offset, ref_anchor)` that the host *owns* — captured **once**, as a snapshot of the live synced clock, when that MCU's first piece is about to be sent (by then the live clock has stabilized), and then frozen for good. All motion pieces and the endstop trip-time inversion project against this locked reference, so a later re-sync can never move where a host time lands → consecutive pieces stay contiguous (no gaps/overlaps, no piece scheduled in the past). There is **no active rate correction** (no continuous re-trim — re-trimming a multiplier against absolute elapsed time would itself reintroduce seam jumps). The live re-synced clock keeps being rebased and is used only for acks / the `-308` scheduling check / 32→64-bit widening, and to *monitor* drift against the locked reference: when the live clock diverges beyond `MAX_DRIFT_PPM_DEFAULT = 100` for **3 consecutive** samples, fail loud with a new distinct fault that actually halts (`invoke_async_shutdown`). The coarse offset re-base happens between prints (idle), not mid-run.

## Boundaries & Constraints

**Always:**
- Queue pieces against the **locked reference**, never the continuously re-synced clock — that jump-on-rebase is the root cause of the boundary gaps/overlaps.
- Capture the reference **once**, when the MCU's first piece is about to be sent, then keep it locked for the run (never re-captured mid-print). `is_queue_empty` (if it exists) means only what it says; it is not the capture gate.
- Monitor drift from the live re-synced `freq` measured against the **locked reference** (`drift = live vs ref_freq`); reuse the existing measurement, do not add a parallel estimator.
- Host-side only — no new MCU shared state, nothing on the C/Rust seam (per `mcu-c-rust-boundary.md`).
- The projection and its inverse (endstop trip-time) go through the **same** locked-reference base so host→MCU and MCU→host stay consistent. Naming must make clear which clock each time is in (live-synced vs locked-reference; host-seconds vs MCU-ticks).
- The unsyncable-drift fault fires on the **commanded** live-vs-reference drift at sample time — never wait for delayed effect — after 3 consecutive out-of-bound samples, and must reach a real shutdown (`invoke_async_shutdown`, latched once), not just a logged exception.
- Correctable bound reuses `MAX_DRIFT_PPM_DEFAULT = 100`; beyond it for 3 consecutive samples the fault fires.
- Before a reference is captured (bootstrapping, clock not yet synced), fall back to the live clock so behavior equals today's.

**Ask First:**
- Any change to the `klippy/clocksync.py` heater clocksync.

**Never:**
- Do not touch the mainline non-motion clocksync (`klippy/clocksync.py`) — it stays byte-for-byte identical.
- Do not re-tune, re-derive, or replace the frequency/drift estimator.
- Do not overload `-308 PieceStartInPast` for the unsyncable fault.
- Do not actively re-correct the rate mid-run (no continuous re-trim): once the reference is locked, the mapping is fixed; drift is detected and faulted, not chased.
- Do not change trajectory shape — this is timing only.
- Do not attempt to fix the PC8 stale-`print_time` homing-after-idle guard (`klippy/mcu.py:490`) — out of scope.

## I/O & Edge-Case Matrix

| Scenario | Input / State | Expected Output / Behavior | Error Handling |
|----------|--------------|---------------------------|----------------|
| First piece for an MCU | Live clock synced | Reference frozen once from the live clock; all subsequent pieces project against it | N/A |
| Live clock re-synced mid-run | Reference already locked | A given host time projects to the **same** MCU tick before and after the re-sync (no seam jump) | N/A |
| Drift within bound | Live vs reference ≤ 100 ppm | Pieces stay contiguous; streak counter resets each in-bound sample; no fault | N/A |
| Divergence beyond bound | Live vs reference > 100 ppm for 3 consecutive samples | `ClockDivergenceUnsyncable (-112)` raised at the 3rd sample, magnitude in detail; `mcu.py` `invoke_async_shutdown` | Fail loud; halts; fires once (latched) |
| Endstop trip under a locked reference | Homing trip clock, reference active | Trip clock MCU→host inverts through the same locked reference; reconstructed position matches the live-clock baseline within tolerance | N/A |
| First piece before clock synced | `ref_freq` unset, `clock_freq == 0` | Capture is a no-op (warns); projection falls back to the live clock until a reference can be frozen | warn, no fault |

</frozen-after-approval>

## Code Map

- `rust/host-rt/src/passthrough_queue/router.rs` -- `McuRecord`: **live** `clock_freq`/`clock_offset`/`last_clock` (rebased) + **locked** `ref_freq`/`ref_offset`/`ref_anchor` + `out_of_bound_streak`. `projection_base()` returns the locked reference when captured, else live. `host_time_to_mcu_clock`/`clock_to_host_secs`/`print_time_to_host_secs` all use `projection_base()` (no ratio). `capture_reference` (one-shot), `reference_freq`, `note_drift_sample` (streak), and `check_drift() -> DriftCheck` (the integrated, unit-tested fault decision: compares live vs locked reference, resets the streak on indeterminate `freq<=0`, returns `Fault{drift_ppm}` on the 3rd consecutive out-of-bound).
- `rust/host-rt/src/clock_sync.rs` -- `drift_ppm_between(live, reference)`, `drift_within_authority` (≤ `MAX_DRIFT_PPM_DEFAULT=100`), `MAX_CONSECUTIVE_OUT_OF_BOUND=3`. Estimator unchanged (`drift_ppm` delegates). No trim/gain.
- `rust/motion-engine/src/anchor.rs` -- reverted to `(t0, fresh)`; no reset signal needed.
- `rust/motion-engine/src/bridge.rs` -- both dispatch sites: capture the reference **once** (`reference_freq().is_none()` → `capture_reference`) before projecting; **fail loud** (`DispatchError::ReferenceCaptureFailed`) if the clock isn't synced at first piece. `set_clock_est`: reject non-finite/negative `freq` up front, then `router.check_drift` → `-112` via `clock_divergence_unsyncable_err` on the `Fault` outcome.
- `klippy/mcu.py` -- `_engine_clock_est_cb` calls `invoke_async_shutdown` (reactor-thread-safe) on `set_clock_est` failure, latched to fire once; `clocksync.py` untouched.
- `rust/motion-engine/src/homing.rs` + `motion_history.rs` -- endstop trip-time MCU→host inversion via `clock_to_host_secs` → locked-reference base; round-trip preserved.
- `rust/runtime/src/error.rs` -- `FaultCode::ClockDivergenceUnsyncable = -112` (+ `from_u16`, `code_name`, test list). Acks/`-308`/widening (`compute_ack_clock` etc.) deliberately stay on the **live** clock.

## Tasks & Acceptance

**Execution:**
- [x] `rust/host-rt/src/passthrough_queue/router.rs` -- live + locked-reference fields (documented), `projection_base()`; `capture_reference` one-shot (no-op if already locked; false if clock unsynced); `reference_freq`; `note_drift_sample` streak counter; `host_time_to_mcu_clock`/`clock_to_host_secs`/`print_time_to_host_secs` all on `projection_base()`; removed the `speed_ratio` trim machinery + `is_queue_empty` + `InvalidSpeedRatio`.
- [x] `rust/host-rt/src/clock_sync.rs` -- `drift_ppm_between`, `drift_within_authority`, `MAX_CONSECUTIVE_OUT_OF_BOUND=3`; removed trim/gain helpers; `drift_ppm` delegates (estimator unchanged).
- [x] `rust/motion-engine/src/anchor.rs` -- reverted to `(t0, fresh)`.
- [x] `rust/motion-engine/src/bridge.rs` -- capture-once-at-first-piece in both dispatch sites (fail-loud on capture error, warn if unsynced); `set_clock_est` drift-streak fault (`note_drift_sample` → `-112` on 3rd consecutive out-of-bound, before rebase); removed the trim.
- [x] `rust/motion-engine/src/stream_planner.rs` -- `DispatchError::ReferenceCaptureFailed`.
- [x] `klippy/mcu.py` -- `_engine_clock_est_cb` uses `invoke_async_shutdown`, latched once; `clocksync.py` untouched.
- [x] `rust/runtime/src/error.rs` -- `ClockDivergenceUnsyncable = -112` in enum, `from_u16`, `code_name`, exhaustiveness test list.
- [x] `rust/host-rt/src/clock_sync/ratio_tests.rs` -- drift definition (both signs), within-authority at/below/above the inclusive ±bound, threshold = 3.
- [x] `rust/host-rt/src/passthrough_queue/router/tests.rs` -- `capture_reference` needs a synced clock; **one-shot** (second call after a live move does not re-freeze); **frozen reference keeps forward + inverse projection stable across a live re-sync** (the core anti-gap property); `note_drift_sample` streaks and resets.
- [x] `rust/motion-engine/src/anchor/tests.rs` -- restored to the `(t0, fresh)` suite.
- [x] `rust/motion-engine/src/homing/tests.rs` -- endstop reconstruction against a locked reference matches the live-clock baseline.

**Acceptance Criteria:**
- Given an MCU's first piece, when it is about to be sent, then the reference is frozen once from the live clock and never re-captured for the run.
- Given a locked reference, when the live clock is re-synced, then a given host time projects to the **same** MCU tick, forward and inverse (no boundary jump) — verified by `frozen_reference_keeps_piece_projection_stable_across_resync`.
- Given live-vs-reference drift > 100 ppm for 3 consecutive samples, when `set_clock_est` runs, then it raises `-112` and `mcu.py` `invoke_async_shutdown`s (fail-loud actually halts, once).
- Given `klippy/clocksync.py`, when this feature lands, then it is byte-for-byte unchanged.
- Given drift monitoring, when it runs, then its only input is the live re-synced `freq` measured against the locked reference (no parallel estimator, no active rate correction).

## Spec Change Log

### 2026-06-30 — review loop 1: stable-reference redesign (dderg)
- **Trigger.** Adversarial review found two blockers in the first implementation: (1) the `nominal/measured` trim was applied to the *re-synced* `clock_freq`, which (a) biased projection toward nominal — counterproductive for cross-MCU coordination — and (b) still queued pieces against the clock that jumps on every rebase, so the gap/overlap root cause remained; (2) the `-112` `PyErr` out of `set_clock_est` is swallowed by `klippy/mcu.py`/`clocksync.py` `try/except` → no shutdown, so CAP-3 fail-loud never fires.
- **Resolution (dderg).** Do not queue against the re-synced clock at all. Introduce a host-owned **frozen reference** captured on `timeline_reset` when the queue is empty; queue pieces against `reference × speed_ratio`; `speed_ratio` trims toward `measured/ref_freq` (drift of the live clock vs the frozen reference). Fail-loud must reach `invoke_shutdown` in `mcu.py`.
- **Known-bad avoided.** Queuing against the rebased clock (boundary jumps); trim biasing to nominal (wrong base + wrong sign); a fault that only logs.
- **KEEP.** `FaultCode::ClockDivergenceUnsyncable = -112` wiring; the locked-reference idea; the convention that acks/`-308`/widening use the live clock while projection+endstop use the reference.

### 2026-06-30 — review loop 2: lock once, no retrim (dderg)
- **Trigger.** Loop-2 review found: (a) **blind** — applying the trim as `(host − ref_offset)·ref_freq·speed_ratio` with a *changing* `speed_ratio` re-introduces seam jumps amplified by elapsed time (frozen base, but a non-frozen 4th factor) → defeats contiguity under drift on long prints; (b) **edge/acceptance** — `invoke_shutdown` runs shutdown handlers on the serial-reader thread (must be `invoke_async_shutdown`); fault re-fires every sync (no latch); `print_time_to_host_secs` left split across two clock bases; `is_queue_empty` (host ready-queue) doesn't reflect the MCU ring.
- **Resolution (dderg).** Stop chasing the rate. **Set the reference once**, when the first piece is about to be sent (the live clock has stabilized by then), and **keep it locked** — no re-capture, no re-trim. Remove `speed_ratio` entirely (it would be permanently `1.0`). Drift is **monitored, not corrected**: fault `-112` after 3 consecutive samples beyond 100 ppm. `invoke_async_shutdown`, latched. `print_time_to_host_secs` routed through the locked reference. `is_queue_empty` removed (was only the old capture gate; capture is now "first piece, once").
- **Known-bad avoided.** A time-amplified seam jump from a varying ratio; a non-thread-safe shutdown; shutdown spam; a split torque-path clock base; mis-naming a host-queue check as MCU emptiness.
- **KEEP.** `-112` wiring; the live-vs-reference clock split; the host-only boundary; clear clock naming.

### 2026-06-30 — review loop 3: fail-loud hardening + test discrimination (patches)
- **Trigger.** Loop-3 review found no correctness defect (architecture sound; all prior blockers resolved) but flagged: the integrated 3-consecutive→`-112` path and the homing/inverse tests didn't *discriminate*; non-finite/negative `freq` could reach the live clock; an indeterminate (`freq<=0`) sample could bridge an out-of-bound streak; first-piece-before-sync warned instead of failing loud; a stale log message still said "speed-ratio trim / correctable".
- **Resolution.** Extracted the fault decision into `PassthroughRouter::check_drift -> DriftCheck` (unit-tested: 3-consecutive faults, an in-bound or indeterminate sample resets, no-reference is Ok); `set_clock_est` rejects non-finite/negative `freq` (fail loud) and delegates to `check_drift`; capture failure at first piece now returns `DispatchError::ReferenceCaptureFailed` (no silent live-fallback, closes the multi-MCU partial-capture gap); homing + router-inverse tests re-sync the live clock after capture / use an independent probe tick so they fail if the locked reference is bypassed; reworded the fault message to monitor-only language; corrected the live-clock field doc (`-308` is firmware-raised).
- **KEEP.** Everything from loops 1–2 plus the `check_drift` integrated decision as the single fault-policy site.

## Design Notes

Two clocks per MCU, named to keep their domains distinct:
- **Locked reference** `(ref_freq, ref_offset, ref_anchor)`: a one-shot snapshot of the live clock, taken by `capture_reference` the first time a piece is dispatched for that MCU, then frozen. `host_time_to_mcu_clock` / `clock_to_host_secs` / `print_time_to_host_secs` project against this via `projection_base()`. Because the whole mapping is fixed, every host time maps to the same MCU tick for the life of the run → consecutive pieces are contiguous by construction; a re-sync can never schedule a piece in the past.
- **Live re-synced clock** `(clock_freq, clock_offset, last_clock)`: keeps being rebased by `set_clock_est_rebased`; used by `compute_ack_clock`/`ack_clock_and_freq`/`wall_time_at_mcu` (they need the MCU's *real current* tick) and for drift monitoring.

No active rate correction. Each sample:

```
if reference locked:
    drift = (live_freq − ref_freq) / ref_freq            // live vs the locked reference
    streak = within_bound ? 0 : streak + 1               // note_drift_sample
    if streak >= 3:  raise ClockDivergenceUnsyncable(drift) → invoke_async_shutdown (latched)
```

Why no trim: any multiplier that changes during the run multiplies the absolute elapsed time from the frozen anchor, so a late-print change yanks the seam by `elapsed · ref_freq · Δratio` — the exact `-308` gap the lock was meant to prevent. Locking the entire mapping and failing loud on excessive divergence is the contiguity-preserving choice; the coarse offset re-base happens between prints (idle), outside this spec.

## Verification

**Commands:**
- `cd rust && cargo nextest run -p host-rt -p motion-engine -p runtime` -- expected: all green (drift helpers, capture-once, frozen-projection-stable, drift-streak, endstop round-trip).
- `./scripts/ci.sh quick` -- expected: fully green (ruff, rust-test, rust-clippy `-D warnings`, rust-fmt, watchdog-canary).
- `./scripts/ci.sh py` -- expected: green (touches `klippy/mcu.py`).
- `./scripts/ci.sh rust-mcu-h7 && ./scripts/ci.sh rust-mcu-f4 && ./scripts/ci.sh rust-mcu-g0` -- expected: all build (`runtime` is in the diff).

**Manual checks (if no CLI):**
- Two-MCU bench run (trident-bench, axes split across H723 + F446) with clocks measurably out of rate: coordinated print completes, no `-308`, no visible gaps/overlaps at piece boundaries, homing triggers at the correct position; a deliberately >100 ppm divergence produces a clean `-112` shutdown (not silent bad motion). Inspect via `mcu-diagnostics`/`query-logs` for the `capture_reference` event and absence of `-308`.

## Suggested Review Order

**The core idea — two clocks per MCU**

- Entry point: the live-vs-locked split and which consumers read which clock.
  [`router.rs:59`](../../rust/host-rt/src/passthrough_queue/router.rs#L59)
- The projection base — locked reference when captured, else live fallback.
  [`router.rs:88`](../../rust/host-rt/src/passthrough_queue/router.rs#L88)
- One-shot capture: snapshot the live clock once, never re-freeze.
  [`router.rs:443`](../../rust/host-rt/src/passthrough_queue/router.rs#L443)
- Forward projection now rides `projection_base()` (no ratio), the anti-gap property.
  [`router.rs:605`](../../rust/host-rt/src/passthrough_queue/router.rs#L605)
- Endstop inverse rides the same base so homing round-trips.
  [`router.rs:580`](../../rust/host-rt/src/passthrough_queue/router.rs#L580)

**Fail-loud drift monitoring**

- The integrated fault decision (3 consecutive out-of-bound, indeterminate resets).
  [`router.rs:495`](../../rust/host-rt/src/passthrough_queue/router.rs#L495)
- Bound + consecutive-sample threshold.
  [`clock_sync.rs:19`](../../rust/host-rt/src/clock_sync.rs#L19)
- `set_clock_est`: reject bad freq, then `check_drift` → `-112`.
  [`bridge.rs:2556`](../../rust/motion-engine/src/bridge.rs#L2556)
- The host-raised `-112` fault (log + PyErr, monitor-only wording).
  [`bridge.rs:632`](../../rust/motion-engine/src/bridge.rs#L632)
- Wire-stable fault code in the `-11x` clock-sync family.
  [`error.rs:134`](../../rust/runtime/src/error.rs#L134)
- The shutdown actually halts: reactor-marshalled, latched once.
  [`mcu.py:1280`](../../klippy/mcu.py#L1280)

**Capture-at-first-piece wiring**

- Main dispatch: capture once before projecting; fail loud if unsynced.
  [`bridge.rs:3275`](../../rust/motion-engine/src/bridge.rs#L3275)
- Nudge/drip path: same capture rule.
  [`bridge.rs:3409`](../../rust/motion-engine/src/bridge.rs#L3409)

**Tests (supporting)**

- Drift-fault discrimination: 3-consecutive, reset, indeterminate, no-reference.
  [`router/tests.rs`](../../rust/host-rt/src/passthrough_queue/router/tests.rs)
- Frozen-reference stability across a live re-sync (forward + inverse).
  [`router/tests.rs`](../../rust/host-rt/src/passthrough_queue/router/tests.rs)
- Endstop reconstruction follows the locked reference after a re-sync.
  [`homing/tests.rs`](../../rust/motion-engine/src/homing/tests.rs)
- Drift helpers + inclusive bound + threshold.
  [`ratio_tests.rs`](../../rust/host-rt/src/clock_sync/ratio_tests.rs)
