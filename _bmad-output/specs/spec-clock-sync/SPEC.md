---
id: SPEC-clock-sync
companions: []
sources: [../../../docs/human-spec/clock-sync.md]
---

> **Canonical contract.** This SPEC and the files in `companions:` are the complete, preservation-validated contract for what to build, test, and validate. Source documents listed in frontmatter are for traceability only — consult them only if you need narrative rationale or prose color this contract intentionally omits.

# MCU clock synchronization (per-MCU time multiplier)

## Why

Pain to solve. Each MCU runs off its own crystal, so the chips' clocks drift relative to each other — sometimes as bounded jitter, sometimes as a steady rate difference (one clock genuinely faster than another). When a single logical axis or a coordinated motion is split across MCUs, that drift desyncs them: pieces queued at the previous piece's end on a faster clock arrive before the slower clock is ready, or vice versa. Today the host holds a per-MCU **offset** between host clock and MCU clock (`clock_offset`, `rust/host-rt/src/passthrough_queue/router.rs:56-57`) and the current correction (`rust/motion-engine/src/anchor.rs:31-66`) re-anchors the next piece's start time by the drift duration — `anchor_segment` shifts `t0` forward when it detects an underrun (`t0 + seg_t_start < host_now`). That shift produces discrete, inconsistent motion: every sync either inserts a gap or overlaps adjacent pieces, and an overlap pushes a piece's `start_time` behind the stepping playhead, tripping the runtime fail-loud `PieceStartInPast` (`-308`, `rust/runtime/src/motion_core.rs:125-128`; >~200 µs late = fatal MCU halt). The drift signal is already measured well — a per-MCU Kalman-style frequency/drift estimator exists (`rust/host-rt/src/clock_sync.rs`, `drift_ppm` at `:310`) — but it is only used to *re-project start times*, not to *rate-match the motion itself*. The fix is to add a per-MCU **speed ratio** alongside the stored offset and continuously speed up or slow down the pieces pushed to each MCU so coordinated axes track each other smoothly, instead of papering over drift with start-time jumps that gap, overlap, and fault. The offset can be re-established while everything is idle; the ratio is tuned continuously while running.

## Capabilities

- id: CAP-1
  intent: The planner applies a per-MCU time multiplier to the timing of newly pushed pieces, scaling them to match that MCU's measured clock rate so coordinated axes on different MCUs stay synchronized without discrete start-time shifts.
  success: On a bench/sim run with two MCUs whose clocks run at measurably different rates, axes split across them hold sync through a continuous print with no piece-boundary gaps or overlaps and no `-308` `PieceStartInPast`; the multiplier is stored and updated per MCU at clock-sync time, and disabling it (multiplier ≡ 1.0) reproduces today's gap/overlap/`-308` behavior.

- id: CAP-2
  intent: The multiplier update is gain-limited so corrections converge toward the true rate difference without hunting — a meaningful correction each sync, but not one that overshoots and oscillates.
  success: Against a step change in a simulated MCU clock rate, the multiplier converges toward the true ratio and settles within a bounded band; it does not oscillate beyond a defined tolerance across successive updates (a regression test asserts monotone-ish convergence and no sustained overshoot).

- id: CAP-3
  intent: When the correction needed to sync an MCU exceeds the multiplier's correctable authority — clocks too far apart to rate-match — the system fails loud immediately with a clear, distinct fault code rather than silently attempting to compensate.
  success: Injecting a clock divergence beyond the correctable bound raises an immediate, distinctly-coded fault at the moment the out-of-bound correction is *commanded* (not after the adjusted pieces drain), and no silent compensation or unbounded multiplier is ever applied; the fault detail carries the offending drift/correction magnitude.

- id: CAP-4
  intent: Homing stays correct under the per-MCU offset+ratio clock mapping — the endstop-trigger timestamp an MCU reports is converted back to host time through the same offset and speed ratio applied to that MCU, so the triggered position is computed correctly.
  success: A homing run on an MCU carrying a non-unity speed ratio reports a triggered position consistent (within tolerance) with the unity-ratio baseline; unit tests cover the endstop trigger-time round-trip through the offset+ratio mapping in both directions.

## Constraints

- **Fail-loud is immediate and decoupled from effect verification.** The corrective effect of a multiplier is only observable *after* the adjusted pieces are consumed (the loop is inherently delayed). The unsyncable-drift fault (CAP-3) must therefore trigger on the *commanded* correction magnitude at push time, never wait for delayed feedback — so an operator learns to tune the algorithm rather than have it silently chase way-out-of-sync clocks. Consistent with the project fail-loud rule: late/un-syncable timing is faulted, never padded or advanced.
- **Reuse the existing per-MCU frequency/drift estimate as the input signal; do not add a parallel estimator.** The measurement already exists (`ClockSyncEstimator`, `clock_sync.rs:34-62`; `drift_ppm` `:310`; quality gates `is_quality_gate_passed` `:333` with `MAX_DRIFT_PPM_DEFAULT = 100`, `MAX_RESIDUAL_US_DEFAULT = 100`; cross-MCU gate `MAX_CROSS_MCU_FREQ_RATIO_OFFSET = 1e-3` in `stream.rs`). This work is the *actuation* — turning that drift signal into a piece-timing multiplier — not new sensing.
- **The multiplier supersedes routine drift re-anchoring; it does not replace genuine resets.** The steady-state gap/overlap behavior in `anchor.rs:31-66` (re-anchor on `underrun`) is what this replaces for ordinary clock drift. Re-anchoring on a real `timeline_reset` (e.g. a fresh print start) stays.
- **Host-side correction only — no new MCU-side shared state.** The multiplier is applied where piece timings are generated/projected on the host (`rust/motion-engine/src/bridge.rs:3237-3274`, `router.rs:427-436` `host_time_to_mcu_clock`); this keeps the change inside the host runtime and off the C/Rust MCU seam per `docs/rewrite/mcu-c-rust-boundary.md`.
- **Two correction levers per MCU: offset and speed ratio.** The stored host↔MCU offset may be re-established while the printer is idle; the speed ratio is tuned continuously while running. Both are valid; the ratio is the steady-state mechanism, the idle offset re-base is a coarse reset.
- **Correction is bounded and per-MCU.** One speed ratio per MCU, updated at clock-sync, with a defined maximum deviation from 1.0; beyond it, CAP-3 fires rather than the ratio saturating silently. The correctable bound reuses the existing quality gate — `MAX_DRIFT_PPM_DEFAULT = 100` (`clock_sync.rs:15`) — rather than introducing a new limit.
- **Endstop timestamps round-trip through the same offset+ratio.** Any host-side time mapping the ratio touches must be applied (and inverted) consistently on the endstop-trigger path, so homing is never silently desynced by an active ratio.

## Non-goals

- **Do not touch the mainline non-motion clocksync.** Heaters and other non-motion devices keep using `klippy/clocksync.py` (`ClockSync`) exactly as in mainline; it stays byte-for-byte identical and is the upstream source that seeds the per-MCU frequency estimate (`set_clock_est_callback` → `router.set_clock_est_rebased`).
- Re-tuning, re-deriving, or replacing the frequency/drift *estimator* itself — it is reused as-is.
- Guaranteeing a fix for the `digital_out` PC8 (Z-enable) "scheduled with stale print_time" host guard (`klippy/mcu.py:490`, `MIN_SCHEDULE_LEAD = 0.050`) on first homing after idle — investigated as possibly-related (see Open Questions), but it is a separate host-side lead guard, not the `-308` motion path, and is not a committed deliverable here.
- Closed-loop trajectory-time optimization or any change to trajectory shape — sync corrects *timing rate*, not the planned path.

## Success signal

A coordinated print whose axes are split across two MCUs runs to completion with the clocks measurably out of rate, and the axes stay in sync the whole way — no `-308 PieceStartInPast`, no visible gaps or overlaps at piece boundaries where today's shift-based correction would jump — homing still triggers at the correct position with a non-unity ratio active, while a deliberately out-of-bound clock divergence produces an immediate, clearly-coded fail-loud instead of silent drift compensation. Verified by unit tests (ratio convergence, fail-loud bound, endstop trigger-time round-trip) plus a bench run.

## Assumptions

- The per-MCU speed ratio scales the timing/duration of pushed pieces and is applied host-side at piece generation/projection, introducing no new MCU shared state (consistent with the C/Rust boundary doc). Confirmed: multiplier is host-only.
- The unsyncable-drift fault is a new, distinct code (or a clearly-attributed reuse of the clock-sync-quality fault family `ClockSyncQuality = -110` / `ClockSyncTimeout = -111`, `error.rs:132-133`) rather than overloading `-308 PieceStartInPast`, which stays the runtime "piece arrived late" fault.
- The corrective effect is verified empirically — unit tests (ratio convergence, fail-loud bound, endstop round-trip) plus a bench run — rather than by a runtime closed loop on realized residual drift; the fail-loud decision is open-loop at command time (bound checked on the commanded correction).

## Open Questions

- Is the PC8 "scheduled with stale print_time" fault on first homing after idle (host guard `mcu.py:490`, `lead=-397.5ms`) the same root cause as the motion drift, or an independent staleness in `estimated_print_time` after the printer sits idle? Tracked as a non-goal for this spec; left open as the one question to revisit if it surfaces during implementation.
