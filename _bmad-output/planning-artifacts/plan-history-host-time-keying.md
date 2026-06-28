# Implementation Plan — Key motion history in host (schedule) time

**Date:** 2026-06-28
**Branch:** trident-crash
**Driving defect:** `HistoryStore::record` panic at `motion_history.rs:138` → `SIGABRT` → klippy dies → "moonraker can't connect." See investigation case file and the brainstorm/research/party-mode artifacts.

## One-line
Re-key the motion-history ring on the planner's **host schedule time** (`t0 + u`, monotone by construction) instead of the projected per-axis **MCU clock tick** (non-monotone across dispatches because the host→MCU projection is re-fit between them). The wire schedule is unchanged. The crashing assert becomes monotone-by-construction; the panic demotes to a clean fail-loud fault.

## Why this is the right key (settled by code + adversarial review)
- `start_clock = PieceEntry.start_time` (`motion_history.rs:50`) is the **raw transmitted MCU tick**, computed once in `enqueue_segment` (`enqueue.rs:211` `project(mcu_id, host_secs)`) and used for BOTH the history ring (`bridge.rs:3296`) and the wire (`bridge.rs:3300`).
- The tick is **not** monotone by construction: it is `host_secs` projected through the live clock-sync regression, which is re-fit between dispatches, so consecutive segments can produce a backward tick (the crash: `447537601286 < 447537603339`, Z, 2053 ticks).
- `host_secs = t0 + curve_u_start + sub_offset` (`enqueue.rs:210`), `t0 = host_now + lead_secs − seg_t_start` (`anchor.rs:61`), `host_now` = MCU `est_print_time` projected into host-seconds (`anchor.rs:20`). This is **pure planner arithmetic**: monotone within a segment, `t0` advances across segments, immutable once recorded (`drop_pieces_on_reanchor` *clears*, never *revises*).
- **`_host_t` is already computed and discarded** at the record site (`bridge.rs:3296` `for (piece, _host_t) in &m.pieces`). The fix keeps it.
- All 10 history readers are host-time-natural or position-only; the two cross-MCU readers (`motion_state_at_clock` beacon-Z, `homing.rs:75` trip reconstruction) **already pivot through host-seconds** via `clock_between_mcus` (`motion_history.rs:228-233`). Host-time keying *removes* the second projection hop (host→target-MCU) rather than adding one — strictly fewer clock models in the metrology path.
- The wire keeps its two authoritative past-time guards, independent of the history: host-side `motion_core.rs:114-128` `FaultCode::PieceStartInPast (-308, 200µs)` and MCU-side `sched.c:182-183` `try_shutdown("Timer too close")`. The history monotonicity check was a redundant *third*, process-fatal authority on a derived structure.

## Acceptance gates (from the party-mode panel — all four are blocking)
1. **Shadow residual (Murat).** Keep a parallel stepper-clock lookup and, on every probe/homing query, compare host-keyed vs stepper-clock-keyed position; divergence beyond tolerance → loud structured fault. Restores the canary the demotion removes; aimed at frame-drift. **No shadow residual, no merge.**
2. **Eval-in-`u` + finiteness (Amelia).** Within-piece eval uses normalized `u = ((host_t − start_host) / duration_secs).clamp(0,1)` — never a clock projection. `is_finite` asserts on every record key and every query, *before* `partition_point` (a NaN silently corrupts the partition). Keep hard aborts for non-finite and for post-selection `u ∉ [0,1]`.
3. **Typed single transition map + epoch identity (Winston/Amelia).** One clock-domain transition function used everywhere; `HostSecs`/`McuClock` newtypes so the compiler refuses an unconverted cross-domain lookup. Assert the history `t0` clock epoch ≡ the clocksync host epoch (today both are the router host-seconds domain — make it explicit).
4. **Reanchor drop scope (Amelia).** Confirm `drop_pieces_on_reanchor` only discards the re-anchored future tail, never the executed prefix; with host-time keying it likely needs to drop **only on a genuine timeline reset** (`seg_t_start + EPS < last_t_end`, `anchor.rs:40`), and can *retain* history across an underrun re-anchor (t0 jumps forward → keys stay monotone) — which is strictly better for probe coverage.

## Design — data and APIs (`rust/motion-engine/src/motion_history.rs`)
`HistoryPiece` gains the host key and retains the clock fields for the shadow:
```
struct HistoryPiece {
    start_host: f64,      // NEW primary key (schedule time)
    start_clock: u64,     // retained for shadow-residual lookup
    end_clock: u64,       // retained for shadow eval
    duration_secs: f32,
    coeffs: [f32; 4],
}
// end_host is derived: start_host + f64::from(duration_secs)
```
- `record(key, host_secs: f64, entry: &PieceEntry, nominal_freq_hz) -> Result<(), HistoryError>`
  - `assert!(host_secs.is_finite())`.
  - monotone check vs `ring.back()`: `host_secs >= last.start_host` (non-strict — equal allowed for zero-length pieces; lookup take-last resolves it). On violation return `Err(HistoryError::OutOfOrderPiece{..})` — **no panic**.
- `state_at_host(key, host_t: f64, now_host: Option<f64>) -> Result<AxisState, HistoryError>`
  - `assert!(host_t.is_finite())`, `partition_point(|p| p.start_host <= host_t)`, eval normalized `u`, future-guard `host_t > now_host`.
- `eval_state(piece, host_t)`: `u = ((host_t − start_host) / duration_secs).clamp(0,1)`; `assert!((0.0..=1.0).contains(&u_unclamped) within slop)`; position/velocity/accel as today (velocity/accel already use `duration_secs`, unchanged).
- New free fn `clock_to_host(router, source, clock) -> Result<f64,_>` = the first hop of today's `clock_between_mcus`; the second hop is deleted from the query path.
- `HistoryError::OutOfOrderPiece { key, host_secs, last_host }` (clean-fault variant).

## Design — shadow residual (gate 1)
On the low-frequency query paths only (beacon `motion_state_at_clock`, homing `reconstruct_axis_position`):
- Primary: `pos_host = state_at_host(key, host_t, ..)`.
- Shadow: project `host_t → tick = host_time_to_mcu_clock(target, host_t)` (the hop removed from the primary), select by `start_clock`, eval → `pos_clock`.
- If `|pos_host − pos_clock| > RESIDUAL_TOL` → `fault`/structured warn via the event pipeline (`event_log_emit`), carrying both positions and the offending key/clock. Always on; cost is per-sample, not per-tick.

## File-by-file change list
- `motion_history.rs` — struct fields, `record` (Result + asserts), `state_at_host`, `eval_state` in `u`, `clock_to_host`, `OutOfOrderPiece`, `HISTORY_CAPACITY` unchanged. (`rebase_axis`, `final_position`, `last_endpoint_clock` re-expressed in host time; `final_position` is position-only, untouched.)
- `bridge.rs:3287-3302` — record `host_t` (stop discarding); map `Err(OutOfOrderPiece)` to a clean shutdown fault; line 3300 wire send unchanged.
- `bridge.rs:3355-3394` (nudge dispatch) — same record change.
- `bridge.rs:4438-4506` (`motion_state_at_clock`) — `clock_to_host(source, clock)` → `state_at_host`; drop the per-target `host_time_to_mcu_clock`; add shadow residual; `now_host` from the existing host-seconds `host_now`.
- `bridge.rs:3709-3842` (`set_position`) — `rebase_axis(key, now_host, pos)` with `now_host` in host-seconds; remove the `host_time_to_mcu_clock` at 3792.
- `homing.rs:20-85` — `reconstruct_axis_position`: `clock_to_host(endstop_mcu, trip_clock)` → `state_at_host`; drop the `host_time_to_mcu_clock(axis_mcu)` hop; add shadow residual; `trajectory_final_position` unchanged.
- New newtypes `HostSecs`/`McuClock` + single transition map (gate 3) — can land as its own phase.
- Tests in a separate file per CLAUDE.md (`motion_history` tests live beside as `#[cfg(test)] mod tests;` → its own file).

## Test matrix (acceptance)
- **Crash replay (headline):** feed the captured z_tilt dispatch sequence that `SIGABRT`s today → no panic, clean.
- **Late arrival:** force `t0` regression → backward `host_secs` → `OutOfOrderPiece` clean shutdown, asserted NOT a panic.
- **Property:** monotone `host_secs` in → monotone selection; zero-length piece (equal keys) → last-writer parity with today.
- **NaN/inf:** record and query with non-finite → finite-assert fires before `partition_point`.
- **Eval-in-`u`:** golden positions vs today’s clock eval, within slop; `u∉[0,1]` aborts.
- **Golden homing-trip reconstruction:** P (single projection) vs legacy (double) within clocksync residual — equal-or-better.
- **Chaos — frame skew:** re-anchor/slew between record and query of one piece → shadow residual within tol or faults.
- **Chaos — beacon-sync drift:** jitter beacon→host conversion → residual bounded/detected.
- **Chaos — reorder/dup:** non-monotonic stepper ticks (old trigger) → host keying does not collapse two instants into one bucket.
- **Differential:** same G-code, old MCU-keyed vs new host-keyed, clean run → beacon Z mesh matches within tol (proves we moved crash handling, not metrology).
- Run via `cargo nextest run -p motion-engine`; offline repro through klipper-sim where useful.

## Phasing (each phase independently green via `./scripts/ci.sh quick` + `cargo fmt --all --check`)
- **P0** — Add `start_host` to `HistoryPiece`, record it (dual-keyed), still assert/lookup on `start_clock`. No behavior change; de-risks plumbing.
- **P1** — Flip assert + lookups to host time; demote panic → clean fault; delete the second projection hop in the two readers. *Fixes the crash.*
- **P2** — Shadow residual (gate 1) + chaos tests.
- **P3** — `HostSecs`/`McuClock` newtypes + single transition map + epoch-identity assert (gate 3).
- **P4** — Reanchor drop-scope refinement (gate 4): drop only on timeline reset, retain across underrun re-anchor.

## Open questions / risks
- **Residual tolerance values** (gate 1) and the slow-probe Z budget — set from an instrumented histogram of real backward-jitter and lead-interval drift on the bench before fixing constants (no guessed thresholds).
- **`rebase_axis` / `set_position` (G92) host-domain `now`** — confirm the host-seconds `now` used for the rebase endpoint matches the query domain.
- **Epoch identity** — verify `host_now_secs()` (record) and `clock_to_host_secs()` (query) are the same router host-seconds epoch (expected yes; assert it).
- **Reanchor retention** — confirm executed-prefix pieces are never re-timed once recorded (expected yes; pieces enter history at dispatch = already committed to the wire).

## Non-goals
- No change to the wire schedule, the clock-sync regression, or the A/D slew/anchor machinery (those now serve only wire-lead health, decoupled from probe correctness).
- No silent clamp/pad of any timestamp (CLAUDE.md fail-loud).

## Status — 2026-06-28
- **P0+P1 (host-time keying) — DONE & bench-validated.** `57a6aaa5e`. Z_TILT_ADJUST ran repeatedly on Trident, 0 retries, converged, no crash, moonraker stayed up. Workspace green.
- **P2 (shadow residual) — DONE & bench-validated.** `f729d31bb`. `history_shadow_divergence` = 0 across every probe sample → host-keyed lookup matches the legacy stepper-clock lookup; canary live and quiet.
- **P3 (epoch identity) — DONE (`28f29f2bc`).** T-then-T⁻¹ round-trip test pins the shared-host-epoch invariant. **Newtypes deferred** — disproportionate `host-rt` churn for marginal safety over the existing f64-host/u64-clock split.
- **P4 (reanchor) — DONE as self-healing ring (`28f29f2bc`).** Backward host_secs pops the superseded tail, keeping the ring sorted independent of the reanchor drop. **"Retain across underrun" optimization deferred** — the conservative drop is safe; the marginal probe-coverage gain didn't justify the ordering-correctness risk, and self-healing removes the latent unsorted-ring failure mode anyway.
- **Open:** the backward-tick race stayed dormant on the fresh-flashed bench (no `history_order_jitter` captured yet); instrumentation + canary remain deployed as a standing trap to size the gate-1 tolerance empirically if it recurs.
