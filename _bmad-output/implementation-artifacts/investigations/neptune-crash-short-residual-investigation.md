# Investigation: neptune_crash_short PieceStartInPast — residual after all landed fixes

## Hand-off Brief

1. **What happened.** `neptune_crash_short.gcode` still faults the MCU with `PieceStartInPast` (-308) on the neptune bench, timing identical to before the replan short-circuit (`3d647b6f2`). Mechanism Confirmed: `rust/runtime/src/motion_core.rs:114` raises -308 when a dispatched piece's start is >200µs behind MCU `now`.
2. **Where the case stands.** The fixes a first-principles brainstorm proposed (finality-barrier eviction + batched re-plan cadence) are **already shipped** on HEAD. The offline "planner 6–10× too slow" number that looked like the root cause was measured with `repro_plan_stall --cap 1`, which **bypasses the production 64-move coalescing** — so it overstated cost. Root-cause confidence is RETRACTED to Medium.
3. **What's needed next.** A fresh VictoriaLogs bench trace to measure the *actual* coalesced batch size, plan wall time, and lead during the fault — discriminating (a) coalescing defeated by throttled gate feed, (b) an irreducibly large open tail from sub-0.5mm facets (→ arc-fitter), or (c) the gate/clock paths.

## Case Info

| Field | Value |
| ----- | ----- |
| Ticket | N/A |
| Date opened | 2026-06-24 |
| Status | Active — root-cause claim retracted; awaiting representative measurement |
| System | neptune bench (Pi host + H7/F446, our fork); branch `ethercat-ipc-hardening` @ `cab999e37` |
| Evidence sources | 4 prior investigations; current Rust source; `repro_plan_stall` (caveated); `spec-incremental-stream-planning.md` (done) |

## Problem Statement

The replan short-circuit (`3d647b6f2`) was predicted to be necessary-but-insufficient (it only skips empty commits). User confirms identical timing/crash after deploying it. The task was to find the true root cause.

## Confirmed Findings

### Finding 1: fault mechanism + thread architecture

`-308` raised at `runtime/src/motion_core.rs:114` (`MAX_START_IN_PAST_SECS = 200e-6`), via `fault_helpers.rs:109`. Pump and planner are **separate threads** (`bridge.rs:2800` `run_pump`; `bridge.rs:3364` planner → `run_loop` `stream_planner.rs:701`). A long plan freezes the *committed frontier*, not dispatch; the pump starves only if the frontier fails to keep its lead (`pump.rs:335` `MAX_LEAD_SECS = 2.0`) positive.

### Finding 2: the eviction + batch-cadence fixes the brainstorm proposed are ALREADY SHIPPED

- **Finality-barrier eviction** — `spec-incremental-stream-planning.md` (status `done`, baseline `c25061e58`). Barrier = reconvergence point of the backward velocity sweep (`velocity.rs` `VelocityProfile.barrier`); commit held back by `brake_to_rest_setback = v_peak·t_brake` (static braking-distance window). Eviction shrinks the buffer to the open tail (≈ one braking distance) ⇒ each solve bounded, flat-in-depth. **Explicitly forbids a fit/plan/lower cache** ("if reuse seems required, the barrier logic is likely wrong — HALT").
- **Batched cadence** — `run_loop` `stream_planner.rs:757-779` coalesces up to `COALESCE_BATCH_MOVES = 64` (`:125`) via `try_recv`, commits ONCE; the inline comment cites the same O(n²)-per-move reasoning the brainstorm re-derived.

### Finding 3: 36% of facets are sub-0.5mm (min 76µm)

Offline parse of the gcode: 111/310 displacing moves < 0.5mm, min 0.0758mm, max 151.985mm; 565.7mm path; motion-time lower bound 5.16s. With arc-fit off each tiny facet stays a clothoid → the open tail (one braking distance) spans many segments → larger per-solve cost regardless of cadence.

## Deduced Conclusions

### Deduction 1: the cap=1 root-cause measurement was unrepresentative

`repro_plan_stall --cap 1` (the basis of the retracted "8.0s compute / 6–10× too slow" claim) commits **per move**, bypassing the production 64-move coalescing. Production cadence lies between cap=1 (gate trickles 1 move → coalescing defeated → per-move) and cap=64 (gate bursts → full coalescing → bounded per-batch). Which end the bench sits at is **unmeasured** — the clean cap=64 re-measure is blocked because the shared harness file is mid-edit on the arc-fitter track.

### Deduction 2: the short-circuit no-op is still expected

The short-circuit only skips zero-commit calls; with eviction + coalescing already present, its identical-timing result is unsurprising regardless of the residual mechanism.

## Hypothesized Paths

### H1: compute-bound starvation — DOWNGRADED (Confirmed → Open/conditional)

Live ONLY if the gate's throttled feed delivers ≈1 move at a time, defeating the 64-move coalescing → per-move cadence → the cap=1 cost regime. If the gate bursts, coalescing engages and per-batch cost is bounded by the open tail. **Binding question: gate-feed burstiness vs coalescing.** Confirm/refute with the bench trace's per-commit `buffered_before`.

### H2: open tail irreducibly large from sub-0.5mm facets

Even with full coalescing + eviction, one braking distance of 76µm facets is many segments ⇒ each bounded solve is still costly. Lever = arc-fitter (user's track), which collapses tiny facets → fewer segments per solve. Refute if trace shows small per-batch segment counts.

### H3: gate/pacing or clock-projection residual

Prior-art H15 (gate paces on wrong frontier) / clock double-writer — fixes landed (`ff4affea4`, `3b163c6ac`, clock-rebase). "Identical timing" favors a deterministic cause over jitter, but not ruled out. Refute/confirm with `feed_throttle` + `transit_diag_alert` in the trace.

## Source Code Trace

| Element | Detail |
| ------- | ------ |
| Error origin | `runtime/src/motion_core.rs:114` → `fault_helpers.rs:109`; `error.rs:187` `-308` |
| Trigger | dispatched piece start >200µs behind MCU now |
| Condition | committed frontier fails to keep pump lead (`pump.rs:335` 2.0s) positive |
| Already-shipped mitigations | eviction (`velocity.rs` barrier + `stream.rs` setback); coalescing (`stream_planner.rs:757`, `COALESCE_BATCH_MOVES=64`) |

## Conclusion

**Confidence: Medium. The earlier High "planner 6–10× too slow" root-cause claim is RETRACTED.** Fault mechanism stays Confirmed. The cause is NOT a missing batch-cadence or plan-reuse fix — both are done, and plan-reuse-as-caching is explicitly forbidden by the done spec. The cap=1 measurement bypassed production coalescing and overstated cost. The true residual is one of: coalescing defeated by throttled gate feed (H1), an irreducibly large open tail from sub-0.5mm facets (H2 → arc-fitter), or gate/clock (H3). A fresh bench trace is the genuine discriminator; offline analysis is exhausted against the unrepresentative harness.

## Recommended Next Steps

### Diagnostic (the real next step)

Fresh VictoriaLogs trace of the crash, via the `query-logs` / `mcu-diagnostics` skills. Capture per-commit: coalesced `buffered_before` (batch size actually achieved), `commit_fire_count` cadence, plan wall time, `feed_throttle` state, lead remaining, `transit_diag_alert`. Expected discriminators:
- batch size ≈1 while moves available ⇒ coalescing defeated (H1) → fix the cadence-vs-gate interaction.
- batch size large but per-batch segment count huge / plan time high ⇒ H2 → arc-fitter.
- frontier advancing on time but lead mis-accounted / projection off ⇒ H3.

### Fix direction (deferred until the trace picks the mechanism)

Do NOT re-spec batch cadence or plan-reuse (shipped / forbidden). Per mechanism: H1 → revisit the coalesce trigger under throttled feed; H2 → arc-fitter (separate track); H3 → reopen `piece-start-in-past-clock-rebase` / gate accounting.

## Side Findings

- The party-mode roundtable (Winston/Quinn/Amelia/Murat) independently re-derived the exact shipped design — finality-barrier eviction + static braking-distance window + batched cadence. Validates the existing implementation's design; does not produce new work.

## Follow-up: 2026-06-24 #3 — bench trace pulled; root cause CONFIRMED (per-batch plan latency)

### New Evidence (VictoriaLogs, neptune, print-1782307428 @ 13:23:48Z, session k-1782305529)

- **Fault = PieceStartInPast (-308), not a discontinuity panic.** `fault_code 65228`, `detail 199083`; full-text search for "discontinuity"/"panic"/"position" in the window returns only SD-print start/exit. Confirms the user's "is it the discontinuity?" → **no, it's the timing fault.**
- **Coalescing WORKS:** 314 `submit_move_enter` → **5 `commit_decision` / 5 `pipe_plan`** = ~64-move batches (`COALESCE_BATCH_MOVES`). H1-as-"coalescing-defeated" is **refuted**. `stall_skip` fired twice (the short-circuit is live).
- **The per-batch velocity plan is 640–750ms on the Pi:** `pipe_plan` `plan_ms`: batch18 (13 moves)=111; **batch19 (lines 10-73, 64 moves)=746; batch20 (63-126)=695; batch21 (114-177)=641.** Plans run back-to-back (~8ms between), planner thread ~100% busy.
- **Dispatch happens only AFTER each plan (same thread).** `commit_decision` batch21: `commit_count=160, barrier=166, t_committed=2.230`; `dispatch_committed` batch21: `t_start=2.230 → t_end=3.638` = **1.41s of motion dispatched per batch**.
- **Fatal kill is tiny and on the EtherCAT MCU:** `transit_diag_alert` `arrival_lead_us=-7568` (7.5ms late), `send_gap_us=7829` (**7.8ms pump send-gap**), `mcu=1`, `piece_count=1`, `room=958` → `ethercat drive fault` → `EXIT_ON_FAULT`. On the **1kHz EtherCAT loop** a 7.8ms gap = ~7 missed cycles = fatal.
- `seg0_deficit` `deficit_us=249987` ≈ **exactly `DEFAULT_LEAD_SECS` (0.25s)** — the planner's maintained lead, and the cold-start deficit.

### CONFIRMED Root Cause (High) — it's per-batch LATENCY, not aggregate throughput

A coalesced 64-move batch expands to ~150–167 segments (sub-0.5mm facets, arc-fit off) and takes **640–750ms to velocity-plan on the Pi**. Dispatch to the pump occurs **only after** each plan completes on the same planner thread, so **no pieces flow for ~700ms per batch**. The maintained lead is only ~0.25s and the print cold-starts with an empty buffer, so during a ~700ms planning window the **1kHz EtherCAT pump starves** (7.8ms send-gap → arrival_lead −7.5ms → -308).

**Crucial nuance:** throughput is FINE — batch21 spent 641ms wall to produce **1.41s** of motion (~2.2× realtime). The planner keeps up on *average*. The failure is **worst-case per-batch planning latency (≈700ms) vs the buffered lead (≈250ms)** — a latency/jitter problem against a hard 1ms real-time deadline, not "the planner is 6–10× too slow." My earlier cap=1 "8s total compute / 6–10× too slow" framing measured the wrong quantity (per-move total, not per-batch latency); the retraction's *cause* was wrong but its *caution* (cap=1 unrepresentative) was right. Net: the original "planner too slow" instinct was correct, re-cast as latency.

### Fix directions (evidence-ranked)

1. **Bound batch by segment-count / predicted plan-time, not move-count.** `COALESCE_BATCH_MOVES=64` is too coarse for tiny facets (64 moves → ~150 segs → 700ms). Cap so each plan stays well under the lead; finer dispatch granularity keeps the EtherCAT pump fed. Most direct planner-side fix.
2. **Arc-fitter (user's track).** Collapsing sub-0.5mm facets cuts segments/batch → shorter plans. Attacks the same driver from geometry.
3. **Overlap plan with dispatch / deepen absorb-buffer.** Dispatch is serialized after the plan; let committed pieces flow during planning, or let the buffer build toward the pump's 2.0s `MAX_LEAD_SECS` so a 700ms plan is absorbed (tension with the eviction spec's lead discipline — Ask First).
4. **Parallelize the SOCP across Pi cores** (CLAUDE.md-endorsed) — constant-factor, complements 1–2.

### Offline reproduction recipe (the user's goal)

Drive the gcode through the **coalescing cadence** (batches of `COALESCE_BATCH_MOVES=64`, NOT `--cap 1`), measure **per-batch `plan_us`**, and assert max per-batch plan latency > maintained lead. `repro_plan_stall --cap 64` models this — currently blocked only because the shared harness file is mid-edit on the arc-fit track. Target signature to reproduce: a 64-move batch of this file planning in ≳640ms on the Pi (≈110–130ms on the Mac, scaled by the ~5–6× factor). Pass/fail oracle: worst-batch `plan_us` vs `DEFAULT_LEAD_SECS`.

### Updated Conclusion

**Confidence: High.** Root cause CONFIRMED from the bench: per-batch velocity-plan latency (~700ms for 64 tiny-facet moves) exceeds the ~250ms lead, starving the 1kHz EtherCAT pump → -308. Coalescing and eviction both work; the binding cost is per-batch SOCP latency on dense sub-0.5mm geometry. Not a discontinuity, not a clock bug, not a missing cadence/reuse fix.
