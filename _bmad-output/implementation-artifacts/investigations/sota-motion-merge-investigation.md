# Investigation: Merging origin/sota-motion into curvature-profile-1

## Hand-off Brief

1. **What happened.** Merging `origin/sota-motion` into `curvature-profile-1` (PR #100) produces conflicts in the motion-engine hot path; sota-motion added input-shapers-on-live-stream + pressure-advance + axis-chains while curvature-profile-1 added the single-MCU wire protocol + streaming/backpressure refactor — both rewrote `stream.rs`/`stream_planner.rs` and their tests. (Confirmed)
2. **Where the case stands.** Active — merge in progress; `pump.rs` resolved (ours supersedes), `bridge.rs`/`deferred-work.md` auto-resolved by rerere (unverified), 4 files (28 markers) open.
3. **What's needed next.** Per-file 3-way analysis to classify each hunk orthogonal/overlapping/superseding and pick a per-hunk resolution, then a sequenced merge plan.

## Case Info

| Field            | Value |
| ---------------- | ----- |
| Ticket           | PR #100 (base sota-motion) |
| Date opened      | 2026-06-23 |
| Status           | Active |
| System           | git merge (no-commit) in worktree curvature-profile-1; merge-base `c28737259` |
| Evidence sources | git conflict markers, 3-way diffs (`:1:`/`:2:`/`:3:`), sota-motion commit log |

## Problem Statement

"Investigate the merge conflicts and how we should merge them." The branch must take the latest `origin/sota-motion`. The conflicts are heavy and concentrated in the motion planner.

## Evidence Inventory

| Source   | Status | Notes |
| -------- | ------ | ----- |
| git merge state | Available | MERGING; `git diff --diff-filter=U` lists the open files |
| Conflict markers | Available | pump.rs(1, resolved), stream.rs(8), stream/tests.rs(11), stream_planner.rs(3), stream_planner/tests.rs(6) |
| sota-motion commit log | Available | input shapers `164077c20`, pressure advance `05394a7ce`, comment-strip `dd3993af6`, ethercat buzz `0fc4a5fd6` |
| 3-way churn | Available | see Confirmed Finding 1 |
| rerere auto-resolutions | Partial | bridge.rs / deferred-work.md resolved from a prior recorded resolution — not yet verified correct |

## Confirmed Findings

### Finding 1: The conflict is feature-divergence, not textual drift

**Evidence:** `git diff --numstat c28737259 {HEAD,origin/sota-motion}` per file:
- `stream.rs` — OURS +368/-26, THEIRS +330/-40 (both heavy → real overlap)
- `stream/tests.rs` — OURS +581/-19, THEIRS +151/-18 (ours dominates)
- `stream_planner.rs` — OURS +396/-97, THEIRS +15/-5 (theirs tiny)
- `stream_planner/tests.rs` — OURS +282/-7, THEIRS +65/-0 (theirs additive)

**Detail:** sota-motion's motion-engine commits: `164077c20` port input shapers to live stream, `05394a7ce` pressure advance on live pipeline, `dd3993af6` strip comment blocks. Ours: single-MCU PushPieces wire change + streaming/backpressure (commit_stall_brake, head_trim_feasible, handle_control, bounded channels).

### Finding 2: pump.rs resolved by superseding

**Evidence:** `pump.rs:1046` conflict — theirs referenced `r.front_start_time`, a field removed by our frame-level `PushPiecesResponse`. Resolved to ours (the `emit_transit_diag` loop). Theirs could not compile against the merged protocol.

## Deduced Conclusions

### Deduction 1: The two feature sets are largely ORTHOGONAL — the merge is tractable, not a rewrite-collision

**Based on:** Findings 1–2 + four 3-way per-file analyses.

**Reasoning:** Ours owns the *commit/seam/backpressure pipeline* (arc-setback seam selection, head-trim, `commit_stall_brake`, bounded channels, single-MCU wire). Theirs adds a *post-processing pass* (input-shaper + pressure-advance via `apply_axis_chains`, new `axis_chains`/`post_history` state, `lower_move` gains a chains arg) plus axis-chains plumbing. These are different layers of the pipeline; in `stream.rs` they textually interleave at only **two** points (the lowering-loop `lower_move` call, and `reset()`), everything else is additive-both-sides (new enum variants, new struct fields, new free functions appended). Test conflicts are **substance-orthogonal** — ours adds backpressure/stall-brake/proptest tests, theirs adds shaper tests; the only collision is constructor-signature drift (`StreamState::new`/`spawn` gained an `axis_chains` arg).

**Conclusion:** Resolvable by hand with the compiler + `motion-engine` suite as a strong net. Residual risk concentrates in ~3 spots, all precisely characterized below.

## Confirmed Findings (cont.)

### Finding 3: rerere auto-resolutions are CORRECT

**Evidence:** 3-way verification of `bridge.rs` and `deferred-work.md`. `bridge.rs` RESULT carries all of ours (pump_backlog, dispatch_anchor, move_seq, STREAM_MAX_BUFFER_MOVES, queued_motion_secs/dispatched_lead_secs) AND all of theirs (`#![allow(deprecated)]`, resonance_buzz, max_extrude_only_* into init_planner/VelocityConfig, axis_chains compile + `spawn(stream_cfg, axis_chains, …)`, update_post_processor rewrite). No symbol references a removed/renamed def. Confidence High. `deferred-work.md` correct but missing one blank line before `## Pump-backlog` (cosmetic).

### Finding 4: Three high-risk integration spots (silent correctness, no compile error)

**Evidence:** stream.rs analyzer.
1. `stream.rs` lowering loop (~345-359): must use `seam_xyz.push(...)` (ours — feeds `head_trim_feasible`) **and** the 6-arg `lower_move(..., &self.axis_chains.chains)` (theirs — applies PA/shaper) **and** `seg.source_line` (ours — wire G-code mapping). Picking one side silently drops a feature.
2. `stream.rs` free-function block (~482-706): keep ours' `commit_stall_brake`/`head_trim_feasible`/`trim_front_to_seam` AND theirs' `post_commit_count`/`trim_post_history` + the whole `apply_axis_chains…` pipeline.
3. `keep_secs` liveness: ours' arc-setback seam logic **replaced** theirs' `keep_secs`-gated seam selection, so the production analyzer says `keep_secs` is dead → drop it, keep `max_buffer_moves`. The test analyzer (tests-only view) assumed it survives. **The compiler settles this** — if nothing references `keep_secs` after resolution, drop it from `StreamConfig` and from the test `cfg()` helpers.

## Merge Plan (the recommended resolution)

Order: small→core→tests, building incrementally so the compiler pins the `keep_secs` question before the test sweep.

1. **pump.rs** — DONE (took ours; theirs used the removed `r.front_start_time`).
2. **bridge.rs**, **deferred-work.md** — `git add` (rerere correct); optionally add the one blank line.
3. **stream_planner.rs** (3 hunks): `#![allow(deprecated)]` as line 1, then `use …VecDeque`; imports = keep ours' `{…, TrySendError, bounded}` + `use trajectory::{AxisChainSet, ShapedSegment}` (drop `unbounded`); keep ours' `other => handle_control(…)` catch-all and **add** a `StreamMsg::SetAxisChains(chains) => state.set_axis_chains(chains)` arm **inside `handle_control`** (else non-exhaustive panic). Discard theirs' inline arms (they drop `tally.reset()`).
4. **stream.rs** (8 hunks): integrate all — StreamError = Geometry+BrakeToRestShortfall+PostProcess (and Display arms); `reset()` clears all 4 new fields; `new()` inits all 4 new fields; keep `max_buffer_moves()`/drop `keep_secs`; the two HIGH-RISK hunks per Finding 4.
5. **stream/tests.rs (11)** + **stream_planner/tests.rs (6)**: keep ALL tests from both sides; reconcile every `StreamState::new`/`spawn` call to the merged signature (`AxisChainSet::default()` arg). Update the **non-conflicted** call sites too (`nonstop_flood…`, `voron_cube…`, `cold_run_infill…`) — the signature change breaks them silently. Drop `keep_secs` from `cfg()` helpers if step 4 drops it.
6. **Verify ladder:** `cargo check -p motion-engine` (settles keep_secs) → `cargo nextest run -p motion-engine` → full `cargo nextest run` → `./scripts/ci.sh quick` + `./scripts/ci.sh py` → **bench print** (only true test of the hot path).

## Final Conclusion

**Confidence: Medium-High.** The merge is a hand-integration, not a rewrite war: ours (commit/seam/backpressure + wire) and theirs (shaper/PA post-pass + axis-chains) are orthogonal layers colliding at ~3 well-identified points. rerere already resolved `bridge.rs` correctly. The compiler + the `motion-engine` suite catch the mechanical 90%; the 3 silent-correctness spots are enumerated. A bench print is the final gate.

**Status:** Active — plan ready, awaiting go to execute.
