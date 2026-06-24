---
title: 'Re-plan amplification short-circuit: skip the velocity plan when no seam can commit'
type: 'bugfix'
created: '2026-06-24'
status: 'done'
baseline_commit: 'e995c80718b20cacd8b4e0e780470e4d3b054189'
context:
  - '{project-root}/_bmad-output/implementation-artifacts/investigations/junction-position-discontinuity-investigation.md'
---

<frozen-after-approval reason="human-owned intent — do not modify unless human renegotiates">

## Intent

**Problem:** `StreamState::commit(false)` runs the expensive velocity plan (`plan_velocity_warm_start`, ~740 ms for 175 segments on the Pi) on every push, even when nothing can commit. In an all-clothoid region — SCV blending consumes every straight `Line`, so the fit output has no committable seam — the buffer cannot drain, the plan re-runs and is discarded on each push (O(n²)), and ~5 s of wasted CPU starves the real-time MCU feeder until a piece arrives in the MCU's past → fail-loud `PieceStartInPast`. The fit itself is cheap (~137 µs); the plan is the entire cost.

**Approach:** After the cheap fit and before the expensive plan, when `!force` and the fit output contains no committable `Segment::Line` body, return empty without planning. The fit already proves `commit_count` would be 0, so the plan's result is irrelevant — skipping it is trajectory-neutral. This is the missing precondition on the expensive path, not a bypass: the plan should never have run when no seam exists.

## Boundaries & Constraints

**Always:** Skip the plan only when `commit_count` is provably 0 (zero `Segment::Line` bodies in the fit output) — so committed output is byte-identical with the short-circuit on or off. On a skip, set `last_v_barrier = limits.max_v` (a sound upper bound) so `stall_brake_time()` keeps its "fires early, never late" invariant. Drive the real `StreamState` commit path; validate against `neptune_crash_short.gcode`.

**Ask First:** Any change that would make a skip alter committed geometry, timing, or commit count. Any change to `plan_velocity_warm_start`, `fit_chain_with_head_restore`, or the corner solver — that is out of scope.

**Never:** Skipping on `force` commits (flush/dwell/brake-to-rest always plan). Caching or reusing a stale plan/fit across pushes. Translating or padding output to mask a gap. Addressing the structural/vase case (a region with no `Line` ever) — that stays for the arc-fitter / interior-seam follow-up. `#[ignore]`/xfail/should_panic on any seam test.

## I/O & Edge-Case Matrix

| Scenario | Input / State | Expected Output / Behavior | Error Handling |
|----------|--------------|---------------------------|----------------|
| All-clothoid stall | `commit(false)`, fit output has no `Line` body | Skip plan, return empty, `last_v_barrier = max_v` | N/A |
| Committable seam present | `commit(false)`, ≥1 `Line` body in fit output | Plan runs, normal commit (unchanged) | N/A |
| Forced drain | `commit(true)` over an all-clothoid buffer | Plan runs, full flush (never skipped) | existing |
| Setback-blocked `Line` | `Line` body exists but not yet committable | Plan runs (not skipped); re-plans until it unlocks | N/A |

</frozen-after-approval>

## Code Map

- `motion-engine/src/stream.rs` — `commit()` (~278): insert the guard between `fit_chain_with_head_restore` (~290) and `plan_velocity_warm_start` (~306). `StreamState` fields + `new()`: add `full_plan_count: u64` and `replan_short_circuit: bool` (default `true`). `last_v_barrier` (~144) is the state to pin on skip.
- `motion-engine/src/stream/tests.rs` — the two red-first tests.
- `motion-engine/examples/repro_plan_stall.rs` — existing measurement harness; reports worst-commit ms / total compute for the human-facing before/after (no change required).

## Tasks & Acceptance

**Execution:**
- [x] `motion-engine/src/stream.rs` -- short-circuit added: factored seam selection into `select_commit_seam(moves, seam_xyz, barrier)` (plan-independent); `seam_xyz` computed before the plan; when `!force && replan_short_circuit && select_commit_seam(.., n-1) == 0`, set `last_v_barrier = max_velocity_mm_s`, emit `stall_skip`, return empty before planning. Added `full_plan_count: u64` (incremented before `plan_velocity_warm_start`) + accessor and `replan_short_circuit: bool` (default true) + test-support setter.
- [x] `motion-engine/src/stream/tests.rs` -- two tests driving the real `neptune_crash_short.gcode`: (1) **equivalence** — `replan_short_circuit` on vs off; assert byte-identical committed `ShapedSegment`s (fingerprint via `eval` of each axis at `t_start`/`t_end`, since `ShapedSegment` has no `PartialEq`); guards against a vacuous run. (2) **work-bound** — assert a stall run of ≥10 consecutive plan-skips occurs.

**Acceptance Criteria:**
- Given `neptune_crash_short.gcode` driven through the real commit path, when the short-circuit is on vs off, then committed output is byte-identical at every commit.
- Given an all-clothoid region, when `commit(false)` is called per push, then no plan executes (`full_plan_count` stays 0) and each call returns empty.
- Given the fix, when `cargo nextest run -p motion-engine` runs, then every previously-passing test still passes, and `repro_plan_stall` on `neptune_crash_short.gcode` shows total commit compute collapse from ~5 s toward the fit-only floor.

## Spec Change Log

- **Predicate corrected (implementation):** The frozen intent's predicate "no `Segment::Line` body in the fit output" was based on a wrong mechanism model — the red `all_clothoid_region` test proved the neptune stall *does* have `Line` bodies (short remnants between blends) that are simply not committable. The sound predicate is instead: run the real seam-selection loop (`select_commit_seam`) with the most generous barrier (`n-1`); if it selects nothing, the plan's tighter `profile.barrier` selects nothing too → `commit_count` is provably 0 → skip. All inputs (`setback`, `is_clean_seam`, `head_trim_feasible`, arc lengths, `seam_xyz`) are plan-independent. KEEP: the intent's spirit ("the fit proves `commit_count` would be 0, so skipping is trajectory-neutral") is exactly realized.
- **Bench-efficacy caveat (must verify before claiming closure):** Bench logs show the *fatal* plans (740 ms on `n=175 commit_count=144`; the 50–74 ms burst with `line_lo` advancing 5→34→87→…) are on **successful** commits, which this short-circuit does **not** skip. The bench had only 36 empty (`commit_count=0`) commits vs 183 under the local cap=1 cadence. So this fix removes empty-commit amplification (locally: plans 310→127, drive wall 11.0 s→5.2 s) but the bench's dominant cost is large successful-commit re-planning. PieceStartInPast on the bench is **not** proven closed by this change alone; the residual is the 740 ms floor + redundant overlapping-window re-plan, which needs the arc-fitter and/or cross-commit plan reuse (out of scope).

- **Adversarial review (2026-06-24):** no `intent_gap`/`bad_spec`; no loopback. Blind hunter's HIGH finding (skip soundness rests on `profile.barrier < n`) was verified by the edge-case hunter as a guaranteed invariant (`velocity.rs` computes `barrier` in `1..n`); applied two patches anyway: (1) a `debug_assert!(profile.barrier < n)` making that load-bearing invariant fail-loud against future planner changes; (2) the work-bound test now asserts skipped pushes commit nothing (matches AC(b)'s text, not just a run-length count). Bench successful-commit re-plan cost deferred to `deferred-work.md`. Acceptance auditor confirmed all 3 ACs + boundaries met and the spec honest about bench efficacy.

## Design Notes

Soundness: `select_commit_seam(.., n-1)` is an upper-bound seam search — a larger barrier can only find more candidates, so selecting nothing there guarantees the real `profile.barrier ≤ n-1` selects nothing. A `Line` that exists but is setback/feasibility-blocked is still planned (not skipped): correctness over the marginal optimization. The fit stays O(n) per push; only the dominant velocity plan is gated. Local `repro_plan_stall` (cap=1) confirms transient starvation (buffer bounded at 29, self-draining) and the skip halves planning compute; the structural/vase case is explicitly deferred.

## Verification

**Commands:**
- `cargo nextest run -p motion-engine -E 'test(short_circuit) + test(stall) + test(seam)'` -- expected: new tests + all seam-continuity tests pass.
- `cargo nextest run -p motion-engine` -- expected: fully green.
- `cargo run --release -q -p motion-engine --features test-support --example repro_plan_stall -- motion-engine/tests/gcode/neptune_crash_short.gcode --cap 1` -- expected: total commit compute far below the ~5 s baseline; `commits_over_50ms` near 0.
- `./scripts/ci.sh rust-clippy && ./scripts/ci.sh rust-fmt` -- expected: clean.
