---
title: 'Front-edge blend-budget invariance (take-3 OverCommit fix)'
type: 'bugfix'
created: '2026-06-22'
status: 'done'
baseline_commit: '67e10342b'
context:
  - '{project-root}/docs/rewrite/windowed-fit-ceiling-jitter.md'
  - '{project-root}/_bmad-output/implementation-artifacts/investigations/print-completes-early-investigation.md'
---

<frozen-after-approval reason="human-owned intent — do not modify unless human renegotiates">

## Intent

**Problem:** The streaming chain fitter re-solves a corner's biclothoid at a different (measured ~2×) curvature depending on where the look-ahead window starts. After a partial commit, `trim_front_to_seam` physically shortens the head move in the buffer; `classify_junction` then budgets the head move's outgoing corner as `0.5·min(len_in, len_out)` (fitter.rs:332), so the shorter `len_in` yields a smaller budget and a sharper apex (`kappa_peak = trim_ref·theta / trim`, smaller trim → higher curvature). The sharper apex's corner cap can fall below the velocity already committed at the preceding seam → `velocity plan: OverCommitted` → `stream_planner_fatal` → klippy aborts mid-print.

**Approach:** Make the leading corner's blend budget invariant to front-trimming: thread the head length consumed by the just-committed blend back into the budget for the head move's outgoing junction, so the corner re-solves to the same curvature it had when that seam was committed. The committed entry velocity then remains feasible across the re-plan. Once stable, remove the interim `WARM_START_REFIT_SLACK_REL` guard slack in velocity.rs.

## Boundaries & Constraints

**Always:**
- A given physical corner's fitted curvature/cap must be identical whether it is fit mid-window or as the post-commit leading corner (within numeric tol).
- The fix is additive/backward-compatible: `fit_chain` with no committed-head context behaves exactly as today (fresh stream, offline replay, existing tests unchanged).
- Honor the project "fail loud" rule: a genuine over-commit (lookahead truly too short) must still error; only the spurious window-front inconsistency is removed.

**Ask First:**
- Any change that alters geometry already dispatched to the MCU (committed segments), or that relaxes/removes the `OverCommitted` guard itself rather than fixing the fit.
- Any change to the biclothoid solver math (biclothoid.rs) beyond how `budget` is supplied.

**Never:**
- Do not "fix" this by widening `WARM_START_REFIT_SLACK_REL` to absorb large (≥few %) caps — that masks real over-commits. (The slack is being removed, not grown.)
- Do not add host-side flow control here — separate concern, out of scope.
- No new public config knobs exposed to printer.cfg.

## I/O & Edge-Case Matrix

| Scenario | Input / State | Expected Output / Behavior | Error Handling |
|----------|--------------|---------------------------|----------------|
| Fresh stream, no commit yet | window with full-length head move | unchanged fit vs today | N/A |
| Post-commit leading corner | head move trimmed by committed blend of length `h` | leading corner budget computed as if head un-trimmed → same `kappa_peak`/cap as pre-commit window | N/A |
| One-move-per-commit replay (cold_run) | bench limits, commit(false) each move | every commit returns Ok; no `OverCommitted` | N/A |
| Genuine over-commit (lookahead truly too short) | committed exit velocity unbrakeable within remaining tail | still `Err(OverCommitted)` | fail loud, unchanged |

</frozen-after-approval>

## Code Map

- `rust/geometry/src/fitter.rs` -- `fit_chain` (L184), head/tail reserve (L200-205), `classify_junction` budget (L332); the corner whose budget shrinks after front-trim.
- `rust/geometry/src/fitter/biclothoid.rs` -- `solve`: `trim = min(ideal, budget)`, `kappa_peak = trim_ref·theta/trim`; READ-ONLY (confirms budget→curvature), do not change math.
- `rust/geometry/src/fitter.rs` ChainFitConfig (L41) / CornerFitConfig -- where a committed-head-length context would attach.
- `rust/motion-engine/src/stream.rs` -- `commit` (L208+): computes `commit_count`, calls `fit_chain` (~L221), `trim_front_to_seam` (~L346) trims the head; must record the consumed head length and thread it into the next fit.
- `rust/geometry/src/velocity.rs` -- `WARM_START_REFIT_SLACK_REL` + `warm_start_refit_slack` + the two clamp sites: REVERT once the fit fix lands.
- `rust/motion-engine/src/stream/tests.rs` -- `cold_run_infill_streams_without_overcommit` (#[ignore]d): the acceptance oracle; un-ignore.

## Tasks & Acceptance

**Execution:**
- [x] `rust/geometry/src/fitter.rs` -- thread a per-fit "leading-head consumed length" into the budget for the head move's outgoing junction so its budget matches the pre-trim window (e.g. budget uses `len_in + committed_head` for the head junction only); keep the no-context path byte-identical to today.
- [x] `rust/motion-engine/src/stream.rs` -- after a partial commit that trims the head, record the consumed head length and supply it to the next `fit_chain` call; reset it when the buffer drains to rest / on force-commit.
- [x] `rust/geometry/src/velocity.rs` -- revert `WARM_START_REFIT_SLACK_REL`, `warm_start_refit_slack`, the `let mut entry_v`, and both clamp sites back to the original strict `+ VELOCITY_EPS_MM_S` guards.
- [x] `rust/motion-engine/src/stream/tests.rs` -- remove `#[ignore]` from `cold_run_infill_streams_without_overcommit`; add an invariance unit test in `rust/geometry/src/fitter/tests.rs` asserting a corner's `kappa_peak`/cap is equal (tight tol) whether fit mid-window or as the post-commit leading corner.
- [x] `docs/rewrite/windowed-fit-ceiling-jitter.md` -- mark Status resolved; note the implemented mechanism.

**Acceptance Criteria:**
- Given cold_run's infill replayed one move per commit under bench limits (100/1000, jerk 1e6), when each `commit(false)` runs, then none errors and the final flush succeeds (the previously-`#[ignore]`d test passes).
- Given the same corner fit in a long window and again as the leading corner after the preceding move is committed+trimmed, when both fits run, then its `kappa_peak` matches within `1e-6` (new invariance test).
- Given a fresh stream / offline `dump_stream_trajectory` replay, when fit runs with no committed-head context, then output is byte-identical to pre-change (no regression in existing fitter/velocity/stream tests).
- Given the slack reverted, when `cargo nextest run -p geometry -p motion-engine` runs, then green, and `cargo clippy --workspace --all-targets -- -D warnings` + `cargo fmt --all --check` pass.

## Design Notes

The biclothoid trim is symmetric: a blend consumes `trim` from each adjacent move. The full-window fit budgets the 53→54 and 54→55 corners each from `0.5·len(move54)`, so together they fit within move54. When move53 commits, `trim_front_to_seam` shortens move54 by the 53→54 trim, and the 54→55 budget recomputes from the shortened length — sharper. The committed entry velocity at the seam was dispatched against the *original* (gentler) apex, so the re-fit must reproduce that apex. Restoring the consumed head length into the head junction's budget reproduces it exactly. Measured target: cold_run move 55 `kappa` stays 0.248 (cap 60), not 0.492 (cap 45), as the window front advances 53→54.

## Verification

**Commands:**
- `cd rust && cargo nextest run -p motion-engine -E 'test(cold_run_infill_streams_without_overcommit)'` -- expected: PASS (no longer ignored/erroring)
- `cd rust && cargo nextest run -p geometry -p motion-engine` -- expected: all green
- `cd rust && cargo run --release -q -p motion-engine --example dump_stream_trajectory -- /tmp/cold_run.gcode /tmp/cr.csv --cap 1` (temporarily bench limits) -- expected: no `commit failed`
- `cd rust && cargo clippy --workspace --all-targets -- -D warnings && cargo fmt --all --check` -- expected: clean

## Suggested Review Order

**The fix — budget invariance in the fitter**

- Entry point: streaming fit variant that restores the trimmed head length; `fit_chain` delegates with 0.0 (byte-identical old path).
  [`fitter.rs:196`](../../rust/geometry/src/fitter.rs#L196)

- The actual budget change — restore added only to the leading move's outgoing junction.
  [`fitter.rs:355`](../../rust/geometry/src/fitter.rs#L355)

- Restore applied to junction i==0 only.
  [`fitter.rs:210`](../../rust/geometry/src/fitter.rs#L210)

**Carry-forward state in the streaming planner**

- New field: head length consumed at the last seam, fed into the next fit.
  [`stream.rs:105`](../../rust/motion-engine/src/stream.rs#L105)

- The fit call now threads the carried head length.
  [`stream.rs:230`](../../rust/motion-engine/src/stream.rs#L230)

- Set after a partial commit (trim amount, else 0.0); 0.0 on full-drain/reset.
  [`stream.rs:355`](../../rust/motion-engine/src/stream.rs#L355)

- `trim_front_to_seam` now returns the consumed head length.
  [`stream.rs:437`](../../rust/motion-engine/src/stream.rs#L437)

**Tests**

- Regression: cold_run infill streamed one-move-per-commit must not error (was #[ignore]d; fails without the fix).
  [`stream/tests.rs:102`](../../rust/motion-engine/src/stream/tests.rs#L102)

- Invariance: a corner's apex curvature is equal full-window vs head-trimmed+restored, and sharper without restore.
  [`fitter/tests.rs:366`](../../rust/geometry/src/fitter/tests.rs#L366)
