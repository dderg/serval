---
title: 'Seam-continuity fix (B-now): stop committing at blend-entry clothoids'
type: 'bugfix'
created: '2026-06-24'
status: 'done'
baseline_commit: '0a4d87d71'
context:
  - '{project-root}/_bmad-output/implementation-artifacts/investigations/junction-position-discontinuity-investigation.md'
---

<frozen-after-approval reason="human-owned intent — do not modify unless human renegotiates">

## Intent

**Problem:** A non-forced commit can cut at a blend's **entry** clothoid. `is_clean_seam` (`stream.rs:613`) admits any `moves[i]` whose source line is in the `unblended` set, but `emit_blend` stamps a blend's entry half with the *incoming* move's source line (`fitter.rs:412`); when that line is collinear-tagged by an unrelated junction, the entry clothoid is wrongly accepted. Committing there advances the odometer to the blend entry while the buffer keeps the move untrimmed (head-trim fires only at blend *exits*), so the next re-fit re-solves the blend from a different start — a C0 seam (Y, |Δ|≈0.1545 mm on the cube at commit cap ≤ 24) that trips the fail-loud junction panic.

**Approach:** Restrict `is_clean_seam` to `Segment::Line` only — the function's own documented intent ("resumes a straight line body … never inside a blend"). The dropped `unblended` clause is redundant for real collinear seams (already Lines) and was the sole over-admission. ~3 lines + dead-code removal; no solver, fit-signature, or new-struct change.

## Boundaries & Constraints

**Always:** Commit cuts only where the fit output resumes a straight Line body. Drive the real `StreamState` commit path. The cube board reaches `worst = 0.0` at every cap 1..256. Continuity comes from rejecting the bad cut, never from translating output.

**Ask First:** Any need to touch `biclothoid::solve` / `classify_junction` / `fit_chain_with_head_restore` signatures — that is B-full; HALT. Any change to commit cadence on representative gcode.

**Never:** A `SeamBoundary` struct, boundary-condition solver, or mid-curve commits (all B-full). Skip / `#[ignore]` / xfail / baseline-to-green any seam test.

## I/O & Edge-Case Matrix

| Scenario | Input / State | Expected |
|----------|--------------|----------|
| Unblended or blend-exit seam | `moves[i]` = Line | clean → committable |
| Blend-entry clothoid (the bug) | `moves[i]` = Clothoid half1, source line in `unblended` | **not clean** → fall back to preceding Line seam |
| Blend interior | `moves[i]` = Clothoid | not clean |

</frozen-after-approval>

## Code Map

- `motion-engine/src/stream.rs:613` — `is_clean_seam`: the predicate to tighten.
- `motion-engine/src/stream.rs:357-358,369,1` — dead `unblended: HashSet` build + call-site arg + the `HashSet` import (line 391's tracing reads the report directly and stays).
- `motion-engine/src/stream.rs:609-612` — stale narration comment, remove.
- `motion-engine/src/stream/tests.rs` — unit test site.
- `motion-engine/tests/seam_continuity.rs` — deterministic oracle: cap 8/16/24 flip red→green; 32/64/256 stay green. No edit.

## Tasks & Acceptance

**Execution:**
- [x] `motion-engine/src/stream.rs` — tighten `is_clean_seam` to accept only `Segment::Line`; delete the dead `unblended` HashSet build + param, drop the unused `HashSet` import, remove the stale comment.
- [x] `motion-engine/src/stream/tests.rs` — add `is_clean_seam_rejects_blend_entry_clothoid` and `…_accepts_line_seam`: a half1 clothoid carrying a source line present in `unblended` is not clean; a Line at the same index is.

**Acceptance Criteria:**
- Given current HEAD, when `cargo nextest run -p motion-engine -E 'test(seam_continuity)'` runs, then cap 8/16/24/32/64/256 all PASS with `worst = 0.0` at every cap.
- Given a fit output whose blend-entry clothoid carries a collinear-tagged source line, when `is_clean_seam` is queried there, then it returns false and the commit falls back to the preceding Line seam.
- Given the fix, when `cargo nextest run -p motion-engine` runs, then every previously-passing test still passes (only the intended cap 8/16/24 red→green flips change).

## Design Notes

Root-cause trace and the overturned `SeamBoundary` hypothesis live in the investigation (`context:`). One-line summary: cap=8 commit 103 selects a Clothoid half1 (line 177, collinear-tagged) → odometer at blend entry, buffer untrimmed → commit 104 re-fits and lands 0.1545 mm off. The windowed schedule-fuzzer is handled as separate test-platform work, not this spec.

## Verification

**Commands:**
- `cargo nextest run -p motion-engine -E 'test(seam_continuity)'` — all caps PASS, `worst = 0.0`.
- `cargo nextest run -p motion-engine` — fully green.
- `./scripts/ci.sh rust-clippy && ./scripts/ci.sh rust-fmt` — clean.

## Suggested Review Order

- The fix: the predicate now keys on segment type only, so a blend-entry clothoid can never be cut at.
  [`stream.rs:607`](../../rust/motion-engine/src/stream.rs#L607)

- The call site and the dead code it shed: `unblended` set + arg gone, commit-point selection otherwise unchanged.
  [`stream.rs:367`](../../rust/motion-engine/src/stream.rs#L367)

- Unit guard — a clothoid is rejected regardless of source line (the deterministic pin is the cap 8/16/24 integration tests).
  [`tests.rs:850`](../../rust/motion-engine/src/stream/tests.rs#L850)

- Unit guard — a straight line body is accepted.
  [`tests.rs:844`](../../rust/motion-engine/src/stream/tests.rs#L844)
