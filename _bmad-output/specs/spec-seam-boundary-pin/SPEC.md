---
id: SPEC-seam-boundary-pin
companions:
  - ../../implementation-artifacts/investigations/junction-position-discontinuity-investigation.md
  - ../../implementation-artifacts/spec-seam-boundary-pin.md
sources: []
---

> **Canonical contract.** This SPEC and the files in `companions:` are the complete, preservation-validated contract for what to build, test, and validate. Source documents listed in frontmatter are for traceability only.

# Streaming seam-continuity fix (B-now): never commit at a blend-entry clothoid

## Why

Streaming commits in the motion planner can break C0 position continuity at a commit seam, tripping the fail-loud `check_junction_position_continuity` panic and aborting the print — Y axis, |Δ|≈0.1545 mm on the Voron cube at commit cap ≤ 24, clean at cap ≥ 32.

Root cause: `is_clean_seam` (`stream.rs`) admitted a blend's **entry** clothoid as a commit point. `emit_blend` stamps that entry half with the *incoming* move's source line (`fitter.rs:412`), and `is_clean_seam` also accepted any seam whose source line was in the `unblended` set — so when an unrelated collinear junction tagged that line, the entry clothoid was wrongly accepted. Committing there advanced the odometer to the blend entry while the buffer kept the move untrimmed (head-trim fires only at blend *exits*), so the next re-fit re-solved the blend from a different start and opened the seam.

## Capabilities

- id: CAP-1
  intent: The streaming commit never cuts inside a blend — it cuts only where the fit output resumes a straight Line body (always zero curvature), matching the function's documented intent.
  success: `is_clean_seam` accepts a seam only when the resuming move is a `Segment::Line`; the Voron cube reaches `worst = 0.0` at every commit cap 1..256, not just ≥ 32, including the forced-commit-then-replan cases.

- id: CAP-2
  intent: The fix is the minimal predicate change — no change to the corner solver, the fit signatures, or the velocity warm-start.
  success: Only `is_clean_seam` and the dead `unblended` set it consulted change; `biclothoid::solve` / `classify_junction` / `fit_chain_with_head_restore` are byte-for-byte untouched.

## Constraints

- **Honest fix.** Continuity comes from refusing the invalid commit point (falling back to the nearest straight-line seam), never from translating output geometry to mask a gap.
- **No throughput regression.** Commit cadence and batch size on representative gcode are unchanged — the fix only declines an invalid cut; straight-line seams persist around every blend, so a valid seam is always within reach.
- **Fail loud preserved.** The `check_junction_position_continuity` panic stays; this fix removes its trigger, it does not silence it.
- **Real path only.** Must drive the real `StreamState` commit path; validated by the offline seam-continuity test platform (deterministic cap tests + windowed schedule fuzzer), not a mock.

## Non-goals

- Pinning a full `SeamBoundary` (position + heading + curvature) exit→entry state, or any boundary-condition / asymmetric-G2 clothoid solver — that is **B-full**, the substrate for mid-run commits and multi-segment (3-line → S) fits. Deferred.
- Mid-curve / mid-blend commits.
- Re-admitting the arc rest-seam commit point that Line-only drops — unreachable until native G2/G3 arc-input streaming lands; recorded in `deferred-work.md` with the fix (gate the dropped clause on a true rest seam).
- Re-litigating the MCU tick-projection seam (TickChain) — that was the tick manifestation; this is the mm-position bug.

## Success signal

The seam-continuity red board goes green by reproduction, not suppression: `seam_continuity_cap_8/16/24` and the windowed fuzzer turn green, `worst = 0.0` at every cap, the cube prints without the junction panic, and commit cadence on representative gcode is unchanged.

## Note on the original hypothesis

This spec first hypothesized a `SeamBoundary` double-derivation (odometer-vs-buffer position reconciled by the `head_len_restore` scalar) requiring a fit-signature refactor. The offline investigation **overturned** that: the bug is the `is_clean_seam` over-admission above, fixable in ~3 lines with no solver/struct change. The `SeamBoundary` contract remains a sound idea for B-full, but it was not the cause and was not built — see the investigation's 2026-06-24 follow-up.
