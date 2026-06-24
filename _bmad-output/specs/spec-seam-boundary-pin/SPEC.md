---
id: SPEC-seam-boundary-pin
companions:
  - ../../implementation-artifacts/investigations/junction-position-discontinuity-investigation.md
  - ../spec-seam-continuity-test-platform/SPEC.md
sources: []
---

> **Canonical contract.** This SPEC and the files in `companions:` are the complete, preservation-validated contract for what to build, test, and validate. Source documents listed in frontmatter are for traceability only.

# Streaming seam-continuity fix — pin the committed seam boundary (B-now)

## Why

The streaming commit derives the seam position **twice, independently**, and reconciles them with a scalar band-aid. The committed batch's exit position comes from `advance_odometer` walking the *reconstructed* `outcome.moves` (`stream.rs:337`); the continuation's entry comes from `trim_front_to_seam` slicing the *original* `self.buffer` (`stream.rs:445`); the only thing tying them together is `head_len_restore: f64` (`fitter.rs:355`), which merely nudges the blend **budget**. When the restored length does not reproduce the boundary corner's `trim` exactly, the re-fit's start `a_start = vertex − trim·t_in` (`biclothoid.rs:42`) lands somewhere else — a C0 position break (Y, |Δ|≈0.1545 mm on the Voron cube at commit cap ≤ 24) that trips the fail-loud `panic!` in `check_junction_position_continuity` and aborts the print.

B-now replaces the scalar restore with a real exit→entry **`SeamBoundary`** and makes the boundary-corner re-fit **reproduce** the committed corner — so position continuity is a *consequence the harness asserts*, never a translation correction (which would close C0 while opening a heading/curvature lie). Scope is commits at **clean, zero-curvature seams** — the points `is_clean_seam` already selects. Mid-run / nonzero-curvature commits and the boundary-condition-aware (asymmetric/G2) clothoid solver they require are out of scope here; they are B-full, which is also the multi-segment-fit primitive.

## Capabilities

- id: CAP-1
  intent: The committed batch emits a single `SeamBoundary` (its full kinematic exit state — position, heading, tangent, and the existing `entry_v`/consumed-head length) and the next `fit_chain` consumes it as its entry, replacing the `head_len_restore: f64` scalar so the seam position has one source of truth instead of two independent derivations.
  success: `fit_chain_with_head_restore`'s scalar restore parameter is gone, replaced by a `SeamBoundary`; the committed exit and the continuation entry reference the same value; no call site re-derives the seam position from a separate buffer slice.

- id: CAP-2
  intent: At a clean (κ=0) seam, the boundary-corner re-fit reproduces the pre-commit corner's `trim`/`a_start` exactly, so the continuation's first lowered piece `coeffs[0]` equals the committed batch's last piece `coeffs[3]` bit-for-bit.
  success: Driving `crash_short_cube.gcode` through the seam-continuity harness reports `worst = 0.0` at **every** commit cap 1..256 (not only ≥32), including the forced-commit-then-replan cases that land on clean seams.

- id: CAP-3
  intent: Position continuity is achieved by matched geometry, asserted — never by translating the continuation to close the gap.
  success: The fix contains no `+= delta` / re-anchor-to-pin step; the seam descriptor matches because the re-fit produced the same corner, verified by C1 velocity and curvature/blend-budget invariance at the seam (not only the C0 magnitude) staying within the harness tolerance.

- id: CAP-4
  intent: A commit requested at a non-clean (κ≠0) seam — outside B-now's scope — fails loud rather than silently translating to fake continuity.
  success: If `is_clean_seam` ever admits a still-curving seam, the commit path returns a clear error (B-full territory) instead of emitting a translated, geometrically-dishonest continuation; this is exercised by a test.

## Constraints

- **Honest C0.** Continuity must come from the re-fit reproducing the committed corner, not from output translation. No re-anchoring the continuation by the gap delta. A fix that drives `fatal` to 0 while leaving `worst > 0` (or by shoving pieces into alignment) is a regression, not a fix.
- **No throughput regression.** B-now must not change commit-point selection, cadence, or batch size — it only makes the *already-chosen* clean-seam commit reproduce on re-fit. Commit timing on representative slicer output must be unchanged (the planner never trades trajectory time for an easier seam).
- **No solver reformulation.** B-now must require **zero** change to `biclothoid::solve`'s signature or its symmetric structure. The moment the fix needs the solver to accept a G2 entry boundary (nonzero entry curvature/`sigma`), it has crossed into B-full and must stop.
- **Real path only.** Must drive the real `StreamState` / `fit_chain_with_head_restore` / biclothoid commit path and be validated through the seam-continuity test platform (sibling spec), not a mock or parallel planner.
- **Clean-seam precondition, verified first.** B-now assumes the failing commit lands at a κ=0 seam. The first task is the diagnostic that confirms this for the cube; if `is_clean_seam` is admitting a still-curving point, B-now is insufficient and the work escalates to B-full rather than being forced through.

## Non-goals

- The boundary-condition-aware (asymmetric / full-G2) clothoid solver — that is B-full.
- Mid-run / mid-curve seam commits — B-full.
- Multi-segment fitting (e.g. 3 lines → one S-clothoid) — future, built on B-full's entry interface.
- Changing `is_clean_seam` / finality-barrier commit-point policy — B-now keeps the existing selection and only fixes reproduction at the chosen seam.
- Re-litigating the MCU tick-projection seam (TickChain) — that was the tick manifestation; B-now is the mm-position bug.

## Success signal

The seam-continuity harness red board goes green by reproduction, not suppression: `seam_continuity_cap_8/16/24` and the schedule fuzzer turn green, `worst = 0.0` at every cap, and the cube prints without the junction panic — with commit cadence on representative gcode measurably unchanged. The down payment is small (no solver change); the same `SeamBoundary` exit→entry contract it introduces is exactly the interface B-full and multi-segment fitting will extend.

## Assumptions

- The cube's failing commit lands at a clean κ=0 seam (confirmed by the first diagnostic). If false, B-now alone cannot close the cube and B-full is pulled forward.
- `is_clean_seam` is honest about κ=0; B-now's scope guard (CAP-4) fails loud if it is not.
- The seam-continuity test platform is the validation oracle: a B-now pass there implies the bench will not panic at a clean-seam commit.
