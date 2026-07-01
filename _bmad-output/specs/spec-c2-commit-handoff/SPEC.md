---
id: SPEC-c2-commit-handoff
companions:
  - brownfield.md
sources: []
---

> **Canonical contract.** This SPEC and the files in `companions:` are the complete, preservation-validated contract for what to build, test, and validate. Source documents listed in frontmatter are for traceability only.

# Curvature-Continuous Commit Handoff

## Why

A pain to solve, blocking downstream quality work. The streaming motion planner re-plans a sliding look-ahead window on every commit and hands the boundary to the next window as a single **scalar entry velocity**. With only a scalar, the planner may legally cut the trajectory **only where curvature is zero** (`is_clean_seam`, stream.rs:704-707) — a straight-line point. A fitted arc with its clothoid eases (a clothoid-arc-clothoid run) therefore cannot be committed until a κ=0 seam appears past it, so the run is re-fit window after window. Any ease fit beyond the simplest gentle form drifts the fitted curvature and seam position under that repeated re-fit, producing C0 discontinuities at small commit caps — the `pump.rs` junction-continuity guard then aborts the process (the original Neptune EtherCAT servo-X crash). It also blocks the arc-ease quality fix (single monotonic clothoid eases, no bare arc→line seams, no overshoot biclothoids), which is correct on the whole-buffer fit but drifts under streaming. The fix is one clear contract: **across a commit, the resume curvature must equal the committed curvature**, so a run can commit mid-arc and the next window continues the same circle or a κ-matched clothoid, never re-fitting across the boundary.

## Capabilities

- id: CAP-1
  intent: The planner can place a non-forced commit seam inside an arc or clothoid (nonzero κ), not only at a zero-curvature line point.
  success: `is_clean_seam` accepts a nonzero-κ cut; a streaming run with arc-fit on commits forward progress even when its only finality-eligible seams fall inside a blend, instead of stalling until a κ=0 seam.

- id: CAP-2
  intent: Window N+1 reads the carried G2 endpoint — position, tangent, signed curvature — and fits its continuation forward from it, choosing arc or clothoid as the fit dictates, so the resumed segment starts at exactly the committed curvature.
  success: At every commit seam the start curvature of the first resumed segment equals the end curvature of the last committed segment within the junction-continuity tolerance; a window-invariance test asserts a committed boundary's κ equals the whole-buffer fit's κ at that same point.

- id: CAP-3
  intent: The velocity warm-start plans from the carried boundary curvature instead of assuming κ=0 at the seam.
  success: `plan_velocity_warm_start` accepts an entry curvature alongside entry velocity and produces a valid profile for a mid-arc resume with no `OverCommitted` abort.

## Constraints

- The curvature-match contract is enforced loud: a resume κ diverging from the committed κ beyond the junction-continuity tolerance raises the existing fatal guard (`pump.rs` junction continuity), never silently padded.
- The finality-barrier guarantee is preserved: every committed body stays a function of geometry alone — final under append and output-equivalent to a full re-plan.
- No throughput loss: the change must remove the "commit only at κ=0" restriction without dropping arcs or shortening trajectories to satisfy a seam.
- Host-side only: changes live in the Rust motion-engine streaming path; the C/Rust MCU boundary is untouched.
- The carried boundary state is exactly the G2 endpoint — position, tangent, and signed curvature (carried as a curvature vector, so the bending plane travels with it) — and nothing more; the osculating circle and any resume geometry are derivable from it. It travels in `StreamState` alongside `entry_v`/`committed_head_len`, not a side channel.
- Comments are a failure of expression; unit tests live in a separate file; run the suite with `cargo nextest`.

## Non-goals

- Not changing the arc/corner fitting geometry. The per-end ease + shrink-to-fit quality fix is a separate effort that this spec unblocks.
- Not introducing a new cornering velocity model. The warm-start already curvature-limits per segment; this only seeds the entry curvature.
- Not committing across an extrusion-rate (epmm) discontinuity as if curvature-continuous.
- Not carrying derivatives above curvature (no jerk-continuity contract across the seam).

## Success signal

With arc-fit on, the Voron cube perimeter and the original crash file stream to completion at any commit cadence, curvature-continuous at every commit seam: `arc_fit_voron_cube_perimeter_is_c0_at_every_commit_cadence` (motion-engine seam_test_harness) is green at caps 1, 2, 4, 8, 16, 64, 100000, and a new assertion confirms a committed boundary's curvature equals the whole-buffer fit's curvature there. No `junction_position_discontinuity` or `OverCommitted` aborts.

## Assumptions

- Position, tangent, and signed curvature fully determine the resume: the osculating circle is derivable, and the same state serves an arc or clothoid resume uniformly — no arc identity or clothoid σ need be carried. The contract is G2 (curvature) continuity, not G3 (curvature-rate).
- The warm-start uses the carried entry curvature as a fixed boundary condition — as it already does the entry velocity — not a re-derived value, so the seam stays drift-free.
- The tolerance already used by the `pump.rs` junction-position-continuity guard is the right tolerance for the curvature-match contract.
