# Investigation: Arc-Line Boundary Stop Anchor

## Hand-off Brief

1. **What happened.** User reported an acceleration spike where `arc_fit__circle` transitions between an arc and a line.
2. **Where the case stands.** Concluded: `fit_chain` now reports uneased non-tangent arc-line run boundaries as stop anchors.
3. **What's needed next.** Optional broader CI (`./scripts/ci.sh quick`) before PR.

## Case Info

| Field            | Value                                                                 |
| ---------------- | --------------------------------------------------------------------- |
| Ticket           | N/A                                                                   |
| Date opened      | 2026-06-26                                                            |
| Status           | Concluded                                                             |
| System           | macOS workspace `/Users/daniladergachev/Developer/kalico/.worktrees/arc-line-boundary` |
| Evidence sources | Source code                                                           |

## Problem Statement

User reported that `arc_fit__circle` has a huge acceleration spike where an arc meets a line. Existing line-line safety forces rest for non-tangent/non-collinear seams; the same safety should apply to arcs.

## Evidence Inventory

| Source | Status | Notes |
| ------ | ------ | ----- |
| `rust/geometry/src/velocity.rs` | Available | Non-collinear unblended junctions already become stop anchors. |
| `rust/geometry/src/fitter.rs` | Available | Arc-fit run boundaries were skipped from unblended reporting. |

## Investigation Backlog

| # | Path to Explore | Priority | Status | Notes |
| - | --------------- | -------- | ------ | ----- |
| 1 | Add targeted arc-line boundary test | High | Done | Asserts both fit report and velocity zero anchor. |

## Timeline of Events

| Time | Event | Source | Confidence |
| ---- | ----- | ------ | ---------- |
| 2026-06-26 | User reported arc/line acceleration spike | User message | Confirmed |

## Confirmed Findings

### Finding 1: Velocity stop anchors already exist for non-collinear unblended seams

**Evidence:** `rust/geometry/src/velocity.rs:157`

**Detail:** `stop_lines` includes all unblended reasons except `Collinear`, and anchors those seams to zero velocity.

### Finding 2: Arc-fit run boundaries skipped unblended reporting

**Evidence:** `rust/geometry/src/fitter.rs:276`

**Detail:** The previous condition skipped both `junction_internal` and `run_boundary`, so plain line-to-reconstructed-arc seams never reached the velocity stop-anchor path.

## Deduced Conclusions

### Deduction 1: Missing unblended report causes nonzero arc-line seam velocity

**Based on:** Findings 1 and 2.

**Reasoning:** Velocity can only force rest for seams listed in `FitReport.unblended`; arc-fit boundaries were intentionally omitted, so uneased non-tangent arc-line seams stayed eligible for nonzero continuity.

**Conclusion:** The fix belongs in `fit_chain` boundary reporting, not in the velocity planner.

## Hypothesized Paths

### Hypothesis 1: Reporting uneased non-tangent arc boundaries as `ArcIncident` fixes the spike

**Status:** Confirmed

**Theory:** The existing stop-anchor path will pin such seams to zero once the fitter reports them.

**Supporting indicators:** Existing velocity tests already cover non-collinear unblended seams pinning zero.

**Would confirm:** A targeted chain-fit/velocity test for a faceted arc with sharp line leads.

**Would refute:** Test still showing nonzero `entry_v` or `exit_v` at the arc boundary after reporting.

**Resolution:** `uneased_arc_line_boundaries_pin_velocity_to_rest` confirms both `ArcIncident` reporting and zero seam velocities.

## Missing Evidence

| Gap | Impact | How to Obtain |
| --- | ------ | ------------- |
| Exact `arc_fit__circle` fixture | Could confirm the original visual spike directly | Locate benchmark/visual fixture or ask user if not in repo |

## Source Code Trace

| Element | Detail |
| ------- | ------ |
| Error origin | `rust/geometry/src/fitter.rs:276`, run-boundary report suppression |
| Trigger | Arc-fit replaces faceted line run with `Segment::Arc` and no boundary easement |
| Condition | Boundary line is non-tangent to reconstructed arc and no clothoid easement is emitted |
| Related files | `rust/geometry/src/velocity.rs`, `rust/geometry/src/fitter/chain/tests.rs` |

## Conclusion

**Confidence:** High

The root cause was arc-fit boundary suppression bypassing the existing velocity rest-anchor mechanism. Targeted tests confirm the fix pins uneased non-tangent arc-line boundaries to rest.

## Recommended Next Steps

### Fix direction

Implemented: report uneased non-tangent line/arc run boundaries as `ArcIncident`; leave eased boundaries and tangent line/arc handoffs unreported.

### Diagnostic

Completed: targeted chain fitter tests and the full `geometry` crate test set pass.

## Reproduction Plan

Create a faceted circular arc with sharp incoming and outgoing line leads, run `fit_chain` with arc fit enabled, then verify `plan_velocity` pins the arc boundary velocities to zero.

## Side Findings

- `arc_fit__circle` was not found as an exact symbol in the repo with text search.
