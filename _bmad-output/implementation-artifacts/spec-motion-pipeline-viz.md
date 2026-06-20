---
title: 'Motion pipeline visualization tool'
type: 'feature'
created: '2026-06-19'
status: 'done'
baseline_commit: '04c823c5c'
context: []
---

<frozen-after-approval reason="human-owned intent — do not modify unless human renegotiates">

## Intent

**Problem:** No way to visually inspect the motion trajectory at intermediate pipeline stages. When debugging fitting or TOPP, developers must read logs or trust the math — no spatial feedback loop.

**Approach:** A Python CLI script that takes G-code, runs it through pipeline stages via PyO3 bridge debug methods, and outputs PNG plots at three stages: raw input path (polyline), corner-fitted path (smooth curves), and TOPP velocity profile.

## Boundaries & Constraints

**Always:** Use matplotlib (already a prototype dep). Run fully offline — no printer or Klipper instance required. Output standalone PNGs. All Rust changes are additive debug extraction — no production pipeline modification.

**Ask First:** Whether TOPP stage should show velocity-vs-arc-length, or a velocity-colored XY path, or both.

**Never:** Modify production pipeline control flow. Add new dependencies. Require a running printer.

## I/O & Edge-Case Matrix

| Scenario | Input / State | Expected Output / Behavior | Error Handling |
|----------|--------------|---------------------------|----------------|
| Valid G-code | File with G1/G2/G3 moves | 3 PNGs in output dir | N/A |
| No spatial moves | Only G28/M-codes | Skip plots, print "no moves" | N/A |
| 2D-only | XY moves, Z=0 | XY plots render normally | N/A |

</frozen-after-approval>

## Code Map

- `scripts/viz_pipeline.py` -- new CLI tool: parse args, call bridge, plot 3 stages
- `rust/motion-engine/src/bridge.rs` -- add `debug_pipeline_snapshot()` PyO3 method
- `rust/geometry/src/pipeline.rs` -- geometry pipeline entry (raw moves)
- `rust/geometry/src/fitter.rs` -- `fit_corners()` produces fitted curves
- `rust/temporal/src/multi/mod.rs` -- `plan_batch()` produces TOPP profiles
- `scripts/fitter_prototype/analyze.py` -- reference matplotlib patterns

## Tasks & Acceptance

**Execution:**
- [x] `rust/motion-engine/src/viz.rs` -- add `pipeline_snapshot()` PyO3 function that takes waypoints + limits, runs geometry→fitting→velocity planning, returns dict of XY point arrays (raw + fitted) and velocity samples (s, v) as Python lists
- [x] `scripts/viz_pipeline.py` -- create CLI script: argparse (gcode path, output dir), parse G-code, call snapshot function, plot 3 stages with matplotlib Agg backend, save PNGs

**Acceptance Criteria:**
- Given a G-code file with mixed G1/G2 moves, when running `python scripts/viz_pipeline.py input.gcode -o ./viz-out/`, then 3 PNGs are produced
- Given the raw path plot, it shows connected line segments matching G-code geometry
- Given the fitted path plot, corners are visibly smoothed compared to the raw path
- Given the velocity plot, it shows the velocity envelope across arc-length

## Design Notes

Pipeline stage data extraction points:
- **Raw path:** `GeometryPipeline::process()` → `Move` list → evaluate `PathSegment` start/end points → XY arrays
- **Fitted path:** `fit_corners()` → `CubicSegment` list → evaluate `VectorNurbs<3>` on dense grid → XY arrays
- **TOPP profile:** `plan_batch()` → `TopProfile` list → extract `GridSample.(s, v)` pairs

The bridge method evaluates curves to point arrays on the Rust side (avoids exposing NURBS internals) and returns numpy-friendly flat lists. Minimal engine init: config with axis count and default limits, no MCU connection.

## Verification

**Commands:**
- `cargo nextest run -p motion-engine` -- expected: existing tests pass
- `python scripts/viz_pipeline.py test.gcode -o /tmp/viz/` -- expected: 3 PNGs created

## Suggested Review Order

**Pipeline snapshot (Rust entry point)**

- Standalone PyO3 function; takes waypoints + limits, returns dict of point arrays
  [`viz.rs:8`](../../rust/motion-engine/src/viz.rs#L8)

- Builds geometry::Move objects from consecutive waypoint pairs, filters zero-displacement
  [`viz.rs:61`](../../rust/motion-engine/src/viz.rs#L61)

- Samples fitted path at 2 pts/mm by evaluating PositionProfile::point_at along each segment
  [`viz.rs:115`](../../rust/motion-engine/src/viz.rs#L115)

**G-code parser (Python)**

- Handles G0/G00 (rapid), G1/G01 (linear), G2/G02/G3/G03 (arcs via linearization)
  [`viz_pipeline.py:50`](../../scripts/viz_pipeline.py#L50)

- Arc linearization: samples arc into 0.5mm chord segments for the fitting pipeline
  [`viz_pipeline.py:18`](../../scripts/viz_pipeline.py#L18)

**Module registration**

- Two-line addition: module declaration + pyfunction export
  [`lib.rs:44`](../../rust/motion-engine/src/lib.rs#L44)

**Tests**

- 7 unit tests: move construction, raw/fitted point counts, velocity samples, edge cases
  [`tests.rs:1`](../../rust/motion-engine/src/viz/tests.rs#L1)
