---
title: 'Motion-10: end-to-end integration test for the new geometry trajectory pipeline'
type: 'feature'
created: '2026-06-18'
status: 'done'
baseline_commit: '9b5382c051c502af56dda09091a669c511064c71'
context: ['{project-root}/CLAUDE.md']
---

<frozen-after-approval reason="human-owned intent — do not modify unless human renegotiates">

## Intent

**Problem:** Steps 4–9 built a complete `Move`-based trajectory chain (`fit_chain → plan_velocity → lower_profile`), but it is **dormant** — nothing runs the stages together. Each stage has its own unit tests, but the assembled pipeline has never been exercised end-to-end on a representative multi-move path, so cross-stage contract drift (move-count/source alignment, velocity continuity across blends, endpoint fidelity) is unverified.

**Approach:** Add a `geometry`-crate integration test that constructs representative `Move` sequences through the public `line_move`/`arc_move` API — the same shape the Python→PyO3 bridge feeds — then drives them through `fit_chain → plan_velocity → lower_profile` and asserts whole-trajectory invariants. Test-only; no production code changes, no gcode parsing (the lexer lives in Python), no FFI.

## Boundaries & Constraints

**Always:** Build moves only through the existing public API (`line_move`, `arc_move`, `MoveContext`, `VelocityLimits`). Drive the real public stage functions (`fit_chain`, `plan_velocity`, `lower_profile`) with default configs unless a scenario needs otherwise. Assert via a shared invariant helper so every scenario gets the same checks. Use representative printer limits (e.g. max_v 300 mm/s, accel 5000 mm/s², scv 5 mm/s).

**Ask First:** Adding or changing any non-test (`src/`) file. Modifying a stage's public signature or default config to make a test pass (that signals a real bug — surface it, don't paper over it).

**Never:** Parse gcode text or add a Rust gcode reducer. Touch the legacy NURBS path (`GeometryPipeline`/`CubicSegment`) or the FFI/`motion-engine` integration. Duplicate per-stage unit-test coverage that already exists — this test is about the *seams between* stages.

## I/O & Edge-Case Matrix

| Scenario | Input (moves) | Expected end-to-end behavior |
|----------|--------------|------------------------------|
| Cornered polyline | 4 `line_move` legs forming a square (90° corners) | `report.blended + report.chains > 0` (corners → clothoids); full chain succeeds; samples non-empty; all invariants hold |
| Arc path | `arc_move` quarter/half circle (+ flanking lines) | curvature-bounded velocity (`velocity.report.curvature_bound > 0`); invariants hold |
| Extruding path | mixed line+arc with monotone E deltas | follower samples non-decreasing; invariants hold |
| Long straight run | single long `line_move`, feed below max_v | profile cruises at feed cap; invariants hold |
| Single move | one `line_move` | `fit_chain` no-op passthrough; lowers to a valid trajectory |
| Empty | `[]` | `fit_chain`→`plan_velocity`→`lower_profile` all yield empty, no error |

</frozen-after-approval>

## Code Map

- `rust/geometry/src/frontend.rs` -- `line_move`/`arc_move`/`MoveContext`/`VelocityLimits` — move constructors the test calls (the bridge-equivalent input shape).
- `rust/geometry/src/fitter.rs` -- `fit_chain(&[Move], ChainFitConfig)` → `FitOutcome { moves, report }`.
- `rust/geometry/src/velocity.rs` -- `plan_velocity(&FitOutcome, VelocityConfig)` → `VelocityProfile { moves: Vec<MoveVelocity{entry_v,exit_v,peak_v,samples,…}>, report }`.
- `rust/geometry/src/execution.rs` -- `lower_profile(&FitOutcome, &VelocityProfile, rate_hz)` → `Vec<LoweredSample{t_s, position: Option<[f64;3]>, followers: Vec<f64>}>`.
- `rust/geometry/src/path/mod.rs` -- `PositionProfile::point_at` / `Segment::s_len` — for endpoint-fidelity assertions.
- `rust/geometry/tests/integration_pipeline.rs` -- **NEW** the integration test + shared invariant helper.

## Tasks & Acceptance

**Execution:**
- [x] `rust/geometry/tests/integration_pipeline.rs` -- build per-scenario `Vec<Move>` via `line_move`/`arc_move`; run `fit_chain → plan_velocity → lower_profile` at a representative `rate_hz`; one `#[test]` per I/O-matrix row.
- [x] `rust/geometry/tests/integration_pipeline.rs` -- shared `assert_trajectory_invariants(geometry, profile, samples, limits)` helper applying the cross-stage invariants below.

**Acceptance Criteria:**
- Given any successful plan, when inspecting `samples`, then `t_s` is finite, starts at 0.0, and is monotonically non-decreasing; every `position`/`followers` value is finite; total time > 0.
- Given a spatial trajectory, when measuring per-step speed `‖Δposition‖ / Δt` between consecutive samples, then it never exceeds `limits.max_velocity_mm_s` beyond a small relative tolerance.
- Given a spatial trajectory, when comparing the final sample position to the last move's terminal point (`point_at(s_len)`), then they match within tolerance (the path is fully traversed).
- Given a blended chain, when reading `profile.moves`, then velocity is continuous across junctions (`exit_v[k] ≈ entry_v[k+1]`).
- Given the cornered-polyline scenario, when fitting, then at least one clothoid blend is produced and the lowered trajectory slows through the corner (per-step speed dips below the straight-run cruise speed near the junction).
- Given monotone extrusion input, when inspecting follower samples, then they are non-decreasing.
- Given empty input, when running the full chain, then every stage returns an empty result without error.

## Design Notes

Move-count/source alignment is a real cross-stage contract: `plan_velocity` consumes the post-`fit_chain` `FitOutcome` and `lower_profile` re-checks `geometry.moves.len() == profile.moves.len()` and per-move `source` equality — a successful end-to-end run is itself the proof these line up, which no per-stage unit test covers.

The speed-cap assertion must tolerate clothoid corner slow-downs (speed only ever dips *below* the cap) and fixed-rate sampling discretization at `rate_hz`; use a relative tolerance, not exact equality. Pick `rate_hz` representative of the trajectory sample clock (e.g. ~10–100 kHz) but modest enough to keep sample counts test-friendly.

## Verification

**Commands:**
- `cargo nextest run -p geometry -E 'binary(integration_pipeline)'` -- expected: all new scenarios green.
- `cargo nextest run -p geometry` -- expected: full geometry suite still green (no regressions).
- `./scripts/ci.sh quick` -- expected: fully green (clippy `-D warnings`, fmt, rust tests) before PR.

## Suggested Review Order

**Design intent — how the stages are wired**

- Entry point: the three-stage chain assembled exactly as the PyO3 bridge would drive it.
  [`integration_pipeline.rs:54`](../../rust/geometry/tests/integration_pipeline.rs#L54)

- Moves built only through the public `line_move`/`arc_move` API (bridge-equivalent input shape).
  [`integration_pipeline.rs:36`](../../rust/geometry/tests/integration_pipeline.rs#L36)

**Cross-stage invariants (the load-bearing assertions)**

- Fixed-rate uniform Δt + per-step speed cap — verifies step-9 lowering and rules out truncated trajectories.
  [`integration_pipeline.rs:108`](../../rust/geometry/tests/integration_pipeline.rs#L108)

- Velocity continuity across move junctions (shared planner node).
  [`integration_pipeline.rs:133`](../../rust/geometry/tests/integration_pipeline.rs#L133)

- Endpoint fidelity — last sample reaches the commanded terminal vertex.
  [`integration_pipeline.rs:145`](../../rust/geometry/tests/integration_pipeline.rs#L145)

**Scenarios (most architecturally interesting first)**

- Irregular polyline → per-corner clothoid blends; corners cruise slower than straights.
  [`integration_pipeline.rs:159`](../../rust/geometry/tests/integration_pipeline.rs#L159)

- Co-circular square → step-8 chain reconstruction (arc), with excursion guard.
  [`integration_pipeline.rs:203`](../../rust/geometry/tests/integration_pipeline.rs#L203)

- Tight arc → curvature-bounded velocity.
  [`integration_pipeline.rs:239`](../../rust/geometry/tests/integration_pipeline.rs#L239)

- Extruding move → per-segment-local follower ramp 0→e_delta.
  [`integration_pipeline.rs:270`](../../rust/geometry/tests/integration_pipeline.rs#L270)

**Boundary scenarios (supporting)**

- Long straight cruises at the feed cap; single-move passthrough; empty input → empty trajectory.
  [`integration_pipeline.rs:295`](../../rust/geometry/tests/integration_pipeline.rs#L295)
