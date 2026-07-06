# Code map — Pressure Advance on the new path

Load-bearing implementation reference for SPEC-pressure-advance-port. File:line anchors verified against the live tree on 2026-06-22; re-confirm before editing.

## Live path facts

- Active solver is `temporal::multi` (Consolini-Locatelli SOCP). `temporal::topp` is the **dead** solver — still compiled, not on the live path.
- Live path consumes followers only for emission and cross-batch continuity (`follower_history`, `exchange_follower_tails`), never to cap XYZ path velocity.
- Input shaping already rides the post-processor rail: the shaper kernel is applied at emit time, after planning, with no feedback into limits (`trajectory/src/emit_shaped.rs:227` spatial, `:336` follower). PA rides the same rail.

## Work-item hooks

| CAP | What | Where |
|-----|------|-------|
| CAP-1 | Linear PA gain kernel (reuse as-is) | `rust/trajectory/src/post_processor.rs:201` `apply_derivative_gain` |
| CAP-1 | Confirm gain applied on follower emit | `rust/trajectory/src/emit_shaped.rs` follower branch (~`:303`/`:336`) |
| CAP-1 | PA type + action dispatch | `rust/trajectory/src/post_processor.rs` `PostProcessorType::LinearPressureAdvance{k}`, `PostProcessorInstance::action()` |
| CAP-2 | Config parse / instance build | `rust/motion-engine/src/config.rs` (`build_instance`, `"linear_pressure_advance"`) |
| CAP-2 | Config section (Python) | `klippy/extras/post_processor.py`; `[post_processor NAME]` + `[axis e] post_processors:` in `klippy/motion.py:629-648` |
| CAP-2 | Runtime tune command | `klippy/motion.py:737-755` `cmd_SET_POST_PROCESSOR` → `motion-engine` `update_post_processor` |
| CAP-3 | Extrude-only = virtual path | `rust/geometry/src/frontend.rs:123-131` (`try_new_virtual`, ratio ±1) |
| CAP-3 | Mainline reference for fields/logic | `git show main:klippy/kinematics/extruder.py` lines 222-235 (fields), 306-308 (`limit_speed`) |
| CAP-3 | Where extruder limits live now | `rust/motion-engine/src/config.rs` (limit assembly), `rust/temporal/src/limits.rs` |
| CAP-4 | Model seam | dispatch on `PostProcessorType` at emit; add arm + `type` string + correction fn for new models. Application order = `[axis] post_processors:` declaration order, type-agnostic |
| Constraint | Delete dead coupling | `rust/temporal/src/topp/follower.rs` (`follower_sets`, `emit_base_follower_rows`, `pa_demand_linearized`) and callers under `rust/temporal/src/topp/` |

## Gain-kernel correctness (why CAP-1 ports as-is)

`BezierPiece.coeffs` are **power-basis (monomial) coefficients in `(u − u_start)`**, with `u` in real time units — not Bernstein coefficients (`to_bernstein()` at `nurbs/src/bezier.rs:50` does the conversion). Therefore:

- `differentiate()` (`nurbs/src/bezier.rs:24`) returns `[1·c₁, 2·c₂, 3·c₃, …]` — the monomial coeffs of `dp/dt`, one shorter, **aligned at index 0**.
- `apply_derivative_gain` (`post_processor.rs:211`) computes `coeffs[i] + k·deriv[i]` with the top index padded by 0 = exactly `p(t) + k·p'(t)`. No per-piece duration scaling is needed because the basis is already in real-time `(u − u_start)`.

This is why the gain is correct and time-consistent across pieces of differing duration.

## Test anchors

- `rust/trajectory/src/post_processor/tests.rs:74` `derivative_gain_applied_exactly_on_nurbs` — exact numeric value check (PASS).
- `rust/trajectory/src/post_processor/tests.rs:82` `derivative_gain_preserves_degree_and_pieces` (PASS).
- `rust/nurbs/tests/differentiate.rs` `differentiate_cubic_matches_finite_diff` (PASS).
- `motion-engine planner::tests::update_post_processor_applies_to_new_plans_only` (PASS) — live swap path.
- `motion-engine::streaming_replan::update_post_processor_commits_held_output_before_swap` (PASS).
- `rust/motion-engine/tests/follower_lane_e2e.rs:51` declares a `linear_pressure_advance` case — extend here for CAP-1 end-to-end assertion.

## Why decoupling is safe and smooth_time is deferred

Mainline ships linear PA *with* `smooth_time` because its base trajectory is trapezoidal (instantaneous accel steps) — bare PA there injects extruder velocity discontinuities. Our base trajectory is jerk-limited and C²-continuous, so `ė` and `a` are already continuous and the `k·a` correction inherits that smoothness; the smoothing is upstream in the planner, not in the PA pass. tanh would only be needed later to *saturate the magnitude* of the correction at high accel — a different concern than smoothness.
