# `geometry`

Geometry and velocity-planning primitives for the kalico motion planner.
Production consumers live in `motion-pipeline/src/fit_stage.rs`,
`planner.rs`, and `lower_stage.rs`.

## Public surface

`line_move` builds `Move`s from G-code-shaped waypoints. The streaming fitter
uses `geometry::fitter` to blend corners and reconstruct faceted runs.
`plan_velocity_stops_select_prefix` plans the fitted chain and reconstructs
only the selected prefix for lowering.

Continuation carries a scalar entry speed (`0.0` at rest). After committing
`count` moves, the next window starts at `profile.boundary_speeds[count]`.
Velocity profiles retain their phase laws and emitted acceleration; no
separate boundary-acceleration state is needed.

Velocity planning supports unlimited jerk (`f64::INFINITY`), not finite jerk
limits. Acceleration, curvature, feedrate, and stop constraints still apply.
