# `geometry`

Geometry and velocity-planning primitives for the kalico motion planner.
`motion-engine/src/stream/fitter.rs` is the main consumer — read it first to
see these primitives assembled into the live streaming pipeline.

## Public surface

Build `Move`s from G-code-shaped waypoints, fit them into a G2-continuous
chain (lines blended by biclothoid corners, or reconstructed into arcs from
faceted runs), plan a velocity profile over the fitted
chain, then lower it to fixed-rate trajectory samples:

```rust
use geometry::{CornerFitConfig, MoveContext, VelocityLimits, fit_corners, line_move, lower_profile};
use geometry::velocity::{BoundaryState, plan_velocity_warm_start};

let limits = VelocityLimits::try_new(max_velocity, max_accel, square_corner_velocity, max_jerk)?;
let ctx = MoveContext { extruder_axis, feedrate_mm_s, limits, source };
let moves = vec![line_move(start, end, e_delta, ctx)?];

let fitted = fit_corners(&moves, CornerFitConfig::default())?;
let profile = plan_velocity_warm_start(&fitted, integration_tol, max_v_cap, max_a_cap, BoundaryState::REST)?;
let samples = lower_profile(&fitted, &profile, sample_rate_hz)?;
```

The streaming fitter (`motion-engine`) drives the same primitives
incrementally — `plan_junction_reduced`, `arc_candidate_fits`, `RunFit`, and
`blend_moves` in `geometry::fitter` — rather than calling `fit_corners` over
a complete move buffer; see `stream/fitter.rs` for how a bounded lookahead
window decides run extents before committing.
