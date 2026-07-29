# Feature status

Honest per-feature status for the `sota-motion` fork.

## Status at a glance

Three tiers, honestly applied:

| Tier | Meaning |
| --- | --- |
| **solid** | used daily on real hardware |
| **verified in sim** | passes the simulator tests; not recently exercised on real hardware |
| **exploratory** | in the pipeline, design still open |

| Feature | Status |
| --- | --- |
| Core pipeline (fitter → planner → lowerer → shaper) | **solid** |
| Corner-deviation clothoid blending | **solid** |
| Follower axes (extruder as a plain axis) | **solid** |
| `smooth_bell` smoothing | **solid** |
| Nonlinear pressure advance | **solid** |
| EtherCAT servo path (test bench) | **solid** |
| Sim + snapshot test infrastructure | **solid** |
| Structured logging / crash forensics | **solid** |
| Step/dir motor path | **solid** |
| Phase stepping | **verified in sim (2026-07)**; not recently exercised on real hardware |
| Explicit `max_jerk` as a first-class limit | **exploratory** — enforced, but whether it earns its keep next to smoothing kernels is open |
| Kinematics beyond cartesian / corexy | **exploratory** |
| Per-axis-group limit model | **exploratory** |

### Drive types

| Drive type | Status | Notes |
| --- | --- | --- |
| Step/dir | **solid** | classic path |
| Phase stepping | **verified in sim (2026-07)**; not recently exercised on real hardware | opt in with `phase_stepping: 1` in the stepper's existing section. Switch-endstop homing on a phase-stepped axis is not covered by the sim tests — only sensorless |
| EtherCAT servo | **solid on the test bench** | industrial servo on X, steppers elsewhere |

## Known limits

- **Jerk on corner blends: enforced, with one soft spot.** The jerk limit
  applies through clothoid blends, including the rotating acceleration's
  normal component. Where blend geometry makes the jerk budget infeasible
  outright, the planner follows the hard acceleration limit instead and
  jerk goes soft there (`rust/geometry/src/velocity/ride.rs`).
- **No per-axis XY limits.** Global limits plus Z-only caps, nothing
  finer.
- **Kinematics:** cartesian and corexy only.
- **Config is not mainline-compatible.** `[kinematics]`, `[motor]`,
  `[axis]`, `[post_processor]` replace the classic sections. This is
  intentional. Migration guide:
  [docs/Config_Migration.md](Config_Migration.md).
- **`mode_inverse` accel demand is unmodeled.** The planner does not fold
  its `k2·jerk` motor-accel demand into the accel limits, so at high jerk
  settings the motor command can exceed `max_accel`
  ([docs/rewrite/shaper.md](rewrite/shaper.md)).
