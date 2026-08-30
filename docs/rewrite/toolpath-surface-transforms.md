# Toolpath Surface Transforms (bed mesh)

> AI generated, reviewed in design discussion 2026-07-06. Architectural
> invariant — flag drift, don't drift silently. Design note; no code has
> landed yet.

## The problem

Mainline applies bed mesh as a g-code-level move transform
(`gcode_move.set_move_transform`) with a `MoveSplitter` that chops every XY
move into segments whenever the mesh Z drifts by `split_delta_z` (0.025 mm,
sampled every `move_check_distance` = 5 mm). Z follows the mesh as a
piecewise-linear staircase. That approach is incompatible with this engine:
pre-split moves feed the fitter jagged 3D polylines where the user commanded
straight lines, defeating arc reconstruction and G2 blending, and every
split segment eats lookahead. `bed_mesh.set_mesh` on this branch hard-raises
today for exactly this reason.

## The invariant: two coordinate spaces, one crossing

**Everything upstream of the lowerer is gcode-space cartesian. Everything
downstream is machine-space cartesian. The bridge inverse is the only place
the two spaces convert.**

- `submit_move` deltas, the ingress contiguity odometer, the fitter's
  lines/arcs, the planner's velocity plan — all gcode space. The planner
  never sees the mesh.
- The lowerer's output (`ShapedSegment` axis curves), the shaper, kinematic
  lane mixing in enqueue, everything on the wire — machine space.
- Measured positions (endstop/probe trigger positions) are **machine-space
  by construction**: a trigger is a physical event, and it does not matter
  whether the approach move that got there was warped.
- The bridge is the only layer that converts between the spaces, and it
  does so at rest. The crossing is enforced by types, not convention:
  `geometry::GcodePos` / `geometry::MachinePos` (`rust/geometry/src/space.rs`)
  wrap every cartesian position the bridge holds — `commanded_pos`, the
  stream odometer inputs, and everything returned to Python are `GcodePos`;
  trip/abort reconstructions from motion history, MCU step-counter seeds
  (`build_serial_seed_sends`), and history rebase targets
  (`reanchor_axis_targets`) are `MachinePos`. The only converters are
  `gcode_from_machine` / `machine_from_gcode` on the bridge (backed by
  `SurfaceTransform::gcode_z` and `correction_at`), so a forgotten crossing
  is a compile error. Full kinematic states (`motion_state_at_clock`, the
  live motor queries) cross via `SurfaceTransform::unwarp_z_state`, the
  exact inverse of the lowerer's chain rule. No code below the bridge
  converts. A path that feeds gcode-space Z where machine-space is expected
  (or vice versa) is a bug — assert and fail loudly, don't compensate.
- A mesh swap (`swap_bed_mesh`) blocks until the pipeline has drained and
  adopted the token, holding the bridge's mesh handle locked across the
  wait — no crossing can ever invert through a mesh the lowerer is not yet
  warping with.
- `set_position` renames the physical rest point in gcode space: the stream
  odometer takes the gcode value, while the step-counter seeds and the
  motion-history rebase take the forward-warped machine value. Skipping
  that warp is the contact-probe ratchet: every touch re-seeds the machine
  frame shifted by `correction_at(x, y)` and the sample spread grows until
  the step dispatcher faults (`space/tests.rs::contact_touch_cycle_is_a_fixed_point`
  encodes the loop).

The payoff of this framing: there is **no suspend/restore mode**. Commanded
motion is always warped, measurements are never contaminated by the warp,
so homing and probing need no bypass, no per-move "raw" flag, and no
stateful protocol that an error path can leave half-toggled. Mainline's
raw-coordinate bypass (`manual_move`, `drip_move` dodging the transform
layer) exists because its probe readback goes through gcode-space
bookkeeping; ours doesn't, so the entire failure class is structurally
absent.

## Where the warp lives

In the lowerer's `Sampler` (`rust/motion-pipeline/src/lowering.rs`) — the
one place that holds, per output sample, both the segment geometry and the
arc-length profile from which the full cartesian state `(x, y, z, v, a)`
is derivable. Note the existing `axis_base_state` is a *per-axis* function
(`axis, t → (pos, vel, accel)`); the warp needs `x, y, ẋ, ẏ` alongside
`z` at the same `t`, so it lives as a bundled-sample step above the
per-axis split (evaluating `seg.point_at(s)` / `heading_at(s)` once per
sample), not inside the per-axis Z branch:

```
z_machine = z_g + fade(z_g) * (mesh(x, y) - fade_target) + fade_target
```

with the chain-rule coupling terms for the derivatives:

```
ż += fade * (∂mesh/∂x * ẋ + ∂mesh/∂y * ẏ)    (+ fade' term inside the band)
```

and the corresponding second-derivative terms for accel. These matter for
servo accel feedforward (EtherCAT), and they are cheap once the surface has
analytic gradients — which drives the surface representation below.

The lowerer already fits arbitrary sampled signals to NURBS via adaptive
refinement (`refine_span`); a mesh-warped Z just yields a few more pieces.
Mesh-active moves bypass the closed-form straight fast path
(`lower_straight_from_phases`) and take the sampled path, the same gating
that already exists for ramped followers.

**Fast path preserved only for exact constants:** a move keeps the closed-form
path when the correction is exactly constant — a flat mesh, motion above
`fade_end`, or no spatial motion. Any nonzero variation takes the sampled
path, even when it is below the position-fit tolerance. Freezing each short
move at its own start correction makes adjacent moves disagree at their
shared endpoint; those sub-microstep position jumps accumulate in the step
lattice and eventually demand several roots at one clock.

## Surface representation

Bicubic cardinal spline over the probed points (mainline's build-time
interpolant, tension 0.2), **evaluated directly at runtime** — not
pre-densified into a grid with bilinear lookup. Rationale:

- Bilinear has a discontinuous gradient at every cell boundary; the
  chain-rule Z velocity would have C⁰ kinks → jerk spikes and fragmented
  NURBS fits. The cardinal spline is C¹ with analytic gradients, a few
  dozen flops per sample.
- Mainline only pre-densified because per-move Python lookup had to be
  dirt cheap. The lowerer doesn't have that constraint.
- Known limitation, accepted for v1: C¹ but not C² across knots, so Z
  accel feedforward has small jumps at cell boundaries. If that ever shows
  on servos, fit a genuinely C² surface (natural bicubic spline) at build
  time — same runtime shape, different solve.

Fade follows mainline semantics: full correction below `fade_start`, linear
ramp to zero at `fade_end`, correction fading toward `fade_target` (mesh
mean by default) rather than toward 0, constant `fade_target` applied above
the band. Fade is a function of *gcode* Z, which the sampler has (the
pre-warp value). Like the surface, the linear fade is C⁰ at the band edges
(fade′ jumps by 1/fade_dist), another small accel-feedforward
discontinuity accepted for v1 with the same smooth-ramp upgrade path if it
ever shows.

## State and lifecycle

The mesh is the only mutable state, and it changes at exactly two
commands: activate (load/SET) and clear. Both flow through the pipe as a
control token (`SetMesh(Arc<Mesh>)` / `SetMesh(None)`) behind a `Drain`, in
stream order, following the `SetAxisChains` precedent. On every mesh swap,
re-derive the gcode position from the machine position through the *new*
mesh at that rest point, so the gcode↔machine mapping never goes stale.

Probing, profile management, and g-code command surface stay in Python;
only the built surface (control net + params + fade config) crosses the
bridge.

## Safety: a gross-error gate plus a bounded envelope

The mesh coupling is *additive* to commanded Z motion, and the planner
already grants commanded ż_g the full Z limit on steep combined moves
(`CartesianLimits::for_move` caps at `max_z_velocity / z_unit`). A
statically *sound* activation check is therefore impossible without the
planner reserving Z headroom per move while a mesh is active — cross-stage
coupling that breaks the "planner never sees the mesh" property and costs
more in maintained complexity than it protects against. We deliberately
don't do that. Instead:

**Gross-error gate.** At mesh activation, bound the worst case analytically
from the spline control net: max |∇mesh| and max surface curvature. Compute

- worst-case coupled Z velocity = max slope × machine max XY velocity
- worst-case coupled Z accel ≈ max slope × max XY accel + max curvature × v²max

and compare them against the Z budget (the Z axis limits, or the separate
`z_velocity_limit`/`z_accel_limit` in `[bed_mesh]`). Exceeding the budget
does not block activation: the mesh loads anyway with a prominent console
warning and a `bed_mesh_z_budget_exceeded` warn event, so the mesh stays
inspectable in the frontend. Enforcement is opt-in via
`BED_MESH_CHECK CHECK_Z_LIMITS=1` (e.g. in `PRINT_START`), which raises
the same message as a hard error. Realistic meshes give coupling of a few
mm/s against typical Z limits; a mesh that trips this check means a
genuinely warped bed or absurdly low Z limits — a hardware-setup problem,
surfaced before the first move.

**Documented exceedance envelope.** A move that is simultaneously
Z-velocity-limited, crossing the steepest mesh cell, with slopes aligned,
can transiently exceed the configured Z limit by up to the coupled worst
case above. Inside the fade band the fade derivative adds a further
bounded term: adopt mainline's activation-time validation
`fade_dist > mesh z-range`, which caps it at a known multiple of ż_g.
Both bounds are computable at activation; the total envelope
(`z_limit + coupling_max`, fade term folded in) is logged when the mesh
activates. This transient is *accepted* design margin — configured Z
limits are not at the physical skip threshold — and it is the price of
continuous Z with an untouched planner. (Mainline is stricter here only
because its split sub-moves each pass `kin.check_move`, which is the
staircase we're eliminating.)

**Assert at the envelope, not the limit.** The lowerer's loud backstop
assert fires when warped Z motion exceeds the *envelope* — that means the
math is wrong somewhere, a real logic error. Exceeding the raw configured
limit within the envelope is expected behavior and must not abort a print.

**Zero reference is required.** The mesh is normalized so it evaluates to
exactly 0 at `zero_reference_position` (default: the Z-homing XY); fail
activation if the reference lies outside the mesh. A mesh that is nonzero
at the home point silently shifts the global Z datum — that bug is made
impossible, not detected.

## Homing, probing, calibration

- **Homing:** approach moves are warped like any move (harmless — the
  endstop trips physically; with zero reference at the home XY the warp
  there is ~0 anyway). The trigger position arrives machine-space; the
  bridge inverse yields the gcode position. This mirrors mainline's
  `reset_last_position` reconciliation, relocated to the one seam.
- **Discrete probing** (probe, z_tilt, QGL, screws_tilt): trigger positions
  are machine-space physical measurements; an active mesh cannot corrupt
  them. No clearing required for correctness.
- **Continuous scan probing** (Beacon-style eddy-current sweeps): the one
  case where the warp touches a measurement, because samples stream in
  *during* commanded motion — an active mesh would ride Z up and down under
  the sweep and fold the old mesh's shape into the scan. Calibration paths
  therefore clear the mesh first **as an accuracy measure**, and the host
  emits a loud warning if a continuous scan starts while a mesh is active
  (Python owns both the scan path and the mesh state, so the condition is
  trivially detectable). The system stays correct if someone forgets;
  accuracy degrades, data doesn't corrupt — but a contaminated scan can be
  *saved* as a profile, so the warning keeps that from happening silently.
- `BED_MESH_CALIBRATE` clearing the mesh is likewise hygiene (flat travel
  during probing, matches mainline expectations), not a correctness
  requirement.

## Inverse transform

The general algebraic fade inverse (solving for the fade factor when the
gcode Z is unknown, as mainline's `get_position` does) is **v1-required**,
not deferred: the mesh-swap reconciliation above must run at an arbitrary
rest Z, and the common workflow — home, QGL, park at Z=5–10, then
`BED_MESH_PROFILE LOAD` with a typical fade band of 1–10 — activates the
mesh with the toolhead mid-band. Restricting activation height instead
would trade ~10 lines of well-understood arithmetic (ported from mainline,
quoted in the reference below) for user-facing errors; not worth it. The
inverse remains a pure function invoked only at the bridge conversion
sites and the mesh-swap reconciliation, all at rest.

## Scope

Non-affine surface transforms only:

- **v1: bed mesh** (surface × fade), as above.
- **Later: z_thermal** — a time-varying uniform Z offset; the degenerate
  surface with zero gradient. It inherits all the machinery (sampler seam,
  token updates, inverse) for free.
- **Out of scope: skew correction and other affine transforms.** Affine
  maps take lines to lines and would belong at classify time if ever
  wanted; the position here is that skew is a build-quality problem, not a
  firmware problem.
- No general duck-typed transform-chain abstraction à la mainline's
  `set_move_transform`. One optional surface in the lowerer seam is the
  whole design.

## Reference: mainline mechanics (for comparison)

`klippy/extras/bed_mesh.py` on `main`: dense grid built once
(lagrange/bicubic per `algo`), runtime bilinear `ZMesh.calc_z`; fade factor
`(fade_end - z) / (fade_end - fade_start)` clamped to [0,1]; applied offset
`factor * (calc_z(x,y) - fade_target) + fade_target`; `MoveSplitter` emits
a sub-move whenever the faded offset drifts ≥ `split_delta_z` along 5 mm
checkpoints; inverse in `get_position` solves the fade algebra;
`set_zero_reference` subtracts `calc_z(ref)` from the whole matrix;
`BED_MESH_CALIBRATE` clears the mesh before probing; homing/probing bypass
the transform via raw toolhead coordinates and reconcile through
`gcode_move.reset_last_position`.
