---
id: SPEC-motion-pipeline-rewrite
companions:
  - architecture.md
sources: []
---

> **Canonical contract.** This SPEC and the files in `companions:` are the complete, preservation-validated contract for what to build, test, and validate. Source documents listed in frontmatter are for traceability only — consult them only if you need narrative rationale or prose color this contract intentionally omits.

# Motion-pipeline rewrite — geometry-first planner

## Why

A vision to realize and an opportunity to capture. The current planner reparameterizes a cubic-Bézier path to arc length (a per-segment numerical table) and runs a convex SOCP plus an SLP jerk-refinement loop plus iterative junction joining — correct, but too heavy to keep up in real time, and it never refits the faceted G1 stream into continuous motion (it caps junction velocity at geometric kinks instead of inserting smooth blends). The rewrite keeps the proven MCU/execution and structured-logging layers and restructures the geometry-and-planning front: a typed segment library (`Line | Arc | Clothoid` + a follower channel), a shaper-agnostic fitter that turns corners into clothoid blends, and a velocity planner that decouples lateral handling (into the geometry) from tangential jerk (a 1D S-curve in lookahead) — which lets the heavy convex solver drop out in favor of a fast forward-backward sweep. The ultimate payoff is the project's non-negotiable, minimum print time, but the immediate work is the *foundation*: interfaces that let each piece be tuned to its own optimum later, with no layer capping what an adjacent piece can achieve. The foundation's job is to not get in the way. Affected: anyone running this fork on real hardware, where throughput is the whole point.

## Capabilities

- id: CAP-1
  intent: The planner can ingest slicer G-code (G0/G1/G2/G3) and produce an internal geometric path of typed segments for any printable move, with extrusion carried as a follower channel.
  success: For a representative slicer program, the reconstructed path stays within junction-deviation δ of the intended geometry at every point, and a pure retraction (`G1 E-` with no XYZ) plans as a zero-length-spatial-path follower move.

- id: CAP-2
  intent: The planner can compute a time-optimal speed profile for a given geometry under the machine's acceleration limits via a forward-backward sweep, not a convex solver.
  success: For test and recorded move sequences in the accel-only configuration, the speed profile is no slower than the current planner's under the same limits — checkable offline, without whole-print simulation.

- id: CAP-3
  intent: The planner can refit sharp corners and tight arc seams into curvature-continuous clothoid blends, raising cornering speed within δ.
  success: For a corner test case the fitter emits a clothoid blend that is curvature-continuous (G2) and collinear (G1) with its neighbors and stays within δ; cornering speed exceeds a full-stop junction, and the blend is set by δ and max_accel alone, independently of the velocity planner.

- id: CAP-4
  intent: The planner can emit position-vs-time for execution, composed from the geometry and the speed profile, evaluable at the MCU's fixed step rate.
  success: The fixed-rate evaluator reproduces the planned path within step tolerance at the configured frequency; clothoid position is produced by a Fresnel approximation, not a per-segment arc-length table.

- id: CAP-5
  intent: The planner can handle 3D corner blends and bed-mesh-dependent Z while keeping planning effort at 2D cost.
  success: A vase-mode print and a meshed print both plan with Z folded in as a velocity cap; a mesh too steep for the Z limit fails loudly rather than crawling silently.

- id: CAP-6
  intent: Input shaping and pressure advance run as per-axis time-domain post-processors at execution; the extruder follower advances on the post-shaper (realized) velocity.
  success: A pure retraction plans as a virtual-path follower move; the follower position is reconstructed from the shaped signal (odometer) so PA reflects actual filament speed; shaper and PA never run upstream of velocity planning.

## Constraints

- Minimum print time is the objective; smoothness is only a means to raise the acceleration ceiling, never an end. The planner never knowingly ships a cheaper algorithm that yields a measurably slower trajectory than the best computable on the active hardware.
- Fail loudly: a too-steep mesh, a late segment, an infeasible move, or a segment that cannot absorb its own jerk-smoothing raises an explicit error code — never a padded start time or a silent slowdown.
- Design representation (curvature, via typed segments) and execution representation (time-polynomials) are separate artifacts; no single object serves both roles. Jerk-smoothing lives entirely in `s(t)` (timing) and never alters geometry — so it can produce no geometric artifact; its only costs are time and required braking length.
- The fitter is **shaper-agnostic by authority, not by approximation**: the planner never limits acceleration for ringing reasons. `max_accel` is the user's sovereign knob (resonance measurement is imperfect; `calibrate_shaper` only suggests). Clothoid smoothness helps the shaper one-way, never negotiated.
- Lateral (centripetal) jerk is bounded for free by the clothoid geometry — there is no jerk constraint for it in the planner. Tangential jerk is a 1D S-curve in the velocity lookahead (not a post-processor); on short straights it trims peak velocity so the accel→decel reversal fits a bounded-jerk ramp.
- Velocity planning is a forward-backward sweep over closed-form clothoid corner caps with S-curve ramps — not a convex SOCP. Decoupling lateral (geometry) from tangential (1D) is what removes the heavy solver and buys real-time.
- Acceleration is the invariant; corner velocity is derived (`v = √(a/κ)`). Each clothoid carries a pointwise cap `v ≤ √(a/κ(s))` (the apex binds); the velocity profile over a clothoid is an output of the global lookahead, never prescribed locally (it may be all-decel, all-accel, a dip, or cruise).
- Acceleration continuity is two mechanisms: lateral via curvature continuity (the fitter inserts clothoids so κ never steps), tangential via a single shared boundary sample node in the joint sweep. The fitter guarantees collinearity (G1) at every segment boundary except move starts.
- Segments are a Rust enum `{ Line | Arc | Clothoid }` plus a follower/virtual-path channel (extruder = follower; pure retraction = zero-length spatial path). Each is closed-form in design space; the clothoid's `κ(s)` is linear/closed-form (velocity planning needs only κ), and its position uses a Fresnel approximation at execution lowering — no generic numerical integration of κ, no `dyn` dispatch.
- Machine limits are a swappable convex body, queried only as "max accel/vel in direction d." V1 body: a global scalar XY disk × a separate Z limit (3D from day one). *(Carried from the original map; not re-examined in design discussion.)*
- V1 user knobs: absolute `max_accel` and junction deviation δ — equivalently Square Corner Velocity, `SCV = √(a_max/κ_peak(90°, δ))`, with corner-angle dependence from a closed-form `κ_peak(angle, δ)`.
- The pipeline is a chain of independently testable IR→IR interfaces; the IR is progressively enriched (geometry → +caps → +timing), ideally type-encoded so a pass cannot run before its inputs exist.
- The existing MCU/execution transport and the structured-logging layer are reused unchanged.
- Build as a walking skeleton: ship the constant-speed-per-feature velocity rule first (mainline-parity), then turn on limit-riding through clothoids as a measurable upgrade — without touching the fitter or the S-curve lookahead.

## Non-goals

- A shaper-aware fitter, or any guarantee that the *post-shaper* path lands within δ. Decided non-goal: the planner never caps acceleration for ringing; δ is a pre-shaper geometry tolerance only. Recoverable later (the fitter is just a pass) without disturbing the architecture.
- Per-axis XY / ellipse constraint body / anisotropic blend shapes and their boundary-discontinuity handling. The convex-body abstraction makes it a later body swap with no downstream changes.
- Native G5 cubic-Bézier input (no slicer emits it). Dropped in V1; a later front-end may add it without refitting.
- Helical (Z-component) G2/G3 arcs. No mainstream slicer emits them; V1 does not support them and fails loudly — pending a decision to instead fold Z as a dependent axis.
- Full jerk-limited, non-convex (SLP) TOPP. Explicitly replaced: lateral jerk into geometry, tangential jerk into a 1D S-curve.
- Per-piece performance tuning to reach ultimate trajectory speed, and whole-print (faster-than-real-time) benchmarking. This spec builds the foundation that *enables* that tuning; it does not perform it.

## Success signal

The foundation is sound when every pipeline interface is independently testable and no layer caps the trajectory optimality a downstream piece can reach. Concretely: the walking skeleton runs end-to-end; the heavy convex solver is gone — velocity planning is a forward-backward sweep that keeps up in real time on the active host; and tightening any single pass (a better clothoid, the limit-riding velocity rule, a finer S-curve) improves its output and flows through to the trajectory without edits to adjacent layers.

## Assumptions

- This is a from-scratch rewrite of the geometry and planning layers only; the MCU/execution transport and structured-logging layers stay as-is (confirmed — they work well).
- Whole-print simulation faster than real time does not exist yet, so V1 validation is per-interface and on synthetic or recorded move sequences rather than whole prints.

## Open Questions

- Jerk-limit floor: the largest tangential jerk that still lets `max_accel` reach the no-ring ceiling, set so smoothing never inflates a braking distance past the safety margin (derivable, not guessed).
- `κ_peak(angle, δ)` closed form, and the SCV↔δ mapping (with corner-angle dependence) built on it.
- Forward-backward sweep convergence: does the S-curve-distance + clothoid-cap reconciliation converge cheaply and non-iteratively enough to be real-time on the active host? *(The last unverified load-bearing claim.)*
- Ride-the-accel-limit through clothoids (a cheap 1D ODE per clothoid, captures the budget-trading speed) vs the constant-speed-per-feature cap (simpler skeleton): when to switch on.
- Execution lowering: Fresnel approximation degree/tolerance and piece-boundary continuity when baking clothoid + `s(t)` into time-polynomials.
- Dumb-gcode reconstruction: detecting faceted-curve "chains" vs intended corners in plain G1 streams.
- Back-to-back clothoids with no straight between → `dκ/ds` (lateral jerk) step at the junction. Edge case.
- Helical G2/G3 support: fail loudly vs fold Z as a dependent axis.
- Extruder post-IS profile: can the full follower profile computation be simplified? TODO after basic movement works.
- *(Carried from the original map, not re-examined in discussion:)* the convex-body shape, the 3D planar pairwise-blend solver, and bed-mesh-Z as a dependent axis.
