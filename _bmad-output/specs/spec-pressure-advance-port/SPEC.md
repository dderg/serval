---
id: SPEC-pressure-advance-port
companions: [code-map.md]
sources: [../../brainstorming/brainstorming-session-2026-06-22-0901.md]
---

> **Canonical contract.** This SPEC and the files in `companions:` are the complete, preservation-validated contract for what to build, test, and validate. Source documents listed in frontmatter are for traceability only — consult them only if you need narrative rationale or prose color this contract intentionally omits.

# Pressure Advance on the new (post-TOPP) motion path

## Why

Pressure Advance (PA) was left behind when TOPP was removed; the new `temporal::multi` (Consolini-Locatelli) path ships without it. PA is feed-forward filament compensation — push extra on accel, pull back on decel — needed for print quality parity. This is an **opportunity to capture cleanly**: the new path already carries the linear-PA kernel, config surface, and live runtime-tuning path (all tested), and removing TOPP already deleted the extruder→XYZ limit coupling for free. So the work is to re-light PA on the existing post-processor rail, add the one missing extruder limit (extrude-only), structure for future models, and make sure the old coupling cannot creep back. The anchor every trade-off resolves against: **never spend XYZ trajectory time to babysit the extruder** (project throughput non-negotiable).

## Capabilities

- id: CAP-1
  intent: System applies linear PA as an emit-time correction `e(t) += k·ė(t)` on the extruder follower curve, after the toolhead trajectory is planned.
  success: A co-extruding move planned with PA `k>0` emits an extruder trajectory shifted by `k·ė` relative to the same move at `k=0`, while the XYZ trajectory and its timing are byte-for-byte identical between the two. End-to-end test on the follower-emit path asserts both.

- id: CAP-2
  intent: Operator configures PA via `[post_processor]` config and retunes it live with `SET_POST_PROCESSOR NAME=<pa> K=<value>`.
  success: A live `SET_POST_PROCESSOR` changes `k` and the new value applies only to plans produced after the command (already covered by `update_post_processor_applies_to_new_plans_only`); held output committed before the swap is unchanged.

- id: CAP-3
  intent: Planner limits pure-extrusion (virtual-path) moves by the extruder's own velocity/accel, using mainline's config fields.
  success: A pure-E move (e.g. retraction) respects `max_extrude_only_velocity` / `max_extrude_only_accel`; a co-move's XYZ velocity/accel is provably unaffected by any extruder limit.

- id: CAP-4
  intent: PA model is selected by the post-processor `type`, dispatched at emit so additional models can be added without touching planning, limits, or emit plumbing.
  success: Linear is the only model shipped; the dispatch seam is demonstrable such that adding a `tanh` arm would require only a new `PostProcessorType` variant, a new `type` string, and a new correction function — no changes to limit/planning code or the emit wiring. Application order against other post-processors follows the axis declaration order, with no model-specific handling.

## Constraints

- Post-processors (PA and input shaping) are applied **after** limit computation and **never** feed back into limit calculation. Limits are computed first; post-processing decorates the planned motion.
- On co-moves the extruder is a pure follower with **no** limit of its own; if PA or the follow ratio pushes it past its kinematic limits, that is accepted (Klipper-style). Trajectory smoothness, not a clamp, keeps it sane.
- No extruder limit may constrain XYZ velocity or acceleration. The TOPP-era follower→XYZ coupling stays gone.
- Extrude-only limiting reuses mainline field names verbatim — `max_extrude_only_velocity`, `max_extrude_only_accel` — applied only to virtual-path segments, mainline-style (`limit_speed(max_e_velocity * inv_extrude_r, max_e_accel * inv_extrude_r)`).
- Post-processors apply to an axis in the order they are declared in `[axis] post_processors:` — type-agnostic. PA and input shaping are not special-cased relative to each other; declaration order is the order of application.
- The dead `temporal::topp` follower coupling (`follower_sets`, `emit_base_follower_rows`, `pa_demand_linearized`) must be deleted, not left dormant, so the decoupling cannot regress.
- Fail loudly per project rule: unexpected planner state raises a clear error rather than silently padding or recovering.

## Non-goals

- tanh PA model — deferred. CAP-4 provides the seam only; no tanh arm ships now.
- `smooth_time` — deferred and likely redundant: the C²-continuous jerk-limited base trajectory already makes `ė`/`a` continuous, so linear PA injects no velocity discontinuity to smear. Re-introduce only if a concrete need appears.
- `max_extrude_only_distance` and `instantaneous_corner_velocity` — out of scope.
- Any plan-time extruder constraint or revival of the TOPP solver.
- MCU step-generator behavior under a large-`k` PA step-rate spike — not our concern. The host emits the trajectory; how the MCU executes it is the MCU's domain.

## Success signal

A representative co-extruding print runs with PA active: corner/segment-boundary extrusion is pressure-compensated, the XYZ trajectory timing is identical to a PA-off run (zero throughput cost), and pure-E retractions obey the extrude-only limits. The `temporal::topp` follower-coupling code is deleted and the full Rust suite is green.

## Assumptions

- The follower-emit path (`emit_shaped.rs` follower branch) is where the gain must be confirmed applied; the gain kernel itself (`apply_derivative_gain`) is already real, correct, and value-tested, so the remaining step-1 work is wiring confirmation, not building the math.
- The live runtime already swaps `k` correctly between plans (tested), so CAP-2 is largely confirmation plus config-surface parity.

