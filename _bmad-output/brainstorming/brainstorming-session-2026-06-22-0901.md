---
stepsCompleted: [1, 2]
selected_approach: 'progressive-flow'
techniques_used: ['first-principles', 'what-if', 'constraint-mapping']
inputDocuments: []
session_topic: 'Porting Pressure Advance (PA) to the new (post-TOPP) motion path'
session_goals: 'Decide the right architecture for PA in the new path; support linear + tanh (and other) PA models from bleeding-edge; decide whether to port the post-processor logic as-is; remove extruder-derived XYZ limit coupling'
ideas_generated: []
context_file: ''
---

# Brainstorming Session Results

**Facilitator:** dderg
**Date:** 2026-06-22

## Session Overview

**Topic:** Porting Pressure Advance (PA) to the new (post-TOPP) motion path

**Goals:**
- Re-evaluate whether PA-as-post-processor is still the right architecture now that TOPP is gone
- Support multiple PA models: linear, tanh (from bleeding-edge branch), and room for more
- Determine if there is any reason NOT to port the old-path post-processor logic as-is
- Drop the TOPP-era coupling where XYZ motion limits were derived from extruder limits

### Session Setup

_Technical design brainstorm. User is the project owner / motion-planner architect. Facilitation should stay generative — surface architectural options, tradeoffs, and risks before converging._

## Phase 1 — Expansive Exploration (First Principles + What-If)

**First principle of PA:** a feed-forward correction on extruder position derived from extruder acceleration — push extra filament on accel, pull back on decel, to hold deposited width constant. `e(t) += k·ė(t)` is the linear instance.

**Grounded findings (verified against the live tree):**
- Live solver is `temporal::multi` (Consolini-Locatelli SOCP). The `temporal::topp` module is the dead solver.
- The follower→XYZ limit coupling (`follower_sets`, `emit_base_follower_rows`, `pa_demand_linearized`) exists ONLY under `temporal/src/topp/`. The live path never caps XYZ path velocity by extruder limits.
- Live path consumes followers only for **emission + cross-batch continuity** (`follower_history`, `exchange_follower_tails`), not for limiting the toolhead.
- **Input shaping is already a post-processor on the live path** — kernel applied at emit time (`emit_shaped.rs:227/336`), after planning, with no feedback into limits. This is the exact rail PA should ride.

**Decision implied:** the "decouple extruder limits from XYZ" goal is already satisfied by the live path — nothing to remove, only (a) don't port `topp/follower.rs` PA-constraint linearization, (b) consider deleting the dead `topp` follower coupling.

**Option space for where PA lives:**
1. Pure emit-time post-pass (`e(t)+=k·ė(t)` on final extruder NURBS) — matches input-shaping rail. ← front-runner
2. Post-pass + independent extruder velocity/accel clamp (clamp extruder, never toolhead)
3. Plan-time follower constraint (TOPP way) — REJECTED (dead solver, against decoupling goal)
4. Two-rate / smooth-time model port (richer model, orthogonal to placement)
5. Model-plugin seam: `PostProcessorType` enum already shaped for Linear / Tanh / future

## Phase 2 — Constraint Mapping (decisions)

**Q1 — accept transient extruder over-accel during PA, no clamp (Klipper-style):** YES. PA never slows the toolhead. Trajectory smoothness (not a clamp) keeps it sane.

**Q2 — extrude-only limits (REFINED to minimal):** NO generic follower limit. The follower just follows on co-moves — if it exceeds, it exceeds (consistent with Q1). The ONLY extruder limits are mainline's extrude-only fields, applied ONLY to virtual-path segments (`frontend.rs:123-131`):
- `max_extrude_only_velocity` (mainline internal `max_e_velocity`)
- `max_extrude_only_accel` (mainline internal `max_e_accel`)
- applied like mainline `limit_speed(max_e_velocity * inv_extrude_r, max_e_accel * inv_extrude_r)`; ratio ±1 on a pure virtual path.
- Adjacent mainline fields `max_extrude_only_distance` / `instantaneous_corner_velocity` are out of scope unless wanted.
Drop any `max_e_velocity` notion for the co-move follower. No XYZ coupling anywhere.

**Q3 — smooth_time / tanh:** NOT building tanh or smooth_time now. Build linear PA only. Keep the model seam so tanh can be added later. Bring smooth_time back only if a need appears.

**Key insight — why decoupling is safe AND why smooth_time is likely redundant here:**
- mainline ships linear PA *with* smooth_time because its base trajectory is trapezoidal (instantaneous accel steps) → bare PA would inject extruder velocity discontinuities.
- Our base trajectory is jerk-limited, C²-continuous piecewise cubic. `ė` and `a` are already continuous, so the `k·a` correction inherits that smoothness — no hard velocity step to smear. Smoothing is upstream in the planner, not in the PA pass.
- tanh remains useful only to *saturate the magnitude* of correction at high accel (a different concern than smoothness), hence "tanh-ready but not now."

## Phase 3 — Architecture (developed)

**PA placement:** emit-time post-processor on the existing input-shaping rail (`emit_shaped.rs`). Reuse `PostProcessorType::LinearPressureAdvance{k}` + `apply_derivative_gain`. Never feeds limits.

**"Port as-is?" — answered:** port the *emit-time derivative-gain* half as-is (it IS the new rail). REJECT the TOPP follower-constraint half (`topp/follower.rs` PA linearization). The old PA was two entangled things; only the clean half ports.

**Model seam (tanh-ready, minimal):** emit application dispatches on `PostProcessorType` → `correction(piece_derivatives)`. Linear = `k·ė`. Adding tanh later = new enum arm + new `type` string + new correction fn; no emit-plumbing or limit changes. The enum already provides this; the only discipline is keeping `k·ė` assumptions out of the planning/limit side (guaranteed by Q1).

**Two clean regimes for the extruder:**
- Co-move (XYZ+E): XYZ-limited, extruder follows arc-length, PA decorates at emit (may transiently exceed extruder accel — accepted).
- Extrude-only (virtual path): limited by mainline `max_extrude_only_velocity` / `max_extrude_only_accel`.

## Phase 4 — Action Path

**Build / port:**
1. **Linear PA = emit-time post-processor.** Reuse `PostProcessorType::LinearPressureAdvance{k}`, `apply_derivative_gain`, `[post_processor]` config, `SET_POST_PROCESSOR`. Clean half ports as-is.
2. **Extrude-only limits only.** Bring back mainline `max_extrude_only_velocity` / `max_extrude_only_accel`; apply ONLY to virtual-path segments. No co-move follower limit.
3. **Model seam.** Emit dispatches on `PostProcessorType` so a future `Tanh` arm needs no plumbing/limit changes.

**Reject / delete:**
4. Do NOT port `topp/follower.rs` PA-constraint linearization.
5. Delete dead `topp` follower coupling (`follower_sets` / `emit_base_follower_rows` / `pa_demand_linearized`) to prevent resurrection.

**Defer:** tanh, smooth_time.

**Verify before "done" (decided ≠ done):**
- ✅ RESOLVED: `apply_derivative_gain` (`post_processor.rs:201`) is real and correct. `BezierPiece.coeffs` are power-basis in `(u−u_start)` with `u` in real time, so `differentiate()` yields `dp/dt` aligned at index 0; line 211 computes `p(t)+k·p'(t)` exactly — no per-piece duration scaling needed. Tested by exact value check `derivative_gain_applied_exactly_on_nurbs` (PASS). Live runtime swap path tested by `update_post_processor_applies_to_new_plans_only` (PASS). **Step 1 shrinks to "confirm end-to-end + ensure follower emit path applies gain," not "build."**
- ✅ RESOLVED: PA × input-shaping order — application order is the `[axis] post_processors:` declaration order, type-agnostic. No special-casing between PA and shaping.
- ✅ RESOLVED (out of scope): step-rate ceiling on the transient PA spike — not our concern. Host emits the trajectory; MCU execution is the MCU's domain.
