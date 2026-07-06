# Investigation: How jerk is used in the live motion planner

## Hand-off Brief

1. **What happened.** The user reports jerk (a) isn't read from `max_jerk` and (b) is applied only on straight-line acceleration from standstill, not at cruise or during deceleration. **Confirmed in substance, but the root cause is architectural:** the live planner is the *geometry* velocity-envelope model (`geometry::plan_velocity_warm_start`), whose tangential-jerk limit is a phase-plane reachability bound anchored only at rest anchors — not the everywhere-enforced TOPP/SOCP jerk constraint, which exists in `temporal/` but is **test-only / not wired into production**.
2. **Where the case stands.** Root cause Confirmed via code trace + the spec-motion-11 cutover dev-log. The two symptom clusters map to two distinct facts: a jerk-config wiring gap (only `[printer] max_jerk` is honored; `[limit]` sections are inert; the **viz hardcodes the default**), and a jerk *model* that only shapes accel-from-rest and decel-to-stop, leaving mid-run decel-into-curvature and the cruise apex governed by accel/disk reach with no jerk term.
3. **What's needed next.** Per the frozen architecture (spec-motion-pipeline-rewrite), the SOTA target is the *decoupled* planner (lateral jerk → clothoid geometry; tangential jerk → closed-form seven-segment S-curve; node-based forward-backward sweep), NOT the sampled Consolini-Locatelli SOCP in `temporal/` — which the architecture explicitly *avoids* and spec-motion-11 Stage 3 *removes*. The real fix is to finish the decoupled model: carry tangential acceleration continuously across the run (spec-motion-7 limit-riding) so the current C1 velocity-ceiling jerk bound becomes a jerk-continuous accel profile. Plus the wiring gaps (viz + per-section jerk). Confirm which surface the user observed (viz vs. live print) to prioritize.

## Case Info

| Field            | Value                                                                      |
| ---------------- | -------------------------------------------------------------------------- |
| Ticket           | N/A                                                                        |
| Date opened      | 2026-06-20                                                                 |
| Status           | Active                                                                     |
| System           | curvature-profile worktree; Rust motion-engine; HEAD 9ebf70d7a            |
| Evidence sources | Rust source (geometry/temporal/motion-engine), klippy/motion.py, spec-motion-6 & spec-motion-11 |

## Problem Statement

User (verbatim): "the way jerk is used now, there are a few problems. seems it's not being read from max_jerk parameter, and it's only applied on straight line acceleration from stand still, but not when it reaches the cruize speed, and not when it starts deccelerating."

## Confirmed Findings

### Finding 1: The live planner is the geometry velocity-envelope model; the TOPP/SOCP jerk planner is test-only

**Evidence:** `motion-engine/src/bridge.rs:3333` spawns `StreamPlannerHandle::spawn(stream_cfg, …)` → `stream_planner.rs:56` → `StreamState::new` (`crate::stream`) → `stream.rs:189` calls `plan_velocity_warm_start` imported from `geometry` (`stream.rs:6`). The TOPP path (`planner.rs::PlannerHandle` → `to_temporal_limits()` → `trajectory::beta::plan_velocity_inner` → `temporal::multi::plan_batch`) is referenced **only** in `planner/tests.rs` and `bridge/tests.rs`.

**Corroboration:** spec-motion-11 dev-log (2026-06-19): "the per-axis temporal planner is test-only"; "`[limit <name>]` sections … never consulted by the stream."

### Finding 2: Jerk config wiring — only `[printer] max_jerk` is honored; `[limit]`-section jerk is inert; the viz ignores jerk entirely

**Evidence:**
- Live path reads global jerk: `bridge.rs:3319` `max_jerk_mm_s3: cart.max_jerk` (from `[printer] max_jerk`, `klippy/motion.py:645`, default `2×max_accel`).
- `[limit]`-section jerk would flow only through `PlannerConfig::path_velocity_config()` (`config.rs:571`) — which has **no callers** (grep: definition only). The stream never consumes it. `to_temporal_limits()` (`config.rs:599`) reads section jerk but feeds only the test-only TOPP planner.
- **Visualization ignores jerk config:** `viz.rs:7` `pipeline_snapshot(... max_velocity, max_accel, square_corner_velocity ...)` takes **no `max_jerk` arg**; `viz.rs:32` calls `geometry::plan_velocity(&outcome, geometry::VelocityConfig::default())` → hardcoded `max_jerk_mm_s3 = 100_000.0` (`velocity.rs:28`).

**Detail:** If the user set jerk in a `[limit]` section it is silently ignored. If the user is reading jerk behavior off the matplotlib/native-geometry viz, it never reflects any configured jerk — fully explaining "not being read from max_jerk."

### Finding 3: The tangential-jerk model is a rest-anchored velocity envelope, not an everywhere jerk constraint

**Evidence:** `geometry/src/velocity.rs:237-264` runs a forward jerk reach (`scurve::max_reachable_velocity(run_start_v[j], arc_from_run_start[j]+len, accel, jerk)`) and a backward jerk reach (`… (0.0, arc_to_run_end[j]+len, …)`). Per-sample reconstruction `disk.rs:142-167 profile_speed` takes `min(forward_disk, backward_disk, jerk_forward, jerk_backward, ceiling)`. `JerkAnchors` (`velocity.rs:292-297`) sets `bwd_v: 0.0` — the backward jerk ramp is anchored at the run's terminal rest (v=0). Commit 311ac5144 moved the anchor from per-move seams to run rest-anchors.

**Detail / mapping to symptoms:**
- **Accel from rest:** `jerk_forward` binds from the v=0 start anchor → jerk-limited ramp is visible (matches "applied on straight-line accel from standstill").
- **Cruise:** constant velocity ⇒ tangential accel ≈ 0 ⇒ no jerk to round; only the cruise *peak* is trimmed (`scurve::jerk_limited_apex`, spec-motion-6). "No jerk at cruise" is expected, not a defect.
- **Deceleration:** `jerk_backward` is anchored at run-end v=0, so decel **into a genuine stop** is jerk-shaped. But deceleration **into a mid-run curvature-limited feature** is bounded by the accel/disk reach + curvature ceiling (`disk.rs:159`), which carry **no jerk term** → tangential accel can step at those corners (effectively unbounded jerk there). This is the most likely basis for "not when it starts decelerating."

**Corroboration:** spec-motion-6 line 40: "No fully-coupled jerk TOPP / SOCP / SLP; no per-axis or lateral jerk constraint." The envelope model is by-design partial.

## Deduced Conclusions

### Deduction 1: Two independent issues are being reported as one

**Based on:** Findings 2 and 3.

**Reasoning:** "Not read from max_jerk" is a *config-wiring* issue (inert `[limit]` jerk + viz hardcodes the default). "Only on accel-from-rest" is a *model-scope* issue (rest-anchored envelope, no jerk term on mid-run decel/curvature transitions). They have different fixes.

**Conclusion:** Treat them separately; do not assume one patch addresses both.

## Hypothesized Paths

### Hypothesis 1: The user observed jerk via the visualization, not a live print

**Status:** Open

**Theory:** Recent commits (b8fc39fee native-geometry viz, 311ac5144 jerk-anchor) show active viz work. `pipeline_snapshot` hardcodes default jerk and takes no `max_jerk`, so the viz never reflects config.

**Would confirm:** User states they judged jerk from the viz / `pipeline_snapshot` output.
**Would refute:** User changed `[printer] max_jerk` and saw no change on a real print (that would point at the live path / a deeper wiring bug to chase).

## Source Code Trace

| Element       | Detail                                                                                   |
| ------------- | ---------------------------------------------------------------------------------------- |
| Live planner  | `bridge.rs:3333` → `stream_planner.rs:56` → `stream.rs:189` `plan_velocity_warm_start` (geometry) |
| Jerk value (live) | `bridge.rs:3319` `cart.max_jerk` ← `[printer] max_jerk` (`klippy/motion.py:645`)       |
| Jerk value (viz)  | `viz.rs:32` `VelocityConfig::default()` → `velocity.rs:28` hardcoded `100_000.0`        |
| Jerk model    | `velocity.rs:237-264` sweeps; `disk.rs:142-167 profile_speed`; anchors `velocity.rs:292` |
| Dead/test-only| `config.rs:571 path_velocity_config` (no callers); `planner.rs PlannerHandle` (tests only); `temporal/` TOPP |

## Conclusion

**Confidence:** High

The premise is substantially correct, but the cause is architectural. The production trajectory is built by the geometry velocity-envelope planner, which (a) honors only the global `[printer] max_jerk` — `[limit]`-section jerk is inert and the viz ignores jerk altogether — and (b) enforces tangential jerk only as a rest-anchored reachability envelope, so it shapes accel-from-rest and decel-to-stop but leaves the cruise interior and mid-run decel-into-curvature governed by accel/disk limits with no jerk term. The fully-coupled jerk planner (TOPP/SOCP in `temporal/`) that would bound jerk everywhere is implemented but dormant (test-only), pending spec-motion-11's production cutover.

## Recommended Next Steps

### Fix direction
- **Wiring (cheap):** give `pipeline_snapshot` a `max_jerk` arg and stop hardcoding `VelocityConfig::default()` in `viz.rs:32`; decide whether `[limit]`-section jerk should be honored or explicitly rejected (fail-loudly) instead of silently inert.
- **Model (the real question):** the frozen architecture (architecture.md:53-55, SPEC.md:23) *decouples* jerk — lateral into clothoid geometry, tangential into a closed-form seven-segment S-curve — and *avoids* the sampled Consolini-Locatelli SOCP (architecture.md:124; "the coupled-jerk SOCP we are avoiding"). So the fix is NOT to wire in `temporal`/TOPP (spec-motion-11 Stage 3 *removes* it). It is to finish the decoupled model: make tangential acceleration continuous across each run (ride |a|=a_max, relax to a=0 only at true rest anchors — spec-motion-7), upgrading the current C1 velocity-ceiling jerk bound (`scurve::max_reachable_velocity`) into a jerk-continuous acceleration profile. This is what kills the mid-run decel/curvature jerk steps without re-introducing a convex solver.

### Diagnostic
- Confirm the observation surface (viz vs. live print) — settles Hypothesis 1 and prioritizes the wiring vs. model fix.

## Side Findings
- `path_velocity_config()` / `path_velocity_limits()` are retained on `PlannerConfig` for the test-only temporal planner; they are dead w.r.t. production (spec-motion-11 line 86). Candidate for removal at cutover.
