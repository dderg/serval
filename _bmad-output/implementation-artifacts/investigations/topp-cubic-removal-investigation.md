# Investigation: Can we remove the TOPP solver and the old cubic-based pipeline?

## Hand-off Brief

1. **What happened.** The live production PyO3 path (`StreamPlannerHandle` → `stream::commit` → `geometry::plan_velocity_warm_start`, the curvature-aware RK4 disk-kinematics solver) no longer touches TOPP or the cubic `trajectory::beta`/`reparam`/`smooth_fit` pipeline — **Confirmed** by tracing the bridge from `init_planner` down (bridge.rs:617,3339 → stream.rs:201-202).
2. **Where the case stands.** Both old systems (`temporal` TOPP solver + `trajectory::beta` cubic reparam) are reachable **only** through `planner.rs::PlannerHandle`, which is never spawned in production — it is test/example-only. So they are removable *in principle*. The blocker is **feature parity**: the new geometry pipeline explicitly rejects G5 cubic béziers, G2/G3 quadratics, and multi-follower moves (bridge.rs:3448/3481/3376), and `temporal`'s *limit/binding value types* (not the solver) are still imported by live config/diagnostics code.
3. **What's needed next.** Decide the cut in two stages: (a) immediately delete the dead old planner path (`planner.rs` MoveArmThread + `trajectory` cubic modules + `temporal::topp`/`multi` solver) **once** the geometry pipeline reaches parity on bézier/quadratic/follower moves; (b) first untangle the residual `temporal::{Limits,LimitSet,LimitKind,BindingConstraint}` *type* dependency in `config.rs`/`binding_report.rs`. Do not remove yet — parity gap is real.

## Case Info

| Field            | Value                                                                      |
| ---------------- | -------------------------------------------------------------------------- |
| Ticket           | N/A                                                                        |
| Date opened      | 2026-06-21                                                                 |
| Status           | Active (exploration / area-mapping)                                        |
| System           | kalico fork, branch `curvature-profile`, Rust workspace `rust/`            |
| Evidence sources | Source code (motion-engine, temporal, trajectory, geometry), graphify graph, Cargo manifests |

## Problem Statement

User: "We still haven't ripped out TOPP solver, how are we using it and can we remove the old cubic based pipeline?"

Refined by evidence: "TOPP solver" = the `temporal` crate (`topp` + `multi` modules, Consolini-Locatelli 2024 SOCP via Clarabel). "Old cubic based pipeline" = `trajectory::beta`/`reparam`/`smooth_fit` (arc-length→time cubic reparameterization that consumes TOPP's v(s) profile). These are **one coupled stack**, both driven by `planner.rs::PlannerHandle::append_and_replan`.

## Evidence Inventory

| Source                          | Status    | Notes                                                                 |
| ------------------------------- | --------- | --------------------------------------------------------------------- |
| `motion-engine/src/bridge.rs`   | Available | Production PyO3 entry; holds `StreamPlannerHandle`, never `PlannerHandle` |
| `motion-engine/src/stream*.rs`  | Available | Live planner thread + `commit()` dispatch                             |
| `motion-engine/src/planner.rs`  | Available | Dead old `PlannerHandle`/`MoveArmThread` (TOPP+cubic driver)          |
| `temporal/` crate               | Available | TOPP SOCP solver; also exports limit/binding *value types*           |
| `trajectory/` crate             | Available | Cubic reparam pipeline + the live `ShapedSegment` output type        |
| `geometry/` crate               | Partial   | New `plan_velocity_warm_start` solver — not deep-read this pass       |
| Throughput parity data (new vs old) | Missing | No measurement that new geometry pipeline ≥ TOPP trajectory time     |

## Investigation Backlog

| # | Path to Explore                                                                 | Priority | Status | Notes |
| - | ------------------------------------------------------------------------------- | -------- | ------ | ----- |
| 1 | Untangle `temporal::{Limits,LimitSet,LimitKind,BindingConstraint}` type usage in `config.rs`/`binding_report.rs` from the solver | High | Open | These types survive even after the solver is cut; must move to `geometry` or a types-only crate |
| 2 | Confirm geometry pipeline parity for G5 bézier (3448), G2/G3 quadratic (3481), multi-follower (3376) | High | Open | These are the explicit "not yet supported by the new pipeline" gaps gating removal |
| 3 | Throughput/optimality comparison: `geometry::plan_velocity_warm_start` vs `temporal` TOPP on representative slicer output | High | Open | CLAUDE.md: throughput is non-negotiable; old stack is the optimality reference |
| 4 | Enumerate tests/examples that would break on removal (streaming_replan.rs, follower_lane_e2e.rs, z_hop_cold_boot.rs, temporal/*, trajectory/* test suites) | Medium | Open | Removal cost is dominated by test churn, not prod code |

## Confirmed Findings

### Finding 1: Production PyO3 planner is `StreamPlannerHandle`, not `PlannerHandle`

**Evidence:** `bridge.rs:617` `planner: Mutex<Option<StreamPlannerHandle>>`; initialized `None` at `bridge.rs:874`; set to `Some(StreamPlannerHandle::spawn(...))` at `bridge.rs:3339`. `PlannerHandle` is imported nowhere in bridge.rs (only `crate::planner::{DispatchError, HomeDripParams, NudgeParams}` types, bridge.rs:22).

**Detail:** The only `*Handle::spawn` in production is `StreamPlannerHandle::spawn` (bridge.rs:3339). `grep` for `PlannerHandle::spawn|MoveArmThread|PlannerHandle::new` outside tests returns nothing. The old `MoveArmThread` is never started.

### Finding 2: The live path never calls TOPP or the cubic reparam pipeline

**Evidence:** `stream_planner.rs` imports only `trajectory::ShapedSegment`, `geometry::Move`, `crate::stream` — no `temporal`, no `trajectory::beta`/`reparam`, no `topp`. `stream.rs:201-202`: `commit()` calls `fit_chain(...)` then `plan_velocity_warm_start(...)`, both imported from `geometry` (stream.rs:5-6).

**Detail:** `trajectory::ShapedSegment` is consumed purely as the **output data type** of the live path, not the cubic algorithm. The live velocity solve is `geometry::velocity::plan_velocity_warm_start` (curvature-aware RK4 disk kinematics — the `curvature-profile` branch work).

### Finding 3: TOPP + cubic are coupled and reachable only via the dead `PlannerHandle`

**Evidence:** `planner.rs:14` `use trajectory::streaming::{...ShaperState}`; `planner.rs:620,904` `state.append_and_replan(...)`; that path runs `trajectory::beta::beta_loop` → `temporal::multi::plan_batch` (beta.rs:440, per agent trace) → Clarabel SOCP. `planner.rs:992` `temporal::multi::GridStrategy::Adaptive`. All inside `MoveArmThread::run_planner`, which is never spawned (Finding 1).

**Detail:** `temporal` Cargo description: "Consolini-Locatelli 2024 SOCP" (temporal/Cargo.toml:7). Only two non-test workspace dependents of `temporal`: `motion-engine` and `trajectory` (Cargo manifests). `trajectory` depends on `temporal` (trajectory/Cargo.toml:9). So the dependency chain for the solver is `planner.rs → trajectory::beta → temporal`, all dead in prod.

### Finding 4: The new geometry pipeline is NOT at feature parity — removal is gated

**Evidence:** `bridge.rs:3448` "submit_bezier (G5 cubic) is not yet supported by the new geometry pipeline"; `bridge.rs:3481` "submit_quadratic (G2/G3 arc) is not yet supported by the new …"; `bridge.rs:3376` "submit_move: multiple follower axes not yet supported by the new pipeline."

**Detail:** The old TOPP/cubic stack handled curves and multi-axis followers (temporal `follower` module, 24.7KB). The new pipeline currently rejects those move classes. Removing the old stack now would permanently drop capability the new path can't yet serve.

### Finding 5: `temporal` is still a *live* dependency for value types, independent of the solver

**Evidence:** `config.rs:485,491,501-502,599,644-657` use `temporal::{Limits,LimitSet,AxisSet,LimitKind,LimitsError,N_SPATIAL}`. `binding_report.rs:3,15,32-34,83,147,167` use `temporal::{BindingConstraint,LimitKind}`. These are in non-test motion-engine code.

**Detail:** However, `config.rs::to_temporal_limits()` (the `temporal::Limits` constructor) is called **only** from `planner.rs:518,982` — the dead path. So `temporal::Limits` itself dies with the planner, but `LimitSet`/`LimitKind`/`AxisSet`/`BindingConstraint` usage in `config.rs::to_set`/`binding_report.rs` needs a per-symbol check to see which survive into the `StreamPlanner` config/diagnostics surface.

## Deduced Conclusions

### Deduction 1: Both "TOPP solver" and "old cubic pipeline" are dead production code, removable as a unit

**Based on:** Findings 1, 2, 3.

**Reasoning:** Production enters only via `StreamPlannerHandle` (F1). That path uses `geometry` velocity planning exclusively (F2). TOPP and cubic reparam are reachable only through `PlannerHandle`/`MoveArmThread`, which is never spawned (F3). Nothing in the live path can reach them.

**Conclusion:** `planner.rs` (old MoveArmThread/PlannerHandle), the `trajectory` cubic modules (`beta`, `reparam`, `smooth_fit`, `plan_velocity`, `streaming` replan, `emit_shaped`'s cubic-fit consumers), and the entire `temporal` *solver* (`topp`, `multi`) are dead in production and could be deleted without changing runtime behavior — subject to Deductions 2 & 3.

### Deduction 2: Removal must wait on geometry-pipeline parity

**Based on:** Finding 4 + CLAUDE.md throughput/optimality constraint.

**Reasoning:** The new pipeline rejects bézier/quadratic/follower moves (F4). Deleting the old stack removes the only implementation that handles them. CLAUDE.md forbids shipping a measurably worse trajectory; the old TOPP stack is also the optimality reference there's currently no parity measurement against (Backlog #3).

**Conclusion:** The old stack is functioning as the **fallback / reference implementation** while the geometry pipeline matures. Premature removal forfeits capability and the throughput baseline.

### Deduction 3: `temporal` cannot be fully deleted until its value types are rehomed

**Based on:** Finding 5.

**Reasoning:** Even with the solver dead, `config.rs`/`binding_report.rs` import `temporal` limit/binding *types* in live code. `trajectory::ShapedSegment` (live output type) also keeps the `trajectory` crate alive.

**Conclusion:** "Remove TOPP" splits into: delete the solver modules (`topp`, `multi`) — clean; but keep or migrate `temporal`'s `limits`/binding type module, and keep `trajectory` for `ShapedSegment`. The crates shrink; they don't vanish.

## Hypothesized Paths

### Hypothesis 1: `emit_shaped` / `smooth_fit` cubic post-processing is also dead

**Status:** Open

**Theory:** `trajectory::emit_shaped` + `smooth_fit` (C² cubic fitting) were part of the old post-processor chain. If the live `geometry` path emits `ShapedSegment` directly via `lowering.rs`, these are dead too.

**Would confirm:** No non-test reference to `trajectory::emit_shaped`/`smooth_fit`/`post_processor` from the `StreamPlanner`/`stream`/`lowering` live path.

**Would refute:** `lowering.rs` or `stream.rs` calls into `trajectory::emit_shaped`/`post_processor`.

**Resolution:** Pending — agent traces put `emit_shaped` under the old `beta_loop`, but `lowering.rs` was not fully traced this pass.

## Missing Evidence

| Gap                                              | Impact                                                       | How to Obtain                                            |
| ------------------------------------------------ | ----------------------------------------------------------- | ------------------------------------------------------- |
| Per-symbol liveness of `temporal::{LimitSet,LimitKind,AxisSet,BindingConstraint}` in the StreamPlanner config/diag surface | Decides whether `temporal` shrinks to a types crate or stays | Trace `config.rs::to_set` & `binding_report` callers from `init_planner`/`StreamPlanner` |
| Geometry-pipeline parity status for bézier/quadratic/follower | Decides *when* removal is safe                               | Check roadmap / open stories for G5/G2/G3 + follower support in geometry |
| Throughput/optimality: geometry vs TOPP          | CLAUDE.md gate — must not regress trajectory time            | Offline run via klipper-sim on representative slicer G-code |

## Source Code Trace

| Element        | Detail                                                                                   |
| -------------- | ---------------------------------------------------------------------------------------- |
| Live entry     | `bridge.rs:3350` `submit_move` → `StreamPlannerHandle` (bridge.rs:617,3339)               |
| Live solve     | `stream.rs:201-202` `fit_chain` + `plan_velocity_warm_start` (both `geometry`)            |
| Dead old entry | `planner.rs:190` `PlannerHandle`, `planner.rs:620/904` `append_and_replan` — never spawned |
| Old cubic      | `trajectory::beta`/`reparam`/`smooth_fit` (reachable only from `planner.rs`)              |
| Old TOPP       | `temporal::multi::plan_batch` → `topp` SOCP (Clarabel, CL-2024)                           |
| Residual live  | `config.rs`/`binding_report.rs` import `temporal` *types*; `trajectory::ShapedSegment` is the live output type |

## Conclusion

**Confidence:** High (on the production wiring); Medium (on the exact residual-type cut-line and `emit_shaped` liveness).

The user's premise is **Confirmed**: TOPP and the cubic pipeline are still in the tree, but they are **already disconnected from the live print path**. Production runs entirely through `StreamPlannerHandle` → `geometry::plan_velocity_warm_start`. The old `temporal` (TOPP/SOCP) + `trajectory::beta` (cubic reparam) stack survives only behind `planner.rs::PlannerHandle`/`MoveArmThread`, which is never instantiated — it is test/example-only dead code plus a fallback/reference.

So **how are we using TOPP?** In production: we are *not*, except that `config.rs`/`binding_report.rs` still import `temporal`'s limit/binding *value types*. The TOPP *solver* is exercised only by tests and the unspawned old planner.

**Can we remove the old cubic pipeline?** Yes structurally, but **not yet safely**: (1) the new geometry pipeline still rejects G5 bézier, G2/G3 quadratic, and multi-follower moves (bridge.rs:3448/3481/3376); (2) there is no measurement that geometry's trajectory matches/beats TOPP's optimality, which CLAUDE.md makes non-negotiable; (3) `temporal`'s value types and `trajectory::ShapedSegment` must be rehomed before the crates can shrink.

## Recommended Next Steps

### Fix direction

Stage the teardown rather than a single rip-out:
1. **Now (low-risk):** delete the unspawned `planner.rs` `MoveArmThread`/`PlannerHandle` path and its dedicated tests/examples (`streaming_replan.rs`, `follower_lane_e2e.rs`, `z_hop_cold_boot.rs`) — this severs the only live reference to TOPP+cubic, immediately clarifying that they're dead. Verify `config.rs::to_temporal_limits` (only callers are planner.rs:518,982) goes dead and can be deleted with it.
2. **Then (rehome):** migrate `temporal::{LimitSet,LimitKind,AxisSet,BindingConstraint}` usage in `config.rs`/`binding_report.rs` onto `geometry`'s own limit types (or a types-only crate). This is the prerequisite for deleting the `temporal` solver crate.
3. **Gated on parity (Backlog #2,#3):** once `geometry` handles bézier/quadratic/follower moves and a throughput spot-check shows no regression vs TOPP, delete `temporal::{topp,multi}` and `trajectory::{beta,reparam,smooth_fit,plan_velocity,streaming}` (keep `ShapedSegment`).

### Diagnostic

- Run Backlog #1 (per-symbol temporal-type liveness) and resolve Hypothesis 1 (`emit_shaped`/`smooth_fit` liveness) by tracing `lowering.rs` — these two checks fully scope the crate-shrink.
- Confirm via `cargo build -p motion-engine` after step 1 that nothing outside tests referenced the deleted planner.

## Side Findings

- `submit_move` already rejects multiple follower axes (bridge.rs:3376) — the new pipeline is single-follower today; relevant to the follower-parity gate.
- The `temporal` solver carries a known real-time cost (agent cited a throughput roadmap doc, `docs/superpowers/specs/2026-06-14-temporal-solver-throughput-roadmap.md`, ~hundreds of ms worst-case replan) — an additional motivation to retire it once geometry parity lands, but also a reminder the geometry path must be measured, not assumed faster.
- Two of the three parallel research agents initially mis-reported TOPP as "LIVE in production" by tracing upward from `temporal` and assuming `planner.rs` was the entry point — a good reminder that stronghold-from-the-bridge-down beats narrative-from-the-leaf-up.
