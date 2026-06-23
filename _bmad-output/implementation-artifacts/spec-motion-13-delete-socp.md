---
title: 'Motion-13: delete the dead SOCP — temporal crate, trajectory SOCP modules, old planner.rs'
type: 'chore'
created: '2026-06-20'
status: 'draft'
baseline_commit: '9ebf70d7afb9cc54f82195b539fba7535f94125f'
context:
  - '{project-root}/CLAUDE.md'
  - '{project-root}/_bmad-output/implementation-artifacts/spec-motion-11-pipeline-production-cutover.md'
  - '{project-root}/_bmad-output/implementation-artifacts/spec-motion-12-tangential-jerk-c2-continuity.md'
---

> **Split note (2026-06-20):** This spec was T5 of Motion-12. It is now its own change because the deletion is independent of the C2 build (the SOCP is already off the live path) and bundling a large deletion into the C2 crux PR makes a deletion fault indistinguishable from a C2 math bug at bisect.

<frozen-after-approval reason="human-owned intent — do not modify unless human renegotiates">

## Intent

**Problem:** The sampled Consolini-Locatelli coupled-jerk SOCP (TOPP) and the old `PlannerHandle` are **dead on the `curvature-profile` branch** — the live geometry/stream path (`stream_planner.rs` / `bridge.rs`) never calls them — but they still sit in the tree: the whole `rust/temporal/` crate, `trajectory/src/{beta,plan_velocity,reparam,utilization}.rs`, the old `trajectory::streaming` `ShaperState`, and `motion-engine/src/planner.rs`. They carry maintenance weight, compile time, and the standing risk that someone re-wires them.

**Approach:** Delete them, and **use the compiler as the source of truth.** Remove `temporal` from the workspace, then fix every breakage by **deleting the now-orphaned consumer** (the old planner + its tests, the SOCP-era binding-report / limit-config surface) rather than migrating it. Only genuinely-live *shared* types get migrated. "Rip it out and see what screams" is the method on purpose: it **proves** nothing live depended on it — which prose review cannot. A symbol that turns out to have a live (`stream_planner`/`bridge`) consumer means it was *not* dead — HALT and surface that, do not migrate it on a whim.

## Boundaries & Constraints

**Always:** Keep the live-path tests green (`pump_loop`, `runtime_caps`, follower-row emit, logging/MCU; `binding_report_e2e` **only if** its types survive — see the binding-report decision below). Keep `trajectory::ShapedSegment` + the emit/shaping modules the live path needs (`emit_shaped`, `fit`, `kernel`, `shaper`, `smooth_fit`, `pad`, `odometer`, `post_processor`, `parallel`). Migrate genuinely-shared dispatch types out of `planner.rs` **before** deleting it. `./scripts/ci.sh quick` green (clippy `-D warnings`, fmt) is the gate.

**Ask First:** If any symbol slated for deletion has a **live** `stream_planner.rs`/`bridge.rs` consumer, HALT — it isn't dead; surface it rather than deleting or migrating reflexively.

**Never:** Re-introduce the sampled Consolini-Locatelli coupled-jerk SOCP, in production or as a kept oracle (architecture.md: "the coupled-jerk SOCP we are avoiding"; user: deleted, untrusted).

</frozen-after-approval>

## Dead-code map — what's dead, and how we know

Traced 2026-06-20 (call-graph grep, not prose):

- **`to_temporal_limits` / `LimitSet` / `Limits` consumers:** `planner.rs:518,982` (deleted here), `motion-engine/tests/follower_lane_e2e.rs:273` (deleted here), `config/tests.rs`. **No `stream_planner`/`bridge` caller.**
- **`BindingConstraint` / `label_binding` / `BindingReport` consumers:** `planner.rs:514,717` (deleted here), `motion-engine/tests/binding_report_e2e.rs`. **No live caller.**
- **Therefore:** once `planner.rs` and the old tests go, `temporal::limits`, `BindingConstraint`, `config.rs::{to_temporal_limits,to_set}`, and `binding_report.rs` are all orphaned. Winston's round-1 finding (that these symbols are "live" because they are textually referenced) was a false alarm: the referencing code is itself in this deletion set.

**Binding-report decision point.** The original Motion-12 KEEP list kept `binding_report.rs` + `binding_report_e2e` — but their only consumers are the deleted old planner + that one test. Two options; pick one explicitly (the compiler will force the issue once `temporal` is gone):
- **(a) Default — delete.** Remove `binding_report.rs` + `binding_report_e2e` + `config.rs::{to_temporal_limits,to_set}` as part of the dead SOCP surface. Nothing in the new geometry planner produces binding reports today.
- **(b) Keep diagnostics — migrate.** If "which limit is binding (velocity/accel/jerk)" diagnostics are wanted in the new geometry/stream planner, migrate `BindingConstraint` + `LimitKind` (+ `LimitSet` if needed) into a **surviving** crate (e.g. `geometry`, or a small new `limits` crate) and re-wire `binding_report` onto the live path. This is **net-new work, out of scope here** — flag it, do not smuggle it in.

Either way: the KEEP list now lives **here**, not in Motion-12 (the split moved the whole deletion scope out of Motion-12). `binding_report_e2e`'s fate is decided in this spec, not assumed kept.

## Tasks & Acceptance

**One PR.** Suggested commit order TD1 → TD2 → TD3 → TD4, each bisect-clean (`cargo build` succeeds at every commit boundary).

**TD1 — migrate shared dispatch types, delete old planner**
- [ ] Migrate `DispatchError` / `HomeDripParams` / `NudgeParams` out of `planner.rs` (consumed by `stream_planner.rs`/`bridge.rs`) into `stream_planner.rs` or a new `dispatch`-adjacent module.
- [ ] Delete `motion-engine/src/planner.rs` + `planner/tests.rs`; drop `pub mod planner;` from `motion-engine/src/lib.rs`.
- AC-D4: `grep -rn "DispatchError\|HomeDripParams\|NudgeParams" rust/motion-engine/src/planner.rs` returns nothing (file deleted / types fully migrated, not re-exported from a deleted home).

**TD2 — delete trajectory SOCP modules**
- [ ] Delete `trajectory/src/{beta,plan_velocity,reparam,utilization}.rs` (+ their `*/tests.rs`) and the old `trajectory::streaming` `ShaperState` + temporal-typed `shape_batch`/`ShapeError` surface in `trajectory/src/lib.rs`.
- [ ] Keep `trajectory::ShapedSegment` + the emit/shaping modules the live path needs.

**TD3 — delete the temporal crate**
- [ ] Delete the whole `rust/temporal/` crate; remove it from the workspace members and from `trajectory`/`motion-engine` `Cargo.toml`.

**TD4 — resolve orphans + delete old-path tests/examples**
- [ ] Resolve the `binding_report` / limit-config orphans per the **binding-report decision** above (default: delete `binding_report.rs` + `binding_report_e2e` + `config.rs::{to_temporal_limits,to_set}`; or migrate `BindingConstraint`/`LimitKind` if diagnostics are wanted).
- [ ] Delete old-path tests/examples: `motion-engine/{examples/plan_gcode.rs, examples/piece_stream_diff.rs, tests/streaming_replan.rs, tests/follower_lane_e2e.rs, tests/z_hop_cold_boot.rs}`, `trajectory/tests/*` SOCP/old-planner repros, all `temporal/tests` + `temporal/examples`.
- [ ] Keep live-path tests (`pump_loop`, `runtime_caps`, follower-row emit, logging/MCU; `binding_report_e2e` only under option (b)).

**Acceptance Criteria (spec-level):**
- AC-D1: workspace builds with `temporal` gone; `./scripts/ci.sh quick` green (clippy `-D warnings`, fmt). `cargo nextest run` green.
- AC-D2: live-path behavior unchanged — existing live-path tests green; if Motion-12's T4 feasibility gate is merged, it stays green (deletion changed nothing observable on the live path).
- AC-D3: `grep -rn "temporal\|PlannerHandle\|plan_velocity_inner" rust/*/src` shows zero references. This is now **honest** — the limit/binding symbols were deleted or migrated to a surviving crate, not whitelisted to satisfy the grep.
- AC-D4: as in TD1.

## Relationship to Motion-12

- **Independent.** The SOCP is already off the live path, so this can land **before or after** Motion-12. Landing it **first** shrinks the tree before the C2 crux and removes the bisect ambiguity (a deletion fault masquerading as a C2 math bug). **Never bundle** this into the same PR as the Motion-12 T3 crux.
- The original Motion-12 "Ask First" coupling ("remove the old path only after the C2 feasibility gate is green and merged") is **relaxed** by the split: because the deletion is provably dead-code only (this spec's method *proves* it), it does not depend on the C2 gate. If a residual live dependency surfaces, that is exactly the HALT condition in Boundaries.

## Verification

**Commands:**
- `./scripts/ci.sh quick` — ruff/clippy `-D warnings`/fmt/rust tests green.
- `cargo nextest run` — full Rust suite green with `temporal` gone.
- `grep -rn "temporal\|PlannerHandle\|plan_velocity_inner" rust/*/src` — zero references (AC-D3).
- `grep -rn "DispatchError\|HomeDripParams\|NudgeParams" rust/motion-engine/src/planner.rs` — empty (AC-D4).
