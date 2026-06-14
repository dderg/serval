# Observability — Binding-Constraint Reporting Implementation Plan (Plan 6)

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Surface the per-grid binding-constraint data the planner already computes (which `[limit]` row pins the speed where) up through the trajectory API to motion-bridge, where it is aggregated per print and emitted as structured logs — answering "how often does each limit bind?" and "why is the machine slow here?" from `query-logs`.

**Architecture:** `temporal` already fills `VerifyReport.binding_per_grid` (per-sample winning `BindingConstraint`) and computes the worst-pinned sample (`global_worst_idx` / `global_worst_ratio`) inside `check_chain`. This plan: (1) `temporal` tallies a small per-profile `BindingSummary { histogram, worst }` in that same pass and hangs it on `TopProfile` — pure data, no logging deps; (2) `trajectory` merges the per-segment summaries into one `ReplanBindingSummary` per replan batch and carries it on `ReplanReport`; (3) `motion-bridge` resolves limit-set indices to config names, accumulates summaries into a per-window/per-print tally, and emits two host-tracing events (`binding_rollup`, `binding_hist`) on a ~1s cadence. The pure planner (`temporal`/`trajectory`) stays log-free; the decision to emit lives only at the motion-bridge boundary.

**Tech stack:** Rust (`temporal`, `trajectory`, `motion-bridge` crates). Host structured logs via `tracing` → `events/host-rust.jsonl` (the existing `replan_stats` path; **no `log_codes.rs` edits** — that table is the MCU wire protocol, host events use free-form `event=` strings). Tests: `cargo nextest run` from `rust/` (never bare `cargo test`); `cargo test --doc` if doc examples are touched.

**Spec:** `docs/superpowers/specs/2026-06-12-follower-axes-and-limits-design.md` §5 (Observability paragraph) and §6 work-item 6. Builds directly on the `BindingConstraint` groundwork from Plan 3 (`docs/superpowers/plans/2026-06-12-planner-extension-follower-rows.md`).

**Repo rules for every task:** unit tests in separate files from tested code; no explanatory comments — name/extract instead; fail loudly; commit after every task; no Claude/Anthropic commit trailers; `cargo fmt --all --check` before any PR push. All line numbers below are anchors — verify by symbol name / grep, never trust the number.

---

## Design decisions this plan makes (beyond the spec's text)

The spec says only "the planner knows which constraint row binds at every point and reports it through the structured log pipeline … discoverable at the moment someone asks why," and (§6) "Small, rides on 3." Making that executable required five concrete decisions, all confirmed with the user 2026-06-13:

1. **Aggregate richly in the pure solver, emit sparingly at the boundary.** Computing per-sample attribution is near-free (the verify pass already walks every sample). Emitting is the cost. So `temporal` produces a small fixed `BindingSummary` (a histogram of how many grid samples each `BindingConstraint` won, plus the single worst-pinned sample); only that crosses the trajectory→bridge seam, never per-sample data. The firehose is kept out of the API *and* the log.

2. **Two host events, both emitted per ~1s window, both tagged with the live `print_id`.** `binding_rollup` (one line/window) carries the window's worst pin — answers "why slow *here*" with a motion-timeline timestamp. `binding_hist` (one line per non-zero `(constraint, set)` in the window) carries the window's count delta — `query-logs` sums these grouped by `print_id` to answer "how often does X bind?" over a whole print. No fragile cross-print-boundary flush: each window is self-contained, so per-print totals reconstruct by summation. Line rate is flat (~1 + a few per second) regardless of print density.

3. **No `log_codes.rs` entries.** Host-originated structured logs (the existing `replan_stats`, `planner_recv_gap`, `move_arm_drained`) use free-form `event=` strings and do *not* appear in `rust/runtime/src/log_codes.rs` — that table is strictly the MCU wire protocol (numeric subsystem/event resolved host-side). The binding events follow the `replan_stats` precedent exactly.

4. **Location anchor = motion-timeline time, not xyz.** The worst pin is stamped with `state.t_appended` (the planner-clock end of the just-appended window) — a single scalar the planner already holds. `query-logs` and the motion-history service map time→position. Threading xyz (evaluating fitted NURBS per worst sample) is deferred as a purely additive upgrade; per "most bang for the buck at this stage," time correlation satisfies the spec's "slowed here."

5. **Name resolution degrades loud-but-soft, never crashes motion.** Limit-set names live config-side (`limit_sections[i].name`); the trailing runtime-caps set (always appended last by `to_temporal_limits`, present only when runtime caps are set) resolves to `"runtime_caps"` via `names.get(set).unwrap_or("runtime_caps")`. A set index with no name is a config-wiring bug, but crashing the planner thread over an *observability* lookup is worse than the bug, so it falls back to the synthesized label rather than panicking. This is the one deliberate exception to fail-loudly, scoped to the log path.

**Deferred consciously (door open, nothing built):** true per-move / change-triggered transition events (the firehose tier — built only if real prints show the rollup's coarse "where" is insufficient); xyz on the worst pin; live (intra-print) histogram rollups beyond the per-window delta.

---

## File map

- `rust/temporal/src/lib.rs` — `BindingConstraint` gains `Eq`/`Hash`; new `BindingSummary` + `WorstBinding`; `TopProfile.binding` field.
- `rust/temporal/src/topp/verify.rs` — `VerifyReport.binding_summary`; computed in `check_chain`.
- `rust/temporal/src/topp/output.rs` — `assemble` hangs the summary on `TopProfile`.
- `rust/temporal/src/topp/follower/tests.rs` — summary assertions on the scheduling harness.
- `rust/trajectory/src/beta.rs` — `ReplanBindingSummary`/`ReplanWorstBinding`; `aggregate_binding`; threaded through `BetaIterResult` → `PlannedBatch` → `PlanOutput`.
- `rust/trajectory/src/lib.rs` — re-export the two new public types.
- `rust/trajectory/src/streaming/mod.rs` — `ReplanReport.binding` (drop `Copy`).
- `rust/trajectory/src/streaming/state.rs` — wire the summary onto the returned `ReplanReport`.
- `rust/trajectory/tests/binding_report.rs` — new integration test (created).
- `rust/motion-bridge/src/config.rs` — `PlannerConfig::limit_set_names`.
- `rust/motion-bridge/src/binding_report.rs` — new module: `label_binding` + `BindingAccumulator` (created).
- `rust/motion-bridge/src/lib.rs` — `mod binding_report;`.
- `rust/motion-bridge/src/planner.rs` — accumulator wiring in `run_loop` (Move cadence + StreamOpen/Shutdown flush).

---

### Task 1: `temporal` — per-profile `BindingSummary` on `TopProfile`

`check_chain` already produces `binding_per_grid` and tracks `global_worst_ratio`/`global_worst_idx`. Tally a histogram from the final per-grid tags and capture the worst pin; hang both on `TopProfile`. Pure data, no new deps.

**Files:**
- Modify: `rust/temporal/src/lib.rs` (`BindingConstraint` derive; new structs; `TopProfile` field)
- Modify: `rust/temporal/src/topp/verify.rs` (`VerifyReport` field; compute in `check_chain`)
- Modify: `rust/temporal/src/topp/output.rs` (`assemble`)
- Test: `rust/temporal/src/topp/follower/tests.rs`

- [ ] **Step 1: Write the failing test.** Append to `rust/temporal/src/topp/follower/tests.rs`, mirroring the existing `follower_velocity_caps_cruise_speed` harness (straight 100 mm line, generous gantry, follower set on axis 3 with `v_max = 50`, ratio 0.5 → path cruise caps at 100 mm/s; the follower velocity row binds at cruise). Use whatever `schedule_*`/harness helper the sibling tests use to obtain a `TopProfile`:

```rust
#[test]
fn binding_summary_reports_velocity_pin() {
    // SAME setup as follower_velocity_caps_cruise_speed: build the single-segment
    // follower scenario and obtain the solved `TopProfile` (call it `profile`),
    // using the identical harness helper the sibling test uses.
    let profile = /* …harness solve… returns temporal::TopProfile… */;

    let worst = profile
        .binding
        .worst
        .expect("a velocity-capped cruise must produce a worst-pinned sample");
    assert!(
        matches!(worst.constraint, crate::BindingConstraint::Velocity { .. }),
        "worst pin should be a velocity row, got {:?}",
        worst.constraint
    );
    assert!(
        (0.9..=1.05).contains(&worst.ratio),
        "cruise rides the cap; ratio = {}",
        worst.ratio
    );

    let velocity_count: u32 = profile
        .binding
        .histogram
        .iter()
        .filter(|(c, _)| matches!(c, crate::BindingConstraint::Velocity { .. }))
        .map(|(_, n)| *n)
        .sum();
    assert!(velocity_count > 0, "cruise samples should tally as Velocity bindings");
}
```

- [ ] **Step 2: Run to verify it fails** — `cargo nextest run -p temporal -E 'test(binding_summary_reports_velocity_pin)'`
  Expected: FAIL to compile (`profile.binding` field does not exist).

- [ ] **Step 3a: Add types + `Eq`/`Hash` in `rust/temporal/src/lib.rs`.** Change the `BindingConstraint` derive (anchor: `grep -n "pub enum BindingConstraint" rust/temporal/src/lib.rs`) from

```rust
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BindingConstraint {
```
to
```rust
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum BindingConstraint {
```
(every variant carries only `usize` — `Eq`/`Hash` derive cleanly and are needed for `HashMap` keying.)

Add, immediately after the `BindingConstraint` enum:

```rust
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WorstBinding {
    pub constraint: BindingConstraint,
    pub ratio: f64,
    pub grid_index: usize,
    pub s: f64,
}

#[derive(Debug, Clone, Default)]
pub struct BindingSummary {
    pub histogram: Vec<(BindingConstraint, u32)>,
    pub worst: Option<WorstBinding>,
}
```

Add the field to `TopProfile` (anchor: `grep -n "pub struct TopProfile" rust/temporal/src/lib.rs`):

```rust
#[derive(Debug, Clone)]
pub struct TopProfile {
    pub samples: Vec<GridSample>,
    pub status: SolveStatus,
    pub grid_scheme: GridScheme,
    pub total_time: f64,
    pub binding: BindingSummary,
}
```

- [ ] **Step 3b: Compute in `rust/temporal/src/topp/verify.rs`.** Import the new types — change the top `use` (anchor line 3) to:

```rust
use crate::{BindingConstraint, BindingSummary, FollowerDemand, Limits, WorstBinding, restricted_norm};
```

Add the field to `VerifyReport` (anchor: `pub struct VerifyReport`):

```rust
    pub binding_summary: BindingSummary,
```

In the `n == 0` early return inside `check_chain`, add `binding_summary: BindingSummary::default(),` to the constructed `VerifyReport`.

At the end of `check_chain`, just before `let worst_violation = global_worst_ratio - 1.0;`, build the summary from the now-final `binding_per_grid` and worst indices:

```rust
    let mut histogram_map: std::collections::HashMap<BindingConstraint, u32> =
        std::collections::HashMap::new();
    for tag in &binding_per_grid {
        match tag {
            BindingConstraint::None | BindingConstraint::Boundary => {}
            other => *histogram_map.entry(*other).or_insert(0) += 1,
        }
    }
    let histogram: Vec<(BindingConstraint, u32)> = histogram_map.into_iter().collect();

    let worst_tag = binding_per_grid[global_worst_idx];
    let worst = if global_worst_ratio >= SLACK_THRESHOLD
        && !matches!(worst_tag, BindingConstraint::None | BindingConstraint::Boundary)
    {
        Some(WorstBinding {
            constraint: worst_tag,
            ratio: global_worst_ratio,
            grid_index: global_worst_idx,
            s: chain.s[global_worst_idx],
        })
    } else {
        None
    };
    let binding_summary = BindingSummary { histogram, worst };
```

Add `binding_summary,` to the final `VerifyReport { … }` constructor.

- [ ] **Step 3c: Surface it in `rust/temporal/src/topp/output.rs`.** In `assemble`, add the field to the `TopProfile { … }` constructor (anchor: `grep -n "TopProfile {" rust/temporal/src/topp/output.rs`):

```rust
        binding: verify.binding_summary.clone(),
```

- [ ] **Step 3d: Fix any other `TopProfile` constructors.** `grep -rn "TopProfile {" rust/temporal/` — any *test* or helper that builds a `TopProfile` literally needs `binding: BindingSummary::default(),` added (import `crate::BindingSummary` / `temporal::BindingSummary`). `assemble` is the only production constructor.

- [ ] **Step 4: Run to verify it passes** — `cargo nextest run -p temporal`
  Expected: PASS (the new test plus the full suite — zero behavioral change; binding tags and feasibility logic are untouched).

- [ ] **Step 5: Commit**

```bash
git add rust/temporal/src/lib.rs rust/temporal/src/topp/verify.rs rust/temporal/src/topp/output.rs rust/temporal/src/topp/follower/tests.rs
git commit -m "feat(temporal): per-profile binding summary on TopProfile"
```

---

### Task 2: `trajectory` — aggregate per-batch `ReplanBindingSummary` onto `ReplanReport`

Merge the per-segment `BindingSummary`s (one per `TopProfile` in `BatchOutput.profiles`) into one summary per replan, where the profiles are still alive inside `run_one_iteration`, and thread it out through the existing `BetaIterResult → PlannedBatch → PlanOutput → ReplanReport` chain.

**Files:**
- Modify: `rust/trajectory/src/beta.rs`
- Modify: `rust/trajectory/src/lib.rs` (re-export)
- Modify: `rust/trajectory/src/streaming/mod.rs` (`ReplanReport`)
- Modify: `rust/trajectory/src/streaming/state.rs`
- Test: `rust/trajectory/tests/binding_report.rs` (created)

- [ ] **Step 1: Write the failing integration test.** Create `rust/trajectory/tests/binding_report.rs`. Model it on the existing follower integration test (`grep -rln "ReplanReport\|append_and_replan\|ReplanContext" rust/trajectory/tests/` — reuse that file's setup helpers verbatim for building a `ReplanContext` with an extruder follower, a `[limit extruder]` velocity cap tight enough to pin cruise, and feeding a straight extruding move through `append_and_replan`):

```rust
// Reuse the harness from the existing follower integration test:
// build a ShaperState + ReplanContext with axis e (index 3), [limit extruder]
// v_max small enough that the follower velocity row pins cruise, then:
#[test]
fn replan_report_carries_binding_summary() {
    // …harness setup ⇒ `state: ShaperState`, `ctx: ReplanContext`, `mv: CubicSegment`…
    let report = state.append_and_replan(mv, &ctx).expect("replan solves");

    let worst = report
        .binding
        .worst
        .expect("a velocity-pinned extruding move reports a worst binding");
    assert!(matches!(
        worst.constraint,
        temporal::BindingConstraint::Velocity { .. }
            | temporal::BindingConstraint::PaVelocity { .. }
    ));
    assert!(report.binding.histogram.iter().any(|(_, n)| *n > 0));
}
```

- [ ] **Step 2: Run to verify it fails** — `cargo nextest run -p trajectory -E 'test(replan_report_carries_binding_summary)'`
  Expected: FAIL to compile (`report.binding` does not exist).

- [ ] **Step 3a: Define the summary types + aggregator in `rust/trajectory/src/beta.rs`.** Add immediately after the `PlanStats` struct (anchor: `grep -n "pub struct PlanStats" rust/trajectory/src/beta.rs`):

```rust
#[derive(Debug, Clone, Copy)]
pub struct ReplanWorstBinding {
    pub constraint: temporal::BindingConstraint,
    pub ratio: f64,
}

#[derive(Debug, Clone, Default)]
pub struct ReplanBindingSummary {
    pub histogram: Vec<(temporal::BindingConstraint, u32)>,
    pub worst: Option<ReplanWorstBinding>,
}

fn aggregate_binding(profiles: &[temporal::TopProfile]) -> ReplanBindingSummary {
    use std::collections::HashMap;
    let mut hist: HashMap<temporal::BindingConstraint, u32> = HashMap::new();
    let mut worst: Option<ReplanWorstBinding> = None;
    for p in profiles {
        for (c, n) in &p.binding.histogram {
            *hist.entry(*c).or_insert(0) += *n;
        }
        if let Some(w) = &p.binding.worst {
            if worst.is_none_or(|cur| w.ratio > cur.ratio) {
                worst = Some(ReplanWorstBinding {
                    constraint: w.constraint,
                    ratio: w.ratio,
                });
            }
        }
    }
    ReplanBindingSummary {
        histogram: hist.into_iter().collect(),
        worst,
    }
}
```

(`Option::is_none_or` is stable; if clippy's MSRV check flags it, use `worst.map_or(true, |cur| w.ratio > cur.ratio)`.)

- [ ] **Step 3b: Carry it on `BetaIterResult` and populate in `run_one_iteration`.** Add to `struct BetaIterResult` (anchor: `grep -n "struct BetaIterResult" rust/trajectory/src/beta.rs`):

```rust
    binding: ReplanBindingSummary,
```

In `run_one_iteration`, just before the final `Ok(BetaIterResult { … })` (anchor: `grep -n "Ok(BetaIterResult {" rust/trajectory/src/beta.rs`), compute from the still-in-scope `batch_output`:

```rust
    let binding = aggregate_binding(&batch_output.profiles);
```

and add `binding,` to the returned `BetaIterResult { … }`.

- [ ] **Step 3c: Thread through `PlannedBatch` and `PlanOutput`.** Add to `struct PlannedBatch` (anchor: `grep -n "struct PlannedBatch" rust/trajectory/src/beta.rs`):

```rust
    pub binding: ReplanBindingSummary,
```

In `plan_batch_full`, add to the constructed `PlannedBatch { … }`:

```rust
        binding: outcome.result.binding,
```

Add to `struct PlanOutput`:

```rust
    pub binding: ReplanBindingSummary,
```

In `plan_velocity_inner`: the empty-segments early return gets `binding: ReplanBindingSummary::default(),`; the normal return gets `binding: planned.binding,`.

- [ ] **Step 3d: Re-export the public types from `rust/trajectory/src/lib.rs`.** Next to the other `pub use beta::…` lines (`grep -n "pub use beta" rust/trajectory/src/lib.rs`; if none, add one):

```rust
pub use beta::{ReplanBindingSummary, ReplanWorstBinding};
```

- [ ] **Step 3e: Put it on `ReplanReport` in `rust/trajectory/src/streaming/mod.rs`.** Change the derive (drop `Copy` — the summary holds a `Vec`) and add the field:

```rust
#[derive(Debug, Clone)]
pub struct ReplanReport {
    pub split_us: u64,
    pub solve_us: u64,
    pub rebuild_us: u64,
    pub window_segments: usize,
    pub plan: PlanStats,
    pub fallback_rung: u8,
    pub binding: crate::ReplanBindingSummary,
}
```

`grep -rn "ReplanReport" rust/` — fix any site that relied on `Copy` (used the value twice after a move). The production consumer (`motion-bridge` planner) destructures it once; expect zero or trivial fixes.

- [ ] **Step 3f: Wire it in `rust/trajectory/src/streaming/state.rs`.** Change the destructure (anchor: `let (PlanOutput { fitted, stats }, time_offset, fallback_rung) =`) to include `binding`:

```rust
        let (PlanOutput { fitted, stats, binding }, time_offset, fallback_rung) =
```

Add to the returned `ReplanReport { … }` (anchor: `grep -n "Ok(ReplanReport {" rust/trajectory/src/streaming/state.rs`):

```rust
            binding,
```

- [ ] **Step 4: Run to verify it passes** — `cargo nextest run -p trajectory`
  Expected: PASS (new test + full suite; existing behavior unchanged — the summary is additive).

- [ ] **Step 5: Commit**

```bash
git add rust/trajectory/src/beta.rs rust/trajectory/src/lib.rs rust/trajectory/src/streaming/mod.rs rust/trajectory/src/streaming/state.rs rust/trajectory/tests/binding_report.rs
git commit -m "feat(trajectory): aggregate per-batch binding summary onto ReplanReport"
```

---

### Task 3: `motion-bridge` — limit-set names + binding label formatter

Two pure pieces: a config method exposing limit-section names in temporal's set order, and a formatter turning a `BindingConstraint` into structured label fields.

**Files:**
- Modify: `rust/motion-bridge/src/config.rs` (`limit_set_names`)
- Create: `rust/motion-bridge/src/binding_report.rs` (`label_binding` + `BindingLabel`)
- Modify: `rust/motion-bridge/src/lib.rs` (`mod binding_report;`)
- Test: `rust/motion-bridge/src/config/tests.rs`, `rust/motion-bridge/src/binding_report.rs` (`#[cfg(test)]` mod)

- [ ] **Step 1: Write the failing tests.**

In `rust/motion-bridge/src/config/tests.rs` (anchor an existing limit-config test with `grep -n "limit_sections\|LimitSection" rust/motion-bridge/src/config/tests.rs` and reuse its `PlannerConfig` builder):

```rust
#[test]
fn limit_set_names_follow_section_order() {
    // build a PlannerConfig with [limit gantry] (x,y) then [limit extruder] (e),
    // using the same builder the sibling config tests use:
    let cfg = /* …PlannerConfig with sections named "gantry","extruder"… */;
    assert_eq!(cfg.limit_set_names(), vec!["gantry".to_string(), "extruder".to_string()]);
}
```

In a new `#[cfg(test)] mod tests;` at the bottom of `rust/motion-bridge/src/binding_report.rs`, in `rust/motion-bridge/src/binding_report/tests.rs`:

```rust
use super::*;
use temporal::BindingConstraint;

#[test]
fn labels_pa_accel_with_resolved_name() {
    let names = vec!["gantry".to_string(), "extruder".to_string()];
    let label = label_binding(BindingConstraint::PaAccel { set: 1 }, &names).unwrap();
    assert_eq!(label.limit, "extruder");
    assert_eq!(label.derivative, "accel");
    assert!(label.via_pa);
}

#[test]
fn labels_spatial_velocity_without_pa() {
    let names = vec!["gantry".to_string()];
    let label = label_binding(BindingConstraint::Velocity { set: 0 }, &names).unwrap();
    assert_eq!(label.limit, "gantry");
    assert_eq!(label.derivative, "velocity");
    assert!(!label.via_pa);
}

#[test]
fn trailing_set_index_resolves_to_runtime_caps() {
    let names = vec!["gantry".to_string()];
    let label = label_binding(BindingConstraint::AccelNorm { set: 1 }, &names).unwrap();
    assert_eq!(label.limit, "runtime_caps");
}

#[test]
fn none_and_boundary_have_no_label() {
    let names = vec!["gantry".to_string()];
    assert!(label_binding(BindingConstraint::None, &names).is_none());
    assert!(label_binding(BindingConstraint::Boundary, &names).is_none());
}
```

- [ ] **Step 2: Run to verify they fail** — `cargo nextest run -p motion-bridge -E 'test(limit_set_names_follow_section_order) or test(labels_) or test(trailing_set) or test(none_and_boundary)'`
  Expected: FAIL to compile (`limit_set_names` / `binding_report` do not exist).

- [ ] **Step 3a: Add `limit_set_names` in `rust/motion-bridge/src/config.rs`.** Inside `impl PlannerConfig` (the same block as `to_temporal_limits`), add — the order mirrors `to_temporal_limits`'s `sets` push order; the optional trailing runtime-caps set has no section name and is resolved at lookup, so it is intentionally omitted here:

```rust
    pub fn limit_set_names(&self) -> Vec<String> {
        self.limit_sections.iter().map(|s| s.name.clone()).collect()
    }
```

- [ ] **Step 3b: Create `rust/motion-bridge/src/binding_report.rs`** with the formatter (the accumulator is added in Task 4 — this step is the label half only):

```rust
use temporal::BindingConstraint;

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BindingLabel {
    pub limit: String,
    pub derivative: &'static str,
    pub via_pa: bool,
}

pub fn label_binding(c: BindingConstraint, names: &[String]) -> Option<BindingLabel> {
    let (set, derivative, via_pa) = match c {
        BindingConstraint::Velocity { set } => (set, "velocity", false),
        BindingConstraint::AccelNorm { set } => (set, "accel", false),
        BindingConstraint::JerkNorm { set } => (set, "jerk", false),
        BindingConstraint::PaVelocity { set } => (set, "velocity", true),
        BindingConstraint::PaAccel { set } => (set, "accel", true),
        BindingConstraint::PaJerk { set } => (set, "jerk", true),
        _ => return None,
    };
    let limit = names
        .get(set)
        .cloned()
        .unwrap_or_else(|| "runtime_caps".to_string());
    Some(BindingLabel {
        limit,
        derivative,
        via_pa,
    })
}

#[cfg(test)]
mod tests;
```

- [ ] **Step 3c: Register the module** in `rust/motion-bridge/src/lib.rs` next to the other `mod` declarations (`grep -n "^mod \|^pub mod " rust/motion-bridge/src/lib.rs`):

```rust
mod binding_report;
```

- [ ] **Step 4: Run to verify they pass** — `cargo nextest run -p motion-bridge -E 'test(limit_set_names_follow_section_order) or test(labels_) or test(trailing_set) or test(none_and_boundary)'`
  Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add rust/motion-bridge/src/config.rs rust/motion-bridge/src/binding_report.rs rust/motion-bridge/src/binding_report/tests.rs rust/motion-bridge/src/config/tests.rs rust/motion-bridge/src/lib.rs
git commit -m "feat(motion-bridge): limit-set names and binding label formatter"
```

---

### Task 4: `motion-bridge` — `BindingAccumulator` and emit wiring

Accumulate per-replan summaries into a per-window tally, emit `binding_rollup` + `binding_hist` on a ~1s cadence, flush at stream-open and shutdown.

**Files:**
- Modify: `rust/motion-bridge/src/binding_report.rs` (accumulator)
- Modify: `rust/motion-bridge/src/binding_report/tests.rs` (accumulator tests)
- Modify: `rust/motion-bridge/src/planner.rs` (wiring in `run_loop`)

- [ ] **Step 1: Write the failing accumulator tests.** Append to `rust/motion-bridge/src/binding_report/tests.rs`:

```rust
use std::time::{Duration, Instant};
use trajectory::{ReplanBindingSummary, ReplanWorstBinding};

fn summary(set: usize, count: u32, ratio: f64) -> ReplanBindingSummary {
    ReplanBindingSummary {
        histogram: vec![(BindingConstraint::Velocity { set }, count)],
        worst: Some(ReplanWorstBinding {
            constraint: BindingConstraint::Velocity { set },
            ratio,
        }),
    }
}

#[test]
fn record_tallies_window_and_keeps_max_ratio_worst() {
    let t0 = Instant::now();
    let mut acc = BindingAccumulator::new(t0);
    acc.record(&summary(0, 3, 0.8), 1.0);
    acc.record(&summary(0, 2, 0.95), 2.0);
    assert_eq!(acc.window_count(BindingConstraint::Velocity { set: 0 }), 5);
    let (constraint, ratio, t) = acc.worst().unwrap();
    assert_eq!(constraint, BindingConstraint::Velocity { set: 0 });
    assert!((ratio - 0.95).abs() < 1e-12);
    assert!((t - 2.0).abs() < 1e-12);
}

#[test]
fn maybe_rollup_resets_only_after_the_interval() {
    let t0 = Instant::now();
    let names = vec!["gantry".to_string()];
    let mut acc = BindingAccumulator::new(t0);
    acc.record(&summary(0, 1, 0.9), 1.0);

    acc.maybe_rollup(t0 + Duration::from_millis(500), &names);
    assert_eq!(acc.window_count(BindingConstraint::Velocity { set: 0 }), 1);

    acc.maybe_rollup(t0 + Duration::from_millis(1100), &names);
    assert_eq!(acc.window_count(BindingConstraint::Velocity { set: 0 }), 0);
    assert!(acc.worst().is_none());
}

#[test]
fn flush_emits_and_clears_a_partial_window() {
    let t0 = Instant::now();
    let names = vec!["gantry".to_string()];
    let mut acc = BindingAccumulator::new(t0);
    acc.record(&summary(0, 1, 0.9), 1.0);
    acc.flush(t0 + Duration::from_millis(100), &names);
    assert_eq!(acc.window_count(BindingConstraint::Velocity { set: 0 }), 0);
}
```

- [ ] **Step 2: Run to verify they fail** — `cargo nextest run -p motion-bridge -E 'test(record_tallies) or test(maybe_rollup) or test(flush_emits)'`
  Expected: FAIL to compile (`BindingAccumulator` does not exist).

- [ ] **Step 3: Implement the accumulator** in `rust/motion-bridge/src/binding_report.rs`. Add the imports at the top and the struct below `label_binding`:

```rust
use std::collections::HashMap;
use std::time::{Duration, Instant};
use trajectory::ReplanBindingSummary;

pub const ROLLUP_INTERVAL: Duration = Duration::from_secs(1);

pub struct BindingAccumulator {
    window: HashMap<BindingConstraint, u64>,
    window_samples: u64,
    worst: Option<(BindingConstraint, f64, f64)>,
    last_rollup: Instant,
}

impl BindingAccumulator {
    pub fn new(now: Instant) -> Self {
        Self {
            window: HashMap::new(),
            window_samples: 0,
            worst: None,
            last_rollup: now,
        }
    }

    pub fn record(&mut self, summary: &ReplanBindingSummary, t: f64) {
        for (c, n) in &summary.histogram {
            *self.window.entry(*c).or_insert(0) += u64::from(*n);
            self.window_samples += u64::from(*n);
        }
        if let Some(w) = &summary.worst {
            if self.worst.is_none_or(|(_, r, _)| w.ratio > r) {
                self.worst = Some((w.constraint, w.ratio, t));
            }
        }
    }

    pub fn maybe_rollup(&mut self, now: Instant, names: &[String]) {
        if now.duration_since(self.last_rollup) >= ROLLUP_INTERVAL && !self.window.is_empty() {
            self.emit(names);
            self.reset(now);
        }
    }

    pub fn flush(&mut self, now: Instant, names: &[String]) {
        if !self.window.is_empty() {
            self.emit(names);
            self.reset(now);
        }
    }

    fn reset(&mut self, now: Instant) {
        self.window.clear();
        self.window_samples = 0;
        self.worst = None;
        self.last_rollup = now;
    }

    fn emit(&self, names: &[String]) {
        if let Some((c, ratio, t)) = self.worst {
            if let Some(l) = label_binding(c, names) {
                tracing::info!(
                    subsystem = "motion",
                    event = "binding_rollup",
                    limit = %l.limit,
                    derivative = l.derivative,
                    via_pa = l.via_pa,
                    ratio,
                    t,
                    window_samples = self.window_samples,
                    "binding rollup"
                );
            }
        }
        for (c, count) in &self.window {
            if let Some(l) = label_binding(*c, names) {
                tracing::info!(
                    subsystem = "motion",
                    event = "binding_hist",
                    limit = %l.limit,
                    derivative = l.derivative,
                    via_pa = l.via_pa,
                    count = *count,
                    "binding histogram"
                );
            }
        }
    }

    #[cfg(test)]
    pub fn window_count(&self, c: BindingConstraint) -> u64 {
        self.window.get(&c).copied().unwrap_or(0)
    }

    #[cfg(test)]
    pub fn worst(&self) -> Option<(BindingConstraint, f64, f64)> {
        self.worst
    }
}
```

(If clippy's MSRV lint rejects `Option::is_none_or`, swap to `self.worst.map_or(true, |(_, r, _)| w.ratio > r)`.)

- [ ] **Step 4: Run to verify they pass** — `cargo nextest run -p motion-bridge -E 'test(record_tallies) or test(maybe_rollup) or test(flush_emits)'`
  Expected: PASS.

- [ ] **Step 5: Wire into `rust/motion-bridge/src/planner.rs` `run_loop`.** After `let mut thread_state = PlannerThreadState::build(&config);` (anchor line ~456), add the names table and accumulator (both live for the loop's lifetime; `limit_sections` is config-static, so the names need no rebuild):

```rust
    let limit_names = config.limit_set_names();
    let mut binding_acc = crate::binding_report::BindingAccumulator::new(Instant::now());
```

In the `PlannerMsg::Move(m)` arm, the existing `replan_stats` block destructures `report`. Extend that destructure (anchor: `let ReplanReport {`) to bind `binding`:

```rust
                let ReplanReport {
                    split_us,
                    solve_us,
                    rebuild_us,
                    window_segments,
                    plan,
                    fallback_rung,
                    binding,
                } = report;
```

Immediately after the existing `replan_stats` `tracing::debug!(…)` call in that arm, add:

```rust
                binding_acc.record(&binding, state.t_appended);
                binding_acc.maybe_rollup(Instant::now(), &limit_names);
```

In the `PlannerMsg::KalicoStreamOpen { home_pos }` arm (anchor line ~712), **before** the existing state reset, flush the finished print's partial window:

```rust
                binding_acc.flush(Instant::now(), &limit_names);
```

In the `PlannerMsg::Shutdown` arm (anchor: `PlannerMsg::Shutdown => return,`), flush before returning:

```rust
            PlannerMsg::Shutdown => {
                binding_acc.flush(Instant::now(), &limit_names);
                return;
            }
```

(`Instant` is already imported in `planner.rs`.)

- [ ] **Step 6: Run** — `cargo nextest run -p motion-bridge`
  Expected: PASS.

- [ ] **Step 7: Commit**

```bash
git add rust/motion-bridge/src/binding_report.rs rust/motion-bridge/src/binding_report/tests.rs rust/motion-bridge/src/planner.rs
git commit -m "feat(motion-bridge): accumulate and emit binding-constraint rollups"
```

---

### Task 5: End-to-end verification, sim sanity, and gate

- [ ] **Step 1: Purity guard.** Confirm no logging dependency leaked into the pure crates:
  `grep -rn "tracing\|kalico_log\|klog" rust/temporal/src/ rust/trajectory/src/` → expect zero hits (binding data is plain returned structs; emission lives only in motion-bridge).
  `grep -rn "static\|lazy_static\|OnceLock" rust/temporal/src/ | grep -v test` → no new globals.

- [ ] **Step 2: Full workspace suite** — `cargo nextest run` from `rust/` → PASS. `cargo test --doc` if any doc example was touched. If `klippy/` was touched (it must NOT be in this plan — confirm `git status`), also run `./scripts/ci.sh py`.

- [ ] **Step 3: Gate** — `./scripts/ci.sh quick` fully green (ruff, rust workspace tests, clippy `-D warnings`, `cargo fmt --check`, watchdog canary). Re-run `cargo fmt --all --check` last, after any late edit.

- [ ] **Step 4: kalico-sim sanity (manual verification).** Use the `kalico-sim` skill to run a migrated fixture (an extruding print with `[axis e]` + a tight `[limit extruder]`) through the host pipeline, then use the `query-logs` skill to confirm the events land and aggregate:
  - `subsystem:=motion event:=binding_rollup` lines appear during the print, each with `limit`, `derivative`, `via_pa`, `ratio`, `t`, `window_samples`. (The event field is the bare name `binding_rollup`; `motion` is the separate `subsystem` field, not a name prefix — matches the `replan_stats` precedent.)
  - `event:=binding_hist | stats by (limit, derivative) sum(count) as total` returns a non-empty per-limit breakdown.
  - A travel-only (non-extruding) fixture still emits spatial rollups (`limit` = the gantry/z section names) and no follower entries.
  Record the LogsQL queries used in the commit message so the observability surface is reproducible.

- [ ] **Step 5: Commit**

```bash
git commit --allow-empty -m "test: binding-constraint observability verified end-to-end (plan 6)"
```

---

## Self-review notes (spec → plan coverage)

- §5 "the planner knows which constraint row binds at every point and reports it through the structured log pipeline" → Task 1 computes the per-profile summary from `binding_per_grid`; Tasks 2–4 carry and emit it. ✓
- §5 example "slowed here by `[limit extruder]` accel via the PA post-processor" → `label_binding` produces `limit=extruder, derivative=accel, via_pa=true` (Task 3); `binding_rollup` carries the motion-timeline `t` for "here". ✓
- §6 "Binding-constraint reporting via structured logs. Small, rides on 3." → no solver changes; reuses Plan 3's `BindingConstraint`; four small tasks. ✓
- Plan 3 decision-3 use case "show whether the PA-jerk row ever binds at corners on real prints" → `binding_hist` with `BindingConstraint::PaJerk` tallies; `query-logs … event:=binding_hist | stats by (derivative, via_pa) sum(count) as total` answers it. ✓
- Planner stays a pure function / oracle API → Task 5 Step 1 guards it; `temporal`/`trajectory` gain no logging deps. ✓
- Fail loudly, with the one scoped exception → name resolution degrades to `"runtime_caps"`/section-order rather than crashing the planner thread over a log lookup (decision 5, documented). ✓
- No placeholders: every code step shows complete code; test bodies that depend on existing harness helpers point at the exact sibling test and the grep to find it. ✓

**Known-approximation register (each conscious, none silent):**
1. Worst-pin time anchor is `state.t_appended` (window-end planner clock), not the exact sample time — coarse but correlatable; xyz deferred (decision 4).
2. A print's final sub-second window may be flushed at the next `KalicoStreamOpen` and so tagged with the next print's `print_id`; negligible for "how-often" aggregates, and `binding_rollup`/`binding_hist` during the print carry the correct live `print_id` (decision 2).
3. `binding_hist` line count per window scales with the number of distinct `(constraint, set)` pins (typically <10 on a real machine), not with move count — flat host cost (decision 1).
