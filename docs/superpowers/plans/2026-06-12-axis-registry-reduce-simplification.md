# Axis Registry & Reduce Simplification Implementation Plan (Plan 2)

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make axes config-declared objects (`[axis <name>]` with `follows`/`motors`), replace the E-mode zoo with one follower rule (ratio = delta / 3D path length), and delete every E-special code path — leaving loud errors where plans 3–4 will land.

**Architecture:** Spec: `docs/superpowers/specs/2026-06-12-follower-axes-and-limits-design.md` §1–§2. A `CubicSegment` carries `followers: Vec<FollowerDemand { axis_index, ratio }>` instead of `e_mode`/`extrusion_per_xy_mm`/`e_independent`. The reduce boundary computes per-follower deltas from a config-derived word list (`FollowerWord { letter, axis_index }`) against a per-follower nominal ledger; ratio uses **3D** arc length, so vase mode and hop-retracts become ordinary moves. `e_independent.rs` (trapezoid scheduler), `ELimits`, and `partition.rs` (E-gap machinery) are deleted with nothing replacing them: follower-only moves are a fatal reduce error until plan 3; the live-path `ExtrusionNotSupported` rejection stays until plan 4. The axis registry lives in `motion-bridge/src/config.rs` next to plan 1's `LimitSection`; klippy gains `[axis]` sections and rejects `[firmware_retraction]`.

**Tech stack:** Rust (nurbs/geometry/trajectory/motion-bridge), PyO3 bridge, klippy Python. Tests: `cargo nextest run` from `rust/` (never bare `cargo test`).

**PRECONDITION: Plan 1 (`docs/superpowers/plans/2026-06-12-limits-rework.md`) must be fully landed** — this plan builds on `temporal::Limits` as `LimitSet` collections, `LimitSection`/`axis_index` in motion-bridge config, and the plan-1 `init_planner` signature. Verify before starting: `git log --oneline | head -20` shows plan 1's final commit (`feat: unified [limit] sections end-to-end`) and `cargo nextest run` passes from `rust/`.

**Line numbers in this plan are pre-plan-1 approximations — always anchor by symbol name and the given grep commands, never by line.**

**Out of scope (later plans):** planner constraint rows for follower axes (plan 3 — follower-covering `[limit]` sections are validated but produce no temporal rows here, which is safe because nothing moves a follower axis after this plan), any E motion/emission/PA (plan 4 — `classify.rs` keeps rejecting live extrusion), kinematics modules and motor-role mapping (plan 5 — `motors:` keys are parsed and stored, not consumed).

**Repo rules for every task:** unit tests in separate files from tested code; no explanatory comments — name/extract instead; fail loudly; commit after every task; no Claude/Anthropic commit trailers; `cargo fmt --all --check` before any PR push.

---

### Task 1: 3D path arc length in nurbs

`xy_arc_length` ignores Z; the follower ratio needs full 3D path length. Add `path_arc_length` beside it (same adaptive-subdivision algorithm, Z included). `xy_arc_length` itself dies in Task 2 when its last consumer swaps.

**Files:**
- Modify: `rust/nurbs/src/arc_length.rs` (add fn next to `xy_arc_length`, find with `grep -n "pub fn xy_arc_length" rust/nurbs/src/arc_length.rs`)
- Test: wherever `xy_arc_length`'s existing tests live — `grep -rn "xy_arc_length" rust/nurbs/` — add the new tests beside them (separate test file/module from the implementation)

- [ ] **Step 1: Write failing tests**

```rust
#[test]
fn path_arc_length_matches_xy_for_planar_curve() {
    let xyz = VectorNurbs::<f64, 3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![
            [0.0, 0.0, 0.0],
            [10.0, 0.0, 0.0],
            [20.0, 10.0, 0.0],
            [30.0, 10.0, 0.0],
        ],
    )
    .unwrap();
    let xy = xy_arc_length(&xyz);
    let full = path_arc_length(&xyz);
    assert!((xy - full).abs() < 1e-9);
}

#[test]
fn path_arc_length_includes_z_component() {
    let xyz = VectorNurbs::<f64, 3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
            [0.0, 0.0, 2.0],
            [0.0, 0.0, 3.0],
        ],
    )
    .unwrap();
    assert!((path_arc_length(&xyz) - 3.0).abs() < 1e-9);
    assert!(xy_arc_length(&xyz).abs() < 1e-9);
}

#[test]
fn path_arc_length_diagonal_line_exact() {
    let xyz = VectorNurbs::<f64, 3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![
            [0.0, 0.0, 0.0],
            [1.0, 1.0, 1.0],
            [2.0, 2.0, 2.0],
            [3.0, 3.0, 3.0],
        ],
    )
    .unwrap();
    assert!((path_arc_length(&xyz) - 3.0 * 3.0_f64.sqrt()).abs() < 1e-9);
}
```

- [ ] **Step 2: Run to verify failure** — `cargo nextest run -p nurbs -E 'test(path_arc_length)'` → FAIL (undefined function)

- [ ] **Step 3: Implement** — copy `xy_arc_length`'s body verbatim, rename, and extend the speed integrand to three components:

```rust
#[cfg(feature = "host")]
#[must_use]
pub fn path_arc_length(xyz: &crate::VectorNurbs<f64, 3>) -> f64 {
    let knots = xyz.knots();
    let u_start = knots[0];
    let u_end = knots[knots.len() - 1];

    let deriv = vector_derivative(xyz);

    let speed = |u: f64| -> f64 {
        let d = vector_eval(&deriv, u);
        (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt()
    };

    let span = u_end - u_start;
    let mut prev_estimate: Option<f64> = None;
    let mut subintervals: usize = 1;

    loop {
        let mut sum = 0.0_f64;
        for i in 0..subintervals {
            let a = u_start + span * (i as f64) / (subintervals as f64);
            let b = u_start + span * ((i + 1) as f64) / (subintervals as f64);
            sum += integrate_arc_length(speed, a, b, 5);
        }

        if let Some(prev) = prev_estimate {
            let tol = 1e-9 * sum.abs().max(1e-300);
            if (sum - prev).abs() < tol {
                return sum;
            }
        }

        if subintervals >= 64 {
            return sum;
        }

        prev_estimate = Some(sum);
        subintervals *= 2;
    }
}
```

(Fixed `D = 3`, no const-generic bound — the only caller passes `VectorNurbs<f64, 3>`.)

- [ ] **Step 4: Run** — `cargo nextest run -p nurbs -E 'test(path_arc_length)'` → PASS
- [ ] **Step 5: Commit** — `feat(nurbs): 3D path_arc_length`

---

### Task 2: geometry crate — followers replace the E-mode zoo

The fat single-crate task: `CubicSegment` swaps `e_mode`/`extrusion_per_xy_mm`/`e_independent` for `followers`, reduce generalizes the E ledger to a per-follower-word ledger, classification becomes one rule, the helical error dies. Intermediate steps won't compile; the crate must build and its suite pass at the end.

**Files:**
- Modify: `rust/geometry/src/segment.rs` (struct + `try_new` + delete `EMode`)
- Modify: `rust/geometry/src/lib.rs` (exports: add `FollowerWord`, `FollowerDemand`; drop `EMode`)
- Modify: `rust/geometry/src/reduce.rs` (`ModalState`, `ReduceEvent::Curve`, G5/G5.1/G92 sites, drop dead `e_delta_mm` marker field)
- Modify: `rust/geometry/src/pipeline.rs` (`GeometryPipeline::new` signature, `handle_curve`, replace `classify_e_mode`)
- Modify: `rust/geometry/src/error.rs` (error variants)
- Modify: `rust/geometry/src/splitter.rs` (carry followers through splits)
- Modify: `rust/geometry/src/telemetry.rs` (only if the `e_delta_mm` deletion orphans a field — follow the compiler)
- Tests: `rust/geometry/src/segment/tests.rs`, `rust/geometry/src/pipeline/tests.rs`, `rust/geometry/tests/*.rs` — find all with `grep -rln "EMode\|e_mode\|extrusion_per_xy_mm\|e_independent" rust/geometry/`

- [ ] **Step 1: New types and segment swap.** In `segment.rs`, delete `pub enum EMode` and replace the `CubicSegment` E fields:

```rust
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FollowerDemand {
    pub axis_index: usize,
    pub ratio: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub struct CubicSegment {
    pub xyz: VectorNurbs<f64, 3>,
    pub followers: Vec<FollowerDemand>,
    pub feedrate_mm_s: f64,
    pub source: SourceRange,
    pub split_info: Option<SplitInfo>,
}
```

`try_new` keeps every cubic/finite/feedrate check verbatim, deletes the whole `match e_mode` block, and validates followers instead:

```rust
pub fn try_new(
    xyz: VectorNurbs<f64, 3>,
    followers: Vec<FollowerDemand>,
    feedrate_mm_s: f64,
    source: SourceRange,
    split_info: Option<SplitInfo>,
) -> Result<Self, crate::GeometryError> {
    // ... existing degree / control-point / knot / finite checks unchanged ...
    for (i, f) in followers.iter().enumerate() {
        if !f.ratio.is_finite() || f.ratio == 0.0 {
            return Err(crate::GeometryError::FollowerInvariantViolation {
                reason: "follower ratio must be finite and nonzero",
            });
        }
        if followers[..i].iter().any(|p| p.axis_index == f.axis_index) {
            return Err(crate::GeometryError::FollowerInvariantViolation {
                reason: "duplicate follower axis",
            });
        }
    }
    // ... feedrate check unchanged ...
    Ok(Self { xyz, followers, feedrate_mm_s, source, split_info })
}
```

A travel move is `followers: vec![]` — there is no mode.

- [ ] **Step 2: Errors.** In `error.rs` (find variants with `grep -n "HelicalExtrusionUnsupported\|EModeInvariantViolation\|ZeroMotion" rust/geometry/src/error.rs`): delete `HelicalExtrusionUnsupported` and `EModeInvariantViolation`; add `FollowerInvariantViolation { reason: &'static str }` and `FollowerOnlyMoveUnsupported`. In the `Fatal` enum (`grep -n "Fatal" rust/geometry/src/error.rs rust/geometry/src/lib.rs`): delete `Fatal::HelicalExtrusionUnsupported`, add `Fatal::FollowerOnlyMoveUnsupported { line_no: u32 }`. Error text must say what and when: `"follower-only move (no spatial displacement) is not yet plannable"`.

- [ ] **Step 3: Reduce — per-follower ledger.** In `lib.rs` add:

```rust
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FollowerWord {
    pub letter: u8,
    pub axis_index: usize,
}
```

In `reduce.rs`:
- `ModalState`: delete `e: f64`; add `follower_ledger: Vec<f64>`; `ModalState::new(n_followers: usize)` initializes `vec![0.0; n_followers]`.
- `reduce()` and `ReduceIter` gain `followers: &'a [FollowerWord]` (threaded from the pipeline).
- `ReduceEvent::Curve`: replace `e_delta: Option<f64>` with `follower_deltas: Vec<(usize, f64)>` (axis_index, delta).
- Both the G5 and G5.1 emit sites replace the `params.e()` block with:

```rust
let follower_deltas: Vec<(usize, f64)> = followers
    .iter()
    .enumerate()
    .filter_map(|(k, fw)| {
        params.get(fw.letter).map(|new_pos| {
            let d = new_pos - state.follower_ledger[k];
            state.follower_ledger[k] = new_pos;
            (fw.axis_index, d)
        })
    })
    .collect();
```

- The G92 site replaces `if let Some(e) = params.e() { state.e = e; }` with:

```rust
for (k, fw) in followers.iter().enumerate() {
    if let Some(pos) = params.get(fw.letter) {
        state.follower_ledger[k] = pos;
    }
}
```

- Delete `e_delta_mm: Option<f64>` from `ReduceEvent::Marker` — it is `None` at every emit site (verify: `grep -n "e_delta_mm" rust/geometry/src/reduce.rs`), so the consuming arm in `pipeline.rs` (`grep -n "e_delta_mm" rust/geometry/src/pipeline.rs`) is dead; delete that arm and, if the compiler shows the `telemetry.rs` field now unconstructed, delete it and its consumers too.

- [ ] **Step 4: Pipeline — one classification rule.** `GeometryPipeline::new(params: FitterParams, followers: Vec<FollowerWord>)`; `process()` threads `&self.followers` into `reduce`. Replace `classify_e_mode` and `build_linear_e_curve` entirely with:

```rust
fn classify_followers(
    xyz: &nurbs::VectorNurbs<f64, 3>,
    follower_deltas: &[(usize, f64)],
) -> Result<Vec<FollowerDemand>, GeometryError> {
    const EPS_PATH: f64 = 1e-6;
    const EPS_FOLLOWER: f64 = 1e-6;
    let path_len = nurbs::arc_length::path_arc_length(xyz);
    let any_follower_motion = follower_deltas
        .iter()
        .any(|&(_, d)| d.abs() > EPS_FOLLOWER);
    if path_len <= EPS_PATH {
        if any_follower_motion {
            return Err(GeometryError::FollowerOnlyMoveUnsupported);
        }
        return Err(GeometryError::ZeroMotion);
    }
    Ok(follower_deltas
        .iter()
        .filter(|&&(_, d)| d.abs() > EPS_FOLLOWER)
        .map(|&(axis_index, d)| FollowerDemand {
            axis_index,
            ratio: d / path_len,
        })
        .collect())
}
```

`handle_curve` becomes: `classify_followers(...)` → `CubicSegment::try_new(xyz, followers, feedrate_mm_s, source, None)`; `Err(ZeroMotion)` → skip (as today); `Err(FollowerOnlyMoveUnsupported)` → `Item::Fatal(Fatal::FollowerOnlyMoveUnsupported { line_no })`. The `HelicalExtrusionUnsupported` arm is deleted — Z+follower is now an ordinary move. The last `xy_arc_length` consumer is gone: delete `xy_arc_length` from `rust/nurbs/src/arc_length.rs` and its tests (`grep -rn "xy_arc_length" rust/` must return only this plan).

- [ ] **Step 5: Splitter.** In `splitter.rs`: delete the `EMode::Independent` early-return (`grep -n "Independent" rust/geometry/src/splitter.rs`); at the segment-reconstruction call (`grep -n "try_new" rust/geometry/src/splitter.rs`), pass `segment.followers.clone()` — the ratio is per-mm-of-path, so it is split-invariant by construction; no recomputation.

- [ ] **Step 6: Port geometry tests.** `grep -rln "EMode\|e_mode\|extrusion_per_xy_mm\|e_independent\|Helical" rust/geometry/` — mechanical mapping: `EMode::Travel, 0.0, None` args → `vec![]`; `EMode::CoupledToXy, r, None` → `vec![FollowerDemand { axis_index: 3, ratio: r }]`; tests asserting `HelicalExtrusionUnsupported` flip to asserting success with the correct 3D-length ratio; tests of `Independent` classification flip to asserting `Fatal::FollowerOnlyMoveUnsupported`. Add new pipeline tests (in `pipeline/tests.rs`, using a pipeline constructed with `vec![FollowerWord { letter: b'E', axis_index: 3 }]`):

```rust
#[test]
fn vase_mode_helix_classifies_with_3d_ratio() {
    // Previously Fatal::HelicalExtrusionUnsupported.
    let gcode = "G5 X10 Y0 Z0.3 I3 J0 P-3 Q0 E0.5 F3000\n";
    // Run through a pipeline built with vec![FollowerWord { letter: b'E', axis_index: 3 }];
    // collect Items; assert exactly one Item::Segment(Segment::Cubic(seg));
    // assert seg.followers.len() == 1, axis_index == 3,
    // and (seg.followers[0].ratio - 0.5 / nurbs::arc_length::path_arc_length(&seg.xyz)).abs() < 1e-12.
}

#[test]
fn z_hop_with_follower_classifies() {
    let gcode = "G5 X0 Y0 Z2.0 I0.1 J0 P-0.1 Q0 E-3.2 F3000\n";
    // Assert one cubic segment with followers == [FollowerDemand { axis_index: 3, ratio: -3.2 / path_len }].
}

#[test]
fn follower_only_move_is_fatal() {
    let gcode = "G5 X0 Y0 I0.1 J0 P-0.1 Q0 E-3.2 F3000\n";
    // Wait — I/J displacement still bends the curve; use a truly zero-length curve:
    // start at origin, end at origin, degenerate tangents are rejected by G5 parsing,
    // so drive this through TWO lines: a real move to X10, then "G5 X10 Y0 I1 J0 P-1 Q0 E5 F3000"
    // (endpoint == current position, zero net displacement, zero path length).
    // Assert the second item is Item::Fatal(Fatal::FollowerOnlyMoveUnsupported { .. }).
}

#[test]
fn absolute_word_ledger_survives_g92() {
    let gcode = "G5 X10 Y0 I3 J0 P-3 Q0 E10 F3000\nG92 E0\nG5 X20 Y0 I3 J0 P-3 Q0 E10 F3000\n";
    // Both curves must classify with the SAME positive ratio (each saw delta 10.0).
}
```

(Flesh out the bodies following the existing harness idioms in `pipeline/tests.rs` — pipeline construction, `process`, item collection are already demonstrated there; the G-code strings and assertions above are the substance. For `follower_only_move_is_fatal`, if the zero-length G5 trips a different pre-existing parse guard, surface it — do not weaken the guard to make the test pass.)

Also port the out-of-crate `GeometryPipeline::new` callers now (they compile against geometry directly): `rust/temporal/tests/adaptive_tolerance.rs` and `rust/temporal/tests/prototype.rs` — find with `grep -rln "GeometryPipeline::new" rust/ | grep -v geometry/src` — pass `vec![]` (no followers) unless the test exercises extrusion.

- [ ] **Step 7: Run** — `cargo nextest run -p geometry -p nurbs` → PASS
- [ ] **Step 8: Commit** — `feat(geometry): followers replace E modes; 3D path ratio; follower-only moves fatal`

---

### Task 3: trajectory crate — delete the E machinery

Deletion-shaped: the trapezoid scheduler, `ELimits`, and the E-gap partition all die; `followers` passes through where `e_mode` did. The batch is now always one contiguous run.

**Files:**
- Delete: `rust/trajectory/src/e_independent.rs`, `rust/trajectory/src/e_independent/tests.rs`, `rust/trajectory/src/partition.rs`, `rust/trajectory/src/partition/tests.rs`
- Modify: `rust/trajectory/src/lib.rs` (module decls, `ELimits`, `ShapeSegmentInput`, the `e_limits` field on the plan-input struct — find with `grep -n "e_limits\|ELimits" rust/trajectory/src/lib.rs`)
- Modify: `rust/trajectory/src/beta.rs`, `rust/trajectory/src/plan_velocity.rs`, `rust/trajectory/src/emit_shaped.rs`, `rust/trajectory/src/streaming/state.rs`, `rust/trajectory/src/streaming/emit.rs`, `rust/trajectory/src/streaming/mod.rs`
- Tests: `grep -rln "e_mode\|ELimits\|e_limits\|EGap\|partition\|e_independent\|extrusion_per_xy_mm" rust/trajectory/`

- [ ] **Step 1: Types.** In `lib.rs`: delete `pub struct ELimits` and the `e_limits` field (every writer found via compiler); delete `pub mod e_independent;` and `pub mod partition;`. `ShapeSegmentInput` swaps its three E fields for one:

```rust
#[derive(Debug, Clone, Copy)]
pub struct ShapeSegmentInput<'a> {
    pub temporal: temporal::multi::SegmentInput<'a>,
    pub followers: &'a [geometry::segment::FollowerDemand],
    pub feedrate_mm_s: f64,
}
```

In `emit_shaped.rs`, `ShapedSegment` (fields at `grep -n "e_mode\|extrusion" rust/trajectory/src/emit_shaped.rs`) swaps `e_mode` + `extrusion_per_xy_mm` for `followers: Vec<geometry::segment::FollowerDemand>` (populate with `m.followers.to_vec()`), and the `e_independent: None` output line is deleted.

- [ ] **Step 2: beta.rs — single-run collapse.** `partition_batch` is gone; every `BatchPartition` consumer collapses to the whole-batch run. Concretely, in `beta.rs`:
  - Delete `assemble_e_only_output`, `build_e_halos`, `assemble_with_e_gaps`, and every `e_gaps`/`EGap`/`e_halos` reference (`grep -n "e_gap\|e_halo\|EGap\|e_only" rust/trajectory/src/beta.rs`).
  - Where functions took `partition: &BatchPartition`, take the segments directly and use the single run `0..input.segments.len()`; `collect_xy_meta` iterates all segment indices; `compute_batch_t_end` is the last run's end with no gap additions.
  - The `if partition.runs.is_empty()` early-out (empty batch) becomes `if input.segments.is_empty()`.
  - `EmitSegmentMeta` construction (`grep -n "EmitSegmentMeta" rust/trajectory/src/beta.rs`) carries `followers: input.segments[i].followers` instead of `e_mode`/`extrusion_per_xy_mm`.
  - Same sweep in `plan_velocity.rs` and `streaming/` (`grep -rn "partition\|e_limits\|e_mode\|extrusion_per_xy_mm\|e_independent" rust/trajectory/src/` until zero hits) — `streaming/state.rs` passes `m.segment.followers.clone()` (or slice) where it passed the three E fields. Follow the compiler; the shape of every change is "three fields become one, gaps machinery deletes."

- [ ] **Step 3: Port trajectory tests.** Same mechanical mapping as Task 2 Step 6. Tests that exercised E-gap scheduling (`grep -rln "EGap\|e_gap\|schedule_e\|Independent" rust/trajectory/`) are deleted with the machinery — they test behavior that no longer exists. Tests that used `EMode::Travel`/`CoupledToXy` as incidental fixture data get `vec![]` / a `FollowerDemand`.

- [ ] **Step 4: Run** — `cargo nextest run -p trajectory` → PASS
- [ ] **Step 5: Commit** — `feat(trajectory): delete E trapezoid/ELimits/partition; followers pass through`

---

### Task 4: motion-bridge — axis registry and config integration

**Files:**
- Modify: `rust/motion-bridge/src/config.rs` (registry beside plan 1's `LimitSection`; replace plan 1's hardcoded `axis_index`)
- Modify: `rust/motion-bridge/src/classify.rs` (segment construction; rejection stays)
- Modify: `rust/motion-bridge/src/bridge.rs` (`init_planner` gains the axes param), `rust/motion-bridge/src/planner.rs` (follow compiler: `e_limits` removal, `ShapeSegmentInput` construction sites — `grep -rn "e_limits\|ELimits\|e_mode" rust/motion-bridge/ rust/kalico-host-rt/`)
- Test: `rust/motion-bridge/src/config/tests.rs`

- [ ] **Step 1: Write failing registry tests** in `config/tests.rs`:

```rust
fn decl(name: &str, follows: &[&str]) -> AxisDecl {
    AxisDecl {
        name: name.into(),
        follows: follows.iter().map(|s| s.to_string()).collect(),
        motors: vec![],
    }
}

#[test]
fn registry_orders_spatial_then_followers() {
    let reg = AxisRegistry::try_new(vec![
        decl("e", &["x", "y", "z"]),
        decl("x", &[]),
        decl("y", &[]),
        decl("z", &[]),
    ])
    .unwrap();
    assert_eq!(reg.axis_index("x").unwrap(), 0);
    assert_eq!(reg.axis_index("e").unwrap(), 3);
    assert_eq!(
        reg.follower_words(),
        vec![geometry::FollowerWord { letter: b'E', axis_index: 3 }]
    );
}

#[test]
fn registry_requires_spatial_axes() {
    let err = AxisRegistry::try_new(vec![decl("x", &[]), decl("y", &[])]).unwrap_err();
    assert!(matches!(err, AxisConfigError::MissingSpatialAxis { name } if name == "z"));
}

#[test]
fn registry_rejects_reserved_letters_and_long_names() {
    for bad in ["i", "j", "p", "q", "f", "g", "m", "n", "t", "ab"] {
        let mut decls = vec![decl("x", &[]), decl("y", &[]), decl("z", &[])];
        decls.push(decl(bad, &["x"]));
        assert!(AxisRegistry::try_new(decls).is_err(), "expected rejection: {bad}");
    }
}

#[test]
fn follows_must_reference_declared_axes_and_spatial_cannot_follow() {
    let decls = vec![decl("x", &[]), decl("y", &[]), decl("z", &[]), decl("e", &["w"])];
    assert!(matches!(
        AxisRegistry::try_new(decls).unwrap_err(),
        AxisConfigError::UnknownFollowTarget { .. }
    ));
    let decls = vec![decl("x", &["y"]), decl("y", &[]), decl("z", &[])];
    assert!(matches!(
        AxisRegistry::try_new(decls).unwrap_err(),
        AxisConfigError::SpatialAxisCannotFollow { .. }
    ));
}

#[test]
fn limit_sections_partition_spatial_follower_mixed() {
    let reg = AxisRegistry::try_new(vec![
        decl("x", &[]),
        decl("y", &[]),
        decl("z", &[]),
        decl("e", &["x", "y", "z"]),
    ])
    .unwrap();
    let mut cfg = PlannerConfig::default();
    cfg.axis_registry = reg;
    cfg.limit_sections.push(LimitSection {
        name: "extruder".into(),
        axes: vec![3],
        max_velocity: Some(75.0),
        max_accel: Some(1500.0),
        max_jerk: None,
    });
    cfg.to_temporal_limits().unwrap();
    cfg.limit_sections.push(LimitSection {
        name: "mixed".into(),
        axes: vec![0, 3],
        max_velocity: Some(10.0),
        max_accel: None,
        max_jerk: None,
    });
    assert!(matches!(
        cfg.to_temporal_limits().unwrap_err(),
        LimitConfigError::MixedSpatialFollower { .. }
    ));
}

#[test]
fn follower_axis_without_limit_coverage_is_an_error() {
    let reg = AxisRegistry::try_new(vec![
        decl("x", &[]),
        decl("y", &[]),
        decl("z", &[]),
        decl("e", &["x", "y", "z"]),
    ])
    .unwrap();
    let mut cfg = PlannerConfig::default();
    cfg.axis_registry = reg;
    assert!(matches!(
        cfg.to_temporal_limits().unwrap_err(),
        LimitConfigError::NoFollowerCoverage { .. }
    ));
}
```

- [ ] **Step 2: Run to verify failure** — `cargo nextest run -p motion-bridge -E 'test(config)'` → FAIL

- [ ] **Step 3: Implement the registry** in `config.rs` (delete plan 1's free `axis_index` fn; `SPATIAL` order is the fixed Cartesian frame the geometry pipeline emits):

```rust
const SPATIAL: [&str; 3] = ["x", "y", "z"];
const RESERVED_LETTERS: [u8; 9] = [b'i', b'j', b'p', b'q', b'f', b'g', b'm', b'n', b't'];

#[derive(Debug, Clone, PartialEq)]
pub struct AxisDecl {
    pub name: String,
    pub follows: Vec<String>,
    pub motors: Vec<String>,
}

#[derive(Debug, Clone, PartialEq, Default)]
pub struct AxisRegistry {
    ordered: Vec<AxisDecl>,
}

#[derive(Debug, Error)]
pub enum AxisConfigError {
    #[error("axis '{name}' must be a single ascii letter a-z")]
    BadName { name: String },
    #[error("axis '{name}': letter is reserved for G-code structure")]
    ReservedLetter { name: String },
    #[error("duplicate axis '{name}'")]
    Duplicate { name: String },
    #[error("required spatial axis '{name}' is not declared")]
    MissingSpatialAxis { name: String },
    #[error("axis '{axis}': follows references undeclared axis '{target}'")]
    UnknownFollowTarget { axis: String, target: String },
    #[error("spatial axis '{name}' cannot declare follows")]
    SpatialAxisCannotFollow { name: String },
}

impl AxisRegistry {
    pub fn try_new(decls: Vec<AxisDecl>) -> Result<Self, AxisConfigError> {
        for d in &decls {
            let bytes = d.name.as_bytes();
            if bytes.len() != 1 || !bytes[0].is_ascii_lowercase() {
                return Err(AxisConfigError::BadName { name: d.name.clone() });
            }
            if RESERVED_LETTERS.contains(&bytes[0]) {
                return Err(AxisConfigError::ReservedLetter { name: d.name.clone() });
            }
            if decls.iter().filter(|o| o.name == d.name).count() > 1 {
                return Err(AxisConfigError::Duplicate { name: d.name.clone() });
            }
        }
        let mut ordered = Vec::with_capacity(decls.len());
        for name in SPATIAL {
            let d = decls
                .iter()
                .find(|d| d.name == name)
                .ok_or(AxisConfigError::MissingSpatialAxis { name: name.into() })?;
            if !d.follows.is_empty() {
                return Err(AxisConfigError::SpatialAxisCannotFollow { name: name.into() });
            }
            ordered.push(d.clone());
        }
        for d in &decls {
            if SPATIAL.contains(&d.name.as_str()) {
                continue;
            }
            for target in &d.follows {
                if !decls.iter().any(|o| &o.name == target) {
                    return Err(AxisConfigError::UnknownFollowTarget {
                        axis: d.name.clone(),
                        target: target.clone(),
                    });
                }
            }
            ordered.push(d.clone());
        }
        Ok(Self { ordered })
    }

    pub fn axis_index(&self, name: &str) -> Result<usize, AxisConfigError> {
        self.ordered
            .iter()
            .position(|d| d.name == name)
            .ok_or(AxisConfigError::BadName { name: name.into() })
    }

    #[must_use]
    pub fn n_axes(&self) -> usize {
        self.ordered.len()
    }

    #[must_use]
    pub fn is_spatial(&self, index: usize) -> bool {
        index < SPATIAL.len()
    }

    #[must_use]
    pub fn follower_words(&self) -> Vec<geometry::FollowerWord> {
        self.ordered
            .iter()
            .enumerate()
            .skip(SPATIAL.len())
            .map(|(axis_index, d)| geometry::FollowerWord {
                letter: d.name.as_bytes()[0].to_ascii_uppercase(),
                axis_index,
            })
            .collect()
    }
}
```

`PlannerConfig` gains `pub axis_registry: AxisRegistry` (default: the three spatial axes). `Default for AxisRegistry` must build the spatial-only registry via `try_new`, not bypass validation.

- [ ] **Step 4: Limit-section resolution.** Extend plan 1's `to_temporal_limits`: partition sections by `axes` membership using `axis_registry.is_spatial`. All-spatial sections → `temporal::LimitSet` as in plan 1 (`AxisSet::from_indices` still 3-axis — follower indices never reach temporal in this plan). All-follower sections are recorded for coverage only. Mixed → new error variant. Add to `LimitConfigError`:

```rust
#[error("[limit {section}]: mixing spatial and follower axes in one set is not yet supported")]
MixedSpatialFollower { section: String },
#[error("follower axis '{axis}': no [limit] section declares max_velocity and max_accel covering it")]
NoFollowerCoverage { axis: String },
```

Coverage rule: every non-spatial registry axis must appear in ≥1 section carrying finite `max_velocity` AND ≥1 carrying finite `max_accel` (jerk not required until plan 3). The runtime-caps overlay (plan 1) stays spatial-only — it uses `AxisSet::all()`, which is the spatial set.

- [ ] **Step 5: classify.rs + plumbing.** `CubicSegment::try_new(xyz, vec![], feedrate_mm_s, source, None)` at the construction site; `MoveClass`/`ExtrusionNotSupported` untouched. `bridge.rs::init_planner` gains a leading param `axes: Vec<(String, Vec<String>, Vec<String>)>` (name, follows, motors) → `AxisDecl`s → `AxisRegistry::try_new` → store in config → eager `cfg.to_temporal_limits()?` validation as plan 1 established. Everywhere `ShapeSegmentInput`/plan-input structs are built (`grep -rn "e_limits\|e_mode\|extrusion_per_xy_mm" rust/motion-bridge/ rust/kalico-host-rt/`), apply the Task 3 shapes; delete the `PlannerConfig` `e_limits` field and its default.

- [ ] **Step 6: Run** — `cargo nextest run -p motion-bridge` → PASS, then full workspace `cargo nextest run` from `rust/` → PASS (catches stragglers in kalico-host-rt and integration tests).
- [ ] **Step 7: Commit** — `feat(motion-bridge): axis registry; follower-aware [limit] validation; e_limits dies`

---

### Task 5: klippy — `[axis]` sections, firmware-retraction rejection

**Files:**
- Create: `klippy/extras/axis.py`
- Modify: `klippy/motion_toolhead.py` (read axes, pass to bridge, reject `[firmware_retraction]`, validate limit-section axis names against declarations)
- Modify: `klippy/extras/limit.py` (drop the hardcoded `SUPPORTED_AXES` check — authority moves to the declared-axes validation)
- Modify: `klippy/motion_bridge.py` (wrapper + `_StubBridge` signatures)

- [ ] **Step 1: `klippy/extras/axis.py`:**

```python
RESERVED_LETTERS = ("i", "j", "p", "q", "f", "g", "m", "n", "t")


class AxisSection:
    def __init__(self, config):
        self.name = config.get_name().split(None, 1)[1]
        if len(self.name) != 1 or not self.name.islower() or not self.name.isalpha():
            raise config.error(
                "[%s]: axis name must be a single letter a-z" % config.get_name()
            )
        if self.name in RESERVED_LETTERS:
            raise config.error(
                "[%s]: letter '%s' is reserved for G-code structure"
                % (config.get_name(), self.name)
            )
        self.follows = [a.strip().lower() for a in config.getlist("follows", [])]
        self.motors = [m.strip() for m in config.getlist("motors", [])]

    def get_status(self, eventtime):
        return {"follows": list(self.follows), "motors": list(self.motors)}


def load_config_prefix(config):
    return AxisSection(config)
```

- [ ] **Step 2: `motion_toolhead.py`.** In init (next to plan 1's `_read_limits` call):

```python
if config.has_section("firmware_retraction"):
    raise config.error(
        "[firmware_retraction] is not supported: it presupposes an extruder "
        "concept the motion system does not have"
    )
self.axis_sections = []
for sc in config.get_prefix_sections("axis "):
    name = sc.get_name().split(None, 1)[1]
    follows = [a.strip().lower() for a in sc.getlist("follows", [])]
    motors = [m.strip() for m in sc.getlist("motors", [])]
    self.axis_sections.append((name, follows, motors))
declared = {name for name, _, _ in self.axis_sections}
for required in ("x", "y", "z"):
    if required not in declared:
        raise config.error(
            "[axis %s] section is required (every axis must be declared)" % required
        )
for _, axes, _, _, _ in self.limit_sections:
    for a in axes:
        if a not in declared:
            raise config.error(
                "[limit] references undeclared axis '%s' (declare [axis %s])" % (a, a)
            )
```

(The `limit_sections` tuple shape is plan 1 Task 6's `(name, axes, v, a, j)` — adjust the destructuring to match what landed.) The `_init_planner` call prepends `list(self.axis_sections)` as the first argument to `bridge.init_planner(...)`. Cross-validation note: deep validation (follows targets, reserved letters, follower coverage) is Rust-side in the registry; klippy only checks structure it alone knows (sections present, names declared).

- [ ] **Step 3: `limit.py`** — delete the `SUPPORTED_AXES` tuple and its loop; keep non-empty/at-least-one-cap checks.

- [ ] **Step 4: `motion_bridge.py`** — `init_planner` wrapper passes the axes list through; `_StubBridge.init_planner` accepts it (no-op).

- [ ] **Step 5: Commit** — `feat(klippy): [axis] sections; reject [firmware_retraction]; limit axes validated against declarations`

---

### Task 6: Fixture sweep

**Files:** discovered, not fixed in advance.

- [ ] **Step 1:** `grep -rln "\[limit" --include="*.cfg" .` — every fixture plan 1 migrated now also needs axis declarations. Add to each:

```ini
[axis x]
[axis y]
[axis z]
```

For fixtures with an extruder (`grep -l "\[extruder\]" <those files>`), also add:

```ini
[axis e]
follows: x, y, z

[limit extruder]
axes: e
max_velocity: 75
max_accel: 1500
```

(Nothing consumes `e` motion yet — the declarations exercise registry + coverage validation and are the forward-looking shape. Live extrusion remains rejected; sim prints stay travel-only, unchanged from today.)

- [ ] **Step 2:** Commit — `chore: declare [axis] sections in config fixtures`

---

### Task 7: Fossil sweep and end-to-end verification

- [ ] **Step 1: Grep for zero survivors** (outside `docs/` and this plan):

Run: `grep -rn "EMode\|e_mode\|extrusion_per_xy_mm\|e_independent\|ELimits\|e_limits\|xy_arc_length\|Helical\|EGap\|e_gaps\|e_halos\|schedule_e\|e_delta" rust/ klippy/ --include="*.rs" --include="*.py" | grep -v test/klippy`

Every hit dies or gets a written justification in the commit message. Expected legitimate survivors: none in `rust/`; legacy `ToolHead`-path code in `klippy/` keeps its behavior (legacy class untouched, as in plan 1).

- [ ] **Step 2:** `cargo nextest run` from `rust/` → full PASS; `cargo test --doc` if doc examples were touched; `cargo fmt --all --check` → clean.
- [ ] **Step 3: Sim verification** (kalico-sim skill):
  - Migrated fixture boots clean; homing + travel square runs (behavior identical to pre-plan-2).
  - Fixture missing `[axis x]` → startup error naming the section.
  - Fixture with `[firmware_retraction]` → startup error.
  - Fixture with `[axis e]` but no covering `[limit]` → startup error naming follower coverage.
- [ ] **Step 4: Commit** — `feat: axis registry + follower segments end-to-end (plan 2)`

---

## Self-review notes (spec → plan coverage)

- `[axis <name>]` config objects, `follows`, `motors:` key parsed (consumed in plan 5): Tasks 4–5 ✓
- Axis name = G-code letter, structural-letter collisions rejected: Tasks 4–5 (RESERVED_LETTERS both sides) ✓
- 3D arc length; vase mode / hop-retract become ordinary moves; helical error deleted: Tasks 1–2 ✓
- One follower rule, segments carry ratio only, no modes: Task 2 ✓
- Absolute→delta at the boundary, per-follower ledger, G92 resets ledger: Task 2 Step 3 ✓
- `e_independent` trapezoid, `ELimits`, partition/E-gap machinery deleted with nothing replacing them: Task 3 ✓
- Follower-only moves fail loudly (until plan 3): Task 2 Step 4 ✓
- Live-path extrusion rejection stays (until plan 4): Task 4 Step 5 (explicitly untouched) ✓
- `[firmware_retraction]` rejected at load: Task 5 Step 2 ✓
- Coverage: every declared axis in ≥1 limit (follower axes validated bridge-side; spatial via plan 1 temporal check); mixed sets rejected loudly: Task 4 Step 4 ✓
- MCU untouched, temporal crate untouched: no task modifies them ✓
