# Per-Axis Emission Chain Implementation Plan (Plan 4)

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Every axis runs the same emission chain — input track → post-processor chain → fit — with the follower's input track built by odometer quadrature over the followed axes' realized curves; `[post_processor]` config objects unify shaper kernels and linear pressure advance with runtime-tunable parameters; the live-path `ExtrusionNotSupported` rejection dies and follower pieces flow down the already-existing lane 3.

**Architecture:** Spec: `docs/superpowers/specs/2026-06-12-follower-axes-and-limits-design.md` §4 (post-processor abstraction, post-chain limits) and §5 (emission chain, two ledgers). `trajectory` gains a `post_processor` module (trait + `smooth_zv`/`smooth_mzv`/`linear_pressure_advance` types, per-axis compiled chains) and an `odometer` module (Gauss–Legendre arc length over exact polynomial derivatives); `emit_shaped` becomes a two-pass per-axis loop over registry-indexed tracks (`ShapedSegment.axes: Vec<_>`). `motion-engine` parses `[post_processor]` sections, compiles chains, generalizes `update_shaper` → `update_post_processor`, and lifts the classify rejection. klippy parses the sections, rejects `[input_shaper]`, and gains `SET_POST_PROCESSOR`.

**Tech stack:** Rust (`trajectory`, `motion-engine`, `geometry` crates), pyo3 bridge (`bridge.rs`), klippy Python (`motion_toolhead.py`, `motion_engine.py`). Tests: `cargo nextest run` from `rust/` (never bare `cargo test`); Python via `./scripts/ci.sh py`.

**PRECONDITION: Plan 3 (`docs/superpowers/plans/2026-06-12-planner-extension-follower-rows.md`) is fully landed.** Executors verify before starting: `git log --oneline | head` shows plan 3's final commit, `rg "follower_pa" rust/trajectory/src/lib.rs` hits (`ShapeBatchInput.follower_pa: [f64; temporal::MAX_AXES]`), and `cargo nextest run` from `rust/` is green. Plan 3 was in flight while this plan was written — **all code excerpts here are anchors, not gospel; anchor by symbol name and the given grep commands, never by line, and re-read every touched file before editing.**

**Out of scope (later plans/deferred):** kinematics modules beyond the existing corexy special-case (plan 5); binding-constraint reporting (plan 6); windowed post-chain rows for *spatial* axes (deferred tightening, spec §6); nonlinear post-processor types; `SET_PRESSURE_ADVANCE` / `[input_shaper]` compat shims; the published `toolhead` status shim (plan 5). Mixed spatial+follower limit sets stay rejected.

**Merge ordering:** this plan's branch lands only after plan 3 is merged — Task 9 (rejection lift) must never ship against a solver that doesn't constrain follower demands.

**Repo rules for every task:** unit tests in separate files from tested code; no explanatory comments — name/extract instead; fail loudly (no silent fallbacks); commit after every task; no Claude/Anthropic commit trailers; `cargo fmt --all --check` before any PR push; `./scripts/ci.sh quick` green before opening/updating the PR, plus `./scripts/ci.sh py` because klippy changes.

---

## Design decisions this plan makes (agreed with the user 2026-06-12)

1. **One abstraction, normalized execution order.** Shaper kernels and linear PA are both linear time-invariant operators (spec §4). The trait's single source of truth is `action()` returning a `PlanAction`: `Kernel(PiecewisePolynomialKernel<f64>)` or `DerivativeGain { k }` (PA = `track + k·track′`). Because linear operators commute, the emission runner normalizes any chain to: exact derivative-gain ops applied symbolically on the NURBS first, then kernel convolutions on the sampled signal, then one `fit_c2_cubic_with_bc`. Mathematically identical to declared order, computationally exact for the PA half. A future nonlinear type breaks commutativity and must bring its own runner mode — until then nothing is order-sensitive.

2. **v1 chain-compilation boundary (loud).** The landed temporal API accepts per axis: one kernel (`ReplanContext.kernels` / `ShapeBatchInput.shaper`) and one PA gain (`ShapeBatchInput.follower_pa`). Therefore: a chain compiles to `CompiledChain { kernel: Option<...>, gain: f64 }`; **more than one kernel or more than one derivative-gain on a single axis is a config-load error naming the limitation** (two gains compose to a second-derivative term `k₁k₂·F″` no row family expresses; two kernels would need a composed plan window nothing builds yet). Purely additive to lift later.

3. **Registry-indexed tracks.** `ShapedSegment.axes: [ScalarNurbs; 3]` becomes `Vec<ScalarNurbs<f64>>`, index = axis registry index (0..2 spatial, 3.. followers). `enqueue_segment` already guards `axis_idx >= seg.axes.len()` and `McuAxisConfig.axes` already lists per-MCU axis indices, so lane 3 dispatch is wiring, not protocol work.

4. **Odometer mirrors plan 3's rows.** Realized speed is `‖v(t)‖` over the followed axes' *post-chain* tracks; follower velocity is `ratio(t)·‖v(t)‖` with `ratio(t)` piecewise per segment — exactly the solver's row definition, so plan and emission agree by construction. Distance via Gauss–Legendre over exact polynomial derivatives per Bézier piece, host f64.

5. **Runtime tuning generalizes the existing path.** `Planner::update_shaper` / `PlannerMsg::UpdateShaper` (see `rust/motion-engine/src/planner.rs`, `grep -n "fn update_shaper" rust/motion-engine/src/planner.rs`) becomes `update_post_processor(name, param, value)`; the change lands in the planner thread's config and takes effect at the next replan — committed trajectory is never rewritten. klippy command: `SET_POST_PROCESSOR NAME=<instance> <PARAM>=<value>`.

6. **Two ledgers need no new machinery.** Nominal: klippy's existing absolute→delta normalization (plan 2) is untouched. Physical: once lane 3 flows, `HistoryStore` (`rust/motion-engine/src/motion_history.rs`) records follower pieces per `AxisKey` like any axis; the physical endpoint is its per-axis endpoint. Plan 4 adds tests, not surface.

7. **Follower-only moves** (`CubicSegment.virtual_path_mm: Some(len)`): the follower track comes straight from the plan — `track = start + ratio × s(t)` with `s(t)` the planned path progress; no odometer (the spatial curve is identically zero). This retires `ShapeError::VirtualPathUnrouted`.

---

## Task 1: `trajectory::post_processor` — trait, types, compiled chains

**Files:**
- Create: `rust/trajectory/src/post_processor.rs`
- Create: `rust/trajectory/src/post_processor/tests.rs`
- Modify: `rust/trajectory/src/lib.rs` (declare module, re-export)

- [ ] **Step 1: Write failing tests** in `post_processor/tests.rs`:

```rust
use super::*;

fn pa(k: f64) -> PostProcessorInstance {
    PostProcessorInstance::new("pa", PostProcessorType::LinearPressureAdvance { k })
}
fn zv(hz: f64) -> PostProcessorInstance {
    PostProcessorInstance::new("is", PostProcessorType::SmoothZv { frequency_hz: hz })
}

#[test]
fn compile_empty_chain_is_identity() {
    let c = CompiledChain::compile(&[]).unwrap();
    assert!(c.kernel.is_none());
    assert_eq!(c.gain, 0.0);
}

#[test]
fn compile_kernel_plus_gain() {
    let c = CompiledChain::compile(&[zv(50.0), pa(0.04)]).unwrap();
    assert!(c.kernel.is_some());
    assert_eq!(c.gain, 0.04);
}

#[test]
fn compile_order_irrelevant_for_linear_ops() {
    let a = CompiledChain::compile(&[zv(50.0), pa(0.04)]).unwrap();
    let b = CompiledChain::compile(&[pa(0.04), zv(50.0)]).unwrap();
    assert_eq!(a.gain, b.gain);
    assert_eq!(a.kernel.is_some(), b.kernel.is_some());
}

#[test]
fn compile_two_kernels_rejected() {
    let err = CompiledChain::compile(&[zv(50.0), zv(40.0)]).unwrap_err();
    assert!(matches!(err, PostProcessorError::UnsupportedComposition { .. }));
}

#[test]
fn compile_two_gains_rejected() {
    let err = CompiledChain::compile(&[pa(0.04), pa(0.01)]).unwrap_err();
    assert!(matches!(err, PostProcessorError::UnsupportedComposition { .. }));
}

#[test]
fn set_param_updates_gain() {
    let mut inst = pa(0.04);
    inst.set_param("k", 0.06).unwrap();
    let c = CompiledChain::compile(std::slice::from_ref(&inst)).unwrap();
    assert_eq!(c.gain, 0.06);
}

#[test]
fn set_param_unknown_key_fails() {
    let mut inst = zv(50.0);
    assert!(inst.set_param("k", 1.0).is_err());
}

fn t_squared_cubic() -> nurbs::ScalarNurbs<f64> {
    // t² degree-elevated to a cubic on [0,1]: Bernstein control points [0, 0, 1/3, 1].
    nurbs::ScalarNurbs::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![0.0, 0.0, 1.0 / 3.0, 1.0],
    )
    .unwrap()
}

#[test]
fn derivative_gain_applied_exactly_on_nurbs() {
    // PA k=0.5 on track(t)=t² ⇒ out(t) = t² + 0.5·2t = t² + t.
    let out = apply_derivative_gain(&t_squared_cubic(), 0.5);
    for t in [0.0, 0.25, 0.5, 0.75, 1.0] {
        assert!((nurbs::eval::eval(&out, t) - (t * t + t)).abs() < 1e-12);
    }
}
```

(If `ScalarNurbs::try_new` is spelled differently, match the constructor used in `rust/trajectory/src/beta.rs`'s `constant_cubic_nurbs`.)

- [ ] **Step 2: Run to verify failure** — `cargo nextest run -p trajectory -E 'test(post_processor)'` → FAIL (module missing)

- [ ] **Step 3: Implement** `post_processor.rs`:

```rust
use nurbs::algebra::PiecewisePolynomialKernel;
use nurbs::ScalarNurbs;

#[derive(Debug, Clone, PartialEq)]
pub enum PostProcessorType {
    SmoothZv { frequency_hz: f64 },
    SmoothMzv { frequency_hz: f64 },
    LinearPressureAdvance { k: f64 },
}

#[derive(Debug, Clone)]
pub struct PostProcessorInstance {
    name: String,
    ty: PostProcessorType,
}

#[derive(Debug, Clone)]
pub enum PlanAction {
    Kernel(PiecewisePolynomialKernel<f64>),
    DerivativeGain { k: f64 },
}

#[derive(Debug, Clone, Default)]
pub struct CompiledChain {
    pub kernel: Option<PiecewisePolynomialKernel<f64>>,
    pub gain: f64,
}

#[derive(Debug, thiserror::Error)]
pub enum PostProcessorError {
    #[error("post_processor '{name}': unknown parameter '{key}'")]
    UnknownParam { name: String, key: String },
    #[error(
        "axis chain unsupported: {detail}. v1 allows at most one kernel and one \
         derivative-gain post-processor per axis"
    )]
    UnsupportedComposition { detail: String },
}

impl PostProcessorInstance {
    pub fn new(name: &str, ty: PostProcessorType) -> Self { /* ... */ }
    pub fn name(&self) -> &str { /* ... */ }
    pub fn action(&self) -> PlanAction {
        // SmoothZv/SmoothMzv → PlanAction::Kernel via the existing builders in
        // rust/trajectory/src/kernel.rs (grep -n "smooth_zv\|0.8025" rust/trajectory/src/kernel.rs);
        // LinearPressureAdvance → DerivativeGain
    }
    pub fn set_param(&mut self, key: &str, value: f64) -> Result<(), PostProcessorError> {
        // "frequency_hz" for kernel types, "k" for PA; anything else → UnknownParam
    }
}

impl CompiledChain {
    pub fn compile(chain: &[PostProcessorInstance]) -> Result<Self, PostProcessorError> {
        // fold actions; second kernel or second gain → UnsupportedComposition
    }
}

pub fn apply_derivative_gain(track: &ScalarNurbs<f64>, k: f64) -> ScalarNurbs<f64> {
    // exact: track + k·derivative(track), via nurbs::eval::derivative,
    // nurbs::algebra::scalar_multiply, nurbs::algebra::add_with_knot_union.
    // NOTE: derivative drops degree 3→2; degree-elevate back to 3 before the add
    // (grep -rn "degree_elev\|elevate" rust/nurbs/src/) so the result stays uniform-cubic.
}
```

Move/reuse the frequency→kernel construction from `rust/trajectory/src/kernel.rs` rather than duplicating constants. In `lib.rs`: `pub mod post_processor;` and re-export `PostProcessorInstance, PostProcessorType, CompiledChain, PostProcessorError`.

- [ ] **Step 4: Run** — `cargo nextest run -p trajectory` → PASS
- [ ] **Step 5: Commit** — `feat(trajectory): post-processor trait, linear PA + kernel types, compiled per-axis chains`

---

## Task 2: `trajectory::odometer` — realized arc length and follower track

**Files:**
- Create: `rust/trajectory/src/odometer.rs`
- Create: `rust/trajectory/src/odometer/tests.rs`
- Modify: `rust/trajectory/src/lib.rs` (declare module)

- [ ] **Step 1: Write failing tests:**

```rust
use super::*;

fn linear_cubic(p0: f64, p1: f64, t0: f64, t1: f64) -> nurbs::ScalarNurbs<f64> {
    // straight line as a cubic: equally spaced Bernstein control points on [t0, t1]
    nurbs::ScalarNurbs::try_new(
        3,
        vec![t0, t0, t0, t0, t1, t1, t1, t1],
        vec![p0, p0 + (p1 - p0) / 3.0, p0 + 2.0 * (p1 - p0) / 3.0, p1],
    )
    .unwrap()
}
fn constant_cubic(v: f64, t0: f64, t1: f64) -> nurbs::ScalarNurbs<f64> {
    linear_cubic(v, v, t0, t1)
}

#[test]
fn straight_line_distance_is_exact() {
    // x(t)=120t, y(t)=50t, z(t)=0 on [0,2] as cubics ⇒ distance(t) = 130t.
    let axes = vec![linear_cubic(0.0, 240.0, 0.0, 2.0), linear_cubic(0.0, 100.0, 0.0, 2.0),
                    constant_cubic(0.0, 0.0, 2.0)];
    let odo = Odometer::build(&axes, 0.0, 2.0, 64).unwrap();
    assert!((odo.distance_at(1.0) - 130.0).abs() < 1e-9);
    assert!((odo.distance_at(2.0) - 260.0).abs() < 1e-9);
}

#[test]
fn curved_path_matches_dense_reference() {
    // quarter-turn-ish Bezier in XY; reference = trapezoid over 100_000 samples of ‖v‖.
    // assert relative error < 1e-7
}

#[test]
fn follower_track_pays_out_ratio_times_distance() {
    // ratio(t) piecewise: 0.05 on [0,1), 0.0 on [1,2). start=7.0.
    // track(2.0) == 7.0 + 0.05·distance_at(1.0); track is monotone on [0,1).
}

#[test]
fn ratio_discontinuity_lands_at_segment_boundary_sample() {
    // boundary between ratios must be a quadrature breakpoint, not interpolated across.
}
```

- [ ] **Step 2: Run to verify failure** — `cargo nextest run -p trajectory -E 'test(odometer)'` → FAIL

- [ ] **Step 3: Implement** `odometer.rs`:

```rust
pub struct Odometer { /* cumulative arc-length table: knots t_i, distance_i, per-interval GL nodes */ }

impl Odometer {
    /// axes: post-chain spatial tracks sharing one time domain [t_start, t_end].
    /// Breakpoints = union of all axes' Bézier piece boundaries (extract via
    /// nurbs::bezier::extract_bezier_pieces — same call enqueue_segment uses).
    /// Per interval: 8-point Gauss–Legendre on ‖(x′,y′,z′)(t)‖ with exact
    /// polynomial derivatives (nurbs::eval::derivative once per axis, then eval).
    pub fn build(axes: &[ScalarNurbs<f64>], t_start: f64, t_end: f64, min_intervals: usize)
        -> Result<Self, OdometerError>;
    pub fn distance_at(&self, t: f64) -> f64; // monotone piecewise interpolation on the table
}

/// ratio_spans: (t_span_end, ratio) per source segment, covering [t_start, t_end] exactly —
/// gaps or overlap are a loud OdometerError, not patched.
pub fn follower_track(
    odo: &Odometer,
    start: f64,
    ratio_spans: &[(f64, f64)],
    t_start: f64,
    t_end: f64,
) -> impl Fn(f64) -> f64 + '_;
```

`follower_track` evaluates `start + Σ ratioᵢ·(distance_at(min(t, spanᵢ_end)) − distance_at(spanᵢ_start)))` — splitting at span boundaries so a ratio change never bleeds across. 8-point GL is exact for polynomials to degree 15; `‖v‖` is a square root of a degree-4 polynomial, not itself polynomial, hence the dense-reference test rather than an exactness claim.

- [ ] **Step 4: Run** — `cargo nextest run -p trajectory` → PASS
- [ ] **Step 5: Commit** — `feat(trajectory): odometer quadrature over realized spatial tracks`

---

## Task 3: `ShapedSegment.axes` goes registry-wide (`Vec`), mechanical sweep

Pure representation change, no behavior: `[ScalarNurbs<f64>; 3]` → `Vec<ScalarNurbs<f64>>` (always length ≥ 3; followers appended by Task 4).

**Files:**
- Modify: `rust/trajectory/src/lib.rs` (`ShapedSegment`)
- Modify: every consumer — find them all: `rg -ln "ShapedSegment" rust/` (expect: `trajectory/src/{emit_shaped,beta,streaming/*}`, `motion-engine/src/{planner,enqueue,dispatch}` + tests)

- [ ] **Step 1: Write failing test** (in `rust/trajectory/src/tests.rs`): construct a `ShapedSegment` with 4 axes; assert `seg.axes.len() == 4`. → does not compile against the array type.
- [ ] **Step 2: Change the field, chase compiler errors.** `emit_shaped` builds `vec![x, y, z]`; `enqueue_segment`'s existing `axis_idx >= seg.axes.len()` guard and corexy indexing (`rust/motion-engine/src/enqueue.rs`) survive unchanged. No site may silently truncate or pad — where a consumer assumed exactly 3, keep the assumption as an explicit `assert!`/error if it is real, otherwise generalize.
- [ ] **Step 3: Run** — `cargo nextest run` (workspace) → PASS, zero behavior change.
- [ ] **Step 4: Commit** — `refactor(trajectory): ShapedSegment carries registry-indexed track vector`

---

## Task 4: per-axis emission chain in `emit_shaped` (two passes)

The core. `emit_shaped_with_left_bc` (`rust/trajectory/src/emit_shaped.rs`) becomes a two-pass loop driven by per-axis `CompiledChain`s instead of bare kernels.

**Files:**
- Modify: `rust/trajectory/src/emit_shaped.rs`
- Modify: `rust/trajectory/src/emit_shaped/tests.rs`
- Modify: `rust/trajectory/src/lib.rs` (`ShapeBatchInput` gains `chains`; `ShapeError::VirtualPathUnrouted` deleted)

New signature (kernels parameter replaced; `FollowerSpec` tells pass two what to build):

```rust
pub struct AxisChainSet {
    /// index = axis registry index; spatial 0..3 then followers.
    pub chains: Vec<CompiledChain>,
    /// (follower_axis_index, followed_axis_indices) — from AxisRegistry.follows.
    pub followers: Vec<(usize, Vec<usize>)>,
}

pub fn emit_shaped_with_left_bc(
    planned: &[FittedSegment],
    meta: &[EmitSegmentMeta],
    chains: &AxisChainSet,
    history: &PerAxisHistory<'_>,
    follower_start: &[f64],          // physical start position per follower axis
    batch_t_start: f64,
    batch_t_end: f64,
    first_seg_left_bc: &[Option<f64>],
) -> Result<Vec<ShapedSegment>, ShapeError>
```

**Pass one** (axes with no `follows` entry): exactly today's per-axis body — constant short-circuit, kernel convolution via `pad_segment_axis_with_history` + `ShapedSignal`, or passthrough refit — except the kernel comes from `chains.chains[axis].kernel` and a nonzero `gain` applies `apply_derivative_gain` to the *fitted input* before convolution (decision 1's normalized order: gain symbolically first, kernel on samples second).

**Pass two** (followers, after pass one finished the axes they follow): per batch, build one `Odometer` over the followed axes' pass-one output curves (knot-unioned time domain across segments); per segment, assemble `ratio_spans` from `meta[seg].followers` (a follower absent from a segment's demands has ratio 0 for that span); the input track is `follower_track(...)`; then the follower's own chain: gain applies as `track(t) + k·track′(t)` where `track′(t) = ratio(t)·‖v(t)‖` is available in closed form from the odometer integrand — no numerical differentiation; kernel (rare) convolves the sampled closure; finally `fit_c2_cubic_with_bc` with the same C1 left-BC threading the spatial axes use (`prev_right_bc` per axis, now sized `n_axes`).

**Virtual-path segments** (`FittedSegment` whose source had `virtual_path_mm`): pass one emits constant tracks for spatial axes; pass two uses planned path progress `s(t)` directly — the fitted "spatial" curve is zero-displacement, so thread `s(t)` through `EmitSegmentMeta` (add `virtual_path: Option<f64>` plus whatever the plan-velocity layer already retains for it — `rg -n "virtual_path" rust/trajectory/src/` for plan 3's plumbing) and pay out `start + ratio·s(t)`. Delete `ShapeError::VirtualPathUnrouted` and route the streaming-path rejection (grep its raise site) into this path.

- [ ] **Step 1: Write failing tests:**

```rust
#[test]
fn passthrough_chains_reproduce_legacy_output_bitwise() {
    // Build a 3-segment batch; run old-style (no chains, no followers) and assert each
    // spatial axis's control points and knots are bit-identical to a golden capture
    // taken at the previous commit. THE regression gate for approach A.
}

#[test]
fn follower_track_integral_matches_ratio_times_arclength() {
    // one straight XY segment, follower ratio 0.05, passthrough everywhere:
    // follower end == start + 0.05·segment_length (1e-9); C1 at segment seams.
}

#[test]
fn follower_samples_post_kernel_path() {
    // 90° corner, smooth_mzv on x and y: follower end < start + ratio·nominal_length
    // (the kernel shortcuts the corner); equality on a straight line.
}

#[test]
fn pa_gain_boosts_follower_during_accel() {
    // gain k on follower axis: emitted follower velocity at mid-accel ≈ ratio·ṡ·(1) + k·ratio·s̈
    // sampled via finite difference of the fitted track against the plan profile (1e-3 rel).
}

#[test]
fn follower_only_move_emits_planned_track() {
    // virtual_path segment: spatial tracks constant, follower track == start + ratio·s(t).
}
```

- [ ] **Step 2: Run to verify failure** — `cargo nextest run -p trajectory -E 'test(emit_shaped)'` → FAIL
- [ ] **Step 3: Implement** as specified above. Keep `SMOOTH_FIT_TOLERANCE_MM` for followers too.
- [ ] **Step 4: Run** — `cargo nextest run -p trajectory` → PASS including the bitwise gate.
- [ ] **Step 5: Commit** — `feat(trajectory): two-pass per-axis emission chain with odometer follower tracks`

---

## Task 5: streaming integration — chains and follower history through replan

**Files:**
- Modify: `rust/trajectory/src/streaming/mod.rs` (`ReplanContext` gains `chains: AxisChainSet`; `EmitContext`/`AxisShaperQueue` sized by `n_axes`), `streaming/state.rs`, `streaming/emit.rs` (freeze-zone `max_h` over **all** axes' kernels incl. followers; `pending_freeze`/history trimming per axis generalizes — `PerAxisHistory` already carries 4 lanes, widen to `n_axes`), `streaming/tests.rs`
- Modify: `rust/trajectory/src/beta.rs` call sites (`rg -n "emit_shaped" rust/trajectory/src/`)

- [ ] **Step 1: Write failing test** in `streaming/tests.rs`: streaming replan over an extruding two-batch sequence emits follower pieces whose value at the batch seam is continuous (1e-9) and whose final value equals `start + Σ ratio·realized_distance`; a second test: follower history at the seam feeds pass two (no jump when the freeze zone splits a segment).
- [ ] **Step 2: Run to verify failure** — `cargo nextest run -p trajectory -E 'test(streaming)'` → FAIL
- [ ] **Step 3: Implement.** Thread `chains` from `ReplanContext` into both `plan_velocity` input assembly (the solver side already takes `follower_pa` + `shaper` — populate them **from the same `CompiledChain`s**, deleting any separate plumbing so plan and emission cannot disagree) and the Task-4 emission call. Track follower physical start across replans (the `follower_start` argument) in `ShaperState` next to where per-axis emit history already lives.
- [ ] **Step 4: Run** — `cargo nextest run -p trajectory` → PASS
- [ ] **Step 5: Commit** — `feat(trajectory): streaming replan threads post-processor chains and follower state`

---

## Task 6: motion-engine config — `[post_processor]` sections, chain compilation, planner init

**Files:**
- Modify: `rust/motion-engine/src/config.rs` (+ `config/tests.rs`): `AxisDecl` gains `post_processors: Vec<String>`; new `PostProcessorDecl { name, ty: String, params: Vec<(String, f64)> }`; `AxisRegistry::try_new` (or a sibling `compile_chains`) validates — every referenced name declared, unknown `type:` rejected (`UnsupportedKind` exists, repoint its message at `[post_processor]`), `CompiledChain::compile` errors surfaced verbatim at load
- Modify: `rust/motion-engine/src/planner.rs`: `PlannerConfig` drops `shaper: ShaperConfig` for `chains: AxisChainSet` + named instances `Vec<PostProcessorInstance>`; `build_replan_context` / `shaper_config_to_plan_shapers` / `emit_kernels` derive from chains (`grep -n "shaper" rust/motion-engine/src/planner.rs`); `update_shaper`/`PlannerMsg::UpdateShaper` → `update_post_processor(name: &str, key: &str, value: f64)` / `PlannerMsg::UpdatePostProcessor` — handler mutates the named instance, recompiles chains, rebuilds the replan context **for subsequent replans only**; unknown name/key → loud `PlannerError`
- Modify: `rust/motion-engine/src/bridge.rs`: pyo3 `init_planner` (grep `fn init_planner`) replaces `shaper_type_x/freq_x/y` args with `post_processors: Vec<(String, String, Vec<(String, f64)>)>` and per-axis `post_processors` lists arriving inside the existing `axes` arg; `update_shaper` pyfunction → `update_post_processor(name, key, value)`

- [ ] **Step 1: Write failing tests** in `config/tests.rs` (axis with unknown chain name; duplicate `[post_processor]` name; two kernels on one axis → `UnsupportedComposition` text mentions "v1"; happy path compiles `is`+`pa` on axis `e`).
- [ ] **Step 2: Run to verify failure** — `cargo nextest run -p motion-engine -E 'test(post_processor)'` → FAIL
- [ ] **Step 3: Implement**, deleting `ShaperConfig`/`AxisShaper` from `trajectory/src/lib.rs` once nothing references them (`rg -ln "ShaperConfig|AxisShaper" rust/` must come back empty outside git history).
- [ ] **Step 4: Run** — `cargo nextest run` (workspace) → PASS
- [ ] **Step 5: Commit** — `feat(motion-engine): [post_processor] chains replace ShaperConfig; runtime update_post_processor`

---

## Task 7: klippy — sections, rejection, `SET_POST_PROCESSOR`

**Files:**
- Modify: `klippy/motion_toolhead.py` — next to the existing `[axis]`/`[limit]` parsing (`grep -n "axis_sections\|limit_sections" klippy/motion_toolhead.py`): parse `[post_processor <name>]` (`type:` + numeric params, pass through verbatim — klippy validates nothing the bridge already validates); axis sections gain `post_processors:`; `_init_planner` ships both through the new `init_planner` signature
- Modify: `klippy/motion_engine.py` — `init_planner` wrapper matches Task 6's signature; `update_shaper` wrapper → `update_post_processor`
- Modify/Delete: `klippy/extras/input_shaper.py` — the section joins the rejected-legacy list: loading `[input_shaper]` raises a config error pointing at `[post_processor]` (follow the existing legacy-rejection pattern, `grep -n "is not supported" klippy/motion_toolhead.py`)
- Add: `SET_POST_PROCESSOR NAME=<name> <PARAM>=<VALUE>` gcode command in `motion_toolhead.py` → `bridge.update_post_processor(name, param.lower(), float(value))`; errors propagate to the console verbatim
- Modify: config fixtures — `rg -l "input_shaper" test/ config/ printer*.cfg` and the fixtures plan 2 touched (`git show cc8d5167e --stat` for the list); declare `[post_processor]` sections where shapers were configured
- Test: extend the Python suite where `[axis]`/`[limit]` parsing is tested (`rg -ln "axis_sections\|limit" test/klippy/ klippy/test* 2>/dev/null` — anchor to wherever plan 2's config tests live)

- [ ] **Step 1: Write failing Python tests** (same file as plan 2's section-parsing tests): missing-type `[post_processor]` fails; `[input_shaper]` rejected with message naming `[post_processor]`; axis referencing undeclared post-processor fails at connect; happy path reaches `init_planner` with the sections.
- [ ] **Step 2: Run to verify failure** — `./scripts/ci.sh py` → FAIL on the new tests
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run** — `./scripts/ci.sh py` → PASS
- [ ] **Step 5: Commit** — `feat(klippy): [post_processor] sections + SET_POST_PROCESSOR; [input_shaper] rejected`

---

## Task 8: runtime tuning end-to-end test

**Files:**
- Modify: `rust/motion-engine/src/planner/tests.rs` (or the integration-test home `rust/tests/` — match where planner-thread tests live: `rg -ln "update_shaper" rust/motion-engine/src/planner/tests.rs rust/tests/`)

- [ ] **Step 1: Write failing test:** start a planner with `pa` gain 0.0; submit extruding batch A; `update_post_processor("pa", "k", 0.05)`; submit batch B. Assert batch A's emitted follower pieces show no PA boost and batch B's do (compare follower velocity at mid-accel between batches); assert nothing already dispatched was re-emitted (piece counts/ids stable). Unknown name errors loudly.
- [ ] **Step 2: Run to verify failure** — `cargo nextest run -p motion-engine -E 'test(update_post_processor)'` → FAIL
- [ ] **Step 3: Implement** whatever plumbing gaps the test exposes (expected: none beyond Task 6).
- [ ] **Step 4: Run** — `cargo nextest run -p motion-engine` → PASS
- [ ] **Step 5: Commit** — `test(motion-engine): runtime post-processor tuning applies to new plans only`

---

## Task 9: lift `ExtrusionNotSupported`; follower demands and virtual paths in classify

**Files:**
- Modify: `rust/motion-engine/src/classify.rs` + `classify/tests.rs`
- Modify: `rust/motion-engine/src/bridge.rs` `submit_move` (grep `fn submit_move`) — resolve the follower axis index from the planner's `AxisRegistry` instead of assuming; `de ≠ 0` with no declared follower axis is a loud error (the registry rule, not a silent drop)

New classify contract:

```rust
pub fn classify_and_build(
    start: [f64; 3],
    dx: f64, dy: f64, dz: f64,
    followers: &[(usize, f64)],   // (axis_index, delta) — resolved by the caller from the registry
    feedrate_mm_s: f64,
) -> Result<ClassifiedMove, ClassifyError>
```

- spatial displacement present → ratio = `delta / distance_3d` per follower; `CubicSegment::try_new(xyz, followers, ...)` with `virtual_path_mm: None`
- no spatial displacement, follower delta present → virtual path: zero-displacement curve, `virtual_path_mm: Some(max |delta|)`, ratio = `delta / virtual_path` (spec §2's fallback line) — `MoveClass` gains `FollowerOnly` (or dies entirely if, after grepping consumers, nothing branches on it: `rg -n "MoveClass" rust/ klippy/`)
- no displacement at all → `ZeroDisplacement` unchanged
- `ClassifyError::ExtrusionNotSupported` deleted; the enum variant's absence must break every reference (`rg -n "ExtrusionNotSupported" rust/ klippy/` → empty after)

- [ ] **Step 1: Write failing tests** (extruding XY move carries ratio `de/dist`; retract-with-hop carries negative ratio over 3D length; follower-only move builds virtual path; `de≠0` without follower axis errors).
- [ ] **Step 2: Run to verify failure** — `cargo nextest run -p motion-engine -E 'test(classify)'` → FAIL
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run** — `cargo nextest run` (workspace) → PASS
- [ ] **Step 5: Commit** — `feat(motion-engine): extruding and follower-only moves flow; ExtrusionNotSupported dies`

---

## Task 10: lane-3 end-to-end + two-ledger assertions

**Files:**
- Modify: the end-to-end home `rust/runtime/tests/e2e_trajectory.rs` or `rust/tests/` (match existing extrusionless e2e idioms: `rg -ln "submit_move" rust/runtime/tests/ rust/tests/`)
- Modify: `rust/motion-engine/src/motion_history/tests.rs`

- [ ] **Step 1: Write failing tests:**
  - e2e: configured follower axis `e` follows x,y,z with a `pa` chain; submit a corner print path; assert `PushPieces` arrive with `axis_idx == 3`, the follower lane's `HistoryStore` endpoint equals the odometer prediction, and with a smoothing kernel on x/y the physical endpoint is **less** than the nominal ledger total (the accepted shortfall, spec §5) — assert the inequality and that nobody "corrects" it.
  - history: `state_at_clock` on the follower axis mid-move returns position between start and end, monotone for positive ratio.
- [ ] **Step 2: Run to verify failure** — `cargo nextest run -E 'test(e2e) and test(follower)'` → FAIL
- [ ] **Step 3: Implement** remaining wiring gaps (expected: MCU axis config for lane 3 in test harness fixtures — `rg -n "axes" rust/motion-engine/src/test_support.rs`).
- [ ] **Step 4: Run** — `cargo nextest run` → PASS
- [ ] **Step 5: Commit** — `test: follower lane end-to-end; physical-vs-nominal ledger shortfall asserted`

---

## Task 11: offline validation + gates

- [ ] **Step 1:** `./scripts/ci.sh quick` → green; `./scripts/ci.sh py` → green; `cargo test --doc` if any doc examples were touched.
- [ ] **Step 2:** klipper-sim sanity (see the `mcu-sim` skill): run a real sliced G-code file (G5-converted via `compat`) through the simulator on this branch; confirm follower lane pieces appear and total extrusion ≈ slicer total minus shaping shortfall. Record numbers in the PR description.
- [ ] **Step 3:** `cargo fmt --all --check`, then commit any stragglers and open/update the PR (base: `sota-motion`).

---

## Self-review notes (already applied)

- Spec §5 chain stages map: Task 4 (input track + chain + fit), Task 2 (odometer), Task 1+6 (post-processor registry), Task 9 (follower-only fallback line), Task 10 (two ledgers, MCU untouched — no MCU files appear anywhere in this plan).
- Plan/emission single-source rule (decision 5 of plan 3 + decision 1 here): Task 5 explicitly deletes separate `follower_pa`/`shaper` plumbing in favor of deriving both from `CompiledChain`.
- `ShaperConfig`/`AxisShaper`/`update_shaper`/`ExtrusionNotSupported`/`VirtualPathUnrouted` all have explicit deletion steps with grep-empty checks.
