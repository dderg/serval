# Planner Extension Implementation Plan (Plan 3)

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Extend the TOPP/SLP solver with the follower row families from the spec — follower velocity/accel/jerk rows, pressure-advance mixed-derivative rows, path-snap support, and input-shaper folding (follower rows written on the shaped combination of plan variables, with the committed tail entering as constants) — plus the follower-only-move degenerate path that plan 2 left as a fatal error.

**Architecture:** Spec: `docs/superpowers/specs/2026-06-12-follower-axes-and-limits-design.md` §4 (plus §2's follower-only rule). `temporal` gains follower axes (indices ≥ 3 in the existing `AxisSet` bitmask), per-segment `FollowerDemand { axis, ratio, pa_k }`, and a new row-family module `topp/follower.rs`. Without a shaper the rows are convex and join the base SOCP; with a shaper or PA they become iterate-linearized cuts in the existing SLP loop, built on a **frozen time map** (sample times computed from the current `b̄` iterate) that is re-frozen each outer iteration. Path snap (`s⁗`) is one more finite-difference order in `stencil.rs`. The committed tail enters as constant history terms in the window operator; trajectory's streaming layer extracts it from the shaped pieces it already retains for the freeze zone.

**Tech stack:** Rust (`temporal`, `trajectory`, `motion-bridge`, `geometry` crates; Clarabel SOCP). Tests: `cargo nextest run` from `rust/` (never bare `cargo test`); doc-tests via `cargo test --doc` if touched.

**PRECONDITION: Plan 2 (`docs/superpowers/plans/2026-06-12-axis-registry-reduce-simplification.md`) must be fully landed** — this plan builds on `geometry::FollowerDemand`, `CubicSegment.followers`, trajectory's `ShapeSegmentInput.followers`, and the bridge's `AxisRegistry` + follower-coverage validation. Verify before starting: `git log --oneline | head -20` shows plan 2's final commit (`feat: axis registry + follower segments end-to-end (plan 2)`) and `cargo nextest run` passes from `rust/`.

**All line numbers in this plan are pre-plan-2 approximations — anchor by symbol name and the given grep commands, never by line.**

**Out of scope (later plans):** emission of any follower track — odometer quadrature, PA application, the two ledgers, lifting the live-path `ExtrusionNotSupported` rejection (all plan 4); kinematics modules (plan 5); binding-constraint *reporting* through structured logs (plan 6 — but this plan's `BindingConstraint` variants are its groundwork); klippy config for PA gain (`pa_k` arrives through the temporal/trajectory API only, exercised by tests; the config key lands with emission in plan 4); mixed spatial+follower `[limit]` sets (stay rejected, deferred); nonlinear PA (the row builder takes a linear gain; the trait slot is plan 4's).

**Repo rules for every task:** unit tests in separate files from tested code; no explanatory comments — name/extract instead; fail loudly; commit after every task; no Claude/Anthropic commit trailers; `cargo fmt --all --check` before any PR push.

---

## Design decisions this plan makes (beyond the spec's text — review these first)

The spec writes the follower rows in scalar path-derivative form (`|ratio|·ṡ ≤ v_max` etc.) and says shaper folding writes them "on the shaped combination of plan variables." Making that executable required four concrete decisions:

1. **Per-axis windowed vectors, norm by supporting hyperplane.** X and Y may run different kernels (different shaper frequencies), and Z is typically passthrough — a single scalar convolution of `ṡ` cannot represent that. So the shaped follower demand is built per followed axis: shaped axis velocity `V_α(i) = Σ_j W^α_ij · c′_α,j·√b_j + hist`, and the speed demand is `|r|·‖V(i)‖`. On straights with one kernel this reduces exactly to the spec's scalar `K∗ṡ`. The norm is handled by supporting-hyperplane cuts at the iterate (the constraint is convex in the affine window expressions, so hyperplanes are valid outer cuts).
2. **Frozen time map.** Kernel delays are in time; solver samples are in path position; the time of sample `i` depends on the solution. Each follower outer iteration freezes `t̄_i` from the current `b̄`, builds the window weights `W` on it, solves, and re-freezes. Same fixed-point posture as the existing path-jerk and axis-jerk SLP phases, with the same divergence guards — capped iterations, loud `ScheduleError` on non-convergence. Follower cuts are **rebuilt from scratch whenever the time map is re-frozen** (cuts built on a stale `W` constrain the wrong operator).
3. **PA-jerk row stays in nominal path form.** `|r·(s⃛ + k·s⁗)| ≤ j_max` uses *path* snap with the identity window (no shaper folding on this one row). Folding it would require 4th-order axis geometry (`c⁗`) that nothing else needs. Since the smoothing kernel is an averaging operator, the nominal row is conservative — never under-constrains; it is exact on straights and slightly over-tight inside the shaper window at corners. Accepted for v1; tightening it later is additive.
4. **Cross-chain tails within a batch.** Chain junctions are full stops (plan 1), but the shaper window spans a stop — the neighbor chain's ramp contributes to shaped speed near the boundary. After the existing joining sweeps converge, one tail-exchange pass re-solves each chain with its neighbors' boundary-window velocity samples as constants, iterated to a small cap. The batch-boundary case (committed tail of the previous plan) enters the same way, supplied by trajectory's streaming layer.

A fifth, smaller one: **follower-only moves get a virtual path** — a chain whose spatial geometry is identically zero and whose arc length is the largest follower displacement; only follower rows bind, and the G-code feedrate caps `ṡ` as on any move (spec §2's "fallback line, not a mode").

---

## Row-family reference (used by every task below)

Solver variables per grid point `i`: `b_i = ṡ²`, `a_i = s̈`. Existing identities (stencil.rs):

```
ṡ = √b          s̈ = a          s⃛ = √b · b″ / 2        (b″ via b_dd_weights)
s⁗ = a·b″/2 + b·b‴/2                                   (b‴ via new b_ddd_weights; derivation:
                                                        s⁗ = d/dt(s⃛) = √b·d/ds(√b·b″/2)
                                                            = b′b″/4 + b·b‴/2, and b′ = 2a)
```

Frozen time map from the iterate `b̄`: `t̄_0 = 0`, `t̄_{i+1} = t̄_i + 2h_i/(√b̄_i + √b̄_{i+1})`.

Window operator for followed axis `α` with kernel `K_α` (support `[−h_k, +h_k]`, integral 1):
`W^α_ij = K_α(t̄_i − t̄_j) · q_j` with trapezoid quadrature weights `q_j = (t̄_{j+1} − t̄_{j−1})/2` (one-sided at the ends). History term `hist^α_i = Σ_m K_α(t̄_i − τ_m) · Δτ · v^α_hist(τ_m)` over supplied pre-chain samples (`τ_m < 0`). Passthrough kernel ⇒ `W = I`, `hist = 0` (the **identity window**).

Shaped per-axis quantities at sample `i` (affine or linearized in the solver variables):

```
V_α(i) = Σ_j W^α_ij · c′_α,j · √b_j                + hist_v^α_i     (√b linearized at b̄)
A_α(i) = Σ_j W^α_ij · (c″_α,j·b_j + c′_α,j·a_j)    + hist_a^α_i     (exactly affine)
J_α(i) = Σ_j W^α_ij · (c‴_α,j·b_j^{3/2} + 3c″_α,j·√b_j·a_j + c′_α,j·s⃛_j) + hist_j^α_i
                                                                     (linearized like the existing axis-jerk cuts)
```

Follower demand rows, for follower with ratio `r` and PA gain `k` (k = 0 when no PA), per limit set `S` covering that follower axis:

```
velocity:  |r| · (‖V(i)‖ + k·‖A(i)‖)  ≤ v_max(S)
accel:     |r| · (‖A(i)‖ + k·‖J(i)‖)  ≤ a_max(S)
jerk:      |r| · (‖J(i)‖ + k·|s⁗_i|)  ≤ j_max(S)      (s⁗ nominal — decision 3)
```

With the identity window and k = 0 these collapse to the spec's base rows `|r|·ṡ ≤ v_max` (a plain `b`-cap, convex), `|r|·|a_i| ≤ a_max` (two linear rows, convex), `|r|·|s⃛_i| ≤ j_max` (joins the path-jerk SLP with a per-point effective cap). Anything windowed or PA-shifted is a cut family: at the iterate, compute `û = V̄/max(‖V̄‖, ε)` (and the A/J analogues), emit the hyperplane row `|r|·Σ_α û_α·(affine V_α expr) + k·(…) ≤ rhs`. Linearizing `√b` at `b̄` (`√b ≈ √b̄/2 + b/(2√b̄)`) keeps every entry affine in `(b, a)`.

---

### Task 1: `temporal::Limits` — follower axes in the axis space

Axis indices 0–2 stay the spatial frame; 3..MAX_AXES are follower axes. A set is spatial xor follower; mixed is a loud error (deferred feature). Coverage validation extends to however many axes the machine declares.

**Files:**
- Modify: `rust/temporal/src/limits.rs`
- Modify: `rust/temporal/src/limits/tests.rs`
- Modify: every `Limits::try_new` / `AxisSet::all` caller — `grep -rn "Limits::try_new\|AxisSet::all" rust/`

- [ ] **Step 1: Write failing tests** (append to `limits/tests.rs`):

```rust
#[test]
fn follower_axis_coverage_is_validated() {
    let spatial = [
        set(&[0, 1], 300.0, 3000.0, 6000.0),
        set(&[2], 15.0, 100.0, 200.0),
    ];
    let err = Limits::try_new(&spatial, 4).unwrap_err();
    assert!(matches!(err, LimitsError::NoVelocityCoverage { axis: 3 }));

    let mut with_e = spatial.to_vec();
    with_e.push(set(&[3], 75.0, 1500.0, 3000.0));
    let lim = Limits::try_new(&with_e, 4).unwrap();
    assert_eq!(lim.follower_sets().count(), 1);
    assert_eq!(lim.spatial_sets().count(), 2);
}

#[test]
fn mixed_spatial_follower_set_is_rejected() {
    let sets = [
        set(&[0, 1, 2], 300.0, 3000.0, 6000.0),
        set(&[0, 3], 10.0, 100.0, 200.0),
    ];
    assert!(matches!(
        Limits::try_new(&sets, 4).unwrap_err(),
        LimitsError::MixedSpatialFollower { set: 1 }
    ));
}

#[test]
fn three_axis_construction_is_unchanged() {
    let lim = Limits::axis_boxes([300.0; 3], [3000.0; 3], [6000.0; 3]);
    assert_eq!(lim.sets().len(), 3);
    assert_eq!(lim.follower_sets().count(), 0);
}

#[test]
fn spatial_helpers_reject_follower_sets() {
    let s = set(&[3], 75.0, 1500.0, 3000.0);
    assert!(!s.axes.is_spatial());
}
```

- [ ] **Step 2: Run to verify failure** — `cargo nextest run -p temporal -E 'test(limits)'` → FAIL

- [ ] **Step 3: Implement.** In `limits.rs`:
  - `pub const N_SPATIAL: usize = 3;` `pub const MAX_AXES: usize = 8;` (the `AxisSet(u8)` bitmask already holds 8).
  - `AxisSet`: add `pub fn is_spatial(self) -> bool { self.0 < (1 << N_SPATIAL) }` and `pub fn is_follower(self) -> bool { self.0 >> N_SPATIAL != 0 && self.0 & ((1 << N_SPATIAL) - 1) == 0 }`; rename `AxisSet::all()` → `AxisSet::spatial()` (it means "all spatial" everywhere it is used — fix callers, `grep -rn "AxisSet::all" rust/`).
  - `Limits::try_new(sets: &[LimitSet], n_axes: usize)`: assert `N_SPATIAL <= n_axes && n_axes <= MAX_AXES`; reject any set that is neither `is_spatial` nor `is_follower` with new `LimitsError::MixedSpatialFollower { set: usize }`; run the existing per-derivative coverage loop over `0..n_axes` instead of `0..MAX_AXES`. Store `n_axes: u8` on `Limits` with a getter.
  - Add `pub fn spatial_sets(&self) -> impl Iterator<Item = (usize, &LimitSet)>` and `follower_sets(&self)` (enumerated with their set index, for `BindingConstraint` attribution).
  - `axis_boxes` / `norm_all` call `try_new(_, N_SPATIAL)`.
  - `restricted_norm`, `kappa_set`, `mvc_b`, `a_tan_cap`, `j_tan_cap`, `b_cent_cap`: these index `[f64; 3]` geometry — add `debug_assert!(axes.is_spatial())` at the top of `restricted_norm`/`kappa_set` and make every `Limits` geometry helper iterate `self.spatial_sets()` instead of `self.sets()`. Follower sets must be physically unreachable from spatial geometry code.
  - `scale_limits` in `scaling.rs` rebuilds via `try_new(_, limits.n_axes())` — port.
  - Every existing `try_new` caller gains `, 3` (or `N_SPATIAL`) — mechanical, `grep -rn "Limits::try_new" rust/`.

- [ ] **Step 4: Run** — `cargo nextest run -p temporal` → PASS (zero behavioral change for 3-axis limits).
- [ ] **Step 5: Commit** — `feat(temporal): follower axes in the limit axis space; mixed sets rejected`

---

### Task 2: follower demands into `SegmentInput` and `ChainGrid`

`temporal` must not depend on `geometry`, so it gets its own demand type. A segment with no follower motion carries an empty slice — rows are simply not emitted.

**Files:**
- Modify: `rust/temporal/src/lib.rs` (new type + export)
- Modify: `rust/temporal/src/multi/mod.rs` (`SegmentInput`), `rust/temporal/src/topp/chain.rs` (`ChainGrid`), `rust/temporal/src/topp/mod.rs` (`schedule_segment_*` plumbing)
- Modify: callers — `grep -rn "SegmentInput {" rust/`
- Test: `rust/temporal/src/topp/chain/tests.rs` (or wherever `from_segment_grids` tests live — `grep -rln "from_segment_grids" rust/temporal/`)

- [ ] **Step 1: Write failing test** — a two-segment chain built from inputs with different follower ratios exposes them per grid point:

```rust
#[test]
fn chain_carries_per_segment_follower_demands() {
    // build two segment grids as the existing from_segment_grids tests do,
    // with followers = [FollowerDemand { axis: 3, ratio: 0.05, pa_k: 0.0 }]
    // on the first and [] on the second;
    // assert chain.followers_at(i) == first slice for i in the first
    // segment_range and is empty in the second.
}
```

(Flesh out with the existing harness idioms in the chain tests — grid construction is already demonstrated there.)

- [ ] **Step 2: Run to verify failure** — `cargo nextest run -p temporal -E 'test(follower_demands)'` → FAIL

- [ ] **Step 3: Implement.** In `lib.rs`:

```rust
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FollowerDemand {
    pub axis: usize,
    pub ratio: f64,
    pub pa_k: f64,
}
```

`SegmentInput` gains `pub followers: &'a [FollowerDemand]`. `ChainGrid` gains `pub followers: Vec<Vec<FollowerDemand>>` indexed like `limits` (one entry per limits_idx / segment), plus `pub fn followers_at(&self, i: usize) -> &[FollowerDemand]` mirroring `limits_at`. `from_segment_grids` threads the slices through; the single-segment constructors take `&[FollowerDemand]` (existing callers pass `&[]`). Validate at chain construction: every `axis` must be `>= N_SPATIAL`, `< limits.n_axes()`, `ratio` finite, `pa_k` finite and `>= 0.0` — violation is a panic-free constructor error (fail loudly, new `ChainError` variant or extend the existing one — `grep -n "enum.*Error" rust/temporal/src/topp/chain.rs`).

- [ ] **Step 4: Run** — `cargo nextest run -p temporal` → PASS
- [ ] **Step 5: Commit** — `feat(temporal): segments carry follower demands into the chain`

---

### Task 3: window operator + frozen time map (standalone math module)

The reusable core: time map from an iterate, kernel sampling, history folding, identity degenerate. No solver integration yet — pure functions, heavily unit-tested.

**Files:**
- Create: `rust/temporal/src/topp/window.rs` (+ `mod window;` in `topp/mod.rs`)
- Create: `rust/temporal/src/topp/window/tests.rs`

- [ ] **Step 1: Write failing tests:**

```rust
use super::*;

#[test]
fn time_map_of_constant_speed_is_uniform() {
    let b = vec![4.0; 5];
    let h = vec![1.0; 4];
    let t = frozen_time_map(&b, &h);
    for (i, ti) in t.iter().enumerate() {
        assert!((ti - 0.5 * i as f64).abs() < 1e-12);
    }
}

#[test]
fn identity_window_is_identity() {
    let w = WindowOperator::identity(5);
    let signal = [1.0, 2.0, 3.0, 4.0, 5.0];
    for i in 0..5 {
        let row = w.row(i);
        let applied: f64 = row.weights.iter().map(|&(j, wj)| wj * signal[j]).sum();
        assert!((applied + row.history - signal[i]).abs() < 1e-12);
    }
}

#[test]
fn kernel_window_weights_sum_to_one_in_the_interior() {
    // constant speed 100 mm/s over 200 samples, smooth-zv kernel at 40 Hz:
    // any interior row's weights (plus zero history) must sum to ~1, so a
    // constant signal is reproduced.
    let kernel = test_bell_kernel(40.0);
    let b = vec![100.0_f64.powi(2); 200];
    let h = vec![0.1; 199];
    let t = frozen_time_map(&b, &h);
    let w = WindowOperator::from_kernel(&kernel, &t, &WindowHistory::empty());
    let mid = w.row(100);
    let total: f64 = mid.weights.iter().map(|&(_, wj)| wj).sum();
    assert!((total - 1.0).abs() < 2e-3, "got {total}");
}

#[test]
fn history_supplies_the_left_edge() {
    // near sample 0 the kernel reaches before the chain; with a constant
    // history at the same speed, row 0 must still reproduce the constant.
    let kernel = test_bell_kernel(40.0);
    let b = vec![100.0_f64.powi(2); 200];
    let h = vec![0.1; 199];
    let t = frozen_time_map(&b, &h);
    let hist = WindowHistory::constant_speed(100.0, kernel_half_support(&kernel), 64);
    let w = WindowOperator::from_kernel(&kernel, &t, &hist);
    let row0 = w.row(0);
    let interior: f64 = row0.weights.iter().map(|&(_, wj)| wj).sum::<f64>() * 100.0;
    assert!((interior + row0.history_scale * 100.0 /*…see Step 3 shape…*/ - 100.0).abs() < 1.0);
}

#[test]
fn right_edge_extends_with_terminal_hold() {
    // mirror of the left edge: beyond the last sample the signal is held at
    // the terminal value; an interior-constant signal stays reproduced at the
    // final row.
}
```

(`test_bell_kernel` builds the same `w(t) = c·(h²−t²)²` kernel as `trajectory/src/kernel.rs::build_smooth_zv_kernel` — copy the five coefficients into the test helper; `temporal` already depends on `nurbs`, which owns `PiecewisePolynomialKernel`.)

- [ ] **Step 2: Run to verify failure** — `cargo nextest run -p temporal -E 'test(window)'` → FAIL

- [ ] **Step 3: Implement** in `window.rs`:

```rust
pub fn frozen_time_map(b: &[f64], h_intervals: &[f64]) -> Vec<f64> {
    let mut t = Vec::with_capacity(b.len());
    let mut acc = 0.0;
    t.push(0.0);
    for i in 0..h_intervals.len() {
        let v_sum = b[i].max(0.0).sqrt() + b[i + 1].max(0.0).sqrt();
        assert!(v_sum > 0.0, "frozen time map: zero speed across interval {i}");
        acc += 2.0 * h_intervals[i] / v_sum;
        t.push(acc);
    }
    t
}

#[derive(Debug, Clone)]
pub struct WindowRow {
    pub weights: Vec<(usize, f64)>,
    pub history: f64,
}

#[derive(Debug, Clone)]
pub struct WindowOperator {
    rows: Vec<WindowRow>,
}

#[derive(Debug, Clone)]
pub struct WindowHistory {
    pub dt: f64,
    pub samples: Vec<f64>,
}
```

`WindowOperator::identity(n)` → row `i` is `{ weights: vec![(i, 1.0)], history: 0.0 }`. `WindowOperator::from_kernel(kernel, t_map, history)`: for each target `i`, collect `(j, K(t_i − t_j)·q_j)` over sources within the kernel support (trapezoid `q_j = (t[j+1] − t[j−1])/2`, one-sided at the edges); fold `history.samples` (timestamped at `−m·dt` going backwards from `t = 0`) into the constant `history` term; extend past the right edge by holding the last source sample (add the leftover kernel mass `(1 − Σ weights − history_mass)` onto the final source index). `history` here is the already-summed scalar `Σ_m K(t_i + m·dt)·dt·sample_m` — the caller pre-multiplies the history *signal values*, so the operator stores per-row constants per signal kind (the caller calls `from_kernel` once per signal kind with the matching history samples; velocity/accel/jerk histories are separate `WindowHistory` values).

Resolve the exact shape during implementation — the tests above pin the semantics (constant reproduction at interior, left edge with history, right edge with terminal hold); adjust the test helper calls to the final signature, never weaken the assertions.

- [ ] **Step 4: Run** — `cargo nextest run -p temporal -E 'test(window)'` → PASS
- [ ] **Step 5: Commit** — `feat(temporal): frozen-time-map window operator with history folding`

---

### Task 4: base follower velocity/accel rows (identity window, no PA) in the SOCP

Convex case first: `|r|·√b ≤ v_max` is a `b`-cap row; `|r|·|a_i| ≤ a_max` is a sign pair. New module owns follower row emission; `build_chain` calls into it.

**Files:**
- Create: `rust/temporal/src/topp/follower.rs` (+ `mod follower;`)
- Create: `rust/temporal/src/topp/follower/tests.rs`
- Modify: `rust/temporal/src/topp/constraints.rs` (`build_chain` calls the new emitter; find the velocity block with `grep -n "v_max" rust/temporal/src/topp/constraints.rs`)
- Modify: `rust/temporal/src/lib.rs` / `rust/temporal/src/topp/verify.rs` (`BindingConstraint`, `ratios_at`)

- [ ] **Step 1: Write failing solver-level tests** in `follower/tests.rs` (use the existing single-segment scheduling harness idioms — `grep -rn "schedule_segment_with_tolerance" rust/temporal/` for examples):

```rust
#[test]
fn follower_velocity_caps_cruise_speed() {
    // straight 100 mm line, gantry limits generous (v=500), follower set
    // axis 3 with v_max = 50, demand ratio 0.5 ⇒ path cruise must cap at
    // 50 / 0.5 = 100 mm/s. Assert peak profile velocity ≈ 100 within 1%.
}

#[test]
fn follower_accel_caps_path_accel() {
    // same line, follower a_max = 500, ratio 0.5 ⇒ path accel ≤ 1000.
    // Assert max |dv/dt| across profile samples ≤ 1000 * 1.01.
}

#[test]
fn zero_ratio_emits_nothing() {
    // followers = [] and followers = [ratio 0.0 rejected by chain validation]:
    // profile identical to the no-follower solve (compare total_time bitwise-ish, 1e-12).
}

#[test]
fn binding_tag_names_the_follower_set() {
    // in the velocity-capped case, verify report at a cruise sample tags
    // BindingConstraint::Velocity { set: <index of the follower set> }.
}
```

- [ ] **Step 2: Run to verify failure** — `cargo nextest run -p temporal -E 'test(follower)'` → FAIL

- [ ] **Step 3: Implement emission** in `follower.rs`:

```rust
pub(crate) fn emit_base_follower_rows(
    chain: &ChainGrid,
    off_b: usize,
    off_a: usize,
    push_row: …,           // same closure/buffer plumbing as the existing families
) -> usize {
    let mut count = 0;
    for i in 0..chain.s.len() {
        let lim = chain.limits_at(i);
        for f in chain.followers_at(i) {
            if f.pa_k != 0.0 {
                continue; // PA rows are Task 7's cut family
            }
            let r = f.ratio.abs();
            for (set_idx, set) in lim.follower_sets() {
                if !set.axes.contains(f.axis) {
                    continue;
                }
                if set.v_max.is_finite() {
                    let cap = (set.v_max / r).powi(2);
                    push_row(&[(off_b + i, -1.0)], cap);
                    count += 1;
                }
                if set.a_max.is_finite() {
                    push_row(&[(off_a + i, -r)], set.a_max);
                    push_row(&[(off_a + i, r)], set.a_max);
                    count += 2;
                }
                let _ = set_idx;
            }
        }
    }
    count
}
```

Wire into `build_chain` next to the existing velocity family (inside the Nonneg run accounting — these are all Nonneg rows). Multiple followers and multiple covering sets simply emit more rows; rows intersect, no precedence.

**verify.rs:** extend `ratios_at` inputs with the point's `&[FollowerDemand]`; for each demand and covering follower set push entries `(r·ṡ / v_max, Velocity { set })`, `(r·|s̈| / a_max, AccelNorm { set })`, `(r·|s⃛| / j_max, JerkNorm { set })`. The existing variants carry set indices already (plan 1) — no new variants needed for the base rows. `check_chain` passes `chain.followers_at(i)` through.

- [ ] **Step 4: Run** — `cargo nextest run -p temporal` → PASS
- [ ] **Step 5: Commit** — `feat(temporal): base follower velocity/accel rows with binding attribution`

---

### Task 5: follower jerk via the path-jerk SLP (per-point effective cap)

`|r|·|s⃛| ≤ j_max` is exactly the existing path-jerk constraint with cap `j_max/|r|` — but `r` varies per segment, so the global `j_path` scalar becomes a per-point envelope.

**Files:**
- Modify: `rust/temporal/src/topp/constraints.rs` (the `j_path` computation — `grep -n "j_path" rust/temporal/src/topp/`)
- Modify: `rust/temporal/src/topp/solver.rs` (`find_jerk_violators_chain`, `append_path_jerk_cut_weights` — these consume `j_path`)
- Test: `rust/temporal/src/topp/follower/tests.rs`

- [ ] **Step 1: Write failing test:**

```rust
#[test]
fn follower_jerk_cap_binds_through_the_slp() {
    // straight line, follower jerk j_max = 1000, ratio 0.5 ⇒ effective path
    // jerk cap 2000, well below the spatial sets' caps. Solve; run verify;
    // assert worst jerk ratio ≤ 1 + 5e-2 and that the profile's measured
    // |s⃛| (finite-differenced from samples) stays ≤ 2000 * 1.05.
}
```

- [ ] **Step 2: Run to verify failure** — `cargo nextest run -p temporal -E 'test(follower_jerk)'` → FAIL (cap ignored)

- [ ] **Step 3: Implement.** Replace the scalar `j_path` with a per-point vector `j_path_at: Vec<f64>`:

```rust
let j_path_at: Vec<f64> = (0..n)
    .map(|i| {
        let mut cap = chain.limits_at(i).j_path();
        for f in chain.followers_at(i) {
            if f.pa_k != 0.0 {
                continue;
            }
            for (_, set) in chain.limits_at(i).follower_sets() {
                if set.axes.contains(f.axis) && set.j_max.is_finite() {
                    cap = cap.min(set.j_max / f.ratio.abs());
                }
            }
        }
        cap
    })
    .collect();
```

Thread it through the bundle to `find_jerk_violators_chain` (violation test uses `j_path_at[i]`) and `append_path_jerk_cut_weights` (cut target uses the same). Where the auxiliary `t_k` rows in `build_chain` divide by `j_path`, use `j_path_at[k + 1]` (the interior point the row anchors). The all-spatial case reduces to the old scalar at every point — the existing suite must pass unchanged.

- [ ] **Step 4: Run** — `cargo nextest run -p temporal` → PASS
- [ ] **Step 5: Commit** — `feat(temporal): per-point path-jerk envelope; follower jerk caps join the SLP`

---

### Task 6: path snap support

`s⁗ = a·b″/2 + b·b‴/2` needs third-difference weights for `b`. Implement Fornberg's finite-difference weight algorithm (exact for arbitrary nonuniform stencils) rather than hand-derived closed forms.

**Files:**
- Modify: `rust/temporal/src/topp/stencil.rs`
- Modify: stencil tests — `grep -rln "b_dd_weights" rust/temporal/` for where they live
- Modify: `rust/temporal/src/topp/verify.rs` (expose `s_ddddot_at` for later consumers)

- [ ] **Step 1: Write failing tests:**

```rust
#[test]
fn b_ddd_weights_exact_on_cubics() {
    // b(s) = s³ on points s = {0.0, 0.7, 1.5, 2.6}: third derivative is 6.0
    // everywhere; the 4-point weights must reproduce it to 1e-9.
}

#[test]
fn s_snap_on_constant_jerk_profile() {
    // construct b(s) so that s(t) has constant jerk J (b = (v0 + …)² sampled
    // on a uniform grid is messy — instead verify the chain rule directly:
    // pick b(s) = (1 + s)⁴ ⇒ ṡ = (1+s)², s̈ = 2(1+s)³, s⃛ = 6(1+s)⁴,
    // s⁗ = 24(1+s)⁵; evaluate s_ddddot_at_weights on a fine grid at an
    // interior point and assert within 1% of the analytic value.
}
```

- [ ] **Step 2: Run to verify failure** — `cargo nextest run -p temporal -E 'test(snap) or test(b_ddd)'` → FAIL

- [ ] **Step 3: Implement** in `stencil.rs`: a small Fornberg weight generator

```rust
pub fn fornberg_weights(x0: f64, xs: &[f64], order: usize) -> Vec<Vec<f64>>
```

(standard recurrence, ~25 lines — returns weights for derivatives 0..=order at `x0` over nodes `xs`; cite none, derive none: the recurrence is `c1 = 1; for j in 1..n { … }` per Fornberg 1988, implement from the recurrence in any numerical-methods reference and pin it with the exactness tests above). Then:

```rust
pub fn b_ddd_weights_at(i: usize, s: &[f64]) -> ([usize; 4], [f64; 4])
```

choosing the 4-point stencil `{i-1, i, i+1, i+2}` clamped at the boundaries, and

```rust
pub fn s_ddddot_at_weights(b: &[f64], a_i: f64, i: usize, s: &[f64], h_intervals: &[f64]) -> f64 {
    let b_dd = /* existing b_dd_weights path */;
    let (idx, w) = b_ddd_weights_at(i, s);
    let b_ddd = (0..4).map(|k| w[k] * b[idx[k]]).sum::<f64>();
    a_i * b_dd / 2.0 + b[i].max(0.0) * b_ddd / 2.0
}
```

Verify the existing `b_dd_weights` against `fornberg_weights` in a test (they must agree to 1e-12) — if they do, optionally collapse the old closed form onto Fornberg in a follow-up, not now.

- [ ] **Step 4: Run** — `cargo nextest run -p temporal` → PASS
- [ ] **Step 5: Commit** — `feat(temporal): path snap via Fornberg third-difference weights`

---

### Task 7: PA rows as an SLP cut family (identity window)

PA shifts each demand one derivative up: `|r|(ṡ + k·s̈) ≤ v_max`, `|r|(s̈ + k·s⃛) ≤ a_max`, `|r|(s⃛ + k·s⁗) ≤ j_max`. All contain nonlinear-in-`b` terms (`√b`, `s⃛`, `s⁗`) ⇒ cuts in the SLP, reusing the axis-jerk trust-region loop.

**Files:**
- Modify: `rust/temporal/src/topp/follower.rs` (violator scan + cut builder)
- Modify: `rust/temporal/src/topp/solver.rs` (`SlpCut` gains a `Follower` variant; the SLP9 loop's cut-collection call sites — `grep -n "build_axis_jerk_cuts_chain\|SlpCut" rust/temporal/src/topp/solver.rs`)
- Modify: `rust/temporal/src/lib.rs` (`BindingConstraint` gains `PaVelocity { set }`, `PaAccel { set }`, `PaJerk { set }`), `verify.rs` (`ratios_at` PA entries — demand formulas above, `s⁗` via Task 6)
- Test: `rust/temporal/src/topp/follower/tests.rs`

- [ ] **Step 1: Write failing tests:**

```rust
#[test]
fn pa_velocity_row_slows_the_accel_phase() {
    // straight line, follower v_max = 50, ratio 0.5, pa_k = 0.05.
    // During cruise: demand = 0.5·ṡ ≤ 50 ⇒ ṡ ≤ 100 (as Task 4).
    // During accel at s̈ = A: demand = 0.5·(ṡ + 0.05·A) — the profile must
    // hold 0.5·(ṡ + 0.05·s̈) ≤ 50·(1+5e-2) at every sample (assert by
    // finite-differencing the solved profile).
}

#[test]
fn pa_accel_row_holds_pointwise() {
    // same setup, follower a_max = 500: assert 0.5·|s̈ + 0.05·s⃛| ≤ 500·1.05
    // at every interior sample of the solved profile.
}

#[test]
fn pa_jerk_row_holds_pointwise() {
    // follower j_max = 5000: assert 0.5·|s⃛ + 0.05·s⁗| ≤ 5000·1.05 pointwise.
}

#[test]
fn verify_tags_pa_rows() {
    // in the first test's accel phase, the verify report's binding tag at the
    // worst sample is BindingConstraint::PaVelocity { set } for the follower set.
}
```

- [ ] **Step 2: Run to verify failure** — `cargo nextest run -p temporal -E 'test(pa_)'` → FAIL

- [ ] **Step 3: Implement.**
  - **Demand evaluation at the iterate** (shared with verify): for point `i`, `d_v = r·(√b̄ + k·ā)`, `d_a = r·(ā + k·s⃛(b̄))`, `d_j = r·(s⃛(b̄) + k·s⁗(b̄, ā))`. Ratios against the covering follower sets' caps.
  - **Violator scan** `find_follower_violators(chain, b̄, ā) -> Vec<FollowerViolator { i, set, family, ratio }>` — same shape as `find_jerk_violators_chain`, threshold `1 + SLP9_EPS_FEAS`.
  - **Cut builder**: linearize each demand at the iterate. Every term is already linearized elsewhere in the codebase — reuse the gradient pieces: `√b → b/(2√b̄) + √b̄/2`; `s⃛ → (√b̄·δb″ + b̄″·δb_i/(2√b̄))/2` expanded over the `b_dd` stencil exactly as `append_path_jerk_cut_weights` does; `s⁗` gradient over the 4-point `b_ddd` stencil plus the `a·b″/2` cross terms (gradient w.r.t. `a_i` is `b̄″/2`; w.r.t. stencil `b`s via the two weight sets). Emit two Nonneg rows (±) per violator with the same `row_scale` conditioning as `append_axis_jerk_cut_to_clarabel` — follow that function's structure literally, swapping the gradient terms.
  - **`SlpCut::Follower(FollowerCut)`** where `FollowerCut` stores the prebuilt sparse entries + rhs pair, so `solve_with_cuts` appends it without follower knowledge.
  - **Loop placement**: extend the SLP9 outer loop's cut-collection step to also call `find_follower_violators`/`build_follower_cuts` and merge the cut lists; the existing trust-region, backtracking, target-decay, and divergence machinery applies unchanged. Worst-ratio bookkeeping takes the max over axis-jerk and follower families.
  - **verify.rs**: PA entries per demand with the new `BindingConstraint` variants; tie-break order: existing classes first, then `PaVelocity > PaAccel > PaJerk`; PA-jerk joins the jerk class (`max_jerk`), PA-velocity/accel the non-jerk class.

- [ ] **Step 4: Run** — `cargo nextest run -p temporal` → PASS (including all pre-existing SLP tests — the follower scan returns empty when no demands exist).
- [ ] **Step 5: Commit** — `feat(temporal): pressure-advance rows as SLP cuts with snap-backed jerk`

---

### Task 8: shaper folding — windowed follower rows with history constants

The deep step. When any followed axis has a non-passthrough kernel, the Task-4 base rows and Task-7 cuts for that follower are replaced by windowed ones built on the frozen time map; an outer fixed-point loop re-freezes the map until it stabilizes. Kernels and history enter through new chain-level inputs.

**Files:**
- Modify: `rust/temporal/src/topp/chain.rs` (`ChainGrid` gains `pub axis_kernels: [Option<PiecewisePolynomialKernel<f64>>; 3]`, `pub follower_history: Option<FollowerHistory>`)
- Modify: `rust/temporal/src/lib.rs`:

```rust
#[derive(Debug, Clone, Default)]
pub struct FollowerHistory {
    pub dt: f64,
    pub axis_velocity: [Vec<f64>; 3],
}
```

- Modify: `rust/temporal/src/topp/follower.rs` (windowed demand evaluation + cuts), `rust/temporal/src/topp/solver.rs` (outer re-freeze loop), `rust/temporal/src/topp/window.rs` (whatever Task 3's shape needs to serve per-axis signals)
- Modify: `rust/temporal/src/multi/mod.rs` + `multi/joining.rs` (tail exchange — Step 5)
- Test: `rust/temporal/src/topp/follower/tests.rs`

- [ ] **Step 1: Write failing tests:**

```rust
#[test]
fn passthrough_kernels_reproduce_identity_rows() {
    // same problem as Task 4's velocity test but with axis_kernels =
    // [None, None, None] explicitly: total_time equal to the Task-4 result
    // within 1e-9.
}

#[test]
fn folded_rows_recover_speed_at_a_smoothed_start() {
    // One chain accelerating from rest, follower v_max tight, X kernel active.
    // The shaped speed lags the nominal during the ramp (averaging with the
    // zero history), so the windowed velocity row binds LATER than the
    // identity row would: total_time(folded) ≤ total_time(identity) − margin.
    // Pick margin from a first run; assert the inequality strictly, then pin.
}

#[test]
fn folded_demand_holds_against_brute_force_convolution() {
    // Ground truth: take the solved profile, build the continuous nominal
    // axis-velocity signals from the profile samples + chain geometry,
    // numerically convolve with the kernel at 1 kHz sampling (the same math
    // as trajectory's ShapedSignal::eval), compute r·‖shaped v⃗‖ pointwise,
    // and assert ≤ v_max·(1 + 5e-2) everywhere. THIS is the test that
    // makes the whole folding claim falsifiable.
}

#[test]
fn nonzero_history_constrains_the_chain_start() {
    // Same chain but follower_history = constant 100 mm/s on X for the full
    // window width, follower v_max tight: the windowed speed at sample 0 now
    // includes the history mass, so the feasible start speed drops. Assert
    // the brute-force check (as above, with the history signal prepended)
    // still holds ≤ v_max·1.05.
}

#[test]
fn refreeze_divergence_fails_loudly() {
    // Force non-convergence by setting the refreeze cap to 1 via a test-only
    // constructor/config knob and giving a problem whose time map shifts
    // (kernel active, strong accel): expect Err(ScheduleError::FollowerSlpDiverged).
}
```

- [ ] **Step 2: Run to verify failure** — `cargo nextest run -p temporal -E 'test(folded) or test(refreeze)'` → FAIL

- [ ] **Step 3: Windowed demand machinery** in `follower.rs`:
  - `build_follower_windows(chain, b̄) -> FollowerWindows`: time map via `frozen_time_map`; per followed axis `α`, `WindowOperator::from_kernel(kernel_α, …)` or `identity`; history terms per signal kind from `FollowerHistory` (velocity history given; accel/jerk histories finite-differenced from the velocity samples — document that choice in the function name, e.g. `history_accel_from_velocity`).
  - Windowed demands at the iterate: `V_α(i)`, `A_α(i)`, `J_α(i)` per the reference section, evaluated numerically from `(b̄, ā)`; demands `d_v = r(‖V‖ + k‖A‖)`, `d_a = r(‖A‖ + k‖J‖)`, `d_j = r(‖J‖ + k|s⁗|)`.
  - Cut builder: hyperplane directions `û = V̄/max(‖V̄‖, FLOOR)` etc.; row entries = `|r|·Σ_α û_α · ∂V_α/∂(b_j, a_j)` over the window's source samples (affine pieces per the reference; `√b` and `s⃛` linearized at `b̄` exactly as in Task 7). One Nonneg row per violator (the hyperplane already encodes the binding side; emit the ± pair only for the scalar `s⁗` tail).
  - When every followed axis is passthrough **and** `pa_k == 0`, Task 4's static rows remain the emission path (they are exact and convex — never replace exact with linearized when the exact form exists). The follower SLP scan skips such demands.

- [ ] **Step 4: Re-freeze outer loop** in `solver.rs`, wrapping the existing phases:

```text
solve base+jerk SLP (existing)                       — iterate 0
loop up to FOLLOWER_REFREEZE_MAX (8):
    t̄ ← frozen_time_map(b̄);  W ← build_follower_windows
    inner: existing SLP9-style cut loop, with follower cuts built on THIS W
           (follower cuts from previous freezes are discarded; path-jerk and
            axis-jerk cuts persist as today)
    drift ← max_i |t̄_new_i − t̄_i| relative to the kernel half-support
    if drift < REFREEZE_DRIFT_TOL (1e-2) and worst follower ratio ≤ 1 + SLP9_EPS_FEAS:
        converged
fail: ScheduleError::FollowerSlpDiverged { refreezes, worst_ratio }
```

Chains with no active windows (no kernels, no PA) skip the wrapper entirely — zero cost on today's paths.

- [ ] **Step 5: Cross-chain tail exchange** in `multi/joining.rs`: after `join_until_converged` returns, if any chain has active windows, run up to `TAIL_EXCHANGE_MAX (3)` passes: for each chain, sample its neighbors' solved boundary-window velocity signals (per axis, from the neighbor profile within one kernel width of the shared stop) into a `FollowerHistory` (left neighbor → history; right neighbor → terminal extension samples — add a mirrored `follower_terminal` field alongside `follower_history` if the right side needs more than terminal-hold), re-solve dirty chains, stop when no chain's total time moved by more than 0.1%. Junction velocities stay 0 — only the window constants change. If the pass cap is hit, fail loudly (`BatchError` variant naming the junction). The batch-boundary (streaming) history arrives via `BatchInput` → first chain's `follower_history` (Task 10).

- [ ] **Step 6: verify.rs** — windowed demands enter `check_chain` through the same evaluation functions (verify receives the final frozen windows from the solve, not a fresh map — expose them on the output bundle), so the report covers folded rows.

- [ ] **Step 7: Run** — `cargo nextest run -p temporal` → PASS
- [ ] **Step 8: Commit** — `feat(temporal): shaper-folded follower rows on a frozen time map with history constants`

---

### Task 9: follower-only moves — virtual path

Plan 2 made these `Fatal::FollowerOnlyMoveUnsupported`. Now: the move's path length is the largest follower displacement; spatial geometry is identically zero; G-code feedrate and the follower's own rows cap it.

**Files:**
- Modify: `rust/geometry/src/pipeline.rs` (`classify_followers` — the `FollowerOnlyMoveUnsupported` arm), `rust/geometry/src/segment.rs` (carry the virtual length), `rust/geometry/src/error.rs` (delete `FollowerOnlyMoveUnsupported` + the `Fatal` variant)
- Modify: `rust/temporal/src/topp/chain.rs` (virtual-path chain constructor) and `rust/trajectory/src/` plumbing (`grep -rn "FollowerOnlyMoveUnsupported" rust/` for every consumer)
- Tests: `rust/geometry/src/pipeline/tests.rs`, `rust/temporal/src/topp/follower/tests.rs`

- [ ] **Step 1: Write failing tests:**

```rust
// geometry/pipeline tests: the plan-2 follower_only_move_is_fatal test flips —
// the same two-line G-code now yields a segment with
// virtual_path_mm == Some(3.2) (the |E| displacement) and
// followers == [FollowerDemand { axis_index: 3, ratio: -1.0 }].

// temporal follower tests:
#[test]
fn virtual_path_plans_under_follower_limits_and_feedrate() {
    // virtual path L = 10 mm, follower set v_max = 75, a_max = 1500,
    // feedrate 40 mm/s ⇒ cruise = min(40, 75) = 40; with feedrate 200 ⇒ 75.
    // Assert both, plus accel phase ≤ 1500 (ratio = 1).
}
```

- [ ] **Step 2: Run to verify failure** — scoped nextest → FAIL

- [ ] **Step 3: Implement.**
  - `CubicSegment` gains `pub virtual_path_mm: Option<f64>`; `try_new` validates: if `Some(l)`, `l > 0.0` finite, the xyz curve must have zero displacement (all control points equal within 1e-9 — assert, fail loudly), and `followers` non-empty.
  - `classify_followers`: the `path_len <= EPS_PATH && any_follower_motion` arm returns the virtual classification: `L = max |delta|` over followers, ratios `delta_i / L`, `virtual_path_mm: Some(L)` flagged to the caller (return an enum `Classified::Spatial(Vec<FollowerDemand>) | Classified::VirtualPath { length, followers }` — adjust `handle_curve`).
  - temporal: `ChainGrid::virtual_path(length, n, limits, followers) -> ChainGrid` — uniform `s` grid over `[0, L]`, `PointGeom` all-zero (every spatial row family already skips on `restricted_norm < COMP_FLOOR`; confirm `mvc_b` returns the `b_cap` ceiling, and that the velocity↔accel relation rows are geometry-free — they are), `inter_geom` empty. The feedrate cap is the caller's existing `b_cap` mechanism (`grep -n "b_cap" rust/temporal/src/topp/constraints.rs` — confirm it derives from feedrate; it does via the MVC seed). Follower rows from Tasks 4–8 do the rest.
  - trajectory: where segments are mapped to `temporal::SegmentInput` (`grep -rn "SegmentInput" rust/trajectory/`), route `virtual_path_mm` segments to the virtual-path constructor; they form their own single-segment chain (no fusing with spatial neighbors — tangent continuity is undefined against a zero curve; junction classification treats them as corner stops on both sides).
  - Delete the `Fatal::FollowerOnlyMoveUnsupported` variant and every consumer.

- [ ] **Step 4: Run** — `cargo nextest run -p geometry -p temporal -p trajectory` → PASS
- [ ] **Step 5: Commit** — `feat: follower-only moves plan on a virtual path (spec §2 degenerate rule)`

---

### Task 10: trajectory + motion-bridge plumbing

Follower limit sections reach temporal as real sets; segments' `FollowerDemand`s (with `pa_k`) reach `SegmentInput`; kernels and streaming history reach the chain.

**Files:**
- Modify: `rust/motion-bridge/src/config.rs` (`to_temporal_limits`: follower sections convert instead of coverage-only; `n_axes` from the registry; `MixedSpatialFollower` rejection stays)
- Modify: `rust/motion-bridge/src/config/tests.rs`
- Modify: `rust/trajectory/src/lib.rs` / `beta.rs` / `plan_velocity.rs` (thread `followers` from `ShapeSegmentInput` into `temporal::SegmentInput`, mapping `geometry::segment::FollowerDemand { axis_index, ratio }` → `temporal::FollowerDemand { axis, ratio, pa_k }`; `pa_k` comes from a new `ReplanContext.follower_pa: Vec<(usize, f64)>` defaulting empty)
- Modify: `rust/trajectory/src/streaming/state.rs` + `streaming/emit.rs` (history extraction)
- Tests: `rust/motion-bridge/src/config/tests.rs`, trajectory integration test

- [ ] **Step 1: Write failing bridge test:**

```rust
#[test]
fn follower_sections_become_temporal_sets() {
    // registry with axis e (index 3), [limit extruder] axes: e v=75 a=1500:
    // to_temporal_limits() now returns a Limits whose follower_sets() has one
    // entry with AxisSet containing 3, and n_axes() == 4.
}
```

- [ ] **Step 2: Run to verify failure**, implement the config conversion (replace plan 2's "recorded for coverage only" branch: follower section axes map straight into `AxisSet::from_indices`; pass `axis_registry.n_axes()` to `Limits::try_new`). The runtime-caps overlay stays `AxisSet::spatial()`.

- [ ] **Step 3: trajectory threading** — at every `temporal::SegmentInput` construction site, attach the segment's followers (mapped, `pa_k` looked up per axis from `ReplanContext.follower_pa`, default `0.0`). Kernels: the followed axes' kernels are exactly the chain's `axis_kernels` — populate from the same `ReplanContext.kernels` used by emission (`grep -n "kernels" rust/trajectory/src/streaming/mod.rs`), converting `PlanShaper`/`AxisShaper` to `PiecewisePolynomialKernel` via the existing `to_kernel` path.

- [ ] **Step 4: streaming history** — in `append_and_replan` (streaming/state.rs), when any kernel is active and any uncommitted segment carries followers, sample the realized per-axis velocities over `[t_freeze − max_h, t_freeze]` from the retained shaped pieces (`self.axes[α].pieces` + `pending_freeze` — the same data the freeze zone already preserves; differentiate the Bezier pieces at `HISTORY_DT = max_h / 32` steps) into `temporal`'s `FollowerHistory`, passed through `plan_velocity` → `BatchInput`. When no shaped history exists yet (cold start), the history is all-zero — correct, the machine was at rest.

- [ ] **Step 5: Integration test** (new `rust/trajectory/tests/follower_rows.rs`): plan a 3-segment batch (straight, corner stop, straight) with an extruder follower (ratio 0.05, `[limit extruder]` v=75 a=1500), X/Y smooth-zv kernels, `pa_k = 0.04`; assert (a) it solves, (b) brute-force shaped-demand check as in Task 8's test holds over the emitted profile, (c) with `pa_k = 0` total time strictly decreases or stays equal (PA rows only ever tighten).

- [ ] **Step 6: Run** — `cargo nextest run` from `rust/` → full workspace PASS
- [ ] **Step 7: Commit** — `feat(trajectory,motion-bridge): follower demands, kernels, and history reach the solver`

---

### Task 11: fossil sweep and end-to-end verification

- [ ] **Step 1:** `grep -rn "FollowerOnlyMoveUnsupported\|NoFollowerCoverage.*coverage only\|recorded for coverage" rust/ klippy/ --include="*.rs" --include="*.py"` — plan-2 stopgaps must be gone or rewritten; every survivor justified in the commit message.
- [ ] **Step 2:** `cargo nextest run` from `rust/` → PASS; `cargo test --doc` if doc examples touched; `cargo fmt --all --check` → clean. If `klippy/` was touched (it should not be in this plan — confirm with `git status`), run `./scripts/ci.sh py`.
- [ ] **Step 3:** kalico-sim sanity: boot a migrated fixture with `[axis e]` + `[limit extruder]`; travel-only prints behave identically to plan-2 (followers empty on live segments — live extrusion is still rejected until plan 4); a fixture whose `[limit extruder]` declares only `max_jerk` errors at startup naming velocity/accel coverage.
- [ ] **Step 4:** Pure-function check: every new temporal test runs with zero hardware/bridge involvement — the planner remains `(geometry, rows, kernels, history) → profile`. Confirm no new `static`/global entered `temporal` (`grep -rn "static\|lazy_static\|OnceLock" rust/temporal/src/ | grep -v test`).
- [ ] **Step 5: Commit** — `feat: follower/PA/shaper-folded constraint rows end-to-end (plan 3)`

---

## Self-review notes (spec → plan coverage)

- Follower rows, same row shape as everything else: Task 4 (convex base), Task 5 (jerk) ✓
- PA rows `|r(ṡ+k·s̈)| ≤ v`, `|r(s̈+k·s⃛)| ≤ a`, mixed-derivative, SLP-linearized: Task 7 ✓
- Jerk under PA requires path snap; derivative order extended, separately testable: Task 6 (snap), Task 7 (consumes) ✓
- Shaper folding: rows on the shaped combination, known weights, no feedback loop *within a solve* (the operator is written into the inequality; the outer re-freeze is over the time map, not the shaper): Task 8 ✓
- Rows couple across segment boundaries; committed tail enters as constants: Task 8 Step 5 (cross-chain), Task 10 Step 4 (batch boundary / streaming) ✓
- Solve cost grows with window width — spent deliberately; chains without kernels/PA skip everything: Task 8 Step 4 ✓
- Shaper trait contract "expose your action as a linear operator": satisfied structurally — temporal consumes `PiecewisePolynomialKernel` directly, which *is* the linear operator; the trajectory-side trait formalization belongs to plan 4's post-processor/emission work ✓
- Follower-only moves planned as regular moves (path length = follower displacement, feedrate applies, own rows cap): Task 9 ✓
- Planner stays a pure function / oracle API: Task 11 Step 4 ✓
- Fail loudly: `MixedSpatialFollower`, chain validation errors, `FollowerSlpDiverged`, tail-exchange cap — all hard errors, no silent fallbacks ✓
- Observability groundwork: `BindingConstraint::{PaVelocity, PaAccel, PaJerk}` + follower set attribution (Tasks 4, 7); reporting itself is plan 6 ✓
- Deferred consciously: mixed spatial+follower sets, nonlinear PA, follower's own shaper in config, windowed PA-jerk (decision 3), `c⁗` axis geometry — all additive later

**Known-approximation register (each conservative, none silent):**
1. PA-jerk row is nominal-path-form, not windowed (decision 3) — over-tight only inside the shaper window at corners.
2. The windowed rows hold on the discretization + frozen time map; the brute-force convolution tests (Tasks 8, 10) are the falsifiable guard that the discretization error stays within the 5% feasibility band the SLP already tolerates.
3. Right-edge terminal-hold extension of the window signal — replanned away by streaming before dispatch (only the region `≤ t_decel_start − max_h` is ever committed).
