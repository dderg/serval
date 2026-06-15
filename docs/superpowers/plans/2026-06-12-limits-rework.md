# Unified `[limit]` Sections Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the legacy limit model (global scalars broadcast to per-axis boxes + separate centripetal cap + SCV) with named `[limit]` sections — each a set of axes with norm caps — flowing as constraint rows into the existing SOCP solver; delete every legacy limit concept loudly.

**Architecture:** Spec: `docs/superpowers/specs/2026-06-12-follower-axes-and-limits-design.md` §3. `temporal::Limits` becomes a collection of `LimitSet { axes, v_max, a_max, j_max }`. Velocity norm caps are linear rows in `b = ṡ²`; multi-axis accel norm caps are SecondOrder cone rows (the solver is already an SOCP); singleton sets reproduce today's per-axis rows. The centripetal cap is subsumed by the accel-norm row's orthogonal component (`κ_S`-derived `b` caps stay as MVC seeds / anti-aliasing rows). Config: klippy parses `[limit <name>]` sections (new `klippy/extras/limit.py`), `MotionToolhead` rejects all legacy `[printer]` limit keys, the bridge receives the section list verbatim, Rust validates coverage and fails loudly. `M204`/`SET_VELOCITY_LIMIT` become a runtime *overlay* cap over all axes (can tighten below config, never exceed it).

**Tech stack:** Rust (temporal/trajectory/motion-engine crates, Clarabel SOCP), PyO3 bridge, klippy Python. Tests: `cargo nextest run` from `rust/` (never bare `cargo test`); doc-tests via `cargo test --doc` if touched.

**Out of scope (later plans):** follower axes / axis `e` in `[limit]` sections (unknown axis names are errors for now), `steppers:` key, `ELimits`/`e_independent` (untouched, dies in plans 2–4), binding-constraint observability reporting (plan 7 — but `BindingConstraint` carries set indices after this plan, which is its groundwork).

**Repo rules that apply to every task:** unit tests live in separate files from tested code; no explanatory comments — name/extract instead; fail loudly (no silent fallbacks); commit after every task; no Claude/Anthropic commit trailers; `cargo fmt --all --check` before any PR push.

---

### Task 1: Store inter-sample geometry instead of pre-baked κ (behavior-preserving)

The inter-sample centripetal rows currently consume `(θ, κ)` pairs with κ = full-path curvature. Per-set κ needs `c′`/`c″` at those samples. This task swaps the storage to geometry and recomputes κ at the consumer — output must be bit-identical-ish (same formula, same samples), so the whole suite passes unchanged.

**Files:**
- Modify: `rust/temporal/src/topp/path.rs` (struct `ArclengthGrid`, `sample_arclength_grid`)
- Modify: `rust/temporal/src/topp/chain.rs` (field type), `rust/temporal/src/topp/scaling.rs` (`scale_grid`, `scale_chain_grid`), `rust/temporal/src/topp/constraints.rs` (inter-sample block, ~line 527)
- Modify: any other `inter_kappa` consumer — find with `grep -rn "inter_kappa" rust/`

- [ ] **Step 1: Add the new sample type and swap the field**

In `path.rs`:

```rust
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct InterSample {
    pub theta: f64,
    pub c_prime: [f64; 3],
    pub c_double_prime: [f64; 3],
}
```

Replace `pub inter_kappa: Vec<Vec<(f64, f64)>>` with `pub inter_geom: Vec<Vec<InterSample>>` in `ArclengthGrid`, and the same rename in `ChainGrid` (chain.rs:38).

In `sample_arclength_grid`, the per-node loop already computes `c_prime_i` / `c_double_prime_i` from `dc_du`, `d2c_du2` via `du_ds`, `d2u_ds2` (path.rs:120-147). Extract that math into a helper so the inter-sample loop can reuse it:

```rust
fn arc_geometry_at(
    d1: &Option<VectorNurbs<f64, 3>>,
    d2: &Option<VectorNurbs<f64, 3>>,
    u: f64,
    floor: f64,
) -> ([f64; 3], [f64; 3]) {
    let dc_du = eval_or_zero(d1, u);
    let d2c_du2 = eval_or_zero(d2, u);
    let f = dot3(dc_du, dc_du).sqrt().max(floor);
    let df_du = dot3(d2c_du2, dc_du) / f;
    let du_ds = 1.0 / f;
    let d2u_ds2 = -df_du / (f * f * f);
    let c_prime = scale3(dc_du, du_ds);
    let c_double_prime = add3(scale3(d2c_du2, du_ds * du_ds), scale3(dc_du, d2u_ds2));
    (c_prime, c_double_prime)
}
```

Use it in the inter-sample loop (replacing `kappa_at_u(u_mid)`):

```rust
let (c_prime, c_double_prime) = arc_geometry_at(&d1, &d2, u_mid, floor);
InterSample { theta, c_prime, c_double_prime }
```

If the node loop's existing math can't be literally shared (it also needs `c_triple_prime`), keep the node loop as is and have `arc_geometry_at` duplicate only the two formulas above — they must match the node-loop formulas exactly.

- [ ] **Step 2: Recompute κ at the consumer**

In `constraints.rs` inter-sample block (~line 533), κ in arc-length parameterization is `‖c′ × c″‖` (this is what `kappa_at_u` computed — verify against path.rs:149-150 which uses the same cross-product on arc-length vectors):

```rust
for sample in &chain.inter_geom[i] {
    let cross = cross3(sample.c_prime, sample.c_double_prime);
    let kappa = dot3(cross, cross).sqrt();
    ...same logic as before with (sample.theta, kappa)...
}
```

Export `cross3`/`dot3` from path.rs (or a small shared module) instead of duplicating.

- [ ] **Step 3: Fix scaling**

`scale_grid` / `scale_chain_grid` (scaling.rs:68-72, 150-154) scaled κ by `* s`. Geometry scales like the node arrays: `c_prime` unchanged, `c_double_prime.map(|v| v * s)` (verify: node `c_double_prime` scales `* s` at scaling.rs:56-60 and 130).

- [ ] **Step 4: Run the full temporal suite**

Run: `cargo nextest run -p temporal` from `rust/`
Expected: PASS, zero behavioral change. If any solver test shifts, the inter κ formula doesn't match the old `kappa_at_u` — fix the formula, do not update test expectations.

- [ ] **Step 5: Commit** — `refactor(temporal): store inter-sample geometry, derive kappa at consumers`

---

### Task 2: New limit types and geometry helpers (additive — old `Limits` untouched)

**Files:**
- Modify: `rust/temporal/src/limits.rs` (append new types; old `Limits` struct stays until Task 3)
- Create: `rust/temporal/src/limits/tests.rs` (`#[cfg(test)] mod tests;` hookup at the bottom of limits.rs)

- [ ] **Step 1: Write failing unit tests** in `limits/tests.rs`:

```rust
use super::*;

fn set(axes: &[usize], v: f64, a: f64, j: f64) -> LimitSet {
    LimitSet { axes: AxisSet::from_indices(axes), v_max: v, a_max: a, j_max: j }
}

#[test]
fn coverage_validation_rejects_uncovered_axis() {
    let err = NormLimits::try_new(&[set(&[0, 1], 300.0, 3000.0, 6000.0)]).unwrap_err();
    assert!(matches!(err, LimitsError::NoVelocityCoverage { axis: 2 }));
}

#[test]
fn coverage_is_per_derivative() {
    let err = NormLimits::try_new(&[
        set(&[0, 1, 2], 300.0, f64::INFINITY, f64::INFINITY),
        set(&[0, 1], f64::INFINITY, 3000.0, 6000.0),
    ])
    .unwrap_err();
    assert!(matches!(err, LimitsError::NoAccelCoverage { axis: 2 }));
}

#[test]
fn rejects_nonpositive_caps() {
    let err = NormLimits::try_new(&[set(&[0], 0.0, 100.0, 200.0)]).unwrap_err();
    assert!(matches!(err, LimitsError::BadCap { set: 0 }));
}

#[test]
fn mvc_b_is_min_over_sets() {
    let lim = NormLimits::try_new(&[
        set(&[0, 1], 60.0, 6000.0, 12000.0),
        set(&[1], 40.0, f64::INFINITY, f64::INFINITY),
        set(&[2], 15.0, 100.0, 200.0),
    ])
    .unwrap();
    let pure_y = [0.0, 1.0, 0.0];
    assert!((lim.mvc_b(&pure_y, 1e-9) - 1600.0).abs() < 1e-9);
    let diag = [std::f64::consts::FRAC_1_SQRT_2, std::f64::consts::FRAC_1_SQRT_2, 0.0];
    let expected = (40.0 / std::f64::consts::FRAC_1_SQRT_2).powi(2).min(3600.0);
    assert!((lim.mvc_b(&diag, 1e-9) - expected).abs() < 1e-6);
}

#[test]
fn kappa_set_is_orthogonal_component_of_restricted_second_derivative() {
    let c_prime = [1.0, 0.0, 0.0];
    let c_double_prime = [0.0, 2.0, 1.0];
    assert!((kappa_set(&c_prime, &c_double_prime, AxisSet::from_indices(&[0, 1]), 1e-12) - 2.0).abs() < 1e-12);
    assert!((kappa_set(&c_prime, &c_double_prime, AxisSet::from_indices(&[0, 1, 2]), 1e-12) - 5.0_f64.sqrt()).abs() < 1e-12);
    let c_dp_tangential = [3.0, 0.0, 0.0];
    assert!(kappa_set(&c_prime, &c_dp_tangential, AxisSet::from_indices(&[0]), 1e-12).abs() < 1e-12);
}

#[test]
fn b_cent_cap_uses_per_set_kappa() {
    let lim = NormLimits::try_new(&[
        set(&[0, 1], 300.0, 1000.0, 2000.0),
        set(&[2], 15.0, 100.0, 200.0),
    ])
    .unwrap();
    let c_prime = [1.0, 0.0, 0.0];
    let c_double_prime = [0.0, 0.5, 0.0];
    assert!((lim.b_cent_cap(&c_prime, &c_double_prime, 1e-12) - 2000.0).abs() < 1e-9);
    let c_dp_z_only = [0.0, 0.0, 0.5];
    assert!((lim.b_cent_cap(&c_prime, &c_dp_z_only, 1e-12) - 200.0).abs() < 1e-9);
}
```

(Name the new struct `NormLimits` during this task only; Task 3 renames it to `Limits` when the old struct dies.)

- [ ] **Step 2: Run to verify failure** — `cargo nextest run -p temporal -E 'test(limits)'` → FAIL (types undefined)

- [ ] **Step 3: Implement** in `limits.rs`:

```rust
pub const MAX_AXES: usize = 3;
pub const MAX_LIMIT_SETS: usize = 8;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AxisSet(u8);

impl AxisSet {
    #[must_use]
    pub fn from_indices(indices: &[usize]) -> Self {
        let mut bits = 0_u8;
        for &i in indices {
            assert!(i < MAX_AXES, "axis index {i} out of range");
            bits |= 1 << i;
        }
        assert!(bits != 0, "empty axis set");
        Self(bits)
    }
    #[must_use]
    pub fn all() -> Self {
        Self((1 << MAX_AXES) - 1)
    }
    #[must_use]
    pub fn contains(self, axis: usize) -> bool {
        self.0 & (1 << axis) != 0
    }
    pub fn indices(self) -> impl Iterator<Item = usize> {
        (0..MAX_AXES).filter(move |&i| self.contains(i))
    }
    #[must_use]
    pub fn count(self) -> usize {
        self.0.count_ones() as usize
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LimitSet {
    pub axes: AxisSet,
    pub v_max: f64,
    pub a_max: f64,
    pub j_max: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct NormLimits {
    sets: [LimitSet; MAX_LIMIT_SETS],
    n_sets: u8,
}

#[derive(Debug, thiserror::Error, PartialEq)]
pub enum LimitsError {
    #[error("no limit sets declared")]
    Empty,
    #[error("more than {MAX_LIMIT_SETS} limit sets")]
    TooMany,
    #[error("limit set {set}: caps must be positive (or +inf for undeclared)")]
    BadCap { set: usize },
    #[error("axis {axis}: no limit set declares a finite max_velocity covering it")]
    NoVelocityCoverage { axis: usize },
    #[error("axis {axis}: no limit set declares a finite max_accel covering it")]
    NoAccelCoverage { axis: usize },
    #[error("axis {axis}: no limit set declares a finite max_jerk covering it")]
    NoJerkCoverage { axis: usize },
}

impl NormLimits {
    pub fn try_new(sets: &[LimitSet]) -> Result<Self, LimitsError> {
        if sets.is_empty() {
            return Err(LimitsError::Empty);
        }
        if sets.len() > MAX_LIMIT_SETS {
            return Err(LimitsError::TooMany);
        }
        for (idx, s) in sets.iter().enumerate() {
            let ok = |c: f64| c > 0.0 && !c.is_nan();
            if !(ok(s.v_max) && ok(s.a_max) && ok(s.j_max)) {
                return Err(LimitsError::BadCap { set: idx });
            }
        }
        for axis in 0..MAX_AXES {
            let covered = |f: fn(&LimitSet) -> f64| {
                sets.iter().any(|s| s.axes.contains(axis) && f(s).is_finite())
            };
            if !covered(|s| s.v_max) {
                return Err(LimitsError::NoVelocityCoverage { axis });
            }
            if !covered(|s| s.a_max) {
                return Err(LimitsError::NoAccelCoverage { axis });
            }
            if !covered(|s| s.j_max) {
                return Err(LimitsError::NoJerkCoverage { axis });
            }
        }
        let filler = sets[0];
        let mut arr = [filler; MAX_LIMIT_SETS];
        arr[..sets.len()].copy_from_slice(sets);
        Ok(Self { sets: arr, n_sets: sets.len() as u8 })
    }

    #[must_use]
    pub fn sets(&self) -> &[LimitSet] {
        &self.sets[..self.n_sets as usize]
    }

    #[must_use]
    pub fn axis_boxes(v: [f64; 3], a: [f64; 3], j: [f64; 3]) -> Self {
        let sets: Vec<LimitSet> = (0..3)
            .map(|ax| LimitSet {
                axes: AxisSet::from_indices(&[ax]),
                v_max: v[ax],
                a_max: a[ax],
                j_max: j[ax],
            })
            .collect();
        Self::try_new(&sets).expect("axis_boxes: finite positive caps")
    }

    #[must_use]
    pub fn norm_all(v: f64, a: f64, j: f64) -> Self {
        Self::try_new(&[LimitSet { axes: AxisSet::all(), v_max: v, a_max: a, j_max: j }])
            .expect("norm_all: finite positive caps")
    }

    #[must_use]
    pub fn mvc_b(&self, c_prime: &[f64; 3], floor: f64) -> f64 {
        let mut bound = f64::INFINITY;
        for s in self.sets() {
            if !s.v_max.is_finite() {
                continue;
            }
            let p = restricted_norm(c_prime, s.axes);
            if p > floor {
                let vb = s.v_max / p;
                bound = bound.min(vb * vb);
            }
        }
        bound
    }

    #[must_use]
    pub fn a_tan_cap(&self, c_prime: &[f64; 3], floor: f64) -> f64 {
        self.tan_cap(c_prime, floor, |s| s.a_max)
    }

    #[must_use]
    pub fn j_tan_cap(&self, c_prime: &[f64; 3], floor: f64) -> f64 {
        self.tan_cap(c_prime, floor, |s| s.j_max)
    }

    fn tan_cap(&self, c_prime: &[f64; 3], floor: f64, cap: fn(&LimitSet) -> f64) -> f64 {
        let mut bound = f64::INFINITY;
        for s in self.sets() {
            let c = cap(s);
            if !c.is_finite() {
                continue;
            }
            let p = restricted_norm(c_prime, s.axes);
            if p > floor {
                bound = bound.min(c / p);
            }
        }
        bound
    }

    #[must_use]
    pub fn j_path(&self) -> f64 {
        self.sets()
            .iter()
            .map(|s| s.j_max)
            .filter(|j| j.is_finite())
            .fold(f64::INFINITY, f64::min)
    }

    #[must_use]
    pub fn v_ceiling(&self) -> f64 {
        self.sets()
            .iter()
            .map(|s| s.v_max)
            .filter(|v| v.is_finite())
            .fold(f64::NEG_INFINITY, f64::max)
    }

    #[must_use]
    pub fn b_cent_cap(&self, c_prime: &[f64; 3], c_double_prime: &[f64; 3], kappa_floor: f64) -> f64 {
        let mut bound = f64::INFINITY;
        for s in self.sets() {
            if !s.a_max.is_finite() {
                continue;
            }
            let k = kappa_set(c_prime, c_double_prime, s.axes, kappa_floor);
            if k > kappa_floor {
                bound = bound.min(s.a_max / k);
            }
        }
        bound
    }
}

#[must_use]
pub fn restricted_norm(v: &[f64; 3], axes: AxisSet) -> f64 {
    axes.indices().map(|i| v[i] * v[i]).sum::<f64>().sqrt()
}

#[must_use]
pub fn kappa_set(c_prime: &[f64; 3], c_double_prime: &[f64; 3], axes: AxisSet, floor: f64) -> f64 {
    let mut pp = 0.0;
    let mut pq = 0.0;
    let mut qq = 0.0;
    for i in axes.indices() {
        pp += c_prime[i] * c_prime[i];
        pq += c_prime[i] * c_double_prime[i];
        qq += c_double_prime[i] * c_double_prime[i];
    }
    if pp.sqrt() <= floor {
        return qq.sqrt();
    }
    (qq - pq * pq / pp).max(0.0).sqrt()
}
```

Note for `axis_boxes`: callers passing `f64::INFINITY` jerk would fail coverage — all three arrays must be finite. That matches every existing test's old `Limits::new` usage.

- [ ] **Step 4: Run** — `cargo nextest run -p temporal -E 'test(limits)'` → PASS
- [ ] **Step 5: Commit** — `feat(temporal): norm-limit sets with per-set geometry helpers`

---

### Task 3: Swap `Limits` to the set model across the temporal crate

This is the fat task: delete the old struct, rename `NormLimits` → `Limits`, port every consumer. The crate must compile and its full suite pass at the end; intermediate steps won't compile — that's expected within the task.

**Files:**
- Modify: `rust/temporal/src/limits.rs` (delete old struct + `new()`, rename)
- Modify: `rust/temporal/src/topp/constraints.rs`, `topp/solver.rs`, `topp/verify.rs`, `topp/scaling.rs`, `topp/chain.rs` (only if it references fields), `multi/junction.rs`, `multi/parallel.rs`, `multi/mod.rs`
- Modify: all temporal test files constructing `Limits::new(...)` — find with `grep -rln "Limits::new" rust/temporal/`

- [ ] **Step 1: Delete old struct, rename `NormLimits` to `Limits`** (and in Task-2 tests).

- [ ] **Step 2: Port `constraints.rs`** — block by block:

**`velocity_mvc_b` (line 178):** delete the free function; replace both call sites (lines 283-290) with `chain.limits_at(0).mvc_b(&chain.geom[0].c_prime, COMP_FLOOR)` etc.

**`b_max_cent` (lines 212-230):** replace both loops with:

```rust
let mut b_max_cent: Vec<f64> = (0..n)
    .map(|i| {
        chain
            .limits_at(i)
            .b_cent_cap(&chain.geom[i].c_prime, &chain.geom[i].c_double_prime, kappa_floor)
            .min(b_cap)
    })
    .collect();
for j in &chain.junctions {
    let cap = chain.limits[j.limits_idx]
        .b_cent_cap(&j.geom.c_prime, &j.geom.c_double_prime, kappa_floor)
        .min(b_cap);
    b_max_cent[j.idx] = b_max_cent[j.idx].min(cap);
}
```

(`PointGeom.kappa` becomes unread by this file — leave the field for now, Task 8 sweeps it.)

**`a_env`/`j_env` (lines 232-278):** replace each point/junction inner per-axis loop with:

```rust
let a_tan_i = lim.a_tan_cap(&geom.c_prime, COMP_FLOOR);
let j_tan_i = lim.j_tan_cap(&geom.c_prime, COMP_FLOOR);
if a_tan_i.is_finite() && j_tan_i.is_finite() {
    a_env = a_env.max(a_tan_i);
    j_env = j_env.max(j_tan_i);
}
```

**`j_path` (line 331):** `let j_path = chain.limits.iter().map(Limits::j_path).fold(f64::INFINITY, f64::min);`

**Velocity block (lines 409-447):** per-set linear rows. Replace the two per-axis loops with (same shape for nodes and junction duals):

```rust
for set in lim.sets() {
    if !set.v_max.is_finite() {
        continue;
    }
    let p = restricted_norm(&geom.c_prime, set.axes);
    if p < COMP_FLOOR {
        continue;
    }
    let rhs = (set.v_max / p).powi(2);
    if rhs > b_cap {
        continue;
    }
    push_row(&mut a_rows, &mut b_rhs, &[(off_b + i, -1.0)], rhs);
    count += 1;
}
```

**Accel block (lines 449-517):** singleton sets keep the two linear rows; multi-axis sets emit a SecondOrder cone. Cone entries are consumed in row order, so a pending Nonneg run must flush before each SOC. Replace the whole block body with:

```rust
const BLOCK_D_SAFETY: f64 = 0.1;
let mut nonneg_run = 0_usize;
let mut emit_accel = |i: usize,
                      geom: &PointGeom,
                      lim: &Limits,
                      a_rows: &mut Vec<Vec<f64>>,
                      b_rhs: &mut Vec<f64>,
                      cones: &mut Vec<(Cone, usize)>,
                      nonneg_run: &mut usize| {
    let b_cap_i = b_max_cent[i].min(b_cap);
    let a_cap_i = b_cap_i / (2.0 * h_bar(i));
    for set in lim.sets() {
        if !set.a_max.is_finite() {
            continue;
        }
        let gp_n = restricted_norm(&geom.c_prime, set.axes);
        let gpp_n = restricted_norm(&geom.c_double_prime, set.axes);
        if gp_n < COMP_FLOOR && gpp_n < COMP_FLOOR {
            continue;
        }
        if gpp_n * b_cap_i + gp_n * a_cap_i < BLOCK_D_SAFETY * set.a_max {
            continue;
        }
        if set.axes.count() == 1 {
            let ax = set.axes.indices().next().expect("singleton");
            let gp = geom.c_prime[ax];
            let gpp = geom.c_double_prime[ax];
            push_row(a_rows, b_rhs, &[(off_b + i, -gpp), (off_a + i, -gp)], set.a_max);
            push_row(a_rows, b_rhs, &[(off_b + i, gpp), (off_a + i, gp)], set.a_max);
            *nonneg_run += 2;
        } else {
            if *nonneg_run > 0 {
                cones.push((Cone::Nonneg, *nonneg_run));
                *nonneg_run = 0;
            }
            push_row(a_rows, b_rhs, &[], set.a_max);
            for ax in set.axes.indices() {
                push_row(
                    a_rows,
                    b_rhs,
                    &[(off_b + i, geom.c_double_prime[ax]), (off_a + i, geom.c_prime[ax])],
                    0.0,
                );
            }
            cones.push((Cone::SecondOrder, 1 + set.axes.count()));
        }
    }
};
for i in 0..n {
    emit_accel(i, &chain.geom[i], chain.limits_at(i), &mut a_rows, &mut b_rhs, &mut cones, &mut nonneg_run);
}
for j in &chain.junctions {
    emit_accel(j.idx, &j.geom, &chain.limits[j.limits_idx], &mut a_rows, &mut b_rhs, &mut cones, &mut nonneg_run);
}
if nonneg_run > 0 {
    cones.push((Cone::Nonneg, nonneg_run));
}
```

(If the closure fights the borrow checker over `push_row`, inline it as a function taking all buffers — the structure is what matters. The SOC encodes `b_rhs − A·x ∈ K`: head slack `= a_max`, member slacks `= −(c″·b + c′·a)` per axis, so the cone enforces `‖accel_S‖ ≤ a_max`.)

**Inter-sample block (lines 527-554):** the interval owner today is `chain.limits_at(i + 1)`; keep that. Replace the κ loop:

```rust
for sample in &chain.inter_geom[i] {
    let inter_cap = lim
        .b_cent_cap(&sample.c_prime, &sample.c_double_prime, kappa_floor)
        .min(b_cap);
    let interp_node_cap = (1.0 - sample.theta) * node_cap_i + sample.theta * node_cap_j;
    if inter_cap >= interp_node_cap * (1.0 - 1e-9) {
        continue;
    }
    push_row(
        &mut a_rows,
        &mut b_rhs,
        &[(off_b + i, -(1.0 - sample.theta)), (off_b + i + 1, -sample.theta)],
        inter_cap,
    );
    count += 1;
}
```

- [ ] **Step 3: Port `solver.rs` jerk cuts (lines ~710-840).** The per-axis jerk ratio loops (`for ax in 0..3 { ratio = j[ax].abs() / lim.j_max[ax] }`) become per-set: the code already assembles the per-axis jerk vector `j = c‴·ṡ³ + 3c″·ṡ·s̈ + c′·s⃛`; replace the per-axis ratio with, per set with finite `j_max`: `ratio = restricted_norm(&jerk_vec, set.axes) / set.j_max`, and where the cut target uses `lim.j_max[ax]` (lines 798-832), use `set.j_max` of the binding set. Preserve the existing target_ratio scaling logic verbatim — only the (value, limit) pair changes from per-axis to per-set.

- [ ] **Step 4: Port `verify.rs`.** In `ratios_at`, the `vel`/`accel`/`jerk` 3-vectors stay; replace the 10-entry per-axis table + centripetal entry with per-set entries:

```rust
for (set_idx, set) in lim.sets().iter().enumerate() {
    if set.v_max.is_finite() {
        entries.push((restricted_norm(&vel, set.axes) / set.v_max, BindingConstraint::Velocity { set: set_idx }));
    }
    if set.a_max.is_finite() {
        entries.push((restricted_norm(&accel, set.axes) / set.a_max, BindingConstraint::AccelNorm { set: set_idx }));
    }
    if set.j_max.is_finite() {
        entries.push((restricted_norm(&jerk, set.axes) / set.j_max, BindingConstraint::JerkNorm { set: set_idx }));
    }
}
```

Change `BindingConstraint` variants: `Velocity { axis: Axis }` → `Velocity { set: usize }`, `AxisAccel`/`AxisJerk` → `AccelNorm`/`JerkNorm { set: usize }`, delete `Centripetal`. Update the tie-break ordering comment-free logic (Velocity > AccelNorm > JerkNorm; lower set index wins) and every test/consumer referencing the old variants (`grep -rn "BindingConstraint::" rust/`). The jerk/non-jerk class split (`max_jerk` vs `max_non_jerk`) keeps: JerkNorm entries are the jerk class.

- [ ] **Step 5: Port `scaling.rs`.**

```rust
pub(crate) fn scale_limits(&self, limits: &Limits) -> Limits {
    let s = self.sigma();
    let sets: Vec<LimitSet> = limits
        .sets()
        .iter()
        .map(|l| LimitSet { axes: l.axes, v_max: l.v_max / s, a_max: l.a_max / s, j_max: l.j_max / s })
        .collect();
    Limits::try_new(&sets).expect("scaling preserves validity")
}
```

`for_limits` / `for_chain` sigma: `limits.v_ceiling()` (max finite v over sets) replacing the per-axis max. `scale_chain_grid`: `inter_geom` scales like Task 1 Step 3.

- [ ] **Step 6: Reduce `multi/junction.rs` to a classifier.**

- **Delete the junction-deviation machinery entirely** (it is mainline's SCV/JD transplant — a virtual-rounding pretense that implies infinite jerk at the kink): `sharp_corner_jd_cap`, `V_JD_REVERSAL_FLOOR_MM_S`, `ALPHA_COLLINEAR_THRESHOLD`, `ALPHA_REVERSAL_THRESHOLD`, and the `chord_tolerance_mm` parameter of `compute_junction_velocity`.
- **There is no junction velocity to compute — delete the calculation, keep only the classifier.** A junction is either tangent-continuous within `THETA_FUSE_RAD` (→ the segments fuse into one chain, the point becomes an ordinary interior grid point, and the solver's constraint rows govern it like everywhere else) or it is not (→ full stop, `v = 0`). `compute_junction_velocity` is replaced by `classify_junction` alone; `JunctionResult.v_junction`, `per_axis_velocity_cap`, `centripetal_cap`, `cap_v_max`, `min_with_tag`, and the entire `JunctionBindingCap` enum are deleted. The contract: whatever feeds the planner is responsible for tangent continuity; the planner does not negotiate with kinks.
- Consumers (`grep -rn "compute_junction_velocity\|JunctionBindingCap\|v_junction" rust/`): wherever a smooth junction's `v_junction` was consumed as a chain-boundary condition, that junction must instead be fused (it already is — smooth junctions fuse via the existing chain machinery); corner junctions get boundary velocity `0.0`. If the executor finds a code path that genuinely needs a nonzero boundary velocity at a point that cannot fuse (e.g. a forced split), **stop and surface it for review** — do not reinvent a junction cap.

What survives in the module: `classify_junction` with its existing tangent helpers (`forward_unit_tangent_at_*`) and `THETA_FUSE_RAD`. If `JunctionResult.kappa_left/right` turn out to have consumers (the chain-fusion path may use them), keep `curvature_at_start/end` for those; otherwise delete them too. Reword the `THETA_FUSE_RAD` doc comment — its "scv impulse budget" justification is gone; the fuse threshold survives purely as a numerical collinearity epsilon.

**Consequence, accepted deliberately:** tangent-discontinuous input (today: all compat-converted G1 polylines) full-stops at every corner until upstream corner blending exists (future work, its own brainstorm). Do not soften this with any floor or virtual rounding — the slowdown is the honest signal that the blender is missing.

- [ ] **Step 7: Port `multi/parallel.rs` and `multi/mod.rs`.** `base_v_max = chain.limits[0].v_max` (line 278) → `chain.limits[0].v_ceiling()`; the rescale at lines 348-352 maps sets like `scale_limits`. `multi/mod.rs` is type-mechanical (`Limits` is still `Copy`).

- [ ] **Step 8: Port every test.** `grep -rln "Limits::new" rust/temporal/` — mechanical mapping:
  - `Limits::new([vx,vy,vz],[ax,ay,az],[jx,jy,jz], cent)` where the test exercises per-axis behavior → `Limits::axis_boxes([vx,vy,vz],[ax,ay,az],[jx,jy,jz])`.
  - Tests that specifically exercise **centripetal/cornering** behavior (grep test names/asserts for `centripetal`, `corner`, `kappa`) → `Limits::norm_all(v, a, j)` with `a` = the old `a_centripetal_max`. Their numeric expectations may tighten (the norm row also caps the tangential+normal vector sum, which the old box+cent pair did not); update expectations guided by `verify.rs` ratios — a result is correct when verify reports all ratios ≤ 1 and the binding tag matches the test's intent. Do not loosen a test by deleting its assert; re-derive the expected number.
  - `chain/tests_support.rs` and any `fn lim(...)` helpers: port once, all dependent tests follow.

- [ ] **Step 9: Full crate suite** — `cargo nextest run -p temporal` → PASS.
- [ ] **Step 10: Commit** — `feat(temporal): limits as named axis-set norm rows; centripetal cap subsumed by accel-norm SOC`

---

### Task 4: motion-engine config model

**Files:**
- Modify: `rust/motion-engine/src/config.rs` (replace `PlannerLimits`)
- Modify: `rust/motion-engine/src/config/tests.rs` (rewrite limit tests)

- [ ] **Step 1: Write failing tests** in `config/tests.rs` (replacing the `to_temporal_limits`, `junction_deviation_mm`, and centripetal tests at lines 19-69):

```rust
#[test]
fn sections_convert_to_temporal_sets() {
    let cfg = PlannerConfig::default();
    let lims = cfg.to_temporal_limits().unwrap();
    assert_eq!(lims.sets().len(), 2);
    assert_eq!(lims.sets()[0].v_max, 300.0);
    assert_eq!(lims.sets()[1].a_max, 100.0);
}

#[test]
fn jerk_defaults_to_twice_accel_per_section() {
    let cfg = PlannerConfig::default();
    let lims = cfg.to_temporal_limits().unwrap();
    assert_eq!(lims.sets()[0].j_max, 6000.0);
}

#[test]
fn missing_axis_coverage_is_an_error() {
    let mut cfg = PlannerConfig::default();
    cfg.limit_sections.retain(|s| s.name != "z");
    assert!(cfg.to_temporal_limits().is_err());
}

#[test]
fn unknown_axis_name_is_an_error() {
    assert!(axis_index("e").is_err());
    assert_eq!(axis_index("x").unwrap(), 0);
}

#[test]
fn runtime_caps_append_an_all_axis_overlay() {
    let mut cfg = PlannerConfig::default();
    cfg.runtime_caps = RuntimeCaps { velocity: Some(100.0), accel: Some(1000.0) };
    let lims = cfg.to_temporal_limits().unwrap();
    let overlay = lims.sets().last().unwrap();
    assert_eq!(overlay.v_max, 100.0);
    assert_eq!(overlay.a_max, 1000.0);
    assert_eq!(overlay.axes, temporal::AxisSet::all());
}

#[test]
fn section_with_no_caps_is_an_error() {
    let mut cfg = PlannerConfig::default();
    cfg.limit_sections.push(LimitSection {
        name: "empty".into(),
        axes: vec![0],
        max_velocity: None,
        max_accel: None,
        max_jerk: None,
    });
    assert!(cfg.to_temporal_limits().is_err());
}
```

- [ ] **Step 2: Run to verify failure** — `cargo nextest run -p motion-engine -E 'test(config)'` → FAIL
- [ ] **Step 3: Implement** in `config.rs` — delete `PlannerLimits`, `to_temporal_limits` (old), `junction_deviation_mm`:

```rust
#[derive(Debug, Clone, PartialEq)]
pub struct LimitSection {
    pub name: String,
    pub axes: Vec<usize>,
    pub max_velocity: Option<f64>,
    pub max_accel: Option<f64>,
    pub max_jerk: Option<f64>,
}

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct RuntimeCaps {
    pub velocity: Option<f64>,
    pub accel: Option<f64>,
}

#[derive(Debug, Error)]
pub enum LimitConfigError {
    #[error("unknown axis '{name}' in [limit] section (supported: x, y, z)")]
    UnknownAxis { name: String },
    #[error("[limit {section}]: declare at least one of max_velocity, max_accel, max_jerk")]
    EmptySection { section: String },
    #[error("invalid limit configuration: {0}")]
    Invalid(#[from] temporal::LimitsError),
}

pub fn axis_index(name: &str) -> Result<usize, LimitConfigError> {
    match name {
        "x" => Ok(0),
        "y" => Ok(1),
        "z" => Ok(2),
        other => Err(LimitConfigError::UnknownAxis { name: other.to_string() }),
    }
}

const JERK_DEFAULT_ACCEL_MULTIPLE: f64 = 2.0;

impl LimitSection {
    fn to_set(&self) -> Result<temporal::LimitSet, LimitConfigError> {
        if self.max_velocity.is_none() && self.max_accel.is_none() && self.max_jerk.is_none() {
            return Err(LimitConfigError::EmptySection { section: self.name.clone() });
        }
        let j_max = self
            .max_jerk
            .or(self.max_accel.map(|a| a * JERK_DEFAULT_ACCEL_MULTIPLE))
            .unwrap_or(f64::INFINITY);
        Ok(temporal::LimitSet {
            axes: temporal::AxisSet::from_indices(&self.axes),
            v_max: self.max_velocity.unwrap_or(f64::INFINITY),
            a_max: self.max_accel.unwrap_or(f64::INFINITY),
            j_max,
        })
    }
}
```

In `PlannerConfig`: `limits: PlannerLimits` → `limit_sections: Vec<LimitSection>`, plus `runtime_caps: RuntimeCaps`. (No junction-deviation field: the JD machinery dies in Task 3; `ReplanContext.junction_chord_tolerance_mm` is deleted in Task 5.) Conversion:

```rust
impl PlannerConfig {
    pub fn to_temporal_limits(&self) -> Result<temporal::Limits, LimitConfigError> {
        let mut sets = Vec::with_capacity(self.limit_sections.len() + 1);
        for section in &self.limit_sections {
            sets.push(section.to_set()?);
        }
        if self.runtime_caps.velocity.is_some() || self.runtime_caps.accel.is_some() {
            let a = self.runtime_caps.accel.unwrap_or(f64::INFINITY);
            sets.push(temporal::LimitSet {
                axes: temporal::AxisSet::all(),
                v_max: self.runtime_caps.velocity.unwrap_or(f64::INFINITY),
                a_max: a,
                j_max: if a.is_finite() { a * JERK_DEFAULT_ACCEL_MULTIPLE } else { f64::INFINITY },
            });
        }
        Ok(temporal::Limits::try_new(&sets)?)
    }
}
```

Default impl: sections `gantry {x,y}, v=300, a=3000` and `z {z}, v=15, a=100`, `runtime_caps: default`. (Temporal must re-export `AxisSet`, `LimitSet`, `LimitsError` from its lib.rs — add if missing.)

- [ ] **Step 4: Run** — `cargo nextest run -p motion-engine -E 'test(config)'` → PASS (other motion-engine code is still broken — that's Task 5; scope the run to the config tests, or accept compile failure here and fold the run into Task 5's verification if the crate won't build test-by-test).
- [ ] **Step 5: Commit** — `feat(motion-engine): [limit] section config model with runtime overlay caps`

---

### Task 5: Bridge entry points and planner plumbing

**Files:**
- Modify: `rust/motion-engine/src/bridge.rs` (`init_planner` ~line 2206, `update_limits` ~line 2976)
- Modify: `rust/motion-engine/src/planner.rs` (`build_replan_context` ~line 765, `update_limits` on the planner, any `PlannerLimits` mention — `grep -rn "PlannerLimits\|to_temporal_limits\|junction_deviation" rust/motion-engine/ rust/host-rt/`)

- [ ] **Step 1: `init_planner` signature.** Replace the five scalar limit params with the section list:

```rust
#[pyo3(signature = (
    limits,
    shaper_type_x,
    shaper_freq_x,
    shaper_type_y,
    shaper_freq_y,
    mcus,
    window_capacity = 32,
    beta_max_iters = 10,
))]
fn init_planner(
    &self,
    limits: Vec<(String, Vec<String>, Option<f64>, Option<f64>, Option<f64>)>,
    ...
) -> PyResult<()> {
```

Body: build `Vec<LimitSection>` (mapping axis names through `axis_index`, any error → `PyErr::new::<pyo3::exceptions::PyValueError, _>(e.to_string())`), set `cfg.limit_sections`, then **validate eagerly**: `cfg.to_temporal_limits().map_err(|e| PyValueError::new_err(e.to_string()))?;` before storing — config errors must surface at startup, not first move.

- [ ] **Step 2: `update_limits` → `update_runtime_caps`:**

```rust
fn update_runtime_caps(&self, velocity: Option<f64>, accel: Option<f64>) -> PyResult<()> {
    let new_limits = {
        let mut cfg = self.planner_config.lock().unwrap_or_else(|p| p.into_inner());
        cfg.runtime_caps = config::RuntimeCaps { velocity, accel };
        cfg.to_temporal_limits()
            .map_err(|e| PyRuntimeError::new_err(e.to_string()))?
    };
    let guard = self.planner.lock().unwrap_or_else(|p| p.into_inner());
    let planner = guard.as_ref().ok_or_else(|| {
        PyRuntimeError::new_err("planner not initialized — call init_planner first")
    })?;
    planner.update_limits(new_limits).map_err(planner_err)
}
```

Change the planner-side `update_limits` to take `temporal::Limits` directly (it previously took `PlannerLimits` and converted internally — find and follow the chain).

- [ ] **Step 3: `build_replan_context`:** `limits: config.to_temporal_limits().expect("limit sections validated in init_planner")`. Delete `junction_chord_tolerance_mm` from `ReplanContext` (trajectory crate) and trace every consumer down to the now-parameterless `compute_junction_velocity` call — `grep -rn "junction_chord_tolerance\|chord_tolerance" rust/`.

- [ ] **Step 4: Whole-workspace build & test** — `cargo nextest run` from `rust/` → PASS (this catches every remaining `PlannerLimits` / old-`Limits` straggler across host-rt and integration tests, e.g. `rust/trajectory/tests/jog_50mm_live_limits.rs` — port them with `axis_boxes`/`norm_all` or the new sections as fits each test's intent).
- [ ] **Step 5: Commit** — `feat(motion-engine): init_planner takes [limit] sections; runtime caps replace update_limits`

---

### Task 6: klippy — `[limit]` sections, legacy rejection, command overlay

**Files:**
- Create: `klippy/extras/limit.py`
- Modify: `klippy/toolhead.py` (extract `_read_limits`), `klippy/motion_toolhead.py` (override + new init args + command overrides), `klippy/motion_engine.py` (wrapper + `_StubEngine`)

- [ ] **Step 1: `klippy/extras/limit.py`** (claims the section so configfile accepts it, validates, exposes status):

```python
SUPPORTED_AXES = ("x", "y", "z")


class LimitSection:
    def __init__(self, config):
        self.name = config.get_name().split(None, 1)[1]
        self.axes = [a.strip().lower() for a in config.getlist("axes")]
        for a in self.axes:
            if a not in SUPPORTED_AXES:
                raise config.error(
                    "[%s]: unknown axis '%s' (supported: %s)"
                    % (config.get_name(), a, ", ".join(SUPPORTED_AXES))
                )
        if not self.axes:
            raise config.error("[%s]: axes must not be empty" % config.get_name())
        self.max_velocity = config.getfloat("max_velocity", None, above=0.0)
        self.max_accel = config.getfloat("max_accel", None, above=0.0)
        self.max_jerk = config.getfloat("max_jerk", None, above=0.0)
        if self.max_velocity is None and self.max_accel is None and self.max_jerk is None:
            raise config.error(
                "[%s]: declare at least one of max_velocity, max_accel, max_jerk"
                % config.get_name()
            )

    def get_status(self, eventtime):
        return {
            "axes": list(self.axes),
            "max_velocity": self.max_velocity,
            "max_accel": self.max_accel,
            "max_jerk": self.max_jerk,
        }


def load_config_prefix(config):
    return LimitSection(config)
```

- [ ] **Step 2: Extract `_read_limits` in `toolhead.py`.** Move the limit reads from `ToolHead.__init__` (the block from `self.max_velocity = config.getfloat("max_velocity", above=0.0)` through the `orig_cfg` assignments, toolhead.py ~lines 268-292) into a new method `def _read_limits(self, config):` called from the same spot. Legacy `ToolHead` behavior is unchanged.

- [ ] **Step 3: Override in `motion_toolhead.py`:**

```python
LEGACY_LIMIT_KEYS = (
    "max_velocity",
    "max_accel",
    "max_accel_to_decel",
    "minimum_cruise_ratio",
    "square_corner_velocity",
    "max_z_velocity",
    "max_z_accel",
)


def _read_limits(self, config):
    for key in LEGACY_LIMIT_KEYS:
        if config.get(key, None) is not None:
            raise config.error(
                "[printer] %s is not supported: declare motion limits in "
                "[limit <name>] sections (axes + max_velocity/max_accel/max_jerk)"
                % key
            )
    self.limit_sections = []
    velocities, accels = [], []
    for sc in config.get_prefix_sections("limit "):
        name = sc.get_name().split(None, 1)[1]
        axes = [a.strip().lower() for a in sc.getlist("axes")]
        v = sc.getfloat("max_velocity", None, above=0.0)
        a = sc.getfloat("max_accel", None, above=0.0)
        j = sc.getfloat("max_jerk", None, above=0.0)
        self.limit_sections.append((name, axes, v, a, j))
        if v is not None:
            velocities.append(v)
        if a is not None:
            accels.append(a)
    if not self.limit_sections:
        raise config.error(
            "at least one [limit <name>] section is required "
            "(every axis needs max_velocity and max_accel coverage)"
        )
    if not velocities or not accels:
        raise config.error(
            "[limit] sections must declare max_velocity and max_accel coverage"
        )
    self.max_velocity = min(velocities)
    self.max_accel = min(accels)
    self.min_cruise_ratio = 0.0
    self.square_corner_velocity = 0.0
    self.orig_cfg = {}
    self.runtime_velocity = None
    self.runtime_accel = None
```

Note `self.max_velocity`/`self.max_accel` remain as conservative compat scalars — `get_max_velocity()` consumers (homing, extruder checks) keep working. Place the method on `MotionToolhead`; it overrides the base hook from Step 2. Also remove the now-dead `max_z_velocity`/`max_z_accel` reads at motion_toolhead.py:273-278 and fix their consumers (line ~209 z_ratio math and line ~351 feedrate clamp) to read from the z-covering limit sections: add a helper `def _axis_limit(self, axis, kind)` returning `min` over `self.limit_sections` entries covering `axis` with that cap declared (`None` entries skipped); fail loudly (`raise self.printer.config_error(...)`) if nothing covers it — coverage validation should have caught it already.

- [ ] **Step 4: `_init_planner` call** (motion_toolhead.py ~line 597): replace the five scalars with:

```python
self.bridge.init_planner(
    list(self.limit_sections),
    shaper_type_x,
    shaper_freq_x,
    shaper_type_y,
    shaper_freq_y,
    topology,
)
```

- [ ] **Step 5: Command overlay.** Replace the four bridge-update methods (motion_toolhead.py:496-510) — none call `super()` anymore:

```python
def set_accel(self, accel):
    if accel is not None and accel > 0.0:
        self.runtime_accel = accel
        self.bridge.update_runtime_caps(self.runtime_velocity, self.runtime_accel)

def reset_accel(self):
    self.runtime_accel = None
    self.bridge.update_runtime_caps(self.runtime_velocity, self.runtime_accel)

def cmd_SET_VELOCITY_LIMIT(self, gcmd):
    for unsupported in (
        "SQUARE_CORNER_VELOCITY",
        "MINIMUM_CRUISE_RATIO",
        "ACCEL_TO_DECEL",
    ):
        if gcmd.get_float(unsupported, None) is not None:
            raise gcmd.error(
                "%s is not supported: declare limits in [limit] config sections"
                % unsupported
            )
    v = gcmd.get_float("VELOCITY", None, above=0.0)
    a = gcmd.get_float("ACCEL", None, above=0.0)
    if v is None and a is None:
        gcmd.respond_info(
            "runtime caps: velocity=%s accel=%s"
            % (self.runtime_velocity, self.runtime_accel)
        )
        return
    if v is not None:
        self.runtime_velocity = v
    if a is not None:
        self.runtime_accel = a
    self.bridge.update_runtime_caps(self.runtime_velocity, self.runtime_accel)

def cmd_RESET_VELOCITY_LIMIT(self, gcmd):
    self.runtime_velocity = None
    self.runtime_accel = None
    self.bridge.update_runtime_caps(None, None)
```

Semantics note carried from the spec discussion: the overlay can only *tighten* — an `M204`/`SET_VELOCITY_LIMIT` above the config sections has no effect, because config states physics and rows intersect. This differs from mainline (where SVL could raise the ceiling); it's intentional.

- [ ] **Step 6: `motion_engine.py`** — update the `init_planner` wrapper signature pass-through, rename `update_limits` → `update_runtime_caps` in the wrapper and in `_StubEngine` (no-op accepting `(velocity, accel)`).

- [ ] **Step 7: Commit** — `feat(klippy): [limit] sections replace [printer] limit keys; SVL/M204 become runtime overlay caps`

---### Task 7: Fixture and config sweep

**Files:** discovered, not fixed in advance.

- [ ] **Step 1: Find every config fixture the motion stack loads:**

Run: `grep -rln "max_velocity" --include="*.cfg" . | grep -v test/klippy`
(Mainline's `test/klippy/*.cfg` regression corpus targets legacy `ToolHead` and is not run by our suites — leave it.)

- [ ] **Step 2:** For each fixture that boots `MotionToolhead` (kalico-sim configs, any rust test fixtures, `config/` examples our benches derive from): delete the legacy `[printer]` limit keys and add equivalent sections, e.g.:

```ini
[limit gantry]
axes: x, y
max_velocity: 300
max_accel: 3000

[limit z]
axes: z
max_velocity: 15
max_accel: 100
```

(Carry each fixture's own numbers over; `max_z_velocity/max_z_accel` values become the `[limit z]` section. Drop `square_corner_velocity` — nothing replaces it.)

- [ ] **Step 3: Commit** — `chore: migrate config fixtures to [limit] sections`

---

### Task 8: Legacy fossil sweep

- [ ] **Step 1: Grep and delete or justify each hit:**

Run: `grep -rn "square_corner\|centripetal\|junction_deviation_mm\|max_accel_to_decel\|a_centripetal" rust/ klippy/ --include="*.rs" --include="*.py" | grep -v test/klippy`

Expected survivors only: legacy `toolhead.py` base-class code (legacy `ToolHead` path keeps its behavior), the `junction.rs` `scv impulse budget` comment (reword it — the concept is gone), and this plan/spec under `docs/`. Everything else dies: `PointGeom.kappa` and `ArclengthGrid.kappa` if now unread (check `grep -rn "\.kappa" rust/temporal/`), unused `Axis` enum variants in verify, dead `orig_cfg` keys.

- [ ] **Step 2: Re-run everything** — `cargo nextest run` from `rust/` → PASS; `cargo test --doc` if any doc examples were touched.
- [ ] **Step 3: Commit** — `chore: delete legacy limit fossils (centripetal, SCV, jerk broadcast)`

---

### Task 9: End-to-end verification

- [ ] **Step 1:** `cargo nextest run` from `rust/` → full PASS. `cargo fmt --all --check` → clean.
- [ ] **Step 2:** Boot a simulated printer via the **kalico-sim** skill with a migrated fixture; verify: clean startup (no config errors), a homing + square + diagonal G-code runs, and a fixture with a deliberate `[printer] max_accel` errors at startup naming `[limit]` sections.
- [ ] **Step 3:** In the sim, exercise `SET_VELOCITY_LIMIT ACCEL=500` mid-stream and `RESET_VELOCITY_LIMIT`; verify no planner error and visibly slower motion under the cap (compare trajectory durations via sim step counts).
- [ ] **Step 4:** Diagonal-vs-axis sanity check (the √2 bug death): with `[limit gantry] axes: x,y max_accel: 3000`, a pure-X 100 mm move and a 45° 100 mm move should now show the *same* peak toolhead acceleration in planner output (previously the diagonal reached ~4243). A unit-level assertion of this already lives in the Task 3 test updates; this step is the integration confirmation.
- [ ] **Step 4b:** Corner full-stop check: a two-move L-shaped G-code (90° corner) must come to rest at the corner (velocity → 0 at the junction). Print-time regression on polyline G-code versus pre-rework is **expected and accepted** — it is the honest cost of the deleted junction-deviation pretense, recovered by the future upstream corner-blending plan.
- [ ] **Step 5: Commit** any fixes, then final commit — `feat: unified [limit] sections end-to-end`

---

## Self-review notes (spec → plan coverage)

- `[limit]` named coordinate sets, singleton=box / multi=norm: Tasks 2-6 ✓
- Mandatory coverage, fail loudly at load: Task 2 (`LimitsError`), Task 5 (eager validation in `init_planner`), Task 6 (config errors) ✓
- Norm rows in solver (linear velocity rows, SOC accel rows): Task 3 ✓
- Centripetal cap deleted, subsumed by accel-norm orthogonal component: Tasks 1-3, 8 ✓
- SCV and the whole junction-deviation pretense deleted; sharp junctions are full stops until upstream corner blending exists (separate future plan): Tasks 3, 5, 8 ✓
- Jerk broadcast (`2×accel` hardcode) dies; jerk is a declarable cap with a documented per-section default: Task 4 ✓
- Legacy `[printer]` keys rejected with errors naming the replacement: Task 6 ✓
- Planner stays a pure function: unchanged — config still arrives as data; no new side channels ✓
- Overlapping sets (60k gantry + 40k Y) work via row intersection: no special code — covered by Task 2 `mvc_b` test ✓
- Deferred consciously: `steppers:` key, axis `e`, observability reporting (BindingConstraint set indices laid in Task 3 are its groundwork)
