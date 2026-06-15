# Exact-Geometry Curve Reparametrization Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make every smooth (cusp-free) curved segment plan successfully by keeping the cubic geometry exact through the reduce/fit pipeline — sampling the exact curve at an accurately-inverted parameter instead of fitting a low-degree polynomial to position-vs-arc-length `x(s)`.

**Architecture:** In `trajectory/src/reparam.rs::compose_segment`, replace the failing `fit_x_to_arc_length_piece` call (3-D `x(s)` polynomial fit, degree-capped at 5, no recourse) with: (1) an accurate `s→u` inversion (per-segment arc-length table seed + one analytic-derivative Newton step, with a zero-tangent/cusp guard), and (2) a per-piece fit of `x(u(s(t)))` in time, sampling the **exact** curve at the inverted `u`. Genuine cusps still fail loud via a new typed error. The downstream time-domain fitter (`fit_and_split`) and the temporal solver are unchanged.

**Tech Stack:** Rust (`rust/` workspace), `nurbs` + `trajectory` crates, f64 host build, `cargo nextest`. Spec: `docs/superpowers/specs/2026-06-14-exact-geometry-curve-reparametrization-design.md`.

**Validation basis:** A throwaway spike (reverted) ran all four crashing arches through the real `shape_batch` pipeline with this exact mechanic: all passed, start/end joints exact to machine precision, geometry contribution ≤ 0.1 µm, no regressions.

---

## File Structure

| File | Responsibility | Change |
|---|---|---|
| `rust/trajectory/src/lib.rs` | `ShapeError` enum | Add `ZeroTangent { index, u }` variant. |
| `rust/trajectory/src/reparam.rs` | arc-length reparametrization (`compose_segment`) | New helpers `invert_s_to_u`, `solve_power_basis`, `fit_position_of_t`; rewrite the non-near-zero branch; drop the unused tolerance param. |
| `rust/trajectory/src/reparam/tests.rs` | unit tests for reparam | Add inversion + per-piece-fit unit tests. (Module is already `#[cfg(test)] mod tests;` at the bottom of reparam.rs.) |
| `rust/trajectory/src/beta.rs` | batch driver | Build the per-segment arc-length table at the accuracy target; remove `arc_fit_tolerance`; update the `compose_segment` call. |
| `rust/trajectory/tests/curve_reparam_regression.rs` | NEW end-to-end regression | Four-arch fit success + geometric accuracy + exact joints + cusp-still-rejected. |
| `rust/nurbs/src/algebra.rs` | algebra | Remove dead `fit_x_to_arc_length_piece`. |
| `rust/nurbs/tests/fit_x_to_arc_length_piece.rs` | (dead test) | Delete. |
| `rust/trajectory/tests/experiment_smooth_curve_fit.rs` | scratch | Delete. |

**Constants (defined once in `reparam.rs`):**
```rust
/// Arc-length table accuracy for the s→u inversion (built once per segment).
const ARC_TABLE_TOL: f64 = 1e-9;
const ARC_TABLE_SAMPLES: usize = 16384;
/// |x'(u)| (mm per unit u) below this is a cusp — not a smooth segment.
const TANGENT_SPEED_FLOOR: f64 = 1e-6;
/// Per-piece time-domain position fit: degree and accuracy gate (geometry budget,
/// far below the 5 µm user fit_tolerance_mm).
const POS_FIT_DEGREE: usize = 9;
const POS_FIT_TOL_MM: f64 = 1e-6;
const POS_FIT_MAX_SUBDIV: usize = 8;
```

---

## Task 1: Add the `ZeroTangent` error variant

**Files:**
- Modify: `rust/trajectory/src/lib.rs` (the `ShapeError` enum, around line 81)

- [ ] **Step 1: Add the variant**

In `ShapeError` (after the `Algebra { index, detail }` variant), add:
```rust
    #[error(
        "segment {index}: zero tangent (cusp) at u={u} — the curve has a stationary \
         point and is not plannable as a single smooth segment"
    )]
    ZeroTangent { index: usize, u: f64 },
```

- [ ] **Step 2: Build to verify it compiles**

Run: `cd rust && cargo build -p trajectory`
Expected: compiles (any unused-variant warning is fine; it is used in Task 4).

- [ ] **Step 3: Commit**
```bash
cd rust && git add trajectory/src/lib.rs
git commit -m "feat(trajectory): add ShapeError::ZeroTangent for cusp segments"
```

---

## Task 2: Accurate `s→u` inversion with cusp guard

**Files:**
- Modify: `rust/trajectory/src/reparam.rs`
- Test: `rust/trajectory/src/reparam/tests.rs`

The inversion seeds from the existing linear-interpolation table lookup, then takes one Newton step using the analytic curve speed `|x'(u)|` (from `vector_derivative`). A near-zero speed is a cusp → `ZeroTangent`.

- [ ] **Step 1: Write the failing test**

Add to `rust/trajectory/src/reparam/tests.rs`:
```rust
use nurbs::VectorNurbs;

fn arch() -> VectorNurbs<f64, 3> {
    VectorNurbs::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![
            [150.0, 150.0, 5.0],
            [150.0, 180.0, 5.0],
            [200.0, 180.0, 5.0],
            [200.0, 150.0, 5.0],
        ],
    )
    .unwrap()
}

#[test]
fn invert_s_to_u_is_accurate() {
    let curve = arch();
    let deriv = nurbs::eval::vector_derivative(&curve);
    // Production table (the accuracy under test) and a finer reference for ground truth.
    let table = nurbs::arc_length::build_arc_length_table_vector(
        &curve, super::ARC_TABLE_TOL, super::ARC_TABLE_SAMPLES,
    )
    .unwrap();
    let tv = table.as_view();
    let reference = nurbs::arc_length::build_arc_length_table_vector(&curve, 1e-12, 16384).unwrap();
    let rv = reference.as_view();

    for i in 1..20 {
        let u_true = i as f64 / 20.0;
        let s = nurbs::arc_length::arc_length_from_param(&rv, u_true);
        let u_got = super::invert_s_to_u(&tv, &deriv, s, 0).expect("smooth curve must invert");
        assert!((u_got - u_true).abs() < 1e-7, "u err at u={u_true}: got {u_got}");
        let p_got = nurbs::eval::vector_eval(&curve, u_got);
        let p_true = nurbs::eval::vector_eval(&curve, u_true);
        let perr = ((p_got[0] - p_true[0]).powi(2)
            + (p_got[1] - p_true[1]).powi(2)
            + (p_got[2] - p_true[2]).powi(2))
        .sqrt();
        assert!(perr < 1e-6, "pos err {perr} mm at u={u_true}");
    }
}

#[test]
fn invert_s_to_u_rejects_cusp() {
    // P1==P0==P2 => zero start tangent (exact cusp).
    let curve = VectorNurbs::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [5.0, 0.0, 0.0]],
    )
    .unwrap();
    let deriv = nurbs::eval::vector_derivative(&curve);
    let table = nurbs::arc_length::build_arc_length_table_vector(
        &curve, super::ARC_TABLE_TOL, super::ARC_TABLE_SAMPLES,
    )
    .unwrap();
    let tv = table.as_view();
    // s≈0 maps to u≈0 where the tangent vanishes.
    let err = super::invert_s_to_u(&tv, &deriv, 1e-6, 3);
    assert!(matches!(err, Err(crate::ShapeError::ZeroTangent { index: 3, .. })));
}
```

- [ ] **Step 2: Run to verify it fails**

Run: `cd rust && cargo nextest run -p trajectory -E 'test(invert_s_to_u)'`
Expected: FAIL to compile (`invert_s_to_u`, `ARC_TABLE_TOL`, `ARC_TABLE_SAMPLES` not defined).

- [ ] **Step 3: Add the constants and the helper**

At the top of `rust/trajectory/src/reparam.rs` (after the existing `NEAR_ZERO_V` const), add the constant block from the **File Structure** section above. Then add:
```rust
use nurbs::VectorNurbs;

/// Invert arc length `s` to curve parameter `u`: seed from the table, then one
/// Newton step against the analytic curve speed. Returns `ZeroTangent` at a cusp.
fn invert_s_to_u(
    table: &nurbs::ArcLengthTableRef<'_, f64>,
    deriv: &VectorNurbs<f64, 3>,
    s: f64,
    index: usize,
) -> Result<f64, crate::ShapeError> {
    let s_clamped = s.clamp(0.0, table.s_max());
    let u0 = nurbs::arc_length::param_from_arc_length(table, s_clamped);
    let d0 = nurbs::eval::vector_eval(deriv, u0);
    let speed = (d0[0] * d0[0] + d0[1] * d0[1] + d0[2] * d0[2]).sqrt();
    if speed < TANGENT_SPEED_FLOOR {
        return Err(crate::ShapeError::ZeroTangent { index, u: u0 });
    }
    let s0 = nurbs::arc_length::arc_length_from_param(table, u0);
    let u = u0 - (s0 - s_clamped) / speed;
    Ok(u.clamp(0.0, table.u_max()))
}
```

- [ ] **Step 4: Run to verify it passes**

Run: `cd rust && cargo nextest run -p trajectory -E 'test(invert_s_to_u)'`
Expected: PASS (2 tests).

- [ ] **Step 5: Commit**
```bash
cd rust && git add trajectory/src/reparam.rs trajectory/src/reparam/tests.rs
git commit -m "feat(reparam): accurate s->u inversion with cusp guard"
```

---

## Task 3: Per-piece time-domain fit of the exact curve

**Files:**
- Modify: `rust/trajectory/src/reparam.rs`
- Test: `rust/trajectory/src/reparam/tests.rs`

`fit_position_of_t` samples the **exact** curve at Chebyshev time-nodes (via `s(t)` → `invert_s_to_u`), fits a degree-`POS_FIT_DEGREE` power-basis polynomial per axis about `t_lo`, checks the residual against the exact curve at dense points, and bisects the time interval (splitting `s_of_t`) if the residual exceeds `POS_FIT_TOL_MM`. `solve_power_basis` is a small Gaussian-elimination Vandermonde solve in the shifted monomial basis `(t - origin)^k`.

- [ ] **Step 1: Write the failing test**

Add to `rust/trajectory/src/reparam/tests.rs`:
```rust
use nurbs::bezier::BezierPiece;

#[test]
fn fit_position_of_t_tracks_exact_curve() {
    let curve = arch();
    let deriv = nurbs::eval::vector_derivative(&curve);
    let table = nurbs::arc_length::build_arc_length_table_vector(
        &curve, super::ARC_TABLE_TOL, super::ARC_TABLE_SAMPLES,
    )
    .unwrap();
    let tv = table.as_view();
    let s_max = tv.s_max();

    // A constant-speed s(t): s(t) = (s_max) * t over t in [0,1] (one piece).
    let s_of_t = BezierPiece { u_start: 0.0, u_end: 1.0, coeffs: vec![0.0, s_max] };

    let pieces = super::fit_position_of_t(&curve, &deriv, &tv, &s_of_t, 0)
        .expect("smooth curve must fit");
    assert!(!pieces.is_empty());

    // Sample the fitted result densely and compare to the exact curve.
    let mut max_err = 0.0_f64;
    for j in 0..=400 {
        let t = j as f64 / 400.0;
        // locate the fitted piece covering t
        let arr = pieces
            .iter()
            .find(|a| t >= a[0].u_start - 1e-12 && t <= a[0].u_end + 1e-12)
            .unwrap();
        let got = [arr[0].evaluate(t), arr[1].evaluate(t), arr[2].evaluate(t)];
        let s = s_of_t.evaluate(t);
        let u = super::invert_s_to_u(&tv, &deriv, s, 0).unwrap();
        let truth = nurbs::eval::vector_eval(&curve, u);
        let e = ((got[0] - truth[0]).powi(2)
            + (got[1] - truth[1]).powi(2)
            + (got[2] - truth[2]).powi(2))
        .sqrt();
        max_err = max_err.max(e);
    }
    assert!(max_err < super::POS_FIT_TOL_MM * 2.0, "max pos err {max_err} mm");
    // Joints land on the exact curve endpoints.
    let first = &pieces[0];
    let p0 = [first[0].evaluate(0.0), first[1].evaluate(0.0), first[2].evaluate(0.0)];
    let c0 = nurbs::eval::vector_eval(&curve, 0.0);
    assert!((p0[0] - c0[0]).abs() < 1e-9 && (p0[1] - c0[1]).abs() < 1e-9);
}
```

- [ ] **Step 2: Run to verify it fails**

Run: `cd rust && cargo nextest run -p trajectory -E 'test(fit_position_of_t)'`
Expected: FAIL to compile (`fit_position_of_t`, `solve_power_basis` not defined).

- [ ] **Step 3: Add the solver and the fitter**

Add to `rust/trajectory/src/reparam.rs`:
```rust
/// Solve for power-basis coefficients c[k] of p(x) = Σ c[k] (x - origin)^k that
/// interpolate (nodes[i], vals[i]). Square system (len == degree+1), Gaussian
/// elimination with partial pivoting. Nodes must be distinct.
fn solve_power_basis(nodes: &[f64], vals: &[f64], origin: f64) -> Vec<f64> {
    let n = nodes.len();
    let mut a = vec![vec![0.0_f64; n + 1]; n];
    for i in 0..n {
        let dx = nodes[i] - origin;
        let mut p = 1.0;
        for k in 0..n {
            a[i][k] = p;
            p *= dx;
        }
        a[i][n] = vals[i];
    }
    for col in 0..n {
        let mut piv = col;
        for r in (col + 1)..n {
            if a[r][col].abs() > a[piv][col].abs() {
                piv = r;
            }
        }
        a.swap(col, piv);
        let d = a[col][col];
        for r in 0..n {
            if r == col {
                continue;
            }
            let f = a[r][col] / d;
            for c in col..=n {
                a[r][c] -= f * a[col][c];
            }
        }
    }
    (0..n).map(|k| a[k][n] / a[k][k]).collect()
}

/// Fit position-over-time x(t) by sampling the EXACT curve at the inverted
/// parameter. Bisects the time interval on residual miss. Returns one or more
/// contiguous [x,y,z] power-basis pieces over s_of_t's time domain.
fn fit_position_of_t(
    curve: &VectorNurbs<f64, 3>,
    deriv: &VectorNurbs<f64, 3>,
    table: &nurbs::ArcLengthTableRef<'_, f64>,
    s_of_t: &BezierPiece<f64>,
    index: usize,
) -> Result<Vec<[BezierPiece<f64>; 3]>, crate::ShapeError> {
    fit_position_of_t_rec(curve, deriv, table, s_of_t, index, 0)
}

fn fit_position_of_t_rec(
    curve: &VectorNurbs<f64, 3>,
    deriv: &VectorNurbs<f64, 3>,
    table: &nurbs::ArcLengthTableRef<'_, f64>,
    s_of_t: &BezierPiece<f64>,
    index: usize,
    depth: usize,
) -> Result<Vec<[BezierPiece<f64>; 3]>, crate::ShapeError> {
    let t_lo = s_of_t.u_start;
    let t_hi = s_of_t.u_end;
    let n = POS_FIT_DEGREE + 1;

    // Chebyshev nodes in t, sample exact curve at inverted u.
    let mut nodes_t = Vec::with_capacity(n);
    let mut vals: [Vec<f64>; 3] = [Vec::with_capacity(n), Vec::with_capacity(n), Vec::with_capacity(n)];
    let mid = 0.5 * (t_lo + t_hi);
    let half = 0.5 * (t_hi - t_lo);
    for i in 0..n {
        let theta = (i as f64) * std::f64::consts::PI / ((n - 1) as f64);
        let t = (mid + half * theta.cos()).clamp(t_lo, t_hi);
        let s = s_of_t.evaluate(t);
        let u = invert_s_to_u(table, deriv, s, index)?;
        let p = nurbs::eval::vector_eval(curve, u);
        nodes_t.push(t);
        for axis in 0..3 {
            vals[axis].push(p[axis]);
        }
    }

    let axes: [BezierPiece<f64>; 3] = std::array::from_fn(|axis| BezierPiece {
        u_start: t_lo,
        u_end: t_hi,
        coeffs: solve_power_basis(&nodes_t, &vals[axis], t_lo),
    });

    // Residual against the exact curve at dense check points.
    let mut max_err = 0.0_f64;
    let checks = 4 * n;
    for i in 0..=checks {
        let t = t_lo + (t_hi - t_lo) * (i as f64 / checks as f64);
        let s = s_of_t.evaluate(t);
        let u = invert_s_to_u(table, deriv, s, index)?;
        let truth = nurbs::eval::vector_eval(curve, u);
        for axis in 0..3 {
            max_err = max_err.max((axes[axis].evaluate(t) - truth[axis]).abs());
        }
    }

    if max_err <= POS_FIT_TOL_MM {
        return Ok(vec![axes]);
    }
    if depth >= POS_FIT_MAX_SUBDIV {
        return Err(crate::ShapeError::FitFailure {
            index,
            detail: nurbs::algebra::FitError::ToleranceNotReached {
                achieved_mm: max_err,
                at_degree: POS_FIT_DEGREE as u8,
            },
        });
    }
    let (left, right) = nurbs::bezier::split_piece_at(s_of_t, mid);
    let mut out = fit_position_of_t_rec(curve, deriv, table, &left, index, depth + 1)?;
    out.extend(fit_position_of_t_rec(curve, deriv, table, &right, index, depth + 1)?);
    Ok(out)
}
```

- [ ] **Step 4: Run to verify it passes**

Run: `cd rust && cargo nextest run -p trajectory -E 'test(fit_position_of_t)'`
Expected: PASS.

- [ ] **Step 5: Commit**
```bash
cd rust && git add trajectory/src/reparam.rs trajectory/src/reparam/tests.rs
git commit -m "feat(reparam): per-piece time fit of exact curve (sample+fit, subdivide)"
```

---

## Task 4: Wire approach B into `compose_segment` and the batch driver

**Files:**
- Modify: `rust/trajectory/src/reparam.rs` (`compose_segment` non-near-zero branch + signature)
- Modify: `rust/trajectory/src/beta.rs` (table build accuracy; remove `arc_fit_tolerance`; update call)

- [ ] **Step 1: Replace the non-near-zero branch of `compose_segment`**

In `rust/trajectory/src/reparam.rs`, change the signature to drop the now-unused tolerance and to build the derivative once:
```rust
pub fn compose_segment(
    curve: &nurbs::VectorNurbs<f64, 3>,
    table: &nurbs::ArcLengthTableRef<'_, f64>,
    s_pieces: &SOfTPieces,
) -> Result<Vec<[BezierPiece<f64>; 3]>, crate::ShapeError> {
    let deriv = nurbs::eval::vector_derivative(curve);
    let mut result = Vec::with_capacity(s_pieces.pieces.len());

    for (k, s_piece) in s_pieces.pieces.iter().enumerate() {
        if s_pieces.near_zero[k] {
            // UNCHANGED near-zero (constant-position) branch:
            let s_k = s_piece.coeffs[0];
            let u_k = nurbs::arc_length::param_from_arc_length(table, s_k);
            let pos = nurbs::eval::vector_eval(curve, u_k);
            let axes: [BezierPiece<f64>; 3] = std::array::from_fn(|axis| BezierPiece {
                u_start: s_piece.u_start,
                u_end: s_piece.u_end,
                coeffs: vec![pos[axis]],
            });
            result.push(axes);
        } else {
            let pieces = fit_position_of_t(curve, &deriv, table, s_piece, k)?;
            result.extend(pieces);
        }
    }

    Ok(result)
}
```
Delete the old non-near-zero body (the `fit_x_to_arc_length_piece` call, `s_piece_adjusted`, and the `compose_vector_piece` call). Note: `fit_position_of_t` consumes the `s_piece` (which is `s(t)` over its time domain) directly — the previous `s_lo/s_hi` clamping is absorbed by `invert_s_to_u`'s `s.clamp(0.0, s_max())`.

- [ ] **Step 2: Update `beta.rs`**

In `rust/trajectory/src/beta.rs` (around lines 494–506), change the table build accuracy and the call, and remove `arc_fit_tolerance`:
```rust
            let table = nurbs::arc_length::build_arc_length_table_vector(
                curve,
                crate::reparam::ARC_TABLE_TOL,
                crate::reparam::ARC_TABLE_SAMPLES,
            )
            .map_err(|e| ShapeError::ArcLength {
                index: global_idx,
                detail: format!("{e}"),
            })?;

            let composed = crate::reparam::compose_segment(curve, &table.as_view(), &s_pieces)?;
```
Make the two constants visible to `beta.rs`: in `reparam.rs` change `const ARC_TABLE_TOL` / `const ARC_TABLE_SAMPLES` to `pub(crate) const`.

- [ ] **Step 3: Build and run the existing trajectory suite**

Run: `cd rust && cargo nextest run -p trajectory`
Expected: compiles; pre-existing tests pass. (The scratch `experiment_smooth_curve_fit.rs` and `nurbs` dead test are handled in Task 6; they may still reference the old API — that is fine until Task 6, since they are separate crates/targets. If `experiment_smooth_curve_fit.rs` fails to compile because `compose_segment`'s signature is internal, ignore — it does not call it.)

- [ ] **Step 4: Commit**
```bash
cd rust && git add trajectory/src/reparam.rs trajectory/src/beta.rs
git commit -m "feat(reparam): compose via exact geometry; retire x(s) fit + 0.1um constant"
```

---

## Task 5: End-to-end regression test (four arches + accuracy + joints + cusp)

**Files:**
- Create: `rust/trajectory/tests/curve_reparam_regression.rs`

- [ ] **Step 1: Write the test**

Create `rust/trajectory/tests/curve_reparam_regression.rs`:
```rust
//! Regression: smooth curved segments plan successfully and track the exact
//! curve; genuine cusps still fail loud. Guards the fix for the planner_fatal
//! abort on curved (G5) segments.

use nurbs::VectorNurbs;
use temporal::multi::{GridStrategy, SegmentInput};
use trajectory::{AxisChainSet, ShapeBatchInput, ShapeError, ShapeSegmentInput};

fn cubic(p: [[f64; 3]; 4]) -> VectorNurbs<f64, 3> {
    VectorNurbs::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        p.to_vec(),
    )
    .unwrap()
}

fn limits() -> temporal::Limits {
    let sets = temporal::Limits::axis_boxes([500.0; 3], [5_000.0; 3], [100_000.0; 3])
        .sets()
        .to_vec();
    temporal::Limits::try_new(&sets, 3).unwrap()
}

fn run(curve: &VectorNurbs<f64, 3>, feed: f64) -> Result<trajectory::ShapeBatchOutput, ShapeError> {
    let segments = [ShapeSegmentInput {
        temporal: SegmentInput { curve, limits: limits(), followers: &[], virtual_path: None },
        followers: &[],
        feedrate_mm_s: feed,
    }];
    let chains = AxisChainSet::spatial(
        trajectory::CompiledChain::default(),
        trajectory::CompiledChain::default(),
        trajectory::CompiledChain::default(),
    );
    let input = ShapeBatchInput {
        chains: &chains,
        follower_start: &[],
        follower_history: None,
        segments: &segments,
        grid_strategy: GridStrategy::Adaptive { min_n: 20, max_n: 200, target_grid_spacing_mm: 0.5 },
        worker_threads: 1,
        fit_tolerance_mm: 0.005,
        beta_max_iters: 5,
        beta_convergence_ratio: 1.02,
        initial_v: 0.0,
        initial_a: 0.0,
        terminal_v: 0.0,
        start_d2_override: None,
    };
    trajectory::shape_batch(&input)
}

const ARCHES: &[(&str, [[f64; 3]; 4])] = &[
    ("gentle", [[150., 150., 5.], [150., 180., 5.], [200., 180., 5.], [200., 150., 5.]]),
    ("wide", [[100., 100., 5.], [100., 200., 5.], [250., 200., 5.], [250., 100., 5.]]),
    ("tight_loop", [[150., 150., 5.], [165., 210., 5.], [135., 210., 5.], [150., 150.5, 5.]]),
    ("s_curve", [[100., 150., 5.], [160., 150., 5.], [140., 150., 5.], [200., 150., 5.]]),
];

#[test]
fn smooth_arches_plan_successfully() {
    for (name, cps) in ARCHES {
        let curve = cubic(*cps);
        for &feed in &[25.0_f64, 50.0, 100.0] {
            let r = run(&curve, feed);
            assert!(r.is_ok(), "{name} @ {feed}mm/s should plan, got {r:?}");
        }
    }
}

#[test]
fn planned_curve_tracks_geometry_and_joints_exact() {
    for (name, cps) in &[ARCHES[0], ARCHES[2]] {
        let curve = cubic(*cps);
        let out = run(&curve, 25.0).expect("plan ok");
        let seg = &out.segments[0];

        // Dense polyline of the true curve for nearest-distance fidelity.
        let poly: Vec<[f64; 3]> =
            (0..=5000).map(|i| nurbs::eval::vector_eval(&curve, i as f64 / 5000.0)).collect();
        let nearest = |p: [f64; 3]| {
            poly.iter()
                .map(|q| ((p[0]-q[0]).powi(2)+(p[1]-q[1]).powi(2)+(p[2]-q[2]).powi(2)).sqrt())
                .fold(f64::INFINITY, f64::min)
        };

        let (t0, t1) = (seg.t_start, seg.t_end);
        assert!(t1.is_finite() && t1 > t0);
        let mut max_dev = 0.0_f64;
        for i in 0..=200 {
            let t = t0 + (t1 - t0) * (i as f64 / 200.0);
            let p = [seg.axes[0].eval(t), seg.axes[1].eval(t), seg.axes[2].eval(t)];
            max_dev = max_dev.max(nearest(p));
        }
        assert!(max_dev < 0.025, "{name} max geom deviation {max_dev} mm");

        // Joints: planned start/end land on curve(0)/curve(1) to ~machine precision.
        let start = [seg.axes[0].eval(t0), seg.axes[1].eval(t0), seg.axes[2].eval(t0)];
        let end = [seg.axes[0].eval(t1), seg.axes[1].eval(t1), seg.axes[2].eval(t1)];
        assert!(nearest(start) < 1e-6 && nearest(end) < 1e-6, "{name} joints off curve");
    }
}

#[test]
fn exact_cusp_still_fails_loud() {
    // P1==P0==P2 — zero start tangent. Must NOT silently plan.
    let curve = cubic([[0., 0., 0.], [0., 0., 0.], [0., 0., 0.], [5., 0., 0.]]);
    let r = run(&curve, 30.0);
    assert!(r.is_err(), "cusp must fail loud, got {r:?}");
}
```
Note on `seg.axes[i].eval(t)`: confirm the per-axis evaluation method name on the returned segment's NURBS axes (e.g. `nurbs::eval::eval(&seg.axes[i], t)` if `.eval` is not an inherent method). Use whichever the type exposes; the four-arch `is_ok()` test does not depend on it.

- [ ] **Step 2: Run**

Run: `cd rust && cargo nextest run -p trajectory -E 'test(smooth_arches) + test(planned_curve) + test(exact_cusp)'`
Expected: PASS (3 tests). Before this fix all three would fail (arches errored, cusp errored with the opaque fit error rather than being the asserted behavior).

- [ ] **Step 3: Commit**
```bash
cd rust && git add trajectory/tests/curve_reparam_regression.rs
git commit -m "test(trajectory): curved-segment fit regression (arches + joints + cusp)"
```

---

## Task 6: Remove dead code and scratch; full green

**Files:**
- Modify: `rust/nurbs/src/algebra.rs` (delete `fit_x_to_arc_length_piece`)
- Delete: `rust/nurbs/tests/fit_x_to_arc_length_piece.rs`
- Delete: `rust/trajectory/tests/experiment_smooth_curve_fit.rs`

- [ ] **Step 1: Confirm no remaining live callers**

Run: `cd rust && grep -rn "fit_x_to_arc_length_piece" nurbs/src trajectory/src motion-bridge/src`
Expected: only the definition in `nurbs/src/algebra.rs`. (If anything else appears, stop and reassess.)

- [ ] **Step 2: Delete the dead function and its scratch/test files**

- Remove `pub fn fit_x_to_arc_length_piece` (algebra.rs:134–238) and any now-unused private helpers it alone used (check `lagrange_interpolation_pascal_shifted`, `horner_pascal_shifted` — keep if still referenced elsewhere; clippy will flag dead ones).
```bash
cd rust && git rm nurbs/tests/fit_x_to_arc_length_piece.rs trajectory/tests/experiment_smooth_curve_fit.rs
```

- [ ] **Step 3: Full workspace test + lint**

Run: `cd rust && cargo nextest run`
Expected: all pass.

Run: `cd /Users/daniladergachev/Developer/kalico/.worktrees/g2g3g5 && ./scripts/ci.sh quick`
Expected: green (ruff, rust tests, clippy `-D warnings`, fmt, watchdog canary). Fix any clippy dead-code/unused-import fallout from the deletions.

- [ ] **Step 4: Performance guard**

Confirm the arc-length table is built once per segment (in `beta.rs`'s per-segment loop), not per piece. Run the regression once and eyeball timing:
Run: `cd rust && cargo nextest run -p trajectory -E 'test(smooth_arches_plan_successfully)'`
Expected: completes in a few seconds for the 12 plans; no per-piece table construction (review the `compose_segment` body — it takes the table by reference and never calls `build_arc_length_table_vector`).

- [ ] **Step 5: Commit**
```bash
cd rust && git add -A
git commit -m "refactor(nurbs): remove dead fit_x_to_arc_length_piece + scratch tests"
```

---

## Self-Review

**Spec coverage:**
- §3 approach B (exact geometry, fit 1-D bridge) → Tasks 2–4. (Implemented via the spec's spike-validated sample-and-fit mechanic, which the spec sanctions as satisfying the accuracy gate.)
- §4.2 efficient `s→u` inversion (per-segment table + analytic-derivative Newton, no per-piece rebuild) → Task 2 + Task 6 perf guard.
- §4.4 retire the 0.1 µm `arc_fit_tolerance` → Task 4 step 2.
- §4.5 cusp/zero-tangent fail-loud → Task 1 + Task 2 + Task 5 cusp test.
- §4.6 remove dead `fit_x_to_arc_length_piece` → Task 6.
- §6 tests: four-arch regression, accuracy, joints, cusp, full suite, perf, (live bench is a post-merge manual step, not a unit task).

**Placeholder scan:** No TBD/“handle errors”/vague steps; every code step has complete code. The one judgement call (`seg.axes[i].eval` method name) is flagged with the resolution and isolated to the non-load-bearing assertion.

**Type consistency:** `invert_s_to_u(table, deriv, s, index) -> Result<f64, ShapeError>`, `fit_position_of_t(curve, deriv, table, s_of_t, index) -> Result<Vec<[BezierPiece;3]>, ShapeError>`, `compose_segment(curve, table, s_pieces)` and the `ARC_TABLE_TOL`/`ARC_TABLE_SAMPLES`/`POS_FIT_*`/`TANGENT_SPEED_FLOOR` constants are used identically across Tasks 2–6. `ShapeError::ZeroTangent { index, u }` is defined in Task 1 and matched in Tasks 2 & 5.
