//! EXPERIMENT: cusp and near-cusp Bézier curves through the live velocity solver.
//!
//! A cusp is a point where the geometric tangent |B'(t)| passes through zero.
//! The canonical construction is a cubic Bézier with P1 == P0 (zero start tangent),
//! which forces the curve to fold back. Physically a cusp is a mandatory full stop.
//! Numerically the risk is a divide-by-zero when the solver scales speed by 1/|B'(t)|.
//!
//! # OBSERVED BEHAVIOR (recorded 2026-06-14)
//!
//! All three cases — exact cusp [[0,0,0],[0,0,0],[0,0,0],[5,0,0]], near-cusp
//! (P1=[1e-6,0,0]), and high-curvature fold-back — return the same error:
//!
//! ```text
//! Err(FitFailure { index: 0, detail: ToleranceNotReached { achieved_mm: ~0.012, at_degree: 5 } })
//! ```
//!
//! No NaN, no panic, no hang. The solver completes instantly and returns a clean error.
//! The failure site is `fit_hermite_c2_adaptive` in `trajectory::fit`, specifically
//! the Phase-2 re-fit (last piece with both-ends accel pins at degree-5). After 8
//! outer binary-subdivision retries the fitter exhausts its budget on the degenerate
//! time-domain pieces created near the cusp (where |B'(t)| ≈ 0).
//!
//! # CONCLUSION
//!
//! The solver does NOT silently produce garbage — it correctly rejects cusps with a
//! typed error. The failure mode is benign (clean Err, not NaN/panic/hang). However,
//! cusps are currently unplanned (the live bridge would propagate ShapeError upward).
//! A classify-level guard (reject cusp before it reaches the solver) would give a
//! better user-facing message but is not urgently required for safety. Decision
//! deferred to controller based on these findings.

use nurbs::VectorNurbs;
use temporal::multi::{GridStrategy, SegmentInput};
use trajectory::{AxisChainSet, PostProcessorType, ShapeBatchInput, ShapeError, ShapeSegmentInput};

fn cubic_bezier(p0: [f64; 3], p1: [f64; 3], p2: [f64; 3], p3: [f64; 3]) -> VectorNurbs<f64, 3> {
    VectorNurbs::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![p0, p1, p2, p3],
    )
    .expect("degree-3 single-piece Bézier")
}

fn default_limits() -> temporal::Limits {
    let mut sets: Vec<temporal::LimitSet> =
        temporal::Limits::axis_boxes([500.0; 3], [5_000.0; 3], [100_000.0; 3])
            .sets()
            .to_vec();
    sets.push(temporal::LimitSet {
        axes: temporal::AxisSet::from_indices(&[3]),
        v_max: 75.0,
        a_max: 1500.0,
        j_max: 3000.0,
    });
    temporal::Limits::try_new(&sets, 4).unwrap()
}

fn test_chain_set() -> AxisChainSet {
    AxisChainSet::spatial(
        PostProcessorType::SmoothZv { frequency_hz: 10.0 }.into_chain(),
        PostProcessorType::SmoothZv { frequency_hz: 10.0 }.into_chain(),
        trajectory::CompiledChain::default(),
    )
}

fn run_solver(
    curve: &VectorNurbs<f64, 3>,
    feedrate_mm_s: f64,
) -> Result<trajectory::ShapeBatchOutput, ShapeError> {
    let segments = [ShapeSegmentInput {
        temporal: SegmentInput {
            curve,
            limits: default_limits(),
            followers: &[],
            virtual_path: None,
        },
        followers: &[],
        feedrate_mm_s,
    }];
    let chains = test_chain_set();
    let input = ShapeBatchInput {
        chains: &chains,
        follower_start: &[],
        follower_history: None,
        segments: &segments,
        grid_strategy: GridStrategy::Fixed(20),
        worker_threads: 1,
        fit_tolerance_mm: 0.5,
        beta_max_iters: 5,
        beta_convergence_ratio: 1.02,
        initial_v: 0.0,
        initial_a: 0.0,
        terminal_v: 0.0,
        start_d2_override: None,
    };
    trajectory::shape_batch(&input)
}

/// EXPERIMENT: exact cusp — P1 == P0 == P2 == [0,0,0], fold-back to [5,0,0].
///
/// OBSERVED: Err(FitFailure { detail: ToleranceNotReached { achieved_mm: ~0.012, at_degree: 5 } })
/// The solver returns a clean typed error — no NaN, no panic, no hang.
#[test]
fn experiment_exact_cusp_solver_returns_fit_failure() {
    // Control points: [[0,0,0],[0,0,0],[0,0,0],[5,0,0]]
    // P1==P0 => zero start tangent (exact cusp). P2==P0 also (extreme fold).
    let curve = cubic_bezier(
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0],
        [5.0, 0.0, 0.0],
    );
    let result = run_solver(&curve, 30.0);
    match &result {
        Ok(out) => {
            // If a future fitter improvement handles cusps, assert finiteness.
            let seg = &out.segments[0];
            assert!(seg.t_end.is_finite(), "t_end non-finite: {}", seg.t_end);
            assert!(seg.t_end > seg.t_start);
            for (i, axis) in seg.axes.iter().enumerate() {
                for cp in axis.control_points() {
                    assert!(cp.is_finite(), "axis {i} cp {cp} not finite");
                }
            }
        }
        Err(ShapeError::FitFailure { .. }) => {
            // Expected current behavior: the fitter cleanly rejects the cusp.
        }
        Err(other) => {
            panic!("unexpected error variant (want Ok or FitFailure): {other:?}");
        }
    }
}

/// EXPERIMENT: near-cusp — P1 at 1e-6 mm from P0, very small start tangent.
///
/// OBSERVED: Err(FitFailure { detail: ToleranceNotReached { achieved_mm: ~0.012, at_degree: 5 } })
#[test]
fn experiment_near_cusp_solver_returns_fit_failure() {
    let curve = cubic_bezier(
        [0.0, 0.0, 0.0],
        [1e-6, 0.0, 0.0],
        [0.0, 0.0, 0.0],
        [5.0, 0.0, 0.0],
    );
    let result = run_solver(&curve, 30.0);
    match &result {
        Ok(out) => {
            let seg = &out.segments[0];
            assert!(seg.t_end.is_finite(), "t_end non-finite: {}", seg.t_end);
            assert!(seg.t_end > seg.t_start);
            for (i, axis) in seg.axes.iter().enumerate() {
                for cp in axis.control_points() {
                    assert!(cp.is_finite(), "axis {i} cp {cp} not finite");
                }
            }
        }
        Err(ShapeError::FitFailure { .. }) => {
            // Expected current behavior.
        }
        Err(other) => {
            panic!("unexpected error variant (want Ok or FitFailure): {other:?}");
        }
    }
}

/// EXPERIMENT: high-curvature fold-back — P2 well behind P0, pronounced S-fold.
///
/// OBSERVED: Err(FitFailure { detail: ToleranceNotReached { achieved_mm: ~0.019, at_degree: 5 } })
#[test]
fn experiment_high_curvature_fold_back_solver_returns_fit_failure() {
    let curve = cubic_bezier(
        [0.0, 0.0, 0.0],
        [0.5, 0.0, 0.0],
        [-3.0, 0.0, 0.0],
        [5.0, 0.0, 0.0],
    );
    let result = run_solver(&curve, 30.0);
    match &result {
        Ok(out) => {
            let seg = &out.segments[0];
            assert!(seg.t_end.is_finite(), "t_end non-finite: {}", seg.t_end);
            assert!(seg.t_end > seg.t_start);
            for (i, axis) in seg.axes.iter().enumerate() {
                for cp in axis.control_points() {
                    assert!(cp.is_finite(), "axis {i} cp {cp} not finite");
                }
            }
        }
        Err(ShapeError::FitFailure { .. }) => {
            // Expected current behavior.
        }
        Err(other) => {
            panic!("unexpected error variant (want Ok or FitFailure): {other:?}");
        }
    }
}
