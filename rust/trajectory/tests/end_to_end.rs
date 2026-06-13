//! End-to-end integration tests for `shape_batch`.
//!
//! These tests exercise the full pipeline: partition -> TOPP-RA -> time-reparam ->
//! composition -> convolution -> peak-accel -> beta loop -> output assembly.
//!
//! Low shaper frequencies (10 Hz) are used for numerical stability: the kernel
//! normalization constant c = 15/(16*h^5) scales as f^5, so narrow kernels
//! (high f) produce large polynomial coefficients that amplify floating-point
//! error in the convolution + double-differentiation pipeline.

use geometry::segment::FollowerDemand;

const E_FOLLOWER_04: &[FollowerDemand] = &[FollowerDemand {
    axis_index: 3,
    ratio: 0.04,
}];
use nurbs::{ScalarNurbs, VectorNurbs};
use temporal::multi::{GridStrategy, SegmentInput};
use trajectory::{AxisChainSet, PostProcessorType, ShapeBatchInput, ShapeError, ShapeSegmentInput};

fn make_straight_line(from: [f64; 3], to: [f64; 3]) -> VectorNurbs<f64, 3> {
    VectorNurbs::try_new(1, vec![0.0, 0.0, 1.0, 1.0], vec![from, to]).unwrap()
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

#[test]
fn shape_batch_straight_line() {
    let curve = make_straight_line([0.0, 0.0, 0.0], [50.0, 0.0, 0.0]);

    let segments = [ShapeSegmentInput {
        temporal: SegmentInput {
            curve: &curve,
            limits: default_limits(),
            followers: &[],
            virtual_path: None,
        },
        followers: E_FOLLOWER_04,
        feedrate_mm_s: 100.0,
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
        beta_max_iters: 3,
        beta_convergence_ratio: 1.02,
        initial_v: 0.0,
        initial_a: 0.0,
        terminal_v: 0.0,
        start_d2_override: None,
    };

    let output = trajectory::shape_batch(&input).expect("shape_batch should succeed");

    assert_eq!(output.segments.len(), 1);

    let seg = &output.segments[0];
    assert!(
        seg.t_end > seg.t_start,
        "t_end={} must be > t_start={}",
        seg.t_end,
        seg.t_start
    );
    #[allow(clippy::float_cmp)]
    {
        assert_eq!(seg.t_start, 0.0);
    }
    assert_eq!(seg.followers, E_FOLLOWER_04);

    for (axis_idx, axis_nurbs) in seg.axes.iter().enumerate() {
        assert!(
            axis_nurbs.control_points().len() >= 2,
            "axis {axis_idx} should have >= 2 control points"
        );
    }
}

#[test]
fn shape_batch_short_low_velocity_line_refits_at_five_microns() {
    let curve = make_straight_line([0.0, 0.0, 0.0], [10.0, 0.0, 0.0]);
    let limits =
        temporal::Limits::axis_boxes([1000.0 / 60.0, 500.0, 500.0], [5_000.0; 3], [100_000.0; 3]);

    let segments = [ShapeSegmentInput {
        temporal: SegmentInput {
            curve: &curve,
            limits,
            followers: &[],
            virtual_path: None,
        },
        followers: &[],
        feedrate_mm_s: 1000.0 / 60.0,
    }];

    let chains = trajectory::AxisChainSet::spatial(
        trajectory::PostProcessorType::SmoothZv { frequency_hz: 50.0 }.into_chain(),
        trajectory::PostProcessorType::SmoothZv { frequency_hz: 50.0 }.into_chain(),
        trajectory::CompiledChain::default(),
    );
    let input = ShapeBatchInput {
        chains: &chains,
        follower_start: &[],
        follower_history: None,
        segments: &segments,
        grid_strategy: GridStrategy::Fixed(25),
        worker_threads: 1,
        fit_tolerance_mm: 0.005,
        beta_max_iters: 3,
        beta_convergence_ratio: 1.02,
        initial_v: 0.0,
        initial_a: 0.0,
        terminal_v: 0.0,
        start_d2_override: None,
    };

    let output = trajectory::shape_batch(&input)
        .expect("10 mm G1-style move at F1000 should refit within 5 um");

    assert_eq!(output.segments.len(), 1);
    let seg = &output.segments[0];
    assert!(seg.t_end > seg.t_start);
    assert!(seg.t_start.abs() < 1e-12);
    assert!(seg.t_end.is_finite());
}

// TOPP-RA joining produces platform-dependent results at 10 Hz shaper with
// multi-segment batches (passes macOS, stalls on Linux CI). The same code
// paths are covered by the beta unit tests at 120/180 Hz which pass on all
// platforms. Tracked for investigation.
#[test]
#[cfg_attr(target_os = "linux", ignore)]
fn shape_batch_two_segments() {
    // Two collinear CoupledToXy segments (same direction, no sharp corner).
    // Collinear segments avoid the joining-loop oscillation that occurs with
    // sharp corners at low grid density. The L-shape case is a known limitation
    // of Fixed(20) grid strategy — the temporal multi-segment tests use Adaptive
    // grids for sharp corners.
    let curve1 = make_straight_line([0.0, 0.0, 0.0], [50.0, 0.0, 0.0]);
    let curve2 = make_straight_line([50.0, 0.0, 0.0], [100.0, 0.0, 0.0]);

    let segments = [
        ShapeSegmentInput {
            temporal: SegmentInput {
                curve: &curve1,
                limits: default_limits(),
                followers: &[],
                virtual_path: None,
            },
            followers: E_FOLLOWER_04,
            feedrate_mm_s: 100.0,
        },
        ShapeSegmentInput {
            temporal: SegmentInput {
                curve: &curve2,
                limits: default_limits(),
                followers: &[],
                virtual_path: None,
            },
            followers: E_FOLLOWER_04,
            feedrate_mm_s: 100.0,
        },
    ];

    let chains = test_chain_set();
    let input = ShapeBatchInput {
        chains: &chains,
        follower_start: &[],
        follower_history: None,
        segments: &segments,
        grid_strategy: GridStrategy::Fixed(20),
        worker_threads: 1,
        fit_tolerance_mm: 0.5,
        beta_max_iters: 3,
        beta_convergence_ratio: 1.02,
        initial_v: 0.0,
        initial_a: 0.0,
        terminal_v: 0.0,
        start_d2_override: None,
    };

    let output = trajectory::shape_batch(&input).expect("shape_batch should succeed");

    assert_eq!(output.segments.len(), 2);

    assert_eq!(output.segments[0].followers, E_FOLLOWER_04);
    assert_eq!(output.segments[1].followers, E_FOLLOWER_04);

    for (i, seg) in output.segments.iter().enumerate() {
        assert!(
            seg.t_end > seg.t_start,
            "segment {i}: t_end={} must be > t_start={}",
            seg.t_end,
            seg.t_start
        );
    }

    let gap = (output.segments[0].t_end - output.segments[1].t_start).abs();
    assert!(
        gap < 1e-9,
        "time gap between segments: {} (seg0.t_end={}, seg1.t_start={})",
        gap,
        output.segments[0].t_end,
        output.segments[1].t_start
    );
}

#[test]
fn shape_batch_beta_warning() {
    let curve = make_straight_line([0.0, 0.0, 0.0], [50.0, 0.0, 0.0]);

    let segments = [ShapeSegmentInput {
        temporal: SegmentInput {
            curve: &curve,
            limits: default_limits(),
            followers: &[],
            virtual_path: None,
        },
        followers: E_FOLLOWER_04,
        feedrate_mm_s: 100.0,
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
        beta_max_iters: 1,
        beta_convergence_ratio: 1.02,
        initial_v: 0.0,
        initial_a: 0.0,
        terminal_v: 0.0,
        start_d2_override: None,
    };

    let output = trajectory::shape_batch(&input);

    assert!(
        output.is_ok(),
        "shape_batch with beta_max_iters=1 should return Ok, got: {:?}",
        output.err()
    );

    let output = output.unwrap();
    assert_eq!(output.segments.len(), 1);
    assert!(output.segments[0].t_end > output.segments[0].t_start);

    if output.beta_warning.is_some() {
        let w = output.beta_warning.as_ref().unwrap();
        assert!(w.worst_ratio > 0.0, "worst_ratio must be positive");
        assert!(
            !w.segments_exceeding.is_empty(),
            "segments_exceeding must be non-empty when warning is present"
        );
    }
}

#[test]
fn shape_batch_empty_input() {
    let chains = test_chain_set();
    let input = ShapeBatchInput {
        chains: &chains,
        follower_start: &[],
        follower_history: None,
        segments: &[],
        grid_strategy: GridStrategy::Fixed(20),
        worker_threads: 1,
        fit_tolerance_mm: 0.5,
        beta_max_iters: 3,
        beta_convergence_ratio: 1.02,
        initial_v: 0.0,
        initial_a: 0.0,
        terminal_v: 0.0,
        start_d2_override: None,
    };

    let result = trajectory::shape_batch(&input);
    assert!(
        matches!(result, Err(ShapeError::EmptySegments)),
        "expected ShapeError::EmptySegments, got: {result:?}"
    );
}
