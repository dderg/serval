use super::*;
use crate::{AxisChainSet, ShapeBatchInput, ShapeSegmentInput};
use geometry::segment::FollowerDemand;

const E_FOLLOWER_04: &[FollowerDemand] = &[FollowerDemand {
    axis_index: 3,
    ratio: 0.04,
}];
use nurbs::VectorNurbs;

fn default_limits() -> temporal::Limits {
    let mut sets: Vec<temporal::LimitSet> = temporal::Limits::axis_boxes(
        [500.0, 500.0, 500.0],
        [5_000.0, 5_000.0, 5_000.0],
        [100_000.0, 100_000.0, 100_000.0],
    )
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

fn default_chain_set() -> AxisChainSet {
    AxisChainSet::spatial(
        crate::PostProcessorType::SmoothZv {
            frequency_hz: 180.0,
        }
        .into_chain(),
        crate::PostProcessorType::SmoothZv {
            frequency_hz: 120.0,
        }
        .into_chain(),
        crate::CompiledChain::default(),
    )
}

fn straight_linear(start: [f64; 3], end: [f64; 3]) -> VectorNurbs<f64, 3> {
    VectorNurbs::try_new(1, vec![0.0, 0.0, 1.0, 1.0], vec![start, end]).unwrap()
}

#[test]
fn single_straight_line_converges() {
    let curve = straight_linear([0.0, 0.0, 0.0], [50.0, 0.0, 0.0]);
    let generous_limits = default_limits();
    let segments = [ShapeSegmentInput {
        temporal: temporal::multi::SegmentInput {
            curve: &curve,
            limits: generous_limits,
            followers: &[],
            virtual_path: None,
        },
        followers: E_FOLLOWER_04,
        feedrate_mm_s: 100.0,
    }];

    let chains = default_chain_set();
    let input = ShapeBatchInput {
        follower_history: None,
        segments: &segments,
        grid_strategy: temporal::multi::GridStrategy::Fixed(10),
        worker_threads: 1,
        chains: &chains,
        follower_start: &[],
        fit_tolerance_mm: 0.5,
        beta_max_iters: 1,
        beta_convergence_ratio: 1.02,
        initial_v: 0.0,
        initial_a: 0.0,
        terminal_v: 0.0,
        start_d2_override: None,
    };

    let output = crate::shape_batch(&input).expect("should succeed");

    assert_eq!(output.segments.len(), 1);
    assert!(output.segments[0].t_end > output.segments[0].t_start);
    assert_eq!(output.segments[0].followers, E_FOLLOWER_04);

    for axis_nurbs in &output.segments[0].axes {
        assert!(
            axis_nurbs.control_points().len() >= 2,
            "shaped axis should have at least 2 control points"
        );
    }
}

#[test]
fn derate_detects_exceeding_peaks() {
    let make_axis = |x_start: f64, x_end: f64| {
        nurbs::bezier::bezier_pieces_to_nurbs(&[nurbs::bezier::BezierPiece {
            u_start: 0.0,
            u_end: 1.0,
            coeffs: vec![x_start, x_end - x_start],
        }])
    };
    let fitted = vec![crate::fit::FittedSegment {
        axes: [
            make_axis(0.0, 100.0),
            make_axis(0.0, 100.0),
            make_axis(0.0, 100.0),
        ],
        t_start: 0.0,
        t_end: 1.0,
        virtual_s_of_t: None,
    }];
    let machine = vec![[5000.0, 5000.0, 5000.0]];
    let peaks_within = vec![[4000.0, 3000.0, 2000.0]];
    let info = compute_derate(&peaks_within, &machine, &fitted);
    assert!(!info.needs_derate);

    let peaks_exceed = vec![[6000.0, 3000.0, 2000.0]];
    let info = compute_derate(&peaks_exceed, &machine, &fitted);
    assert!(info.needs_derate);
    assert!((info.worst_ratio - 1.2).abs() < 1e-10);
    assert_eq!(info.exceeding_indices, vec![0]);
}

#[test]
fn effective_machine_a_max_terminal_known_is_identity() {
    let machine = vec![
        [5_000.0, 5_000.0, 5_000.0],
        [3_000.0, 4_000.0, 2_500.0],
        [1_000.0, 1_500.0, 2_000.0],
    ];
    let effective = effective_machine_a_max(&machine, SafetyMode::TerminalKnown);
    assert_eq!(effective, machine);
}

#[test]
fn effective_machine_a_max_worst_case_only_halves_last_segment() {
    let machine = vec![
        [5_000.0, 5_000.0, 5_000.0],
        [3_000.0, 4_000.0, 2_500.0],
        [1_000.0, 1_500.0, 2_000.0],
    ];
    let effective = effective_machine_a_max(&machine, SafetyMode::WorstCaseFuture);

    assert_eq!(effective[0], machine[0]);
    assert_eq!(effective[1], machine[1]);
    for axis in 0..3 {
        assert!(
            (effective[2][axis] - machine[2][axis] * 0.5).abs() < 1e-12,
            "axis {axis}: expected {} (half of {}), got {}",
            machine[2][axis] * 0.5,
            machine[2][axis],
            effective[2][axis],
        );
    }
}

#[test]
fn effective_machine_a_max_worst_case_single_segment() {
    let machine = vec![[5_000.0, 4_000.0, 3_000.0]];
    let effective = effective_machine_a_max(&machine, SafetyMode::WorstCaseFuture);
    assert_eq!(effective.len(), 1);
    for axis in 0..3 {
        assert!((effective[0][axis] - machine[0][axis] * 0.5).abs() < 1e-12);
    }
}

#[test]
fn effective_machine_a_max_worst_case_empty_is_empty() {
    let machine: Vec<[f64; 3]> = vec![];
    let effective = effective_machine_a_max(&machine, SafetyMode::WorstCaseFuture);
    assert!(effective.is_empty());
}

#[test]
fn jerk_limited_z_move_converges_under_worst_case_derate() {
    let curve = straight_linear([0.0, 0.0, 0.0], [0.0, 0.0, 10.0]);
    let sets = vec![
        temporal::LimitSet {
            axes: temporal::AxisSet::from_indices(&[0, 1]),
            v_max: 800.0,
            a_max: 30_000.0,
            j_max: 60_000.0,
        },
        temporal::LimitSet {
            axes: temporal::AxisSet::from_indices(&[2]),
            v_max: 25.0,
            a_max: 100.0,
            j_max: 200.0,
        },
    ];
    let limits = temporal::Limits::try_new(&sets, 3).unwrap();
    let segments = [ShapeSegmentInput {
        temporal: temporal::multi::SegmentInput {
            curve: &curve,
            limits,
            followers: &[],
            virtual_path: None,
        },
        followers: &[],
        feedrate_mm_s: 15.0,
    }];
    let chains = AxisChainSet::passthrough_spatial();
    let input = ShapeBatchInput {
        follower_history: None,
        segments: &segments,
        grid_strategy: temporal::multi::GridStrategy::Adaptive {
            min_n: 20,
            max_n: 200,
            target_grid_spacing_mm: 0.5,
        },
        worker_threads: 1,
        chains: &chains,
        follower_start: &[],
        fit_tolerance_mm: 0.005,
        beta_max_iters: 10,
        beta_convergence_ratio: 1.02,
        initial_v: 0.0,
        initial_a: 0.0,
        terminal_v: 0.0,
        start_d2_override: None,
    };

    let output =
        plan_velocity_inner(&input, SafetyMode::WorstCaseFuture).expect("plan should succeed");
    assert!(
        output.stats.beta_converged,
        "jerk-limited z move must converge (got {} iterations)",
        output.stats.beta_iterations
    );
    assert!(
        output.stats.beta_iterations <= 4,
        "convergence must take a few derate steps, not the full budget (got {})",
        output.stats.beta_iterations
    );
}

#[test]
fn feed_cruise_reads_full_velocity_utilization() {
    let base = temporal::Limits::axis_boxes([1000.0; 3], [1.0e9; 3], [1.0e12; 3]);
    let curve = nurbs::VectorNurbs::try_new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![[0.0, 0.0, 0.0], [200.0, 0.0, 0.0]],
    )
    .unwrap();
    let feed = 494.0;
    let limits = crate::streaming::state::per_segment_limits(&curve, &base, feed);

    let segments = [ShapeSegmentInput {
        temporal: temporal::multi::SegmentInput {
            curve: &curve,
            limits,
            followers: &[],
            virtual_path: None,
        },
        followers: &[],
        feedrate_mm_s: feed,
    }];
    let chains = AxisChainSet::passthrough_spatial();
    let input = ShapeBatchInput {
        follower_history: None,
        segments: &segments,
        grid_strategy: temporal::multi::GridStrategy::Adaptive {
            min_n: 20,
            max_n: 400,
            target_grid_spacing_mm: 0.5,
        },
        worker_threads: 1,
        chains: &chains,
        follower_start: &[],
        fit_tolerance_mm: 0.005,
        beta_max_iters: 10,
        beta_convergence_ratio: 1.02,
        initial_v: feed,
        initial_a: 0.0,
        terminal_v: feed,
        start_d2_override: None,
    };

    let output = plan_velocity_inner(&input, SafetyMode::WorstCaseFuture).expect("plan");
    assert_eq!(
        output.binding.peak_util_family,
        Some(crate::utilization::UtilFamily::Velocity),
        "a feed cruise must be velocity-bound, not jerk/accel"
    );
    assert!(
        (output.binding.peak_utilization - 1.0).abs() < 0.05,
        "cruising at feed must read ~1.0 (feed credited), not feed/box; got {}",
        output.binding.peak_utilization,
    );
    assert_eq!(
        output.binding.worst.map(|w| w.kind),
        Some(temporal::LimitKind::Feedrate),
        "a feed-bound move (accel/jerk slack) must name the dynamic feedrate set as \
         its limiter; got {:?}",
        output.binding.worst,
    );
}
