use super::*;
use crate::{ShapeBatchInput, ShapeSegmentInput, ShaperConfig};
use geometry::segment::FollowerDemand;

const E_FOLLOWER_04: &[FollowerDemand] = &[FollowerDemand {
    axis_index: 3,
    ratio: 0.04,
}];
use nurbs::VectorNurbs;

fn default_limits() -> temporal::Limits {
    temporal::Limits::axis_boxes(
        [500.0, 500.0, 500.0],
        [5_000.0, 5_000.0, 5_000.0],
        [100_000.0, 100_000.0, 100_000.0],
    )
}

fn default_shaper_config() -> ShaperConfig {
    ShaperConfig {
        x: crate::AxisShaper::SmoothZv {
            frequency_hz: 180.0,
        },
        y: crate::AxisShaper::SmoothZv {
            frequency_hz: 120.0,
        },
        z: crate::AxisShaper::Passthrough,
    }
}

fn straight_linear(start: [f64; 3], end: [f64; 3]) -> VectorNurbs<f64, 3> {
    VectorNurbs::try_new(1, vec![0.0, 0.0, 1.0, 1.0], vec![start, end]).unwrap()
}

#[test]
fn single_straight_line_converges() {
    let curve = straight_linear([0.0, 0.0, 0.0], [50.0, 0.0, 0.0]);
    let generous_limits = temporal::Limits::axis_boxes(
        [500.0, 500.0, 500.0],
        [5_000.0, 5_000.0, 5_000.0],
        [100_000.0, 100_000.0, 100_000.0],
    );
    let segments = [ShapeSegmentInput {
        temporal: temporal::multi::SegmentInput {
            curve: &curve,
            limits: generous_limits,
            followers: &[],
        },
        followers: E_FOLLOWER_04,
        feedrate_mm_s: 100.0,
    }];

    let input = ShapeBatchInput {
        segments: &segments,
        grid_strategy: temporal::multi::GridStrategy::Fixed(10),
        worker_threads: 1,
        shaper: default_shaper_config(),
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
