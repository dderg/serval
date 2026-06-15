use super::*;
use crate::{
    BindingConstraint, BindingSummary, GridSample, GridScheme, SolveStatus, TopProfile,
    WorstBinding,
};

#[test]
fn partition_splits_only_at_non_collinear() {
    let collinear = [true, false, true];
    let chains = partition_chains(4, &collinear);
    assert_eq!(chains, vec![0..=1, 2..=3]);
}

#[test]
fn partition_all_collinear_is_one_chain() {
    let collinear = [true, true];
    assert_eq!(partition_chains(3, &collinear), vec![0..=2]);
}

#[test]
fn partition_none_collinear_is_all_singletons() {
    let collinear = [false, false];
    assert_eq!(partition_chains(3, &collinear), vec![0..=0, 1..=1, 2..=2]);
}

#[test]
fn partition_single_segment_no_junctions() {
    assert_eq!(partition_chains(1, &[]), vec![0..=0]);
}

#[test]
fn slice_duplicates_junction_sample_and_splits_time() {
    // 2-segment chain profile: ranges (0,2) and (2,4), 5 samples, uniform v=10.
    let ranges = vec![(0usize, 2usize), (2, 4)];
    let samples: Vec<GridSample> = (0..5)
        .map(|i| GridSample {
            s: i as f64,
            v: 10.0,
            a: 0.0,
            b: 100.0,
            binding: BindingConstraint::None,
        })
        .collect();
    let chain_profile = TopProfile {
        samples,
        status: SolveStatus::Solved,
        grid_scheme: GridScheme::UniformArclength,
        total_time: 0.4,
        binding: BindingSummary::default(),
        deadline_truncated: false,
    };
    let per_segment = slice_chain_profile(&chain_profile, &ranges);
    assert_eq!(per_segment.len(), 2);
    assert_eq!(per_segment[0].samples.len(), 3);
    assert_eq!(per_segment[1].samples.len(), 3);
    assert_eq!(per_segment[0].samples[2].v, per_segment[1].samples[0].v);
    assert!((per_segment[1].samples[0].s - 0.0).abs() < 1e-12);
    assert!((per_segment[0].samples[2].s - 2.0).abs() < 1e-12);
    assert!((per_segment[0].total_time - 0.2).abs() < 1e-9);
    assert!((per_segment[1].total_time - 0.2).abs() < 1e-9);
}

#[test]
fn binding_summary_assigned_to_first_slice_only() {
    let ranges = vec![(0usize, 2usize), (2, 4), (4, 6)];
    let samples: Vec<GridSample> = (0..7)
        .map(|i| GridSample {
            s: i as f64,
            v: 10.0,
            a: 0.0,
            b: 100.0,
            binding: BindingConstraint::None,
        })
        .collect();
    let chain_binding = BindingSummary {
        histogram: vec![(BindingConstraint::Velocity { set: 0 }, 5)],
        worst: Some(WorstBinding {
            constraint: BindingConstraint::Velocity { set: 0 },
            ratio: 1.0,
            grid_index: 2,
            s: 2.0,
            kind: crate::LimitKind::Config,
        }),
    };
    let chain_profile = TopProfile {
        samples,
        status: SolveStatus::Solved,
        grid_scheme: GridScheme::UniformArclength,
        total_time: 1.2,
        binding: chain_binding,
        deadline_truncated: false,
    };
    let per_segment = slice_chain_profile(&chain_profile, &ranges);
    assert_eq!(per_segment.len(), 3);
    assert!(
        !per_segment[0].binding.histogram.is_empty(),
        "first slice must carry the chain binding summary"
    );
    assert!(
        per_segment[1].binding.histogram.is_empty(),
        "second slice must have empty binding summary"
    );
    assert!(
        per_segment[2].binding.histogram.is_empty(),
        "third slice must have empty binding summary"
    );
    let total_histogram_count: u32 = per_segment
        .iter()
        .flat_map(|p| p.binding.histogram.iter())
        .map(|(_, count)| count)
        .sum();
    assert_eq!(
        total_histogram_count, 5,
        "histogram count must be 5 (once per chain), not 15 (once per slice)"
    );
}
