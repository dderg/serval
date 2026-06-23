use super::*;

#[test]
fn shape_batch_rejects_empty_segments() {
    let chains = crate::AxisChainSet::spatial(
        crate::PostProcessorType::SmoothZv {
            frequency_hz: 180.0,
        }
        .into_chain(),
        crate::PostProcessorType::SmoothMzv {
            frequency_hz: 120.0,
        }
        .into_chain(),
        crate::CompiledChain::default(),
    );
    let input = ShapeBatchInput {
        chains: &chains,
        follower_start: &[],
        follower_history: None,
        segments: &[],
        grid_strategy: temporal::multi::GridStrategy::Fixed(100),
        worker_threads: 1,
        fit_tolerance_mm: 0.001,
        beta_max_iters: 5,
        beta_convergence_ratio: 1.02,
        initial_v: 0.0,
        initial_a: 0.0,
        terminal_v: 0.0,
        start_d2_override: None,
    };
    let result = shape_batch(&input);
    assert!(matches!(result, Err(ShapeError::EmptySegments)));
}

#[test]
fn shaped_segment_carries_registry_indexed_tracks() {
    let constant = |v: f64| {
        nurbs::ScalarNurbs::try_new(
            3,
            vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
            vec![v, v, v, v],
        )
        .unwrap()
    };
    let seg = ShapedSegment {
        axes: vec![constant(0.0), constant(1.0), constant(2.0), constant(3.0)],
        followers: vec![],
        t_start: 0.0,
        t_end: 1.0,
        motor_mask: 0,
        source_line: 0,
    };
    assert_eq!(seg.axes.len(), 4);
}
