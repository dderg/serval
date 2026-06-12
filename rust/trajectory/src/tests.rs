use super::*;

#[test]
fn shape_batch_rejects_empty_segments() {
    let input = ShapeBatchInput {
        follower_pa: [0.0; temporal::MAX_AXES],
        follower_history: None,
        segments: &[],
        grid_strategy: temporal::multi::GridStrategy::Fixed(100),
        worker_threads: 1,
        shaper: ShaperConfig {
            x: AxisShaper::SmoothZv {
                frequency_hz: 180.0,
            },
            y: AxisShaper::SmoothMzv {
                frequency_hz: 120.0,
            },
            z: AxisShaper::Passthrough,
        },
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
    };
    assert_eq!(seg.axes.len(), 4);
}
