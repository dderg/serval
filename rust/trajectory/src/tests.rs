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
