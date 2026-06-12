use super::*;

#[test]
fn trapezoid_reaches_delta_exactly() {
    let pieces = plan_correction_profile(10.0, 5.0, 100.0).unwrap();
    let last = pieces.last().unwrap();
    assert!((last.coeffs[3] - 10.0).abs() < 1e-9);
    assert!(
        (pieces.iter().map(|p| p.duration).sum::<f64>()
            - profile_duration(10.0, 5.0, 100.0).unwrap())
        .abs()
            < 1e-9
    );
}

#[test]
fn profile_is_continuous_in_position_and_velocity() {
    let pieces = plan_correction_profile(-3.7, 8.0, 500.0).unwrap();
    for w in pieces.windows(2) {
        let end_p = w[0].coeffs[3];
        let start_p = w[1].coeffs[0];
        assert!((end_p - start_p).abs() < 1e-9);
        let end_v = 3.0 * (w[0].coeffs[3] - w[0].coeffs[2]) / w[0].duration;
        let start_v = 3.0 * (w[1].coeffs[1] - w[1].coeffs[0]) / w[1].duration;
        assert!((end_v - start_v).abs() < 1e-6);
    }
}

#[test]
fn short_move_uses_triangular_profile() {
    let pieces = plan_correction_profile(0.01, 50.0, 100.0).unwrap();
    let peak_v = pieces
        .iter()
        .map(|p| (3.0 * (p.coeffs[1] - p.coeffs[0]) / p.duration).abs())
        .fold(0.0_f64, f64::max);
    assert!(peak_v < 50.0);
    assert!((pieces.last().unwrap().coeffs[3] - 0.01).abs() < 1e-9);
}

#[test]
fn rejects_nonpositive_speed_accel_and_zero_delta() {
    assert!(plan_correction_profile(1.0, 0.0, 100.0).is_err());
    assert!(plan_correction_profile(1.0, 5.0, -1.0).is_err());
    assert!(plan_correction_profile(0.0, 5.0, 100.0).is_err());
}

#[test]
fn no_piece_exceeds_max_piece_duration() {
    let pieces = plan_correction_profile(50.0, 2.0, 100.0).unwrap();
    for p in &pieces {
        assert!(p.duration <= MAX_CORRECTION_PIECE_SECS + 1e-9);
    }
}
