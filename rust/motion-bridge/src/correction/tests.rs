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

#[test]
fn single_segment_profile_has_zero_velocity_ends() {
    let pieces = plan_correction_profile(3.0, 5.0, 100.0).unwrap();
    let first = pieces.first().unwrap();
    let last = pieces.last().unwrap();
    assert!(
        (first.coeffs[1] - first.coeffs[0]).abs() < 1e-9,
        "starts at rest"
    );
    assert!(
        (last.coeffs[3] - last.coeffs[2]).abs() < 1e-9,
        "ends at rest"
    );
}

#[test]
fn sequence_single_segment_equals_profile() {
    let a = plan_correction_profile(3.0, 5.0, 100.0).unwrap();
    let b = plan_correction_sequence(&[3.0], 5.0, 100.0).unwrap();
    assert_eq!(a.len(), b.len());
    for (x, y) in a.iter().zip(b.iter()) {
        assert_eq!(x.coeffs, y.coeffs);
        assert_eq!(x.duration, y.duration);
    }
}

#[test]
fn sequence_is_globally_contiguous() {
    let pieces = plan_correction_sequence(&[1.0, -1.0, 0.6, -0.6, 0.3], 50.0, 5000.0).unwrap();
    for w in pieces.windows(2) {
        assert!(
            (w[0].coeffs[3] - w[1].coeffs[0]).abs() < 1e-9,
            "no gap between segments"
        );
        let end_v = 3.0 * (w[0].coeffs[3] - w[0].coeffs[2]) / w[0].duration;
        let start_v = 3.0 * (w[1].coeffs[1] - w[1].coeffs[0]) / w[1].duration;
        assert!(
            (end_v - start_v).abs() < 1e-6,
            "velocity discontinuity between pieces"
        );
    }
    let sum: f64 = [1.0, -1.0, 0.6, -0.6, 0.3].iter().sum();
    assert!((pieces.last().unwrap().coeffs[3] - sum).abs() < 1e-6);
}

#[test]
fn sequence_drops_subepsilon_and_rejects_all_empty() {
    let pieces = plan_correction_sequence(&[2.0, 1e-9, -2.0], 50.0, 5000.0).unwrap();
    assert!((pieces.last().unwrap().coeffs[3]).abs() < 1e-6);
    assert!(plan_correction_sequence(&[1e-9, -1e-9], 50.0, 5000.0).is_err());
    assert!(plan_correction_sequence(&[1.0], 0.0, 5000.0).is_err());
    assert!(plan_correction_sequence(&[], 50.0, 5000.0).is_err());
    assert!(plan_correction_sequence(&[f64::NAN], 50.0, 5000.0).is_err());
    assert!(plan_correction_sequence(&[1.0, f64::INFINITY], 50.0, 5000.0).is_err());
}

#[test]
fn overlay_piece_entries_stamp_mask_and_monotonic_start_times() {
    let pieces = plan_correction_profile(2.0, 5.0, 100.0).unwrap();
    let mask = 1u8 << 3;
    let entries = to_overlay_piece_entries(&pieces, |secs| (secs * 1e6) as u64, 7.0, mask);
    assert_eq!(entries.len(), pieces.len());
    let mut t = 7.0_f64;
    let mut prev_start = 0u64;
    for ((entry, host_secs), piece) in entries.iter().zip(&pieces) {
        assert_eq!(entry.motor_mask, mask);
        assert_eq!(entry.start_time, (t * 1e6) as u64);
        assert!((host_secs - t).abs() < 1e-12);
        assert!((f64::from(entry.duration) - piece.duration).abs() < 1e-6);
        assert!(
            entry.start_time >= prev_start,
            "start_times must be monotonic"
        );
        prev_start = entry.start_time;
        t += piece.duration;
    }
}
