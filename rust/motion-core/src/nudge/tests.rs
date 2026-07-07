use super::*;

fn phase_displacement(p: &NudgePiece) -> f64 {
    p.piece.evaluate(p.piece.u_end) - p.piece.evaluate(p.piece.u_start)
}

fn total_displacement(segs: &[NudgePiece]) -> f64 {
    segs.iter().map(phase_displacement).sum()
}

fn total_duration(segs: &[NudgePiece]) -> f64 {
    segs.iter().map(|s| s.piece.u_end - s.piece.u_start).sum()
}

#[test]
fn box_profile_when_accel_zero_is_constant_velocity() {
    let segs = plan_nudge_profile(2, 1.0, 10.0, 0.0, 0b0000_0010, 0.0).unwrap();
    assert!((total_duration(&segs) - 0.1).abs() < 1e-9);
    assert!(segs.iter().all(|s| s.motor_mask == 0b0000_0010));
    assert!(segs.iter().all(|s| s.axis == 2));
    assert!((total_displacement(&segs) - 1.0).abs() < 1e-6);
}

#[test]
fn trapezoid_profile_total_displacement_matches_delta() {
    let segs = plan_nudge_profile(2, 10.0, 100.0, 1000.0, 0b0000_0010, 0.0).unwrap();
    assert!((total_displacement(&segs) - 10.0).abs() < 1e-6);
    assert!(total_duration(&segs) > 0.0);
}

#[test]
fn short_move_degenerates_to_triangle_no_cruise() {
    let segs = plan_nudge_profile(2, 0.2, 100.0, 1000.0, 0b0000_0010, 0.0).unwrap();
    assert!((total_displacement(&segs) - 0.2).abs() < 1e-6);
}

#[test]
fn negative_delta_moves_negative() {
    let segs = plan_nudge_profile(2, -1.0, 10.0, 0.0, 0b0000_0010, 0.0).unwrap();
    assert!((total_displacement(&segs) + 1.0).abs() < 1e-6);
}

#[test]
fn t_start_base_offsets_all_segment_times() {
    let segs = plan_nudge_profile(2, 1.0, 10.0, 1000.0, 0b0000_0010, 5.0).unwrap();
    assert!(
        segs[0].piece.u_start >= 5.0 - 1e-12,
        "first segment must start at t_start_base"
    );
    for seg in &segs {
        assert!(seg.piece.u_start >= 5.0 - 1e-12);
    }
}

#[test]
fn no_phase_is_zero_width() {
    let segs = plan_nudge_profile(2, 0.2, 100.0, 1000.0, 0b0000_0010, 0.0).unwrap();
    assert!(
        segs.iter().all(|s| s.piece.u_end > s.piece.u_start),
        "a zero-width phase would build a degenerate knot range downstream"
    );
}

#[test]
fn zero_delta_is_rejected_loudly() {
    let err = plan_nudge_profile(2, 0.0, 10.0, 1000.0, 0b0000_0010, 0.0).unwrap_err();
    assert!(err.contains("degenerate"), "got: {err}");
}
