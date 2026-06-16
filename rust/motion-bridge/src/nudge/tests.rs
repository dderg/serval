use super::*;

fn axis_total_displacement(segs: &[trajectory::ShapedSegment], axis: u8) -> f64 {
    let ax = axis as usize;
    segs.iter()
        .map(|s| {
            let curve = &s.axes[ax];
            nurbs::eval::eval(curve, s.t_end) - nurbs::eval::eval(curve, s.t_start)
        })
        .sum()
}

#[test]
fn box_profile_when_accel_zero_is_constant_velocity() {
    let segs = plan_nudge_profile(2, 1.0, 10.0, 0.0, 0b0000_0010, 0.0).unwrap();
    let total: f64 = segs.iter().map(|s| s.t_end - s.t_start).sum();
    assert!((total - 0.1).abs() < 1e-9);
    assert!(segs.iter().all(|s| s.motor_mask == 0b0000_0010));
    assert!((axis_total_displacement(&segs, 2) - 1.0).abs() < 1e-6);
}

#[test]
fn trapezoid_profile_total_displacement_matches_delta() {
    let segs = plan_nudge_profile(2, 10.0, 100.0, 1000.0, 0b0000_0010, 0.0).unwrap();
    assert!((axis_total_displacement(&segs, 2) - 10.0).abs() < 1e-6);
    assert!(segs.iter().map(|s| s.t_end - s.t_start).sum::<f64>() > 0.0);
}

#[test]
fn short_move_degenerates_to_triangle_no_cruise() {
    let segs = plan_nudge_profile(2, 0.2, 100.0, 1000.0, 0b0000_0010, 0.0).unwrap();
    assert!((axis_total_displacement(&segs, 2) - 0.2).abs() < 1e-6);
}

#[test]
fn negative_delta_moves_negative() {
    let segs = plan_nudge_profile(2, -1.0, 10.0, 0.0, 0b0000_0010, 0.0).unwrap();
    assert!((axis_total_displacement(&segs, 2) + 1.0).abs() < 1e-6);
}

#[test]
fn t_start_base_offsets_all_segment_times() {
    let segs = plan_nudge_profile(2, 1.0, 10.0, 1000.0, 0b0000_0010, 5.0).unwrap();
    assert!(
        segs[0].t_start >= 5.0 - 1e-12,
        "first segment must start at t_start_base"
    );
    for seg in &segs {
        assert!(seg.t_start >= 5.0 - 1e-12);
    }
}
