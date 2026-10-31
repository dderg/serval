use super::*;
use trajectory::NudgeProfile;

fn displacement(profile: &NudgeProfile) -> f64 {
    profile.position(profile.t_end()) - profile.position(profile.t_start())
}

#[test]
fn box_profile_when_accel_zero_is_constant_velocity() {
    let profile = plan_nudge_profile(2, 1.0, 10.0, 0.0, 5.0).unwrap();
    assert!((profile.duration() - 0.1).abs() < 1e-9);
    assert!(profile.position(profile.t_start()).abs() < 1e-12);
    assert!((profile.position(profile.t_end()) - 1.0).abs() < 1e-9);
    let mid = profile.eval(5.05);
    assert!((mid.position - 0.5).abs() < 1e-9);
    assert!((mid.velocity - 10.0).abs() < 1e-9);
    assert!(mid.acceleration.abs() < 1e-12);
    assert_eq!(profile.acceleration_bounds(), (0.0, 0.0));
}

#[test]
fn trapezoid_profile_total_displacement_matches_delta() {
    let profile = plan_nudge_profile(2, 10.0, 100.0, 1000.0, 7.5).unwrap();
    assert!((profile.t_start() - 7.5).abs() < 1e-12);
    assert!(profile.position(profile.t_start()).abs() < 1e-12);
    assert!((profile.position(profile.t_end()) - 10.0).abs() < 1e-9);
    assert!(profile.velocity(profile.t_start()).abs() < 1e-12);
    assert!(profile.velocity(profile.t_end()).abs() < 1e-12);
    assert!(profile.duration() > 0.0);
}

#[test]
fn short_move_degenerates_to_triangle_no_cruise() {
    let profile = plan_nudge_profile(2, 0.2, 100.0, 1000.0, 0.0).unwrap();
    assert!((displacement(&profile) - 0.2).abs() < 1e-6);
    assert!(
        profile.velocity(profile.t_start() + 0.5 * profile.duration()) < 100.0 - 1e-9,
        "a short move never reaches the requested speed"
    );
}

#[test]
fn negative_delta_moves_negative() {
    let profile = plan_nudge_profile(2, -1.0, 10.0, 0.0, 0.0).unwrap();
    assert!((displacement(&profile) + 1.0).abs() < 1e-6);
}

#[test]
fn t_start_base_offsets_the_profile_domain() {
    let profile = plan_nudge_profile(2, 1.0, 10.0, 1000.0, 5.0).unwrap();
    assert!((profile.t_start() - 5.0).abs() < 1e-12);
    assert!((profile.t_end() - (5.0 + profile.duration())).abs() < 1e-12);
    assert!(profile.breakpoints().iter().all(|&t| t >= 5.0 - 1e-12));
}

#[test]
fn no_breakpoint_interval_is_zero_width() {
    let profile = plan_nudge_profile(2, 0.2, 100.0, 1000.0, 0.0).unwrap();
    assert!(
        profile.breakpoints().windows(2).all(|w| w[1] > w[0]),
        "a zero-width phase would build a degenerate knot range downstream"
    );
}

#[test]
fn zero_delta_is_rejected_loudly() {
    let err = plan_nudge_profile(2, 0.0, 10.0, 1000.0, 0.0).unwrap_err();
    assert!(err.contains("displacement must be nonzero"), "got: {err}");
}

#[test]
fn extruder_lane_nudge_is_planned_like_a_spatial_one() {
    let extruder = plan_nudge_profile(3, 5.0, 5.0, 1000.0, 0.0).unwrap();
    let spatial = plan_nudge_profile(0, 5.0, 5.0, 1000.0, 0.0).unwrap();
    assert!((displacement(&extruder) - 5.0).abs() < 1e-6);
    assert!(extruder.duration() > 0.0);
    assert_eq!(extruder, spatial);
}

#[test]
fn lane_beyond_mcu_capacity_is_rejected_loudly() {
    let err = plan_nudge_profile(MAX_AXES as u8, 1.0, 10.0, 1000.0, 0.0).unwrap_err();
    assert!(err.contains("out of range"), "got: {err}");
}

#[test]
fn bad_speed_is_rejected_loudly() {
    let err = plan_nudge_profile(2, 1.0, 0.0, 1000.0, 0.0).unwrap_err();
    assert!(err.contains("bad speed"), "got: {err}");
}
