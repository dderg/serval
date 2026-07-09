use motion_pipeline::StreamConfig;

use super::*;
use crate::TrajectoryPieces;

fn config(max_jerk: f64) -> StreamConfig {
    StreamConfig {
        corner: geometry::CornerFitConfig::default(),
        integration_tol: 1e-7,
        max_extrude_only_velocity_mm_s: f64::INFINITY,
        max_extrude_only_accel_mm_s2: f64::INFINITY,
        fit_tol_mm: 0.005,
        fit_tol_accel_mm_s2: 50.0,
        max_buffer_moves: 1024,
        limits: geometry::VelocityLimits::try_new(100.0, 1000.0, 8.0, max_jerk).unwrap(),
    }
}

fn audit_x(x_pieces: Vec<Vec<f64>>, config: &StreamConfig) -> AuditReport {
    let traj = TrajectoryPieces {
        x: x_pieces,
        y: Vec::new(),
        z: Vec::new(),
        e: Vec::new(),
        t_end: 0.0,
    };
    audit_trajectory(&traj, config, &AuditBudgets::for_config(config))
}

fn kinds(violations: &[Violation]) -> Vec<ViolationKind> {
    violations.iter().map(|v| v.kind).collect()
}

#[test]
fn clean_continuous_cubics_pass() {
    let cfg = config(1e6);
    let report = audit_x(
        vec![
            vec![0.0, 0.1, 0.0, 0.0, 0.0, 100.0],
            vec![0.1, 0.2, 0.1, 3.0, 30.0, -100.0],
        ],
        &cfg,
    );
    assert!(report.hard_ok(), "{report}");
    assert!(report.target_ok(), "{report}");
    assert!(report.extrema[0].max_velocity > 0.0);
}

#[test]
fn nan_coefficient_is_hard() {
    let cfg = config(1e6);
    let report = audit_x(vec![vec![0.0, 0.1, 0.0, f64::NAN]], &cfg);
    assert_eq!(kinds(&report.hard), [ViolationKind::NonFiniteCoefficient]);
}

#[test]
fn non_positive_span_is_hard() {
    let cfg = config(1e6);
    let report = audit_x(vec![vec![0.1, 0.1, 0.0]], &cfg);
    assert_eq!(kinds(&report.hard), [ViolationKind::NonPositiveSpan]);
}

#[test]
fn time_gap_while_moving_is_hard() {
    let cfg = config(1e6);
    let report = audit_x(
        vec![vec![0.0, 0.1, 0.0, 1.0], vec![0.2, 0.3, 0.1, 1.0]],
        &cfg,
    );
    assert_eq!(kinds(&report.hard), [ViolationKind::TimeGap]);
}

#[test]
fn time_gap_at_rest_is_a_hold() {
    let cfg = config(1e6);
    let report = audit_x(vec![vec![0.0, 0.1, 5.0], vec![0.2, 0.3, 5.0]], &cfg);
    assert!(report.hard_ok(), "{report}");
}

#[test]
fn overlapping_pieces_are_hard() {
    let cfg = config(1e6);
    let report = audit_x(vec![vec![0.0, 0.2, 5.0], vec![0.1, 0.3, 5.0]], &cfg);
    assert_eq!(kinds(&report.hard), [ViolationKind::TimeOverlap]);
}

#[test]
fn position_step_at_seam_is_hard() {
    let cfg = config(1e6);
    let report = audit_x(vec![vec![0.0, 0.1, 5.0], vec![0.1, 0.2, 5.01]], &cfg);
    assert_eq!(kinds(&report.hard), [ViolationKind::SeamPosition]);
}

#[test]
fn accel_step_at_seam_is_target_under_jerk_limiting() {
    let cfg = config(1e6);
    let report = audit_x(
        vec![vec![0.0, 0.1, 0.0, 1.0], vec![0.1, 0.2, 0.1, 1.0, 100.0]],
        &cfg,
    );
    assert!(report.hard_ok(), "{report}");
    assert!(kinds(&report.target).contains(&ViolationKind::SeamAccel));
}

#[test]
fn accel_step_at_seam_is_allowed_without_jerk_limiting() {
    let cfg = config(f64::INFINITY);
    let report = audit_x(
        vec![vec![0.0, 0.1, 0.0, 1.0], vec![0.1, 0.2, 0.1, 1.0, 100.0]],
        &cfg,
    );
    assert!(
        !kinds(&report.target).contains(&ViolationKind::SeamAccel),
        "{report}"
    );
}

#[test]
fn velocity_above_limit_is_target() {
    let cfg = config(1e6);
    let report = audit_x(vec![vec![0.0, 0.1, 0.0, 150.0]], &cfg);
    assert!(report.hard_ok(), "{report}");
    assert_eq!(kinds(&report.target), [ViolationKind::Velocity]);
}

#[test]
fn interior_accel_spike_is_target() {
    let cfg = config(1e6);
    let report = audit_x(vec![vec![0.0, 0.2, 0.0, 0.0, 2000.0, -10000.0]], &cfg);
    assert!(
        kinds(&report.target).contains(&ViolationKind::Accel),
        "{report}"
    );
}

#[test]
fn sliver_piece_jerk_spike_is_target() {
    let cfg = config(1e6);
    let dt = 2e-7;
    let jerk = 5e7;
    let report = audit_x(vec![vec![0.0, dt, 0.0, 0.0, 0.0, jerk / 6.0]], &cfg);
    assert!(
        kinds(&report.target).contains(&ViolationKind::Jerk),
        "{report}"
    );
    assert!(report.extrema[0].min_piece_duration_s <= dt);
}
