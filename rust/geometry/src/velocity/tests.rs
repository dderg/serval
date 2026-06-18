use super::*;
use crate::fitter::{FitReport, UnblendedJunction};
use crate::frontend::{Move, VelocityLimits};
use crate::path::{Arc, Clothoid, Line, PathSegment, Segment};
use crate::segment::FollowerDemand;

fn limits(max_v: f64, accel: f64) -> VelocityLimits {
    VelocityLimits::try_new(max_v, accel, 5.0).unwrap()
}

fn src(line_no: u32) -> SourceRange {
    SourceRange {
        start_line: line_no,
        end_line: line_no,
    }
}

fn spatial_move(seg: Segment, feed: f64, max_v: f64, accel: f64, line_no: u32) -> Move {
    Move {
        segment: PathSegment::try_new(seg, Vec::new()).unwrap(),
        feedrate_mm_s: feed,
        limits: limits(max_v, accel),
        source: src(line_no),
    }
}

fn line_move(len: f64, feed: f64, max_v: f64, accel: f64, line_no: u32) -> Move {
    let seg = Segment::Line(Line::try_new([0.0, 0.0, 0.0], [len, 0.0, 0.0]).unwrap());
    spatial_move(seg, feed, max_v, accel, line_no)
}

fn arc_move(radius: f64, sweep: f64, feed: f64, max_v: f64, accel: f64, line_no: u32) -> Move {
    let arc = Arc::try_new(
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        radius,
        0.0,
        sweep,
    )
    .unwrap();
    spatial_move(Segment::Arc(arc), feed, max_v, accel, line_no)
}

fn clothoid_move(kappa_peak: f64, length: f64, feed: f64, accel: f64, line_no: u32) -> Move {
    let clo = Clothoid::try_new(
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        0.0,
        kappa_peak / length,
        length,
    )
    .unwrap();
    spatial_move(Segment::Clothoid(clo), feed, feed, accel, line_no)
}

fn virtual_move(virtual_path: f64, feed: f64, max_v: f64, accel: f64, line_no: u32) -> Move {
    let seg = PathSegment::try_new_virtual(
        vec![FollowerDemand {
            axis_index: 3,
            ratio: 0.05,
        }],
        virtual_path,
    )
    .unwrap();
    Move {
        segment: seg,
        feedrate_mm_s: feed,
        limits: limits(max_v, accel),
        source: src(line_no),
    }
}

fn outcome(moves: Vec<Move>, unblended: Vec<UnblendedJunction>) -> FitOutcome {
    FitOutcome {
        moves,
        report: FitReport {
            blended: 0,
            unblended,
            consumed_legs: 0,
            chains: 0,
        },
    }
}

fn move_time(m: &MoveVelocity) -> f64 {
    traversal_time(&m.samples)
}

fn assert_disk_feasible(m: &MoveVelocity, kappa0: f64, sigma: f64) {
    for s in &m.samples {
        let kappa = (kappa0 + sigma * s.s).abs();
        assert!(
            s.v * s.v * kappa <= m.accel + 1e-3,
            "centripetal a_c exceeds a_max at s={}",
            s.s
        );
    }
}

#[test]
fn straight_line_cruises_at_feed_limit() {
    let out = outcome(vec![line_move(100.0, 30.0, 200.0, 1000.0, 1)], Vec::new());
    let plan = plan_velocity(&out, VelocityConfig::default()).unwrap();
    let m = &plan.moves[0];
    assert!((m.peak_v - 30.0).abs() < 1e-6);
    assert_eq!(m.entry_v, 0.0);
    assert_eq!(m.exit_v, 0.0);
    assert_eq!(m.samples.first().unwrap().s, 0.0);
    assert_eq!(m.samples.last().unwrap().s, 100.0);
}

#[test]
fn arc_cruise_capped_at_sqrt_a_r() {
    let (radius, accel) = (20.0, 2000.0);
    let out = outcome(
        vec![arc_move(
            radius,
            std::f64::consts::PI,
            500.0,
            500.0,
            accel,
            1,
        )],
        Vec::new(),
    );
    let plan = plan_velocity(&out, VelocityConfig::default()).unwrap();
    let expected = (accel * radius).sqrt();
    assert!((plan.moves[0].peak_v - expected).abs() < 1e-3);
    assert_eq!(plan.report.curvature_bound, 1);
}

#[test]
fn clothoid_rides_above_the_constant_curvature_ceiling() {
    let (kappa_peak, length, accel) = (0.2_f64, 4.0_f64, 1000.0_f64);
    let constant_ceiling = (accel / kappa_peak).sqrt();
    let out = outcome(
        vec![
            line_move(60.0, 300.0, 300.0, accel, 1),
            clothoid_move(kappa_peak, length, 300.0, accel, 2),
            line_move(60.0, 300.0, 300.0, accel, 3),
        ],
        Vec::new(),
    );
    let plan = plan_velocity(&out, VelocityConfig::default()).unwrap();
    let clo = &plan.moves[1];
    assert!(clo.peak_v > constant_ceiling + 1.0);
    assert!(plan.report.limit_ride >= 1);
    assert_disk_feasible(clo, 0.0, kappa_peak / length);
    assert!(move_time(clo) < length / constant_ceiling);
}

#[test]
fn seam_velocity_is_continuous_across_the_chain() {
    let accel = 1000.0;
    let out = outcome(
        vec![
            line_move(40.0, 300.0, 300.0, accel, 1),
            clothoid_move(0.2, 4.0, 300.0, accel, 2),
            clothoid_move(0.2, 4.0, 300.0, accel, 2),
            line_move(40.0, 300.0, 300.0, accel, 3),
        ],
        Vec::new(),
    );
    let plan = plan_velocity(&out, VelocityConfig::default()).unwrap();
    for pair in plan.moves.windows(2) {
        assert!((pair[0].exit_v - pair[1].entry_v).abs() < 1e-9);
    }
    for m in &plan.moves {
        assert_eq!(m.samples.first().unwrap().v, m.entry_v);
        assert_eq!(m.samples.last().unwrap().v, m.exit_v);
    }
    assert!(plan.report.limit_ride >= 1);
}

#[test]
fn sharp_corner_pins_zero() {
    let out = outcome(
        vec![
            line_move(50.0, 100.0, 200.0, 1000.0, 1),
            line_move(50.0, 100.0, 200.0, 1000.0, 2),
        ],
        vec![UnblendedJunction {
            line_no: 2,
            reason: UnblendReason::NearReversal,
        }],
    );
    let plan = plan_velocity(&out, VelocityConfig::default()).unwrap();
    assert_eq!(plan.moves[0].exit_v, 0.0);
    assert_eq!(plan.moves[1].entry_v, 0.0);
    assert_eq!(plan.report.stops, 1);
}

#[test]
fn stop_does_not_leak_into_adjacent_blend_entry() {
    let out = outcome(
        vec![
            line_move(50.0, 200.0, 300.0, 1000.0, 1),
            line_move(50.0, 200.0, 300.0, 1000.0, 2),
            clothoid_move(0.2, 4.0, 200.0, 1000.0, 2),
            clothoid_move(0.2, 4.0, 200.0, 1000.0, 3),
            line_move(50.0, 200.0, 300.0, 1000.0, 3),
        ],
        vec![UnblendedJunction {
            line_no: 2,
            reason: UnblendReason::NearReversal,
        }],
    );
    let plan = plan_velocity(&out, VelocityConfig::default()).unwrap();
    assert_eq!(plan.moves[0].exit_v, 0.0);
    assert_eq!(plan.moves[1].entry_v, 0.0);
    assert!(plan.moves[1].exit_v > 0.0);
    assert!(plan.moves[2].entry_v > 0.0);
    assert_eq!(plan.report.stops, 1);
}

#[test]
fn collinear_junction_is_not_a_stop() {
    let out = outcome(
        vec![
            line_move(100.0, 80.0, 200.0, 1000.0, 1),
            line_move(100.0, 80.0, 200.0, 1000.0, 2),
        ],
        vec![UnblendedJunction {
            line_no: 2,
            reason: UnblendReason::Collinear,
        }],
    );
    let plan = plan_velocity(&out, VelocityConfig::default()).unwrap();
    assert!(plan.moves[0].exit_v > 0.0);
    assert_eq!(plan.report.stops, 0);
}

#[test]
fn short_move_peak_trimmed_by_jerk_below_accel_apex() {
    let (len, accel) = (0.5, 1000.0);
    let out = outcome(vec![line_move(len, 300.0, 300.0, accel, 1)], Vec::new());
    let plan = plan_velocity(&out, VelocityConfig::default()).unwrap();
    let m = &plan.moves[0];
    let accel_apex = (accel * len).sqrt();
    assert!(m.peak_v < accel_apex);
    assert_eq!(plan.report.jerk_bound, 1);
}

#[test]
fn infinite_jerk_recovers_constant_accel_apex() {
    let (len, accel) = (0.5, 1000.0);
    let out = outcome(vec![line_move(len, 300.0, 300.0, accel, 1)], Vec::new());
    let cfg = VelocityConfig {
        max_jerk_mm_s3: f64::INFINITY,
        ..VelocityConfig::default()
    };
    let plan = plan_velocity(&out, cfg).unwrap();
    let m = &plan.moves[0];
    let accel_apex = (accel * len).sqrt();
    assert!((m.peak_v - accel_apex).abs() < 1e-6);
    assert_eq!(plan.report.jerk_bound, 0);
}

#[test]
fn invalid_config_is_rejected() {
    let out = outcome(vec![line_move(10.0, 50.0, 100.0, 1000.0, 1)], Vec::new());
    for bad in [0.0, -1.0, f64::NAN] {
        let cfg = VelocityConfig {
            max_jerk_mm_s3: bad,
            ..VelocityConfig::default()
        };
        assert_eq!(plan_velocity(&out, cfg), Err(VelocityError::InvalidConfig));
    }
    for bad in [0.0, -1.0, f64::NAN, f64::INFINITY, 1e-12] {
        let cfg = VelocityConfig {
            integration_tol: bad,
            ..VelocityConfig::default()
        };
        assert_eq!(plan_velocity(&out, cfg), Err(VelocityError::InvalidConfig));
    }
    let ok = VelocityConfig {
        max_jerk_mm_s3: f64::INFINITY,
        ..VelocityConfig::default()
    };
    assert!(plan_velocity(&out, ok).is_ok());
}

#[test]
fn empty_and_single_move() {
    let empty = plan_velocity(&outcome(Vec::new(), Vec::new()), VelocityConfig::default()).unwrap();
    assert!(empty.moves.is_empty());

    let single = plan_velocity(
        &outcome(vec![line_move(10.0, 50.0, 100.0, 1000.0, 1)], Vec::new()),
        VelocityConfig::default(),
    )
    .unwrap();
    assert_eq!(single.moves.len(), 1);
    assert_eq!(single.moves[0].entry_v, 0.0);
    assert_eq!(single.moves[0].exit_v, 0.0);
}

#[test]
fn non_spatial_move_bracketed_by_stops() {
    let out = outcome(
        vec![
            line_move(40.0, 100.0, 200.0, 1000.0, 1),
            virtual_move(5.0, 40.0, 100.0, 1000.0, 2),
            line_move(40.0, 100.0, 200.0, 1000.0, 3),
        ],
        vec![
            UnblendedJunction {
                line_no: 2,
                reason: UnblendReason::NonSpatial,
            },
            UnblendedJunction {
                line_no: 3,
                reason: UnblendReason::NonSpatial,
            },
        ],
    );
    let plan = plan_velocity(&out, VelocityConfig::default()).unwrap();
    let retract = &plan.moves[1];
    assert_eq!(retract.entry_v, 0.0);
    assert_eq!(retract.exit_v, 0.0);
    assert!(retract.peak_v > 0.0);
    assert_eq!(plan.report.stops, 2);
}

#[test]
fn forward_backward_feasibility_holds_chainwide() {
    let accel = 1500.0;
    let sigma = 0.1;
    let clo = Clothoid::try_new(
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        0.0,
        sigma,
        6.0,
    )
    .unwrap();
    let out = outcome(
        vec![
            line_move(60.0, 250.0, 300.0, accel, 1),
            arc_move(15.0, std::f64::consts::FRAC_PI_2, 250.0, 300.0, accel, 2),
            spatial_move(Segment::Clothoid(clo), 250.0, 300.0, accel, 3),
            line_move(20.0, 250.0, 300.0, accel, 4),
        ],
        vec![UnblendedJunction {
            line_no: 4,
            reason: UnblendReason::NoBudget,
        }],
    );
    let plan = plan_velocity(&out, VelocityConfig::default()).unwrap();

    assert_eq!(plan.moves.first().unwrap().entry_v, 0.0);
    assert_eq!(plan.moves.last().unwrap().exit_v, 0.0);
    for m in &plan.moves {
        assert!(m.peak_v <= 300.0 + 1e-6);
        let accel_budget = 2.0 * m.accel * m.length + 1e-6;
        assert!((m.exit_v * m.exit_v - m.entry_v * m.entry_v).abs() <= accel_budget);
    }
    assert_disk_feasible(&plan.moves[2], 0.0, sigma);
    assert_eq!(plan.moves[2].exit_v, 0.0);
    assert_eq!(plan.moves[3].entry_v, 0.0);
}

#[test]
fn infinite_jerk_is_feasible_and_jerk_unbound() {
    let accel = 1500.0;
    let sigma = 0.1;
    let clo = Clothoid::try_new(
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        0.0,
        sigma,
        6.0,
    )
    .unwrap();
    let out = outcome(
        vec![
            line_move(60.0, 250.0, 300.0, accel, 1),
            spatial_move(Segment::Clothoid(clo), 250.0, 300.0, accel, 2),
            line_move(60.0, 250.0, 300.0, accel, 3),
        ],
        Vec::new(),
    );
    let cfg = VelocityConfig {
        max_jerk_mm_s3: f64::INFINITY,
        ..VelocityConfig::default()
    };
    let plan = plan_velocity(&out, cfg).unwrap();
    assert_eq!(plan.report.jerk_bound, 0);
    assert!(plan.report.limit_ride >= 1);
    assert_disk_feasible(&plan.moves[1], 0.0, sigma);
}

#[test]
fn plan_is_deterministic() {
    let build = || {
        outcome(
            vec![
                line_move(60.0, 250.0, 300.0, 1500.0, 1),
                arc_move(15.0, 1.0, 250.0, 300.0, 1500.0, 2),
                clothoid_move(0.2, 4.0, 250.0, 1500.0, 3),
            ],
            Vec::new(),
        )
    };
    let a = plan_velocity(&build(), VelocityConfig::default()).unwrap();
    let b = plan_velocity(&build(), VelocityConfig::default()).unwrap();
    assert_eq!(a, b);
}

#[test]
fn limit_riding_beats_the_constant_ceiling_skeleton() {
    let (kappa_peak, clo_len, accel) = (0.2_f64, 4.0_f64, 1000.0_f64);
    let constant_ceiling = (accel / kappa_peak).sqrt();
    let out = outcome(
        vec![
            line_move(60.0, 300.0, 300.0, accel, 1),
            clothoid_move(kappa_peak, clo_len, 300.0, accel, 2),
            line_move(60.0, 300.0, 300.0, accel, 3),
        ],
        Vec::new(),
    );
    let plan = plan_velocity(&out, VelocityConfig::default()).unwrap();
    let skeleton =
        move_time(&plan.moves[0]) + clo_len / constant_ceiling + move_time(&plan.moves[2]);
    assert!(plan.report.traversal_time_s > 0.0);
    assert!(plan.report.traversal_time_s < skeleton);
}

struct Mock {
    s_len: f64,
    kappa_peak: (f64, f64),
    kappa_endpoints: (f64, f64),
    sigma: f64,
}

impl CurvatureProfile for Mock {
    fn s_len(&self) -> f64 {
        self.s_len
    }
    fn kappa(&self, _s: f64) -> f64 {
        0.0
    }
    fn dkappa_ds(&self, _s: f64) -> f64 {
        self.sigma
    }
    fn kappa_peak(&self) -> (f64, f64) {
        self.kappa_peak
    }
    fn kappa_endpoints(&self) -> (f64, f64) {
        self.kappa_endpoints
    }
}

#[test]
fn falsified_sigma_is_inconsistent() {
    let bad = Mock {
        s_len: 4.0,
        kappa_peak: (4.0, 0.4),
        kappa_endpoints: (0.0, 0.4),
        sigma: 0.5,
    };
    assert_eq!(
        validate_segment(&bad, bad.s_len, 7, 1e-6),
        Err(VelocityError::Inconsistent { line_no: 7 })
    );
}

#[test]
fn interior_peak_is_non_alphabet() {
    let bad = Mock {
        s_len: 4.0,
        kappa_peak: (2.0, 0.4),
        kappa_endpoints: (0.0, 0.0),
        sigma: 0.0,
    };
    assert_eq!(
        validate_segment(&bad, bad.s_len, 9, 1e-6),
        Err(VelocityError::NonAlphabet { line_no: 9 })
    );
}

#[test]
fn non_finite_curvature_is_rejected() {
    let bad = Mock {
        s_len: 4.0,
        kappa_peak: (0.0, f64::INFINITY),
        kappa_endpoints: (0.0, 0.0),
        sigma: 0.0,
    };
    assert_eq!(
        validate_segment(&bad, bad.s_len, 11, 1e-6),
        Err(VelocityError::NonFinite { line_no: 11 })
    );

    let bad_len = Mock {
        s_len: 0.0,
        kappa_peak: (0.0, 0.0),
        kappa_endpoints: (0.0, 0.0),
        sigma: 0.0,
    };
    assert_eq!(
        validate_segment(&bad_len, bad_len.s_len, 12, 1e-6),
        Err(VelocityError::NonFinite { line_no: 12 })
    );
}
