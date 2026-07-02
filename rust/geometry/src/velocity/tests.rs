use super::*;
use crate::fitter::{FitReport, UnblendedJunction};
use crate::frontend::{Move, VelocityLimits};
use crate::path::{Arc, Clothoid, Line, PathSegment, Segment};
use crate::segment::FollowerDemand;

const DEFAULT_JERK_MM_S3: f64 = 100_000.0;
const DEFAULT_INTEGRATION_TOL: f64 = 1e-7;

fn limits(max_v: f64, accel: f64) -> VelocityLimits {
    VelocityLimits::try_new(max_v, accel, 5.0, DEFAULT_JERK_MM_S3).unwrap()
}

fn with_jerk(mut moves: Vec<Move>, jerk: f64) -> Vec<Move> {
    for m in &mut moves {
        m.limits.max_jerk_mm_s3 = jerk;
    }
    moves
}

fn plan(out: &FitOutcome) -> Result<VelocityProfile, VelocityError> {
    plan_velocity_warm_start(
        out,
        DEFAULT_INTEGRATION_TOL,
        f64::INFINITY,
        f64::INFINITY,
        BoundaryState::REST,
    )
}

fn plan_warm(out: &FitOutcome, entry_v: f64) -> Result<VelocityProfile, VelocityError> {
    plan_velocity_warm_start(
        out,
        DEFAULT_INTEGRATION_TOL,
        f64::INFINITY,
        f64::INFINITY,
        BoundaryState { v: entry_v, a: 0.0 },
    )
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
    let plan = plan(&out).unwrap();
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
    let plan = plan(&out).unwrap();
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
    let plan = plan(&out).unwrap();
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
    let plan = plan(&out).unwrap();
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
fn clothoid_emitted_accel_is_disk_feasible_and_tracks_velocity() {
    // The reconstructed tangential accel on a curved member must (a) fit inside the
    // acceleration disk together with the centripetal share, and (b) be the real
    // `dv/dt` of the speed profile — not a value bounded loose from it. Verified on
    // the curved member of a chain, where the seam velocities are feasible.
    let (kappa_peak, length, accel) = (0.2_f64, 4.0_f64, 1000.0_f64);
    let out = outcome(
        vec![
            line_move(60.0, 300.0, 300.0, accel, 1),
            clothoid_move(kappa_peak, length, 300.0, accel, 2),
            line_move(60.0, 300.0, 300.0, accel, 3),
        ],
        Vec::new(),
    );
    let plan = plan(&out).unwrap();
    let sigma = kappa_peak / length;
    let s = &plan.moves[1].samples;
    let mut t = vec![0.0_f64];
    for w in s.windows(2) {
        t.push(t[t.len() - 1] + 2.0 * (w[1].s - w[0].s) / (w[0].v + w[1].v).max(1e-9));
    }
    for (i, p) in s.iter().enumerate() {
        let a_c = p.v * p.v * (sigma * p.s);
        let total = (p.a * p.a + a_c * a_c).sqrt();
        assert!(
            total <= accel + 1.0,
            "total accel {total} exceeds a_max at s={}",
            p.s
        );
        if i > 0 && i < s.len() - 1 {
            let fd = (s[i + 1].v - s[i - 1].v) / (t[i + 1] - t[i - 1]).max(1e-9);
            assert!(
                (p.a - fd).abs() <= 0.02 * accel + 1.0,
                "emitted a={} disagrees with dv/dt={fd} at s={}",
                p.a,
                p.s
            );
        }
    }
}

#[test]
fn clothoid_rides_the_disk_above_the_old_cruise_ceiling() {
    // The vector-jerk *cruise* ceiling `(jerk / sigma)^(1/3)` is gone: it only bound
    // at `a_t = 0`, so it flattened the clothoid instead of letting it ride the
    // acceleration disk. The clothoid now enters well above that old value — pacing
    // to the disk, not the cruise ceiling — while every sample stays disk-feasible.
    let (kappa_peak, length, accel) = (0.8_f64, 4.0_f64, 2000.0_f64);
    let out = outcome(
        vec![
            line_move(60.0, 300.0, 300.0, accel, 1),
            clothoid_move(kappa_peak, length, 300.0, accel, 2),
            line_move(60.0, 300.0, 300.0, accel, 3),
        ],
        Vec::new(),
    );
    let plan = plan(&out).unwrap();
    let line = &plan.moves[0];
    let clothoid = &plan.moves[1];
    let sigma = kappa_peak / length;
    let old_cruise = (DEFAULT_JERK_MM_S3 / sigma).cbrt();
    assert!(
        line.exit_v < line.peak_v - 1.0,
        "the approach line must still decelerate into the clothoid"
    );
    assert!(
        clothoid.peak_v > old_cruise + 1.0,
        "clothoid peak {} must ride above the removed cruise ceiling {old_cruise}",
        clothoid.peak_v
    );
    assert_disk_feasible(clothoid, 0.0, sigma);
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
    let plan = plan(&out).unwrap();
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
    let plan = plan(&out).unwrap();
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
    let plan = plan(&out).unwrap();
    assert!(plan.moves[0].exit_v > 0.0);
    assert_eq!(plan.report.stops, 0);
}

#[test]
fn short_move_peak_trimmed_by_jerk_below_accel_apex() {
    let (len, accel) = (0.5, 1000.0);
    let out = outcome(vec![line_move(len, 300.0, 300.0, accel, 1)], Vec::new());
    let plan = plan(&out).unwrap();
    let m = &plan.moves[0];
    let accel_apex = (accel * len).sqrt();
    assert!(m.peak_v < accel_apex);
    assert_eq!(plan.report.jerk_bound, 1);
}

#[test]
fn infinite_jerk_recovers_constant_accel_apex() {
    let (len, accel) = (0.5, 1000.0);
    let out = outcome(
        with_jerk(vec![line_move(len, 300.0, 300.0, accel, 1)], f64::INFINITY),
        Vec::new(),
    );
    let plan = plan(&out).unwrap();
    let m = &plan.moves[0];
    let accel_apex = (accel * len).sqrt();
    assert!((m.peak_v - accel_apex).abs() < 1e-6);
    assert_eq!(plan.report.jerk_bound, 0);
}

#[test]
fn invalid_integration_tol_is_rejected() {
    let out = outcome(vec![line_move(10.0, 50.0, 100.0, 1000.0, 1)], Vec::new());
    for bad in [0.0, -1.0, f64::NAN, f64::INFINITY, 1e-12] {
        assert_eq!(
            plan_velocity_warm_start(&out, bad, f64::INFINITY, f64::INFINITY, BoundaryState::REST),
            Err(VelocityError::InvalidConfig)
        );
    }
    assert!(plan(&out).is_ok());
}

#[test]
fn invalid_extrude_only_limits_are_rejected() {
    let out = outcome(vec![line_move(10.0, 50.0, 100.0, 1000.0, 1)], Vec::new());
    for bad in [0.0, -1.0, f64::NAN] {
        assert_eq!(
            plan_velocity_warm_start(
                &out,
                DEFAULT_INTEGRATION_TOL,
                bad,
                f64::INFINITY,
                BoundaryState::REST
            ),
            Err(VelocityError::InvalidConfig)
        );
        assert_eq!(
            plan_velocity_warm_start(
                &out,
                DEFAULT_INTEGRATION_TOL,
                f64::INFINITY,
                bad,
                BoundaryState::REST
            ),
            Err(VelocityError::InvalidConfig)
        );
    }
}

#[test]
fn extrude_only_velocity_caps_pure_e_move() {
    let out = outcome(
        vec![virtual_move(10.0, 100.0, 200.0, 1000.0, 1)],
        Vec::new(),
    );
    let plan = plan_velocity_warm_start(
        &out,
        DEFAULT_INTEGRATION_TOL,
        5.0,
        f64::INFINITY,
        BoundaryState::REST,
    )
    .unwrap();
    let peak = plan.moves[0].peak_v;
    assert!(
        peak <= 5.0 + 1e-9,
        "pure-E peak {peak} exceeds extrude-only cap"
    );
    assert!(
        peak > 4.9,
        "pure-E move should ride the 5.0 cap, got {peak}"
    );
}

#[test]
fn extrude_only_accel_caps_pure_e_move() {
    let out = outcome(
        with_jerk(
            vec![virtual_move(0.5, 100.0, 100.0, 1000.0, 1)],
            f64::INFINITY,
        ),
        Vec::new(),
    );
    let plan = plan_velocity_warm_start(
        &out,
        DEFAULT_INTEGRATION_TOL,
        f64::INFINITY,
        10.0,
        BoundaryState::REST,
    )
    .unwrap();
    let apex = (10.0_f64 * 0.5).sqrt();
    assert!((plan.moves[0].peak_v - apex).abs() < 1e-6);
}

#[test]
fn extrude_only_limits_do_not_affect_spatial_move() {
    let out = outcome(vec![line_move(10.0, 50.0, 100.0, 1000.0, 1)], Vec::new());
    let base = plan(&out).unwrap().moves[0].peak_v;
    let capped =
        plan_velocity_warm_start(&out, DEFAULT_INTEGRATION_TOL, 1.0, 1.0, BoundaryState::REST)
            .unwrap()
            .moves[0]
            .peak_v;
    assert!(
        (base - capped).abs() < 1e-9,
        "extrude-only limits leaked into a spatial move: {base} vs {capped}"
    );
}

#[test]
fn empty_and_single_move() {
    let empty = plan(&outcome(Vec::new(), Vec::new())).unwrap();
    assert!(empty.moves.is_empty());

    let single = plan(&outcome(
        vec![line_move(10.0, 50.0, 100.0, 1000.0, 1)],
        Vec::new(),
    ))
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
    let plan = plan(&out).unwrap();
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
    let plan = plan(&out).unwrap();

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
        with_jerk(
            vec![
                line_move(60.0, 250.0, 300.0, accel, 1),
                spatial_move(Segment::Clothoid(clo), 250.0, 300.0, accel, 2),
                line_move(60.0, 250.0, 300.0, accel, 3),
            ],
            f64::INFINITY,
        ),
        Vec::new(),
    );
    let plan = plan(&out).unwrap();
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
    let a = plan(&build()).unwrap();
    let b = plan(&build()).unwrap();
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
    let plan = plan(&out).unwrap();
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

#[test]
fn warm_start_zero_matches_rest_to_rest() {
    let build = || {
        outcome(
            vec![
                line_move(60.0, 250.0, 300.0, 1500.0, 1),
                arc_move(15.0, 1.0, 250.0, 300.0, 1500.0, 2),
            ],
            Vec::new(),
        )
    };
    let rest = plan(&build()).unwrap();
    let warm0 = plan_warm(&build(), 0.0).unwrap();
    assert_eq!(rest, warm0);
}

#[test]
fn warm_start_enters_at_the_given_velocity() {
    let out = outcome(vec![line_move(100.0, 30.0, 200.0, 1000.0, 1)], Vec::new());
    let entry_v = 20.0;
    let plan = plan_warm(&out, entry_v).unwrap();
    let m = &plan.moves[0];
    assert_eq!(m.entry_v, entry_v);
    assert_eq!(m.samples.first().unwrap().v, entry_v);
    assert_eq!(m.exit_v, 0.0);
    assert_disk_feasible(m, 0.0, 0.0);
}

#[test]
fn warm_start_is_faster_than_starting_from_rest() {
    let out = || outcome(vec![line_move(100.0, 30.0, 200.0, 1000.0, 1)], Vec::new());
    let rest = plan(&out()).unwrap();
    let warm = plan_warm(&out(), 20.0).unwrap();
    assert!(warm.report.traversal_time_s < rest.report.traversal_time_s);
}

#[test]
fn warm_start_over_commit_cannot_brake_in_window() {
    let (length, accel, feed_ceiling, entry_v) = (1.0_f64, 1000.0_f64, 200.0_f64, 100.0_f64);
    let max_brakeable_entry_v = (2.0 * accel * length).sqrt();
    assert!(
        entry_v <= feed_ceiling,
        "entry must clear the feed ceiling so this exercises the can't-stop guard, not the ceiling guard"
    );
    assert!(
        entry_v > max_brakeable_entry_v,
        "entry must exceed the braking-distance budget so the can't-stop guard trips"
    );
    let out = outcome(
        vec![line_move(length, feed_ceiling, feed_ceiling, accel, 5)],
        Vec::new(),
    );
    assert_eq!(
        plan_warm(&out, entry_v),
        Err(VelocityError::OverCommitted { line_no: 5 })
    );
}

#[test]
fn warm_start_entry_above_move_ceiling_fails_loudly() {
    let out = outcome(vec![line_move(100.0, 30.0, 200.0, 1000.0, 7)], Vec::new());
    assert_eq!(
        plan_warm(&out, 50.0),
        Err(VelocityError::OverCommitted { line_no: 7 })
    );
}

#[test]
fn warm_start_negative_or_nan_entry_is_invalid_config() {
    let out = outcome(vec![line_move(100.0, 30.0, 200.0, 1000.0, 1)], Vec::new());
    assert_eq!(plan_warm(&out, -1.0), Err(VelocityError::InvalidConfig));
    assert_eq!(plan_warm(&out, f64::NAN), Err(VelocityError::InvalidConfig));
}

#[test]
fn barrier_is_the_last_cruise_seam_not_the_terminal_rest() {
    // Four collinear cruise edges. Each reaches the 100 mm/s feed and the final
    // brake to the buffer's tentative terminal rest fits inside the last edge, so
    // the entry of the last move (seam n-1) is still at cruise — that is the
    // barrier. The terminal seam (n, v=0) must never be selected.
    let out = outcome(
        vec![
            line_move(60.0, 100.0, 300.0, 1500.0, 1),
            line_move(60.0, 100.0, 300.0, 1500.0, 2),
            line_move(60.0, 100.0, 300.0, 1500.0, 3),
            line_move(60.0, 100.0, 300.0, 1500.0, 4),
        ],
        Vec::new(),
    );
    let plan = plan(&out).unwrap();
    let n = plan.moves.len();
    assert!(plan.barrier < n, "terminal rest must never be the barrier");
    assert_eq!(plan.barrier, n - 1);
    assert!(
        (plan.v_barrier - 100.0).abs() < 1e-6,
        "barrier pinned to cruise, got v_barrier {}",
        plan.v_barrier
    );
}

#[test]
fn warm_start_steep_decel_entry_is_not_rejected_as_a_rest_anchor() {
    // A short window entered fast: braking 57 -> 0 needs 57^2/(2*1000) = 1.62 mm,
    // which fits in 2 mm (feasible, passes the OverCommitted guard) but is steep.
    // The warm-start entry (v>0) must keep its real decel; it must NOT be
    // force-rested and rejected with RestAnchorAccel the way a true v=0 anchor is.
    let out = outcome(vec![line_move(2.0, 60.0, 100.0, 1000.0, 89)], Vec::new());
    let plan = plan_warm(&out, 57.0)
        .expect("feasible steep brake-to-rest must not be rejected as a corrupt rest anchor");
    assert_eq!(plan.moves[0].entry_v, 57.0);
}

#[test]
fn pin_rest_anchor_raises_on_nonzero_entry_accel() {
    let mut s = VelSample {
        s: 0.0,
        v: 0.0,
        a: 5.0,
    };
    assert_eq!(
        pin_rest_anchor(Some(&mut s), 7, 1.0e5),
        Err(VelocityError::RestAnchorAccel { line_no: 7 })
    );
}

#[test]
fn pin_rest_anchor_zeroes_small_accel() {
    let mut s = VelSample {
        s: 0.0,
        v: 0.0,
        a: 1e-9,
    };
    assert_eq!(pin_rest_anchor(Some(&mut s), 7, 1.0e5), Ok(()));
    assert_eq!(s.a, 0.0);
}

#[test]
fn pin_rest_anchor_tolerates_infinite_jerk_step() {
    let mut s = VelSample {
        s: 0.0,
        v: 0.0,
        a: 200.0,
    };
    assert_eq!(pin_rest_anchor(Some(&mut s), 7, f64::INFINITY), Ok(()));
    assert_eq!(s.a, 0.0);
}

#[test]
fn first_negative_velocity_flags_sub_zero_sample() {
    let samples = vec![
        VelSample {
            s: 0.0,
            v: 10.0,
            a: 0.0,
        },
        VelSample {
            s: 1.0,
            v: -13.5,
            a: 0.0,
        },
        VelSample {
            s: 2.0,
            v: 5.0,
            a: 0.0,
        },
    ];
    assert_eq!(first_negative_velocity(&samples), Some(-13.5));
}

#[test]
fn first_negative_velocity_ignores_float_noise_and_zero() {
    let samples = vec![
        VelSample {
            s: 0.0,
            v: 0.0,
            a: 0.0,
        },
        VelSample {
            s: 1.0,
            v: -1e-9,
            a: 0.0,
        },
        VelSample {
            s: 2.0,
            v: 42.0,
            a: 0.0,
        },
    ];
    assert_eq!(first_negative_velocity(&samples), None);
}

fn decel_ramp_into_tight_arc() -> Vec<Move> {
    let mut moves: Vec<Move> = (1..=3)
        .map(|i| line_move(0.5, 60.0, 500.0, 1000.0, i))
        .collect();
    moves.push(arc_move(0.2, 1.2, 60.0, 500.0, 1000.0, 4));
    moves.push(line_move(0.5, 60.0, 500.0, 1000.0, 5));
    moves
}

/// The 2026-07-02 Neptune bench crash geometry, distilled: a wipe ends in a
/// 0.062 mm line followed by an extrude-only retract (a rest anchor). The
/// profile crosses the seam entering the short line braking at ~-700 mm/s²;
/// a re-plan of the tail is feasible only if that in-flight deceleration is
/// carried across the cut — anchored at zero acceleration, the jerk-limited
/// stop needs more arc than the 0.062 mm move has.
fn wipe_into_retract() -> (Vec<Move>, Vec<bool>) {
    let lims = VelocityLimits::try_new(100.0, 1000.0, 30.0, 100_000.0).unwrap();
    let line = |len: f64, line_no: u32| {
        let seg = Segment::Line(Line::try_new([0.0, 0.0, 0.0], [len, 0.0, 0.0]).unwrap());
        Move {
            segment: PathSegment::try_new(seg, Vec::new()).unwrap(),
            feedrate_mm_s: 98.871,
            limits: lims,
            source: src(line_no),
        }
    };
    let retract = Move {
        segment: PathSegment::try_new_virtual(
            vec![FollowerDemand {
                axis_index: 3,
                ratio: 1.0,
            }],
            0.8,
        )
        .unwrap(),
        feedrate_mm_s: 25.0,
        limits: lims,
        source: src(3),
    };
    let moves = vec![line(1.0, 1), line(0.062, 2), retract, line(2.0, 4)];
    let stop_before = vec![false, false, true, true];
    (moves, stop_before)
}

fn plan_stops_full(
    moves: &[Move],
    stop_before: &[bool],
    entry: BoundaryState,
) -> Result<VelocityProfile, VelocityError> {
    plan_velocity_stops(moves, stop_before, 1e-4, 25.0, 1000.0, entry)
}

#[test]
fn mid_brake_seam_replans_from_the_carried_state() {
    let (moves, stop_before) = wipe_into_retract();
    let p = plan_stops_full(&moves, &stop_before, BoundaryState::REST).unwrap();
    let k = 1;
    assert!(k <= p.barrier, "the mid-brake seam sits inside the barrier");
    let entry = p.boundaries[k];
    assert!(
        entry.a < -1.0,
        "the profile must cross the seam decelerating (a = {})",
        entry.a
    );
    let replan = plan_stops_full(&moves[k..], &stop_before[k..], entry).unwrap_or_else(|e| {
        panic!(
            "re-plan from carried state (v={}, a={}) failed: {e:?}",
            entry.v, entry.a
        )
    });
    assert_eq!(replan.boundaries.last().copied(), Some(BoundaryState::REST));
}

/// The invariant streaming emission relies on: every boundary state at or
/// before the finality barrier warm-starts a re-plan of the remaining window
/// without error.
#[test]
fn barrier_boundary_states_are_valid_warm_starts() {
    let ramp = decel_ramp_into_tight_arc();
    let ramp_stops = vec![false; ramp.len()];
    let (wipe, wipe_stops) = wipe_into_retract();
    for (moves, stops) in [(&ramp, &ramp_stops), (&wipe, &wipe_stops)] {
        let p = plan_stops_full(moves, stops, BoundaryState::REST).unwrap();
        assert!(p.barrier >= 1);
        for k in 1..=p.barrier {
            plan_stops_full(&moves[k..], &stops[k..], p.boundaries[k]).unwrap_or_else(|e| {
                panic!(
                    "re-plan from boundaries[{k}] = {:?} failed: {e:?}",
                    p.boundaries[k]
                )
            });
        }
    }
}

/// A window cut must not bend the trajectory: the re-planned tail's samples
/// lie on the same jerk-limited curve as the uncut plan — velocity and
/// acceleration both continue across the seam.
#[test]
fn replanned_tail_continues_the_uncut_profile() {
    let (moves, stop_before) = wipe_into_retract();
    let p = plan_stops_full(&moves, &stop_before, BoundaryState::REST).unwrap();
    let k = 1;
    let replan = plan_stops_full(&moves[k..], &stop_before[k..], p.boundaries[k]).unwrap();
    for (uncut, cut) in p.moves[k..].iter().zip(replan.moves.iter()) {
        let entry_dv = (uncut.entry_v - cut.entry_v).abs();
        assert!(
            entry_dv < 1e-3,
            "velocity discontinuity {entry_dv} at re-planned move {}",
            uncut.source.start_line
        );
        let a_uncut = uncut.samples.first().unwrap().a;
        let a_cut = cut.samples.first().unwrap().a;
        assert!(
            (a_uncut - a_cut).abs() < 2.0,
            "acceleration discontinuity at move {}: uncut {a_uncut} vs re-planned {a_cut}",
            uncut.source.start_line
        );
    }
    let t_uncut: f64 = p.moves[k..]
        .iter()
        .map(|m| traversal_time(&m.samples))
        .sum();
    let t_cut: f64 = replan
        .moves
        .iter()
        .map(|m| traversal_time(&m.samples))
        .sum();
    assert!(
        (t_uncut - t_cut).abs() < 1e-3 * t_uncut.max(1.0),
        "the cut costs trajectory time: uncut tail {t_uncut}s vs re-planned {t_cut}s"
    );
}

#[test]
fn boundaries_span_the_window_and_anchor_entry() {
    let moves = decel_ramp_into_tight_arc();
    let entry = BoundaryState { v: 10.0, a: 0.0 };
    let stops = vec![false; moves.len()];
    let p = plan_stops_full(&moves, &stops, entry).unwrap();
    assert_eq!(p.boundaries.len(), moves.len() + 1);
    assert_eq!(p.boundaries[0], entry);
    assert_eq!(p.boundaries.last().copied(), Some(BoundaryState::REST));
}
