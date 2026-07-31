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
    let seg = PathSegment::try_new_virtual(vec![FollowerDemand::constant(3, 0.05)], virtual_path)
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

/// Share of the disk rim a constant-curvature member's emitted profile actually
/// reaches. The rim is where the curvature has spent the whole acceleration
/// budget, so a ramp held to one cap set over its whole climb has no authority
/// left there and stops short — that was 0.858 of the rim. The cap ladder buys
/// each speed stretch the authority the disk leaves at its own speed and the
/// same arc measures 0.991. The bound is the cap direction — never above the
/// rim — plus the measured reach, so a loss of the ride is still caught.
const ARC_CRUISE_REACHED_SHARE: f64 = 0.99;

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
    let rim = (accel * radius).sqrt();
    let peak = plan.moves[0].peak_v;
    assert!(
        peak <= rim + 1e-3,
        "peak {peak} rode over the disk rim {rim}"
    );
    assert!(
        peak >= rim * ARC_CRUISE_REACHED_SHARE,
        "peak {peak} fell below the closed form's measured reach on the rim {rim}"
    );
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

/// The vector-jerk *cruise* ceiling `(jerk / sigma)^(1/3)` is gone: it only bound
/// at `a_t = 0`, so it flattened the clothoid instead of letting it ride the
/// acceleration disk. The emitted closed-form chain still rides well above the
/// blend's own constant-curvature ceiling `sqrt(accel / kappa_peak)`, which is
/// the capability that ceiling's removal bought. It no longer clears the removed
/// cruise value itself: the sampled marcher reached 79.37 mm/s here by riding the
/// ceiling curve, and the closed form reaches 59.06.
///
/// That shortfall is the *envelope's*, measured: the clothoid's peak is its own
/// entry speed and its exit sits exactly on the member ceiling, so the member is
/// braking the whole way and the plan never chooses the peak. The cap ladder
/// prices a plan's ramp at the authority each speed stretch really has and moved
/// this number not at all; what sets it is `reach_span`, which still buys the
/// whole brake envelope at one cap set.
#[test]
fn clothoid_rides_the_disk_above_its_constant_curvature_ceiling() {
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
    let apex_ceiling = (accel / kappa_peak).sqrt();
    assert!(
        line.exit_v < line.peak_v - 1.0,
        "the approach line must still decelerate into the clothoid"
    );
    assert!(
        clothoid.peak_v > apex_ceiling + 1.0,
        "clothoid peak {} must ride above its constant-curvature ceiling {apex_ceiling}",
        clothoid.peak_v
    );
    assert!(
        clothoid.peak_v >= 59.0,
        "clothoid peak {} fell below the closed form's measured ride",
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
        segment: PathSegment::try_new_virtual(vec![FollowerDemand::constant(3, 1.0)], 0.8).unwrap(),
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

/// The 2026-07-02 Neptune SCV-20 crash geometry: a wipe whose feedrate steps
/// down move-to-move (so the run is not a uniform-ceiling straight run and
/// reconstructs on the grid, not in closed form) ends in a 0.785 mm line
/// followed by an extrude-only retract. The profile crosses the seam entering
/// that last line already braking for the retract's rest anchor.
fn graded_wipe_into_retract() -> (Vec<Move>, Vec<bool>) {
    let lims = VelocityLimits::try_new(400.0, 2000.0, 20.0, 100_000.0).unwrap();
    let line = |len: f64, feed: f64, line_no: u32| {
        let seg = Segment::Line(Line::try_new([0.0, 0.0, 0.0], [len, 0.0, 0.0]).unwrap());
        Move {
            segment: PathSegment::try_new(seg, Vec::new()).unwrap(),
            feedrate_mm_s: feed,
            limits: lims,
            source: src(line_no),
        }
    };
    let extrude_only = |line_no: u32| Move {
        segment: PathSegment::try_new_virtual(vec![FollowerDemand::constant(3, 1.0)], 0.8).unwrap(),
        feedrate_mm_s: 25.0,
        limits: lims,
        source: src(line_no),
    };
    let moves = vec![
        line(1.2, 150.0, 1),
        line(0.9, 110.0, 2),
        line(0.785, 98.871_216_666_666_67, 3),
        extrude_only(4),
        line(2.766, 150.0, 5),
        extrude_only(6),
        line(0.224, 122.333_466_666_666_67, 7),
    ];
    let stop_before = vec![false, false, false, true, true, true, true];
    (moves, stop_before)
}

/// A cut on a grid-reconstructed brake stretch must carry the profile's true
/// state: the emitted curve is braking toward the retract's rest anchor, so
/// the carried acceleration is negative — not the forward integrator's
/// steering acceleration, which never brakes and reads a large positive value
/// while the velocity rides the descending brake envelope.
#[test]
fn grid_path_cut_carries_the_true_brake_state() {
    let (moves, stops) = graded_wipe_into_retract();
    let p = plan_stops_full(&moves, &stops, BoundaryState::REST).unwrap();
    let k = 2;
    assert!(k <= p.barrier);
    let entry = p.boundaries[k];
    assert!(
        entry.a < 0.0,
        "the profile crosses the seam braking for the retract, carried a = {}",
        entry.a
    );
    let replan = plan_stops_full(&moves[k..], &stops[k..], entry).unwrap_or_else(|e| {
        panic!(
            "re-plan from carried state (v={}, a={}) failed: {e:?}",
            entry.v, entry.a
        )
    });
    assert_eq!(replan.boundaries.last().copied(), Some(BoundaryState::REST));
}

/// The warm-start invariant on the grid-reconstruction path: the carried
/// velocity is a grid-integrated sample, while the next window re-derives its
/// entry bounds in closed form — every boundary at or before the barrier must
/// still be accepted.
#[test]
fn grid_path_boundary_states_are_valid_warm_starts() {
    let (moves, stops) = graded_wipe_into_retract();
    let p = plan_stops_full(&moves, &stops, BoundaryState::REST).unwrap();
    assert!(p.barrier >= 2);
    for k in 1..=p.barrier {
        plan_stops_full(&moves[k..], &stops[k..], p.boundaries[k]).unwrap_or_else(|e| {
            panic!(
                "re-plan from boundaries[{k}] = {:?} failed: {e:?}",
                p.boundaries[k]
            )
        });
    }
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

#[test]
fn infinite_jerk_disables_jerk_limiting_and_still_plans_to_rest() {
    let moves = vec![
        line_move(5.0, 100.0, 300.0, 3000.0, 1),
        line_move(5.0, 100.0, 300.0, 3000.0, 2),
    ];
    let finite = plan(&outcome(with_jerk(moves.clone(), 30_000.0), Vec::new())).unwrap();
    let inf = plan(&outcome(with_jerk(moves, f64::INFINITY), Vec::new())).unwrap();

    assert!(inf.report.traversal_time_s <= finite.report.traversal_time_s + 1e-9);
    assert_eq!(inf.boundaries.last().copied(), Some(BoundaryState::REST));
    let a_rail = 3000.0 * (1.0 + 1e-6);
    for m in &inf.moves {
        assert!(m.samples.iter().all(|s| s.a.abs() <= a_rail));
        assert!(m.samples.iter().all(|s| s.v <= 100.0 * (1.0 + 1e-6)));
        assert!(
            !m.phases.is_empty(),
            "a straight move under infinite jerk carries its trapezoid as zero-jerk phases"
        );
        for p in &m.phases {
            assert_eq!(p.j, 0.0);
            assert!(p.a0.abs() <= a_rail, "phase accel {} beyond rail", p.a0);
        }
        let phase_len: f64 = m
            .phases
            .iter()
            .map(|p| p.v0 * p.dt + 0.5 * p.a0 * p.dt * p.dt)
            .sum();
        assert!(
            (phase_len - 5.0).abs() <= 1e-6,
            "phases cover the move: {phase_len}"
        );
    }
    let trapezoid_accels: Vec<f64> = inf.moves[0].phases.iter().map(|p| p.a0).collect();
    assert!(
        trapezoid_accels.iter().any(|&a| a > 2999.0),
        "leading move accelerates at the rail: {trapezoid_accels:?}"
    );
}

/// The neptune_cube "discontinuity" corner (source line 308): a constant-
/// curvature arc entered near its curvature ceiling with the exit seam pinned
/// far below it. The backward brake-envelope pass used to integrate the
/// approach to the ceiling with `a` past the disk rail (the rail collapses
/// toward zero at the cap faster than the jerk budget — starved by the
/// rotating normal component — can shed `a`), touching the cap ~0.05 mm
/// before even the zero-jerk disk bound allows. The forward pass then rode
/// the cap into that infeasible envelope, and emission rail-clamped each
/// sample's `a` while `v` kept the envelope's super-disk descent — adjacent
/// samples disagreeing about the profile by 3x, which the lowering turned
/// into 2x executed acceleration spikes. Every adjacent sample window must be
/// kinematically consistent: chord acceleration within the accel budget,
/// total (chord + centripetal) acceleration within the budget, and the chord
/// within the window's endpoint `a` range.
#[test]
fn arc_brake_from_curvature_ceiling_emits_consistent_samples() {
    let (len, kappa) = (0.524_480_983_717_975_f64, 1.913_785_577_602_848_f64);
    let accel = 10_000.0;
    let feed = 98.871_216_666_666_67;
    let out = outcome(
        with_jerk(
            vec![
                arc_move(1.0 / kappa, len * kappa, feed, 1000.0, accel, 1),
                line_move(5.0, 18.365, 1000.0, accel, 2),
            ],
            2_000_000.0,
        ),
        Vec::new(),
    );
    let plan = plan_warm(&out, 65.663).unwrap();
    let arc = &plan.moves[0];
    assert!((arc.exit_v - 18.365).abs() < 0.5, "exit_v={}", arc.exit_v);
    let tol = 0.05 * accel;
    for w in arc.samples.windows(2) {
        let (p, q) = (&w[0], &w[1]);
        let ds = q.s - p.s;
        if ds <= 1e-9 {
            continue;
        }
        let chord = (q.v * q.v - p.v * p.v) / (2.0 * ds);
        assert!(
            chord.abs() <= accel + tol,
            "chord accel {chord:.0} exceeds budget at s={:.6}",
            p.s
        );
        let v_mid = 0.5 * (p.v + q.v);
        let a_n = kappa * v_mid * v_mid;
        let total = (chord * chord + a_n * a_n).sqrt();
        assert!(
            total <= accel + tol,
            "total accel {total:.0} exceeds budget at s={:.6} (chord={chord:.0}, a_n={a_n:.0})",
            p.s
        );
        let (lo, hi) = (p.a.min(q.a), p.a.max(q.a));
        assert!(
            chord >= lo - tol && chord <= hi + tol,
            "window mean accel {chord:.0} outside endpoint range [{lo:.0}, {hi:.0}] at s={:.6}",
            p.s
        );
    }
}

fn window_consistency(m: &MoveVelocity, kappa0: f64, sigma: f64, accel: f64) {
    let tol = 0.05 * accel;
    for w in m.samples.windows(2) {
        let (p, q) = (&w[0], &w[1]);
        let ds = q.s - p.s;
        if ds <= 1e-9 {
            continue;
        }
        let chord = (q.v * q.v - p.v * p.v) / (2.0 * ds);
        let kappa = (kappa0 + sigma * 0.5 * (p.s + q.s)).abs();
        let v_mid = 0.5 * (p.v + q.v);
        let total = (chord * chord + (kappa * v_mid * v_mid).powi(2)).sqrt();
        assert!(
            total <= accel + tol,
            "total accel {total:.0} exceeds budget at s={:.6} (chord={chord:.0})",
            p.s
        );
    }
}

/// The corner at waypoints 249-254 of neptune_cube/discontinuity under
/// printer limits (v=100, a=1000, j=1e6): straight, clothoid up to kappa
/// 1.6056, constant-curvature arc, clothoid down, straight. The arc pins the
/// profile exactly on the curvature ceiling (rail = 0), and the flight
/// landing that reattaches the backward envelope to the ascending ceiling
/// coming off that pinch used to adopt the cap chord's slope unclamped —
/// teleporting the envelope one super-disk velocity step up, which the
/// binding-envelope gate then rejected, failing the whole plan on ordinary
/// sliced gcode. The plan must succeed and every corner move's samples must
/// stay within the acceleration disk.
#[test]
fn curvature_pinch_corner_plans_with_disk_consistent_samples() {
    let (accel, jerk) = (1000.0, 1_000_000.0);
    let feed = 98.871_216_666_666_67;
    let (clo_len, clo_k1) = (0.326_111_947_928_510_2_f64, 1.605_579_859_690_027_5_f64);
    let sigma = clo_k1 / clo_len;
    let arc_len = 0.651_922_925_578_820_3_f64;
    let up = Clothoid::try_new(
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        0.0,
        sigma,
        clo_len,
    )
    .unwrap();
    let down = Clothoid::try_new(
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        clo_k1,
        -sigma,
        clo_len,
    )
    .unwrap();
    let arc = Arc::try_new(
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        1.0 / clo_k1,
        0.0,
        arc_len * clo_k1,
    )
    .unwrap();
    let out = outcome(
        with_jerk(
            vec![
                line_move(9.934_551_366_509_675, feed, 100.0, accel, 1),
                spatial_move(Segment::Clothoid(up), feed, 100.0, accel, 2),
                spatial_move(Segment::Arc(arc), feed, 100.0, accel, 2),
                spatial_move(Segment::Clothoid(down), feed, 100.0, accel, 3),
                line_move(9.934_551_366_509_675, feed, 100.0, accel, 4),
            ],
            jerk,
        ),
        Vec::new(),
    );
    let plan = plan(&out).expect("ordinary sliced corner must plan");
    window_consistency(&plan.moves[1], 0.0, sigma, accel);
    window_consistency(&plan.moves[2], clo_k1, 0.0, accel);
    window_consistency(&plan.moves[3], clo_k1, -sigma, accel);
}

/// The reference corner blend as `fit_corners` builds it at corner_deviation
/// 0.05 over a 90 degree turn: two clothoid halves meeting at the curvature
/// peak, with a straight lead-in and lead-out.
fn reference_blend_moves() -> (Vec<Move>, f64, f64, f64, f64) {
    let (accel, jerk, feed) = (60_000.0, 1.5e8, 300.0);
    let half_len = 0.141_146_f64;
    let kappa_peak = 11.128_902_f64;
    let sigma = kappa_peak / half_len;
    let up = Clothoid::try_new(
        [0.0; 3],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        0.0,
        sigma,
        half_len,
    )
    .unwrap();
    let down = Clothoid::try_new(
        [0.0; 3],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        kappa_peak,
        -sigma,
        half_len,
    )
    .unwrap();
    let moves = with_jerk(
        vec![
            line_move(10.0, feed, feed, accel, 1),
            line_move(0.5, feed, feed, accel, 2),
            spatial_move(Segment::Clothoid(up), feed, feed, accel, 3),
            spatial_move(Segment::Clothoid(down), feed, feed, accel, 4),
            line_move(10.0, feed, feed, accel, 5),
        ],
        jerk,
    );
    (moves, sigma, accel, jerk, feed)
}

#[test]
fn a_blend_entry_seam_is_capped_by_the_clothoid_jerk_rail_the_disk_never_sees() {
    let (moves, sigma, _accel, jerk, feed) = reference_blend_moves();
    let plan = plan(&outcome(moves, Vec::new())).expect("the reference blend must plan");

    let seam = plan.boundaries[2].v;
    assert!(
        sigma * seam * seam * seam <= jerk,
        "kappa = 0 blend seam at {seam} mm/s owes sigma*v^3 = {} of a {jerk} budget",
        sigma * seam * seam * seam
    );
    assert!(
        seam < feed,
        "the rail must bind below the feedrate, got {seam} mm/s"
    );
}

#[test]
fn a_brake_limited_straight_demands_a_braking_entry_acceleration() {
    let (moves, sigma, accel, jerk, feed) = reference_blend_moves();
    let mut report = VelocityReport::default();
    let caps = build_move_caps(&moves, f64::INFINITY, f64::INFINITY, &mut report).unwrap();
    let apex = disk::limit_speed(sigma * caps[2].kin.length, accel);

    let required = required_entry_state(&caps[1].kin, BoundaryState { v: apex, a: 0.0 })
        .expect("a straight always names its entry requirement");

    assert!(
        required.a < 0.0,
        "a short straight braking into the blend apex must be entered already braking, got {}",
        required.a
    );
    assert!(required.a.abs() <= accel, "required brake over the budget");
    assert!(required.v > apex && required.v <= feed);
    assert!(
        sigma * required.v.powi(3) > jerk,
        "the straight owes no normal jerk, so its own requirement may sit above the blend's rail"
    );
}

/// The apex is where the blend is slowest, so the half before it is entered fast
/// at its own `kappa = 0` seam and arrives already braking. The seam has no
/// authority to build that brake — the normal jerk there is `sigma v^3` with no
/// acceleration term in it — so the requirement has to name the brake, not a
/// cruise.
#[test]
fn a_blend_half_demands_a_braking_entry_above_its_apex() {
    let (moves, sigma, accel, _jerk, _feed) = reference_blend_moves();
    let mut report = VelocityReport::default();
    let caps = build_move_caps(&moves, f64::INFINITY, f64::INFINITY, &mut report).unwrap();
    let apex = disk::limit_speed(sigma * caps[2].kin.length, accel);

    let required = required_entry_state(&caps[2].kin, BoundaryState { v: apex, a: 0.0 })
        .expect("the up-clothoid must name its entry requirement");

    assert!(
        required.v > apex,
        "the seam must be entered above the apex it dips to, got {} against {apex}",
        required.v
    );
    assert!(
        required.a < 0.0,
        "a half braking into its apex must be entered already braking, got {}",
        required.a
    );
    assert!(
        required.a.abs() <= accel,
        "required brake {} over the budget {accel}",
        required.a
    );
}

#[test]
fn a_required_entry_state_actually_closes_its_member() {
    let (moves, _sigma, _accel, _jerk, _feed) = reference_blend_moves();
    let mut report = VelocityReport::default();
    let caps = build_move_caps(&moves, f64::INFINITY, f64::INFINITY, &mut report).unwrap();
    let kin = &caps[0].kin;
    let exit = BoundaryState { v: 40.0, a: -250.0 };

    let required = required_entry_state(kin, exit).expect("a straight always names its entry");
    let chain = profile::straight_chain_between(
        (required.v, required.a),
        (exit.v, exit.a),
        kin.length,
        kin.flat_ceiling,
        kin.accel,
        kin.jerk,
    )
    .expect("the state a member requires at its entry must close that member");
    let (_, v_end, a_end) = chain.last().unwrap().end_state();
    assert!((v_end - exit.v).abs() < 1e-6, "landed at {v_end}");
    assert!((a_end - exit.a).abs() < 1e-3, "landed with a = {a_end}");
}

#[test]
fn the_envelope_publishes_boundary_accelerations_and_names_what_it_cannot_reach() {
    let (moves, _sigma, _accel, _jerk, _feed) = reference_blend_moves();
    let plan = plan(&outcome(moves, Vec::new())).expect("the reference blend must plan");
    let report = plan.report;

    assert!(
        report.boundary_accel_seams > 0,
        "a corner's blend halves must pin at least one seam acceleration"
    );
    assert!(report.worst_boundary_accel_mm_s2 > 0.0);
    assert_eq!(
        report.worst_unreachable.is_some(),
        report.reachability.unreachable > 0,
        "an unreachable count must come with the member that produced it"
    );
    if let Some(worst) = report.worst_unreachable {
        assert!(worst.move_index < plan.moves.len());
    }
}
