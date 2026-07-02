use super::causal::fit;
use crate::path::Segment;
use crate::path::lowering::LoweredSample;
use crate::velocity::plan_velocity_warm_start;
use crate::{
    ChainFitConfig, FitOutcome, Move, MoveContext, SourceRange, VelocityLimits, VelocityProfile,
    arc_move, line_move, lower_profile,
};

const MAX_V: f64 = 300.0;
const ACCEL: f64 = 5000.0;
const SCV: f64 = 5.0;
const JERK: f64 = 100_000.0;
const RATE_HZ: f64 = 20_000.0;

const POS_TOL: f64 = 1e-6;
const SPEED_REL_TOL: f64 = 1e-3;

fn ctx(line_no: u32, feedrate_mm_s: f64) -> MoveContext {
    MoveContext {
        extruder_axis: 3,
        feedrate_mm_s,
        limits: VelocityLimits::try_new(MAX_V, ACCEL, SCV, JERK).unwrap(),
        source: SourceRange {
            start_line: line_no,
            end_line: line_no,
        },
    }
}

fn line(line_no: u32, feed: f64, start: [f64; 3], end: [f64; 3], e: f64) -> Move {
    line_move(start, end, e, ctx(line_no, feed)).expect("line_move")
}

fn line_lim(
    line_no: u32,
    feed: f64,
    start: [f64; 3],
    end: [f64; 3],
    max_v: f64,
    accel: f64,
    scv: f64,
) -> Move {
    let ctx = MoveContext {
        extruder_axis: 3,
        feedrate_mm_s: feed,
        limits: VelocityLimits::try_new(max_v, accel, scv, JERK).unwrap(),
        source: SourceRange {
            start_line: line_no,
            end_line: line_no,
        },
    };
    line_move(start, end, 0.0, ctx).expect("line_move")
}

#[test]
fn slow_zhop_does_not_cap_following_fast_travel() {
    // A slow Z-hop (max_z_velocity=5, max_z_accel=100) chained with a long fast
    // XY travel (max_v=500, accel=30000). The travel must plan against its OWN
    // X-axis acceleration — not inherit the Z-hop's, which would cap a 240 mm move
    // to ~155 mm/s peak (~78 mm/s average). This is the z_tilt post-nudge slowdown.
    let z = line_lim(1, 5.0, [0.0, 0.0, 0.0], [0.0, 0.0, 1.8], 5.0, 100.0, 1.0);
    let xy = line_lim(
        2,
        500.0,
        [0.0, 0.0, 1.8],
        [-240.0, 0.0, 1.8],
        500.0,
        30000.0,
        1.0,
    );
    let p = plan(&[z, xy]);
    let mut line_peak = 0.0_f64;
    for (gm, pm) in p.geometry.moves.iter().zip(&p.profile.moves) {
        if !is_clothoid(gm) {
            line_peak = line_peak.max(pm.peak_v);
        }
    }
    assert!(
        line_peak > 400.0,
        "fast XY travel after a slow Z-hop capped at peak {line_peak} mm/s; \
         it should approach its own 500 mm/s feed (Z-hop accel must not bleed in)"
    );
}

fn polyline(feed: f64, verts: &[[f64; 3]]) -> Vec<Move> {
    verts
        .windows(2)
        .enumerate()
        .map(|(i, w)| line(i as u32 + 1, feed, w[0], w[1], 0.0))
        .collect()
}

struct Planned {
    geometry: FitOutcome,
    profile: VelocityProfile,
    samples: Vec<LoweredSample>,
}

fn plan(moves: &[Move]) -> Planned {
    let geometry = fit(moves, ChainFitConfig::default()).expect("fit");
    let profile = plan_velocity_warm_start(
        &geometry,
        1e-7,
        f64::INFINITY,
        f64::INFINITY,
        crate::velocity::BoundaryState::REST,
    )
    .expect("plan_velocity_warm_start");
    let samples = lower_profile(&geometry, &profile, RATE_HZ).expect("lower_profile");
    Planned {
        geometry,
        profile,
        samples,
    }
}

fn dist(a: [f64; 3], b: [f64; 3]) -> f64 {
    let dx = b[0] - a[0];
    let dy = b[1] - a[1];
    let dz = b[2] - a[2];
    (dx * dx + dy * dy + dz * dz).sqrt()
}

fn is_clothoid(m: &Move) -> bool {
    matches!(m.segment.spatial, Some(Segment::Clothoid(_)))
}

fn assert_trajectory_invariants(p: &Planned) {
    let s = &p.samples;
    assert!(!s.is_empty(), "trajectory produced no samples");

    assert_eq!(s[0].t_s, 0.0, "trajectory must start at t=0");
    let mut prev_t = f64::NEG_INFINITY;
    for smp in s {
        assert!(smp.t_s.is_finite(), "non-finite sample time");
        assert!(
            smp.t_s + 1e-12 >= prev_t,
            "time went backwards: {} after {}",
            smp.t_s,
            prev_t
        );
        prev_t = smp.t_s;
        if let Some(pos) = smp.position {
            assert!(
                pos.iter().all(|c| c.is_finite()),
                "non-finite position {pos:?}"
            );
        }
        assert!(
            smp.followers.iter().all(|f| f.is_finite()),
            "non-finite follower sample {:?}",
            smp.followers
        );
    }
    assert!(
        s.last().unwrap().t_s > 0.0,
        "total trajectory time must be positive"
    );

    let step = 1.0 / RATE_HZ;
    let cap = MAX_V * (1.0 + SPEED_REL_TOL);
    for (i, w) in s.windows(2).enumerate() {
        let dt = w[1].t_s - w[0].t_s;
        if i + 2 == s.len() {
            assert!(
                dt > 0.0 && dt <= step + 1e-12,
                "final interval {dt} should be in (0, {step}]"
            );
        } else {
            assert!(
                (dt - step).abs() <= step * 1e-6,
                "non-uniform sample interval {dt}, expected {step}"
            );
        }
        if let (Some(a), Some(b)) = (w[0].position, w[1].position) {
            let v = dist(a, b) / dt;
            assert!(v <= cap, "per-step speed {v} exceeds cap {cap}");
        }
    }

    for w in p.profile.moves.windows(2) {
        let diff = (w[0].exit_v - w[1].entry_v).abs();
        assert!(
            diff <= 1e-9,
            "velocity discontinuity at junction: exit {} vs entry {}",
            w[0].exit_v,
            w[1].entry_v
        );
    }
}

fn assert_reaches(p: &Planned, terminal: [f64; 3]) {
    let last = p
        .samples
        .last()
        .unwrap()
        .position
        .expect("spatial trajectory must carry an endpoint position");
    assert!(
        dist(last, terminal) < POS_TOL,
        "trajectory ended at {last:?}, expected {terminal:?}"
    );
}

#[test]
fn cornered_polyline_blends_and_slows_through_corners() {
    let non_cocircular_verts = [
        [0.0, 0.0, 0.0],
        [50.0, 0.0, 0.0],
        [80.0, 35.0, 0.0],
        [30.0, 55.0, 0.0],
        [5.0, 20.0, 0.0],
    ];
    let p = plan(&polyline(200.0, &non_cocircular_verts));

    assert!(
        p.geometry.report.blended > 0,
        "expected corner blends, got report {:?}",
        p.geometry.report
    );
    assert!(
        p.geometry.moves.iter().any(is_clothoid),
        "expected at least one clothoid blend segment"
    );
    assert_trajectory_invariants(&p);
    assert_reaches(&p, [5.0, 20.0, 0.0]);

    let mut line_peak = 0.0_f64;
    let mut clothoid_min = f64::INFINITY;
    for (gm, pm) in p.geometry.moves.iter().zip(&p.profile.moves) {
        match gm.segment.spatial {
            Some(Segment::Clothoid(_)) => clothoid_min = clothoid_min.min(pm.peak_v),
            Some(_) => line_peak = line_peak.max(pm.peak_v),
            None => {}
        }
    }
    assert!(
        line_peak >= 0.9 * 200.0,
        "straight legs should accelerate to near feed, peaked at {line_peak}"
    );
    assert!(
        clothoid_min < line_peak,
        "sharpest corner {clothoid_min} should be slower than straight cruise {line_peak}"
    );
}

#[test]
fn square_stays_sharp_without_arc_fit() {
    let cocircular_square_verts = [
        [0.0, 0.0, 0.0],
        [50.0, 0.0, 0.0],
        [50.0, 50.0, 0.0],
        [0.0, 50.0, 0.0],
        [0.0, 0.0, 0.0],
    ];
    let p = plan(&polyline(200.0, &cocircular_square_verts));

    assert_eq!(
        p.geometry.report.chains, 0,
        "square must not reconstruct as a chain with arc fitting off, got report {:?}",
        p.geometry.report
    );
    assert_trajectory_invariants(&p);
    assert_reaches(&p, [0.0, 0.0, 0.0]);

    let excursion = p
        .samples
        .iter()
        .filter_map(|s| s.position)
        .map(|pos| dist(pos, [0.0, 0.0, 0.0]))
        .fold(0.0, f64::max);
    assert!(
        excursion > 40.0,
        "square trajectory barely left origin: {excursion}"
    );
}

#[test]
fn arc_path_is_curvature_bounded() {
    let feed = 250.0;
    let arc_radius = 5.0;
    let curvature_speed_cap = (ACCEL * arc_radius).sqrt();
    assert!(
        curvature_speed_cap < feed,
        "test only exercises curvature binding when the cap {curvature_speed_cap} is below feed {feed}"
    );
    let arc = arc_move(
        [20.0, 0.0, 0.0],
        [25.0, 5.0, 0.0],
        0.0,
        arc_radius,
        true,
        0.0,
        ctx(2, feed),
    )
    .expect("arc_move");
    let moves = vec![
        line(1, feed, [0.0, 0.0, 0.0], [20.0, 0.0, 0.0], 0.0),
        arc,
        line(3, feed, [25.0, 5.0, 0.0], [45.0, 5.0, 0.0], 0.0),
    ];
    let p = plan(&moves);

    assert!(
        p.profile.report.curvature_bound >= 1,
        "tight arc should register a curvature-bounded move, report {:?}",
        p.profile.report
    );
    assert_trajectory_invariants(&p);
    assert_reaches(&p, [45.0, 5.0, 0.0]);
}

#[test]
fn extruding_move_lowers_monotone_followers() {
    let e_delta = 2.0;
    let moves = vec![line(1, 60.0, [0.0, 0.0, 0.0], [40.0, 0.0, 0.0], e_delta)];
    let p = plan(&moves);

    assert_trajectory_invariants(&p);
    assert_reaches(&p, [40.0, 0.0, 0.0]);

    let mut prev = f64::NEG_INFINITY;
    for smp in &p.samples {
        assert_eq!(smp.followers.len(), 1, "expected one follower lane");
        let f = smp.followers[0];
        assert!(f + 1e-12 >= prev, "follower decreased: {f} after {prev}");
        prev = f;
    }
    let last_f = p.samples.last().unwrap().followers[0];
    assert!(
        (last_f - e_delta).abs() < 1e-6,
        "follower should reach e_delta {e_delta}, got {last_f}"
    );
}

#[test]
fn long_straight_cruises_at_feed_cap() {
    let feed = 150.0;
    let moves = vec![line(1, feed, [0.0, 0.0, 0.0], [200.0, 0.0, 0.0], 0.0)];
    let p = plan(&moves);

    assert_eq!(p.geometry.report.blended, 0, "single move must not blend");
    assert_eq!(p.geometry.report.chains, 0, "single move must not chain");
    assert!(
        p.profile.report.feedrate_bound >= 1,
        "straight move should be feedrate-bounded"
    );
    let peak = p.profile.moves[0].peak_v;
    assert!(
        (peak - feed).abs() < 0.01 * feed,
        "long straight should cruise at feed {feed}, peaked at {peak}"
    );
    assert_trajectory_invariants(&p);
    assert_reaches(&p, [200.0, 0.0, 0.0]);
}

#[test]
fn single_move_passthrough() {
    let moves = vec![line(1, 100.0, [0.0, 0.0, 0.0], [10.0, 0.0, 0.0], 0.0)];
    let outcome = fit(&moves, ChainFitConfig::default()).expect("fit");
    assert_eq!(
        outcome.moves.len(),
        1,
        "single move passes through unchanged"
    );
    assert_eq!(outcome.report.blended, 0);

    let p = plan(&moves);
    assert_trajectory_invariants(&p);
    assert_reaches(&p, [10.0, 0.0, 0.0]);
}

#[test]
fn empty_input_yields_empty_trajectory() {
    let moves: Vec<Move> = Vec::new();
    let geometry = fit(&moves, ChainFitConfig::default()).expect("fit on empty");
    assert!(geometry.moves.is_empty());

    let profile = plan_velocity_warm_start(
        &geometry,
        1e-7,
        f64::INFINITY,
        f64::INFINITY,
        crate::velocity::BoundaryState::REST,
    )
    .expect("plan_velocity_warm_start empty");
    assert!(profile.moves.is_empty());

    let samples = lower_profile(&geometry, &profile, RATE_HZ).expect("lower_profile empty");
    assert!(
        samples.is_empty(),
        "empty input must yield empty trajectory"
    );
}
