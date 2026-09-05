use super::fit_corners;
use crate::velocity::plan_velocity_warm_start;
use crate::{CornerFitConfig, Move, MoveContext, SourceRange, VelocityLimits, line_move};

const MAX_V: f64 = 100.0;
const ACCEL: f64 = 1000.0;
const SCV: f64 = 5.0;

fn ctx(max_v: f64, accel: f64, line_no: u32) -> MoveContext {
    MoveContext {
        extruder_axis: 0,
        feedrate_mm_s: max_v,
        limits: VelocityLimits::try_new(max_v, accel, SCV, f64::INFINITY).unwrap(),
        source: SourceRange {
            start_line: line_no,
            end_line: line_no,
        },
    }
}

fn acceleration_distance(max_v: f64, accel: f64) -> f64 {
    max_v * max_v / (2.0 * accel)
}

fn minimum_cruise_distance(max_v: f64, accel: f64) -> f64 {
    2.0 * acceleration_distance(max_v, accel)
}

struct Sample {
    s: f64,
    v: f64,
    a: f64,
}

fn run_samples(max_v: f64, accel: f64, waypoints: &[[f64; 3]]) -> Vec<Sample> {
    let moves: Vec<Move> = waypoints
        .windows(2)
        .enumerate()
        .map(|(i, w)| line_move(w[0], w[1], 0.0, ctx(max_v, accel, i as u32)).unwrap())
        .collect();
    let outcome = fit_corners(&moves, CornerFitConfig::default()).unwrap();
    let profile = plan_velocity_warm_start(
        &outcome,
        1e-7,
        f64::INFINITY,
        f64::INFINITY,
        crate::velocity::BoundaryState::REST,
    )
    .unwrap();
    let mut out = Vec::new();
    let mut s_off = 0.0;
    for m in &profile.moves {
        for smp in &m.samples {
            out.push(Sample {
                s: s_off + smp.s,
                v: smp.v,
                a: smp.a,
            });
        }
        s_off += m.length;
    }
    out
}

#[test]
fn cruise_onset_reaches_flat_velocity_ceiling() {
    let plateau_length = 5.0;
    let length = minimum_cruise_distance(MAX_V, ACCEL) + plateau_length;
    let samples = run_samples(MAX_V, ACCEL, &[[0.0, 0.0, 0.0], [length, 0.0, 0.0]]);
    assert!(
        samples.iter().any(|p| p.v >= MAX_V - 1e-3),
        "fixture must reach cruise"
    );
    assert!(
        samples
            .iter()
            .any(|p| p.v >= MAX_V - 1e-6 && p.a.abs() <= 1e-6),
        "cruise samples must stay on the zero-acceleration rail"
    );
}

#[test]
fn cruise_onset_tracks_ceiling_not_collinear_junction() {
    let d_accel = acceleration_distance(MAX_V, ACCEL);
    let length = minimum_cruise_distance(MAX_V, ACCEL) + 5.0;
    for l1 in [d_accel + 1.5, d_accel, d_accel - 1.5] {
        let samples = run_samples(
            MAX_V,
            ACCEL,
            &[[0.0, 0.0, 0.0], [l1, 0.0, 0.0], [length, 0.0, 0.0]],
        );
        assert!(
            samples.iter().any(|p| p.v >= MAX_V - 1e-3),
            "L1={l1:.2}: run must reach cruise"
        );
        for w in samples.windows(2).take_while(|w| w[0].v < MAX_V - 1e-9) {
            assert!(
                w[1].v + 1e-6 >= w[0].v,
                "L1={l1:.2}: v regressed at s={:.4} ({:.5} -> {:.5})",
                w[0].s,
                w[0].v,
                w[1].v
            );
        }
    }
}

#[test]
fn cruise_onset_uses_full_acceleration_limit() {
    let length = minimum_cruise_distance(MAX_V, ACCEL) + 1.0;
    let samples = run_samples(MAX_V, ACCEL, &[[0.0, 0.0, 0.0], [length, 0.0, 0.0]]);
    assert!(
        samples.iter().any(|p| p.a >= ACCEL - 1e-6),
        "acceleration-limited profile must reach a_max"
    );
    assert!(
        samples.iter().any(|p| p.a <= -ACCEL + 1e-6),
        "acceleration-limited profile must reach -a_max"
    );
    assert!(samples.iter().all(|p| p.a.abs() <= ACCEL + 1e-6));
}

#[test]
fn sub_cruise_triangular_straight_obeys_acceleration_law() {
    let length = 0.8 * minimum_cruise_distance(MAX_V, ACCEL);
    let samples = run_samples(MAX_V, ACCEL, &[[0.0, 0.0, 0.0], [length, 0.0, 0.0]]);
    let peak = samples.iter().fold(0.0_f64, |value, p| value.max(p.v));
    let expected_peak = (ACCEL * length).sqrt();
    assert!(peak < MAX_V - 1e-3, "fixture must stay sub-cruise");
    assert!(
        (peak - expected_peak).abs() <= 1e-3,
        "triangular peak {peak} must match sqrt(a_max * length)={expected_peak}"
    );
    assert!(samples.iter().all(|p| p.a.abs() <= ACCEL + 1e-6));
}

#[test]
fn cruise_plateau_interior_has_zero_acceleration() {
    let plateau_length = 30.0;
    let d_accel = acceleration_distance(MAX_V, ACCEL);
    let length = minimum_cruise_distance(MAX_V, ACCEL) + plateau_length;
    let samples = run_samples(MAX_V, ACCEL, &[[0.0, 0.0, 0.0], [length, 0.0, 0.0]]);
    let interior: Vec<&Sample> = samples
        .iter()
        .filter(|p| p.v >= MAX_V - 1e-6 && p.s > d_accel && p.s < d_accel + plateau_length)
        .collect();
    assert!(!interior.is_empty(), "expected a cruise plateau interior");
    assert!(
        interior.iter().all(|p| p.a.abs() <= 1e-6),
        "cruise plateau interior must stay on the a_t=0 rail"
    );
}

#[test]
fn short_straight_reaches_cruise_with_infinite_jerk() {
    let max_v = 30.0;
    let accel = 200.0;
    let length = 25.0;
    let required = minimum_cruise_distance(max_v, accel);
    assert!(
        length > required,
        "{length}mm must exceed the exact {required}mm acceleration-and-braking distance"
    );
    let samples = run_samples(max_v, accel, &[[0.0, 0.0, 0.0], [length, 0.0, 0.0]]);
    let peak = samples.iter().fold(0.0_f64, |value, p| value.max(p.v));
    assert!(
        peak >= max_v - 1e-3,
        "{length}mm exceeds the {required}mm distance required to reach {max_v}mm/s and brake"
    );
    assert!(samples.iter().all(|p| p.a.abs() <= accel + 1e-6));
}

#[test]
fn cruise_onset_accel_within_envelope() {
    let length = minimum_cruise_distance(MAX_V, ACCEL) + 1.0;
    let samples = run_samples(MAX_V, ACCEL, &[[0.0, 0.0, 0.0], [length, 0.0, 0.0]]);
    assert!(samples.iter().all(|p| p.a.abs() <= ACCEL + 1e-6));
    let (first, last) = (samples.first().unwrap(), samples.last().unwrap());
    assert_eq!((first.v, first.a), (0.0, 0.0));
    assert_eq!((last.v, last.a), (0.0, 0.0));
}
