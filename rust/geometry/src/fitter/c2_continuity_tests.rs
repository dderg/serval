use super::fit_corners;
use crate::path::CurvatureProfile;
use crate::velocity::plan_velocity_warm_start;
use crate::{CornerFitConfig, Move, MoveContext, SourceRange, VelocityLimits, line_move};

const MAX_V: f64 = 150.0;
const ACCEL: f64 = 200.0;
const MAX_JERK: f64 = f64::INFINITY;
const SCV: f64 = 5.0;

fn ctx(line_no: u32) -> MoveContext {
    MoveContext {
        extruder_axis: 0,
        feedrate_mm_s: MAX_V,
        limits: VelocityLimits::try_new(MAX_V, ACCEL, SCV, MAX_JERK).unwrap(),
        source: SourceRange {
            start_line: line_no,
            end_line: line_no,
        },
    }
}

fn serpentine() -> Vec<Move> {
    let pts = [
        [0.0, 0.0, 0.0],
        [90.0, 0.0, 0.0],
        [90.0, 90.0, 0.0],
        [180.0, 90.0, 0.0],
        [180.0, 0.0, 0.0],
    ];
    pts.windows(2)
        .enumerate()
        .map(|(i, w)| line_move(w[0], w[1], 0.0, ctx(i as u32)).unwrap())
        .collect()
}

struct Sample {
    s: f64,
    v: f64,
    a: f64,
}

fn plan_samples() -> Vec<Sample> {
    let moves = serpentine();
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
fn c2_accel_from_rest_reaches_acceleration_limit() {
    let accel_plateau_probe_mm = 0.5;
    let s = plan_samples();
    let probe = s
        .iter()
        .find(|p| p.s >= accel_plateau_probe_mm)
        .expect("profile reaches the accel plateau probe distance");
    assert!(
        probe.a >= 0.9 * ACCEL,
        "accel-from-rest a_t={:.2} at s={:.3} should reach the a_max plateau ({ACCEL})",
        probe.a,
        probe.s,
    );
    assert!(probe.a <= ACCEL + 1e-6, "a_t must not exceed a_max");
}

#[test]
fn c2_velocity_is_continuous_at_sample_boundaries() {
    let samples = plan_samples();
    let mut boundaries = 0;
    for w in samples.windows(2) {
        if (w[1].s - w[0].s).abs() <= 1e-9 {
            boundaries += 1;
            assert!(
                (w[1].v - w[0].v).abs() <= 1e-6,
                "velocity discontinuity at s={:.4}: {:.6} -> {:.6}",
                w[0].s,
                w[0].v,
                w[1].v
            );
        }
    }
    assert!(boundaries > 0, "fixture must include sample boundaries");
}

#[test]
fn c2_accel_within_envelope() {
    for smp in &plan_samples() {
        assert!(
            smp.a.abs() <= ACCEL + 1e-6,
            "|a_t|={:.4} exceeds a_max={ACCEL} at s={:.4}",
            smp.a.abs(),
            smp.s
        );
    }
}

#[test]
fn c2_tangential_within_acceleration_disk() {
    let moves = serpentine();
    let outcome = fit_corners(&moves, CornerFitConfig::default()).unwrap();
    let profile = plan_velocity_warm_start(
        &outcome,
        1e-7,
        f64::INFINITY,
        f64::INFINITY,
        crate::velocity::BoundaryState::REST,
    )
    .unwrap();
    for (gm, vm) in outcome.moves.iter().zip(profile.moves.iter()) {
        let seg = gm.segment.spatial.as_ref().unwrap();
        for (sample_index, smp) in vm.samples.iter().enumerate() {
            let a_t = smp.a;
            let kappa = seg.kappa(smp.s).abs();
            let a_n = kappa * smp.v * smp.v;
            let disk = (a_t * a_t + a_n * a_n).sqrt();
            assert!(
                disk <= ACCEL + 1e-3,
                "disk magnitude {disk:.3} exceeds a_max={ACCEL} at move line {} sample {sample_index}, local s={:.4} (kappa={kappa:.6}, a_t={a_t:.2}, a_n={a_n:.2})",
                gm.source.start_line,
                smp.s,
            );
        }
    }
}

#[test]
fn c2_zero_state_at_rest_anchors() {
    let s = plan_samples();
    let first = s.first().unwrap();
    let last = s.last().unwrap();
    assert_eq!((first.v, first.a), (0.0, 0.0));
    assert_eq!((last.v, last.a), (0.0, 0.0));
}
