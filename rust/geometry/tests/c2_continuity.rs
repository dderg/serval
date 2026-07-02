use geometry::path::CurvatureProfile;
use geometry::{
    ChainFitConfig, Move, MoveContext, SourceRange, VelocityLimits, fit_chain, line_move,
    plan_velocity,
};

const MAX_V: f64 = 150.0;
const ACCEL: f64 = 200.0;
const JERK: f64 = 4000.0;
const SCV: f64 = 5.0;

fn ctx(line_no: u32) -> MoveContext {
    MoveContext {
        extruder_axis: 0,
        feedrate_mm_s: MAX_V,
        limits: VelocityLimits::try_new(MAX_V, ACCEL, SCV, JERK).unwrap(),
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
    let outcome = fit_chain(&moves, ChainFitConfig::default()).unwrap();
    let profile = plan_velocity(&outcome, 1e-7, f64::INFINITY, f64::INFINITY).unwrap();
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
fn c2_accel_from_rest_is_full_jerk_trapezoid() {
    let accel_plateau_probe_mm = 0.5;
    let s = plan_samples();
    let probe = s
        .iter()
        .find(|p| p.s >= accel_plateau_probe_mm)
        .expect("profile reaches the accel plateau probe distance");
    assert!(
        probe.a >= 0.9 * ACCEL,
        "accel-from-rest a_t={:.2} at s={:.3} should be on the a_max plateau \
         ({ACCEL}); the C1 (2/9)*jerk ceiling ride only reaches ~{:.1} here",
        probe.a,
        probe.s,
        0.66 * ACCEL
    );
    assert!(probe.a <= ACCEL + 1e-6, "a_t must not exceed a_max");
}

#[test]
fn c2_no_crossover_accel_step() {
    let s = plan_samples();
    for w in s.windows(2) {
        let step = (w[1].a - w[0].a).abs();
        assert!(
            step < ACCEL,
            "tangential accel steps {step:.3} (>= {ACCEL}) at s={:.4}: {:.3} -> {:.3}",
            w[0].s,
            w[0].a,
            w[1].a
        );
    }
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
    let outcome = fit_chain(&moves, ChainFitConfig::default()).unwrap();
    let profile = plan_velocity(&outcome, 1e-7, f64::INFINITY, f64::INFINITY).unwrap();
    for (gm, vm) in outcome.moves.iter().zip(profile.moves.iter()) {
        let seg = gm.segment.spatial.as_ref().unwrap();
        for smp in &vm.samples {
            let kappa = seg.kappa(smp.s.clamp(0.0, seg.s_len())).abs();
            let a_n = kappa * smp.v * smp.v;
            let disk = (smp.a * smp.a + a_n * a_n).sqrt();
            assert!(
                disk <= ACCEL + 1e-3,
                "disk magnitude {disk:.3} exceeds a_max={ACCEL} at s={:.4} (a_t={:.2}, a_n={a_n:.2})",
                smp.s,
                smp.a
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
