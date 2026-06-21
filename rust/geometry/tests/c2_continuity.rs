//! T3 acceptance: the live velocity planner is C2-within-run / C1-at-rest.
//!
//! Reproduces the demo4 serpentine and asserts, reading the planned analytic
//! `a_t` (never finite-differenced):
//!   * accel-from-rest is a full-jerk trapezoid, not the C1 `(2/9)*jerk`
//!     velocity-ceiling ride (`max_reachable_velocity`);
//!   * tangential acceleration is continuous across mid-run junctions — the
//!     `+max -> -max` cruise/decel crossover step is dissolved by the bridge;
//!   * `|a_t| <= a_max` everywhere and `(v,a)=(0,0)` is pinned at rest anchors.
//!
//! Carve-out: the biclothoid corner apex carries a small tangential `a_t` step
//! (`a_t` tracks the curvature speed-limit slope `v*dv_lim/ds`, which inherits
//! the clothoid's `dkappa/ds` jump — a G2-not-G3 property). Per spec-motion-12
//! that lateral-jerk-induced step is the fitter shape's responsibility, not a
//! tangential-jerk-feasibility bug; it is bounded by `a_max` and is far below
//! the `2*a_max` C1 crossover step this work targets.

use geometry::{
    ChainFitConfig, Move, MoveContext, SourceRange, VelocityConfig, VelocityLimits, fit_chain,
    line_move, plan_velocity,
};

const MAX_V: f64 = 150.0;
const ACCEL: f64 = 200.0;
const JERK: f64 = 4000.0;
const SCV: f64 = 5.0;

fn ctx(line_no: u32) -> MoveContext {
    MoveContext {
        extruder_axis: 0,
        feedrate_mm_s: MAX_V,
        limits: VelocityLimits::try_new(MAX_V, ACCEL, SCV).unwrap(),
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
    let config = VelocityConfig {
        consistency_tol: 1e-6,
        max_jerk_mm_s3: JERK,
        integration_tol: 1e-7,
    };
    let profile = plan_velocity(&outcome, config).unwrap();
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
    // By s=0.5mm a full-jerk ramp has long since reached the accel plateau
    // (a=a_max at s~0.08mm). The C1 (2/9)*jerk ceiling-ride is still climbing
    // there: a ~ (2/3)*jerk^(2/3)*s^(1/3) ~ 0.66*a_max. Reading the plateau
    // separates the two unambiguously.
    let s = plan_samples();
    let probe = s
        .iter()
        .find(|p| p.s >= 0.5)
        .expect("profile reaches 0.5mm");
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
    // The C1 cruise/decel crossover steps a_t by 2*a_max (+max -> -max). After
    // bridging, no adjacent pair may step by even a_max — the corner κ' carve-out
    // stays well under that bound.
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
fn c2_zero_state_at_rest_anchors() {
    let s = plan_samples();
    let first = s.first().unwrap();
    let last = s.last().unwrap();
    assert_eq!((first.v, first.a), (0.0, 0.0));
    assert_eq!((last.v, last.a), (0.0, 0.0));
}
