//! Motion-14 acceptance: the cruise on/off-ramp is jerk-limited.
//!
//! The velocity planner jerk-limits the *start* of acceleration (jerk-up from
//! rest) but not its *end*. When `v` reaches `max_velocity` and the move goes
//! to cruise, tangential acceleration `a_t` steps discontinuously from its
//! current value (here `sqrt(2*j*v_max) ~ 894`) to `0` — an unbounded
//! tangential jerk at every cruise entry, mirrored leaving cruise into a decel
//! (`0 -> -a`). This is a C1 violation on the flat velocity ceiling, distinct
//! from the mid-run sign-flip crossovers Motion-12 T3 already bridges.
//!
//! The minimal repro is a single trapezoidal straight (`L=40, v100 a1000
//! j4000`) that reaches cruise: `d_accel ~ 7.45mm`, cruise reached at
//! `a ~ sqrt(2*j*v_max) ~ 894` (jerk-up never reaches `a_max=1000` because
//! `v_max=100 < a_max^2/(2j)=125`). On the committed planner `a_t` steps
//! `894 -> 0` at the cruise onset (`s~7.45`) and `0 -> -894` at the cruise exit
//! (`s~32.55`).
//!
//! `a` here is the planner's analytic tangential accel (`reconstruct_run` ->
//! `eval_profile().a`), never a finite difference of `v`.
//!
//! `EPS_A` discriminates "step" from "jerk-limited roll-off": a jerk-limited
//! arc changes `a_t` by at most `max_jerk * dt_sample` per step, which for the
//! bridge's roll-off grid is `~a_max/48`; we allow `a_max/10`, a 5x margin over
//! that grid yet an order of magnitude below the `a_max`-scale C1 step. The
//! `c2_continuity` C1-crossover test cannot catch this — its `< a_max` bound is
//! looser than the `894 < 1000` cruise step.

use geometry::{
    ChainFitConfig, Move, MoveContext, SourceRange, VelocityConfig, VelocityLimits, fit_chain,
    line_move, plan_velocity,
};

const MAX_V: f64 = 100.0;
const ACCEL: f64 = 1000.0;
const JERK: f64 = 4000.0;
const SCV: f64 = 5.0;
const EPS_A: f64 = ACCEL / 10.0;

fn ctx(max_v: f64, accel: f64, line_no: u32) -> MoveContext {
    MoveContext {
        extruder_axis: 0,
        feedrate_mm_s: max_v,
        limits: VelocityLimits::try_new(max_v, accel, SCV).unwrap(),
        source: SourceRange {
            start_line: line_no,
            end_line: line_no,
        },
    }
}

struct Sample {
    s: f64,
    v: f64,
    a: f64,
}

/// Plan a straight run through the live velocity planner and return its samples
/// with cumulative arclength. `waypoints` are collinear so the run spans them
/// as one uninterrupted profile (collinear junctions stay transparent).
fn run_samples(max_v: f64, accel: f64, jerk: f64, waypoints: &[[f64; 3]]) -> Vec<Sample> {
    let moves: Vec<Move> = waypoints
        .windows(2)
        .enumerate()
        .map(|(i, w)| line_move(w[0], w[1], 0.0, ctx(max_v, accel, i as u32)).unwrap())
        .collect();
    let outcome = fit_chain(&moves, ChainFitConfig::default()).unwrap();
    let config = VelocityConfig {
        consistency_tol: 1e-6,
        max_jerk_mm_s3: jerk,
        integration_tol: 1e-7,
        ..VelocityConfig::default()
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

/// Adjacent-sample `a_t` steps that exceed the jerk-limited tolerance.
fn accel_steps(samples: &[Sample], eps: f64) -> Vec<(f64, f64, f64)> {
    samples
        .windows(2)
        .map(|w| ((w[1].a - w[0].a).abs(), w[0].s, w[1].a))
        .filter(|(step, ..)| *step > eps)
        .collect()
}

fn assert_no_step(samples: &[Sample], eps: f64, label: &str) {
    let steps: Vec<_> = samples
        .windows(2)
        .map(|w| ((w[1].a - w[0].a).abs(), w[0].s, w[0].a, w[1].a))
        .filter(|(step, ..)| *step > eps)
        .collect();
    assert!(
        steps.is_empty(),
        "{label}: tangential accel steps (cruise on/off-ramp must be \
         jerk-limited, |Δa_t| <= {eps}):\n{}",
        steps
            .iter()
            .map(|(step, s, a0, a1)| format!("  s={s:.4}: {a0:.2} -> {a1:.2} (|Δa_t|={step:.2})"))
            .collect::<Vec<_>>()
            .join("\n")
    );
}

#[test]
fn cruise_onset_no_tangential_accel_step() {
    let samples = run_samples(MAX_V, ACCEL, JERK, &[[0.0, 0.0, 0.0], [40.0, 0.0, 0.0]]);
    assert!(
        samples.iter().any(|p| p.v >= MAX_V - 1e-3),
        "fixture must reach cruise"
    );
    assert_no_step(&samples, EPS_A, "single trapezoidal straight");
}

#[test]
fn cruise_onset_tracks_ceiling_not_collinear_junction() {
    // d_accel(rest->cruise, v100 a1000 j4000) ~ 7.45mm. Placing a collinear
    // junction just before / at / just after the cruise onset must not move the
    // roll-off off s=d_accel, and the junction itself stays transparent — both
    // captured by "no a_t step anywhere", since a junction-induced step would
    // appear at s=L1.
    let d_accel = 7.45;
    for l1 in [d_accel + 1.5, d_accel, d_accel - 1.5] {
        let samples = run_samples(
            MAX_V,
            ACCEL,
            JERK,
            &[[0.0, 0.0, 0.0], [l1, 0.0, 0.0], [40.0, 0.0, 0.0]],
        );
        assert!(
            samples.iter().any(|p| p.v >= MAX_V - 1e-3),
            "L1={l1:.2}: run must reach cruise"
        );
        assert_no_step(&samples, EPS_A, &format!("collinear straights, L1={l1:.2}"));
        // The roll-off straddles the junction; v must stay monotone up to cruise
        // (no boundary sample snapping back to the nominal cruise speed).
        for w in samples.windows(2).take_while(|w| w[0].v < MAX_V - 1e-9) {
            assert!(
                w[1].v + 1e-6 >= w[0].v,
                "L1={l1:.2}: v regressed at s={:.4} ({:.5} -> {:.5}) — junction \
                 boundary spiked back to the ceiling",
                w[0].s,
                w[0].v,
                w[1].v
            );
        }
    }
}

#[test]
fn cruise_onset_low_jerk_rolls_off_below_a_max() {
    // v_max < a_max^2/(2j): jerk-up never reaches a_max before the ceiling, so
    // a_t rolls off from sqrt(2*j*v_max) < a_max. j=2000 => a_max^2/(2j)=250 >
    // v_max=100; the unbridged step would be sqrt(2*2000*100) ~ 632.
    // At j=2000 each roll-off spans ~18.6mm, so the straight must be long
    // enough (here 80mm) to hold a genuine cruise plateau between them.
    let low_jerk = 2000.0;
    let samples = run_samples(MAX_V, ACCEL, low_jerk, &[[0.0, 0.0, 0.0], [80.0, 0.0, 0.0]]);
    assert!(
        samples.iter().any(|p| p.v >= MAX_V - 1e-3),
        "fixture must reach cruise"
    );
    let onset_accel = (2.0 * low_jerk * MAX_V).sqrt();
    assert!(
        onset_accel < ACCEL,
        "low-jerk regime: onset accel {onset_accel:.1} must be below a_max={ACCEL}"
    );
    assert_no_step(&samples, EPS_A, "low-jerk straight");
    assert!(
        samples.iter().all(|p| p.a.abs() <= onset_accel + EPS_A),
        "low-jerk roll-off must peak near sqrt(2*j*v_max)={onset_accel:.1}, not a_max"
    );
}

#[test]
fn sub_cruise_triangular_straight_unchanged() {
    // A straight too short to reach cruise (peaks below v_max) never touches the
    // flat ceiling, so the ceiling bridge must not fire — the profile stays the
    // smooth jerk-up/down it already was (no spurious step, |a_t| <= a_max).
    let samples = run_samples(MAX_V, ACCEL, JERK, &[[0.0, 0.0, 0.0], [8.0, 0.0, 0.0]]);
    assert!(
        samples.iter().all(|p| p.v < MAX_V - 1e-3),
        "fixture must stay sub-cruise"
    );
    assert_no_step(&samples, EPS_A, "sub-cruise triangular straight");
    assert!(samples.iter().all(|p| p.a.abs() <= ACCEL + 1e-6));
}

#[test]
fn cruise_plateau_interior_not_bridged() {
    // The bridge fires only on the cruise entry/exit edges, never the interior
    // of a long plateau: every sample strictly inside the cruise band sits on
    // the rail (a_t == 0), with the roll-offs confined to the edges.
    let samples = run_samples(MAX_V, ACCEL, JERK, &[[0.0, 0.0, 0.0], [60.0, 0.0, 0.0]]);
    let interior: Vec<&Sample> = samples
        .iter()
        .filter(|p| p.v >= MAX_V - 1e-6 && p.s > 15.0 && p.s < 45.0)
        .collect();
    assert!(!interior.is_empty(), "expected a cruise plateau interior");
    assert!(
        interior.iter().all(|p| p.a.abs() <= EPS_A),
        "cruise plateau interior must stay on the a_t=0 rail (bridge fires only \
         at the entry/exit edges)"
    );
}

#[test]
fn short_would_overlap_move_is_one_smooth_sub_cruise_arc() {
    // A straight long enough for the base sweep to *claim* it reaches v_max, yet
    // shorter than the jerk-limited accel-up + brake-down distance (~31.6mm at
    // j4000), cannot reach the ceiling and stop within its length — the entry and
    // exit roll-offs would overlap. The analytic profile resolves this as a single
    // sub-cruise S-curve: it peaks strictly below v_max with no tangential-accel
    // steps. (The old planner instead touched the ceiling and kept two cruise-touch
    // steps; that fallback is gone.)
    let samples = run_samples(MAX_V, ACCEL, JERK, &[[0.0, 0.0, 0.0], [25.0, 0.0, 0.0]]);
    let peak = samples.iter().fold(0.0_f64, |m, p| m.max(p.v));
    assert!(
        peak < MAX_V - 1.0,
        "25mm cannot reach v_max jerk-feasibly and brake to rest; must peak \
         sub-cruise, got {peak}"
    );
    assert_no_step(&samples, EPS_A, "short would-overlap straight");
    assert!(samples.iter().all(|p| p.a.abs() <= ACCEL + 1e-6));
}

#[test]
fn cruise_onset_accel_within_envelope() {
    // The roll-off must not borrow accel past a_max anywhere on the straight.
    let samples = run_samples(MAX_V, ACCEL, JERK, &[[0.0, 0.0, 0.0], [40.0, 0.0, 0.0]]);
    assert!(
        accel_steps(&samples, EPS_A).is_empty()
            && samples.iter().all(|p| p.a.abs() <= ACCEL + 1e-6)
    );
    let (first, last) = (samples.first().unwrap(), samples.last().unwrap());
    assert_eq!((first.v, first.a), (0.0, 0.0));
    assert_eq!((last.v, last.a), (0.0, 0.0));
}
