//! [`may_open_run`] says "no run" only where no continuation lets the
//! fitter's own candidate test open one with the pair as its first facets.

use proptest::prelude::*;
use proptest::test_runner::FileFailurePersistence;

use super::*;
use crate::frontend::{MoveContext, line_move};
use crate::segment::SourceRange;

const E_AXIS: usize = 3;
const ACCEL: f64 = 10_000.0;
const FEED: f64 = 100.0;
const CONTINUATIONS: usize = 48;

fn line(start: [f64; 3], end: [f64; 3], ratio: f64, deviation: f64) -> Move {
    let context = MoveContext {
        extruder_axis: E_AXIS,
        feedrate_mm_s: FEED,
        limits: VelocityLimits::try_new(
            1000.0,
            ACCEL,
            crate::CornerBudget::OverFloor(deviation),
            100_000.0,
        )
        .expect("the limits are valid"),
        source: SourceRange {
            start_line: 1,
            end_line: 1,
        },
    };
    let length = crate::vec3::dist(start, end);
    line_move(start, end, ratio * length, context).expect("the line moves")
}

fn heading(angle: f64) -> [f64; 3] {
    [libm::cos(angle), libm::sin(angle), 0.0]
}

fn step(from: [f64; 3], angle: f64, length: f64) -> [f64; 3] {
    let h = heading(angle);
    [from[0] + length * h[0], from[1] + length * h[1], from[2]]
}

/// The third facet a circle through the pair's three vertices would
/// continue with, at the second facet's chord, and spread around it.
fn continuations(
    p0: [f64; 3],
    p1: [f64; 3],
    p2: [f64; 3],
    turns: &[f64],
    lengths: &[f64],
) -> Vec<[f64; 3]> {
    let out = crate::vec3::sub(p2, p1);
    let out_angle = libm::atan2(out[1], out[0]);
    let in_vec = crate::vec3::sub(p1, p0);
    let turn = out_angle - libm::atan2(in_vec[1], in_vec[0]);
    let mut ends = vec![step(p2, out_angle + turn, crate::vec3::norm(out))];
    for (&turn, &length) in turns.iter().zip(lengths) {
        ends.push(step(p2, out_angle + turn, length));
    }
    ends
}

fn config() -> CornerFitConfig {
    CornerFitConfig::default()
}

proptest! {
    #![proptest_config(ProptestConfig {
        cases: 2048,
        failure_persistence: Some(Box::new(FileFailurePersistence::Direct(
            "proptest-regressions/may_open_run.txt",
        ))),
        ..ProptestConfig::default()
    })]

    /// Whatever third and fourth facet follow, and whatever the second
    /// facet merges into first, a pair it rules out never passes the
    /// fitter's candidate test as a run's first two facets.
    #[test]
    fn a_pair_it_rules_out_never_opens_a_run(
        a in 0.05_f64..400.0,
        b in 0.05_f64..400.0,
        theta in 1e-3_f64..std::f64::consts::PI,
        sign in prop::sample::select(vec![-1.0_f64, 1.0]),
        deviation in 0.002_f64..0.3,
        ratio in prop::sample::select(vec![0.0_f64, 0.03]),
        turns in prop::collection::vec(-0.2_f64..0.2, CONTINUATIONS),
        lengths in prop::collection::vec(0.01_f64..400.0, CONTINUATIONS),
        bend in -1e-3_f64..1e-3,
        reach in 0.05_f64..400.0,
    ) {
        let p0 = [0.0, 0.0, 0.2];
        let p1 = step(p0, 0.0, a);
        let p2 = step(p1, sign * theta, b);
        let first = line(p0, p1, ratio, deviation);
        let second = line(p1, p2, ratio, deviation);
        let cfg = config();
        prop_assume!(arc_candidate_fits(&[first.clone(), second.clone()], cfg));
        prop_assume!(!may_open_run(&first, &second, cfg));

        let beyond = line(p2, step(p2, sign * theta + bend, reach), ratio, deviation);
        let merged = merge_collinear_lines(&second, &beyond, &[], cfg);
        let seconds = std::iter::once(second.clone()).chain(merged);
        for second in seconds {
            let end = crate::path::lowering::PositionProfile::point_at(
                line_of(&second).expect("a line"),
                line_of(&second).expect("a line").s_len(),
            );
            for third_end in continuations(p0, p1, end, &turns, &lengths) {
                if crate::vec3::dist(end, third_end) <= 1e-9 {
                    continue;
                }
                let third = line(end, third_end, ratio, deviation);
                prop_assert!(
                    !arc_candidate_fits(&[first.clone(), second.clone(), third.clone()], cfg),
                    "a {a} b {b} theta {theta} deviation {deviation}: ruled out, yet opens a run"
                );
            }
        }
    }

    /// Facets of one circle within its tolerance: the pair always may.
    #[test]
    fn facets_of_a_circle_may_open_a_run(
        radius in 1.0_f64..100_000.0,
        deviation in 0.002_f64..0.3,
        sag_share in 0.05_f64..1.0,
    ) {
        let sagitta = 0.5 * sag_share * deviation;
        prop_assume!(sagitta < radius);
        let sweep = 2.0 * libm::acos(1.0 - sagitta / radius);
        let chord = 2.0 * radius * libm::sin(0.5 * sweep);
        let at = |k: f64| [radius * libm::sin(k * sweep), radius * (1.0 - libm::cos(k * sweep)), 0.0];
        let (p0, p1, p2, p3) = (at(0.0), at(1.0), at(2.0), at(3.0));
        prop_assume!(chord > 0.01);
        let facets = [
            line(p0, p1, 0.03, deviation),
            line(p1, p2, 0.03, deviation),
            line(p2, p3, 0.03, deviation),
        ];
        let cfg = config();
        prop_assume!(arc_candidate_fits(&facets, cfg));
        prop_assert!(may_open_run(&facets[0], &facets[1], cfg));
    }
}

/// The bench zigzag's 0.4 mm step-over at the 0.05 mm a shaped budget
/// fits arcs to; at a tolerance near the step-over itself the two strokes
/// may lie on one annulus and the bound rightly cannot rule them out.
#[test]
fn a_zigzag_near_reversal_cannot_open_a_run() {
    let deviation = 0.05;
    let first = line([60.0, 20.0, 0.3], [140.0, 20.0, 0.3], 0.00375, deviation);
    let second = line([140.0, 20.0, 0.3], [60.0, 20.4, 0.3], 0.00375, deviation);
    assert!(arc_candidate_fits(
        &[first.clone(), second.clone()],
        config()
    ));
    assert!(!may_open_run(&first, &second, config()));
}

#[test]
fn a_square_corner_of_long_lines_cannot_open_a_run() {
    let first = line([0.0, 0.0, 0.2], [100.0, 0.0, 0.2], 0.03, 0.05);
    let second = line([100.0, 0.0, 0.2], [100.0, 100.0, 0.2], 0.03, 0.05);
    assert!(!may_open_run(&first, &second, config()));
}
