//! A ratio step a kilometre into a print: the odometer reads the path in
//! steps of its own ulp there, and the ratio ramp's slope turns one of them
//! into a velocity change the advance must not read as a step (#1021).

use std::sync::Arc;

use geometry::FollowerDemand;
use nurbs::bezier::{BezierPiece, bezier_pieces_to_nurbs};
use trajectory::{AxisChainSet, CompiledChain, ContinuousAxis, ContinuousSegment};

use super::{ProjectionTiming, ProjectionWindow, project_followers};
use crate::lowering::FitTol;
use crate::shaper::Lookahead;
use crate::worker_pool::WorkerPool;

const E_AXIS: usize = 3;
const FIT_TOL: FitTol = FitTol::absolute(1e-4, 50.0);
const ADVANCE_K: f64 = 0.02;
const TRAVEL_MM_S: f64 = 1000.0;
/// Past 2^20 mm the odometer's ulp is 2.3e-10 mm.
const TRAVEL_S: f64 = 1050.0;
const CRAWL_MM_S: f64 = 1.0;
const STEP_RATIO: f64 = 0.1;

fn quintic(start: [f64; 3], end: [f64; 3], duration: f64) -> Vec<f64> {
    let [p0, v0, a0] = start;
    let [p1, v1, a1] = end;
    let t = duration;
    let (t2, t3, t4, t5) = (t * t, t * t * t, t * t * t * t, t * t * t * t * t);
    let gap = p1 - p0 - v0 * t - 0.5 * a0 * t2;
    let speed_gap = v1 - v0 - a0 * t;
    let accel_gap = a1 - a0;
    vec![
        p0,
        v0,
        0.5 * a0,
        (10.0 * gap - 4.0 * speed_gap * t + 0.5 * accel_gap * t2) / t3,
        (-15.0 * gap + 7.0 * speed_gap * t - accel_gap * t2) / t4,
        (6.0 * gap - 3.0 * speed_gap * t + 0.5 * accel_gap * t2) / t5,
    ]
}

fn spline(coeffs: Vec<f64>, (t0, t1): (f64, f64)) -> ContinuousAxis {
    ContinuousAxis::Spline(Arc::new(bezier_pieces_to_nurbs(&[BezierPiece {
        u_start: t0,
        u_end: t1,
        coeffs,
    }])))
}

fn end_of(coeffs: &[f64], t: f64) -> f64 {
    coeffs.iter().rev().fold(0.0, |acc, c| acc * t + c)
}

/// The leaders along `y`, `moved` in local time from `y0`, while the
/// extruder lays `ratio` of their path from `e0`.
fn leg(moved: &[f64], span: (f64, f64), ratio: f64, (y0, e0): (f64, f64)) -> ContinuousSegment {
    let mut y = moved.to_vec();
    y[0] += y0;
    let mut e: Vec<f64> = moved.iter().map(|c| ratio * c).collect();
    e[0] = e0;
    let followers: Vec<FollowerDemand> = (ratio != 0.0)
        .then(|| FollowerDemand::constant(E_AXIS, ratio))
        .into_iter()
        .collect();
    ContinuousSegment {
        axes: Arc::from([
            spline(vec![100.0], span),
            spline(y, span),
            spline(vec![0.2], span),
            spline(e, span),
        ]),
        followers: Arc::from(followers),
        spatial_path: true,
        t_start: span.0,
        t_end: span.1,
        motor_mask: 0,
        source_line: 1,
        rest_at_end: false,
    }
}

/// A travel of about 1.05e6 mm, a brake into a crawl, and a ratio step
/// from travel to extrusion at the crawl, then a stop.
fn step_after_a_long_travel() -> Vec<ContinuousSegment> {
    let (rise_time, brake_time, crawl_time, stop_time) = (1.0, 2.0, 0.05, 0.15);
    let legs = [
        (
            quintic(
                [0.0, 0.0, 0.0],
                [0.5 * TRAVEL_MM_S * rise_time, TRAVEL_MM_S, 0.0],
                rise_time,
            ),
            rise_time,
            0.0,
        ),
        (vec![0.0, TRAVEL_MM_S], TRAVEL_S, 0.0),
        (
            quintic(
                [0.0, TRAVEL_MM_S, 0.0],
                [0.5 * (TRAVEL_MM_S + CRAWL_MM_S) * brake_time, CRAWL_MM_S, 0.0],
                brake_time,
            ),
            brake_time,
            0.0,
        ),
        (vec![0.0, CRAWL_MM_S], crawl_time, 0.0),
        (vec![0.0, CRAWL_MM_S], crawl_time, STEP_RATIO),
        (
            quintic(
                [0.0, CRAWL_MM_S, 0.0],
                [0.5 * CRAWL_MM_S * stop_time, 0.0, 0.0],
                stop_time,
            ),
            stop_time,
            STEP_RATIO,
        ),
    ];
    let (mut t, mut y, mut e) = (0.0, 100.0, 0.0);
    legs.into_iter()
        .map(|(moved, duration, ratio)| {
            let segment = leg(&moved, (t, t + duration), ratio, (y, e));
            let travel = end_of(&moved, duration) - moved[0];
            (t, y, e) = (t + duration, y + travel, e + ratio * travel);
            segment
        })
        .collect()
}

#[test]
fn a_ratio_step_a_kilometre_into_the_print_reads_no_advance_step() {
    let follower = CompiledChain::compile(&[trajectory::PostProcessorInstance::new(
        "pa",
        &trajectory::algos::LinearPressureAdvance,
        vec![ADVANCE_K],
    )])
    .expect("an advance compiles");
    let chains = AxisChainSet {
        chains: vec![
            CompiledChain::default(),
            CompiledChain::default(),
            CompiledChain::default(),
            follower,
        ],
        followers: vec![(E_AXIS, vec![0, 1, 2])],
    };
    let track = step_after_a_long_travel();
    let pool = WorkerPool::new();
    let mut committed = track.clone();
    project_followers(
        ProjectionWindow {
            base: &track,
            frontier: &track,
            chains: &chains,
            fit_tol: FIT_TOL,
            lookahead: Lookahead::Forced,
            pool: &pool,
        },
        &mut committed,
        &mut Vec::new(),
        &mut ProjectionTiming::default(),
    )
    .unwrap_or_else(|error| panic!("the projection failed: {error:?}"));
}
