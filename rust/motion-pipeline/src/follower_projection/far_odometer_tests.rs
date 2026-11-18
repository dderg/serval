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
const TRAVEL_MM_S: f64 = 100_000.0;
/// Past 2^20 mm the odometer's ulp is 2.3e-10 mm.
const TRAVEL_S: f64 = 10.5;
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

/// A travel of about 1.2e6 mm, a brake into a crawl, and a ratio step
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
                [
                    0.5 * (TRAVEL_MM_S + CRAWL_MM_S) * brake_time,
                    CRAWL_MM_S,
                    0.0,
                ],
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

fn project(track: &[ContinuousSegment]) -> Vec<ContinuousSegment> {
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
    let pool = WorkerPool::new();
    let mut committed = track.to_vec();
    project_followers(
        ProjectionWindow {
            base: track,
            frontier: track,
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
    committed
}

#[test]
fn a_ratio_step_a_kilometre_into_the_print_reads_no_advance_step() {
    project(&step_after_a_long_travel());
}

const OUT_AND_BACK_S: f64 = 1.0;
const PRINT_PATH_OFFSET_MM: f64 = 1e5;
const PRINT_SPEED_MM_S: f64 = 20.0;

/// A travel out along `y` and back to where it left, `path_mm` long in
/// all and always as long in time, then an extrusion that speeds up,
/// steps its ratio, crawls and stops. Only the odometer reading at the
/// extrusion depends on `path_mm`.
fn extrusion_after_an_out_and_back(path_mm: f64) -> Vec<ContinuousSegment> {
    let half = 0.5 * path_mm;
    let (rise, cruise, crawl, stop) = (0.05, 0.1, 0.05, 0.15);
    let legs = [
        (quintic([0.0, 0.0, 0.0], [half, 0.0, 0.0], OUT_AND_BACK_S), OUT_AND_BACK_S, 0.0),
        (quintic([0.0, 0.0, 0.0], [-half, 0.0, 0.0], OUT_AND_BACK_S), OUT_AND_BACK_S, 0.0),
        (
            quintic([0.0, 0.0, 0.0], [0.5 * PRINT_SPEED_MM_S * rise, PRINT_SPEED_MM_S, 0.0], rise),
            rise,
            0.04,
        ),
        (vec![0.0, PRINT_SPEED_MM_S], cruise, 0.06),
        (
            quintic(
                [0.0, PRINT_SPEED_MM_S, 0.0],
                [0.5 * (PRINT_SPEED_MM_S + CRAWL_MM_S) * crawl, CRAWL_MM_S, 0.0],
                crawl,
            ),
            crawl,
            0.06,
        ),
        (
            quintic([0.0, CRAWL_MM_S, 0.0], [0.5 * CRAWL_MM_S * stop, 0.0, 0.0], stop),
            stop,
            0.06,
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

/// How far a print has run must not change what it extrudes next: the
/// same extrusion after a 2 mm and after a 100 m out-and-back travel lays
/// the same extruder track, sample for sample.
#[test]
fn the_extruder_track_does_not_depend_on_how_far_the_print_has_run() {
    let near = project(&extrusion_after_an_out_and_back(2.0));
    let far = project(&extrusion_after_an_out_and_back(PRINT_PATH_OFFSET_MM));
    let samples = 64;
    let mut worst = (0.0_f64, 0.0_f64, 0.0);
    for (near, far) in near.iter().zip(&far).skip(2) {
        for k in 0..=samples {
            let t = near.t_start + (near.t_end - near.t_start) * f64::from(k) / f64::from(samples);
            let pva = |segment: &ContinuousSegment| {
                segment.axes[E_AXIS]
                    .eval_pva(t)
                    .unwrap_or_else(|error| panic!("extruder at t={t}: {error:?}"))
            };
            let (a, b) = (pva(near), pva(far));
            worst.0 = worst.0.max((a.position - b.position).abs());
            worst.1 = worst.1.max((a.velocity - b.velocity).abs());
            if (a.velocity - b.velocity).abs() == worst.1 {
                worst.2 = t;
            }
        }
    }
    assert!(
        worst.0 == 0.0 && worst.1 == 0.0,
        "100 m into the print the extruder track moves by up to {} mm and {} mm/s (t={})",
        worst.0,
        worst.1,
        worst.2
    );
}

/// The two probes of the seed the Benchy on `printer.cfg` without
/// `shaper_y` was refused at (#1021, t = 582.10261963530729 on the
/// minimized file, commit 81a5da543): an ulp of odometer apart past
/// 2^14 mm, on a crawl valley side whose weight climbs 5.8e5 per mm. The
/// read moves 1.5e-9 mm/s between them; the in-segment check allowed only
/// the ratio times the leaders' speed residual (3.8e-12) and 1e-9 of
/// rounding, the join check also the misalignment.
#[test]
fn the_benchy_seed_probes_explain_their_read_change() {
    let before = super::ReadProbe {
        s: 16_384.195_945_522_12,
        read_slope: 0.766_702_981_280_104_7 * 0.252_672_397_177_862_94
            + 0.038_814_097_549_786_424 * 579_971.277_588_857_2,
        leader_speed: 1.839_700_841_300_515e-2,
        ratio: 0.038_814_097_549_786_424,
    };
    let after = super::ReadProbe {
        s: 16_384.195_945_522_122,
        read_slope: 0.038_814_097_550_570_574 * 579_970.185_970_952_6,
        leader_speed: 1.839_700_851_062_980_2e-2,
        ratio: 0.038_814_097_550_570_574,
    };
    let step = super::AdvanceStep {
        t: 582.102_619_635_307_3,
        velocity_before: 1.804_240_804_837_163e-4,
        velocity_after: 1.804_255_880_620_719e-4,
        step_mm: 4.522_735_066_724_714_5e-11,
    };
    assert!(
        before.explains_read_step(&after, &step),
        "a read moving along its own slope over the ulp between the probes is no step"
    );
}
