//! A ratio step where the leaders turn around: the advance's turnaround
//! valley and the step's ramp ask the extruder for acceleration over the
//! same path, and together they must stay within its limit.

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
const EXTRUDER_MAX_ACCEL: f64 = 5000.0;
const ADVANCE_K: f64 = 0.5;

/// The quintic in local time over `[0, duration]` leaving `start` and
/// reaching `end`, each a position, velocity and acceleration.
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

/// One leg of the leaders along `y` at `x = 100`, moving towards `+y`
/// wherever the extruder lays `ratio` of the path they cover, from `e0`.
fn leg(y: &[f64], span: (f64, f64), ratio: f64, e0: f64) -> ContinuousSegment {
    let e: Vec<f64> = std::iter::once(e0)
        .chain(y[1..].iter().map(|c| ratio * c))
        .collect();
    let followers: Vec<FollowerDemand> = (ratio != 0.0)
        .then(|| FollowerDemand::constant(E_AXIS, ratio))
        .into_iter()
        .collect();
    ContinuousSegment {
        axes: Arc::from([
            spline(vec![100.0], span),
            spline(y.to_vec(), span),
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

fn eval(coeffs: &[f64], t: f64) -> [f64; 3] {
    let mut out = [0.0; 3];
    for (power, c) in coeffs.iter().enumerate() {
        let p = power as i32;
        out[0] += c * libm::pow(t, f64::from(p));
        if power >= 1 {
            out[1] += f64::from(p) * c * libm::pow(t, f64::from(p - 1));
        }
        if power >= 2 {
            out[2] += f64::from(p * (p - 1)) * c * libm::pow(t, f64::from(p - 2));
        }
    }
    out
}

/// The leaders brake at `decel` from `cruise` into a turnaround while the
/// extruder lays `ratio`, then travel back out of it, accelerating as hard,
/// with nothing laid: an extrusion ending in a travel that reverses over it
/// (`voron_first_layers` line 3654 into 3655 on the p1 bowden profile).
fn extrusion_into_a_reversing_travel(
    cruise: f64,
    decel: f64,
    ratio: f64,
) -> Vec<ContinuousSegment> {
    let mut segments = Vec::new();
    let (mut t, mut y, mut e) = (0.0, 100.0, 0.0);
    let rise_time = 0.15;
    let rise = quintic(
        [y, 0.0, 0.0],
        [y + 0.6 * cruise * rise_time, cruise, -decel],
        rise_time,
    );
    segments.push(leg(&rise, (t, t + rise_time), ratio, e));
    let [end_y, _, _] = eval(&rise, rise_time);
    e += ratio * (end_y - y);
    (t, y) = (t + rise_time, end_y);
    let brake_time = cruise / decel;
    let brake = vec![y, cruise, -0.5 * decel];
    segments.push(leg(&brake, (t, t + brake_time), ratio, e));
    let [turn_y, _, _] = eval(&brake, brake_time);
    e += ratio * (turn_y - y);
    (t, y) = (t + brake_time, turn_y);
    let back_time = 0.03;
    let back = vec![y, 0.0, -0.5 * decel];
    segments.push(leg(&back, (t, t + back_time), 0.0, e));
    let [back_y, back_v, _] = eval(&back, back_time);
    (t, y) = (t + back_time, back_y);
    let stop_time = 0.1;
    let stop = quintic(
        [y, back_v, -decel],
        [y + 0.5 * back_v * stop_time, 0.0, 0.0],
        stop_time,
    );
    segments.push(leg(&stop, (t, t + stop_time), 0.0, e));
    segments
}

fn project(track: &[ContinuousSegment]) -> Vec<ContinuousSegment> {
    let follower = CompiledChain::compile(&[trajectory::PostProcessorInstance::new(
        "pa",
        &trajectory::algos::LinearPressureAdvance,
        vec![ADVANCE_K],
    )])
    .expect("an advance compiles")
    .with_follower_limits(trajectory::FollowerLimits {
        max_velocity_mm_s: 120.0,
        max_accel_mm_s2: EXTRUDER_MAX_ACCEL,
    });
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
fn a_ratio_step_at_a_turnaround_keeps_the_extruder_within_its_limit() {
    let track = extrusion_into_a_reversing_travel(40.0, 861.19, 0.033_645);
    let committed = project(&track);
    let mut worst = (0.0_f64, 0.0);
    for segment in &committed {
        let samples = 4000;
        for k in 0..=samples {
            let t = segment.t_start
                + (segment.t_end - segment.t_start) * f64::from(k) / f64::from(samples);
            let accel = segment.axes[E_AXIS]
                .eval_pva(t)
                .unwrap_or_else(|error| panic!("extruder at t={t}: {error:?}"))
                .acceleration;
            if accel.abs() > worst.0 {
                worst = (accel.abs(), t);
            }
        }
    }
    assert!(
        worst.0 <= EXTRUDER_MAX_ACCEL + FIT_TOL.accel_mm_s2,
        "the extruder accelerates at {} mm/s² at t={}, past its {EXTRUDER_MAX_ACCEL} mm/s² limit",
        worst.0,
        worst.1
    );
}
