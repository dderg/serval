//! Property fuzz of the follower projection against an exact oracle: random
//! leader tracks — strokes along one line with ratio steps, reversals through
//! rest, holds and extrude-only retracts and primes — whose shaped frontier
//! re-times each stroke over the same path. The extruder must lay `∫ r ds`
//! over the path the shaped leaders travel wherever the ratio ramp reads a
//! constant ratio and wherever the leaders dwell, keep its velocity continuous across
//! every seam, weld every seam, and commit the same track streamed a few
//! segments at a time as all at once.

use std::sync::Arc;

use geometry::FollowerDemand;
use nurbs::bezier::{BezierPiece, bezier_pieces_to_nurbs};
use proptest::prelude::*;
use proptest::test_runner::FileFailurePersistence;
use trajectory::{AxisChainSet, CompiledChain, ContinuousAxis, ContinuousSegment};

use super::{FollowerState, ProjectionTiming, ProjectionWindow, project_followers};
use crate::lowering::FitTol;
use crate::shaper::Lookahead;
use crate::worker_pool::WorkerPool;

const E_AXIS: usize = 3;
const FIT_TOL: FitTol = FitTol::absolute(1e-4, 50.0);
const SAMPLES_PER_SEGMENT: usize = 16;
const ORACLE_SLACK_MM: f64 = 1e-9;

fn config() -> ProptestConfig {
    ProptestConfig {
        cases: std::env::var("PROPTEST_CASES")
            .map(|value| value.parse().expect("PROPTEST_CASES must be an integer"))
            .unwrap_or(64),
        failure_persistence: Some(Box::new(FileFailurePersistence::Direct(
            "proptest-regressions/projection_fuzz.txt",
        ))),
        ..ProptestConfig::default()
    }
}

#[derive(Debug, Clone, Copy)]
enum Step {
    Stroke {
        reverse: bool,
        length: f64,
        pace: f64,
        end_speed: f64,
        shaped_speed_scale: f64,
        ratio: f64,
    },
    Hold {
        duration: f64,
    },
    ExtrudeOnly {
        delta: f64,
        duration: f64,
    },
}

impl Step {
    fn is_stroke(self) -> bool {
        matches!(self, Step::Stroke { .. })
    }
}

fn step_strategy() -> impl Strategy<Value = Step> {
    let ratio = prop_oneof![1 => Just(0.0), 4 => 0.01..0.1_f64];
    prop_oneof![
        8 => (any::<bool>(), 0.05..20.0_f64, 1.0..200.0_f64, 0.0..300.0_f64, 0.5..1.5_f64, ratio)
            .prop_map(|(reverse, length, pace, end_speed, shaped_speed_scale, ratio)| {
                Step::Stroke { reverse, length, pace, end_speed, shaped_speed_scale, ratio }
            }),
        1 => (0.001..0.2_f64).prop_map(|duration| Step::Hold { duration }),
        1 => (-1.5..1.5_f64, 0.01..0.2_f64)
            .prop_map(|(delta, duration)| Step::ExtrudeOnly { delta, duration }),
    ]
}

/// One segment as the oracle knows it: its shaped leader distance along
/// the line as a cubic in local time from where along the travelled path
/// it starts, its ratio, or the extrude-only cubic it moves the extruder
/// by.
#[derive(Debug, Clone)]
enum Truth {
    Stroke {
        shaped: [f64; 4],
        ratio: f64,
        path_start_mm: f64,
    },
    Hold,
    ExtrudeOnly {
        motion: [f64; 4],
    },
}

/// The track, and where along the travelled path its ratio steps — every
/// boundary between two strokes of different ratios, rest or not — and
/// the fastest the shaped leaders travel it.
#[derive(Debug, Clone)]
struct Track {
    base: Vec<ContinuousSegment>,
    frontier: Vec<ContinuousSegment>,
    truth: Vec<Truth>,
    e_at_start: Vec<f64>,
    ratio_steps_mm: Vec<f64>,
    fastest_shaped_mm_s: f64,
}

impl Track {
    /// How far along the path the ratio ramp of a follower without limits
    /// reaches either side of a point: half the path the fastest leaders
    /// cover in the shortest ramp.
    fn ramp_reach_mm(&self) -> f64 {
        0.5 * self.fastest_shaped_mm_s * geometry::RATIO_RAMP_MIN_TIME_S
    }

    /// Whether the design holds at local time `dt` into segment `i`: the
    /// leaders dwell there (a hold or an extrude-only move), or the ramp
    /// reads one ratio across its reach. A stroke that stops for an instant
    /// is no dwell: the ramp spreads a ratio step across the stop.
    fn ramp_reads_one_ratio(&self, i: usize, dt: f64) -> bool {
        let Truth::Stroke {
            shaped,
            path_start_mm,
            ..
        } = &self.truth[i]
        else {
            return true;
        };
        let s = path_start_mm + eval(shaped, dt);
        let reach = self.ramp_reach_mm();
        self.ratio_steps_mm
            .iter()
            .all(|step| (step - s).abs() > reach)
    }
}

/// The fastest a cubic distance `coeffs` moves over `[0, duration]`.
fn peak_speed(coeffs: &[f64; 4], duration: f64) -> f64 {
    let vertex = -coeffs[2] / (3.0 * coeffs[3]);
    [0.0, duration, vertex]
        .into_iter()
        .filter(|dt| (0.0..=duration).contains(dt))
        .map(|dt| eval_d(coeffs, dt).abs())
        .fold(0.0, f64::max)
}

fn hermite(length: f64, (v0, v1): (f64, f64), duration: f64) -> [f64; 4] {
    let a2 = (3.0 * length / duration - 2.0 * v0 - v1) / duration;
    let a3 = (v0 + v1 - 2.0 * length / duration) / (duration * duration);
    [0.0, v0, a2, a3]
}

fn eval(coeffs: &[f64], dt: f64) -> f64 {
    coeffs.iter().rev().fold(0.0, |acc, c| acc * dt + c)
}

fn eval_d(coeffs: &[f64], dt: f64) -> f64 {
    coeffs
        .iter()
        .enumerate()
        .skip(1)
        .rev()
        .fold(0.0, |acc, (k, c)| acc * dt + k as f64 * c)
}

fn spline(coeffs: Vec<f64>, (t0, t1): (f64, f64)) -> ContinuousAxis {
    ContinuousAxis::Spline(Arc::new(bezier_pieces_to_nurbs(&[BezierPiece {
        u_start: t0,
        u_end: t1,
        coeffs,
    }])))
}

fn segment(
    axes: [Vec<f64>; 4],
    span: (f64, f64),
    followers: &[FollowerDemand],
    spatial_path: bool,
) -> ContinuousSegment {
    let [x, y, z, e] = axes;
    ContinuousSegment {
        axes: Arc::from([
            spline(x, span),
            spline(y, span),
            spline(z, span),
            spline(e, span),
        ]),
        followers: Arc::from(followers),
        spatial_path,
        t_start: span.0,
        t_end: span.1,
        motor_mask: 0,
        source_line: 1,
        rest_at_end: false,
    }
}

/// A seam can carry speed only between two strokes along the same sense;
/// every other boundary is a rest.
fn rest_where_required(steps: &[Step]) -> Vec<Step> {
    let mut sense = false;
    let senses: Vec<bool> = steps
        .iter()
        .map(|step| {
            if let Step::Stroke { reverse, .. } = step {
                sense ^= reverse;
            }
            sense
        })
        .collect();
    steps
        .iter()
        .enumerate()
        .map(|(i, &step)| match step {
            Step::Stroke {
                reverse,
                length,
                pace,
                end_speed,
                shaped_speed_scale,
                ratio,
            } => {
                let carries = steps
                    .get(i + 1)
                    .is_some_and(|next| next.is_stroke() && senses[i + 1] == senses[i]);
                Step::Stroke {
                    reverse,
                    length,
                    pace,
                    end_speed: if carries { end_speed } else { 0.0 },
                    shaped_speed_scale,
                    ratio,
                }
            }
            other => other,
        })
        .collect()
}

/// Each stroke's duration, from its raw entry and exit speeds, and the
/// shaped speed at its exit: the raw speed rescaled, capped where either
/// stroke meeting there would overshoot its own length.
fn timing(steps: &[Step]) -> Vec<(f64, f64)> {
    let mut entry = 0.0_f64;
    let durations: Vec<f64> = steps
        .iter()
        .map(|step| match *step {
            Step::Stroke {
                length,
                pace,
                end_speed,
                ..
            } => {
                let duration = length / (entry.max(end_speed) / 3.0 + pace);
                entry = end_speed;
                duration
            }
            Step::Hold { duration } | Step::ExtrudeOnly { duration, .. } => {
                entry = 0.0;
                duration
            }
        })
        .collect();
    let monotone_cap = |i: usize| match steps.get(i) {
        Some(Step::Stroke { length, .. }) => 3.0 * length / durations[i],
        _ => 0.0,
    };
    steps
        .iter()
        .enumerate()
        .map(|(i, step)| match *step {
            Step::Stroke {
                end_speed,
                shaped_speed_scale,
                ..
            } => (
                durations[i],
                (end_speed * shaped_speed_scale)
                    .min(monotone_cap(i))
                    .min(monotone_cap(i + 1)),
            ),
            _ => (durations[i], 0.0),
        })
        .collect()
}

fn build_track(heading: f64, steps: &[Step]) -> Track {
    let steps = rest_where_required(steps);
    let timing = timing(&steps);
    let direction = [libm::cos(heading), libm::sin(heading)];
    let (mut t, mut along, mut e, mut sense) = (0.0, 0.0, 0.0, 1.0_f64);
    let (mut raw_speed, mut shaped_speed) = (0.0_f64, 0.0_f64);
    let (mut travelled, mut last_ratio) = (0.0_f64, None);
    let mut track = Track {
        base: Vec::new(),
        frontier: Vec::new(),
        truth: Vec::new(),
        e_at_start: Vec::new(),
        ratio_steps_mm: Vec::new(),
        fastest_shaped_mm_s: 0.0,
    };
    let leaders_at = |along: f64, local: &[f64], sense: f64| -> [Vec<f64>; 3] {
        let line = |axis: usize| {
            let mut coeffs: Vec<f64> = local.iter().map(|c| sense * direction[axis] * c).collect();
            coeffs[0] += 100.0 + along * direction[axis];
            coeffs
        };
        [line(0), line(1), vec![0.2]]
    };
    for (step, &(duration, shaped_end)) in steps.iter().zip(&timing) {
        track.e_at_start.push(e);
        match *step {
            Step::Stroke {
                reverse,
                length,
                end_speed,
                ratio,
                ..
            } => {
                if reverse {
                    sense = -sense;
                }
                let raw = hermite(length, (raw_speed, end_speed), duration);
                let shaped = hermite(length, (shaped_speed, shaped_end), duration);
                let span = (t, t + duration);
                let followers: Vec<FollowerDemand> = (ratio > 0.0)
                    .then(|| FollowerDemand::constant(E_AXIS, ratio))
                    .into_iter()
                    .collect();
                let e_line = |local: &[f64]| {
                    let mut coeffs: Vec<f64> = local.iter().map(|c| ratio * c).collect();
                    coeffs[0] += e;
                    coeffs
                };
                let [rx, ry, rz] = leaders_at(along, &raw, sense);
                let [sx, sy, sz] = leaders_at(along, &shaped, sense);
                track
                    .base
                    .push(segment([rx, ry, rz, e_line(&raw)], span, &followers, true));
                track
                    .frontier
                    .push(segment([sx, sy, sz, e_line(&raw)], span, &followers, true));
                track.truth.push(Truth::Stroke {
                    shaped,
                    ratio,
                    path_start_mm: travelled,
                });
                if last_ratio.is_some_and(|last| last != ratio) {
                    track.ratio_steps_mm.push(travelled);
                }
                last_ratio = Some(ratio);
                track.fastest_shaped_mm_s =
                    track.fastest_shaped_mm_s.max(peak_speed(&shaped, duration));
                travelled += length;
                along += sense * length;
                e += ratio * length;
                t = span.1;
                raw_speed = end_speed;
                shaped_speed = shaped_end;
            }
            Step::Hold { .. } => {
                let span = (t, t + duration);
                let [x, y, z] = leaders_at(along, &[0.0], 1.0);
                let hold = segment([x, y, z, vec![e]], span, &[], true);
                track.base.push(hold.clone());
                track.frontier.push(hold);
                track.truth.push(Truth::Hold);
                t = span.1;
            }
            Step::ExtrudeOnly { delta, .. } => {
                let span = (t, t + duration);
                let motion = hermite(delta, (0.0, 0.0), duration);
                let mut e_coeffs = motion.to_vec();
                e_coeffs[0] += e;
                let [x, y, z] = leaders_at(along, &[0.0], 1.0);
                let move_e = segment([x, y, z, e_coeffs], span, &[], false);
                track.base.push(move_e.clone());
                track.frontier.push(move_e);
                track.truth.push(Truth::ExtrudeOnly { motion });
                e += delta;
                t = span.1;
            }
        }
    }
    track
}

/// The extruder position and velocity the design lays at local time `dt`
/// into segment `i`: the segment's ratio over the shaped leaders' distance,
/// or the extrude-only motion itself.
fn oracle(track: &Track, i: usize, dt: f64) -> (f64, f64) {
    let e0 = track.e_at_start[i];
    match &track.truth[i] {
        Truth::Stroke { shaped, ratio, .. } => {
            (e0 + ratio * eval(shaped, dt), ratio * eval_d(shaped, dt))
        }
        Truth::Hold => (e0, 0.0),
        Truth::ExtrudeOnly { motion } => (e0 + eval(motion, dt), eval_d(motion, dt)),
    }
}

fn chains(follower: CompiledChain) -> AxisChainSet {
    AxisChainSet {
        chains: vec![
            CompiledChain::default(),
            CompiledChain::default(),
            CompiledChain::default(),
            follower,
        ],
        followers: vec![(E_AXIS, vec![0, 1, 2])],
    }
}

fn project_all_at_once(track: &Track, chains: &AxisChainSet) -> Vec<ContinuousSegment> {
    let pool = WorkerPool::new();
    let mut committed = track.frontier.clone();
    project_followers(
        ProjectionWindow {
            base: &track.base,
            frontier: &track.frontier,
            chains,
            fit_tol: FIT_TOL,
            lookahead: Lookahead::Forced,
            pool: &pool,
        },
        &mut committed,
        &mut Vec::new(),
        &mut ProjectionTiming::default(),
    )
    .unwrap_or_else(|error| panic!("one-shot projection failed: {error:?}"));
    committed
}

/// Feed the track the way the shaper does: a few more segments each call,
/// asking to commit every segment whose kernel window the buffered track
/// already covers, dropping what was committed, and forcing the rest out at
/// the end.
fn project_streamed(
    track: &Track,
    chains: &AxisChainSet,
    batch_sizes: &[usize],
) -> Vec<ContinuousSegment> {
    let reach_ahead = chains.chains[E_AXIS]
        .kernel()
        .map_or(0.0, |kernel| (-kernel.support().0).max(0.0));
    let pool = WorkerPool::new();
    let mut states: Vec<FollowerState> = Vec::new();
    let mut timing = ProjectionTiming::default();
    let mut committed = Vec::new();
    let (mut start, mut end) = (0, 0);
    let mut batches = batch_sizes.iter().cycle();
    loop {
        let forced = end == track.base.len();
        if !forced {
            end = (end + batches.next().expect("cycled")).min(track.base.len());
        }
        let base = &track.base[start..end];
        let frontier = &track.frontier[start..end];
        let covered_through = frontier.last().map_or(f64::NEG_INFINITY, |seg| seg.t_end);
        let mut out: Vec<ContinuousSegment> = if forced {
            frontier.to_vec()
        } else {
            frontier
                .iter()
                .take_while(|seg| seg.t_end + reach_ahead <= covered_through)
                .cloned()
                .collect()
        };
        project_followers(
            ProjectionWindow {
                base,
                frontier,
                chains,
                fit_tol: FIT_TOL,
                lookahead: if forced {
                    Lookahead::Forced
                } else {
                    Lookahead::Required
                },
                pool: &pool,
            },
            &mut out,
            &mut states,
            &mut timing,
        )
        .unwrap_or_else(|error| panic!("streamed projection failed at {start}..{end}: {error:?}"));
        start += out.len();
        committed.extend(out);
        if forced {
            assert_eq!(
                start,
                track.base.len(),
                "a forced projection commits everything"
            );
            return committed;
        }
    }
}

fn follower_pv(seg: &ContinuousSegment, t: f64) -> (f64, f64) {
    let pva = seg.axes[E_AXIS]
        .eval_pva(t)
        .unwrap_or_else(|error| panic!("follower at t={t}: {error:?}"));
    (pva.position, pva.velocity)
}

/// The extruder's fit tolerance on one segment: the path deviation scaled
/// by that segment's own ratio, the whole deviation where it extrudes
/// nothing — what the projection refits each committed segment to.
fn segment_tol_mm(seg: &ContinuousSegment) -> f64 {
    FIT_TOL.pos_mm * crate::lowering::follower_tol_scale(&seg.followers, E_AXIS) + ORACLE_SLACK_MM
}

fn local_samples(seg: &ContinuousSegment) -> impl Iterator<Item = f64> + '_ {
    (0..=SAMPLES_PER_SEGMENT)
        .map(move |k| (seg.t_end - seg.t_start) * k as f64 / SAMPLES_PER_SEGMENT as f64)
}

fn assert_lays_the_oracle(
    track: &Track,
    committed: &[ContinuousSegment],
) -> Result<(), TestCaseError> {
    for (i, seg) in committed.iter().enumerate() {
        let tol = segment_tol_mm(seg);
        for dt in local_samples(seg).filter(|&dt| track.ramp_reads_one_ratio(i, dt)) {
            let (e, _) = follower_pv(seg, seg.t_start + dt);
            let (expected, _) = oracle(track, i, dt);
            prop_assert!(
                (e - expected).abs() <= tol,
                "segment {i} at t={}: extruder at {e}, the design lays {expected} (tol {tol})",
                seg.t_start + dt
            );
        }
    }
    Ok(())
}

/// The ramp spreads every ratio step along the path, so across a seam the
/// extruder's velocity is continuous to within what the device resolves
/// (the acceleration tolerance over one device interval) and the rounding
/// of two evaluations of one continuous signal.
fn assert_seams_weld(committed: &[ContinuousSegment]) -> Result<(), TestCaseError> {
    for (i, pair) in committed.windows(2).enumerate() {
        let t = pair[0].t_end;
        let (e_before, v_before) = follower_pv(&pair[0], t);
        let (e_after, v_after) = follower_pv(&pair[1], t);
        prop_assert!(
            (e_after - e_before).abs() <= ORACLE_SLACK_MM * e_before.abs().max(1.0),
            "seam {i} at t={t}: the extruder jumps {} mm",
            e_after - e_before
        );
        let allowance = FIT_TOL.accel_mm_s2 * geometry::MIN_DEVICE_INTERVAL_S
            + super::ADVANCE_STEP_VELOCITY_ROUNDING * v_before.abs().max(1.0);
        prop_assert!(
            (v_after - v_before).abs() <= allowance,
            "seam {i} at t={t}: extruder velocity steps {} mm/s",
            v_after - v_before
        );
    }
    Ok(())
}

fn assert_streams_like_one_shot(
    streamed: &[ContinuousSegment],
    one_shot: &[ContinuousSegment],
) -> Result<(), TestCaseError> {
    prop_assert_eq!(streamed.len(), one_shot.len());
    for (i, (a, b)) in streamed.iter().zip(one_shot).enumerate() {
        let tol = 2.0 * segment_tol_mm(a);
        for dt in local_samples(a) {
            let t = a.t_start + dt;
            let (ea, _) = follower_pv(a, t);
            let (eb, _) = follower_pv(b, t);
            prop_assert!(
                (ea - eb).abs() <= tol,
                "segment {i} at t={t}: streamed {ea}, all at once {eb}"
            );
        }
    }
    Ok(())
}

/// A follower kernel blurs the extruder in time, so the design holds only
/// where the kernel reads nothing but a hold: there the extruder sits on
/// exactly what the shaped path laid up to it.
fn assert_kernel_settles_in_holds(
    track: &Track,
    committed: &[ContinuousSegment],
    (reach_back, reach_ahead): (f64, f64),
) -> Result<(), TestCaseError> {
    for (i, seg) in committed.iter().enumerate() {
        if !matches!(track.truth[i], Truth::Hold) {
            continue;
        }
        let tol = segment_tol_mm(seg);
        for dt in local_samples(seg) {
            let t = seg.t_start + dt;
            if t - reach_back < seg.t_start || t + reach_ahead > seg.t_end {
                continue;
            }
            let (e, _) = follower_pv(seg, t);
            let (expected, _) = oracle(track, i, dt);
            prop_assert!(
                (e - expected).abs() <= tol,
                "hold {i} at t={t}: the kernel left the extruder at {e}, the design lays {expected}"
            );
        }
    }
    Ok(())
}

fn smoothing(smooth_time: f64) -> CompiledChain {
    CompiledChain::compile(&[trajectory::PostProcessorInstance::new(
        "st",
        &trajectory::algos::SmoothTriangle,
        vec![smooth_time],
    )])
    .expect("a single smoothing kernel always compiles")
}

proptest! {
    #![proptest_config(config())]

    #[test]
    fn a_smoothed_follower_settles_on_the_laid_material_in_every_hold(
        heading in 0.0..std::f64::consts::TAU,
        steps in prop::collection::vec(step_strategy(), 1..12),
        smooth_time in 0.002..0.04_f64,
    ) {
        let track = build_track(heading, &steps);
        let follower = smoothing(smooth_time);
        let (k_lo, k_hi) = follower.kernel().expect("smoothing has a kernel").support();
        let chains = chains(follower);
        let one_shot = project_all_at_once(&track, &chains);
        assert_kernel_settles_in_holds(&track, &one_shot, (k_hi, -k_lo))?;
    }

    #[test]
    fn a_smoothed_follower_streams_like_it_projects_all_at_once(
        heading in 0.0..std::f64::consts::TAU,
        steps in prop::collection::vec(step_strategy(), 1..12),
        batch_sizes in prop::collection::vec(1..4_usize, 1..4),
        smooth_time in 0.002..0.04_f64,
    ) {
        let track = build_track(heading, &steps);
        let chains = chains(smoothing(smooth_time));
        let one_shot = project_all_at_once(&track, &chains);
        let streamed = project_streamed(&track, &chains, &batch_sizes);
        assert_streams_like_one_shot(&streamed, &one_shot)?;
    }

    #[test]
    fn projection_lays_the_ratio_along_the_shaped_path(
        heading in 0.0..std::f64::consts::TAU,
        steps in prop::collection::vec(step_strategy(), 1..12),
        batch_sizes in prop::collection::vec(1..4_usize, 1..4),
    ) {
        let track = build_track(heading, &steps);
        let chains = chains(CompiledChain::default());
        let one_shot = project_all_at_once(&track, &chains);
        assert_lays_the_oracle(&track, &one_shot)?;
        assert_seams_weld(&one_shot)?;
        let streamed = project_streamed(&track, &chains, &batch_sizes);
        assert_streams_like_one_shot(&streamed, &one_shot)?;
    }

}
