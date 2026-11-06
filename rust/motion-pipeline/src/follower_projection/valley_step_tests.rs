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
const EXTRUDER_MAX_VELOCITY: f64 = 120.0;
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

/// One leg of the leaders along `y` at `x = 100`, moving towards `sense`
/// (±1) along it, while the extruder lays `ratio` of the path they cover
/// from `e0`.
fn leg(y: &[f64], span: (f64, f64), (ratio, sense): (f64, f64), e0: f64) -> ContinuousSegment {
    let e: Vec<f64> = std::iter::once(e0)
        .chain(y[1..].iter().map(|c| sense * ratio * c))
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

/// The leaders brake at `accel` from `cruise` into a turnaround while the
/// extruder lays `ratio_in`, then head back out of it as hard up to
/// `leave` while it lays `ratio_out`, and come to rest.
fn reversal(
    (cruise, leave): (f64, f64),
    accel: f64,
    (ratio_in, ratio_out): (f64, f64),
) -> Vec<ContinuousSegment> {
    let mut segments = Vec::new();
    let (mut t, mut y, mut e) = (0.0, 100.0, 0.0);
    let mut lay = |mut moved: Vec<f64>, duration: f64, (ratio, sense): (f64, f64)| {
        moved[0] += y;
        segments.push(leg(&moved, (t, t + duration), (ratio, sense), e));
        let [end_y, end_v, _] = eval(&moved, duration);
        e += sense * ratio * (end_y - y);
        (t, y) = (t + duration, end_y);
        end_v
    };
    let rise_time = 0.15;
    lay(
        quintic(
            [0.0, 0.0, 0.0],
            [0.6 * cruise * rise_time, cruise, -accel],
            rise_time,
        ),
        rise_time,
        (ratio_in, 1.0),
    );
    let brake_time = cruise / accel;
    lay(vec![0.0, cruise, -0.5 * accel], brake_time, (ratio_in, 1.0));
    let out = lay(
        vec![0.0, 0.0, -0.5 * accel],
        leave / accel,
        (ratio_out, -1.0),
    );
    let stop_time = (2.0 * leave / accel).max(0.15);
    lay(
        quintic(
            [0.0, out, -accel],
            [0.6 * out * stop_time, 0.0, 0.0],
            stop_time,
        ),
        stop_time,
        (ratio_out, -1.0),
    );
    segments
}

/// The leaders speed out of a rest along `y` while the extruder lays
/// `ratio`, brake into a crawl at `crawl` mm/s, their path speed
/// `crawl + curve·t²` over the 0.1 s around it, speed up again and come to
/// rest; the extrusion ends in a travel `before_crawl` seconds before the
/// crawl.
fn crawl_past_a_step(
    crawl: f64,
    curve: f64,
    before_crawl: f64,
    ratio: f64,
) -> Vec<ContinuousSegment> {
    let mut segments = Vec::new();
    let (mut t, mut y, mut e) = (0.0, 100.0, 0.0);
    let mut lay = |mut moved: Vec<f64>, duration: f64, ratio: f64| {
        moved[0] += y;
        segments.push(leg(&moved, (t, t + duration), (ratio, 1.0), e));
        let [end_y, _, _] = eval(&moved, duration);
        e += ratio * (end_y - y);
        (t, y) = (t + duration, end_y);
    };
    let (half, rise_time, brake_time) = (0.05, 0.25, 0.2);
    let entry = crawl + curve * half * half;
    let entry_accel = 2.0 * curve * half;
    let peak = entry + 0.5 * entry_accel * brake_time;
    let braked = 0.5 * (peak + entry) * brake_time;
    let rise = quintic(
        [0.0, 0.0, 0.0],
        [0.5 * peak * rise_time, peak, 0.0],
        rise_time,
    );
    lay(rise, rise_time, ratio);
    let brake = quintic([0.0, peak, 0.0], [braked, entry, -entry_accel], brake_time);
    lay(brake, brake_time, ratio);
    let through = |from: f64| vec![0.0, crawl + curve * from * from, curve * from, curve / 3.0];
    lay(through(-half), half - before_crawl, ratio);
    lay(through(-before_crawl), half + before_crawl, 0.0);
    let leave = quintic([0.0, entry, entry_accel], [braked, peak, 0.0], brake_time);
    lay(leave, brake_time, 0.0);
    let stop = quintic(
        [0.0, peak, 0.0],
        [0.5 * peak * rise_time, 0.0, 0.0],
        rise_time,
    );
    lay(stop, rise_time, 0.0);
    segments
}

/// Motion along one axis piece by piece: each piece's time span and its
/// polynomial in the time since the span's start.
type Profile = Vec<((f64, f64), Vec<f64>)>;

/// `p(x + by)`'s coefficients from `p(x)`'s.
fn shifted(coeffs: &[f64], by: f64) -> Vec<f64> {
    let mut out = vec![0.0; coeffs.len()];
    for (power, &c) in coeffs.iter().enumerate() {
        let mut binomial = 1.0;
        for (lower, slot) in out.iter_mut().enumerate().take(power + 1) {
            *slot += c * binomial * libm::pow(by, f64::from((power - lower) as i32));
            binomial *= (power - lower) as f64 / (lower + 1) as f64;
        }
    }
    out
}

/// The pieces of `profile` over `(from, to)`, scaled by `scale` about the
/// profile's value at `from` and moved to start at `start`, each of
/// degree five.
fn sliced(profile: &Profile, (from, to): (f64, f64), scale: f64, start: f64) -> Vec<BezierPiece> {
    let origin = at(profile, from);
    profile
        .iter()
        .filter(|((t0, t1), _)| *t1 > from + 1e-12 && *t0 < to - 1e-12)
        .map(|((t0, t1), coeffs)| {
            let begin = t0.max(from);
            let mut local = shifted(coeffs, begin - t0);
            local.resize(6, 0.0);
            for c in &mut local {
                *c *= scale;
            }
            local[0] += start - scale * origin;
            BezierPiece {
                u_start: begin,
                u_end: t1.min(to),
                coeffs: local,
            }
        })
        .collect()
}

fn at(profile: &Profile, t: f64) -> f64 {
    let ((t0, _), coeffs) = profile
        .iter()
        .find(|((t0, t1), _)| *t0 <= t && t <= *t1)
        .unwrap_or_else(|| panic!("the profile does not reach t={t}"));
    eval(coeffs, t - t0)[0]
}

/// The leaders' segments over `spans`, each laying `ratio` of the path
/// `profile` takes them along `y` at `x = 100`, heading towards `sense`
/// (±1) along it.
fn segments(profile: &Profile, spans: &[((f64, f64), (f64, f64))]) -> Vec<ContinuousSegment> {
    let mut e = 0.0;
    spans
        .iter()
        .map(|&(span, (ratio, sense))| {
            let pieces = |scale: f64, start: f64| {
                ContinuousAxis::Spline(Arc::new(bezier_pieces_to_nurbs(&sliced(
                    profile, span, scale, start,
                ))))
            };
            let followers: Vec<FollowerDemand> = (ratio != 0.0)
                .then(|| FollowerDemand::constant(E_AXIS, ratio))
                .into_iter()
                .collect();
            let segment = ContinuousSegment {
                axes: Arc::from([
                    spline(vec![100.0], span),
                    pieces(1.0, at(profile, span.0)),
                    spline(vec![0.2], span),
                    pieces(sense * ratio, e),
                ]),
                followers: Arc::from(followers),
                spatial_path: true,
                t_start: span.0,
                t_end: span.1,
                motor_mask: 0,
                source_line: 1,
                rest_at_end: false,
            };
            e += sense * ratio * (at(profile, span.1) - at(profile, span.0));
            segment
        })
        .collect()
}

/// The leaders speed out of a rest while the extruder lays `ratio`, cruise
/// at `cruise`, brake into a turnaround at `accel`, the braking building
/// up over `onset`, and head back out of it as hard, easing off over
/// `onset` into a cruise back at `cruise`, and come to rest.
fn turnaround_between_cruises(
    cruise: f64,
    accel: f64,
    onset: f64,
    ratio: f64,
) -> Vec<ContinuousSegment> {
    let rise_time = 0.15;
    let settle = |from: f64, to: f64| {
        quintic(
            [0.0, from, 0.0],
            [0.5 * (from + to) * rise_time, to, 0.0],
            rise_time,
        )
    };
    let mut profile: Profile = Vec::new();
    let mut spans = Vec::new();
    let (mut t, mut y) = (0.0, 100.0);
    let mut lay = |mut moved: Vec<f64>, duration: f64, sense: f64| {
        moved[0] += y;
        let [end_y, _, _] = eval(&moved, duration);
        profile.push(((t, t + duration), moved));
        spans.push(((t, t + duration), (ratio, sense)));
        (t, y) = (t + duration, end_y);
    };
    let braked = cruise - 0.5 * accel * onset;
    assert!(
        braked > 0.0,
        "the leaders brake into the turnaround past the onset"
    );
    lay(settle(0.0, cruise), rise_time, 1.0);
    lay(vec![0.0, cruise], 0.2, 1.0);
    lay(
        quintic(
            [0.0, cruise, 0.0],
            [cruise * onset - accel * onset * onset / 6.0, braked, -accel],
            onset,
        ),
        onset,
        1.0,
    );
    lay(vec![0.0, braked, -0.5 * accel], braked / accel, 1.0);
    lay(vec![0.0, 0.0, -0.5 * accel], braked / accel, -1.0);
    lay(
        quintic(
            [0.0, -braked, -accel],
            [-braked * onset - accel * onset * onset / 3.0, -cruise, 0.0],
            onset,
        ),
        onset,
        -1.0,
    );
    lay(vec![0.0, -cruise], 0.2, -1.0);
    lay(settle(-cruise, 0.0), rise_time, -1.0);
    segments(&profile, &spans)
}

/// The leaders speed out of a rest while the extruder lays `ratio`, cruise
/// at `cruise`, brake at `accel` into a dwell and leave it gently. The
/// raw moves brake at a constant rate and stop at the dwell's start; the
/// shaped ones bring the braking on over `onset` and ease it off at a
/// constant jerk over `ease`, coming to rest `ease/2` into the dwell. The
/// raw and the shaped segments, in that order.
fn shaped_stop_into_a_dwell(
    cruise: f64,
    accel: f64,
    (onset, ease): (f64, f64),
    ratio: f64,
) -> (Vec<ContinuousSegment>, Vec<ContinuousSegment>) {
    let rise_time = 0.15;
    let settle = |from: f64, to: f64| {
        quintic(
            [0.0, from, 0.0],
            [0.5 * (from + to) * rise_time, to, 0.0],
            rise_time,
        )
    };
    let mut shaped: Profile = Vec::new();
    let (mut t, mut y) = (0.0, 100.0);
    let mut lay = |profile: &mut Profile, mut moved: Vec<f64>, duration: f64| {
        moved[0] += y;
        let [end_y, _, _] = eval(&moved, duration);
        profile.push(((t, t + duration), moved));
        (t, y) = (t + duration, end_y);
        (t, y)
    };
    lay(&mut shaped, settle(0.0, cruise), rise_time);
    lay(&mut shaped, vec![0.0, cruise], 0.2);
    let onset_drop = 0.5 * accel * onset;
    let ease_drop = 0.5 * accel * ease;
    let hold = (cruise - onset_drop - ease_drop) / accel;
    assert!(
        hold > 0.0,
        "the leaders brake to rest within the onset and the ease"
    );
    lay(
        &mut shaped,
        quintic(
            [0.0, cruise, 0.0],
            [
                cruise * onset - accel * onset * onset / 6.0,
                cruise - onset_drop,
                -accel,
            ],
            onset,
        ),
        onset,
    );
    lay(
        &mut shaped,
        vec![0.0, cruise - onset_drop, -0.5 * accel],
        hold,
    );
    let (rest_at, rest) = lay(
        &mut shaped,
        vec![0.0, ease_drop, -0.5 * accel, accel / (6.0 * ease)],
        ease,
    );
    let dwell_end = rest_at + 0.05;
    lay(&mut shaped, vec![0.0], dwell_end - rest_at);
    lay(&mut shaped, settle(0.0, cruise), rise_time);
    lay(&mut shaped, settle(cruise, 0.0), rise_time);
    let stop_at = rest_at - 0.5 * ease;
    let cruise_from = shaped[1].0.0;
    let cruised = at(&shaped, cruise_from);
    let braking = 2.0 * (cruised + cruise * (stop_at - cruise_from) - rest) / cruise;
    let brake_from = stop_at - braking;
    assert!(
        braking > 0.0 && brake_from > cruise_from,
        "the raw moves brake within the cruise"
    );
    let mut raw: Profile = shaped[..1].to_vec();
    raw.push(((cruise_from, brake_from), vec![cruised, cruise]));
    raw.push((
        (brake_from, stop_at),
        vec![
            cruised + cruise * (brake_from - cruise_from),
            cruise,
            -0.5 * cruise / braking,
        ],
    ));
    raw.push(((stop_at, dwell_end), vec![rest]));
    raw.extend_from_slice(&shaped[shaped.len() - 2..]);
    let spans: Vec<((f64, f64), (f64, f64))> = raw
        .iter()
        .map(|&(span, _)| (span, (if span.0 == stop_at { 0.0 } else { ratio }, 1.0)))
        .collect();
    (segments(&raw, &spans), segments(&shaped, &spans))
}

fn project(
    (base, frontier): (&[ContinuousSegment], &[ContinuousSegment]),
    max_velocity_mm_s: f64,
) -> Vec<ContinuousSegment> {
    let follower = CompiledChain::compile(&[trajectory::PostProcessorInstance::new(
        "pa",
        &trajectory::algos::LinearPressureAdvance,
        vec![ADVANCE_K],
    )])
    .expect("an advance compiles")
    .with_follower_limits(trajectory::FollowerLimits {
        max_velocity_mm_s,
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
    let mut committed = frontier.to_vec();
    project_followers(
        ProjectionWindow {
            base,
            frontier,
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

fn assert_within_the_extruder_limit(track: &[ContinuousSegment]) {
    assert_within_the_extruder_limits((track, track), EXTRUDER_MAX_VELOCITY);
}

fn assert_within_the_extruder_limits(
    tracks: (&[ContinuousSegment], &[ContinuousSegment]),
    max_velocity: f64,
) {
    let committed = project(tracks, max_velocity);
    let mut fastest = (0.0_f64, 0.0);
    let mut strongest = (0.0_f64, 0.0);
    for segment in &committed {
        let samples = 4000;
        for k in 0..=samples {
            let t = segment.t_start
                + (segment.t_end - segment.t_start) * f64::from(k) / f64::from(samples);
            let pva = segment.axes[E_AXIS]
                .eval_pva(t)
                .unwrap_or_else(|error| panic!("extruder at t={t}: {error:?}"));
            if pva.velocity.abs() > fastest.0 {
                fastest = (pva.velocity.abs(), t);
            }
            if pva.acceleration.abs() > strongest.0 {
                strongest = (pva.acceleration.abs(), t);
            }
        }
    }
    assert!(
        strongest.0 <= EXTRUDER_MAX_ACCEL + FIT_TOL.accel_mm_s2,
        "the extruder accelerates at {} mm/s² at t={}, past its {EXTRUDER_MAX_ACCEL} mm/s² limit",
        strongest.0,
        strongest.1
    );
    let speed_miss = 2.0 * (FIT_TOL.pos_mm * FIT_TOL.accel_mm_s2).sqrt();
    assert!(
        fastest.0 <= max_velocity + speed_miss,
        "the extruder moves at {} mm/s at t={}, past its {max_velocity} mm/s limit",
        fastest.0,
        fastest.1
    );
}

/// An extrusion ending in a travel that reverses over it
/// (`voron_first_layers` line 3654 into 3655 on the p1 bowden profile).
#[test]
fn a_ratio_step_down_at_a_turnaround_keeps_the_extruder_within_its_limit() {
    assert_within_the_extruder_limit(&reversal((40.0, 40.0), 861.19, (0.033_645, 0.0)));
}

/// A travel reversing into an extrusion back over it (`corner_flow`'s
/// travel to the origin into the flow case's first line, on the p1 bowden
/// profile).
#[test]
fn a_ratio_step_up_at_a_turnaround_keeps_the_extruder_within_its_limit() {
    assert_within_the_extruder_limit(&reversal((40.0, 40.0), 864.0, (0.0, 0.033_26)));
}

/// The same step where the extrusion keeps accelerating long past the
/// turnaround (`corner_flow_cases` at s=2521.08 on the p1 bowden profile):
/// the side trail probes that leg far apart, none between the edge of the
/// valley's zero stretch, which the leaders already cross at speed, and
/// the stretch of the side where the rebuild bends hardest.
#[test]
fn a_ratio_step_up_at_a_turnaround_into_a_long_leg_keeps_the_extruder_within_its_limit() {
    assert_within_the_extruder_limit(&reversal((40.0, 300.0), 864.0, (0.0, 0.033_26)));
}

/// A crawl 15 ms past where an extrusion ends in a travel (the shape of
/// `voron_dense_top_layers` at s=448.51 on the p1 bowden profile, gscale
/// 0.45): the laid ratio is zero over the crawl valley's before side, but
/// the advance reads the extrusion's ramp there, which a side sized for no
/// ratio at all rebuilds the lead too steeply for (6900 mm/s²).
#[test]
fn a_crawl_valley_side_sizes_its_rebuild_by_the_ramped_ratio_the_advance_reads() {
    assert_within_the_extruder_limit(&crawl_past_a_step(1.8, 30_000.0, 0.015, 0.034));
}

/// The leaders brake into a dwell while the extruder lays the ratio of a
/// dense top layer, their own demand within the extruder's limit, the
/// shaped braking easing off into the rest ([`shaped_stop_into_a_dwell`]):
/// the rest valley's before side, as wide as the ramp the planner budgets
/// for releasing the lead, rebuilds it far steeper than the limit, the
/// shape of `voron_dense_top_layers_zhop` at t=14.934 on the p4 high-accel
/// profile once the crawl valley reserves stop covering it.
#[test]
fn a_rest_valley_side_sizes_its_rebuild_within_the_extruder_limit() {
    let (raw, shaped) = shaped_stop_into_a_dwell(50.0, 2000.0, (0.02, 0.01), 0.034);
    assert_within_the_extruder_limits((&raw, &shaped), EXTRUDER_MAX_VELOCITY);
}

/// A turnaround between two cruises whose advanced flow stays within
/// 20 mm/s on either side: the valley's sides release and rebuild the lead
/// while the leaders brake and speed up across them, which adds
/// `k·r·W′·ṡ²`, up to `8/9·k·r·a`, to the flow, past the limit.
#[test]
fn a_crawl_valley_side_sizes_its_rebuild_within_the_extruder_velocity() {
    let track = turnaround_between_cruises(40.0, 864.0, 0.02, 0.033_6);
    assert_within_the_extruder_limits((&track, &track), 20.0);
}
