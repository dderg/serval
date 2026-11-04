use geometry::path::Segment;
use geometry::segment::SourceRange;
use geometry::{CornerFitConfig, MoveContext, ScalarLaw, VelocityLimits, line_move};
use trajectory::{AxisChainSet, ContinuousAxis, ContinuousSegment};

use super::continuous_axis_enclosure;
use super::taylor::Window;
use crate::Pipeline;
use crate::shaper::SQUARE_ROOT_ONSET_EDGE_SLACK_S;
use crate::types::{BaseItem, StreamConfig};

const ACCURACY: [f64; 3] = [1e-9, f64::INFINITY, 1e-6];
const WINDOWS_PER_SEGMENT: u32 = 6;
const CHECKS_PER_PIECE: u32 = 24;

fn limits() -> VelocityLimits {
    VelocityLimits::try_new(300.0, 25_000.0, 0.05, f64::INFINITY).unwrap()
}

fn config() -> StreamConfig {
    StreamConfig {
        corner: CornerFitConfig::default(),
        integration_tol: 1e-7,
        max_extrude_only_velocity_mm_s: f64::INFINITY,
        max_extrude_only_accel_mm_s2: f64::INFINITY,
        fit_tol_mm: 0.005,
        fit_tol_accel_mm_s2: 50.0,
        max_buffer_moves: 256,
        limits: limits(),
        step_capacity: None,
    }
}

fn moves_through(points: &[[f64; 2]], feedrate: f64) -> Vec<geometry::Move> {
    points
        .windows(2)
        .enumerate()
        .map(|(line, pair)| {
            line_move(
                [pair[0][0], pair[0][1], 0.2],
                [pair[1][0], pair[1][1], 0.2],
                0.03 * libm::hypot(pair[1][0] - pair[0][0], pair[1][1] - pair[0][1]),
                MoveContext {
                    extruder_axis: 3,
                    feedrate_mm_s: feedrate,
                    limits: limits(),
                    source: SourceRange {
                        start_line: line as u32,
                        end_line: line as u32,
                    },
                },
            )
            .unwrap()
        })
        .collect()
}

fn lowered(moves: Vec<geometry::Move>, home: [f64; 2]) -> Vec<ContinuousSegment> {
    let mut pipeline = Pipeline::new(
        config(),
        AxisChainSet::default(),
        vec![home[0], home[1], 0.2, 0.0],
        0.0,
    );
    let Pipeline { front, .. } = &mut pipeline;
    let mut segments = Vec::new();
    let mut keep = |item: BaseItem| {
        if let BaseItem::Seg(segment) = item {
            segments.push(segment);
        }
        true
    };
    for movement in moves {
        assert!(front.feed(movement.into(), &mut keep));
    }
    assert!(front.finish(&mut keep));
    segments
}

fn polygon(sides: usize, radius: f64) -> Vec<[f64; 2]> {
    (0..=sides)
        .map(|k| {
            let angle = std::f64::consts::TAU * k as f64 / sides as f64;
            [radius * libm::cos(angle), radius * libm::sin(angle)]
        })
        .collect()
}

#[derive(Default, Debug)]
struct Coverage {
    lines: usize,
    arcs: usize,
    clothoids: usize,
    rails: usize,
    followers: usize,
}

fn assert_enclosed(axis: &ContinuousAxis, (lo, hi): (f64, f64)) {
    let pieces = continuous_axis_enclosure(axis, (lo, hi), ACCURACY)
        .expect("an analytic planar axis has an enclosure");
    assert!(!pieces.is_empty(), "no enclosure over [{lo}, {hi}]");
    if let ContinuousAxis::Analytic { span, .. } = axis {
        let contacts: Vec<f64> = span.cap_contact_times().collect();
        for piece in pieces
            .iter()
            .filter(|piece| piece.pva.iter().any(|order| !order.rounding.is_finite()))
        {
            assert!(
                contacts.iter().any(|&contact| {
                    contact >= piece.lo - SQUARE_ROOT_ONSET_EDGE_SLACK_S
                        && contact <= piece.hi + SQUARE_ROOT_ONSET_EDGE_SLACK_S
                }),
                "[{}, {}] is unbounded away from every cap contact {contacts:?}",
                piece.lo,
                piece.hi
            );
        }
    }
    for piece in &pieces {
        for k in 0..CHECKS_PER_PIECE {
            let t = piece.lo
                + (piece.hi - piece.lo) * (f64::from(k) + 0.5) / f64::from(CHECKS_PER_PIECE);
            let truth = axis.eval_pva(t).expect("the analytic axis evaluates");
            let x = (t - piece.center) / piece.radius;
            for (order, (value, noise)) in [
                (truth.position, 1e-12),
                (truth.velocity, 1e-11),
                (truth.acceleration, 1e-9),
            ]
            .into_iter()
            .enumerate()
            {
                let polynomial = &piece.pva[order];
                let miss = (value - polynomial.eval(x)).abs();
                assert!(
                    miss <= polynomial.rounding + noise * (1.0 + value.abs()),
                    "order {order} at t={t}: the enclosure on [{}, {}] misses {value} by \
                     {miss:e}, past its bound {:e}",
                    piece.lo,
                    piece.hi,
                    polynomial.rounding
                );
            }
        }
    }
}

fn check_every_analytic_axis(segments: &[ContinuousSegment], coverage: &mut Coverage) {
    for segment in segments {
        for (index, axis) in segment.axes.iter().enumerate() {
            let ContinuousAxis::Analytic { span, .. } = axis else {
                continue;
            };
            if index >= 3 {
                coverage.followers += 1;
            } else {
                match &span.source.segment.spatial {
                    Some(Segment::Line(_)) => coverage.lines += 1,
                    Some(Segment::Arc(_)) => coverage.arcs += 1,
                    Some(Segment::Clothoid(_)) => coverage.clothoids += 1,
                    None => {}
                }
            }
            coverage.rails += span
                .phases
                .iter()
                .filter(|phase| matches!(phase.law, ScalarLaw::DiskRail { .. }))
                .count();
            let width = (segment.t_end - segment.t_start) / f64::from(WINDOWS_PER_SEGMENT);
            for window in 0..WINDOWS_PER_SEGMENT {
                let lo = segment.t_start + width * f64::from(window);
                assert_enclosed(axis, (lo, lo + width));
            }
        }
    }
}

#[test]
fn analytic_lines_corners_and_arcs_are_enclosed_by_their_formulas() {
    let mut coverage = Coverage::default();
    let square = [
        [0.0, 0.0],
        [20.0, 0.0],
        [20.0, 20.0],
        [0.0, 20.0],
        [0.0, 0.0],
    ];
    check_every_analytic_axis(
        &lowered(moves_through(&square, 200.0), [0.0, 0.0]),
        &mut coverage,
    );
    let circle = polygon(48, 10.0);
    check_every_analytic_axis(
        &lowered(moves_through(&circle, 150.0), circle[0]),
        &mut coverage,
    );
    assert!(
        coverage.lines > 0
            && coverage.clothoids + coverage.arcs > 0
            && coverage.rails > 0
            && coverage.followers > 0,
        "the moves cover too little: {coverage:?}"
    );
}

/// Every analytic axis read window by window with its jerk: each stretch
/// one formula covers is read where its jerk exists, and no sample of the
/// axis's own jerk leaves the model's bound.
#[test]
fn analytic_axis_windows_enclose_the_axis_jerk() {
    let circle = polygon(48, 10.0);
    let square = [
        [0.0, 0.0],
        [20.0, 0.0],
        [20.0, 20.0],
        [0.0, 20.0],
        [0.0, 0.0],
    ];
    let mut read = 0usize;
    for segments in [
        lowered(moves_through(&square, 200.0), [0.0, 0.0]),
        lowered(moves_through(&circle, 150.0), circle[0]),
    ] {
        for segment in &segments {
            for axis in segment.axes.iter() {
                let Some(windows) = super::AxisWindows::of(axis) else {
                    continue;
                };
                let mut breaks = vec![segment.t_start, segment.t_end];
                breaks.extend(windows.breaks(segment.t_start, segment.t_end));
                breaks.sort_by(f64::total_cmp);
                for stretch in breaks.windows(2).filter(|stretch| stretch[1] > stretch[0]) {
                    let quarter = 0.25 * (stretch[1] - stretch[0]);
                    let window = Window::new(stretch[0] + quarter, stretch[1] - quarter);
                    let Some([_, _, _, jerk]) = windows.models(window, true) else {
                        continue;
                    };
                    read += 1;
                    for k in 0..=8 {
                        let t = window.lo + (window.hi - window.lo) * f64::from(k) / 8.0;
                        let truth = axis
                            .eval_pvaj(t)
                            .expect("the axis has exact derivatives")
                            .jerk;
                        let x = (t - window.center) / window.radius;
                        let value: f64 = jerk
                            .coefficients()
                            .iter()
                            .rev()
                            .fold(0.0, |acc, c| acc * x + c);
                        assert!(
                            (truth - value).abs() <= jerk.rad() + 1e-6 * (1.0 + truth.abs()),
                            "at t={t} the axis's jerk {truth} leaves its model {value} ± {}",
                            jerk.rad()
                        );
                    }
                }
            }
        }
    }
    assert!(read > 0, "no window read a jerk");
}

fn profile_axes() -> Vec<ContinuousAxis> {
    let nudge = |accel| {
        ContinuousAxis::Nudge(
            trajectory::NudgeProfile::try_new(-3.0, 40.0, accel, 2.0).expect("a nudge"),
        )
    };
    let buzz = |sweep_end_hz, ramp| ContinuousAxis::Buzz {
        base_position: 120.0,
        sign: -1.0,
        profile: std::sync::Arc::new(
            trajectory::BuzzProfile::try_new(0.4, 20.0, sweep_end_hz, 0.3, ramp, 5.0)
                .expect("a buzz"),
        ),
    };
    vec![
        nudge(2_000.0),
        nudge(0.0),
        nudge(40_000.0),
        buzz(20.0, 0.05),
        buzz(80.0, 0.05),
        buzz(80.0, 0.0),
    ]
}

fn profile_domain(axis: &ContinuousAxis) -> (f64, f64) {
    match axis {
        ContinuousAxis::Nudge(profile) => (profile.t_start(), profile.t_end()),
        ContinuousAxis::Buzz { profile, .. } => (profile.t_start(), profile.t_end()),
        _ => unreachable!("a profile axis"),
    }
}

#[test]
fn nudges_and_buzzes_are_enclosed_by_their_formulas() {
    for axis in profile_axes() {
        let (start, end) = profile_domain(&axis);
        let inset = 1e-9 * (end - start);
        assert_enclosed(&axis, (start + inset, end - inset));
    }
}

#[test]
fn nudge_and_buzz_windows_enclose_their_jerk() {
    for axis in profile_axes() {
        let (start, end) = profile_domain(&axis);
        let windows = super::AxisWindows::of(&axis).expect("a profile axis has windows");
        let mut breaks = vec![start, end];
        breaks.extend(windows.breaks(start, end));
        breaks.sort_by(f64::total_cmp);
        for stretch in breaks.windows(2).filter(|stretch| stretch[1] > stretch[0]) {
            for k in 0..16 {
                let width = (stretch[1] - stretch[0]) / 16.0;
                let window = Window::new(
                    stretch[0] + width * f64::from(k),
                    stretch[0] + width * f64::from(k + 1),
                );
                let [_, _, _, jerk] = windows
                    .models(window, true)
                    .expect("one formula covers the window");
                for j in 1..8 {
                    let t = window.lo + (window.hi - window.lo) * f64::from(j) / 8.0;
                    let truth = axis.eval_pvaj(t).expect("the axis evaluates").jerk;
                    let x = (t - window.center) / window.radius;
                    let value: f64 = jerk
                        .coefficients()
                        .iter()
                        .rev()
                        .fold(0.0, |acc, c| acc * x + c);
                    assert!(
                        (truth - value).abs() <= jerk.rad() + 1e-6 * (1.0 + truth.abs()),
                        "at t={t} the jerk {truth} leaves its model {value} ± {}",
                        jerk.rad()
                    );
                }
            }
        }
    }
}
