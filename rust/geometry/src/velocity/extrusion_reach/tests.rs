use nurbs::algebra::PiecewisePolynomialKernel;

use super::*;
use crate::frontend::{LeaderSpillId, MoveContext, RestSettle, VelocityLimits, line_move};
use crate::ratio_ramp::RATIO_RAMP_MAX_TIME_S;
use crate::segment::SourceRange;

const RATIO: f64 = 0.03;
const RAMP_TIME_S: f64 = RATIO_RAMP_MAX_TIME_S;
const KERNEL_REACH_S: f64 = 0.004;
const SPILL_S: f64 = 2.0 * KERNEL_REACH_S;
const EXTRUDE_ONLY: (f64, f64) = (50.0, 5000.0);

fn caps() -> ExtruderCaps {
    ExtruderCaps {
        axis_index: 3,
        max_velocity_mm_s: 50.0,
        max_accel_mm_s2: 5000.0,
        advance_slope_s: 0.02,
        advance_curvature_s2_per_mm: 0.0,
        path_jerk_per_accel_hz: [0.0, 1000.0, 1000.0, 1000.0, 0.0, 1000.0, 1000.0, 1000.0],
        unshaped_leaders: [false, false, true],
        own_jerk_per_accel_hz: f64::INFINITY,
        instantaneous_corner_velocity_mm_s: 1.0,
        leader_spill: LeaderSpillId::UNSHAPED,
        leader_kernel_reach_s: KERNEL_REACH_S,
        rest_settle: RestSettle::of_kernels(&[&box_kernel()], PATH_DEVIATION_MM),
    }
}

const PATH_DEVIATION_MM: f64 = 0.005;
const STREAM_ACCEL: f64 = 20_000.0;

fn box_kernel() -> PiecewisePolynomialKernel {
    PiecewisePolynomialKernel::single_poly(
        vec![0.5 / KERNEL_REACH_S],
        (-KERNEL_REACH_S, KERNEL_REACH_S),
    )
}

/// A stream of moves along x, each `(length, ratio, feedrate)`; a zero
/// length is an extrude-only move of `ratio` millimetres of filament.
fn stream(pieces: &[(f64, f64, f64)]) -> Vec<Move> {
    stream_under(caps(), pieces)
}

fn stream_under(caps: ExtruderCaps, pieces: &[(f64, f64, f64)]) -> Vec<Move> {
    let limits = VelocityLimits::try_new(
        1000.0,
        STREAM_ACCEL,
        crate::CornerBudget::Shaped(0.0),
        f64::INFINITY,
    )
    .unwrap()
    .with_extruder_caps(Some(caps))
    .unwrap();
    let mut x = 0.0;
    pieces
        .iter()
        .enumerate()
        .map(|(index, &(length, ratio, feedrate_mm_s))| {
            let ctx = MoveContext {
                extruder_axis: 3,
                feedrate_mm_s,
                limits,
                source: SourceRange {
                    start_line: index as u32,
                    end_line: index as u32,
                },
            };
            let start = x;
            x += length;
            let e_delta = if length == 0.0 { ratio } else { ratio * length };
            line_move([start, 0.0, 0.0], [x, 0.0, 0.0], e_delta, ctx).unwrap()
        })
        .collect()
}

fn batch(moves: &[Move]) -> Vec<f64> {
    extrusion_budgets(moves, EXTRUDE_ONLY.0, EXTRUDE_ONLY.1)
        .iter()
        .map(|budget| budget.ratio_in_reach)
        .collect()
}

fn reach() -> ExtrusionReach {
    ExtrusionReach::new(EXTRUDE_ONLY.0, EXTRUDE_ONLY.1)
}

fn settled(reach: &mut ExtrusionReach) -> Vec<f64> {
    std::iter::from_fn(|| reach.pop().map(|(_, budget)| budget.ratio_in_reach)).collect()
}

fn streamed(moves: &[Move]) -> Vec<f64> {
    let mut reach = reach();
    let mut ratios = Vec::new();
    for m in moves {
        reach.push(m.clone());
        ratios.extend(settled(&mut reach));
    }
    reach.close();
    ratios.extend(settled(&mut reach));
    assert!(reach.is_empty());
    ratios
}

#[track_caller]
fn assert_ratios(ratios: &[f64], expected: &[f64]) {
    let close = ratios.len() == expected.len()
        && ratios
            .iter()
            .zip(expected)
            .all(|(ratio, expected)| (ratio - expected).abs() <= 1e-12 * expected.abs());
    assert!(close, "{ratios:?} against {expected:?}");
}

#[test]
fn a_travel_meets_the_extrusion_its_spill_carries_across_the_blends_beside_it() {
    let line = 2.0 * RATIO;
    let moves = stream(&[
        (10.0, line, 150.0),
        (0.001, RATIO, 150.0),
        (0.45, 0.0, 150.0),
        (0.001, RATIO, 150.0),
        (10.0, line, 150.0),
    ]);
    assert_ratios(&batch(&moves), &[line; 5]);
}

#[test]
fn ratios_beyond_the_spill_and_the_ramp_stay_out_of_reach() {
    let travel = 150.0 * (SPILL_S + 0.5 * RAMP_TIME_S) + 0.1;
    let moves = stream(&[
        (10.0, RATIO, 150.0),
        (travel, 0.0, 150.0),
        (10.0, 2.0 * RATIO, 150.0),
    ]);
    assert_ratios(&batch(&moves), &[RATIO, 2.0 * RATIO, 2.0 * RATIO]);
    let moves = stream(&[
        (10.0, RATIO, 150.0),
        (travel - 0.2, 0.0, 150.0),
        (10.0, 2.0 * RATIO, 150.0),
    ]);
    assert_ratios(&batch(&moves), &[2.0 * RATIO; 3]);
}

#[test]
fn the_ramp_spreads_at_the_fastest_speed_the_leaders_reach_nearby() {
    let spilled = 1000.0 * SPILL_S;
    let ramp = 1000.0 * 0.5 * RAMP_TIME_S;
    let slow = |travel: f64| {
        batch(&stream(&[
            (10.0, RATIO, 10.0),
            (travel, 0.0, 1000.0),
            (10.0, 2.0 * RATIO, 10.0),
        ]))
    };
    assert_ratios(&slow(spilled + 0.9 * ramp), &[2.0 * RATIO; 3]);
    assert_ratios(
        &slow(spilled + 1.1 * ramp),
        &[RATIO, 2.0 * RATIO, 2.0 * RATIO],
    );
}

#[test]
fn a_rest_that_outlasts_the_spill_keeps_the_extrusion_beyond_it_out_of_reach() {
    let moves = stream(&[(10.0, 0.0, 150.0), (0.0, 1.0, 40.0), (10.0, RATIO, 150.0)]);
    assert!(least_rest_to_rest_time(1.0, 40.0, 5000.0) > SPILL_S);
    assert_ratios(&batch(&moves), &[0.0, 0.0, RATIO]);
    let brief = 0.001;
    assert!(least_rest_to_rest_time(brief, 40.0, 5000.0) < SPILL_S);
    let moves = stream(&[(10.0, 0.0, 150.0), (0.0, brief, 40.0), (10.0, RATIO, 150.0)]);
    assert_ratios(&batch(&moves), &[RATIO, 0.0, RATIO]);
}

#[test]
fn a_streamed_move_settles_once_the_moves_after_it_cover_its_reach() {
    let speed_window_mm = 150.0 * (SPILL_S + RAMP_TIME_S);
    let ratio_reach_mm = 150.0 * (SPILL_S + 0.5 * RAMP_TIME_S);
    let travel = 0.4 * speed_window_mm;
    assert!(travel < ratio_reach_mm && ratio_reach_mm < 2.0 * travel);
    let moves = stream(&[
        (10.0, RATIO, 150.0),
        (travel, 0.0, 150.0),
        (travel, 0.0, 150.0),
        (0.3 * speed_window_mm, 0.0, 150.0),
    ]);
    let mut reach = reach();
    for m in &moves[..3] {
        reach.push(m.clone());
    }
    assert!(
        reach.pop().is_none(),
        "the moves ahead fall short of its reach"
    );
    reach.push(moves[3].clone());
    assert_ratios(&settled(&mut reach), &[RATIO]);
    reach.close();
    assert_ratios(&settled(&mut reach), &[RATIO, RATIO, 0.0]);
}

#[test]
fn a_stream_settles_every_move_as_the_whole_print_would() {
    let pieces: Vec<(f64, f64, f64)> = (0..400)
        .map(|index| {
            let length = [0.3, 0.001, 0.45, 0.001, 2.0, 0.0, 25.0][index % 7];
            let ratio = [RATIO, 0.5 * RATIO, 0.0, 0.7 * RATIO, 1.3 * RATIO, 0.8, 0.0][index % 7];
            let feedrate = [300.0, 300.0, 1000.0, 300.0, 120.0, 40.0, 1000.0][index % 7];
            (length, ratio, feedrate)
        })
        .collect();
    let moves = stream(&pieces);
    assert_eq!(streamed(&moves), batch(&moves));
    let mut reach = reach();
    let mut budgets = Vec::new();
    for m in &moves {
        reach.push(m.clone());
        budgets.extend(std::iter::from_fn(|| reach.pop().map(|(_, budget)| budget)));
    }
    reach.close();
    budgets.extend(std::iter::from_fn(|| reach.pop().map(|(_, budget)| budget)));
    assert_eq!(
        budgets,
        extrusion_budgets(&moves, EXTRUDE_ONLY.0, EXTRUDE_ONLY.1)
    );
}

#[test]
fn a_move_reads_the_path_to_the_nearest_rest_where_it_narrows_a_ramp() {
    let narrows_within = rest_room_reach_mm(150.0);
    let moves = stream(&[
        (10.0, RATIO, 150.0),
        (5.0, 2.0 * RATIO, 150.0),
        (0.0, 1.0, 40.0),
        (narrows_within + 1.0, RATIO, 150.0),
        (narrows_within + 2.0, 2.0 * RATIO, 150.0),
        (narrows_within - 1.0, RATIO, 150.0),
    ]);
    let rooms: Vec<f64> = extrusion_budgets(&moves, EXTRUDE_ONLY.0, EXTRUDE_ONLY.1)
        .iter()
        .map(|budget| budget.rest_room_mm)
        .collect();
    let expected = [
        0.0,
        5.0,
        f64::INFINITY,
        0.0,
        f64::INFINITY,
        narrows_within - 1.0,
    ];
    assert!(
        rooms
            .iter()
            .zip(expected)
            .all(|(room, expected)| *room == expected || (room - expected).abs() <= 1e-9),
        "{rooms:?} against {expected:?}"
    );
}

#[test]
fn the_history_keeps_only_what_a_move_still_to_come_can_reach() {
    let moves = stream(&vec![(1.0, RATIO, 150.0); 2000]);
    let mut reach = reach();
    let mut longest = 0;
    for m in moves {
        reach.push(m);
        settled(&mut reach);
        longest = longest.max(reach.marks.marks.len());
    }
    let window_s = SPILL_S + RAMP_TIME_S;
    let history_mm = 1000.0 * window_s;
    let waiting_mm = (150.0 * window_s).max(rest_room_reach_mm(150.0));
    assert!(
        (longest as f64) < history_mm + waiting_mm + 3.0,
        "{longest} marks kept"
    );
}

#[test]
fn an_extrude_only_move_holds_for_the_kernel_reach_beside_motion_where_the_extruder_has_no_room() {
    let moves = stream(&[
        (10.0, RATIO, 150.0),
        (0.0, -0.5, 40.0),
        (0.0, 0.5, 40.0),
        (10.0, RATIO, 150.0),
        (0.0, -0.5, 40.0),
    ]);
    let holds: Vec<[f64; 2]> = extrusion_budgets(&moves, EXTRUDE_ONLY.0, EXTRUDE_ONLY.1)
        .iter()
        .map(|budget| budget.rest_holds_s)
        .collect();
    assert_eq!(
        holds,
        [
            [0.0, 0.0],
            [KERNEL_REACH_S, 0.0],
            [0.0, KERNEL_REACH_S],
            [0.0, 0.0],
            [KERNEL_REACH_S, 0.0],
        ]
    );
}

/// The caps with room on the extruder beyond the extrude-only caps.
fn roomy_caps() -> ExtruderCaps {
    ExtruderCaps {
        max_velocity_mm_s: f64::INFINITY,
        max_accel_mm_s2: 4.0 * EXTRUDE_ONLY.1,
        ..caps()
    }
}

/// The box kernel `1/2R` on `(−R, R)` leaves the shaped path settling
/// under an acceleration `A` within `A·(R − h)³/12R` of the rest point
/// `h` past the stop; the hold is the first cell start where that is
/// within half the path deviation.
#[test]
fn an_extrude_only_move_holds_until_the_shaped_path_settles_within_the_radius() {
    let radius = 0.5 * PATH_DEVIATION_MM;
    let settled_after = KERNEL_REACH_S - libm::cbrt(radius * 12.0 * KERNEL_REACH_S / STREAM_ACCEL);
    let cell = KERNEL_REACH_S / crate::frontend::SETTLE_CELLS as f64;
    let hold = (settled_after / cell).ceil() * cell;
    let moves = stream_under(
        roomy_caps(),
        &[
            (10.0, RATIO, 150.0),
            (0.0, -0.5, 40.0),
            (0.0, 0.5, 40.0),
            (10.0, RATIO, 150.0),
            (0.0, -0.5, 40.0),
        ],
    );
    let holds: Vec<[f64; 2]> = extrusion_budgets(&moves, EXTRUDE_ONLY.0, EXTRUDE_ONLY.1)
        .iter()
        .map(|budget| budget.rest_holds_s)
        .collect();
    let expected = [
        [0.0, 0.0],
        [hold, 0.0],
        [0.0, hold],
        [0.0, 0.0],
        [hold, 0.0],
    ];
    let close = holds
        .iter()
        .flatten()
        .zip(expected.iter().flatten())
        .all(|(got, want)| (got - want).abs() <= 1e-12);
    assert!(
        close && hold < KERNEL_REACH_S,
        "{holds:?} against {expected:?}"
    );
}

#[test]
fn the_holds_of_a_brief_rest_cover_the_kernels_on_both_sides_together() {
    let filament = 0.001;
    let moves = stream_under(
        roomy_caps(),
        &[
            (10.0, RATIO, 150.0),
            (0.0, filament, 40.0),
            (10.0, RATIO, 150.0),
        ],
    );
    let [before, after] = extrusion_budgets(&moves, EXTRUDE_ONLY.0, EXTRUDE_ONLY.1)[1].rest_holds_s;
    let run = least_rest_to_rest_time(filament, 40.0, EXTRUDE_ONLY.1);
    assert!(
        before < KERNEL_REACH_S && after < KERNEL_REACH_S,
        "{before} {after}"
    );
    assert!(
        (before + run + after - 2.0 * KERNEL_REACH_S).abs() <= 1e-12,
        "{before} + {run} + {after}"
    );
}

/// The hold the box kernel settles at under [`STREAM_ACCEL`].
fn settled_hold() -> f64 {
    let radius = 0.5 * PATH_DEVIATION_MM;
    let settled_after = KERNEL_REACH_S - libm::cbrt(radius * 12.0 * KERNEL_REACH_S / STREAM_ACCEL);
    let cell = KERNEL_REACH_S / crate::frontend::SETTLE_CELLS as f64;
    (settled_after / cell).ceil() * cell
}

/// An extrusion along x, a retract, a z hop of `hop_mm` at 40 mm/s, a
/// travel along x, the hop back down, a prime and an extrusion: the
/// retract and the prime meet the travel only across the unshaped z.
fn hop_stream(hop_mm: f64) -> Vec<Move> {
    let limits = VelocityLimits::try_new(
        1000.0,
        STREAM_ACCEL,
        crate::CornerBudget::Shaped(0.0),
        f64::INFINITY,
    )
    .unwrap()
    .with_extruder_caps(Some(roomy_caps()))
    .unwrap();
    let points: [([f64; 3], f64, f64); 7] = [
        ([10.0, 0.0, 0.0], 10.0 * RATIO, 150.0),
        ([10.0, 0.0, 0.0], -0.5, 40.0),
        ([10.0, 0.0, hop_mm], 0.0, 40.0),
        ([20.0, 0.0, hop_mm], 0.0, 150.0),
        ([20.0, 0.0, 0.0], 0.0, 40.0),
        ([20.0, 0.0, 0.0], 0.5, 40.0),
        ([30.0, 0.0, 0.0], 10.0 * RATIO, 150.0),
    ];
    let mut from = [0.0; 3];
    points
        .iter()
        .enumerate()
        .map(|(index, &(to, e_delta, feedrate_mm_s))| {
            let ctx = MoveContext {
                extruder_axis: 3,
                feedrate_mm_s,
                limits,
                source: SourceRange {
                    start_line: index as u32,
                    end_line: index as u32,
                },
            };
            let m = line_move(from, to, e_delta, ctx).unwrap();
            from = to;
            m
        })
        .collect()
}

#[test]
fn a_z_hop_longer_than_the_kernels_leaves_no_hold_on_its_side() {
    let holds: Vec<[f64; 2]> = extrusion_budgets(&hop_stream(0.4), EXTRUDE_ONLY.0, EXTRUDE_ONLY.1)
        .iter()
        .map(|budget| budget.rest_holds_s)
        .collect();
    assert!(0.4 / 40.0 > KERNEL_REACH_S);
    let hold = settled_hold();
    assert_eq!(holds[1], [hold, 0.0], "{holds:?}");
    assert_eq!(holds[5], [0.0, hold], "{holds:?}");
}

#[test]
fn a_z_hop_shorter_than_the_kernels_shortens_the_hold_by_its_least_time() {
    let hop_mm = 0.04;
    let lead = hop_mm / 40.0;
    assert!(lead < settled_hold());
    let holds: Vec<[f64; 2]> =
        extrusion_budgets(&hop_stream(hop_mm), EXTRUDE_ONLY.0, EXTRUDE_ONLY.1)
            .iter()
            .map(|budget| budget.rest_holds_s)
            .collect();
    let shortened = settled_hold() - lead;
    assert!((holds[1][1] - shortened).abs() <= 1e-12, "{holds:?}");
    assert!((holds[5][0] - shortened).abs() <= 1e-12, "{holds:?}");
}

#[test]
fn the_axes_in_reach_are_those_the_path_moves_on_within_the_spill() {
    let limits = VelocityLimits::try_new(
        1000.0,
        20_000.0,
        crate::CornerBudget::Shaped(0.0),
        f64::INFINITY,
    )
    .unwrap()
    .with_extruder_caps(Some(caps()))
    .unwrap();
    let corners = [
        [0.0, 0.0, 0.0],
        [10.0, 0.0, 0.0],
        [20.0, 0.0, 0.0],
        [20.0, 10.0, 0.0],
        [20.0, 10.0, 0.2],
    ];
    let moves: Vec<Move> = corners
        .windows(2)
        .enumerate()
        .map(|(index, ends)| {
            let ctx = MoveContext {
                extruder_axis: 3,
                feedrate_mm_s: 150.0,
                limits,
                source: SourceRange {
                    start_line: index as u32,
                    end_line: index as u32,
                },
            };
            line_move(ends[0], ends[1], RATIO, ctx).unwrap()
        })
        .collect();
    let axes: Vec<[bool; 3]> = extrusion_budgets(&moves, EXTRUDE_ONLY.0, EXTRUDE_ONLY.1)
        .iter()
        .map(|budget| budget.axes_in_reach)
        .collect();
    assert_eq!(
        axes,
        [
            [true, false, false],
            [true, true, false],
            [true, true, true],
            [false, true, true],
        ]
    );
}

#[test]
fn a_stream_after_a_drained_rest_budgets_as_a_fresh_one() {
    let before = stream(&[(10.0, 3.0 * RATIO, 150.0), (0.2, 3.0 * RATIO, 150.0)]);
    let after = stream(&[(0.2, RATIO, 150.0), (10.0, RATIO, 150.0)]);
    let mut reach = reach();
    for m in before {
        reach.push(m);
    }
    reach.close();
    settled(&mut reach);
    for m in after.iter().cloned() {
        reach.push(m);
    }
    reach.close();
    assert_eq!(settled(&mut reach), streamed(&after));
}
