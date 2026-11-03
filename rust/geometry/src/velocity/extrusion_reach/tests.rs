use super::*;
use crate::frontend::{MoveContext, VelocityLimits, line_move};
use crate::segment::SourceRange;

const RATIO: f64 = 0.03;
const RAMP_TIME_S: f64 = 0.015;
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
        path_jerk_per_accel_hz: [1000.0, 1000.0, f64::INFINITY],
        own_jerk_per_accel_hz: f64::INFINITY,
        instantaneous_corner_velocity_mm_s: 1.0,
        ratio_ramp_time_s: RAMP_TIME_S,
        leader_kernel_reach_s: KERNEL_REACH_S,
    }
}

/// A stream of moves along x, each `(length, ratio, feedrate)`; a zero
/// length is an extrude-only move of `ratio` millimetres of filament.
fn stream(pieces: &[(f64, f64, f64)]) -> Vec<Move> {
    let limits = VelocityLimits::try_new(1000.0, 20_000.0, 0.0, f64::INFINITY)
        .unwrap()
        .with_extruder_caps(Some(caps()))
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
    ratios_in_reach(moves, EXTRUDE_ONLY.0, EXTRUDE_ONLY.1)
}

fn reach() -> ExtrusionReach {
    ExtrusionReach::new(EXTRUDE_ONLY.0, EXTRUDE_ONLY.1)
}

fn settled(reach: &mut ExtrusionReach) -> Vec<f64> {
    std::iter::from_fn(|| reach.pop().map(|(_, ratio)| ratio)).collect()
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
        (1.0, 0.0, 150.0),
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
    let waiting_mm = 150.0 * window_s;
    assert!(
        (longest as f64) < history_mm + waiting_mm + 3.0,
        "{longest} marks kept"
    );
}
