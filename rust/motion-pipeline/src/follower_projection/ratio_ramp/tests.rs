use std::sync::Arc;

use geometry::RatioRampRule;
use nurbs::bezier::{BezierPiece, bezier_pieces_to_nurbs};
use trajectory::{ContinuousAxis, ContinuousSegment};

use super::super::FollowerState;
use super::{
    CLUSTER_REACH_S, CROSSING_PROBES, Crossing, ENVELOPE_SLOPE, KERNEL_PEAK, NODE_SPACING_S,
    RampLeg, ShapedLeaders,
    WIDTH_SLOPE_BOUND, WidthField,
};

/// `(length, ratio at start, ratio at end)` per span.
type Table = Vec<(f64, f64, f64)>;

fn line_table() -> Table {
    vec![
        (10.0, 0.0, 0.0),
        (20.0, 0.03, 0.03),
        (10.0, 0.045, 0.045),
        (2.0, 0.045, 0.02),
        (10.0, 0.02, 0.02),
        (0.4, 0.0, 0.0),
        (0.3, 0.03, 0.03),
        (10.0, 0.0, 0.0),
    ]
}

/// A table whose seams sit in clusters far shorter than a window, as the
/// fitter's corner blends leave them: 0.01 mm spans stepping up and down.
fn clustered_table() -> Table {
    let mut table = vec![(10.0, 0.03, 0.03)];
    for index in 0..8 {
        let ratio = if index % 2 == 0 { 0.0 } else { 0.045 };
        table.push((0.01, ratio, ratio));
    }
    table.push((0.01, 0.03, 0.01));
    table.push((0.01, 0.01, 0.03));
    table.push((10.0, 0.03, 0.03));
    table
}

/// `table` with `pieces` spans of `piece` mm cut off both ends of every
/// span long enough, each carrying the span's own ratio there: the same
/// ratio function.
fn cut(table: &Table, piece: f64, pieces: usize) -> Table {
    let mut out = Vec::new();
    for &(length, r0, r1) in table {
        let at = |offset: f64| r0 + (r1 - r0) * offset / length;
        if 4.0 * piece * pieces as f64 >= length {
            out.push((length, r0, r1));
            continue;
        }
        let mut cuts: Vec<f64> = (0..=pieces).map(|k| piece * k as f64).collect();
        cuts.extend((0..=pieces).rev().map(|k| length - piece * k as f64));
        for pair in cuts.windows(2) {
            out.push((pair[1] - pair[0], at(pair[0]), at(pair[1])));
        }
    }
    out
}

fn length(table: &Table) -> f64 {
    table.iter().map(|span| span.0).sum()
}

/// A width field with a rest at both ends of `[0, length]` and nodes every
/// `spacing` mm in between, each asking for `target(s)`.
fn width(length: f64, spacing: f64, target: impl Fn(f64) -> f64) -> WidthField {
    let mut field = WidthField::default();
    field.push_rest(0.0);
    let mut s = spacing;
    while s < length {
        field.push(s, target(s));
        s += spacing;
    }
    field.push_rest(length);
    field
}

fn cruise(length: f64) -> WidthField {
    width(length, 0.5, |_| 1.0)
}

/// Widths that change as the leaders slow into corners and speed up out of
/// them, down to a few hundredths of a millimetre.
fn cornering(length: f64) -> WidthField {
    width(length, 0.25, |s| {
        0.05 + 1.2 * (0.5 + 0.5 * libm::sin(0.7 * s)).powi(3)
    })
}

fn smoothed(table: &Table, field: WidthField) -> FollowerState {
    let mut state = FollowerState::default();
    for &(length, r0, r1) in table {
        state.push_span(length, r0, r1);
    }
    state.smoothing.width = field;
    state.smoothing.base_slope = Some(state.spans[0].ratio_slope());
    state.smoothing.record_seams(&state.spans);
    state
}

fn probes(length: f64, count: usize) -> impl Iterator<Item = f64> {
    (1..count).map(move |k| length * k as f64 / count as f64 + 1e-7)
}

/// The table's own ratio range over `[lo, hi]`.
fn table_range(state: &FollowerState, lo: f64, hi: f64) -> (f64, f64) {
    state
        .spans
        .iter()
        .filter(|span| span.s1 > lo && span.s0 < hi)
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(low, high), span| {
            (
                low.min(span.r0.min(span.r1)),
                high.max(span.r0.max(span.r1)),
            )
        })
}

#[test]
fn a_table_cut_into_short_spans_is_smoothed_exactly_as_the_whole_one() {
    for table in [line_table(), clustered_table()] {
        for field in [cruise, cornering] {
            let total = length(&table);
            let whole = smoothed(&table, field(total));
            let pieces = smoothed(&cut(&table, 0.01, 5), field(total));
            for s in probes(total, 20_000) {
                let (whole_ratio, whole_slope) = whole.ratio_and_slope(s);
                let (cut_ratio, cut_slope) = pieces.ratio_and_slope(s);
                assert!(
                    (whole_ratio - cut_ratio).abs() < 1e-12,
                    "the cut table lays {cut_ratio} at {s} mm, the whole one {whole_ratio}"
                );
                assert!(
                    (whole_slope - cut_slope).abs() < 1e-9,
                    "the cut table's slope is {cut_slope} at {s} mm, the whole one's {whole_slope}"
                );
                assert!(
                    (whole.spans_e(s) - pieces.spans_e(s)).abs() < 1e-12,
                    "the cut table has laid {} mm by {s} mm, the whole one {}",
                    pieces.spans_e(s),
                    whole.spans_e(s)
                );
            }
        }
    }
}

#[test]
fn the_smoothed_ratio_stays_within_the_ratios_its_window_spans() {
    for table in [line_table(), clustered_table()] {
        for field in [cruise, cornering] {
            let total = length(&table);
            let state = smoothed(&table, field(total));
            for s in probes(total, 20_000) {
                let h = state.smoothing.width.width(s);
                let (low, high) = table_range(&state, s - h, s + h);
                let (ratio, _) = state.ratio_and_slope(s);
                assert!(
                    ratio >= low - 1e-12 && ratio <= high + 1e-12,
                    "the smoothed ratio {ratio} at {s} mm leaves [{low}, {high}]"
                );
            }
        }
    }
}

#[test]
fn the_smoothing_lays_exactly_the_table_material_wherever_the_window_is_flat() {
    for table in [line_table(), clustered_table()] {
        for field in [cruise, cornering] {
            let total = length(&table);
            let state = smoothed(&table, field(total));
            let mut flat_probes = 0;
            for s in probes(total, 5_000) {
                let h = state.smoothing.width.width(s);
                let (low, high) = table_range(&state, s - h, s + h);
                if low != high {
                    continue;
                }
                flat_probes += 1;
                let correction = state.smoothing.correction(s).value;
                assert!(
                    correction.abs() < 1e-15,
                    "the smoothed track is {correction} mm off the table at {s} mm, where \
                     the ratio is flat across the window"
                );
            }
            assert!(
                flat_probes > 1000,
                "only {flat_probes} probes sit in flat windows"
            );
            let end = state.smoothing.correction(total).value;
            assert_eq!(
                end, 0.0,
                "the smoothed track ends off the table at the rest"
            );
        }
    }
}

#[test]
fn the_smoothed_ratio_is_c1_and_its_derivatives_are_exact() {
    for table in [line_table(), clustered_table()] {
        for field in [cruise, cornering] {
            let total = length(&table);
            let state = smoothed(&table, field(total));
            let mut points: Vec<f64> = state.spans.iter().map(|span| span.s0).collect();
            points.extend(state.smoothing.bends(0.0, total));
            points.retain(|&s| s > 1e-3 && s < total - 1e-3);
            let eps = 1e-9;
            for &s in &points {
                let (left_ratio, left_slope) = state.ratio_and_slope(s - eps);
                let (right_ratio, right_slope) = state.ratio_and_slope(s + eps);
                assert!(
                    (left_ratio - right_ratio).abs() < 1e-9,
                    "the ratio jumps at {s} mm: {left_ratio} -> {right_ratio}"
                );
                assert!(
                    (left_slope - right_slope).abs() < 1e-5,
                    "the ratio's slope jumps at {s} mm: {left_slope} -> {right_slope}"
                );
            }
            let h = 1e-6;
            for s in probes(total, 3_000) {
                if points.iter().any(|&point| (point - s).abs() < 4.0 * h) {
                    continue;
                }
                let jet = state.smoothing.correction(s);
                let before = state.smoothing.correction(s - h);
                let after = state.smoothing.correction(s + h);
                for (order, value, numeric) in [
                    (1, jet.d1, (after.value - before.value) / (2.0 * h)),
                    (2, jet.d2, (after.d1 - before.d1) / (2.0 * h)),
                    (3, jet.d3, (after.d2 - before.d2) / (2.0 * h)),
                ] {
                    assert!(
                        (value - numeric).abs() < 1e-5 * (1.0 + numeric.abs()),
                        "derivative {order} of the correction at {s} mm is {value}, its \
                         difference quotient {numeric}"
                    );
                }
            }
        }
    }
}

#[test]
fn a_cluster_of_short_spans_is_spread_over_the_whole_window() {
    let table = clustered_table();
    let total = length(&table);
    let state = smoothed(&table, cruise(total));
    let variation: f64 = state
        .smoothing
        .seams
        .iter()
        .map(|seam| seam.step.abs())
        .sum();
    let steepest = probes(total, 50_000)
        .map(|s| state.ratio_and_slope(s).1.abs())
        .fold(0.0, f64::max);
    let width = 1.0;
    assert!(
        steepest <= KERNEL_PEAK * variation / width * (1.0 + 1e-9),
        "a cluster of steps adding to {variation} climbs {steepest} per mm under a {width} mm \
         window"
    );
    let (_, bound) = state.smoothing.ratio_bounds(&state.spans, 0.0, total);
    assert!(
        steepest <= bound,
        "the slope bound {bound} is under the slope {steepest} the smoothing reaches"
    );
}

#[test]
fn the_slope_bound_covers_every_window() {
    for table in [line_table(), clustered_table()] {
        let total = length(&table);
        let state = smoothed(&table, cornering(total));
        let mut lo = 0.0;
        while lo < total {
            let hi = (lo + 1.7).min(total);
            let (ratio_bound, slope_bound) = state.smoothing.ratio_bounds(&state.spans, lo, hi);
            for k in 0..400 {
                let s = lo + (hi - lo) * (k as f64 + 0.5) / 400.0;
                let (ratio, slope) = state.ratio_and_slope(s);
                assert!(
                    ratio.abs() <= ratio_bound * (1.0 + 1e-12),
                    "ratio {ratio} at {s} mm over the bound {ratio_bound}"
                );
                assert!(
                    slope.abs() <= slope_bound,
                    "slope {slope} at {s} mm over the bound {slope_bound}"
                );
            }
            lo = hi;
        }
    }
}

/// A move that leaves the leaders at rest — an extrude-only move, a dwell
/// — stacks its width nodes on the rest node, and the zero-length stretch
/// it rides there is bounded like any other.
#[test]
fn a_zero_length_stretch_on_stacked_rest_nodes_is_bounded() {
    let table = line_table();
    let total = length(&table);
    let mut field = WidthField::default();
    field.push_rest(0.0);
    for target in [0.0, 0.0, 0.2] {
        field.push(0.0, target);
    }
    let mut s = 0.5;
    while s < total {
        field.push(s, 1.0);
        s += 0.5;
    }
    field.push_rest(total);
    let state = smoothed(&table, field);
    let (ratio_bound, slope_bound) = state.smoothing.ratio_bounds(&state.spans, 0.0, 0.0);
    let (ratio, slope) = state.ratio_and_slope(0.0);
    assert!(ratio.abs() <= ratio_bound && slope.abs() <= slope_bound);
    assert!(state.smoothing.bends(0.0, 0.0).contains(&0.0));
}

#[test]
fn a_rest_an_ulp_past_a_resting_node_closes_the_window_on_its_own_place() {
    let mut field = WidthField::default();
    field.push_rest(0.0);
    field.push(10.0, 1.0);
    let resting = 49.91556775667482;
    let seam = trajectory::next_toward(resting, f64::INFINITY);
    field.push_rest(resting);
    field.push(resting, 0.01);
    field.push_rest(seam);
    field.push(50.0, 0.2);
    assert_eq!(field.width(seam), 0.0);
}

#[test]
fn an_excursion_between_two_rests_within_a_node_spacing_opens_a_window() {
    let mut field = WidthField::default();
    field.push_rest(0.0);
    field.push_rest(0.04);
    let seam = 0.00072;
    let width = field.width(seam);
    assert!(
        width > 0.0 && width <= WIDTH_SLOPE_BOUND * seam,
        "a {width} mm window at {seam} mm off the rest"
    );
    assert_eq!(field.width(0.0), 0.0);
    assert_eq!(field.width(0.04), 0.0);
}

#[test]
fn the_width_is_c2_lipschitz_and_closes_on_every_rest() {
    let mut field = WidthField::default();
    field.push_rest(0.0);
    let targets = [
        (0.3, 1.0),
        (0.6, 1.4),
        (0.65, 0.02),
        (1.0, 2.0),
        (3.0, 2.0),
        (3.0, 0.0),
        (3.4, 0.6),
        (5.0, 0.9),
        (5.5, 3.0),
        (9.0, 3.0),
    ];
    for (s, target) in targets {
        field.push(s, target);
    }
    field.push_rest(12.0);
    let rests: Vec<f64> = field
        .nodes
        .iter()
        .filter(|node| node.envelope == 0.0)
        .map(|node| node.s)
        .collect();
    let h = 1e-7;
    let mut s = 1e-4;
    while s < 12.0 {
        let jet = field.jet(s);
        assert!(
            jet.d1.abs() <= WIDTH_SLOPE_BOUND + 1e-12,
            "the width climbs {} per mm at {s} mm",
            jet.d1
        );
        let nearest_rest = rests
            .iter()
            .map(|rest| (rest - s).abs())
            .fold(f64::INFINITY, f64::min);
        assert!(
            jet.value <= WIDTH_SLOPE_BOUND * nearest_rest + 1e-12,
            "a {} mm window at {s} mm reaches a rest {nearest_rest} mm away",
            jet.value
        );
        if !rests.iter().any(|rest| (rest - s).abs() < 2.0 * h) {
            let (before, after) = (field.jet(s - h), field.jet(s + h));
            assert!(
                (after.value - before.value - 2.0 * h * jet.d1).abs() < 1e-9,
                "the width's slope at {s} mm is not its derivative"
            );
            assert!(
                (after.d1 - before.d1 - 2.0 * h * jet.d2).abs() < 1e-6,
                "the width's curvature at {s} mm is not its slope's derivative"
            );
        }
        s += 1e-3;
    }
    for node in field.nodes.iter().filter(|node| node.envelope > 0.0) {
        let eps = 1e-9;
        let (before, after) = (field.jet(node.s - eps), field.jet(node.s + eps));
        assert!(
            (after.d2 - before.d2).abs() < 1e-4,
            "the width's curvature jumps at the node at {} mm: {} -> {}",
            node.s,
            before.d2,
            after.d2
        );
    }
    for pair in field.nodes.windows(2) {
        assert!(
            (pair[1].envelope - pair[0].envelope).abs()
                <= ENVELOPE_SLOPE * (pair[1].s - pair[0].s) + 1e-12,
            "the envelope steps from {} to {} over {} mm",
            pair[0].envelope,
            pair[1].envelope,
            pair[1].s - pair[0].s
        );
    }
}

#[test]
fn a_final_width_node_never_moves_again() {
    let mut field = WidthField::default();
    field.place_rest(0.0);
    let speeds = [
        2.0, 3.0, 3.0, 3.0, 2.5, 0.4, 0.05, 0.4, 2.0, 3.0, 3.0, 3.0, 3.0, 1.0, 0.0, 0.0, 1.5, 3.0,
        3.0, 3.0, 3.0, 3.0, 3.0,
    ];
    let mut s = 0.0;
    let mut frozen: Vec<(usize, f64)> = Vec::new();
    for (tick, speed) in speeds.into_iter().enumerate() {
        s += speed;
        field.place_sample(s, tick as f64 * NODE_SPACING_S);
        field.assign(f64::INFINITY);
        for &(index, envelope) in &frozen {
            assert_eq!(
                field.nodes[index].envelope, envelope,
                "node {index} moved after it was final"
            );
        }
        field.final_through();
        frozen = (0..field.final_prefix)
            .map(|index| (index, field.nodes[index].envelope))
            .collect();
    }
    assert!(
        field.final_prefix > 10,
        "only {} nodes became final",
        field.final_prefix
    );
}

/// The shaped leaders creep into and out of a rest by rounding: a time node
/// a few ulps off the rest is at the rest, so the width falls into it and
/// rises out of it linearly rather than flattening onto a sliver of a
/// window just short of it.
#[test]
fn a_time_node_a_rounding_off_a_rest_narrows_the_width_into_it_linearly() {
    let mut field = WidthField::default();
    field.push_rest(0.0);
    let rest: f64 = 2.5;
    let up = |s: f64| trajectory::next_toward(s, f64::INFINITY);
    let leaving = up(up(rest));
    field.push(1.0, 0.5);
    field.push(2.0, 0.5);
    field.push(rest, 0.25);
    field.push_rest(up(rest));
    field.push(up(rest), 0.0);
    for s in [leaving, 3.5, 4.5] {
        field.push(s, 0.5);
    }
    field.push_rest(5.0);
    for d in [1e-5, 1e-4, 1e-3] {
        for s in [rest - d, leaving + d] {
            let width = field.width(s);
            assert!(
                width >= 0.5 * d,
                "a {width} mm window {d} mm off the rest at {rest} mm"
            );
        }
    }
}

const REST_LEG_HALF_TIME_S: f64 = 0.01;
const REST_LEG_SPILL_MM: f64 = 0.01;

/// Leaders moving along x and y as `x` and `y` lay them over `[t0, t1]`,
/// z and the follower held.
fn leaders(x: &[BezierPiece], y: &[BezierPiece], (t0, t1): (f64, f64)) -> ContinuousSegment {
    let held = Arc::new(bezier_pieces_to_nurbs(&[BezierPiece {
        u_start: t0,
        u_end: t1,
        coeffs: vec![0.0],
    }]));
    ContinuousSegment {
        axes: Arc::from([
            ContinuousAxis::Spline(Arc::new(bezier_pieces_to_nurbs(x))),
            ContinuousAxis::Spline(Arc::new(bezier_pieces_to_nurbs(y))),
            ContinuousAxis::Spline(Arc::clone(&held)),
            ContinuousAxis::Spline(held),
        ]),
        followers: Arc::from([]),
        spatial_path: false,
        t_start: t0,
        t_end: t1,
        motor_mask: 0,
        source_line: 1,
        rest_at_end: false,
    }
}

fn still(t0: f64, t1: f64) -> Vec<BezierPiece> {
    vec![BezierPiece {
        u_start: t0,
        u_end: t1,
        coeffs: vec![0.0],
    }]
}

/// A resting leg as shaping leaves it: the leaders brake along x into a
/// rest halfway through, speed falling as the square of the time left,
/// and leave it the same way.
fn braking_into_a_rest_and_leaving() -> ContinuousSegment {
    let half = REST_LEG_HALF_TIME_S;
    let (t0, t_rest, t1) = (0.0, half, 2.0 * half);
    let d = REST_LEG_SPILL_MM;
    let c = d / half.powi(3);
    let x = [
        BezierPiece {
            u_start: t0,
            u_end: t_rest,
            coeffs: vec![0.0, 3.0 * c * half * half, -3.0 * c * half, c],
        },
        BezierPiece {
            u_start: t_rest,
            u_end: t1,
            coeffs: vec![d, 0.0, 0.0, c],
        },
    ];
    leaders(&x, &still(t0, t1), (t0, t1))
}

/// Leaders turning around along x at `turn` under `TURN_ACCEL_MM_S2`, the
/// speed there no more than `drift` along y, the rounding a fit leaves of
/// an exact zero.
fn turning_around_with_drift(turn: f64, drift: f64) -> ContinuousSegment {
    let (t0, t1) = (0.0, 2.0 * turn);
    let x = [BezierPiece {
        u_start: t0,
        u_end: t1,
        coeffs: vec![
            0.5 * TURN_ACCEL_MM_S2 * turn * turn,
            -TURN_ACCEL_MM_S2 * turn,
            0.5 * TURN_ACCEL_MM_S2,
        ],
    }];
    let y = [BezierPiece {
        u_start: t0,
        u_end: t1,
        coeffs: vec![0.0, drift],
    }];
    leaders(&x, &y, (t0, t1))
}

const TURN_ACCEL_MM_S2: f64 = 20_000.0;

/// At a turnaround the leaders' speed is a zero's rounding, and so is the
/// direction it points: the speed leaves the zero at the whole
/// acceleration whichever way that rounding points.
#[test]
fn a_turnaround_crossing_reads_the_whole_acceleration_whatever_its_rounding() {
    let turn = 0.01;
    for drift in [1e-11, 3e-12] {
        let shaped = turning_around_with_drift(turn, drift);
        let crossing = Crossing::at(&ShapedLeaders::of(&shaped, &[0, 1]), turn);
        assert!(
            crossing.turns_around(),
            "a turnaround drifting {drift} mm/s along y crosses as {crossing:?}"
        );
        assert_eq!(
            crossing.rate, TURN_ACCEL_MM_S2,
            "a turnaround drifting {drift} mm/s along y crosses as {crossing:?}"
        );
    }
}

/// Leaders that start from a standstill at `t0` along x under a constant
/// jerk, covering `REST_LEG_SPILL_MM` by `t0 + duration`.
fn starting_from_standstill(t0: f64, duration: f64) -> ContinuousSegment {
    let t1 = t0 + duration;
    let x = [BezierPiece {
        u_start: t0,
        u_end: t1,
        coeffs: vec![0.0, 0.0, 0.0, REST_LEG_SPILL_MM / duration.powi(3)],
    }];
    leaders(&x, &still(t0, t1), (t0, t1))
}

/// Leaders that brake along x under a constant jerk into a standstill at
/// `t0 + duration`, covering `REST_LEG_SPILL_MM`.
fn braking_to_standstill(t0: f64, duration: f64) -> ContinuousSegment {
    let t1 = t0 + duration;
    let c = REST_LEG_SPILL_MM / duration.powi(3);
    let x = [BezierPiece {
        u_start: t0,
        u_end: t1,
        coeffs: vec![0.0, 3.0 * c * duration * duration, -3.0 * c * duration, c],
    }];
    leaders(&x, &still(t0, t1), (t0, t1))
}

fn moving_leg<'a>(
    shaped: &'a ContinuousSegment,
    shaped_leaders: &'a ShapedLeaders<'a>,
    start_s: f64,
) -> RampLeg<'a> {
    RampLeg {
        shaped,
        leaders: &[0, 1],
        shaped_leaders,
        support: (shaped.t_start, shaped.t_end),
        start_s,
        rests: false,
    }
}

/// The width field's time samples from `t0` on, as `(s − s0, t − t0)`.
fn samples_from(field: &WidthField, t0: f64, s0: f64) -> Vec<(f64, f64)> {
    field
        .candidates
        .iter()
        .filter_map(|candidate| match candidate {
            super::Candidate::Time(sample) if sample.t >= t0 => {
                Some((sample.s - s0, sample.t - t0))
            }
            _ => None,
        })
        .collect()
}

/// A full stop decouples planning: a leg the leaders open from a
/// standstill samples the width at the times it would be planned from rest,
/// wherever on the clock the stream before left them standing.
#[test]
fn a_leg_opening_from_standstill_samples_the_width_as_planned_from_rest() {
    let duration = 0.02;
    let from_rest = starting_from_standstill(0.0, duration);
    let from_rest_leaders = ShapedLeaders::of(&from_rest, &[0, 1]);
    let mut planned_from_rest = WidthField::default();
    planned_from_rest.extend(
        &from_rest_leaders,
        &moving_leg(&from_rest, &from_rest_leaders, 0.0),
        REST_LEG_SPILL_MM,
    );
    let stand_still_at = 0.0337;
    let before = braking_to_standstill(stand_still_at - duration, duration);
    let before_leaders = ShapedLeaders::of(&before, &[0, 1]);
    let after = starting_from_standstill(stand_still_at, duration);
    let after_leaders = ShapedLeaders::of(&after, &[0, 1]);
    let mut stream = WidthField::default();
    stream.extend(
        &before_leaders,
        &moving_leg(&before, &before_leaders, 0.0),
        REST_LEG_SPILL_MM,
    );
    stream.extend(
        &after_leaders,
        &moving_leg(&after, &after_leaders, REST_LEG_SPILL_MM),
        2.0 * REST_LEG_SPILL_MM,
    );
    let (streamed, from_rest) = (
        samples_from(&stream, stand_still_at, REST_LEG_SPILL_MM),
        samples_from(&planned_from_rest, 0.0, 0.0),
    );
    assert!(
        streamed.len() == from_rest.len()
            && streamed.iter().zip(&from_rest).all(|(a, b)| {
                (a.0 - b.0).abs() <= SAMPLE_ROUNDING && (a.1 - b.1).abs() <= SAMPLE_ROUNDING
            }),
        "after a standstill the stream samples the width at {streamed:?}, from rest at \
         {from_rest:?}"
    );
}

/// What clock offsets leave of two samples of one motion.
const SAMPLE_ROUNDING: f64 = 1e-12;

/// A resting leg steps the ratio from its tail to its lead where the
/// leaders' distance from the rest point puts the rest, which the odometer
/// can reach while they still creep toward it, short of where the valley
/// finds them resting. The width rests there all the same, so the ratio
/// step stays a step, and the valley's zero weight is all that keeps it
/// from the advance.
#[test]
fn a_rest_valley_covers_the_ratio_step_its_leg_lays_short_of_the_rest() {
    let travel = 10.0;
    let short_of_rest = 4e-7;
    let tail = REST_LEG_SPILL_MM - short_of_rest;
    let mut state = FollowerState::default();
    state.push_span(travel, 0.0, 0.0);
    state.push_span(tail, 0.0, 0.0);
    state.push_span(REST_LEG_SPILL_MM + short_of_rest, 0.03, 0.03);
    let shaped = braking_into_a_rest_and_leaving();
    let shaped_leaders = ShapedLeaders::of(&shaped, &[0, 1]);
    let leg = RampLeg {
        shaped: &shaped,
        leaders: &[0, 1],
        shaped_leaders: &shaped_leaders,
        support: (shaped.t_start, shaped.t_end),
        start_s: travel,
        rests: true,
    };
    let rule = RatioRampRule {
        advance_slope_s: 0.5,
        max_velocity_mm_s: 45.0,
        max_accel_mm_s2: 2000.0,
    };
    state.smoothing.lay(&state.spans, &leg, &rule);
    let ratio_step = travel + tail;
    let (weight, _, _) = state.smoothing.valley_weight(ratio_step);
    assert_eq!(
        weight, 0.0,
        "the advance reads the ratio step at {ratio_step} mm with weight {weight}"
    );
}

/// The leaders brake into a rest a rounding past the last time node: the
/// node stacks with the rest where the leaders stop, so the window closes
/// there however they move on after it.
#[test]
fn a_time_node_a_rounding_short_of_a_rest_stacks_on_the_rest() {
    let rest = 2.5;
    let short = 0.9 * super::REST_TARGET_MM;
    let mut field = WidthField::default();
    field.push_rest(0.0);
    field.push(1.0, 0.5);
    field.push(2.0, 0.5);
    field.push(rest - short, 0.5);
    field.push_rest(rest);
    let alone = field.width(rest);
    for s in [3.0, 3.5, 4.5] {
        field.push(s, 0.5);
    }
    field.push_rest(5.0);
    let moving_on = field.width(rest);
    assert!(
        alone == 0.0 && moving_on == 0.0,
        "the window at the rest at {rest} mm is {alone} mm while the leaders rest there and \
         {moving_on} mm once they move on"
    );
}

const BRAKING_SPEED_MM_S: f64 = 60.0;
const BRAKING_ACCEL_MM_S2: f64 = -2000.0;
const BRAKING_JERK_MM_S3: f64 = 1e5;
const BRAKING_TIME_S: f64 = 0.02;
const SLIVER_S: f64 = 1e-7;
const SLIVER_JERK_MM_S3: f64 = 1e7;

/// The leaders braking along x over `[0, BRAKING_TIME_S]` as a fit lays
/// them, closing on a sliver piece whose jerk is [`SLIVER_JERK_MM_S3`]
/// if `sliver`: the same motion to within the sliver's acceleration
/// change, as two fits of one shaped signal lay it.
fn braking_leaders(sliver: bool) -> ContinuousSegment {
    let motion = [
        0.0,
        BRAKING_SPEED_MM_S,
        0.5 * BRAKING_ACCEL_MM_S2,
        BRAKING_JERK_MM_S3 / 6.0,
    ];
    let mut pieces = vec![BezierPiece {
        u_start: 0.0,
        u_end: BRAKING_TIME_S,
        coeffs: motion.to_vec(),
    }];
    if sliver {
        let start = BRAKING_TIME_S - SLIVER_S;
        let at = |order: usize| -> f64 {
            motion
                .iter()
                .enumerate()
                .skip(order)
                .map(|(power, coefficient)| {
                    let falling = (power - order + 1..=power).product::<usize>() as f64;
                    coefficient * falling * start.powi((power - order) as i32)
                })
                .sum()
        };
        pieces[0].u_end = start;
        pieces.push(BezierPiece {
            u_start: start,
            u_end: BRAKING_TIME_S,
            coeffs: vec![at(0), at(1), 0.5 * at(2), SLIVER_JERK_MM_S3 / 6.0],
        });
    }
    let held = Arc::new(bezier_pieces_to_nurbs(&[BezierPiece {
        u_start: 0.0,
        u_end: BRAKING_TIME_S,
        coeffs: vec![0.0],
    }]));
    ContinuousSegment {
        axes: Arc::from([
            ContinuousAxis::Spline(Arc::new(bezier_pieces_to_nurbs(&pieces))),
            ContinuousAxis::Spline(Arc::clone(&held)),
            ContinuousAxis::Spline(Arc::clone(&held)),
            ContinuousAxis::Spline(held),
        ]),
        followers: Arc::from([]),
        spatial_path: true,
        t_start: 0.0,
        t_end: BRAKING_TIME_S,
        motor_mask: 0,
        source_line: 1,
        rest_at_end: false,
    }
}

/// Two fits of one braking, one closing on a sliver piece of a jerk a
/// hundred times the leaders' own, meet a seam with the same jerk: within
/// twice the sliver's acceleration change over a probe interval.
#[test]
fn a_sliver_piece_of_the_leader_fit_leaves_the_crossing_jerk_alone() {
    let seam = BRAKING_TIME_S - CLUSTER_REACH_S;
    let jerk = |sliver: bool| {
        let leaders = braking_leaders(sliver);
        Crossing::across(&ShapedLeaders::of(&leaders, &[0, 1]), seam, BRAKING_TIME_S).1
    };
    let probe_interval = CLUSTER_REACH_S / CROSSING_PROBES as f64;
    let bound = 2.0 * SLIVER_JERK_MM_S3 * SLIVER_S / probe_interval;
    let (clean, with_sliver) = (jerk(false), jerk(true));
    assert!(
        (clean - BRAKING_JERK_MM_S3).abs() <= 1e-6 * BRAKING_JERK_MM_S3,
        "the crossing reads a jerk of {clean} mm/s³ off a braking of {BRAKING_JERK_MM_S3}"
    );
    assert!(
        (with_sliver - clean).abs() <= bound,
        "a sliver piece moves the crossing's jerk from {clean} to {with_sliver} mm/s³, past \
         {bound}"
    );
}
