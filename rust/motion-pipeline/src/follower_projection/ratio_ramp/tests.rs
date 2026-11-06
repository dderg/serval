use std::sync::Arc;

use geometry::{RATIO_RAMP_MAX_TIME_S, RATIO_RAMP_MIN_TIME_S, RatioRampRule, RatioStep};
use nurbs::bezier::{BezierPiece, bezier_pieces_to_nurbs};
use proptest::prelude::*;
use proptest::test_runner::FileFailurePersistence;
use trajectory::{ContinuousAxis, ContinuousSegment};

use super::super::{
    FollowerState, OdometerLeaves, SpanSide, Table as SpanTable, shaped_leader_arc_length,
};
use super::{
    AdvanceValley, CLUSTER_REACH_S, CROSSING_PROBES, Crossing, ENVELOPE_SLOPE, KERNEL_PEAK,
    NODE_SPACING_PER_WIDTH, NODE_SPACING_S, PathReach, RampLeg, RatioSmoothing, RestSide,
    ShapedLeaders, Spill, TimedSeam, ValleyKind, WIDTH_SLOPE_BOUND, WidthField, slowest_crawl,
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

/// An extrusion into a wipe, the wipe's ratio ramping back through zero
/// into an extrusion, and a travel.
fn retracting_table() -> Table {
    vec![
        (10.0, 0.0, 0.0),
        (10.0, 0.04, 0.04),
        (10.0, -0.05, -0.05),
        (10.0, -0.05, 0.03),
        (10.0, 0.03, 0.03),
        (10.0, 0.0, 0.0),
    ]
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
    state
        .smoothing
        .record_seams((&state.spans, &state.advance_spans));
    state
}

fn probes(length: f64, count: usize) -> impl Iterator<Item = f64> {
    (1..count).map(move |k| length * k as f64 / count as f64 + 1e-7)
}

/// The stretch of the table the smoothing averages at `s`: the moving
/// window there, and every cluster of seams it or its own window reaches,
/// with both sides of its seams.
fn averaged_stretch(state: &FollowerState, s: f64) -> (f64, f64) {
    let h = state.smoothing.width.width(s);
    state
        .smoothing
        .laid
        .clusters_reaching(&state.smoothing.width, (s - h, s + h))
        .fold((s - h, s + h), |(lo, hi), cluster| {
            (
                lo.min(trajectory::next_toward(cluster.first(), f64::NEG_INFINITY)),
                hi.max(trajectory::next_toward(cluster.last().s, f64::INFINITY)),
            )
        })
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
                let (whole_ratio, whole_slope) = whole.ratio_and_slope(s, SpanSide::After);
                let (cut_ratio, cut_slope) = pieces.ratio_and_slope(s, SpanSide::After);
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
                let (lo, hi) = averaged_stretch(&state, s);
                let (low, high) = table_range(&state, lo, hi);
                let (ratio, _) = state.ratio_and_slope(s, SpanSide::After);
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
                let (lo, hi) = averaged_stretch(&state, s);
                let (low, high) = table_range(&state, lo, hi);
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
                let (left_ratio, left_slope) = state.ratio_and_slope(s - eps, SpanSide::After);
                let (right_ratio, right_slope) = state.ratio_and_slope(s + eps, SpanSide::After);
                let (_, slope_on) = state.ratio_and_slope(s, SpanSide::After);
                assert!(
                    (left_ratio - right_ratio).abs() < 1e-9,
                    "the ratio jumps at {s} mm: {left_ratio} -> {right_ratio}"
                );
                assert!(
                    (left_slope - right_slope).abs() < 1e-5,
                    "the ratio's slope jumps at {s} mm: {left_slope} -> {right_slope}"
                );
                assert!(
                    (slope_on - right_slope).abs() < 1e-5,
                    "the ratio's slope on {s} mm is {slope_on}, beside it {right_slope}"
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
fn the_advance_reads_the_laid_ratio_held_at_zero_where_it_retracts() {
    let table = retracting_table();
    let total = length(&table);
    for field in [cruise, cornering] {
        let state = smoothed(&table, field(total));
        let (mut extruding, mut retracting) = (0, 0);
        for s in probes(total, 20_000) {
            let (lo, hi) = averaged_stretch(&state, s);
            let (low, high) = table_range(&state, lo, hi);
            let (read, _) = state.ratio_and_slope_on(SpanTable::Advance, s, SpanSide::After);
            assert!(
                read >= -1e-12 && read <= high.max(0.0) + 1e-12,
                "the advance reads {read} at {s} mm, outside [0, {}]",
                high.max(0.0)
            );
            if low >= 0.0 {
                extruding += 1;
                let (laid, _) = state.ratio_and_slope(s, SpanSide::After);
                assert_eq!(
                    read, laid,
                    "the advance reads {read} at {s} mm where the window extrudes {laid}"
                );
            } else if high <= 0.0 {
                retracting += 1;
                assert!(
                    read.abs() <= 1e-12,
                    "the advance reads {read} at {s} mm where the window only retracts"
                );
            }
        }
        assert!(
            extruding > 1000 && retracting > 1000,
            "the probes must cover both, yet {extruding} extrude and {retracting} retract"
        );
    }
}

#[test]
fn the_ratio_the_advance_reads_is_c1_through_a_wipe() {
    let table = retracting_table();
    let total = length(&table);
    for field in [cruise, cornering] {
        let state = smoothed(&table, field(total));
        let mut points: Vec<f64> = state.advance_spans.iter().map(|span| span.s0).collect();
        points.extend(state.smoothing.bends(0.0, total));
        points.retain(|&s| s > 1e-3 && s < total - 1e-3);
        assert!(
            state.advance_spans.len() > state.spans.len(),
            "the ramp through zero must split the advance's table"
        );
        let eps = 1e-9;
        let read = |s: f64| state.ratio_and_slope_on(SpanTable::Advance, s, SpanSide::After);
        for &s in &points {
            let ((left_ratio, left_slope), (right_ratio, right_slope)) =
                (read(s - eps), read(s + eps));
            assert!(
                (left_ratio - right_ratio).abs() < 1e-9,
                "the read ratio jumps at {s} mm: {left_ratio} -> {right_ratio}"
            );
            assert!(
                (left_slope - right_slope).abs() < 1e-5,
                "the read ratio's slope jumps at {s} mm: {left_slope} -> {right_slope}"
            );
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
        .laid
        .seams
        .iter()
        .map(|seam| seam.step.abs())
        .sum();
    let steepest = probes(total, 50_000)
        .map(|s| state.ratio_and_slope(s, SpanSide::After).1.abs())
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
                let (ratio, slope) = state.ratio_and_slope(s, SpanSide::After);
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
/// it rides there is bounded like any other. No window reaches a seam from
/// a rest, so the stretch bends nowhere.
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
    let (ratio, slope) = state.ratio_and_slope(0.0, SpanSide::After);
    assert!(ratio.abs() <= ratio_bound && slope.abs() <= slope_bound);
    assert!(!state.smoothing.touches(0.0, 0.0));
    assert!(state.smoothing.bends(0.0, 0.0).is_empty());
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

/// #773: off a rest a sample becomes a node only once that rest's cone no
/// longer caps its target, so on a motion too short for its targets the
/// only node kept can lie micrometres short of the rest it ends in, whose
/// cone then crushes it to a sliver every window along the motion
/// flattens onto. The motion opens the window halfway along it instead,
/// the widest the two rests' cones allow its target.
#[test]
fn a_motion_whose_nodes_the_closing_rest_crushes_opens_its_window_halfway() {
    let (end, target) = (3.5995653532023386, 0.856049567919998);
    let mut field = WidthField::default();
    field.push_rest(0.0);
    field.push(3.5995408790997963, target);
    field.push_rest(end);
    let halfway = 0.5 * end;
    assert_eq!(
        field.width(halfway),
        target.min(ENVELOPE_SLOPE * halfway),
        "the motion's window halfway along it"
    );
    let seam = 0.1135885147517546;
    assert!(
        field.width(seam) >= 0.5 * ENVELOPE_SLOPE * seam,
        "a {} mm window at the ratio step {seam} mm off the rest",
        field.width(seam)
    );
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
        let crossing = Crossing::valley_at(&ShapedLeaders::of(&shaped, &[0, 1]), turn);
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
    (shaped_leaders, odometer): (&'a ShapedLeaders<'a>, &'a OdometerLeaves),
    start_s: f64,
) -> RampLeg<'a> {
    RampLeg {
        shaped,
        leaders: &[0, 1],
        shaped_leaders,
        odometer,
        support: (shaped.t_start, shaped.t_end),
        start_s,
        rests: false,
    }
}

/// The odometer ingest sums the arc of the leaders `shaped_leaders` read
/// off `shaped` with.
fn odometer(shaped: &ContinuousSegment, shaped_leaders: &ShapedLeaders) -> OdometerLeaves {
    shaped_leader_arc_length(shaped, &[0, 1], shaped_leaders).1
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
    let from_rest_odometer = odometer(&from_rest, &from_rest_leaders);
    let mut planned_from_rest = WidthField::default();
    planned_from_rest.extend(
        &from_rest_leaders,
        &moving_leg(&from_rest, (&from_rest_leaders, &from_rest_odometer), 0.0),
        REST_LEG_SPILL_MM,
    );
    let stand_still_at = 0.0337;
    let before = braking_to_standstill(stand_still_at - duration, duration);
    let before_leaders = ShapedLeaders::of(&before, &[0, 1]);
    let before_odometer = odometer(&before, &before_leaders);
    let after = starting_from_standstill(stand_still_at, duration);
    let after_leaders = ShapedLeaders::of(&after, &[0, 1]);
    let after_odometer = odometer(&after, &after_leaders);
    let mut stream = WidthField::default();
    stream.extend(
        &before_leaders,
        &moving_leg(&before, (&before_leaders, &before_odometer), 0.0),
        REST_LEG_SPILL_MM,
    );
    stream.extend(
        &after_leaders,
        &moving_leg(&after, (&after_leaders, &after_odometer), REST_LEG_SPILL_MM),
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
    let shaped_odometer = odometer(&shaped, &shaped_leaders);
    let leg = RampLeg {
        shaped: &shaped,
        leaders: &[0, 1],
        shaped_leaders: &shaped_leaders,
        odometer: &shaped_odometer,
        support: (shaped.t_start, shaped.t_end),
        start_s: travel,
        rests: true,
    };
    let rule = RatioRampRule {
        advance_slope_s: 0.5,
        max_velocity_mm_s: 45.0,
        max_accel_mm_s2: 2000.0,
        kernel_jerk_per_accel_hz: f64::INFINITY,
    };
    state
        .smoothing
        .lay((&state.spans, &state.advance_spans), &leg, &rule);
    let ratio_step = travel + tail;
    let (weight, _, _) = state.smoothing.valley_weight(ratio_step);
    assert_eq!(
        weight, 0.0,
        "the advance reads the ratio step at {ratio_step} mm with weight {weight}"
    );
}

/// How far the odometer's table of a leg can run past the leg's quadrature
/// of its leaders' speed: 13 ulps at 921 mm, as `voron_dense_top_layers_zhop`
/// on the printer config laid a stop into a retract.
const LEG_TABLE_PAST_QUADRATURE_MM: f64 = 1.5e-12;

/// Leaders resting through a rest leg's end rest up to the seam the next
/// leg opens on, wherever the leg's quadrature of their path rounds short
/// of the table. A valley closing short of the seam would ramp the advance
/// back in over that rounding alone, under the leaders creeping to their
/// stop, where one ulp of path steps the weight the advance reads by a
/// fifth.
#[test]
fn leaders_resting_through_a_legs_end_rest_up_to_its_seam() {
    let travel = 10.0;
    let duration = 2.0 * REST_LEG_HALF_TIME_S;
    let rest_leg_table_mm = REST_LEG_SPILL_MM + LEG_TABLE_PAST_QUADRATURE_MM;
    let rule = RatioRampRule {
        advance_slope_s: 0.5,
        max_velocity_mm_s: 45.0,
        max_accel_mm_s2: 2000.0,
        kernel_jerk_per_accel_hz: f64::INFINITY,
    };
    let mut state = FollowerState::default();
    state.push_span(travel, 0.03, 0.03);
    state.push_span(rest_leg_table_mm, 0.03, 0.03);
    let seam = state.s_ingested_end;
    let stopping = braking_to_standstill(0.0, duration);
    let stopping_leaders = ShapedLeaders::of(&stopping, &[0, 1]);
    let stopping_odometer = odometer(&stopping, &stopping_leaders);
    let stop = RampLeg {
        rests: true,
        ..moving_leg(&stopping, (&stopping_leaders, &stopping_odometer), travel)
    };
    state
        .smoothing
        .lay((&state.spans, &state.advance_spans), &stop, &rule);
    state.push_span(REST_LEG_SPILL_MM, 0.03, 0.03);
    let leaving = starting_from_standstill(duration, duration);
    let leaving_leaders = ShapedLeaders::of(&leaving, &[0, 1]);
    let leaving_odometer = odometer(&leaving, &leaving_leaders);
    state.smoothing.lay(
        (&state.spans, &state.advance_spans),
        &moving_leg(&leaving, (&leaving_leaders, &leaving_odometer), seam),
        &rule,
    );
    for s in [
        seam - 0.5 * LEG_TABLE_PAST_QUADRATURE_MM,
        trajectory::next_toward(seam, f64::NEG_INFINITY),
    ] {
        let (weight, _, _) = state.smoothing.valley_weight(s);
        assert_eq!(
            weight, 0.0,
            "the advance reads the rest leg at {s} mm, before the seam at {seam} mm, with \
             weight {weight}"
        );
    }
}

/// The odometer and the path the leaders cover reach a resting leg's ratio
/// step to different roundings, the step a few dozen ulps short of the
/// rest or past it. The width rests on the step itself, so no window
/// straddles it however close past it: one that did would smooth the
/// material over picometres and read its step as a curvature that no
/// enclosure of the follower holds.
#[test]
fn the_width_rests_on_the_ratio_step_a_resting_leg_lays_beside_the_rest() {
    let travel = 10.0;
    for off_rest in [-4e-7, 2e-11] {
        let tail = REST_LEG_SPILL_MM + off_rest;
        let mut state = FollowerState::default();
        state.push_span(travel, 0.0, 0.0);
        state.push_span(tail, 0.0, 0.0);
        state.push_span(REST_LEG_SPILL_MM - off_rest, 0.03, 0.03);
        let shaped = braking_into_a_rest_and_leaving();
        let shaped_leaders = ShapedLeaders::of(&shaped, &[0, 1]);
        let shaped_odometer = odometer(&shaped, &shaped_leaders);
        let leg = RampLeg {
            shaped: &shaped,
            leaders: &[0, 1],
            shaped_leaders: &shaped_leaders,
            odometer: &shaped_odometer,
            support: (shaped.t_start, shaped.t_end),
            start_s: travel,
            rests: true,
        };
        let rule = RatioRampRule {
            advance_slope_s: 0.5,
            max_velocity_mm_s: 45.0,
            max_accel_mm_s2: 2000.0,
            kernel_jerk_per_accel_hz: f64::INFINITY,
        };
        state
            .smoothing
            .lay((&state.spans, &state.advance_spans), &leg, &rule);
        state.smoothing.close_at(travel + 2.0 * REST_LEG_SPILL_MM);
        let ratio_step = travel + tail;
        let width = &state.smoothing.width;
        assert_eq!(
            width.width(ratio_step),
            0.0,
            "a ratio step {off_rest} mm off the rest at {ratio_step} mm"
        );
        for past in [1e-13, 1e-12, 1e-11, 1e-9, 1e-7] {
            let s = ratio_step + past;
            assert!(
                width.width(s) < past,
                "a {} mm window {past} mm past the ratio step {off_rest} mm off the rest \
                 straddles it",
                width.width(s)
            );
        }
    }
}

/// Leaving a rest, the leaders creep a rounding's worth before they move:
/// the first time sample past it lies micrometres off the rest, where its
/// cone caps the window to a sliver. Kept as a node, the smootherstep to
/// the next flattened every window up to that node onto the sliver; the
/// width rises off the rest to the first node the cone does not cap
/// instead, no narrower than the cone along the way.
#[test]
fn the_width_leaving_a_rest_rises_with_the_rest_cone() {
    let creep = 1.83e-6;
    let path_per_sample = 0.25;
    let mut field = WidthField::default();
    field.place_rest(0.0);
    field.place_sample(0.0, 0.0);
    let mut s = creep;
    for tick in 1..40 {
        field.place_sample(s, tick as f64 * NODE_SPACING_S);
        s += path_per_sample;
    }
    field.close(s);
    for d in [1e-5, 1e-4, 1e-3, 0.028, 0.1, 0.5] {
        let width = field.width(d);
        assert!(
            width >= ENVELOPE_SLOPE * d,
            "a {width} mm window {d} mm past the rest, under its {} mm cone",
            ENVELOPE_SLOPE * d
        );
    }
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
        Crossing::over(
            &ShapedLeaders::of(&leaders, &[0, 1]),
            (seam, BRAKING_TIME_S),
        )
        .jerk
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
fn model_at(model: &crate::shaper::taylor::TaylorModel, x: f64) -> f64 {
    model
        .coefficients()
        .iter()
        .rev()
        .fold(0.0, |acc, c| acc * x + c)
}

/// Over every stretch of path between the smoothing's bends and the
/// table's seams, its correction and the advance's valley weight read on a
/// model of the path hold every reading the evaluator makes there, with
/// its derivatives.
#[test]
fn the_smoothing_models_hold_its_correction_and_valley_weight_between_bends() {
    use crate::shaper::taylor::{Ball, TaylorModel};
    let mut read = 0usize;
    for table in [line_table(), clustered_table()] {
        for field in [cruise, cornering] {
            let total = length(&table);
            let mut state = smoothed(&table, field(total));
            state.smoothing.valleys = vec![
                super::AdvanceValley {
                    start: 0.3 * total,
                    end: 0.3 * total,
                    before: 1.5,
                    after: 2.0,
                    floor_before: 0.3,
                    floor_after: 0.3,
                    kind: super::ValleyKind::Crawl,
                },
                super::AdvanceValley {
                    start: 0.6 * total,
                    end: 0.6 * total + 1.0,
                    before: 0.8,
                    after: 0.5,
                    floor_before: 0.0,
                    floor_after: 0.0,
                    kind: super::ValleyKind::Rest,
                },
            ];
            let mut points: Vec<f64> = state.spans.iter().map(|span| span.s0).collect();
            points.extend(state.smoothing.bends(0.0, total));
            points.extend([0.0, total]);
            points.retain(|&s| (0.0..=total).contains(&s));
            points.sort_by(f64::total_cmp);
            points.dedup();
            for stretch in points.windows(2).filter(|pair| pair[1] - pair[0] > 1e-9) {
                for part in 0..4 {
                    let width = (stretch[1] - stretch[0]) / 4.0;
                    let (lo, hi) = (
                        stretch[0] + width * f64::from(part),
                        stretch[0] + width * f64::from(part + 1),
                    );
                    let (centre, radius) = (0.5 * (lo + hi), 0.5 * (hi - lo));
                    let s = TaylorModel::affine(Ball::exact(centre), radius);
                    let reach = PathReach {
                        lo: trajectory::next_toward(lo, f64::INFINITY),
                        hi: trajectory::next_toward(hi, f64::NEG_INFINITY),
                        exact: (false, false),
                    };
                    let correction = state
                        .smoothing
                        .correction_model(SpanTable::Laid, &s, reach)
                        .expect("a stretch between bends reads one formula");
                    let weight = state
                        .smoothing
                        .valley_weight_model(&s, reach)
                        .expect("a stretch between bends lies on one side of every valley")
                        .models();
                    read += 1;
                    for k in 1..8 {
                        let at = lo + (hi - lo) * f64::from(k) / 8.0;
                        let x = (at - centre) / radius;
                        let jet = state.smoothing.correction(at);
                        let (w, w1, w2) = state.smoothing.valley_weight(at);
                        for (name, truth, model) in [
                            ("value", jet.value, &correction.value),
                            ("d1", jet.d1, &correction.d1),
                            ("d2", jet.d2, &correction.d2),
                            ("d3", jet.d3, &correction.d3),
                            ("W", w, &weight[0]),
                            ("W'", w1, &weight[1]),
                            ("W''", w2, &weight[2]),
                        ] {
                            let miss = (truth - model_at(model, x)).abs();
                            assert!(
                                miss <= model.rad() + 1e-9 * (1.0 + truth.abs()),
                                "{name} at {at} mm on [{lo}, {hi}]: the evaluator reads {truth}, \
                                 the model misses it by {miss:e} past its bound {:e}",
                                model.rad()
                            );
                        }
                    }
                }
            }
        }
    }
    assert!(read > 100, "too few stretches read: {read}");
}
const CORNER_SPEED_MM_S: f64 = 40.0;
const CORNER_CRAWL_MM_S: f64 = 4.0;
const CORNER_BRAKE_MM_S2: f64 = 6400.0;

/// The leaders' path and speed at `t`: a cruise at `CORNER_SPEED_MM_S`
/// braking at `brake` into a corner they cross at `CORNER_CRAWL_MM_S`, then
/// speeding up the same way, as a slicer's short travel between two
/// extrusions plans.
fn slow_corner(brake: f64, t: f64) -> f64 {
    let ramp = (CORNER_SPEED_MM_S - CORNER_CRAWL_MM_S) / CORNER_BRAKE_MM_S2;
    let covered = |speed_at: &dyn Fn(f64) -> f64, from: f64, to: f64| {
        let steps = 64;
        (0..steps)
            .map(|k| {
                let at = from + (to - from) * (k as f64 + 0.5) / steps as f64;
                speed_at(at) * (to - from) / steps as f64
            })
            .sum::<f64>()
    };
    let speed = |t: f64| {
        let off = (t - brake - ramp).abs();
        if off >= ramp {
            CORNER_SPEED_MM_S
        } else {
            CORNER_CRAWL_MM_S + CORNER_BRAKE_MM_S2 * off
        }
    };
    covered(&speed, 0.0, t)
}

/// A ratio step crossed at speed just ahead of a slow corner asks for a
/// window `V·τ/2` wide, `V` the speed the leaders cross it at. The corner's
/// own target is a fraction of that, and its cone would narrow the window
/// right where the leaders still cruise into it: the seam's width holds the
/// window open wherever the seam is inside it.
#[test]
fn a_slow_corner_beside_a_ratio_step_does_not_narrow_its_window() {
    let rule = RatioRampRule {
        advance_slope_s: 0.02,
        max_velocity_mm_s: 50.0,
        max_accel_mm_s2: 5000.0,
        kernel_jerk_per_accel_hz: f64::INFINITY,
    };
    let (seam_t, brake, end) = (0.052, 0.05, 0.12);
    let mut field = WidthField {
        rule: Some(rule),
        ..WidthField::default()
    };
    field.place_rest(0.0);
    let seam_s = slow_corner(brake, seam_t);
    field.add_seam(TimedSeam {
        t: seam_t,
        s: seam_s,
        crossing: Crossing {
            speed: CORNER_SPEED_MM_S,
            rate: CORNER_BRAKE_MM_S2,
            jerk: 0.0,
        },
        step: 0.034,
        peak_ratio: 0.034,
        ramp_time: None,
    });
    let seam_width = 0.5 * CORNER_SPEED_MM_S * field.seam_ramp_time(0);
    let mut t = NODE_SPACING_S;
    while t < end {
        field.place_sample(slow_corner(brake, t), t);
        t += NODE_SPACING_S;
    }
    field.close(slow_corner(brake, end));
    let corner = slow_corner(
        brake,
        brake + (CORNER_SPEED_MM_S - CORNER_CRAWL_MM_S) / CORNER_BRAKE_MM_S2,
    );
    assert!(
        corner - seam_s < seam_width,
        "the corner at {corner} mm lies beyond the {seam_width} mm window of the seam at \
         {seam_s} mm"
    );
    let narrowest = probes(2.0 * seam_width, 400)
        .map(|offset| seam_s - seam_width + offset)
        .map(|s| (field.width(s), s))
        .fold((f64::INFINITY, 0.0), |narrowest, probe| {
            if probe.0 < narrowest.0 {
                probe
            } else {
                narrowest
            }
        });
    assert!(
        narrowest.0 >= seam_width * (1.0 - 1e-12),
        "the window at {} mm is {} mm wide, under the {seam_width} mm the seam at {seam_s} mm \
         asks for",
        narrowest.1,
        narrowest.0
    );
}

/// The rule bounds a ramp whose window is `V·τ/2` wide on every window
/// that reaches its seam: a width that changes along them steepens the
/// ramp's ends by its slope, which on a 300 mm/s cruise of a 20 000 mm/s²
/// extruder left 561 mm/s² over the limit. Wherever the time grid falls
/// against the seam, the width holds `V·τ/2` exactly from one width before
/// the seam to one width past it.
#[test]
fn a_ratio_step_crossed_at_cruise_holds_its_width_over_every_window_reaching_it() {
    let end = 0.3;
    for speed in [150.0, 300.0] {
        for phase in probes(NODE_SPACING_S, 16) {
            let seam_t = 0.1 + phase;
            let seam_s = speed * seam_t;
            let mut field = WidthField {
                rule: Some(DIRECT_DRIVE),
                ..WidthField::default()
            };
            field.place_rest(0.0);
            field.add_seam(TimedSeam {
                t: seam_t,
                s: seam_s,
                crossing: Crossing {
                    speed,
                    rate: 0.0,
                    jerk: 0.0,
                },
                step: LINE_RATIO,
                peak_ratio: LINE_RATIO,
                ramp_time: None,
            });
            let seam_width = 0.5 * speed * field.seam_ramp_time(0);
            let mut t = NODE_SPACING_S;
            while t < end {
                field.place_sample(speed * t, t);
                t += NODE_SPACING_S;
            }
            field.close(speed * end);
            for s in probes(2.0 * seam_width, 400).map(|offset| seam_s - seam_width + offset) {
                let width = field.width(s);
                assert!(
                    (width - seam_width).abs() <= 1e-12 * seam_width,
                    "at {speed} mm/s the window at {s} mm is {width} mm wide, not the \
                     {seam_width} mm the seam at {seam_s} mm asks for"
                );
            }
        }
    }
}

/// Moving width nodes lie most of the narrower of their targets apart on
/// the path, so the smootherstep between two of them bends the width no
/// sharper than a window's own width allows.
#[test]
fn moving_width_nodes_lie_a_window_apart() {
    let mut field = WidthField {
        rule: Some(RatioRampRule {
            advance_slope_s: 0.02,
            max_velocity_mm_s: 50.0,
            max_accel_mm_s2: 5000.0,
            kernel_jerk_per_accel_hz: f64::INFINITY,
        }),
        ..WidthField::default()
    };
    field.place_rest(0.0);
    let brake = 0.05;
    let mut t = NODE_SPACING_S;
    while t < 0.12 {
        field.place_sample(slow_corner(brake, t), t);
        t += NODE_SPACING_S;
    }
    field.close(slow_corner(brake, 0.12));
    assert_moving_nodes_lie_a_window_apart(&field);
}

/// Leaders creep off a rest under a constant jerk, a ratio step just past
/// it: the grid's tick on the rest point takes the seam's wide target, and
/// the next tick, a few hundredths of a micron along, is no window apart
/// from it. Kept, it would take the rest's cone as a sliver of an envelope
/// that the smootherstep to the next node flattens onto.
#[test]
fn a_stream_creeping_off_a_rest_keeps_no_node_a_sliver_past_it() {
    let mut field = WidthField {
        rule: Some(RatioRampRule {
            advance_slope_s: 0.02,
            max_velocity_mm_s: 50.0,
            max_accel_mm_s2: 5000.0,
            kernel_jerk_per_accel_hz: f64::INFINITY,
        }),
        ..WidthField::default()
    };
    let creep = |t: f64| CREEP_JERK_MM_S3 * t.powi(3);
    let (seam_t, end) = (2.0 * NODE_SPACING_S, 0.06);
    field.place_rest(0.0);
    field.add_seam(TimedSeam {
        t: seam_t,
        s: creep(seam_t),
        crossing: Crossing {
            speed: CORNER_SPEED_MM_S,
            rate: CORNER_BRAKE_MM_S2,
            jerk: 0.0,
        },
        step: 0.034,
        peak_ratio: 0.034,
        ramp_time: None,
    });
    let mut t = 0.0;
    while t < end {
        field.place_sample(creep(t), t);
        t += NODE_SPACING_S;
    }
    field.close(creep(end));
    assert_moving_nodes_lie_a_window_apart(&field);
}

const CREEP_JERK_MM_S3: f64 = 1500.0;

fn assert_moving_nodes_lie_a_window_apart(field: &WidthField) {
    for pair in field.nodes.windows(2) {
        let (a, b) = (pair[0], pair[1]);
        if a.target > 0.0 && b.target > 0.0 {
            assert!(
                b.s - a.s >= NODE_SPACING_PER_WIDTH * a.target.min(b.target),
                "width nodes at {} and {} mm lie closer than their targets {} and {}",
                a.s,
                b.s,
                a.target,
                b.target
            );
        }
    }
}

/// A corner blend tens of nanometres long ramps the table's ratio at
/// millions per millimetre. Summed seam by seam, its two slope steps cancel
/// only to an ulp of `m·h²`, and where the leaders crawl through a wide
/// window the laid track jittered by 1e-10 mm, more than a source fit
/// split down to nanoseconds can follow.
#[test]
fn a_nanometre_blend_lays_a_smooth_track_through_a_wide_window() {
    let blend_start = 10.0;
    let table = vec![
        (blend_start, 0.0, 0.0),
        (2.7e-8, 0.0, 0.085),
        (10.0, 0.085, 0.085),
    ];
    let total = length(&table);
    let state = smoothed(&table, cruise(total));
    let h = state.smoothing.width.width(blend_start);
    let step = 1e-9;
    for centre in [-h, -0.5 * h, 0.5 * h, h].map(|offset| blend_start + offset) {
        for k in 0..2_000 {
            let s = centre - 1e-6 + step * f64::from(k);
            let (a, b) = (
                state.smoothing.correction(s),
                state.smoothing.correction(s + step),
            );
            let drift = b.value - a.value - 0.5 * step * (a.d1 + b.d1);
            assert!(
                drift.abs() < 1e-14,
                "the laid track drifts {drift} mm off its own ratio over {step} mm at {s} mm"
            );
        }
    }
}

/// The leaders cross a crawl moving, so its valley leaves zero flat over
/// its floor: `W'·ṡ²` of opposite signs either side would step the advanced
/// velocity there. With no floor the valley is `ψ`, kinked, as at a
/// turnaround, where the leaders' zero speed makes the slope harmless.
#[test]
fn a_crawl_valley_leaves_zero_flat_and_its_derivatives_are_exact() {
    let crawl = AdvanceValley {
        start: 5.0,
        end: 5.0,
        before: 2.0,
        after: 1.5,
        floor_before: 0.01,
        floor_after: 0.01,
        kind: ValleyKind::Crawl,
    };
    for side in [-1.0, 1.0] {
        let (weight, slope, _) = crawl.weight(5.0 + side * 1e-9);
        assert!(
            weight < 1e-12 && slope.abs() < 1e-6,
            "the crawl valley leaves zero at {weight} with slope {slope}"
        );
    }
    let h = 1e-6;
    for k in 1..4_000 {
        let s = 2.9 + 3.7 * f64::from(k) / 4_000.0;
        if (s - 5.0).abs() < 4.0 * h {
            continue;
        }
        let (weight, slope, curvature) = crawl.weight(s);
        let (before, after) = (crawl.weight(s - h), crawl.weight(s + h));
        assert!(
            (0.0..=1.0).contains(&weight),
            "the crawl valley weighs {weight} at {s} mm"
        );
        for (order, value, numeric) in [
            (1, slope, (after.0 - before.0) / (2.0 * h)),
            (2, curvature, (after.1 - before.1) / (2.0 * h)),
        ] {
            assert!(
                (value - numeric).abs() < 1e-5 * (1.0 + numeric.abs()),
                "derivative {order} of the crawl valley at {s} mm is {value}, its \
                 difference quotient {numeric}"
            );
        }
    }
    let kinked = AdvanceValley {
        floor_before: 0.0,
        floor_after: 0.0,
        ..crawl
    };
    let (_, slope, _) = kinked.weight(5.0 + 1e-9);
    assert!(
        (slope - 3.0 / 1.5).abs() < 1e-6,
        "a valley without a floor leaves zero with slope {slope}"
    );
}

/// A crawl behind a kernel the turning guard does not hold is worth a
/// valley only where an advance reading the path speed through it would
/// leave the follower's acceleration limit: there the kernel bounds the
/// advance's jerk, and releasing the lead over the crawl's reach asks more
/// than the crawl. A follower the guard holds lays one at every crawl,
/// whose turning the guard bounds only down to the valley speed. A
/// turnaround releases it whatever the advance asks.
#[test]
fn a_crawl_lays_a_valley_where_the_guard_holds_the_follower_or_the_advance_would_leave_the_limit() {
    let guarded = RatioRampRule {
        advance_slope_s: 0.5,
        max_velocity_mm_s: 50.0,
        max_accel_mm_s2: 2000.0,
        kernel_jerk_per_accel_hz: f64::INFINITY,
    };
    let behind_a_kernel = RatioRampRule {
        kernel_jerk_per_accel_hz: 100.0,
        ..guarded
    };
    let crawl = Crossing {
        speed: 0.647,
        rate: 119.0,
        jerk: 119.0 * 119.0 / 0.647,
    };
    assert!(crawl.needs_valley(&guarded, 0.0337));
    assert!(!crawl.needs_valley(&behind_a_kernel, 0.2));
    assert!(crawl.needs_valley(&behind_a_kernel, 0.5));
    let cruise = Crossing {
        speed: 2.5,
        ..crawl
    };
    assert!(!cruise.needs_valley(&guarded, 0.5));
    assert!(!cruise.needs_valley(&behind_a_kernel, 0.5));
    let turnaround = Crossing {
        speed: 0.0,
        rate: 119.0,
        jerk: 0.0,
    };
    let no_advance = RatioRampRule {
        advance_slope_s: 0.0,
        ..guarded
    };
    assert!(turnaround.needs_valley(&no_advance, 0.0337));
}

/// The P1 bowden kernel crawl of `voron_first_layers` at t≈63.92 s: read
/// through the path speed, `r·(a + k·s⃛)` asks for 30000 mm/s² of the
/// extruder's 5000, but the 40 ms triangle after the advance meets
/// `s̈`'s slope only up to its total variation of 100 Hz times `a`,
/// `r·a·(1 + 100·k)` ≈ 3700 mm/s². Releasing the lead there drove the
/// extruder to 49.4 mm/s against its 45.
#[test]
fn a_kernel_after_the_advance_spares_a_crawl_its_valley() {
    let kernel_free = RatioRampRule {
        advance_slope_s: 0.5,
        max_velocity_mm_s: 45.0,
        max_accel_mm_s2: 5000.0,
        kernel_jerk_per_accel_hz: f64::INFINITY,
    };
    let crawl = Crossing {
        speed: 0.705,
        rate: 2156.0,
        jerk: 1.74e6,
    };
    let ratio = 0.034;
    assert!(crawl.needs_valley(&kernel_free, ratio));
    let smoothed = RatioRampRule {
        kernel_jerk_per_accel_hz: 2.0 / (0.5 * 0.04),
        ..kernel_free
    };
    assert!(!crawl.needs_valley(&smoothed, ratio));
    let turnaround = Crossing {
        speed: 0.0,
        ..crawl
    };
    assert!(turnaround.needs_valley(&smoothed, ratio));
}

/// The P1 Bowden crawl of `voron_first_layers` at t≈10.485 s (gscale 0.45
/// ablation): the seam into the leg crawls at 1.651 mm/s and lays its
/// valley, and the leg's own slowest point, 1e-5 mm on, is the same crawl
/// read a hair inside the leg. It lies where that valley already holds the
/// weight at zero, within its floor `v²/a`, so the advance reads one valley
/// there, not the product `W₁·W₂` that steepens the lead's rebuild.
#[test]
fn a_crawl_inside_a_laid_valleys_floor_lays_no_second_valley() {
    let (turn, reach, floor) = (468.816, 0.184_87, 3.830e-3);
    let mut smoothing = RatioSmoothing::default();
    smoothing.place_valley(turn, (turn, reach, reach), floor);
    smoothing.place_valley(turn + 1e-5, (turn + 1e-5, 0.184_83, 0.184_83), 3.831e-3);
    let laid = AdvanceValley {
        start: turn,
        end: turn,
        before: reach,
        after: reach,
        floor_before: floor,
        floor_after: floor,
        kind: ValleyKind::Crawl,
    };
    for k in -19..=19 {
        let s = turn + reach * f64::from(k) / 20.0;
        assert_eq!(
            smoothing.valley_weight(s),
            laid.weight(s),
            "the advance reads the crawl at s={s} mm through more than its valley"
        );
    }
}

/// A near-reversal through a short step crawls at the seam into the step
/// and again at the step's own slowest point, 0.02 mm on, past the first
/// valley's floor but well within its reach. The advance releases its lead
/// once and rebuilds it once: one valley holding the weight at zero across
/// both, its after side the later crawl's with that crawl's floor, not the
/// product `W₁·W₂`, which rebuilds the lead up to twice as steeply.
#[test]
fn a_crawl_within_the_last_crawl_valleys_reach_widens_it_to_one_valley() {
    let (turn, reach, floor) = (500.0, 0.18, 4.0e-3);
    let (next, next_reach, next_floor) = (500.02, 0.17, 3.0e-3);
    let mut smoothing = RatioSmoothing::default();
    smoothing.place_valley(turn, (turn, reach, reach), floor);
    smoothing.place_valley(next, (next, next_reach, next_reach), next_floor);
    let merged = AdvanceValley {
        start: turn,
        end: next,
        before: reach,
        after: next_reach,
        floor_before: floor,
        floor_after: next_floor,
        kind: ValleyKind::Crawl,
    };
    for k in -40..=40 {
        let s = 500.01 + 0.2 * f64::from(k) / 40.0;
        assert_eq!(
            smoothing.valley_weight(s),
            merged.weight(s),
            "the advance reads the crawls at s={s} mm through more than one valley"
        );
    }
}

#[test]
fn crawls_further_apart_than_their_reach_lay_a_valley_each() {
    let mut smoothing = RatioSmoothing::default();
    smoothing.place_valley(500.0, (500.0, 0.18, 0.18), 4.0e-3);
    smoothing.place_valley(500.4, (500.4, 0.17, 0.17), 3.0e-3);
    let first = AdvanceValley {
        start: 500.0,
        end: 500.0,
        before: 0.18,
        after: 0.18,
        floor_before: 4.0e-3,
        floor_after: 4.0e-3,
        kind: ValleyKind::Crawl,
    };
    let second = AdvanceValley {
        start: 500.4,
        end: 500.4,
        before: 0.17,
        after: 0.17,
        floor_before: 3.0e-3,
        floor_after: 3.0e-3,
        kind: ValleyKind::Crawl,
    };
    for k in -40..=40 {
        let s = 500.2 + 0.4 * f64::from(k) / 40.0;
        let (w1, w1s, w1c) = first.weight(s);
        let (w2, w2s, w2c) = second.weight(s);
        assert_eq!(
            smoothing.valley_weight(s),
            (
                w1 * w2,
                w1s * w2 + w1 * w2s,
                w1c * w2 + 2.0 * w1s * w2s + w1 * w2c
            ),
            "the crawls at 500 and 500.4 mm read at s={s} mm"
        );
    }
}

/// The smoothing of a follower whose kernel-free advance reads its valleys.
fn advanced_smoothing() -> RatioSmoothing {
    let mut smoothing = RatioSmoothing::default();
    smoothing.width.rule = Some(RatioRampRule {
        advance_slope_s: 0.5,
        max_velocity_mm_s: 45.0,
        max_accel_mm_s2: 2000.0,
        kernel_jerk_per_accel_hz: f64::INFINITY,
    });
    smoothing
}

/// A crawl valley holding the weight at zero back to where no rest or
/// floor holds it steps the weight from 1 to 0 there, which the advance
/// reads as a step in the follower's velocity: it fails loudly.
#[test]
#[should_panic(expected = "steps its weight at s=499.5")]
fn a_crawl_valley_held_at_zero_back_to_where_the_leaders_move_fails_loudly() {
    let mut smoothing = advanced_smoothing();
    smoothing.place_valley(500.0, (499.5, 0.0, 0.18), 4.0e-3);
}

/// Held at zero back to the floor, where the leaders rest, or to the end
/// of a rest valley, the weight is zero either side of the held stretch's
/// start.
#[test]
fn a_crawl_valley_held_at_zero_back_to_a_rest_keeps_the_weight_continuous() {
    let mut floored = advanced_smoothing();
    floored.close_at(499.5);
    floored.place_valley(500.0, (499.5, 0.0, 0.18), 4.0e-3);
    let mut rested = advanced_smoothing();
    rested.valleys.push(AdvanceValley {
        start: 499.4,
        end: 499.5,
        before: 0.2,
        after: 0.0,
        floor_before: 0.0,
        floor_after: 0.0,
        kind: ValleyKind::Rest,
    });
    rested.place_valley(500.0, (499.5, 0.0, 0.18), 4.0e-3);
    for s in [499.5 - 1e-9, 499.5 + 1e-9] {
        assert_eq!(rested.valley_weight(s).0, 0.0, "the weight at s={s} mm");
    }
}

const CRAWL_LEG_SEAM_SPEED_MM_S: f64 = 2.83;
const CRAWL_LEG_PEAK_GAIN_MM_S: f64 = 3.0 * CRAWL_LEG_SEAM_SPEED_MM_S;
const CRAWL_LEG_SIDEWAYS_MM_S: f64 = 0.3;
const CRAWL_LEG_PIECE_S: f64 = 0.1;
const CRAWL_LEG_REVERSAL_POWER: i32 = 8;

/// Leaders on a leg from `t0` that cross its opening seam along x at
/// [`CRAWL_LEG_SEAM_SPEED_MM_S`], speed up, and reverse x sharply right
/// before the closing seam, which they cross `end_excess_mm_s` faster than
/// the opening one, drifting sideways along y the whole time: the shaped
/// reversals of the rest ladder, which pass a hair beside zero inside the
/// last probe interval of the move into them. Returns the leaders and the
/// time of the reversal.
fn reversing_just_before_its_end(t0: f64, end_excess_mm_s: f64) -> (ContinuousSegment, f64) {
    let (v0, gain, piece) = (
        CRAWL_LEG_SEAM_SPEED_MM_S,
        CRAWL_LEG_PEAK_GAIN_MM_S,
        CRAWL_LEG_PIECE_S,
    );
    let power = CRAWL_LEG_REVERSAL_POWER;
    let (t_mid, t1) = (t0 + piece, t0 + 2.0 * piece);
    let x_mid = v0 * piece + 2.0 * gain * piece / 3.0;
    let drop = 2.0 * v0 + gain + end_excess_mm_s;
    let degree = power as usize + 1;
    let mut speeding_up = vec![0.0; degree + 1];
    speeding_up[..4].copy_from_slice(&[0.0, v0, gain / piece, -gain / (3.0 * piece * piece)]);
    let mut reversal = vec![0.0; degree + 1];
    reversal[0] = x_mid;
    reversal[1] = v0 + gain;
    reversal[degree] = -drop / (f64::from(power + 1) * piece.powi(power));
    let x = [
        BezierPiece {
            u_start: t0,
            u_end: t_mid,
            coeffs: speeding_up,
        },
        BezierPiece {
            u_start: t_mid,
            u_end: t1,
            coeffs: reversal,
        },
    ];
    let sideways = CRAWL_LEG_SIDEWAYS_MM_S;
    let y = [
        BezierPiece {
            u_start: t0,
            u_end: t_mid,
            coeffs: vec![0.0, sideways],
        },
        BezierPiece {
            u_start: t_mid,
            u_end: t1,
            coeffs: vec![sideways * piece, sideways],
        },
    ];
    let reversal_t = t_mid + piece * libm::pow((v0 + gain) / drop, 1.0 / f64::from(power));
    (leaders(&x, &y, (t0, t1)), reversal_t)
}

/// A crawl in the last probe interval of a leg is found however the
/// leg's two seam speeds compare: the seam a hair slower is not the
/// slowest point, so the crawl is not left to a seam crossing that never
/// sees it. Which seam reads slower is rounding where the leg's ends
/// mirror each other, so missing it made a stream lay a valley where the
/// same motion planned from rest laid none.
#[test]
fn a_crawl_right_before_a_legs_end_is_found_whichever_seam_is_slower() {
    for end_excess_mm_s in [-1e-6, 0.0, 1e-6] {
        let (shaped, reversal_t) = reversing_just_before_its_end(3.0, end_excess_mm_s);
        let shaped_leaders = ShapedLeaders::of(&shaped, &[0, 1]);
        let shaped_odometer = odometer(&shaped, &shaped_leaders);
        let leg = moving_leg(&shaped, (&shaped_leaders, &shaped_odometer), 100.0);
        let found = slowest_crawl(&shaped_leaders, &leg, f64::INFINITY);
        let Some((_, crossing)) = found else {
            panic!(
                "a leg whose closing seam is {end_excess_mm_s} mm/s faster than its opening \
                 one finds no crawl at its reversal at t={reversal_t}"
            );
        };
        assert!(
            (crossing.speed - CRAWL_LEG_SIDEWAYS_MM_S).abs() < 1e-6,
            "the crawl at the reversal at t={reversal_t} crosses as {crossing:?}"
        );
    }
}

const LINE_RATIO: f64 = 0.4 * 0.2 / (std::f64::consts::PI * 0.875 * 0.875);
const STOP: Crossing = Crossing {
    speed: 70.25,
    rate: 15068.0,
    jerk: 1.18e6,
};
const STOP_SPILL_MM: f64 = 0.25;
const LINE_END_MM: f64 = 30.0;

/// The table of a line ending in a rest at [`LINE_END_MM`], its ratio
/// stepping to `last_ratio` `last_mm` before the end.
fn line_into_rest(last_mm: f64, last_ratio: f64) -> Vec<super::super::RatioSpan> {
    let mut state = FollowerState::default();
    state.push_span(LINE_END_MM - last_mm, LINE_RATIO, LINE_RATIO);
    state.push_span(last_mm, last_ratio, last_ratio);
    state.spans
}

/// The side a line ending in a rest opens as the kernel-free fast bench
/// config stops it from 300 mm/s, over a spill of `spill_s`.
fn stopping_side(spill_s: f64) -> RestSide {
    RestSide {
        rest: LINE_END_MM,
        seam: LINE_END_MM - STOP_SPILL_MM,
        outward: -1.0,
        crossing: STOP,
        spill: Spill {
            duration: spill_s,
            crossing: STOP,
        },
        beyond: STOP,
    }
}

const DIRECT_DRIVE: RatioRampRule = RatioRampRule {
    advance_slope_s: 0.03,
    max_velocity_mm_s: 50.0,
    max_accel_mm_s2: 20000.0,
    kernel_jerk_per_accel_hz: f64::INFINITY,
};

/// A rest valley's side spans the spill alone where the extruder releases
/// the lead within the shortest ramp its limits allow and that fits within
/// the spill: the kernel-free fast bench config's 20000 mm/s² extruder
/// stopping a 300 mm/s line. A spill that takes no time ramps out over
/// that shortest ramp.
#[test]
fn a_rest_side_spans_the_spill_alone_where_the_lead_releases_within_it() {
    let table = line_into_rest(1.0, LINE_RATIO);
    assert_eq!(
        stopping_side(0.0074).width(&table, &DIRECT_DRIVE, LINE_END_MM),
        STOP_SPILL_MM
    );
    assert_eq!(
        stopping_side(0.0).width(&table, &DIRECT_DRIVE, LINE_END_MM),
        STOP_SPILL_MM + STOP.reach(RATIO_RAMP_MIN_TIME_S)
    );
}

/// A rest valley's side reaches as far as the longest ramp where a ratio
/// step lies so near the rest that the width field narrows its ramp past
/// the extruder's limits, where a Bowden advance's path alone leaves its
/// limit, and where the table does not reach that far yet.
#[test]
fn a_rest_side_reaches_the_longest_ramp_where_the_lead_or_a_near_step_needs_it() {
    let longest = STOP_SPILL_MM + STOP.reach(RATIO_RAMP_MAX_TIME_S);
    assert_eq!(
        stopping_side(0.0074).width(
            &line_into_rest(0.4, 3.0 * LINE_RATIO),
            &DIRECT_DRIVE,
            LINE_END_MM
        ),
        longest
    );
    let bowden = RatioRampRule {
        advance_slope_s: 0.5,
        max_velocity_mm_s: 45.0,
        max_accel_mm_s2: 2000.0,
        kernel_jerk_per_accel_hz: f64::INFINITY,
    };
    let table = line_into_rest(1.0, LINE_RATIO);
    assert_eq!(
        stopping_side(0.0074).width(&table, &bowden, LINE_END_MM),
        longest
    );
    let leaving = RestSide {
        rest: 0.0,
        seam: STOP_SPILL_MM,
        outward: 1.0,
        ..stopping_side(0.0074)
    };
    assert_eq!(leaving.width(&table, &DIRECT_DRIVE, 1.0), longest);
    assert_eq!(
        leaving.width(&table, &DIRECT_DRIVE, LINE_END_MM),
        STOP_SPILL_MM
    );
}

const CURVED_SPILL_MM: f64 = 0.1875;

/// The stopping side of a stop in `voron_dense_top_layers` on the
/// kernel-free fast bench config, whose shaped axes curve into it: the
/// leaders cross the raw seam at 38.8 mm/s, and cruise at up to 98 mm/s
/// past it.
fn curved_stop() -> RestSide {
    RestSide {
        rest: LINE_END_MM,
        seam: LINE_END_MM - CURVED_SPILL_MM,
        outward: -1.0,
        crossing: Crossing {
            speed: 38.8,
            rate: 4127.0,
            jerk: 4.884e5,
        },
        spill: Spill {
            duration: 0.0106,
            crossing: Crossing {
                speed: 38.8,
                rate: 5348.0,
                jerk: 1.758e6,
            },
        },
        beyond: Crossing {
            speed: 98.0,
            rate: 9203.0,
            jerk: 3.12e6,
        },
    }
}

/// The tail of a curved stop falls linearly across the spill, its slope
/// stepping at the raw seam. Read at the speed the leaders cruise past the
/// seam, that slope seam passed for a step the width field narrows past
/// the extruder's limits, and the side released the lead from the longest
/// ramp out.
#[test]
fn a_stopping_side_reads_its_spill_where_the_leaders_slow_into_the_rest() {
    let mut stop = FollowerState::default();
    stop.push_span(LINE_END_MM - CURVED_SPILL_MM, LINE_RATIO, LINE_RATIO);
    stop.push_span(CURVED_SPILL_MM, LINE_RATIO, 0.95 * LINE_RATIO);
    assert_eq!(
        curved_stop().width(&stop.spans, &DIRECT_DRIVE, LINE_END_MM),
        CURVED_SPILL_MM
    );
}

/// A ratio ramp opening 1.5 mm before the rest moves the ratio by 30 %
/// over 1.3 mm; the window its seam's distance from the rest leaves it
/// ramps only the part of that within it, not the whole ramp's change as
/// one step.
#[test]
fn a_rest_side_reads_a_long_ramp_by_its_change_within_the_window() {
    let ramp_start = LINE_END_MM - 1.5;
    let mut stop = FollowerState::default();
    stop.push_span(ramp_start, LINE_RATIO, LINE_RATIO);
    stop.push_span(
        LINE_END_MM - CURVED_SPILL_MM - ramp_start,
        LINE_RATIO,
        0.7 * LINE_RATIO,
    );
    stop.push_span(CURVED_SPILL_MM, 0.7 * LINE_RATIO, 0.665 * LINE_RATIO);
    assert_eq!(
        curved_stop().width(&stop.spans, &DIRECT_DRIVE, LINE_END_MM),
        CURVED_SPILL_MM
    );
}

/// A ratio step an ulp past the rest node its leg comes to rest at: the
/// width rises off the rest so slowly there that the step's window is a
/// few ulps wide, all of it within the inset a wider reach is read at. A
/// reach starting on the step still reads it at its first floats, as the
/// bench prints planned from rest read it where the leaders stand on the
/// step: its model holds every float of the reach, or there is none and
/// the window splits.
#[test]
fn a_reach_starting_on_a_step_an_ulp_past_a_rest_holds_the_step_window() {
    use crate::shaper::taylor::{Ball, TaylorModel};
    let step_at = 1000.0_f64;
    let rest = trajectory::next_toward(step_at, f64::NEG_INFINITY);
    let mut field = WidthField::default();
    field.push_rest(0.0);
    field.push(996.0, 1.0);
    field.push_rest(rest);
    field.push(1004.0, 1.0);
    field.push_rest(2000.0);
    let state = smoothed(&vec![(step_at, 0.03, 0.03), (1000.0, 0.0, 0.0)], field);
    let mut floats = vec![step_at];
    while floats.len() < 32 {
        let last = floats[floats.len() - 1];
        floats.push(trajectory::next_toward(last, f64::INFINITY));
    }
    let reach = PathReach {
        lo: step_at,
        hi: floats[floats.len() - 1],
        exact: (false, false),
    };
    assert!(
        state.smoothing.correction(step_at).d1 != 0.0,
        "the step's window must open at the step for the reach to read it"
    );
    let (centre, radius) = (0.5 * (reach.lo + reach.hi), 0.5 * (reach.hi - reach.lo));
    let s = TaylorModel::affine(Ball::exact(centre), radius);
    let Some(correction) = state.smoothing.correction_model(SpanTable::Laid, &s, reach) else {
        return;
    };
    for at in floats {
        let x = (at - centre) / radius;
        let truth = state.smoothing.correction(at).d1;
        let miss = (truth - model_at(&correction.d1, x)).abs();
        assert!(
            miss <= correction.d1.rad() + 1e-9 * (1.0 + truth.abs()),
            "d1 at {at} mm: the evaluator reads {truth}, the model misses it by {miss:e} past \
             its bound {:e}",
            correction.d1.rad()
        );
    }
}

/// A ratio change from `r_before` to `r_after` starting at `step_s`: a
/// step, or a slide through `r_mid` over the two lengths of `slide`, a
/// corner blend's ratio ramp, either of which may be zero, between rests
/// `0` and `rest_ahead` past the change's end, under a width field asking
/// for `target` every `spacing` mm, no closer than the real field lays its
/// nodes ([`super::NODE_SPACING_PER_WIDTH`] of the target), crossed by
/// leaders at `speed` accelerating at `accel`, never by so much they stop
/// within the change and a window of the target.
#[derive(Debug, Clone, Copy)]
struct RampCrossing {
    r_before: f64,
    r_mid: f64,
    r_after: f64,
    step_s: f64,
    slide: (f64, f64),
    rest_ahead: f64,
    target: f64,
    spacing: f64,
    speed: f64,
    accel: f64,
    advance_slope_s: f64,
}

impl RampCrossing {
    fn state(&self) -> FollowerState {
        let (first, second) = self.slide;
        let length = self.step_s + first + second + self.rest_ahead;
        let mid = self.mid();
        let table = [
            (self.step_s, self.r_before, self.r_before),
            (first, self.r_before, mid),
            (second, mid, self.r_after),
            (self.rest_ahead, self.r_after, self.r_after),
        ]
        .into_iter()
        .filter(|&(span, _, _)| span > 0.0)
        .collect();
        smoothed(&table, width(length, self.spacing, |_| self.target))
    }

    /// Where the change's seams lie: its start, and the slide's inner seam
    /// and end where it has them.
    fn seams(&self) -> Vec<f64> {
        let (first, second) = self.slide;
        let mut seams = vec![self.step_s];
        for end in [self.step_s + first, self.step_s + first + second] {
            if end > seams[seams.len() - 1] {
                seams.push(end);
            }
        }
        seams
    }

    /// The ratio between the slide's two lengths: `r_mid` where both are
    /// laid, else the end the missing one would reach.
    fn mid(&self) -> f64 {
        match self.slide {
            (0.0, _) => self.r_before,
            (_, 0.0) => self.r_after,
            _ => self.r_mid,
        }
    }

    /// The ratio's total variation across the change.
    fn variation(&self) -> f64 {
        let mid = self.mid();
        (mid - self.r_before).abs() + (self.r_after - mid).abs()
    }

    fn path_speed(&self, s: f64) -> f64 {
        (self.speed * self.speed + 2.0 * self.accel * (s - self.step_s)).sqrt()
    }

    /// The advanced follower's velocity and acceleration at `s`,
    /// `f + k·ḟ` and its rate, with `f = r̃(s)·ṡ`.
    fn follower_demand(&self, state: &FollowerState, s: f64) -> (f64, f64) {
        let laid = state.laid_on(SpanTable::Laid, s, SpanSide::After);
        let (r, slope, curvature) = (laid.ratio, laid.slope, laid.curvature);
        let (v, a, k) = (self.path_speed(s), self.accel, self.advance_slope_s);
        let velocity = r * v + k * (slope * v * v + r * a);
        let acceleration =
            slope * v * v + r * a + k * (curvature * v * v * v + 3.0 * slope * v * a);
        (velocity, acceleration)
    }
}

fn ramp_crossing_strategy() -> impl Strategy<Value = RampCrossing> {
    let slide_share = prop_oneof![Just(0.0), 1e-7..1e-3_f64, 1e-3..0.8_f64];
    (
        (0.0..0.05_f64, 0.0..0.05_f64, 0.0..0.05_f64),
        (
            0.2..6.0_f64,
            (slide_share.clone(), slide_share),
            0.2..6.0_f64,
        ),
        (0.05..2.0_f64, 0.75..1.5_f64),
        (5.0..300.0_f64, -0.45..0.45_f64),
        0.005..0.08_f64,
    )
        .prop_map(
            |(
                (r_before, r_mid, r_after),
                (step_s, (first_share, second_share), rest_ahead),
                (target, spacing_share),
                (speed, accel_share),
                k,
            )| {
                let slide = (first_share * target, second_share * target);
                RampCrossing {
                    r_before,
                    r_mid,
                    r_after,
                    step_s,
                    slide,
                    rest_ahead,
                    target,
                    spacing: spacing_share * target,
                    speed,
                    accel: accel_share * speed * speed / (target + slide.0 + slide.1),
                    advance_slope_s: k,
                }
            },
        )
}

proptest! {
    #![proptest_config(ProptestConfig {
        cases: std::env::var("PROPTEST_CASES")
            .map_or(256, |cases| cases.parse().expect("PROPTEST_CASES must be an integer")),
        failure_persistence: Some(Box::new(FileFailurePersistence::Direct(
            "proptest-regressions/ratio_ramp.txt",
        ))),
        max_global_rejects: 65_536,
        ..ProptestConfig::default()
    })]

    /// Wherever a ratio step or a slide one cluster of seams holds lies,
    /// under whatever width the rests' cones leave it, the advanced
    /// follower's velocity and acceleration across it, its curvature's
    /// steps at the ramp's ends among them, stay within what the ramp rule
    /// budgets a step of the ratio's total variation there at the
    /// cluster's width: the planner and the ramp time hold the follower's
    /// limits on that budget. A slide longer than its ends' windows is the
    /// moving window's, outside the rule's step.
    #[test]
    fn a_ratio_ramp_asks_no_more_than_its_rule_at_the_speed_it_is_crossed(
        crossing in ramp_crossing_strategy(),
    ) {
        assert_ramp_within_rule(&crossing)?;
    }
}

/// The advanced follower across `crossing`'s ramp against the rule's
/// budget for it (see
/// `a_ratio_ramp_asks_no_more_than_its_rule_at_the_speed_it_is_crossed`).
fn assert_ramp_within_rule(crossing: &RampCrossing) -> Result<(), TestCaseError> {
    let state = crossing.state();
    let seams = crossing.seams();
    let laid = &state.smoothing.laid;
    let start = laid.seams.partition_point(|seam| seam.s < seams[0]);
    let cluster = laid.cluster_holding(&state.smoothing.width, start);
    prop_assume!(cluster.seams.len() == laid.seams.len() - start);
    let h = cluster.width;
    prop_assume!(h > 0.0 && crossing.variation() > 0.0);
    let (lo, hi) = (seams[0] - h, seams[seams.len() - 1] + h);
    let fastest = crossing.path_speed(lo).max(crossing.path_speed(hi));
    let rule = RatioRampRule {
        advance_slope_s: crossing.advance_slope_s,
        max_velocity_mm_s: f64::INFINITY,
        max_accel_mm_s2: f64::INFINITY,
        kernel_jerk_per_accel_hz: f64::INFINITY,
    };
    let step = RatioStep {
        ratio_step: crossing.variation(),
        peak_ratio: crossing.r_after.max(crossing.r_before).max(crossing.mid()),
        speed_mm_s: fastest,
        accel_mm_s2: crossing.accel.abs(),
        jerk_mm_s3: 0.0,
    };
    let (velocity_budget, accel_budget) = rule.demand(2.0 * h / fastest, &step);
    let samples = 2_000;
    let edge_inset = 1e-9 * h;
    let points = (0..=samples)
        .map(|k| lo + (hi - lo) * f64::from(k) / f64::from(samples))
        .chain([lo + edge_inset, hi - edge_inset]);
    for s in points {
        let (velocity, acceleration) = crossing.follower_demand(&state, s);
        prop_assert!(
            velocity.abs() <= velocity_budget * (1.0 + 1e-9) + 1e-9,
            "the follower runs at {velocity} mm/s at {s} mm across a ramp budgeted {velocity_budget}"
        );
        prop_assert!(
            acceleration.abs() <= accel_budget * (1.0 + 1e-9) + 1e-6,
            "the follower accelerates at {acceleration} mm/s² at {s} mm across a ramp budgeted \
                 {accel_budget}"
        );
    }
    Ok(())
}

/// Found by `a_ratio_ramp_asks_no_more_than_its_rule_at_the_speed_it_is_crossed`
/// on windows that followed the width along the path: a step 0.2 mm short
/// of a rest, whose cone narrows the window past it at a quarter of the
/// path, read 2.5 times its budget where the window's back edge left it.
#[test]
fn a_step_short_of_a_rest_is_ramped_within_its_rule() {
    assert_ramp_within_rule(&RampCrossing {
        r_before: 0.0,
        r_mid: 0.0,
        r_after: 0.040_570_777_556_576_1,
        step_s: 0.2,
        slide: (0.0, 0.0),
        rest_ahead: 0.2,
        target: 0.05,
        spacing: 0.05,
        speed: 5.0,
        accel: 0.0,
        advance_slope_s: 0.005,
    })
    .unwrap();
}

/// Found by `a_ratio_ramp_asks_no_more_than_its_rule_at_the_speed_it_is_crossed`
/// on slides whose slopes a window following the width along the path
/// averaged: a corner blend's slide a micrometre long, 0.2 mm short of a
/// rest, read 2.5 times its budget where the window's back edge left it,
/// as the step it all but is did.
#[test]
fn a_slide_short_of_a_rest_is_ramped_within_its_rule() {
    assert_ramp_within_rule(&RampCrossing {
        r_before: 0.0,
        r_mid: 0.0,
        r_after: 0.001_835_452_892_171_063_4,
        step_s: 0.2,
        slide: (0.0, 1e-6),
        rest_ahead: 0.2,
        target: 0.05,
        spacing: 0.05,
        speed: 5.0,
        accel: 0.0,
        advance_slope_s: 0.005,
    })
    .unwrap();
}
