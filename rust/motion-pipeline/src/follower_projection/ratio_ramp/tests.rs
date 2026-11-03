use super::super::FollowerState;
use super::{ENVELOPE_SLOPE, KERNEL_PEAK, WIDTH_SLOPE_BOUND, WidthField};

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
    let mut field = WidthField {
        spacing,
        ..WidthField::default()
    };
    field.place_rest(0.0);
    let mut s = spacing;
    while s < length {
        field.push(s, Some(target(s)));
        s += spacing;
    }
    field.place_rest(length);
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
    let mut field = WidthField {
        spacing: 0.5,
        ..WidthField::default()
    };
    field.place_rest(0.0);
    for target in [0.0, 0.0, 0.2] {
        field.push(0.0, Some(target));
    }
    let mut s = 0.5;
    while s < total {
        field.push(s, Some(1.0));
        s += 0.5;
    }
    field.place_rest(total);
    let state = smoothed(&table, field);
    let (ratio_bound, slope_bound) = state.smoothing.ratio_bounds(&state.spans, 0.0, 0.0);
    let (ratio, slope) = state.ratio_and_slope(0.0);
    assert!(ratio.abs() <= ratio_bound && slope.abs() <= slope_bound);
    assert!(state.smoothing.bends(0.0, 0.0).contains(&0.0));
}

#[test]
fn the_width_is_c2_lipschitz_and_closes_on_every_rest() {
    let mut field = WidthField {
        spacing: 0.3,
        ..WidthField::default()
    };
    field.place_rest(0.0);
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
        field.push(s, Some(target));
    }
    field.place_rest(12.0);
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
    let mut field = WidthField {
        spacing: 0.5,
        ..WidthField::default()
    };
    field.place_rest(0.0);
    let speeds = [
        2.0, 3.0, 3.0, 3.0, 2.5, 0.4, 0.05, 0.4, 2.0, 3.0, 3.0, 3.0, 3.0, 1.0, 0.0, 0.0, 1.5, 3.0,
        3.0, 3.0, 3.0, 3.0, 3.0,
    ];
    let mut s = 0.0;
    let mut frozen: Vec<(usize, f64)> = Vec::new();
    for speed in speeds {
        s += speed;
        field.place_time_node(s);
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
    let mut field = WidthField {
        spacing: 0.5,
        ..WidthField::default()
    };
    field.place_rest(0.0);
    let rest: f64 = 2.5;
    let up = |s: f64| trajectory::next_toward(s, f64::INFINITY);
    let leaving = up(up(rest));
    for s in [1.0, 2.0, rest, up(rest), up(rest), leaving, 3.5, 4.5] {
        field.place_time_node(s);
    }
    field.close(5.0);
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
