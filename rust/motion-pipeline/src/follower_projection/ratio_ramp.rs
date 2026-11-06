use std::collections::VecDeque;

use geometry::{
    ADVANCE_VALLEY_SPEED_MM_S, AdvanceValley, RATIO_RAMP_ENVELOPE_SLOPE as ENVELOPE_SLOPE,
    RATIO_RAMP_MAX_TIME_S, RATIO_RAMP_MIN_TIME_S, RatioRampRule, RatioStep,
    TURNAROUND_SPEED_TIME_S, ValleyKind, crawl_floor, leader_reach, turns_around, valley_reach,
};

use super::{
    FloorLeaders, LeaderReads, OdometerLeaves, RatioSpan, SPAN_LOOKUP_SLACK_MM, SPAN_MIN_LEN_MM,
    ShapedLeaders, Table,
};

mod crawl_sides;
mod jet;
mod models;

use crawl_sides::{Side, SideTrail, StepWindows, Unlaid, crawl_side};
pub(super) use jet::Jet;
use jet::{horner, horner_with_slope};
pub(super) use models::{PathReach, ValleyWeight, narrow_floats};

/// `max |h'|` the width reaches: `3·ENVELOPE_SLOPE` rising off a rest,
/// `1.875·ENVELOPE_SLOPE` between two moving nodes.
pub(super) const WIDTH_SLOPE_BOUND: f64 = 3.0 * ENVELOPE_SLOPE;
/// Width nodes per ramp time: one every half of the shortest ramp.
const WIDTH_NODES_PER_RAMP_TIME: f64 = 2.0;
const NODE_SPACING_S: f64 = RATIO_RAMP_MIN_TIME_S / WIDTH_NODES_PER_RAMP_TIME;
/// How far apart on the path two moving width nodes lie at least, in
/// units of the narrower of their targets: the smootherstep between nodes
/// `L` apart that differ by at most `ENVELOPE_SLOPE·L` bends `h` by up to
/// `h''' = 60·ENVELOPE_SLOPE/L²`, which the smoothed ratio's curvature
/// carries, so an advance turns it into extruder acceleration. Nodes most
/// of a window's width apart keep that bend within the window's own; short
/// of a whole width, since at cruise on the shortest ramp a width is one
/// time tick of path exactly, and a tie there would hang the choice on how
/// the path's rounding falls.
const NODE_SPACING_PER_WIDTH: f64 = 0.75;
/// The path one time tick covers at cruise on the shortest ramp, in units
/// of that ramp's width `V·τ/2`; on a longer ramp a tick is less of it.
const TICK_WIDTHS: f64 = 2.0 / WIDTH_NODES_PER_RAMP_TIME;
/// How far apart on the path two moving width nodes lie at most at cruise,
/// in units of the width `V·τ/2` both target: a sample becomes a node on
/// the first tick past both half a ramp time after the last node and
/// [`NODE_SPACING_PER_WIDTH`] of a width along the path from it.
const NODE_GAP_WIDTHS: f64 = (1.0 + 0.5 * TICK_WIDTHS).max(NODE_SPACING_PER_WIDTH + TICK_WIDTHS);
/// How far along the path a seam holds the width its ramp asks for, in
/// units of that width `V·τ/2`: a window that wide reaches the seam from
/// up to one width away, and the nodes bracketing that reach lie within
/// one node gap more.
const SEAM_WIDTH_PLATEAU_WIDTHS: f64 = 1.0 + NODE_GAP_WIDTHS;
/// How fast the ramp time a seam asks for falls off in time away from it:
/// at a steady speed the width target `v·τ/2` then falls off along the
/// path no faster than [`ENVELOPE_SLOPE`].
const RAMP_TIME_SLOPE: f64 = 2.0 * ENVELOPE_SLOPE;
/// How far in time a seam raises the ramp time around it: it holds its own
/// within one ramp time of the seam, then falls off.
const RAMP_TIME_REACH_S: f64 =
    RATIO_RAMP_MAX_TIME_S + (RATIO_RAMP_MAX_TIME_S - RATIO_RAMP_MIN_TIME_S) / RAMP_TIME_SLOPE;
/// How far in time a seam's width reaches at cruise: until the longest
/// ramp's falls off its plateau by [`ENVELOPE_SLOPE`] to the shortest
/// ramp's own width. A node short of that would cap the plateau by its
/// cone.
const SEAM_WIDTH_REACH_S: f64 = (RATIO_RAMP_MAX_TIME_S
    * (1.0 + ENVELOPE_SLOPE * SEAM_WIDTH_PLATEAU_WIDTHS)
    - RATIO_RAMP_MIN_TIME_S)
    / (2.0 * ENVELOPE_SLOPE);
/// How far in time a seam raises the ramp time or the width around it.
const SEAM_REACH_S: f64 = RAMP_TIME_REACH_S.max(SEAM_WIDTH_REACH_S);
/// How far along the path a seam's width reaches before it falls off to
/// nothing, in units of that width: its plateau, then [`ENVELOPE_SLOPE`]
/// of it per unit of path.
const SEAM_WIDTH_REACH_WIDTHS: f64 = SEAM_WIDTH_PLATEAU_WIDTHS + 1.0 / ENVELOPE_SLOPE;
/// How long the leaders take at most from a node to a seam whose width
/// reaches it along the path, accelerating steadily from rest: over `t` at
/// `a` they cover `a·t²/2` and cross at up to `V = a·(t + τ_max/2)`, whose
/// width `V·τ_max/2` reaches [`SEAM_WIDTH_REACH_WIDTHS`] of itself, so
/// `t² ≤ K·(t + τ_max/2)` with `K` that many longest ramps, and `t` falls
/// short of `K + τ_max/2`. From a faster start they cover the path sooner.
const SEAM_PATH_REACH_S: f64 = (SEAM_WIDTH_REACH_WIDTHS + 0.5) * RATIO_RAMP_MAX_TIME_S;
/// How far in time the seams whose steps a window may gather with one lie
/// from it: half the longest ramp.
const CLUSTER_REACH_S: f64 = 0.5 * RATIO_RAMP_MAX_TIME_S;
/// How long after a node the leaders must be laid before its target is
/// known: every seam that raises its ramp time or its width, and every
/// seam those gather, lies within.
const TARGET_DELAY_S: f64 = SEAM_REACH_S.max(SEAM_PATH_REACH_S) + CLUSTER_REACH_S;
/// A node's target below which the leaders count as resting there: the
/// fit's back-and-forth around a rest point, far under any ramp.
const REST_TARGET_MM: f64 = SPAN_LOOKUP_SLACK_MM;
const SMOOTHERSTEP: [f64; 6] = [0.0, 0.0, 0.0, 10.0, -15.0, 6.0];
const SMOOTHERSTEP_PEAK_CURVATURE: f64 = 5.773_502_691_896_258;
/// `ψ(x) = 1 − (1 − x)³`: zero with slope 3 at a rest, flat to second
/// order at the moving node.
const RISE_FROM_REST: [f64; 4] = [0.0, 3.0, -3.0, 1.0];
const FALL_TO_REST: [f64; 4] = [1.0, 0.0, 0.0, -1.0];
const REST_PEAK_CURVATURE: f64 = 6.0;
/// `K(u) = ¾(1 − u²)` on `[−1, 1]`: its peak, `∫|u|K` and `½∫u²K`.
const KERNEL_PEAK: f64 = 0.75;
const KERNEL_ABS_MOMENT: f64 = 0.375;
const KERNEL_HALF_VARIANCE: f64 = 0.1;
/// `∫₋₁ˣ ∫₋₁ʸ K`: a unit ramp `x₊` smoothed by the kernel.
const SMOOTHED_RAMP: [f64; 5] = [0.1875, 0.5, 0.375, 0.0, -0.0625];
/// `∫₋₁ˣ` of [`SMOOTHED_RAMP`]: `x₊²/2` smoothed by the kernel.
const SMOOTHED_HALF_SQUARE: [f64; 6] = [0.05, 0.1875, 0.25, 0.125, 0.0, -0.0125];
/// [`SMOOTHED_HALF_SQUARE`] about the window's edge, `P₂(w − 1)` in powers
/// of `w = 1 + x`: a fourfold root there.
const SMOOTHED_HALF_SQUARE_OFF_EDGE: [f64; 6] = [0.0, 0.0, 0.0, 0.0, 0.0625, -0.0125];
const WIDTH_BISECTIONS: usize = 64;
/// How close in ulps a Newton step on a window edge settles before the
/// float the edge flips at is bracketed and bisected for.
const NEWTON_SETTLED_ULPS: f64 = 4.0;
/// Speed probes per leader knot interval a rest leg is searched with for
/// where the shaped leaders come to rest.
const REST_PROBES_PER_INTERVAL: usize = 8;
const REST_REFINEMENTS: usize = 64;

/// One segment's shaped leaders as the smoothing reads them: where on the
/// span table it starts, the time it covers, the path they cover by each
/// instant of it (the odometer ingest summed its arc with), and whether its
/// raw leaders rest along it (an extrude-only move or a dwell).
pub(super) struct RampLeg<'a> {
    pub(super) shaped_leaders: &'a ShapedLeaders<'a>,
    pub(super) odometer: &'a OdometerLeaves,
    pub(super) support: (f64, f64),
    pub(super) start_s: f64,
    pub(super) rests: bool,
}

/// A segment whose projection waits for every seam and width node that can
/// reach into it, with what its ingest read off its leaders.
#[derive(Debug)]
pub(super) struct PendingSegment {
    pub(super) t_end: f64,
    pub(super) s_start: f64,
    pub(super) s_length: f64,
    pub(super) reads: LeaderReads,
    table_end: f64,
}

/// Where the span table's ratio or its slope steps, and whether the step
/// is more than the rounding of a span cut where the ratio runs on
/// straight ([`SEAM_SLOPE_ROUNDING`]): only such a seam joins a cluster
/// ([`SeamCluster`]).
#[derive(Debug, Clone, Copy)]
struct Seam {
    s: f64,
    step: f64,
    slope_after: f64,
    significant: bool,
}

impl Seam {
    /// The seam where `after` follows `before`, if the ratio or its slope
    /// steps there.
    fn between(before: &RatioSpan, after: &RatioSpan) -> Option<Self> {
        let step = after.r0 - before.r1;
        let (slope_before, slope_after) = (before.ratio_slope(), after.ratio_slope());
        let slope_step = (slope_after - slope_before).abs();
        (step != 0.0 || slope_after != slope_before).then_some(Self {
            s: after.s0,
            step,
            slope_after,
            significant: step != 0.0
                || slope_step > SEAM_SLOPE_ROUNDING * slope_before.abs().max(slope_after.abs()),
        })
    }
}

/// How far, as a share of the steeper side, the slopes of two spans a
/// straight run of the ratio is cut into differ at most by rounding: each
/// slope is a difference of the cut's ratios over a span a hundredth of a
/// millimetre long or more.
const SEAM_SLOPE_ROUNDING: f64 = 1e-6;

/// A fresh revision: no two states of any table or width field share one.
fn next_revision() -> u64 {
    static NEXT: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(1);
    NEXT.fetch_add(1, std::sync::atomic::Ordering::Relaxed)
}

/// The seams of one span table as the smoothing sums them: where its ratio
/// or its slope steps, the table's slope behind the first of them, how far
/// along the path its spans have been examined for seams, how its seams
/// cluster under the width field of the revision `clusters` was built at
/// ([`Self::refresh`]), and, in path order, the ratio steps a crawl valley
/// holds the weight at zero around, each with the half-width its window is
/// kept within there ([`geometry::crawl_step_window`]).
#[derive(Debug, Default)]
pub(super) struct SeamTable {
    seams: Vec<Seam>,
    base_slope: Option<f64>,
    examined_through: f64,
    revision: u64,
    /// The first seam recorded since the clusters were last built.
    recorded_from: Option<usize>,
    clusters: Clusters,
    caps: Vec<(f64, f64)>,
}

/// Where the clusters of a table's seams lie under one width field: each
/// cluster's seams and width in path order, the kinks the moving window
/// averages, one at each cluster that changes the slope ([`SeamCluster`]),
/// and the clusters with a
/// stationary share — a ratio step, or more than one seam.
#[derive(Debug, Default)]
struct Clusters {
    width_revision: u64,
    seams_revision: u64,
    spans: Vec<ClusterSpan>,
    kinks: Vec<Seam>,
    stationary: Vec<usize>,
    /// The stationary share of each cluster, piece by piece
    /// ([`ClusterPiece`]), and which of them each cluster holds.
    pieces: Vec<ClusterPiece>,
    piece_ranges: Vec<std::ops::Range<usize>>,
    /// How far from its seams any cluster's stationary share reaches at
    /// most.
    widest_reach: f64,
}

/// One piece of a cluster's stationary share of `X`: the polynomial in
/// `s − from` it is over `[from, to)`, between two floats where a seam
/// enters or leaves the cluster's window or passes its point
/// ([`window_edges`]), where the window holds the same seams throughout.
#[derive(Clone, Copy, Debug)]
struct ClusterPiece {
    from: f64,
    to: f64,
    coefficients: [f64; SMOOTHED_HALF_SQUARE.len()],
    /// How far each coefficient may be off the exact one.
    roundings: [f64; SMOOTHED_HALF_SQUARE.len()],
}

/// The seams of a cluster inside one window, `first..past`, and the end
/// of those behind its point.
#[derive(Clone, Copy, Debug, PartialEq)]
struct ClusterWindow {
    first: usize,
    past: usize,
    behind: usize,
}

/// One cluster of a table: its seams `start..end` and its width.
#[derive(Clone, Copy, Debug)]
struct ClusterSpan {
    start: usize,
    end: usize,
    width: f64,
}

impl SeamTable {
    /// Nothing to smooth: the table has no seam and no slope.
    fn is_inert(&self) -> bool {
        self.seams.is_empty() && self.base_slope.unwrap_or(0.0) == 0.0
    }

    /// Records the seams of the spans not yet examined, the table's slope
    /// behind them first if none is known yet, and returns the range of
    /// those spans.
    fn record(&mut self, spans: &[RatioSpan]) -> std::ops::Range<usize> {
        if self.base_slope.is_none() {
            self.base_slope = Some(spans[0].ratio_slope());
        }
        let first = spans
            .partition_point(|span| span.s0 <= self.examined_through)
            .max(1);
        for index in first..spans.len() {
            let (before, after) = (&spans[index - 1], &spans[index]);
            self.examined_through = after.s0;
            if let Some(seam) = Seam::between(before, after) {
                self.recorded_from.get_or_insert(self.seams.len());
                self.seams.push(seam);
                self.revision = next_revision();
            }
        }
        first..spans.len()
    }

    /// Rebuilds the clusters where the seams or the width field changed
    /// since they were built: from the cluster holding the seam before the
    /// first one recorded since, or the first one whose width may have
    /// changed ([`WidthField::changed_from`]), whose join with it may have.
    /// A cluster rebuilt on the same seams at the same width keeps its
    /// pieces.
    fn refresh(&mut self, width: &WidthField) {
        let width_changed = self.clusters.width_revision != width.revision;
        if !width_changed && self.clusters.seams_revision == self.revision {
            return;
        }
        let mut from = self.recorded_from.take().unwrap_or(self.seams.len());
        if width_changed {
            from = from.min(
                self.seams
                    .partition_point(|seam| seam.s < width.changed_from),
            );
        }
        let kept = from.checked_sub(1).map_or(0, |before| {
            self.clusters
                .spans
                .partition_point(|span| span.end <= before)
        });
        let mut start = self.clusters.spans.get(kept).map_or(0, |span| span.start);
        if kept == 0 {
            start = 0;
        }
        let clusters = &mut self.clusters;
        let kept_stationary = clusters.stationary.partition_point(|&index| index < kept);
        let rebuilt: Vec<(ClusterSpan, Vec<ClusterPiece>)> = clusters.stationary[kept_stationary..]
            .iter()
            .zip(&clusters.piece_ranges[kept_stationary..])
            .map(|(&index, range)| {
                (
                    clusters.spans[index],
                    clusters.pieces[range.clone()].to_vec(),
                )
            })
            .collect();
        clusters.spans.truncate(kept);
        let kept_kinks = match self.seams.get(start) {
            Some(seam) if start > 0 => clusters.kinks.partition_point(|kink| kink.s < seam.s),
            _ => 0,
        };
        clusters.kinks.truncate(kept_kinks);
        clusters.stationary.truncate(kept_stationary);
        clusters.piece_ranges.truncate(kept_stationary);
        clusters
            .pieces
            .truncate(clusters.piece_ranges.last().map_or(0, |range| range.end));
        while start < self.seams.len() {
            let end = self.cluster_end(width, start);
            let seams = &self.seams[start..end];
            let span = ClusterSpan {
                start,
                end,
                width: seams
                    .iter()
                    .filter(|seam| seam.significant || seams.len() == 1)
                    .map(|seam| self.step_width(width, seam.s))
                    .fold(f64::INFINITY, f64::min),
            };
            let clusters = &mut self.clusters;
            let slope_in = clusters
                .kinks
                .last()
                .map_or(self.base_slope.unwrap_or(0.0), |kink| kink.slope_after);
            let slope_out = seams[seams.len() - 1].slope_after;
            if slope_out != slope_in {
                clusters.kinks.push(Seam {
                    s: seams[0].s,
                    step: 0.0,
                    slope_after: slope_out,
                    significant: true,
                });
            }
            if (seams.len() > 1 || seams[0].step != 0.0) && span.width > 0.0 {
                clusters.stationary.push(clusters.spans.len());
                let first = clusters.pieces.len();
                let unchanged = rebuilt
                    .binary_search_by(|(old, _)| old.start.cmp(&start))
                    .ok()
                    .map(|index| &rebuilt[index])
                    .filter(|(old, _)| old.end == end && old.width == span.width);
                match unchanged {
                    Some((_, pieces)) => clusters.pieces.extend_from_slice(pieces),
                    None => clusters.pieces.extend(cluster_pieces(seams, span.width)),
                }
                clusters.piece_ranges.push(first..clusters.pieces.len());
            }
            clusters.spans.push(span);
            start = end;
        }
        let clusters = &mut self.clusters;
        clusters.widest_reach = clusters
            .stationary
            .iter()
            .zip(&clusters.piece_ranges)
            .map(|(&index, range)| {
                let span = clusters.spans[index];
                let pieces = &clusters.pieces[range.clone()];
                (self.seams[span.start].s - pieces[0].from)
                    .max(pieces[pieces.len() - 1].to - self.seams[span.end - 1].s)
            })
            .fold(0.0, f64::max);
        clusters.width_revision = width.revision;
        clusters.seams_revision = self.revision;
    }

    /// The clusters, built under `width` as it stands.
    fn fresh(&self, width: &WidthField) -> &Clusters {
        assert!(
            self.clusters.width_revision == width.revision
                && self.clusters.seams_revision == self.revision,
            "the ratio smoothing reads its seam clusters built at width {} and seams {}, now \
             {} and {}",
            self.clusters.width_revision,
            self.clusters.seams_revision,
            width.revision,
            self.revision
        );
        &self.clusters
    }

    fn cluster(&self, span: ClusterSpan) -> SeamCluster<'_> {
        SeamCluster {
            seams: &self.seams[span.start..span.end],
            pieces: &[],
            width: span.width,
        }
    }

    /// The stationary cluster `stationary`, the index into
    /// [`Clusters::stationary`], with its pieces.
    fn stationary_cluster(&self, clusters: &Clusters, stationary: usize) -> SeamCluster<'_> {
        SeamCluster {
            pieces: &self.clusters.pieces[clusters.piece_ranges[stationary].clone()],
            ..self.cluster(clusters.spans[clusters.stationary[stationary]])
        }
    }

    /// The cluster holding seam `index`.
    #[cfg(test)]
    fn cluster_holding(&self, width: &WidthField, index: usize) -> SeamCluster<'_> {
        let spans = &self.fresh(width).spans;
        self.cluster(spans[spans.partition_point(|span| span.end <= index)])
    }

    fn seams_near(&self, lo: f64, hi: f64) -> &[Seam] {
        let first = self.seams.partition_point(|seam| seam.s <= lo);
        let last = self.seams.partition_point(|seam| seam.s < hi);
        &self.seams[first..last.max(first)]
    }

    /// Drops the seams at or behind `back_edge` but for the cluster
    /// running past it, kept whole, carrying the slope the last dropped
    /// opens; returns the first seam kept, or `back_edge` with none.
    fn prune_through(&mut self, width: &WidthField, back_edge: f64) -> f64 {
        let past = self.seams.partition_point(|seam| seam.s <= back_edge);
        let spans = &self.fresh(width).spans;
        let drop = spans
            .get(spans.partition_point(|span| span.end <= past))
            .map_or(past, |span| span.start);
        if drop > 0 {
            self.base_slope = Some(self.seams[drop - 1].slope_after);
            let kept_from = self.seams.get(drop).map_or(f64::INFINITY, |seam| seam.s);
            self.seams.drain(..drop);
            let dropped_caps = self.caps.partition_point(|&(s, _)| s < kept_from);
            self.caps.drain(..dropped_caps);
            self.revision = next_revision();
            let clusters = &mut self.clusters;
            let dropped = clusters.spans.partition_point(|span| span.end <= drop);
            clusters.spans.drain(..dropped);
            for span in &mut clusters.spans {
                span.start -= drop;
                span.end -= drop;
            }
            let dropped_kinks = clusters.kinks.partition_point(|kink| kink.s < kept_from);
            clusters.kinks.drain(..dropped_kinks);
            let dropped_stationary = clusters
                .stationary
                .partition_point(|&index| index < dropped);
            clusters.stationary.drain(..dropped_stationary);
            for index in &mut clusters.stationary {
                *index -= dropped;
            }
            clusters.piece_ranges.drain(..dropped_stationary);
            let dropped_pieces = clusters
                .piece_ranges
                .first()
                .map_or(clusters.pieces.len(), |range| range.start);
            clusters.pieces.drain(..dropped_pieces);
            for range in &mut clusters.piece_ranges {
                *range = range.start - dropped_pieces..range.end - dropped_pieces;
            }
            clusters.seams_revision = self.revision;
            self.recorded_from = self.recorded_from.map(|index| index.saturating_sub(drop));
        }
        self.seams
            .first()
            .map_or(back_edge, |seam| seam.s.min(back_edge))
    }

    /// Keeps the window of the ratio step at `s` within `half_width`, and
    /// rebuilds the clusters from the one holding it on.
    fn cap_step(&mut self, s: f64, half_width: f64) {
        let at = self.caps.partition_point(|&(capped, _)| capped < s);
        match self.caps.get_mut(at) {
            Some(cap) if cap.0 == s => cap.1 = cap.1.min(half_width),
            _ => self.caps.insert(at, (s, half_width)),
        }
        let seam = self.seams.partition_point(|seam| seam.s < s);
        self.recorded_from = Some(self.recorded_from.map_or(seam, |from| from.min(seam)));
        self.revision = next_revision();
    }

    /// The half-width the seam at `s` is smoothed over: the width field's
    /// there, within the cap a crawl valley keeps a ratio step there to.
    fn step_width(&self, width: &WidthField, s: f64) -> f64 {
        let field = width.step_width(s);
        let at = self.caps.partition_point(|&(capped, _)| capped < s);
        match self.caps.get(at) {
            Some(&(capped, half_width)) if capped == s => field.min(half_width),
            _ => field,
        }
    }

    /// The first seam of the first cluster whose smoothing may still
    /// change: one holding a seam past `width_final`, where the width is
    /// not final yet, or one a seam past it may still join — the next seam
    /// laid, or one still to come at `table_end` or past it, closer than
    /// the widest the two widths may yet be together.
    fn first_unfinal(&self, width: &WidthField, width_final: f64, table_end: f64) -> Option<f64> {
        let widest = |s: f64| {
            if s <= width_final {
                self.step_width(width, s)
            } else {
                width.width(width_final) + WIDTH_SLOPE_BOUND * (s - width_final)
            }
        };
        self.fresh(width).spans.iter().find_map(|span| {
            let cluster = self.cluster(*span);
            let last = cluster.last().s;
            let settled = last <= width_final
                && match self.seams.get(span.end) {
                    Some(next) if next.s <= width_final => true,
                    Some(next) => next.s - last >= widest(last) + widest(next.s),
                    None => table_end - last >= widest(last) + widest(table_end),
                };
            (!settled).then(|| cluster.first())
        })
    }

    /// Where the window of the cluster ending on the last significant seam
    /// opens, where a seam laid at `table_end` or past it may still join it
    /// ([`Self::joins`]): `σ − h(σ)` rises along the path, so one `σ′` past
    /// `table_end` joins it only where `table_end` lies closer to it than
    /// their widths there. Joining narrows the window, so it opens no
    /// earlier than it does now.
    fn joinable_opening(&self, width: &WidthField, table_end: f64, reach: f64) -> Option<f64> {
        let clusters = self.fresh(width);
        let last = self.seams.iter().rposition(|seam| seam.significant)?;
        let seam = self.seams[last];
        let joinable =
            seam.slope_after != 0.0 && table_end - seam.s < self.step_width(width, seam.s) + reach;
        let span = clusters.spans[clusters.spans.partition_point(|span| span.end <= last)];
        joinable.then(|| self.seams[span.start].s - span.width)
    }

    /// The table's slope just past `s`, as the seams laid so far leave it.
    fn slope_left_of(&self, s: f64) -> f64 {
        self.seams
            .partition_point(|seam| seam.s <= s)
            .checked_sub(1)
            .map_or(self.base_slope.unwrap_or(0.0), |last| {
                self.seams[last].slope_after
            })
    }

    /// Whether the significant seams `a < b`, the next after `a`, join: a
    /// sloped span runs between them, short enough that their own windows
    /// overlap. A seam a rest holds has none; a span of constant ratio
    /// between two changes leaves each its own; and a sloped span longer
    /// than its ends' windows leaves its slope to the moving window, which
    /// narrows along it as the width does toward a rest.
    fn joins(&self, width: &WidthField, a: usize, b: usize) -> bool {
        let (before, after) = (self.seams[a].s, self.seams[b].s);
        self.seams[a].slope_after != 0.0
            && after - before < self.step_width(width, before) + self.step_width(width, after)
    }

    fn significant_after(&self, index: usize) -> Option<usize> {
        (index + 1..self.seams.len()).find(|&next| self.seams[next].significant)
    }

    /// Where the cluster opening at seam `start` ends: a significant seam
    /// with every one after it that joins the last ([`Self::joins`]), and
    /// the seams between them; a seam that is not significant stands
    /// alone.
    fn cluster_end(&self, width: &WidthField, start: usize) -> usize {
        let mut anchor = start;
        if self.seams[start].significant {
            while let Some(next) = self
                .significant_after(anchor)
                .filter(|&next| self.joins(width, anchor, next))
            {
                anchor = next;
            }
        }
        anchor + 1
    }

    /// The clusters with a stationary share whose windows reach into
    /// `[lo, hi]` ([`SeamCluster::support`]), in path order: every one whose
    /// first seam lies short of the widest cluster's reach past `hi`, and
    /// its last no further than that before `lo`.
    fn clusters_reaching<'a>(
        &'a self,
        width: &'a WidthField,
        (lo, hi): (f64, f64),
    ) -> impl Iterator<Item = SeamCluster<'a>> + 'a {
        let clusters = self.fresh(width);
        let reach = clusters.widest_reach;
        let span = |index: &usize| clusters.spans[*index];
        let first = clusters
            .stationary
            .partition_point(|index| self.seams[span(index).end - 1].s <= lo - reach);
        let past = clusters
            .stationary
            .partition_point(|index| self.seams[span(index).start].s <= hi + reach);
        (first..past.max(first))
            .map(move |stationary| self.stationary_cluster(clusters, stationary))
            .filter(move |cluster| {
                let (from, to) = cluster.support();
                from <= hi && to > lo
            })
    }

    /// The table the moving window averages over `(lo, hi)`: each
    /// cluster's change of slope gathered into one kink at its first seam
    /// ([`SeamCluster`]), and the slope that leaves at `lo`.
    fn moving_view(&self, width: &WidthField, (lo, hi): (f64, f64)) -> SeamView<'_> {
        let kinks = &self.fresh(width).kinks;
        let first = kinks.partition_point(|kink| kink.s <= lo);
        let past = kinks.partition_point(|kink| kink.s < hi).max(first);
        SeamView {
            seams: &kinks[first..past],
            slope_before: first
                .checked_sub(1)
                .map_or(self.base_slope.unwrap_or(0.0), |last| {
                    kinks[last].slope_after
                }),
        }
    }

    /// `X = Ẽ − E` at `s` with its first three path derivatives under the
    /// width field `width`: the extrusion, ratio, slope and curvature the
    /// smoothing adds to the table's. The table's slope, each cluster's
    /// change of it gathered at its first seam, is averaged by the window
    /// at `s` ([`slope_correction`]); what the clusters change beyond that,
    /// their ratio steps and slides, each by its own stationary window
    /// ([`SeamCluster::correction`]).
    fn correction(&self, width: &WidthField, s: f64) -> Jet {
        if self.is_inert() {
            return Jet::ZERO;
        }
        let h = width.jet(s);
        if h.value <= 0.0 {
            return Jet::ZERO;
        }
        let moving = self.moving_view(width, (s - h.value, s + h.value));
        self.clusters_reaching(width, (s, s))
            .fold(slope_correction(&moving, h, s), |total, cluster| {
                total + cluster.correction(s)
            })
    }
}

/// Which spans the smoothed table reads over a stretch
/// ([`RatioSmoothing::reading`]): those overlapping the moving windows'
/// reach, and those touching a cluster reaching the stretch.
pub(super) struct SpanReading<'a> {
    moving: (f64, f64),
    clusters: Vec<SeamCluster<'a>>,
}

impl SpanReading<'_> {
    pub(super) fn reads(&self, span: &RatioSpan) -> bool {
        (span.s1 > self.moving.0 && span.s0 < self.moving.1)
            || self
                .clusters
                .iter()
                .any(|cluster| span.s1 >= cluster.first() && span.s0 <= cluster.last().s)
    }

    /// The lowest and highest ratio of the `spans` read
    /// ([`read_ratio_range`]).
    fn ratio_range(&self, spans: &[RatioSpan]) -> Option<(f64, f64)> {
        read_ratio_range(
            spans,
            self.moving,
            self.clusters
                .iter()
                .map(|cluster| (cluster.first(), cluster.last().s)),
        )
    }
}

/// The lowest and highest ratio of the `spans` the smoothing reads: those
/// overlapping the `moving` reach, those touching the seams `[first, last]`
/// of each of the `clusters`, and the last one's end ratio held on past the
/// table where the moving reach runs past it; `None` where none is read.
fn read_ratio_range(
    spans: &[RatioSpan],
    moving: (f64, f64),
    clusters: impl Iterator<Item = (f64, f64)>,
) -> Option<(f64, f64)> {
    let hull = |range: (f64, f64), read: &[RatioSpan]| {
        read.iter().fold(range, |(low, high), span| {
            (
                low.min(span.r0.min(span.r1)),
                high.max(span.r0.max(span.r1)),
            )
        })
    };
    let within = |from: usize, past: usize| &spans[from..past.max(from)];
    let moved = within(
        spans.partition_point(|span| span.s1 <= moving.0),
        spans.partition_point(|span| span.s0 < moving.1),
    );
    let (low, high) = clusters.fold(
        hull((f64::INFINITY, f64::NEG_INFINITY), moved),
        |range, (first, last)| {
            let touching = within(
                spans.partition_point(|span| span.s1 < first),
                spans.partition_point(|span| span.s0 <= last),
            );
            hull(range, touching)
        },
    );
    let (low, high) = match spans.last() {
        Some(last) if moving.1 > last.s1 => (low.min(last.r1), high.max(last.r1)),
        _ => (low, high),
    };
    (low <= high).then_some((low, high))
}

/// Seams in path order beside the slope the table holds before the first.
struct SeamView<'a> {
    seams: &'a [Seam],
    slope_before: f64,
}

impl SeamView<'_> {
    /// Nothing to average: no seam and no slope.
    fn is_flat(&self) -> bool {
        self.seams.is_empty() && self.slope_before == 0.0
    }
}

/// The slope's share of `X` under the window of width `h` at `s`, of the
/// table `view` holds about it: the spans between the seams inside by
/// their moments ([`SeamMoments`]) and the two running past its edges
/// ([`edge_spans`]).
fn slope_correction(view: &SeamView<'_>, h: Jet, s: f64) -> Jet {
    let (lo, hi) = (s - h.value, s + h.value);
    let first = view.seams.partition_point(|seam| seam.s <= lo);
    let past = view.seams.partition_point(|seam| seam.s < hi).max(first);
    let slope_left = first
        .checked_sub(1)
        .map_or(view.slope_before, |last| view.seams[last].slope_after);
    let window = &view.seams[first..past];
    let mut moments = SeamMoments::default();
    for pair in window.windows(2) {
        let (seam, next) = (pair[0], pair[1]);
        moments.add_span((s - seam.s, s - next.s), seam.slope_after, next.s - seam.s);
    }
    let inverse = h.recip();
    let ends = window
        .first()
        .zip(window.last())
        .map(|(first_seam, last_seam)| (s - first_seam.s, s - last_seam.s, last_seam.slope_after));
    moments.correction(h, inverse) + edge_spans(h, inverse, slope_left, ends)
}

/// A run of the table's seams that sloped spans join, each shorter than
/// the windows, `h(σ)` either side of its ends, overlap ([`SeamTable::joins`]):
/// a lone seam, or a corner blend's ratio slide. A window that followed the width along the path would sweep a
/// ratio change there at `1 ± h'` times the path's speed, and step the
/// ratio's curvature at its edge by `(1 ∓ h')²` times what the ramp rule
/// budgets ([`geometry::RatioRampRule`]). So the cluster's change is split
/// in two: its change of slope, gathered into one kink at its first seam,
/// which the moving window averages and lets down to nothing at every
/// rest — nothing at all where the cluster opens and closes on spans of
/// constant ratio; and the rest of its change, its ratio steps and its
/// slopes less the one it leaves on, flat either side, averaged over one
/// stationary half-width, the narrowest its seams hold. Averaged over one
/// width, a change whose ratio varies by `V` in total asks no more of the
/// ratio's derivatives than a step of `V` there.
#[derive(Clone, Copy, Debug)]
struct SeamCluster<'a> {
    seams: &'a [Seam],
    pieces: &'a [ClusterPiece],
    width: f64,
}

impl SeamCluster<'_> {
    fn first(&self) -> f64 {
        self.seams[0].s
    }

    fn last(&self) -> Seam {
        self.seams[self.seams.len() - 1]
    }

    /// The slope the cluster leaves on, past its last seam: what the
    /// moving window averages across it.
    fn slope_out(&self) -> f64 {
        self.last().slope_after
    }

    /// Where the stationary share is something: from the first float the
    /// first seam enters the window to the first the last leaves it;
    /// empty for a cluster without one.
    fn support(&self) -> (f64, f64) {
        match (self.pieces.first(), self.pieces.last()) {
            (Some(first), Some(last)) => (first.from, last.to),
            _ => (f64::INFINITY, f64::NEG_INFINITY),
        }
    }

    /// The piece holding `s`, if any.
    fn piece_at(&self, s: f64) -> Option<&ClusterPiece> {
        let index = self
            .pieces
            .partition_point(|piece| piece.from <= s)
            .checked_sub(1)?;
        let piece = &self.pieces[index];
        (s < piece.to).then_some(piece)
    }

    /// The stationary share of `X` at `s` ([`cluster_pieces`]).
    fn correction(&self, s: f64) -> Jet {
        self.piece_at(s).map_or(Jet::ZERO, |piece| {
            Jet::variable(s)
                .offset(-piece.from)
                .poly(&piece.coefficients)
        })
    }
}

/// The pieces of the stationary share of `X` of a cluster holding `seams`,
/// averaged over the half-width `h`: in each, with `x = (s − σ)/h`, each
/// ratio step `Δ` the window holds adds `Δ·h·(P₁(x) − x₊)`, the smoothed
/// ramp less the table's own kink ([`SMOOTHED_RAMP`]); the cluster's slopes
/// less the one it leaves on add what the window's moments of them make
/// ([`models::cluster_slope_polynomial`]), read span by span as a rise
/// over each, so a corner blend nanometres long, whose slope steps by
/// millions, leaves no two steps cancelling to an ulp of `m·h²`.
fn cluster_pieces(seams: &[Seam], h: f64) -> Vec<ClusterPiece> {
    let mut cuts: Vec<f64> = seams
        .iter()
        .flat_map(|seam| {
            let [enters, leaves] = window_edges(seam.s, h);
            [enters, seam.s, leaves]
        })
        .collect();
    cuts.sort_by(f64::total_cmp);
    cuts.dedup();
    cuts.windows(2)
        .map(|pair| {
            let (from, to) = (pair[0], pair[1]);
            let first = seams.partition_point(|seam| seam.s <= from - h);
            let past = seams.partition_point(|seam| seam.s < from + h).max(first);
            let window = ClusterWindow {
                first,
                past,
                behind: seams
                    .partition_point(|seam| seam.s <= from)
                    .clamp(first, past),
            };
            let slope = models::cluster_slope_polynomial(seams, h, window, from, to - from);
            let mut coefficients = slope.map(|term| term.mid);
            let mut roundings = slope.map(|term| term.rad);
            for (index, seam) in seams.iter().enumerate().take(past).skip(first) {
                if seam.step == 0.0 {
                    continue;
                }
                let mut ramp = SMOOTHED_RAMP.to_vec();
                if index < window.behind {
                    ramp[1] -= 1.0;
                }
                add_composed(
                    (&mut coefficients, &mut roundings),
                    &ramp,
                    seam.step * h,
                    (from - seam.s) / h,
                    h,
                );
            }
            ClusterPiece {
                from,
                to,
                coefficients,
                roundings,
            }
        })
        .collect()
}

/// Adds `factor·q((y + offset·h)/h)` to `coefficients`, in powers of `y`:
/// `q` shifted by `offset` and scaled by `1/h`; each coefficient's
/// rounding to `roundings`, a few operations on terms no larger than the
/// magnitudes summed, within `γ` of them.
fn add_composed(
    (coefficients, roundings): (&mut [f64], &mut [f64]),
    q: &[f64],
    factor: f64,
    offset: f64,
    h: f64,
) {
    let mut shifted = q.to_vec();
    let mut magnitudes: Vec<f64> = q.iter().map(|c| c.abs()).collect();
    for lowest in 0..shifted.len() {
        for power in (lowest + 1..shifted.len()).rev() {
            shifted[power - 1] += offset * shifted[power];
            magnitudes[power - 1] += offset.abs() * magnitudes[power];
        }
    }
    let gamma = 4.0 * (q.len() as f64 + 3.0) * f64::EPSILON;
    let mut scale = factor;
    for ((slot, rounding), (term, magnitude)) in coefficients
        .iter_mut()
        .zip(roundings.iter_mut())
        .zip(shifted.iter().zip(&magnitudes))
    {
        *slot += scale * term;
        *rounding += gamma * (scale.abs() * magnitude + slot.abs());
        scale /= h;
    }
}

/// The floats where the seam at `seam` enters and leaves the window of
/// half-width `h` that reads it ([`cluster_pieces`]): the first `s`
/// with `seam < s + h`, and the first with `seam <= s − h`.
fn window_edges(seam: f64, h: f64) -> [f64; 2] {
    let first_holding = |start: f64, holds: &dyn Fn(f64) -> bool| {
        let mut s = start;
        while !holds(s) {
            s = trajectory::next_toward(s, f64::INFINITY);
        }
        loop {
            let before = trajectory::next_toward(s, f64::NEG_INFINITY);
            if !holds(before) {
                return s;
            }
            s = before;
        }
    };
    [
        first_holding(seam - h, &|s| seam < s + h),
        first_holding(seam + h, &|s| seam <= s - h),
    ]
}

/// The spans inside one window as [`slope_correction`] sums
/// them, each seam at `u = s − σ`: the moments `Σ m·(u_aᵏ − u_bᵏ)` of the
/// table's slope over each span between two seams, and the table's own
/// kink behind `s` they carry, with its two derivatives.
///
/// With `b` the coefficients of [`SMOOTHED_HALF_SQUARE`],
/// `h²·G₂(u/h) = Σₖ bₖ·uᵏ·h²⁻ᵏ − u₊²/2`. The slope `m` over a span adds
/// `m·h²·(G₂(x_a) − G₂(x_b))`, its ends' `x = u/h`, which sums to the
/// slope steps' `Δm·h²·G₂` and the slope behind the window's `0.1·h²·m`.
/// So the correction is the moments weighted by powers of `h`, and a
/// moment's path derivative is
/// `(Σ c·uᵏ)' = k·Σ c·uᵏ⁻¹`: each seam costs a few products whatever the
/// jets of `h`. A span inside the window enters as its rise `m·(u_a − u_b)`
/// times `(u_aᵏ − u_bᵏ)/(u_a − u_b)`, never as two slope steps of opposite
/// sign: a corner blend nanometres long steps the slope by millions, and
/// those would cancel only to an ulp of `m·h²`. The two spans running past
/// the window's edges are [`edge_spans`].
#[derive(Default)]
struct SeamMoments {
    slope: [f64; SMOOTHED_HALF_SQUARE.len()],
    behind: [f64; 3],
}

impl SeamMoments {
    /// The span of slope `slope` and `length` between the seams at
    /// `u_a > u_b`. A seam at `s` lies behind it, as the table reads the
    /// span it opens there.
    fn add_span(&mut self, (u_a, u_b): (f64, f64), slope: f64, length: f64) {
        if slope == 0.0 {
            return;
        }
        let rise = slope * length;
        let mut divided_difference = 0.0;
        let mut power_b = 1.0;
        for moment in &mut self.slope[1..] {
            divided_difference = divided_difference * u_a + power_b;
            power_b *= u_b;
            *moment += rise * divided_difference;
        }
        if u_b >= 0.0 {
            self.behind[0] += rise * 0.5 * (u_a + u_b);
            self.behind[1] += rise;
        } else if u_a >= 0.0 {
            self.behind[0] += 0.5 * slope * u_a * u_a;
            self.behind[1] += slope * u_a;
            self.behind[2] += slope;
        }
    }

    /// The moment `moments[k]` with its path derivatives.
    fn moment(moments: &[f64], k: usize) -> Jet {
        let derivative = |order: usize| {
            if order > k {
                return 0.0;
            }
            let falling: usize = (k + 1 - order..=k).product();
            falling as f64 * moments[k - order]
        };
        Jet {
            value: moments[k],
            d1: derivative(1),
            d2: derivative(2),
            d3: derivative(3),
        }
    }

    /// The slope's `X` but for [`edge_spans`] with the width `h` and its
    /// inverse: the moments of power `k` weigh with `h²⁻ᵏ`; `h⁰` is no
    /// product.
    fn correction(&self, h: Jet, inverse: Jet) -> Jet {
        let inverse_squared = inverse * inverse;
        let powers_of_h = [
            Some(h * h),
            Some(h),
            None,
            Some(inverse),
            Some(inverse_squared),
            Some(inverse_squared * inverse),
        ];
        let kink = Jet {
            value: -self.behind[0],
            d1: -self.behind[1],
            d2: -self.behind[2],
            d3: 0.0,
        };
        powers_of_h
            .iter()
            .enumerate()
            .fold(kink, |total, (index, &power)| {
                let weight = Self::moment(&self.slope, index).scale(SMOOTHED_HALF_SQUARE[index]);
                total + power.map_or(weight, |power| power * weight)
            })
    }
}

/// What the two spans of the table running past the window's edges add to
/// `X`: `slope_left` from behind the back edge up to the first seam inside,
/// at `u_first`, `m·h²·(G₂(1) − G₂(x))`, and `slope_right` from the last,
/// at `u_last`, on past the front edge, `m·h²·G₂(x)`. Each `G₂` is read
/// about the edge it lies nearer, `P₂(x)` as `P₂(w − 1)` with `w = 1 + x`
/// and `0.1 − G₂(x)` as `P₂(−x)` with `w = 1 − x`: a span that stops short
/// of `s` is as steep as its rise over `w·h`, and `P₂`'s fourfold root
/// there keeps the product of the two within an ulp of the rise. No seam
/// inside: one span over the whole window, `0.1·h²·m`.
fn edge_spans(h: Jet, inverse: Jet, slope_left: f64, ends: Option<(f64, f64, f64)>) -> Jet {
    let Some((u_first, u_last, slope_right)) = ends else {
        return (h * h).scale(KERNEL_HALF_VARIANCE * slope_left);
    };
    let back = if slope_left == 0.0 {
        Jet::ZERO
    } else {
        half_square_about_edges(h, inverse, u_first)
            .1
            .scale(slope_left)
    };
    let front = if slope_right == 0.0 {
        Jet::ZERO
    } else {
        half_square_about_edges(h, inverse, u_last)
            .0
            .scale(slope_right)
    };
    h * h * (back + front)
}

/// `(G₂(x), 0.1 − G₂(x))` at `x = u/h`, the one off the nearer edge read
/// about it (see [`edge_spans`]).
fn half_square_about_edges(h: Jet, inverse: Jet, u: f64) -> (Jet, Jet) {
    let u = Jet::variable(u);
    let complement = |part: Jet| part.scale(-1.0).offset(KERNEL_HALF_VARIANCE);
    if u.value < 0.0 {
        let ahead = ((h + u) * inverse).poly(&SMOOTHED_HALF_SQUARE_OFF_EDGE);
        (ahead, complement(ahead))
    } else {
        let behind = ((h + u.scale(-1.0)) * inverse).poly(&SMOOTHED_HALF_SQUARE_OFF_EDGE);
        (complement(behind), behind)
    }
}

/// One node of the width field: its path position, the half-width its
/// leaders ask for there, and the envelope the field takes.
#[derive(Debug, Clone, Copy)]
struct WidthNode {
    s: f64,
    target: f64,
    envelope: f64,
}

/// Where and when the leaders are at one tick of the time grid, and half
/// the path they cover between the ticks on either side once the next is
/// laid.
#[derive(Debug, Clone, Copy)]
struct TimeSample {
    s: f64,
    t: f64,
    base: Option<f64>,
}

/// The width field's time ticks since the leaders last stood still: every
/// [`NODE_SPACING_S`] from `origin`, and the next one to sample.
#[derive(Debug, Clone, Copy)]
struct SampleGrid {
    origin: f64,
    next: i64,
}

impl SampleGrid {
    fn time(&self, tick: i64) -> f64 {
        self.origin + tick as f64 * NODE_SPACING_S
    }
}

/// What may still become a node, in path order: a time sample, or a rest.
#[derive(Debug, Clone, Copy)]
enum Candidate {
    Time(TimeSample),
    Rest(f64),
}

impl Candidate {
    fn s(&self) -> f64 {
        match *self {
            Self::Time(sample) => sample.s,
            Self::Rest(s) => s,
        }
    }
}

/// A ratio step as its ramp time is derived: where and when the leaders
/// cross it, how fast and under what acceleration, how far the ratio moves
/// there — its step and the change along the span it opens, which a window
/// may gather as one step where that span is short — and the largest ratio
/// about it.
#[derive(Debug, Clone, Copy)]
struct TimedSeam {
    t: f64,
    s: f64,
    crossing: Crossing,
    step: f64,
    peak_ratio: f64,
    ramp_time: Option<f64>,
}

/// The last moving node the width field kept: when and where, its ramp
/// time and its target.
#[derive(Debug, Clone, Copy)]
struct KeptNode {
    t: f64,
    s: f64,
    ramp_time: f64,
    target: f64,
}

/// What the next moving node keeps its distance from: the rest node the
/// leaders last left, or the last moving node kept since.
#[derive(Debug, Clone, Copy)]
enum LastKept {
    Rest(f64),
    Moving(KeptNode),
}

/// How the leaders cross the last half of the longest ramp of a laid leg
/// ending at `until`.
#[derive(Debug, Clone, Copy)]
struct LegTail {
    until: f64,
    crossing: Crossing,
}

/// The part of `support` a ramp laid at `t` may reach: half the longest
/// ramp on either side.
fn ramp_reach_within(t: f64, (start, end): (f64, f64)) -> (f64, f64) {
    (
        (t - CLUSTER_REACH_S).clamp(start, end),
        (t + CLUSTER_REACH_S).clamp(start, end),
    )
}

/// The smoothing's half-width `h(s)` along the path.
///
/// Each ratio step asks for its own ramp time `τ_σ`
/// ([`RatioRampRule::ramp_time`]) from the fastest and most accelerating
/// the leaders are over its ramp and the steps within half the longest ramp
/// of it, which one window may gather. The ramp time at `t` is the largest
/// of the seams around, each holding `τ_σ` within `τ_σ` of its seam and
/// falling off by half the time beyond, never under the shortest ramp.
///
/// The rule bounds a ramp whose window is `H_σ = V·τ_σ/2` wide wherever the
/// seam is inside it, `V` the fastest the leaders cross it at. The leaders'
/// own speed alone would not lay that: where they brake hard next to the
/// step, or slow into a corner a few millimetres off it, the targets fall
/// off faster than the envelope may, and the slow node's cone narrows the
/// window below `H_σ` exactly where the leaders are fast. So each seam
/// also asks for its width ([`Self::seam_width_at`]): every node within
/// [`SEAM_WIDTH_PLATEAU_WIDTHS`] widths of it on the path targets at least
/// `H_σ`, falling off by [`ENVELOPE_SLOPE`] beyond, however long the
/// leaders take between the two, up to a rest, which no window reaches
/// across. So only a rest narrows a seam's window below its width, and the
/// planner slows a step where the rest leaves its ramp too little path
/// ([`geometry::rest_room_reach_mm`]).
///
/// The leaders are sampled every half of the shortest ramp in time. A
/// sample becomes a node once every seam that sets its ramp time `τ` or its
/// width is laid — the seams within [`SEAM_PATH_REACH_S`] after it, the
/// furthest a steady acceleration from rest crosses one whose width reaches
/// back to it — and only half its own ramp time after the last one and no
/// nearer on the path than [`NODE_SPACING_PER_WIDTH`] times the narrower of
/// the two targets: the smootherstep between two of them bends `h` no
/// sharper than a window's own width allows. Off a rest, a sample becomes a
/// node only once the rest's cone no longer caps its target: a node the
/// cone caps is a sliver of its window, micrometres past the rest where the
/// leaders creep out of it, which the smootherstep to the next node would
/// flatten onto, while the rise off the rest opens the window with the
/// distance. Where the rest's cone meets the cone of the first node kept
/// past it, short of that node, a node marks the peak, so the rise holds
/// the cone at least ([`Self::push_cone_peak`]). A node's own target is
/// half the path the leaders cover in `τ` around it — `v·τ/2` at cruise,
/// `a·τ²/8` at a turnaround, the accelerating path on both sides at a slow
/// corner — and zero at a rest: where the leaders cross a leg seam at rest,
/// at a floor, and at every sample inside a dwell, each of which is kept.
/// The envelope is the largest `ENVELOPE_SLOPE`-Lipschitz field under every
/// target, and `h` blends neighbouring envelope values by a smootherstep
/// (flat to second order at each node, so `h` is C2), or rises off a rest
/// node as `ψ`, linearly at first: a window shrinks to nothing at a rest at
/// most as fast as the distance to it, so it never reaches across.
///
/// `|h'| ≤ WIDTH_SLOPE_BOUND < 1` everywhere, which keeps the smoothed
/// ratio a convex combination of the table's (see [`RatioSmoothing`]).
#[derive(Debug, Default)]
struct WidthField {
    nodes: Vec<WidthNode>,
    candidates: VecDeque<Candidate>,
    seams: Vec<TimedSeam>,
    /// How many of the first `seams` a floor closed: no leg after it is
    /// reached by their ramps.
    closed_seams: usize,
    rule: Option<RatioRampRule>,
    grid: Option<SampleGrid>,
    pending: Option<usize>,
    before_pending: f64,
    last_kept: Option<LastKept>,
    final_prefix: usize,
    /// Where the leaders last rested, which no seam's width reaches across.
    rest_behind: Option<f64>,
    /// Which state the nodes stand in ([`next_revision`]): every change to
    /// a node takes a fresh one.
    revision: u64,
    /// The first path distance whose width may have changed since the
    /// seam clusters were last built ([`SeamTable::refresh`]); none past
    /// the end, every one at the start.
    changed_from: f64,
}

impl WidthField {
    /// Samples the times `leg` covers on the grid the stream laid since the
    /// leaders last stood still: a leg they open from a standstill — the
    /// stream's start, a drain or a dwell — lays a fresh one from its own
    /// start, so the motion after a rest is sampled as it would be planned
    /// from rest. A standstill is the leaders covering less than
    /// [`REST_TARGET_MM`] over a node spacing, whether at speed or by
    /// accelerating: the shaped leaders' fit leaves a dwell's end a rounding
    /// off an exact zero (1e-14 mm/s, 1e-9 mm/s²). Ticks in a time gap
    /// before `leg`, legal only at rest, sample its start.
    fn extend(&mut self, leaders: &ShapedLeaders, leg: &RampLeg, table_end: f64) {
        let grid = &leg.odometer.grid;
        let (t0, t1) = (grid[0], grid[grid.len() - 1]);
        let opens_from_standstill = leaders.speed(t0) <= REST_SPEED_MM_S
            && leaders.accel_norm(t0) <= REST_SPEED_MM_S / NODE_SPACING_S;
        let mut ticks = match self.grid {
            Some(ticks) if !opens_from_standstill => ticks,
            _ => SampleGrid {
                origin: t0,
                next: 0,
            },
        };
        if self.nodes.is_empty() && self.candidates.is_empty() {
            self.place_rest(leg.start_s);
        }
        loop {
            let t = ticks.time(ticks.next);
            if t >= t1 {
                break;
            }
            let s = leg.start_s + leg.odometer.covered_until(t);
            self.place_sample(s.min(table_end), t.max(t0));
            ticks.next += 1;
        }
        self.grid = Some(ticks);
    }

    fn place_sample(&mut self, s: f64, t: f64) {
        if let Some(pending) = self.pending {
            self.resolve(pending, s);
        } else {
            self.before_pending = s;
        }
        self.candidates
            .push_back(Candidate::Time(TimeSample { s, t, base: None }));
        self.pending = Some(self.candidates.len() - 1);
    }

    /// The pending sample's base: half the path between the samples on
    /// either side of it, `next` the one after.
    fn resolve(&mut self, pending: usize, next: f64) {
        let Candidate::Time(sample) = &mut self.candidates[pending] else {
            unreachable!("the pending candidate is a time sample");
        };
        let base = 0.5 * (next.max(sample.s) - self.before_pending);
        sample.base = Some(if base <= REST_TARGET_MM { 0.0 } else { base });
        self.before_pending = sample.s;
    }

    fn add_seam(&mut self, seam: TimedSeam) {
        assert!(
            self.seams.last().is_none_or(|last| seam.t >= last.t),
            "a ratio step at t={} is laid behind the last one at t={}",
            seam.t,
            self.seams.last().map_or(f64::NAN, |last| last.t)
        );
        self.seams.push(seam);
    }

    /// Widens every open seam whose ramp reaches into the leg over
    /// `support` by how the leaders cross what it reaches there. Its ramp
    /// time is not derived yet: the leaders are laid only up to `support`'s
    /// start, and [`Self::assign`] derives none within half the longest ramp
    /// of that.
    fn widen_seams_into(&mut self, crossings: &mut LegCrossings, support: (f64, f64)) {
        let first = self
            .seams
            .partition_point(|seam| seam.t + CLUSTER_REACH_S <= support.0)
            .max(self.closed_seams);
        for seam in &mut self.seams[first..] {
            assert!(
                seam.ramp_time.is_none(),
                "the ratio step at t={} has its ramp time derived before the leg from t={} its \
                 ramp reaches into is laid",
                seam.t,
                support.0
            );
            seam.crossing = seam
                .crossing
                .widest(crossings.over(ramp_reach_within(seam.t, support)));
        }
    }

    /// The ramp time seam `index` asks for, with the steps it gathers.
    fn seam_ramp_time(&mut self, index: usize) -> f64 {
        if let Some(ramp_time) = self.seams[index].ramp_time {
            return ramp_time;
        }
        let seam = self.seams[index];
        let (step, peak_ratio) = self
            .seams
            .iter()
            .filter(|other| (other.t - seam.t).abs() <= CLUSTER_REACH_S)
            .fold((0.0, 0.0_f64), |(step, peak), other| {
                (step + other.step.abs(), peak.max(other.peak_ratio))
            });
        let rule = self.rule.expect("a seam is laid under a ramp rule");
        let ramp_time = rule.ramp_time(&RatioStep {
            ratio_step: step,
            peak_ratio,
            speed_mm_s: seam.crossing.speed,
            accel_mm_s2: seam.crossing.rate,
            jerk_mm_s3: seam.crossing.jerk,
        });
        self.seams[index].ramp_time = Some(ramp_time);
        ramp_time
    }

    /// The seams whose ramp time or width reaches the time `t`.
    fn seams_reaching(&self, t: f64) -> std::ops::Range<usize> {
        let first = self.seams.partition_point(|seam| seam.t < t - SEAM_REACH_S);
        let last = self
            .seams
            .partition_point(|seam| seam.t <= t + SEAM_REACH_S);
        first..last
    }

    /// The ramp time at `t`: the largest a seam around asks for there.
    fn ramp_time_at(&mut self, t: f64) -> f64 {
        self.seams_reaching(t)
            .fold(RATIO_RAMP_MIN_TIME_S, |ramp_time, index| {
                let seam_ramp_time = self.seam_ramp_time(index);
                let beyond_plateau = ((self.seams[index].t - t).abs() - seam_ramp_time).max(0.0);
                ramp_time.max(seam_ramp_time - RAMP_TIME_SLOPE * beyond_plateau)
            })
    }

    /// The width seam `index`'s ramp asks for, `V·τ/2`.
    fn seam_width(&mut self, index: usize) -> f64 {
        0.5 * self.seams[index].crossing.speed * self.seam_ramp_time(index)
    }

    /// Whether seam `index`'s width reaches a node at `s` past it along the
    /// path, with no rest between.
    fn reaches_along_path(&mut self, index: usize, s: f64) -> bool {
        let seam_s = self.seams[index].s;
        self.rest_behind.is_none_or(|rest| seam_s > rest)
            && s - seam_s < SEAM_WIDTH_REACH_WIDTHS * self.seam_width(index)
    }

    /// The width the seams around ask for at the node at `t`, `s`: each its
    /// ramp's `V·τ/2` within [`SEAM_WIDTH_PLATEAU_WIDTHS`] of it on the
    /// path, falling off by [`ENVELOPE_SLOPE`] beyond (see [`WidthField`]),
    /// from every seam within [`SEAM_REACH_S`] of it in time, and from every
    /// other laid between the rests behind it and at `rest_ahead`.
    fn seam_width_at(&mut self, t: f64, s: f64, rest_ahead: f64) -> f64 {
        let rest_behind = self.rest_behind.unwrap_or(f64::NEG_INFINITY);
        let laid = self
            .seams
            .partition_point(|seam| seam.t <= t + SEAM_PATH_REACH_S);
        (0..laid).fold(0.0, |width, index| {
            let seam = self.seams[index];
            let within_time = (seam.t - t).abs() <= SEAM_REACH_S;
            if !(within_time || (rest_behind < seam.s && seam.s < rest_ahead)) {
                return width;
            }
            let seam_width = self.seam_width(index);
            let beyond_plateau =
                ((s - seam.s).abs() - SEAM_WIDTH_PLATEAU_WIDTHS * seam_width).max(0.0);
            width.max(seam_width - ENVELOPE_SLOPE * beyond_plateau)
        })
    }

    /// Where the leaders rest next after the first candidate, as far as
    /// the candidates show it.
    fn rest_ahead(&self) -> f64 {
        self.candidates
            .iter()
            .skip(1)
            .find_map(|candidate| match *candidate {
                Candidate::Rest(s) => Some(s),
                Candidate::Time(sample) if sample.base == Some(0.0) => Some(sample.s),
                Candidate::Time(_) => None,
            })
            .unwrap_or(f64::INFINITY)
    }

    /// Turns every candidate at or before `through` whose base is known
    /// into a node or drops it, in order, then every seam no candidate
    /// still to come takes a ramp time or a width from.
    fn assign(&mut self, through: f64) {
        while let Some(&candidate) = self.candidates.front() {
            match candidate {
                Candidate::Rest(s) => self.rest_at(s),
                Candidate::Time(sample) => {
                    let Some(base) = sample.base.filter(|_| sample.t <= through) else {
                        break;
                    };
                    if base == 0.0 {
                        self.rest_at(sample.s);
                    } else {
                        self.keep_if_due(sample, base);
                    }
                }
            }
            self.candidates.pop_front();
            self.pending = self.pending.and_then(|pending| pending.checked_sub(1));
        }
        let (needed_from, front_s) = match self.candidates.front() {
            Some(&Candidate::Time(sample)) => (sample.t, sample.s),
            Some(&Candidate::Rest(s)) => (through, s),
            None => (
                through,
                self.nodes.last().map_or(f64::NEG_INFINITY, |node| node.s),
            ),
        };
        let needed_from = needed_from - TARGET_DELAY_S;
        let mut drop = 0;
        while drop < self.seams.len()
            && self.seams[drop].t < needed_from
            && !self.reaches_along_path(drop, front_s)
        {
            drop += 1;
        }
        self.seams.drain(..drop);
        self.closed_seams = self.closed_seams.saturating_sub(drop);
    }

    fn rest_at(&mut self, s: f64) {
        self.push_rest(s);
        let rest = self.nodes.last().expect("a rest leaves a node").s;
        self.last_kept = Some(LastKept::Rest(rest));
        self.rest_behind = Some(rest);
    }

    /// Makes `sample` a node of target `base` scaled to the ramp time
    /// there, or the width the seams around ask for if wider, once it lies
    /// far enough past the last node kept in time and along the path, or
    /// past the rest the leaders last left for its cone to hold that target.
    /// Every node kept so opens a window: a node the cone of a rest held at
    /// zero width would space the nodes after it, and a motion shorter than
    /// half a ramp time between two rests would keep no open node, a ratio
    /// step in it unramped.
    fn keep_if_due(&mut self, sample: TimeSample, base: f64) {
        let ramp_time = self.ramp_time_at(sample.t);
        let rest_ahead = self.rest_ahead();
        let target = (base * ramp_time / RATIO_RAMP_MIN_TIME_S)
            .max(self.seam_width_at(sample.t, sample.s, rest_ahead));
        let last_kept = self
            .last_kept
            .expect("the width field lays a rest before its first sample");
        let due = match last_kept {
            LastKept::Rest(rest) => target <= ENVELOPE_SLOPE * (sample.s - rest),
            LastKept::Moving(last) => {
                sample.t >= last.t + 0.5 * last.ramp_time - 0.5 * NODE_SPACING_S
                    && sample.s - last.s >= NODE_SPACING_PER_WIDTH * target.min(last.target)
            }
        };
        if due {
            if let LastKept::Rest(rest) = last_kept {
                self.push_cone_peak(rest, sample.s, target);
            }
            self.push(sample.s, target);
            assert!(
                self.nodes.last().is_some_and(|node| node.envelope > 0.0),
                "the width node kept at {} mm opens no window",
                sample.s
            );
            self.last_kept = Some(LastKept::Moving(KeptNode {
                t: sample.t,
                s: sample.s,
                ramp_time,
                target,
            }));
        }
    }

    /// The peak where the cone of the rest at `rest` meets the cone of the
    /// first node kept past it, at `s` with `target`, as a node of its own.
    /// Every sample between was due no node only because the rest's cone
    /// capped its target, so the envelope there is that cone up to the
    /// peak. Rising off the rest toward the node alone, the width would rise
    /// at `3·target/(s − rest)` per millimetre, under the cone wherever the
    /// node lies far past it for its target — 0.21 where a slow corner
    /// 14.8 mm past a prime first fitted under it — while the ramp of a step
    /// beside the rest ([`RestSide::narrows_a_step`],
    /// [`geometry::rest_room_reach_mm`]) is budgeted on the cone. Rising to
    /// the peak, it holds the cone at least, and falls to the node at
    /// `15/8` of the slope. A node within its spacing of the peak keeps
    /// the cone without one.
    fn push_cone_peak(&mut self, rest: f64, s: f64, target: f64) {
        let peak_mm = 0.5 * (target / ENVELOPE_SLOPE + (s - rest));
        if s - (rest + peak_mm) >= NODE_SPACING_PER_WIDTH * target {
            self.push(rest + peak_mm, ENVELOPE_SLOPE * peak_mm);
        }
    }

    /// A floor at `s`: the leaders rest there and nothing after reaches
    /// back, so the sample still waiting on the path ahead takes `s` as it,
    /// and every candidate becomes a node or is dropped.
    fn close(&mut self, s: f64) {
        if let Some(pending) = self.pending.take() {
            self.resolve(pending, s);
        }
        self.place_rest(s);
        self.assign(f64::INFINITY);
        self.closed_seams = self.seams.len();
    }

    /// Every ratio step laid has its node, as at a floor: a new rule
    /// derives only the ramps still to come.
    fn is_settled(&self) -> bool {
        self.candidates.is_empty() && self.seams.is_empty()
    }

    fn place_rest(&mut self, s: f64) {
        self.candidates.push_back(Candidate::Rest(s));
    }

    /// A rest at `s` among the candidates the leg being laid placed, in
    /// path order: a ratio step the table lays where the leaders rest, which
    /// the path they cover may reach only to a rounding.
    fn place_rest_among_laid(&mut self, s: f64) {
        let at = self
            .candidates
            .partition_point(|candidate| candidate.s() <= s);
        self.candidates.insert(at, Candidate::Rest(s));
        self.pending = self
            .pending
            .map(|pending| if at <= pending { pending + 1 } else { pending });
    }

    /// A rest past a node already at zero width keeps its own place rather
    /// than stacking on it ([`Self::push`]): `h` is zero between the two
    /// either way, and a leg seam the leaders cross at rest needs its rest
    /// node on the seam itself, or the window there, however narrow,
    /// straddles the seam and reads half its ratio step.
    ///
    /// Leaders that leave a rest and come back to one within a node spacing
    /// leave no time sample in between, and `h` would be zero all along the
    /// path they cover: a ratio step there would step the follower's
    /// velocity. A node halfway along it, its target half that path as a
    /// sample's base, opens the widest window the rests' cones allow. A
    /// sample kept on the rest point itself stacks on the rest and closes to
    /// zero width with it, so it is dropped first and leaves the path it
    /// came after to the halfway node. So do samples this rest's cone caps
    /// every one of, whose window it would crush to slivers along the whole
    /// motion: the halfway node takes the largest of their targets.
    fn push_rest(&mut self, s: f64) {
        self.drop_samples_stacking_on(s);
        let crushed_target = self.drop_samples_crushed_by(s);
        let Some(&last) = self.nodes.last() else {
            return self.push(s, 0.0);
        };
        if last.envelope != 0.0 {
            return self.push(s, 0.0);
        }
        if s - last.s > REST_TARGET_MM {
            self.push(
                0.5 * (last.s + s),
                crushed_target.unwrap_or(0.5 * (s - last.s)),
            );
            self.push(s, 0.0);
        } else if s > last.s {
            self.nodes.push(WidthNode {
                s,
                target: 0.0,
                envelope: 0.0,
            });
            self.changed(self.nodes.len() - 1);
        }
    }

    /// Drops every moving node since the last rest node where a rest at
    /// `rest` caps each of their targets, and returns the largest. Each
    /// dropped node's envelope is under both rests' cones at its place, at
    /// most `ENVELOPE_SLOPE` times half the motion, which the halfway node
    /// reaches with the largest target.
    fn drop_samples_crushed_by(&mut self, rest: f64) -> Option<f64> {
        let opening = self.nodes.iter().rposition(|node| node.target == 0.0)?;
        let moving = &self.nodes[opening + 1..];
        let crushed = !moving.is_empty()
            && moving
                .iter()
                .all(|node| ENVELOPE_SLOPE * (rest - node.s) < node.target);
        if !crushed {
            return None;
        }
        assert!(
            opening + 1 >= self.final_prefix,
            "a rest at {rest} mm would drop the final ratio ramp width node at {} mm",
            self.nodes[opening + 1].s
        );
        let target = moving.iter().map(|node| node.target).fold(0.0, f64::max);
        self.nodes.truncate(opening + 1);
        self.changed(opening + 1);
        Some(target)
    }

    fn drop_samples_stacking_on(&mut self, rest: f64) {
        while self.nodes.len() > self.final_prefix
            && self
                .nodes
                .last()
                .is_some_and(|last| last.target > 0.0 && rest <= last.s + REST_TARGET_MM)
        {
            self.nodes.pop();
            self.changed(self.nodes.len());
        }
    }

    /// A node within [`REST_TARGET_MM`] past the last one stacks with it:
    /// the leaders cover no more between them than the fit's back-and-forth
    /// around a rest, and a node a rounding short of a rest would take the
    /// rest's cone as a sliver of an envelope that the smootherstep
    /// flattens onto, narrowing every window short of the rest to as much.
    /// The stack sits where the leaders rest: a rest pulls the moving nodes
    /// stacked short of it onto itself, as the window must close exactly
    /// there whatever the leaders do after.
    fn push(&mut self, s: f64, target: f64) {
        let s = match self.nodes.last() {
            None => s,
            Some(last) => {
                assert!(
                    s >= last.s - SPAN_LOOKUP_SLACK_MM,
                    "a ratio ramp width node at {s} mm falls behind the last one at {} mm",
                    last.s
                );
                if s > last.s + REST_TARGET_MM {
                    s
                } else if target == 0.0 && s > last.s {
                    self.pull_moving_stack_onto_rest(s);
                    s
                } else {
                    last.s
                }
            }
        };
        self.nodes.push(WidthNode {
            s,
            target,
            envelope: target,
        });
        self.settle(self.nodes.len() - 1);
    }

    fn pull_moving_stack_onto_rest(&mut self, rest: f64) {
        let stacked_at = self.nodes.last().expect("a stack to pull").s;
        let moving = self
            .nodes
            .iter()
            .rev()
            .take_while(|node| node.s == stacked_at && node.envelope > 0.0)
            .count();
        let first_pulled = self.nodes.len() - moving;
        assert!(
            moving == 0 || first_pulled >= self.final_prefix,
            "a rest at {rest} mm would pull the final ratio ramp width node at {stacked_at} mm \
             onto itself"
        );
        for node in &mut self.nodes[first_pulled..] {
            node.s = rest;
        }
        self.changed(first_pulled);
    }

    /// Records that node `index` or those past it changed: the width
    /// changes from the node before it on, and with a fresh revision.
    /// Dropping nodes off the front changes no width a kept seam reads, the
    /// field keeping the node at or before the first.
    fn changed(&mut self, index: usize) {
        let from = index
            .checked_sub(1)
            .and_then(|before| self.nodes.get(before))
            .map_or(f64::NEG_INFINITY, |node| node.s);
        self.changed_from = self.changed_from.min(from);
        self.revision = next_revision();
    }

    fn cone(&self, from: usize, to: usize) -> f64 {
        self.nodes[from].envelope + ENVELOPE_SLOPE * (self.nodes[to].s - self.nodes[from].s).abs()
    }

    fn settle(&mut self, index: usize) {
        let mut envelope = self.nodes[index].envelope.min(self.nodes[index].target);
        if index > 0 {
            envelope = envelope.min(self.cone(index - 1, index));
        }
        self.nodes[index].envelope = envelope;
        let mut lowest = index;
        for back in (0..index).rev() {
            let lowered = self.cone(back + 1, back);
            if lowered >= self.nodes[back].envelope {
                break;
            }
            self.nodes[back].envelope = lowered;
            lowest = back;
        }
        self.changed(lowest);
    }

    /// A node is final once no later node can lower it: every node still
    /// to come lies past the frontier, the first candidate, and its cone is
    /// at least the slope times the distance there.
    fn is_final(&self, index: usize) -> bool {
        let node = self.nodes[index];
        let frontier = self.candidates.front().map_or_else(
            || self.nodes.last().map_or(node.s, |last| last.s),
            Candidate::s,
        );
        node.envelope <= ENVELOPE_SLOPE * (frontier - node.s)
    }

    /// The width is final up to the last node of the final prefix.
    fn final_through(&mut self) -> Option<f64> {
        while self.final_prefix < self.nodes.len() && self.is_final(self.final_prefix) {
            self.final_prefix += 1;
        }
        self.final_prefix
            .checked_sub(1)
            .map(|last| self.nodes[last].s)
    }

    /// The node at `s`, or the two around it. A query within the span
    /// lookup slack past either end node reads that node, as the span table
    /// extends its own ends.
    fn interval(&self, s: f64) -> Result<usize, (usize, usize)> {
        let (first, last) = (self.nodes[0].s, self.nodes[self.nodes.len() - 1].s);
        assert!(
            s >= first - SPAN_LOOKUP_SLACK_MM && s <= last + SPAN_LOOKUP_SLACK_MM,
            "the ratio ramp width is asked for at {s} mm, outside its nodes [{first}, {last}]"
        );
        let after = self.nodes.partition_point(|node| node.s < s);
        if after == self.nodes.len() {
            return Ok(after - 1);
        }
        if self.nodes[after].s == s || after == 0 {
            return Ok(after);
        }
        Err((after - 1, after))
    }

    /// The formula of the width at `s`: on a node that has a node past it,
    /// the interval that node opens, whose derivatives it takes from its
    /// start; else [`Self::interval`]'s.
    fn formula_at(&self, s: f64) -> Result<usize, (usize, usize)> {
        match self.interval(s) {
            Ok(node)
                if self.nodes[node].s == s
                    && self.nodes.get(node + 1).is_some_and(|next| next.s > s) =>
            {
                Err((node, node + 1))
            }
            interval => interval,
        }
    }

    fn jet(&self, s: f64) -> Jet {
        let (a, b) = match self.formula_at(s) {
            Ok(node) => return Jet::constant(self.nodes[node].envelope),
            Err(interval) => interval,
        };
        let (a, b) = (self.nodes[a], self.nodes[b]);
        let x = Jet::variable(s).offset(-a.s).scale(1.0 / (b.s - a.s));
        match (a.envelope == 0.0, b.envelope == 0.0) {
            (true, true) => Jet::ZERO,
            (true, false) => x.poly(&RISE_FROM_REST).scale(b.envelope),
            (false, true) => x.poly(&FALL_TO_REST).scale(a.envelope),
            (false, false) => x
                .poly(&SMOOTHERSTEP)
                .scale(b.envelope - a.envelope)
                .offset(a.envelope),
        }
    }

    /// The half-width a seam at `s` is smoothed over: the width there;
    /// none past the last node, whose width no evaluation within
    /// [`RatioSmoothing::final_through`] reads, nor before the first, where
    /// the field opened past a seam no evaluation reads.
    fn step_width(&self, s: f64) -> f64 {
        let (Some(first), Some(last)) = (self.nodes.first(), self.nodes.last()) else {
            return 0.0;
        };
        if s < first.s - SPAN_LOOKUP_SLACK_MM || s > last.s + SPAN_LOOKUP_SLACK_MM {
            return 0.0;
        }
        self.width(s)
    }

    /// [`Self::jet`]'s value alone, by the same arithmetic.
    fn width(&self, s: f64) -> f64 {
        match self.interval(s) {
            Ok(node) => self.nodes[node].envelope,
            Err((a, _)) => self.width_after(a, s),
        }
    }

    /// The width at `s`, strictly between node `a` and the next.
    fn width_after(&self, a: usize, s: f64) -> f64 {
        let (a, b) = (self.nodes[a], self.nodes[a + 1]);
        let x = (s - a.s) * (1.0 / (b.s - a.s));
        match (a.envelope == 0.0, b.envelope == 0.0) {
            (true, true) => 0.0,
            (true, false) => horner(x, &RISE_FROM_REST) * b.envelope,
            (false, true) => horner(x, &FALL_TO_REST) * a.envelope,
            (false, false) => horner(x, &SMOOTHERSTEP) * (b.envelope - a.envelope) + a.envelope,
        }
    }

    /// [`Self::width_after`], by the same arithmetic, with the width's
    /// slope along the path.
    fn width_and_slope_after(&self, a: usize, s: f64) -> (f64, f64) {
        let (a, b) = (self.nodes[a], self.nodes[a + 1]);
        let per_s = 1.0 / (b.s - a.s);
        let x = (s - a.s) * per_s;
        let scaled = |coeffs: &[f64], scale: f64, offset: f64| {
            let (value, slope) = horner_with_slope(x, coeffs);
            (value * scale + offset, slope * scale * per_s)
        };
        match (a.envelope == 0.0, b.envelope == 0.0) {
            (true, true) => (0.0, 0.0),
            (true, false) => scaled(&RISE_FROM_REST, b.envelope, 0.0),
            (false, true) => scaled(&FALL_TO_REST, a.envelope, 0.0),
            (false, false) => scaled(&SMOOTHERSTEP, b.envelope - a.envelope, a.envelope),
        }
    }

    /// A reader of [`Self::width`] for a search whose queries close in on
    /// one point: it tries the node interval of its last query first.
    fn searcher(&self) -> WidthSearch<'_> {
        WidthSearch {
            field: self,
            last: std::cell::Cell::new(None),
        }
    }

    /// The nodes bounding `[lo, hi]` and every node between: the last node
    /// at or before `lo` through the first at or past `hi`. On nodes stacked
    /// at a rest the first at `hi == lo` comes before the last at `lo`, and
    /// the last at `lo` bounds both ends.
    fn covering(&self, lo: f64, hi: f64) -> &[WidthNode] {
        assert!(
            lo <= hi,
            "the ratio ramp width is bounded over [{lo}, {hi}] mm, a reversed stretch"
        );
        let first = self
            .nodes
            .partition_point(|node| node.s <= lo)
            .saturating_sub(1);
        let last = self.nodes.partition_point(|node| node.s < hi).max(first);
        &self.nodes[first..=last.min(self.nodes.len() - 1)]
    }

    /// `(max h, max |h''|)` over `[lo, hi]`.
    fn bounds(&self, lo: f64, hi: f64) -> (f64, f64) {
        let nodes = self.covering(lo, hi);
        let widest = nodes.iter().map(|node| node.envelope).fold(0.0, f64::max);
        let curvature = nodes
            .windows(2)
            .filter(|pair| pair[1].s > pair[0].s)
            .map(|pair| {
                let span = pair[1].s - pair[0].s;
                let (a, b) = (pair[0].envelope, pair[1].envelope);
                if a == 0.0 || b == 0.0 {
                    REST_PEAK_CURVATURE * a.max(b) / (span * span)
                } else {
                    SMOOTHERSTEP_PEAK_CURVATURE * (b - a).abs() / (span * span)
                }
            })
            .fold(0.0, f64::max);
        (widest, curvature)
    }

    fn prune_before(&mut self, s: f64) {
        let keep = self
            .nodes
            .partition_point(|node| node.s <= s)
            .saturating_sub(1)
            .min(self.final_prefix.saturating_sub(1));
        self.nodes.drain(..keep);
        self.final_prefix -= keep;
    }
}

/// See [`WidthField::searcher`].
struct WidthSearch<'a> {
    field: &'a WidthField,
    last: std::cell::Cell<Option<usize>>,
}

impl WidthSearch<'_> {
    /// [`WidthField::width`]: strictly inside the last query's node
    /// interval, the interval search would find it again.
    fn width(&self, s: f64) -> f64 {
        let nodes = &self.field.nodes;
        if let Some(a) = self.last.get() {
            if nodes[a].s < s && s < nodes[a + 1].s {
                return self.field.width_after(a, s);
            }
        }
        match self.field.interval(s) {
            Ok(node) => nodes[node].envelope,
            Err((a, _)) => {
                self.last.set(Some(a));
                self.field.width_after(a, s)
            }
        }
    }

    /// [`Self::width`] with its slope along the path, read as flat on a
    /// node.
    fn width_and_slope(&self, s: f64) -> (f64, f64) {
        let nodes = &self.field.nodes;
        if let Some(a) = self.last.get() {
            if nodes[a].s < s && s < nodes[a + 1].s {
                return self.field.width_and_slope_after(a, s);
            }
        }
        match self.field.interval(s) {
            Ok(node) => (nodes[node].envelope, 0.0),
            Err((a, _)) => {
                self.last.set(Some(a));
                self.field.width_and_slope_after(a, s)
            }
        }
    }
}

/// The extrusion ratio a ratio-ramped follower lays: the span table's
/// ratio `r(s)` smoothed along the path by the kernel `K(u) = ¾(1 − u²)`,
/// so that every step of the table — in the ratio or in its slope, at a
/// seam or across a cluster of seams however short — is spread over the
/// path the leaders cover in about `τ` around it, and an advance applied to
/// it emits no position or velocity step. Each cluster of seams
/// ([`SeamCluster`]) splits its change in two: its change of slope, which
/// a window of the half-width `h(s)` the leaders' speed sets at `s`
/// ([`WidthField`]) averages as it moves along the path; and the rest, its
/// ratio steps and slides, flat either side, which one window of the
/// narrowest width its seams hold averages where it lies.
///
/// The smoothing acts on the cumulative extrusion `E` (`E' = r`):
///
/// `Ẽ(s) = ∫ E(s − h(s)·u)·K(u) du`,  `r̃ = Ẽ' = ∫ r(s − h·u)·(1 − h'·u)·K(u) du`
///
/// for the moving share, and the same with `h` the cluster's own and
/// `h' = 0` for each cluster's stationary share.
///
/// - Exact material: a symmetric kernel reproduces a linear `E`, so
///   `Ẽ = E` wherever the ratio is constant across the windows — after
///   every transition and at every rest, where `h = 0` and no cluster's
///   window reaches, a seam's width being at most [`ENVELOPE_SLOPE`] of
///   its distance from the rest. Along a sloped stretch the moving share
///   leads by `0.1·h²·r'` and gives it back where the slope ends.
/// - Range: `|h'| < 1` keeps the moving weights `(1 − h'u)·K(u)`
///   nonnegative and of unit sum.
/// - C1: `K` vanishes at its ends and `h` is C2, so `r̃'` is continuous;
///   `r̃''` steps where a seam enters or leaves a window.
/// - Span-independent: cutting a span into pieces of equal ratio changes
///   nothing beyond rounding; a seam the cut lays only by rounding joins no
///   cluster ([`SEAM_SLOPE_ROUNDING`]).
///
/// With the seams `σᵢ` (ratio step `Δᵢ`, slope step `Δmᵢ`) a window of
/// half-width `h` holds and `x = (s − σ)/h`, the correction `X = Ẽ − E`
/// it makes is
///
/// `X = 0.1·h²·m₋ + Σ_{|x|<1} Δ·h·G₁(x) + Δm·h²·G₂(x)`
///
/// with `m₋` the slope at `s − h` and `G₁ = P₁ − x₊`, `G₂ = P₂ − x₊²/2`
/// the kernel's smoothing of a ramp and a parabola ([`SMOOTHED_RAMP`],
/// [`SMOOTHED_HALF_SQUARE`]). Every `Gᵢ` and its first two derivatives
/// vanish at `x = −1`, and `G₂ → 0.1` at `x = 1` where the seam's slope
/// step joins `m₋`, so `X` is C2 in `s`: the ratio `r + X'` is C1.
///
/// Bounds, `λ = WIDTH_SLOPE_BOUND`: a cluster whose ratio varies by `V` in
/// total, opening and closing on spans of constant ratio, crossed at path
/// speed `v`, adds `k·r̃'·v²` to an advanced extruder velocity, with
/// `|r̃'| ≤ max|r'| + ¾·V/h`; the moving share, with
///
/// `|r̃'| ≤ (1 + λ)²·max|r'| + 0.375·|h''|·(r_max − r_min)`.
///
/// At cruise `h = v·τ/2`, so a cluster adds `1.5·k·V·v/τ` to the extruder
/// velocity and `6·k·V·v/τ²` to its acceleration, whether it is one seam
/// or a slide of many.
#[derive(Debug, Default)]
pub(super) struct RatioSmoothing {
    width: WidthField,
    laid: SeamTable,
    advance: SeamTable,
    valleys: Vec<AdvanceValley>,
    open_rest: Option<OpenRest>,
    floor: f64,
    taken_through: f64,
    reach_into_next: Option<Crossing>,
    laid_tails: VecDeque<LegTail>,
    backlog: VecDeque<PendingSegment>,
    /// Crawls whose valleys wait for the ratio or the leaders past the laid
    /// table: the next leg's spans decide whether one is laid, and how
    /// wide its after side reaches.
    waiting_crawls: Vec<(f64, Crossing)>,
    trail: SideTrail,
}

impl RatioSmoothing {
    pub(super) fn clear(&mut self) {
        *self = Self::default();
    }

    /// Nothing to smooth: the laid table has no seam and no slope.
    pub(super) fn is_inert(&self) -> bool {
        self.laid.is_inert()
    }

    /// No window may reach back past `s`: the leaders rest there, or the
    /// stream drained there and what precedes it is already committed. A
    /// rest valley still open closes there, with nothing after it.
    pub(super) fn close_at(&mut self, s: f64) {
        self.floor = self.floor.max(s);
        self.reach_into_next = None;
        self.waiting_crawls.clear();
        self.laid_tails.clear();
        self.close_rest(s, 0.0);
        self.width.close(s);
        self.refresh_clusters();
    }

    /// Rebuilds the tables' clusters where their seams or the width field
    /// changed ([`SeamTable::refresh`]): every entry that lays either ends
    /// here, before anything reads them.
    fn refresh_clusters(&mut self) {
        self.laid.refresh(&self.width);
        self.advance.refresh(&self.width);
        self.width.changed_from = f64::INFINITY;
    }

    /// A rest valley opens with no end: the advance reads nothing from its
    /// start on until a leg that moves again closes it.
    fn close_rest(&mut self, end: f64, after: f64) {
        if self.open_rest.take().is_none() {
            return;
        }
        let valley = self
            .valleys
            .last_mut()
            .expect("an open rest valley is the last laid");
        assert_eq!(
            valley.kind,
            ValleyKind::Rest,
            "follower projection: a valley {valley:?} was laid over an open rest"
        );
        valley.end = end.max(valley.start);
        valley.after = after;
        self.assert_weight_continuous();
    }

    /// Takes in `leg`'s spans: the width nodes it covers, the seams the
    /// table gained with the ramp time `rule` derives for each, the seams
    /// `advance_spans`, the table an advance reads, gained, and the advance
    /// valleys its opening seam and its slowest point inside need, and the
    /// one the last leg's slowest point waited on these spans for.
    pub(super) fn lay(
        &mut self,
        (spans, advance_spans): (&[RatioSpan], &[RatioSpan]),
        leg: &RampLeg,
        rule: &RatioRampRule,
    ) {
        assert!(
            self.width.rule.is_none_or(|laid| laid == *rule) || self.width.is_settled(),
            "a follower's ratio ramp rule changed from {:?} to {rule:?} with ratio steps still \
             waiting on their ramp times",
            self.width.rule
        );
        self.width.rule = Some(*rule);
        let leaders = leg.shaped_leaders;
        let table_end = spans.last().map_or(leg.start_s, |last| last.s1);
        if table_end <= leg.start_s {
            self.width.extend(leaders, leg, table_end);
            for crawl in std::mem::take(&mut self.waiting_crawls) {
                self.lay_crawl_valley((spans, advance_spans), rule, crawl, f64::INFINITY);
            }
            self.close_at(leg.start_s);
            return;
        }
        self.trail.record(leg, table_end);
        let (t_start, t_end) = leg.support;
        let opening = Crossing::valley_at(leaders, t_start);
        let crossing = self
            .reach_into_next
            .map_or(opening, |closing| closing.common(opening));
        let seam = leg.start_s;
        let reach = crossing.valley_reach().min(seam - self.floor);
        if reach <= SPAN_MIN_LEN_MM {
            self.width.place_rest(seam);
        }
        self.width.extend(leaders, leg, table_end);
        let new_seams = self.record_seams((spans, advance_spans));
        for crawl in std::mem::take(&mut self.waiting_crawls) {
            self.lay_crawl_valley((spans, advance_spans), rule, crawl, table_end);
        }
        let closes_a_rest =
            self.lay_rest_valley((spans, advance_spans), leg, rule, crossing, table_end);
        if reach > SPAN_MIN_LEN_MM && self.open_rest.is_none() && !closes_a_rest {
            self.lay_crawl_valley((spans, advance_spans), rule, (seam, crossing), table_end);
        }
        let slowest = (!leg.rests && self.open_rest.is_none())
            .then(|| slowest_crawl(leaders, leg, table_end))
            .flatten();
        if let Some(crawl) = slowest {
            self.lay_crawl_valley((spans, advance_spans), rule, crawl, table_end);
        }
        let mut crossings = LegCrossings::of(leaders);
        self.width.widen_seams_into(&mut crossings, leg.support);
        self.time_seams(
            &spans[new_seams.start.saturating_sub(1)..],
            &mut crossings,
            leg,
        );
        self.keep_tail(&mut crossings, leg);
        self.width.assign(t_end - TARGET_DELAY_S);
        self.reach_into_next = Some(Crossing::valley_at(leaders, t_end));
        self.refresh_clusters();
    }

    /// A valley at the crawl `turn` where reading the path speed through it
    /// at the ratio within its reach would ask too much of the follower,
    /// each side as wide as its rebuild needs ([`crawl_side`]), the reach
    /// no further back than the leaders last rested
    /// ([`Self::rested_through`]). The advance there reads the ratio laid
    /// after the table's end too, so a reach running past `table_end`, or
    /// an after side past the leaders probed, waits for the next leg; an
    /// infinite `table_end`, where the leaders come to rest past the table,
    /// lays it with what is laid. A crawl behind one still waiting waits
    /// too: valleys are laid in path order.
    fn lay_crawl_valley(
        &mut self,
        tables: (&[RatioSpan], &[RatioSpan]),
        rule: &RatioRampRule,
        (turn, crossing): (f64, Crossing),
        table_end: f64,
    ) {
        if !self.waiting_crawls.is_empty() {
            self.waiting_crawls.push((turn, crossing));
            return;
        }
        let reach = crossing
            .valley_reach()
            .min(turn - self.rested_through(turn));
        if reach <= SPAN_MIN_LEN_MM {
            return;
        }
        if !crossing.needs_valley(rule, peak_ratio_within(tables.0, turn, reach)) {
            if turn + reach > table_end {
                self.waiting_crawls.push((turn, crossing));
            }
            return;
        }
        if let Err(Unlaid::Waits) =
            self.lay_valley(tables, rule, (turn, crossing), reach, table_end)
        {
            self.waiting_crawls.push((turn, crossing));
        }
    }

    /// Where the leaders last rested before `turn`: the floor, or the end
    /// of the last rest valley laid before it.
    fn rested_through(&self, turn: f64) -> f64 {
        self.valleys
            .iter()
            .rev()
            .find(|valley| valley.kind == ValleyKind::Rest && valley.end <= turn)
            .map_or(self.floor, |rest| rest.end.max(self.floor))
    }

    /// The valley at the crawl `turn`, unless the last one laid already
    /// holds the weight flat at zero there ([`AdvanceValley::holds_zero_at`]):
    /// a crawl at a seam is the slowest point of the leg before or after it
    /// too, read a hair inside that leg. Each side is as wide as its rebuild
    /// needs ([`crawl_side`]), past the leaders' `reach`; the before side
    /// only where the valley does not widen the last one over it
    /// ([`Self::place_valley`]). Where no before side reaching no further
    /// back than the leaders last rested fits, the valley holds the weight
    /// at zero back to there instead: the leaders leave that rest with no
    /// lead to release, and the after side rebuilds it. A follower the
    /// turning guard does not hold ([`guarded_by_the_turning_guard`]) has
    /// no room left for its sides: its before side is the leaders' reach,
    /// as far back as the projection still holds the path.
    ///
    /// For a follower the guard holds, a ratio step of either table within
    /// [`geometry::crawl_step_window`] of the crawl, or inside a side, is
    /// the valley's own ([`StepWindows`]): the weight is held at zero over
    /// its capped window, so the advance reads none of its ramp, and the
    /// sides start past it, from the leaders crossing its edges.
    fn lay_valley(
        &mut self,
        tables: (&[RatioSpan], &[RatioSpan]),
        rule: &RatioRampRule,
        crawl: (f64, Crossing),
        reach: f64,
        table_end: f64,
    ) -> Result<(), Unlaid> {
        let turn = crawl.0;
        if self
            .valleys
            .last()
            .is_some_and(|valley| valley.holds_zero_at(turn, SPAN_LOOKUP_SLACK_MM))
        {
            return Ok(());
        }
        self.refresh_clusters();
        let guarded = guarded_by_the_turning_guard(rule);
        let mut steps = StepWindows::around(turn, crawl.1, reach, guarded);
        let untaken = |s: f64| self.step_window_untaken(s);
        steps.own_within(tables, (turn - steps.window, turn + steps.window), &untaken);
        let rested = self.rested_through(turn);
        let laid = (&*self, tables.1, table_end);
        let (zero, sides, floors) = loop {
            let zero = steps.zero_stretch();
            let edges = steps.edges(&self.trail, table_end)?;
            let after = crawl_side(Side::After, edges.1, reach, laid, rule, f64::INFINITY)?
                .unwrap_or_else(|| {
                    panic!(
                        "follower projection: no width of the After side of the crawl valley at \
                         s={turn} ({:?}) fits its rebuild within the follower's limit",
                        edges.1
                    )
                });
            let widens_the_last = guarded && self.last_crawl_reach_end() > zero.0 - reach;
            let (start, before) = if !guarded {
                (zero.0, reach.min(self.back_reach_limit(zero.0)))
            } else if widens_the_last {
                (zero.0, reach)
            } else {
                crawl_side(Side::Before, edges.0, reach, laid, rule, zero.0 - rested)?
                    .map_or((rested, 0.0), |before| (zero.0, before))
            };
            let side_from = if widens_the_last {
                zero.0
            } else {
                start - before
            };
            if !steps.own_within(tables, (side_from, zero.1 + after), &untaken) {
                break (
                    (start, zero.1),
                    (before, after),
                    (edges.0.floor, edges.1.floor),
                );
            }
        };
        steps.assert_flow_fits(self, tables.0, rule);
        for owned in &steps.owned {
            for &s in &owned.run {
                assert!(
                    self.step_window_untaken(s),
                    "follower projection: a crawl valley caps the seam at s={s}, whose window \
                     the projection has taken through s={}",
                    self.taken_through
                );
                let seams = match owned.table {
                    Table::Laid => &mut self.laid,
                    Table::Advance => &mut self.advance,
                };
                seams.cap_step(s, steps.window);
            }
        }
        self.place_valley(turn, (zero, sides), floors);
        Ok(())
    }

    /// Whether the projection has taken no path the window the width field
    /// lays around the seam at `s` reaches: none has read its ramp yet, so
    /// a crawl valley may still cap it.
    fn step_window_untaken(&self, s: f64) -> bool {
        let read_from = match self.width.nodes.first() {
            None => s,
            Some(first) if s >= first.s => s - self.width.step_width(s),
            Some(_) => f64::NEG_INFINITY,
        };
        self.taken_through <= read_from
    }

    fn last_crawl_reach_end(&self) -> f64 {
        self.valleys
            .iter()
            .rev()
            .find(|valley| valley.kind == ValleyKind::Crawl)
            .map_or(f64::NEG_INFINITY, AdvanceValley::reach_end)
    }

    /// A crawl valley at `turn`, its weight at zero from `start` on, with
    /// its sides `before` and `after` wide. A crawl whose sides would
    /// overlap the last crawl valley's widens it instead: the advance
    /// releases its lead once and rebuilds it once across both, from this
    /// crawl, holding the weight at zero between them, where two valleys
    /// would multiply their weights and rebuild it steeper than either.
    /// Laid crawl valleys never overlap. The turning guard leaves room for
    /// the sides from the same rule, so a new crawl valley reaches back all
    /// of `before` or fails loudly: the guard cannot see what the
    /// projection has taken.
    fn place_valley(
        &mut self,
        turn: f64,
        ((start, end), (before, after)): ((f64, f64), (f64, f64)),
        (floor_before, floor_after): (f64, f64),
    ) {
        if self
            .valleys
            .last()
            .is_some_and(|valley| valley.holds_zero_at(turn, SPAN_LOOKUP_SLACK_MM))
        {
            return;
        }
        if let Some(laid) = self
            .valleys
            .iter()
            .find(|valley| valley.holds_zero_at(turn, SPAN_LOOKUP_SLACK_MM))
        {
            panic!(
                "follower projection: the crawl at s={turn} lies where the advance valley \
                 {laid:?} already holds the weight at zero, behind the last one laid"
            );
        }
        let overlapped = self
            .valleys
            .iter_mut()
            .rev()
            .find(|valley| valley.kind == ValleyKind::Crawl)
            .filter(|last| start - before < last.reach_end());
        match overlapped {
            Some(last) => {
                assert!(
                    turn > last.end,
                    "follower projection: the crawl at s={turn} lies behind the end of the last \
                     crawl valley {last:?}"
                );
                last.end = end;
                last.after = after;
                last.floor_after = floor_after;
            }
            None => {
                let back = self.back_reach_limit(turn);
                assert!(
                    back >= turn - start + before,
                    "follower projection: the crawl valley at s={turn} reaches {} mm back, but \
                     the projection has taken the path to {back} mm before it",
                    turn - start + before
                );
                self.valleys.push(AdvanceValley {
                    start,
                    end,
                    before,
                    after,
                    floor_before,
                    floor_after,
                    kind: ValleyKind::Crawl,
                });
            }
        }
        self.assert_crawl_valleys_apart();
        self.assert_weight_continuous();
    }

    fn assert_crawl_valleys_apart(&self) {
        let mut crawls = self
            .valleys
            .iter()
            .filter(|valley| valley.kind == ValleyKind::Crawl);
        let Some(mut previous) = crawls.next() else {
            return;
        };
        for valley in crawls {
            assert!(
                previous.reach_end() <= valley.reach_start(),
                "follower projection: the crawl valleys {previous:?} and {valley:?} overlap"
            );
            previous = valley;
        }
    }

    /// The weight `W` is continuous across every crawl valley an advance
    /// reads: a side laid zero wide steps it between 1 and 0 at the
    /// valley's edge, which only another valley holding zero up to that
    /// edge from the side's way, or the floor, leaves unread. A rest
    /// valley's own edges lie where the leaders rest, whose zero speed
    /// leaves its steps unread; a follower with no advance reads none.
    fn assert_weight_continuous(&self) {
        if self
            .width
            .rule
            .is_none_or(|rule| rule.advance_slope_s == 0.0)
        {
            return;
        }
        let crawls = self
            .valleys
            .iter()
            .filter(|valley| valley.kind == ValleyKind::Crawl);
        for valley in crawls {
            let steps = [
                (valley.before == 0.0).then_some((valley.start, Side::Before)),
                (valley.after == 0.0 && valley.end.is_finite())
                    .then_some((valley.end, Side::After)),
            ];
            for (edge, side) in steps.into_iter().flatten() {
                let unread = edge <= self.floor
                    || self.valleys.iter().any(|other| {
                        !std::ptr::eq(other, valley) && holds_zero_up_to(other, edge, side)
                    });
                assert!(
                    unread,
                    "follower projection: the advance valley {valley:?} steps its weight at \
                     s={edge} on its {side:?} side, where no other valley holds it at zero \
                     (floor {}, valleys {:?})",
                    self.floor, self.valleys
                );
            }
        }
    }

    /// Records the seams of both tables' spans not yet examined, and
    /// returns the range of the laid table's.
    fn record_seams(
        &mut self,
        (spans, advance_spans): (&[RatioSpan], &[RatioSpan]),
    ) -> std::ops::Range<usize> {
        self.advance.record(advance_spans);
        let laid = self.laid.record(spans);
        self.refresh_clusters();
        laid
    }

    /// Hands the width field the seams between consecutive `spans`, timed
    /// where the leaders of `leg` cross them, with how they cross the ramp
    /// on either side: within `leg`, and over the tails of the legs before
    /// it the ramp reaches back into.
    fn time_seams(&mut self, spans: &[RatioSpan], crossings: &mut LegCrossings, leg: &RampLeg) {
        let path = LegPath::of(crossings.leaders, leg);
        for pair in spans.windows(2) {
            let (before, after) = (&pair[0], &pair[1]);
            let Some(seam) = Seam::between(before, after) else {
                continue;
            };
            let t = path.time_at(after.s0);
            let crossing = self
                .laid_tails
                .iter()
                .filter(|tail| tail.until > t - CLUSTER_REACH_S)
                .fold(
                    crossings.over(ramp_reach_within(t, leg.support)),
                    |widest, tail| widest.widest(tail.crossing),
                );
            self.width.add_seam(TimedSeam {
                s: after.s0,
                t,
                crossing,
                step: seam.step.abs() + (after.r1 - after.r0).abs(),
                peak_ratio: before.r1.abs().max(after.r0.abs()).max(after.r1.abs()),
                ramp_time: None,
            });
        }
    }

    /// Remembers how the leaders of `leg` cross its last half of the longest
    /// ramp, which a ramp laid on a later leg may reach back into.
    fn keep_tail(&mut self, crossings: &mut LegCrossings, leg: &RampLeg) {
        let (t_start, t_end) = leg.support;
        self.laid_tails
            .retain(|tail| tail.until > t_end - CLUSTER_REACH_S);
        self.laid_tails.push_back(LegTail {
            until: t_end,
            crossing: crossings.over(((t_end - CLUSTER_REACH_S).max(t_start), t_end)),
        });
    }

    pub(super) fn queue(
        &mut self,
        t_end: f64,
        (s_start, s_length): (f64, f64),
        table_end: f64,
        reads: LeaderReads,
    ) {
        self.backlog.push_back(PendingSegment {
            t_end,
            s_start,
            s_length,
            reads,
            table_end,
        });
    }

    /// The queued segments no node, seam or valley still to come can reach
    /// into, in order.
    pub(super) fn take_final(&mut self, spans: &[RatioSpan]) -> Vec<PendingSegment> {
        self.refresh_clusters();
        let table_end = spans.last().map_or(self.floor, |last| last.s1);
        let final_through = self
            .final_through(table_end)
            .min(table_end - self.unlaid_valley_reach(table_end))
            .max(self.floor);
        let ready = self
            .backlog
            .iter()
            .take_while(|segment| {
                segment.table_end <= self.floor
                    || segment.table_end + SPAN_LOOKUP_SLACK_MM <= final_through
            })
            .count();
        let drained: Vec<PendingSegment> = self.backlog.drain(..ready).collect();
        if let Some(last) = drained.last() {
            self.taken_through = self
                .taken_through
                .max(last.table_end)
                .max(last.s_start + last.s_length);
        }
        self.trail.forget_before(self.floor.max(self.taken_through));
        drained
    }

    /// How far back from `seam` a valley laid there may reach: not past a
    /// floor, and not into a segment already taken for projection, whose
    /// advance read the weight before the valley existed. The reserve
    /// [`Self::unlaid_valley_reach`] keeps this from binding where the
    /// leaders' reach changes slowly; a rest side whose seam's crossing
    /// reaches much further than the one the reserve was taken at (hard
    /// braking into a rest right after a cruise) releases the advance over
    /// what is left, and a crawl valley fails loudly ([`Self::lay_valley`]).
    fn back_reach_limit(&self, seam: f64) -> f64 {
        (seam - self.floor.max(self.taken_through)).max(0.0)
    }

    /// How far back from the table's end a valley the next leg may open
    /// there, or lay at a crawl still to come or waiting on its spans,
    /// reaches: a rest or a turnaround releases the advance over the
    /// leaders' reach at that seam, which is no further than the reach of
    /// the crossing the last leg closed on; a crawl valley's side reaches
    /// back no further than the leaders cover in the
    /// [`geometry::VALLEY_LEAD_BUDGET_S`] before it, which the turning guard
    /// leaves its rebuild room within.
    fn unlaid_valley_reach(&self, table_end: f64) -> f64 {
        let opening = self.reach_into_next.map_or(0.0, |closing| {
            closing.valley_reach().min(table_end - self.floor)
        });
        let lead = table_end
            - self
                .trail
                .lead_budget_back()
                .map_or(self.floor, |back| back.max(self.floor));
        opening.max(lead)
    }

    /// How far the smoothed ratio is final: its width there is, no window
    /// reaches a seam whose cluster may still change, nor past the table,
    /// and no such cluster's own window reaches back to it
    /// ([`SeamTable::first_unfinal`]). A cluster reaches back to its first
    /// seam `σ` less its width, at most the width at `σ`, and `σ − h(σ)`
    /// rises along the path (`|h'| ≤ λ < 1`), by at least `1 − λ` per unit
    /// of path past the last point the width is final at.
    fn final_through(&mut self, table_end: f64) -> f64 {
        let Some(width_final) = self.width.final_through() else {
            return self.floor;
        };
        let nearest_unfinal = [&self.laid, &self.advance]
            .into_iter()
            .filter_map(|table| table.first_unfinal(&self.width, width_final, table_end))
            .fold(table_end, f64::min);
        let width = self.width.searcher();
        let fits = |s: f64| s + width.width(s) <= nearest_unfinal;
        let reach_back_from = |seam: f64| {
            if seam <= width_final {
                seam - width.width(seam)
            } else {
                width_final - width.width(width_final)
                    + (1.0 - WIDTH_SLOPE_BOUND) * (seam - width_final)
            }
        };
        let mut hi = width_final
            .min(table_end)
            .min(reach_back_from(nearest_unfinal));
        if fits(hi) {
            return hi.max(self.floor);
        }
        let mut lo = self.width.nodes[0].s.max(self.floor);
        if !fits(lo) || lo > hi {
            return self.floor;
        }
        for _ in 0..WIDTH_BISECTIONS {
            let mid = 0.5 * (lo + hi);
            if mid == lo || mid == hi {
                break;
            }
            if fits(mid) {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        lo
    }

    /// `X = Ẽ − E` of the laid table at `s` with its first three path
    /// derivatives ([`SeamTable::correction`]).
    pub(super) fn correction(&self, s: f64) -> Jet {
        self.correction_of(Table::Laid, s)
    }

    /// [`Self::correction`] of `table`.
    pub(super) fn correction_of(&self, table: Table, s: f64) -> Jet {
        self.seam_table(table).correction(&self.width, s)
    }

    fn seam_table(&self, table: Table) -> &SeamTable {
        match table {
            Table::Laid => &self.laid,
            Table::Advance => &self.advance,
        }
    }

    /// Whether the smoothing changes anything over `[lo, hi]`: the table
    /// slopes where the windows there open, a window there reaches a seam,
    /// or a cluster's own window reaches into it ([`SeamCluster`]). The
    /// moving window is judged by the table's own seams and slopes, which
    /// gather into the clusters' kinks only where those lie. `s ± h(s)` grows along
    /// the path (`|h'| < 1`), so a seam outside the stretch is reached from
    /// the stretch's end nearer it, if at all, and one inside from itself,
    /// unless the leaders rest on it, where no window reaches it — though
    /// the windows past it still open on the slope it leaves. Only the
    /// width over the stretch and at the clusters reaching it decides,
    /// never what is laid past it.
    pub(super) fn touches(&self, lo: f64, hi: f64) -> bool {
        if self.is_inert() {
            return false;
        }
        let (back, ahead) = self.moving_reach(lo, hi);
        if self.laid.slope_left_of(back) != 0.0 {
            return true;
        }
        let (widest, _) = self.width.bounds(lo, hi);
        self.laid
            .seams_near(lo - widest, hi + widest)
            .iter()
            .any(|seam| {
                if seam.s < lo {
                    seam.s > back
                } else if seam.s > hi {
                    seam.s < ahead
                } else {
                    self.reach_back(seam.s) > 0.0 || seam.slope_after != 0.0
                }
            })
            || self
                .laid
                .clusters_reaching(&self.width, (lo, hi))
                .next()
                .is_some()
    }

    /// The path the moving windows over `[lo, hi]` average, `s ± h(s)`
    /// growing along the path.
    fn moving_reach(&self, lo: f64, hi: f64) -> (f64, f64) {
        (lo - self.reach_back(lo), hi + self.reach_back(hi))
    }

    /// Which spans the smoothed table reads anywhere on `[lo, hi]`: those
    /// the moving windows there average ([`Self::moving_reach`]), and those
    /// either side of every seam of a cluster whose own window reaches into
    /// it. A span only touching the windows' reach is read nowhere on the
    /// stretch, so nothing laid past what it reads decides which.
    pub(super) fn reading(&self, lo: f64, hi: f64) -> SpanReading<'_> {
        SpanReading {
            moving: self.moving_reach(lo, hi),
            clusters: self.laid.clusters_reaching(&self.width, (lo, hi)).collect(),
        }
    }

    /// The first path distance the smoothing from `s` on reads its spans
    /// and seams at ([`Self::reading`]): what the window at `s` averages,
    /// and the first seam of every cluster of either table reaching `s` or
    /// past it.
    pub(super) fn read_from(&self, s: f64) -> f64 {
        [&self.laid, &self.advance]
            .into_iter()
            .filter_map(|table| {
                table
                    .clusters_reaching(&self.width, (s, f64::INFINITY))
                    .next()
                    .map(|cluster| cluster.first())
            })
            .fold(s - self.reach_back(s), f64::min)
    }

    /// `max |r̃|` over `[lo, hi]` of `table`, laid as `spans`: the largest
    /// ratio of the spans the smoothed table reads there, as
    /// [`Self::reading`] reads the laid table's, the smoothed ratio being a
    /// convex combination of theirs.
    pub(super) fn peak_ratio_in(&self, table: Table, spans: &[RatioSpan], lo: f64, hi: f64) -> f64 {
        let clusters = self
            .seam_table(table)
            .clusters_reaching(&self.width, (lo, hi))
            .map(|cluster| (cluster.first(), cluster.last().s));
        read_ratio_range(spans, self.moving_reach(lo, hi), clusters)
            .map_or(0.0, |(low, high)| low.abs().max(high.abs()))
    }

    /// Whether the smoothed `table` up to `hi` reads nothing laid past
    /// `table_end`: the moving window at `hi` stops short of it, the window
    /// of a seam at `table_end` or past it, `σ − h(σ)` rising along the
    /// path, opens past `hi`, and so does that of the cluster such a seam
    /// may still join ([`SeamTable::joinable_opening`]).
    pub(super) fn reads_within(&self, table: Table, hi: f64, table_end: f64) -> bool {
        let reach = self.reach_back(table_end);
        hi + self.reach_back(hi) <= table_end
            && table_end - reach >= hi
            && self
                .seam_table(table)
                .joinable_opening(&self.width, table_end, reach)
                .is_none_or(|opening| opening > hi)
    }

    /// `(max |r̃|, max |r̃'|)` over `[lo, hi]`, see [`RatioSmoothing`]: the
    /// ratio within the spans the smoothed table reads there
    /// ([`Self::reading`]); the slope within the moving window's bound on
    /// the slopes it averages there, each cluster's gathered at its first
    /// seam ([`SeamTable::moving_view`]), and what each cluster reaching in
    /// adds over its own window, its slopes less the one it leaves on and
    /// `¾` of its steps over its width. A span only touching the stretch is
    /// read nowhere on it, so nothing laid past what the stretch reads
    /// decides the bound.
    pub(super) fn ratio_bounds(&self, spans: &[RatioSpan], lo: f64, hi: f64) -> (f64, f64) {
        let (_, curvature) = self.width.bounds(lo, hi);
        let reading = self.reading(lo, hi);
        let Some((low, high)) = reading.ratio_range(spans) else {
            return (0.0, 0.0);
        };
        let moving = self.laid.moving_view(&self.width, reading.moving);
        let slope = moving
            .seams
            .iter()
            .map(|kink| kink.slope_after.abs())
            .fold(moving.slope_before.abs(), f64::max);
        let stretch = (1.0 + WIDTH_SLOPE_BOUND) * (1.0 + WIDTH_SLOPE_BOUND);
        let stationary = reading
            .clusters
            .iter()
            .map(|cluster| {
                let out = cluster.slope_out();
                let (steepest, steps) =
                    cluster
                        .seams
                        .iter()
                        .fold((0.0_f64, 0.0), |(steepest, steps), seam| {
                            (
                                steepest.max((seam.slope_after - out).abs()),
                                steps + seam.step.abs(),
                            )
                        });
                steepest + KERNEL_PEAK * steps / cluster.width
            })
            .sum::<f64>();
        (
            low.abs().max(high.abs()),
            stretch * slope + stationary + KERNEL_ABS_MOMENT * curvature * (high - low),
        )
    }

    /// Where over `[lo, hi]` the smoothed ratio's curvature, or the
    /// advance's read, bends: where each kink the moving window averages
    /// ([`SeamTable::moving_view`]) enters and leaves a window, where each
    /// seam enters and leaves its cluster's own window
    /// ([`window_edges`]), the width nodes, and the valley
    /// ends. A stretch the smoothing does not touch bends only at valley
    /// ends, whatever the smoothing holds elsewhere: the advance's table
    /// steps its ratio only where the laid one does, and its slope only
    /// there or inside a laid span that slopes through zero. A window edge
    /// lies on its side of the seam clamped to the nodes, so one that side
    /// puts outside `[lo, hi]` is not searched for.
    pub(super) fn bends(&self, lo: f64, hi: f64) -> Vec<f64> {
        let mut bends: Vec<f64> = self.valley_ends(lo, hi).collect();
        if !self.touches(lo, hi) {
            return bends;
        }
        let (widest, _) = self.width.bounds(lo, hi);
        bends.extend(self.width.covering(lo, hi).iter().map(|node| node.s));
        let (first, last) = (
            self.width.nodes[0].s,
            self.width.nodes[self.width.nodes.len() - 1].s,
        );
        let laid = self
            .laid
            .moving_view(&self.width, (lo - widest, hi + widest))
            .seams;
        let advance = self
            .advance
            .moving_view(&self.width, (lo - widest, hi + widest))
            .seams;
        let advance_only = advance.iter().filter(|seam| {
            laid.binary_search_by(|laid| laid.s.total_cmp(&seam.s))
                .is_err()
        });
        for seam in laid.iter().chain(advance_only) {
            let anchor = seam.s.clamp(first, last);
            if anchor >= lo {
                bends.push(self.window_edge(seam.s, -1.0, widest));
            }
            if anchor <= hi {
                bends.push(self.window_edge(seam.s, 1.0, widest));
            }
        }
        for table in [&self.laid, &self.advance] {
            for cluster in table.clusters_reaching(&self.width, (lo, hi)) {
                bends.extend(cluster.pieces.iter().map(|piece| piece.from));
                bends.extend(cluster.pieces.last().map(|piece| piece.to));
            }
        }
        bends
    }

    /// The `s` on `side` of `seam` where `s ∓ h(s) = seam`: `s ± h(s)` is
    /// increasing since `|h'| < 1`. A safeguarded Newton closes in on it,
    /// and the float it flips at is bracketed from there and bisected for.
    fn window_edge(&self, seam: f64, side: f64, widest: f64) -> f64 {
        let width = self.width.searcher();
        let reach = |s: f64| (s - seam) * side - width.width(s);
        let (first, last) = (
            self.width.nodes[0].s,
            self.width.nodes[self.width.nodes.len() - 1].s,
        );
        let mut near = seam.clamp(first, last);
        let mut far = (seam + side * (widest / (1.0 - WIDTH_SLOPE_BOUND))).clamp(first, last);
        if reach(far) < 0.0 || reach(near) >= 0.0 {
            return near;
        }
        let strictly_between =
            |s: f64, near: f64, far: f64| (s - near) * side > 0.0 && (far - s) * side > 0.0;
        let mut s = near;
        for _ in 0..WIDTH_BISECTIONS {
            let (h, slope) = width.width_and_slope(s);
            let reached = (s - seam) * side - h;
            if reached < 0.0 {
                near = s;
            } else {
                far = s;
            }
            let newton = s - reached / (side - slope);
            let ulp = (trajectory::next_toward(s, f64::INFINITY) - s).abs();
            if (newton - s).abs() <= NEWTON_SETTLED_ULPS * ulp {
                break;
            }
            s = if strictly_between(newton, near, far) {
                newton
            } else {
                0.5 * (near + far)
            };
        }
        let s_reached = s == far;
        let mut step = trajectory::next_toward(s, if s_reached { near } else { far }) - s;
        loop {
            let probe = s + step;
            if !strictly_between(probe, near, far) {
                break;
            }
            let probe_reached = reach(probe) >= 0.0;
            if probe_reached {
                far = probe;
            } else {
                near = probe;
            }
            if probe_reached != s_reached {
                break;
            }
            step *= 2.0;
        }
        for _ in 0..WIDTH_BISECTIONS {
            let mid = 0.5 * (near + far);
            if mid == near || mid == far {
                break;
            }
            if reach(mid) < 0.0 {
                near = mid;
            } else {
                far = mid;
            }
        }
        far
    }

    /// How far back from `s` its window averages the table.
    pub(super) fn reach_back(&self, s: f64) -> f64 {
        match (self.width.nodes.first(), self.width.nodes.last()) {
            (Some(first), Some(last)) if s >= first.s => self.width.width(s.min(last.s)),
            _ => 0.0,
        }
    }

    /// Drops what no window from `s` on reaches: the seams behind the
    /// farthest a cluster's own window can reach `s` from, `h(s)/(1 − λ)`
    /// back, but for the clusters holding a seam past that, kept whole,
    /// and the width nodes behind the first seam kept.
    pub(super) fn prune_before(&mut self, s: f64) {
        self.valleys.retain(|valley| valley.reach_end() >= s);
        let (Some(first), Some(last)) = (self.width.nodes.first(), self.width.nodes.last()) else {
            return;
        };
        let s = s.min(last.s);
        if first.s > s {
            return;
        }
        self.refresh_clusters();
        let back_edge = s - self.width.width(s) / (1.0 - WIDTH_SLOPE_BOUND);
        let kept_from = [&mut self.laid, &mut self.advance]
            .into_iter()
            .map(|table| table.prune_through(&self.width, back_edge))
            .fold(back_edge, f64::min);
        self.width.prune_before(kept_from);
        self.refresh_clusters();
    }

    /// Within a leg whose raw leaders rest, the shaped leaders rest where
    /// their speed is down to [`REST_SPEED_MM_S`], and the table steps the
    /// ratio from the leg's tail to its lead where their distance from the
    /// rest point puts the rest ([`super::FollowerState::ingest`]), which
    /// the odometer can reach while they still creep toward it. The width
    /// rests there all the same, so the ratio step stays a step that any
    /// weight above zero hands the advance as a position step. A rest
    /// valley takes in both: it opens at the first, unless one is open
    /// already, and the first leg that moves again closes it at the last.
    /// Each side spans the shaped spill between that rest and the raw seam,
    /// over which the advance builds or releases its lead from the rest out,
    /// and reaches past the seam only where the lead or a ratio step near
    /// the rest needs it ([`RestSide::width`]). A rest leg the leaders cross
    /// without coming to rest opens none. The crawls still waiting lie
    /// before the rest and are laid first, reading the leaders at rest past
    /// the trail, as the turning guard leaves room for them: valleys are
    /// laid in path order. Returns whether `leg` closes a rest valley.
    fn lay_rest_valley(
        &mut self,
        (spans, advance_spans): (&[RatioSpan], &[RatioSpan]),
        leg: &RampLeg,
        rule: &RatioRampRule,
        crossing: Crossing,
        table_end: f64,
    ) -> bool {
        let seam = leg.start_s;
        let leaders = leg.shaped_leaders;
        let (t_start, t_end) = leg.support;
        if !leg.rests {
            let Some(open) = self.open_rest else {
                return false;
            };
            let side = RestSide {
                rest: open.end,
                seam,
                outward: 1.0,
                crossing,
                spill: open.departure,
                beyond: Crossing::over(leaders, (t_start, (t_start + CLUSTER_REACH_S).min(t_end))),
            };
            self.close_rest(open.end, side.width(spans, rule, table_end));
            return true;
        }
        let Some(resting) = resting_stretch(leaders, leg, REST_SPEED_MM_S, table_end) else {
            return false;
        };
        let steps = ratio_steps_within(spans, seam, table_end);
        if let Some((first_step, last_step)) = steps {
            self.width.place_rest_among_laid(first_step);
            self.width.place_rest_among_laid(last_step);
        }
        let (rest_start, rest_end) = steps.map_or(
            (resting.first_s, resting.last_s),
            |(first_step, last_step)| {
                (
                    resting.first_s.min(first_step),
                    resting.last_s.max(last_step),
                )
            },
        );
        if self.open_rest.is_none() {
            for crawl in std::mem::take(&mut self.waiting_crawls) {
                self.lay_crawl_valley((spans, advance_spans), rule, crawl, f64::INFINITY);
            }
            let side = RestSide {
                rest: rest_start,
                seam,
                outward: -1.0,
                crossing,
                spill: Spill::over(leaders, (t_start, resting.first_t)),
                beyond: self
                    .laid_tails
                    .iter()
                    .fold(crossing, |widest, tail| widest.widest(tail.crossing)),
            };
            let before = side
                .width(spans, rule, table_end)
                .min(rest_start - seam + self.back_reach_limit(seam));
            self.valleys.push(AdvanceValley {
                start: rest_start,
                end: f64::INFINITY,
                before,
                after: 0.0,
                floor_before: 0.0,
                floor_after: 0.0,
                kind: ValleyKind::Rest,
            });
            self.assert_weight_continuous();
        }
        self.open_rest = Some(OpenRest {
            end: rest_end,
            departure: Spill::over(leaders, (resting.last_t, t_end)),
        });
        false
    }

    /// The weight `W` the advance reads the ratio with at `s`, with its
    /// first two path derivatives: 1 away from every valley. Valleys reach
    /// as far as their seams' crossings, so neighbours may overlap in any
    /// order; their weights multiply.
    pub(super) fn valley_weight(&self, s: f64) -> (f64, f64, f64) {
        self.valleys_over(s, s)
            .fold((1.0, 0.0, 0.0), |(w, w1, w2), valley| {
                let (v, v1, v2) = valley.weight(s);
                (w * v, w1 * v + w * v1, w2 * v + 2.0 * w1 * v1 + w * v2)
            })
    }

    /// Fails loudly where the ramp of a ratio step of `table` reaches `s`
    /// inside a crawl valley's side of a follower the turning guard holds
    /// ([`guarded_by_the_turning_guard`]) and the follower's acceleration
    /// there, its flow's `flow_accel` and the advance's slope times
    /// `advanced_accel`, the second time derivative of the weighted read,
    /// passes its limit: the ramp's rule and the side's rebuild each fit it
    /// alone, and nothing budgets their sum. Such a valley owns the steps
    /// inside its sides and holds the weight at zero over their windows
    /// ([`Self::lay_valley`]); a step past a side whose window reaches back
    /// into it is not owned. A follower behind its own kernel has that
    /// kernel spread both.
    pub(super) fn assert_ramp_in_a_crawl_side_fits(
        &self,
        table: Table,
        s: f64,
        (flow_accel, advanced_accel): (f64, f64),
    ) {
        let Some(rule) = self.width.rule.filter(guarded_by_the_turning_guard) else {
            return;
        };
        let Some(valley) = self
            .valleys_over(s, s)
            .find(|valley| valley.kind == ValleyKind::Crawl && valley.weight(s).0 > 0.0)
        else {
            return;
        };
        let accel = (flow_accel + rule.advance_slope_s * advanced_accel).abs();
        if accel <= rule.max_accel_mm_s2 {
            return;
        }
        if let Some(cluster) = self
            .seam_table(table)
            .clusters_reaching(&self.width, (s, s))
            .find(|cluster| cluster.seams.iter().any(|seam| seam.step != 0.0))
        {
            panic!(
                "follower projection: the ratio steps of the cluster {:?} of the {table:?} table \
                 ramp over {} mm either side, into the side of the crawl valley {valley:?} at \
                 s={s}, where the follower asks {accel} mm/s², past its limit of {} mm/s²",
                cluster.seams, cluster.width, rule.max_accel_mm_s2
            );
        }
    }

    fn valleys_over(&self, lo: f64, hi: f64) -> impl Iterator<Item = &AdvanceValley> + '_ {
        self.valleys
            .iter()
            .filter(move |valley| valley.reach_start() < hi && valley.reach_end() > lo)
    }

    /// Where the valleys overlapping `[lo, hi]` start and end bending.
    fn valley_ends(&self, lo: f64, hi: f64) -> impl Iterator<Item = f64> + '_ {
        self.valleys_over(lo, hi).flat_map(|valley| {
            [
                valley.reach_start(),
                valley.start,
                valley.end,
                valley.reach_end(),
            ]
        })
    }
}

/// A rest valley laid up to the last rest leg so far: where the leaders
/// last rest, and the spill they leave it over.
#[derive(Debug, Clone, Copy)]
struct OpenRest {
    end: f64,
    departure: Spill,
}

/// The shaped spill between a raw seam and where the leaders rest beside
/// it: how long they take over it, and how they cross it.
#[derive(Debug, Clone, Copy)]
struct Spill {
    duration: f64,
    crossing: Crossing,
}

impl Spill {
    fn over(leaders: &ShapedLeaders, (from, until): (f64, f64)) -> Self {
        Self {
            duration: until - from,
            crossing: Crossing::over(leaders, (from, until)),
        }
    }
}

/// One side of a rest valley: where the leaders rest and the raw seam
/// beside it, `outward` from the rest along the path (−1 before it, 1
/// after), how they cross that seam, the spill between, and how they
/// cross the path past the seam the side may reach.
#[derive(Debug, Clone, Copy)]
struct RestSide {
    rest: f64,
    seam: f64,
    outward: f64,
    crossing: Crossing,
    spill: Spill,
    beyond: Crossing,
}

impl RestSide {
    /// How far from the rest the side spans: the spill, and past the seam
    /// - as far as the longest ramp reaches where a ratio step within that
    ///   reach lies so near the rest that the window the width field narrows
    ///   to there ramps it beyond `rule`'s limits
    ///   ([`Self::narrows_a_step`]), or where the table does not reach that
    ///   far yet: the valley keeps the advance from reading it;
    /// - else nowhere where the shortest ramp `rule` allows for releasing or
    ///   building the lead of an advance on the ratio there, a step of that
    ///   whole ratio to the advance, fits within the spill, which then
    ///   carries the whole lead up to the rest;
    /// - else as far as the leaders cover at the seam in half that ramp,
    ///   read as they cross the whole reach. A spill that takes no time — a
    ///   leader stopping without a kernel, its acceleration stepping at the
    ///   rest — always reaches out.
    fn width(self, spans: &[RatioSpan], rule: &RatioRampRule, table_end: f64) -> f64 {
        let spill_mm = (self.seam - self.rest).abs();
        let longest = self.crossing.valley_reach();
        let far = self.seam + self.outward * longest;
        let reach = (self.rest.min(far), self.rest.max(far));
        if far > table_end || self.narrows_a_step(spans, rule, reach) {
            return spill_mm + longest;
        }
        let ratio = peak_ratio_within(spans, self.seam, spill_mm + longest);
        let lead_ramp_time = |leaders: Crossing| {
            rule.ramp_time(&RatioStep {
                ratio_step: ratio,
                peak_ratio: ratio,
                speed_mm_s: leaders.speed,
                accel_mm_s2: leaders.rate,
                jerk_mm_s3: leaders.jerk,
            })
        };
        let over_spill = self.crossing.widest(self.spill.crossing);
        if lead_ramp_time(over_spill) <= self.spill.duration {
            return spill_mm;
        }
        spill_mm
            + self
                .crossing
                .reach(lead_ramp_time(over_spill.widest(self.beyond)))
    }

    /// Whether the table steps its ratio or its slope inside `(from, to)`
    /// so near the rest that the width field, whose envelope falls off
    /// toward the rest by [`ENVELOPE_SLOPE`], ramps the step over less time
    /// than `rule`'s limits allow as the leaders cross the step's window
    /// ([`Self::window_crossing`]). The window ramps the ratio the table
    /// changes within it: the step, and the slopes either side over as much
    /// of the window as their spans fill.
    fn narrows_a_step(
        self,
        spans: &[RatioSpan],
        rule: &RatioRampRule,
        (from, to): (f64, f64),
    ) -> bool {
        let first = spans.partition_point(|span| span.s0 <= from).max(1);
        spans[first - 1..]
            .windows(2)
            .take_while(|pair| pair[1].s0 < to)
            .any(|pair| {
                let (before, after) = (&pair[0], &pair[1]);
                let Some(seam) = Seam::between(before, after) else {
                    return false;
                };
                let distance = (seam.s - self.rest).abs();
                let window = ENVELOPE_SLOPE * distance;
                let leaders = self.window_crossing(distance + window);
                if leaders.speed == 0.0 {
                    return false;
                }
                let within_window =
                    |span: &RatioSpan| span.ratio_slope().abs() * window.min(span.s1 - span.s0);
                let (velocity, accel) = rule.demand(
                    2.0 * window / leaders.speed,
                    &RatioStep {
                        ratio_step: seam.step.abs() + within_window(before) + within_window(after),
                        peak_ratio: before.r1.abs().max(after.r0.abs()).max(after.r1.abs()),
                        speed_mm_s: leaders.speed,
                        accel_mm_s2: leaders.rate,
                        jerk_mm_s3: leaders.jerk,
                    },
                );
                velocity > rule.max_velocity_mm_s || accel > rule.max_accel_mm_s2
            })
    }

    /// How the leaders cross a step's window reaching `reach` from the
    /// rest: as they cross the spill, and where the window runs past the raw
    /// seam, no faster than the most they accelerate past it takes them
    /// over the overhang `d` from their speed at the seam, `√(v² + 2·a·d)`:
    /// a step inside the spill is ramped where the leaders slow into the
    /// rest or leave it, not at the speed they cruise far past the seam.
    fn window_crossing(self, reach: f64) -> Crossing {
        let over_spill = self.crossing.widest(self.spill.crossing);
        let past_seam = over_spill.widest(self.beyond);
        let overhang = reach - (self.seam - self.rest).abs();
        if overhang <= 0.0 {
            return over_spill;
        }
        Crossing {
            speed: libm::sqrt(
                over_spill.speed * over_spill.speed + 2.0 * past_seam.rate * overhang,
            )
            .min(past_seam.speed),
            ..past_seam
        }
    }
}

/// The speed under which the leaders rest: they cover no more than
/// [`REST_TARGET_MM`], the width field's rest, between two width nodes.
const REST_SPEED_MM_S: f64 = REST_TARGET_MM / NODE_SPACING_S;

/// The path the leaders of a leg cover from its start, by time, read off
/// the odometer whose arc its table laid.
struct LegPath<'l, 'a> {
    leaders: &'l ShapedLeaders<'a>,
    odometer: &'l OdometerLeaves,
    grid: &'l [f64],
    start_s: f64,
}

impl<'l, 'a> LegPath<'l, 'a> {
    fn of(leaders: &'l ShapedLeaders<'a>, leg: &RampLeg<'l>) -> Self {
        Self {
            leaders,
            odometer: leg.odometer,
            grid: &leg.odometer.grid,
            start_s: leg.start_s,
        }
    }

    fn speed(&self, t: f64) -> f64 {
        self.leaders.speed(t)
    }

    fn covered(&self, t: f64) -> f64 {
        self.odometer.covered_until(t)
    }

    /// When the leaders reach path `s`: the leg's start for a seam laid
    /// there, else the time the path they cover reaches `s`.
    fn time_at(&self, s: f64) -> f64 {
        let grid = &self.grid;
        if s <= self.start_s + SPAN_LOOKUP_SLACK_MM {
            return grid[0];
        }
        let (mut lo, mut hi) = (grid[0], grid[grid.len() - 1]);
        for _ in 0..WIDTH_BISECTIONS {
            let mid = 0.5 * (lo + hi);
            if mid == lo || mid == hi {
                break;
            }
            if self.start_s + self.covered(mid) < s {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        hi
    }

    /// Whether the leaders provably stay at `speed` or faster over the
    /// whole leg, knot interval by knot interval.
    fn outruns(&self, speed: f64) -> bool {
        self.grid.windows(2).all(|pair| {
            self.leaders
                .speed_floor(
                    pair[0],
                    0.5 * (pair[0] + pair[1]),
                    pair[1],
                    FloorLeaders::Cached,
                )
                .is_some_and(|floor| floor >= speed)
        })
    }
}

/// The shaped leaders' speed along a leg, probed
/// [`REST_PROBES_PER_INTERVAL`] times per knot interval.
struct LegSpeed<'l, 'a> {
    path: LegPath<'l, 'a>,
    probes: Vec<f64>,
    table_end: f64,
}

impl<'l, 'a> LegSpeed<'l, 'a> {
    fn over(path: LegPath<'l, 'a>, table_end: f64) -> Self {
        let grid = &path.grid;
        let probes = grid
            .windows(2)
            .flat_map(|pair| {
                (0..REST_PROBES_PER_INTERVAL).map(move |index| {
                    pair[0] + (pair[1] - pair[0]) * index as f64 / REST_PROBES_PER_INTERVAL as f64
                })
            })
            .chain(std::iter::once(grid[grid.len() - 1]))
            .collect();
        Self {
            path,
            probes,
            table_end,
        }
    }

    fn at(&self, t: f64) -> f64 {
        self.path.speed(t)
    }

    /// The probes either side of probe `index`.
    fn around(&self, index: usize) -> (f64, f64) {
        (
            self.probes[index.saturating_sub(1)],
            self.probes[(index + 1).min(self.probes.len() - 1)],
        )
    }

    /// The slowest time between the probes either side of probe `index`.
    fn slowest_near(&self, index: usize) -> f64 {
        let (mut lo, mut hi) = self.around(index);
        for _ in 0..REST_REFINEMENTS {
            let (left, right) = (lo + (hi - lo) / 3.0, hi - (hi - lo) / 3.0);
            if self.at(left) <= self.at(right) {
                hi = right;
            } else {
                lo = left;
            }
        }
        0.5 * (lo + hi)
    }

    /// Whether the leaders provably stay faster than `rest_speed` between
    /// the probes either side of probe `index`.
    fn outruns_rest_near(&self, index: usize, rest_speed: f64) -> bool {
        let (lo, hi) = self.around(index);
        self.path
            .leaders
            .speed_floor(lo, self.probes[index], hi, FloorLeaders::Every)
            .is_some_and(|floor| floor > rest_speed)
    }

    fn path_at(&self, t: f64) -> f64 {
        let leg_end = self.path.grid[self.path.grid.len() - 1];
        if t >= leg_end {
            return self.table_end;
        }
        (self.path.start_s + self.path.covered(t)).min(self.table_end)
    }
}

/// Where and when the shaped leaders of a rest leg first and last rest.
#[derive(Debug, Clone, Copy)]
struct Resting {
    first_t: f64,
    first_s: f64,
    last_t: f64,
    last_s: f64,
}

/// Where along `leg` the shaped leaders rest, first to last: the first and
/// the last time their speed is down to `rest_speed`, the minimum of each
/// probe interval refined so that a turnaround, at rest for an instant
/// only, is found.
fn resting_stretch(
    leaders: &ShapedLeaders,
    leg: &RampLeg,
    rest_speed: f64,
    table_end: f64,
) -> Option<Resting> {
    let speed = LegSpeed::over(LegPath::of(leaders, leg), table_end);
    let resting = |t: f64| speed.at(t) <= rest_speed;
    let rest_time = |index: usize| {
        let probe = speed.probes[index];
        if resting(probe) {
            return Some(probe);
        }
        if speed.outruns_rest_near(index, rest_speed) {
            return None;
        }
        let slowest = speed.slowest_near(index);
        resting(slowest).then_some(slowest)
    };
    let first = (0..speed.probes.len()).find_map(rest_time)?;
    let last = (0..speed.probes.len())
        .rev()
        .find_map(rest_time)
        .expect("a first rest is also a last");
    let edge = |moving: f64, rest: f64| {
        let (mut moving, mut rest) = (moving, rest);
        for _ in 0..REST_REFINEMENTS {
            let mid = 0.5 * (moving + rest);
            if resting(mid) {
                rest = mid;
            } else {
                moving = mid;
            }
        }
        rest
    };
    let grid = &speed.path.grid;
    let first_t = edge(grid[0], first);
    let last_t = edge(grid[grid.len() - 1], last);
    Some(Resting {
        first_t,
        first_s: speed.path_at(first_t),
        last_t,
        last_s: speed.path_at(last_t),
    })
}

/// The first and the last seam in `[from, to)` where the table's ratio
/// itself steps.
fn ratio_steps_within(spans: &[RatioSpan], from: f64, to: f64) -> Option<(f64, f64)> {
    let first_after = spans.partition_point(|span| span.s0 < from).max(1);
    let mut steps = spans[first_after - 1..]
        .windows(2)
        .take_while(|pair| pair[1].s0 < to)
        .filter_map(|pair| Seam::between(&pair[0], &pair[1]))
        .filter(|seam| seam.step != 0.0)
        .map(|seam| seam.s);
    let first = steps.next()?;
    Some((first, steps.last().unwrap_or(first)))
}

/// Where inside `leg`, off its seams, the shaped leaders are slowest, if
/// they crawl there ([`Crossing::crawls`]): the path there and how a valley
/// there reads them. Shaping spreads a raw reversal over the kernels'
/// support, so where the move into it brakes unlike the move out of it
/// launches, the shaped velocity passes through zero off the raw seam,
/// inside either move, where the seam's crossing does not see it; where the
/// axes are shaped unlike each other it passes a hair beside zero instead.
/// Such a crawl can hide in the probe interval next to a seam the leaders
/// cross too fast to crawl there, so that interval is searched as well as
/// the slowest probe's, however the leg's two seam speeds compare: mirrored
/// seams tie to rounding, which the clock offset decides. A leg whose every
/// knot interval, or every searched probe, provably stays faster has none,
/// and one slowest at an end leaves it to that seam's crossing.
fn slowest_crawl(
    leaders: &ShapedLeaders,
    leg: &RampLeg,
    table_end: f64,
) -> Option<(f64, Crossing)> {
    let path = LegPath::of(leaders, leg);
    if path.outruns(ADVANCE_VALLEY_SPEED_MM_S) {
        return None;
    }
    let speed = LegSpeed::over(path, table_end);
    let probe_speed = |index: usize| speed.at(speed.probes[index]);
    let (slowest_probe, _) = speed
        .probes
        .iter()
        .map(|&t| speed.at(t))
        .enumerate()
        .min_by(|left, right| left.1.total_cmp(&right.1))
        .expect("a leg has probes");
    let last_probe = speed.probes.len() - 1;
    let next_to_a_fast_seam = [0, last_probe]
        .into_iter()
        .filter(|&end| end != slowest_probe && probe_speed(end) >= ADVANCE_VALLEY_SPEED_MM_S);
    let (turn, _) = std::iter::once(slowest_probe)
        .chain(next_to_a_fast_seam)
        .filter_map(|index| {
            let searched_speed = probe_speed(index);
            if searched_speed >= ADVANCE_VALLEY_SPEED_MM_S
                && speed.outruns_rest_near(index, ADVANCE_VALLEY_SPEED_MM_S)
            {
                return None;
            }
            let turn = speed.slowest_near(index);
            let turn_speed = speed.at(turn);
            let at_an_end = (index == 0 || index == last_probe) && turn_speed >= searched_speed;
            (!at_an_end).then_some((turn, turn_speed))
        })
        .min_by(|left, right| left.1.total_cmp(&right.1))?;
    let crossing = Crossing::valley_at(leaders, turn);
    crossing.crawls().then(|| (speed.path_at(turn), crossing))
}

/// The largest extrusion ratio `spans` lay within `reach` of `s`.
fn peak_ratio_within(spans: &[RatioSpan], s: f64, reach: f64) -> f64 {
    spans
        .iter()
        .filter(|span| span.s1 >= s - reach && span.s0 <= s + reach)
        .map(|span| span.r0.abs().max(span.r1.abs()))
        .fold(0.0, f64::max)
}

/// Whether `valley`'s weight is zero up to `edge` from the `side` of it a
/// zero-wide side of another valley would read 1 on: its zero stretch
/// spans `edge`, and does not end there with a step of its own that way.
fn holds_zero_up_to(valley: &AdvanceValley, edge: f64, side: Side) -> bool {
    let spans = valley.start <= edge && edge <= valley.end;
    let steps_there = match side {
        Side::Before => valley.start == edge && valley.before == 0.0,
        Side::After => valley.end == edge && valley.after == 0.0,
    };
    spans && !steps_there
}

/// Whether the turning guard leaves room for this follower's crawl valley
/// sides: it holds an advance no kernel follows, read off the same caps
/// as the guard's own demand.
fn guarded_by_the_turning_guard(rule: &RatioRampRule) -> bool {
    rule.advance_slope_s > 0.0 && !rule.kernel_jerk_per_accel_hz.is_finite()
}

/// Probes a ramp's crossing is searched with for the leaders' fastest
/// speed, acceleration and jerk.
const CROSSING_PROBES: usize = 16;

/// The crossings one leg's seams read of its leaders, each window probed
/// once: the seams of a leg shorter than the ramps' reach all read the
/// whole leg.
struct LegCrossings<'l, 'a> {
    leaders: &'l ShapedLeaders<'a>,
    read: Vec<((f64, f64), Crossing)>,
}

impl<'l, 'a> LegCrossings<'l, 'a> {
    fn of(leaders: &'l ShapedLeaders<'a>) -> Self {
        Self {
            leaders,
            read: Vec::new(),
        }
    }

    /// [`Crossing::over`] `window`.
    fn over(&mut self, window: (f64, f64)) -> Crossing {
        if let Some(&(_, crossing)) = self.read.iter().find(|(read, _)| *read == window) {
            return crossing;
        }
        let crossing = Crossing::over(self.leaders, window);
        self.read.push((window, crossing));
        crossing
    }
}

/// How the leaders cross a seam: their path speed, the magnitude of its
/// rate, and how steeply that rate changes.
#[derive(Debug, Clone, Copy)]
struct Crossing {
    speed: f64,
    rate: f64,
    jerk: f64,
}

impl Crossing {
    /// The leaders' path speed at `t` and their path acceleration, signed,
    /// as they leave `t`. Where the speed is a zero's rounding — at rest,
    /// or at a turnaround, under what [`TURNAROUND_SPEED_TIME_S`] of the
    /// acceleration builds — its direction is rounding too, and so is the
    /// tangential acceleration read along it: the speed leaves a zero at
    /// the whole acceleration.
    fn speed_and_path_accel(leaders: &ShapedLeaders, t: f64) -> (f64, f64) {
        let motion = leaders.motion(t);
        let speed = motion.speed();
        let accel = motion.accel_norm();
        if speed > accel * TURNAROUND_SPEED_TIME_S {
            (speed, motion.speed_deriv(speed))
        } else {
            (speed, accel)
        }
    }

    /// How a valley at `t` reads the leaders crossing it, with the jerk
    /// [`Self::needs_valley`] weighs: the path speed's second rate at `t`,
    /// which carries the tangent's turning `a⊥²/v` through a crawl. Under
    /// [`ADVANCE_VALLEY_SPEED_MM_S`] they leave under the whole of their
    /// acceleration, mostly across the path where they nearly reverse, not
    /// the path speed's rate at its minimum.
    fn valley_at(leaders: &ShapedLeaders, t: f64) -> Self {
        let motion = leaders.motion(t);
        let speed = motion.speed();
        let accel = motion.accel_norm();
        if speed <= accel * TURNAROUND_SPEED_TIME_S {
            return Self {
                speed,
                rate: accel,
                jerk: 0.0,
            };
        }
        let rate = if speed < ADVANCE_VALLEY_SPEED_MM_S {
            accel
        } else {
            motion.speed_deriv(speed).abs()
        };
        let jerk = motion
            .speed_second(speed)
            .unwrap_or_else(|| leaders.speed_second(t, speed))
            .abs();
        Self { speed, rate, jerk }
    }

    /// The fastest and the most accelerating the leaders are over `window`,
    /// and the steepest their path acceleration changes there.
    ///
    /// The jerk is the largest mean over the intervals between probes, the
    /// change in path acceleration across one over its length `Δt`. The
    /// leaders are a fit, which holds their acceleration within its
    /// tolerance `A` but leaves their jerk free: a sliver piece may carry
    /// any, and where the fit lays one depends on how the shaper batched
    /// it. The mean is within `2A/Δt` of the leaders' own whatever pieces
    /// the fit lays. A window of no length spans no interval and meets no
    /// jerk.
    fn over(leaders: &ShapedLeaders, (from, until): (f64, f64)) -> Self {
        let probe_at = |index: usize| from + (until - from) * index as f64 / CROSSING_PROBES as f64;
        let (speed, path_accel) = Self::speed_and_path_accel(leaders, from);
        let mut widest = Self {
            speed,
            rate: path_accel.abs(),
            jerk: 0.0,
        };
        let mut previous = (from, path_accel);
        for index in 1..=CROSSING_PROBES {
            let at = probe_at(index);
            let (speed, path_accel) = Self::speed_and_path_accel(leaders, at);
            let jerk = if at > previous.0 {
                (path_accel - previous.1).abs() / (at - previous.0)
            } else {
                0.0
            };
            widest = widest.widest(Self {
                speed,
                rate: path_accel.abs(),
                jerk,
            });
            previous = (at, path_accel);
        }
        widest
    }

    fn widest(self, other: Self) -> Self {
        Self {
            speed: self.speed.max(other.speed),
            rate: self.rate.max(other.rate),
            jerk: self.jerk.max(other.jerk),
        }
    }

    /// What both sides of a seam share. A rate on one side only — a leader
    /// starting from rest with an acceleration step, as an unshaped z does
    /// — leaves the seam crossed at rest: an advance steps the follower's
    /// velocity there by `k·r·Δa` whether the ratio is smoothed or not, so
    /// a window would only carry the ratio step's material into the move.
    fn common(self, other: Self) -> Self {
        Self {
            speed: self.speed.min(other.speed),
            rate: self.rate.min(other.rate),
            jerk: self.jerk.min(other.jerk),
        }
    }

    /// How far the leaders reach from the seam in half the ramp time.
    fn reach(self, ramp_time: f64) -> f64 {
        leader_reach(self.speed, self.rate, ramp_time)
    }

    /// How far a turnaround or crawl valley here reaches each way.
    fn valley_reach(self) -> f64 {
        valley_reach(self.speed, self.rate)
    }

    /// The leaders turn around at the seam ([`turns_around`]).
    fn turns_around(self) -> bool {
        turns_around(self.speed, self.rate)
    }

    /// The leaders turn around, or crawl under
    /// [`ADVANCE_VALLEY_SPEED_MM_S`].
    fn crawls(self) -> bool {
        self.turns_around() || self.speed < ADVANCE_VALLEY_SPEED_MM_S
    }

    /// An advance of `rule`'s slope releases its lead around where the
    /// leaders turn around, or crawl where reading the path speed through
    /// at `ratio` would ask the follower for more than its acceleration
    /// limit: `r·(a + k·J)`, `J` the slope of the path's acceleration the
    /// follower meets ([`Self::advanced_jerk`]). Elsewhere releasing the
    /// lead over the crawl's short reach would ask more of the follower
    /// than the crawl. A follower the turning guard holds
    /// ([`guarded_by_the_turning_guard`]) releases it at every crawl: the
    /// guard bounds the turning `a⊥²/v` the path speed carries only down to
    /// [`ADVANCE_VALLEY_SPEED_MM_S`], and leaves room for the rebuild of a
    /// valley at every crawl it may find.
    fn needs_valley(self, rule: &RatioRampRule, ratio: f64) -> bool {
        self.turns_around()
            || (self.crawls()
                && (guarded_by_the_turning_guard(rule)
                    || ratio * (self.rate + rule.advance_slope_s * self.advanced_jerk(rule))
                        > rule.max_accel_mm_s2))
    }

    /// The path speed's second rate `s⃛` carries the tangent's turning
    /// `a⊥²/v`; a kernel after the advance differentiates `s̈` instead, so
    /// the follower meets at most the kernel's jerk gain times the rate.
    fn advanced_jerk(self, rule: &RatioRampRule) -> f64 {
        if rule.kernel_jerk_per_accel_hz.is_finite() {
            self.jerk.min(rule.kernel_jerk_per_accel_hz * self.rate)
        } else {
            self.jerk
        }
    }

    /// The floor a valley here keeps flat ([`crawl_floor`]).
    fn crawl_floor(self) -> f64 {
        crawl_floor(self.speed, self.rate)
    }
}

#[cfg(test)]
mod tests;
