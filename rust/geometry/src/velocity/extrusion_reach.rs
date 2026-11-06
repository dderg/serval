use std::collections::VecDeque;

use crate::Move;
use crate::frontend::{ExtruderCaps, LeaderSpillId, SettleDemand};
use crate::path::CurvatureProfile;

use super::extruder_caps::{DemandReach, extruder_accel_cap};
use super::seam_envelope::{ReachableSpeed, SeamEnvelope, SeamLimit, SeamTerm};
use super::z_caps::z_capped;
use crate::ratio_ramp::{RATIO_RAMP_MAX_TIME_S, rest_room_reach_mm};

/// The largest extrusion ratio `m` lays along the path on `axis`: 0 for
/// travel, retraction (a negative ratio) and an extrude-only move, whose
/// ratio is per millimetre of filament rather than of path.
pub(super) fn extruding_ratio(m: &Move, axis: usize) -> f64 {
    if m.segment.spatial.is_none() {
        return 0.0;
    }
    m.segment
        .followers
        .iter()
        .filter(|follower| follower.axis_index == axis)
        .map(|follower| follower.ratio.max(follower.ratio_end))
        .fold(0.0, f64::max)
}

/// What `segment`'s own extruder demand meets: its own ratio and axes,
/// which the ratio and axes within its reach only add to.
fn own_reach(m: &Move, segment: &crate::path::Segment, caps: &ExtruderCaps) -> DemandReach {
    DemandReach {
        ratio: extruding_ratio(m, caps.axis_index),
        ratio_step: 0.0,
        axes: moves_on(segment),
    }
}

/// The most acceleration the planner can plan `m` at, at any speed: its
/// limit, held by the extruder's acceleration cap on its own extrusion
/// ratio and axes at rest, which every speed, and the ratio and axes within
/// its reach, only tighten.
fn reaching_accel(m: &Move) -> f64 {
    let accel = m.limits.accel_mm_s2;
    let (Some(segment), Some(caps)) = (&m.segment.spatial, m.limits.extruder) else {
        return accel;
    };
    let (_, kappa_peak) = segment.kappa_peak();
    extruder_accel_cap(
        segment,
        &caps,
        own_reach(m, segment, &caps),
        kappa_peak,
        (0.0, accel),
    )
}

/// The most acceleration `m` can be planned at from its own caps alone,
/// with the leaders crossing it at no more than `reachable_mm_s`: its
/// limit, held by the z cap ([`super::z_caps::z_capped`]) and by the
/// extruder's acceleration cap on its own extrusion ratio and axes
/// ([`super::extruder_caps::extruder_accel_cap`]), which the ratio and axes
/// within its reach only tighten, as the extruder's velocity cap only
/// trades the budget down.
fn realizable_accel(m: &Move, reachable_mm_s: f64) -> f64 {
    let Some(segment) = &m.segment.spatial else {
        return m.limits.accel_mm_s2;
    };
    let (_, kappa_peak) = segment.kappa_peak();
    let path_ceiling = m.feedrate_mm_s.min(m.limits.max_velocity_mm_s);
    let (z_ceiling, z_accel) = z_capped(segment, &m.limits, kappa_peak, path_ceiling);
    let Some(caps) = m.limits.extruder else {
        return z_accel;
    };
    extruder_accel_cap(
        segment,
        &caps,
        own_reach(m, segment, &caps),
        kappa_peak,
        (z_ceiling.min(reachable_mm_s), z_accel),
    )
}

/// A seam as the reach reads it: the fastest the leaders cross it at the
/// acceleration the moves either side can take, and its limit.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Seam {
    speed_mm_s: f64,
    limit: SeamLimit,
}

impl Seam {
    fn rest() -> Self {
        Self {
            speed_mm_s: 0.0,
            limit: SeamLimit::ceiling(0.0),
        }
    }
}

/// The speed a reach of `speed` climbs to over `length` at `accel`.
fn climbed(speed: f64, length: f64, accel: f64) -> f64 {
    (speed * speed + 2.0 * accel * length).sqrt()
}

/// One side of a seam as the move there limits it: the fastest the move
/// lets the leaders cross the seam at the acceleration it can take, its
/// feed ceiling and the curvature it meets the seam on.
#[derive(Debug, Clone, Copy, PartialEq)]
struct SeamSide {
    speed_mm_s: f64,
    limit: SeamLimit,
}

impl SeamSide {
    /// The start of a stream the leaders enter at `entry_mm_s`.
    fn start(entry_mm_s: f64) -> Self {
        Self {
            speed_mm_s: entry_mm_s,
            limit: SeamLimit::ceiling(entry_mm_s),
        }
    }

    /// A move's side of a seam where it meets the seam on curvature
    /// `kappa` under `ceiling`, accelerating at up to `accel`.
    fn of(ceiling: f64, kappa: f64, accel: f64) -> Self {
        let limit = SeamLimit {
            ceiling_mm_s: ceiling,
            kappa: kappa.abs(),
        };
        Self {
            speed_mm_s: limit.at(accel),
            limit,
        }
    }

    /// The seam where `self` meets `other`: the slower of their speeds,
    /// the lower ceiling and the sharper curvature.
    fn meeting(&self, other: &Self) -> Seam {
        Seam {
            speed_mm_s: self.speed_mm_s.min(other.speed_mm_s),
            limit: SeamLimit {
                ceiling_mm_s: self.limit.ceiling_mm_s.min(other.limit.ceiling_mm_s),
                kappa: self.limit.kappa.max(other.limit.kappa),
            },
        }
    }
}

/// What the extruder's demand while the leaders cross one move can draw on.
///
/// The shaped path at an instant averages the planned path within the
/// leaders' kernel reach `T` of it, so a move's planned acceleration acts
/// on the shaped path while it lies within `spill_s = 2T` of the move in
/// time. The ratio ramp lays there the ratio within its half-width: at most
/// half the path the leaders cover in the longest ramp `τ_max`,
/// `v·τ_max/2` at path speeds up to `v`, and never across a rest of the
/// leaders. Those speeds are the leaders' within `2T + τ_max` of the move.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Reach {
    axis: usize,
    spill_s: f64,
    leader_spill: LeaderSpillId,
}

impl Reach {
    fn of(caps: &ExtruderCaps) -> Self {
        Self {
            axis: caps.axis_index,
            spill_s: 2.0 * caps.leader_kernel_reach_s,
            leader_spill: caps.leader_spill,
        }
    }

    fn speed_window_s(&self) -> f64 {
        self.spill_s + RATIO_RAMP_MAX_TIME_S
    }
}

/// One move as the reach reads it: where it lies along the stream's path,
/// its feed ceiling, the acceleration it may take and the most the planner
/// can plan it at ([`reaching_accel`]), the seam it starts at, the fastest
/// the seams behind let the leaders reach that seam and the envelope of the
/// seams behind that can bind there, an extrude-only move's
/// least time, the spatial axes it moves on, what it extrudes, the reach
/// of the extruder it is budgeted for, the extruder's caps, where the
/// leaders last rest before it, and once the moves after it bound how fast
/// the leaders can cross it, that speed and what follows from it. An
/// extrude-only move is a rest of the leaders, and so is the stream's
/// start.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Mark {
    start_mm: f64,
    end_mm: f64,
    ceiling_mm_s: f64,
    accel_mm_s2: f64,
    reaching_accel_mm_s2: f64,
    seam: Seam,
    forward_mm_s: f64,
    behind: SeamEnvelope,
    rest_time_s: f64,
    rest: bool,
    moves_on: [bool; 3],
    extruding: Option<(usize, f64)>,
    reach: Option<Reach>,
    extruder: Option<ExtruderCaps>,
    rest_behind_mm: f64,
    settled: Option<Settled>,
}

/// What a mark's reachable speed settles: the speed, the least time the
/// leaders take over the move, and the largest acceleration the move can
/// be planned at ([`realizable_accel`]).
#[derive(Debug, Clone, Copy, PartialEq)]
struct Settled {
    reachable: ReachableSpeed,
    least_time_s: f64,
    realizable_accel_mm_s2: f64,
}

impl Mark {
    fn length_mm(&self) -> f64 {
        self.end_mm - self.start_mm
    }

    fn ratio_on(&self, axis: usize) -> f64 {
        match self.extruding {
            Some((on, ratio)) if on == axis => ratio,
            _ => 0.0,
        }
    }

    fn settled(&self) -> Settled {
        self.settled
            .expect("a scan reads only marks whose reachable speed settled")
    }

    /// Its edge facing away from the move a scan starts at.
    fn far_edge_mm(&self, ahead: bool) -> f64 {
        if ahead { self.end_mm } else { self.start_mm }
    }

    /// Its edge facing the move a scan starts at.
    fn near_edge_mm(&self, ahead: bool) -> f64 {
        if ahead { self.start_mm } else { self.end_mm }
    }
}

/// The least time a move from rest to rest takes over `length` at speeds up
/// to `speed` and accelerations up to `accel`.
fn least_rest_to_rest_time(length: f64, speed: f64, accel: f64) -> f64 {
    if length * accel >= speed * speed {
        length / speed + speed / accel
    } else {
        2.0 * (length / accel).sqrt()
    }
}

/// What the marks dropped from the history leave behind: where the last
/// of them ends, the fastest the leaders reach on them, their largest
/// acceleration and the largest they can realize, and the span of their
/// ratios per axis.
#[derive(Debug, Clone, Default, PartialEq)]
struct Dropped {
    end_mm: f64,
    reachable_mm_s: f64,
    accel_mm_s2: f64,
    realizable_accel_mm_s2: f64,
    ratios: Vec<(usize, RatioSpan)>,
}

impl Dropped {
    fn absorb(&mut self, mark: &Mark) {
        let settled = mark.settled();
        self.end_mm = mark.end_mm;
        self.reachable_mm_s = self.reachable_mm_s.max(settled.reachable.peak_mm_s);
        self.accel_mm_s2 = self.accel_mm_s2.max(mark.accel_mm_s2);
        self.realizable_accel_mm_s2 = self
            .realizable_accel_mm_s2
            .max(settled.realizable_accel_mm_s2);
        let Some((axis, ratio)) = mark.extruding else {
            return;
        };
        match self.ratios.iter_mut().find(|(on, _)| *on == axis) {
            Some((_, span)) => *span = span.with(ratio),
            None => self.ratios.push((axis, RatioSpan::of(ratio))),
        }
    }

    fn ratio_on(&self, axis: usize) -> f64 {
        self.span_on(axis).map_or(0.0, |span| span.largest)
    }

    fn span_on(&self, axis: usize) -> Option<RatioSpan> {
        self.ratios
            .iter()
            .find(|(on, _)| *on == axis)
            .map(|(_, span)| *span)
    }
}

/// The motion on shaped leaders beside a rest: the largest acceleration
/// and extrusion ratio of the moves on them, and the least time between
/// the rest and the nearest of them.
#[derive(Debug, Clone, Copy, PartialEq)]
struct ShapedMotion {
    accel_mm_s2: f64,
    ratio: f64,
    lead_s: f64,
}

impl ShapedMotion {
    fn wider(a: Option<Self>, b: Option<Self>) -> Option<Self> {
        match (a, b) {
            (Some(a), Some(b)) => Some(Self {
                accel_mm_s2: a.accel_mm_s2.max(b.accel_mm_s2),
                ratio: a.ratio.max(b.ratio),
                lead_s: a.lead_s.min(b.lead_s),
            }),
            (a, b) => a.or(b),
        }
    }
}

/// A scan for [`ShapedMotion`] outward from a rest: what it found, the
/// least time of the moves it crossed, and whether it reached a rest.
#[derive(Debug, Clone, Copy, Default)]
struct ShapedScan {
    motion: Option<ShapedMotion>,
    elapsed_s: f64,
    rested: bool,
}

fn either(a: [bool; 3], b: [bool; 3]) -> [bool; 3] {
    std::array::from_fn(|axis| a[axis] || b[axis])
}

/// The largest and least extrusion ratios over a stretch of path, travel
/// counting as 0.
#[derive(Debug, Clone, Copy, PartialEq)]
struct RatioSpan {
    largest: f64,
    least: f64,
}

impl RatioSpan {
    fn of(ratio: f64) -> Self {
        Self {
            largest: ratio,
            least: ratio,
        }
    }

    fn with(self, ratio: f64) -> Self {
        self.union(Self::of(ratio))
    }

    fn union(self, other: Self) -> Self {
        Self {
            largest: self.largest.max(other.largest),
            least: self.least.min(other.least),
        }
    }

    fn step(&self) -> f64 {
        self.largest - self.least
    }
}

/// The spatial axes `segment` moves on.
fn moves_on(segment: &crate::path::Segment) -> [bool; 3] {
    std::array::from_fn(|axis| segment.lean_on(axis).plane != 0.0)
}

/// Where the leaders' spill ends on one side of a move, and whether they
/// rest there.
#[derive(Debug, Clone, Copy)]
struct SpillEdge {
    mm: f64,
    behind_rest: bool,
}

impl SpillEdge {
    /// Whether a ramp `ramp_mm` wide at the edge lays onto a move starting
    /// at `near_mm`, the scan heading `toward` it: never across a rest.
    fn reaches(&self, toward: f64, near_mm: f64, rest: bool, ramp_mm: f64) -> bool {
        !(self.behind_rest || rest) && toward * (near_mm - self.mm) <= ramp_mm
    }
}

/// The extrude-only caps the planner drives an extrude-only move under.
#[derive(Debug, Clone, Copy, PartialEq)]
struct ExtrudeOnly {
    velocity_mm_s: f64,
    accel_mm_s2: f64,
}

#[derive(Debug)]
struct Marks {
    marks: VecDeque<Mark>,
    path_end_mm: f64,
    dropped: Dropped,
    extrude_only: ExtrudeOnly,
    last_rest_mm: f64,
    rests_mm: VecDeque<f64>,
    tail: SeamSide,
    forward_mm_s: f64,
    behind: SeamEnvelope,
    settled_count: usize,
}

impl Marks {
    fn new(extrude_only: ExtrudeOnly, entry_mm_s: f64) -> Self {
        let tail = SeamSide::start(entry_mm_s);
        Self {
            marks: VecDeque::new(),
            path_end_mm: 0.0,
            dropped: Dropped::default(),
            extrude_only,
            last_rest_mm: 0.0,
            rests_mm: VecDeque::new(),
            tail,
            forward_mm_s: entry_mm_s,
            behind: SeamEnvelope::of(tail.limit),
            settled_count: 0,
        }
    }

    fn push(&mut self, m: &Move) {
        let start_mm = self.path_end_mm;
        let speed = m.feedrate_mm_s.min(m.limits.max_velocity_mm_s);
        let accel = m.limits.accel_mm_s2;
        let (length, ceiling_mm_s, (entry_kappa, exit_kappa), rest_time_s) =
            match &m.segment.spatial {
                Some(segment) => (segment.s_len(), speed, segment.kappa_endpoints(), 0.0),
                None => (
                    0.0,
                    0.0,
                    (0.0, 0.0),
                    least_rest_to_rest_time(
                        m.segment.virtual_path_mm.unwrap_or(0.0),
                        speed.min(self.extrude_only.velocity_mm_s),
                        accel.min(self.extrude_only.accel_mm_s2),
                    ),
                ),
            };
        let reaching = reaching_accel(m);
        let seam = self
            .tail
            .meeting(&SeamSide::of(ceiling_mm_s, entry_kappa, reaching));
        let forward_mm_s = self.forward_mm_s.min(seam.speed_mm_s);
        let behind = self.behind.with(SeamTerm {
            limit: seam.limit,
            run_mm: 0.0,
        });
        self.tail = SeamSide::of(ceiling_mm_s, exit_kappa, reaching);
        self.forward_mm_s = climbed(forward_mm_s, length, reaching);
        self.behind = behind.across(length);
        self.path_end_mm += length;
        let rest = m.segment.spatial.is_none();
        self.marks.push_back(Mark {
            start_mm,
            end_mm: self.path_end_mm,
            ceiling_mm_s,
            accel_mm_s2: accel,
            reaching_accel_mm_s2: reaching,
            seam,
            forward_mm_s,
            behind,
            rest_time_s,
            rest,
            moves_on: m.segment.spatial.as_ref().map_or([false; 3], moves_on),
            extruding: m
                .limits
                .extruder
                .map(|caps| (caps.axis_index, extruding_ratio(m, caps.axis_index))),
            reach: m
                .limits
                .extruder
                .filter(|_| m.segment.spatial.is_some())
                .map(|caps| Reach::of(&caps)),
            extruder: m.limits.extruder,
            rest_behind_mm: self.last_rest_mm,
            settled: None,
        });
        if rest {
            self.last_rest_mm = self.path_end_mm;
            self.rests_mm.push_back(self.path_end_mm);
        }
    }

    /// The seam entering `marks[seam]`, the end of the stream's a rest
    /// once it is closed; `None` past the moves known.
    fn seam_at(&self, seam: usize, closed: bool) -> Option<Seam> {
        match self.marks.get(seam) {
            Some(mark) => Some(mark.seam),
            None if closed => Some(Seam::rest()),
            None => None,
        }
    }

    /// The reach from ahead of the seam leaving `marks[index]`: its own
    /// speed, and that of every seam ahead carried back to it, up to where
    /// the climb back alone is as fast as the slowest so far; with the
    /// envelope of the seams it passed and of a rest where it stopped,
    /// which every seam beyond is no tighter than. `None` while moves still
    /// to come could lower it.
    fn backward_at_exit(&self, index: usize, closed: bool) -> Option<(f64, SeamEnvelope)> {
        let rest = Seam::rest();
        if self.marks[index].rest {
            return Some((rest.speed_mm_s, SeamEnvelope::of(rest.limit)));
        }
        let exit = self.seam_at(index + 1, closed)?;
        let mut slowest = exit.speed_mm_s;
        let mut envelope = SeamEnvelope::of(exit.limit);
        let (mut back, mut run_mm) = (0.0, 0.0);
        for ahead in index + 1..self.marks.len() {
            let mark = &self.marks[ahead];
            back = climbed(back, mark.length_mm(), mark.reaching_accel_mm_s2);
            run_mm += mark.length_mm();
            if back >= slowest {
                let beyond = SeamTerm {
                    limit: rest.limit,
                    run_mm,
                };
                return Some((slowest, envelope.with(beyond)));
            }
            let seam = self.seam_at(ahead + 1, closed)?;
            slowest = slowest.min(seam.speed_mm_s.hypot(back));
            envelope = envelope.with(SeamTerm {
                limit: seam.limit,
                run_mm,
            });
        }
        Some((slowest, envelope))
    }

    /// Settles the reachable speed of every mark whose exit seam the moves
    /// known bound, in order; `moves[index - first]` is `marks[index]`'s
    /// move.
    fn settle<M: std::ops::Index<usize, Output = Move> + ?Sized>(
        &mut self,
        closed: bool,
        moves: &M,
        first: usize,
    ) {
        while self.settled_count < self.marks.len() {
            let index = self.settled_count;
            let Some(ahead) = self.backward_at_exit(index, closed) else {
                return;
            };
            let mark = &self.marks[index];
            let length = mark.length_mm();
            let reachable = ReachableSpeed::across(
                (mark.forward_mm_s, mark.behind),
                ahead,
                (mark.ceiling_mm_s, mark.reaching_accel_mm_s2),
                length,
            );
            let least_time_s = if mark.rest {
                mark.rest_time_s
            } else {
                length / reachable.peak_mm_s
            };
            self.marks[index].settled = Some(Settled {
                reachable,
                least_time_s,
                realizable_accel_mm_s2: realizable_accel(
                    &moves[index - first],
                    reachable.peak_mm_s,
                ),
            });
            self.settled_count += 1;
        }
    }

    /// The marks on one side of `marks[index]`, nearest first.
    fn beside(&self, index: usize, ahead: bool) -> impl Iterator<Item = &Mark> {
        let count = if ahead {
            self.marks.len() - index - 1
        } else {
            index
        };
        (1..=count).map(move |step| &self.marks[if ahead { index + step } else { index - step }])
    }

    /// What `absorb` folds from the marks on one side of `marks[index]` the
    /// leaders can reach within `window_s`, and from the dropped marks
    /// behind, `None` while moves still to come could add to it.
    fn fold_beside<T>(
        &self,
        (index, ahead): (usize, bool),
        window_s: f64,
        closed: bool,
        init: T,
        absorb: impl Fn(T, &Mark) -> T,
        absorb_dropped: impl FnOnce(T, &Dropped) -> T,
    ) -> Option<T> {
        let mut elapsed = 0.0;
        let mut folded = init;
        for other in self.beside(index, ahead) {
            if elapsed > window_s {
                return Some(folded);
            }
            let settled = other.settled?;
            folded = absorb(folded, other);
            elapsed += settled.least_time_s;
        }
        if elapsed > window_s {
            return Some(folded);
        }
        match (ahead, closed) {
            (true, true) => Some(folded),
            (true, false) => None,
            (false, _) => Some(absorb_dropped(folded, &self.dropped)),
        }
    }

    /// The fastest the leaders reach on one side of `marks[index]` within
    /// `window_s`, `None` while moves still to come could raise it.
    fn fastest_beside(
        &self,
        index: usize,
        ahead: bool,
        window_s: f64,
        closed: bool,
    ) -> Option<f64> {
        self.fold_beside(
            (index, ahead),
            window_s,
            closed,
            0.0,
            |fastest: f64, other| fastest.max(other.settled().reachable.peak_mm_s),
            |fastest, dropped| fastest.max(dropped.reachable_mm_s),
        )
    }

    fn strongest_beside(
        &self,
        index: usize,
        ahead: bool,
        window_s: f64,
        closed: bool,
    ) -> Option<f64> {
        self.fold_beside(
            (index, ahead),
            window_s,
            closed,
            0.0,
            |strongest: f64, other| strongest.max(other.settled().realizable_accel_mm_s2),
            |strongest, dropped| strongest.max(dropped.realizable_accel_mm_s2),
        )
    }

    /// The spatial axes the path moves on within the leaders' spill of
    /// `marks[index]`, up to the first rest either side, which the rest
    /// holds keep the shaped path still across: the shaped path there moves
    /// only on them, and only their planned acceleration reaches its jerk.
    /// Without a spill the shaped path is the planned one, the move's own.
    /// `None` while moves still to come could add to them. The history
    /// behind always covers the spill, so the dropped marks never reach it.
    fn axes_in_reach(&self, index: usize, closed: bool) -> Option<[bool; 3]> {
        let mark = self.marks[index];
        let Some(reach) = mark.reach.filter(|reach| reach.spill_s > 0.0) else {
            return Some(mark.moves_on);
        };
        let mut axes = mark.moves_on;
        for ahead in [false, true] {
            (axes, _) = self.fold_beside(
                (index, ahead),
                reach.spill_s,
                closed,
                (axes, false),
                |(axes, rested), other| {
                    if rested || other.rest {
                        (axes, true)
                    } else {
                        (either(axes, other.moves_on), false)
                    }
                },
                |folded, _| folded,
            )?;
        }
        Some(axes)
    }

    /// The ratios on one side of `marks[index]` its demand meets, its own
    /// included: over the marks the leaders reach within the spill, then
    /// over the path a ramp `ramp_mm` wide lays beyond where the spill ends,
    /// both up to the first rest, which the rest holds keep the shaped path
    /// still across. `None` while moves still to come could add to them.
    /// The dropped marks count where the scan reaches them.
    fn ratio_beside(
        &self,
        index: usize,
        ahead: bool,
        reach: Reach,
        ramp_mm: f64,
        closed: bool,
    ) -> Option<RatioSpan> {
        let toward = if ahead { 1.0 } else { -1.0 };
        let mut elapsed = 0.0;
        let mut span = RatioSpan::of(self.marks[index].ratio_on(reach.axis));
        let mut spill_edge = SpillEdge {
            mm: self.marks[index].far_edge_mm(ahead),
            behind_rest: false,
        };
        let mut in_spill = true;
        for other in self.beside(index, ahead) {
            if in_spill {
                if other.rest {
                    return Some(span);
                }
                span = span.with(other.ratio_on(reach.axis));
                let settled = other.settled?;
                let spill_left_s = reach.spill_s - elapsed;
                elapsed += settled.least_time_s;
                spill_edge = if elapsed <= reach.spill_s {
                    SpillEdge {
                        mm: other.far_edge_mm(ahead),
                        behind_rest: other.rest,
                    }
                } else {
                    in_spill = false;
                    let spilled_mm = spill_left_s * settled.reachable.peak_mm_s;
                    SpillEdge {
                        mm: other.near_edge_mm(ahead) + toward * spilled_mm,
                        behind_rest: other.rest,
                    }
                };
                continue;
            }
            if !spill_edge.reaches(toward, other.near_edge_mm(ahead), other.rest, ramp_mm) {
                return Some(span);
            }
            span = span.with(other.ratio_on(reach.axis));
        }
        let unknown_edge_mm = if ahead {
            self.path_end_mm
        } else {
            self.dropped.end_mm
        };
        if !in_spill && !spill_edge.reaches(toward, unknown_edge_mm, false, ramp_mm) {
            return Some(span);
        }
        match (ahead, closed) {
            (true, true) => Some(span),
            (true, false) => None,
            (false, _) => Some(
                self.dropped
                    .span_on(reach.axis)
                    .map_or(span, |dropped| span.union(dropped)),
            ),
        }
    }

    /// The fastest the leaders reach within the speed window of
    /// `marks[index]`, `None` while moves still to come could raise it.
    fn fastest_in_reach(&self, index: usize, reach: Reach, closed: bool) -> Option<f64> {
        let window_s = reach.speed_window_s();
        let mut fastest = self.marks[index].settled().reachable.peak_mm_s;
        for ahead in [false, true] {
            fastest = fastest.max(self.fastest_beside(index, ahead, window_s, closed)?);
        }
        Some(fastest)
    }

    /// The largest acceleration a move within the speed window of
    /// `marks[index]` can realize, its own included ([`realizable_accel`]):
    /// what the toolhead may do over the reach of a ratio ramp at its joins,
    /// however a short move beside it is slowed. `None` while moves still to
    /// come could raise it.
    fn strongest_in_reach(&self, index: usize, reach: Reach, closed: bool) -> Option<f64> {
        let window_s = reach.speed_window_s();
        let mut strongest = self.marks[index].settled().realizable_accel_mm_s2;
        for ahead in [false, true] {
            strongest = strongest.max(self.strongest_beside(index, ahead, window_s, closed)?);
        }
        Some(strongest)
    }

    /// The ratios the demand of `marks[index]`, `reach` around it with the
    /// leaders up to `fastest`, meets.
    fn ratio_in_reach(
        &self,
        index: usize,
        reach: Reach,
        fastest: f64,
        closed: bool,
    ) -> Option<RatioSpan> {
        let ramp_mm = 0.5 * fastest * RATIO_RAMP_MAX_TIME_S;
        let behind = self.ratio_beside(index, false, reach, ramp_mm, closed)?;
        let ahead = self.ratio_beside(index, true, reach, ramp_mm, closed)?;
        Some(behind.union(ahead))
    }

    /// The path from the start of `marks[index]` to the leaders' nearest
    /// rest behind or ahead of it — an extrude-only move or an end of the
    /// stream — where it is near enough to narrow the ratio ramp of a step
    /// there ([`rest_room_reach_mm`] of the fastest the shaped leaders cross
    /// it, the leaders around it up to `fastest`), infinite where none is.
    /// `None` while moves still to come could bring one near.
    fn rest_room(&self, index: usize, reach: Reach, fastest: f64, closed: bool) -> Option<f64> {
        let mark = self.marks[index];
        let narrows_within = rest_room_reach_mm(reach.leader_spill.spill().mass * fastest);
        let behind = mark.start_mm - mark.rest_behind_mm;
        let first_ahead = self.rests_mm.partition_point(|&rest| rest < mark.start_mm);
        let ahead = match self.rests_mm.get(first_ahead) {
            Some(rest) => rest - mark.start_mm,
            None if closed => self.path_end_mm - mark.start_mm,
            None if self.path_end_mm - mark.start_mm > narrows_within => f64::INFINITY,
            None => return None,
        };
        let room = behind.min(ahead);
        Some(if room <= narrows_within {
            room
        } else {
            f64::INFINITY
        })
    }

    /// The scan for motion on the leaders the kernels shape within
    /// `window_s` on one side of `marks[index]`, up to the first rest: the
    /// largest acceleration and extrusion ratio on `caps`' axis of the
    /// moves on a shaped leader, and the least time the moves between take
    /// before the first of them, which only unshaped leaders cross; no
    /// motion where no shaped leader moves there. `None` while moves still
    /// to come could change it. The history behind counts as shaped motion
    /// where the scan reaches it.
    fn shaped_scan_beside(
        &self,
        (index, ahead): (usize, bool),
        window_s: f64,
        caps: &ExtruderCaps,
        closed: bool,
    ) -> Option<ShapedScan> {
        let shaped =
            |moves_on: [bool; 3]| (0..3).any(|axis| moves_on[axis] && !caps.unshaped_leaders[axis]);
        self.fold_beside(
            (index, ahead),
            window_s,
            closed,
            ShapedScan::default(),
            |scan, other| {
                if scan.rested || other.rest {
                    return ShapedScan {
                        rested: true,
                        ..scan
                    };
                }
                let found = shaped(other.moves_on).then(|| ShapedMotion {
                    accel_mm_s2: other.accel_mm_s2,
                    ratio: other.ratio_on(caps.axis_index),
                    lead_s: scan.elapsed_s,
                });
                ShapedScan {
                    motion: ShapedMotion::wider(scan.motion, found),
                    elapsed_s: scan.elapsed_s + other.settled().least_time_s,
                    rested: false,
                }
            },
            |scan, dropped| {
                if scan.rested {
                    return scan;
                }
                let history = ShapedMotion {
                    accel_mm_s2: dropped.accel_mm_s2,
                    ratio: dropped.ratio_on(caps.axis_index),
                    lead_s: scan.elapsed_s,
                };
                ShapedScan {
                    motion: ShapedMotion::wider(scan.motion, Some(history)),
                    ..scan
                }
            },
        )
    }

    /// How long an extrude-only `marks[index]` holds the toolhead still
    /// before and after its own motion, where it starts or ends a run of
    /// extrude-only moves beside motion of the leaders the kernels shape:
    /// until their shaped path, settling under the largest acceleration
    /// the moves on them take, stands within the settle's radius of the
    /// rest point and asks the extruder for no more than the extrude-only
    /// caps leave it ([`crate::frontend::SettleSide::hold_s`]), less the
    /// time moves on unshaped leaders alone already put between, and
    /// together with that time and the run at least the kernels' reach on
    /// both sides, so the shaped path past the stop never meets the one
    /// ahead of the start across the run. `None` while moves still to come
    /// could change it. Nothing holds at a stream's edges, where the
    /// lowerer's rest holds already keep the leaders still.
    fn rest_holds(&self, index: usize, closed: bool) -> Option<[f64; 2]> {
        let mark = self.marks[index];
        let (true, Some(caps)) = (mark.rest, mark.extruder) else {
            return Some([0.0; 2]);
        };
        let first = index
            - self
                .beside(index, false)
                .take_while(|other| other.rest)
                .count();
        let last = index
            + self
                .beside(index, true)
                .take_while(|other| other.rest)
                .count();
        if last + 1 == self.marks.len() && !closed {
            return None;
        }
        let settle = caps.rest_settle.settle();
        let motion_beside = |side: usize, from: usize, ahead: bool, at_edge: bool| {
            if at_edge {
                return Some(None);
            }
            self.shaped_scan_beside((from, ahead), settle.sides[side].reach_s, &caps, closed)
                .map(|scan| scan.motion)
        };
        let motion = [
            motion_beside(0, first, false, first == 0)?,
            motion_beside(1, last, true, last + 1 == self.marks.len())?,
        ];
        let mut holds = [0, 1].map(|side| {
            motion[side].map_or(0.0, |motion| {
                let demand = SettleDemand {
                    ratio: motion.ratio,
                    advance_slope_s: caps.advance_slope_s,
                    advance_curvature_s2_per_mm: caps.advance_curvature_s2_per_mm,
                    own_jerk_per_accel_hz: caps.own_jerk_per_accel_hz,
                    max_velocity_mm_s: caps.max_velocity_mm_s,
                    max_accel_mm_s2: caps.max_accel_mm_s2,
                    extrude_only_velocity_mm_s: self.extrude_only.velocity_mm_s,
                    extrude_only_accel_mm_s2: self.extrude_only.accel_mm_s2,
                };
                let settled =
                    settle.sides[side].hold_s(settle.radius_mm, motion.accel_mm_s2, &demand);
                (settled - motion.lead_s).max(0.0)
            })
        });
        if let [Some(before), Some(after)] = motion {
            let run_s: f64 = (first..=last).map(|at| self.marks[at].rest_time_s).sum();
            let reaches = [settle.sides[0].reach_s, settle.sides[1].reach_s];
            let leads = [before.lead_s, after.lead_s];
            let short =
                reaches[0] + reaches[1] - (leads[0] + holds[0] + run_s + holds[1] + leads[1]);
            if short > 0.0 {
                let left = [0, 1].map(|side| reaches[side] - leads[side] - holds[side]);
                let room = left[0] + left[1];
                holds = [0, 1].map(|side| holds[side] + short * left[side] / room);
            }
        }
        Some([
            if index == first { holds[0] } else { 0.0 },
            if index == last { holds[1] } else { 0.0 },
        ])
    }

    fn budget(&self, index: usize, closed: bool) -> Option<ExtrusionBudget> {
        let reachable = self.marks[index].settled?.reachable;
        let extruding = match self.marks[index].reach {
            Some(reach) => {
                let fastest = self.fastest_in_reach(index, reach, closed)?;
                let ratios = self.ratio_in_reach(index, reach, fastest, closed)?;
                ExtrudingReach {
                    ratio_in_reach: ratios.largest,
                    ratio_step_in_reach: ratios.step(),
                    rest_room_mm: self.rest_room(index, reach, fastest, closed)?,
                    fastest_reachable_mm_s: fastest,
                    strongest_accel_mm_s2: self.strongest_in_reach(index, reach, closed)?,
                }
            }
            None => ExtrudingReach::NONE,
        };
        Some(ExtrusionBudget {
            ratio_in_reach: extruding.ratio_in_reach,
            ratio_step_in_reach: extruding.ratio_step_in_reach,
            axes_in_reach: self.axes_in_reach(index, closed)?,
            rest_holds_s: self.rest_holds(index, closed)?,
            rest_room_mm: extruding.rest_room_mm,
            fastest_reachable_mm_s: extruding.fastest_reachable_mm_s,
            strongest_accel_mm_s2: extruding.strongest_accel_mm_s2,
            reachable,
        })
    }

    fn drop_front(&mut self) {
        let mark = self.marks.pop_front().expect("a mark to drop");
        if mark.rest {
            self.rests_mm.pop_front();
        }
        self.dropped.absorb(&mark);
        self.settled_count -= 1;
    }
}

/// What a move's plan takes from the moves around it: the largest
/// extrusion ratio its extruder demand can meet and the largest step of the
/// ratio there, the larger ratio less the smaller with travel at 0, whose
/// ramp its budget leaves room for, the spatial axes the path moves on
/// within the leaders' spill of it, whose kernels the shaped path's jerk
/// there passes through, for an extrude-only move how long it holds the
/// toolhead still before and after its own motion, so the extruder never
/// adds the leaders' spill to it, the path from its start to the leaders'
/// nearest rest where that narrows the ratio ramp of a step there, the
/// fastest the leaders reach within their reach of it, which bounds the
/// shaped speed at its joins (infinite on a move no extruder budget
/// reaches), the largest acceleration a move within that reach may take,
/// which a ratio ramp at its joins rides (0 on a move no extruder budget
/// reaches), and the fastest the leaders can cross it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ExtrusionBudget {
    pub ratio_in_reach: f64,
    pub ratio_step_in_reach: f64,
    pub axes_in_reach: [bool; 3],
    pub rest_holds_s: [f64; 2],
    pub rest_room_mm: f64,
    pub fastest_reachable_mm_s: f64,
    pub strongest_accel_mm_s2: f64,
    pub reachable: ReachableSpeed,
}

/// The part of an [`ExtrusionBudget`] a move's extruder reach gives it.
struct ExtrudingReach {
    ratio_in_reach: f64,
    ratio_step_in_reach: f64,
    rest_room_mm: f64,
    fastest_reachable_mm_s: f64,
    strongest_accel_mm_s2: f64,
}

impl ExtrudingReach {
    const NONE: Self = Self {
        ratio_in_reach: 0.0,
        ratio_step_in_reach: 0.0,
        rest_room_mm: f64::INFINITY,
        fastest_reachable_mm_s: f64::INFINITY,
        strongest_accel_mm_s2: 0.0,
    };
}

/// Each move's [`ExtrusionBudget`]: the largest extrusion ratio within its
/// [`Reach`], which the leaders' kernels and a ratio ramp carry across the
/// joins however short the moves between, and an extrude-only move's rest
/// holds. Moves on a stream pass through in order and leave once the moves
/// after them cover their reach, so a move's budget never depends on how
/// far the stream had arrived when it was planned.
#[derive(Debug)]
pub struct ExtrusionReach {
    marks: Marks,
    pending: VecDeque<Move>,
    closed: bool,
    history_mm: f64,
}

impl ExtrusionReach {
    /// The reach of a stream whose extrude-only moves the planner drives
    /// under these caps.
    pub fn new(max_extrude_only_velocity_mm_s: f64, max_extrude_only_accel_mm_s2: f64) -> Self {
        Self {
            marks: Marks::new(
                ExtrudeOnly {
                    velocity_mm_s: max_extrude_only_velocity_mm_s,
                    accel_mm_s2: max_extrude_only_accel_mm_s2,
                },
                0.0,
            ),
            pending: VecDeque::new(),
            closed: false,
            history_mm: 0.0,
        }
    }

    /// Adds the stream's next move. After the stream came to rest and
    /// settled every move, the next one starts a new stream: the toolhead
    /// rests between them, so nothing before reaches it.
    pub fn push(&mut self, m: Move) {
        if self.closed && self.pending.is_empty() {
            self.clear();
        }
        if let Some(caps) = m.limits.extruder {
            self.history_mm = self
                .history_mm
                .max(m.limits.max_velocity_mm_s * Reach::of(&caps).speed_window_s());
        }
        self.marks.push(&m);
        self.pending.push_back(m);
        self.closed = false;
    }

    /// The stream comes to rest after the moves pushed so far: they settle
    /// on the moves known now.
    pub fn close(&mut self) {
        self.closed = true;
    }

    /// The oldest move still waiting, once the moves after it cover its
    /// reach, with its budget.
    pub fn pop(&mut self) -> Option<(Move, ExtrusionBudget)> {
        if self.pending.is_empty() {
            return None;
        }
        let first = self.marks.marks.len() - self.pending.len();
        self.marks.settle(self.closed, &self.pending, first);
        let budget = self.marks.budget(first, self.closed)?;
        let m = self.pending.pop_front().expect("a waiting move");
        self.forget_beyond_history();
        Some((m, budget))
    }

    pub fn is_empty(&self) -> bool {
        self.pending.is_empty()
    }

    /// Forgets the stream: the next move starts a new one.
    pub fn clear(&mut self) {
        *self = Self::new(
            self.marks.extrude_only.velocity_mm_s,
            self.marks.extrude_only.accel_mm_s2,
        );
    }

    /// Drops the settled marks no move still to settle can reach: those
    /// further behind the oldest waiting move than the widest reach seen
    /// covers at its speed limit.
    fn forget_beyond_history(&mut self) {
        let settled = self.marks.marks.len() - self.pending.len();
        let front = self
            .marks
            .marks
            .get(settled)
            .map_or(self.marks.path_end_mm, |mark| mark.start_mm);
        for _ in 0..settled {
            if front - self.marks.marks[0].end_mm <= self.history_mm {
                break;
            }
            self.marks.drop_front();
        }
    }
}

/// [`ExtrusionReach`] over a whole stream at once, the leaders entering it
/// at `entry_mm_s` and resting after it, nothing else before or after it.
pub fn extrusion_budgets(
    moves: &[Move],
    entry_mm_s: f64,
    max_extrude_only_velocity_mm_s: f64,
    max_extrude_only_accel_mm_s2: f64,
) -> Vec<ExtrusionBudget> {
    let mut marks = Marks::new(
        ExtrudeOnly {
            velocity_mm_s: max_extrude_only_velocity_mm_s,
            accel_mm_s2: max_extrude_only_accel_mm_s2,
        },
        entry_mm_s,
    );
    for m in moves {
        marks.push(m);
    }
    marks.settle(true, moves, 0);
    (0..moves.len())
        .map(|index| {
            marks
                .budget(index, true)
                .expect("a closed stream settles every move")
        })
        .collect()
}

#[cfg(test)]
mod tests;
