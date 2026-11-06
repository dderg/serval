use std::collections::VecDeque;

use crate::Move;
use crate::frontend::{ExtruderCaps, LeaderSpillId, SettleDemand};
use crate::path::CurvatureProfile;

use super::extruder_caps::extruder_capped;
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

/// The most acceleration `m` can be planned at from its own caps alone:
/// its limit, held by the z cap ([`super::z_caps::z_capped`]) and by the
/// extruder caps on its own extrusion ratio and axes
/// ([`super::extruder_caps::extruder_capped`]), which the ratio and axes
/// within its reach only tighten.
fn realizable_accel(m: &Move) -> f64 {
    let Some(segment) = &m.segment.spatial else {
        return m.limits.accel_mm_s2;
    };
    let (_, kappa_peak) = segment.kappa_peak();
    let path_ceiling = m.feedrate_mm_s.min(m.limits.max_velocity_mm_s);
    let z_held = z_capped(segment, &m.limits, kappa_peak, path_ceiling);
    let (_, accel) = m.limits.extruder.map_or(z_held, |caps| {
        let own = ExtrusionBudget {
            ratio_in_reach: extruding_ratio(m, caps.axis_index),
            ratio_step_in_reach: 0.0,
            axes_in_reach: moves_on(segment),
            rest_holds_s: [0.0; 2],
            rest_room_mm: f64::INFINITY,
            fastest_ceiling_mm_s: f64::INFINITY,
            strongest_accel_mm_s2: 0.0,
        };
        extruder_capped(segment, &caps, &own, kappa_peak, z_held)
    });
    accel
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
/// the fastest the leaders can cross it, the least time they take and the
/// acceleration they may take it at, the spatial axes it moves on, what it
/// extrudes, the reach of the extruder it is budgeted for, the extruder's
/// caps, and where the leaders last rest before it. An extrude-only move
/// is a rest of the leaders, and so is the stream's start.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Mark {
    start_mm: f64,
    end_mm: f64,
    ceiling_mm_s: f64,
    least_time_s: f64,
    accel_mm_s2: f64,
    realizable_accel_mm_s2: f64,
    rest: bool,
    moves_on: [bool; 3],
    extruding: Option<(usize, f64)>,
    reach: Option<Reach>,
    extruder: Option<ExtruderCaps>,
    rest_behind_mm: f64,
}

impl Mark {
    fn ratio_on(&self, axis: usize) -> f64 {
        match self.extruding {
            Some((on, ratio)) if on == axis => ratio,
            _ => 0.0,
        }
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
/// of them ends, their fastest ceiling, their largest acceleration and the
/// largest they can realize, and the span of their ratios per axis.
#[derive(Debug, Clone, Default, PartialEq)]
struct Dropped {
    end_mm: f64,
    ceiling_mm_s: f64,
    accel_mm_s2: f64,
    realizable_accel_mm_s2: f64,
    ratios: Vec<(usize, RatioSpan)>,
}

impl Dropped {
    fn absorb(&mut self, mark: &Mark) {
        self.end_mm = mark.end_mm;
        self.ceiling_mm_s = self.ceiling_mm_s.max(mark.ceiling_mm_s);
        self.accel_mm_s2 = self.accel_mm_s2.max(mark.accel_mm_s2);
        self.realizable_accel_mm_s2 = self.realizable_accel_mm_s2.max(mark.realizable_accel_mm_s2);
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
}

impl Marks {
    fn new(extrude_only: ExtrudeOnly) -> Self {
        Self {
            marks: VecDeque::new(),
            path_end_mm: 0.0,
            dropped: Dropped::default(),
            extrude_only,
            last_rest_mm: 0.0,
            rests_mm: VecDeque::new(),
        }
    }

    fn push(&mut self, m: &Move) {
        let start_mm = self.path_end_mm;
        let speed = m.feedrate_mm_s.min(m.limits.max_velocity_mm_s);
        let (length, ceiling_mm_s, least_time_s) = match &m.segment.spatial {
            Some(segment) => {
                let length = segment.s_len();
                (length, speed, length / speed)
            }
            None => (
                0.0,
                0.0,
                least_rest_to_rest_time(
                    m.segment.virtual_path_mm.unwrap_or(0.0),
                    speed.min(self.extrude_only.velocity_mm_s),
                    m.limits.accel_mm_s2.min(self.extrude_only.accel_mm_s2),
                ),
            ),
        };
        self.path_end_mm += length;
        let rest = m.segment.spatial.is_none();
        self.marks.push_back(Mark {
            start_mm,
            end_mm: self.path_end_mm,
            ceiling_mm_s,
            least_time_s,
            accel_mm_s2: m.limits.accel_mm_s2,
            realizable_accel_mm_s2: realizable_accel(m),
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
        });
        if rest {
            self.last_rest_mm = self.path_end_mm;
            self.rests_mm.push_back(self.path_end_mm);
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
            folded = absorb(folded, other);
            elapsed += other.least_time_s;
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

    /// The fastest ceiling on one side of `marks[index]` the leaders can
    /// reach within `window_s`, `None` while moves still to come could.
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
            |fastest: f64, other| fastest.max(other.ceiling_mm_s),
            |fastest, dropped| fastest.max(dropped.ceiling_mm_s),
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
            |strongest: f64, other| strongest.max(other.realizable_accel_mm_s2),
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
                let spill_left_s = reach.spill_s - elapsed;
                elapsed += other.least_time_s;
                spill_edge = if elapsed <= reach.spill_s {
                    SpillEdge {
                        mm: other.far_edge_mm(ahead),
                        behind_rest: other.rest,
                    }
                } else {
                    in_spill = false;
                    let spilled_mm = spill_left_s * other.ceiling_mm_s;
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

    /// The fastest planned ceiling the leaders reach within the speed
    /// window of `marks[index]`, `None` while moves still to come could
    /// raise it.
    fn fastest_in_reach(&self, index: usize, reach: Reach, closed: bool) -> Option<f64> {
        let window_s = reach.speed_window_s();
        let mut fastest = self.marks[index].ceiling_mm_s;
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
        let mut strongest = self.marks[index].realizable_accel_mm_s2;
        for ahead in [false, true] {
            strongest = strongest.max(self.strongest_beside(index, ahead, window_s, closed)?);
        }
        Some(strongest)
    }

    /// The ratios the demand of `marks[index]`, `reach` around it with
    /// ceilings up to `fastest`, meets.
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
    /// it, the planned ceilings around it up to `fastest`), infinite where
    /// none is. `None` while moves still to come could bring one near.
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
                    elapsed_s: scan.elapsed_s + other.least_time_s,
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
            let run_s: f64 = (first..=last).map(|at| self.marks[at].least_time_s).sum();
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
        let extruding = match self.marks[index].reach {
            Some(reach) => {
                let fastest = self.fastest_in_reach(index, reach, closed)?;
                let ratios = self.ratio_in_reach(index, reach, fastest, closed)?;
                ExtrudingReach {
                    ratio_in_reach: ratios.largest,
                    ratio_step_in_reach: ratios.step(),
                    rest_room_mm: self.rest_room(index, reach, fastest, closed)?,
                    fastest_ceiling_mm_s: fastest,
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
            fastest_ceiling_mm_s: extruding.fastest_ceiling_mm_s,
            strongest_accel_mm_s2: extruding.strongest_accel_mm_s2,
        })
    }

    fn drop_front(&mut self) {
        let mark = self.marks.pop_front().expect("a mark to drop");
        if mark.rest {
            self.rests_mm.pop_front();
        }
        self.dropped.absorb(&mark);
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
/// nearest rest where that narrows the ratio ramp of a step there, and the
/// fastest planned ceiling within the leaders' reach of it, which bounds
/// the shaped speed at its joins (infinite on a move no extruder budget
/// reaches), and the largest acceleration a move within that reach may
/// take, which a ratio ramp at its joins rides (0 on a move no extruder
/// budget reaches).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ExtrusionBudget {
    pub ratio_in_reach: f64,
    pub ratio_step_in_reach: f64,
    pub axes_in_reach: [bool; 3],
    pub rest_holds_s: [f64; 2],
    pub rest_room_mm: f64,
    pub fastest_ceiling_mm_s: f64,
    pub strongest_accel_mm_s2: f64,
}

/// The part of an [`ExtrusionBudget`] a move's extruder reach gives it.
struct ExtrudingReach {
    ratio_in_reach: f64,
    ratio_step_in_reach: f64,
    rest_room_mm: f64,
    fastest_ceiling_mm_s: f64,
    strongest_accel_mm_s2: f64,
}

impl ExtrudingReach {
    const NONE: Self = Self {
        ratio_in_reach: 0.0,
        ratio_step_in_reach: 0.0,
        rest_room_mm: f64::INFINITY,
        fastest_ceiling_mm_s: f64::INFINITY,
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
            marks: Marks::new(ExtrudeOnly {
                velocity_mm_s: max_extrude_only_velocity_mm_s,
                accel_mm_s2: max_extrude_only_accel_mm_s2,
            }),
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
        let index = self.marks.marks.len() - self.pending.len();
        let budget = self.marks.budget(index, self.closed)?;
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

/// [`ExtrusionReach`] over a whole stream at once, nothing before or after
/// it.
pub fn extrusion_budgets(
    moves: &[Move],
    max_extrude_only_velocity_mm_s: f64,
    max_extrude_only_accel_mm_s2: f64,
) -> Vec<ExtrusionBudget> {
    let mut marks = Marks::new(ExtrudeOnly {
        velocity_mm_s: max_extrude_only_velocity_mm_s,
        accel_mm_s2: max_extrude_only_accel_mm_s2,
    });
    for m in moves {
        marks.push(m);
    }
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
