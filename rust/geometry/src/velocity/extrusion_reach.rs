use std::collections::VecDeque;

use crate::Move;
use crate::frontend::ExtruderCaps;
use crate::path::CurvatureProfile;
use crate::ratio_ramp::RATIO_RAMP_MAX_TIME_S;

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
}

impl Reach {
    fn of(caps: &ExtruderCaps) -> Self {
        Self {
            axis: caps.axis_index,
            spill_s: 2.0 * caps.leader_kernel_reach_s,
        }
    }

    fn speed_window_s(&self) -> f64 {
        self.spill_s + RATIO_RAMP_MAX_TIME_S
    }
}

/// One move as the reach reads it: where it lies along the stream's path,
/// the fastest the leaders can cross it and the least time they take, the
/// spatial axes it moves on, what it extrudes, the reach of the extruder it
/// is budgeted for, and how long the leaders' shaped path moves on past a
/// raw stop and before a raw start. An extrude-only move is a rest of the
/// leaders.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Mark {
    start_mm: f64,
    end_mm: f64,
    ceiling_mm_s: f64,
    least_time_s: f64,
    rest: bool,
    moves_on: [bool; 3],
    extruding: Option<(usize, f64)>,
    reach: Option<Reach>,
    leader_kernel_reach_s: f64,
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
/// of them ends, their fastest ceiling and their largest ratio per axis.
#[derive(Debug, Clone, Default, PartialEq)]
struct Dropped {
    end_mm: f64,
    ceiling_mm_s: f64,
    ratios: Vec<(usize, f64)>,
}

impl Dropped {
    fn absorb(&mut self, mark: &Mark) {
        self.end_mm = mark.end_mm;
        self.ceiling_mm_s = self.ceiling_mm_s.max(mark.ceiling_mm_s);
        let Some((axis, ratio)) = mark.extruding else {
            return;
        };
        match self.ratios.iter_mut().find(|(on, _)| *on == axis) {
            Some((_, largest)) => *largest = largest.max(ratio),
            None => self.ratios.push((axis, ratio)),
        }
    }

    fn ratio_on(&self, axis: usize) -> f64 {
        self.ratios
            .iter()
            .find(|(on, _)| *on == axis)
            .map_or(0.0, |(_, ratio)| *ratio)
    }
}

fn either(a: [bool; 3], b: [bool; 3]) -> [bool; 3] {
    std::array::from_fn(|axis| a[axis] || b[axis])
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
}

impl Marks {
    fn new(extrude_only: ExtrudeOnly) -> Self {
        Self {
            marks: VecDeque::new(),
            path_end_mm: 0.0,
            dropped: Dropped::default(),
            extrude_only,
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
        self.marks.push_back(Mark {
            start_mm,
            end_mm: self.path_end_mm,
            ceiling_mm_s,
            least_time_s,
            rest: m.segment.spatial.is_none(),
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
            leader_kernel_reach_s: m
                .limits
                .extruder
                .map_or(0.0, |caps| caps.leader_kernel_reach_s),
        });
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

    /// The largest ratio on one side of `marks[index]` its demand meets:
    /// over the marks the leaders reach within the spill, then over the
    /// path a ramp `ramp_mm` wide lays beyond where the spill ends, up to
    /// the first rest. `None` while moves still to come could add to it.
    fn ratio_beside(
        &self,
        index: usize,
        ahead: bool,
        reach: Reach,
        ramp_mm: f64,
        closed: bool,
    ) -> Option<f64> {
        let toward = if ahead { 1.0 } else { -1.0 };
        let mut elapsed = 0.0;
        let mut ratio: f64 = 0.0;
        let mut spill_edge = SpillEdge {
            mm: self.marks[index].far_edge_mm(ahead),
            behind_rest: false,
        };
        let mut in_spill = true;
        for other in self.beside(index, ahead) {
            if in_spill {
                ratio = ratio.max(other.ratio_on(reach.axis));
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
                return Some(ratio);
            }
            ratio = ratio.max(other.ratio_on(reach.axis));
        }
        let unknown_edge_mm = if ahead {
            self.path_end_mm
        } else {
            self.dropped.end_mm
        };
        if !in_spill && !spill_edge.reaches(toward, unknown_edge_mm, false, ramp_mm) {
            return Some(ratio);
        }
        match (ahead, closed) {
            (true, true) => Some(ratio),
            (true, false) => None,
            (false, _) => Some(ratio.max(self.dropped.ratio_on(reach.axis))),
        }
    }

    /// The largest ratio the demand of `marks[index]` meets, `None` while
    /// moves still to come could add to it. `closed`: none will.
    fn ratio_in_reach(&self, index: usize, closed: bool) -> Option<f64> {
        let mark = self.marks[index];
        let Some(reach) = mark.reach else {
            return Some(0.0);
        };
        let window_s = reach.speed_window_s();
        let mut fastest = mark.ceiling_mm_s;
        for ahead in [false, true] {
            fastest = fastest.max(self.fastest_beside(index, ahead, window_s, closed)?);
        }
        let ramp_mm = 0.5 * fastest * RATIO_RAMP_MAX_TIME_S;
        let mut ratio = mark.ratio_on(reach.axis);
        for ahead in [false, true] {
            ratio = ratio.max(self.ratio_beside(index, ahead, reach, ramp_mm, closed)?);
        }
        Some(ratio)
    }

    /// How long an extrude-only `marks[index]` holds the toolhead still
    /// before and after its own motion: the leaders' kernel reach on each
    /// side where the leaders move, so the shaped path has come to rest
    /// before the extruder starts and stays at rest until it stops. `None`
    /// while the move after it is still to come. Nothing holds at a stream's
    /// edges, where the lowerer's rest holds already keep the leaders still.
    fn rest_holds(&self, index: usize, closed: bool) -> Option<[f64; 2]> {
        let mark = self.marks[index];
        if !mark.rest {
            return Some([0.0; 2]);
        }
        let after = self.marks.get(index + 1);
        if after.is_none() && !closed {
            return None;
        }
        let before = index.checked_sub(1).map(|previous| &self.marks[previous]);
        let hold_beside = |other: Option<&Mark>| {
            if other.is_some_and(|other| !other.rest) {
                mark.leader_kernel_reach_s
            } else {
                0.0
            }
        };
        Some([hold_beside(before), hold_beside(after)])
    }

    fn budget(&self, index: usize, closed: bool) -> Option<ExtrusionBudget> {
        Some(ExtrusionBudget {
            ratio_in_reach: self.ratio_in_reach(index, closed)?,
            axes_in_reach: self.axes_in_reach(index, closed)?,
            rest_holds_s: self.rest_holds(index, closed)?,
        })
    }

    fn drop_front(&mut self) {
        let mark = self.marks.pop_front().expect("a mark to drop");
        self.dropped.absorb(&mark);
    }
}

/// What a move's plan takes from the moves around it: the largest
/// extrusion ratio its extruder demand can meet, the spatial axes the path
/// moves on within the leaders' spill of it, whose kernels the shaped
/// path's jerk there passes through, and for an extrude-only move how long
/// it holds the toolhead still before and after its own motion, so the
/// extruder never adds the leaders' spill to it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ExtrusionBudget {
    pub ratio_in_reach: f64,
    pub axes_in_reach: [bool; 3],
    pub rest_holds_s: [f64; 2],
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
