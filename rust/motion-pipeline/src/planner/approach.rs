//! Slows the corners a stop at their acceleration would round past their
//! budget ([`ShapedCorners::slows`]). Each such corner's blend, and a
//! short approach split off the lines either side of it
//! ([`ShapedCorners::approach_mm`]), get the acceleration
//! [`ShapedCorners::corner_accel`] leaves it baked into their limits, so
//! only the path the shaper reads through the corner slows. Corners close
//! together trace a curve the shaper shrinks as a whole: each blend's
//! halves get the speed at which a circle through the corner's turn over
//! the shorter leg to a neighbouring corner keeps its shrink within the
//! budget's room beyond the cut ([`ShapedCorners::arc_speed_cap`]).
//! Every other curve, an arc or a clothoid no corner blend claims, is
//! capped at the speed that keeps the shaper's pull on it within its
//! budget's room. A move leaves here final: the window plans it as it will
//! commit.

use std::collections::VecDeque;

use geometry::path::lowering::PositionProfile;
use geometry::path::{CurvatureProfile, Line, PathSegment, Segment};
use geometry::shaped_corner::{blend_apex, stop_turn};
use geometry::{CornerFitConfig, Move, ShapedCorners, seam_requires_stop};

/// The moves the stage holds back: a blend's verdict reaches the move
/// before its first half, which waits for the second, and its curve cap
/// reaches the blend before it across a consumed line.
const HELD_MOVES: usize = 4;

pub(super) struct Approaches {
    corners: Option<&'static ShapedCorners>,
    config: CornerFitConfig,
    held: VecDeque<(u64, Vec<Move>)>,
    next_id: u64,
    /// A slowed corner's acceleration and the approach still to slow past
    /// it on the moves to come.
    slowing_ahead: Option<(f64, f64)>,
    since_corner_mm: f64,
    last_apex: Option<HeldApex>,
    /// The last held move when it is a clothoid that may yet turn out to
    /// open a corner blend; a curve otherwise.
    unsettled_clothoid: Option<u64>,
}

/// A blend still held, as its curve cap needs it once the next corner
/// shows how far away that is.
struct HeldApex {
    halves: [u64; 2],
    theta: f64,
    room_mm: f64,
    leg_in_mm: f64,
}

impl Approaches {
    pub(super) fn new(config: CornerFitConfig) -> Self {
        Self {
            corners: config.shaped_corners,
            config,
            held: VecDeque::new(),
            next_id: 0,
            slowing_ahead: None,
            since_corner_mm: f64::INFINITY,
            last_apex: None,
            unsettled_clothoid: None,
        }
    }

    /// Takes `m` and releases every move no later corner can slow.
    pub(super) fn push(&mut self, m: Move, release: &mut impl FnMut(Move)) {
        let Some(corners) = self.corners else {
            release(m);
            return;
        };
        let mut pieces = vec![m];
        if let Some((accel, approach_mm)) = self.slowing_ahead.take() {
            self.slow_ahead(&mut pieces, accel, approach_mm);
        }
        let leg_mm = self.since_corner_mm;
        let verdict = self.held.back().map_or(Verdict::None, |(id, previous)| {
            let before = last(previous);
            if seam_requires_stop(before, &pieces[0], self.config) {
                Verdict::Stop {
                    turns: stop_turn(before, &pieces[0]).is_some(),
                    slowed: stopped_corner_accel(before, &pieces[0], corners),
                }
            } else {
                BlendCorner::of(before, &pieces[0], corners)
                    .map_or(Verdict::None, |apex| Verdict::Blend(*id, apex))
            }
        });
        let closes_blend = matches!(verdict, Verdict::Blend(..));
        if let Some(id) = self.unsettled_clothoid.take() {
            if !closes_blend {
                self.cap_curves(id, corners);
            }
        }
        match &pieces[0].segment.spatial {
            Some(Segment::Arc(_)) => pieces
                .iter_mut()
                .for_each(|piece| cap_curve(piece, corners)),
            Some(Segment::Clothoid(_)) if !closes_blend => {
                self.unsettled_clothoid = Some(self.next_id);
            }
            _ => {}
        }
        let corner = match verdict {
            Verdict::None => false,
            Verdict::Stop { turns, slowed } => {
                if let Some((accel, approach_mm)) = slowed {
                    self.slow_back(self.held.len(), accel, approach_mm);
                    self.slow_ahead(&mut pieces, accel, approach_mm);
                }
                if turns {
                    self.close_curve(corners, leg_mm);
                }
                turns
            }
            Verdict::Blend(half1_id, apex) => {
                if let Some((accel, approach_mm)) = apex.slowed {
                    let (_, half1) = self
                        .held
                        .back_mut()
                        .expect("a blend follows its first half");
                    half1.iter_mut().for_each(|piece| bake(piece, accel));
                    pieces.iter_mut().for_each(|piece| bake(piece, accel));
                    self.slow_back(self.held.len() - 1, accel, approach_mm);
                    self.slowing_ahead = Some((accel, approach_mm));
                }
                self.close_curve(corners, leg_mm);
                let held = HeldApex {
                    halves: [half1_id, self.next_id],
                    theta: apex.theta,
                    room_mm: apex.budget_mm - apex.cut_mm,
                    leg_in_mm: leg_mm,
                };
                let cap = corners.arc_speed_cap(held.leg_in_mm / held.theta, held.room_mm);
                self.cap_halves(held.halves, cap);
                pieces.iter_mut().for_each(|piece| cap_speed(piece, cap));
                self.last_apex = Some(held);
                true
            }
        };
        self.since_corner_mm = if corner { 0.0 } else { self.since_corner_mm };
        self.since_corner_mm += pieces.iter().map(path_length).sum::<f64>();
        self.held.push_back((self.next_id, pieces));
        self.next_id += 1;
        while self.held.len() > HELD_MOVES {
            for piece in self.held.pop_front().expect("held").1 {
                release(piece);
            }
        }
    }

    /// Slows `approach_mm` of path back from the end of the held moves
    /// before `end`, splitting the line it ends on.
    fn slow_back(&mut self, end: usize, accel: f64, approach_mm: f64) {
        let mut left = approach_mm;
        for (_, pieces) in self.held.range_mut(..end).rev() {
            let mut index = pieces.len();
            while left > 0.0 && index > 0 {
                index -= 1;
                let length = path_length(&pieces[index]);
                if length > left && splits(&pieces[index], length - left, approach_mm) {
                    let (body, mut tail) = split_at(&pieces[index], length - left);
                    bake(&mut tail, accel);
                    pieces.splice(index..=index, [body, tail]);
                    left = 0.0;
                } else {
                    bake(&mut pieces[index], accel);
                    left -= length;
                }
            }
            if left <= 0.0 {
                return;
            }
        }
    }

    /// Slows `approach_mm` of path on from the start of `pieces`, splitting
    /// the line it ends on, and carries what is left to the moves to come.
    fn slow_ahead(&mut self, pieces: &mut Vec<Move>, accel: f64, approach_mm: f64) {
        let mut left = approach_mm;
        let mut index = 0;
        while left > 0.0 && index < pieces.len() {
            let length = path_length(&pieces[index]);
            if length > left && splits(&pieces[index], left, approach_mm) {
                let (mut head, body) = split_at(&pieces[index], left);
                bake(&mut head, accel);
                pieces.splice(index..=index, [head, body]);
                left = 0.0;
            } else {
                bake(&mut pieces[index], accel);
                left -= length;
            }
            index += 1;
        }
        if left > 0.0 {
            self.slowing_ahead = Some((accel, left));
        }
    }

    /// The last blend's curve cap once the next corner lies `leg_out_mm`
    /// past it: the shorter of its two legs bends the curve.
    fn close_curve(&mut self, corners: &ShapedCorners, leg_out_mm: f64) {
        let Some(apex) = self.last_apex.take() else {
            return;
        };
        let leg = apex.leg_in_mm.min(leg_out_mm);
        self.cap_halves(
            apex.halves,
            corners.arc_speed_cap(leg / apex.theta, apex.room_mm),
        );
    }

    fn cap_curves(&mut self, id: u64, corners: &ShapedCorners) {
        for (held, pieces) in &mut self.held {
            if *held == id {
                pieces
                    .iter_mut()
                    .for_each(|piece| cap_curve(piece, corners));
            }
        }
    }

    fn cap_halves(&mut self, halves: [u64; 2], cap: f64) {
        for (id, pieces) in &mut self.held {
            if halves.contains(id) {
                pieces.iter_mut().for_each(|piece| cap_speed(piece, cap));
            }
        }
    }

    /// Releases every held move: the stream ends or drains here.
    pub(super) fn flush(&mut self, release: &mut impl FnMut(Move)) {
        if let (Some(id), Some(corners)) = (self.unsettled_clothoid, self.corners) {
            self.cap_curves(id, corners);
        }
        self.reset();
        for piece in self.held.drain(..).flat_map(|(_, pieces)| pieces) {
            release(piece);
        }
    }

    pub(super) fn clear(&mut self) {
        self.reset();
        self.held.clear();
    }

    fn reset(&mut self) {
        self.slowing_ahead = None;
        self.since_corner_mm = f64::INFINITY;
        self.last_apex = None;
        self.unsettled_clothoid = None;
    }

    pub(super) fn is_empty(&self) -> bool {
        self.held.is_empty()
    }
}

fn last(pieces: &[Move]) -> &Move {
    pieces.last().expect("a held move has a piece")
}

/// The acceleration a corner between `up` and `down` slows to, cut
/// `cut_mm` and turning `theta`, and the approach either side of it that
/// slows, where a stop there at the moves' acceleration rounds past the
/// budget; `None` where it does not, or where the cut alone spends the
/// budget (a run of facets the fitter replaced, held as a curve instead).
fn slowed_accel(
    up: &Move,
    down: &Move,
    theta: f64,
    cut_mm: f64,
    corners: &ShapedCorners,
) -> Option<(f64, f64)> {
    let (accel, budget) = corner_budget(up, down);
    let budget_mm = corners.budget_mm(budget, theta, accel);
    (cut_mm < budget_mm && corners.slows(budget, theta, cut_mm, accel)).then(|| {
        (
            corners.corner_accel(theta, cut_mm, accel, budget_mm),
            corners.approach_mm(accel),
        )
    })
}

/// The acceleration a corner between `up` and `down` is held at the most,
/// and its budget.
fn corner_budget(up: &Move, down: &Move) -> (f64, geometry::CornerBudget) {
    let accel = up.limits.accel_mm_s2.max(down.limits.accel_mm_s2);
    let budget = up.limits.corner.stricter(
        down.limits.corner,
        up.limits.accel_mm_s2.min(down.limits.accel_mm_s2),
    );
    (accel, budget)
}

fn stopped_corner_accel(
    before: &Move,
    after: &Move,
    corners: &ShapedCorners,
) -> Option<(f64, f64)> {
    let theta = stop_turn(before, after)?;
    slowed_accel(before, after, theta, 0.0, corners)
}

/// What the seam between the last held move and the arriving one is.
enum Verdict {
    None,
    Stop {
        turns: bool,
        slowed: Option<(f64, f64)>,
    },
    Blend(u64, BlendCorner),
}

/// A blend the fitter cut within its budget: a clothoid pair whose apex
/// leaves room in the budget at the moves' acceleration, and the
/// acceleration it slows to where its stop rounds past the budget.
struct BlendCorner {
    theta: f64,
    cut_mm: f64,
    budget_mm: f64,
    slowed: Option<(f64, f64)>,
}

impl BlendCorner {
    fn of(half1: &Move, half2: &Move, corners: &ShapedCorners) -> Option<Self> {
        let (theta, cut_mm) = blend_apex(half1, half2)?;
        let (accel, budget) = corner_budget(half1, half2);
        let budget_mm = corners.budget_mm(budget, theta, accel);
        (cut_mm < budget_mm).then(|| Self {
            theta,
            cut_mm,
            budget_mm,
            slowed: slowed_accel(half1, half2, theta, cut_mm, corners),
        })
    }
}

/// Caps a curve that is no corner blend at the speed that keeps the
/// shaper's pull on its tightest radius within its budget's room
/// ([`ShapedCorners::curve_room_mm`]).
fn cap_curve(m: &mut Move, corners: &ShapedCorners) {
    let Some(segment) = &m.segment.spatial else {
        return;
    };
    let (_, kappa_peak) = segment.kappa_peak();
    if kappa_peak.abs() > 0.0 {
        let room = corners.curve_room_mm(m.limits.corner, m.limits.accel_mm_s2);
        cap_speed(m, corners.arc_speed_cap(1.0 / kappa_peak.abs(), room));
    }
}

fn cap_speed(m: &mut Move, speed: f64) {
    m.feedrate_mm_s = m.feedrate_mm_s.min(speed);
}

fn path_length(m: &Move) -> f64 {
    m.segment
        .spatial
        .as_ref()
        .map_or(0.0, |segment| segment.s_len())
}

fn bake(m: &mut Move, accel: f64) {
    m.limits.accel_mm_s2 = m.limits.accel_mm_s2.min(accel);
}

/// Whether `m` is a line cut `at` mm along leaves no piece shorter than a
/// quarter of `approach_mm`.
fn splits(m: &Move, at: f64, approach_mm: f64) -> bool {
    let sliver = 0.25 * approach_mm;
    line_length(m).is_some_and(|length| at >= sliver && length - at >= sliver)
}

fn line_length(m: &Move) -> Option<f64> {
    match &m.segment.spatial {
        Some(Segment::Line(line)) => Some(line.s_len()),
        _ => None,
    }
}

/// `m`, a line, cut `at` mm along it into two collinear moves carrying its
/// limits, source and the share of its followers each covers.
fn split_at(m: &Move, at: f64) -> (Move, Move) {
    let Some(Segment::Line(line)) = &m.segment.spatial else {
        unreachable!("only a line is split")
    };
    let length = line.s_len();
    let piece = |from: f64, to: f64| {
        let segment = Line::try_new(line.point_at(from), line.point_at(to))
            .expect("a split line keeps a positive length");
        let followers = m
            .segment
            .followers
            .iter()
            .map(|f| f.span(from, to, length))
            .filter(|f| f.max_abs_ratio() > 0.0)
            .collect();
        Move {
            segment: PathSegment::try_new(Segment::Line(segment), followers)
                .expect("a split line's followers fit it"),
            ..m.clone()
        }
    };
    (piece(0.0, at), piece(at, length))
}
