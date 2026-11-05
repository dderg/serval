//! Slows the corners a stop at their acceleration would round past their
//! budget ([`ShapedCorners::slows`]). Each such corner's blend, and a
//! short approach split off the lines either side of it
//! ([`ShapedCorners::approach_mm`]), get the acceleration
//! [`ShapedCorners::corner_accel`] leaves it baked into their limits, so
//! only the path the shaper reads through the corner slows. A move leaves
//! here final: the window plans it as it will commit.

use std::collections::VecDeque;

use geometry::path::lowering::PositionProfile;
use geometry::path::{CurvatureProfile, Line, PathSegment, Segment};
use geometry::shaped_corner::{blend_apex, stop_turn};
use geometry::{CornerFitConfig, Move, ShapedCorners, seam_requires_stop};

/// The moves the stage holds back: a blend's verdict reaches the move
/// before its first half, which waits for the second.
const HELD_MOVES: usize = 2;

pub(super) struct Approaches {
    corners: Option<&'static ShapedCorners>,
    config: CornerFitConfig,
    held: VecDeque<Vec<Move>>,
    next_head_accel: Option<f64>,
}

impl Approaches {
    pub(super) fn new(config: CornerFitConfig) -> Self {
        Self {
            corners: config.shaped_corners,
            config,
            held: VecDeque::new(),
            next_head_accel: None,
        }
    }

    /// Takes `m` and releases every move no later corner can slow.
    pub(super) fn push(&mut self, m: Move, release: &mut impl FnMut(Move)) {
        let Some(corners) = self.corners else {
            release(m);
            return;
        };
        let mut pieces = vec![m];
        if let Some(accel) = self.next_head_accel.take() {
            slow_head(&mut pieces, corners, accel);
        }
        if let Some(previous) = self.held.back_mut() {
            let before = last(previous);
            if seam_requires_stop(before, &pieces[0], self.config) {
                if let Some(accel) = stopped_corner_accel(before, &pieces[0], corners) {
                    slow_tail(previous, corners, accel);
                    slow_head(&mut pieces, corners, accel);
                }
            } else if let Some(accel) = blend_corner_accel(before, &pieces[0], corners) {
                bake(last_mut(previous), accel);
                bake(&mut pieces[0], accel);
                if let Some(before_blend) = self.held.iter_mut().rev().nth(1) {
                    slow_tail(before_blend, corners, accel);
                }
                self.next_head_accel = Some(accel);
            }
        }
        self.held.push_back(pieces);
        while self.held.len() > HELD_MOVES {
            for piece in self.held.pop_front().expect("held") {
                release(piece);
            }
        }
    }

    /// Releases every held move: the stream ends or drains here.
    pub(super) fn flush(&mut self, release: &mut impl FnMut(Move)) {
        self.next_head_accel = None;
        for piece in self.held.drain(..).flatten() {
            release(piece);
        }
    }

    pub(super) fn clear(&mut self) {
        self.next_head_accel = None;
        self.held.clear();
    }

    pub(super) fn is_empty(&self) -> bool {
        self.held.is_empty()
    }
}

fn last(pieces: &[Move]) -> &Move {
    pieces.last().expect("a held move has a piece")
}

fn last_mut(pieces: &mut [Move]) -> &mut Move {
    pieces.last_mut().expect("a held move has a piece")
}

/// The acceleration a corner between `up` and `down` slows to, cut
/// `cut_mm` and turning `theta`, where a stop there at the moves'
/// acceleration rounds past the budget; `None` where it does not, or where
/// the cut alone spends the budget (a run of facets the fitter replaced,
/// held as a curve instead).
fn slowed_accel(
    up: &Move,
    down: &Move,
    theta: f64,
    cut_mm: f64,
    corners: &ShapedCorners,
) -> Option<f64> {
    let accel = up.limits.accel_mm_s2.max(down.limits.accel_mm_s2);
    let budget = up.limits.corner.stricter(
        down.limits.corner,
        up.limits.accel_mm_s2.min(down.limits.accel_mm_s2),
    );
    let budget_mm = corners.budget_mm(budget, theta, accel);
    (cut_mm < budget_mm && corners.slows(budget, theta, cut_mm, accel))
        .then(|| corners.corner_accel(theta, cut_mm, accel, budget_mm))
}

fn stopped_corner_accel(before: &Move, after: &Move, corners: &ShapedCorners) -> Option<f64> {
    let theta = stop_turn(before, after)?;
    slowed_accel(before, after, theta, 0.0, corners)
}

fn blend_corner_accel(half1: &Move, half2: &Move, corners: &ShapedCorners) -> Option<f64> {
    let (theta, cut) = blend_apex(half1, half2)?;
    slowed_accel(half1, half2, theta, cut, corners)
}

fn bake(m: &mut Move, accel: f64) {
    m.limits.accel_mm_s2 = m.limits.accel_mm_s2.min(accel);
}

/// Slows the last piece of `pieces`, split off its line first where the
/// line outlasts two approaches.
fn slow_tail(pieces: &mut Vec<Move>, corners: &ShapedCorners, accel: f64) {
    let m = pieces.pop().expect("a held move has a piece");
    let approach = corners.approach_mm(m.limits.accel_mm_s2);
    match line_length(&m).filter(|&length| length >= 2.0 * approach) {
        Some(length) => {
            let (body, mut tail) = split_at(&m, length - approach);
            bake(&mut tail, accel);
            pieces.extend([body, tail]);
        }
        None => {
            let mut m = m;
            bake(&mut m, accel);
            pieces.push(m);
        }
    }
}

/// Slows the first piece of `pieces`, split off its line first where the
/// line outlasts two approaches.
fn slow_head(pieces: &mut Vec<Move>, corners: &ShapedCorners, accel: f64) {
    let m = pieces.remove(0);
    let approach = corners.approach_mm(m.limits.accel_mm_s2);
    match line_length(&m).filter(|&length| length >= 2.0 * approach) {
        Some(_) => {
            let (mut head, body) = split_at(&m, approach);
            bake(&mut head, accel);
            pieces.splice(0..0, [head, body]);
        }
        None => {
            let mut m = m;
            bake(&mut m, accel);
            pieces.insert(0, m);
        }
    }
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
