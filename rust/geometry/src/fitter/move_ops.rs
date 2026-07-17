use crate::frontend::Move;
use crate::path::lowering::PositionProfile;
use crate::path::{CurvatureProfile, Line, PathSegment, Segment};
use crate::segment::{FollowerDemand, SourceRange};
use crate::vec3::{dist, dot, madd, norm_sq, sub};

use super::emit::{emit_blend, emit_consumption, emit_move};
use super::{CornerFitConfig, FacetConsumption, FitError, JunctionBlend};

/// The two clothoid-half moves a blend contributes between `m_in` and `m_out`.
pub fn blend_moves(
    blend: &JunctionBlend,
    m_in: &Move,
    m_out: &Move,
) -> Result<Vec<Move>, FitError> {
    let mut out = Vec::with_capacity(2);
    emit_blend(&mut out, &blend.0, m_in, m_out)?;
    Ok(out)
}

/// The G2 clothoid chain that replaces the consumed `mids` together with
/// `m_in`'s trimmed tail and `m_out`'s trimmed head.
pub fn consumption_moves(
    consumption: &FacetConsumption,
    m_in: &Move,
    mids: &[&Move],
    m_out: &Move,
) -> Result<Vec<Move>, FitError> {
    let mut out = Vec::new();
    emit_consumption(&mut out, consumption, m_in, mids, m_out)?;
    Ok(out)
}

/// A move's body with blend trims applied at either end. `None` when the trims
/// consume the whole move; non-line moves pass through untrimmed.
pub fn trim_line_move(m: &Move, trim_start: f64, trim_end: f64) -> Result<Option<Move>, FitError> {
    let mut out = Vec::with_capacity(1);
    let consumed = emit_move(&mut out, m, trim_start, trim_end)?;
    Ok(if consumed { None } else { out.pop() })
}

/// Where the move's spatial segment begins; `None` for non-spatial moves.
#[must_use]
pub fn spatial_start(m: &Move) -> Option<[f64; 3]> {
    m.segment.spatial.as_ref().map(|seg| seg.point_at(0.0))
}

/// Where the move's spatial segment ends; `None` for non-spatial moves.
#[must_use]
pub fn spatial_end(m: &Move) -> Option<[f64; 3]> {
    m.segment
        .spatial
        .as_ref()
        .map(|seg| seg.point_at(m.segment.s_len()))
}

/// A spatial line move with no follower demand (no extrusion) — the moves
/// `align_travels` is allowed to re-anchor onto their fitted neighbors.
#[must_use]
pub fn is_travel(m: &Move) -> bool {
    matches!(m.segment.spatial, Some(Segment::Line(_)))
        && !m
            .segment
            .followers
            .iter()
            .any(|f| f.max_abs_ratio() > 1e-12)
}

pub(super) fn line_of(m: &Move) -> Option<&Line> {
    match &m.segment.spatial {
        Some(Segment::Line(line)) => Some(line),
        _ => None,
    }
}

/// Junctions turning less than this merge instead of blending. Slicers break
/// gentle curves and width transitions into sub-degree facets; blending those
/// leaves micrometre-scale bodies between the blend halves, and lowering such
/// slivers rings the fitted acceleration far past the machine limits.
const MERGE_MAX_TURN_RAD: f64 = std::f64::consts::PI / 180.0;

/// Sub-degree facets usually exist *because* the slicer stepped the feedrate
/// or extrusion width there, so an exact-match gate would never merge
/// anything. Within this relative band the merged move takes the slower
/// feedrate and the length-weighted extrusion ratio.
const MERGE_FEEDRATE_REL_TOL: f64 = 0.1;

const MERGE_CONTIGUITY_EPS_MM: f64 = 1e-9;

/// One line covering `prev` then `next`, when the junction between them turns
/// less than [`MERGE_MAX_TURN_RAD`], their demands agree within the merge
/// bands, and every vertex the chord replaces — the junction plus `absorbed`,
/// the vertices already merged into `prev` — stays within the junction's
/// corner-deviation budget, the same budget a blend would have spent there.
/// `None` when any gate fails; the junction then blends or stays sharp as
/// usual.
pub fn merge_collinear_lines(
    prev: &Move,
    next: &Move,
    absorbed: &[[f64; 3]],
    config: CornerFitConfig,
) -> Option<Move> {
    let (line_prev, line_next) = (line_of(prev)?, line_of(next)?);
    if prev.limits != next.limits {
        return None;
    }
    let (f1, f2) = (prev.feedrate_mm_s, next.feedrate_mm_s);
    if (f1 - f2).abs() > MERGE_FEEDRATE_REL_TOL * f1.max(f2) {
        return None;
    }
    if dist(line_prev.end, line_next.start) > MERGE_CONTIGUITY_EPS_MM {
        return None;
    }
    let t_in = line_prev.heading_at(line_prev.s_len());
    let t_out = line_next.heading_at(0.0);
    let theta = libm::acos(dot(t_in, t_out).clamp(-1.0, 1.0));
    if theta > MERGE_MAX_TURN_RAD {
        return None;
    }

    let chord_len = dist(line_prev.start, line_next.end);
    let followers = merged_followers(prev, next, chord_len, config.extrusion_ramp_rel_tol)?;

    let budget = prev.limits.corner_deviation_mm;
    if !(budget.is_finite() && budget > 0.0) {
        return None;
    }
    let (start, end) = (line_prev.start, line_next.end);
    let within_budget = |v: &[f64; 3]| dist_to_segment(*v, start, end) <= budget;
    if !within_budget(&line_prev.end) || !absorbed.iter().all(within_budget) {
        return None;
    }

    let line = Line::try_new(start, end).ok()?;
    let segment = PathSegment::try_new(Segment::Line(line), followers).ok()?;
    Some(Move {
        segment,
        feedrate_mm_s: f1.min(f2),
        limits: prev.limits,
        source: SourceRange {
            start_line: prev.source.start_line,
            end_line: next.source.end_line,
        },
    })
}

/// The merged move's followers: both sides must demand the same shape — no
/// followers at all (travels), or one constant same-sign ratio on the same
/// axis within the extrusion ramp band — and the merge spreads the sides'
/// total extrusion over the chord, so the filament laid down is preserved.
fn merged_followers(
    prev: &Move,
    next: &Move,
    chord_len: f64,
    band: f64,
) -> Option<Vec<FollowerDemand>> {
    match (
        prev.segment.followers.as_slice(),
        next.segment.followers.as_slice(),
    ) {
        ([], []) => Some(Vec::new()),
        ([a], [b]) if a.axis_index == b.axis_index && !a.is_ramped() && !b.is_ramped() => {
            if a.ratio * b.ratio <= 0.0 {
                return None;
            }
            if (a.ratio - b.ratio).abs() > band * a.ratio.abs().max(b.ratio.abs()) {
                return None;
            }
            let e_total = a.ratio * prev.segment.s_len() + b.ratio * next.segment.s_len();
            Some(vec![FollowerDemand::constant(
                a.axis_index,
                e_total / chord_len,
            )])
        }
        _ => None,
    }
}

fn dist_to_segment(p: [f64; 3], s0: [f64; 3], s1: [f64; 3]) -> f64 {
    let d = sub(s1, s0);
    let len_sq = norm_sq(d);
    if len_sq <= 0.0 {
        return dist(p, s0);
    }
    let t = (dot(sub(p, s0), d) / len_sq).clamp(0.0, 1.0);
    dist(p, madd(s0, t, d))
}
