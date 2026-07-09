use crate::frontend::Move;
use crate::path::lowering::PositionProfile;
use crate::path::{Line, Segment};

use super::emit::{emit_blend, emit_consumption, emit_move};
use super::{FacetConsumption, FitError, JunctionBlend};

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
