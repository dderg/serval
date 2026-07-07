use crate::GeometryError;
use crate::frontend::Move;
use crate::path::lowering::PositionProfile;
use crate::path::{CurvatureProfile, Line, PathSegment, Segment};
use crate::segment::FollowerDemand;
use crate::vec3::madd;

use super::move_ops::line_of;
use super::{FitError, biclothoid, biclothoid_followers};

pub(super) const BUDGET_EPS_MM: f64 = 1e-9;
/// Over-trim beyond this is a real overlap of two neighbors' claims, not
/// floating-point noise — the same order as the pipeline's position-contiguity
/// tolerance at ingress.
const OVER_TRIM_TOL_MM: f64 = 1e-6;

pub(super) fn emit_move(
    out: &mut Vec<Move>,
    m: &Move,
    trim_start: f64,
    trim_end: f64,
) -> Result<bool, FitError> {
    let Some(Segment::Line(line)) = &m.segment.spatial else {
        out.push(m.clone());
        return Ok(false);
    };
    if trim_start <= 0.0 && trim_end <= 0.0 {
        out.push(m.clone());
        return Ok(false);
    }

    let line_no = m.source.start_line;
    let heading = line.heading_at(0.0);
    let new_len = line.s_len() - trim_start - trim_end;
    if new_len < -OVER_TRIM_TOL_MM {
        return Err(FitError::OverTrimmedLine {
            line_no,
            excess_mm: -new_len,
        });
    }
    if new_len <= BUDGET_EPS_MM {
        return Ok(true);
    }

    let new_start = madd(line.start, trim_start, heading);
    let new_end = madd(line.start, trim_start + new_len, heading);
    let trimmed = Line::try_new(new_start, new_end).map_err(internal(line_no))?;
    let followers = m
        .segment
        .followers
        .iter()
        .map(|f| f.span(trim_start, trim_start + new_len, line.s_len()))
        .filter(|f| f.max_abs_ratio() > 0.0)
        .collect();
    let segment =
        PathSegment::try_new(Segment::Line(trimmed), followers).map_err(internal(line_no))?;
    out.push(Move {
        segment,
        feedrate_mm_s: m.feedrate_mm_s,
        limits: m.limits,
        source: m.source,
    });
    Ok(false)
}

pub(super) fn emit_blend(
    out: &mut Vec<Move>,
    bi: &biclothoid::Biclothoid,
    m_in: &Move,
    m_out: &Move,
) -> Result<(), FitError> {
    let (line_in, line_out) = match (line_of(m_in), line_of(m_out)) {
        (Some(a), Some(b)) => (a, b),
        _ => unreachable!("biclothoid blends are only planned between lines"),
    };
    let (f_in, f_out) = biclothoid_followers(bi, m_in, m_out, line_in, line_out);

    let seg_in = PathSegment::try_new(Segment::Clothoid(bi.half1.clone()), f_in)
        .map_err(internal(m_in.source.start_line))?;
    out.push(Move {
        segment: seg_in,
        feedrate_mm_s: m_in.feedrate_mm_s,
        limits: m_in.limits,
        source: m_in.source,
    });

    let seg_out = PathSegment::try_new(Segment::Clothoid(bi.half2.clone()), f_out)
        .map_err(internal(m_out.source.start_line))?;
    out.push(Move {
        segment: seg_out,
        feedrate_mm_s: m_out.feedrate_mm_s,
        limits: m_out.limits,
        source: m_out.source,
    });
    Ok(())
}

/// The ratio a demand carries for `axis`, or 0 when the axis has no follower.
fn ratio_start_for(followers: &[FollowerDemand], axis: usize) -> f64 {
    followers
        .iter()
        .find(|f| f.axis_index == axis)
        .map_or(0.0, |f| f.ratio)
}

fn ratio_end_for(followers: &[FollowerDemand], axis: usize) -> f64 {
    followers
        .iter()
        .find(|f| f.axis_index == axis)
        .map_or(0.0, |f| f.ratio_end)
}

fn follower_axes(a: &[FollowerDemand], b: &[FollowerDemand]) -> Vec<usize> {
    let mut axes: Vec<usize> = Vec::new();
    for f in a.iter().chain(b) {
        if !axes.contains(&f.axis_index) {
            axes.push(f.axis_index);
        }
    }
    axes
}

/// Whether two extruding lines' ratios differ enough to leave the corner
/// unblended. Only axes that extrude on *both* sides gate; a side with ratio 0
/// (travel) is exempt, so travel↔extrude corners still blend and ramp to zero.
pub(super) fn extrusion_step(
    in_followers: &[FollowerDemand],
    out_followers: &[FollowerDemand],
    rel_tol: f64,
) -> bool {
    follower_axes(in_followers, out_followers)
        .into_iter()
        .any(|axis| {
            let r_in = ratio_end_for(in_followers, axis);
            let r_out = ratio_start_for(out_followers, axis);
            r_in != 0.0
                && r_out != 0.0
                && (r_out - r_in).abs() > rel_tol * r_in.abs().max(r_out.abs())
        })
}

/// One side of a blend as the follower solver sees it: the neighbor's
/// demands over its full segment, and how much of that segment's tail (for
/// the inbound side) or head (outbound) the blend replaces.
pub(super) struct SeamSide<'a> {
    pub followers: &'a [FollowerDemand],
    pub seg_len: f64,
    pub trim: f64,
}

impl SeamSide<'_> {
    fn demand(&self, axis: usize) -> Option<&FollowerDemand> {
        self.followers.iter().find(|f| f.axis_index == axis)
    }

    /// Ratio at the post-trim end seam, and the E the trimmed tail carried.
    fn exit_anchor(&self, axis: usize) -> (f64, f64) {
        let Some(d) = self.demand(axis) else {
            return (0.0, 0.0);
        };
        let seam = self.seg_len - self.trim;
        let e = d.offset_at(self.seg_len, self.seg_len) - d.offset_at(seam, self.seg_len);
        (d.ratio_at(seam, self.seg_len), e)
    }

    /// Ratio at the post-trim start seam, and the E the trimmed head carried.
    fn entry_anchor(&self, axis: usize) -> (f64, f64) {
        let Some(d) = self.demand(axis) else {
            return (0.0, 0.0);
        };
        (
            d.ratio_at(self.trim, self.seg_len),
            d.offset_at(self.trim, self.seg_len),
        )
    }
}

/// Split a blend's inbound/outbound demands into the two blend halves as a
/// pair of linear ratio ramps. The blend's endpoints anchor at the neighbors'
/// ratios *at the trimmed seams* — for a constant neighbor that is its one
/// ratio, for a ramped neighbor (an arc-run reconstruction) the ratio its
/// window now starts or ends with — so `ė = r·v` is continuous where the
/// blend meets the trimmed neighbors. The shared midpoint ratio is then
/// whatever conserves the E the trimmed material carried across the halves'
/// actual arc lengths. Anchoring at rescaled ratios instead (the trimmed E
/// spread uniformly over the shorter half) would conserve E too, but it steps
/// the extrusion rate by `trim/len` at both outer seams — the very
/// discontinuity the ramp exists to remove.
pub(super) fn blend_followers(
    inbound: &SeamSide,
    outbound: &SeamSide,
    len1: f64,
    len2: f64,
) -> (Vec<FollowerDemand>, Vec<FollowerDemand>) {
    let axes = follower_axes(inbound.followers, outbound.followers);
    let mut half1 = Vec::with_capacity(axes.len());
    let mut half2 = Vec::with_capacity(axes.len());
    for axis in axes {
        let (r_in, e_in) = inbound.exit_anchor(axis);
        let (r_out, e_out) = outbound.entry_anchor(axis);
        let e_target = e_in + e_out;
        let r_mid = (2.0 * e_target - r_in * len1 - r_out * len2) / (len1 + len2);
        let a = FollowerDemand::ramp(axis, r_in, r_mid);
        let b = FollowerDemand::ramp(axis, r_mid, r_out);
        if a.max_abs_ratio() > 0.0 {
            half1.push(a);
        }
        if b.max_abs_ratio() > 0.0 {
            half2.push(b);
        }
    }
    (half1, half2)
}

pub(super) fn internal(line_no: u32) -> impl Fn(GeometryError) -> FitError {
    move |source| FitError::Internal { line_no, source }
}
