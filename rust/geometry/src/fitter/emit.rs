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
    bi: &biclothoid::GeneralBlend,
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

/// The clothoid segments replacing a consumed facet chain and its neighbors'
/// trimmed ends. Boundary segments carry the neighbor they seam into
/// (source, limits, feedrate capped by it); interior segments carry the
/// consumed span. Every segment's feedrate is capped at the consumed facets'
/// minimum — the G-code asked for those speeds over the region the blend now
/// covers.
pub(super) fn emit_consumption(
    out: &mut Vec<Move>,
    bi: &super::FacetConsumption,
    m_in: &Move,
    mids: &[&Move],
    m_out: &Move,
) -> Result<(), FitError> {
    let (line_in, line_out) = match (line_of(m_in), line_of(m_out)) {
        (Some(a), Some(b)) => (a, b),
        _ => unreachable!("consumption is only planned between lines"),
    };
    let followers = super::consumption_followers(&bi.0, m_in, mids, m_out, line_in, line_out);
    let feedrates = super::consumption_feedrates(&bi.0, m_in, mids, m_out);
    let limits = super::consumption_limits(&bi.0, m_in, mids, m_out);
    let interior_source = crate::segment::SourceRange {
        start_line: mids[0].source.start_line,
        end_line: mids[mids.len() - 1].source.end_line,
    };
    let last = bi.0.segments.len() - 1;

    for (i, (((seg, f), feed), lims)) in
        bi.0.segments
            .iter()
            .zip(followers)
            .zip(feedrates)
            .zip(limits)
            .enumerate()
    {
        let source = match i {
            0 => m_in.source,
            i if i == last => m_out.source,
            _ => interior_source,
        };
        let segment = PathSegment::try_new(Segment::Clothoid(seg.clone()), f)
            .map_err(internal(source.start_line))?;
        out.push(Move {
            segment,
            feedrate_mm_s: feed,
            limits: lims,
            source,
        });
    }
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

/// Whether every consumed move's demands fit inside the ramp the anchors
/// span: per axis, each facet's ratio endpoints must lie within the band
/// between the inbound side's exit ratio and the outbound side's entry
/// ratio, widened by `rel_tol`. A travel facet (ratio 0) between two
/// extruding anchors falls outside the band and stays a sharp boundary, as
/// does an extruding facet between travels.
pub(super) fn ratios_within_ramp_band(
    in_followers: &[FollowerDemand],
    consumed: &[&[FollowerDemand]],
    out_followers: &[FollowerDemand],
    rel_tol: f64,
) -> bool {
    let mut axes = follower_axes(in_followers, out_followers);
    for f in consumed.iter().flat_map(|c| c.iter()) {
        if !axes.contains(&f.axis_index) {
            axes.push(f.axis_index);
        }
    }
    axes.into_iter().all(|axis| {
        let r_in = ratio_end_for(in_followers, axis);
        let r_out = ratio_start_for(out_followers, axis);
        let tol = rel_tol * r_in.abs().max(r_out.abs());
        let lo = r_in.min(r_out) - tol;
        let hi = r_in.max(r_out) + tol;
        consumed.iter().all(|facet| {
            let (r0, r1) = facet
                .iter()
                .find(|f| f.axis_index == axis)
                .map_or((0.0, 0.0), |f| (f.ratio, f.ratio_end));
            (lo..=hi).contains(&r0) && (lo..=hi).contains(&r1)
        })
    })
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
    let mut per_seg = chain_blend_followers(inbound, &[], outbound, &[len1, len2]);
    let half2 = per_seg.pop().expect("two segments in");
    let half1 = per_seg.pop().expect("two segments in");
    (half1, half2)
}

/// [`blend_followers`] over a chain of blend segments that may also replace
/// whole moves between its two seams: the consumed demands' full E joins the
/// conservation target, so the material the swallowed moves would have
/// extruded still leaves the nozzle across the blend. The ratio profile over
/// the chain is a tent — linear from the inbound anchor to a peak at the
/// segment boundary nearest the chain's arclength middle, then linear to the
/// outbound anchor — sliced at every segment boundary, so each segment
/// carries a linear ramp and the whole is rate-continuous.
pub(super) fn chain_blend_followers(
    inbound: &SeamSide,
    consumed: &[(&[FollowerDemand], f64)],
    outbound: &SeamSide,
    seg_lens: &[f64],
) -> Vec<Vec<FollowerDemand>> {
    let mut bounds = Vec::with_capacity(seg_lens.len() + 1);
    bounds.push(0.0);
    for len in seg_lens {
        bounds.push(bounds.last().expect("non-empty") + len);
    }
    let total = *bounds.last().expect("non-empty");
    let peak_idx = (1..seg_lens.len())
        .min_by(|a, b| {
            (bounds[*a] - 0.5 * total)
                .abs()
                .total_cmp(&(bounds[*b] - 0.5 * total).abs())
        })
        .expect("at least two segments");
    let (len1, len2) = (bounds[peak_idx], total - bounds[peak_idx]);

    let mut axes = follower_axes(inbound.followers, outbound.followers);
    for f in consumed.iter().flat_map(|(c, _)| c.iter()) {
        if !axes.contains(&f.axis_index) {
            axes.push(f.axis_index);
        }
    }

    let mut out = vec![Vec::with_capacity(axes.len()); seg_lens.len()];
    for axis in axes {
        let (r_in, e_in) = inbound.exit_anchor(axis);
        let (r_out, e_out) = outbound.entry_anchor(axis);
        let e_consumed: f64 = consumed
            .iter()
            .filter_map(|(c, len)| {
                c.iter()
                    .find(|f| f.axis_index == axis)
                    .map(|d| d.offset_at(*len, *len))
            })
            .sum();
        let e_target = e_in + e_consumed + e_out;
        let r_peak = (2.0 * e_target - r_in * len1 - r_out * len2) / (len1 + len2);
        let ratio_at = |s: f64| {
            if s <= bounds[peak_idx] {
                r_in + (r_peak - r_in) * s / len1
            } else {
                r_peak + (r_out - r_peak) * (s - bounds[peak_idx]) / len2
            }
        };
        for (i, demands) in out.iter_mut().enumerate() {
            let d = FollowerDemand::ramp(axis, ratio_at(bounds[i]), ratio_at(bounds[i + 1]));
            if d.max_abs_ratio() > 0.0 {
                demands.push(d);
            }
        }
    }
    out
}

pub(super) fn internal(line_no: u32) -> impl Fn(GeometryError) -> FitError {
    move |source| FitError::Internal { line_no, source }
}
