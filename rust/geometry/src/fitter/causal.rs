use crate::frontend::Move;
use crate::path::{Arc, CurvatureProfile, PathSegment, Segment};
use crate::segment::FollowerDemand;

use super::biclothoid::GeneralBlend;
use super::kernels::Reconstruction;
use super::{FitError, blend_followers, internal};

pub(super) fn trim_arc(arc: &Arc, head: f64, tail: f64) -> Result<Arc, crate::GeometryError> {
    if head <= 0.0 && tail <= 0.0 {
        return Ok(arc.clone());
    }
    let sign = arc.sweep.signum();
    let start_angle = arc.start_angle + sign * head / arc.radius;
    let sweep = arc.sweep - sign * (head + tail) / arc.radius;
    Arc::try_new(arc.origin, arc.u, arc.v, arc.radius, start_angle, sweep)
}

pub(super) fn emit_general_blend(
    out: &mut Vec<Move>,
    blend: &GeneralBlend,
    in_followers: &[FollowerDemand],
    out_followers: &[FollowerDemand],
    m_in: &Move,
    m_out: &Move,
) -> Result<(), FitError> {
    let len1 = blend.half1.s_len();
    let len2 = blend.half2.s_len();
    let (f_in, f_out) = blend_followers(
        in_followers,
        out_followers,
        blend.trim_in,
        len1,
        blend.trim_out,
        len2,
    );

    let seg_in = PathSegment::try_new(Segment::Clothoid(blend.half1.clone()), f_in)
        .map_err(internal(m_in.source.start_line))?;
    out.push(Move {
        segment: seg_in,
        feedrate_mm_s: m_in.feedrate_mm_s,
        limits: m_in.limits,
        source: m_in.source,
    });

    let seg_out = PathSegment::try_new(Segment::Clothoid(blend.half2.clone()), f_out)
        .map_err(internal(m_out.source.start_line))?;
    out.push(Move {
        segment: seg_out,
        feedrate_mm_s: m_out.feedrate_mm_s,
        limits: m_out.limits,
        source: m_out.source,
    });
    Ok(())
}

pub(super) fn emit_reconstruction(
    out: &mut Vec<Move>,
    recon: &Reconstruction,
    m_in: &Move,
    m_out: &Move,
    head_blend_trim: f64,
    tail_blend_trim: f64,
) -> Result<(), FitError> {
    let mut push =
        |spatial: Segment, src: &Move, followers: Vec<FollowerDemand>| -> Result<(), FitError> {
            let segment = PathSegment::try_new(spatial, followers)
                .map_err(internal(src.source.start_line))?;
            out.push(Move {
                segment,
                feedrate_mm_s: src.feedrate_mm_s,
                limits: src.limits,
                source: src.source,
            });
            Ok(())
        };
    for up in &recon.up {
        push(
            Segment::Clothoid(up.clone()),
            m_in,
            recon.up_followers.clone(),
        )?;
    }
    let arc_len = recon.arc.s_len();
    let remaining = arc_len - head_blend_trim - tail_blend_trim;
    assert!(
        remaining >= -1e-9 * arc_len.max(1.0),
        "fitter: blend trims exceed the arc at line {}: len={arc_len} head={head_blend_trim} tail={tail_blend_trim}",
        m_in.source.start_line
    );
    if remaining > crate::LENGTH_EPS_MM {
        let arc = trim_arc(&recon.arc, head_blend_trim, tail_blend_trim)
            .map_err(internal(m_in.source.start_line))?;
        push(Segment::Arc(arc), m_in, recon.followers.clone())?;
    }
    for down in &recon.down {
        push(
            Segment::Clothoid(down.clone()),
            m_out,
            recon.down_followers.clone(),
        )?;
    }
    Ok(())
}
