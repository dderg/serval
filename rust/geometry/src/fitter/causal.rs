// TODO: orphaned by the stream-planner rewire; heart config is still parsed but unread — pending decision
#![allow(dead_code)]

use crate::frontend::Move;
use crate::path::lowering::PositionProfile;
use crate::path::{Arc, CurvatureProfile, Line, PathSegment, Segment};
use crate::segment::FollowerDemand;

use super::BUDGET_EPS_MM;
use super::biclothoid::GeneralBlend;
use super::heart::Heart;
use super::kernels::{self, Reconstruction};
use super::overlap;
use super::vec3::{dist, dot};
use super::{
    ChainFitConfig, CornerFitConfig, FitError, FitOutcome, FitReport, JunctionPlan, UnblendReason,
    UnblendedJunction, blend_trim, classify_junction, emit_blend, emit_move, internal, is_travel,
    line_of, scaled_followers,
};

struct Run {
    start: usize,
    end: usize,
    recon: Reconstruction,
    head_blend_trim: f64,
    tail_blend_trim: f64,
}

pub(super) fn fit(moves: &[Move], config: ChainFitConfig) -> Result<FitOutcome, FitError> {
    if moves.len() <= 1 {
        return Ok(FitOutcome {
            moves: moves.to_vec(),
            report: FitReport::default(),
        });
    }

    let heart = config.heart.build();
    let mut runs = detect_runs(moves, config, heart.as_ref())?;

    let mut in_reductions = vec![0.0_f64; moves.len().saturating_sub(1)];
    let mut out_reductions = vec![0.0_f64; moves.len().saturating_sub(1)];
    for r in &runs {
        if r.start >= 2 {
            out_reductions[r.start - 2] = r.recon.head_line_trim;
        }
        if r.end + 1 < in_reductions.len() {
            in_reductions[r.end + 1] = r.recon.tail_line_trim;
        }
    }

    let mut plans = Vec::with_capacity(moves.len() - 1);
    for (i, pair) in moves.windows(2).enumerate() {
        plans.push(classify_junction(
            &pair[0],
            &pair[1],
            config.corner,
            in_reductions[i],
            out_reductions[i],
        )?);
    }

    let (arc_blends, line_blend_trims) = resolve_run_boundaries(&mut runs, moves, config.corner);

    let mut out = Vec::new();
    let mut report = FitReport::default();
    for (i, m) in moves.iter().enumerate() {
        match run_role(&runs, i) {
            RunRole::Interior => report.consumed_legs += 1,
            RunRole::Start(r) => {
                let trim_start = if i > 0 {
                    junction_trim(&runs, &plans, &line_blend_trims, i - 1)
                } else {
                    0.0
                };
                if emit_move(&mut out, m, trim_start, r.recon.head_consumption)? {
                    report.consumed_legs += 1;
                }
                emit_reconstruction(
                    &mut out,
                    &r.recon,
                    m,
                    &moves[r.end],
                    r.head_blend_trim,
                    r.tail_blend_trim,
                )?;
                report.chains += 1;
            }
            RunRole::End(r) => {
                let trim_end = if i < plans.len() {
                    junction_trim(&runs, &plans, &line_blend_trims, i)
                } else {
                    0.0
                };
                if emit_move(&mut out, m, r.recon.tail_consumption, trim_end)? {
                    report.consumed_legs += 1;
                }
            }
            RunRole::None => {
                let trim_start = if i > 0 {
                    junction_trim(&runs, &plans, &line_blend_trims, i - 1)
                } else {
                    0.0
                };
                let trim_end = if i < plans.len() {
                    junction_trim(&runs, &plans, &line_blend_trims, i)
                } else {
                    0.0
                };
                if emit_move(&mut out, m, trim_start, trim_end)? {
                    report.consumed_legs += 1;
                }
            }
        }

        if i < plans.len() && !junction_internal(&runs, i) {
            if let Some(blend) = &arc_blends[i] {
                let in_followers = runs
                    .iter()
                    .find(|r| r.end == i)
                    .map(|r| &r.recon.followers)
                    .unwrap_or(&m.segment.followers);
                let out_followers = runs
                    .iter()
                    .find(|r| r.start == i + 1)
                    .map(|r| &r.recon.followers)
                    .unwrap_or(&moves[i + 1].segment.followers);
                report.blended += 1;
                emit_general_blend(
                    &mut out,
                    blend,
                    in_followers,
                    out_followers,
                    m,
                    &moves[i + 1],
                )?;
            } else if let Some(reason) = run_boundary_unblend(&runs, moves, i, config.corner) {
                report.unblended.push(UnblendedJunction {
                    line_no: moves[i + 1].source.start_line,
                    reason,
                });
            } else if !run_boundary(&runs, i) {
                match &plans[i] {
                    JunctionPlan::Blend(bi) => {
                        report.blended += 1;
                        emit_blend(&mut out, &bi.0, m, &moves[i + 1])?;
                    }
                    JunctionPlan::Unblended(reason) => report.unblended.push(UnblendedJunction {
                        line_no: moves[i + 1].source.start_line,
                        reason: *reason,
                    }),
                }
            }
        }
    }

    let out = align_travels(out)?;
    Ok(FitOutcome { moves: out, report })
}

fn align_travels(mut out: Vec<Move>) -> Result<Vec<Move>, FitError> {
    let n = out.len();
    for i in 0..n {
        if !is_travel(&out[i]) {
            continue;
        }
        let Some(Segment::Line(line)) = &out[i].segment.spatial else {
            continue;
        };
        let line = line.clone();
        let prev_end = (0..i).rev().find_map(|k| {
            out[k]
                .segment
                .spatial
                .as_ref()
                .map(|s| s.point_at(s.s_len()))
        });
        let next_start =
            (i + 1..n).find_map(|k| out[k].segment.spatial.as_ref().map(|s| s.point_at(0.0)));
        let a = prev_end.unwrap_or(line.start);
        let b = next_start.unwrap_or(line.point_at(line.s_len()));
        if dist(a, line.start) <= BUDGET_EPS_MM
            && dist(b, line.point_at(line.s_len())) <= BUDGET_EPS_MM
        {
            continue;
        }
        let new_line = Line::try_new(a, b).map_err(internal(out[i].source.start_line))?;
        out[i].segment =
            PathSegment::try_new(Segment::Line(new_line), out[i].segment.followers.clone())
                .map_err(internal(out[i].source.start_line))?;
    }
    Ok(out)
}

fn detect_runs(
    moves: &[Move],
    config: ChainFitConfig,
    heart: &dyn Heart,
) -> Result<Vec<Run>, FitError> {
    let Some(arc) = config.arc_fit else {
        return Ok(Vec::new());
    };
    let min_run = (arc.min_run_facets.max(3)) as usize;
    let tol = super::span_tolerance(moves);
    if !tol.is_finite() {
        return Ok(Vec::new());
    }

    let mut runs = Vec::new();
    let n = moves.len();
    let mut i = 0;
    while i < n {
        if line_of(&moves[i]).is_none() {
            i += 1;
            continue;
        }
        let mut j = i;
        while j + 1 < n && line_of(&moves[j + 1]).is_some() {
            j += 1;
        }
        runs.extend(chain_runs(moves, i, j, heart, tol, min_run, config.corner)?);
        i = j + 1;
    }
    Ok(runs)
}

fn chain_runs(
    moves: &[Move],
    start: usize,
    end: usize,
    heart: &dyn Heart,
    tol: f64,
    min_run: usize,
    corner: CornerFitConfig,
) -> Result<Vec<Run>, FitError> {
    let chain = &moves[start..=end];
    let chain_len = chain.len();
    let spans = heart.arc_spans(chain, tol, min_run, corner);

    let mut occupied = vec![false; chain_len];
    for &(a, b) in &spans {
        for slot in occupied.iter_mut().take(b + 1).skip(a) {
            *slot = true;
        }
    }

    let mut runs = Vec::new();
    for (a, b) in spans {
        let (gs, ge) = (start + a, start + b);
        let facets = &moves[gs..=ge];
        let Some(mut recon) = kernels::reconstruct(facets, tol)? else {
            continue;
        };
        let head = (a > 0 && !occupied[a - 1])
            .then(|| kernels::neighbor(&chain[a - 1], true))
            .flatten();
        let tail = (b + 1 < chain_len && !occupied[b + 1])
            .then(|| kernels::neighbor(&chain[b + 1], false))
            .flatten();
        kernels::ease_run(&mut recon, facets, head.as_ref(), tail.as_ref(), tol)?;
        runs.push(Run {
            start: gs,
            end: ge,
            recon,
            head_blend_trim: 0.0,
            tail_blend_trim: 0.0,
        });
    }
    Ok(runs)
}

enum RunRole<'a> {
    None,
    Start(&'a Run),
    Interior,
    End(&'a Run),
}

fn run_role(runs: &[Run], i: usize) -> RunRole<'_> {
    for r in runs {
        if i == r.start {
            return RunRole::Start(r);
        }
        if i == r.end {
            return RunRole::End(r);
        }
        if i > r.start && i < r.end {
            return RunRole::Interior;
        }
    }
    RunRole::None
}

fn junction_internal(runs: &[Run], k: usize) -> bool {
    runs.iter().any(|r| r.start <= k && k < r.end)
}

fn run_boundary(runs: &[Run], k: usize) -> bool {
    runs.iter()
        .any(|r| k == r.end || (r.start > 0 && k == r.start - 1))
}

fn junction_trim(runs: &[Run], plans: &[JunctionPlan], line_blend_trims: &[f64], j: usize) -> f64 {
    if junction_internal(runs, j) {
        return 0.0;
    }
    for r in runs {
        if r.start > 0 && j == r.start - 1 {
            return r.recon.head_line_trim + line_blend_trims[j];
        }
        if j == r.end {
            return r.recon.tail_line_trim + line_blend_trims[j];
        }
    }
    blend_trim(&plans[j])
}

fn run_boundary_unblend(
    runs: &[Run],
    moves: &[Move],
    j: usize,
    config: CornerFitConfig,
) -> Option<UnblendReason> {
    for r in runs {
        if r.start > 0 && j == r.start - 1 {
            return r
                .recon
                .up
                .is_empty()
                .then(|| arc_boundary_unblend(&moves[j], &r.recon.arc, true, config))
                .flatten();
        }
        if j == r.end {
            return r
                .recon
                .down
                .is_empty()
                .then(|| arc_boundary_unblend(&moves[j + 1], &r.recon.arc, false, config))
                .flatten();
        }
    }
    None
}

fn arc_boundary_unblend(
    neighbor: &Move,
    arc: &Arc,
    head: bool,
    config: CornerFitConfig,
) -> Option<UnblendReason> {
    let Some(spatial) = &neighbor.segment.spatial else {
        return Some(UnblendReason::NonSpatial);
    };
    let Segment::Line(line) = spatial else {
        return Some(UnblendReason::ArcIncident);
    };
    let line_t = if head {
        line.heading_at(line.s_len())
    } else {
        line.heading_at(0.0)
    };
    let arc_t = if head {
        arc.heading_at(0.0)
    } else {
        arc.heading_at(arc.s_len())
    };
    let theta = dot(line_t, arc_t).clamp(-1.0, 1.0).acos();
    (theta > config.theta_min_rad).then_some(UnblendReason::ArcIncident)
}

fn bare_line<'a>(moves: &'a [Move], runs: &[Run], idx: usize) -> Option<&'a Line> {
    if idx >= moves.len() || runs.iter().any(|r| r.start <= idx && idx <= r.end) {
        return None;
    }
    match &moves[idx].segment.spatial {
        Some(Segment::Line(line)) => Some(line),
        _ => None,
    }
}

fn resolve_run_boundaries(
    runs: &mut [Run],
    moves: &[Move],
    corner: CornerFitConfig,
) -> (Vec<Option<GeneralBlend>>, Vec<f64>) {
    let n = moves.len().saturating_sub(1);
    let mut blends: Vec<Option<GeneralBlend>> = (0..n).map(|_| None).collect();
    let mut line_trims = vec![0.0; n];
    let delta = super::span_tolerance(moves);
    if !(delta.is_finite() && delta > 0.0) {
        return (blends, line_trims);
    }

    for k in 0..runs.len() {
        let j = runs[k].end;
        if j < n && runs[k].recon.down.is_empty() {
            if k + 1 < runs.len() && runs[k + 1].start == j + 1 {
                if runs[k + 1].recon.up.is_empty() {
                    if let Some(blend) = overlap::resolve_arc_arc(
                        &runs[k].recon.arc,
                        &runs[k + 1].recon.arc,
                        corner,
                        delta,
                    ) {
                        runs[k].tail_blend_trim = blend.trim_in;
                        runs[k + 1].head_blend_trim = blend.trim_out;
                        blends[j] = Some(blend);
                    }
                }
            } else if let Some(line) = bare_line(moves, runs, j + 1) {
                if let Some(blend) =
                    overlap::resolve_arc_line(&runs[k].recon.arc, line, true, corner, delta)
                {
                    runs[k].tail_blend_trim = blend.trim_in;
                    line_trims[j] = blend.trim_out;
                    blends[j] = Some(blend);
                }
            }
        }

        if runs[k].start > 0 && runs[k].recon.up.is_empty() {
            let jh = runs[k].start - 1;
            if let Some(line) = bare_line(moves, runs, jh) {
                if let Some(blend) =
                    overlap::resolve_arc_line(&runs[k].recon.arc, line, false, corner, delta)
                {
                    line_trims[jh] = blend.trim_in;
                    runs[k].head_blend_trim = blend.trim_out;
                    blends[jh] = Some(blend);
                }
            }
        }
    }
    (blends, line_trims)
}

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
    let f_in = scaled_followers(in_followers, blend.trim_in / len1);
    let f_out = scaled_followers(out_followers, blend.trim_out / len2);

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
    let arc = trim_arc(&recon.arc, head_blend_trim, tail_blend_trim)
        .map_err(internal(m_in.source.start_line))?;
    push(Segment::Arc(arc), m_in, recon.followers.clone())?;
    for down in &recon.down {
        push(
            Segment::Clothoid(down.clone()),
            m_out,
            recon.down_followers.clone(),
        )?;
    }
    Ok(())
}

#[cfg(test)]
mod tests;
