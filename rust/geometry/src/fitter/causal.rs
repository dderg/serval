use crate::frontend::Move;
use crate::path::lowering::PositionProfile;
use crate::path::{Arc, CurvatureProfile, PathSegment, Segment};
use crate::segment::FollowerDemand;

use super::heart::Heart;
use super::kernels::{self, Reconstruction};
use super::vec3::dot;
use super::{
    ChainFitConfig, CornerFitConfig, FitError, FitOutcome, FitReport, JunctionPlan, UnblendReason,
    UnblendedJunction, blend_trim, classify_junction, emit_blend, emit_move, internal,
    junction_deviation, line_of,
};

struct Run {
    start: usize,
    end: usize,
    recon: Reconstruction,
}

pub(super) fn fit(
    moves: &[Move],
    config: ChainFitConfig,
    head_len_restore: f64,
) -> Result<FitOutcome, FitError> {
    if moves.len() <= 1 {
        return Ok(FitOutcome {
            moves: moves.to_vec(),
            report: FitReport::default(),
        });
    }

    let mut plans = Vec::with_capacity(moves.len() - 1);
    for (i, pair) in moves.windows(2).enumerate() {
        let restore = if i == 0 { head_len_restore } else { 0.0 };
        plans.push(classify_junction(
            &pair[0],
            &pair[1],
            config.corner,
            restore,
        )?);
    }

    let heart = config.heart.build();
    let runs = detect_runs(moves, config, heart.as_ref())?;

    let mut out = Vec::new();
    let mut report = FitReport::default();
    for (i, m) in moves.iter().enumerate() {
        match run_role(&runs, i) {
            RunRole::Interior => report.consumed_legs += 1,
            RunRole::Start(r) => {
                let trim_start = if i > 0 {
                    junction_trim(&runs, &plans, i - 1)
                } else {
                    0.0
                };
                if emit_move(&mut out, m, trim_start, r.recon.head_consumption)? {
                    report.consumed_legs += 1;
                }
                emit_reconstruction(&mut out, &r.recon, m, &moves[r.end])?;
                report.chains += 1;
            }
            RunRole::End(r) => {
                let trim_end = if i < plans.len() {
                    junction_trim(&runs, &plans, i)
                } else {
                    0.0
                };
                if emit_move(&mut out, m, r.recon.tail_consumption, trim_end)? {
                    report.consumed_legs += 1;
                }
            }
            RunRole::None => {
                let trim_start = if i > 0 {
                    junction_trim(&runs, &plans, i - 1)
                } else {
                    0.0
                };
                let trim_end = if i < plans.len() {
                    junction_trim(&runs, &plans, i)
                } else {
                    0.0
                };
                if emit_move(&mut out, m, trim_start, trim_end)? {
                    report.consumed_legs += 1;
                }
            }
        }

        if i < plans.len() && !junction_internal(&runs, i) {
            if let Some(reason) = run_boundary_unblend(&runs, moves, i, config.corner) {
                report.unblended.push(UnblendedJunction {
                    line_no: moves[i + 1].source.start_line,
                    reason,
                });
            } else if !run_boundary(&runs, i) {
                match &plans[i] {
                    JunctionPlan::Blend(bi) => {
                        report.blended += 1;
                        emit_blend(&mut out, bi, m, &moves[i + 1])?;
                    }
                    JunctionPlan::Unblended(reason) => report.unblended.push(UnblendedJunction {
                        line_no: moves[i + 1].source.start_line,
                        reason: *reason,
                    }),
                }
            }
        }
    }

    Ok(FitOutcome { moves: out, report })
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
    let scv_delta = moves
        .iter()
        .map(|m| junction_deviation(m.limits))
        .filter(|d| d.is_finite() && *d > 0.0)
        .fold(f64::INFINITY, f64::min);
    if !scv_delta.is_finite() {
        return Ok(Vec::new());
    }
    let tol = scv_delta;

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

fn junction_trim(runs: &[Run], plans: &[JunctionPlan], j: usize) -> f64 {
    if junction_internal(runs, j) {
        return 0.0;
    }
    for r in runs {
        if r.start > 0 && j == r.start - 1 {
            return r.recon.head_line_trim;
        }
        if j == r.end {
            return r.recon.tail_line_trim;
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

fn emit_reconstruction(
    out: &mut Vec<Move>,
    recon: &Reconstruction,
    m_in: &Move,
    m_out: &Move,
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
    push(
        Segment::Arc(recon.arc.clone()),
        m_in,
        recon.followers.clone(),
    )?;
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
