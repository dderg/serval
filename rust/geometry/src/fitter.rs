mod biclothoid;
mod causal;
mod heart;
mod kernels;
mod overlap;
mod vec3;

pub use heart::HeartKind;

use std::f64::consts::{PI, SQRT_2};

use crate::GeometryError;
use crate::frontend::{Move, VelocityLimits};
use crate::path::lowering::PositionProfile;
use crate::path::{CurvatureProfile, Line, PathSegment, Segment};
use crate::segment::FollowerDemand;

const COLLINEAR_EPS_RAD: f64 = 1e-3;
const BUDGET_EPS_MM: f64 = 1e-9;
const TURN_NORMAL_EPS: f64 = 1e-9;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CornerFitConfig {
    pub theta_min_rad: f64,
    pub theta_max_rad: f64,
}

impl Default for CornerFitConfig {
    fn default() -> Self {
        Self {
            theta_min_rad: COLLINEAR_EPS_RAD,
            theta_max_rad: PI - COLLINEAR_EPS_RAD,
        }
    }
}

const ARC_MIN_RUN_FACETS: u32 = 3;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ArcFitConfig {
    pub min_run_facets: u32,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ChainFitConfig {
    pub corner: CornerFitConfig,
    pub arc_fit: Option<ArcFitConfig>,
    pub heart: HeartKind,
}

impl Default for ChainFitConfig {
    fn default() -> Self {
        Self {
            corner: CornerFitConfig::default(),
            arc_fit: None,
            heart: HeartKind::default(),
        }
    }
}

impl ChainFitConfig {
    pub fn with_arc_fit(min_run_facets: u32) -> Self {
        Self {
            arc_fit: Some(ArcFitConfig { min_run_facets }),
            ..Self::default()
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UnblendReason {
    Collinear,
    NearReversal,
    ZeroDeviation,
    NoBudget,
    ArcIncident,
    NonSpatial,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct UnblendedJunction {
    pub line_no: u32,
    pub reason: UnblendReason,
}

#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct FitReport {
    pub blended: u32,
    pub unblended: Vec<UnblendedJunction>,
    pub consumed_legs: u32,
    pub chains: u32,
}

#[derive(Debug, Clone, PartialEq)]
pub struct FitOutcome {
    pub moves: Vec<Move>,
    pub report: FitReport,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FitError {
    Internal { line_no: u32, source: GeometryError },
}

enum JunctionPlan {
    Blend(biclothoid::Biclothoid),
    Unblended(UnblendReason),
}

pub fn fit_corners(moves: &[Move], config: CornerFitConfig) -> Result<FitOutcome, FitError> {
    if moves.len() <= 1 {
        return Ok(FitOutcome {
            moves: moves.to_vec(),
            report: FitReport::default(),
        });
    }

    let mut plans = Vec::with_capacity(moves.len() - 1);
    for pair in moves.windows(2) {
        plans.push(classify_junction(
            &pair[0], &pair[1], config, 0.0, 0.0, 0.0,
        )?);
    }

    let mut out = Vec::new();
    let mut report = FitReport::default();
    for (i, m) in moves.iter().enumerate() {
        let trim_start = if i > 0 {
            blend_trim(&plans[i - 1])
        } else {
            0.0
        };
        let trim_end = if i < plans.len() {
            blend_trim(&plans[i])
        } else {
            0.0
        };
        if emit_move(&mut out, m, trim_start, trim_end)? {
            report.consumed_legs += 1;
        }

        if i < plans.len() {
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

    Ok(FitOutcome { moves: out, report })
}

pub fn fit_chain(moves: &[Move], config: ChainFitConfig) -> Result<FitOutcome, FitError> {
    fit_chain_with_head_restore(moves, config, 0.0)
}

/// Streaming variant of [`fit_chain`]. `head_len_restore` is the spatial length
/// already consumed from the head move's front by the blend committed at the
/// previous seam. It is added back into the leading junction's blend budget so a
/// corner re-fits to the same curvature it had before that commit trimmed the
/// head — otherwise the shorter head move yields a smaller budget, a sharper
/// apex, and a corner cap below the already-committed entry velocity (an abort).
/// A fresh fit passes `0.0` and is byte-identical to [`fit_chain`]. See
/// docs/rewrite/windowed-fit-ceiling-jitter.md.
pub fn fit_chain_with_head_restore(
    moves: &[Move],
    config: ChainFitConfig,
    head_len_restore: f64,
) -> Result<FitOutcome, FitError> {
    causal::fit(moves, config, head_len_restore)
}

fn classify_junction(
    m_in: &Move,
    m_out: &Move,
    config: CornerFitConfig,
    head_len_restore: f64,
    in_reduction: f64,
    out_reduction: f64,
) -> Result<JunctionPlan, FitError> {
    let (line_in, line_out) = match (line_of(m_in), line_of(m_out)) {
        (Some(a), Some(b)) => (a, b),
        _ => {
            let reason = if m_in.segment.spatial.is_none() || m_out.segment.spatial.is_none() {
                UnblendReason::NonSpatial
            } else {
                UnblendReason::ArcIncident
            };
            return Ok(JunctionPlan::Unblended(reason));
        }
    };

    let t_in = line_in.heading_at(line_in.s_len());
    let t_out = line_out.heading_at(0.0);
    let theta = dot(t_in, t_out).clamp(-1.0, 1.0).acos();
    if theta <= config.theta_min_rad {
        return Ok(JunctionPlan::Unblended(UnblendReason::Collinear));
    }
    if theta >= config.theta_max_rad {
        return Ok(JunctionPlan::Unblended(UnblendReason::NearReversal));
    }

    let delta = junction_deviation(m_in.limits).min(junction_deviation(m_out.limits));
    if delta <= 0.0 {
        return Ok(JunctionPlan::Unblended(UnblendReason::ZeroDeviation));
    }

    let v = match turn_normal(t_in, t_out) {
        Some(v) => v,
        None => return Ok(JunctionPlan::Unblended(UnblendReason::Collinear)),
    };

    let vertex = line_in.point_at(line_in.s_len());
    let in_len = line_in.s_len() + head_len_restore - in_reduction;
    let out_len = line_out.s_len() - out_reduction;
    let budget = 0.5 * in_len.min(out_len);
    let line_no = m_out.source.start_line;

    match biclothoid::solve(vertex, t_in, v, theta, delta, budget)
        .map_err(|source| FitError::Internal { line_no, source })?
    {
        Some(bi) => Ok(JunctionPlan::Blend(bi)),
        None => Ok(JunctionPlan::Unblended(UnblendReason::NoBudget)),
    }
}

fn emit_move(
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
    if new_len <= BUDGET_EPS_MM {
        return Ok(true);
    }

    let new_start = madd(line.start, trim_start, heading);
    let new_end = madd(line.start, trim_start + new_len, heading);
    let trimmed = Line::try_new(new_start, new_end).map_err(internal(line_no))?;
    let segment = PathSegment::try_new(Segment::Line(trimmed), m.segment.followers.clone())
        .map_err(internal(line_no))?;
    out.push(Move {
        segment,
        feedrate_mm_s: m.feedrate_mm_s,
        limits: m.limits,
        source: m.source,
    });
    Ok(false)
}

fn emit_blend(
    out: &mut Vec<Move>,
    bi: &biclothoid::Biclothoid,
    m_in: &Move,
    m_out: &Move,
) -> Result<(), FitError> {
    let scale = bi.trim / bi.half1.s_len();
    let followers_in = scaled_followers(&m_in.segment.followers, scale);
    let followers_out = scaled_followers(&m_out.segment.followers, scale);

    let seg_in = PathSegment::try_new(Segment::Clothoid(bi.half1.clone()), followers_in)
        .map_err(internal(m_in.source.start_line))?;
    out.push(Move {
        segment: seg_in,
        feedrate_mm_s: m_in.feedrate_mm_s,
        limits: m_in.limits,
        source: m_in.source,
    });

    let seg_out = PathSegment::try_new(Segment::Clothoid(bi.half2.clone()), followers_out)
        .map_err(internal(m_out.source.start_line))?;
    out.push(Move {
        segment: seg_out,
        feedrate_mm_s: m_out.feedrate_mm_s,
        limits: m_out.limits,
        source: m_out.source,
    });
    Ok(())
}

fn scaled_followers(followers: &[FollowerDemand], scale: f64) -> Vec<FollowerDemand> {
    followers
        .iter()
        .map(|f| FollowerDemand {
            axis_index: f.axis_index,
            ratio: f.ratio * scale,
        })
        .collect()
}

fn junction_deviation(limits: VelocityLimits) -> f64 {
    let scv = limits.square_corner_velocity_mm_s;
    scv * scv * (SQRT_2 - 1.0) / limits.accel_mm_s2
}

fn turn_normal(t_in: [f64; 3], t_out: [f64; 3]) -> Option<[f64; 3]> {
    let d = dot(t_out, t_in);
    let perp = [
        t_out[0] - d * t_in[0],
        t_out[1] - d * t_in[1],
        t_out[2] - d * t_in[2],
    ];
    let n = norm(perp);
    if n < TURN_NORMAL_EPS {
        None
    } else {
        Some([perp[0] / n, perp[1] / n, perp[2] / n])
    }
}

fn line_of(m: &Move) -> Option<&Line> {
    match &m.segment.spatial {
        Some(Segment::Line(line)) => Some(line),
        _ => None,
    }
}

fn blend_trim(plan: &JunctionPlan) -> f64 {
    match plan {
        JunctionPlan::Blend(bi) => bi.trim,
        JunctionPlan::Unblended(_) => 0.0,
    }
}

fn internal(line_no: u32) -> impl Fn(GeometryError) -> FitError {
    move |source| FitError::Internal { line_no, source }
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn norm(a: [f64; 3]) -> f64 {
    dot(a, a).sqrt()
}

fn madd(p: [f64; 3], s: f64, d: [f64; 3]) -> [f64; 3] {
    [p[0] + s * d[0], p[1] + s * d[1], p[2] + s * d[2]]
}

fn dist(a: [f64; 3], b: [f64; 3]) -> f64 {
    let dx = a[0] - b[0];
    let dy = a[1] - b[1];
    let dz = a[2] - b[2];
    (dx * dx + dy * dy + dz * dz).sqrt()
}

#[cfg(test)]
mod tests;
