mod biclothoid;
mod causal;
mod kernels;
mod linalg;
mod overlap;
use crate::vec3;

use std::f64::consts::{PI, SQRT_2};

use crate::GeometryError;
use crate::frontend::{Move, VelocityLimits};
use crate::path::lowering::PositionProfile;
use crate::path::{CurvatureProfile, Line, PathSegment, Segment};
use crate::segment::FollowerDemand;
use vec3::{dot, madd, turn_normal};

const COLLINEAR_EPS_RAD: f64 = 1e-3;
const BUDGET_EPS_MM: f64 = 1e-9;
/// Over-trim beyond this is a real overlap of two neighbors' claims, not
/// floating-point noise — the same order as the pipeline's position-contiguity
/// tolerance at ingress.
const OVER_TRIM_TOL_MM: f64 = 1e-6;
pub(crate) const TURN_NORMAL_EPS: f64 = 1e-9;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CornerFitConfig {
    pub theta_min_rad: f64,
    pub theta_max_rad: f64,
    /// Above this relative difference in per-axis extrusion ratio across a
    /// corner, the junction is left unblended (a full stop) rather than blended
    /// with a mid-corner ramp — see [`UnblendReason::ExtrusionStep`].
    pub extrusion_ramp_rel_tol: f64,
}

const EXTRUSION_RAMP_REL_TOL: f64 = 0.25;

impl Default for CornerFitConfig {
    fn default() -> Self {
        Self {
            theta_min_rad: COLLINEAR_EPS_RAD,
            theta_max_rad: PI - COLLINEAR_EPS_RAD,
            extrusion_ramp_rel_tol: EXTRUSION_RAMP_REL_TOL,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ArcFitConfig {
    pub min_run_facets: u32,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ChainFitConfig {
    pub corner: CornerFitConfig,
    pub arc_fit: Option<ArcFitConfig>,
}

impl Default for ChainFitConfig {
    fn default() -> Self {
        Self {
            corner: CornerFitConfig::default(),
            arc_fit: None,
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
    /// The streaming fitter emitted the upstream move while its input was
    /// empty, so this junction was cut without a blend: the toolhead must be
    /// at rest across it regardless of what a blend could have achieved.
    StreamCut,
    /// The two extruding lines demand extrusion ratios (`de/ds`) that differ by
    /// more than [`CornerFitConfig::extrusion_ramp_rel_tol`]. A blend ramps the
    /// ratio across the corner, but only a modest step can be ramped at corner
    /// speed without a visible flow surge; an abrupt step (e.g. wall→infill
    /// width change) is left as a sharp corner so the planner stops there and
    /// the ratio changes at rest. Travel↔extrude transitions (one side ratio 0)
    /// are exempt: ramping to or from zero is always desirable, so they still
    /// blend.
    ExtrusionStep,
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
    Internal {
        line_no: u32,
        source: GeometryError,
    },
    /// The trims claimed at a line's two ends exceed its length by more than a
    /// rounding margin: two neighbors (runs, blends) each consumed geometry the
    /// other also claimed. Emitting would silently drop the overlap and leave a
    /// position discontinuity, so the fit fails here instead.
    OverTrimmedLine {
        line_no: u32,
        excess_mm: f64,
    },
}

/// A solved biclothoid corner blend, opaque outside the fitter. `trim` is the
/// spatial length the blend consumes from each adjoining line's end/start.
pub struct JunctionBlend(biclothoid::Biclothoid);

impl JunctionBlend {
    #[must_use]
    pub fn trim(&self) -> f64 {
        self.0.trim
    }
}

pub enum JunctionPlan {
    Blend(JunctionBlend),
    Unblended(UnblendReason),
}

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

/// Whether one arc can pass through all these facets within the junction
/// deviation: every facet is a line with matching extrusion ratio (rounding
/// tolerance only), consecutive junctions turn consistently in one plane, and
/// the shared circle fit stays within tolerance. Failure is final under
/// append: an arc through a longer prefix would also pass through this one.
#[must_use]
pub fn arc_candidate_fits(facets: &[Move], config: ChainFitConfig) -> bool {
    if config.arc_fit.is_none() {
        return false;
    }
    let tol = span_tolerance(facets);
    tol.is_finite() && kernels::arc_candidate(facets, config.corner, tol)
}

/// Classify a line-line junction whose adjoining lengths are partly consumed
/// by neighboring arc-run easing: `in_reduction` is spent from `m_in`'s length
/// (a run behind it eased into its head), `out_reduction` from `m_out`'s (a
/// run ahead eases into its tail).
pub fn plan_junction_reduced(
    m_in: &Move,
    m_out: &Move,
    config: CornerFitConfig,
    in_reduction: f64,
    out_reduction: f64,
) -> Result<JunctionPlan, FitError> {
    classify_junction(m_in, m_out, config, in_reduction, out_reduction)
}

/// A sealed arc run's reconstruction: the arc, its easing clothoids into the
/// neighbor lines, and any boundary blends resolved against them. Built only
/// once the run's extent and both neighbors are final — easing refits the
/// circle so the arc geometry does not exist before then.
pub struct RunFit {
    recon: kernels::Reconstruction,
    tol: f64,
    head_blend_trim: f64,
    tail_blend_trim: f64,
    head_line_extra: f64,
    tail_line_extra: f64,
}

impl RunFit {
    /// Reconstruct and ease a sealed run. `head`/`tail` are the adjoining
    /// moves when they are plain (not part of another run); `None` when the
    /// run abuts another run, a stream edge, or nothing. Returns `Ok(None)`
    /// when no valid reconstruction exists — the facets stay plain lines.
    pub fn fit(
        facets: &[Move],
        head: Option<&Move>,
        tail: Option<&Move>,
    ) -> Result<Option<RunFit>, FitError> {
        let tol = span_tolerance(facets);
        if !tol.is_finite() {
            return Ok(None);
        }
        let Some(mut recon) = kernels::reconstruct(facets, tol)? else {
            return Ok(None);
        };
        let head_nb = head.and_then(|m| kernels::neighbor(m, true));
        let tail_nb = tail.and_then(|m| kernels::neighbor(m, false));
        kernels::ease_run(&mut recon, facets, head_nb.as_ref(), tail_nb.as_ref(), tol)?;
        Ok(Some(RunFit {
            recon,
            tol,
            head_blend_trim: 0.0,
            tail_blend_trim: 0.0,
            head_line_extra: 0.0,
            tail_line_extra: 0.0,
        }))
    }

    /// Length consumed from the head neighbor's tail (easing plus boundary
    /// blend) — the neighbor's emission trim and the run's first-facet head
    /// trim, exactly as the batch fit applies them.
    #[must_use]
    pub fn head_boundary_trim(&self) -> f64 {
        self.recon.head_line_trim + self.head_line_extra
    }

    #[must_use]
    pub fn tail_boundary_trim(&self) -> f64 {
        self.recon.tail_line_trim + self.tail_line_extra
    }

    /// Easing consumption alone — the budget reduction junction classification
    /// applies two junctions out from the run.
    #[must_use]
    pub fn head_line_trim(&self) -> f64 {
        self.recon.head_line_trim
    }

    #[must_use]
    pub fn tail_line_trim(&self) -> f64 {
        self.recon.tail_line_trim
    }

    #[must_use]
    pub fn head_consumption(&self) -> f64 {
        self.recon.head_consumption
    }

    #[must_use]
    pub fn tail_consumption(&self) -> f64 {
        self.recon.tail_consumption
    }

    /// Blend the run's un-eased head into the bare neighbor line before it.
    /// Returns the blend's clothoid halves to emit between the neighbor and
    /// the run (empty when no blend applies).
    pub fn blend_head_with_line(
        &mut self,
        neighbor: &Move,
        run_first: &Move,
        corner: CornerFitConfig,
    ) -> Result<Vec<Move>, FitError> {
        if !self.recon.up.is_empty() {
            return Ok(Vec::new());
        }
        let Some(line) = line_of(neighbor) else {
            return Ok(Vec::new());
        };
        let Some(blend) = overlap::resolve_arc_line(&self.recon.arc, line, false, corner, self.tol)
        else {
            return Ok(Vec::new());
        };
        self.head_line_extra = blend.trim_in;
        self.head_blend_trim = blend.trim_out;
        let mut out = Vec::with_capacity(2);
        causal::emit_general_blend(
            &mut out,
            &blend,
            &neighbor.segment.followers,
            &self.recon.followers,
            neighbor,
            run_first,
        )?;
        Ok(out)
    }

    /// Blend the run's un-eased tail into the bare neighbor line after it.
    pub fn blend_tail_with_line(
        &mut self,
        run_last: &Move,
        neighbor: &Move,
        corner: CornerFitConfig,
    ) -> Result<Vec<Move>, FitError> {
        if !self.recon.down.is_empty() {
            return Ok(Vec::new());
        }
        let Some(line) = line_of(neighbor) else {
            return Ok(Vec::new());
        };
        let Some(blend) = overlap::resolve_arc_line(&self.recon.arc, line, true, corner, self.tol)
        else {
            return Ok(Vec::new());
        };
        self.tail_blend_trim = blend.trim_in;
        self.tail_line_extra = blend.trim_out;
        let mut out = Vec::with_capacity(2);
        causal::emit_general_blend(
            &mut out,
            &blend,
            &self.recon.followers,
            &neighbor.segment.followers,
            run_last,
            neighbor,
        )?;
        Ok(out)
    }

    /// Blend two adjacent runs' arcs at their shared junction.
    pub fn blend_tail_with_run(
        &mut self,
        next: &mut RunFit,
        run_last: &Move,
        next_first: &Move,
        corner: CornerFitConfig,
    ) -> Result<Vec<Move>, FitError> {
        if !(self.recon.down.is_empty() && next.recon.up.is_empty()) {
            return Ok(Vec::new());
        }
        let Some(blend) =
            overlap::resolve_arc_arc(&self.recon.arc, &next.recon.arc, corner, self.tol)
        else {
            return Ok(Vec::new());
        };
        self.tail_blend_trim = blend.trim_in;
        next.head_blend_trim = blend.trim_out;
        let mut out = Vec::with_capacity(2);
        causal::emit_general_blend(
            &mut out,
            &blend,
            &self.recon.followers,
            &next.recon.followers,
            run_last,
            next_first,
        )?;
        Ok(out)
    }

    /// The run's replacement pieces: up-easing clothoids, the (blend-trimmed)
    /// arc, and down-easing clothoids. The first/last facets' remaining stubs
    /// are the caller's to emit around these.
    pub fn pieces(&self, m_start: &Move, m_end: &Move) -> Result<Vec<Move>, FitError> {
        let mut out = Vec::new();
        causal::emit_reconstruction(
            &mut out,
            &self.recon,
            m_start,
            m_end,
            self.head_blend_trim,
            self.tail_blend_trim,
        )?;
        Ok(out)
    }
}

/// The cocircularity tolerance the run detector derives from the moves' corner
/// limits: the smallest positive junction deviation in the window.
fn span_tolerance(moves: &[Move]) -> f64 {
    moves
        .iter()
        .map(|m| junction_deviation(m.limits))
        .filter(|d| d.is_finite() && *d > 0.0)
        .fold(f64::INFINITY, f64::min)
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
        plans.push(classify_junction(&pair[0], &pair[1], config, 0.0, 0.0)?);
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
                    emit_blend(&mut out, &bi.0, m, &moves[i + 1])?;
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

fn classify_junction(
    m_in: &Move,
    m_out: &Move,
    config: CornerFitConfig,
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

    if extrusion_step(
        &m_in.segment.followers,
        &m_out.segment.followers,
        config.extrusion_ramp_rel_tol,
    ) {
        return Ok(JunctionPlan::Unblended(UnblendReason::ExtrusionStep));
    }

    let t_in = line_in.heading_at(line_in.s_len());
    let t_out = line_out.heading_at(0.0);
    let theta = libm::acos(dot(t_in, t_out).clamp(-1.0, 1.0));
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
    let in_len = line_in.s_len() - in_reduction;
    let out_len = line_out.s_len() - out_reduction;
    let budget = 0.5 * in_len.min(out_len);
    let line_no = m_out.source.start_line;

    match biclothoid::solve(vertex, t_in, v, theta, delta, budget)
        .map_err(|source| FitError::Internal { line_no, source })?
    {
        Some(bi) => Ok(JunctionPlan::Blend(JunctionBlend(bi))),
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
    let len1 = bi.half1.s_len();
    let len2 = bi.half2.s_len();
    let (f_in, f_out) = blend_followers(
        &m_in.segment.followers,
        &m_out.segment.followers,
        bi.trim,
        len1,
        bi.trim,
        len2,
    );

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
fn extrusion_step(
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

/// Split a blend's inbound/outbound demands into the two clothoid halves as a
/// pair of linear ratio ramps. The blend's endpoints anchor at the neighbors'
/// own ratios, so `ė = r·v` is continuous where the blend meets the trimmed
/// lines; the shared midpoint ratio is then whatever conserves the E the
/// trimmed line material carried (`r_in·trim_in + r_out·trim_out`) across the
/// halves' actual arc lengths. Anchoring at rescaled ratios instead (the
/// trimmed E spread uniformly over the shorter clothoid) would conserve E too,
/// but it steps the extrusion rate by `trim/len` at both outer seams — the
/// very discontinuity the ramp exists to remove.
fn blend_followers(
    in_followers: &[FollowerDemand],
    out_followers: &[FollowerDemand],
    trim_in: f64,
    len1: f64,
    trim_out: f64,
    len2: f64,
) -> (Vec<FollowerDemand>, Vec<FollowerDemand>) {
    let axes = follower_axes(in_followers, out_followers);
    let mut half1 = Vec::with_capacity(axes.len());
    let mut half2 = Vec::with_capacity(axes.len());
    for axis in axes {
        let r_in = ratio_end_for(in_followers, axis);
        let r_out = ratio_start_for(out_followers, axis);
        let e_target = r_in * trim_in + r_out * trim_out;
        let r_mid = (2.0 * e_target - r_in * len1 - r_out * len2) / (len1 + len2);
        half1.push(FollowerDemand::ramp(axis, r_in, r_mid));
        half2.push(FollowerDemand::ramp(axis, r_mid, r_out));
    }
    (half1, half2)
}

fn junction_deviation(limits: VelocityLimits) -> f64 {
    let scv = limits.square_corner_velocity_mm_s;
    scv * scv * (SQRT_2 - 1.0) / limits.accel_mm_s2
}

fn line_of(m: &Move) -> Option<&Line> {
    match &m.segment.spatial {
        Some(Segment::Line(line)) => Some(line),
        _ => None,
    }
}

fn blend_trim(plan: &JunctionPlan) -> f64 {
    match plan {
        JunctionPlan::Blend(bi) => bi.trim(),
        JunctionPlan::Unblended(_) => 0.0,
    }
}

fn internal(line_no: u32) -> impl Fn(GeometryError) -> FitError {
    move |source| FitError::Internal { line_no, source }
}

#[cfg(test)]
mod tests;

#[cfg(test)]
mod c2_continuity_tests;
#[cfg(test)]
mod cruise_onset_tests;
#[cfg(test)]
mod plan_velocity_bench;
