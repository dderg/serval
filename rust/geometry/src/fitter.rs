mod biclothoid;
mod causal;
mod config;
mod emit;
mod kernels;
mod linalg;
mod move_ops;
mod overlap;
mod runfit;
use crate::vec3;

use crate::GeometryError;
use crate::frontend::{Move, VelocityLimits};
use crate::path::lowering::PositionProfile;
use crate::path::{CurvatureProfile, Line};
use crate::segment::FollowerDemand;
use vec3::{dot, turn_normal};

pub use config::CornerFitConfig;
pub use move_ops::{
    blend_moves, consumption_moves, is_travel, merge_collinear_lines, spatial_end, spatial_start,
    trim_line_move,
};
pub use runfit::RunFit;

use emit::{BUDGET_EPS_MM, internal};
use move_ops::line_of;

pub(crate) const TURN_NORMAL_EPS: f64 = 1e-9;

/// The fastest the planner can drive a piece: its feedrate, the machine cap,
/// and the centripetal ceiling `√(A/κ)` at the piece's curvature peak — the
/// same `disk::limit_speed` bound velocity planning enforces.
fn worst_case_speed(seg: &impl CurvatureProfile, feedrate: f64, limits: VelocityLimits) -> f64 {
    let (_, kappa_peak) = seg.kappa_peak();
    let curvature_cap = if kappa_peak > 0.0 {
        (limits.accel_mm_s2 / kappa_peak).sqrt()
    } else {
        f64::INFINITY
    };
    feedrate.min(limits.max_velocity_mm_s).min(curvature_cap)
}

/// Whether every ramp on the piece keeps its *additional* extruder load
/// within `accel_budget`. With ratio slope `m = dr/ds` and the path at speed
/// `v ≤ v_cap`: `ė = r·v` and `ë = r·a + m·v²`. The `r`-terms are the load
/// the G-code's own constant-ratio flow already commands — not the fitter's
/// to police — so the gate charges only the slope's marginal `m·v²` of
/// extruder acceleration, monotone in `v`, worst-cased at `v_cap`. Constant
/// demands pass unconditionally.
pub(crate) fn ramps_admitted(
    accel_budget: f64,
    followers: &[FollowerDemand],
    seg: &impl CurvatureProfile,
    feedrate: f64,
    limits: VelocityLimits,
) -> bool {
    let len = seg.s_len();
    let v_cap = worst_case_speed(seg, feedrate, limits);
    followers
        .iter()
        .all(|d| d.ratio_slope(len).abs() * v_cap * v_cap <= accel_budget)
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UnblendReason {
    Collinear,
    NearReversal,
    ZeroDeviation,
    NoBudget,
    ArcIncident,
    NonSpatial,
    /// The streaming fit stage emitted the upstream move while its input was
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
    /// The blend's extrusion ramp would demand more extruder acceleration
    /// than [`CornerFitConfig::ramp_accel_budget_mm_s2`] allows in the worst
    /// case, so the corner stays sharp and the planner stops there instead.
    ExtrusionRampInfeasible,
}

#[cfg(test)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct UnblendedJunction {
    pub line_no: u32,
    pub reason: UnblendReason,
}

#[cfg(test)]
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct FitReport {
    pub blended: u32,
    pub unblended: Vec<UnblendedJunction>,
    pub consumed_legs: u32,
    pub chains: u32,
}

#[cfg(test)]
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

/// A solved clothoid-pair corner blend, opaque outside the fitter. The trims
/// are the spatial lengths the blend consumes from the inbound line's end and
/// the outbound line's start — equal for a symmetric blend, unequal when the
/// blend extended into the roomier side.
pub struct JunctionBlend(biclothoid::GeneralBlend);

impl JunctionBlend {
    #[must_use]
    pub fn trim_in(&self) -> f64 {
        self.0.trim_in
    }

    #[must_use]
    pub fn trim_out(&self) -> f64 {
        self.0.trim_out
    }
}

pub enum JunctionPlan {
    Blend(JunctionBlend),
    Unblended(UnblendReason),
}

/// Whether one arc can pass through all these facets within the junction
/// deviation: every facet is a line with matching extrusion ratio (rounding
/// tolerance only), consecutive junctions turn consistently in one plane, and
/// the shared circle fit stays within tolerance. Failure is final under
/// append: an arc through a longer prefix would also pass through this one.
#[must_use]
pub fn arc_candidate_fits(facets: &[Move], config: CornerFitConfig) -> bool {
    let tol = span_tolerance(facets);
    tol.is_finite() && kernels::arc_candidate(facets, config, tol)
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

/// The cocircularity tolerance the run detector derives from the moves' corner
/// limits: the smallest positive corner deviation in the window.
fn span_tolerance(moves: &[Move]) -> f64 {
    moves
        .iter()
        .map(|m| m.limits.corner_deviation_mm)
        .filter(|d| d.is_finite() && *d > 0.0)
        .fold(f64::INFINITY, f64::min)
}

#[cfg(test)]
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
            blend_trim(&plans[i - 1], JunctionBlend::trim_out)
        } else {
            0.0
        };
        let trim_end = if i < plans.len() {
            blend_trim(&plans[i], JunctionBlend::trim_in)
        } else {
            0.0
        };
        if emit::emit_move(&mut out, m, trim_start, trim_end)? {
            report.consumed_legs += 1;
        }

        if i < plans.len() {
            match &plans[i] {
                JunctionPlan::Blend(bi) => {
                    report.blended += 1;
                    emit::emit_blend(&mut out, &bi.0, m, &moves[i + 1])?;
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

/// A solved blend that replaces a whole squeezed facet along with the tail
/// and head of its neighbors, opaque outside the fitter. The blend owes the
/// facet nothing — no contact, no tangency — it only stays within the
/// junction deviation of the polyline it replaces and carries the facet's
/// extrusion in its follower ramp.
pub struct FacetConsumption(biclothoid::ChainBlend);

impl FacetConsumption {
    #[must_use]
    pub fn trim_in(&self) -> f64 {
        self.0.trim_in
    }

    #[must_use]
    pub fn trim_out(&self) -> f64 {
        self.0.trim_out
    }
}

/// Whether `m_mid` looks like a consumable facet from its entry side alone:
/// a line so short that the corner into it is squeezed below the
/// deviation-optimal blend. The stream stage uses this to decide whether the
/// element after `m_mid` is worth waiting for; [`plan_facet_consumption`]
/// re-checks everything with both anchors known.
#[must_use]
pub fn facet_consumption_candidate(
    m_in: &Move,
    m_mid: &Move,
    config: CornerFitConfig,
    in_reduction: f64,
) -> bool {
    let (Some(line_in), Some(line_mid)) = (line_of(m_in), line_of(m_mid)) else {
        return false;
    };
    if 0.5 * (line_in.s_len() - in_reduction) <= BUDGET_EPS_MM {
        return false;
    }
    let t_in = line_in.heading_at(line_in.s_len());
    let t_mid = line_mid.heading_at(0.0);
    let theta1 = libm::acos(dot(t_in, t_mid).clamp(-1.0, 1.0));
    if theta1 <= config.theta_min_rad || theta1 >= config.theta_max_rad {
        return false;
    }
    let delta =
        junction_deviation(m_in.limits, config).min(junction_deviation(m_mid.limits, config));
    if !(delta.is_finite() && delta > 0.0) {
        return false;
    }
    let Ok(t_ad1) = biclothoid::trim_at_delta(theta1, delta) else {
        return false;
    };
    0.5 * line_mid.s_len() < t_ad1
}

/// Plan the consumption of the facet chain `mids` by one G2 blend from
/// `m_in` to `m_out`. `Ok(None)` when any gate fails — the junctions then
/// blend (or stay sharp) pairwise as usual. Gates: every move is a line;
/// every corner turn is blendable and all turn the same way (each clothoid
/// pair has one curvature bump, so S-jogs stay pairwise); every facet is
/// squeezed below its corners' deviation-optimal trims (a roomy facet blends
/// better on its own); the solved blend corners no harder than the pairwise
/// per-junction blends it replaces (near the squeeze limit, two blends with
/// their own footprints traverse faster than one blend squeezed through the
/// whole turn); every facet's extrusion fits the ramp the anchors span; and
/// the blend's extrusion ramps pass the kinematic gate.
pub fn plan_facet_consumption(
    m_in: &Move,
    mids: &[&Move],
    m_out: &Move,
    config: CornerFitConfig,
    in_reduction: f64,
) -> Result<Option<FacetConsumption>, FitError> {
    let [first_mid, ..] = mids else {
        return Ok(None);
    };
    if !facet_consumption_candidate(m_in, first_mid, config, in_reduction) {
        return Ok(None);
    }
    let (Some(line_in), Some(line_out)) = (line_of(m_in), line_of(m_out)) else {
        return Ok(None);
    };
    let mut facet_lines = Vec::with_capacity(mids.len());
    for m in mids {
        let Some(line) = line_of(m) else {
            return Ok(None);
        };
        facet_lines.push(line);
    }
    let line_no = first_mid.source.start_line;
    let internal_err = |source| FitError::Internal { line_no, source };

    let t_in = line_in.heading_at(line_in.s_len());
    let t_out = line_out.heading_at(0.0);
    let mut tangents = Vec::with_capacity(mids.len() + 2);
    tangents.push(t_in);
    tangents.extend(facet_lines.iter().map(|l| l.heading_at(0.0)));
    tangents.push(t_out);

    let mut thetas = Vec::with_capacity(tangents.len() - 1);
    let mut first_normal = None;
    for w in tangents.windows(2) {
        let theta = libm::acos(dot(w[0], w[1]).clamp(-1.0, 1.0));
        if theta <= config.theta_min_rad || theta >= config.theta_max_rad {
            return Ok(None);
        }
        let Some(n) = turn_normal(w[0], w[1]) else {
            return Ok(None);
        };
        if dot(n, *first_normal.get_or_insert(n)) <= 0.0 {
            return Ok(None);
        }
        thetas.push(theta);
    }
    let theta_ab = libm::acos(dot(t_in, t_out).clamp(-1.0, 1.0));
    if theta_ab <= config.theta_min_rad || theta_ab >= config.theta_max_rad {
        return Ok(None);
    }

    let consumed_followers: Vec<&[FollowerDemand]> = mids
        .iter()
        .map(|m| m.segment.followers.as_slice())
        .collect();
    if !emit::ratios_within_ramp_band(
        &m_in.segment.followers,
        &consumed_followers,
        &m_out.segment.followers,
        config.extrusion_ramp_rel_tol,
    ) {
        return Ok(None);
    }

    let delta = std::iter::once(m_in)
        .chain(mids.iter().copied())
        .chain(std::iter::once(m_out))
        .map(|m| junction_deviation(m.limits, config))
        .fold(f64::INFINITY, f64::min);
    if !(delta.is_finite() && delta > 0.0) {
        return Ok(None);
    }
    let mut trims_at_delta = Vec::with_capacity(thetas.len());
    for theta in &thetas {
        trims_at_delta.push(biclothoid::trim_at_delta(*theta, delta).map_err(internal_err)?);
    }
    for (i, line) in facet_lines.iter().enumerate() {
        if 0.5 * line.s_len() >= trims_at_delta[i].min(trims_at_delta[i + 1]) {
            return Ok(None);
        }
    }

    let t_ad_ab = biclothoid::trim_at_delta(theta_ab, delta).map_err(internal_err)?;
    let t_cap = (0.5 * (line_in.s_len() - in_reduction))
        .min(0.5 * line_out.s_len())
        .min(t_ad_ab);
    let mut vertices = Vec::with_capacity(mids.len() + 1);
    vertices.push(line_in.point_at(line_in.s_len()));
    vertices.extend(facet_lines.iter().map(|l| l.end));
    let Some(blend) = biclothoid::solve_consume_chain(&vertices, t_in, t_out, delta, t_cap) else {
        return Ok(None);
    };

    let kappa_consume = blend
        .segments
        .iter()
        .map(|s| s.kappa_peak().1)
        .fold(0.0, f64::max);
    let mut span_lens = Vec::with_capacity(mids.len() + 2);
    span_lens.push(line_in.s_len() - in_reduction);
    span_lens.extend(facet_lines.iter().map(|l| l.s_len()));
    span_lens.push(line_out.s_len());
    let mut kappa_pairwise = 0.0_f64;
    for (i, theta) in thetas.iter().enumerate() {
        let trim = trims_at_delta[i].min(0.5 * span_lens[i].min(span_lens[i + 1]));
        kappa_pairwise = kappa_pairwise
            .max(biclothoid::symmetric_blend_kappa_peak(*theta, trim).map_err(internal_err)?);
    }
    if kappa_consume > kappa_pairwise {
        return Ok(None);
    }

    let followers = consumption_followers(&blend, m_in, mids, m_out, line_in, line_out);
    let feedrates = consumption_feedrates(&blend, m_in, mids, m_out);
    let admitted = blend
        .segments
        .iter()
        .zip(&followers)
        .zip(&feedrates)
        .zip(consumption_limits(&blend, m_in, mids, m_out))
        .all(|(((seg, f), feed), limits)| {
            ramps_admitted(config.ramp_accel_budget_mm_s2, f, seg, *feed, limits)
        });
    if !admitted {
        return Ok(None);
    }
    Ok(Some(FacetConsumption(blend)))
}

fn consumption_followers(
    blend: &biclothoid::ChainBlend,
    m_in: &Move,
    mids: &[&Move],
    m_out: &Move,
    line_in: &Line,
    line_out: &Line,
) -> Vec<Vec<FollowerDemand>> {
    let consumed: Vec<(&[FollowerDemand], f64)> = mids
        .iter()
        .map(|m| (m.segment.followers.as_slice(), m.segment.s_len()))
        .collect();
    let seg_lens: Vec<f64> = blend.segments.iter().map(CurvatureProfile::s_len).collect();
    emit::chain_blend_followers(
        &emit::SeamSide {
            followers: &m_in.segment.followers,
            seg_len: line_in.s_len(),
            trim: blend.trim_in,
        },
        &consumed,
        &emit::SeamSide {
            followers: &m_out.segment.followers,
            seg_len: line_out.s_len(),
            trim: blend.trim_out,
        },
        &seg_lens,
    )
}

/// Each blend segment's feedrate: the consumed facets' minimum (the G-code
/// asked for those speeds over the region the blend now covers), with the
/// boundary segments additionally capped by the neighbor they seam into.
fn consumption_feedrates(
    blend: &biclothoid::ChainBlend,
    m_in: &Move,
    mids: &[&Move],
    m_out: &Move,
) -> Vec<f64> {
    let mid_min = mids
        .iter()
        .map(|m| m.feedrate_mm_s)
        .fold(f64::INFINITY, f64::min);
    let last = blend.segments.len() - 1;
    (0..blend.segments.len())
        .map(|i| match i {
            0 => mid_min.min(m_in.feedrate_mm_s),
            i if i == last => mid_min.min(m_out.feedrate_mm_s),
            _ => mid_min,
        })
        .collect()
}

/// Each blend segment's velocity limits: the componentwise minimum over the
/// consumed facets for interior segments, the seamed neighbor's own limits at
/// the boundaries — mirroring how the pairwise blend halves inherit theirs.
fn consumption_limits(
    blend: &biclothoid::ChainBlend,
    m_in: &Move,
    mids: &[&Move],
    m_out: &Move,
) -> Vec<VelocityLimits> {
    let interior = mids
        .iter()
        .map(|m| m.limits)
        .reduce(|a, b| VelocityLimits {
            max_velocity_mm_s: a.max_velocity_mm_s.min(b.max_velocity_mm_s),
            accel_mm_s2: a.accel_mm_s2.min(b.accel_mm_s2),
            corner_deviation_mm: a.corner_deviation_mm.min(b.corner_deviation_mm),
            max_jerk_mm_s3: a.max_jerk_mm_s3.min(b.max_jerk_mm_s3),
        })
        .expect("mids is non-empty");
    let last = blend.segments.len() - 1;
    (0..=last)
        .map(|i| match i {
            0 => m_in.limits,
            i if i == last => m_out.limits,
            _ => interior,
        })
        .collect()
}

fn classify_junction(
    m_in: &Move,
    m_out: &Move,
    config: CornerFitConfig,
    in_reduction: f64,
    out_reduction: f64,
) -> Result<JunctionPlan, FitError> {
    let (line_in, line_out) = match (move_ops::line_of(m_in), move_ops::line_of(m_out)) {
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

    if emit::extrusion_step(
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

    let delta =
        junction_deviation(m_in.limits, config).min(junction_deviation(m_out.limits, config));
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

    let Some(bi) = biclothoid::solve_line_line(vertex, t_in, v, theta, delta, budget)
        .map_err(|source| FitError::Internal { line_no, source })?
    else {
        return Ok(JunctionPlan::Unblended(UnblendReason::NoBudget));
    };

    let (f_in, f_out) = biclothoid_followers(&bi, m_in, m_out, line_in, line_out);
    let admitted = ramps_admitted(
        config.ramp_accel_budget_mm_s2,
        &f_in,
        &bi.half1,
        m_in.feedrate_mm_s,
        m_in.limits,
    ) && ramps_admitted(
        config.ramp_accel_budget_mm_s2,
        &f_out,
        &bi.half2,
        m_out.feedrate_mm_s,
        m_out.limits,
    );
    if !admitted {
        return Ok(JunctionPlan::Unblended(
            UnblendReason::ExtrusionRampInfeasible,
        ));
    }
    Ok(JunctionPlan::Blend(JunctionBlend(bi)))
}

fn biclothoid_followers(
    bi: &biclothoid::GeneralBlend,
    m_in: &Move,
    m_out: &Move,
    line_in: &Line,
    line_out: &Line,
) -> (Vec<FollowerDemand>, Vec<FollowerDemand>) {
    emit::blend_followers(
        &emit::SeamSide {
            followers: &m_in.segment.followers,
            seg_len: line_in.s_len(),
            trim: bi.trim_in,
        },
        &emit::SeamSide {
            followers: &m_out.segment.followers,
            seg_len: line_out.s_len(),
            trim: bi.trim_out,
        },
        bi.half1.s_len(),
        bi.half2.s_len(),
    )
}

#[must_use]
pub fn kernel_corner_deviation_mm(kernel_variance_s2: f64, accel_mm_s2: f64) -> f64 {
    0.5 * kernel_variance_s2 * accel_mm_s2
}

/// The deviation budget a clothoid corner blend may spend. The smoothing
/// kernel pulls the blended corner inward by a further ~(sigma^2/2)*a, additive
/// with the blend's own deviation, so the blend gets what remains of
/// `corner_deviation` after the kernel's share. Arc-run fitting and collinear
/// merging deliberately do NOT use this: there the budget is a reconstruction
/// tolerance ("are these facets one intended curve/line"), and the kernel's
/// pull on the result is the same whether or not the facets fuse.
fn junction_deviation(limits: VelocityLimits, config: CornerFitConfig) -> f64 {
    limits.corner_deviation_mm
        - kernel_corner_deviation_mm(config.kernel_variance_s2, limits.accel_mm_s2)
}

#[cfg(test)]
fn blend_trim(plan: &JunctionPlan, side: impl Fn(&JunctionBlend) -> f64) -> f64 {
    match plan {
        JunctionPlan::Blend(bi) => side(bi),
        JunctionPlan::Unblended(_) => 0.0,
    }
}

#[cfg(test)]
mod tests;

#[cfg(test)]
mod c2_continuity_tests;
#[cfg(test)]
mod consumption_tests;
#[cfg(test)]
mod cruise_onset_tests;
#[cfg(test)]
mod plan_velocity_bench;
