use crate::frontend::Move;
use crate::path::lowering::PositionProfile;
use crate::path::{Arc, Clothoid, CurvatureProfile, Line};
use crate::segment::FollowerDemand;

use super::vec3::{cross, dot, normalize, sub, turn_normal};
use super::{BUDGET_EPS_MM, FitError, internal, line_of};

mod circle;
mod ease;
mod follower;

pub(super) use circle::arc_candidate;
#[cfg(test)]
pub(super) use circle::center_through_endpoints;
pub(super) use ease::{ease_run, neighbor};
pub(super) use follower::arc_len;

const ANGLE_EPS_RAD: f64 = 1e-9;

#[derive(Clone)]
pub(super) struct Reconstruction {
    pub up: Vec<Clothoid>,
    pub up_followers: Vec<FollowerDemand>,
    pub arc: Arc,
    pub down: Vec<Clothoid>,
    pub down_followers: Vec<FollowerDemand>,
    pub followers: Vec<FollowerDemand>,
    pub head_consumption: f64,
    pub tail_consumption: f64,
    pub head_line_trim: f64,
    pub tail_line_trim: f64,
}

/// `head_travel_len`/`tail_travel_len`: the length of that end's neighbor
/// when it is a travel the emission stage re-anchors onto whatever the run
/// emits (`TravelAligningSender`), so the arc need not pass through the
/// run's boundary vertex there. Welding moves the vertex by at most the
/// fit residual, so the travel qualifies only when it is long enough to
/// absorb that without degenerating. A welded end keeps the least-squares
/// circle exactly; a bound end anchors it through the vertex.
pub(super) fn reconstruct(
    facets: &[Move],
    tol: f64,
    head_travel_len: Option<f64>,
    tail_travel_len: Option<f64>,
) -> Result<Option<Reconstruction>, FitError> {
    let line_no = facets[0].source.start_line;
    let lines: Vec<&Line> = match facets.iter().map(line_of).collect::<Option<Vec<_>>>() {
        Some(l) => l,
        None => return Ok(None),
    };
    if lines.len() < 2 {
        return Ok(None);
    }

    let t0 = lines[0].heading_at(0.0);
    let v0 = match turn_normal(t0, lines[1].heading_at(0.0)) {
        Some(v) => v,
        None => return Ok(None),
    };
    let plane_normal = normalize(cross(t0, v0));
    let fit = match circle::fit_circle_through_vertices(&lines, plane_normal) {
        Some(f) => f,
        None => return Ok(None),
    };
    if fit.residual > tol {
        return Ok(None);
    }
    if !(fit.radius.is_finite() && fit.radius > BUDGET_EPS_MM) {
        return Ok(None);
    }
    if circle::max_sagitta(&lines, fit.radius) > tol {
        return Ok(None);
    }

    let last = lines[lines.len() - 1];
    let p0 = lines[0].start;
    let p1 = last.point_at(last.s_len());
    let welded =
        |travel_len: Option<f64>| travel_len.is_some_and(|len| len > fit.residual + BUDGET_EPS_MM);
    let anchored = match (welded(head_travel_len), welded(tail_travel_len)) {
        (true, true) => Some((fit.origin, fit.radius)),
        (false, true) => circle::center_through_vertex(p0, fit.origin, fit.radius),
        (true, false) => circle::center_through_vertex(p1, fit.origin, fit.radius),
        (false, false) => circle::center_through_endpoints(p0, p1, fit.origin, plane_normal),
    };
    let Some((origin, rho)) = anchored else {
        return Ok(None);
    };
    if circle::max_radial_dev(&lines, origin, rho) > tol {
        return Ok(None);
    }

    let u = normalize(sub(lines[0].start, origin));
    let v = cross(plane_normal, u);
    let mut sweep = 0.0_f64;
    let mut prev = sub(lines[0].start, origin);
    for l in &lines {
        let cur = sub(l.point_at(l.s_len()), origin);
        sweep += libm::atan2(dot(cross(prev, cur), plane_normal), dot(prev, cur));
        prev = cur;
    }
    if !(sweep.is_finite() && sweep.abs() > ANGLE_EPS_RAD) {
        return Ok(None);
    }
    let arc = Arc::try_new(origin, u, v, rho, 0.0, sweep).map_err(internal(line_no))?;

    let head_consumption = lines[0].s_len();
    let tail_consumption = lines[lines.len() - 1].s_len();
    let recon_len = arc_len(&arc);
    let (_, followers, _) = follower::construct_followers(
        facets,
        &lines,
        head_consumption,
        tail_consumption,
        recon_len,
        None,
        None,
    );

    Ok(Some(Reconstruction {
        up: Vec::new(),
        up_followers: Vec::new(),
        arc,
        down: Vec::new(),
        down_followers: Vec::new(),
        followers,
        head_consumption,
        tail_consumption,
        head_line_trim: 0.0,
        tail_line_trim: 0.0,
    }))
}
