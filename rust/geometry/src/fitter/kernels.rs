use crate::frontend::Move;
use crate::path::lowering::PositionProfile;
use crate::path::{Arc, Clothoid, CurvatureProfile, Line};
use crate::segment::FollowerDemand;

use super::vec3::{cross, dot, norm, normalize, scale, sub, turn_normal};
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
    let head_welded = welded(head_travel_len);
    let tail_welded = welded(tail_travel_len);
    let anchored = match (head_welded, tail_welded) {
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
    let Some(plane_normal) =
        anchored_plane_normal(plane_normal, origin, p0, p1, head_welded, tail_welded)
    else {
        return Ok(None);
    };

    let r0 = sub(p0, origin);
    let u = normalize(sub(r0, scale(plane_normal, dot(r0, plane_normal))));
    let v = cross(plane_normal, u);
    let mut sweep = 0.0_f64;
    let mut prev = sub(lines[0].start, origin);
    for l in &lines {
        let cur = sub(l.point_at(l.s_len()), origin);
        sweep += libm::atan2(dot(cross(prev, cur), plane_normal), dot(prev, cur));
        prev = cur;
    }
    if !tail_welded {
        let r1 = sub(p1, origin);
        let theta = libm::atan2(dot(r1, v), dot(r1, u));
        let tau = 2.0 * std::f64::consts::PI;
        sweep = theta + tau * libm::round((sweep - theta) / tau);
    }
    if !(sweep.is_finite() && sweep.abs() > ANGLE_EPS_RAD) {
        return Ok(None);
    }
    let arc = Arc::try_new(origin, u, v, rho, 0.0, sweep).map_err(internal(line_no))?;

    let head_consumption = lines[0].s_len();
    let tail_consumption = lines[lines.len() - 1].s_len();
    let (_, followers, _) = follower::construct_followers(facets, None, None);

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

const PLANE_TILT_COS_MIN: f64 = 1.0 - 1e-6;

fn anchored_plane_normal(
    n: [f64; 3],
    origin: [f64; 3],
    p0: [f64; 3],
    p1: [f64; 3],
    head_welded: bool,
    tail_welded: bool,
) -> Option<[f64; 3]> {
    let reject = |r: [f64; 3]| {
        let rr = normalize(r);
        normalize(sub(n, scale(rr, dot(n, rr))))
    };
    let candidate = match (head_welded, tail_welded) {
        (true, true) => return Some(n),
        (true, false) => reject(sub(p1, origin)),
        (false, true) => reject(sub(p0, origin)),
        (false, false) => {
            let c = cross(sub(p0, origin), sub(p1, origin));
            if norm(c) == 0.0 {
                return None;
            }
            let c = normalize(c);
            if dot(c, n) < 0.0 { scale(c, -1.0) } else { c }
        }
    };
    (dot(candidate, n) > PLANE_TILT_COS_MIN).then_some(candidate)
}
