use crate::GeometryError;
use crate::path::Clothoid;
use crate::path::lowering::PositionProfile;

use super::{dist, dot, madd};

pub(super) struct Biclothoid {
    pub half1: Clothoid,
    pub half2: Clothoid,
    pub trim: f64,
}

const DEGENERATE_EPS: f64 = 1e-12;

pub(super) fn solve(
    vertex: [f64; 3],
    t_in: [f64; 3],
    v: [f64; 3],
    theta: f64,
    delta: f64,
    budget: f64,
) -> Result<Option<Biclothoid>, GeometryError> {
    if budget <= super::BUDGET_EPS_MM || delta <= 0.0 {
        return Ok(None);
    }

    let (trim_ref, deviation_ref) = canonical(theta)?;
    if trim_ref <= DEGENERATE_EPS || deviation_ref <= DEGENERATE_EPS {
        return Ok(None);
    }

    let trim_at_delta = trim_ref * delta / deviation_ref;
    let trim = trim_at_delta.min(budget);
    if trim <= super::BUDGET_EPS_MM {
        return Ok(None);
    }

    let kappa_peak = trim_ref * theta / trim;
    let length = theta / kappa_peak;
    let sigma = kappa_peak / length;

    let a_start = madd(vertex, -trim, t_in);
    let half1 = Clothoid::try_new(a_start, t_in, v, 0.0, sigma, length)?;
    let apex = half1.point_at(length);
    let u_apex = half1.heading_at(length);
    let v_apex = inplane_rot90(u_apex, t_in, v);
    let half2 = Clothoid::try_new(apex, u_apex, v_apex, kappa_peak, -sigma, length)?;

    Ok(Some(Biclothoid { half1, half2, trim }))
}

fn canonical(theta: f64) -> Result<(f64, f64), GeometryError> {
    let x = [1.0, 0.0, 0.0];
    let y = [0.0, 1.0, 0.0];
    let sigma = theta;
    let length = 1.0;

    let half1 = Clothoid::try_new([0.0; 3], x, y, 0.0, sigma, length)?;
    let apex = half1.point_at(length);
    let u_apex = half1.heading_at(length);
    let v_apex = inplane_rot90(u_apex, x, y);
    let half2 = Clothoid::try_new(apex, u_apex, v_apex, theta, -sigma, length)?;
    let end = half2.point_at(length);
    let end_heading = half2.heading_at(length);

    if end_heading[1].abs() < DEGENERATE_EPS {
        return Ok((0.0, 0.0));
    }
    let back = end[1] / end_heading[1];
    let corner_x = end[0] - back * end_heading[0];
    let corner = [corner_x, 0.0, 0.0];

    Ok((corner_x, dist(apex, corner)))
}

fn inplane_rot90(w: [f64; 3], u: [f64; 3], v: [f64; 3]) -> [f64; 3] {
    let a = dot(w, u);
    let b = dot(w, v);
    [
        a * v[0] - b * u[0],
        a * v[1] - b * u[1],
        a * v[2] - b * u[2],
    ]
}
