use crate::GeometryError;
use crate::path::Clothoid;
use crate::path::CurvatureProfile;
use crate::path::lowering::PositionProfile;

use super::linalg::solve3;
use super::vec3::{add, cross, dist, dot, madd, normalize, scale, signed_angle, sub};

pub(super) struct Biclothoid {
    pub half1: Clothoid,
    pub half2: Clothoid,
    pub trim: f64,
}

pub(super) struct ClothoidPair {
    pub half1: Clothoid,
    pub half2: Clothoid,
}

const DEGENERATE_EPS: f64 = 1e-12;
const HERMITE_POS_TOL_MM: f64 = 1e-10;
const HERMITE_ANG_TOL_RAD: f64 = 1e-12;
const HERMITE_MAX_ITERS: usize = 60;

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

struct Endpoint {
    pose: [f64; 3],
    tangent: [f64; 3],
    kappa: f64,
}

fn build_pair(
    start: &Endpoint,
    kappa_b: f64,
    plane_n: [f64; 3],
    kappa_peak: f64,
    l1: f64,
    l2: f64,
) -> Option<ClothoidPair> {
    if !(l1 > 0.0 && l2 > 0.0) {
        return None;
    }
    let v_a = normalize(cross(plane_n, start.tangent));
    let sigma1 = (kappa_peak - start.kappa) / l1;
    let half1 = Clothoid::try_new(start.pose, start.tangent, v_a, start.kappa, sigma1, l1).ok()?;
    let mid = half1.point_at(l1);
    let mid_h = half1.heading_at(l1);
    let v_m = normalize(cross(plane_n, mid_h));
    let sigma2 = (kappa_b - kappa_peak) / l2;
    let half2 = Clothoid::try_new(mid, mid_h, v_m, kappa_peak, sigma2, l2).ok()?;
    Some(ClothoidPair { half1, half2 })
}

fn pair_end(pair: &ClothoidPair) -> ([f64; 3], [f64; 3]) {
    let l2 = pair.half2.s_len();
    (pair.half2.point_at(l2), pair.half2.heading_at(l2))
}

fn residual(
    pair: &ClothoidPair,
    end: &Endpoint,
    e1: [f64; 3],
    e2: [f64; 3],
    plane_n: [f64; 3],
) -> [f64; 3] {
    let (pos, heading) = pair_end(pair);
    let dp = sub(pos, end.pose);
    [
        dot(dp, e1),
        dot(dp, e2),
        signed_angle(heading, end.tangent, plane_n),
    ]
}

fn converged(r: [f64; 3]) -> bool {
    (r[0] * r[0] + r[1] * r[1]).sqrt() < HERMITE_POS_TOL_MM && r[2].abs() < HERMITE_ANG_TOL_RAD
}

fn residual_norm(r: [f64; 3]) -> f64 {
    r[0] * r[0] + r[1] * r[1] + r[2] * r[2]
}

fn newton_pair(
    start: &Endpoint,
    end: &Endpoint,
    kappa_b: f64,
    plane_n: [f64; 3],
    e1: [f64; 3],
    e2: [f64; 3],
    mut x: [f64; 3],
) -> Option<ClothoidPair> {
    let mut pair = build_pair(start, kappa_b, plane_n, x[0], x[1], x[2])?;
    let mut r = residual(&pair, end, e1, e2, plane_n);
    let mut lambda = 1e-3;
    for _ in 0..HERMITE_MAX_ITERS {
        if converged(r) {
            return Some(pair);
        }
        let mut j = [[0.0; 3]; 3];
        for c in 0..3 {
            let h = 1e-7 + 1e-6 * x[c].abs();
            let mut xp = x;
            xp[c] += h;
            let pair_p = build_pair(start, kappa_b, plane_n, xp[0], xp[1], xp[2])?;
            let rp = residual(&pair_p, end, e1, e2, plane_n);
            for row in 0..3 {
                j[row][c] = (rp[row] - r[row]) / h;
            }
        }
        let mut jtj = [[0.0; 3]; 3];
        let mut jtr = [0.0; 3];
        for i in 0..3 {
            for k in 0..3 {
                jtj[i][k] = (0..3).map(|row| j[row][i] * j[row][k]).sum();
            }
            jtr[i] = (0..3).map(|row| j[row][i] * r[row]).sum();
        }

        let r0 = residual_norm(r);
        let mut stepped = false;
        for _ in 0..24 {
            let mut a = jtj;
            for (d, row) in a.iter_mut().enumerate() {
                row[d] += lambda * jtj[d][d].max(1e-12);
            }
            let Some(dx) = solve3(a, jtr) else {
                lambda *= 4.0;
                continue;
            };
            let xn = [x[0] - dx[0], x[1] - dx[1], x[2] - dx[2]];
            if xn[1] > 0.0 && xn[2] > 0.0 {
                if let Some(pn) = build_pair(start, kappa_b, plane_n, xn[0], xn[1], xn[2]) {
                    let rn = residual(&pn, end, e1, e2, plane_n);
                    if residual_norm(rn) < r0 {
                        x = xn;
                        pair = pn;
                        r = rn;
                        lambda = (lambda * 0.5).max(1e-12);
                        stepped = true;
                        break;
                    }
                }
            }
            lambda *= 4.0;
            if lambda > 1e12 {
                break;
            }
        }
        if !stepped {
            return None;
        }
    }
    None
}

pub(super) fn hermite_g2(
    p_a: [f64; 3],
    t_a: [f64; 3],
    kappa_a: f64,
    p_b: [f64; 3],
    t_b: [f64; 3],
    kappa_b: f64,
    plane_n: [f64; 3],
) -> Option<ClothoidPair> {
    let start = Endpoint {
        pose: p_a,
        tangent: t_a,
        kappa: kappa_a,
    };
    let end = Endpoint {
        pose: p_b,
        tangent: t_b,
        kappa: kappa_b,
    };
    let e1 = t_a;
    let e2 = normalize(cross(plane_n, e1));

    let theta = signed_angle(t_a, t_b, plane_n);
    let chord = dist(p_a, p_b).max(1e-9);
    let kp_turn = theta / chord - 0.5 * (kappa_a + kappa_b);

    let seeds = [
        [kp_turn, 0.5 * chord, 0.5 * chord],
        [kp_turn, chord, chord],
        [0.5 * (kappa_a + kappa_b), chord, chord],
        [kappa_a, 0.5 * chord, 0.5 * chord],
        [kappa_b, 0.5 * chord, 0.5 * chord],
        [-(kappa_a + kappa_b), chord, chord],
        [0.0, 2.0 * chord, 2.0 * chord],
    ];
    for seed in seeds {
        if let Some(pair) = newton_pair(&start, &end, kappa_b, plane_n, e1, e2, seed) {
            return Some(pair);
        }
    }
    None
}

pub(super) struct GeneralBlend {
    pub half1: Clothoid,
    pub half2: Clothoid,
    pub trim_in: f64,
    pub trim_out: f64,
}

#[derive(Clone, Copy)]
pub(super) struct Anchor {
    pub pose: [f64; 3],
    pub tangent: [f64; 3],
    pub kappa: f64,
}

const KAPPA_LINE_EPS: f64 = 1e-9;

fn rotate_in_plane(w: [f64; 3], ang: f64, n: [f64; 3]) -> [f64; 3] {
    add(scale(w, libm::cos(ang)), scale(cross(n, w), libm::sin(ang)))
}

fn contact(
    vertex: [f64; 3],
    tangent: [f64; 3],
    kappa: f64,
    signed_arclen: f64,
    plane_n: [f64; 3],
) -> ([f64; 3], [f64; 3]) {
    if kappa.abs() < KAPPA_LINE_EPS {
        return (madd(vertex, signed_arclen, tangent), tangent);
    }
    let center = madd(vertex, 1.0 / kappa, cross(plane_n, tangent));
    let ang = kappa * signed_arclen;
    let radial = sub(vertex, center);
    (
        add(center, rotate_in_plane(radial, ang, plane_n)),
        rotate_in_plane(tangent, ang, plane_n),
    )
}

pub(super) fn solve_general(
    anchor_in: Anchor,
    anchor_out: Anchor,
    apex: [f64; 3],
    plane_n: [f64; 3],
    delta: f64,
    budget_in: f64,
    budget_out: f64,
) -> Option<GeneralBlend> {
    if delta <= 0.0 {
        return None;
    }
    let rho_max = budget_in.min(budget_out);
    if rho_max <= super::BUDGET_EPS_MM {
        return None;
    }

    let eval = |rho: f64| -> Option<(ClothoidPair, f64)> {
        let (a, ta) = contact(
            anchor_in.pose,
            anchor_in.tangent,
            anchor_in.kappa,
            -rho,
            plane_n,
        );
        let (b, tb) = contact(
            anchor_out.pose,
            anchor_out.tangent,
            anchor_out.kappa,
            rho,
            plane_n,
        );
        let pair = hermite_g2(a, ta, anchor_in.kappa, b, tb, anchor_out.kappa, plane_n)?;
        let dev = dist(apex, pair.half2.start_pose);
        Some((pair, dev))
    };

    let samples = 48;
    let mut best: Option<(ClothoidPair, f64)> = None;
    for s in 1..=samples {
        let rho = rho_max * (s as f64) / (samples as f64);
        if let Some((pair, dev)) = eval(rho) {
            if dev <= delta {
                best = Some((pair, rho));
            }
        }
    }

    let (pair, rho) = best?;
    Some(GeneralBlend {
        half1: pair.half1,
        half2: pair.half2,
        trim_in: rho,
        trim_out: rho,
    })
}

#[cfg(test)]
mod tests;
