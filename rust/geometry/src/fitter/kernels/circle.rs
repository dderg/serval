use crate::frontend::Move;
use crate::path::lowering::PositionProfile;
use crate::path::{CurvatureProfile, Line};

use super::super::linalg::solve3;
use super::super::vec3::{add, cross, dot, madd, norm, normalize, scale, sub, turn_normal};
use super::super::{BUDGET_EPS_MM, CornerFitConfig, line_of};
use super::ease::EPMM_MIN;

const COPLANAR_TOL: f64 = 1e-6;

pub(super) fn grow_turning_band(moves: &[Move], start: usize, corner: CornerFitConfig) -> usize {
    let n = moves.len();
    let mut end = start;
    let mut plane: Option<[f64; 3]> = None;
    let mut turn_sign: Option<f64> = None;
    while end + 1 < n {
        let (la, lb) = match (line_of(&moves[end]), line_of(&moves[end + 1])) {
            (Some(a), Some(b)) => (a, b),
            _ => break,
        };
        let t_in = la.heading_at(la.s_len());
        let t_out = lb.heading_at(0.0);
        let theta = libm::acos(dot(t_in, t_out).clamp(-1.0, 1.0));
        if theta <= corner.theta_min_rad || theta >= corner.theta_max_rad {
            break;
        }
        let v = match turn_normal(t_in, t_out) {
            Some(v) => v,
            None => break,
        };
        let pn = normalize(cross(t_in, v));
        match plane {
            None => plane = Some(pn),
            Some(p) => {
                if dot(p, pn).abs() < 1.0 - COPLANAR_TOL {
                    break;
                }
            }
        }
        let sign = dot(pn, plane.unwrap()).signum();
        match turn_sign {
            None => turn_sign = Some(sign),
            Some(r) => {
                if r != sign {
                    break;
                }
            }
        }
        end += 1;
    }
    end
}

pub(in crate::fitter) fn arc_candidate(moves: &[Move], corner: CornerFitConfig, tol: f64) -> bool {
    if moves.len() < 2 {
        return true;
    }
    if !follower_band_ok(moves, corner.extrusion_ramp_rel_tol) {
        return false;
    }
    if grow_turning_band(moves, 0, corner) + 1 != moves.len() {
        return false;
    }
    moves.len() < 3 || cocircular(moves, tol)
}

/// Whether every facet's per-axis ratio sits inside a band the run's single
/// linear extrusion ramp can track within `rel_tol`: the signed spread must
/// not exceed `rel_tol` times the smallest magnitude in the window. Both
/// sides are monotone under append (the spread only grows, the floor only
/// shrinks), so a failed prefix stays failed — the growth loop's finality
/// invariant. An axis extruding on one facet but not another has floor zero
/// and only passes with zero spread, so travel and extruding facets never
/// share a run; ratios below `EPMM_MIN` snap to zero first so numeric dust
/// does not split an otherwise-travel run.
fn follower_band_ok(moves: &[Move], rel_tol: f64) -> bool {
    facet_axes(moves).into_iter().all(|axis| {
        let mut lo = f64::INFINITY;
        let mut hi = f64::NEG_INFINITY;
        let mut floor = f64::INFINITY;
        for m in moves {
            let raw = facet_ratio(m, axis);
            let r = if raw.abs() <= EPMM_MIN { 0.0 } else { raw };
            lo = lo.min(r);
            hi = hi.max(r);
            floor = floor.min(r.abs());
        }
        hi - lo <= rel_tol * floor
    })
}

pub(super) fn facet_axes(moves: &[Move]) -> Vec<usize> {
    let mut axes: Vec<usize> = Vec::new();
    for f in moves.iter().flat_map(|m| &m.segment.followers) {
        if !axes.contains(&f.axis_index) {
            axes.push(f.axis_index);
        }
    }
    axes
}

pub(super) fn facet_ratio(m: &Move, axis: usize) -> f64 {
    m.segment
        .followers
        .iter()
        .find(|f| f.axis_index == axis)
        .map_or(0.0, |f| {
            assert!(
                !f.is_ramped(),
                "arc-run facets and neighbors must carry constant follower ratios"
            );
            f.ratio
        })
}

pub(super) fn cocircular(facets: &[Move], tol: f64) -> bool {
    let Some(fit) = circle_fit(facets) else {
        return false;
    };
    if !(fit.residual <= tol && fit.radius.is_finite() && fit.radius > BUDGET_EPS_MM) {
        return false;
    }
    let Some(lines) = facets.iter().map(line_of).collect::<Option<Vec<_>>>() else {
        return false;
    };
    max_sagitta(&lines, fit.radius) <= tol
}

pub(super) fn max_sagitta(lines: &[&Line], radius: f64) -> f64 {
    lines
        .iter()
        .map(|l| {
            let half = 0.5 * l.s_len();
            if half >= radius {
                f64::INFINITY
            } else {
                radius - (radius * radius - half * half).sqrt()
            }
        })
        .fold(0.0, f64::max)
}

pub(super) fn circle_fit(facets: &[Move]) -> Option<CircleFit> {
    let lines: Vec<&Line> = facets.iter().map(line_of).collect::<Option<Vec<_>>>()?;
    if lines.len() < 2 {
        return None;
    }
    let t0 = lines[0].heading_at(0.0);
    let v0 = turn_normal(t0, lines[1].heading_at(0.0))?;
    let plane_normal = normalize(cross(t0, v0));
    fit_circle_through_vertices(&lines, plane_normal)
}

pub(super) fn fit_circle_through_vertices(
    lines: &[&Line],
    plane_normal: [f64; 3],
) -> Option<CircleFit> {
    let mut verts: Vec<[f64; 3]> = Vec::with_capacity(lines.len() + 1);
    verts.push(lines[0].start);
    for l in lines {
        verts.push(l.point_at(l.s_len()));
    }
    let q0 = verts[0];
    let d1 = sub(verts[1], q0);
    if norm(d1) < BUDGET_EPS_MM {
        return None;
    }
    let e1 = normalize(d1);
    let e2 = cross(plane_normal, e1);
    let pts: Vec<(f64, f64)> = verts
        .iter()
        .map(|v| {
            let d = sub(*v, q0);
            (dot(d, e1), dot(d, e2))
        })
        .collect();
    let n = pts.len() as f64;
    let (mut sx, mut sy, mut sxx, mut syy, mut sxy, mut sxz, mut syz, mut sz) =
        (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    for &(x, y) in &pts {
        let z = x * x + y * y;
        sx += x;
        sy += y;
        sxx += x * x;
        syy += y * y;
        sxy += x * y;
        sxz += x * z;
        syz += y * z;
        sz += z;
    }
    let m = [[sxx, sxy, sx], [sxy, syy, sy], [sx, sy, n]];
    let sol = solve3(m, [-sxz, -syz, -sz])?;
    let (cx, cy) = (-0.5 * sol[0], -0.5 * sol[1]);
    let r2 = cx * cx + cy * cy - sol[2];
    if !(r2 > 0.0) {
        return None;
    }
    let radius = r2.sqrt();
    let mut residual = 0.0_f64;
    for &(x, y) in &pts {
        residual = residual.max((((x - cx).powi(2) + (y - cy).powi(2)).sqrt() - radius).abs());
    }
    let origin = add(q0, add(scale(e1, cx), scale(e2, cy)));
    Some(CircleFit {
        origin,
        radius,
        residual,
    })
}

pub(super) struct CircleFit {
    pub origin: [f64; 3],
    pub radius: f64,
    pub residual: f64,
}

/// The circle of (at least) the given radius through both endpoints whose
/// center is nearest the least-squares fit. The radius grows to the half-chord
/// when the fit's radius falls short of reaching both endpoints — the caller's
/// deviation check still gates the enlarged circle against the tolerance.
pub(super) fn center_through_endpoints(
    p0: [f64; 3],
    p1: [f64; 3],
    radius: f64,
    ls_origin: [f64; 3],
    plane_normal: [f64; 3],
) -> Option<([f64; 3], f64)> {
    let chord = sub(p1, p0);
    let c = norm(chord);
    if c < BUDGET_EPS_MM {
        return None;
    }
    let radius = radius.max(0.5 * c);
    let mid = scale(add(p0, p1), 0.5);
    let half = (radius * radius - 0.25 * c * c).max(0.0).sqrt();
    let perp = normalize(cross(plane_normal, chord));
    let a = madd(mid, half, perp);
    let b = madd(mid, -half, perp);
    let center = if norm(sub(a, ls_origin)) <= norm(sub(b, ls_origin)) {
        a
    } else {
        b
    };
    Some((center, radius))
}

pub(super) fn max_radial_dev(lines: &[&Line], origin: [f64; 3], radius: f64) -> f64 {
    let mut worst = 0.0_f64;
    for l in lines {
        worst = worst.max((norm(sub(l.start, origin)) - radius).abs());
    }
    let last = lines[lines.len() - 1];
    worst.max((norm(sub(last.point_at(last.s_len()), origin)) - radius).abs())
}
