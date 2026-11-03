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
    if !(fit.radius.is_finite() && fit.radius > BUDGET_EPS_MM) {
        return false;
    }
    let Some(lines) = facets.iter().map(line_of).collect::<Option<Vec<_>>>() else {
        return false;
    };
    endpoint_anchored_chord_dev_within(&lines, &fit, tol)
}

/// Reconstruction anchors the circle through the run's endpoints, which is
/// stricter than a free fit — and ill-conditioned when the run nearly closes
/// on itself (tiny endpoint chord). Checking it during candidate growth
/// keeps every sealed run reconstructable instead of dissolving later.
fn endpoint_anchored_chord_dev_within(lines: &[&Line], fit: &CircleFit, tol: f64) -> bool {
    let t0 = lines[0].heading_at(0.0);
    let Some(v0) = turn_normal(t0, lines[1].heading_at(0.0)) else {
        return false;
    };
    let plane_normal = normalize(cross(t0, v0));
    let last = lines[lines.len() - 1];
    let p1 = last.point_at(last.s_len());
    minimax_through_endpoints(lines, (lines[0].start, p1), fit.origin, plane_normal)
        .is_some_and(|(origin, rho)| max_chord_dev(lines, origin, rho) <= tol)
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
    let origin = add(q0, add(scale(e1, cx), scale(e2, cy)));
    Some(CircleFit { origin, radius })
}

pub(super) struct CircleFit {
    pub origin: [f64; 3],
    pub radius: f64,
}

/// How far the facet chords stray from the circle: a chord point's distance
/// to it is `||p − o| − ρ|`, and `|p − o|` is convex along the chord, so the
/// worst sits at a vertex or at the chord's foot nearest the center — the
/// vertices' radial error and the sagitta together.
pub(in crate::fitter) fn max_chord_dev(lines: &[&Line], origin: [f64; 3], radius: f64) -> f64 {
    lines
        .iter()
        .map(|l| {
            let end = l.point_at(l.s_len());
            let along = dot(sub(origin, l.start), l.heading_at(0.0)).clamp(0.0, l.s_len());
            [l.start, end, l.point_at(along)]
                .into_iter()
                .map(|p| (norm(sub(p, origin)) - radius).abs())
                .fold(0.0, f64::max)
        })
        .fold(0.0, f64::max)
}

const MINIMAX_ITERS: usize = 40;
const INV_PHI: f64 = 0.618_033_988_749_894_8;

/// The minimum of `f` over `[lo, hi]` by golden-section search, never worse
/// than `seed`: the chord deviation of a one-parameter circle family is a
/// maximum of near-affine terms in absolute value, so unimodal in practice,
/// and the seed guards the cases where it is not.
fn golden_minimum(f: impl Fn(f64) -> f64, (lo, hi): (f64, f64), seed: f64) -> f64 {
    let (mut a, mut b) = (lo, hi);
    let mut c = b - INV_PHI * (b - a);
    let mut d = a + INV_PHI * (b - a);
    let (mut fc, mut fd) = (f(c), f(d));
    for _ in 0..MINIMAX_ITERS {
        if fc <= fd {
            b = d;
            d = c;
            fd = fc;
            c = b - INV_PHI * (b - a);
            fc = f(c);
        } else {
            a = c;
            c = d;
            fc = fd;
            d = a + INV_PHI * (b - a);
            fd = f(d);
        }
    }
    let (best, f_best) = if fc <= fd { (c, fc) } else { (d, fd) };
    if f_best <= f(seed) { best } else { seed }
}

/// The circle through both endpoints that keeps the facet chords closest.
/// Its bulge over the endpoint chord trades the vertices' radial error
/// against the chords' sagitta, so the minimax circle sits between the
/// vertices and the chord midpoints instead of through the vertices.
pub(in crate::fitter) fn minimax_through_endpoints(
    lines: &[&Line],
    (p0, p1): ([f64; 3], [f64; 3]),
    seed_origin: [f64; 3],
    plane_normal: [f64; 3],
) -> Option<([f64; 3], f64)> {
    let chord = sub(p1, p0);
    if norm(chord) < BUDGET_EPS_MM {
        return None;
    }
    let half = 0.5 * norm(chord);
    let mid = scale(add(p0, p1), 0.5);
    let perp = normalize(cross(plane_normal, chord));
    let circle = |bulge: f64| {
        let radius = (half * half + bulge * bulge) / (2.0 * bulge);
        (madd(mid, radius - bulge, perp), radius)
    };
    let offset = dot(sub(seed_origin, mid), perp);
    let seed = (half * half + offset * offset).sqrt() - offset;
    if !(seed > BUDGET_EPS_MM) {
        return None;
    }
    let dev = |bulge: f64| {
        let (origin, radius) = circle(bulge);
        max_chord_dev(lines, origin, radius)
    };
    let spread = 2.0 * dev(seed);
    let bracket = ((seed - spread).max(0.5 * seed), seed + spread);
    Some(circle(golden_minimum(dev, bracket, seed)))
}

/// The circle through one boundary vertex `p` that keeps the facet chords
/// closest, its center on the ray from `p` through the seed center.
pub(super) fn minimax_through_vertex(
    lines: &[&Line],
    p: [f64; 3],
    seed_origin: [f64; 3],
) -> Option<([f64; 3], f64)> {
    let toward = sub(seed_origin, p);
    let seed = norm(toward);
    if seed < BUDGET_EPS_MM {
        return None;
    }
    let direction = normalize(toward);
    let dev = |radius: f64| max_chord_dev(lines, madd(p, radius, direction), radius);
    let spread = 2.0 * dev(seed);
    let bracket = ((seed - spread).max(0.5 * seed), seed + spread);
    let radius = golden_minimum(dev, bracket, seed);
    Some((madd(p, radius, direction), radius))
}

/// The radius about a fixed center that keeps the facet chords closest: the
/// midrange of the chord points' distances to it.
pub(super) fn minimax_radius(lines: &[&Line], origin: [f64; 3]) -> f64 {
    let (nearest, farthest) = lines
        .iter()
        .flat_map(|l| {
            let along = dot(sub(origin, l.start), l.heading_at(0.0)).clamp(0.0, l.s_len());
            [l.start, l.point_at(l.s_len()), l.point_at(along)]
        })
        .map(|p| norm(sub(p, origin)))
        .fold((f64::INFINITY, 0.0_f64), |(lo, hi), d| {
            (lo.min(d), hi.max(d))
        });
    0.5 * (nearest + farthest)
}
