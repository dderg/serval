use std::f64::consts::{FRAC_PI_6, PI};

use crate::GeometryError;
use crate::frontend::Move;
use crate::path::lowering::PositionProfile;
use crate::path::{Arc, Clothoid, CurvatureProfile, Line};
use crate::segment::FollowerDemand;

use super::vec3::{add, cross, dot, madd, norm, normalize, scale, signed_angle, sub, turn_normal};
use super::{BUDGET_EPS_MM, CornerFitConfig, FitError, internal, line_of};

const COPLANAR_TOL: f64 = 1e-6;
const ANGLE_EPS_RAD: f64 = 1e-9;
const EASE_LEAD_MAX_RAD: f64 = FRAC_PI_6;
pub(super) const EPMM_MIN: f64 = 1e-9;
const EPMM_REL_TOL: f64 = 0.25;

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

pub(super) struct Neighbor {
    dir: [f64; 3],
    vertex: [f64; 3],
    length: f64,
    epmm: f64,
    followers: Vec<FollowerDemand>,
}

pub(super) fn neighbor(m: &Move, head: bool) -> Option<Neighbor> {
    let l = line_of(m)?;
    let (vertex, dir) = if head {
        (l.point_at(l.s_len()), l.heading_at(l.s_len()))
    } else {
        (l.point_at(0.0), l.heading_at(0.0))
    };
    Some(Neighbor {
        dir,
        vertex,
        length: l.s_len(),
        epmm: epmm(m),
        followers: m.segment.followers.clone(),
    })
}

pub(super) fn ease_run(
    recon: &mut Reconstruction,
    facets: &[Move],
    head: Option<&Neighbor>,
    tail: Option<&Neighbor>,
    tol: f64,
) -> Result<(), FitError> {
    let run_epmm = epmm(&facets[0]);
    let line_no = facets[0].source.start_line;
    let verts = run_vertices(facets);

    let Some(fit) = joint_refit(&recon.arc, head, tail, &verts, run_epmm, tol, line_no)? else {
        return Ok(());
    };

    recon.arc = fit.arc;
    recon.up = fit.head.into_iter().collect();
    recon.down = fit.tail.into_iter().collect();
    recon.head_line_trim = fit.head_line_trim;
    recon.tail_line_trim = fit.tail_line_trim;
    let lines: Vec<&Line> = facets
        .iter()
        .map(line_of)
        .collect::<Option<Vec<_>>>()
        .expect("run facets are lines");
    recon.followers = run_followers(
        facets,
        &lines,
        recon.head_consumption,
        recon.tail_consumption,
        arc_len(&recon.arc),
    );
    recon.up_followers = match (recon.up.first(), head) {
        (Some(clo), Some(nbr)) => {
            scale_followers(&nbr.followers, recon.head_line_trim / clo.s_len())
        }
        _ => Vec::new(),
    };
    recon.down_followers = match (recon.down.first(), tail) {
        (Some(clo), Some(nbr)) => {
            scale_followers(&nbr.followers, recon.tail_line_trim / clo.s_len())
        }
        _ => Vec::new(),
    };
    Ok(())
}

fn scale_followers(f: &[FollowerDemand], s: f64) -> Vec<FollowerDemand> {
    f.iter()
        .map(|x| FollowerDemand {
            axis_index: x.axis_index,
            ratio: x.ratio * s,
        })
        .collect()
}

struct JointFit {
    arc: Arc,
    head: Option<Clothoid>,
    tail: Option<Clothoid>,
    head_line_trim: f64,
    tail_line_trim: f64,
}

struct EndPlan {
    normal: [f64; 3],
    spiral_dir: [f64; 3],
    curve_sgn: f64,
    turn: f64,
    vertex: [f64; 3],
    neighbor_dir: [f64; 3],
    neighbor_len: f64,
}

#[allow(clippy::too_many_arguments)]
fn joint_refit(
    arc: &Arc,
    head: Option<&Neighbor>,
    tail: Option<&Neighbor>,
    verts: &[[f64; 3]],
    run_epmm: f64,
    tol: f64,
    line_no: u32,
) -> Result<Option<JointFit>, FitError> {
    let pn = normalize(cross(arc.u, arc.v));
    let sgn = arc.sweep.signum();
    let (o0, r0) = (arc.origin, arc.radius);
    let t_start = arc.heading_at(0.0);
    let t_end = arc.heading_at(arc.s_len());

    let head_plan = head.and_then(|n| {
        let turn = signed_angle(n.dir, t_start, pn);
        end_plan(n, turn, sgn, n.dir, sgn, run_epmm, pn, o0)
    });
    let tail_plan = tail.and_then(|n| {
        let turn = signed_angle(t_end, n.dir, pn);
        end_plan(n, turn, sgn, scale(n.dir, -1.0), -sgn, run_epmm, pn, o0)
    });
    if head_plan.is_none() && tail_plan.is_none() {
        return Ok(None);
    }

    let Some((origin, radius)) = choose_circle(
        head_plan.as_ref(),
        tail_plan.as_ref(),
        o0,
        r0,
        arc.u,
        arc.v,
        verts,
        tol,
    ) else {
        return Ok(None);
    };

    let mut head_clo = None;
    let mut tail_clo = None;
    let mut head_trim = 0.0;
    let mut tail_trim = 0.0;
    let b_head = match &head_plan {
        Some(p) => {
            let (clo, b, trim) = build_spiral(origin, radius, p, pn)
                .map_err(internal(line_no))?
                .ok_or(())
                .map_err(|()| FitError::Internal {
                    line_no,
                    source: GeometryError::DegenerateClothoid {
                        reason: "head transition spiral did not close",
                    },
                })?;
            if !within_line(trim, p.neighbor_len) || spiral_dev(&clo, p, o0, r0, pn) > tol {
                return Ok(None);
            }
            head_clo = Some(clo);
            head_trim = trim;
            b
        }
        None => project_to_circle(origin, radius, verts[0]),
    };
    let b_tail = match &tail_plan {
        Some(p) => {
            let (clo, b, trim) = build_spiral(origin, radius, p, pn)
                .map_err(internal(line_no))?
                .ok_or(())
                .map_err(|()| FitError::Internal {
                    line_no,
                    source: GeometryError::DegenerateClothoid {
                        reason: "tail transition spiral did not close",
                    },
                })?;
            if !within_line(trim, p.neighbor_len) || spiral_dev(&clo, p, o0, r0, pn) > tol {
                return Ok(None);
            }
            tail_clo = Some(
                reverse_clothoid(&clo)
                    .ok_or(())
                    .map_err(|()| FitError::Internal {
                        line_no,
                        source: GeometryError::DegenerateClothoid {
                            reason: "tail spiral reverse failed",
                        },
                    })?,
            );
            tail_trim = trim;
            b
        }
        None => project_to_circle(origin, radius, verts[verts.len() - 1]),
    };

    if interior_residual(&origin, radius, verts) > tol {
        return Ok(None);
    }

    let arc = build_arc(origin, radius, b_head, b_tail, pn, sgn).map_err(internal(line_no))?;
    Ok(Some(JointFit {
        arc,
        head: head_clo,
        tail: tail_clo,
        head_line_trim: head_trim,
        tail_line_trim: tail_trim,
    }))
}

#[allow(clippy::too_many_arguments)]
fn end_plan(
    n: &Neighbor,
    turn: f64,
    sweep_sign: f64,
    spiral_dir: [f64; 3],
    curve_sgn: f64,
    run_epmm: f64,
    pn: [f64; 3],
    o0: [f64; 3],
) -> Option<EndPlan> {
    if run_epmm > EPMM_MIN && (n.epmm - run_epmm).abs() > EPMM_REL_TOL * run_epmm {
        return None;
    }
    if turn.signum() != sweep_sign {
        return None;
    }
    let phi = turn.abs();
    if !(ANGLE_EPS_RAD..EASE_LEAD_MAX_RAD).contains(&phi) {
        return None;
    }
    let normal = scale(normalize(cross(pn, spiral_dir)), curve_sgn);
    if dot(normal, sub(o0, n.vertex)) <= 0.0 {
        return None;
    }
    Some(EndPlan {
        normal,
        spiral_dir,
        curve_sgn,
        turn: phi,
        vertex: n.vertex,
        neighbor_dir: n.dir,
        neighbor_len: n.length,
    })
}

fn spiral_center_dist(
    radius: f64,
    line_dir: [f64; 3],
    curve_sgn: f64,
    phi: f64,
    pn: [f64; 3],
) -> Option<f64> {
    let length = 2.0 * radius * phi;
    if !(length.is_finite() && length > BUDGET_EPS_MM) {
        return None;
    }
    let sigma = curve_sgn / (radius * length);
    let v = scale(normalize(cross(pn, line_dir)), curve_sgn);
    let probe = Clothoid::try_new([0.0; 3], line_dir, v, 0.0, sigma, length).ok()?;
    let end = probe.point_at(length);
    let t_end = probe.heading_at(length);
    let center = add(end, scale(cross(pn, t_end), radius * curve_sgn));
    Some(dot(center, v))
}

fn build_spiral(
    origin: [f64; 3],
    radius: f64,
    p: &EndPlan,
    pn: [f64; 3],
) -> Result<Option<(Clothoid, [f64; 3], f64)>, GeometryError> {
    let length = 2.0 * radius * p.turn;
    if !(length.is_finite() && length > BUDGET_EPS_MM) {
        return Ok(None);
    }
    let sigma = p.curve_sgn / (radius * length);
    let v = scale(normalize(cross(pn, p.spiral_dir)), p.curve_sgn);
    let probe = Clothoid::try_new([0.0; 3], p.spiral_dir, v, 0.0, sigma, length)?;
    let end_off = probe.point_at(length);
    let t_end = probe.heading_at(length);
    let b = sub(origin, scale(cross(pn, t_end), radius * p.curve_sgn));
    let a = sub(b, end_off);
    let line_trim = dot(sub(p.vertex, a), p.spiral_dir);
    let clo = Clothoid::try_new(a, p.spiral_dir, v, 0.0, sigma, length)?;
    Ok(Some((clo, b, line_trim)))
}

fn within_line(trim: f64, length: f64) -> bool {
    trim > -BUDGET_EPS_MM && trim < 0.9 * length
}

fn project_to_circle(origin: [f64; 3], radius: f64, p: [f64; 3]) -> [f64; 3] {
    add(origin, scale(normalize(sub(p, origin)), radius))
}

fn max_radial_dev(lines: &[&Line], origin: [f64; 3], radius: f64) -> f64 {
    let mut worst = 0.0_f64;
    for l in lines {
        worst = worst.max((norm(sub(l.start, origin)) - radius).abs());
    }
    let last = lines[lines.len() - 1];
    worst.max((norm(sub(last.point_at(last.s_len()), origin)) - radius).abs())
}

fn center_through_endpoints(
    p0: [f64; 3],
    p1: [f64; 3],
    radius: f64,
    ls_origin: [f64; 3],
    plane_normal: [f64; 3],
) -> Option<[f64; 3]> {
    let chord = sub(p1, p0);
    let c = norm(chord);
    if c < BUDGET_EPS_MM || radius < 0.5 * c {
        return None;
    }
    let mid = scale(add(p0, p1), 0.5);
    let half = (radius * radius - 0.25 * c * c).sqrt();
    let perp = normalize(cross(plane_normal, chord));
    let a = madd(mid, half, perp);
    let b = madd(mid, -half, perp);
    Some(if norm(sub(a, ls_origin)) <= norm(sub(b, ls_origin)) {
        a
    } else {
        b
    })
}

fn spiral_dev(clo: &Clothoid, p: &EndPlan, o0: [f64; 3], r0: f64, pn: [f64; 3]) -> f64 {
    let n_line = normalize(cross(pn, p.neighbor_dir));
    let mut m = 0.0_f64;
    let samples = 16usize;
    for k in 0..=samples {
        let q = clo.point_at(clo.s_len() * k as f64 / samples as f64);
        let off_line = dot(sub(q, p.vertex), n_line).abs();
        let off_arc = (norm(sub(q, o0)) - r0).abs();
        m = m.max(off_line.min(off_arc));
    }
    m
}

fn interior_residual(origin: &[f64; 3], radius: f64, verts: &[[f64; 3]]) -> f64 {
    if verts.len() <= 2 {
        return 0.0;
    }
    verts[1..verts.len() - 1]
        .iter()
        .map(|v| (norm(sub(*v, *origin)) - radius).abs())
        .fold(0.0, f64::max)
}

#[allow(clippy::too_many_arguments)]
fn choose_circle(
    head: Option<&EndPlan>,
    tail: Option<&EndPlan>,
    o0: [f64; 3],
    r0: f64,
    u: [f64; 3],
    v: [f64; 3],
    verts: &[[f64; 3]],
    tol: f64,
) -> Option<([f64; 3], f64)> {
    let pn = normalize(cross(u, v));
    let radius = balanced_radius(r0, verts);
    match (head, tail) {
        (Some(h), Some(t)) => {
            let dh = spiral_center_dist(radius, h.spiral_dir, h.curve_sgn, h.turn, pn)?;
            let dt = spiral_center_dist(radius, t.spiral_dir, t.curve_sgn, t.turn, pn)?;
            let origin = solve_center(h.normal, h.vertex, dh, t.normal, t.vertex, dt, o0, u, v)?;
            Some((origin, radius))
        }
        (Some(e), None) => one_eased_center(e, verts[verts.len() - 1], radius, o0, pn, verts, tol),
        (None, Some(e)) => one_eased_center(e, verts[0], radius, o0, pn, verts, tol),
        (None, None) => None,
    }
}

#[allow(clippy::too_many_arguments)]
fn one_eased_center(
    e: &EndPlan,
    bare_vertex: [f64; 3],
    radius: f64,
    o0: [f64; 3],
    pn: [f64; 3],
    verts: &[[f64; 3]],
    tol: f64,
) -> Option<([f64; 3], f64)> {
    let d = spiral_center_dist(radius, e.spiral_dir, e.curve_sgn, e.turn, pn)?;
    let fallback = add(o0, scale(e.normal, d - dot(sub(o0, e.vertex), e.normal)));

    let base = add(e.vertex, scale(e.normal, d));
    let t = normalize(cross(pn, e.normal));
    let w = sub(base, bare_vertex);
    let wt = dot(w, t);
    let disc = radius * radius - (dot(w, w) - wt * wt);
    if disc < 0.0 {
        return Some((fallback, radius));
    }
    let sq = disc.sqrt();
    let c1 = madd(base, -wt + sq, t);
    let c2 = madd(base, -wt - sq, t);
    let anchored = if norm(sub(c1, o0)) <= norm(sub(c2, o0)) {
        c1
    } else {
        c2
    };
    if interior_residual(&anchored, radius, verts) <= tol {
        Some((anchored, radius))
    } else {
        Some((fallback, radius))
    }
}

fn balanced_radius(r0: f64, verts: &[[f64; 3]]) -> f64 {
    let mut max_sag = 0.0_f64;
    for w in verts.windows(2) {
        let half = 0.5 * norm(sub(w[1], w[0]));
        if half < r0 {
            max_sag = max_sag.max(r0 - (r0 * r0 - half * half).sqrt());
        }
    }
    r0 - 0.5 * max_sag
}

#[allow(clippy::too_many_arguments)]
fn solve_center(
    nh: [f64; 3],
    ph: [f64; 3],
    dh: f64,
    nt: [f64; 3],
    pt: [f64; 3],
    dt: f64,
    o0: [f64; 3],
    u: [f64; 3],
    v: [f64; 3],
) -> Option<[f64; 3]> {
    let a = [[dot(nh, u), dot(nh, v)], [dot(nt, u), dot(nt, v)]];
    let b = [dot(nh, sub(ph, o0)) + dh, dot(nt, sub(pt, o0)) + dt];
    let det = a[0][0] * a[1][1] - a[0][1] * a[1][0];
    if det.abs() < 1e-12 {
        return None;
    }
    let x = (b[0] * a[1][1] - b[1] * a[0][1]) / det;
    let y = (a[0][0] * b[1] - a[1][0] * b[0]) / det;
    Some(add(o0, add(scale(u, x), scale(v, y))))
}

fn build_arc(
    origin: [f64; 3],
    radius: f64,
    b_head: [f64; 3],
    b_tail: [f64; 3],
    pn: [f64; 3],
    sgn: f64,
) -> Result<Arc, GeometryError> {
    let u = normalize(sub(b_head, origin));
    let v = cross(pn, u);
    let mut sweep = signed_angle(u, normalize(sub(b_tail, origin)), pn);
    if sgn > 0.0 && sweep < 0.0 {
        sweep += 2.0 * PI;
    }
    if sgn < 0.0 && sweep > 0.0 {
        sweep -= 2.0 * PI;
    }
    Arc::try_new(origin, u, v, radius, 0.0, sweep)
}

pub(super) fn run_vertices(facets: &[Move]) -> Vec<[f64; 3]> {
    let lines: Vec<&Line> = facets
        .iter()
        .map(line_of)
        .collect::<Option<Vec<_>>>()
        .expect("run facets are lines");
    let mut verts = Vec::with_capacity(lines.len() + 1);
    verts.push(lines[0].start);
    for l in &lines {
        verts.push(l.point_at(l.s_len()));
    }
    verts
}

pub(super) fn reverse_clothoid(c: &Clothoid) -> Option<Clothoid> {
    let l = c.s_len();
    let pn = normalize(cross(c.u, c.v));
    let start = c.point_at(l);
    let u = scale(c.heading_at(l), -1.0);
    let v = cross(pn, u);
    let kappa_0 = -(c.kappa_0 + c.sigma * l);
    Clothoid::try_new(start, u, v, kappa_0, c.sigma, l).ok()
}

pub(super) fn epmm(m: &Move) -> f64 {
    m.segment.followers.iter().map(|f| f.ratio.abs()).sum()
}

pub(super) fn grow_turning_band(
    moves: &[Move],
    start: usize,
    corner: CornerFitConfig,
    gate_epmm: bool,
) -> usize {
    let n = moves.len();
    let mut end = start;
    let mut plane: Option<[f64; 3]> = None;
    let mut turn_sign: Option<f64> = None;
    let e0 = epmm(&moves[start]);
    while end + 1 < n {
        if gate_epmm {
            let e_next = epmm(&moves[end + 1]);
            if e_next < EPMM_MIN || (e_next - e0).abs() > EPMM_REL_TOL * e0 {
                break;
            }
        }
        let (la, lb) = match (line_of(&moves[end]), line_of(&moves[end + 1])) {
            (Some(a), Some(b)) => (a, b),
            _ => break,
        };
        let t_in = la.heading_at(la.s_len());
        let t_out = lb.heading_at(0.0);
        let theta = dot(t_in, t_out).clamp(-1.0, 1.0).acos();
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

pub(super) fn grow_cocircular_span(
    moves: &[Move],
    start: usize,
    band_end: usize,
    tol: f64,
) -> usize {
    let mut best = start;
    let mut end = start + 2;
    while end <= band_end {
        if cocircular(&moves[start..=end], tol) {
            best = end;
            end += 1;
        } else {
            break;
        }
    }
    best
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

fn max_sagitta(lines: &[&Line], radius: f64) -> f64 {
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

fn fit_circle_through_vertices(lines: &[&Line], plane_normal: [f64; 3]) -> Option<CircleFit> {
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

pub(super) fn reconstruct(facets: &[Move], tol: f64) -> Result<Option<Reconstruction>, FitError> {
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
    let fit = match fit_circle_through_vertices(&lines, plane_normal) {
        Some(f) => f,
        None => return Ok(None),
    };
    if fit.residual > tol {
        return Ok(None);
    }
    let (mut origin, rho) = (fit.origin, fit.radius);
    if !(rho.is_finite() && rho > BUDGET_EPS_MM) {
        return Ok(None);
    }
    if max_sagitta(&lines, rho) > tol {
        return Ok(None);
    }

    let last = lines[lines.len() - 1];
    let p1 = last.point_at(last.s_len());
    if let Some(anchored) = center_through_endpoints(lines[0].start, p1, rho, origin, plane_normal)
    {
        if max_radial_dev(&lines, anchored, rho) <= tol {
            origin = anchored;
        }
    }

    let u = normalize(sub(lines[0].start, origin));
    let v = cross(plane_normal, u);
    let mut sweep = 0.0_f64;
    let mut prev = sub(lines[0].start, origin);
    for l in &lines {
        let cur = sub(l.point_at(l.s_len()), origin);
        sweep += dot(cross(prev, cur), plane_normal).atan2(dot(prev, cur));
        prev = cur;
    }
    if !(sweep.is_finite() && sweep.abs() > ANGLE_EPS_RAD) {
        return Ok(None);
    }
    let arc = Arc::try_new(origin, u, v, rho, 0.0, sweep).map_err(internal(line_no))?;

    let head_consumption = lines[0].s_len();
    let tail_consumption = lines[lines.len() - 1].s_len();
    let recon_len = arc_len(&arc);
    let followers = run_followers(
        facets,
        &lines,
        head_consumption,
        tail_consumption,
        recon_len,
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

pub(super) struct CircleFit {
    pub origin: [f64; 3],
    pub radius: f64,
    pub residual: f64,
}

fn run_followers(
    facets: &[Move],
    lines: &[&Line],
    head_consumption: f64,
    tail_consumption: f64,
    recon_len: f64,
) -> Vec<FollowerDemand> {
    let mut totals: Vec<(usize, f64)> = Vec::new();
    let last = lines.len() - 1;
    for (i, m) in facets.iter().enumerate() {
        let covered = if i == 0 {
            head_consumption
        } else if i == last {
            tail_consumption
        } else {
            lines[i].s_len()
        };
        for f in &m.segment.followers {
            let delta_e = f.ratio * covered;
            match totals.iter_mut().find(|(a, _)| *a == f.axis_index) {
                Some(entry) => entry.1 += delta_e,
                None => totals.push((f.axis_index, delta_e)),
            }
        }
    }
    totals
        .into_iter()
        .filter(|(_, e)| e.abs() > 1e-12)
        .map(|(axis_index, e)| FollowerDemand {
            axis_index,
            ratio: e / recon_len,
        })
        .collect()
}

pub(super) fn arc_len(arc: &Arc) -> f64 {
    arc.radius * arc.sweep.abs()
}

fn solve3(m: [[f64; 3]; 3], b: [f64; 3]) -> Option<[f64; 3]> {
    let det = det3(m);
    if det.abs() < 1e-18 {
        return None;
    }
    let mut out = [0.0_f64; 3];
    for col in 0..3 {
        let mut mc = m;
        for row in 0..3 {
            mc[row][col] = b[row];
        }
        out[col] = det3(mc) / det;
    }
    Some(out)
}

fn det3(m: [[f64; 3]; 3]) -> f64 {
    m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
        - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
        + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
}
