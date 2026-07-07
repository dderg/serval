use std::f64::consts::{FRAC_PI_6, PI};

use crate::GeometryError;
use crate::frontend::Move;
use crate::path::lowering::PositionProfile;
use crate::path::{Arc, Clothoid, CurvatureProfile, Line};
use crate::segment::FollowerDemand;

use super::super::vec3::{add, cross, dot, madd, norm, normalize, scale, signed_angle, sub};
use super::super::{BUDGET_EPS_MM, FitError, internal, line_of};
use super::Reconstruction;
use super::follower::{arc_len, construct_followers};

const ANGLE_EPS_RAD: f64 = 1e-9;
pub(super) const EPMM_MIN: f64 = 1e-9;
const EPMM_REL_TOL: f64 = 0.25;
const EASE_LEAD_MAX_RAD: f64 = FRAC_PI_6;

pub(in crate::fitter) struct Neighbor {
    dir: [f64; 3],
    vertex: [f64; 3],
    length: f64,
    epmm: f64,
    followers: Vec<FollowerDemand>,
}

pub(in crate::fitter) fn neighbor(m: &Move, head: bool) -> Option<Neighbor> {
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

pub(in crate::fitter) fn ease_run(
    recon: &mut Reconstruction,
    facets: &[Move],
    head: Option<&Neighbor>,
    tail: Option<&Neighbor>,
    tol: f64,
) -> Result<(), FitError> {
    let head_epmm = epmm(&facets[0]);
    let tail_epmm = epmm(facets.last().expect("run has facets"));
    let line_no = facets[0].source.start_line;
    let verts = run_vertices(facets);
    let pn = normalize(cross(recon.arc.u, recon.arc.v));
    let sgn = recon.arc.sweep.signum();
    let o0 = recon.arc.origin;
    let r0 = recon.arc.radius;

    let head_max = head.and_then(|n| ease_plan(n, n.dir, sgn, head_epmm, pn, o0, r0));
    let tail_max = tail.and_then(|n| ease_plan(n, scale(n.dir, -1.0), -sgn, tail_epmm, pn, o0, r0));
    if head_max.is_none() && tail_max.is_none() {
        return Ok(());
    }

    let head_len = head.map_or(0.0, |n| n.length);
    let tail_len = tail.map_or(0.0, |n| n.length);
    let mut fit = None;
    'search: for &(use_head, use_tail) in &[(true, true), (true, false), (false, true)] {
        let hp0 = if use_head { head_max } else { None };
        let tp0 = if use_tail { tail_max } else { None };
        if hp0.is_none() && tp0.is_none() {
            continue;
        }
        if use_head && use_tail && (head_max.is_none() || tail_max.is_none()) {
            continue;
        }
        for &shrink in &[1.0, 0.5, 0.25, 0.125] {
            let hp = hp0.map(|p| EndPlan {
                phi: p.phi * shrink,
                ..p
            });
            let tp = tp0.map(|p| EndPlan {
                phi: p.phi * shrink,
                ..p
            });
            if hp.map_or(true, |p| p.phi < ANGLE_EPS_RAD)
                && tp.map_or(true, |p| p.phi < ANGLE_EPS_RAD)
            {
                break;
            }
            let attempt = try_ease(
                hp.as_ref(),
                tp.as_ref(),
                head_len,
                tail_len,
                o0,
                r0,
                recon.arc.u,
                recon.arc.v,
                &verts,
                tol,
                pn,
            )
            .map_err(internal(line_no))?;
            if let Some(f) = attempt {
                fit = Some(f);
                break 'search;
            }
        }
    }

    let Some(ease) = fit else {
        return Ok(());
    };

    let b_head = match &ease.head {
        Some(s) => s.b,
        None => project_to_circle(ease.origin, ease.radius, verts[0]),
    };
    let b_tail = match &ease.tail {
        Some(s) => s.b,
        None => project_to_circle(ease.origin, ease.radius, *verts.last().unwrap()),
    };

    recon.arc =
        build_arc(ease.origin, ease.radius, b_head, b_tail, pn, sgn).map_err(internal(line_no))?;

    if let Some(s) = &ease.head {
        recon.head_line_trim = s.trim;
        recon.up = vec![s.clo.clone()];
    }
    if let Some(s) = &ease.tail {
        let reversed = reverse_clothoid(&s.clo).ok_or(FitError::Internal {
            line_no,
            source: GeometryError::DegenerateClothoid {
                reason: "tail spiral reverse failed",
            },
        })?;
        recon.tail_line_trim = s.trim;
        recon.down = vec![reversed];
    }

    let lines: Vec<&Line> = facets
        .iter()
        .map(line_of)
        .collect::<Option<Vec<_>>>()
        .expect("run facets are lines");
    let head_end = ease.head.as_ref().zip(head).map(|(s, n)| EasedEnd {
        neighbor_followers: &n.followers,
        spiral_len: s.clo.s_len(),
        line_trim: s.trim,
    });
    let tail_end = ease.tail.as_ref().zip(tail).map(|(s, n)| EasedEnd {
        neighbor_followers: &n.followers,
        spiral_len: s.clo.s_len(),
        line_trim: s.trim,
    });
    let (up_followers, arc_followers, down_followers) = construct_followers(
        facets,
        &lines,
        recon.head_consumption,
        recon.tail_consumption,
        arc_len(&recon.arc),
        head_end.as_ref(),
        tail_end.as_ref(),
    );
    recon.up_followers = up_followers;
    recon.followers = arc_followers;
    recon.down_followers = down_followers;

    Ok(())
}

/// An end of the reconstruction that eases into its neighbor line through a
/// spiral: the neighbor's demands, the spiral's arc length, and the line
/// footage the spiral replaced.
pub(in crate::fitter::kernels) struct EasedEnd<'a> {
    pub neighbor_followers: &'a [FollowerDemand],
    pub spiral_len: f64,
    pub line_trim: f64,
}

#[derive(Clone, Copy)]
struct EndPlan {
    spiral_dir: [f64; 3],
    curve_sgn: f64,
    phi: f64,
    vertex: [f64; 3],
}

struct SpiralFit {
    clo: Clothoid,
    b: [f64; 3],
    trim: f64,
}

struct EaseFit {
    head: Option<SpiralFit>,
    tail: Option<SpiralFit>,
    origin: [f64; 3],
    radius: f64,
}

/// Solve one easing configuration end to end: refit the circle for the given
/// end plans, then build every planned spiral and validate its line trim. A
/// planned end whose spiral is degenerate or over-claims its neighbor rejects
/// the whole attempt — the caller retries with a smaller lead angle or fewer
/// eased ends — because the refit circle is only valid together with the
/// spirals it was solved for: keeping it while dropping a spiral strands that
/// arc end off the run's vertex.
#[allow(clippy::too_many_arguments)]
fn try_ease(
    hp: Option<&EndPlan>,
    tp: Option<&EndPlan>,
    head_len: f64,
    tail_len: f64,
    o0: [f64; 3],
    r0: f64,
    u: [f64; 3],
    v: [f64; 3],
    verts: &[[f64; 3]],
    tol: f64,
    pn: [f64; 3],
) -> Result<Option<EaseFit>, GeometryError> {
    let Some((origin, radius)) = ease_circle(hp, tp, o0, r0, u, v, verts, tol) else {
        return Ok(None);
    };
    let head = match hp {
        Some(p) => match build_spiral(origin, radius, p, pn)? {
            Some((clo, b, trim)) if within_line(trim, head_len) => Some(SpiralFit { clo, b, trim }),
            _ => return Ok(None),
        },
        None => None,
    };
    let tail = match tp {
        Some(p) => match build_spiral(origin, radius, p, pn)? {
            Some((clo, b, trim)) if within_line(trim, tail_len) => Some(SpiralFit { clo, b, trim }),
            _ => return Ok(None),
        },
        None => None,
    };
    Ok(Some(EaseFit {
        head,
        tail,
        origin,
        radius,
    }))
}

fn max_ease_angle(radius: f64, neighbor_len: f64) -> f64 {
    (0.45 * neighbor_len / radius).min(EASE_LEAD_MAX_RAD)
}

fn ease_plan(
    n: &Neighbor,
    spiral_dir: [f64; 3],
    curve_sgn: f64,
    run_epmm: f64,
    pn: [f64; 3],
    origin: [f64; 3],
    radius: f64,
) -> Option<EndPlan> {
    if run_epmm > EPMM_MIN && (n.epmm - run_epmm).abs() > EPMM_REL_TOL * run_epmm {
        return None;
    }
    let phi = max_ease_angle(radius, n.length);
    if phi < ANGLE_EPS_RAD {
        return None;
    }
    let normal = scale(normalize(cross(pn, spiral_dir)), curve_sgn);
    if dot(normal, sub(origin, n.vertex)) <= 0.0 {
        return None;
    }
    Some(EndPlan {
        spiral_dir,
        curve_sgn,
        phi,
        vertex: n.vertex,
    })
}

fn spiral_center_dist(
    radius: f64,
    line_dir: [f64; 3],
    curve_sgn: f64,
    phi: f64,
    pn: [f64; 3],
) -> Option<f64> {
    let g = probe_geometry(radius, line_dir, curve_sgn, phi, pn)?;
    let normal = scale(g.v, curve_sgn);
    Some(dot(g.center, normal))
}

#[allow(clippy::too_many_arguments)]
fn ease_circle(
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
    let radius = r0;
    match (head, tail) {
        (Some(h), Some(t)) => {
            let dh = spiral_center_dist(radius, h.spiral_dir, h.curve_sgn, h.phi, pn)?;
            let dt = spiral_center_dist(radius, t.spiral_dir, t.curve_sgn, t.phi, pn)?;
            let nh = scale(normalize(cross(pn, h.spiral_dir)), h.curve_sgn);
            let nt = scale(normalize(cross(pn, t.spiral_dir)), t.curve_sgn);
            let origin = solve_center(nh, h.vertex, dh, nt, t.vertex, dt, o0, u, v)?;
            if interior_residual(&origin, radius, verts) <= tol {
                Some((origin, radius))
            } else {
                None
            }
        }
        (Some(e), None) => one_end_center(e, *verts.last().unwrap(), radius, o0, pn, verts, tol),
        (None, Some(e)) => one_end_center(e, verts[0], radius, o0, pn, verts, tol),
        (None, None) => None,
    }
}

#[allow(clippy::too_many_arguments)]
fn one_end_center(
    e: &EndPlan,
    bare_vertex: [f64; 3],
    radius: f64,
    o0: [f64; 3],
    pn: [f64; 3],
    verts: &[[f64; 3]],
    tol: f64,
) -> Option<([f64; 3], f64)> {
    let d = spiral_center_dist(radius, e.spiral_dir, e.curve_sgn, e.phi, pn)?;
    let normal = scale(normalize(cross(pn, e.spiral_dir)), e.curve_sgn);
    let base = add(e.vertex, scale(normal, d));
    let t = normalize(cross(pn, normal));
    let w = sub(base, bare_vertex);
    let wt = dot(w, t);
    let disc = radius * radius - (dot(w, w) - wt * wt);
    if disc >= 0.0 {
        let sq = disc.sqrt();
        let c1 = madd(base, -wt + sq, t);
        let c2 = madd(base, -wt - sq, t);
        let anchored = if norm(sub(c1, o0)) <= norm(sub(c2, o0)) {
            c1
        } else {
            c2
        };
        if interior_residual(&anchored, radius, verts) <= tol {
            return Some((anchored, radius));
        }
    }
    let shift = d - dot(sub(o0, e.vertex), normal);
    let fallback = add(o0, scale(normal, shift));
    let bare_contact = (norm(sub(fallback, bare_vertex)) - radius).abs() <= BUDGET_EPS_MM;
    if bare_contact && interior_residual(&fallback, radius, verts) <= tol {
        Some((fallback, radius))
    } else {
        None
    }
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

fn interior_residual(origin: &[f64; 3], radius: f64, verts: &[[f64; 3]]) -> f64 {
    if verts.len() <= 2 {
        return 0.0;
    }
    verts[1..verts.len() - 1]
        .iter()
        .map(|v| (norm(sub(*v, *origin)) - radius).abs())
        .fold(0.0, f64::max)
}

struct ProbeGeometry {
    sigma: f64,
    length: f64,
    v: [f64; 3],
    end: [f64; 3],
    center: [f64; 3],
}

fn probe_geometry(
    radius: f64,
    line_dir: [f64; 3],
    curve_sgn: f64,
    phi: f64,
    pn: [f64; 3],
) -> Option<ProbeGeometry> {
    let length = 2.0 * radius * phi;
    if !(length.is_finite() && length > BUDGET_EPS_MM) {
        return None;
    }
    let sigma = curve_sgn / (radius * length);
    let v = normalize(cross(pn, line_dir));
    let probe = Clothoid::try_new([0.0; 3], line_dir, v, 0.0, sigma, length).ok()?;
    let end = probe.point_at(length);
    let t_end = probe.heading_at(length);
    let center = madd(end, 1.0 / (sigma * length), cross(pn, t_end));
    Some(ProbeGeometry {
        sigma,
        length,
        v,
        end,
        center,
    })
}

fn build_spiral(
    origin: [f64; 3],
    radius: f64,
    p: &EndPlan,
    pn: [f64; 3],
) -> Result<Option<(Clothoid, [f64; 3], f64)>, GeometryError> {
    let Some(g) = probe_geometry(radius, p.spiral_dir, p.curve_sgn, p.phi, pn) else {
        return Ok(None);
    };
    let a = sub(origin, g.center);
    let b = add(a, g.end);
    let line_trim = dot(sub(p.vertex, a), p.spiral_dir);
    let clo = Clothoid::try_new(a, p.spiral_dir, g.v, 0.0, g.sigma, g.length)?;
    Ok(Some((clo, b, line_trim)))
}

/// A spiral may consume at most half of the neighbor line: the far half
/// belongs to whatever claims the line's other end — another run's easing or a
/// corner blend — which under streaming causality is unknown when this run
/// seals. Claims beyond half can overlap on a short shared line, and the
/// emitted geometry then jumps backward by the overlap.
fn within_line(trim: f64, length: f64) -> bool {
    trim > -BUDGET_EPS_MM && trim < 0.5 * length
}

fn project_to_circle(origin: [f64; 3], radius: f64, p: [f64; 3]) -> [f64; 3] {
    add(origin, scale(normalize(sub(p, origin)), radius))
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

pub(in crate::fitter::kernels) fn epmm(m: &Move) -> f64 {
    m.segment
        .followers
        .iter()
        .map(|f| {
            assert!(
                !f.is_ramped(),
                "arc-run facets and neighbors must carry constant follower ratios"
            );
            f.ratio.abs()
        })
        .sum()
}
