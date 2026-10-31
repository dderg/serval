use std::f64::consts::{FRAC_PI_6, PI};

use crate::GeometryError;
use crate::frontend::Move;
use crate::path::lowering::PositionProfile;
use crate::path::{Arc, Clothoid, CurvatureProfile, Line};
use crate::segment::FollowerDemand;

use super::super::vec3::{add, cross, dot, madd, norm, normalize, scale, signed_angle, sub};
use super::super::{BUDGET_EPS_MM, FitError, internal, line_of};
use super::Reconstruction;
use super::follower::construct_followers;

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

pub(in crate::fitter) enum RunEnd {
    Head,
    Tail,
}

pub(in crate::fitter) fn neighbor(m: &Move, end: RunEnd) -> Option<Neighbor> {
    let l = line_of(m)?;
    let (vertex, dir) = match end {
        RunEnd::Head => (l.point_at(l.s_len()), l.heading_at(l.s_len())),
        RunEnd::Tail => (l.point_at(0.0), l.heading_at(0.0)),
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
    let arc = ArcFrame {
        origin: recon.arc.origin,
        radius: recon.arc.radius,
        u: recon.arc.u,
        v: recon.arc.v,
        sweep: recon.arc.sweep,
        plane_n: normalize(cross(recon.arc.u, recon.arc.v)),
    };
    let sgn = arc.sweep.signum();

    let head_max = head.and_then(|n| {
        ease_plan(
            n,
            Spiral {
                dir: n.dir,
                curve_sgn: sgn,
            },
            head_epmm,
            &arc,
        )
    });
    let tail_max = tail.and_then(|n| {
        ease_plan(
            n,
            Spiral {
                dir: scale(n.dir, -1.0),
                curve_sgn: -sgn,
            },
            tail_epmm,
            &arc,
        )
    });
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
            let ends = EaseEnds {
                head: hp.map(|plan| EaseEnd {
                    plan,
                    line_len: head_len,
                }),
                tail: tp.map(|plan| EaseEnd {
                    plan,
                    line_len: tail_len,
                }),
            };
            let attempt = try_ease(ends, &arc, &verts, tol).map_err(internal(line_no))?;
            if let Some(f) = attempt {
                fit = Some(f);
                break 'search;
            }
        }
    }

    let Some(ease) = fit else {
        return Ok(());
    };

    recon.arc = ease.arc;

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

    let head_end = ease.head.as_ref().zip(head).map(|(_, n)| EasedEnd {
        neighbor_followers: &n.followers,
    });
    let tail_end = ease.tail.as_ref().zip(tail).map(|(_, n)| EasedEnd {
        neighbor_followers: &n.followers,
    });
    let (up_followers, arc_followers, down_followers) =
        construct_followers(facets, head_end.as_ref(), tail_end.as_ref());
    recon.up_followers = up_followers;
    recon.followers = arc_followers;
    recon.down_followers = down_followers;

    Ok(())
}

/// An end of the reconstruction that eases into its neighbor line through a
/// spiral: the neighbor's demands the spiral's outer seam anchors to.
pub(in crate::fitter::kernels) struct EasedEnd<'a> {
    pub neighbor_followers: &'a [FollowerDemand],
}

#[derive(Clone, Copy)]
struct Spiral {
    dir: [f64; 3],
    curve_sgn: f64,
}

#[derive(Clone, Copy)]
struct EndPlan {
    spiral: Spiral,
    phi: f64,
    vertex: [f64; 3],
}

/// A planned eased end together with the neighbor line the spiral trims.
#[derive(Clone, Copy)]
struct EaseEnd {
    plan: EndPlan,
    line_len: f64,
}

#[derive(Clone, Copy)]
struct EaseEnds {
    head: Option<EaseEnd>,
    tail: Option<EaseEnd>,
}

/// The circle the run reconstructed to, in its own plane.
#[derive(Clone, Copy)]
struct ArcFrame {
    origin: [f64; 3],
    radius: f64,
    u: [f64; 3],
    v: [f64; 3],
    sweep: f64,
    plane_n: [f64; 3],
}

struct SpiralFit {
    clo: Clothoid,
    b: [f64; 3],
    trim: f64,
}

struct EaseFit {
    head: Option<SpiralFit>,
    tail: Option<SpiralFit>,
    arc: Arc,
}

/// Solve one easing configuration end to end: refit the circle for the given
/// end plans, then build every planned spiral and validate its line trim, and
/// rebuild the residual mid-arc between the spiral contacts. A planned end
/// whose spiral is degenerate or over-claims its neighbor — or a pair of
/// spirals that would consume more angle than the arc has, leaving a residual
/// sweep that runs backward — rejects the whole attempt: the caller retries
/// with a smaller lead angle or fewer eased ends, because the refit circle is
/// only valid together with the spirals it was solved for.
fn try_ease(
    ends: EaseEnds,
    arc: &ArcFrame,
    verts: &[[f64; 3]],
    tol: f64,
) -> Result<Option<EaseFit>, GeometryError> {
    let sweep0 = arc.sweep;
    let pn = arc.plane_n;
    let consumed = ends.head.map_or(0.0, |e| e.plan.phi) + ends.tail.map_or(0.0, |e| e.plan.phi);
    let expected_sweep = sweep0 - sweep0.signum() * consumed;
    if expected_sweep * sweep0 <= 0.0 {
        return Ok(None);
    }
    let Some((origin, radius)) = ease_circle(ends, arc, verts, tol) else {
        return Ok(None);
    };
    let head = match ends.head {
        Some(e) => match build_spiral(origin, radius, &e.plan, pn)? {
            Some((clo, b, trim)) if within_line(trim, e.line_len) => {
                Some(SpiralFit { clo, b, trim })
            }
            _ => return Ok(None),
        },
        None => None,
    };
    let tail = match ends.tail {
        Some(e) => match build_spiral(origin, radius, &e.plan, pn)? {
            Some((clo, b, trim)) if within_line(trim, e.line_len) => {
                Some(SpiralFit { clo, b, trim })
            }
            _ => return Ok(None),
        },
        None => None,
    };
    let b_head = match &head {
        Some(s) => s.b,
        None => project_to_circle(origin, radius, verts[0]),
    };
    let b_tail = match &tail {
        Some(s) => s.b,
        None => project_to_circle(origin, radius, *verts.last().expect("run has vertices")),
    };
    let Some(arc) = build_arc(origin, radius, b_head, b_tail, pn, expected_sweep)? else {
        return Ok(None);
    };
    Ok(Some(EaseFit { head, tail, arc }))
}

fn max_ease_angle(radius: f64, neighbor_len: f64) -> f64 {
    (0.45 * neighbor_len / radius).min(EASE_LEAD_MAX_RAD)
}

fn ease_plan(n: &Neighbor, spiral: Spiral, run_epmm: f64, arc: &ArcFrame) -> Option<EndPlan> {
    if run_epmm > EPMM_MIN && (n.epmm - run_epmm).abs() > EPMM_REL_TOL * run_epmm {
        return None;
    }
    let phi = max_ease_angle(arc.radius, n.length);
    if phi < ANGLE_EPS_RAD {
        return None;
    }
    let normal = scale(normalize(cross(arc.plane_n, spiral.dir)), spiral.curve_sgn);
    if dot(normal, sub(arc.origin, n.vertex)) <= 0.0 {
        return None;
    }
    Some(EndPlan {
        spiral,
        phi,
        vertex: n.vertex,
    })
}

fn spiral_center_dist(radius: f64, spiral: Spiral, phi: f64, pn: [f64; 3]) -> Option<f64> {
    let g = probe_geometry(radius, spiral, phi, pn)?;
    let normal = scale(g.v, spiral.curve_sgn);
    Some(dot(g.center, normal))
}

fn ease_circle(
    ends: EaseEnds,
    arc: &ArcFrame,
    verts: &[[f64; 3]],
    tol: f64,
) -> Option<([f64; 3], f64)> {
    let pn = arc.plane_n;
    let radius = arc.radius;
    match (ends.head, ends.tail) {
        (Some(h), Some(t)) => {
            let h = h.plan;
            let t = t.plan;
            let dh = spiral_center_dist(radius, h.spiral, h.phi, pn)?;
            let dt = spiral_center_dist(radius, t.spiral, t.phi, pn)?;
            let nh = scale(normalize(cross(pn, h.spiral.dir)), h.spiral.curve_sgn);
            let nt = scale(normalize(cross(pn, t.spiral.dir)), t.spiral.curve_sgn);
            let origin = solve_center(
                CenterConstraint {
                    normal: nh,
                    vertex: h.vertex,
                    offset: dh,
                },
                CenterConstraint {
                    normal: nt,
                    vertex: t.vertex,
                    offset: dt,
                },
                arc,
            )?;
            if interior_residual(&origin, radius, verts) <= tol {
                Some((origin, radius))
            } else {
                None
            }
        }
        (Some(e), None) => one_end_center(&e.plan, *verts.last().unwrap(), arc, verts, tol),
        (None, Some(e)) => one_end_center(&e.plan, verts[0], arc, verts, tol),
        (None, None) => None,
    }
}

fn one_end_center(
    e: &EndPlan,
    bare_vertex: [f64; 3],
    arc: &ArcFrame,
    verts: &[[f64; 3]],
    tol: f64,
) -> Option<([f64; 3], f64)> {
    let radius = arc.radius;
    let o0 = arc.origin;
    let pn = arc.plane_n;
    let d = spiral_center_dist(radius, e.spiral, e.phi, pn)?;
    let normal = scale(normalize(cross(pn, e.spiral.dir)), e.spiral.curve_sgn);
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

/// One end's demand on the refit center: it must sit `offset` along `normal`
/// from the end's own line vertex.
#[derive(Clone, Copy)]
struct CenterConstraint {
    normal: [f64; 3],
    vertex: [f64; 3],
    offset: f64,
}

fn solve_center(
    head: CenterConstraint,
    tail: CenterConstraint,
    arc: &ArcFrame,
) -> Option<[f64; 3]> {
    let (o0, u, v) = (arc.origin, arc.u, arc.v);
    let (nh, nt) = (head.normal, tail.normal);
    let a = [[dot(nh, u), dot(nh, v)], [dot(nt, u), dot(nt, v)]];
    let b = [
        dot(nh, sub(head.vertex, o0)) + head.offset,
        dot(nt, sub(tail.vertex, o0)) + tail.offset,
    ];
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

fn probe_geometry(radius: f64, spiral: Spiral, phi: f64, pn: [f64; 3]) -> Option<ProbeGeometry> {
    let length = 2.0 * radius * phi;
    if !(length.is_finite() && length > BUDGET_EPS_MM) {
        return None;
    }
    let sigma = spiral.curve_sgn / (radius * length);
    let v = normalize(cross(pn, spiral.dir));
    let probe = Clothoid::try_new([0.0; 3], spiral.dir, v, 0.0, sigma, length).ok()?;
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
    let Some(g) = probe_geometry(radius, p.spiral, p.phi, pn) else {
        return Ok(None);
    };
    let a = sub(origin, g.center);
    let b = add(a, g.end);
    let line_trim = dot(sub(p.vertex, a), p.spiral.dir);
    let off_line = sub(sub(p.vertex, a), scale(p.spiral.dir, line_trim));
    if norm(off_line) > super::super::SEAM_CLOSURE_EPS_MM {
        return Ok(None);
    }
    let clo = Clothoid::try_new(a, p.spiral.dir, g.v, 0.0, g.sigma, g.length)?;
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

/// Rebuild the residual mid-arc between the spiral contacts. The principal
/// signed angle is unwrapped toward `expected_sweep` — the original sweep
/// minus the eases' lead angles — not merely toward the original direction:
/// when the eases consume more angle than the arc has, the residual truly
/// runs backward, and forcing it to the original sign would emit a
/// near-full-circle arc (a 35.5mm-radius fitted run traced a 71mm-wide
/// circle around the part, bench 2026-07-27). A residual whose direction
/// disagrees with the expectation is rejected instead.
fn build_arc(
    origin: [f64; 3],
    radius: f64,
    b_head: [f64; 3],
    b_tail: [f64; 3],
    pn: [f64; 3],
    expected_sweep: f64,
) -> Result<Option<Arc>, GeometryError> {
    let u = normalize(sub(b_head, origin));
    let v = cross(pn, u);
    let principal = signed_angle(u, normalize(sub(b_tail, origin)), pn);
    let tau = 2.0 * PI;
    let sweep = principal + tau * ((expected_sweep - principal) / tau).round();
    if !sweep.is_finite()
        || sweep.abs() <= ANGLE_EPS_RAD
        || sweep.signum() != expected_sweep.signum()
    {
        return Ok(None);
    }
    Arc::try_new(origin, u, v, radius, 0.0, sweep).map(Some)
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
