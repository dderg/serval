use crate::GeometryError;
use crate::frontend::Move;
use crate::path::lowering::PositionProfile;
use crate::path::{Arc, Clothoid, CurvatureProfile, Line};
use crate::segment::FollowerDemand;

use super::{
    BUDGET_EPS_MM, ChainFitConfig, CornerFitConfig, FitError, dot, internal, junction_deviation,
    line_of, madd, norm, turn_normal,
};

const COPLANAR_TOL: f64 = 1e-6;
const SEAM_TOL_MM: f64 = 1e-6;
const ANGLE_EPS_RAD: f64 = 1e-9;
const RHO_ARC_BISECTIONS: u32 = 60;

pub(super) struct ChainRun {
    pub start: usize,
    pub end: usize,
    pub recon: Reconstruction,
}

pub(super) struct Reconstruction {
    pub up: Option<Clothoid>,
    pub arc: Arc,
    pub down: Option<Clothoid>,
    pub followers: Vec<FollowerDemand>,
    pub head_consumption: f64,
    pub tail_consumption: f64,
}

pub(super) fn detect_runs(
    moves: &[Move],
    config: ChainFitConfig,
) -> Result<Vec<ChainRun>, FitError> {
    let Some(arc) = config.arc_fit else {
        return Ok(Vec::new());
    };
    let min_run = (arc.min_run_facets.max(3)) as usize;
    let scv_delta = moves
        .iter()
        .map(|m| junction_deviation(m.limits))
        .filter(|d| d.is_finite() && *d > 0.0)
        .fold(f64::INFINITY, f64::min);
    let tol = if scv_delta.is_finite() {
        scv_delta
    } else {
        arc.deviation_tol_mm
    };
    let gate_epmm = moves.iter().any(|m| epmm(m) > EPMM_MIN);
    let mut runs = Vec::new();
    let n = moves.len();
    let mut i = 0;
    while i + 1 < n {
        if line_of(&moves[i]).is_none() || (gate_epmm && epmm(&moves[i]) < EPMM_MIN) {
            i += 1;
            continue;
        }
        let band_end = grow_turning_band(moves, i, config.corner, gate_epmm);
        let mut span_start = i;
        while span_start + min_run <= band_end + 1 {
            let span_end = grow_cocircular_span(moves, span_start, band_end, tol);
            if span_end + 1 - span_start >= min_run {
                if let Some(recon) = reconstruct(&moves[span_start..=span_end], tol)? {
                    runs.push(ChainRun {
                        start: span_start,
                        end: span_end,
                        recon,
                    });
                    span_start = span_end + 1;
                    continue;
                }
            }
            span_start += 1;
        }
        i = band_end + 1;
    }
    Ok(runs)
}

const EPMM_MIN: f64 = 1e-9;
const EPMM_REL_TOL: f64 = 0.25;

fn epmm(m: &Move) -> f64 {
    m.segment.followers.iter().map(|f| f.ratio.abs()).sum()
}

fn grow_turning_band(
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

fn grow_cocircular_span(moves: &[Move], start: usize, band_end: usize, tol: f64) -> usize {
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

fn cocircular(facets: &[Move], tol: f64) -> bool {
    match circle_fit(facets) {
        Some(fit) => fit.residual <= tol && fit.radius.is_finite() && fit.radius > BUDGET_EPS_MM,
        None => false,
    }
}

fn circle_fit(facets: &[Move]) -> Option<CircleFit> {
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

pub(super) fn reconstruct(
    facets: &[Move],
    deviation_tol: f64,
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
    let fit = match fit_circle_through_vertices(&lines, plane_normal) {
        Some(f) => f,
        None => return Ok(None),
    };
    if fit.residual > deviation_tol {
        return Ok(None);
    }
    let (origin, rho) = (fit.origin, fit.radius);
    if !(rho.is_finite() && rho > BUDGET_EPS_MM) {
        return Ok(None);
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
    let followers = run_followers(facets, &lines, head_consumption, tail_consumption, recon_len);

    Ok(Some(Reconstruction {
        up: None,
        arc,
        down: None,
        followers,
        head_consumption,
        tail_consumption,
    }))
}

struct CircleFit {
    origin: [f64; 3],
    radius: f64,
    residual: f64,
}

fn incircle(
    lines: &[&Line],
    e1: [f64; 3],
    e2: [f64; 3],
    inward: &impl Fn([f64; 3]) -> [f64; 3],
) -> Option<CircleFit> {
    let q0 = lines[0].start;
    let mut ata = [[0.0_f64; 3]; 3];
    let mut atb = [0.0_f64; 3];
    for line in lines {
        let t = line.heading_at(0.0);
        let n = inward(t);
        let row = [dot(n, e1), dot(n, e2), -1.0];
        let rhs = dot(n, sub(line.start, q0));
        for r in 0..3 {
            for c in 0..3 {
                ata[r][c] += row[r] * row[c];
            }
            atb[r] += row[r] * rhs;
        }
    }
    let x = solve3(ata, atb)?;
    let origin = add(q0, add(scale(e1, x[0]), scale(e2, x[1])));
    let radius = x[2];
    let mut residual = 0.0_f64;
    for line in lines {
        let t = line.heading_at(0.0);
        let n = inward(t);
        let d = dot(n, sub(origin, line.start));
        residual = residual.max((d - radius).abs());
    }
    Some(CircleFit {
        origin,
        radius,
        residual,
    })
}

fn solve_rho_arc(rho: f64, l_t: f64, line_no: u32) -> Result<Option<f64>, FitError> {
    let spiral_offset_from_arc_centre = |r: f64| -> Result<f64, FitError> {
        let (_, c_in) = spiral_anchor_offset(1.0 / (r * l_t), l_t, r, line_no)?;
        Ok(c_in)
    };
    let mut lo = 0.5 * rho;
    let mut hi = rho;
    let bracket_below = spiral_offset_from_arc_centre(lo)?;
    let bracket_above = spiral_offset_from_arc_centre(hi)?;
    let root_is_bracketed = bracket_below < rho && bracket_above >= rho;
    if !root_is_bracketed {
        return Ok(None);
    }
    for _ in 0..RHO_ARC_BISECTIONS {
        let mid = 0.5 * (lo + hi);
        if spiral_offset_from_arc_centre(mid)? < rho {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    let r = 0.5 * (lo + hi);
    if r.is_finite() && r > BUDGET_EPS_MM {
        Ok(Some(r))
    } else {
        Ok(None)
    }
}

fn spiral_anchor_offset(
    sigma: f64,
    l_t: f64,
    rho_arc: f64,
    line_no: u32,
) -> Result<(f64, f64), FitError> {
    let canon = Clothoid::try_new([0.0; 3], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], 0.0, sigma, l_t)
        .map_err(internal(line_no))?;
    let end = canon.point_at(l_t);
    let phi = 0.5 * sigma * l_t * l_t;
    Ok((end[0] - rho_arc * phi.sin(), end[1] + rho_arc * phi.cos()))
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

fn total_turn(lines: &[&Line]) -> f64 {
    lines
        .windows(2)
        .map(|w| {
            let t_in = w[0].heading_at(0.0);
            let t_out = w[1].heading_at(0.0);
            dot(t_in, t_out).clamp(-1.0, 1.0).acos()
        })
        .sum()
}

fn foot_on_line(p: [f64; 3], line: &Line) -> Option<f64> {
    let t = line.heading_at(0.0);
    let s = dot(sub(p, line.start), t);
    if s.is_finite() { Some(s) } else { None }
}

fn seam_ok(p: [f64; 3], s: f64, line: &Line) -> bool {
    let t = line.heading_at(0.0);
    let foot = madd(line.start, s, t);
    norm(sub(p, foot)) <= SEAM_TOL_MM
}

fn vertices_within_tube(lines: &[&Line], origin: [f64; 3], radius: f64, tol: f64) -> bool {
    for k in 0..lines.len() - 1 {
        let vertex = lines[k].point_at(lines[k].s_len());
        let chord = lines[k].s_len().max(lines[k + 1].s_len());
        let sagitta = chord * chord / (8.0 * radius);
        let radial_deviation = (norm(sub(vertex, origin)) - radius).abs();
        let unfaceting_tube_halfwidth = 2.0 * sagitta + tol;
        if radial_deviation > unfaceting_tube_halfwidth {
            return false;
        }
    }
    true
}

fn arc_len(arc: &Arc) -> f64 {
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

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn normalize(a: [f64; 3]) -> [f64; 3] {
    let n = norm(a);
    [a[0] / n, a[1] / n, a[2] / n]
}

fn add(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn scale(a: [f64; 3], s: f64) -> [f64; 3] {
    [a[0] * s, a[1] * s, a[2] * s]
}

#[cfg(test)]
mod tests;
