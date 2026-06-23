use crate::GeometryError;
use crate::frontend::Move;
use crate::path::lowering::PositionProfile;
use crate::path::{Arc, Clothoid, CurvatureProfile, Line};
use crate::segment::FollowerDemand;

use super::{
    ArcFitConfig, BUDGET_EPS_MM, ChainFitConfig, FitError, dot, internal, junction_deviation,
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
    pub up: Clothoid,
    pub arc: Arc,
    pub down: Clothoid,
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
    let mut runs = Vec::new();
    let n = moves.len();
    let mut i = 0;
    while i + 1 < n {
        if line_of(&moves[i]).is_none() {
            i += 1;
            continue;
        }
        let (end, turning) = grow_run(moves, i, config, arc);
        if turning >= config.min_run_junctions {
            if let Some(recon) = reconstruct(&moves[i..=end], config)? {
                runs.push(ChainRun {
                    start: i,
                    end,
                    recon,
                });
                i = end + 1;
                continue;
            }
        }
        i += 1;
    }
    Ok(runs)
}

fn grow_run(
    moves: &[Move],
    start: usize,
    config: ChainFitConfig,
    arc: ArcFitConfig,
) -> (usize, u32) {
    let n = moves.len();
    let mut end = start;
    let mut turning = 0u32;
    let mut plane: Option<[f64; 3]> = None;
    let mut turn_sign: Option<f64> = None;
    while end + 1 < n {
        let (la, lb) = match (line_of(&moves[end]), line_of(&moves[end + 1])) {
            (Some(a), Some(b)) => (a, b),
            _ => break,
        };
        if la.s_len() > arc.facet_len_max_mm || lb.s_len() > arc.facet_len_max_mm {
            break;
        }
        let t_in = la.heading_at(la.s_len());
        let t_out = lb.heading_at(0.0);
        let theta = dot(t_in, t_out).clamp(-1.0, 1.0).acos();
        if theta >= config.corner.theta_max_rad {
            break;
        }
        if theta > arc.max_turn_rad {
            break;
        }
        if theta <= config.corner.theta_min_rad {
            end += 1;
            continue;
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
        turning += 1;
        end += 1;
    }
    (end, turning)
}

pub(super) fn reconstruct(
    facets: &[Move],
    config: ChainFitConfig,
) -> Result<Option<Reconstruction>, FitError> {
    let line_no = facets[0].source.start_line;
    let lines: Vec<&Line> = match facets.iter().map(line_of).collect::<Option<Vec<_>>>() {
        Some(l) => l,
        None => return Ok(None),
    };

    let t0 = lines[0].heading_at(0.0);
    let tm = lines[lines.len() - 1].heading_at(0.0);
    let v0 = match turn_normal(t0, lines[1].heading_at(0.0)) {
        Some(v) => v,
        None => return Ok(None),
    };
    let plane_normal = normalize(cross(t0, v0));

    let inward = |t: [f64; 3]| normalize(cross(plane_normal, t));
    let fit = match incircle(&lines, t0, v0, &inward) {
        Some(f) => f,
        None => return Ok(None),
    };
    if fit.residual > config.cocircular_tol {
        return Ok(None);
    }
    let (origin, rho) = (fit.origin, fit.radius);
    if !(rho.is_finite() && rho > BUDGET_EPS_MM) {
        return Ok(None);
    }

    let theta_run = total_turn(&lines);
    let delta = facets
        .iter()
        .map(|m| junction_deviation(m.limits))
        .fold(f64::INFINITY, f64::min);
    if !(delta.is_finite() && delta > 0.0) {
        return Ok(None);
    }

    let len_first = lines[0].s_len();
    let len_last = lines[lines.len() - 1].s_len();
    let l_t = (24.0 * rho * delta)
        .sqrt()
        .min(len_first)
        .min(len_last)
        .min(0.5 * rho);
    if l_t <= BUDGET_EPS_MM {
        return Ok(None);
    }
    let rho_arc = match solve_rho_arc(rho, l_t, line_no)? {
        Some(r) => r,
        None => return Ok(None),
    };
    let kappa_arc = 1.0 / rho_arc;
    let delta_sweep = theta_run - l_t / rho_arc;
    if delta_sweep <= ANGLE_EPS_RAD {
        return Ok(None);
    }

    let sigma = kappa_arc / l_t;
    let n0 = inward(t0);
    let (c_rel_along, c_rel_in) = spiral_anchor_offset(sigma, l_t, rho_arc, line_no)?;
    let s0 = sub(origin, add(scale(t0, c_rel_along), scale(n0, c_rel_in)));

    let up = Clothoid::try_new(s0, t0, n0, 0.0, sigma, l_t).map_err(internal(line_no))?;
    let a0 = up.point_at(l_t);
    let v_arc = up.heading_at(l_t);
    let origin_arc = add(a0, scale(inward(v_arc), rho_arc));
    let u_arc = normalize(sub(a0, origin_arc));
    let arc = Arc::try_new(origin_arc, u_arc, v_arc, rho_arc, 0.0, delta_sweep)
        .map_err(internal(line_no))?;
    let a1 = arc.point_at(arc_len(&arc));
    let h1 = arc.heading_at(arc_len(&arc));
    let n_exit = normalize(sub(origin_arc, a1));
    let down =
        Clothoid::try_new(a1, h1, n_exit, kappa_arc, -sigma, l_t).map_err(internal(line_no))?;

    let head_len = match foot_on_line(s0, lines[0]) {
        Some(s) => s,
        None => return Ok(None),
    };
    let tail_pt = down.point_at(l_t);
    let last = lines[lines.len() - 1];
    let tail_len = match foot_on_line(tail_pt, last) {
        Some(s) => s,
        None => return Ok(None),
    };
    if !seam_ok(s0, head_len, lines[0])
        || !seam_ok(tail_pt, tail_len, last)
        || dot(down.heading_at(l_t), tm) < 1.0 - SEAM_TOL_MM
    {
        return Ok(None);
    }
    let head_consumption = len_first - head_len;
    let tail_consumption = tail_len;
    if !(-SEAM_TOL_MM..=len_first + SEAM_TOL_MM).contains(&head_consumption)
        || !(-SEAM_TOL_MM..=len_last + SEAM_TOL_MM).contains(&tail_consumption)
    {
        return Ok(None);
    }
    if !vertices_within_tube(&lines, origin_arc, rho_arc, config.cocircular_tol) {
        return Ok(None);
    }

    let recon_len = up.s_len() + arc_len(&arc) + down.s_len();
    let followers = run_followers(
        facets,
        &lines,
        head_consumption,
        tail_consumption,
        recon_len,
    );

    Ok(Some(Reconstruction {
        up,
        arc,
        down,
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
