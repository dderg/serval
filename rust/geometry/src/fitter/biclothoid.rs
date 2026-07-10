use crate::GeometryError;
use crate::path::Clothoid;
use crate::path::CurvatureProfile;
use crate::path::lowering::PositionProfile;

use super::linalg::solve3;
use super::vec3::{add, cross, dist, dot, madd, normalize, scale, signed_angle, sub};

pub(super) struct ClothoidPair {
    pub half1: Clothoid,
    pub half2: Clothoid,
}

const DEGENERATE_EPS: f64 = 1e-12;
/// Position tolerance in canonical units (fractions of the chord): the
/// Hermite solve is scale-normalized, so this is a relative tolerance.
const HERMITE_POS_TOL: f64 = 1e-12;
const HERMITE_ANG_TOL_RAD: f64 = 1e-12;
const HERMITE_MAX_ITERS: usize = 60;
/// Consecutive accepted Newton steps that fail to at least halve the squared
/// residual before the seed is abandoned as non-convergent.
const WEAK_STREAK_LIMIT: u32 = 12;

/// Blend a line-line corner at `vertex` with the symmetric analytic solve:
/// the deviation-optimal trim, clamped by the shared runway budget, consumed
/// equally from both lines.
pub(super) fn solve_line_line(
    vertex: [f64; 3],
    t_in: [f64; 3],
    v: [f64; 3],
    theta: f64,
    delta: f64,
    budget: f64,
) -> Result<Option<GeneralBlend>, GeometryError> {
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

    Ok(Some(GeneralBlend {
        half1,
        half2,
        trim_in: trim,
        trim_out: trim,
    }))
}

/// The line-line trim at which a symmetric corner blend's deviation reaches
/// `delta` — the deviation-optimal blend footprint per side, unclamped.
pub(super) fn trim_at_delta(theta: f64, delta: f64) -> Result<f64, GeometryError> {
    let (trim_ref, deviation_ref) = canonical(theta)?;
    if trim_ref <= DEGENERATE_EPS || deviation_ref <= DEGENERATE_EPS {
        return Ok(0.0);
    }
    Ok(trim_ref * delta / deviation_ref)
}

/// The peak curvature of the symmetric line-line blend that turns `theta`
/// with `trim` consumed per side — what [`solve_line_line`] would build.
pub(super) fn symmetric_blend_kappa_peak(theta: f64, trim: f64) -> Result<f64, GeometryError> {
    if trim <= super::BUDGET_EPS_MM {
        return Ok(f64::INFINITY);
    }
    let (trim_ref, _) = canonical(theta)?;
    Ok((trim_ref * theta / trim).abs())
}

const CONSUME_COARSE_STEPS: usize = 8;
const CONSUME_REFINE_ITERS: usize = 6;
/// A split per level doubles the pair count: depth 2 allows up to four
/// clothoid pairs across one consumed chain.
const CONSUME_SPLIT_DEPTH: usize = 2;

/// The G2-continuous curve that replaces a consumed facet chain, plus the
/// arclength it claims from each neighbor line.
pub(super) struct ChainBlend {
    pub segments: Vec<Clothoid>,
    pub trim_in: f64,
    pub trim_out: f64,
}

/// Blend across a consumed chain of facets: a G2 sequence of clothoid pairs
/// from a contact `t` before the chain's first vertex on the inbound line to
/// `t` past its last vertex on the outbound line, tangent and curvature-free
/// at both contacts and owing the facets themselves nothing — the whole curve
/// only has to stay within `delta` of the polyline it replaces. A single pair
/// sweeps the whole turn in one curvature bump — the least curvature and the
/// fastest traversal — so the scan runs single-pair first over all contact
/// reaches, and only a chain no single pair can span at any reach falls back
/// to split solves: the span divides at a mid-chain anchor (on a facet, so
/// the anchor state is exact: facet tangent, zero curvature) and each side
/// solves recursively. Each pass picks the largest feasible `t` up to
/// `t_cap`; the deviation is not monotone in `t` (the blend first hugs the
/// chain, then cuts deeper), so a coarse top-down scan finds the feasible
/// band's upper edge before bisecting it. `vertices` runs from the inbound
/// line's end to the outbound line's start.
pub(super) fn solve_consume_chain(
    vertices: &[[f64; 3]],
    t_a: [f64; 3],
    t_b: [f64; 3],
    delta: f64,
    t_cap: f64,
) -> Option<ChainBlend> {
    assert!(
        vertices.len() >= 2,
        "a consumed chain spans at least one facet"
    );
    if t_cap <= super::BUDGET_EPS_MM || delta <= 0.0 {
        return None;
    }
    let plane_n = normalize(cross(t_a, t_b));

    // Every probe below solves near-identical geometry: the converged
    // solution in the chord-normalized frame (which barely moves with the
    // contact reach) seeds the next probe's Newton — including probes whose
    // blend converged but failed the deviation check, and the split-depth
    // rescan, which revisits the same reaches.
    let mut hint = None;
    [0, CONSUME_SPLIT_DEPTH].into_iter().find_map(|depth| {
        scan_consume_reach(vertices, t_a, t_b, delta, t_cap, plane_n, depth, &mut hint)
    })
}

#[allow(clippy::too_many_arguments)]
fn scan_consume_reach(
    vertices: &[[f64; 3]],
    t_a: [f64; 3],
    t_b: [f64; 3],
    delta: f64,
    t_cap: f64,
    plane_n: [f64; 3],
    depth: usize,
    hint: &mut Option<[f64; 3]>,
) -> Option<ChainBlend> {
    let mut eval = |t: f64| -> Option<Vec<ClothoidPair>> {
        let a = madd(vertices[0], -t, t_a);
        let b = madd(vertices[vertices.len() - 1], t, t_b);
        let mut tube = Vec::with_capacity(vertices.len() + 2);
        tube.push(a);
        tube.extend_from_slice(vertices);
        tube.push(b);
        let start = ChainState {
            pos: a,
            tangent: t_a,
        };
        let end = ChainState {
            pos: b,
            tangent: t_b,
        };
        let (pairs, solved) =
            consume_pairs(&start, &end, vertices, &tube, delta, plane_n, depth, *hint);
        *hint = solved.or(*hint);
        pairs
    };

    let mut feasible = None;
    let mut infeasible_above = None;
    for i in 0..CONSUME_COARSE_STEPS {
        let t = t_cap * 0.5_f64.powi(i as i32);
        match eval(t) {
            Some(pairs) => {
                feasible = Some((pairs, t));
                break;
            }
            None => infeasible_above = Some(t),
        }
    }
    let (mut pairs, mut t) = feasible?;

    if let Some(mut hi) = infeasible_above {
        for _ in 0..CONSUME_REFINE_ITERS {
            let mid = 0.5 * (t + hi);
            match eval(mid) {
                Some(p) => {
                    pairs = p;
                    t = mid;
                }
                None => hi = mid,
            }
        }
    }

    let segments = pairs.into_iter().flat_map(|p| [p.half1, p.half2]).collect();
    Some(ChainBlend {
        segments,
        trim_in: t,
        trim_out: t,
    })
}

struct ChainState {
    pos: [f64; 3],
    tangent: [f64; 3],
}

/// Solve `start → end` (both curvature-free states) as one clothoid pair
/// within `delta` of `tube`, or split at a mid-chain facet anchor and solve
/// each side. `interior` is the polyline strictly between the two states.
/// The second element is this level's converged chord-normalized Newton
/// solution whether or not the blend passed the deviation check — the
/// caller's warm-start for the next near-identical probe.
#[allow(clippy::too_many_arguments)]
fn consume_pairs(
    start: &ChainState,
    end: &ChainState,
    interior: &[[f64; 3]],
    tube: &[[f64; 3]],
    delta: f64,
    plane_n: [f64; 3],
    depth: usize,
    hint: Option<[f64; 3]>,
) -> (Option<Vec<ClothoidPair>>, Option<[f64; 3]>) {
    let solved = hermite_g2_hinted(
        start.pos,
        start.tangent,
        0.0,
        end.pos,
        end.tangent,
        0.0,
        plane_n,
        hint,
    );
    let top_hint = solved.as_ref().map(|(_, x)| *x);
    if let Some((pair, _)) = solved {
        if max_dev_from_chain(&pair, tube) <= delta {
            return (Some(vec![pair]), top_hint);
        }
    }
    if depth == 0 || interior.len() < 2 {
        return (None, top_hint);
    }

    let Some((facet_idx, anchor)) = mid_facet_anchor(start, end, interior) else {
        return (None, top_hint);
    };
    let (left, _) = consume_pairs(
        start,
        &anchor,
        &interior[..=facet_idx],
        tube,
        delta,
        plane_n,
        depth - 1,
        None,
    );
    let Some(mut left) = left else {
        return (None, top_hint);
    };
    let (right, _) = consume_pairs(
        &anchor,
        end,
        &interior[facet_idx + 1..],
        tube,
        delta,
        plane_n,
        depth - 1,
        None,
    );
    let Some(right) = right else {
        return (None, top_hint);
    };
    left.extend(right);
    (Some(left), top_hint)
}

/// The split anchor: the midpoint of the facet nearest the chain's arclength
/// middle, tangent along that facet — a state the polyline itself passes
/// through, so anchoring there is always inside the tube. Returns the facet's
/// leading vertex index in `interior`.
fn mid_facet_anchor(
    start: &ChainState,
    end: &ChainState,
    interior: &[[f64; 3]],
) -> Option<(usize, ChainState)> {
    let total: f64 = std::iter::once(start.pos)
        .chain(interior.iter().copied())
        .chain(std::iter::once(end.pos))
        .collect::<Vec<_>>()
        .windows(2)
        .map(|w| dist(w[0], w[1]))
        .sum();

    let mut cum = dist(start.pos, interior[0]);
    let mut best: Option<(usize, f64)> = None;
    for i in 0..interior.len() - 1 {
        let len = dist(interior[i], interior[i + 1]);
        let mid_at = cum + 0.5 * len;
        if len > DEGENERATE_EPS {
            let off = (mid_at - 0.5 * total).abs();
            if best.is_none_or(|(_, b)| off < b) {
                best = Some((i, off));
            }
        }
        cum += len;
    }
    let (i, _) = best?;
    let dir = normalize(sub(interior[i + 1], interior[i]));
    Some((
        i,
        ChainState {
            pos: scale(add(interior[i], interior[i + 1]), 0.5),
            tangent: dir,
        },
    ))
}

const DEV_SAMPLES_PER_HALF: usize = 16;

/// Sampled deviation of the blend from the polyline it replaces: the largest
/// distance from any curve sample to the chain's segments. Sampling
/// under-reads the true maximum by the sagitta between samples — negligible
/// against junction deviations.
fn max_dev_from_chain(pair: &ClothoidPair, chain: &[[f64; 3]]) -> f64 {
    let mut worst = 0.0_f64;
    for half in [&pair.half1, &pair.half2] {
        for i in 0..=DEV_SAMPLES_PER_HALF {
            let s = half.s_len() * (i as f64) / (DEV_SAMPLES_PER_HALF as f64);
            let p = half.point_at(s);
            let d = chain
                .windows(2)
                .map(|seg| dist_to_segment(p, seg[0], seg[1]))
                .fold(f64::INFINITY, f64::min);
            worst = worst.max(d);
        }
    }
    worst
}

fn dist_to_segment(p: [f64; 3], s0: [f64; 3], s1: [f64; 3]) -> f64 {
    let d = sub(s1, s0);
    let len_sq = dot(d, d);
    if len_sq <= 0.0 {
        return dist(p, s0);
    }
    let t = (dot(sub(p, s0), d) / len_sq).clamp(0.0, 1.0);
    dist(p, madd(s0, t, d))
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
    (r[0] * r[0] + r[1] * r[1]).sqrt() < HERMITE_POS_TOL && r[2].abs() < HERMITE_ANG_TOL_RAD
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
) -> Option<[f64; 3]> {
    let pair = build_pair(start, kappa_b, plane_n, x[0], x[1], x[2])?;
    let mut r = residual(&pair, end, e1, e2, plane_n);
    let mut lambda = 1e-3;
    let mut weak_streak = 0u32;
    for _ in 0..HERMITE_MAX_ITERS {
        if converged(r) {
            return Some(x);
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
        // A convergent solve contracts the residual rapidly once inside its
        // basin; a long run of accepted-but-barely-improving steps is an
        // infeasible or ill-conditioned seed grinding toward the iteration
        // cap — give up early, the caller has more seeds.
        if residual_norm(r) > 0.5 * r0 {
            weak_streak += 1;
            if weak_streak >= WEAK_STREAK_LIMIT {
                return None;
            }
        } else {
            weak_streak = 0;
        }
    }
    None
}

/// Solve in a scale-normalized frame — positions in chord units, curvatures
/// in turns-per-chord — so tolerances are relative and the Newton iteration
/// is equally conditioned for 0.01mm and 100mm blends. The converged
/// unknowns rescale back to world units for the returned pair.
#[allow(clippy::too_many_arguments)]
fn hermite_g2_hinted(
    p_a: [f64; 3],
    t_a: [f64; 3],
    kappa_a: f64,
    p_b: [f64; 3],
    t_b: [f64; 3],
    kappa_b: f64,
    plane_n: [f64; 3],
    hint: Option<[f64; 3]>,
) -> Option<(ClothoidPair, [f64; 3])> {
    let chord = dist(p_a, p_b);
    if chord <= DEGENERATE_EPS {
        return None;
    }
    let inv = 1.0 / chord;
    let start = Endpoint {
        pose: scale(p_a, inv),
        tangent: t_a,
        kappa: kappa_a * chord,
    };
    let end = Endpoint {
        pose: scale(p_b, inv),
        tangent: t_b,
        kappa: kappa_b * chord,
    };
    let e1 = t_a;
    let e2 = normalize(cross(plane_n, e1));

    let theta = signed_angle(t_a, t_b, plane_n);
    let (ka, kb) = (start.kappa, end.kappa);
    let kp_turn = theta - 0.5 * (ka + kb);

    let seeds = [
        [kp_turn, 0.5, 0.5],
        [kp_turn, 1.0, 1.0],
        [kp_turn, 0.3, 1.0],
        [kp_turn, 1.0, 0.3],
        [0.5 * (ka + kb), 1.0, 1.0],
        [ka, 0.5, 0.5],
        [kb, 0.5, 0.5],
        [-(ka + kb), 1.0, 1.0],
        [0.0, 2.0, 2.0],
    ];
    for seed in hint.into_iter().chain(seeds) {
        if let Some(x) = newton_pair(&start, &end, kb, plane_n, e1, e2, seed) {
            let world_start = Endpoint {
                pose: p_a,
                tangent: t_a,
                kappa: kappa_a,
            };
            let pair = build_pair(
                &world_start,
                kappa_b,
                plane_n,
                x[0] * inv,
                x[1] * chord,
                x[2] * chord,
            )?;
            return Some((pair, x));
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
    hermite_g2_hinted(p_a, t_a, kappa_a, p_b, t_b, kappa_b, plane_n, None).map(|(pair, _)| pair)
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
mod hermite_grid_tests;
#[cfg(test)]
mod tests;
