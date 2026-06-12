use crate::topp::chain::ChainGrid;
use crate::topp::scaling::SolverScale;
use crate::{Limits, restricted_norm};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Cone {
    Zero,
    Nonneg,
    SecondOrder,
    RotatedSecondOrder,
}

#[derive(Debug, Clone)]
pub struct ConstraintBundle {
    pub n_vars: usize,
    pub n_grid: usize,
    pub cones: Vec<(Cone, usize)>,

    pub a_rows: Vec<Vec<f64>>,
    pub b_rhs: Vec<f64>,
    pub objective: Vec<f64>,
    pub b_max_cent: Vec<f64>,

    pub h_intervals: Vec<f64>,
    pub j_path: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BoundaryInfeasibility {
    StartAboveMvc { mvc_b: f64 },
    EndAboveMvc { mvc_b: f64 },

    EndBelowMinReachable { min_b: f64 },

    EndAboveMaxReachable { max_b: f64 },
}

#[derive(Debug, Clone)]
pub enum BuildOutcome {
    Ok(ConstraintBundle),
    Boundary(BoundaryInfeasibility),
}

#[derive(Debug, Clone, Copy)]
pub struct EndpointConditions {
    pub v_start: f64,
    pub v_end: f64,
    pub a_start: Option<f64>,
}

pub const KAPPA_FLOOR: f64 = 1e-12;

pub const B_MAX_CENT_CAP: f64 = 1e8;

pub(crate) const COMP_FLOOR: f64 = 1e-12;

pub fn rest_boundary_b_cap(d: f64, a_env: f64, j_env: f64) -> f64 {
    let s1 = a_env * a_env * a_env / (6.0 * j_env * j_env);
    let v1_sq = (a_env * a_env / (2.0 * j_env)).powi(2);
    if d <= s1 {
        (6.0_f64.powf(4.0 / 3.0) / 4.0) * (j_env * d * d).powf(2.0 / 3.0)
    } else {
        v1_sq + 2.0 * a_env * (d - s1)
    }
}

pub(crate) fn boundary_reachable_b_upper(s: f64, v0: f64, a0: f64, a_max: f64, j_max: f64) -> f64 {
    debug_assert!(
        a0 <= a_max * (1.0 + 1e-9),
        "a0={a0} exceeds a_max={a_max} in boundary_reachable_b_upper"
    );
    let a0 = a0.min(a_max);

    if s <= 0.0 {
        return v0 * v0;
    }

    let t1 = (a_max - a0) / j_max;
    let s1 = v0 * t1 + 0.5 * a0 * t1 * t1 + (1.0 / 6.0) * j_max * t1 * t1 * t1;
    let v1 = v0 + a0 * t1 + 0.5 * j_max * t1 * t1;

    if s <= s1 {
        let v_t = |t: f64| -> f64 { v0 + a0 * t + 0.5 * j_max * t * t };
        let s_t = |t: f64| -> f64 { v0 * t + 0.5 * a0 * t * t + (1.0 / 6.0) * j_max * t * t * t };
        let t_guess = if v0 > 1e-12 {
            (s / v0).min(t1)
        } else {
            t1 * 0.5
        };
        let mut t = t_guess.clamp(0.0, t1);
        for _ in 0..12 {
            let vt = v_t(t);
            if vt.abs() < 1e-15 {
                break;
            }
            let step = (s_t(t) - s) / vt;
            t = (t - step).clamp(0.0, t1);
        }
        v_t(t).powi(2).max(v0 * v0)
    } else {
        v1 * v1 + 2.0 * a_max * (s - s1)
    }
}

pub(crate) fn boundary_reachable_b_lower(s: f64, v0: f64, a0: f64, a_max: f64, j_max: f64) -> f64 {
    debug_assert!(
        a0 >= -a_max * (1.0 + 1e-9),
        "a0={a0} below -a_max={} in boundary_reachable_b_lower",
        -a_max
    );
    let a0 = a0.max(-a_max);

    if s <= 0.0 {
        return v0 * v0;
    }

    let v_ph1 = |t: f64| -> f64 { v0 + a0 * t - 0.5 * j_max * t * t };
    let s_ph1 = |t: f64| -> f64 { v0 * t + 0.5 * a0 * t * t - (1.0 / 6.0) * j_max * t * t * t };

    let t1_prime = (a0 + a_max) / j_max;

    let t_stop_ph1 = {
        let disc = a0 * a0 + 2.0 * j_max * v0;
        if disc >= 0.0 && v0 > 0.0 {
            let ts = (a0 + disc.sqrt()) / j_max;
            if ts > 0.0 { ts } else { f64::INFINITY }
        } else {
            f64::INFINITY
        }
    };

    let t_end_ph1 = t1_prime.min(t_stop_ph1);
    let s1_prime = s_ph1(t_end_ph1);
    let v1_prime = v_ph1(t_end_ph1).max(0.0);

    if t_stop_ph1 <= t1_prime {
        let s_stop = s_ph1(t_stop_ph1);
        if s >= s_stop {
            return 0.0;
        }

        let t_guess = if v0 > 1e-12 {
            (s / v0).min(t_stop_ph1)
        } else {
            0.0
        };
        let mut t = t_guess.clamp(0.0, t_stop_ph1);
        for _ in 0..12 {
            let vt = v_ph1(t);
            if vt.abs() < 1e-15 {
                break;
            }
            let step = (s_ph1(t) - s) / vt;
            t = (t - step).clamp(0.0, t_stop_ph1);
        }
        v_ph1(t).powi(2).max(0.0)
    } else if s <= s1_prime {
        let t_guess = if v0 > 1e-12 {
            (s / v0).min(t1_prime)
        } else {
            0.0
        };
        let mut t = t_guess.clamp(0.0, t1_prime);
        for _ in 0..12 {
            let vt = v_ph1(t);
            if vt.abs() < 1e-15 {
                break;
            }
            let step = (s_ph1(t) - s) / vt;
            t = (t - step).clamp(0.0, t1_prime);
        }
        v_ph1(t).powi(2).max(0.0)
    } else {
        (v1_prime * v1_prime - 2.0 * a_max * (s - s1_prime)).max(0.0)
    }
}

#[allow(clippy::too_many_lines)]
pub fn build_chain(
    chain: &ChainGrid,
    endpoints: EndpointConditions,
    scale: &SolverScale,
) -> BuildOutcome {
    let n = chain.n_points();
    debug_assert!(n >= 2, "ChainGrid must have at least 2 points");

    let kappa_floor = scale.to_scaled_kappa(KAPPA_FLOOR);
    let b_cap = scale.to_scaled_b(B_MAX_CENT_CAP);
    let h = &chain.h_intervals;
    let h_bar = |i: usize| -> f64 {
        if i == 0 {
            h[0]
        } else if i == n - 1 {
            h[n - 2]
        } else {
            0.5 * (h[i - 1] + h[i])
        }
    };

    let mut b_max_cent: Vec<f64> = (0..n)
        .map(|i| {
            chain
                .limits_at(i)
                .b_cent_cap(
                    &chain.geom[i].c_prime,
                    &chain.geom[i].c_double_prime,
                    kappa_floor,
                )
                .min(b_cap)
        })
        .collect();
    for j in &chain.junctions {
        let cap = chain.limits[j.limits_idx]
            .b_cent_cap(&j.geom.c_prime, &j.geom.c_double_prime, kappa_floor)
            .min(b_cap);
        b_max_cent[j.idx] = b_max_cent[j.idx].min(cap);
    }

    let (a_env, j_env) = {
        let mut a_env = f64::NEG_INFINITY;
        let mut j_env = f64::NEG_INFINITY;
        for i in 0..n {
            let lim = chain.limits_at(i);
            let a_tan_i = lim.a_tan_cap(&chain.geom[i].c_prime, COMP_FLOOR);
            let j_tan_i = lim.j_tan_cap(&chain.geom[i].c_prime, COMP_FLOOR);
            if a_tan_i.is_finite() && j_tan_i.is_finite() {
                a_env = a_env.max(a_tan_i);
                j_env = j_env.max(j_tan_i);
            }
        }
        for j in &chain.junctions {
            let lim = &chain.limits[j.limits_idx];
            let a_tan_i = lim.a_tan_cap(&j.geom.c_prime, COMP_FLOOR);
            let j_tan_i = lim.j_tan_cap(&j.geom.c_prime, COMP_FLOOR);
            if a_tan_i.is_finite() && j_tan_i.is_finite() {
                a_env = a_env.max(a_tan_i);
                j_env = j_env.max(j_tan_i);
            }
        }
        debug_assert!(
            a_env > 0.0 && j_env > 0.0,
            "A_env/J_env must be positive — corrupt grid tangents"
        );
        (a_env, j_env)
    };

    let b_start = endpoints.v_start * endpoints.v_start;
    let b_end = endpoints.v_end * endpoints.v_end;

    let mvc_start = b_max_cent[0].min(chain.limits_at(0).mvc_b(&chain.geom[0].c_prime, COMP_FLOOR));
    let mvc_end = b_max_cent[n - 1].min(
        chain
            .limits_at(n - 1)
            .mvc_b(&chain.geom[n - 1].c_prime, COMP_FLOOR),
    );

    const B_BOUNDARY_REL_TOL: f64 = f64::EPSILON * 4.0;
    if b_start > mvc_start * (1.0 + B_BOUNDARY_REL_TOL) {
        return BuildOutcome::Boundary(BoundaryInfeasibility::StartAboveMvc { mvc_b: mvc_start });
    }
    if b_end > mvc_end * (1.0 + B_BOUNDARY_REL_TOL) {
        return BuildOutcome::Boundary(BoundaryInfeasibility::EndAboveMvc { mvc_b: mvc_end });
    }

    let s_total = chain.s.last().copied().unwrap_or(0.0);
    if let Some(a0) = endpoints.a_start {
        let a0_clamped = a0.clamp(-a_env, a_env);
        let max_b_end =
            boundary_reachable_b_upper(s_total, endpoints.v_start, a0_clamped, a_env, j_env);
        if b_end > max_b_end * (1.0 + B_BOUNDARY_REL_TOL) {
            return BuildOutcome::Boundary(BoundaryInfeasibility::EndAboveMaxReachable {
                max_b: max_b_end,
            });
        }
        let min_b_end =
            boundary_reachable_b_lower(s_total, endpoints.v_start, a0_clamped, a_env, j_env);
        if b_end < min_b_end * (1.0 - B_BOUNDARY_REL_TOL) {
            return BuildOutcome::Boundary(BoundaryInfeasibility::EndBelowMinReachable {
                min_b: min_b_end,
            });
        }
    }

    let n_b = n;
    let n_a = n;
    let n_interior = n.saturating_sub(2);

    let off_b = 0_usize;
    let off_a = n_b;
    let off_t = off_a + n_a;
    let off_x1 = off_t + n_interior;
    let off_x2 = off_x1 + n_interior;

    let n_vars = off_x2 + n_interior;

    let j_path = chain
        .limits
        .iter()
        .map(Limits::j_path)
        .fold(f64::INFINITY, f64::min);
    debug_assert!(j_path > 0.0, "jerk limit must be positive");

    let mut cones: Vec<(Cone, usize)> = Vec::new();
    let mut a_rows: Vec<Vec<f64>> = Vec::new();
    let mut b_rhs: Vec<f64> = Vec::new();

    let push_row =
        |a_rows: &mut Vec<Vec<f64>>, b_rhs: &mut Vec<f64>, entries: &[(usize, f64)], rhs: f64| {
            let mut row = vec![0.0_f64; n_vars];
            for &(idx, coeff) in entries {
                row[idx] = coeff;
            }
            a_rows.push(row);
            b_rhs.push(rhs);
        };

    {
        let mut count = 0_usize;
        push_row(&mut a_rows, &mut b_rhs, &[(off_b, 1.0)], -b_start);
        count += 1;
        push_row(&mut a_rows, &mut b_rhs, &[(off_b + n - 1, 1.0)], -b_end);
        count += 1;
        if endpoints.a_start.is_some() {
            assert!(
                endpoints.v_start > 0.0,
                "a_start pin at a rest start forces b_1 = 0 (rejected trap); \
                 rest starts use the (e2) envelope"
            );
        }
        cones.push((Cone::Zero, count));
    }

    {
        let count = n;

        push_row(
            &mut a_rows,
            &mut b_rhs,
            &[
                (off_a, 1.0),
                (off_b + 1, -1.0 / (2.0 * h[0])),
                (off_b, 1.0 / (2.0 * h[0])),
            ],
            0.0,
        );
        for i in 1..n - 1 {
            let w = crate::topp::stencil::b_d_weights(h[i - 1], h[i]);
            push_row(
                &mut a_rows,
                &mut b_rhs,
                &[
                    (off_a + i, 1.0),
                    (off_b + i - 1, -w[0] / 2.0),
                    (off_b + i, -w[1] / 2.0),
                    (off_b + i + 1, -w[2] / 2.0),
                ],
                0.0,
            );
        }

        push_row(
            &mut a_rows,
            &mut b_rhs,
            &[
                (off_a + n - 1, 1.0),
                (off_b + n - 1, -1.0 / (2.0 * h[n - 2])),
                (off_b + n - 2, 1.0 / (2.0 * h[n - 2])),
            ],
            0.0,
        );
        cones.push((Cone::Zero, count));
    }

    {
        let mut count = 0_usize;
        let emit_velocity = |i: usize,
                             c_prime: &[f64; 3],
                             lim: &Limits,
                             a_rows: &mut Vec<Vec<f64>>,
                             b_rhs: &mut Vec<f64>,
                             count: &mut usize| {
            for set in lim.sets() {
                if !set.v_max.is_finite() {
                    continue;
                }
                let p = restricted_norm(c_prime, set.axes);
                if p < COMP_FLOOR {
                    continue;
                }
                let rhs = (set.v_max / p).powi(2);
                if rhs > b_cap {
                    continue;
                }
                push_row(a_rows, b_rhs, &[(off_b + i, -1.0)], rhs);
                *count += 1;
            }
        };
        for i in 0..n {
            emit_velocity(
                i,
                &chain.geom[i].c_prime,
                chain.limits_at(i),
                &mut a_rows,
                &mut b_rhs,
                &mut count,
            );
        }
        for j in &chain.junctions {
            emit_velocity(
                j.idx,
                &j.geom.c_prime,
                &chain.limits[j.limits_idx],
                &mut a_rows,
                &mut b_rhs,
                &mut count,
            );
        }
        if count > 0 {
            cones.push((Cone::Nonneg, count));
        }
    }

    {
        const BLOCK_D_SAFETY: f64 = 0.1;
        let mut nonneg_run = 0_usize;
        let emit_accel = |i: usize,
                          geom: &crate::topp::chain::PointGeom,
                          lim: &Limits,
                          a_rows: &mut Vec<Vec<f64>>,
                          b_rhs: &mut Vec<f64>,
                          cones: &mut Vec<(Cone, usize)>,
                          nonneg_run: &mut usize| {
            let b_cap_i = b_max_cent[i].min(b_cap);
            let a_cap_i = b_cap_i / (2.0 * h_bar(i));
            for set in lim.sets() {
                if !set.a_max.is_finite() {
                    continue;
                }
                let gp_n = restricted_norm(&geom.c_prime, set.axes);
                let gpp_n = restricted_norm(&geom.c_double_prime, set.axes);
                if gp_n < COMP_FLOOR && gpp_n < COMP_FLOOR {
                    continue;
                }
                if gpp_n * b_cap_i + gp_n * a_cap_i < BLOCK_D_SAFETY * set.a_max {
                    continue;
                }
                if set.axes.count() == 1 {
                    let ax = set.axes.indices().next().expect("singleton");
                    let gp = geom.c_prime[ax];
                    let gpp = geom.c_double_prime[ax];
                    push_row(
                        a_rows,
                        b_rhs,
                        &[(off_b + i, -gpp), (off_a + i, -gp)],
                        set.a_max,
                    );
                    push_row(
                        a_rows,
                        b_rhs,
                        &[(off_b + i, gpp), (off_a + i, gp)],
                        set.a_max,
                    );
                    *nonneg_run += 2;
                } else {
                    if *nonneg_run > 0 {
                        cones.push((Cone::Nonneg, *nonneg_run));
                        *nonneg_run = 0;
                    }
                    push_row(a_rows, b_rhs, &[], set.a_max);
                    for ax in set.axes.indices() {
                        push_row(
                            a_rows,
                            b_rhs,
                            &[
                                (off_b + i, geom.c_double_prime[ax]),
                                (off_a + i, geom.c_prime[ax]),
                            ],
                            0.0,
                        );
                    }
                    cones.push((Cone::SecondOrder, 1 + set.axes.count()));
                }
            }
        };
        for i in 0..n {
            emit_accel(
                i,
                &chain.geom[i],
                chain.limits_at(i),
                &mut a_rows,
                &mut b_rhs,
                &mut cones,
                &mut nonneg_run,
            );
        }
        for j in &chain.junctions {
            emit_accel(
                j.idx,
                &j.geom,
                &chain.limits[j.limits_idx],
                &mut a_rows,
                &mut b_rhs,
                &mut cones,
                &mut nonneg_run,
            );
        }
        if nonneg_run > 0 {
            cones.push((Cone::Nonneg, nonneg_run));
        }
    }

    {
        let count = n;
        for i in 0..n {
            push_row(&mut a_rows, &mut b_rhs, &[(off_b + i, -1.0)], b_max_cent[i]);
        }
        cones.push((Cone::Nonneg, count));
    }

    {
        let mut count = 0_usize;
        for i in 0..n - 1 {
            let lim = chain.limits_at(i + 1);
            let node_cap_i = b_max_cent[i];
            let node_cap_j = b_max_cent[i + 1];
            for sample in &chain.inter_geom[i] {
                let inter_cap = lim
                    .b_cent_cap(&sample.c_prime, &sample.c_double_prime, kappa_floor)
                    .min(b_cap);
                let interp_node_cap = (1.0 - sample.theta) * node_cap_i + sample.theta * node_cap_j;
                if inter_cap >= interp_node_cap * (1.0 - 1e-9) {
                    continue;
                }
                push_row(
                    &mut a_rows,
                    &mut b_rhs,
                    &[
                        (off_b + i, -(1.0 - sample.theta)),
                        (off_b + i + 1, -sample.theta),
                    ],
                    inter_cap,
                );
                count += 1;
            }
        }
        if count > 0 {
            cones.push((Cone::Nonneg, count));
        }
    }

    {
        let mut count = 0_usize;
        if endpoints.v_start == 0.0 {
            for i in 1..n {
                let d = chain.s[i] - chain.s[0];
                let cap = rest_boundary_b_cap(d, a_env, j_env);
                if cap >= b_cap {
                    break;
                }
                push_row(&mut a_rows, &mut b_rhs, &[(off_b + i, -1.0)], cap);
                count += 1;
            }
        }
        if endpoints.v_end == 0.0 {
            for i in (0..n - 1).rev() {
                let d = chain.s[n - 1] - chain.s[i];
                let cap = rest_boundary_b_cap(d, a_env, j_env);
                if cap >= b_cap {
                    break;
                }
                push_row(&mut a_rows, &mut b_rhs, &[(off_b + i, -1.0)], cap);
                count += 1;
            }
        }
        if count > 0 {
            cones.push((Cone::Nonneg, count));
        }
    }

    {
        let mut count = 0_usize;
        if let Some(a0) = endpoints.a_start {
            let v0 = endpoints.v_start;
            debug_assert!(
                v0 > 0.0,
                "tube guard: v_start must be > 0 when a_start is Some"
            );
            for i in 1..n {
                let si = chain.s[i] - chain.s[0];
                let upper = boundary_reachable_b_upper(si, v0, a0, a_env, j_env);
                let lower = boundary_reachable_b_lower(si, v0, a0, a_env, j_env);
                let upper_dominated = upper >= b_cap;
                let lower_vacuous = lower <= 0.0;
                if !upper_dominated {
                    push_row(&mut a_rows, &mut b_rhs, &[(off_b + i, -1.0)], upper);
                    count += 1;
                }
                if !lower_vacuous {
                    push_row(&mut a_rows, &mut b_rhs, &[(off_b + i, 1.0)], -lower);
                    count += 1;
                }
                if upper_dominated && lower_vacuous {
                    break;
                }
            }
        }
        if count > 0 {
            cones.push((Cone::Nonneg, count));
        }
    }

    {
        let mut count = 0_usize;
        for k in 0..n_interior {
            let i = k + 1;
            let t_idx = off_t + k;
            let w = crate::topp::stencil::b_dd_weights(h[i - 1], h[i]);
            let c = h_bar(i) / (2.0 * j_path);
            for sign in [1.0_f64, -1.0] {
                push_row(
                    &mut a_rows,
                    &mut b_rhs,
                    &[
                        (t_idx, 1.0),
                        (off_b + i - 1, -sign * c * w[0]),
                        (off_b + i, -sign * c * w[1]),
                        (off_b + i + 1, -sign * c * w[2]),
                    ],
                    0.0,
                );
            }
            count += 2;
        }
        if count > 0 {
            cones.push((Cone::Nonneg, count));
        }
    }

    {
        let mut count = 0_usize;
        for k in 0..n_interior {
            push_row(&mut a_rows, &mut b_rhs, &[(off_x1 + k, 1.0)], 0.0);
            push_row(&mut a_rows, &mut b_rhs, &[(off_x2 + k, 1.0)], 0.0);
            count += 2;
        }
        if count > 0 {
            cones.push((Cone::Nonneg, count));
        }
    }

    {
        let b_idx = |k: usize| -> usize { off_b + k + 1 };

        for k in 0..n_interior {
            let t_idx = off_t + k;
            let x1_idx = off_x1 + k;
            let x2_idx = off_x2 + k;
            let bi_idx = b_idx(k);
            let hk = h_bar(k + 1);
            let sqrt_hk = hk.sqrt();

            push_row(&mut a_rows, &mut b_rhs, &[(t_idx, 1.0)], hk);
            push_row(&mut a_rows, &mut b_rhs, &[(x2_idx, 2.0)], 0.0);
            push_row(&mut a_rows, &mut b_rhs, &[(t_idx, 1.0)], -hk);
            cones.push((Cone::SecondOrder, 3));

            push_row(&mut a_rows, &mut b_rhs, &[(t_idx, 1.0), (bi_idx, 1.0)], 0.0);
            push_row(&mut a_rows, &mut b_rhs, &[(x1_idx, 2.0 / sqrt_hk)], 0.0);
            push_row(
                &mut a_rows,
                &mut b_rhs,
                &[(t_idx, 1.0), (bi_idx, -1.0)],
                0.0,
            );
            cones.push((Cone::SecondOrder, 3));

            push_row(
                &mut a_rows,
                &mut b_rhs,
                &[(x1_idx, 1.0), (x2_idx, 1.0)],
                0.0,
            );
            push_row(&mut a_rows, &mut b_rhs, &[], 2.0 * hk);
            push_row(
                &mut a_rows,
                &mut b_rhs,
                &[(x1_idx, 1.0), (x2_idx, -1.0)],
                0.0,
            );
            cones.push((Cone::SecondOrder, 3));
        }
    }

    let mut objective = vec![0.0_f64; n_vars];
    for k in 0..n_interior {
        objective[off_t + k] = 1.0;
    }

    debug_assert_eq!(
        a_rows.len(),
        cones.iter().map(|(_, d)| d).sum::<usize>(),
        "row count / cone dimension mismatch"
    );
    debug_assert_eq!(a_rows.len(), b_rhs.len(), "a_rows / b_rhs length mismatch");
    debug_assert!(
        a_rows.iter().all(|r| r.len() == n_vars),
        "all A rows must have width n_vars"
    );

    BuildOutcome::Ok(ConstraintBundle {
        n_vars,
        n_grid: n,
        cones,
        a_rows,
        b_rhs,
        objective,
        b_max_cent,
        h_intervals: chain.h_intervals.clone(),
        j_path,
    })
}

#[cfg(test)]
mod tests;
