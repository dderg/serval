use crate::topp::chain::ChainGrid;
use crate::topp::solver::SolverResult;
use crate::{
    BindingConstraint, BindingSummary, FollowerDemand, Limits, WorstBinding, restricted_norm,
};

pub(crate) const EPS_FEAS: f64 = 2e-3;

pub(crate) const EPS_FEAS_JERK: f64 = 5e-2; // TODO: investigate jerk tolerance

const SLACK_THRESHOLD: f64 = 1e-6;

const BOUNDARY_B_TOL: f64 = 1e-9;

#[derive(Debug, Clone)]
pub(crate) struct VerifyReport {
    pub binding_per_grid: Vec<BindingConstraint>,
    #[allow(dead_code)]
    pub worst_violation: f64,
    pub worst_violation_grid: usize,
    pub feasible: bool,
    pub worst_jerk_ratio: f64,
    pub worst_non_jerk_ratio: f64,
    pub binding_summary: BindingSummary,
}

struct PointInputs<'a> {
    cp: [f64; 3],
    cpp: [f64; 3],
    cppp: [f64; 3],
    s_dot: f64,
    s_ddot: f64,
    s_dddot: f64,
    s_ddddot: f64,
    limits: &'a Limits,
    followers: &'a [FollowerDemand],
}

struct PointRatios {
    worst_ratio: f64,
    worst_tag: BindingConstraint,
    max_jerk: f64,
    max_non_jerk: f64,
}

fn ratios_at(p: &PointInputs<'_>) -> PointRatios {
    let s_dot2 = p.s_dot * p.s_dot;
    let s_dot3 = s_dot2 * p.s_dot;

    let vel = [p.cp[0] * p.s_dot, p.cp[1] * p.s_dot, p.cp[2] * p.s_dot];
    let accel = [
        p.cpp[0] * s_dot2 + p.cp[0] * p.s_ddot,
        p.cpp[1] * s_dot2 + p.cp[1] * p.s_ddot,
        p.cpp[2] * s_dot2 + p.cp[2] * p.s_ddot,
    ];
    let jerk = [
        p.cppp[0] * s_dot3 + 3.0 * p.cpp[0] * p.s_dot * p.s_ddot + p.cp[0] * p.s_dddot,
        p.cppp[1] * s_dot3 + 3.0 * p.cpp[1] * p.s_dot * p.s_ddot + p.cp[1] * p.s_dddot,
        p.cppp[2] * s_dot3 + 3.0 * p.cpp[2] * p.s_dot * p.s_ddot + p.cp[2] * p.s_dddot,
    ];

    let lim = p.limits;
    let mut entries: Vec<(f64, BindingConstraint)> = Vec::with_capacity(3 * lim.sets().len());
    for (set_idx, set) in lim.spatial_sets() {
        if set.v_max.is_finite() {
            entries.push((
                restricted_norm(&vel, set.axes) / set.v_max,
                BindingConstraint::Velocity { set: set_idx },
            ));
        }
    }
    for (set_idx, set) in lim.spatial_sets() {
        if set.a_max.is_finite() {
            entries.push((
                restricted_norm(&accel, set.axes) / set.a_max,
                BindingConstraint::AccelNorm { set: set_idx },
            ));
        }
    }
    for (set_idx, set) in lim.spatial_sets() {
        if set.j_max.is_finite() {
            entries.push((
                restricted_norm(&jerk, set.axes) / set.j_max,
                BindingConstraint::JerkNorm { set: set_idx },
            ));
        }
    }
    for f in p.followers {
        let r = f.ratio.abs();
        let k = f.pa_k;
        for (set_idx, set) in lim.follower_sets() {
            if !set.axes.contains(f.axis) {
                continue;
            }
            if k == 0.0 {
                if set.v_max.is_finite() {
                    entries.push((
                        r * p.s_dot / set.v_max,
                        BindingConstraint::Velocity { set: set_idx },
                    ));
                }
                if set.a_max.is_finite() {
                    entries.push((
                        r * p.s_ddot.abs() / set.a_max,
                        BindingConstraint::AccelNorm { set: set_idx },
                    ));
                }
                if set.j_max.is_finite() {
                    entries.push((
                        r * p.s_dddot.abs() / set.j_max,
                        BindingConstraint::JerkNorm { set: set_idx },
                    ));
                }
            } else {
                if set.v_max.is_finite() {
                    entries.push((
                        r * (p.s_dot + k * p.s_ddot).abs() / set.v_max,
                        BindingConstraint::PaVelocity { set: set_idx },
                    ));
                }
                if set.a_max.is_finite() {
                    entries.push((
                        r * (p.s_ddot + k * p.s_dddot).abs() / set.a_max,
                        BindingConstraint::PaAccel { set: set_idx },
                    ));
                }
                if set.j_max.is_finite() {
                    entries.push((
                        r * (p.s_dddot + k * p.s_ddddot).abs() / set.j_max,
                        BindingConstraint::PaJerk { set: set_idx },
                    ));
                }
            }
        }
    }

    let mut worst_ratio = 0.0_f64;
    let mut worst_tag = BindingConstraint::None;
    let mut max_jerk = 0.0_f64;
    let mut max_non_jerk = 0.0_f64;

    for (ratio, tag) in entries {
        if ratio > worst_ratio {
            worst_ratio = ratio;
            worst_tag = tag;
        }
        if matches!(
            tag,
            BindingConstraint::JerkNorm { .. }
                | BindingConstraint::PaJerk { .. }
                | BindingConstraint::PaVelocity { .. }
                | BindingConstraint::PaAccel { .. }
        ) {
            if ratio > max_jerk {
                max_jerk = ratio;
            }
        } else if ratio > max_non_jerk {
            max_non_jerk = ratio;
        }
    }

    let worst_tag = if worst_ratio < SLACK_THRESHOLD {
        BindingConstraint::None
    } else {
        worst_tag
    };

    PointRatios {
        worst_ratio,
        worst_tag,
        max_jerk,
        max_non_jerk,
    }
}

pub(crate) fn check_chain(chain: &ChainGrid, result: &SolverResult) -> VerifyReport {
    let n = chain.n_points();
    let has_pa = chain.followers.iter().flatten().any(|f| f.pa_k != 0.0);
    debug_assert_eq!(result.b.len(), n);
    debug_assert_eq!(result.a.len(), n);

    if n == 0 {
        return VerifyReport {
            binding_per_grid: Vec::new(),
            worst_violation: f64::NEG_INFINITY,
            worst_violation_grid: 0,
            feasible: true,
            worst_jerk_ratio: 0.0,
            worst_non_jerk_ratio: 0.0,
            binding_summary: BindingSummary::default(),
        };
    }

    let windows = if chain.has_active_windows() {
        Some(crate::topp::follower::build_follower_windows(
            chain, &result.b,
        ))
    } else {
        None
    };

    let mut binding_per_grid: Vec<BindingConstraint> = Vec::with_capacity(n);
    let mut point_worst_ratio: Vec<f64> = Vec::with_capacity(n);
    let mut global_worst_ratio: f64 = f64::NEG_INFINITY;
    let mut global_worst_idx: usize = 0;
    let mut global_worst_tag: BindingConstraint = BindingConstraint::None;
    let mut worst_jerk_ratio: f64 = 0.0;
    let mut worst_non_jerk_ratio: f64 = 0.0;

    for i in 0..n {
        let b_i = result.b[i];
        let a_i = result.a[i];

        let s_dot = b_i.max(0.0).sqrt();
        let s_ddot = a_i;
        let s_dddot = crate::topp::stencil::s_dddot_at_weights(&result.b, i, &chain.h_intervals);
        let s_ddddot = if has_pa {
            crate::topp::stencil::s_ddddot_at_weights(
                &result.b,
                a_i,
                i,
                &chain.s,
                &chain.h_intervals,
            )
        } else {
            0.0
        };

        let g = &chain.geom[i];
        let pr = ratios_at(&PointInputs {
            cp: g.c_prime,
            cpp: g.c_double_prime,
            cppp: g.c_triple_prime,
            s_dot,
            s_ddot,
            s_dddot,
            s_ddddot,
            limits: chain.limits_at(i),
            followers: if windows.is_some() {
                &[]
            } else {
                chain.followers_at(i)
            },
        });

        let mut pr = pr;
        if let Some(w) = &windows {
            for d in crate::topp::follower::windowed_demands_at(w, chain, &result.b, &result.a, i) {
                let ratio = d.value / d.cap;
                let tag = windowed_tag(&d);
                let is_jerk_class = matches!(
                    tag,
                    BindingConstraint::JerkNorm { .. }
                        | BindingConstraint::PaJerk { .. }
                        | BindingConstraint::PaVelocity { .. }
                        | BindingConstraint::PaAccel { .. }
                );
                if is_jerk_class {
                    if ratio > pr.max_jerk {
                        pr.max_jerk = ratio;
                    }
                } else if ratio > pr.max_non_jerk {
                    pr.max_non_jerk = ratio;
                }
                if ratio > pr.worst_ratio {
                    pr.worst_ratio = ratio;
                    pr.worst_tag = tag;
                }
            }
        }
        let pr = pr;
        let final_tag = if (i == 0 || i == n - 1) && b_i.abs() < BOUNDARY_B_TOL {
            BindingConstraint::Boundary
        } else {
            pr.worst_tag
        };

        if pr.worst_ratio > global_worst_ratio {
            global_worst_ratio = pr.worst_ratio;
            global_worst_idx = i;
            global_worst_tag = pr.worst_tag;
        }
        if pr.max_jerk > worst_jerk_ratio {
            worst_jerk_ratio = pr.max_jerk;
        }
        if pr.max_non_jerk > worst_non_jerk_ratio {
            worst_non_jerk_ratio = pr.max_non_jerk;
        }

        point_worst_ratio.push(pr.worst_ratio);
        binding_per_grid.push(final_tag);
    }

    for jd in &chain.junctions {
        let i = jd.idx;
        let b_i = result.b[i];

        let s_dot = b_i.max(0.0).sqrt();
        let s_ddot = result.a[i];
        let s_dddot = crate::topp::stencil::s_dddot_at_weights(&result.b, i, &chain.h_intervals);
        let s_ddddot = if has_pa {
            crate::topp::stencil::s_ddddot_at_weights(
                &result.b,
                s_ddot,
                i,
                &chain.s,
                &chain.h_intervals,
            )
        } else {
            0.0
        };

        let pr = ratios_at(&PointInputs {
            cp: jd.geom.c_prime,
            cpp: jd.geom.c_double_prime,
            cppp: jd.geom.c_triple_prime,
            s_dot,
            s_ddot,
            s_dddot,
            s_ddddot,
            limits: &chain.limits[jd.limits_idx],
            followers: &chain.followers[jd.limits_idx],
        });

        if pr.worst_ratio > global_worst_ratio {
            global_worst_ratio = pr.worst_ratio;
            global_worst_idx = i;
            global_worst_tag = pr.worst_tag;
        }
        if pr.worst_ratio > point_worst_ratio[i] {
            point_worst_ratio[i] = pr.worst_ratio;
            binding_per_grid[i] = pr.worst_tag;
        }
        if pr.max_jerk > worst_jerk_ratio {
            worst_jerk_ratio = pr.max_jerk;
        }
        if pr.max_non_jerk > worst_non_jerk_ratio {
            worst_non_jerk_ratio = pr.max_non_jerk;
        }
    }

    let mut histogram_map: std::collections::HashMap<BindingConstraint, u32> =
        std::collections::HashMap::new();
    for tag in &binding_per_grid {
        match tag {
            BindingConstraint::None | BindingConstraint::Boundary => {}
            other => *histogram_map.entry(*other).or_insert(0) += 1,
        }
    }
    let mut histogram: Vec<(BindingConstraint, u32)> = histogram_map.into_iter().collect();
    histogram.sort_by(|(ca, na), (cb, nb)| nb.cmp(na).then_with(|| ca.cmp(cb)));

    let worst = if global_worst_ratio >= SLACK_THRESHOLD
        && !matches!(
            global_worst_tag,
            BindingConstraint::None | BindingConstraint::Boundary
        ) {
        let set = match global_worst_tag {
            BindingConstraint::Velocity { set }
            | BindingConstraint::AccelNorm { set }
            | BindingConstraint::JerkNorm { set }
            | BindingConstraint::PaVelocity { set }
            | BindingConstraint::PaAccel { set }
            | BindingConstraint::PaJerk { set } => Some(set),
            BindingConstraint::None | BindingConstraint::Boundary => None,
        };
        let kind = set.map_or(crate::LimitKind::Config, |s| {
            chain.limits_at(global_worst_idx).kind(s)
        });
        Some(WorstBinding {
            constraint: global_worst_tag,
            ratio: global_worst_ratio,
            grid_index: global_worst_idx,
            s: chain.s[global_worst_idx],
            kind,
        })
    } else {
        None
    };
    let binding_summary = BindingSummary { histogram, worst };

    let worst_violation = global_worst_ratio - 1.0;
    let feasible =
        worst_jerk_ratio <= 1.0 + EPS_FEAS_JERK && worst_non_jerk_ratio <= 1.0 + EPS_FEAS;
    VerifyReport {
        binding_per_grid,
        worst_violation,
        worst_violation_grid: global_worst_idx,
        feasible,
        worst_jerk_ratio,
        worst_non_jerk_ratio,
        binding_summary,
    }
}

fn windowed_tag(d: &crate::topp::follower::WindowedDemand) -> BindingConstraint {
    use crate::topp::follower::PaFamily;
    match (d.family, d.pa) {
        (PaFamily::Velocity, false) => BindingConstraint::Velocity { set: d.set },
        (PaFamily::Accel, false) => BindingConstraint::AccelNorm { set: d.set },
        (PaFamily::Jerk, false) => BindingConstraint::JerkNorm { set: d.set },
        (PaFamily::Velocity, true) => BindingConstraint::PaVelocity { set: d.set },
        (PaFamily::Accel, true) => BindingConstraint::PaAccel { set: d.set },
        (PaFamily::Jerk, true) => BindingConstraint::PaJerk { set: d.set },
    }
}

#[cfg(test)]
mod tests;
