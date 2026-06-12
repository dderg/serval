use crate::topp::chain::ChainGrid;

pub(crate) fn emit_base_follower_rows(
    chain: &ChainGrid,
    off_b: usize,
    off_a: usize,
    mut push_row: impl FnMut(&[(usize, f64)], f64),
) -> usize {
    let mut count = 0;
    for i in 0..chain.s.len() {
        let lim = chain.limits_at(i);
        for f in chain.followers_at(i) {
            if f.pa_k != 0.0 {
                continue;
            }
            let r = f.ratio.abs();
            for (_, set) in lim.follower_sets() {
                if !set.axes.contains(f.axis) {
                    continue;
                }
                if set.v_max.is_finite() {
                    let cap = (set.v_max / r).powi(2);
                    push_row(&[(off_b + i, -1.0)], cap);
                    count += 1;
                }
                if set.a_max.is_finite() {
                    push_row(&[(off_a + i, -r)], set.a_max);
                    push_row(&[(off_a + i, r)], set.a_max);
                    count += 2;
                }
            }
        }
    }
    count
}

pub(crate) const PA_CUT_MAX_ENTRIES: usize = 10;

#[derive(Debug, Clone, Copy)]
pub(crate) struct FollowerCut {
    pub entries: [(usize, f64); PA_CUT_MAX_ENTRIES],
    pub n_entries: usize,
    pub rhs_pos: f64,
    pub rhs_neg: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum PaFamily {
    Velocity,
    Accel,
    Jerk,
}

struct LinearizedDemand {
    value: f64,
    entries: Vec<(usize, f64)>,
}

fn accumulate(entries: &mut Vec<(usize, f64)>, col: usize, coeff: f64) {
    if coeff == 0.0 {
        return;
    }
    for e in entries.iter_mut() {
        if e.0 == col {
            e.1 += coeff;
            return;
        }
    }
    entries.push((col, coeff));
}

#[allow(clippy::too_many_arguments)]
fn pa_demand_linearized(
    family: PaFamily,
    r: f64,
    k: f64,
    b: &[f64],
    a: &[f64],
    i: usize,
    s: &[f64],
    h_intervals: &[f64],
    off_b: usize,
    off_a: usize,
    b_floor: f64,
) -> LinearizedDemand {
    let n = b.len();
    let (idx2, hl, hr) = crate::topp::stencil::stencil_at(i, n, h_intervals);
    let w2 = crate::topp::stencil::b_dd_weights(hl, hr);
    let b_dd = w2[0] * b[idx2[0]] + w2[1] * b[idx2[1]] + w2[2] * b[idx2[2]];
    let sqrt_b = b[i].max(b_floor).max(f64::MIN_POSITIVE).sqrt();
    let s_dddot = sqrt_b * b_dd / 2.0;

    let mut entries = Vec::new();
    let value;
    match family {
        PaFamily::Velocity => {
            value = r * (sqrt_b + k * a[i]);
            accumulate(&mut entries, off_b + i, r / (2.0 * sqrt_b));
            accumulate(&mut entries, off_a + i, r * k);
        }
        PaFamily::Accel => {
            value = r * (a[i] + k * s_dddot);
            accumulate(&mut entries, off_a + i, r);
            add_s_dddot_gradient(&mut entries, r * k, &idx2, &w2, sqrt_b, b_dd, i, off_b);
        }
        PaFamily::Jerk => {
            let (idx3, w3) = crate::topp::stencil::b_ddd_weights_at(i, s);
            let b_ddd: f64 = (0..4).map(|m| w3[m] * b[idx3[m]]).sum();
            let s_ddddot = a[i] * b_dd / 2.0 + b[i].max(0.0) * b_ddd / 2.0;
            value = r * (s_dddot + k * s_ddddot);
            add_s_dddot_gradient(&mut entries, r, &idx2, &w2, sqrt_b, b_dd, i, off_b);
            accumulate(&mut entries, off_a + i, r * k * b_dd / 2.0);
            for m in 0..3 {
                accumulate(&mut entries, off_b + idx2[m], r * k * a[i] * w2[m] / 2.0);
            }
            for m in 0..4 {
                accumulate(
                    &mut entries,
                    off_b + idx3[m],
                    r * k * b[i].max(0.0) * w3[m] / 2.0,
                );
            }
            accumulate(&mut entries, off_b + i, r * k * b_ddd / 2.0);
        }
    }
    LinearizedDemand { value, entries }
}

fn add_s_dddot_gradient(
    entries: &mut Vec<(usize, f64)>,
    scale: f64,
    idx2: &[usize; 3],
    w2: &[f64; 3],
    sqrt_b: f64,
    b_dd: f64,
    i: usize,
    off_b: usize,
) {
    for m in 0..3 {
        accumulate(entries, off_b + idx2[m], scale * sqrt_b * w2[m] / 2.0);
    }
    accumulate(entries, off_b + i, scale * b_dd / (4.0 * sqrt_b));
}

fn pa_families_for(set: &crate::LimitSet) -> [(PaFamily, f64); 3] {
    [
        (PaFamily::Velocity, set.v_max),
        (PaFamily::Accel, set.a_max),
        (PaFamily::Jerk, set.j_max),
    ]
}

pub(crate) fn max_pa_ratio(b: &[f64], a: &[f64], chain: &ChainGrid) -> f64 {
    let b_floor = 0.0;
    let mut worst: f64 = 0.0;
    for i in 0..b.len() {
        let lim = chain.limits_at(i);
        for f in chain.followers_at(i) {
            if f.pa_k == 0.0 {
                continue;
            }
            let r = f.ratio.abs();
            for (_, set) in lim.follower_sets() {
                if !set.axes.contains(f.axis) {
                    continue;
                }
                for (family, cap) in pa_families_for(set) {
                    if !cap.is_finite() {
                        continue;
                    }
                    let d = pa_demand_linearized(
                        family,
                        r,
                        f.pa_k,
                        b,
                        a,
                        i,
                        &chain.s,
                        &chain.h_intervals,
                        0,
                        b.len(),
                        b_floor,
                    );
                    worst = worst.max(d.value.abs() / cap);
                }
            }
        }
    }
    worst
}

#[allow(clippy::too_many_arguments)]
pub(crate) fn build_follower_pa_cuts(
    b: &[f64],
    a: &[f64],
    chain: &ChainGrid,
    target_ratio: f64,
    eps_feas: f64,
    placement_fraction: f64,
    target_decay: f64,
    b_floor: f64,
) -> Vec<FollowerCut> {
    let n = b.len();
    let off_b = 0;
    let off_a = n;
    let mut cuts = Vec::new();
    for i in 0..n {
        let lim = chain.limits_at(i);
        for f in chain.followers_at(i) {
            if f.pa_k == 0.0 {
                continue;
            }
            let r = f.ratio.abs();
            for (_, set) in lim.follower_sets() {
                if !set.axes.contains(f.axis) {
                    continue;
                }
                for (family, cap) in pa_families_for(set) {
                    if !cap.is_finite() {
                        continue;
                    }
                    let d = pa_demand_linearized(
                        family,
                        r,
                        f.pa_k,
                        b,
                        a,
                        i,
                        &chain.s,
                        &chain.h_intervals,
                        off_b,
                        off_a,
                        b_floor,
                    );
                    let ratio = d.value.abs() / cap;
                    let cap_inflated = if target_ratio <= 1.0 + eps_feas {
                        if ratio <= eps_feas {
                            continue;
                        }
                        cap
                    } else if ratio > placement_fraction * target_ratio {
                        cap * target_ratio
                    } else if ratio > 1.0 + eps_feas {
                        cap * (ratio * target_decay).max(1.0)
                    } else if ratio > eps_feas {
                        cap
                    } else {
                        continue;
                    };
                    if let Some(cut) = finish_cut(&d, b, a, cap_inflated, off_b, off_a) {
                        cuts.push(cut);
                    }
                }
            }
        }
    }
    cuts
}

fn finish_cut(
    d: &LinearizedDemand,
    b: &[f64],
    a: &[f64],
    cap: f64,
    off_b: usize,
    off_a: usize,
) -> Option<FollowerCut> {
    assert!(
        d.entries.len() <= PA_CUT_MAX_ENTRIES,
        "PA cut entry overflow"
    );
    let n = b.len();
    let grad_dot_xbar: f64 = d
        .entries
        .iter()
        .map(|&(col, g)| {
            if col < off_a {
                g * b[col - off_b]
            } else {
                g * a[col - off_a]
            }
        })
        .sum();
    let _ = n;
    let k_const = d.value - grad_dot_xbar;
    let row_scale = d.entries.iter().map(|&(_, g)| g.abs()).fold(0.0, f64::max);
    if row_scale == 0.0 {
        return None;
    }
    let mut entries = [(0usize, 0.0f64); PA_CUT_MAX_ENTRIES];
    for (slot, &(col, g)) in d.entries.iter().enumerate() {
        entries[slot] = (col, g / row_scale);
    }
    Some(FollowerCut {
        entries,
        n_entries: d.entries.len(),
        rhs_pos: (cap - k_const) / row_scale,
        rhs_neg: (cap + k_const) / row_scale,
    })
}

#[cfg(test)]
mod tests;
