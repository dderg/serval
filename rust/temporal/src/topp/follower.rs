use crate::topp::chain::ChainGrid;

pub(crate) fn emit_base_follower_rows(
    chain: &ChainGrid,
    off_b: usize,
    off_a: usize,
    mut push_row: impl FnMut(&[(usize, f64)], f64),
) -> usize {
    if chain.has_active_windows() {
        return 0;
    }
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

use crate::topp::window::{WindowHistory, WindowOperator, frozen_time_map};

pub(crate) const FOLLOWER_REFREEZE_MAX: u32 = 8;
pub(crate) const REFREEZE_DRIFT_TOL: f64 = 1e-2;

pub(crate) struct AxisWindows {
    pub v: WindowOperator,
    pub a: WindowOperator,
    pub j: WindowOperator,
}

pub(crate) struct FollowerWindows {
    pub t_map: Vec<f64>,
    pub axes: [AxisWindows; 3],
    pub max_half_support: f64,
}

fn history_accel_from_velocity(v: &[f64], dt: f64) -> Vec<f64> {
    let n = v.len();
    (0..n)
        .map(|m| {
            if n < 2 {
                0.0
            } else if m == 0 {
                (v[0] - v[1]) / dt
            } else if m == n - 1 {
                (v[n - 2] - v[n - 1]) / dt
            } else {
                (v[m - 1] - v[m + 1]) / (2.0 * dt)
            }
        })
        .collect()
}

pub(crate) fn build_follower_windows(chain: &ChainGrid, b_bar: &[f64]) -> FollowerWindows {
    let t_map = frozen_time_map(b_bar, &chain.h_intervals);
    let n = t_map.len();
    let mut max_half_support = 0.0_f64;
    let axes = std::array::from_fn(|alpha| {
        let Some(kernel) = &chain.axis_kernels[alpha] else {
            return AxisWindows {
                v: WindowOperator::identity(n),
                a: WindowOperator::identity(n),
                j: WindowOperator::identity(n),
            };
        };
        max_half_support = max_half_support.max(kernel.support().1);
        let signal_histories = |h: &Option<crate::FollowerHistory>| match h {
            Some(h) if !h.axis_velocity[alpha].is_empty() => {
                let v = &h.axis_velocity[alpha];
                let a = history_accel_from_velocity(v, h.dt);
                let j = history_accel_from_velocity(&a, h.dt);
                [
                    WindowHistory {
                        dt: h.dt,
                        samples: v.clone(),
                    },
                    WindowHistory {
                        dt: h.dt,
                        samples: a,
                    },
                    WindowHistory {
                        dt: h.dt,
                        samples: j,
                    },
                ]
            }
            _ => [
                WindowHistory::empty(),
                WindowHistory::empty(),
                WindowHistory::empty(),
            ],
        };
        let [hist_v, hist_a, hist_j] = signal_histories(&chain.follower_history);
        let [term_v, term_a, term_j] = signal_histories(&chain.follower_terminal);
        AxisWindows {
            v: WindowOperator::from_kernel_with_terminal(kernel, &t_map, &hist_v, &term_v),
            a: WindowOperator::from_kernel_with_terminal(kernel, &t_map, &hist_a, &term_a),
            j: WindowOperator::from_kernel_with_terminal(kernel, &t_map, &hist_j, &term_j),
        }
    });
    FollowerWindows {
        t_map,
        axes,
        max_half_support: max_half_support.max(f64::MIN_POSITIVE),
    }
}

pub(crate) fn refreeze_drift(windows: &FollowerWindows, b_new: &[f64], chain: &ChainGrid) -> f64 {
    let t_new = frozen_time_map(b_new, &chain.h_intervals);
    let t_old = &windows.t_map;
    let h = windows.max_half_support;
    let d: Vec<f64> = t_new.iter().zip(t_old).map(|(n, o)| n - o).collect();
    let mut worst: f64 = 0.0;
    for i in 0..t_old.len() {
        let mut j = i;
        while j > 0 && t_old[i] - t_old[j - 1] <= h {
            j -= 1;
        }
        for dj in &d[j..=i] {
            worst = worst.max((d[i] - dj).abs());
        }
    }
    worst / h
}

struct WindowedSignals {
    v: [f64; 3],
    a: [f64; 3],
    j: [f64; 3],
}

fn windowed_signals_at(
    windows: &FollowerWindows,
    chain: &ChainGrid,
    b: &[f64],
    a: &[f64],
    i: usize,
) -> WindowedSignals {
    let mut out = WindowedSignals {
        v: [0.0; 3],
        a: [0.0; 3],
        j: [0.0; 3],
    };
    for alpha in 0..3 {
        let aw = &windows.axes[alpha];
        let mut v_acc = aw.v.row(i).history;
        for &(jj, w) in &aw.v.row(i).weights {
            v_acc += w * chain.geom[jj].c_prime[alpha] * b[jj].max(0.0).sqrt();
        }
        let mut a_acc = aw.a.row(i).history;
        for &(jj, w) in &aw.a.row(i).weights {
            let g = &chain.geom[jj];
            a_acc += w * (g.c_double_prime[alpha] * b[jj] + g.c_prime[alpha] * a[jj]);
        }
        let mut j_acc = aw.j.row(i).history;
        for &(jj, w) in &aw.j.row(i).weights {
            let g = &chain.geom[jj];
            let sqrt_b = b[jj].max(0.0).sqrt();
            let s_dddot = crate::topp::stencil::s_dddot_at_weights(b, jj, &chain.h_intervals);
            j_acc += w
                * (g.c_triple_prime[alpha] * b[jj].max(0.0) * sqrt_b
                    + 3.0 * g.c_double_prime[alpha] * sqrt_b * a[jj]
                    + g.c_prime[alpha] * s_dddot);
        }
        out.v[alpha] = v_acc;
        out.a[alpha] = a_acc;
        out.j[alpha] = j_acc;
    }
    out
}

fn norm3(v: &[f64; 3]) -> f64 {
    (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt()
}

pub(crate) struct WindowedDemand {
    pub family: PaFamily,
    pub set: usize,
    pub value: f64,
    pub cap: f64,
    pub pa: bool,
}

pub(crate) fn windowed_demands_at(
    windows: &FollowerWindows,
    chain: &ChainGrid,
    b: &[f64],
    a: &[f64],
    i: usize,
) -> Vec<WindowedDemand> {
    let followers = chain.followers_at(i);
    if followers.is_empty() {
        return Vec::new();
    }
    let sig = windowed_signals_at(windows, chain, b, a, i);
    let (nv, na, nj) = (norm3(&sig.v), norm3(&sig.a), norm3(&sig.j));
    let s_ddddot = if followers.iter().any(|f| f.pa_k != 0.0) && b.len() >= 4 {
        crate::topp::stencil::s_ddddot_at_weights(b, a[i], i, &chain.s, &chain.h_intervals)
    } else {
        0.0
    };
    let lim = chain.limits_at(i);
    let mut out = Vec::new();
    for f in followers {
        let r = f.ratio.abs();
        let k = f.pa_k;
        for (set_idx, set) in lim.follower_sets() {
            if !set.axes.contains(f.axis) {
                continue;
            }
            if set.v_max.is_finite() {
                out.push(WindowedDemand {
                    family: PaFamily::Velocity,
                    set: set_idx,
                    value: r * (nv + k * na),
                    cap: set.v_max,
                    pa: k != 0.0,
                });
            }
            if set.a_max.is_finite() {
                out.push(WindowedDemand {
                    family: PaFamily::Accel,
                    set: set_idx,
                    value: r * (na + k * nj),
                    cap: set.a_max,
                    pa: k != 0.0,
                });
            }
            if set.j_max.is_finite() {
                out.push(WindowedDemand {
                    family: PaFamily::Jerk,
                    set: set_idx,
                    value: r * (nj + k * s_ddddot.abs()),
                    cap: set.j_max,
                    pa: k != 0.0,
                });
            }
        }
    }
    out
}

pub(crate) fn max_windowed_ratio(
    windows: &FollowerWindows,
    chain: &ChainGrid,
    b: &[f64],
    a: &[f64],
) -> f64 {
    let mut worst: f64 = 0.0;
    for i in 0..b.len() {
        for d in windowed_demands_at(windows, chain, b, a, i) {
            worst = worst.max(d.value / d.cap);
        }
    }
    worst
}

#[derive(Debug, Clone)]
pub(crate) struct WindowedCut {
    pub entries: Vec<(usize, f64)>,
    pub rhs: f64,
}

const HYPERPLANE_NORM_FLOOR: f64 = 1e-9;

#[allow(clippy::too_many_arguments)]
pub(crate) fn build_windowed_follower_cuts(
    b: &[f64],
    a: &[f64],
    chain: &ChainGrid,
    windows: &FollowerWindows,
    target_ratio: f64,
    eps_feas: f64,
    placement_fraction: f64,
    target_decay: f64,
    b_floor: f64,
) -> Vec<WindowedCut> {
    let n = b.len();
    let off_b = 0;
    let off_a = n;
    let mut cuts = Vec::new();
    for i in 0..n {
        let followers = chain.followers_at(i);
        if followers.is_empty() {
            continue;
        }
        let sig = windowed_signals_at(windows, chain, b, a, i);
        let (nv, na, nj) = (norm3(&sig.v), norm3(&sig.a), norm3(&sig.j));
        let u_v = sig.v.map(|x| x / nv.max(HYPERPLANE_NORM_FLOOR));
        let u_a = sig.a.map(|x| x / na.max(HYPERPLANE_NORM_FLOOR));
        let u_j = sig.j.map(|x| x / nj.max(HYPERPLANE_NORM_FLOOR));

        let grad_v = |scale: f64, entries: &mut Vec<(usize, f64)>| {
            for alpha in 0..3 {
                let aw = &windows.axes[alpha];
                for &(jj, w) in &aw.v.row(i).weights {
                    let sqrt_b = b[jj].max(b_floor).max(f64::MIN_POSITIVE).sqrt();
                    accumulate(
                        entries,
                        off_b + jj,
                        scale * u_v[alpha] * w * chain.geom[jj].c_prime[alpha] / (2.0 * sqrt_b),
                    );
                }
            }
        };
        let grad_a = |scale: f64, entries: &mut Vec<(usize, f64)>| {
            for alpha in 0..3 {
                let aw = &windows.axes[alpha];
                for &(jj, w) in &aw.a.row(i).weights {
                    let g = &chain.geom[jj];
                    accumulate(
                        entries,
                        off_b + jj,
                        scale * u_a[alpha] * w * g.c_double_prime[alpha],
                    );
                    accumulate(
                        entries,
                        off_a + jj,
                        scale * u_a[alpha] * w * g.c_prime[alpha],
                    );
                }
            }
        };
        let grad_j = |scale: f64, entries: &mut Vec<(usize, f64)>| {
            for alpha in 0..3 {
                let aw = &windows.axes[alpha];
                for &(jj, w) in &aw.j.row(i).weights {
                    let g = &chain.geom[jj];
                    let sqrt_b = b[jj].max(b_floor).max(f64::MIN_POSITIVE).sqrt();
                    let (idx2, hl, hr) =
                        crate::topp::stencil::stencil_at(jj, n, &chain.h_intervals);
                    let w2 = crate::topp::stencil::b_dd_weights(hl, hr);
                    let b_dd = w2[0] * b[idx2[0]] + w2[1] * b[idx2[1]] + w2[2] * b[idx2[2]];
                    let c = scale * u_j[alpha] * w;
                    accumulate(
                        entries,
                        off_b + jj,
                        c * (1.5 * g.c_triple_prime[alpha] * sqrt_b
                            + 3.0 * g.c_double_prime[alpha] * a[jj] / (2.0 * sqrt_b)
                            + g.c_prime[alpha] * b_dd / (4.0 * sqrt_b)),
                    );
                    for m in 0..3 {
                        accumulate(
                            entries,
                            off_b + idx2[m],
                            c * g.c_prime[alpha] * sqrt_b * w2[m] / 2.0,
                        );
                    }
                    accumulate(
                        entries,
                        off_a + jj,
                        c * 3.0 * g.c_double_prime[alpha] * sqrt_b,
                    );
                }
            }
        };

        for d in windowed_demands_at(windows, chain, b, a, i) {
            let ratio = d.value / d.cap;
            let cap_inflated = if target_ratio <= 1.0 + eps_feas {
                if ratio <= eps_feas {
                    continue;
                }
                d.cap
            } else if ratio > placement_fraction * target_ratio {
                d.cap * target_ratio
            } else if ratio > 1.0 + eps_feas {
                d.cap * (ratio * target_decay).max(1.0)
            } else if ratio > eps_feas {
                d.cap
            } else {
                continue;
            };
            let follower = chain
                .followers_at(i)
                .iter()
                .find(|f| {
                    chain
                        .limits_at(i)
                        .sets()
                        .get(d.set)
                        .is_some_and(|set| set.axes.contains(f.axis))
                })
                .expect("demand came from a covering follower");
            let r = follower.ratio.abs();
            let k = follower.pa_k;
            let mut entries = Vec::new();
            match d.family {
                PaFamily::Velocity => {
                    grad_v(r, &mut entries);
                    if k != 0.0 {
                        grad_a(r * k, &mut entries);
                    }
                }
                PaFamily::Accel => {
                    grad_a(r, &mut entries);
                    if k != 0.0 {
                        grad_j(r * k, &mut entries);
                    }
                }
                PaFamily::Jerk => {
                    grad_j(r, &mut entries);
                    if k != 0.0 {
                        let snap_dem = pa_demand_linearized(
                            PaFamily::Jerk,
                            1.0,
                            0.0,
                            b,
                            a,
                            i,
                            &chain.s,
                            &chain.h_intervals,
                            off_b,
                            off_a,
                            b_floor,
                        );
                        let _ = snap_dem;
                        let s_ddddot = crate::topp::stencil::s_ddddot_at_weights(
                            b,
                            a[i],
                            i,
                            &chain.s,
                            &chain.h_intervals,
                        );
                        let sigma = if s_ddddot >= 0.0 { 1.0 } else { -1.0 };
                        let (idx2, hl, hr) =
                            crate::topp::stencil::stencil_at(i, n, &chain.h_intervals);
                        let w2 = crate::topp::stencil::b_dd_weights(hl, hr);
                        let b_dd = w2[0] * b[idx2[0]] + w2[1] * b[idx2[1]] + w2[2] * b[idx2[2]];
                        let (idx3, w3) = crate::topp::stencil::b_ddd_weights_at(i, &chain.s);
                        let b_ddd: f64 = (0..4).map(|m| w3[m] * b[idx3[m]]).sum();
                        let c = r * k * sigma;
                        accumulate(&mut entries, off_a + i, c * b_dd / 2.0);
                        for m in 0..3 {
                            accumulate(&mut entries, off_b + idx2[m], c * a[i] * w2[m] / 2.0);
                        }
                        for m in 0..4 {
                            accumulate(
                                &mut entries,
                                off_b + idx3[m],
                                c * b[i].max(0.0) * w3[m] / 2.0,
                            );
                        }
                        accumulate(&mut entries, off_b + i, c * b_ddd / 2.0);
                    }
                }
            }
            let grad_dot_xbar: f64 = entries
                .iter()
                .map(|&(col, g)| {
                    if col < off_a {
                        g * b[col - off_b]
                    } else {
                        g * a[col - off_a]
                    }
                })
                .sum();
            let k_const = d.value - grad_dot_xbar;
            let row_scale = entries.iter().map(|&(_, g)| g.abs()).fold(0.0, f64::max);
            if row_scale == 0.0 {
                continue;
            }
            for e in &mut entries {
                e.1 /= row_scale;
            }
            cuts.push(WindowedCut {
                entries,
                rhs: (cap_inflated - k_const) / row_scale,
            });
        }
    }
    cuts
}

#[cfg(test)]
mod tests;
