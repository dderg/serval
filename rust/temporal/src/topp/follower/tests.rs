use crate::topp::chain::tests_support::line;
use crate::topp::{
    ToleranceMode, schedule_segment_with_followers, schedule_segment_with_tolerance,
};
use crate::{AxisSet, BindingConstraint, FollowerDemand, GridConfig, GridScheme, LimitSet, Limits};

fn limits_with_follower(v_e: f64, a_e: f64, j_e: f64) -> Limits {
    let mut sets: Vec<LimitSet> = Limits::axis_boxes([500.0; 3], [20_000.0; 3], [400_000.0; 3])
        .sets()
        .to_vec();
    sets.push(LimitSet {
        axes: AxisSet::from_indices(&[3]),
        v_max: v_e,
        a_max: a_e,
        j_max: j_e,
    });
    Limits::try_new(&sets, 4).unwrap()
}

fn grid() -> GridConfig {
    GridConfig {
        scheme: GridScheme::UniformArclength,
        n: 101,
    }
}

fn solve_with_followers(limits: &Limits, followers: &[FollowerDemand]) -> crate::TopProfile {
    let curve = line([0.0; 3], [100.0, 0.0, 0.0]);
    schedule_segment_with_followers(
        &curve,
        limits,
        &grid(),
        0.0,
        0.0,
        ToleranceMode::Tight,
        followers,
    )
    .unwrap()
}

#[test]
fn follower_velocity_caps_cruise_speed() {
    let limits = limits_with_follower(50.0, 1.0e9, 1.0e12);
    let profile = solve_with_followers(
        &limits,
        &[FollowerDemand {
            axis: 3,
            ratio: 0.5,
            pa_k: 0.0,
        }],
    );
    let peak = profile.samples.iter().map(|s| s.v).fold(0.0, f64::max);
    assert!(
        (peak - 100.0).abs() < 1.0,
        "expected cruise cap 100 mm/s, got {peak}"
    );
}

#[test]
fn follower_accel_caps_path_accel() {
    let limits = limits_with_follower(1.0e6, 500.0, 1.0e12);
    let profile = solve_with_followers(
        &limits,
        &[FollowerDemand {
            axis: 3,
            ratio: 0.5,
            pa_k: 0.0,
        }],
    );
    let mut t = 0.0;
    let mut max_dvdt: f64 = 0.0;
    for w in profile.samples.windows(2) {
        let v_avg = (w[0].v + w[1].v).max(1e-9);
        let dt = 2.0 * (w[1].s - w[0].s) / v_avg;
        if dt > 1e-12 {
            max_dvdt = max_dvdt.max(((w[1].v - w[0].v) / dt).abs());
        }
        t += dt;
    }
    let _ = t;
    assert!(
        max_dvdt <= 1000.0 * 1.01,
        "path accel {max_dvdt} exceeds 1000"
    );
}

#[test]
fn zero_follower_slice_matches_plain_solve() {
    let limits = limits_with_follower(1.0e6, 1.0e9, 1.0e12);
    let curve = line([0.0; 3], [100.0, 0.0, 0.0]);
    let with_empty = schedule_segment_with_followers(
        &curve,
        &limits,
        &grid(),
        0.0,
        0.0,
        ToleranceMode::Tight,
        &[],
    )
    .unwrap();
    let plain =
        schedule_segment_with_tolerance(&curve, &limits, &grid(), 0.0, 0.0, ToleranceMode::Tight)
            .unwrap();
    assert!((with_empty.total_time - plain.total_time).abs() < 1e-12);
}

#[test]
fn binding_tag_names_the_follower_set() {
    let limits = limits_with_follower(50.0, 1.0e9, 1.0e12);
    let follower_set_idx = limits.follower_sets().next().map(|(idx, _)| idx).unwrap();
    let profile = solve_with_followers(
        &limits,
        &[FollowerDemand {
            axis: 3,
            ratio: 0.5,
            pa_k: 0.0,
        }],
    );
    let mid = &profile.samples[profile.samples.len() / 2];
    assert_eq!(
        mid.binding,
        BindingConstraint::Velocity {
            set: follower_set_idx
        },
        "cruise sample should bind on the follower velocity row"
    );
}

#[test]
fn follower_jerk_cap_binds_through_the_slp() {
    let limits = limits_with_follower(1.0e6, 1.0e9, 1000.0);
    let profile = solve_with_followers(
        &limits,
        &[FollowerDemand {
            axis: 3,
            ratio: 0.5,
            pa_k: 0.0,
        }],
    );
    let n = profile.samples.len();
    let t: Vec<f64> = {
        let mut acc = vec![0.0];
        for w in profile.samples.windows(2) {
            let v_avg = (w[0].v + w[1].v).max(1e-9);
            acc.push(acc.last().unwrap() + 2.0 * (w[1].s - w[0].s) / v_avg);
        }
        acc
    };
    let mut max_jerk: f64 = 0.0;
    for i in 1..n - 1 {
        let dt_l = t[i] - t[i - 1];
        let dt_r = t[i + 1] - t[i];
        if dt_l < 1e-9 || dt_r < 1e-9 {
            continue;
        }
        let a_l = (profile.samples[i].v - profile.samples[i - 1].v) / dt_l;
        let a_r = (profile.samples[i + 1].v - profile.samples[i].v) / dt_r;
        max_jerk = max_jerk.max(((a_r - a_l) / (0.5 * (dt_l + dt_r))).abs());
    }
    assert!(
        max_jerk <= 2000.0 * 1.10,
        "path jerk {max_jerk} exceeds effective cap 2000"
    );
}

fn finite_diff_derivatives(profile: &crate::TopProfile) -> Vec<(f64, f64, f64, f64)> {
    let n = profile.samples.len();
    let mut t = vec![0.0];
    for w in profile.samples.windows(2) {
        let v_avg = (w[0].v + w[1].v).max(1e-9);
        t.push(t.last().unwrap() + 2.0 * (w[1].s - w[0].s) / v_avg);
    }
    let v: Vec<f64> = profile.samples.iter().map(|s| s.v).collect();
    let deriv = |f: &[f64]| -> Vec<f64> {
        (0..n)
            .map(|i| {
                if i == 0 || i == n - 1 {
                    0.0
                } else {
                    (f[i + 1] - f[i - 1]) / (t[i + 1] - t[i - 1]).max(1e-12)
                }
            })
            .collect()
    };
    let a = deriv(&v);
    let j = deriv(&a);
    let s4 = deriv(&j);
    (0..n).map(|i| (v[i], a[i], j[i], s4[i])).collect()
}

#[test]
fn pa_velocity_row_slows_the_accel_phase() {
    let limits = limits_with_follower(50.0, 1.0e9, 1.0e12);
    let profile = solve_with_followers(
        &limits,
        &[FollowerDemand {
            axis: 3,
            ratio: 0.5,
            pa_k: 0.05,
        }],
    );
    let d = finite_diff_derivatives(&profile);
    for (i, &(v, a, _, _)) in d.iter().enumerate() {
        let demand = 0.5 * (v + 0.05 * a);
        assert!(
            demand <= 50.0 * (1.0 + 5e-2),
            "sample {i}: PA velocity demand {demand} > 50"
        );
    }
}

#[test]
fn pa_accel_row_holds_pointwise() {
    let limits = limits_with_follower(1.0e6, 500.0, 1.0e12);
    let profile = solve_with_followers(
        &limits,
        &[FollowerDemand {
            axis: 3,
            ratio: 0.5,
            pa_k: 0.05,
        }],
    );
    let d = finite_diff_derivatives(&profile);
    for (i, &(_, a, j, _)) in d.iter().enumerate().skip(2).take(d.len() - 4) {
        let demand = 0.5 * (a + 0.05 * j).abs();
        assert!(
            demand <= 500.0 * 1.10,
            "sample {i}: PA accel demand {demand} > 500"
        );
    }
}

#[test]
fn pa_jerk_row_holds_pointwise() {
    let limits = limits_with_follower(1.0e6, 1.0e9, 5000.0);
    let profile = solve_with_followers(
        &limits,
        &[FollowerDemand {
            axis: 3,
            ratio: 0.5,
            pa_k: 0.05,
        }],
    );
    let d = finite_diff_derivatives(&profile);
    for (i, &(_, _, j, s4)) in d.iter().enumerate().skip(3).take(d.len() - 6) {
        let demand = 0.5 * (j + 0.05 * s4).abs();
        assert!(
            demand <= 5000.0 * 1.20,
            "sample {i}: PA jerk demand {demand} > 5000"
        );
    }
}

#[test]
fn verify_tags_pa_rows() {
    let limits = limits_with_follower(50.0, 1.0e9, 1.0e12);
    let follower_set_idx = limits.follower_sets().next().map(|(idx, _)| idx).unwrap();
    let profile = solve_with_followers(
        &limits,
        &[FollowerDemand {
            axis: 3,
            ratio: 0.5,
            pa_k: 0.05,
        }],
    );
    let tagged = profile.samples.iter().any(|s| {
        s.binding
            == BindingConstraint::PaVelocity {
                set: follower_set_idx,
            }
    });
    assert!(tagged, "no sample tagged PaVelocity");
}

use crate::topp::window::eval_kernel;
use nurbs::algebra::PiecewisePolynomialKernel;

fn bell_kernel(frequency_hz: f64) -> PiecewisePolynomialKernel<f64> {
    let t_sm = 0.8025 / frequency_hz;
    let h = t_sm / 2.0;
    let c = 15.0 / (16.0 * h.powi(5));
    PiecewisePolynomialKernel::single_poly_from_absolute(
        vec![c * h.powi(4), 0.0, -2.0 * c * h * h, 0.0, c],
        (-h, h),
    )
}

fn solve_folded(
    limits: &Limits,
    followers: &[FollowerDemand],
    kernels: [Option<PiecewisePolynomialKernel<f64>>; 3],
    history: Option<crate::FollowerHistory>,
) -> crate::TopProfile {
    let curve = line([0.0; 3], [100.0, 0.0, 0.0]);
    let arc = crate::topp::path::sample_arclength_grid(&curve, 401).unwrap();
    let mut chain = crate::topp::chain::ChainGrid::try_from_segment_grids_with_followers(
        vec![arc],
        vec![*limits],
        vec![followers.to_vec()],
        &[false],
    )
    .unwrap();
    chain.axis_kernels = kernels;
    chain.follower_history = history;
    crate::topp::schedule_chain_with_tolerance(
        &chain,
        crate::topp::EndpointConditions {
            v_start: 0.0,
            v_end: 0.0,
            a_start: None,
        },
        ToleranceMode::Tight,
    )
    .unwrap()
}

fn solve_identity_dense(limits: &Limits, followers: &[FollowerDemand]) -> crate::TopProfile {
    let curve = line([0.0; 3], [100.0, 0.0, 0.0]);
    schedule_segment_with_followers(
        &curve,
        limits,
        &GridConfig {
            scheme: GridScheme::UniformArclength,
            n: 401,
        },
        0.0,
        0.0,
        ToleranceMode::Tight,
        followers,
    )
    .unwrap()
}

#[test]
fn passthrough_kernels_reproduce_identity_rows() {
    let limits = limits_with_follower(50.0, 1.0e9, 1.0e12);
    let demand = [FollowerDemand {
        axis: 3,
        ratio: 0.5,
        pa_k: 0.0,
    }];
    let folded = solve_folded(&limits, &demand, [None, None, None], None);
    let identity = solve_identity_dense(&limits, &demand);
    assert!((folded.total_time - identity.total_time).abs() < 1e-9);
}

#[test]
fn folded_rows_recover_speed_at_a_smoothed_start() {
    let limits = limits_with_follower(50.0, 1.0e9, 1.0e12);
    let demand = [FollowerDemand {
        axis: 3,
        ratio: 0.5,
        pa_k: 0.0,
    }];
    let folded = solve_folded(
        &limits,
        &demand,
        [Some(bell_kernel(40.0)), None, None],
        None,
    );
    let identity = solve_identity_dense(&limits, &demand);
    assert!(
        folded.total_time < identity.total_time - 1e-4,
        "folded {} should beat identity {} (shaped speed lags during ramp)",
        folded.total_time,
        identity.total_time
    );
}

fn brute_force_shaped_demand_check(
    profile: &crate::TopProfile,
    kernel: &PiecewisePolynomialKernel<f64>,
    ratio: f64,
    v_max: f64,
    history_v: f64,
) {
    let n = profile.samples.len();
    let mut t = vec![0.0];
    for w in profile.samples.windows(2) {
        let v_avg = (w[0].v + w[1].v).max(1e-9);
        t.push(t.last().unwrap() + 2.0 * (w[1].s - w[0].s) / v_avg);
    }
    let total = *t.last().unwrap();
    let v_at = |tau: f64| -> f64 {
        if tau < 0.0 {
            return history_v;
        }
        if tau >= total {
            return profile.samples[n - 1].v;
        }
        let j = t.partition_point(|&tj| tj <= tau).min(n - 1);
        let (t0, t1) = (t[j - 1], t[j]);
        let (v0, v1) = (profile.samples[j - 1].v, profile.samples[j].v);
        v0 + (v1 - v0) * (tau - t0) / (t1 - t0).max(1e-12)
    };
    let (k_lo, k_hi) = kernel.support();
    let dt = 1e-3;
    let mut tau = 0.0;
    while tau <= total {
        let mut shaped = 0.0;
        let mut z = k_lo;
        while z <= k_hi {
            shaped += eval_kernel(kernel, z) * v_at(tau - z) * dt;
            z += dt;
        }
        let demand = ratio * shaped.abs();
        assert!(
            demand <= v_max * (1.0 + 5e-2),
            "t={tau:.4}: shaped demand {demand} > {v_max}"
        );
        tau += 5e-3;
    }
}

#[test]
fn folded_demand_holds_against_brute_force_convolution() {
    let limits = limits_with_follower(50.0, 1.0e9, 1.0e12);
    let kernel = bell_kernel(40.0);
    let folded = solve_folded(
        &limits,
        &[FollowerDemand {
            axis: 3,
            ratio: 0.5,
            pa_k: 0.0,
        }],
        [Some(kernel.clone()), None, None],
        None,
    );
    brute_force_shaped_demand_check(&folded, &kernel, 0.5, 50.0, 0.0);
}

#[test]
fn nonzero_history_constrains_the_chain_start() {
    let limits = limits_with_follower(50.0, 1.0e9, 1.0e12);
    let kernel = bell_kernel(40.0);
    let half = kernel.support().1;
    let history = crate::FollowerHistory {
        dt: half / 32.0,
        axis_velocity: [vec![100.0; 32], Vec::new(), Vec::new()],
    };
    let folded = solve_folded(
        &limits,
        &[FollowerDemand {
            axis: 3,
            ratio: 0.5,
            pa_k: 0.0,
        }],
        [Some(kernel.clone()), None, None],
        Some(history),
    );
    brute_force_shaped_demand_check(&folded, &kernel, 0.5, 50.0, 100.0);
}

#[test]
fn refreeze_divergence_fails_loudly() {
    let limits = limits_with_follower(50.0, 1.0e9, 1.0e12);
    let curve = line([0.0; 3], [100.0, 0.0, 0.0]);
    let arc = crate::topp::path::sample_arclength_grid(&curve, 401).unwrap();
    let mut chain = crate::topp::chain::ChainGrid::try_from_segment_grids_with_followers(
        vec![arc],
        vec![limits],
        vec![vec![FollowerDemand {
            axis: 3,
            ratio: 0.5,
            pa_k: 0.0,
        }]],
        &[false],
    )
    .unwrap();
    chain.axis_kernels = [Some(bell_kernel(40.0)), None, None];
    let err = crate::topp::schedule_chain_with_refreeze_cap(
        &chain,
        crate::topp::EndpointConditions {
            v_start: 0.0,
            v_end: 0.0,
            a_start: None,
        },
        ToleranceMode::Tight,
        1,
    )
    .unwrap_err();
    assert!(matches!(
        err,
        crate::topp::ScheduleError::FollowerSlpDiverged { refreezes: 1, .. }
    ));
}

#[test]
fn batch_tail_exchange_holds_shaped_demand_across_a_stop() {
    use crate::multi::{BatchInput, BatchShaping, GridStrategy, SegmentInput, plan_batch};
    let limits = limits_with_follower(50.0, 1.0e9, 1.0e12);
    let demand = [FollowerDemand {
        axis: 3,
        ratio: 0.5,
        pa_k: 0.0,
    }];
    let a = line([0.0; 3], [50.0, 0.0, 0.0]);
    let b = line([50.0, 0.0, 0.0], [50.0, 50.0, 0.0]);
    let segments = [
        SegmentInput {
            curve: &a,
            limits,
            followers: &demand,
            virtual_path: None,
        },
        SegmentInput {
            curve: &b,
            limits,
            followers: &demand,
            virtual_path: None,
        },
    ];
    let shaping = BatchShaping {
        axis_kernels: [Some(bell_kernel(40.0)), Some(bell_kernel(40.0)), None],
        follower_history: None,
    };
    let out = plan_batch(BatchInput {
        segments: &segments,
        shaping: Some(&shaping),
        grid_strategy: GridStrategy::Fixed(201),
        worker_threads: 1,
        initial_velocity: 0.0,
        initial_accel: 0.0,
        terminal_velocity: 0.0,
    })
    .unwrap();
    assert_eq!(out.profiles.len(), 2);
    for profile in &out.profiles {
        let peak = profile.samples.iter().map(|s| s.v).fold(0.0, f64::max);
        assert!(peak > 50.0, "chains should still move briskly, got {peak}");
    }
    brute_force_shaped_demand_check(&out.profiles[0], &bell_kernel(40.0), 0.5, 50.0, 0.0);
}

#[test]
fn virtual_path_plans_under_follower_limits_and_feedrate() {
    let limits = limits_with_follower(75.0, 1500.0, 1.0e9);
    let followers = vec![FollowerDemand {
        axis: 3,
        ratio: 1.0,
        pa_k: 0.0,
    }];
    let solve = |feedrate: f64| {
        let chain = crate::topp::chain::ChainGrid::virtual_path(
            10.0,
            51,
            limits,
            followers.clone(),
            feedrate,
        )
        .unwrap();
        crate::topp::schedule_chain_with_tolerance(
            &chain,
            crate::topp::EndpointConditions {
                v_start: 0.0,
                v_end: 0.0,
                a_start: None,
            },
            ToleranceMode::Tight,
        )
        .unwrap()
    };
    let slow = solve(40.0);
    let peak_slow = slow.samples.iter().map(|s| s.v).fold(0.0, f64::max);
    assert!(
        (peak_slow - 40.0).abs() < 1.0,
        "feedrate 40 should cap cruise, got {peak_slow}"
    );
    let fast = solve(200.0);
    let peak_fast = fast.samples.iter().map(|s| s.v).fold(0.0, f64::max);
    assert!(
        (peak_fast - 75.0).abs() < 1.5,
        "follower v_max 75 should cap cruise, got {peak_fast}"
    );
    let mut max_dvdt: f64 = 0.0;
    for w in fast.samples.windows(2) {
        let v_avg = (w[0].v + w[1].v).max(1e-9);
        let dt = 2.0 * (w[1].s - w[0].s) / v_avg;
        if dt > 1e-12 {
            max_dvdt = max_dvdt.max(((w[1].v - w[0].v) / dt).abs());
        }
    }
    assert!(max_dvdt <= 1500.0 * 1.02, "accel {max_dvdt} > 1500");
}

#[test]
fn binding_summary_reports_velocity_pin() {
    let limits = limits_with_follower(50.0, 1.0e9, 1.0e12);
    let profile = solve_with_followers(
        &limits,
        &[FollowerDemand {
            axis: 3,
            ratio: 0.5,
            pa_k: 0.0,
        }],
    );

    let worst = profile
        .binding
        .worst
        .expect("a velocity-capped cruise must produce a worst-pinned sample");
    assert!(
        matches!(
            worst.constraint,
            crate::BindingConstraint::Velocity { .. } | crate::BindingConstraint::JerkNorm { .. }
        ),
        "worst pin should ride a real kinematic cap (velocity or its slack-floating jerk), got {:?}",
        worst.constraint
    );
    assert!(
        (0.9..=1.06).contains(&worst.ratio),
        "cruise rides the cap; ratio = {}",
        worst.ratio
    );

    let velocity_count: u32 = profile
        .binding
        .histogram
        .iter()
        .filter(|(c, _)| matches!(c, crate::BindingConstraint::Velocity { .. }))
        .map(|(_, n)| *n)
        .sum();
    assert!(
        velocity_count > 0,
        "cruise samples should tally as Velocity bindings"
    );

    let counts: Vec<u32> = profile.binding.histogram.iter().map(|(_, n)| *n).collect();
    assert!(
        counts.windows(2).all(|w| w[0] >= w[1]),
        "histogram must be sorted by descending count; got {:?}",
        counts
    );
}
