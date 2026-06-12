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
