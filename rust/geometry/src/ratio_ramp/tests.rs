use super::*;

const SAMPLES: usize = 200_000;

fn smoothstep(u: f64) -> (f64, f64, f64) {
    (u * u * (3.0 - 2.0 * u), 6.0 * u * (1.0 - u), 6.0 - 12.0 * u)
}

/// The advanced follower's peak velocity and acceleration over a ramp,
/// sampled in time: the ratio `r_b + Δr·S(t/τ)` laid at a leader speed
/// `v(t) = v + a·(t − τ/2)`, read by the advance as `f + k·ḟ`.
fn sampled_demand(k: f64, tau: f64, ratio_before: f64, step: &RatioStep) -> (f64, f64) {
    let (v, a) = (step.speed_mm_s, step.accel_mm_s2);
    (0..=SAMPLES).fold((0.0_f64, 0.0_f64), |(velocity, accel), i| {
        let u = i as f64 / SAMPLES as f64;
        let speed = v + a * (u - 0.5) * tau;
        let (s, ds, dds) = smoothstep(u);
        let ratio = ratio_before + step.ratio_step * s;
        let ratio_rate = step.ratio_step * ds / tau;
        let ratio_accel = step.ratio_step * dds / (tau * tau);
        let flow = ratio * speed;
        let flow_rate = ratio_rate * speed + ratio * a;
        let flow_accel = ratio_accel * speed + 2.0 * ratio_rate * a;
        (
            velocity.max((flow + k * flow_rate).abs()),
            accel.max((flow_rate + k * flow_accel).abs()),
        )
    })
}

#[test]
fn the_epanechnikov_step_is_a_smoothstep_over_the_ramp_time() {
    let cdf = |x: f64| {
        let n = 20_000;
        (0..n)
            .map(|i| {
                let y = -1.0 + (x + 1.0) * (i as f64 + 0.5) / n as f64;
                0.75 * (1.0 - y * y) * (x + 1.0) / n as f64
            })
            .sum::<f64>()
    };
    for x in [-0.9, -0.5, 0.0, 0.3, 0.8] {
        let u = 0.5 * (x + 1.0);
        assert!((cdf(x) - smoothstep(u).0).abs() < 1e-8, "at x={x}");
    }
}

fn unlimited(k: f64) -> RatioRampRule {
    RatioRampRule {
        advance_slope_s: k,
        max_velocity_mm_s: f64::INFINITY,
        max_accel_mm_s2: f64::INFINITY,
    }
}

#[test]
fn at_a_steady_speed_the_demand_is_the_sampled_peak_of_the_advanced_smoothstep() {
    for k in [0.0, 0.02, 0.05, 0.5] {
        for tau in [0.005, 0.012, 0.03] {
            let step = RatioStep {
                ratio_step: 0.04,
                peak_ratio: 0.055,
                speed_mm_s: 100.0,
                accel_mm_s2: 0.0,
                jerk_mm_s3: 0.0,
            };
            let (velocity, accel) = unlimited(k).demand(tau, &step);
            let (sampled_velocity, sampled_accel) = sampled_demand(k, tau, 0.015, &step);
            assert!(
                (velocity - sampled_velocity).abs() <= 1e-6 * velocity
                    && (accel - sampled_accel).abs() <= 1e-6 * accel,
                "k={k} τ={tau}: {velocity}/{accel} against sampled {sampled_velocity}/{sampled_accel}"
            );
        }
    }
}

#[test]
fn under_acceleration_the_demand_bounds_the_sampled_ramp_at_its_fastest() {
    for k in [0.0, 0.02, 0.5] {
        for tau in [0.005, 0.03] {
            for accel in [-3000.0, 3000.0] {
                let crossing = RatioStep {
                    ratio_step: 0.033,
                    peak_ratio: 0.033,
                    speed_mm_s: 20.0,
                    accel_mm_s2: accel,
                    jerk_mm_s3: 0.0,
                };
                let fastest = RatioStep {
                    speed_mm_s: 20.0 + 3000.0 * 0.5 * tau,
                    ..crossing
                };
                let (velocity, accel_bound) = unlimited(k).demand(tau, &fastest);
                let (sampled_velocity, sampled_accel) = sampled_demand(k, tau, 0.0, &crossing);
                assert!(
                    sampled_velocity <= velocity * (1.0 + 1e-9)
                        && sampled_accel <= accel_bound * (1.0 + 1e-9),
                    "k={k} τ={tau} a={accel}: sampled {sampled_velocity}/{sampled_accel} past \
                     {velocity}/{accel_bound}"
                );
            }
        }
    }
}

const P1: RatioRampRule = RatioRampRule {
    advance_slope_s: 0.5,
    max_velocity_mm_s: 45.0,
    max_accel_mm_s2: 2000.0,
};

fn cruise(ratio_step: f64, speed_mm_s: f64) -> RatioStep {
    RatioStep {
        ratio_step,
        peak_ratio: ratio_step,
        speed_mm_s,
        accel_mm_s2: 0.0,
        jerk_mm_s3: 0.0,
    }
}

#[test]
fn the_ramp_time_is_the_shortest_that_fits() {
    let rule = RatioRampRule {
        advance_slope_s: 0.03,
        max_velocity_mm_s: 30.0,
        max_accel_mm_s2: 5000.0,
    };
    let step = cruise(0.04, 200.0);
    let tau = rule.ramp_time(&step);
    assert!(tau > RATIO_RAMP_MIN_TIME_S && tau < RATIO_RAMP_MAX_TIME_S);
    let (_, accel) = rule.demand(tau, &step);
    assert!((accel - 5000.0).abs() < 1e-6, "{accel}");
    assert!((tau - (6.0 * 0.03 * 8.0 / 5000.0_f64).sqrt()).abs() < 1e-12);
}

#[test]
fn a_small_step_takes_the_shortest_ramp_and_a_large_one_the_longest() {
    assert_eq!(P1.ramp_time(&cruise(0.0, 100.0)), RATIO_RAMP_MIN_TIME_S);
    assert_eq!(P1.ramp_time(&cruise(0.04, 250.0)), RATIO_RAMP_MAX_TIME_S);
}

#[test]
fn the_leaders_acceleration_lengthens_the_ramp() {
    let slow = cruise(0.033, 0.3);
    let accelerating = RatioStep {
        accel_mm_s2: 460.0,
        ..slow
    };
    assert_eq!(P1.ramp_time(&slow), RATIO_RAMP_MIN_TIME_S);
    assert!(P1.ramp_time(&accelerating) > 0.011);
}

#[test]
fn without_an_advance_the_base_flow_sets_the_ramp() {
    let rule = RatioRampRule {
        advance_slope_s: 0.0,
        max_velocity_mm_s: 50.0,
        max_accel_mm_s2: 1000.0,
    };
    let tau = rule.ramp_time(&cruise(0.1, 200.0));
    assert!((tau - 1.5 * 20.0 / 1000.0).abs() < 1e-12, "{tau}");
}

#[test]
fn the_step_speed_cap_holds_the_longest_ramp_at_the_limits() {
    let ratio = 0.4 * 0.2 / (std::f64::consts::PI * 0.875 * 0.875);
    let cap = P1.step_speed_cap(ratio, ratio, 0.0, 0.0);
    assert!(
        cap > 17.0 && cap < 19.0,
        "a bowden travel-to-extrusion switch caps at {cap}"
    );
    let (velocity, accel) = P1.demand(RATIO_RAMP_MAX_TIME_S, &cruise(ratio, cap));
    assert!(velocity <= 45.0 + 1e-9 && accel <= 2000.0 + 1e-9);
    assert!((accel - 2000.0).abs() < 1e-6 || (velocity - 45.0).abs() < 1e-6);
    assert_eq!(P1.step_speed_cap(0.0, ratio, 0.0, 0.0), f64::INFINITY);
    let under_accel = P1.step_speed_cap(ratio, ratio, 1000.0, 0.0);
    assert!(under_accel < cap, "{under_accel} against {cap}");
    assert_eq!(P1.step_speed_cap(ratio, ratio, 5000.0, 0.0), 0.0);
}

const P4: RatioRampRule = RatioRampRule {
    advance_slope_s: 0.02,
    max_velocity_mm_s: 50.0,
    max_accel_mm_s2: 5000.0,
};
const P4_PEAK_RATIO: f64 = 0.0427;
const P4_SPEED_MM_S: f64 = 65.0;
const P4_ACCEL_MM_S2: f64 = 3750.0;

/// A step crossed where the leaders' own advanced flow asks for `share`
/// of the follower's acceleration limit.
fn crossing_at_path_share(ratio_step: f64, share: f64) -> RatioStep {
    let own_accel = share * P4.max_accel_mm_s2 / P4_PEAK_RATIO;
    RatioStep {
        ratio_step,
        peak_ratio: P4_PEAK_RATIO,
        speed_mm_s: P4_SPEED_MM_S,
        accel_mm_s2: P4_ACCEL_MM_S2,
        jerk_mm_s3: (own_accel - P4_ACCEL_MM_S2).max(0.0) / P4.advance_slope_s,
    }
}

const PATH_SHARES: [f64; 7] = [0.0, 0.5, 0.9, 0.99, 0.999_999, 1.0, 1.5];
const SWEEP_SAMPLES: usize = 20_000;
/// The largest change in ramp time between neighbouring samples of a sweep
/// a continuous ramp time takes.
const SWEEP_TAU_JUMP_S: f64 = 1e-3 * (RATIO_RAMP_MAX_TIME_S - RATIO_RAMP_MIN_TIME_S);

fn largest_jump(taus: impl Iterator<Item = f64>) -> f64 {
    taus.collect::<Vec<_>>()
        .windows(2)
        .map(|pair| (pair[1] - pair[0]).abs())
        .fold(0.0, f64::max)
}

#[test]
fn the_ramp_time_is_continuous_in_the_step_and_shortest_as_it_vanishes() {
    for share in PATH_SHARES {
        let tau = |ratio_step: f64| P4.ramp_time(&crossing_at_path_share(ratio_step, share));
        assert_eq!(
            tau(1e-9),
            RATIO_RAMP_MIN_TIME_S,
            "a vanishing step takes the shortest ramp with the path at {share} of the limit"
        );
        let jump = largest_jump(
            (0..=SWEEP_SAMPLES).map(|index| tau(0.05 * index as f64 / SWEEP_SAMPLES as f64)),
        );
        assert!(
            jump <= SWEEP_TAU_JUMP_S,
            "the ramp time jumps by {jump} s across a step sweep with the path at {share} of \
             the limit"
        );
    }
}

#[test]
fn the_ramp_time_is_continuous_in_the_leaders_own_demand() {
    for ratio_step in [3e-4, 3e-3, 0.03] {
        let jump = largest_jump((0..=SWEEP_SAMPLES).map(|index| {
            let share = 2.0 * index as f64 / SWEEP_SAMPLES as f64;
            P4.ramp_time(&crossing_at_path_share(ratio_step, share))
        }));
        assert!(
            jump <= SWEEP_TAU_JUMP_S,
            "a {ratio_step} step's ramp time jumps by {jump} s as the path's own demand sweeps \
             through the limit"
        );
    }
}
