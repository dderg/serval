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
    let cap = P1.step_speed_cap(ratio, ratio, 0.0, 0.0, f64::INFINITY);
    assert!(
        cap > 17.0 && cap < 19.0,
        "a bowden travel-to-extrusion switch caps at {cap}"
    );
    let (velocity, accel) = P1.demand(RATIO_RAMP_MAX_TIME_S, &cruise(ratio, cap));
    assert!(velocity <= 45.0 + 1e-9 && accel <= 2000.0 + 1e-9);
    assert!((accel - 2000.0).abs() < 1e-6 || (velocity - 45.0).abs() < 1e-6);
    assert_eq!(
        P1.step_speed_cap(0.0, ratio, 0.0, 0.0, f64::INFINITY),
        f64::INFINITY
    );
    let under_accel = P1.step_speed_cap(ratio, ratio, 1000.0, 0.0, f64::INFINITY);
    assert!(under_accel < cap, "{under_accel} against {cap}");
    assert_eq!(
        P1.step_speed_cap(ratio, ratio, 5000.0, 0.0, f64::INFINITY),
        0.0
    );
}

#[test]
fn a_rest_beside_a_step_caps_its_speed_at_the_ramp_the_room_leaves() {
    let rule = RatioRampRule {
        advance_slope_s: 0.03,
        max_velocity_mm_s: 50.0,
        max_accel_mm_s2: 20000.0,
    };
    let (ratio, accel, room_mm) = (0.02743, 25409.0, 10.4);
    let free = rule.step_speed_cap(ratio, ratio, accel, 0.0, f64::INFINITY);
    let beside_rest = rule.step_speed_cap(ratio, ratio, accel, 0.0, room_mm);
    assert!(
        beside_rest < 0.8 * free,
        "a rest {room_mm} mm away caps the step at {beside_rest} mm/s, free {free} mm/s"
    );
    let ramp_time = rest_room_ramp_time(beside_rest, room_mm);
    assert!(ramp_time < RATIO_RAMP_MAX_TIME_S, "{ramp_time}");
    let step = RatioStep {
        ratio_step: ratio,
        peak_ratio: ratio,
        speed_mm_s: beside_rest,
        accel_mm_s2: accel,
        jerk_mm_s3: 0.0,
    };
    let (velocity, accel_demand) = rule.demand(ramp_time, &step);
    assert!(velocity <= 50.0 + 1e-9 && accel_demand <= 20000.0 + 1e-9);
    assert!(
        (velocity - 50.0).abs() < 1e-6 || (accel_demand - 20000.0).abs() < 1e-6,
        "the capped step asks {velocity} mm/s and {accel_demand} mm/s² of the extruder"
    );
    let past_reach = rest_room_reach_mm(free) * (1.0 + 1e-9);
    assert_eq!(
        rule.step_speed_cap(ratio, ratio, accel, 0.0, past_reach),
        free
    );
}
