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
        kernel_jerk_per_accel_hz: f64::INFINITY,
    }
}

#[test]
fn at_a_steady_speed_the_demand_is_the_sampled_peak_of_the_advanced_smoothstep() {
    for k in [0.0, 0.02, 0.05, 0.5] {
        for tau in [0.005, 0.012, 0.03] {
            let step = RatioStep {
                ratio_step: 0.04,
                peak_ratio: 0.055,
                slides: [RatioSlide::FLAT; 2],
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
                    slides: [RatioSlide::FLAT; 2],
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
    kernel_jerk_per_accel_hz: f64::INFINITY,
};

fn cruise(ratio_step: f64, speed_mm_s: f64) -> RatioStep {
    RatioStep {
        ratio_step,
        peak_ratio: ratio_step,
        slides: [RatioSlide::FLAT; 2],
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
        kernel_jerk_per_accel_hz: f64::INFINITY,
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
        kernel_jerk_per_accel_hz: f64::INFINITY,
    };
    let tau = rule.ramp_time(&cruise(0.1, 200.0));
    assert!((tau - 1.5 * 20.0 / 1000.0).abs() < 1e-12, "{tau}");
}

/// A ratio step the leaders cross at whatever speed the search tries, their
/// own flow at most `own_flow_mm_s`, unshaped: `accel` faster half the
/// longest ramp either side of it.
#[derive(Debug, Clone, Copy)]
struct Seam {
    ratio_step: f64,
    peak_ratio: f64,
    accel_mm_s2: f64,
    rest_room_mm: f64,
    own_flow_mm_s: f64,
}

impl Seam {
    fn crossing(self, speed_mm_s: f64) -> StepCrossing {
        StepCrossing {
            step: RatioStep {
                ratio_step: self.ratio_step,
                peak_ratio: self.peak_ratio,
                slides: [RatioSlide::FLAT; 2],
                speed_mm_s,
                accel_mm_s2: self.accel_mm_s2,
                jerk_mm_s3: 0.0,
            },
            rest_room_mm: self.rest_room_mm,
            edge_speed_mm_s: self.edge_speed(speed_mm_s),
            own_flow_mm_s: self.own_flow_mm_s,
        }
    }

    fn edge_speed(self, speed_mm_s: f64) -> f64 {
        speed_mm_s + 0.5 * self.accel_mm_s2 * longest_ramp_time(speed_mm_s, self.rest_room_mm)
    }

    fn cap(self, rule: &RatioRampRule, fastest_mm_s: f64) -> f64 {
        rule.step_speed_cap(fastest_mm_s, |speed_mm_s| self.crossing(speed_mm_s))
    }

    /// The cap were the leaders no faster at the ramp's edges than at the
    /// step.
    fn cap_read_at_the_step(self, rule: &RatioRampRule, fastest_mm_s: f64) -> f64 {
        rule.step_speed_cap(fastest_mm_s, |speed_mm_s| StepCrossing {
            edge_speed_mm_s: speed_mm_s,
            ..self.crossing(speed_mm_s)
        })
    }

    /// What the step asks of the follower at `speed_mm_s` over a ramp of
    /// `tau`: the own flow and the ramp's velocity at the step, the ramp's
    /// acceleration at its edges.
    fn demand(self, rule: &RatioRampRule, tau: f64, speed_mm_s: f64) -> (f64, f64) {
        let crossing = self.crossing(speed_mm_s);
        let at_edges = RatioStep {
            speed_mm_s: crossing.edge_speed_mm_s,
            ..crossing.step
        };
        let (own_velocity, own_accel) = rule.own_demand(&crossing.step);
        (
            own_velocity + rule.ramp_velocity(tau, &crossing.step),
            own_accel + rule.ramp_accel(tau, &at_edges),
        )
    }
}

fn seam(ratio: f64, accel_mm_s2: f64, rest_room_mm: f64) -> Seam {
    Seam {
        ratio_step: ratio,
        peak_ratio: ratio,
        accel_mm_s2,
        rest_room_mm,
        own_flow_mm_s: f64::INFINITY,
    }
}

const FASTEST_MM_S: f64 = 1000.0;

#[test]
fn the_step_speed_cap_holds_the_longest_ramp_at_the_limits() {
    let ratio = 0.4 * 0.2 / (std::f64::consts::PI * 0.875 * 0.875);
    let cap = seam(ratio, 0.0, f64::INFINITY).cap(&P1, FASTEST_MM_S);
    assert!(
        cap > 17.0 && cap < 19.0,
        "a bowden travel-to-extrusion switch caps at {cap}"
    );
    let (velocity, accel) = P1.demand(RATIO_RAMP_MAX_TIME_S, &cruise(ratio, cap));
    assert!(velocity <= 45.0 + 1e-9 && accel <= 2000.0 + 1e-9);
    assert!((accel - 2000.0).abs() < 1e-6 || (velocity - 45.0).abs() < 1e-6);
    assert_eq!(
        seam(0.0, 0.0, f64::INFINITY).cap(&P1, FASTEST_MM_S),
        f64::INFINITY
    );
    assert_eq!(
        seam(ratio, 0.0, f64::INFINITY).cap(&P1, 0.9 * cap),
        f64::INFINITY
    );
    let under_accel = seam(ratio, 1000.0, f64::INFINITY).cap(&P1, FASTEST_MM_S);
    assert!(under_accel < cap, "{under_accel} against {cap}");
    assert_eq!(
        seam(ratio, 5000.0, f64::INFINITY).cap(&P1, FASTEST_MM_S),
        0.0
    );
}

#[test]
fn the_own_flow_counts_no_more_than_its_bound() {
    let rule = RatioRampRule {
        advance_slope_s: 0.02,
        max_velocity_mm_s: 50.0,
        max_accel_mm_s2: 1.0e6,
        kernel_jerk_per_accel_hz: f64::INFINITY,
    };
    let noise = Seam {
        ratio_step: 1e-5,
        peak_ratio: 0.03116,
        ..seam(0.0, 74_000.0, f64::INFINITY)
    };
    let own_limited = noise.cap(&rule, FASTEST_MM_S);
    assert!(
        (own_limited - (50.0 / 0.03116 - 0.02 * 74_000.0)).abs() < 1.0,
        "{own_limited}"
    );
    let bounded = Seam {
        own_flow_mm_s: 50.0 * 0.03116 / 0.03125,
        ..noise
    };
    assert_eq!(bounded.cap(&rule, FASTEST_MM_S), f64::INFINITY);
}

#[test]
fn a_rest_beside_a_step_caps_its_speed_at_the_ramp_the_room_leaves() {
    let rule = RatioRampRule {
        advance_slope_s: 0.03,
        max_velocity_mm_s: 50.0,
        max_accel_mm_s2: 20000.0,
        kernel_jerk_per_accel_hz: f64::INFINITY,
    };
    let (ratio, accel, room_mm) = (0.02743, 25409.0, 10.4);
    let free = seam(ratio, accel, f64::INFINITY).cap(&rule, FASTEST_MM_S);
    let beside_rest = seam(ratio, accel, room_mm).cap(&rule, FASTEST_MM_S);
    assert!(
        beside_rest < 0.8 * free,
        "a rest {room_mm} mm away caps the step at {beside_rest} mm/s, free {free} mm/s"
    );
    let beside = seam(ratio, accel, room_mm);
    let ramp_time = rest_room_ramp_time(beside.edge_speed(beside_rest), room_mm);
    assert!(ramp_time < RATIO_RAMP_MAX_TIME_S, "{ramp_time}");
    let (velocity, accel_demand) = beside.demand(&rule, ramp_time, beside_rest);
    assert!(velocity <= 50.0 + 1e-9 && accel_demand <= 20000.0 + 1e-9);
    assert!(
        (velocity - 50.0).abs() < 1e-6 || (accel_demand - 20000.0).abs() < 1e-6,
        "the capped step asks {velocity} mm/s and {accel_demand} mm/s² of the extruder"
    );
    let past_reach =
        rest_room_reach_mm(seam(ratio, accel, f64::INFINITY).edge_speed(free)) * (1.0 + 1e-9);
    assert_eq!(
        seam(ratio, accel, past_reach).cap(&rule, FASTEST_MM_S),
        free
    );
}

#[test]
fn a_step_is_held_where_its_ramp_edges_are_crossed_fastest() {
    let braking = seam(0.0352, 493.0, f64::INFINITY);
    let cap = braking.cap(&P1, FASTEST_MM_S);
    let read_at_the_step = braking.cap_read_at_the_step(&P1, FASTEST_MM_S);
    assert!(
        cap < read_at_the_step,
        "braking at 493 mm/s² into a bowden switch, its ramp's edges cap it at {cap} mm/s, \
         the step alone at {read_at_the_step} mm/s"
    );
    let (velocity, accel_demand) = braking.demand(&P1, RATIO_RAMP_MAX_TIME_S, cap);
    assert!(
        velocity <= 45.0 + 1e-9 && (accel_demand - 2000.0).abs() < 1e-6,
        "the step capped at {cap} mm/s asks {velocity} mm/s and {accel_demand} mm/s²"
    );
    let kernel_chain = RatioRampRule {
        kernel_jerk_per_accel_hz: 200.0,
        ..P1
    };
    assert_eq!(
        braking.cap(&kernel_chain, FASTEST_MM_S),
        read_at_the_step,
        "a follower kernel spreads the ramp's edge, so the step is read where it is crossed"
    );
}

#[test]
fn the_velocity_a_ramp_adds_is_read_at_the_step_however_fast_its_edges() {
    let rule = RatioRampRule {
        advance_slope_s: 0.03,
        max_velocity_mm_s: 50.0,
        max_accel_mm_s2: 20000.0,
        kernel_jerk_per_accel_hz: f64::INFINITY,
    };
    let travel_switch = seam(0.03526, 28513.0, f64::INFINITY);
    let cap = travel_switch.cap(&rule, FASTEST_MM_S);
    let (velocity, accel_demand) = travel_switch.demand(&rule, RATIO_RAMP_MAX_TIME_S, cap);
    assert!(
        (velocity - 50.0).abs() < 1e-6 && accel_demand < 0.5 * 20000.0,
        "a travel switch capped at {cap} mm/s asks {velocity} mm/s and {accel_demand} mm/s² \
         with its edges {} mm/s fast",
        travel_switch.edge_speed(cap)
    );
    assert_eq!(
        cap,
        travel_switch.cap_read_at_the_step(&rule, FASTEST_MM_S),
        "where the velocity holds the step, faster edges leave its cap"
    );
}

/// The projection lays a step's window `V·τ/2` wide at the fastest speed
/// `V` the leaders cross its ramp at, and a rest's cone narrows it to the
/// step's share of the room: a step crossed slowly into a hard
/// acceleration has its ramp read over the window the room leaves at its
/// edges' speed, not at the step's.
#[test]
fn a_rest_narrows_the_ramp_to_the_window_its_fastest_edge_lays() {
    let rule = RatioRampRule {
        advance_slope_s: 0.02,
        max_velocity_mm_s: 50.0,
        max_accel_mm_s2: 20000.0,
        kernel_jerk_per_accel_hz: f64::INFINITY,
    };
    let travel_switch = seam(0.0337, 23590.0, 10.0);
    let cap = travel_switch.cap(&rule, FASTEST_MM_S);
    let edge_speed = travel_switch.edge_speed(cap);
    let edge_ramp_time = longest_ramp_time(edge_speed, travel_switch.rest_room_mm);
    let (velocity, accel_demand) = travel_switch.demand(&rule, edge_ramp_time, cap);
    assert!(
        velocity <= 50.0 + 1e-9 && accel_demand <= 20000.0 + 1e-9,
        "a travel switch capped at {cap} mm/s, its edges {edge_speed} mm/s fast over a \
         {edge_ramp_time} s window, asks {velocity} mm/s and {accel_demand} mm/s²"
    );
}

const P4: RatioRampRule = RatioRampRule {
    advance_slope_s: 0.02,
    max_velocity_mm_s: 50.0,
    max_accel_mm_s2: 5000.0,
    kernel_jerk_per_accel_hz: f64::INFINITY,
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
        slides: [RatioSlide::FLAT; 2],
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

#[test]
fn the_ramp_at_rest_adds_its_per_accel_share_over_the_longest_ramp() {
    for (k, ratio_step, accel) in [(0.5, 0.0337, 1213.0), (0.02, 0.0168, 25_000.0)] {
        let at_rest = RatioStep {
            ratio_step,
            peak_ratio: 0.0,
            slides: [RatioSlide::FLAT; 2],
            speed_mm_s: 0.0,
            accel_mm_s2: accel,
            jerk_mm_s3: 0.0,
        };
        let (_, ramp_accel) = unlimited(k).demand(RATIO_RAMP_MAX_TIME_S, &at_rest);
        let expected = ramp_accel_at_rest_per_accel(k, ratio_step) * accel;
        assert!(
            (ramp_accel - expected).abs() <= 1e-12 * expected,
            "{ramp_accel} against {expected}"
        );
    }
}

/// The largest `|(h²)″|`, `h·|(h²)‴|` and `gap·|(h²)‴|` against what
/// [`width_square_curvature_per_gap`] allows at `h/gap`, along
/// `h = from + (to − from)·q(x)` over `x = s/gap` in `[0, 1]`, `q` and its
/// first three derivatives given at `x`.
fn square_bends(from: f64, to: f64, gap: f64, q: impl Fn(f64) -> [f64; 4]) -> [f64; 3] {
    let samples = 20_000;
    (0..=samples).fold([0.0_f64; 3], |[slope, curvature, gap_share], i| {
        let [value, d1, d2, d3] = q(f64::from(i) / f64::from(samples));
        let rise = to - from;
        let h = from + rise * value;
        let (h1, h2, h3) = (
            rise * d1 / gap,
            rise * d2 / gap.powi(2),
            rise * d3 / gap.powi(3),
        );
        let square_curvature = (2.0 * (3.0 * h1 * h2 + h * h3)).abs();
        [
            slope.max((2.0 * (h1 * h1 + h * h2)).abs()),
            curvature.max(h * square_curvature),
            gap_share.max(gap * square_curvature / width_square_curvature_per_gap(h / gap)),
        ]
    })
}

fn smootherstep(x: f64) -> [f64; 4] {
    [
        x * x * x * (10.0 - 15.0 * x + 6.0 * x * x),
        30.0 * x * x * (1.0 - x) * (1.0 - x),
        60.0 * x * (1.0 - x) * (1.0 - 2.0 * x),
        60.0 - 360.0 * x + 360.0 * x * x,
    ]
}

fn rise_from_rest(x: f64) -> [f64; 4] {
    let left = 1.0 - x;
    [
        1.0 - left * left * left,
        3.0 * left * left,
        -6.0 * left,
        6.0,
    ]
}

#[test]
fn the_width_square_bounds_hold_every_node_pair_the_field_lays_and_are_reached() {
    let gap = 1.0;
    let narrowest = gap / RATIO_RAMP_NODE_SPACING_PER_WIDTH;
    let widest_rise = RATIO_RAMP_ENVELOPE_SLOPE * gap;
    let steps = 40;
    let mut peaks = [0.0_f64; 3];
    for i in 0..=steps {
        let narrower = narrowest * f64::from(i) / f64::from(steps);
        for j in 0..=steps {
            let wider = narrower + widest_rise * f64::from(j) / f64::from(steps);
            for (from, to) in [(narrower, wider), (wider, narrower)] {
                let bends = square_bends(from, to, gap, smootherstep);
                peaks = std::array::from_fn(|n| peaks[n].max(bends[n]));
            }
        }
    }
    let [rest_slope, rest_curvature, rest_gap_share] =
        square_bends(0.0, widest_rise, gap, rise_from_rest);
    assert!(
        rest_slope <= WIDTH_SQUARE_SLOPE_BOUND && rest_curvature <= WIDTH_SQUARE_CURVATURE_BOUND,
        "a rise off a rest bends h² by {rest_slope} and {rest_curvature}"
    );
    assert!(
        (rest_gap_share - 1.0).abs() <= 1e-12,
        "a rise off a rest bends h²'s slope by {rest_gap_share} of its gap bound at the rest"
    );
    assert!(
        peaks[0] <= WIDTH_SQUARE_SLOPE_BOUND && peaks[0] >= WIDTH_SQUARE_SLOPE_BOUND - 1e-3,
        "the smootherstep bends h² by up to {}, bound {WIDTH_SQUARE_SLOPE_BOUND}",
        peaks[0]
    );
    assert!(
        peaks[1] <= WIDTH_SQUARE_CURVATURE_BOUND * (1.0 + 1e-12)
            && peaks[1] >= WIDTH_SQUARE_CURVATURE_BOUND * (1.0 - 1e-12),
        "the smootherstep bends h²'s slope by up to {} against h, bound \
         {WIDTH_SQUARE_CURVATURE_BOUND}",
        peaks[1]
    );
    assert!(
        peaks[2] <= 1.0 + 1e-12 && peaks[2] >= 1.0 - 1e-12,
        "the smootherstep bends h²'s slope by up to {} of its gap bound",
        peaks[2]
    );
}

#[test]
fn a_run_carries_its_slope_where_it_outlasts_the_ramp_path_and_none_under_half_of_it() {
    let slide = RatioSlide {
        slope: -0.01,
        length_mm: 2.0,
    };
    assert_eq!(slide.moving_slope(2.0), 0.01);
    assert_eq!(slide.moving_slope(1.0), 0.01);
    assert_eq!(slide.moving_slope(4.0), 0.0);
    assert!((slide.moving_slope(3.0) - 0.01 / 3.0).abs() < 1e-15);
}

#[test]
fn a_moving_slope_lengthens_the_ramp_it_asks_for() {
    let rule = RatioRampRule {
        max_accel_mm_s2: 20_000.0,
        ..unlimited(0.03)
    };
    let flat = cruise(0.01, 200.0);
    let sloped = RatioStep {
        slides: [
            RatioSlide::FLAT,
            RatioSlide {
                slope: 0.005,
                length_mm: f64::INFINITY,
            },
        ],
        ..flat
    };
    assert!(rule.ramp_time(&sloped) > rule.ramp_time(&flat));
}

#[test]
fn a_run_bends_its_window_no_sharper_than_half_its_length_allows() {
    let (k, speed, tau, slope) = (0.5, 100.0, 0.002, 2.0e-4);
    let gap = speed * RATIO_RAMP_NODE_SPACING_S;
    let accel_of = |length_mm: f64| {
        let step = RatioStep {
            slides: [RatioSlide::FLAT, RatioSlide { slope, length_mm }],
            ..cruise(0.0, speed)
        };
        unlimited(k).demand(tau, &step).1
    };
    let bend = |per_gap: f64| {
        KERNEL_HALF_VARIANCE
            * slope
            * speed
            * speed
            * (WIDTH_SQUARE_SLOPE_BOUND + k * speed * per_gap / gap)
    };
    let short = speed * tau;
    let within = |asked: f64, expected: f64| (asked - expected).abs() <= 1e-12 * expected;
    assert!(
        within(
            accel_of(short),
            bend(width_square_curvature_per_gap(0.5 * short / gap))
        ),
        "a run one ramp path long asks {}",
        accel_of(short)
    );
    assert!(
        within(
            accel_of(f64::INFINITY),
            bend(WIDTH_SQUARE_CURVATURE_PER_GAP_BOUND)
        ),
        "an endless run asks {}",
        accel_of(f64::INFINITY)
    );
    assert!(accel_of(short) < 0.5 * accel_of(f64::INFINITY));
}
