use super::*;

const TIGHT: f64 = 1e-12;

fn samples(p: &Profile, n: usize) -> Vec<(f64, f64, f64)> {
    (0..n)
        .map(|i| {
            let s = p.length * (i as f64) / ((n - 1) as f64);
            let (v, a) = p.at(s);
            (s, v, a)
        })
        .collect()
}

fn peak_v(p: &Profile) -> f64 {
    samples(p, 4001)
        .iter()
        .fold(0.0_f64, |m, &(_, v, _)| m.max(v))
}

/// Jerk-limiting is checked at the source — every phase's jerk is within the
/// limit and the acceleration is continuous across phase boundaries — not by
/// finite-differencing samples (which a trapezoidal time estimate inflates).
fn assert_jerk_limited(p: &Profile, j_max: f64) {
    if !j_max.is_finite() {
        return;
    }
    for (k, b) in p.phases.iter().enumerate() {
        assert!(b.j.abs() <= j_max + 1.0, "phase {k} jerk {} > {j_max}", b.j);
        if k + 1 < p.phases.len() {
            let end_a = b.end_state().2;
            assert!(
                (end_a - p.phases[k + 1].a0).abs() < 1e-6,
                "acceleration steps at break {k}: {end_a} -> {}",
                p.phases[k + 1].a0
            );
        }
    }
}

fn assert_feasible(p: &Profile, v0: f64, v1: f64, v_max: f64, a_max: f64, j_max: f64) {
    let d = samples(p, 4001);
    assert!(d.iter().all(|&(_, v, _)| v <= v_max + 1e-6), "v > v_max");
    assert!(d.iter().all(|&(_, v, _)| v >= -1e-9), "v < 0");
    assert!(
        d.iter().all(|&(_, _, a)| a.abs() <= a_max + 1e-6),
        "a > a_max"
    );
    assert!((p.at(0.0).0 - v0).abs() < 1e-6, "entry speed");
    assert!((p.at(p.length).0 - v1).abs() < 1e-6, "exit speed");
    // Anchors sit at zero acceleration — except under infinite jerk, where the
    // acceleration steps instantaneously (the downstream rest-anchor pin absorbs
    // it). The exit is always pinned exactly.
    assert!(p.at(p.length).1.abs() < 1e-6, "exit accel nonzero");
    if j_max.is_finite() {
        assert!(p.at(0.0).1.abs() < 1e-6, "entry accel nonzero");
    }
    assert_jerk_limited(p, j_max);
}

fn assert_chain_continuous(chain: &[StraightPhase], j_max: f64, what: &str) {
    for (k, w) in chain.windows(2).enumerate() {
        let (s, v, a) = w[0].end_state();
        let next = w[1];
        assert!(
            (s - next.s0).abs() <= TIGHT * (1.0 + s.abs()),
            "{what}: s breaks at joint {k}: {s} -> {}",
            next.s0
        );
        assert!(
            (v - next.v0).abs() <= TIGHT * (1.0 + v.abs()),
            "{what}: v breaks at joint {k}: {v} -> {}",
            next.v0
        );
        if j_max.is_finite() {
            assert!(
                (a - next.a0).abs() <= TIGHT * (1.0 + a.abs()),
                "{what}: a breaks at joint {k}: {a} -> {}",
                next.a0
            );
        }
        assert!(
            (w[0].t0 + w[0].dt - next.t0).abs() <= TIGHT * (1.0 + next.t0),
            "{what}: t0 breaks at joint {k}"
        );
        assert!(next.dt > 0.0, "{what}: empty phase at {}", k + 1);
    }
    if let Some(first) = chain.first() {
        assert_eq!(first.t0, 0.0, "{what}: chain must start at t0 = 0");
        assert_eq!(first.s0, 0.0, "{what}: chain must start at s0 = 0");
    }
}

fn traversal_time(chain: &[StraightPhase]) -> f64 {
    chain.iter().map(|p| p.dt).sum()
}

#[test]
fn short_move_is_sub_cruise_and_jerk_limited() {
    let p = plan(0.0, 0.0, 10.0, 300.0, 1000.0, 10000.0);
    assert!(peak_v(&p) < 300.0, "10mm must stay sub-cruise");
    assert_feasible(&p, 0.0, 0.0, 300.0, 1000.0, 10000.0);
}

#[test]
fn long_move_cruises_and_brakes_to_rest_smoothly() {
    let p = plan(0.0, 0.0, 50.0, 300.0, 1000.0, 10000.0);
    assert_feasible(&p, 0.0, 0.0, 300.0, 1000.0, 10000.0);
    let last = p.phases[p.phases.len() - 1];
    assert!(
        last.j > 0.0,
        "landing phase must jerk accel back up to zero"
    );
    assert!(last.end_state().2.abs() < 1e-6, "lands at a=0");
}

#[test]
fn reaches_the_feed_ceiling_with_a_cruise_plateau() {
    let p = plan(0.0, 0.0, 400.0, 300.0, 1000.0, 10000.0);
    assert!(
        (peak_v(&p) - 300.0).abs() < 1e-3,
        "must reach the 300 ceiling"
    );
    let cruising = samples(&p, 4001)
        .iter()
        .filter(|&&(_, v, _)| v >= 300.0 - 1e-6)
        .count();
    assert!(cruising > 10, "must hold a cruise plateau");
    assert_feasible(&p, 0.0, 0.0, 300.0, 1000.0, 10000.0);
}

#[test]
fn warm_start_entry_and_rest_exit() {
    let p = plan(50.0, 0.0, 30.0, 300.0, 1000.0, 10000.0);
    assert_feasible(&p, 50.0, 0.0, 300.0, 1000.0, 10000.0);
}

#[test]
fn infinite_jerk_is_accel_limited_triangle() {
    let p = plan(0.0, 0.0, 20.0, 300.0, 1000.0, f64::INFINITY);
    let apex = (1000.0_f64 * 20.0).sqrt();
    assert!(
        (peak_v(&p) - apex).abs() < 1.0,
        "apex {} vs {apex}",
        peak_v(&p)
    );
    assert_feasible(&p, 0.0, 0.0, 300.0, 1000.0, f64::INFINITY);
}

#[test]
fn distance_is_exact_and_jerk_limited() {
    let p = plan(0.0, 0.0, 37.3, 250.0, 1500.0, 8000.0);
    assert!((p.length - 37.3).abs() < 1e-9, "spans the requested length");
    assert_feasible(&p, 0.0, 0.0, 250.0, 1500.0, 8000.0);
}

#[test]
fn sampling_is_deterministic_and_density_independent() {
    let p = plan(0.0, 0.0, 12.0, 300.0, 1000.0, 10000.0);
    for &s in &[0.0, 1.0, 3.3, 6.0, 9.9, 12.0] {
        assert_eq!(p.at(s), p.at(s));
    }
}

/// The closed-form peak must satisfy the ramp-distance equation it was derived
/// from, in every regime the bisection fallback would otherwise cover.
#[test]
fn peak_velocity_solves_the_ramp_distance_equation() {
    let cases = [
        (0.0, 0.0, 10.0, 300.0, 1000.0, 10000.0),
        (0.0, 0.0, 37.3, 250.0, 1500.0, 8000.0),
        (60.0, 60.0, 8.0, 400.0, 2000.0, 60000.0),
        (0.0, 0.0, 200.0, 1e9, 1000.0, 10000.0),
        (50.0, 0.0, 30.0, 300.0, 1000.0, 10000.0),
        (10.0, 90.0, 25.0, 500.0, 3000.0, 25000.0),
        (0.0, 0.0, 20.0, 1e9, 1000.0, f64::INFINITY),
        (30.0, 70.0, 40.0, 1e9, 1200.0, f64::INFINITY),
    ];
    for (v0, v1, length, v_max, a_max, j_max) in cases {
        let vp = peak_velocity(v0, v1, length, v_max, a_max, j_max);
        let span = ramp_dist(v0, vp, a_max, j_max) + ramp_dist(vp, v1, a_max, j_max);
        assert!(
            (span - length).abs() <= 1e-9 * (1.0 + length),
            "peak {vp} spans {span}, wanted {length} for {v0},{v1},{a_max},{j_max}"
        );
        assert!(vp >= v0.max(v1) - TIGHT, "peak {vp} below the boundaries");
    }
}

fn sweep_cases() -> Vec<(f64, f64, f64, f64, f64, f64)> {
    let mut cases = Vec::new();
    for &length in &[1e-6, 0.001, 0.28, 1.0, 10.0, 37.3, 400.0] {
        for &flat in &[30.0, 300.0] {
            for &a_max in &[500.0, 1000.0, 25000.0] {
                for &j_max in &[1e4, 1e5, f64::INFINITY] {
                    for &(v0, v1) in &[(0.0, 0.0), (20.0, 0.0), (0.0, 25.0), (20.0, 20.0)] {
                        if v0 > flat || v1 > flat {
                            continue;
                        }
                        cases.push((v0, v1, length, flat, a_max, j_max));
                    }
                }
            }
        }
    }
    cases
}

#[test]
fn straight_chain_agrees_with_the_plan_oracle() {
    for (v0, v1, length, flat, a_max, j_max) in sweep_cases() {
        let label = format!("L={length} flat={flat} A={a_max} j={j_max} {v0}->{v1}");
        let chain = straight_chain(v0, v1, length, flat, a_max, j_max);
        assert_chain_continuous(&chain, j_max, &label);

        let oracle = plan(v0, v1, length, flat, a_max, j_max);
        assert_eq!(
            chain.len(),
            oracle.phases.len(),
            "{label}: phase count differs"
        );
        for (k, (c, o)) in chain.iter().zip(oracle.phases.iter()).enumerate() {
            for (name, got, want) in [
                ("t0", c.t0, o.t0),
                ("dt", c.dt, o.dt),
                ("s0", c.s0, o.s0),
                ("v0", c.v0, o.v0),
                ("a0", c.a0, o.a0),
                ("j", c.j, o.j),
            ] {
                assert!(
                    (got - want).abs() <= TIGHT * (1.0 + want.abs()),
                    "{label}: phase {k} {name}: {got} vs {want}"
                );
            }
        }

        if chain.is_empty() {
            continue;
        }
        assert!(
            (chain.last().unwrap().end_state().0 - oracle.length).abs()
                <= TIGHT * (1.0 + oracle.length),
            "{label}: chain span differs from the oracle span"
        );
        // A phase's closing instant is its successor's opening instant, and
        // under infinite jerk the acceleration genuinely steps there, so each
        // joint is probed once — as the opening of the phase that owns it.
        // Indexing the oracle by arc length costs `ulp(s) / v` of time
        // resolution, which the state tolerances have to carry near rest.
        for p in &chain {
            for frac in [0.0, 0.25, 0.5, 0.75] {
                let (s, v, a) = p.state_at(frac * p.dt);
                if s >= oracle.length - EPS {
                    continue;
                }
                let tau_slack = 8.0 * f64::EPSILON * (1.0 + s.abs()) / v.max(1e-6);
                let (ov, oa) = oracle.at(s);
                assert!(
                    (v - ov).abs() <= TIGHT * (1.0 + v.abs()) + a.abs() * tau_slack,
                    "{label}: v at s={s}: {v} vs oracle {ov}"
                );
                assert!(
                    (a - oa).abs() <= TIGHT * (1.0 + a.abs()) + p.j.abs() * tau_slack,
                    "{label}: a at s={s}: {a} vs oracle {oa}"
                );
            }
        }
    }
}

#[test]
fn chain_traversal_time_is_the_analytic_sum() {
    for (v0, v1, length, flat, a_max, j_max) in sweep_cases() {
        let label = format!("L={length} flat={flat} A={a_max} j={j_max} {v0}->{v1}");
        let chain = straight_chain(v0, v1, length, flat, a_max, j_max);
        if chain.is_empty() {
            continue;
        }
        let v_peak = peak_velocity(v0, v1, length, flat, a_max, j_max);
        let up = ramp_dist(v0, v_peak, a_max, j_max);
        let down = ramp_dist(v_peak, v1, a_max, j_max);
        let cruise = length - up - down;
        let analytic = ramp_time((v_peak - v0).abs(), a_max, j_max)
            + ramp_time((v1 - v_peak).abs(), a_max, j_max)
            + if cruise > EPS && v_peak > EPS {
                cruise / v_peak
            } else {
                0.0
            };
        let measured = traversal_time(&chain);
        assert!(
            (measured - analytic).abs() <= TIGHT * (1.0 + analytic),
            "{label}: traversal {measured} vs analytic {analytic}"
        );
        let last = chain.last().unwrap();
        assert!(
            (last.t0 + last.dt - measured).abs() <= TIGHT * (1.0 + measured),
            "{label}: t0 accumulator drifted from the dt sum"
        );
    }
}

/// Powers of two throughout so `a0 + j*dt` lands on the requested boundary
/// acceleration bit-exactly, not merely within a tolerance.
const EXACT_A_MAX: f64 = 1024.0;
const EXACT_J_MAX: f64 = 8192.0;
const EXACT_ENTRY_A: f64 = 256.0;
const EXACT_EXIT_A: f64 = 512.0;

#[test]
fn straight_chain_between_hits_both_boundary_accelerations() {
    let entry = (100.0, EXACT_ENTRY_A);
    let exit = (80.0, EXACT_EXIT_A);
    let length = 60.0;
    let chain =
        straight_chain_between(entry, exit, length, 300.0, EXACT_A_MAX, EXACT_J_MAX).unwrap();
    assert_chain_continuous(&chain, EXACT_J_MAX, "between");

    let first = chain[0];
    assert_eq!(first.v0, entry.0);
    assert_eq!(first.a0, entry.1);

    let (s_end, v_end, a_end) = chain.last().unwrap().end_state();
    assert_eq!(a_end, exit.1, "exit acceleration must be hit exactly");
    assert!(
        (v_end - exit.0).abs() <= 1e-9 * (1.0 + exit.0),
        "exit speed {v_end} vs {}",
        exit.0
    );
    assert!(
        (s_end - length).abs() <= 1e-9 * (1.0 + length),
        "span {s_end} vs {length}"
    );

    let reversed = straight_chain_between(
        (80.0, -EXACT_EXIT_A),
        (100.0, -EXACT_ENTRY_A),
        length,
        300.0,
        EXACT_A_MAX,
        EXACT_J_MAX,
    )
    .unwrap();
    assert_chain_continuous(&reversed, EXACT_J_MAX, "between-negative");
    assert_eq!(reversed[0].a0, -EXACT_EXIT_A);
    assert_eq!(reversed.last().unwrap().end_state().2, -EXACT_ENTRY_A);
}

#[test]
fn straight_chain_between_obeys_the_limits_it_was_given() {
    let (v_max, a_max, j_max) = (300.0, EXACT_A_MAX, EXACT_J_MAX);
    let chain =
        straight_chain_between((100.0, 256.0), (80.0, 512.0), 60.0, v_max, a_max, j_max).unwrap();
    for (k, p) in chain.iter().enumerate() {
        assert!(p.j.abs() <= j_max * (1.0 + 1e-12), "phase {k} jerk {}", p.j);
        for frac in [0.0, 0.25, 0.5, 0.75, 1.0] {
            let (_, v, a) = p.state_at(frac * p.dt);
            assert!(a.abs() <= a_max + 1e-9, "phase {k} accel {a}");
            assert!(v >= -1e-9 && v <= v_max + 1e-9, "phase {k} speed {v}");
        }
    }
}

#[test]
fn straight_chain_between_fails_loudly_on_infeasible_boundaries() {
    let (v_max, a_max, j_max) = (300.0, 1024.0, 8192.0);

    let short = straight_chain_between((100.0, 256.0), (80.0, 512.0), 1.0, v_max, a_max, j_max);
    match short {
        Err(VelocityError::InfeasibleBoundary(BoundaryInfeasibility::LengthTooShort {
            length,
            minimum,
        })) => {
            assert_eq!(length, 1.0);
            assert!(minimum > 1.0, "minimum {minimum} must exceed the request");
        }
        other => panic!("expected LengthTooShort, got {other:?}"),
    }

    assert_eq!(
        straight_chain_between((100.0, 2000.0), (80.0, 0.0), 60.0, v_max, a_max, j_max),
        Err(VelocityError::InfeasibleBoundary(
            BoundaryInfeasibility::AccelOverLimit {
                a: 2000.0,
                a_max: 1024.0
            }
        ))
    );

    match straight_chain_between((1.0, -1024.0), (80.0, 0.0), 60.0, v_max, a_max, j_max) {
        Err(VelocityError::InfeasibleBoundary(BoundaryInfeasibility::UnwindBelowRest { v })) => {
            assert!(v < 0.0, "reported unwind speed {v} must be negative")
        }
        other => panic!("expected UnwindBelowRest, got {other:?}"),
    }

    match straight_chain_between((299.0, 1024.0), (80.0, 0.0), 60.0, v_max, a_max, j_max) {
        Err(VelocityError::InfeasibleBoundary(BoundaryInfeasibility::UnwindOverCeiling {
            v,
            v_max: reported,
        })) => {
            assert!(
                v > reported,
                "reported {v} must exceed the ceiling {reported}"
            );
            assert_eq!(reported, v_max);
        }
        other => panic!("expected UnwindOverCeiling, got {other:?}"),
    }

    assert_eq!(
        straight_chain_between(
            (10.0, 256.0),
            (20.0, 0.0),
            60.0,
            v_max,
            a_max,
            f64::INFINITY
        ),
        Err(VelocityError::InfeasibleBoundary(
            BoundaryInfeasibility::UnboundedJerkWithAccelBoundary { a: 256.0 }
        ))
    );

    for bad in [
        straight_chain_between((f64::NAN, 0.0), (20.0, 0.0), 60.0, v_max, a_max, j_max),
        straight_chain_between((10.0, 0.0), (f64::INFINITY, 0.0), 60.0, v_max, a_max, j_max),
        straight_chain_between((-1.0, 0.0), (20.0, 0.0), 60.0, v_max, a_max, j_max),
        straight_chain_between((10.0, 0.0), (20.0, 0.0), -1.0, v_max, a_max, j_max),
        straight_chain_between((10.0, 0.0), (20.0, 0.0), 60.0, v_max, 0.0, j_max),
    ] {
        assert_eq!(
            bad,
            Err(VelocityError::InfeasibleBoundary(
                BoundaryInfeasibility::NonFinite
            ))
        );
    }
}

#[test]
fn straight_chain_between_at_rest_anchors_matches_straight_chain() {
    let mut compared = 0usize;
    for (v0, v1, length, flat, a_max, j_max) in sweep_cases() {
        if length <= 0.0 {
            continue;
        }
        let plain = straight_chain(v0, v1, length, flat, a_max, j_max);
        let between = straight_chain_between((v0, 0.0), (v1, 0.0), length, flat, a_max, j_max);
        let Ok(between) = between else {
            let reachable = ramp_dist(v0, v1, a_max, j_max);
            assert!(
                length < reachable,
                "rest-anchored L={length} {v0}->{v1} A={a_max} j={j_max} must be plannable \
                 (needs {reachable})"
            );
            continue;
        };
        assert_eq!(plain, between, "L={length} {v0}->{v1} A={a_max} j={j_max}");
        compared += 1;
    }
    assert!(
        compared > 100,
        "only {compared} rest-anchored cases compared"
    );
}

#[test]
fn zero_length_chain_is_empty() {
    let chain = straight_chain(0.0, 0.0, 0.0, 300.0, 1000.0, 10000.0);
    assert!(chain.is_empty());
    let p = plan(0.0, 0.0, 0.0, 300.0, 1000.0, 10000.0);
    assert_eq!(p.length, 0.0);
    assert_eq!(p.at(0.0), (0.0, 0.0));
    assert_eq!(p.at(1.0), (0.0, 0.0));
    assert_eq!(
        straight_chain_between((0.0, 0.0), (0.0, 0.0), 0.0, 300.0, 1000.0, 10000.0),
        Ok(Vec::new())
    );
}

#[test]
fn equal_boundary_speeds_still_span_the_length() {
    for &j_max in &[1e4, f64::INFINITY] {
        let chain = straight_chain(40.0, 40.0, 12.0, 300.0, 1000.0, j_max);
        assert_chain_continuous(&chain, j_max, "equal-speeds");
        assert_eq!(chain.first().unwrap().v0, 40.0);
        let (s, v, _) = chain.last().unwrap().end_state();
        assert!((s - 12.0).abs() <= 1e-9, "span {s}");
        assert!((v - 40.0).abs() <= 1e-9, "exit speed {v}");
    }
}

#[test]
fn rest_to_rest_starts_and_ends_at_rest() {
    let chain = straight_chain(0.0, 0.0, 25.0, 300.0, 1000.0, 10000.0);
    assert_chain_continuous(&chain, 10000.0, "rest-to-rest");
    assert_eq!(chain.first().unwrap().v0, 0.0);
    assert_eq!(chain.first().unwrap().a0, 0.0);
    let (s, v, a) = chain.last().unwrap().end_state();
    assert!((s - 25.0).abs() <= 1e-9, "span {s}");
    assert!(v.abs() <= 1e-9, "exit speed {v}");
    assert!(a.abs() <= 1e-9, "exit accel {a}");
}

#[test]
fn infinite_jerk_chain_is_two_accel_steps() {
    let chain = straight_chain(0.0, 0.0, 20.0, 300.0, 1000.0, f64::INFINITY);
    assert_eq!(chain.len(), 2, "accel-limited triangle is two phases");
    assert!(chain.iter().all(|p| p.j == 0.0), "no jerk phases");
    assert_eq!(chain[0].a0, 1000.0);
    assert_eq!(chain[1].a0, -1000.0);
    let apex = (1000.0_f64 * 20.0).sqrt();
    assert!(
        (chain[0].end_state().1 - apex).abs() <= 1e-9 * apex,
        "apex {} vs {apex}",
        chain[0].end_state().1
    );
    let (s, v, _) = chain[1].end_state();
    assert!((s - 20.0).abs() <= 1e-9, "span {s}");
    assert!(v.abs() <= 1e-9, "exit speed {v}");
}
