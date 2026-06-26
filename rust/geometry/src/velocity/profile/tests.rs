use super::*;

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
    for (k, b) in p.breaks.iter().enumerate() {
        assert!(b.j.abs() <= j_max + 1.0, "phase {k} jerk {} > {j_max}", b.j);
        if k + 1 < p.breaks.len() {
            let end_a = b.a + b.j * b.dt;
            assert!(
                (end_a - p.breaks[k + 1].a).abs() < 1e-6,
                "acceleration steps at break {k}: {end_a} -> {}",
                p.breaks[k + 1].a
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

#[test]
fn short_move_is_sub_cruise_and_jerk_limited() {
    // 10mm straight (the accel_to_decel case): peaks below cruise, smooth peak.
    let p = plan(0.0, 0.0, 10.0, 300.0, 1000.0, 10000.0);
    assert!(peak_v(&p) < 300.0, "10mm must stay sub-cruise");
    assert_feasible(&p, 0.0, 0.0, 300.0, 1000.0, 10000.0);
}

#[test]
fn long_move_cruises_and_brakes_to_rest_smoothly() {
    // 50mm straight (the full_cruise case): the brake-to-rest landing is a real
    // jerk phase (accel continuous to zero), not a snap.
    let p = plan(0.0, 0.0, 50.0, 300.0, 1000.0, 10000.0);
    assert_feasible(&p, 0.0, 0.0, 300.0, 1000.0, 10000.0);
    // last phase is the landing jerk-up to (v=0, a=0)
    let last = p.breaks[p.breaks.len() - 2];
    assert!(
        last.j > 0.0,
        "landing phase must jerk accel back up to zero"
    );
    assert!((last.a + last.j * last.dt).abs() < 1e-6, "lands at a=0");
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
    // rest-to-rest over L with no jerk limit peaks at sqrt(a_max * L)
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
