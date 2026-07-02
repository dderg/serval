use super::*;

fn kin(kappa0: f64, sigma: f64, length: f64, accel: f64, jerk: f64, flat: f64) -> Kinematics {
    Kinematics {
        length,
        accel,
        jerk,
        kappa0,
        sigma,
        flat_ceiling: flat,
    }
}

fn single_move_anchors(entry: f64, exit: f64) -> JerkAnchors {
    JerkAnchors {
        fwd_a: 0.0,
        fwd_v: entry,
        fwd_s: 0.0,
        bwd_v: exit,
        bwd_s: 0.0,
    }
}

fn numeric_const_kappa_w(w_in: f64, length: f64, accel: f64, kappa: f64) -> f64 {
    let f = |w: f64| 2.0 * (accel * accel - (kappa * kappa) * (w * w)).max(0.0).sqrt();
    let steps = 200_000u32;
    let h = length / f64::from(steps);
    let mut w = w_in;
    for _ in 0..steps {
        let k1 = f(w);
        let k2 = f(w + 0.5 * h * k1);
        let k3 = f(w + 0.5 * h * k2);
        let k4 = f(w + h * k3);
        w += (h / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
    }
    w
}

#[test]
fn const_kappa_closed_form_matches_numeric_ode() {
    let (w_in, length, accel, kappa) = (100.0, 2.0, 2000.0, 0.02);
    let closed = const_kappa_reach_w(w_in, length, accel, kappa);
    let numeric = numeric_const_kappa_w(w_in, length, accel, kappa);
    let v = closed.sqrt();
    assert!((closed.sqrt() - numeric.sqrt()).abs() < 1e-6 * v);
}

#[test]
fn line_reach_is_constant_accel() {
    let k = kin(0.0, 0.0, 3.0, 1000.0, f64::INFINITY, 1e9);
    let v = disk_reach_v(&k, 40.0, k.length, 1e-7).unwrap();
    let expected = (40.0_f64 * 40.0 + 2.0 * 1000.0 * 3.0).sqrt();
    assert!((v - expected).abs() < 1e-9 * expected);
}

#[test]
fn infinite_jerk_reach_is_pure_disk() {
    let k = kin(0.0, 0.05, 4.0, 1000.0, f64::INFINITY, 300.0);
    let pure = disk_reach_v(&k, 50.0, k.length, 1e-7).unwrap();
    let jerk = scurve::max_reachable_velocity(50.0, k.length, k.accel, k.jerk);
    let reached = pure.min(jerk);
    assert!((reached - pure).abs() < 1e-9 * pure.max(1.0));
}

#[test]
fn zero_curvature_reach_is_scurve() {
    let k = kin(0.0, 0.0, 4.0, 1000.0, 60_000.0, 300.0);
    let disk = disk_reach_v(&k, 20.0, k.length, 1e-7).unwrap();
    let jerk = scurve::max_reachable_velocity(20.0, k.length, k.accel, k.jerk);
    let reached = disk.min(jerk);
    assert!((reached - jerk).abs() < 1e-9 * jerk);
}

#[test]
fn clothoid_samples_respect_the_acceleration_disk() {
    let accel = 1000.0;
    let k = kin(0.0, 0.05, 4.0, accel, f64::INFINITY, 300.0);
    let samples = sample_profile(&k, 100.0, 70.0, &single_move_anchors(100.0, 70.0), 1e-8).unwrap();
    for &(s, v, _a) in &samples {
        let kappa = (k.kappa0 + k.sigma * s).abs();
        let a_c = v * v * kappa;
        assert!(a_c <= accel + 1e-3, "a_c={a_c} at s={s}");
    }
}

#[test]
fn clothoid_total_acceleration_is_within_the_disk() {
    // The emitted tangential accel `a` plus the centripetal `kappa v^2` must stay
    // inside the acceleration disk — the jerk-ride bounds `a` by the disk budget,
    // so feasibility holds without any post-hoc clamp masking it.
    let accel = 1000.0;
    let k = kin(0.0, 0.05, 4.0, accel, 80_000.0, 300.0);
    let samples = sample_profile(&k, 100.0, 70.0, &single_move_anchors(100.0, 70.0), 1e-8).unwrap();
    for &(s, v, a) in &samples {
        let kappa = (k.kappa0 + k.sigma * s).abs();
        let a_c = v * v * kappa;
        let total = (a * a + a_c * a_c).sqrt();
        assert!(
            total <= accel + 1.0,
            "total accel {total} exceeds a_max={accel} at s={s} (a_t={a}, a_c={a_c})"
        );
    }
}

#[test]
fn sample_profile_is_deterministic() {
    let k = kin(0.0, 0.05, 4.0, 1000.0, 80_000.0, 300.0);
    let a = sample_profile(&k, 100.0, 70.0, &single_move_anchors(100.0, 70.0), 1e-8);
    let b = sample_profile(&k, 100.0, 70.0, &single_move_anchors(100.0, 70.0), 1e-8);
    assert_eq!(a, b);
}

#[test]
fn sample_profile_endpoints_are_entry_and_exit() {
    let k = kin(0.0, 0.05, 4.0, 1000.0, 80_000.0, 300.0);
    let s = sample_profile(&k, 100.0, 70.0, &single_move_anchors(100.0, 70.0), 1e-8).unwrap();
    assert_eq!(s.first().unwrap().0, 0.0);
    assert_eq!(s.last().unwrap().0, k.length);
    assert_eq!(s.first().unwrap().1, 100.0);
    assert_eq!(s.last().unwrap().1, 70.0);
}

#[test]
fn limit_speed_is_infinite_for_a_line() {
    assert_eq!(limit_speed(0.0, 1000.0), f64::INFINITY);
    assert!((limit_speed(0.02, 2000.0) - (2000.0_f64 / 0.02).sqrt()).abs() < 1e-9);
}
