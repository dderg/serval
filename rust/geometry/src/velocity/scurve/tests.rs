use super::*;

fn simulate_distance(v_in: f64, v_out: f64, accel: f64, jerk: f64) -> f64 {
    let delta = (v_out - v_in).abs();
    let sign = (v_out - v_in).signum();
    let (t1, tc) = if delta <= accel * accel / jerk {
        ((delta / jerk).sqrt(), 0.0)
    } else {
        (accel / jerk, delta / accel - accel / jerk)
    };
    let total = 2.0 * t1 + tc;
    let steps: u32 = 400_000;
    let dt = total / f64::from(steps);
    let mut v = v_in;
    let mut s = 0.0;
    for i in 0..steps {
        let t = (f64::from(i) + 0.5) * dt;
        let a_mag = if t < t1 {
            jerk * t
        } else if t < t1 + tc {
            jerk * t1
        } else {
            jerk * (total - t)
        };
        v += sign * a_mag * dt;
        s += v * dt;
    }
    s
}

#[test]
fn distance_matches_numeric_integration_triangular() {
    let (v0, v1, a, j) = (10.0, 16.0, 1000.0, 100_000.0);
    assert!((v1 - v0) <= a * a / j);
    let closed = velocity_change_distance(v0, v1, a, j);
    let numeric = simulate_distance(v0, v1, a, j);
    assert!((closed - numeric).abs() < 1e-3 * closed.max(1.0));
}

#[test]
fn distance_matches_numeric_integration_trapezoidal() {
    let (v0, v1, a, j) = (20.0, 120.0, 1000.0, 100_000.0);
    assert!((v1 - v0) > a * a / j);
    let closed = velocity_change_distance(v0, v1, a, j);
    let numeric = simulate_distance(v0, v1, a, j);
    assert!((closed - numeric).abs() < 1e-3 * closed.max(1.0));
}

#[test]
fn distance_is_zero_for_no_change() {
    assert_eq!(velocity_change_distance(50.0, 50.0, 1000.0, 100_000.0), 0.0);
}

#[test]
fn reach_inverts_distance_triangular() {
    let (v0, a, j) = (0.0, 1000.0, 100_000.0);
    let length = 0.05;
    let v1 = max_reachable_velocity(v0, length, a, j);
    assert!((v1 - v0) <= a * a / j);
    assert!((velocity_change_distance(v0, v1, a, j) - length).abs() < 1e-6 * length);
}

#[test]
fn reach_inverts_distance_trapezoidal() {
    let (v0, a, j) = (30.0, 2000.0, 80_000.0);
    let length = 12.0;
    let v1 = max_reachable_velocity(v0, length, a, j);
    assert!((v1 - v0) > a * a / j);
    assert!((velocity_change_distance(v0, v1, a, j) - length).abs() < 1e-6 * length);
}

#[test]
fn reach_reduces_to_constant_accel_when_jerk_infinite() {
    let (v0, a) = (40.0, 1500.0);
    for &length in &[0.01, 1.0, 50.0] {
        let v1 = max_reachable_velocity(v0, length, a, f64::INFINITY);
        let expected = (v0 * v0 + 2.0 * a * length).sqrt();
        assert!((v1 - expected).abs() < 1e-9 * expected);
    }
}

#[test]
fn reach_is_monotone_in_length_and_jerk() {
    let (v0, a) = (10.0, 1000.0);
    let longer = max_reachable_velocity(v0, 5.0, a, 100_000.0);
    let shorter = max_reachable_velocity(v0, 1.0, a, 100_000.0);
    assert!(longer > shorter);
    let stiffer = max_reachable_velocity(v0, 5.0, a, 1_000_000.0);
    assert!(stiffer >= longer - 1e-9);
}

#[test]
fn peak_reduces_to_constant_accel_apex_when_jerk_infinite() {
    let (vs, ve, a) = (10.0, 20.0, 1000.0);
    let length = 0.5;
    let ceiling = 1000.0;
    let peak = peak_velocity(vs, ve, length, a, f64::INFINITY, ceiling);
    let expected = (0.5 * (vs * vs + ve * ve) + a * length).sqrt();
    assert!((peak - expected).abs() < 1e-6 * expected);
    assert!(peak < ceiling);
}

#[test]
fn peak_returns_ceiling_when_room_is_ample() {
    let peak = peak_velocity(0.0, 0.0, 1000.0, 1000.0, 100_000.0, 50.0);
    assert!((peak - 50.0).abs() < 1e-9);
}

#[test]
fn peak_trims_below_constant_accel_apex_on_short_move() {
    let (vs, ve, a, j) = (5.0, 5.0, 1000.0, 50_000.0);
    let length = 0.3;
    let ceiling = 300.0;
    let jerk_peak = peak_velocity(vs, ve, length, a, j, ceiling);
    let accel_apex = (0.5 * (vs * vs + ve * ve) + a * length).sqrt();
    assert!(jerk_peak < accel_apex);
    assert!(jerk_peak < ceiling);
    let d = velocity_change_distance(vs, jerk_peak, a, j)
        + velocity_change_distance(jerk_peak, ve, a, j);
    assert!((d - length).abs() < 1e-6 * length);
}
