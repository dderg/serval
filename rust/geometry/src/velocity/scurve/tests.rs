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
fn reach_inverts_distance_triangular() {
    let (v0, a, j) = (0.0, 1000.0, 100_000.0);
    let length = 0.05;
    let v1 = max_reachable_velocity(v0, length, a, j);
    assert!((v1 - v0) <= a * a / j);
    assert!((simulate_distance(v0, v1, a, j) - length).abs() < 1e-3 * length);
}

#[test]
fn reach_inverts_distance_trapezoidal() {
    let (v0, a, j) = (30.0, 2000.0, 80_000.0);
    let length = 12.0;
    let v1 = max_reachable_velocity(v0, length, a, j);
    assert!((v1 - v0) > a * a / j);
    assert!((simulate_distance(v0, v1, a, j) - length).abs() < 1e-3 * length);
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
fn reach_from_rest_is_finite_and_positive() {
    let v1 = max_reachable_velocity(0.0, 0.02, 1000.0, 50_000.0);
    assert!(v1 > 0.0 && v1.is_finite());
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
