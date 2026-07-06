use super::*;

#[test]
fn eval_piece_linear() {
    // p = [t0, t1, c0, c1] -- pos = c0 + c1*tau, vel = c1, acc = jerk = 0.
    let p = vec![1.0, 2.0, 5.0, 3.0];
    let (pos, vel, acc, jerk) = eval_piece(&p, 1.5);
    assert_eq!(pos, 5.0 + 3.0 * 0.5);
    assert_eq!(vel, 3.0);
    assert_eq!(acc, 0.0);
    assert_eq!(jerk, 0.0);
}

#[test]
fn eval_piece_cubic() {
    // p = [t0, t1, c0, c1, c2, c3].
    let p = vec![0.0, 1.0, 1.0, 2.0, 3.0, 4.0];
    let tau = 0.4;
    let (pos, vel, acc, jerk) = eval_piece(&p, tau);
    let expected_pos = 1.0 + 2.0 * tau + 3.0 * tau.powi(2) + 4.0 * tau.powi(3);
    let expected_vel = 2.0 + 2.0 * 3.0 * tau + 3.0 * 4.0 * tau.powi(2);
    let expected_acc = 2.0 * 3.0 + 6.0 * 4.0 * tau;
    let expected_jerk = 6.0 * 4.0;
    assert!((pos - expected_pos).abs() < 1e-12);
    assert!((vel - expected_vel).abs() < 1e-12);
    assert!((acc - expected_acc).abs() < 1e-12);
    assert!((jerk - expected_jerk).abs() < 1e-12);
}

#[test]
fn eval_piece_degree_seven() {
    // p = [t0, t1, c0..c7] -- 10 floats, degree-7 monomial piece.
    let coeffs = [1.0, -2.0, 0.5, 3.0, -1.5, 2.0, 0.25, -0.75];
    let mut p = vec![0.0, 1.0];
    p.extend_from_slice(&coeffs);
    let tau = 0.6_f64;

    let mut expected_pos = 0.0;
    let mut expected_vel = 0.0;
    let mut expected_acc = 0.0;
    let mut expected_jerk = 0.0;
    for (k, &c) in coeffs.iter().enumerate() {
        expected_pos += c * tau.powi(k as i32);
        if k >= 1 {
            expected_vel += c * (k as f64) * tau.powi((k - 1) as i32);
        }
        if k >= 2 {
            expected_acc += c * (k as f64) * ((k - 1) as f64) * tau.powi((k - 2) as i32);
        }
        if k >= 3 {
            expected_jerk +=
                c * (k as f64) * ((k - 1) as f64) * ((k - 2) as f64) * tau.powi((k - 3) as i32);
        }
    }

    let (pos, vel, acc, jerk) = eval_piece(&p, tau);
    assert!((pos - expected_pos).abs() < 1e-9);
    assert!((vel - expected_vel).abs() < 1e-9);
    assert!((acc - expected_acc).abs() < 1e-9);
    assert!((jerk - expected_jerk).abs() < 1e-9);
}

#[test]
fn frenet_components_split_dot_and_cross() {
    // v = (3, 4) (speed 5), f = (1, 2): tangential = (3+8)/5, normal = |6-4|/5.
    let (tang, norm) = frenet_components(&[3.0], &[4.0], &[1.0], &[2.0]);
    assert!((tang[0] - 2.2).abs() < 1e-12);
    assert!((norm[0] - 0.4).abs() < 1e-12);
}

#[test]
fn frenet_tangential_is_signed_while_braking() {
    // f anti-parallel to v: all tangential, negative; no normal component.
    let (tang, norm) = frenet_components(&[5.0], &[0.0], &[-100.0], &[0.0]);
    assert_eq!(tang[0], -100.0);
    assert_eq!(norm[0], 0.0);
}

#[test]
fn frenet_components_read_zero_when_stopped() {
    let (tang, norm) = frenet_components(&[0.0], &[0.0], &[100.0], &[-50.0]);
    assert_eq!(tang[0], 0.0);
    assert_eq!(norm[0], 0.0);
}

#[test]
fn frenet_components_recover_pure_centripetal_turn() {
    // Circular motion: v = (0, 2), a = (-8, 0) — a ⟂ v, so the whole
    // acceleration is centripetal (|a| = v²/r) and none is tangential.
    let (tang, norm) = frenet_components(&[0.0], &[2.0], &[-8.0], &[0.0]);
    assert_eq!(tang[0], 0.0);
    assert_eq!(norm[0], 8.0);
}

#[test]
fn eval_piece_length_tolerance_matches_explicit_zero_padding() {
    // A short cubic row (6 floats) must evaluate identically to the same
    // coefficients padded out to degree 7 with trailing zeros.
    let short = vec![0.0, 1.0, 1.0, 2.0, 3.0, 4.0];
    let mut padded = short.clone();
    padded.extend_from_slice(&[0.0, 0.0, 0.0, 0.0]);

    for &tau in &[0.0, 0.25, 0.5, 0.9] {
        assert_eq!(eval_piece(&short, tau), eval_piece(&padded, tau));
    }
}
