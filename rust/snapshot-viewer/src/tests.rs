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

#[test]
fn kappa_is_zero_on_a_straight_line() {
    // Constant velocity, zero accel/jerk -- no curvature regardless of speed.
    let (kappa, dkappa_dt) = kappa_and_dkappa_dt(5.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    assert_eq!(kappa, 0.0);
    assert_eq!(dkappa_dt, 0.0);
}

#[test]
fn kappa_constant_on_circle_with_nonconstant_speed() {
    // Circle of radius R, parameterized by theta(t) = t^2 -- so theta' = 2t
    // is NOT constant, i.e. the tangential speed along the circle varies
    // with t. Curvature must still read as exactly 1/R at every t: kappa is
    // a property of the path's shape, not of how fast it's traversed. If
    // the formula secretly depended on ds/dt this test would fail at one of
    // the two very different speeds checked below.
    let r = 3.0_f64;
    let kappa_at = |t: f64| -> f64 {
        let theta = t * t;
        let (s, c) = libm::sincos(theta);
        let vx = -2.0 * r * t * s;
        let vy = 2.0 * r * t * c;
        let ax = -2.0 * r * s - 4.0 * r * t * t * c;
        let ay = 2.0 * r * c - 4.0 * r * t * t * s;
        let jx = -12.0 * r * t * c + 8.0 * r * t.powi(3) * s;
        let jy = -12.0 * r * t * s - 8.0 * r * t.powi(3) * c;
        kappa_and_dkappa_dt(vx, vy, ax, ay, jx, jy).0
    };
    let k_slow = kappa_at(0.3); // speed = 2*r*0.3 = 1.8*r
    let k_fast = kappa_at(0.9); // speed = 2*r*0.9 = 5.4*r -- 3x faster
    assert!((k_slow - 1.0 / r).abs() < 1e-9);
    assert!((k_fast - 1.0 / r).abs() < 1e-9);
}

#[test]
fn dkappa_ds_constant_on_clothoid() {
    // Euler spiral parameterized directly by arc length (dx/ds = cos(phi),
    // dy/ds = sin(phi), phi = sigma*s^2/2) -- so speed == 1 identically and
    // t IS s here, letting dkappa_dt stand in for dkappa_ds directly.
    // kappa(s) = sigma*s by construction; dkappa/ds must read back as the
    // constant sigma at every s, independent of s.
    let sigma = 0.25_f64;
    let dkappa_ds_at = |s: f64| -> f64 {
        let phi = 0.5 * sigma * s * s;
        let (sp, cp) = libm::sincos(phi);
        let vx = cp;
        let vy = sp;
        let ax = -sp * sigma * s;
        let ay = cp * sigma * s;
        let jx = -cp * sigma * sigma * s * s - sp * sigma;
        let jy = -sp * sigma * sigma * s * s + cp * sigma;
        kappa_and_dkappa_dt(vx, vy, ax, ay, jx, jy).1
    };
    assert!((dkappa_ds_at(0.5) - sigma).abs() < 1e-9);
    assert!((dkappa_ds_at(2.0) - sigma).abs() < 1e-9);
    assert!((dkappa_ds_at(4.0) - sigma).abs() < 1e-9);
}

#[test]
fn domain_anomalies_empty_for_contiguous_pieces() {
    let pieces = vec![vec![0.0, 1.0, 0.0], vec![1.0, 2.0, 0.0]];
    assert!(domain_anomalies(&pieces).is_empty());
}

#[test]
fn domain_anomalies_flags_a_gap() {
    // piece[0] ends at 1.0, piece[1] doesn't start until 1.5: a real hole.
    let pieces = vec![vec![0.0, 1.0, 0.0], vec![1.5, 2.0, 0.0]];
    let gaps = domain_anomalies(&pieces);
    assert_eq!(gaps, vec![(1.0, 1.5)]);
    assert!(in_any_span(&gaps, 1.2, 1e-9));
    assert!(!in_any_span(&gaps, 0.5, 1e-9));
}

#[test]
fn domain_anomalies_flags_an_overlap() {
    // piece[1] starts at 0.8, before piece[0] ends at 1.0: double-covered.
    let pieces = vec![vec![0.0, 1.0, 0.0], vec![0.8, 2.0, 0.0]];
    let overlaps = domain_anomalies(&pieces);
    assert_eq!(overlaps, vec![(0.8, 1.0)]);
    assert!(in_any_span(&overlaps, 0.9, 1e-9));
}

#[test]
fn domain_anomalies_tolerates_float_noise_at_a_seam() {
    let pieces = vec![vec![0.0, 1.0, 0.0], vec![1.0 + 1e-13, 2.0, 0.0]];
    assert!(domain_anomalies(&pieces).is_empty());
}
