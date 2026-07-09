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

#[test]
fn classify_window_zero_when_kappa_is_flat_zero() {
    let kappa = vec![0.0, 1e-6, -1e-6, 2e-6];
    let dkappa_ds = vec![0.0, 0.0, 0.0, 0.0];
    assert_eq!(classify_window(&kappa, &dkappa_ds), CurvatureClass::Zero);
}

#[test]
fn classify_window_constant_when_kappa_nonzero_but_steady() {
    let kappa = vec![0.02, 0.021, 0.019, 0.02];
    let dkappa_ds = vec![0.0, 1e-6, -1e-6, 0.0];
    assert_eq!(
        classify_window(&kappa, &dkappa_ds),
        CurvatureClass::Constant
    );
}

#[test]
fn classify_window_linear_when_rate_is_steady_nonzero() {
    let kappa = vec![0.0, 0.01, 0.02, 0.03];
    let dkappa_ds = vec![0.01, 0.0102, 0.0099, 0.0101];
    assert_eq!(classify_window(&kappa, &dkappa_ds), CurvatureClass::Linear);
}

#[test]
fn classify_window_other_when_rate_is_unsteady() {
    let kappa = vec![0.0, 0.05, -0.02, 0.08];
    let dkappa_ds = vec![0.05, -0.07, 0.1, -0.09];
    assert_eq!(classify_window(&kappa, &dkappa_ds), CurvatureClass::Other);
}

#[test]
fn classify_window_ignores_a_handful_of_outliers() {
    // 24 steady samples plus 2 seam-artifact outliers -- the percentile
    // spread must not be blown out by them the way a raw max-min would be.
    let mut dkappa_ds = vec![0.01; 24];
    dkappa_ds[5] = 5.0;
    dkappa_ds[19] = -5.0;
    let kappa: Vec<f64> = (0..24).map(|i| 0.01 * i as f64).collect();
    assert_eq!(classify_window(&kappa, &dkappa_ds), CurvatureClass::Linear);
}

#[test]
fn classify_window_handles_small_windows_with_outliers() {
    // 4-sample window (small, under n=10): 3 steady samples + 1 extreme outlier.
    // Sorted dkappa_ds = [0.01, 0.01, 0.01, 5.0].
    // With the fix: trim = (4/10).max(1).min(1) = 1, so lo=sorted[1]=0.01,
    // hi=sorted[2]=0.01, spread=0.0 -> Constant or Linear, depending on median.
    // Pre-fix: trim = 0 (raw min-max), so lo=sorted[0]=0.01, hi=sorted[3]=5.0,
    // spread=4.99 -> Other. This test proves the fix actually closes the gap.
    let dkappa_ds = vec![0.01, 0.01, 5.0, 0.01];
    let kappa = vec![0.01, 0.02, 0.03, 0.04];
    assert_eq!(classify_window(&kappa, &dkappa_ds), CurvatureClass::Linear);
}

#[test]
fn smooth_classes_kills_an_isolated_single_window_flicker() {
    use CurvatureClass::*;
    let raw = vec![Constant, Constant, Other, Constant, Constant];
    let smoothed = smooth_classes(&raw);
    assert_eq!(
        smoothed,
        vec![Constant, Constant, Constant, Constant, Constant]
    );
}

#[test]
fn smooth_classes_keeps_a_sustained_change() {
    use CurvatureClass::*;
    let raw = vec![Constant, Constant, Other, Other, Other, Constant, Constant];
    let smoothed = smooth_classes(&raw);
    assert_eq!(smoothed, raw);
}

#[test]
fn percentile_spread_reads_zero_at_n_equals_three() {
    // At n=3, trim = (3/10).max(1).min((3-1)/2) = 1, so lo and hi both index
    // the middle element (the median), forcing spread to always be 0.0 regardless
    // of how extreme the two outer samples are. This is the documented, accepted
    // tradeoff: a 3-sample window errs toward "not enough data to call it anomalous"
    // rather than risking a false positive on extreme outer values.
    let sorted = vec![-100.0, 0.01, 100.0];
    assert_eq!(percentile_spread(&sorted), 0.0);
}

#[test]
fn curvature_series_flags_a_cusp_at_zero_speed() {
    // A single linear piece with a zero coefficient (pos = 0 for all t) has
    // vx == vy == 0 everywhere -- every sample is a cusp.
    let xp = vec![vec![0.0, 1.0, 0.0, 0.0]];
    let yp = vec![vec![0.0, 1.0, 0.0, 0.0]];
    let t = vec![0.0, 0.5, 1.0];
    let (_, classes) = curvature_series(&t, &xp, &yp);
    assert!(classes.iter().all(|&c| c == CurvatureClass::Cusp));
}

#[test]
fn curvature_series_flags_gap_samples_and_classifies_the_rest() {
    // x moves at constant velocity 1 mm/s in piece 0 ([0,1)) then, after a
    // real domain gap, again at constant velocity in piece 1 ([1.5,3)); y is
    // stationary throughout, so wherever there IS coverage the path is
    // straight (kappa == 0, class Zero).
    let xp = vec![vec![0.0, 1.0, 0.0, 1.0], vec![1.5, 3.0, 1.5, 1.0]];
    let yp = vec![vec![0.0, 1.0, 0.0, 0.0], vec![1.5, 3.0, 0.0, 0.0]];
    let t = vec![0.2, 0.8, 1.2, 2.0, 2.5];
    let (_, classes) = curvature_series(&t, &xp, &yp);
    assert_eq!(classes[0], CurvatureClass::Zero); // t=0.2, in piece 0
    assert_eq!(classes[1], CurvatureClass::Zero); // t=0.8, in piece 0
    assert_eq!(classes[2], CurvatureClass::Gap); // t=1.2, inside [1.0,1.5) gap
    assert_eq!(classes[3], CurvatureClass::Zero); // t=2.0, in piece 1
    assert_eq!(classes[4], CurvatureClass::Zero); // t=2.5, in piece 1
}

#[test]
fn curvature_series_handles_multiple_windows_all_zero() {
    // 50 samples span 3 windows (24 + 24 + 2, since CLASSIFY_WINDOW_SAMPLES
    // = 24): two full windows plus a trailing partial one. A single
    // constant-velocity piece covering the whole domain is dead straight
    // throughout, so every sample -- in every window -- must classify Zero.
    // This is an end-to-end check that the per-window classify -> smooth ->
    // per-sample expand restructuring doesn't corrupt indices or drop
    // samples across a multi-window trajectory.
    let dt = 0.02;
    let t: Vec<f64> = (0..50).map(|i| i as f64 * dt).collect();
    let xp = vec![vec![0.0, 1.0, 0.0, 2.5]];
    let yp = vec![vec![0.0, 1.0, 0.0, 0.0]];
    let (_, classes) = curvature_series(&t, &xp, &yp);
    assert_eq!(classes.len(), 50);
    assert!(classes.iter().all(|&c| c == CurvatureClass::Zero));
}

#[test]
fn curvature_series_despikes_an_isolated_curved_window_across_multiple_windows() {
    // Same 24+24+2 window layout as above, but the middle window (samples
    // 24..48) carries real curvature: constant y-acceleration bends the
    // heading for exactly one window's worth of samples, flanked by a
    // straight run before and after. The middle window's kappa is well
    // above KAPPA_ZERO_EPS throughout, so in isolation it would NOT
    // classify as Zero -- but flanked by two Zero windows that agree with
    // each other, smooth_classes must fold it back to Zero. Under the
    // pre-fix bug (smoothing a per-sample-expanded array, where every
    // sample in a 24-sample run is identical to its neighbor by
    // construction) a run this wide could never be despiked at all.
    let dt = 0.02;
    let t: Vec<f64> = (0..50).map(|i| i as f64 * dt).collect();
    let vx = 10.0;
    let ay = 5.0;
    let xp = vec![
        vec![0.0, 0.48, 0.0, vx],
        vec![0.48, 0.96, 0.48 * vx, vx],
        vec![0.96, 1.0, 0.96 * vx, vx],
    ];
    let yp = vec![
        vec![0.0, 0.48, 0.0, 0.0],
        vec![0.48, 0.96, 0.0, 0.0, 0.5 * ay],
        vec![0.96, 1.0, 0.5 * ay * 0.48 * 0.48, ay * 0.48],
    ];
    let (kappa, classes) = curvature_series(&t, &xp, &yp);

    // Sanity: the middle window really does carry measurable curvature
    // that smoothing would have to actively fold away.
    assert!(kappa[24..48].iter().all(|k| k.abs() > KAPPA_ZERO_EPS));

    assert_eq!(classes.len(), 50);
    assert!(classes.iter().all(|&c| c == CurvatureClass::Zero));
}

#[test]
fn curvature_series_flags_cusp_at_a_speed_too_small_for_kappa_to_be_meaningful() {
    // A speed of 1e-6 clears FRENET_SPEED_FLOOR (1e-9) by three orders of
    // magnitude, but kappa's speed^3 denominator still makes it blow up to
    // an astronomically large value here (~2e12, the same order of
    // magnitude actually observed near a real cusp) -- this must still read
    // as Cusp, not as a giant-but-"real" curvature number.
    let xp = vec![vec![0.0, 1.0, 0.0, 1e-6, 1000.0, 0.0]];
    let yp = vec![vec![0.0, 1.0, 0.0, 0.0, 1.0, 0.0]];
    let t = vec![0.0];
    let (_, classes) = curvature_series(&t, &xp, &yp);
    assert_eq!(classes[0], CurvatureClass::Cusp);
}

#[test]
fn toolhead_series_samples_on_the_shared_grid() {
    // x = 2*tau (linear), y = 1 + 3*tau^2 (quadratic), both on [0, 1].
    let xp = vec![vec![0.0, 1.0, 0.0, 2.0]];
    let yp = vec![vec![0.0, 1.0, 1.0, 0.0, 3.0]];
    let t = vec![0.0, 0.5, 1.0];
    let s = toolhead_series(&t, &xp, &yp);
    assert_eq!(s.x, vec![0.0, 1.0, 2.0]);
    assert_eq!(s.vx, vec![2.0, 2.0, 2.0]);
    assert_eq!(s.ax, vec![0.0, 0.0, 0.0]);
    assert_eq!(s.y, vec![1.0, 1.75, 4.0]);
    assert_eq!(s.vy, vec![0.0, 3.0, 6.0]);
    assert_eq!(s.ay, vec![6.0, 6.0, 6.0]);
}

#[test]
fn toolhead_series_is_empty_for_empty_lanes() {
    let s = toolhead_series(&[], &[], &[]);
    assert!(s.x.is_empty() && s.y.is_empty());
}
