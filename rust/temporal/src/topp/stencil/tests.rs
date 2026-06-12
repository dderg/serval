use super::*;

#[test]
fn b_dd_weights_exact_on_quadratic_nonuniform() {
    // b(s) = 3s² − 2s + 1 → b″ = 6 everywhere, any spacing.
    let b = |s: f64| 3.0 * s * s - 2.0 * s + 1.0;
    let (hl, hr) = (0.3, 0.7);
    let s_i = 1.0;
    let w = b_dd_weights(hl, hr);
    let approx = w[0] * b(s_i - hl) + w[1] * b(s_i) + w[2] * b(s_i + hr);
    assert!((approx - 6.0).abs() < 1e-10, "got {approx}");
}

#[test]
fn b_d_weights_exact_on_quadratic_nonuniform() {
    let b = |s: f64| 3.0 * s * s - 2.0 * s + 1.0; // b′(1) = 4
    let (hl, hr) = (0.3, 0.7);
    let w = b_d_weights(hl, hr);
    let approx = w[0] * b(1.0 - hl) + w[1] * b(1.0) + w[2] * b(1.0 + hr);
    assert!((approx - 4.0).abs() < 1e-10, "got {approx}");
}

#[test]
fn weights_reduce_to_uniform() {
    let h = 0.5;
    let wd = b_d_weights(h, h);
    assert!((wd[0] - (-1.0 / (2.0 * h))).abs() < 1e-12);
    assert!(wd[1].abs() < 1e-12);
    assert!((wd[2] - 1.0 / (2.0 * h)).abs() < 1e-12);
    let wdd = b_dd_weights(h, h);
    assert!((wdd[0] - 1.0 / (h * h)).abs() < 1e-12);
    assert!((wdd[1] - (-2.0 / (h * h))).abs() < 1e-12);
    assert!((wdd[2] - 1.0 / (h * h)).abs() < 1e-12);
}

#[test]
fn fornberg_matches_b_dd_weights() {
    let (hl, hr) = (0.7, 1.3);
    let xs = [-hl, 0.0, hr];
    let w = fornberg_weights(0.0, &xs, 2);
    let closed = b_dd_weights(hl, hr);
    for k in 0..3 {
        assert!((w[2][k] - closed[k]).abs() < 1e-12, "k={k}");
    }
    let closed_d = b_d_weights(hl, hr);
    for k in 0..3 {
        assert!((w[1][k] - closed_d[k]).abs() < 1e-12, "k={k}");
    }
}

#[test]
fn b_ddd_weights_exact_on_cubics() {
    let s = [0.0, 0.7, 1.5, 2.6];
    let b: Vec<f64> = s.iter().map(|&x: &f64| x.powi(3)).collect();
    for i in 0..4 {
        let (idx, w) = b_ddd_weights_at(i, &s);
        let ddd: f64 = (0..4).map(|k| w[k] * b[idx[k]]).sum();
        assert!((ddd - 6.0).abs() < 1e-9, "i={i} got {ddd}");
    }
}

#[test]
fn s_snap_on_quartic_b_profile() {
    // b(s) = (1+s)^4 ⇒ ṡ = (1+s)², s̈ = 2(1+s)³, s⃛ = 6(1+s)⁴, s⁗ = 24(1+s)⁵
    let n = 401;
    let h = 0.5 / (n - 1) as f64;
    let s: Vec<f64> = (0..n).map(|i| i as f64 * h).collect();
    let b: Vec<f64> = s.iter().map(|&x| (1.0 + x).powi(4)).collect();
    let h_intervals = vec![h; n - 1];
    let i = n / 2;
    let x = s[i];
    let a_i = 2.0 * (1.0 + x).powi(3);
    let snap = s_ddddot_at_weights(&b, a_i, i, &s, &h_intervals);
    let analytic = 24.0 * (1.0 + x).powi(5);
    assert!(
        (snap - analytic).abs() / analytic < 1e-2,
        "got {snap}, want {analytic}"
    );
}
