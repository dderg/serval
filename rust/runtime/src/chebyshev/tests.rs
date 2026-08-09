#![allow(clippy::unwrap_used, clippy::indexing_slicing)]

use super::*;

fn eval_chebyshev_f64(a: &[f32], u: f64) -> f64 {
    // Direct T_k recurrence in f64 — the independent reference for Clenshaw.
    let mut sum = 0.0;
    let mut t_prev = 1.0;
    let mut t_cur = u;
    for (k, &ak) in a.iter().enumerate() {
        let t_k = match k {
            0 => 1.0,
            1 => u,
            _ => {
                let t_next = 2.0 * u * t_cur - t_prev;
                t_prev = t_cur;
                t_cur = t_next;
                t_cur
            }
        };
        sum += f64::from(ak) * t_k;
    }
    sum
}

#[test]
fn clenshaw_matches_f64_reference() {
    let a: [f32; 8] = [1.5, -2.0, 0.25, 3.0, -0.5, 0.125, 0.75, -1.0];
    for i in 0..=100 {
        let u = -1.0 + 2.0 * (i as f32) / 100.0;
        let got = clenshaw(&a, u);
        let want = eval_chebyshev_f64(&a, f64::from(u));
        assert!(
            (f64::from(got) - want).abs() <= 1e-5 * want.abs().max(1.0),
            "u={u}: {got} vs {want}"
        );
    }
}

#[test]
fn clenshaw_short_series() {
    assert_eq!(clenshaw(&[], 0.5), 0.0);
    assert_eq!(clenshaw(&[3.5], -0.7), 3.5);
    let a = [1.0_f32, 2.0];
    assert!((clenshaw(&a, 0.25) - 1.5).abs() < 1e-7);
}

#[test]
fn derivative_series_matches_analytic() {
    // f = T_3 → f' = 6·T_2 + 3·T_0 ; f = T_2 → f' = 4·T_1 ; f = T_1 → f' = T_0.
    let mut out = [0.0_f32; 8];
    let n = derivative_series(&[0.0, 0.0, 0.0, 1.0], 1.0, &mut out);
    assert_eq!(n, 3);
    assert_eq!(&out[..3], &[3.0, 0.0, 6.0]);

    let n = derivative_series(&[0.0, 0.0, 1.0], 1.0, &mut out);
    assert_eq!(n, 2);
    assert_eq!(&out[..2], &[0.0, 4.0]);

    let n = derivative_series(&[0.0, 1.0], 2.5, &mut out);
    assert_eq!(n, 1);
    assert_eq!(out[0], 2.5);

    let n = derivative_series(&[7.0], 2.0, &mut out);
    assert_eq!(n, 1);
    assert_eq!(out[0], 0.0);
}

#[test]
fn derivative_series_matches_finite_differences() {
    let a: [f32; 8] = [0.5, 1.25, -0.75, 2.0, 0.3, -0.6, 0.1, 0.05];
    let mut d = [0.0_f32; 8];
    let n = derivative_series(&a, 1.0, &mut d);
    assert_eq!(n, 7);
    let eps = 1e-3_f32;
    for i in 1..20 {
        let u = -0.95 + 0.1 * (i as f32);
        let fd = (clenshaw(&a, u + eps) - clenshaw(&a, u - eps)) / (2.0 * eps);
        let got = clenshaw(d.get(..n).unwrap(), u);
        assert!(
            (got - fd).abs() < 2e-2 * fd.abs().max(1.0),
            "u={u}: {got} vs {fd}"
        );
    }
}

#[test]
fn derivative_series_zeroes_tail() {
    let mut out = [9.0_f32; 8];
    let n = derivative_series(&[1.0, 2.0, 3.0], 1.0, &mut out);
    assert_eq!(n, 2);
    for &o in &out[2..] {
        assert_eq!(o, 0.0);
    }
}

#[test]
fn second_derivative_composes() {
    // acc series = derivative of the vel series; check against analytic on T_4:
    // T_4' = 8·T_3 + 8·T_1 ; T_4'' = 8·(6T_2 + 3T_0) + 8·T_0 = 32·T_0 + 48·T_2.
    let mut vel = [0.0_f32; 8];
    let n_v = derivative_series(&[0.0, 0.0, 0.0, 0.0, 1.0], 1.0, &mut vel);
    assert_eq!(n_v, 4);
    assert_eq!(&vel[..4], &[0.0, 8.0, 0.0, 8.0]);
    let mut acc = [0.0_f32; 8];
    let n_a = derivative_series(vel.get(..n_v).unwrap(), 1.0, &mut acc);
    assert_eq!(n_a, 3);
    assert_eq!(&acc[..3], &[32.0, 0.0, 48.0]);
}
