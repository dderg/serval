use super::*;

fn eval_monomial(c: &[f64], x: f64) -> f64 {
    c.iter().rev().fold(0.0, |acc, &ck| acc * x + ck)
}

fn eval_chebyshev(a: &[f64], u: f64) -> f64 {
    let mut b1 = 0.0;
    let mut b2 = 0.0;
    for &ak in a.iter().skip(1).rev() {
        let b0 = ak + 2.0 * u * b1 - b2;
        b2 = b1;
        b1 = b0;
    }
    a[0] + u * b1 - b2
}

#[test]
fn tau_to_chebyshev_matches_direct_evaluation() {
    let coeffs = [1.25, -3.0, 0.5, 2.0, -0.75, 0.125, 4.0, -1.5];
    let h = 0.0125;
    let a = monomial_tau_to_chebyshev(&coeffs, h);
    assert_eq!(a.len(), coeffs.len());
    for i in 0..=64 {
        let u = -1.0 + 2.0 * (i as f64) / 64.0;
        let tau = 0.5 * h * (u + 1.0);
        let want = eval_monomial(&coeffs, tau);
        let got = eval_chebyshev(&a, u);
        assert!(
            (got - want).abs() <= 1e-12 * want.abs().max(1.0),
            "u={u}: {got} vs {want}"
        );
    }
}

#[test]
fn round_trip_is_ulp_scale() {
    let coeffs = [0.5, 120.0, -3000.0, 40000.0, -1.0e5, 2.0e5, -1.0e5, 5.0e4];
    let h = 0.004;
    let back = chebyshev_to_monomial_tau(&monomial_tau_to_chebyshev(&coeffs, h), h);
    assert_eq!(back.len(), coeffs.len());
    // Round-trip error must stay ulp-scale in *value* over the piece domain.
    for i in 0..=32 {
        let tau = h * (i as f64) / 32.0;
        let want = eval_monomial(&coeffs, tau);
        let got = eval_monomial(&back, tau);
        assert!(
            (got - want).abs() <= 1e-11 * want.abs().max(1.0),
            "tau={tau}: {got} vs {want}"
        );
    }
}

#[test]
fn low_degree_input_yields_exactly_zero_tail() {
    for deg in 0..7 {
        let mut coeffs = vec![0.0; 8];
        for (k, c) in coeffs.iter_mut().enumerate().take(deg + 1) {
            *c = 1.0 + k as f64 * 0.37;
        }
        let a = monomial_tau_to_chebyshev(&coeffs, 0.017);
        for (k, &ak) in a.iter().enumerate().skip(deg + 1) {
            assert_eq!(ak, 0.0, "deg-{deg} input produced nonzero T_{k}");
        }
    }
}

#[test]
fn chebyshev_basis_maps_to_known_monomials() {
    // T_3(u) = 4u³ − 3u on h = 2 (so τ = u + 1: taylor shift only).
    let mono_u = chebyshev_to_monomial_tau(&[0.0, 0.0, 0.0, 1.0], 2.0);
    let expect = taylor_shift(&[0.0, -3.0, 0.0, 4.0], -1.0);
    for (got, want) in mono_u.iter().zip(&expect) {
        assert!((got - want).abs() < 1e-12, "{mono_u:?} vs {expect:?}");
    }
}

#[test]
fn truncate_respects_sup_norm_budget() {
    let a = [10.0, 5.0, 1.0, 0.04, 0.05, 0.0];
    // Tail |0.0| + |0.05| + |0.04| = 0.09 ≤ 0.1 drops; |1.0| does not.
    assert_eq!(truncate_chebyshev(&a, 0.1), vec![10.0, 5.0, 1.0]);
    assert_eq!(truncate_chebyshev(&a, 0.08), vec![10.0, 5.0, 1.0, 0.04]);
    assert_eq!(
        truncate_chebyshev(&a, 0.01),
        vec![10.0, 5.0, 1.0, 0.04, 0.05]
    );
    assert_eq!(truncate_chebyshev(&[1e-30], 1.0), vec![1e-30]);
    assert_eq!(truncate_chebyshev(&[0.0, 0.0], 0.0), vec![0.0]);
}

#[test]
fn taylor_shift_splits_a_piece_continuously() {
    let coeffs = [2.0, -1.5, 3.25, 0.75, -0.5, 0.125];
    let split = 0.3;
    let right = taylor_shift(&coeffs, split);
    for i in 0..=20 {
        let tau = 0.5 * (i as f64) / 20.0;
        let want = eval_monomial(&coeffs, split + tau);
        let got = eval_monomial(&right, tau);
        assert!(
            (got - want).abs() <= 1e-13 * want.abs().max(1.0),
            "tau={tau}: {got} vs {want}"
        );
    }
    assert!((eval_monomial(&right, 0.0) - eval_monomial(&coeffs, split)).abs() < 1e-14);
}
