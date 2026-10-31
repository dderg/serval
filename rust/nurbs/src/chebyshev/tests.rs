use super::*;

fn eval_monomial(c: &[f64], x: f64) -> f64 {
    c.iter().rev().fold(0.0, |acc, &ck| acc * x + ck)
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

#[test]
fn taylor_shift_preserves_degree_and_leading_coefficient() {
    let coeffs = [0.0, 1.0, -2.0, 4.0];
    let shifted = taylor_shift(&coeffs, -1.0);
    assert_eq!(shifted.len(), coeffs.len());
    assert_eq!(shifted[3], 4.0);
    for i in 0..=16 {
        let x = (i as f64) / 16.0;
        let want = eval_monomial(&coeffs, x - 1.0);
        let got = eval_monomial(&shifted, x);
        assert!((got - want).abs() <= 1e-13 * want.abs().max(1.0));
    }
}

#[test]
fn taylor_shift_of_constant_is_identity() {
    assert_eq!(taylor_shift(&[7.5], 3.0), vec![7.5]);
    assert!(taylor_shift(&[], 3.0).is_empty());
}
