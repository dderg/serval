use nurbs::chebyshev::taylor_shift;
use proptest::prelude::*;
use proptest::test_runner::FileFailurePersistence;

fn horner(coeffs: &[f64], x: f64) -> f64 {
    coeffs.iter().rev().fold(0.0, |acc, &c| acc * x + c)
}

/// Forward-error bound for evaluating a shifted monomial expansion: the shift
/// spreads `|c_i|` over the binomials of `dt`, so every partial sum is bounded
/// by `sum |c_i| (|dt| + |x|)^i`.
fn shift_error_budget(coeffs: &[f64], dt: f64, x: f64) -> f64 {
    let reach = coeffs
        .iter()
        .rev()
        .fold(0.0, |acc, c| acc * (dt.abs() + x.abs()) + c.abs());
    8.0 * (coeffs.len() as f64) * f64::EPSILON * reach
}

fn arb_coeffs() -> impl Strategy<Value = Vec<f64>> {
    prop::collection::vec(-8.0..8.0_f64, 1..=8)
}

fn arb_samples() -> impl Strategy<Value = Vec<f64>> {
    prop::collection::vec(-2.0..2.0_f64, 32)
}

proptest! {
    #![proptest_config(ProptestConfig {
        cases: 512,
        failure_persistence: Some(Box::new(FileFailurePersistence::Direct(
            "proptest-regressions/chebyshev.txt",
        ))),
        ..ProptestConfig::default()
    })]

    #[test]
    fn taylor_shift_evaluates_as_the_shifted_polynomial(
        coeffs in arb_coeffs(),
        dt in -2.0..2.0_f64,
        xs in arb_samples(),
    ) {
        let shifted = taylor_shift(&coeffs, dt);
        for x in xs {
            let want = horner(&coeffs, x + dt);
            let got = horner(&shifted, x);
            let budget = shift_error_budget(&coeffs, dt, x);
            prop_assert!(
                (got - want).abs() <= budget,
                "dt={dt} x={x}: got {got}, want {want}, budget {budget}, coeffs {coeffs:?}"
            );
        }
    }

    #[test]
    fn taylor_shift_preserves_degree_and_leading_coefficient(
        coeffs in arb_coeffs(),
        dt in -2.0..2.0_f64,
    ) {
        let shifted = taylor_shift(&coeffs, dt);
        prop_assert_eq!(shifted.len(), coeffs.len());
        prop_assert_eq!(
            shifted[shifted.len() - 1],
            coeffs[coeffs.len() - 1],
            "a shift is unitriangular: the leading coefficient must be untouched"
        );
    }

    #[test]
    fn taylor_shift_by_zero_is_the_identity(coeffs in arb_coeffs()) {
        prop_assert_eq!(taylor_shift(&coeffs, 0.0), coeffs);
    }

    #[test]
    fn taylor_shift_composes_additively(
        coeffs in arb_coeffs(),
        first in -1.5..1.5_f64,
        second in -1.5..1.5_f64,
        xs in arb_samples(),
    ) {
        let twice = taylor_shift(&taylor_shift(&coeffs, first), second);
        let once = taylor_shift(&coeffs, first + second);
        for x in xs {
            let stepwise = horner(&twice, x);
            let direct = horner(&once, x);
            let budget = shift_error_budget(&coeffs, first.abs() + second.abs(), x);
            prop_assert!(
                (stepwise - direct).abs() <= budget,
                "first={first} second={second} x={x}: {stepwise} vs {direct}, budget {budget}"
            );
        }
    }
}
