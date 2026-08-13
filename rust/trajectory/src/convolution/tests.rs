use nurbs::algebra::PiecewisePolynomialKernel;

use super::ShapedSignal;

fn degree_six_kernel() -> PiecewisePolynomialKernel {
    PiecewisePolynomialKernel::single_poly_from_absolute(
        vec![1.0, 0.0, -3.0, 0.0, 3.0, 0.0, -1.0],
        (-1.0, 1.0),
    )
}

fn binomial(n: usize, k: usize) -> f64 {
    (0..k).fold(1.0, |value, i| value * (n - i) as f64 / (i + 1) as f64)
}

#[test]
fn degree_thirteen_convolution_is_exact() {
    let kernel = degree_six_kernel();
    let signal = ShapedSignal::new_from_evaluator(&kernel, |t| t.powi(7), Vec::new(), 7);
    let t = 0.37_f64;
    let mut expected = 0.0;
    for input_power in 0..=7 {
        let input_coefficient = binomial(7, input_power)
            * t.powi((7 - input_power) as i32)
            * if input_power % 2 == 0 { 1.0 } else { -1.0 };
        for (kernel_power, kernel_coefficient) in [(0, 1.0), (2, -3.0), (4, 3.0), (6, -1.0)] {
            let power = input_power + kernel_power;
            if power % 2 == 0 {
                expected += input_coefficient * kernel_coefficient * 2.0 / (power + 1) as f64;
            }
        }
    }
    assert!((signal.eval(t) - expected).abs() < 1e-12);
}

#[test]
#[should_panic(expected = "exceeds exact quadrature degree 13")]
fn convolution_rejects_unrepresentable_product_degree() {
    let kernel = degree_six_kernel();
    let _ = ShapedSignal::new_from_evaluator(&kernel, |t| t.powi(8), Vec::new(), 8);
}
