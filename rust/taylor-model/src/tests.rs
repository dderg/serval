use super::{Ball, TaylorModel, Window, derivative_of};

const SAMPLES: u32 = 400;

fn eval(model: &TaylorModel, x: f64) -> f64 {
    model
        .coefficients()
        .iter()
        .rev()
        .fold(0.0, |acc, &c| acc.mul_add(x, c))
}

/// Asserts `truth` lies within the model's bound of its polynomial at
/// dense points, up to the polynomial's own float evaluation.
fn assert_encloses(model: &TaylorModel, truth: impl Fn(f64) -> f64, what: &str) {
    let magnitude: f64 = model.coefficients().iter().map(|c| c.abs()).sum();
    for k in 0..=SAMPLES {
        let x = -1.0 + 2.0 * f64::from(k) / f64::from(SAMPLES);
        let miss = (truth(x) - eval(model, x)).abs();
        let evaluation = 64.0 * f64::EPSILON * (magnitude + truth(x).abs());
        assert!(
            miss <= model.rad() + evaluation,
            "{what}: at x={x} the model misses by {miss:e}, past its bound {:e}",
            model.rad()
        );
    }
}

fn argument(centre: f64, half_width: f64) -> TaylorModel {
    TaylorModel::affine(Ball::exact(centre), half_width)
}

#[test]
fn a_tanh_model_encloses_tanh() {
    for (centre, half_width) in [(0.0, 0.4), (1.3, 0.3), (-2.5, 0.7), (8.0, 0.1)] {
        let model = argument(centre, half_width).tanh();
        assert_encloses(
            &model,
            |x| libm::tanh(x.mul_add(half_width, centre)),
            "tanh",
        );
        if half_width <= 0.4 {
            assert!(model.rad() < 1e-8, "tanh bound {:e}", model.rad());
        }
    }
}

#[test]
fn a_reciprocal_model_encloses_the_reciprocal_off_zero() {
    for (centre, half_width) in [(1.0, 0.5), (-3.0, 2.0), (1e-3, 2e-4)] {
        let model = argument(centre, half_width)
            .reciprocal()
            .expect("the range keeps off zero");
        assert_encloses(&model, |x| 1.0 / x.mul_add(half_width, centre), "1/x");
    }
    assert!(argument(1.0, 1.0).reciprocal().is_none());
}

#[test]
fn a_product_past_the_kept_degree_bounds_what_it_drops() {
    let unit = argument(0.0, 1.0);
    let falling_coefficients = [1.0, -0.5, 0.25, -0.125, 0.0625, 0.5, 0.75];
    let rising = derivative_of(&[1.0; 12], 0, &unit);
    let falling = derivative_of(&falling_coefficients, 0, &unit);
    let product = rising.mul(&falling);
    let value = |coefficients: &[f64], x: f64| {
        coefficients
            .iter()
            .rev()
            .fold(0.0_f64, |acc, &c| acc.mul_add(x, c))
    };
    assert_encloses(
        &product,
        |x| value(&[1.0; 12], x) * value(&falling_coefficients, x),
        "product",
    );
    assert!(product.rad() > 0.0, "the dropped terms join the bound");
}

#[test]
fn a_polynomial_in_shifted_time_and_its_derivative_are_enclosed() {
    let coefficients = [0.3, -2.0, 40.0, -900.0, 1.5e4];
    let window = Window::new(10.002, 10.006);
    let origin = 10.0;
    let tau = window.time_since(origin);
    let at = |x: f64| x.mul_add(window.radius, window.center - origin);
    let value = |tau: f64| coefficients.iter().rev().fold(0.0, |acc, &c| acc * tau + c);
    let slope = |tau: f64| {
        coefficients
            .iter()
            .enumerate()
            .skip(1)
            .rev()
            .fold(0.0, |acc, (power, &c)| acc * tau + power as f64 * c)
    };
    assert_encloses(
        &derivative_of(&coefficients, 0, &tau),
        |x| value(at(x)),
        "value",
    );
    assert_encloses(
        &derivative_of(&coefficients, 1, &tau),
        |x| slope(at(x)),
        "slope",
    );
}
