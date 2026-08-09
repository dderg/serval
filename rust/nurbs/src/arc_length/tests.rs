use super::*;

#[test]
fn integrate_constant_returns_length_times_constant() {
    let result = integrate_arc_length(|_u: f64| 2.0_f64, 0.0, 1.0, 5);
    assert!((result - 2.0).abs() < 1e-12);
}

#[test]
fn integrate_linear_matches_closed_form() {
    let result = integrate_arc_length(|u: f64| u, 0.0, 1.0, 5);
    assert!((result - 0.5).abs() < 1e-12);
}

#[test]
fn integrate_quadratic_matches_closed_form() {
    let result = integrate_arc_length(|u: f64| u * u, 0.0, 1.0, 5);
    assert!((result - 1.0 / 3.0).abs() < 1e-12);
}
