#[test]
fn tiny_knot_range_evaluates_without_nan() {
    let curve = nurbs::ScalarNurbs::try_new(1, vec![0.0_f64, 0.0, 1e-8, 1e-8], vec![0.0, 1.0])
        .expect("tiny but positive range is valid");
    let mid = nurbs::eval::eval(&curve, 5e-9);
    assert!(mid.is_finite(), "expected finite eval, got {mid}");
}
