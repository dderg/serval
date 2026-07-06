use super::*;

#[allow(clippy::float_cmp)]
#[test]
fn ref_provides_borrowed_access() {
    let s = [0.0_f64, 0.5, 1.0];
    let u = [0.0_f64, 0.4, 1.0];
    let r = ArcLengthTableRef::new(&s, &u);
    assert_eq!(r.s_max(), 1.0);
    assert_eq!(r.u_max(), 1.0);
}

#[cfg(feature = "host")]
#[allow(clippy::float_cmp)]
#[test]
fn owned_as_view_round_trips() {
    let owned = ArcLengthTable::new(vec![0.0, 0.5, 1.0], vec![0.0, 0.4, 1.0]);
    let view = owned.as_view();
    assert_eq!(view.s_max(), 1.0);
}

#[cfg(feature = "host")]
#[test]
fn integrate_constant_returns_length_times_constant() {
    let result = integrate_arc_length(|_u: f64| 2.0_f64, 0.0, 1.0, 5);
    assert!((result - 2.0).abs() < 1e-12);
}

#[cfg(feature = "host")]
#[test]
fn integrate_linear_matches_closed_form() {
    let result = integrate_arc_length(|u: f64| u, 0.0, 1.0, 5);
    assert!((result - 0.5).abs() < 1e-12);
}

#[cfg(feature = "host")]
#[test]
fn integrate_quadratic_matches_closed_form() {
    let result = integrate_arc_length(|u: f64| u * u, 0.0, 1.0, 5);
    assert!((result - 1.0 / 3.0).abs() < 1e-12);
}

#[allow(clippy::float_cmp)]
#[test]
fn param_from_arc_length_at_endpoints() {
    let table = ArcLengthTableRef::new(&[0.0_f64, 0.5, 1.0], &[0.0, 0.6, 1.0]);
    assert_eq!(param_from_arc_length(&table, 0.0), 0.0);
    assert_eq!(param_from_arc_length(&table, 1.0), 1.0);
}

#[test]
fn param_from_arc_length_interpolates_linearly() {
    let table = ArcLengthTableRef::new(&[0.0_f64, 0.5, 1.0], &[0.0, 0.6, 1.0]);
    let expected_interp = 0.3;
    assert!((param_from_arc_length(&table, 0.25_f64) - expected_interp).abs() < 1e-12);
}

#[allow(clippy::float_cmp)]
#[test]
fn param_from_arc_length_clamps_above_range_in_release() {
    let table = ArcLengthTableRef::new(&[0.0_f64, 1.0], &[0.0, 1.0]);
    let v = param_from_arc_length(&table, 1.0_f64);
    assert_eq!(v, 1.0);
}

#[test]
fn arc_length_from_param_inverts_param_from_arc_length() {
    let table = ArcLengthTableRef::new(&[0.0_f64, 0.4, 1.0], &[0.0, 0.5, 1.0]);
    let u = 0.3_f64;
    let s = arc_length_from_param(&table, u);
    let u_back = param_from_arc_length(&table, s);
    assert!((u - u_back).abs() < 1e-12);
}

#[cfg(feature = "host")]
#[test]
fn build_vector_table_for_3d_linear_curve() {
    let curve = crate::VectorNurbs::try_new(
        1,
        vec![0.0_f64, 0.0, 1.0, 1.0],
        vec![[0.0, 0.0, 0.0], [3.0, 0.0, 4.0]],
    )
    .unwrap();
    let table = build_arc_length_table_vector(&curve, 1e-5, 64).unwrap();
    assert!((table.s_max() - 5.0).abs() < 1e-4);
}
