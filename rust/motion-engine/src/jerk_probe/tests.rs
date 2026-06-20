use super::*;

#[test]
fn j_n_is_sum_of_terms() {
    let s = jerk_at(0.4, 1.5, 30.0, 800.0, 50_000.0);
    assert_eq!(s.j_n, s.j_n_geom + s.j_n_couple);
}

#[test]
fn j_t_passes_through_seg_jerk() {
    let s = jerk_at(0.1, 0.0, 10.0, 0.0, -42.0);
    assert_eq!(s.j_t, -42.0);
}

#[test]
fn geom_term_is_kappa_rate_times_v_cubed() {
    let v = 25.0;
    let s = jerk_at(0.0, 2.0, v, 0.0, 0.0);
    assert_eq!(s.j_n_geom, 2.0 * v * v * v);
    assert_eq!(s.j_n_couple, 0.0);
}

#[test]
fn couple_term_is_two_kappa_v_at() {
    let s = jerk_at(0.5, 0.0, 20.0, 1000.0, 0.0);
    assert_eq!(s.j_n_couple, 2.0 * 0.5 * 20.0 * 1000.0);
    assert_eq!(s.j_n_geom, 0.0);
}

#[test]
fn straight_line_zero_curvature_has_zero_lateral_jerk() {
    let s = jerk_at(0.0, 0.0, 50.0, 3000.0, 100_000.0);
    assert_eq!(s.j_n, 0.0);
    assert_eq!(s.j_t, 100_000.0);
}

#[test]
fn pure_function_same_inputs_same_output() {
    let a = jerk_at(0.3, 1.1, 18.0, 250.0, 7_000.0);
    let b = jerk_at(0.3, 1.1, 18.0, 250.0, 7_000.0);
    assert_eq!(a, b);
}
