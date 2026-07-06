use nurbs::{VectorNurbs, arc_length::path_arc_length};

#[test]
fn path_arc_length_matches_table_for_planar_curve() {
    let xyz = VectorNurbs::<3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![
            [0.0, 0.0, 0.0],
            [10.0, 0.0, 0.0],
            [20.0, 10.0, 0.0],
            [30.0, 10.0, 0.0],
        ],
    )
    .unwrap();
    let table = nurbs::arc_length::build_arc_length_table_vector(&xyz, 1e-9, 64).unwrap();
    let full = path_arc_length(&xyz);
    assert!((table.s_max() - full).abs() < 1e-7);
}

#[test]
fn path_arc_length_includes_z_component() {
    let xyz = VectorNurbs::<3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
            [0.0, 0.0, 2.0],
            [0.0, 0.0, 3.0],
        ],
    )
    .unwrap();
    assert!((path_arc_length(&xyz) - 3.0).abs() < 1e-9);
}

#[test]
fn path_arc_length_diagonal_line_exact() {
    let xyz = VectorNurbs::<3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![
            [0.0, 0.0, 0.0],
            [1.0, 1.0, 1.0],
            [2.0, 2.0, 2.0],
            [3.0, 3.0, 3.0],
        ],
    )
    .unwrap();
    assert!((path_arc_length(&xyz) - 3.0 * 3.0_f64.sqrt()).abs() < 1e-9);
}
