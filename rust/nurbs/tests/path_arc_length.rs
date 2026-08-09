use nurbs::{VectorNurbs, arc_length::path_arc_length};

#[test]
fn path_arc_length_matches_dense_chord_sum_for_planar_curve() {
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
    let samples = 200_000;
    let mut chord_sum = 0.0_f64;
    let mut prev = nurbs::eval::vector_eval(&xyz, 0.0);
    for i in 1..=samples {
        let u = i as f64 / samples as f64;
        let p = nurbs::eval::vector_eval(&xyz, u);
        let dx = p[0] - prev[0];
        let dy = p[1] - prev[1];
        let dz = p[2] - prev[2];
        chord_sum += (dx * dx + dy * dy + dz * dz).sqrt();
        prev = p;
    }
    let full = path_arc_length(&xyz);
    assert!(
        (chord_sum - full).abs() < 1e-6,
        "chord {chord_sum} vs quadrature {full}"
    );
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
