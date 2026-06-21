use super::*;

use nurbs::eval::vector_eval;

#[test]
fn g51_elevation_is_exact_against_the_quadratic() {
    let start = [0.0, 0.0, 0.0];
    let (i, j, dx, dy, dz) = (3.0, 5.0, 8.0, 0.0, 4.0);
    let cubic = g51_control_points(start, i, j, dx, dy, dz);

    let q0 = start;
    let q1 = [start[0] + i, start[1] + j, start[2] + dz / 2.0];
    let q2 = [start[0] + dx, start[1] + dy, start[2] + dz];
    let quad = |t: f64| {
        let mt = 1.0 - t;
        [0usize, 1, 2].map(|k| mt * mt * q0[k] + 2.0 * mt * t * q1[k] + t * t * q2[k])
    };

    let cubic_nurbs = nurbs::VectorNurbs::<f64, 3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        cubic.to_vec(),
    )
    .unwrap();
    for n in 0..=10 {
        let t = f64::from(n) / 10.0;
        let got = vector_eval(&cubic_nurbs, t);
        let want = quad(t);
        for k in 0..3 {
            assert!((got[k] - want[k]).abs() < 1e-12, "t={t} axis={k}");
        }
    }
}

#[test]
fn g51_z_is_linear_after_elevation() {
    let cps = g51_control_points([0.0, 0.0, 0.0], 1.0, 1.0, 0.0, 0.0, 6.0);
    assert!((cps[1][2] - 2.0).abs() < 1e-12); // dz/3
    assert!((cps[2][2] - 4.0).abs() < 1e-12); // 2dz/3
}

#[test]
fn collinear_places_control_points_at_thirds() {
    let cps = to_collinear_bezier([0.0, 0.0, 0.0], [9.0, 0.0, 0.0]);
    assert_eq!(cps[0], [0.0, 0.0, 0.0]);
    assert_eq!(cps[1], [3.0, 0.0, 0.0]);
    assert_eq!(cps[2], [6.0, 0.0, 0.0]);
    assert_eq!(cps[3], [9.0, 0.0, 0.0]);
}

#[test]
fn collinear_handles_3d_diagonal() {
    let cps = to_collinear_bezier([1.0, 2.0, 3.0], [4.0, 8.0, 6.0]);
    assert_eq!(cps[1], [2.0, 4.0, 4.0]);
    assert_eq!(cps[2], [3.0, 6.0, 5.0]);
}

#[test]
fn g5_assembles_control_points_with_linear_z() {
    let cps = g5_control_points([0.0, 0.0, 0.0], 2.0, 4.0, -3.0, 4.0, 10.0, 0.0, 6.0);
    assert_eq!(cps[0], [0.0, 0.0, 0.0]); // P0 = start
    assert_eq!(cps[1], [2.0, 4.0, 2.0]); // P1 = start+(I,J), z = dz/3
    assert_eq!(cps[2], [7.0, 4.0, 4.0]); // P2 = end+(P,Q) = (10-3,0+4), z = 2dz/3
    assert_eq!(cps[3], [10.0, 0.0, 6.0]); // P3 = end
}
