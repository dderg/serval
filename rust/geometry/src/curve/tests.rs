use super::*;

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
    // start (0,0,0), endpoint delta (10,0,6), I/J=(2,4), P/Q=(-3,4)
    let cps = g5_control_points([0.0, 0.0, 0.0], 2.0, 4.0, -3.0, 4.0, 10.0, 0.0, 6.0);
    assert_eq!(cps[0], [0.0, 0.0, 0.0]); // P0 = start
    assert_eq!(cps[1], [2.0, 4.0, 2.0]); // P1 = start+(I,J), z = dz/3
    assert_eq!(cps[2], [7.0, 4.0, 4.0]); // P2 = end+(P,Q) = (10-3,0+4), z = 2dz/3
    assert_eq!(cps[3], [10.0, 0.0, 6.0]); // P3 = end
}
