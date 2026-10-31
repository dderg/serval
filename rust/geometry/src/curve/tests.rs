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
