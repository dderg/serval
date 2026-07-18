use servo_ident::analyze::project_modes;

const COREXY_FRAME: [[f64; 2]; 2] = [[0.5, 0.5], [0.5, -0.5]];

fn frame() -> Vec<Vec<f64>> {
    COREXY_FRAME.iter().map(|r| r.to_vec()).collect()
}

#[test]
fn equal_belt_error_projects_to_pure_x() {
    let a = vec![1.0, 2.0, -3.0];
    let b = a.clone();
    let modes = project_modes(&frame(), &[a.clone(), b]);
    assert_eq!(modes.len(), 2);
    assert_eq!(modes[0], a);
    assert_eq!(modes[1], vec![0.0, 0.0, 0.0]);
}

#[test]
fn opposite_belt_error_projects_to_pure_y() {
    let a = vec![2.0, -4.0];
    let b: Vec<f64> = a.iter().map(|v| -v).collect();
    let modes = project_modes(&frame(), &[a.clone(), b]);
    assert_eq!(modes[0], vec![0.0, 0.0]);
    assert_eq!(modes[1], a);
}

#[test]
fn mixed_error_splits_by_the_frame_weights() {
    let modes = project_modes(&frame(), &[vec![3.0], vec![1.0]]);
    assert_eq!(modes[0], vec![2.0]);
    assert_eq!(modes[1], vec![1.0]);
}

#[test]
#[should_panic(expected = "frame column count")]
fn frame_motor_count_mismatch_panics() {
    project_modes(&frame(), &[vec![1.0]]);
}

#[test]
#[should_panic(expected = "lengths differ")]
fn motor_series_length_mismatch_panics() {
    project_modes(&frame(), &[vec![1.0, 2.0], vec![1.0]]);
}
