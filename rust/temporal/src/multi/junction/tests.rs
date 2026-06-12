use super::*;

fn line(from: [f64; 3], to: [f64; 3]) -> VectorNurbs<f64, 3> {
    VectorNurbs::try_new(1, vec![0.0, 0.0, 1.0, 1.0], vec![from, to]).unwrap()
}

#[test]
fn collinear_junction_is_smooth() {
    let a = line([0.0; 3], [40.0, 0.0, 0.0]);
    let b = line([40.0, 0.0, 0.0], [100.0, 0.0, 0.0]);
    assert!(matches!(
        classify_junction_curves(&a, &b),
        JunctionKind::Smooth
    ));
}

#[test]
fn right_angle_junction_is_corner() {
    let a = line([0.0; 3], [40.0, 0.0, 0.0]);
    let b = line([40.0, 0.0, 0.0], [40.0, 60.0, 0.0]);
    assert!(matches!(
        classify_junction_curves(&a, &b),
        JunctionKind::Corner
    ));
}

#[test]
fn reversal_junction_is_corner() {
    let a = line([0.0; 3], [40.0, 0.0, 0.0]);
    let b = line([40.0, 0.0, 0.0], [0.0, 0.0, 0.0]);
    assert!(matches!(
        classify_junction_curves(&a, &b),
        JunctionKind::Corner
    ));
}

#[test]
fn kink_just_above_fuse_threshold_is_corner() {
    let a = line([0.0; 3], [40.0, 0.0, 0.0]);
    let b = line([40.0, 0.0, 0.0], [80.0, 40.0 * (2e-3_f64).tan(), 0.0]);
    assert!(matches!(
        classify_junction_curves(&a, &b),
        JunctionKind::Corner
    ));
}
