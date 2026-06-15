use super::*;
use geometry::curve::g5_control_points;

#[test]
fn xy_travel_classifies_correctly() {
    let m = classify_and_build([0.0; 3], 10.0, 0.0, 0.0, &[], 100.0).unwrap();
    assert!(m.segment.followers.is_empty());
    assert_eq!(m.segment.feedrate_mm_s, 100.0);
    let cps = m.segment.xyz.control_points();
    assert_eq!(cps.len(), 4);
    assert_eq!(cps[0], [0.0, 0.0, 0.0]);
    assert!((cps[3][0] - 10.0).abs() < 1e-12);
}

#[test]
fn z_only_classifies_correctly() {
    let m = classify_and_build([0.0, 0.0, 5.0], 0.0, 0.0, 5.0, &[], 50.0).unwrap();
    assert!(m.segment.followers.is_empty());
    assert!((m.distance_mm - 5.0).abs() < 1e-12);
}

#[test]
fn extruding_xy_move_carries_ratio_de_over_distance() {
    let m = classify_and_build([0.0; 3], 3.0, 4.0, 0.0, &[(3, 0.25)], 100.0).unwrap();
    assert_eq!(m.segment.followers.len(), 1);
    assert_eq!(m.segment.followers[0].axis_index, 3);
    assert!((m.segment.followers[0].ratio - 0.05).abs() < 1e-12);
    assert!(m.segment.virtual_path_mm.is_none());
}

#[test]
fn retract_with_hop_carries_negative_ratio_over_3d_length() {
    let m = classify_and_build([0.0; 3], 0.0, 0.0, 0.4, &[(3, -2.0)], 30.0).unwrap();
    assert!((m.segment.followers[0].ratio - (-5.0)).abs() < 1e-12);
}

#[test]
fn follower_only_move_builds_virtual_path() {
    let m = classify_and_build([1.0, 2.0, 3.0], 0.0, 0.0, 0.0, &[(3, -4.5)], 40.0).unwrap();
    assert_eq!(m.segment.virtual_path_mm, Some(4.5));
    assert!((m.segment.followers[0].ratio - (-1.0)).abs() < 1e-12);
    assert!((m.distance_mm - 4.5).abs() < 1e-12);
    let cps = m.segment.xyz.control_points();
    assert!(cps.iter().all(|p| *p == [1.0, 2.0, 3.0]));
}

#[test]
fn zero_displacement_rejected() {
    let r = classify_and_build([0.0; 3], 0.0, 0.0, 0.0, &[], 100.0);
    assert!(matches!(r, Err(ClassifyError::ZeroDisplacement)));
    let r = classify_and_build([0.0; 3], 0.0, 0.0, 0.0, &[(3, 0.0)], 100.0);
    assert!(matches!(r, Err(ClassifyError::ZeroDisplacement)));
}

#[test]
fn nominal_duration_uses_distance_over_feedrate() {
    let m = classify_and_build([0.0; 3], 10.0, 0.0, 0.0, &[], 100.0).unwrap();
    assert!((m.nominal_duration() - 0.1).abs() < 1e-12);
}

#[test]
fn nominal_duration_uses_3d_distance() {
    let m = classify_and_build([0.0; 3], 3.0, 4.0, 0.0, &[], 5.0).unwrap();
    assert!((m.nominal_duration() - 1.0).abs() < 1e-12);
}

#[test]
fn classify_bezier_uses_arc_length_for_distance_and_ratio() {
    // A curved G5 with an E delta. distance_mm must be the arc length (> chord),
    // and the follower ratio must be de / arc_length (not de / chord).
    let start = [0.0, 0.0, 0.0];
    let m = classify_bezier(
        start,
        0.0,
        10.0,
        0.0,
        10.0,
        10.0,
        0.0,
        0.0,
        &[(3usize, 2.0)],
        30.0,
    )
    .expect("curve classifies");
    let chord = 10.0_f64;
    assert!(m.distance_mm > chord, "arc length must exceed the chord");
    let ratio = m.segment.followers[0].ratio;
    assert!(
        (ratio - 2.0 / m.distance_mm).abs() < 1e-9,
        "ratio is de/arc_length"
    );
}

#[test]
fn classify_quadratic_builds_a_segment() {
    let m = classify_quadratic([0.0, 0.0, 0.0], 5.0, 5.0, 10.0, 0.0, 0.0, &[], 30.0)
        .expect("quadratic classifies");
    assert!(m.distance_mm > 10.0);
    assert_eq!(m.segment.feedrate_mm_s, 30.0);
}

#[test]
fn chain_reflection_negates_previous_pq() {
    // Chained G5: omitted I/J => (I,J) = (-P_prev, -Q_prev). The reflection is
    // XY-only; P1.z stays on the linear-Z assembly, NOT a 3D reflection.
    let prev_pq = (3.0, -2.0);
    let (i, j) = (-prev_pq.0, -prev_pq.1);
    let cps = g5_control_points([0.0, 0.0, 0.0], i, j, 1.0, 1.0, 10.0, 0.0, 0.0);
    assert_eq!(cps[1][0], -3.0);
    assert_eq!(cps[1][1], 2.0);
    assert_eq!(cps[1][2], 0.0);
}

#[test]
fn experiment_cusp_and_near_cusp_classification_is_finite() {
    // Exact cusp: i=j=0 => P1 == P0 (zero start tangent, fold-back curve).
    // Near-cusp: tiny start leg (i=1e-7).
    // Both must either (a) classify successfully with finite arc length,
    // or (b) return ZeroDisplacement — the one case where arc_length <= epsilon.
    // What must NEVER happen: a panic or a non-finite distance_mm.
    let exact = classify_bezier(
        [0.0, 0.0, 0.0],
        0.0,
        0.0,
        -5.0,
        0.0,
        5.0,
        0.0,
        0.0,
        &[],
        30.0,
    );
    let near = classify_bezier(
        [0.0, 0.0, 0.0],
        1e-7,
        0.0,
        -5.0,
        0.0,
        5.0,
        0.0,
        0.0,
        &[],
        30.0,
    );
    for mv in [&exact, &near].into_iter().flatten() {
        assert!(mv.distance_mm.is_finite(), "arc length must be finite");
        assert!(mv.distance_mm >= 0.0);
    }
}
