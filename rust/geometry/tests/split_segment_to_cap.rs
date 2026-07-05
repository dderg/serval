#![allow(clippy::float_cmp)]

use geometry::{CubicSegment, FollowerDemand, SourceRange, split_segment_to_cap};
use nurbs::VectorNurbs;

fn straight_cubic(length_mm: f64) -> CubicSegment {
    let xyz = VectorNurbs::<f64, 3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![
            [0.0, 0.0, 0.0],
            [length_mm / 3.0, 0.0, 0.0],
            [2.0 * length_mm / 3.0, 0.0, 0.0],
            [length_mm, 0.0, 0.0],
        ],
    )
    .unwrap();
    CubicSegment::try_new(
        xyz,
        vec![],
        100.0,
        SourceRange {
            start_line: 1,
            end_line: 1,
        },
        None,
    )
    .unwrap()
}

#[test]
fn passthrough_when_below_cap() {
    let seg = straight_cubic(5.0);
    let out = split_segment_to_cap(&seg, 12.5).unwrap();
    assert_eq!(out.len(), 1);
    assert!(out[0].split_info.is_none());
}

#[test]
fn passthrough_at_exact_cap() {
    let seg = straight_cubic(12.5);
    let out = split_segment_to_cap(&seg, 12.5).unwrap();
    assert_eq!(out.len(), 1);
    assert!(out[0].split_info.is_none());
}

#[test]
fn splits_into_two_at_25mm() {
    let seg = straight_cubic(25.0);
    let out = split_segment_to_cap(&seg, 12.5).unwrap();
    assert_eq!(out.len(), 2);
    for (i, child) in out.iter().enumerate() {
        let info = child.split_info.expect("split_info populated");
        assert_eq!(info.sub_index, i as u32);
        assert_eq!(info.sub_count, 2);
    }
}

#[test]
fn splits_into_eight_at_100mm() {
    let seg = straight_cubic(100.0);
    let out = split_segment_to_cap(&seg, 12.5).unwrap();
    assert_eq!(out.len(), 8);
}

#[test]
fn metadata_propagates() {
    let seg = straight_cubic(50.0);
    let out = split_segment_to_cap(&seg, 12.5).unwrap();
    for child in &out {
        assert_eq!(child.feedrate_mm_s, seg.feedrate_mm_s);
        assert_eq!(child.followers, seg.followers);
        assert_eq!(child.source, seg.source);
    }
}

#[test]
fn boundary_continuity_within_round_off() {
    const BOUNDARY_TOL: f64 = 1e-12;
    use nurbs::eval::vector_eval;
    let seg = straight_cubic(50.0);
    let out = split_segment_to_cap(&seg, 12.5).unwrap();
    for window in out.windows(2) {
        let left_end = vector_eval(&window[0].xyz, 1.0);
        let right_start = vector_eval(&window[1].xyz, 0.0);
        for axis in 0..3 {
            let diff = (left_end[axis] - right_start[axis]).abs();
            assert!(
                diff < BOUNDARY_TOL,
                "boundary mismatch axis {axis}: {left_end:?} vs {right_start:?}, diff={diff}"
            );
        }
    }
}

#[test]
fn zero_motion_segment_passthrough() {
    let xyz = VectorNurbs::<f64, 3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![[0.0; 3]; 4], // all four CPs at origin → cp_polygon_length == 0
    )
    .unwrap();
    let seg = CubicSegment::try_new(
        xyz,
        vec![],
        100.0,
        SourceRange {
            start_line: 1,
            end_line: 1,
        },
        None,
    )
    .unwrap();
    let out = split_segment_to_cap(&seg, 12.5).unwrap();
    assert_eq!(out.len(), 1);
    assert!(out[0].split_info.is_none());
}

#[test]
fn closed_loop_chord_zero_splits_by_arc_length() {
    let xyz = VectorNurbs::<f64, 3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![
            [0.0, 0.0, 0.0],
            [50.0, 50.0, 0.0],
            [-50.0, 50.0, 0.0],
            [0.0, 0.0, 0.0],
        ],
    )
    .unwrap();
    let seg = CubicSegment::try_new(
        xyz,
        vec![],
        100.0,
        SourceRange {
            start_line: 1,
            end_line: 1,
        },
        None,
    )
    .unwrap();

    let out = split_segment_to_cap(&seg, 12.5).unwrap();
    assert!(out.len() > 1, "closed loop should split, not passthrough");
}

#[test]
fn invalid_cap_rejects_zero() {
    let seg = straight_cubic(50.0);
    let err = split_segment_to_cap(&seg, 0.0).unwrap_err();
    assert!(
        matches!(err, geometry::SplitError::InvalidCap { .. }),
        "got {err:?}"
    );
}

#[test]
fn invalid_cap_rejects_negative() {
    let seg = straight_cubic(50.0);
    let err = split_segment_to_cap(&seg, -1.0).unwrap_err();
    assert!(
        matches!(err, geometry::SplitError::InvalidCap { .. }),
        "got {err:?}"
    );
}

#[test]
fn invalid_cap_rejects_nan() {
    let seg = straight_cubic(50.0);
    let err = split_segment_to_cap(&seg, f64::NAN).unwrap_err();
    assert!(
        matches!(err, geometry::SplitError::InvalidCap { .. }),
        "got {err:?}"
    );
}

#[test]
fn invalid_cap_rejects_infinity() {
    let seg = straight_cubic(50.0);
    let err = split_segment_to_cap(&seg, f64::INFINITY).unwrap_err();
    assert!(
        matches!(err, geometry::SplitError::InvalidCap { .. }),
        "got {err:?}"
    );
}

#[test]
fn follower_segment_splits_with_invariant_ratio() {
    let xyz = VectorNurbs::<f64, 3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 10.0],
            [0.0, 0.0, 20.0],
            [0.0, 0.0, 30.0],
        ],
    )
    .unwrap();
    let followers = vec![FollowerDemand::constant(3, 5.0 / 30.0)];
    let seg = CubicSegment::try_new(
        xyz,
        followers.clone(),
        100.0,
        SourceRange {
            start_line: 1,
            end_line: 1,
        },
        None,
    )
    .unwrap();

    let out = split_segment_to_cap(&seg, 12.5).unwrap();
    assert!(out.len() > 1, "30mm at 12.5mm cap should split");
    for child in &out {
        assert_eq!(child.followers, followers);
    }
}

#[test]
fn ramped_follower_splits_by_arc_length_span() {
    // A follower ramping r0 → r1 over the parent must yield child ramps that
    // interpolate the parent at each child's arc-length span: consecutive
    // children meet continuously, the ends match r0/r1, and total E is conserved.
    let length = 30.0;
    let (r0, r1) = (0.1, 0.4);
    let seg = {
        let mut s = straight_cubic(length);
        s.followers = vec![FollowerDemand::ramp(3, r0, r1)];
        s
    };
    let out = split_segment_to_cap(&seg, 8.0).unwrap();
    assert!(out.len() > 2, "30mm at 8mm cap should split into >2");

    let ratio_at = |s: f64| r0 + (r1 - r0) * s / length;
    let mut total_e = 0.0;
    for (i, child) in out.iter().enumerate() {
        let info = child.split_info.expect("child carries split info");
        let f = child.followers[0];
        assert!((f.ratio - ratio_at(info.s_lo_mm)).abs() < 1e-9);
        assert!((f.ratio_end - ratio_at(info.s_hi_mm)).abs() < 1e-9);
        if i + 1 < out.len() {
            let next = out[i + 1].followers[0];
            assert!(
                (f.ratio_end - next.ratio).abs() < 1e-9,
                "child ramps discontinuous at boundary {i}"
            );
        }
        total_e += f.delta_over(info.s_hi_mm - info.s_lo_mm);
    }
    assert!((out[0].followers[0].ratio - r0).abs() < 1e-9);
    assert!((out.last().unwrap().followers[0].ratio_end - r1).abs() < 1e-9);
    assert!(
        (total_e - 0.5 * (r0 + r1) * length).abs() < 1e-9,
        "total extrusion not conserved: {total_e}"
    );
}
