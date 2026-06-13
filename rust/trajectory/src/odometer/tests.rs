use super::*;

fn linear_cubic(p0: f64, p1: f64, t0: f64, t1: f64) -> nurbs::ScalarNurbs<f64> {
    nurbs::ScalarNurbs::try_new(
        3,
        vec![t0, t0, t0, t0, t1, t1, t1, t1],
        vec![p0, p0 + (p1 - p0) / 3.0, p0 + 2.0 * (p1 - p0) / 3.0, p1],
    )
    .unwrap()
}

fn constant_cubic(v: f64, t0: f64, t1: f64) -> nurbs::ScalarNurbs<f64> {
    linear_cubic(v, v, t0, t1)
}

#[test]
fn straight_line_distance_is_exact() {
    let axes = vec![
        linear_cubic(0.0, 240.0, 0.0, 2.0),
        linear_cubic(0.0, 100.0, 0.0, 2.0),
        constant_cubic(0.0, 0.0, 2.0),
    ];
    let odo = Odometer::build(&axes, 0.0, 2.0, 64).unwrap();
    assert!((odo.distance_at(1.0) - 130.0).abs() < 1e-9);
    assert!((odo.distance_at(2.0) - 260.0).abs() < 1e-9);
}

#[test]
fn curved_path_matches_dense_reference() {
    let x = nurbs::ScalarNurbs::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![0.0, 55.0, 100.0, 100.0],
    )
    .unwrap();
    let y = nurbs::ScalarNurbs::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![0.0, 0.0, 45.0, 100.0],
    )
    .unwrap();
    let z = constant_cubic(0.0, 0.0, 1.0);

    let dx = nurbs::eval::derivative(&x);
    let dy = nurbs::eval::derivative(&y);
    let n = 100_000usize;
    let speed = |t: f64| {
        let vx = nurbs::eval::eval(&dx, t);
        let vy = nurbs::eval::eval(&dy, t);
        vx.hypot(vy)
    };
    let mut reference = 0.0;
    for i in 0..n {
        let t0 = i as f64 / n as f64;
        let t1 = (i + 1) as f64 / n as f64;
        reference += 0.5 * (speed(t0) + speed(t1)) * (t1 - t0);
    }

    let axes = vec![x, y, z];
    let odo = Odometer::build(&axes, 0.0, 1.0, 64).unwrap();
    let got = odo.distance_at(1.0);
    assert!(
        ((got - reference) / reference).abs() < 1e-7,
        "got {got}, reference {reference}"
    );
}

#[test]
fn follower_track_pays_out_ratio_times_distance() {
    let axes = vec![
        linear_cubic(0.0, 240.0, 0.0, 2.0),
        linear_cubic(0.0, 100.0, 0.0, 2.0),
        constant_cubic(0.0, 0.0, 2.0),
    ];
    let odo = Odometer::build(&axes, 0.0, 2.0, 64).unwrap();
    let track = follower_track(&odo, 7.0, &[(1.0, 0.05), (2.0, 0.0)], 0.0, 2.0).unwrap();
    let expected_end = 7.0 + 0.05 * odo.distance_at(1.0);
    assert!((track(2.0) - expected_end).abs() < 1e-9);
    let mut prev = track(0.0);
    assert!((prev - 7.0).abs() < 1e-12);
    for i in 1..=100 {
        let t = i as f64 / 100.0;
        let v = track(t);
        assert!(v >= prev - 1e-12, "not monotone at t={t}");
        prev = v;
    }
}

#[test]
fn ratio_discontinuity_lands_at_segment_boundary_sample() {
    let x_fast_then_slow = nurbs::bezier::bezier_pieces_to_nurbs(&[
        nurbs::bezier::BezierPiece {
            u_start: 0.0,
            u_end: 1.0,
            coeffs: vec![0.0, 10.0, 0.0, 0.0],
        },
        nurbs::bezier::BezierPiece {
            u_start: 1.0,
            u_end: 2.0,
            coeffs: vec![10.0, 20.0, 0.0, 0.0],
        },
    ]);
    let axes = vec![
        x_fast_then_slow,
        constant_cubic(0.0, 0.0, 2.0),
        constant_cubic(0.0, 0.0, 2.0),
    ];
    let odo = Odometer::build(&axes, 0.0, 2.0, 4).unwrap();
    let track = follower_track(&odo, 0.0, &[(1.0, 1.0), (2.0, 0.0)], 0.0, 2.0).unwrap();
    assert!((track(2.0) - 10.0).abs() < 1e-9);
    assert!((track(1.0) - 10.0).abs() < 1e-9);
}

#[test]
fn span_gap_is_a_loud_error() {
    let axes = vec![
        linear_cubic(0.0, 10.0, 0.0, 2.0),
        constant_cubic(0.0, 0.0, 2.0),
        constant_cubic(0.0, 0.0, 2.0),
    ];
    let odo = Odometer::build(&axes, 0.0, 2.0, 4).unwrap();
    assert!(follower_track(&odo, 0.0, &[(1.0, 0.05)], 0.0, 2.0).is_err());
    assert!(follower_track(&odo, 0.0, &[(2.5, 0.05)], 0.0, 2.0).is_err());
    assert!(follower_track(&odo, 0.0, &[], 0.0, 2.0).is_err());
}
