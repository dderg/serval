use super::*;
use nurbs::VectorNurbs;
use temporal::{
    BindingConstraint, BindingSummary, GridSample, GridScheme, SolveStatus, TopProfile,
};

fn uniform_profile(n: usize, total_length: f64, velocity: f64) -> TopProfile {
    let mut samples = Vec::with_capacity(n);
    let b = velocity * velocity;
    for i in 0..n {
        let s = total_length * (i as f64) / ((n - 1) as f64);
        samples.push(GridSample {
            s,
            v: velocity,
            a: 0.0,
            b,
            binding: BindingConstraint::None,
        });
    }
    let total_time = total_length / velocity;
    TopProfile {
        samples,
        status: SolveStatus::Solved,
        grid_scheme: GridScheme::UniformArclength,
        total_time,
        binding: BindingSummary::default(),
    }
}

#[test]
fn s_of_t_uniform_velocity_is_linear() {
    let profile = uniform_profile(11, 50.0, 500.0);
    let s_pieces = build_s_of_t_pieces(&profile, 0.0);

    assert_eq!(s_pieces.pieces.len(), 10);
    assert!(s_pieces.near_zero.iter().all(|nz| !nz));

    assert!(
        (s_pieces.total_duration - 0.1).abs() < 1e-12,
        "total_duration = {}",
        s_pieces.total_duration
    );

    for piece in &s_pieces.pieces {
        assert_eq!(piece.coeffs.len(), 3);
        assert!(
            piece.coeffs[2].abs() < 1e-12,
            "quadratic coeff should be ~0, got {}",
            piece.coeffs[2]
        );
    }
}

#[test]
fn s_of_t_endpoint_consistency() {
    let n = 11;
    let total_length = 50.0;
    let mut samples = Vec::with_capacity(n);
    for i in 0..n {
        let frac = i as f64 / (n - 1) as f64;
        let s = total_length * frac;
        let v = 100.0 * frac;
        samples.push(GridSample {
            s,
            v,
            a: 0.0,
            b: v * v,
            binding: BindingConstraint::None,
        });
    }
    let profile = TopProfile {
        samples,
        status: SolveStatus::Solved,
        grid_scheme: GridScheme::UniformArclength,
        total_time: 1.0,
        binding: BindingSummary::default(),
    };

    let s_pieces = build_s_of_t_pieces(&profile, 0.0);
    assert_eq!(s_pieces.pieces.len(), 10);

    for k in 0..s_pieces.pieces.len() {
        let piece = &s_pieces.pieces[k];
        let s_at_end = piece.evaluate(piece.u_end);
        let expected_s = profile.samples[k + 1].s;
        assert!(
            (s_at_end - expected_s).abs() < 1e-9,
            "piece {k}: s_at_end = {s_at_end}, expected = {expected_s}, diff = {}",
            (s_at_end - expected_s).abs()
        );
    }

    for k in 0..s_pieces.pieces.len() {
        let piece = &s_pieces.pieces[k];
        let s_at_start = piece.evaluate(piece.u_start);
        let expected_s = profile.samples[k].s;
        assert!(
            (s_at_start - expected_s).abs() < 1e-9,
            "piece {k}: s_at_start = {s_at_start}, expected = {expected_s}",
        );
    }
}

#[test]
fn s_of_t_near_zero_handling() {
    let profile = TopProfile {
        samples: vec![
            GridSample {
                s: 0.0,
                v: 0.001,
                a: 0.0,
                b: 1e-6,
                binding: BindingConstraint::None,
            },
            GridSample {
                s: 0.5,
                v: 0.005,
                a: 0.0,
                b: 2.5e-5,
                binding: BindingConstraint::None,
            },
            GridSample {
                s: 1.0,
                v: 0.002,
                a: 0.0,
                b: 4e-6,
                binding: BindingConstraint::None,
            },
        ],
        status: SolveStatus::Solved,
        grid_scheme: GridScheme::UniformArclength,
        total_time: 100.0,
        binding: BindingSummary::default(),
    };

    let s_pieces = build_s_of_t_pieces(&profile, 0.0);
    assert_eq!(s_pieces.pieces.len(), 2);
    assert!(s_pieces.near_zero[0]);
    assert!(s_pieces.near_zero[1]);

    for piece in &s_pieces.pieces {
        assert!(
            piece.coeffs[1].abs() < 1e-15,
            "near-zero piece should have v=0"
        );
        assert!(
            piece.coeffs[2].abs() < 1e-15,
            "near-zero piece should have a/2=0"
        );
    }
}

#[test]
fn s_of_t_global_offset() {
    let profile = uniform_profile(3, 10.0, 100.0);
    let offset = 5.0;
    let s_pieces = build_s_of_t_pieces(&profile, offset);

    #[allow(clippy::float_cmp)]
    {
        assert_eq!(s_pieces.t_start, offset);
        assert_eq!(s_pieces.pieces[0].u_start, offset);
    }
    assert!(
        (s_pieces.t_end - (offset + 10.0 / 100.0)).abs() < 1e-12,
        "t_end = {}",
        s_pieces.t_end
    );
}

#[test]
fn s_of_t_pieces_contiguous() {
    let profile = uniform_profile(6, 25.0, 200.0);
    let s_pieces = build_s_of_t_pieces(&profile, 1.0);

    for k in 0..s_pieces.pieces.len() - 1 {
        assert!(
            (s_pieces.pieces[k].u_end - s_pieces.pieces[k + 1].u_start).abs() < 1e-15,
            "pieces {} and {} are not contiguous",
            k,
            k + 1
        );
    }
}

#[test]
fn compose_straight_line_constant_velocity() {
    let curve = nurbs::VectorNurbs::try_new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![[0.0, 0.0, 0.0], [50.0, 0.0, 0.0]],
    )
    .unwrap();

    let table = nurbs::arc_length::build_arc_length_table_vector(&curve, 1e-6, 1024).unwrap();

    let profile = uniform_profile(11, table.s_max(), 500.0);
    let s_pieces = build_s_of_t_pieces(&profile, 0.0);

    let composed = compose_segment(&curve, &table.as_view(), &s_pieces, 1e-4).unwrap();

    assert_eq!(composed.len(), s_pieces.pieces.len());

    let first = &composed[0];
    let x_at_start = first[0].evaluate(first[0].u_start);
    assert!(
        x_at_start.abs() < 1e-6,
        "x(t=0) = {x_at_start}, expected ~0"
    );

    let last = &composed[composed.len() - 1];
    let x_at_end = last[0].evaluate(last[0].u_end);
    assert!(
        (x_at_end - 50.0).abs() < 0.1,
        "x(t_end) = {x_at_end}, expected ~50"
    );

    for pieces_k in &composed {
        let y_mid = pieces_k[1].evaluate((pieces_k[1].u_start + pieces_k[1].u_end) / 2.0);
        let z_mid = pieces_k[2].evaluate((pieces_k[2].u_start + pieces_k[2].u_end) / 2.0);
        assert!(y_mid.abs() < 1e-6, "y should be ~0, got {y_mid}");
        assert!(z_mid.abs() < 1e-6, "z should be ~0, got {z_mid}");
    }

    let mut prev_x = f64::NEG_INFINITY;
    for pieces_k in &composed {
        let x_start = pieces_k[0].evaluate(pieces_k[0].u_start);
        assert!(
            x_start >= prev_x - 1e-9,
            "x not monotone: prev={prev_x}, curr={x_start}"
        );
        prev_x = pieces_k[0].evaluate(pieces_k[0].u_end);
    }
}

#[test]
fn compose_diagonal_line() {
    let curve = nurbs::VectorNurbs::try_new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![[0.0, 0.0, 0.0], [30.0, 40.0, 0.0]],
    )
    .unwrap();

    let table = nurbs::arc_length::build_arc_length_table_vector(&curve, 1e-6, 1024).unwrap();
    assert!(
        (table.s_max() - 50.0_f64).abs() < 0.01,
        "arc length = {}, expected 50",
        table.s_max()
    );

    let profile = uniform_profile(6, table.s_max(), 250.0);
    let s_pieces = build_s_of_t_pieces(&profile, 0.0);
    let composed = compose_segment(&curve, &table.as_view(), &s_pieces, 1e-4).unwrap();

    let last = &composed[composed.len() - 1];
    let x_end = last[0].evaluate(last[0].u_end);
    let y_end = last[1].evaluate(last[1].u_end);
    let z_end = last[2].evaluate(last[2].u_end);

    assert!((x_end - 30.0).abs() < 0.5, "x_end = {x_end}, expected ~30");
    assert!((y_end - 40.0).abs() < 0.5, "y_end = {y_end}, expected ~40");
    assert!(z_end.abs() < 1e-6, "z_end = {z_end}, expected ~0");
}

fn arch() -> VectorNurbs<f64, 3> {
    VectorNurbs::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![
            [150.0, 150.0, 5.0],
            [150.0, 180.0, 5.0],
            [200.0, 180.0, 5.0],
            [200.0, 150.0, 5.0],
        ],
    )
    .unwrap()
}

#[test]
fn invert_s_to_u_is_accurate() {
    let curve = arch();
    let deriv = nurbs::eval::vector_derivative(&curve);
    let table = nurbs::arc_length::build_arc_length_table_vector(
        &curve, super::ARC_TABLE_TOL, super::ARC_TABLE_SAMPLES,
    )
    .unwrap();
    let tv = table.as_view();
    let reference = nurbs::arc_length::build_arc_length_table_vector(&curve, 1e-10, 16384).unwrap();
    let rv = reference.as_view();

    for i in 1..20 {
        let u_true = i as f64 / 20.0;
        let s = nurbs::arc_length::arc_length_from_param(&rv, u_true);
        let u_got = super::invert_s_to_u(&tv, &deriv, s, 0).expect("smooth curve must invert");
        assert!((u_got - u_true).abs() < 1e-7, "u err at u={u_true}: got {u_got}");
        let p_got = nurbs::eval::vector_eval(&curve, u_got);
        let p_true = nurbs::eval::vector_eval(&curve, u_true);
        let perr = ((p_got[0] - p_true[0]).powi(2)
            + (p_got[1] - p_true[1]).powi(2)
            + (p_got[2] - p_true[2]).powi(2))
        .sqrt();
        assert!(perr < 1e-6, "pos err {perr} mm at u={u_true}");
    }
}

#[test]
fn invert_s_to_u_rejects_cusp() {
    let curve = VectorNurbs::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [5.0, 0.0, 0.0]],
    )
    .unwrap();
    let deriv = nurbs::eval::vector_derivative(&curve);
    let table = nurbs::arc_length::build_arc_length_table_vector(
        &curve, super::ARC_TABLE_TOL, super::ARC_TABLE_SAMPLES,
    )
    .unwrap();
    let tv = table.as_view();
    let err = super::invert_s_to_u(&tv, &deriv, 1e-6, 3);
    assert!(matches!(err, Err(crate::ShapeError::ZeroTangent { index: 3, .. })));
}

#[test]
fn fit_position_of_t_tracks_exact_curve() {
    let curve = arch();
    let deriv = nurbs::eval::vector_derivative(&curve);
    let table = nurbs::arc_length::build_arc_length_table_vector(
        &curve, super::ARC_TABLE_TOL, super::ARC_TABLE_SAMPLES,
    )
    .unwrap();
    let tv = table.as_view();
    let s_max = tv.s_max();

    // Constant-speed s(t) = s_max * t over t in [0,1] (one piece).
    let s_of_t = nurbs::bezier::BezierPiece { u_start: 0.0, u_end: 1.0, coeffs: vec![0.0, s_max] };

    let pieces = super::fit_position_of_t(&curve, &deriv, &tv, &s_of_t, 0)
        .expect("smooth curve must fit");
    assert!(!pieces.is_empty());

    let mut max_err = 0.0_f64;
    for j in 0..=400 {
        let t = j as f64 / 400.0;
        let arr = pieces
            .iter()
            .find(|a| t >= a[0].u_start - 1e-12 && t <= a[0].u_end + 1e-12)
            .unwrap();
        let got = [arr[0].evaluate(t), arr[1].evaluate(t), arr[2].evaluate(t)];
        let s = s_of_t.evaluate(t);
        let u = super::invert_s_to_u(&tv, &deriv, s, 0).unwrap();
        let truth = nurbs::eval::vector_eval(&curve, u);
        let e = ((got[0] - truth[0]).powi(2)
            + (got[1] - truth[1]).powi(2)
            + (got[2] - truth[2]).powi(2))
        .sqrt();
        max_err = max_err.max(e);
    }
    assert!(max_err < super::POS_FIT_TOL_MM * 2.0, "max pos err {max_err} mm");

    let first = &pieces[0];
    let p0 = [first[0].evaluate(0.0), first[1].evaluate(0.0), first[2].evaluate(0.0)];
    let c0 = nurbs::eval::vector_eval(&curve, 0.0);
    assert!((p0[0] - c0[0]).abs() < 1e-9 && (p0[1] - c0[1]).abs() < 1e-9);
}
