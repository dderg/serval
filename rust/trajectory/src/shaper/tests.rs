use super::fixtures::{pad_segment_axis, FittedSegment};
use super::*;
use crate::kernel::build_smooth_zv_kernel;
use nurbs::bezier::{bezier_pieces_to_nurbs, extract_bezier_pieces, BezierPiece};

fn constant_segment(x: f64, y: f64, z: f64, t_start: f64, t_end: f64) -> FittedSegment {
    let make_axis = |val: f64| {
        bezier_pieces_to_nurbs(&[BezierPiece {
            u_start: t_start,
            u_end: t_end,
            coeffs: vec![val],
        }])
    };
    FittedSegment {
        axes: [make_axis(x), make_axis(y), make_axis(z)],
        t_start,
        t_end,
    }
}

fn linear_segment(x_start: f64, x_end: f64, t_start: f64, t_end: f64) -> FittedSegment {
    let dt = t_end - t_start;
    let slope = (x_end - x_start) / dt;
    let x_nurbs = bezier_pieces_to_nurbs(&[BezierPiece {
        u_start: t_start,
        u_end: t_end,
        coeffs: vec![x_start, slope],
    }]);
    let y_nurbs = bezier_pieces_to_nurbs(&[BezierPiece {
        u_start: t_start,
        u_end: t_end,
        coeffs: vec![0.0],
    }]);
    let z_nurbs = bezier_pieces_to_nurbs(&[BezierPiece {
        u_start: t_start,
        u_end: t_end,
        coeffs: vec![0.0],
    }]);
    FittedSegment {
        axes: [x_nurbs, y_nurbs, z_nurbs],
        t_start,
        t_end,
    }
}

#[test]
fn shaped_signal_constant_is_constant() {
    let freq = 150.0;
    let t_sm = 0.8025 / freq;
    let t_sm_half = t_sm / 2.0;
    let kernel = build_smooth_zv_kernel(t_sm);

    let x_val = 42.0;
    let fitted = vec![constant_segment(x_val, 0.0, 0.0, 0.0, 1.0)];

    let padded = pad_segment_axis(0, 0, &fitted, t_sm_half, 0.0, 1.0);
    let sig = ShapedSignal::new(&padded, &kernel, 0.0, 1.0);

    for &t in &[0.0, 0.25, 0.5, 0.75, 1.0] {
        let val = sig.eval(t);
        assert!(
            (val - x_val).abs() < 1e-4,
            "at t={t}: expected {x_val}, got {val}"
        );
    }
}

#[test]
fn shaped_signal_edge_boundary_approximately_correct() {
    let freq = 150.0;
    let t_sm = 0.8025 / freq;
    let t_sm_half = t_sm / 2.0;
    let kernel = build_smooth_zv_kernel(t_sm);

    let x_start = 5.0;
    let x_end = 15.0;
    let fitted = vec![linear_segment(x_start, x_end, 0.0, 1.0)];

    let padded = pad_segment_axis(0, 0, &fitted, t_sm_half, 0.0, 1.0);
    let pieces = extract_bezier_pieces(&padded);

    assert!(pieces[0].u_start < 0.0, "padding should extend before t=0");
    assert!(
        pieces.last().unwrap().u_end > 1.0,
        "padding should extend past t=1"
    );

    let sig = ShapedSignal::new(&padded, &kernel, 0.0, 1.0);

    let val_at_0 = sig.eval(0.0);
    assert!(
        (val_at_0 - x_start).abs() < 0.5,
        "at t=0: expected ~{x_start}, got {val_at_0}"
    );

    let val_at_1 = sig.eval(1.0);
    assert!(
        (val_at_1 - x_end).abs() < 0.5,
        "at t=1: expected ~{x_end}, got {val_at_1}"
    );

    let n_samples = 50;
    let mut prev = f64::NEG_INFINITY;
    for i in 0..=n_samples {
        let t = f64::from(i) / f64::from(n_samples);
        let val = sig.eval(t);
        assert!(
            val >= prev - 1e-10,
            "not monotone at t={t}: prev={prev}, val={val}"
        );
        prev = val;
    }
}
