use nurbs::bezier::BezierPiece;

use super::{ExtrudeOnlyPiece, without_extrude_only_gain};

fn eval(pieces: &[BezierPiece], t: f64) -> f64 {
    let piece = pieces
        .iter()
        .find(|piece| piece.u_start <= t && t <= piece.u_end)
        .expect("t inside the pieces");
    let tau = t - piece.u_start;
    piece.coeffs.iter().rev().fold(0.0, |sum, c| sum * tau + c)
}

#[test]
fn the_gain_of_an_extrude_only_stretch_is_taken_out_where_it_overlaps_the_input() {
    let input = vec![
        BezierPiece {
            u_start: 0.0,
            u_end: 1.0,
            coeffs: vec![1.0, 2.0],
        },
        BezierPiece {
            u_start: 1.0,
            u_end: 2.0,
            coeffs: vec![3.0, -1.0, 0.5],
        },
    ];
    let stretch = ExtrudeOnlyPiece {
        t_start: 0.5,
        t_end: 1.5,
        velocity: 4.0,
        accel: -2.0,
        jerk: 6.0,
    };
    let (k1, k2) = (0.1, 0.01);
    let out = without_extrude_only_gain(input.clone(), [&stretch], (k1, k2));
    assert_eq!(
        out.iter().map(|p| (p.u_start, p.u_end)).collect::<Vec<_>>(),
        vec![(0.0, 0.5), (0.5, 1.0), (1.0, 1.5), (1.5, 2.0)]
    );
    for k in 0..201 {
        let t = 2.0 * (f64::from(k) + 0.5) / 201.0;
        let gain = if (0.5..=1.5).contains(&t) {
            let d = t - stretch.t_start;
            let velocity = 4.0 - 2.0 * d + 3.0 * d * d;
            let accel = -2.0 + 6.0 * d;
            k1 * velocity + k2 * accel
        } else {
            0.0
        };
        let expected = eval(&input, t) - gain;
        assert!(
            (eval(&out, t) - expected).abs() < 1e-12,
            "t={t}: {} against {expected}",
            eval(&out, t)
        );
    }
}

#[test]
fn input_without_extrude_only_motion_is_kept_as_is() {
    let input = vec![BezierPiece {
        u_start: 0.0,
        u_end: 1.0,
        coeffs: vec![1.0, 2.0, 3.0],
    }];
    assert_eq!(
        without_extrude_only_gain(input.clone(), [], (0.5, 0.0)),
        input
    );
}
