use nurbs::bezier::BezierPiece;
use trajectory::{AdvanceModel, NonlinearAdvance};

use super::{apply_nonlinear_advance_pieces, shifted_monomial, state_at};
use crate::lowering::FitTol;

const TOL: FitTol = FitTol {
    pos_mm: 1e-3,
    accel_mm_s2: 50.0,
};

fn fitted_adv(model: AdvanceModel) -> NonlinearAdvance {
    NonlinearAdvance {
        model,
        linear_advance: 0.0174,
        nonlinear_offset: 0.44,
        linearization_velocity: 3.73,
    }
}

fn exact(coeffs: &[f64], tau: f64, adv: NonlinearAdvance) -> (f64, f64, f64) {
    super::exact_output(coeffs, tau, adv)
}

fn eval_out(pieces: &[BezierPiece], t: f64) -> (f64, f64, f64) {
    let p = pieces
        .iter()
        .find(|p| t >= p.u_start && t <= p.u_end)
        .expect("time covered");
    let (pos, vel, acc, _) = state_at(&p.coeffs, t - p.u_start);
    (pos, vel, acc)
}

#[test]
fn constant_velocity_piece_is_exact() {
    let adv = fitted_adv(AdvanceModel::Tanh);
    let pieces = [BezierPiece {
        u_start: 1.0,
        u_end: 1.5,
        coeffs: vec![10.0, 7.0],
    }];
    let out = apply_nonlinear_advance_pieces(&pieces, adv, TOL).unwrap();
    assert_eq!(out.len(), 1);
    for i in 0..=10 {
        let tau = 0.5 * i as f64 / 10.0;
        let (y, yv, _) = exact(&pieces[0].coeffs, tau, adv);
        let (op, ov, oa) = eval_out(&out, 1.0 + tau);
        assert!((op - y).abs() < 1e-12, "pos at {tau}: {op} vs {y}");
        assert!((ov - yv).abs() < 1e-10);
        assert!(oa.abs() < 1e-8);
    }
}

#[test]
fn accelerating_cubic_meets_budgets_for_both_models() {
    for model in [AdvanceModel::Tanh, AdvanceModel::Reciprocal] {
        let adv = fitted_adv(model);
        // 0 → 16 mm/s in 6.4 ms at 2500 mm/s² with a jerk tail: the E-follower
        // transition that forced the fitter to split.
        let pieces = [BezierPiece {
            u_start: 0.0,
            u_end: 6.4e-3,
            coeffs: vec![2.0, 0.0, 1250.0, 40000.0],
        }];
        let out = apply_nonlinear_advance_pieces(&pieces, adv, TOL).unwrap();
        assert!(out.len() > 1, "transition must split");
        for i in 0..=400 {
            let t = 6.4e-3 * i as f64 / 400.0;
            let piece = &pieces[0];
            let (y, _, ya) = exact(&piece.coeffs, t, adv);
            let (op, _, oa) = eval_out(&out, t);
            assert!((op - y).abs() <= TOL.pos_mm, "pos err at {t}");
            assert!((oa - ya).abs() <= TOL.accel_mm_s2, "accel err at {t}");
        }
    }
}

#[test]
fn retraction_velocities_stay_within_budget() {
    let adv = fitted_adv(AdvanceModel::Tanh);
    // +5 → −35 mm/s: a retract pulse crossing zero.
    let pieces = [BezierPiece {
        u_start: 0.0,
        u_end: 4e-3,
        coeffs: vec![0.0, 5.0, -5000.0],
    }];
    let out = apply_nonlinear_advance_pieces(&pieces, adv, TOL).unwrap();
    for i in 0..=200 {
        let t = 4e-3 * i as f64 / 200.0;
        let (y, _, _) = exact(&pieces[0].coeffs, t, adv);
        let (op, _, _) = eval_out(&out, t);
        assert!((op - y).abs() <= TOL.pos_mm);
    }
}

#[test]
fn split_pieces_are_contiguous_and_weld_c2() {
    let adv = fitted_adv(AdvanceModel::Tanh);
    let pieces = [BezierPiece {
        u_start: 0.0,
        u_end: 6.4e-3,
        coeffs: vec![2.0, 0.0, 1250.0, 40000.0],
    }];
    let out = apply_nonlinear_advance_pieces(&pieces, adv, TOL).unwrap();
    for w in out.windows(2) {
        assert_eq!(w[0].u_end, w[1].u_start);
        let h = w[0].u_end - w[0].u_start;
        let (p0, v0, a0, _) = state_at(&w[0].coeffs, h);
        let (p1, v1, a1, _) = state_at(&w[1].coeffs, 0.0);
        assert!((p0 - p1).abs() < 1e-11, "pos weld: {p0} vs {p1}");
        assert!((v0 - v1).abs() < 1e-7, "vel weld: {v0} vs {v1}");
        assert!((a0 - a1).abs() < 1e-2, "accel weld: {a0} vs {a1}");
    }
}

#[test]
fn endpoints_of_every_piece_match_the_exact_law() {
    let adv = fitted_adv(AdvanceModel::Reciprocal);
    let input = BezierPiece {
        u_start: 0.5,
        u_end: 0.5 + 6.4e-3,
        coeffs: vec![2.0, 0.0, 1250.0, 40000.0],
    };
    let out = apply_nonlinear_advance_pieces(std::slice::from_ref(&input), adv, TOL).unwrap();
    for piece in &out {
        for tau_out in [0.0, piece.u_end - piece.u_start] {
            let tau_in = piece.u_start + tau_out - input.u_start;
            let shifted = shifted_monomial(&input.coeffs, tau_in);
            let (y, yv, ya) = exact(&shifted, 0.0, adv);
            let (op, ov, oa, _) = state_at(&piece.coeffs, tau_out);
            assert!((op - y).abs() < 1e-11);
            assert!((ov - yv).abs() < 1e-7);
            assert!((oa - ya).abs() < 1e-2);
        }
    }
}

#[test]
fn mixed_degree_input_yields_uniform_output_degree() {
    let adv = fitted_adv(AdvanceModel::Tanh);
    let pieces = [
        BezierPiece {
            u_start: 0.0,
            u_end: 0.1,
            coeffs: vec![0.0, 4.0],
        },
        BezierPiece {
            u_start: 0.1,
            u_end: 0.104,
            coeffs: vec![0.4, 4.0, 1000.0, 20000.0],
        },
    ];
    let out = apply_nonlinear_advance_pieces(&pieces, adv, TOL).unwrap();
    let len = out[0].coeffs.len();
    assert!(out.iter().all(|p| p.coeffs.len() == len));
    assert_eq!(len, 6);
}

#[test]
fn shifted_monomial_recenters_exactly() {
    let coeffs = [1.0, -2.0, 3.0, 0.5];
    let shifted = shifted_monomial(&coeffs, 0.7);
    for i in 0..=8 {
        let tau = -0.5 + i as f64 / 8.0;
        let (a, _, _, _) = state_at(&coeffs, 0.7 + tau);
        let (b, _, _, _) = state_at(&shifted, tau);
        assert!((a - b).abs() < 1e-12);
    }
}
