use nurbs::bezier::BezierPiece;
use trajectory::{AdvanceModel, NonlinearAdvance};

use super::{apply_nonlinear_advance_pieces, shifted_monomial, state_at};
use crate::lowering::FitTol;

const TOL: FitTol = FitTol {
    pos_mm: 1e-3,
    accel_mm_s2: 50.0,
};

const KERNEL_W: f64 = 0.013;

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
    let out = apply_nonlinear_advance_pieces(&pieces, adv, TOL, KERNEL_W).unwrap();
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
        let out = apply_nonlinear_advance_pieces(&pieces, adv, TOL, KERNEL_W).unwrap();
        assert!(out.len() > 1, "transition must split");
        for i in 0..=400 {
            let t = 6.4e-3 * i as f64 / 400.0;
            let piece = &pieces[0];
            let (y, _, ya) = exact(&piece.coeffs, t, adv);
            let (op, _, oa) = eval_out(&out, t);
            assert!((op - y).abs() <= TOL.pos_mm, "pos err at {t}");
            let accel_budget = TOL.accel_mm_s2 + 1e-3 * ya.abs();
            assert!((oa - ya).abs() <= accel_budget, "accel err at {t}");
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
    let out = apply_nonlinear_advance_pieces(&pieces, adv, TOL, KERNEL_W).unwrap();
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
    let out = apply_nonlinear_advance_pieces(&pieces, adv, TOL, KERNEL_W).unwrap();
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
    let out =
        apply_nonlinear_advance_pieces(std::slice::from_ref(&input), adv, TOL, KERNEL_W).unwrap();
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
    let out = apply_nonlinear_advance_pieces(&pieces, adv, TOL, KERNEL_W).unwrap();
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

#[test]
fn jerk_unlimited_retract_ramp_composes_within_position_budget() {
    // The exact projected-E piece that crashed the Trident bench print
    // (KAMP purge retract, jerk-unlimited config): composed target
    // acceleration reaches -3.3e8 mm/s², where an absolute-only accel
    // budget bisects to the floor and errors despite picometer-level
    // position fit.
    let adv = NonlinearAdvance {
        model: AdvanceModel::Tanh,
        linear_advance: 0.0267,
        nonlinear_offset: 0.359,
        linearization_velocity: 5.99,
    };
    let h = 1.9069671630678187e-6;
    let pieces = [BezierPiece {
        u_start: 0.512,
        u_end: 0.512 + h,
        coeffs: vec![
            5.004980166378835,
            14.488897219473774,
            -24240.94554451031,
            -1874858876.6602907,
            -36085432241226.375,
            -2.2003448759006227e17,
        ],
    }];
    let out = apply_nonlinear_advance_pieces(&pieces, adv, TOL, KERNEL_W).unwrap();
    for i in 0..=100 {
        let tau = h * i as f64 / 100.0;
        let (y, _, _) = exact(&pieces[0].coeffs, tau, adv);
        let (op, _, _) = eval_out(&out, 0.512 + tau);
        assert!((op - y).abs() <= TOL.pos_mm, "pos err at {tau}");
    }
}

#[test]
fn smoothed_residual_of_jerk_unlimited_ramp_meets_post_kernel_budgets() {
    // The waiver rule claims the downstream kernel annihilates the
    // acceleration error it tolerates. Verify directly: compose a
    // retract-like jerk-unlimited ramp (C1 piecewise cubics, |a| up to
    // 5e4 mm/s², velocity crossing zero), take the residual against the
    // exact law, and convolve it with the smooth_triangle kernel using the
    // exact identities (T''*e)(t) = (2/w)²·[e(t+w/2) − 2e(t) + e(t−w/2)]
    // and (T'*e)(t) = (2/w)²·[∫ₜ^{t+w/2}e − ∫_{t−w/2}^t e].
    for model in [AdvanceModel::Tanh, AdvanceModel::Reciprocal] {
        let adv = fitted_adv(model);
        let h = 6.0e-5;
        let mut pieces = Vec::new();
        let (mut p, mut v) = (0.0_f64, 15.0_f64);
        // 300 pieces = 18 ms: a single 13 ms kernel window overlaps ~215
        // active-residual pieces, so any same-sign accumulation across
        // pieces would surface here. Acceleration alternates so velocity
        // oscillates through the steep low-|v| region instead of running
        // away.
        for k in 0..300 {
            let a = if v > 15.0 {
                -5.0e4
            } else if v < -20.0 {
                4.0e4
            } else if k % 2 == 0 {
                -5.0e4
            } else {
                4.0e4
            };
            let j = if k % 3 == 0 { -1.0e9 } else { 8.0e8 };
            let coeffs = vec![p, v, 0.5 * a, j / 6.0];
            let (p1, v1, _, _) = state_at(&coeffs, h);
            pieces.push(BezierPiece {
                u_start: k as f64 * h,
                u_end: (k + 1) as f64 * h,
                coeffs,
            });
            p = p1;
            v = v1;
        }
        let span = pieces.last().unwrap().u_end;
        let out = apply_nonlinear_advance_pieces(&pieces, adv, TOL, KERNEL_W).unwrap();

        let dt = 2.5e-7;
        let n = (span / dt).ceil() as usize;
        let residual: Vec<f64> = (0..=n)
            .map(|i| {
                let t = (i as f64 * dt).min(span);
                let src = pieces
                    .iter()
                    .find(|pc| t >= pc.u_start && t <= pc.u_end)
                    .unwrap();
                let (y, _, _) = exact(&src.coeffs, t - src.u_start, adv);
                let (op, _, _) = eval_out(&out, t);
                op - y
            })
            .collect();
        let e = |i: i64| -> f64 {
            if i < 0 || i as usize >= residual.len() {
                0.0
            } else {
                residual[i as usize]
            }
        };
        let mut prefix = vec![0.0_f64; residual.len() + 1];
        for (i, &r) in residual.iter().enumerate() {
            prefix[i + 1] = prefix[i] + r * dt;
        }
        let integral = |lo: i64, hi: i64| -> f64 {
            let clamp = |i: i64| i.clamp(0, residual.len() as i64) as usize;
            prefix[clamp(hi)] - prefix[clamp(lo)]
        };
        let half = (0.5 * KERNEL_W / dt).round() as i64;
        let g2 = (2.0 / KERNEL_W) * (2.0 / KERNEL_W);
        let lead = (KERNEL_W / dt).ceil() as i64;
        let (mut worst_pos, mut worst_vel, mut worst_acc) = (0.0_f64, 0.0_f64, 0.0_f64);
        for i in -lead..(n as i64 + lead) {
            worst_pos = worst_pos.max(e(i).abs());
            let vel = g2 * (integral(i, i + half) - integral(i - half, i));
            let acc = g2 * (e(i + half) - 2.0 * e(i) + e(i - half));
            worst_vel = worst_vel.max(vel.abs());
            worst_acc = worst_acc.max(acc.abs());
        }
        assert!(worst_pos <= TOL.pos_mm, "{model:?}: pos {worst_pos}");
        assert!(worst_vel <= 0.5, "{model:?}: smoothed vel {worst_vel}");
        assert!(
            worst_acc <= TOL.accel_mm_s2,
            "{model:?}: smoothed accel {worst_acc}"
        );
    }
}
