#![allow(
    clippy::cast_sign_loss,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::float_cmp
)]

use nurbs::chebyshev::monomial_tau_to_chebyshev;
use runtime::motion_core::arm_piece;
use runtime::piece_ring::PieceEntry;

const CUBIC_MONO: [f64; 4] = [1.0, 2.5, -3.0, 4.0];

fn cheb_entry(start_time: u64, duration: f32, mono: &[f64]) -> PieceEntry {
    let cheb = monomial_tau_to_chebyshev(mono, duration as f64);
    let mut coeffs = [0.0_f32; 8];
    for (dst, &src) in coeffs.iter_mut().zip(cheb.iter()) {
        *dst = src as f32;
    }
    PieceEntry {
        start_time,
        duration,
        coeff_count: cheb.len() as u8,
        coeffs,
        ..PieceEntry::zeroed()
    }
}

#[test]
fn accel_matches_velocity_finite_difference() {
    let entry = cheb_entry(5_000_000, 0.5, &CUBIC_MONO);
    let cps = 1.0e9_f32;
    let armed = arm_piece(&entry, cps);
    let start = entry.start_time;
    for &t_s in &[0.0_f32, 0.05, 0.2, 0.45] {
        let now = start + (t_s * cps) as u64;
        let h_cycles = 100_000_u64;
        let h_s = h_cycles as f32 / cps;
        let (_, v0) = armed.eval_pos_vel(now);
        let (_, v1) = armed.eval_pos_vel(now + h_cycles);
        let fd = (v1 - v0) / h_s;
        let a = armed.eval_accel(now);
        assert!(
            (a - fd).abs() <= 0.05 * fd.abs().max(1.0),
            "t={t_s}: accel {a} vs finite-diff {fd}"
        );
    }
}

#[test]
fn accel_has_no_quadratic_term() {
    let entry = cheb_entry(0, 0.5, &CUBIC_MONO);
    let cps = 1.0e9_f32;
    let armed = arm_piece(&entry, cps);
    let a0 = armed.eval_accel(0);
    let a1 = armed.eval_accel((0.1 * cps) as u64);
    let a2 = armed.eval_accel((0.2 * cps) as u64);
    assert!((a2 - a1 - (a1 - a0)).abs() < 1e-3, "{a0} {a1} {a2}");
}

#[test]
fn accel_clamps_before_piece_start() {
    let entry = cheb_entry(1000, 0.5, &CUBIC_MONO);
    let cps = 1.0e9_f32;
    let armed = arm_piece(&entry, cps);
    let at_start = armed.eval_accel(1000);
    let before_start = armed.eval_accel(500);
    assert_eq!(before_start, at_start);
    let analytic_accel_tau0 = 2.0 * CUBIC_MONO[2] as f32;
    assert!(
        (at_start - analytic_accel_tau0).abs() < 1e-2,
        "accel at piece start {at_start} vs analytic {analytic_accel_tau0}"
    );
}
