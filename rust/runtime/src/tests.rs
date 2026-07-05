#![allow(
    clippy::panic,
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::integer_division
)]

use crate::fault_sink::FaultSink;
use crate::motion_core::{arm_piece, get_position_and_velocity};
use crate::piece_ring::{PieceEntry, RingDescriptor};

struct PanicFaultSink;
impl FaultSink for PanicFaultSink {
    fn piece_start_in_past(&self, axis_idx: usize, _deficit_us: u32) {
        panic!("unexpected PieceStartInPast fault on axis {axis_idx}");
    }
}

const CLOCK_FREQ: f32 = 520_000_000.0;
const TICK_CYCLES: u32 = 520_000_000_u32 / 40_000_u32;

#[test]
fn walker_empty_ring_returns_none() {
    let mut ring = RingDescriptor::new_unconfigured();
    let storage: Vec<PieceEntry> = Vec::new();
    let fault = PanicFaultSink;
    let mut armed = None;

    let res = get_position_and_velocity(
        &mut armed,
        &mut ring,
        &storage,
        TICK_CYCLES as u64 * 10,
        TICK_CYCLES,
        CLOCK_FREQ,
        0,
        &fault,
    );
    assert!(res.is_none(), "empty ring must return None");
}

#[test]
fn walker_at_t0_returns_c0_and_c1() {
    let duration_s = 0.1_f32;
    let bernstein = [0.5_f64, 1.0, 1.5, 2.0];
    let d = duration_s as f64;
    let mono_c0 = bernstein[0];
    let mono_c1 = 3.0 * (bernstein[1] - bernstein[0]) / d;
    let mono_c2 = 3.0 * (bernstein[2] - 2.0 * bernstein[1] + bernstein[0]) / (d * d);
    let mono_c3 =
        (bernstein[3] - 3.0 * bernstein[2] + 3.0 * bernstein[1] - bernstein[0]) / (d * d * d);
    let mono = [mono_c0, mono_c1, mono_c2, mono_c3];

    let cheb = nurbs::chebyshev::monomial_tau_to_chebyshev(&mono, d);
    let mut coeffs = [0.0_f32; 8];
    for (dst, &src) in coeffs.iter_mut().zip(cheb.iter()) {
        *dst = src as f32;
    }

    let start = TICK_CYCLES as u64 * 10;
    let entry = PieceEntry {
        start_time: start,
        duration: duration_s,
        motor_mask: 0,
        coeff_count: cheb.len() as u8,
        coeffs,
        ..PieceEntry::zeroed()
    };

    let mut storage = vec![entry; 4];
    let mut ring = RingDescriptor::new(0, 4);
    ring.push(&mut storage, entry).expect("push must succeed");

    let fault = PanicFaultSink;
    let mut armed = None;

    let res = get_position_and_velocity(
        &mut armed,
        &mut ring,
        &storage,
        start,
        TICK_CYCLES,
        CLOCK_FREQ,
        0,
        &fault,
    );
    assert!(res.is_some(), "piece at t=0 must return Some");
    let (p, v) = res.unwrap();

    let c0 = mono_c0 as f32;
    let c1 = mono_c1 as f32;

    assert!((p - c0).abs() < 1e-5, "P(0) must equal c0={c0}; got {p}");
    assert!(
        (v - c1).abs() < 1e-3,
        "V(0) must equal c1={c1} mm/s; got {v}"
    );

    let armed_piece = arm_piece(&entry, CLOCK_FREQ);
    let (p2, v2) = armed_piece.eval_pos_vel(start);
    assert!((p2 - p).abs() < 1e-6, "arm_piece must match walker result");
    assert!((v2 - v).abs() < 1e-6, "arm_piece must match walker result");
}
