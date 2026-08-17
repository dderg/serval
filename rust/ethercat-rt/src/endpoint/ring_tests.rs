//! Equivalence and cost of the setpoint-ring executor against the piece
//! executor it supersedes.
//!
//! Both executors are driven over the same trajectory and the same buzz on the
//! same DC grid, and the CSP command streams they write are compared cycle by
//! cycle. Position and velocity feedforward are bit-exact: the host samples the
//! very same Chebyshev pieces at the very same grid clocks, and its mm→counts
//! conversion is the same `CountMap` arithmetic, one rounding, relative to the
//! same epoch origin. The torque feedforward differs by at most one tenth of a
//! percent of rated torque — the drive's own `torque_offset` quantum — because
//! the host quantizes the model's output to the wire before the cycle-time
//! damper/pin terms are added, where the piece path truncated the full float
//! sum instead.

use std::sync::{Arc, Mutex};

use runtime::piece_ring::PieceEntry;

use super::cycle::{compute_motion_targets, compute_ring_targets};
use super::drive::DriveChain;
use super::{commands, EndpointCtx};
use crate::dynamics::DynamicsModel;
use crate::ffi::EcTelemetry;
use crate::setpoint::Executor;
use crate::setpoint_fill::{ChainFiller, LaneSpec};

const SLOTS: usize = 2;
const CYCLE_NS: u64 = 250_000;
const BASE_NS: u64 = 4_000_000_000;
const COUNTS_PER_MM: f64 = 3_276.8;
const CLAMP_TENTHS: i16 = 300;
/// Cycles the trajectory occupies: two 5 ms ease pieces at 250 µs.
const MOVE_CYCLES: u64 = 40;
/// Grid index the buzz starts at — a few held cycles after the trajectory, so
/// both executors run the drained-and-holding path before it.
const BUZZ_INDEX: u64 = 44;
const TOTAL_CYCLES: u64 = 140;

const COREXY_PROFILE: &str = r#"
version = 6
axes = ["a", "b"]
modes = ["x", "y"]
frame = [[0.5, 0.5], [0.5, -0.5]]
mass = [0.040, 0.080]
viscous = [0.004, 0.004]
coulomb = [1.0, 1.0]
fit_rms_residual = [0.5, 0.5]
"#;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
struct Written {
    target: i32,
    vel: i32,
    torque: i16,
}

/// A drive that lands exactly on its commanded target. The piece executor
/// anchors a buzz on `position_actual` while the ring anchors it on the last
/// commanded target, so a following error would offset the two streams by that
/// error alone; a lag-free drive keeps the comparison about the executors.
#[derive(Default)]
struct MirrorDrive {
    written: Arc<Mutex<Vec<Written>>>,
}

impl MirrorDrive {
    fn with_log(written: Arc<Mutex<Vec<Written>>>) -> Self {
        {
            let mut w = written.lock().expect("log");
            w.clear();
            w.resize(SLOTS, Written::default());
        }
        Self { written }
    }

    fn slot(&self, slot: usize) -> Written {
        self.written.lock().expect("log")[slot]
    }
}

impl DriveChain for MirrorDrive {
    fn cycle_time_ns(&self) -> u64 {
        0
    }
    fn cycle(&mut self) -> (i32, i64) {
        (0, 0)
    }
    fn enable_all(&mut self) -> i32 {
        0
    }
    fn disable_all(&mut self) {}
    fn shutdown(&mut self) {}
    fn set_target_position(&mut self, slot: usize, counts: i32) {
        self.written.lock().expect("log")[slot].target = counts;
    }
    fn set_velocity_offset(&mut self, slot: usize, counts_per_s: i32) {
        self.written.lock().expect("log")[slot].vel = counts_per_s;
    }
    fn set_torque_offset(&mut self, slot: usize, tenths_pct: i16) {
        self.written.lock().expect("log")[slot].torque = tenths_pct;
    }
    fn position_actual(&self, slot: usize) -> i32 {
        self.slot(slot).target
    }
    fn velocity_actual(&self, _slot: usize) -> i32 {
        0
    }
    fn torque_actual(&self, _slot: usize) -> i16 {
        0
    }
    fn error_code(&self, _slot: usize) -> u16 {
        0
    }
    fn telemetry(&self, slot: usize) -> EcTelemetry {
        let w = self.slot(slot);
        EcTelemetry {
            target_position: w.target,
            position_actual: w.target,
            velocity_offset: w.vel,
            torque_offset: w.torque,
            ..EcTelemetry::default()
        }
    }
    fn dump_al_state(&self) {}
}

/// Chebyshev coefficients of the symmetric ease through Bezier controls
/// `[b0, b0, b3, b3]` — zero velocity at both ends, so the trajectory starts
/// and stops at rest and neither executor sees a mid-flight cut.
fn ease(from: f32, to: f32, start_ns: u64, dur_s: f32) -> PieceEntry {
    let (b0, b3) = (from, to);
    let mut entry = PieceEntry {
        start_time: start_ns,
        duration: dur_s,
        coeff_count: 4,
        ..PieceEntry::zeroed()
    };
    entry.coeffs[0] = (5.0 * b0 + 3.0 * b0 + 3.0 * b3 + 5.0 * b3) / 16.0;
    entry.coeffs[1] = (-15.0 * b0 - 3.0 * b0 + 3.0 * b3 + 15.0 * b3) / 32.0;
    let t2_cancels_by_symmetry = 0.0;
    entry.coeffs[2] = t2_cancels_by_symmetry;
    entry.coeffs[3] = (-b0 + 3.0 * b0 - 3.0 * b3 + b3) / 32.0;
    entry
}

fn trajectory() -> Vec<PieceEntry> {
    let half = MOVE_CYCLES / 2 * CYCLE_NS;
    vec![
        ease(0.0, 2.0, BASE_NS, half as f32 / 1e9),
        ease(2.0, 5.0, BASE_NS + half, half as f32 / 1e9),
    ]
}

fn model() -> DynamicsModel {
    DynamicsModel::from_toml_str(COREXY_PROFILE).expect("corexy profile")
}

fn equivalence_ctx(name: &str, executor: Executor, log: Arc<Mutex<Vec<Written>>>) -> EndpointCtx {
    let mut ctx = super::tests::test_ctx_with_drive(name, MirrorDrive::with_log(log));
    ctx.executor = executor;
    ctx.velocity_ff = vec![true; SLOTS];
    ctx.torque_clamp_tenths = vec![CLAMP_TENTHS; SLOTS];
    ctx.dynamics = Some(model());
    ctx
}

fn lane_specs() -> Vec<LaneSpec> {
    (0..SLOTS)
        .map(|slot| LaneSpec {
            axis: slot as u8,
            cmd_counts_per_mm: COUNTS_PER_MM,
            ff_lead_ns: 0,
        })
        .collect()
}

fn buzz_args() -> (u8, u8, u32, u32, u32, u32, u32) {
    (0b11, 0b10, 40_000, 40_000, 50_000, 20, 4)
}

fn run_piece_executor(log: &Arc<Mutex<Vec<Written>>>) -> Vec<Vec<Written>> {
    let mut ctx = equivalence_ctx("eqv-piece", Executor::Piece, Arc::clone(log));
    for entry in trajectory() {
        for ring in &mut ctx.rings {
            ring.push_entry(entry).expect("test ring has room");
        }
    }
    let (slot_mask, sign_mask, f0, f1, amp, dur, ramp) = buzz_args();
    let mut stream = Vec::new();
    for index in 0..TOTAL_CYCLES {
        if index == BUZZ_INDEX {
            let mut base = [0i32; crate::buzz::MAX_BUZZ_SLOTS];
            for (slot, base) in base.iter_mut().enumerate().take(SLOTS) {
                *base = ctx.drive.position_actual(slot);
            }
            assert_eq!(
                ctx.buzz.arm(
                    SLOTS as u8,
                    slot_mask,
                    sign_mask,
                    f0,
                    f1,
                    amp,
                    dur,
                    ramp,
                    base
                ),
                0
            );
        }
        compute_motion_targets(&mut ctx, BASE_NS + index * CYCLE_NS);
        stream.push(log.lock().expect("log").clone());
    }
    stream
}

/// The buzz is armed the way klippy arms it: after the trajectory has drained,
/// with the endpoint's live grid pair in hand and the pump's lead ahead of it.
const BUZZ_ARM_LEAD_CYCLES: u64 = 4;

fn run_ring_executor(log: &Arc<Mutex<Vec<Written>>>) -> Vec<Vec<Written>> {
    let mut ctx = equivalence_ctx("eqv-ring", Executor::SetpointRing, Arc::clone(log));
    let mut filler = ChainFiller::new(&lane_specs(), Some(model()), CYCLE_NS, BUZZ_ARM_LEAD_CYCLES);
    filler
        .observe_grid(0, BASE_NS)
        .expect("the grid only advances");
    for axis in 0..SLOTS as u8 {
        filler.push_pieces(axis, &trajectory());
    }
    fill_all(&mut ctx, &mut filler);
    let (slot_mask, sign_mask, f0, f1, amp, dur, ramp) = buzz_args();
    let mut stream = Vec::new();
    for index in 0..TOTAL_CYCLES {
        filler
            .observe_grid(index, BASE_NS + index * CYCLE_NS)
            .expect("the grid only advances");
        if index == BUZZ_INDEX - BUZZ_ARM_LEAD_CYCLES {
            assert_eq!(
                filler.arm_buzz(slot_mask, sign_mask, f0, f1, amp, dur, ramp),
                0
            );
            fill_all(&mut ctx, &mut filler);
        }
        compute_ring_targets(&mut ctx, index);
        stream.push(log.lock().expect("log").clone());
    }
    for slot in 0..SLOTS {
        assert_eq!(
            ctx.sp_rings[slot].take_fault(),
            None,
            "slot {slot} latched a ring fault over a healthy stream"
        );
    }
    stream
}

fn fill_all(ctx: &mut EndpointCtx, filler: &mut ChainFiller) {
    while filler.wants_drain() {
        let runs = filler.drain().expect("host fill");
        if runs.is_empty() {
            break;
        }
        let (result, entries) = commands::fill_lane_runs(ctx, &runs);
        assert_eq!(result, 0, "endpoint rejected a run");
        assert!(entries > 0);
    }
}

#[test]
fn ring_playback_reproduces_the_piece_command_stream() {
    let log = Arc::new(Mutex::new(Vec::new()));
    let piece_stream = run_piece_executor(&log);
    let ring_stream = run_ring_executor(&log);
    assert_eq!(piece_stream.len(), ring_stream.len());
    let mut moved = 0;
    let mut worst_torque = 0i32;
    for (index, (want, got)) in piece_stream.iter().zip(&ring_stream).enumerate() {
        for slot in 0..SLOTS {
            assert_eq!(
                got[slot].target, want[slot].target,
                "slot {slot} target diverged at grid index {index}"
            );
            assert_eq!(
                got[slot].vel, want[slot].vel,
                "slot {slot} velocity feedforward diverged at grid index {index}"
            );
            let delta = i32::from(got[slot].torque) - i32::from(want[slot].torque);
            assert!(
                delta.abs() <= 1,
                "slot {slot} torque feedforward diverged by {delta} tenths at grid \
                 index {index} (piece {}, ring {})",
                want[slot].torque,
                got[slot].torque
            );
            worst_torque = worst_torque.max(delta.abs());
            if want[slot].vel != 0 {
                moved += 1;
            }
        }
    }
    assert!(
        moved > TOTAL_CYCLES as usize,
        "the comparison must cover moving cycles, not just holds"
    );
    assert!(
        worst_torque > 0,
        "the torque feedforward must be exercised, not identically zero"
    );
}

#[test]
fn ring_playback_costs_less_per_cycle_than_piece_evaluation() {
    let log = Arc::new(Mutex::new(Vec::new()));
    let mut piece_ctx = equivalence_ctx("cost-piece", Executor::Piece, Arc::clone(&log));
    for entry in trajectory() {
        for ring in &mut piece_ctx.rings {
            ring.push_entry(entry).expect("test ring has room");
        }
    }
    let ring_log = Arc::new(Mutex::new(Vec::new()));
    let mut ring_ctx = equivalence_ctx("cost-ring", Executor::SetpointRing, Arc::clone(&ring_log));
    let mut filler = ChainFiller::new(&lane_specs(), Some(model()), CYCLE_NS, 1);
    filler
        .observe_grid(0, BASE_NS)
        .expect("the grid only advances");
    for axis in 0..SLOTS as u8 {
        filler.push_pieces(axis, &trajectory());
    }
    fill_all(&mut ring_ctx, &mut filler);

    let mut piece_ns = Vec::new();
    let mut ring_ns = Vec::new();
    for index in 0..MOVE_CYCLES {
        let clock = BASE_NS + index * CYCLE_NS;
        let started = std::time::Instant::now();
        compute_motion_targets(&mut piece_ctx, clock);
        piece_ns.push(started.elapsed().as_nanos() as u64);
        let started = std::time::Instant::now();
        compute_ring_targets(&mut ring_ctx, index);
        ring_ns.push(started.elapsed().as_nanos() as u64);
    }
    piece_ns.sort_unstable();
    ring_ns.sort_unstable();
    let median = |v: &Vec<u64>| v[v.len() / 2];
    let (piece_median, ring_median) = (median(&piece_ns), median(&ring_ns));
    eprintln!(
        "cycle cost median: piece={piece_median} ns, ring={ring_median} ns \
         (per {SLOTS}-slot cycle, {MOVE_CYCLES} samples)"
    );
    assert!(
        ring_median <= piece_median,
        "the ring executor must not cost more per cycle than piece evaluation: \
         piece={piece_median} ns ring={ring_median} ns"
    );
}
