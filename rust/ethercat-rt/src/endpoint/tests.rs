//! Regression tests for the stream count-anchor lifecycle.
//!
//! The trident bench captures (ident_20260710_002707.scap) showed a silent
//! one-cycle `target_counts` step at every stroke boundary: re-creating the
//! `CountMap` after a mid-stream ring gap anchored the commanded frame at
//! `position_actual`, baking each drive's standing following error into the
//! command and letting paired drives drift apart. These tests drive
//! `compute_motion_targets` with a fake drive that tracks with a constant
//! following error and assert the commanded target is continuous across a
//! gap — and that explicit frame redefinitions (discard_motion) still
//! re-anchor.

use runtime::piece_ring::PieceEntry;

use super::cycle::compute_motion_targets;
use super::drive::DriveChain;
use super::{discard_motion, EndpointCtx};
use crate::buzz::BuzzOsc;
use crate::capture::Capture;
use crate::curves::AxisRing;
use crate::ffi::EcTelemetry;
use crate::mailbox::{MailboxWorker, WorkerScheduling};
use crate::sdo::SdoBus;
use crate::sensorless::SensorlessBank;
use crate::server::FrameServer;
use crate::stream_halt::StreamHalt;
use crate::torque::TorqueGate;

const NUM_SLAVES: usize = 2;
const COUNTS_PER_MM: f64 = 3276.8;
const FOLLOWING_ERROR: [i32; NUM_SLAVES] = [40, -25];
const CYCLE_NS: u64 = 250_000;

struct TrackingLagDrive {
    targets: Vec<i32>,
}

impl DriveChain for TrackingLagDrive {
    fn cycle_time_ns(&self) -> u64 {
        0
    }
    fn cycle(&mut self) -> (i32, i64) {
        (0, 0)
    }
    fn enable(&mut self, _slot: usize) -> i32 {
        0
    }
    fn disable(&mut self, _slot: usize) {}
    fn shutdown(&mut self) {}
    fn set_target_position(&mut self, slot: usize, counts: i32) {
        self.targets[slot] = counts;
    }
    fn set_velocity_offset(&mut self, _slot: usize, _counts_per_s: i32) {}
    fn set_torque_offset(&mut self, _slot: usize, _tenths_pct: i16) {}
    fn position_actual(&self, slot: usize) -> i32 {
        self.targets[slot] - FOLLOWING_ERROR[slot]
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
        EcTelemetry {
            target_position: self.targets[slot],
            position_actual: self.position_actual(slot),
            ..EcTelemetry::default()
        }
    }
    fn dump_al_state(&self) {}
}

struct NoSdo;

impl SdoBus for NoSdo {
    fn read(&mut self, _slot: u8, _index: u16, _subindex: u8) -> Result<(u8, [u8; 4]), i32> {
        Err(-1)
    }
    fn write(&mut self, _slot: u8, _index: u16, _subindex: u8, _bytes: &[u8]) -> Result<(), i32> {
        Err(-1)
    }
}

fn test_ctx(name: &str) -> EndpointCtx {
    let sock = std::env::temp_dir().join(format!("ec-rt-test-{}-{name}.sock", std::process::id()));
    let mut gate = TorqueGate::new();
    let _ = gate.on_set_torque(true, 0);
    gate.enable_finished(true);
    EndpointCtx {
        server: FrameServer::bind(sock.to_str().expect("utf8 socket path"))
            .expect("bind test socket"),
        drive: Box::new(TrackingLagDrive {
            targets: vec![0; NUM_SLAVES],
        }),
        num_slaves: NUM_SLAVES,
        counts_per_mm: vec![COUNTS_PER_MM; NUM_SLAVES],
        invert: vec![false; NUM_SLAVES],
        cmd_counts_per_mm: vec![COUNTS_PER_MM; NUM_SLAVES],
        rotation_distance: vec![40.0; NUM_SLAVES],
        slave_axes: vec![0, 1],
        velocity_ff: vec![false; NUM_SLAVES],
        torque_clamp_tenths: vec![0; NUM_SLAVES],
        ff_lead_ns: vec![0; NUM_SLAVES],
        jump_log_counts: vec![1638; NUM_SLAVES],
        cycle_ns: CYCLE_NS as i64,
        telemetry_period: u64::MAX,
        dynamics: None,
        run_limits: Vec::new(),
        rings: (0..NUM_SLAVES).map(AxisRing::with_slot).collect(),
        buzz: BuzzOsc::new(),
        cmaps: vec![None; NUM_SLAVES],
        last_counts: vec![None; NUM_SLAVES],
        report_anchor: vec![None; NUM_SLAVES],
        last_streamed_target: vec![None; NUM_SLAVES],
        last_sent_retired: 0,
        heartbeat_sent: false,
        gate,
        capture: Capture::new(),
        cycle_index: 0,
        mailbox: MailboxWorker::spawn(NoSdo, |_, _, _| 0, WorkerScheduling::Normal),
        pending_starts: Vec::new(),
        pending_stops: Vec::new(),
        capture_slots: Vec::new(),
        prdiv: 0,
        ff_saturation: 0,
        wkc_consecutive: 0,
        latched_drive_err: 0,
        sensorless: SensorlessBank::new(NUM_SLAVES),
        stream_halt: StreamHalt::default(),
        sync: None,
    }
}

fn piece(start_ns: u64, duration_s: f32, coeffs: &[f32]) -> PieceEntry {
    let mut entry = PieceEntry {
        start_time: start_ns,
        duration: duration_s,
        coeff_count: coeffs.len() as u8,
        ..PieceEntry::zeroed()
    };
    entry.coeffs[..coeffs.len()].copy_from_slice(coeffs);
    entry
}

fn push_all(ctx: &mut EndpointCtx, entry: PieceEntry) {
    for ring in &mut ctx.rings {
        ring.push_entry(entry).expect("test ring has room");
    }
}

fn run_cycles(ctx: &mut EndpointCtx, from_ns: u64, to_ns: u64) {
    let mut t = from_ns;
    while t <= to_ns {
        compute_motion_targets(ctx, t);
        t += CYCLE_NS;
    }
}

fn targets(ctx: &EndpointCtx) -> Vec<i32> {
    (0..NUM_SLAVES)
        .map(|s| ctx.drive.telemetry(s).target_position)
        .collect()
}

/// The bench weld: stroke to rest, ring runs dry (dwell + host latency),
/// then the continuation arrives at the same commanded position. The
/// commanded counts must not move — under the pre-fix per-gap re-anchor
/// they stepped by each drive's following error.
#[test]
fn target_counts_hold_across_a_mid_stream_ring_gap() {
    let mut ctx = test_ctx("gap");

    // Stroke: 0 mm → 5 mm over 10 ms starting at t=1 ms, then a contiguous
    // 10 ms hold at the endpoint — the shape of a stroke braking to rest.
    push_all(&mut ctx, piece(1_000_000, 0.01, &[2.5, 2.5]));
    push_all(&mut ctx, piece(11_000_000, 0.01, &[5.0]));
    run_cycles(&mut ctx, 1_000_000, 20_750_000);
    let at_rest = targets(&ctx);

    // Ring runs dry for ~1.2 s (the ident dwell + think time).
    run_cycles(&mut ctx, 21_000_000, 25_000_000);
    run_cycles(&mut ctx, 1_200_000_000, 1_201_000_000);
    for s in 0..NUM_SLAVES {
        assert!(
            ctx.cmaps[s].is_some(),
            "slot {s}: count anchor must survive a mid-stream gap"
        );
        assert!(
            ctx.last_counts[s].is_some(),
            "slot {s}: target-jump guard baseline must survive a mid-stream gap"
        );
    }

    // Continuation: hold at 5 mm, arriving after the gap.
    push_all(&mut ctx, piece(1_211_000_000, 0.01, &[5.0]));
    run_cycles(&mut ctx, 1_211_000_000, 1_212_000_000);

    let after_gap = targets(&ctx);
    assert_eq!(
        after_gap, at_rest,
        "commanded counts stepped across the gap: the count anchor was \
         re-created from position_actual, baking the following error \
         ({FOLLOWING_ERROR:?} counts) into the command"
    );
}

/// Stop/ResumeStream redefine the host frame: after discard_motion the next
/// stream must re-anchor at the drive's actual position (never yank), and
/// the target-jump guard baseline must reset with it.
#[test]
fn discard_motion_reanchors_the_next_stream() {
    let mut ctx = test_ctx("discard");

    push_all(&mut ctx, piece(1_000_000, 0.01, &[2.5, 2.5]));
    run_cycles(&mut ctx, 1_000_000, 11_000_000);
    let before = targets(&ctx);

    discard_motion(&mut ctx);
    for s in 0..NUM_SLAVES {
        assert!(ctx.cmaps[s].is_none(), "slot {s}: discard drops the anchor");
        assert!(
            ctx.last_counts[s].is_none(),
            "slot {s}: discard resets the jump-guard baseline"
        );
    }

    // New stream restarts the host frame at 0 mm; the drive must not be
    // yanked — the first target re-anchors at its actual position.
    push_all(&mut ctx, piece(20_000_000, 0.01, &[0.0]));
    run_cycles(&mut ctx, 20_000_000, 20_500_000);

    let after = targets(&ctx);
    for s in 0..NUM_SLAVES {
        assert_eq!(
            after[s],
            before[s] - FOLLOWING_ERROR[s],
            "slot {s}: post-discard stream anchors at position_actual"
        );
    }
}
