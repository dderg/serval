//! Regression tests for the stream count-anchor lifecycle.
//!
//! The trident bench captures (ident_20260710_002707.scap) showed a silent
//! one-cycle `target_counts` step at every stroke boundary: re-creating the
//! `CountMap` after a mid-stream ring gap anchored the commanded frame at
//! `position_actual`, baking each drive's standing following error into the
//! command and letting paired drives drift apart. These tests drive
//! `compute_motion_targets` with a fake drive that tracks with a constant
//! following error and assert the commanded-counts frame is continuous
//! across ring gaps AND across discard_motion (homing trips) — falling back
//! to position_actual only where the rotor genuinely moved uncommanded
//! (torque cycle, drive fault, sync coast).

use std::sync::{
    atomic::{AtomicUsize, Ordering},
    Arc,
};

use mcu_protocol::{
    messages::{SetTorque, StepperSuppress},
    Decode, Encode,
};
use runtime::piece_ring::PieceEntry;

use super::cycle::compute_motion_targets;
use super::drive::DriveChain;
use super::{discard_motion, EndpointCtx};
use crate::buzz::BuzzOsc;
use crate::capture::{Capture, CaptureDriveConfig};
use crate::curves::AxisRing;
use crate::damper::DiffDamperBank;
use crate::ffi::EcTelemetry;
use crate::live_tap::LiveTap;
use crate::mailbox::{MailboxWorker, WorkerScheduling};
use crate::scale::CountMap;
use crate::sdo::SdoBus;
use crate::sensorless::SensorlessBank;
use crate::server::FrameServer;
use crate::stream_halt::StreamHalt;
use crate::torque::{TorqueGate, TorqueState};
use crate::trim::DiffTrimBank;

const NUM_SLAVES: usize = 2;
const COUNTS_PER_MM: f64 = 3276.8;
const FOLLOWING_ERROR: [i32; NUM_SLAVES] = [40, -25];
const CYCLE_NS: u64 = 250_000;

struct TrackingLagDrive {
    targets: Vec<i32>,
    drift_counts_per_cycle: Vec<f64>,
    drifted_counts: Vec<f64>,
    torque_offsets: Vec<i16>,
    velocity_offsets: Vec<i32>,
    torques: Vec<i16>,
}

impl TrackingLagDrive {
    fn at_rest() -> Self {
        Self::with_drift(vec![0.0; NUM_SLAVES])
    }

    /// A rotor sliding uncommanded at a constant counts-per-cycle rate on top
    /// of its tracking lag — the raw-encoder motion the damper differentiates.
    fn with_drift(drift_counts_per_cycle: Vec<f64>) -> Self {
        Self {
            targets: vec![0; NUM_SLAVES],
            drift_counts_per_cycle,
            drifted_counts: vec![0.0; NUM_SLAVES],
            torque_offsets: vec![0; NUM_SLAVES],
            velocity_offsets: vec![0; NUM_SLAVES],
            torques: vec![0; NUM_SLAVES],
        }
    }

    /// A pair standing in a constant fight, for the trim tests.
    fn with_torques(torques: Vec<i16>) -> Self {
        Self {
            torques,
            ..Self::at_rest()
        }
    }
}

impl DriveChain for TrackingLagDrive {
    fn cycle_time_ns(&self) -> u64 {
        0
    }
    fn cycle(&mut self) -> (i32, i64) {
        for (pos, drift) in self
            .drifted_counts
            .iter_mut()
            .zip(&self.drift_counts_per_cycle)
        {
            *pos += drift;
        }
        (0, 0)
    }
    fn enable_all(&mut self) -> i32 {
        0
    }
    fn disable_all(&mut self) {}
    fn shutdown(&mut self) {}
    fn set_target_position(&mut self, slot: usize, counts: i32) {
        self.targets[slot] = counts;
    }
    fn set_velocity_offset(&mut self, slot: usize, counts_per_s: i32) {
        self.velocity_offsets[slot] = counts_per_s;
    }
    fn set_torque_offset(&mut self, slot: usize, tenths_pct: i16) {
        self.torque_offsets[slot] = tenths_pct;
    }
    fn position_actual(&self, slot: usize) -> i32 {
        self.targets[slot] - FOLLOWING_ERROR[slot] + self.drifted_counts[slot].round() as i32
    }
    fn velocity_actual(&self, _slot: usize) -> i32 {
        0
    }
    fn torque_actual(&self, slot: usize) -> i16 {
        self.torques[slot]
    }
    fn error_code(&self, _slot: usize) -> u16 {
        0
    }
    fn telemetry(&self, slot: usize) -> EcTelemetry {
        EcTelemetry {
            target_position: self.targets[slot],
            position_actual: self.position_actual(slot),
            torque_offset: self.torque_offsets[slot],
            velocity_offset: self.velocity_offsets[slot],
            ..EcTelemetry::default()
        }
    }
    fn dump_al_state(&self) {}
}

struct TransitionCounts {
    enable: AtomicUsize,
    disable: AtomicUsize,
}

struct RecordingDrive {
    transitions: Arc<TransitionCounts>,
}

impl RecordingDrive {
    fn new(transitions: Arc<TransitionCounts>) -> Self {
        Self { transitions }
    }
}

impl DriveChain for RecordingDrive {
    fn cycle_time_ns(&self) -> u64 {
        0
    }
    fn cycle(&mut self) -> (i32, i64) {
        (0, 0)
    }
    fn enable_all(&mut self) -> i32 {
        self.transitions.enable.fetch_add(1, Ordering::SeqCst);
        0
    }
    fn disable_all(&mut self) {
        self.transitions.disable.fetch_add(1, Ordering::SeqCst);
    }
    fn shutdown(&mut self) {}
    fn set_target_position(&mut self, _slot: usize, _counts: i32) {}
    fn set_velocity_offset(&mut self, _slot: usize, _counts_per_s: i32) {}
    fn set_torque_offset(&mut self, _slot: usize, _tenths_pct: i16) {}
    fn position_actual(&self, _slot: usize) -> i32 {
        0
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
    fn telemetry(&self, _slot: usize) -> EcTelemetry {
        EcTelemetry::default()
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
    test_ctx_with_drive(name, TrackingLagDrive::at_rest())
}

fn test_ctx_with_drive(name: &str, drive: impl DriveChain + 'static) -> EndpointCtx {
    let sock = std::env::temp_dir().join(format!("ec-rt-test-{}-{name}.sock", std::process::id()));
    let mut gate = TorqueGate::new();
    let _ = gate.on_set_torque(true, 0);
    gate.enable_finished(true);
    EndpointCtx {
        server: FrameServer::bind(sock.to_str().expect("utf8 socket path"))
            .expect("bind test socket"),
        drive: Box::new(drive),
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
        group_delay_ns: 0,
        telemetry_period: u64::MAX,
        dynamics: None,
        pin: super::cycle::PinState::default(),
        drive_dirs: vec![1.0; NUM_SLAVES],
        drive_scratch: super::cycle::DriveScratch::new(NUM_SLAVES),
        run_limits: Vec::new(),
        rings: (0..NUM_SLAVES).map(AxisRing::with_slot).collect(),
        buzz: BuzzOsc::new(),
        damper: DiffDamperBank::new(CYCLE_NS as i64),
        trim: DiffTrimBank::new(CYCLE_NS as i64),
        comp: crate::strain_comp::StrainCompBank::new(CYCLE_NS as i64),
        cmaps: vec![None; NUM_SLAVES],
        last_counts: vec![None; NUM_SLAVES],
        last_written_offset: vec![0; NUM_SLAVES],
        report_anchor: vec![None; NUM_SLAVES],
        last_streamed_target: vec![None; NUM_SLAVES],
        suppressed: vec![false; NUM_SLAVES],
        last_sent_retired: 0,
        heartbeat_sent: false,
        gate,
        capture: Capture::new(),
        live_tap: LiveTap::spawn(
            sock.with_extension("live").to_str().expect("utf8 tap path"),
            vec![CaptureDriveConfig {
                slot: 0,
                name: "slot0".into(),
                counts_per_mm: COUNTS_PER_MM,
                rotation_distance: 40.0,
                invert: false,
            }],
            CYCLE_NS as i64,
        )
        .expect("bind test tap socket"),
        reclaim: crate::reclaim::Reclaim::spawn(),
        tap_slots: (0..NUM_SLAVES as u8).collect(),
        cycle_index: 0,
        mailbox: MailboxWorker::spawn(NoSdo, |_, _, _| 0, WorkerScheduling::Normal),
        pending_starts: Vec::new(),
        pending_stops: Vec::new(),
        pending_seed: None,
        capture_slots: Vec::new(),
        prdiv: 0,
        ff_saturation: 0,
        wkc_consecutive: 0,
        latched_drive_err: 0,
        sensorless: SensorlessBank::new(NUM_SLAVES),
        stream_halt: StreamHalt::default(),
        late_tolerance_ns: None,
        timing_armed: true,
        baseline_reanchor_count: 0,
        late_frames: 0,
        late_max_ns: i64::MIN,
        skip_count_policed: 0,
        late_frames_total: 0,
        last_lateness_ns: 0,
        last_dispatch_ns: 0,
        last_pre_work_ns: 0,
        prev_exchange_ns: 0,
        last_wake_late_ns: 0,
        last_recv_ns: 0,
        last_process_ns: 0,
        last_send_ns: 0,
        wake_late_max_ns: i64::MIN,
        recv_max_ns: i64::MIN,
        process_max_ns: i64::MIN,
        send_max_ns: i64::MIN,
        prev_exchange_return: None,
        last_pre_cycle_ns: 0,
        last_post_cycle_ns: 0,
        last_inter_exchange_ns: 0,
        pre_cycle_max_ns: i64::MIN,
        post_cycle_max_ns: i64::MIN,
        inter_exchange_max_ns: i64::MIN,
        last_nivcsw: 0,
        last_fault_ns: 0,
        last_capture_ns: 0,
        last_wkc_ns: 0,
        last_heartbeat_ns: 0,
        last_telemetry_ns: 0,
        fault_max_ns: i64::MIN,
        capture_max_ns: i64::MIN,
        wkc_max_ns: i64::MIN,
        heartbeat_max_ns: i64::MIN,
        telemetry_max_ns: i64::MIN,
    }
}

#[test]
fn set_torque_enable_uses_one_chain_transition_for_two_slaves() {
    let transitions = Arc::new(TransitionCounts {
        enable: AtomicUsize::new(0),
        disable: AtomicUsize::new(0),
    });
    let mut ctx = test_ctx_with_drive(
        "torque-enable-lockstep",
        RecordingDrive::new(Arc::clone(&transitions)),
    );
    ctx.gate = TorqueGate::new();

    super::commands::handle_set_torque(
        &mut ctx,
        1,
        SetTorque {
            value: 1,
            execute_at_ns: 0,
        },
    );

    assert_eq!(ctx.gate.state(), TorqueState::Enabled);
    assert_eq!(transitions.enable.load(Ordering::SeqCst), 1);
    assert_eq!(transitions.disable.load(Ordering::SeqCst), 0);
}

#[test]
fn scheduled_torque_disable_uses_one_chain_transition_for_two_slaves() {
    let transitions = Arc::new(TransitionCounts {
        enable: AtomicUsize::new(0),
        disable: AtomicUsize::new(0),
    });
    let mut ctx = test_ctx_with_drive(
        "torque-disable-lockstep",
        RecordingDrive::new(Arc::clone(&transitions)),
    );

    super::commands::handle_set_torque(
        &mut ctx,
        1,
        SetTorque {
            value: 0,
            execute_at_ns: 5,
        },
    );
    assert_eq!(ctx.gate.state(), TorqueState::Enabled);

    super::cycle::apply_tick_action(&mut ctx, 5, true);

    assert_eq!(ctx.gate.state(), TorqueState::Parked);
    assert_eq!(transitions.enable.load(Ordering::SeqCst), 0);
    assert_eq!(transitions.disable.load(Ordering::SeqCst), 1);
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
        ctx.drive.cycle();
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

/// Stop/ResumeStream redefine the host mm frame (homing set_position), but
/// the commanded-counts frame must stay continuous: at an endstop trip both
/// drives of a belt pair hold unequal elastic following errors, and
/// re-anchoring each at its own strained actual would freeze that
/// differential in as permanent belt tension. The new mm frame grafts onto
/// the last commanded counts instead.
#[test]
fn discard_motion_keeps_commanded_counts_continuous() {
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

    // New stream restarts the host frame at 0 mm at the same physical spot:
    // the commanded counts must not move, or the pair's standing following
    // errors ({FOLLOWING_ERROR:?}) become a trapped differential.
    push_all(&mut ctx, piece(20_000_000, 0.01, &[0.0]));
    run_cycles(&mut ctx, 20_000_000, 20_500_000);

    let after = targets(&ctx);
    assert_eq!(
        after, before,
        "post-discard stream must anchor at the last commanded counts, \
         not position_actual"
    );
}

/// After a torque disable the rotor can move uncommanded (gravity, a hand,
/// belt relaxation), so the commanded frame is void: the first stream after
/// re-enable must anchor at the drive's actual position.
#[test]
fn torque_disable_voids_the_commanded_anchor() {
    let mut ctx = test_ctx("torque-off");

    push_all(&mut ctx, piece(1_000_000, 0.01, &[2.5, 2.5]));
    run_cycles(&mut ctx, 1_000_000, 11_000_000);
    let before = targets(&ctx);

    let _ = ctx.gate.on_set_torque(false, 15_000_000);
    super::cycle::apply_tick_action(&mut ctx, 15_000_000, true);
    for s in 0..NUM_SLAVES {
        assert!(
            ctx.last_streamed_target[s].is_none(),
            "slot {s}: torque disable must void the commanded anchor"
        );
        assert!(
            ctx.cmaps[s].is_none(),
            "slot {s}: torque disable drops the anchor"
        );
    }

    let _ = ctx.gate.on_set_torque(true, 0);
    ctx.gate.enable_finished(true);

    push_all(&mut ctx, piece(30_000_000, 0.01, &[0.0]));
    run_cycles(&mut ctx, 30_000_000, 30_500_000);

    let after = targets(&ctx);
    for s in 0..NUM_SLAVES {
        assert_eq!(
            after[s],
            before[s] - FOLLOWING_ERROR[s],
            "slot {s}: the first stream after a torque cycle anchors at \
             position_actual"
        );
    }
}

/// The full homing-trip shape: a pair is wound into the endstop with unequal
/// following errors, the trip discards motion, and the retract stream must
/// command both drives back through the SAME count deltas — releasing the
/// elastic differential instead of blessing it as the new zero.
#[test]
fn homing_trip_retract_releases_pair_wind_up() {
    let mut ctx = test_ctx("trip-retract");

    // Approach: 0 mm -> 5 mm, trip mid-stroke.
    push_all(&mut ctx, piece(1_000_000, 0.01, &[2.5, 2.5]));
    run_cycles(&mut ctx, 1_000_000, 6_000_000);
    let at_trip = targets(&ctx);
    let pair_offset_at_trip = i64::from(at_trip[0]) - i64::from(at_trip[1]);

    // Trip: Stop discards motion; the host rebases its mm frame (the homing
    // set_position) and streams the retract in the new frame.
    discard_motion(&mut ctx);
    push_all(&mut ctx, piece(10_000_000, 0.01, &[17.5, -2.5]));
    run_cycles(&mut ctx, 10_000_000, 20_000_000);

    let after_retract = targets(&ctx);
    let pair_offset_after = i64::from(after_retract[0]) - i64::from(after_retract[1]);
    assert_eq!(
        pair_offset_after, pair_offset_at_trip,
        "the pair's commanded offset must ride through the trip unchanged; \
         a change means one drive absorbed the other's following error"
    );
}

const CYCLES_PER_S: f64 = 1e9 / CYCLE_NS as f64;

/// Slot 1 mounted mirrored (negative cmd counts/mm): equal-and-opposite
/// HOST-frame drift means both encoders count up at the same rate, and the
/// antisymmetric mechanical damping torque lands as the same drive-frame
/// offset on both. Getting either frame conversion wrong flips a sign here.
#[test]
fn damper_writes_antisymmetric_torque_in_the_drive_frame() {
    let host_diff_mm_s = 10.0;
    let drift = 0.5 * host_diff_mm_s * COUNTS_PER_MM / CYCLES_PER_S;
    let mut ctx = test_ctx_with_drive("damper", TrackingLagDrive::with_drift(vec![drift, drift]));
    ctx.cmd_counts_per_mm[1] = -COUNTS_PER_MM;
    let gain_tenths_per_mm_s = 2.0;
    assert_eq!(ctx.damper.set(NUM_SLAVES, 0, 1, 2_000, 100, 300_000, 0), 0);

    run_cycles(&mut ctx, 0, 200 * CYCLE_NS);

    let expected_mech = -gain_tenths_per_mm_s * host_diff_mm_s;
    let offsets: Vec<i16> = (0..NUM_SLAVES)
        .map(|s| ctx.drive.telemetry(s).torque_offset)
        .collect();
    assert!(
        (f64::from(offsets[0]) - expected_mech).abs() <= 2.0,
        "expected ~{expected_mech}, got {offsets:?} (encoder quantization \
         allows a small ripple)"
    );
    assert!(
        (i32::from(offsets[1]) - i32::from(offsets[0])).abs() <= 1,
        "mirrored slot must get the mechanically opposite torque, which in \
         its inverted drive frame is the same number: {offsets:?}"
    );
}

/// The trim must land on the wire at commanded standstill: with the rings
/// dry, a standing fight (+10%, -10%) integrates into an antisymmetric
/// offset written onto the pair's held targets — the pair unwinds while the
/// target midpoint never moves (carriage-neutral).
#[test]
fn trim_zeroes_a_standing_fight_at_commanded_standstill() {
    let mut ctx = test_ctx_with_drive(
        "trim-standstill",
        TrackingLagDrive::with_torques(vec![100, -100]),
    );
    assert_eq!(ctx.trim.set(NUM_SLAVES, 0, 1, 200_000, 500, 25_000, 0), 0);
    run_cycles(&mut ctx, 0, 40_000_000);
    let t = targets(&ctx);
    assert_eq!(
        i64::from(t[0]),
        -i64::from(t[1]),
        "trim must be carriage-neutral: {t:?}"
    );
    assert!(
        t[0] < -100,
        "positive differential fight must pull slot 0 back: {t:?}"
    );
}

/// Slot 1 mounted mirrored (negative cmd counts/mm). A mechanical fight
/// (+10%, -10% in the host frame) reads (+100, +100) raw off the drives, and
/// the antisymmetric host-frame offset must land as the SAME drive-frame
/// count delta on both slots. Getting either torque- or position-frame
/// conversion wrong flips a sign here.
#[test]
fn trim_handles_a_mirrored_pair_in_both_frames() {
    let mut ctx = test_ctx_with_drive(
        "trim-mirror",
        TrackingLagDrive::with_torques(vec![100, 100]),
    );
    ctx.cmd_counts_per_mm[1] = -COUNTS_PER_MM;
    assert_eq!(ctx.trim.set(NUM_SLAVES, 0, 1, 200_000, 500, 25_000, 0), 0);
    run_cycles(&mut ctx, 0, 40_000_000);
    let t = targets(&ctx);
    assert_eq!(
        t[0], t[1],
        "mirrored slot gets the mechanically opposite offset, which in its \
         inverted drive frame is the same count delta"
    );
    assert!(t[0] < -100, "fight must pull the pair together: {t:?}");
}

/// While the pair streams, a differential torque is legitimate (commanded
/// feedforward, direction-dependent load) — the trim must stay frozen and
/// leave the streamed targets untouched.
#[test]
fn trim_freezes_while_the_pair_is_streaming() {
    let mut trimmed = test_ctx_with_drive(
        "trim-stream-on",
        TrackingLagDrive::with_torques(vec![100, -100]),
    );
    let mut plain = test_ctx_with_drive(
        "trim-stream-off",
        TrackingLagDrive::with_torques(vec![100, -100]),
    );
    assert_eq!(
        trimmed.trim.set(NUM_SLAVES, 0, 1, 200_000, 500, 25_000, 0),
        0
    );

    for ctx in [&mut trimmed, &mut plain] {
        push_all(ctx, piece(1_000_000, 0.05, &[2.5, 2.5]));
        run_cycles(ctx, 1_000_000, 41_000_000);
    }

    assert_eq!(
        targets(&trimmed),
        targets(&plain),
        "in-motion differential torque must not move the targets"
    );
}

/// The settle window keeps the trim blind right after motion stops, while
/// torque telemetry still carries the decel transient.
#[test]
fn trim_waits_out_the_settle_window_after_motion() {
    let mut ctx = test_ctx_with_drive(
        "trim-settle",
        TrackingLagDrive::with_torques(vec![100, -100]),
    );
    assert_eq!(ctx.trim.set(NUM_SLAVES, 0, 1, 200_000, 500, 25_000, 200), 0);
    push_all(&mut ctx, piece(1_000_000, 0.01, &[2.5, 2.5]));
    run_cycles(&mut ctx, 1_000_000, 12_000_000);
    let at_rest = targets(&ctx);
    // 100 ms of standstill: inside the 200 ms settle window, still blind.
    run_cycles(&mut ctx, 12_000_000 + CYCLE_NS, 112_000_000);
    assert_eq!(
        targets(&ctx),
        at_rest,
        "trim must stay blind through the settle window"
    );
    // Well past the window the fight starts unwinding.
    run_cycles(&mut ctx, 112_000_000 + CYCLE_NS, 400_000_000);
    let after = targets(&ctx);
    assert!(
        after[0] < at_rest[0],
        "trim must integrate once settled: {at_rest:?} -> {after:?}"
    );
}

#[test]
fn damper_stays_quiet_on_common_mode_velocity() {
    let drift = 25.0 * COUNTS_PER_MM / CYCLES_PER_S;
    let mut ctx = test_ctx_with_drive(
        "damper-cm",
        TrackingLagDrive::with_drift(vec![drift, drift]),
    );
    assert_eq!(ctx.damper.set(NUM_SLAVES, 0, 1, 2_000, 100, 300_000, 0), 0);

    run_cycles(&mut ctx, 0, 200 * CYCLE_NS);

    for s in 0..NUM_SLAVES {
        assert_eq!(ctx.drive.telemetry(s).torque_offset, 0);
    }
}

/// The stiffness probe's regime: a constant compensation grid uploaded at
/// standstill must reach the drives — held targets follow the (slew-limited)
/// offset even though nothing is streaming.
#[test]
fn strain_comp_moves_held_targets_at_standstill() {
    let mut ctx = test_ctx("comp-hold");
    push_all(&mut ctx, piece(1_000_000, 0.01, &[0.0]));
    run_cycles(&mut ctx, 1_000_000, 12_000_000);
    let held = targets(&ctx);
    assert_eq!(
        ctx.comp
            .set(NUM_SLAVES, 0, 1, 0, 1, 0, 1, 1, 0.0, 0.0, 1.0, 1.0, &[100]),
        0
    );
    run_cycles(&mut ctx, 12_250_000, 200_000_000);
    let now = targets(&ctx);
    let expect = 0.1 * COUNTS_PER_MM;
    assert!(
        (f64::from(now[0] - held[0]) - expect).abs() < 2.0,
        "slot 0 held target must follow +100 um, moved {}",
        now[0] - held[0]
    );
    assert!(
        (f64::from(now[1] - held[1]) + expect).abs() < 2.0,
        "slot 1 held target must follow -100 um, moved {}",
        now[1] - held[1]
    );
}

/// The bench regression: homing ends with Stop/ResumeStream, which clear
/// last_counts while the drive keeps holding its last target — the probe
/// must still reach the drives. The held base comes from the output image.
#[test]
fn strain_comp_reaches_held_targets_after_a_stop_discard() {
    let mut ctx = test_ctx("comp-stop");
    push_all(&mut ctx, piece(1_000_000, 0.01, &[0.0]));
    run_cycles(&mut ctx, 1_000_000, 12_000_000);
    super::discard_motion(&mut ctx);
    assert!(ctx.last_counts.iter().all(Option::is_none));
    let held = targets(&ctx);
    assert_eq!(
        ctx.comp
            .set(NUM_SLAVES, 0, 1, 0, 1, 0, 1, 1, 0.0, 0.0, 1.0, 1.0, &[100]),
        0
    );
    run_cycles(&mut ctx, 12_250_000, 200_000_000);
    let now = targets(&ctx);
    let expect = 0.1 * COUNTS_PER_MM;
    assert!(
        (f64::from(now[0] - held[0]) - expect).abs() < 2.0,
        "slot 0 must follow +100 um after a discard, moved {}",
        now[0] - held[0]
    );
    assert!((f64::from(now[1] - held[1]) + expect).abs() < 2.0);
}

/// Fresh session, torque enabled, nothing ever streamed: the probe still
/// works because enable seeded the output-image targets.
#[test]
fn strain_comp_reaches_targets_that_never_streamed() {
    let mut ctx = test_ctx("comp-fresh");
    let held = targets(&ctx);
    assert_eq!(
        ctx.comp
            .set(NUM_SLAVES, 0, 1, 0, 1, 0, 1, 1, 0.0, 0.0, 1.0, 1.0, &[100]),
        0
    );
    run_cycles(&mut ctx, 1_000_000, 200_000_000);
    let now = targets(&ctx);
    let expect = 0.1 * COUNTS_PER_MM;
    assert!(
        (f64::from(now[0] - held[0]) - expect).abs() < 2.0,
        "slot 0 must follow +100 um with no stream history, moved {}",
        now[0] - held[0]
    );
    assert!((f64::from(now[1] - held[1]) + expect).abs() < 2.0);
}

/// The probe's cleanup: clearing after the last (nonzero) step must ramp
/// the held targets back to their base instead of leaving the pair parked
/// in a standing fight.
#[test]
fn strain_comp_clear_returns_held_targets_to_base() {
    let mut ctx = test_ctx("comp-clear");
    let held = targets(&ctx);
    assert_eq!(
        ctx.comp
            .set(NUM_SLAVES, 0, 1, 0, 1, 0, 1, 1, 0.0, 0.0, 1.0, 1.0, &[-100]),
        0
    );
    run_cycles(&mut ctx, 1_000_000, 200_000_000);
    assert_ne!(targets(&ctx), held, "probe offset must be applied first");
    assert_eq!(
        ctx.comp
            .set(NUM_SLAVES, 0, 1, 0, 1, 0, 0, 0, 0.0, 0.0, 0.0, 0.0, &[]),
        0
    );
    run_cycles(&mut ctx, 200_250_000, 400_000_000);
    assert_eq!(
        targets(&ctx),
        held,
        "clear must unwind the offset completely"
    );
    assert!(!ctx.comp.active());
}

const BUZZ_DYNAMICS: &str = r#"
version = 6
axes = ["a", "b"]
modes = ["a", "b"]
frame = [[1.0, 0.0], [0.0, 1.0]]
mass = [0.00001, 0.00001]
viscous = [0.01, 0.01]
coulomb = [50.0, 50.0]
fit_rms_residual = [0.1, 0.1]
"#;

/// A buzz flips sign(v) every half period; if the Coulomb term stayed in the
/// feedforward it would land as a +/-50 tenths-pct square wave at the buzz
/// frequency, dwarfing the micrometre-scale excitation under test.
#[test]
fn buzzed_slot_ff_carries_no_coulomb_square_wave() {
    let mut ctx = test_ctx("buzz-coulomb");
    ctx.dynamics =
        Some(crate::dynamics::DynamicsModel::from_toml_str(BUZZ_DYNAMICS).expect("valid profile"));
    ctx.torque_clamp_tenths = vec![300; NUM_SLAVES];
    let rc = ctx.buzz.arm(
        NUM_SLAVES as u8,
        0b01,
        0,
        60_000,
        60_000,
        100_000,
        500,
        20,
        [0; crate::buzz::MAX_BUZZ_SLOTS],
    );
    assert_eq!(rc, 0);

    let mut max_abs_offset: i16 = 0;
    let mut max_abs_target: i32 = 0;
    for cycle in 0..1200u64 {
        compute_motion_targets(&mut ctx, cycle * CYCLE_NS);
        ctx.drive.cycle();
        let tel = ctx.drive.telemetry(0);
        max_abs_offset = max_abs_offset.max(tel.torque_offset.abs());
        max_abs_target = max_abs_target.max(tel.target_position.abs());
    }
    assert!(
        max_abs_target > 100,
        "buzz never moved the target: {max_abs_target}"
    );
    assert!(
        max_abs_offset < 5,
        "coulomb square wave leaked into the buzz feedforward: {max_abs_offset}"
    );
}

#[test]
fn streamed_motion_keeps_coulomb_in_the_ff() {
    let mut ctx = test_ctx("stream-coulomb");
    ctx.dynamics =
        Some(crate::dynamics::DynamicsModel::from_toml_str(BUZZ_DYNAMICS).expect("valid profile"));
    ctx.torque_clamp_tenths = vec![300; NUM_SLAVES];
    push_all(&mut ctx, piece(1_000_000, 0.05, &[0.25, 0.25]));
    run_cycles(&mut ctx, 1_000_000, 40_000_000);
    let offset = ctx.drive.telemetry(0).torque_offset;
    assert!(
        (49..=51).contains(&offset),
        "constant-velocity stroke must keep coulomb + viscous FF: {offset}"
    );
}

fn dynamics_msg(mass0: f32) -> mcu_protocol::messages::SetDynamicsModel {
    mcu_protocol::messages::SetDynamicsModel {
        slots_count: NUM_SLAVES as u8,
        modes_count: 2,
        frame: vec![1.0, 0.0, 0.0, 1.0],
        mass: vec![mass0, 0.030],
        viscous: vec![0.004, 0.004],
        coulomb: vec![1.0, 1.0],
        compliance: vec![0.0, 0.0],
        pin_mass: vec![0.0, 0.0],
        pin_zeta: vec![0.0, 0.0],
        pin_lead_us: 0.0,
        pairs: vec![],
    }
}

#[test]
fn set_dynamics_model_installs_wire_pair_and_evaluates_it() {
    let mut ctx = test_ctx("dyn-pair");
    let mut msg = dynamics_msg(0.030);
    msg.modes_count = 1;
    msg.frame = vec![0.5, 0.5];
    msg.mass = vec![0.030];
    msg.viscous = vec![0.0];
    msg.coulomb = vec![0.0];
    msg.compliance = vec![0.0];
    msg.pin_mass = vec![0.0];
    msg.pin_zeta = vec![0.0];
    msg.pairs = vec![mcu_protocol::messages::DynamicsPair {
        first: 0,
        second: 1,
        direction_split: 0.2,
    }];
    let msg = mcu_protocol::messages::SetDynamicsModel::decode(&msg.encoded_to_vec()).unwrap();
    super::commands::handle_set_dynamics_model(&mut ctx, 1, msg);
    let model = ctx.dynamics.as_ref().expect("model installed");
    let acc = [1000.0, 1000.0];
    let vel = [0.0, 0.0];
    let first = model.torque_ff(0, &acc, &vel);
    let second = model.torque_ff(1, &acc, &vel);
    assert!((first - 18.0).abs() < 1e-6, "first torque {first}");
    assert!((second - 12.0).abs() < 1e-6, "second torque {second}");
}

#[test]
fn set_dynamics_model_rejects_zero_pair_first_column() {
    let mut ctx = test_ctx("dyn-zero-pair");
    let mut msg = dynamics_msg(0.030);
    msg.modes_count = 1;
    msg.frame = vec![0.0, 0.0];
    msg.mass = vec![0.030];
    msg.viscous = vec![0.0];
    msg.coulomb = vec![0.0];
    msg.compliance = vec![0.0];
    msg.pin_mass = vec![0.0];
    msg.pin_zeta = vec![0.0];
    msg.pairs = vec![mcu_protocol::messages::DynamicsPair {
        first: 0,
        second: 1,
        direction_split: 0.1,
    }];
    super::commands::handle_set_dynamics_model(&mut ctx, 1, msg);
    assert!(ctx.dynamics.is_none());
}

#[test]
fn set_dynamics_model_installs_model_when_none_was_loaded() {
    let mut ctx = test_ctx("dyn-install");
    assert!(ctx.dynamics.is_none());
    super::commands::handle_set_dynamics_model(&mut ctx, 1, dynamics_msg(0.030));
    let model = ctx.dynamics.as_ref().expect("model installed");
    assert_eq!(model.n_slots, NUM_SLAVES);
    let tau = model.torque_ff(0, &[1000.0, 0.0], &[100.0, 0.0]);
    let expect = 0.030 * 1000.0 + 0.004 * 100.0 + 1.0;
    assert!((tau - expect).abs() < 1e-3, "{tau} vs {expect}");
}

#[test]
fn set_dynamics_model_replaces_existing_model() {
    let mut ctx = test_ctx("dyn-replace");
    super::commands::handle_set_dynamics_model(&mut ctx, 1, dynamics_msg(0.030));
    super::commands::handle_set_dynamics_model(&mut ctx, 2, dynamics_msg(0.045));
    let model = ctx.dynamics.as_ref().expect("model installed");
    let tau = model.torque_ff(0, &[1000.0, 0.0], &[0.0, 0.0]);
    assert!((tau - 45.0).abs() < 1e-3, "{tau}");
}

#[test]
fn set_dynamics_model_wrong_axes_count_keeps_previous_model() {
    let mut ctx = test_ctx("dyn-baddim");
    super::commands::handle_set_dynamics_model(&mut ctx, 1, dynamics_msg(0.030));
    let mut bad = dynamics_msg(0.045);
    bad.slots_count = 3;
    super::commands::handle_set_dynamics_model(&mut ctx, 2, bad);
    let model = ctx.dynamics.as_ref().expect("previous model kept");
    let tau = model.torque_ff(0, &[1000.0, 0.0], &[0.0, 0.0]);
    assert!((tau - 30.0).abs() < 1e-3, "{tau}");
}

#[test]
fn set_dynamics_model_rank_deficient_frame_keeps_previous_model() {
    let mut ctx = test_ctx("dyn-notpd");
    super::commands::handle_set_dynamics_model(&mut ctx, 1, dynamics_msg(0.030));
    let mut bad = dynamics_msg(0.045);
    bad.frame = vec![1.0, 0.0, 1.0, 0.0];
    super::commands::handle_set_dynamics_model(&mut ctx, 2, bad);
    let model = ctx.dynamics.as_ref().expect("previous model kept");
    let tau = model.torque_ff(0, &[1000.0, 0.0], &[0.0, 0.0]);
    assert!((tau - 30.0).abs() < 1e-3, "{tau}");
}

#[test]
fn set_dynamics_model_mass_len_mismatch_keeps_no_model() {
    let mut ctx = test_ctx("dyn-masslen");
    let mut bad = dynamics_msg(0.030);
    bad.mass.pop();
    super::commands::handle_set_dynamics_model(&mut ctx, 1, bad);
    assert!(ctx.dynamics.is_none());
}

#[test]
fn seed_defers_while_ring_drains_and_completes_when_empty() {
    let mut ctx = test_ctx("seed-defer");
    push_all(&mut ctx, piece(1_000_000, 0.01, &[2.5, 2.5]));
    super::commands::handle_seed_servo_home(&mut ctx, 7, 0, 65536);
    assert!(ctx.pending_seed.is_some());
    assert!(ctx.report_anchor[0].is_none());
    super::commands::drain_pending_seed(&mut ctx);
    assert!(
        ctx.pending_seed.is_some(),
        "ring still occupied — must wait"
    );
    for ring in &mut ctx.rings {
        ring.reset();
    }
    super::commands::drain_pending_seed(&mut ctx);
    assert!(ctx.pending_seed.is_none());
    let (_counts, anchor_mm) = ctx.report_anchor[0].expect("seed completed");
    assert_eq!(anchor_mm, 1.0);
}

#[test]
fn seed_fails_when_the_ring_never_drains() {
    let mut ctx = test_ctx("seed-timeout");
    push_all(&mut ctx, piece(1_000_000, 0.01, &[2.5, 2.5]));
    super::commands::handle_seed_servo_home(&mut ctx, 7, 0, 65536);
    let deadline = ctx.pending_seed.as_ref().expect("deferred").deadline_cycle;
    ctx.cycle_index = deadline;
    super::commands::drain_pending_seed(&mut ctx);
    assert!(ctx.pending_seed.is_none());
    assert!(ctx.report_anchor[0].is_none());
}

#[test]
fn seed_completes_immediately_on_an_empty_ring() {
    let mut ctx = test_ctx("seed-empty");
    super::commands::handle_seed_servo_home(&mut ctx, 7, 0, 131072);
    assert!(ctx.pending_seed.is_none());
    let (_counts, anchor_mm) = ctx.report_anchor[0].expect("seed completed");
    assert_eq!(anchor_mm, 2.0);
}

fn connect_test_client(ctx: &mut EndpointCtx, name: &str) -> std::os::unix::net::UnixStream {
    let sock = std::env::temp_dir().join(format!("ec-rt-test-{}-{name}.sock", std::process::id()));
    let client = std::os::unix::net::UnixStream::connect(&sock).expect("connect test client");
    client
        .set_read_timeout(Some(std::time::Duration::from_secs(2)))
        .expect("set read timeout");
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(2);
    while !ctx.server.client_connected() {
        ctx.server.pump();
        assert!(
            std::time::Instant::now() < deadline,
            "server never accepted the test client"
        );
        std::thread::sleep(std::time::Duration::from_millis(2));
    }
    client
}

fn read_set_ff_lead_response(
    client: &mut std::os::unix::net::UnixStream,
) -> mcu_protocol::messages::SetFfLeadResponse {
    use std::io::Read;
    let mut buf = [0u8; 256];
    let n = client.read(&mut buf).expect("read response");
    let (chan, payload) = mcu_transport::frame::decode_frame(&buf[..n]).expect("decode frame");
    assert_eq!(chan, mcu_transport::frame::CHANNEL_CONTROL);
    let (hdr, body) =
        mcu_transport::wire_helpers::decode_message_header(payload).expect("decode header");
    assert_eq!(
        mcu_protocol::messages::MessageKind::from_u16(hdr.kind_raw),
        Some(mcu_protocol::messages::MessageKind::SetFfLeadResponse)
    );
    mcu_protocol::messages::SetFfLeadResponse::decode(body).expect("decode response")
}

#[test]
fn handle_set_ff_lead_updates_slot_and_responds_ok() {
    let name = "ff-lead-ok";
    let mut ctx = test_ctx(name);
    let mut client = connect_test_client(&mut ctx, name);
    let msg = mcu_protocol::messages::SetFfLead {
        slot: 0,
        lead_ns: 12_345,
    };
    super::commands::handle_set_ff_lead(&mut ctx, 9, msg);
    assert_eq!(ctx.ff_lead_ns[0], 12_345);
    assert_eq!(ctx.ff_lead_ns[1], 0);
    assert_eq!(read_set_ff_lead_response(&mut client).result, 0);
}

#[test]
fn handle_set_ff_lead_invalid_slot_leaves_vector_untouched() {
    let name = "ff-lead-bad-slot";
    let mut ctx = test_ctx(name);
    let mut client = connect_test_client(&mut ctx, name);
    let msg = mcu_protocol::messages::SetFfLead {
        slot: NUM_SLAVES as u8,
        lead_ns: 999,
    };
    super::commands::handle_set_ff_lead(&mut ctx, 3, msg);
    assert_eq!(ctx.ff_lead_ns, vec![0; NUM_SLAVES]);
    assert_eq!(read_set_ff_lead_response(&mut client).result, -309);
}

#[test]
fn late_frame_is_counted_but_not_faulted_without_tolerance() {
    let mut ctx = test_ctx("late-no-tol");
    super::cycle::police_frame_timing(&mut ctx, 50_000);
    assert_ne!(ctx.gate.state(), TorqueState::Faulted);
    assert_eq!(ctx.late_frames, 1);
    assert_eq!(ctx.late_max_ns, 50_000);
}

#[test]
fn late_frame_beyond_tolerance_latches_a_fault() {
    let mut ctx = test_ctx("late-fault");
    ctx.late_tolerance_ns = Some(0);
    super::cycle::police_frame_timing(&mut ctx, 50_000);
    assert_eq!(ctx.gate.state(), TorqueState::Faulted);
    assert_eq!(ctx.latched_drive_err, super::cycle::FRAME_LATE_FAULT_CODE);
}

#[test]
fn late_frame_within_tolerance_does_not_fault() {
    let mut ctx = test_ctx("late-within");
    ctx.late_tolerance_ns = Some(100_000);
    super::cycle::police_frame_timing(&mut ctx, 50_000);
    assert_ne!(ctx.gate.state(), TorqueState::Faulted);
}

#[test]
fn first_cycle_arms_and_absorbs_the_bringup_catchup_skip() {
    let mut ctx = test_ctx("arming");
    ctx.timing_armed = false;
    ctx.late_tolerance_ns = Some(0);
    ctx.baseline_reanchor_count = 3;
    super::cycle::police_frame_timing(&mut ctx, 75_000);
    assert_ne!(ctx.gate.state(), TorqueState::Faulted);
    assert_eq!(ctx.late_frames, 0);
    assert_eq!(ctx.baseline_reanchor_count, 0);
    super::cycle::police_frame_timing(&mut ctx, 75_000);
    assert_eq!(ctx.gate.state(), TorqueState::Faulted);
    assert_eq!(ctx.latched_drive_err, super::cycle::FRAME_LATE_FAULT_CODE);
}

#[test]
fn cycle_skip_faults_even_when_lateness_is_within_tolerance() {
    let mut ctx = test_ctx("skip-fault");
    ctx.late_tolerance_ns = Some(1_000_000);
    ctx.baseline_reanchor_count = 5;
    super::cycle::police_frame_timing(&mut ctx, -100_000);
    assert_eq!(ctx.gate.state(), TorqueState::Faulted);
    assert_eq!(ctx.latched_drive_err, super::cycle::CYCLE_SKIP_FAULT_CODE);
}

/// A pure-T2 piece has constant accel: d²/du²·T2 = 4, so
/// accel = 4·C2·(2/duration)².
const T2_C: f32 = 0.625; // accel = 4·0.625·400 = 1000 mm/s² at d = 0.1 s

#[test]
fn group_delay_leads_curve_sampling_by_one_cycle() {
    const APPLY_T: u64 = 1_200_000;
    let vel_piece = piece(1_000_000, 0.1, &[0.0, 1.0]);

    let mut reference = test_ctx("group-delay-ref");
    push_all(&mut reference, vel_piece);
    let (pos_t, _, _) = reference.rings[0].sample(APPLY_T).expect("piece covers T");
    let (pos_lead, _, _) = reference.rings[0]
        .sample(APPLY_T + CYCLE_NS)
        .expect("piece covers T+CYCLE_NS");
    let fixed_map = CountMap::new(COUNTS_PER_MM, 0, 0.0);
    let expected_lead_counts =
        fixed_map.target_counts(f64::from(pos_lead)) - fixed_map.target_counts(f64::from(pos_t));
    assert!(
        expected_lead_counts != 0,
        "constant-velocity piece must advance across one cycle"
    );

    let seed_fixed_map = |ctx: &mut EndpointCtx| {
        for s in 0..NUM_SLAVES {
            ctx.cmaps[s] = Some(fixed_map);
        }
    };

    let mut unleaded = test_ctx("group-delay-zero");
    unleaded.group_delay_ns = 0;
    push_all(&mut unleaded, vel_piece);
    seed_fixed_map(&mut unleaded);
    let unleaded_sample = APPLY_T + unleaded.group_delay_ns;
    compute_motion_targets(&mut unleaded, unleaded_sample);
    let unleaded_targets = targets(&unleaded);

    let mut leaded = test_ctx("group-delay-cycle");
    leaded.group_delay_ns = CYCLE_NS;
    push_all(&mut leaded, vel_piece);
    seed_fixed_map(&mut leaded);
    let leaded_sample = APPLY_T + leaded.group_delay_ns;
    compute_motion_targets(&mut leaded, leaded_sample);
    let leaded_targets = targets(&leaded);

    for s in 0..NUM_SLAVES {
        assert_eq!(
            leaded_targets[s] - unleaded_targets[s],
            expected_lead_counts,
            "slot {s}: group delay must lead curve sampling by velocity*CYCLE_NS counts"
        );
    }
}

// ---- pin-rotor (mode A) torque hold -------------------------------------

const PIN_IDENTITY: &str = r#"
version = 8
axes = ["x", "y"]
modes = ["x", "y"]
frame = [[1.0, 0.0], [0.0, 1.0]]
mass = [0.020, 0.020]
viscous = [0.0, 0.0]
coulomb = [0.0, 0.0]
compliance = [1.0e-5, 0.0]
pin_mass = [0.02, 0.0]
pin_zeta = [0.1, 0.0]
pin_lead_us = 0.0
"#;

fn pin_ctx(name: &str, toml: &str) -> EndpointCtx {
    let mut ctx = test_ctx(name);
    let model = crate::dynamics::DynamicsModel::from_toml_str(toml).unwrap();
    ctx.pin = super::cycle::PinState::build(&model, ctx.cycle_ns);
    ctx.dynamics = Some(model);
    ctx.torque_clamp_tenths = vec![3000; NUM_SLAVES];
    ctx
}

/// Drive one constant-accel (T2) stroke and return slot-0 torque_offset per
/// cycle. The stroke stays inside the 0.1 s piece for all `cycles`.
fn run_accel_collect_torque(ctx: &mut EndpointCtx, cycles: u64) -> Vec<i32> {
    push_all(ctx, piece(1_000_000, 0.1, &[0.0, 0.0, T2_C]));
    let mut out = Vec::with_capacity(cycles as usize);
    for c in 0..cycles {
        compute_motion_targets(ctx, 1_000_000 + c * CYCLE_NS);
        ctx.drive.cycle();
        out.push(i32::from(ctx.drive.telemetry(0).torque_offset));
    }
    out
}

/// A pinned mode holds the rotor: the emitted torque_offset carries the
/// anti-ring pin component (a decaying oscillation at f_b), and the pin
/// never touches the commanded position target.
#[test]
fn pin_mode_holds_rotor_in_two_mass_sim() {
    let mut pinned = pin_ctx("pin-hold", PIN_IDENTITY);
    let mut plain = pin_ctx(
        "pin-hold-plain",
        &PIN_IDENTITY
            .replace("compliance = [1.0e-5, 0.0]", "compliance = [0.0, 0.0]")
            .replace("pin_mass = [0.02, 0.0]", "pin_mass = [0.0, 0.0]"),
    );

    const CYCLES: u64 = 390;
    let t_pin = run_accel_collect_torque(&mut pinned, CYCLES);
    let t_plain = run_accel_collect_torque(&mut plain, CYCLES);

    // Pin does not move the commanded position target (torque-only hold).
    assert_eq!(
        targets(&pinned)[0],
        targets(&plain)[0],
        "pinned mode must carry no position lead"
    );

    // Pin torque = pinned − plain: a decaying oscillation at f_b.
    let ring: Vec<i32> = t_pin.iter().zip(&t_plain).map(|(a, b)| a - b).collect();
    let max_abs = |s: &[i32]| s.iter().map(|v| v.abs()).max().unwrap_or(0);
    let early = max_abs(&ring[..160]);
    let late = max_abs(&ring[310..390]);
    assert!(
        early >= 8,
        "pin ring must be present after the transient: {early}"
    );
    assert!(
        late * 2 <= early,
        "pin ring must decay with zeta: early {early}, late {late}"
    );
    let mut sign_changes = 0;
    let mut last = 0i32;
    for &x in &ring[..250] {
        if x != 0 {
            if last != 0 && (x > 0) != (last > 0) {
                sign_changes += 1;
            }
            last = x;
        }
    }
    assert!(
        sign_changes >= 3,
        "pin torque must oscillate at f_b: {sign_changes} sign changes"
    );
    // The non-pinned mode (slot 1) carries no residual.
    assert_eq!(
        pinned.pin.residual_for_slot(1),
        (0.0, 0.0),
        "non-pinned mode must carry no residual"
    );
}

/// Buzz mode A parameters shared by the through-buzz tests: identity frame,
/// slot 0 driven, no sign flip, 0.1 mm amplitude, long steady tone.
const BUZZ_AMP_NM: u32 = 100_000; // 0.1 mm

/// Arm a single-tone buzz on slot 0 at `freq_millihz` and run it, returning
/// the per-cycle slot-0 pin torque contribution and target positions.
fn run_buzz_collect(ctx: &mut EndpointCtx, freq_millihz: u32, cycles: u64) -> (Vec<f32>, Vec<i32>) {
    let rc = ctx.buzz.arm(
        NUM_SLAVES as u8,
        0b01,
        0,
        freq_millihz,
        freq_millihz,
        BUZZ_AMP_NM,
        2000,
        20,
        [0; crate::buzz::MAX_BUZZ_SLOTS],
    );
    assert_eq!(rc, 0);
    let mut pin = Vec::with_capacity(cycles as usize);
    let mut tgt = Vec::with_capacity(cycles as usize);
    for c in 0..cycles {
        compute_motion_targets(ctx, 1_000_000 + c * CYCLE_NS);
        ctx.drive.cycle();
        pin.push(ctx.pin.slot_torque_at(0));
        tgt.push(ctx.drive.telemetry(0).target_position);
    }
    (pin, tgt)
}

/// The pin predictor runs through a buzz: with the buzz tone parked on the
/// notch the pinned mode integrates the analytic buzz forcing and injects a
/// live torque bounded by ~Q·m_L·a_buzz (Q = 1/2ζ), while the commanded
/// position target stays untouched.
#[test]
fn pin_runs_through_buzz() {
    let zeta = 0.1f32;
    let q = 1.0 / (2.0 * zeta);
    let pin_mass = 0.02f32;
    // f_b = 1/(2π√compliance) ≈ 50.3 Hz; park the tone on the notch.
    let f_notch = 50_000u32;
    let omega_buzz = 2.0 * std::f32::consts::PI * (f_notch as f32 / 1000.0);
    let amp_mm = BUZZ_AMP_NM as f32 * 1e-6;
    let a_buzz = omega_buzz * omega_buzz * amp_mm; // |a_buzz| = ω²·A

    let mut ctx = pin_ctx("pin-through-buzz", PIN_IDENTITY);
    let mut plain = pin_ctx(
        "pin-through-buzz-plain",
        &PIN_IDENTITY
            .replace("compliance = [1.0e-5, 0.0]", "compliance = [0.0, 0.0]")
            .replace("pin_mass = [0.02, 0.0]", "pin_mass = [0.0, 0.0]"),
    );
    const CYCLES: u64 = 500;
    let (pin, tgt) = run_buzz_collect(&mut ctx, f_notch, CYCLES);
    let (plain_pin, plain_tgt) = run_buzz_collect(&mut plain, f_notch, CYCLES);

    // (c) The pin stays torque-only through the buzz: slot-0 targets match
    // the no-pin model exactly (both carry only the buzz offset), and the
    // plain model injects no pin torque.
    assert_eq!(
        tgt, plain_tgt,
        "pinned mode must carry no compliance position lead during a buzz"
    );
    assert!(
        plain_pin.iter().all(|&t| t == 0.0),
        "the no-pin model must inject no pin torque"
    );

    // (a) Pinned mode injects a live torque through the buzz.
    let steady = &pin[200..];
    let max_pin = steady.iter().fold(0.0f32, |m, &t| m.max(t.abs()));
    assert!(
        max_pin > pin_mass * a_buzz,
        "pin torque must be live and Q-amplified on the notch: \
         {max_pin} vs m·a_buzz {}",
        pin_mass * a_buzz
    );
    // (b) …bounded by ~Q·m_L·a_buzz with margin.
    let bound = q * pin_mass * a_buzz;
    assert!(
        max_pin < 2.0 * bound,
        "pin torque must stay within ~Q·m_L·a_buzz: {max_pin} vs bound {bound}"
    );
}

/// A buzz well above f_b sees no resonant amplification: the pinned mode
/// cannot follow, so the injected torque stays far below the on-notch
/// Q·m_L·a_buzz and never grows cycle-over-cycle.
#[test]
fn pin_buzz_above_notch_rolls_off() {
    let zeta = 0.1f32;
    let q = 1.0 / (2.0 * zeta);
    let pin_mass = 0.02f32;
    // ~5× above the ≈50 Hz notch.
    let f_hi = 250_000u32;
    let omega_buzz = 2.0 * std::f32::consts::PI * (f_hi as f32 / 1000.0);
    let amp_mm = BUZZ_AMP_NM as f32 * 1e-6;
    let a_buzz = omega_buzz * omega_buzz * amp_mm;
    let bound = q * pin_mass * a_buzz;

    let mut ctx = pin_ctx("pin-buzz-above", PIN_IDENTITY);
    const CYCLES: u64 = 500;
    let (pin, _tgt) = run_buzz_collect(&mut ctx, f_hi, CYCLES);

    // No Q-amplification: the injected torque stays well under the on-notch
    // bound (the oscillator only sees the direct inertial forcing).
    let max_pin = pin[200..].iter().fold(0.0f32, |m, &t| m.max(t.abs()));
    assert!(
        max_pin < 0.5 * bound,
        "off-notch buzz must roll off below Q·m_L·a_buzz: {max_pin} vs bound {bound}"
    );
    // Non-divergent: the late-window peak does not exceed the earlier steady
    // peak (a B-style inversion would grow without bound).
    let peak = |s: &[f32]| s.iter().fold(0.0f32, |m, &t| m.max(t.abs()));
    let early = peak(&pin[150..300]);
    let late = peak(&pin[350..500]);
    assert!(
        late <= early * 1.1 + 1.0,
        "pin torque must not grow cycle-over-cycle: early {early}, late {late}"
    );
}

/// Streaming a new dynamics model while a buzz is live is accepted and
/// rebuilds the pin-rotor state cleanly: the install succeeds (the new zeta
/// lands in `ctx.dynamics`), the pin oscillator restarts from a defined zero
/// state (residual demodulator zeroed), and the pinned mode keeps injecting
/// torque as the buzz forcing continues to integrate on the fresh state -
/// now with the new damping (a higher zeta lowers the on-notch Q, so the
/// steady pin torque settles lower). This is the mid-sweep re-stream that
/// SERVO_SWEEP_PIN relies on (pin runs THROUGH the buzz, one tone, many
/// models).
#[test]
fn set_dynamics_model_mid_buzz_rebuilds_pin_cleanly() {
    let mut ctx = pin_ctx("pin-mid-buzz-swap", PIN_IDENTITY);
    // Park the tone on the notch (f_b ≈ 50.3 Hz) and run it live.
    let f_notch = 50_000u32;
    let rc = ctx.buzz.arm(
        NUM_SLAVES as u8,
        0b01,
        0,
        f_notch,
        f_notch,
        BUZZ_AMP_NM,
        4000,
        20,
        [0; crate::buzz::MAX_BUZZ_SLOTS],
    );
    assert_eq!(rc, 0);
    // Run the tone on the original zeta=0.1 model until the pin torque has
    // built up to its on-notch steady amplitude (Q = 1/2ζ = 5).
    let mut pin_before = 0.0f32;
    for c in 0..2000u64 {
        compute_motion_targets(&mut ctx, 1_000_000 + c * CYCLE_NS);
        ctx.drive.cycle();
        if c >= 1600 {
            pin_before = pin_before.max(ctx.pin.slot_torque_at(0).abs());
        }
    }
    assert!(
        pin_before > 0.0,
        "pin torque must be live before the swap: {pin_before}"
    );
    assert!(
        ctx.buzz.active(),
        "buzz must still be running before the swap"
    );

    // Swap the model mid-buzz: same pin, larger zeta (0.1 -> 0.3).
    let mut msg = dynamics_msg(0.020);
    msg.frame = vec![1.0, 0.0, 0.0, 1.0];
    msg.mass = vec![0.020, 0.020];
    msg.viscous = vec![0.0, 0.0];
    msg.coulomb = vec![0.0, 0.0];
    msg.compliance = vec![1.0e-5, 0.0];
    msg.pin_mass = vec![0.02, 0.0];
    msg.pin_zeta = vec![0.3, 0.0];
    msg.pin_lead_us = 0.0;
    super::commands::handle_set_dynamics_model(&mut ctx, 7, msg);

    // Install succeeded (rc==0 path): the new zeta is live, the previous
    // model was replaced, and the buzz never stopped.
    let model = ctx.dynamics.as_ref().expect("model still installed");
    assert!(
        (model.pin_zeta[0] - 0.3).abs() < 1e-6,
        "mid-buzz swap must install the new zeta: {}",
        model.pin_zeta[0]
    );
    assert!(ctx.buzz.active(), "buzz keeps running across the swap");

    // The pin state rebuilt: the oscillator + residual demodulator restart
    // from a defined zero state (no carried-over phasor from the old model).
    assert_eq!(
        ctx.pin.residual_for_slot(0),
        (0.0, 0.0),
        "residual must restart on the model swap"
    );

    // Next cycles integrate the buzz forcing on the fresh state: pin torque
    // re-accumulates and settles at the new, more-damped steady amplitude.
    let mut pin_after = 0.0f32;
    for c in 2000..4000u64 {
        compute_motion_targets(&mut ctx, 1_000_000 + c * CYCLE_NS);
        ctx.drive.cycle();
        if c >= 3600 {
            pin_after = pin_after.max(ctx.pin.slot_torque_at(0).abs());
        }
    }
    assert!(
        pin_after > 0.0,
        "pin torque must continue after the mid-buzz swap: {pin_after}"
    );
    // Higher damping => lower on-notch Q => lower steady pin torque. This
    // only holds if the buzz forcing kept integrating on the NEW model.
    assert!(
        pin_after < pin_before,
        "new (higher) zeta must lower the steady pin torque: \
         before {pin_before} after {pin_after}"
    );
}

/// A nonzero pin lead advances the torque waveform: its first positive
/// zero-crossing lands earlier than the unled waveform's.
#[test]
fn pin_lead_advances_phase() {
    let mut unled = pin_ctx("pin-lead-0", PIN_IDENTITY);
    let mut leaded = pin_ctx(
        "pin-lead-1000",
        &PIN_IDENTITY.replace("pin_lead_us = 0.0", "pin_lead_us = 1000.0"),
    );
    let collect = |ctx: &mut EndpointCtx| -> Vec<f32> {
        push_all(ctx, piece(1_000_000, 0.1, &[0.0, 0.0, T2_C]));
        let mut v = Vec::new();
        for c in 0..200u64 {
            compute_motion_targets(ctx, 1_000_000 + c * CYCLE_NS);
            ctx.drive.cycle();
            v.push(ctx.pin.slot_torque_at(0));
        }
        v
    };
    let a = collect(&mut unled);
    let b = collect(&mut leaded);
    let first_pos = |s: &[f32]| {
        s.iter()
            .position(|&x| x > 1.0)
            .expect("pin waveform must cross positive")
    };
    let cross_unled = first_pos(&a);
    let cross_leaded = first_pos(&b);
    assert!(
        cross_leaded < cross_unled,
        "pin lead must advance the phase: leaded {cross_leaded} vs unled {cross_unled}"
    );
}

/// Residual demodulator regression: under a sustained on-notch ring in the
/// pinned mode's following error the demodulated (re, im) phasor must
/// converge to a steady value bounded by the ring amplitude — never diverge
/// (the unclamped low-pass coefficient once let α≥2 blow the accumulator up
/// to ~1e33 with the phase snapping between 0 and −90°).
#[test]
fn pin_residual_demod_converges_and_stays_bounded() {
    let mut ctx = pin_ctx("pin-residual-demod", PIN_IDENTITY);
    let dt = CYCLE_NS as f64 * 1e-9;
    // Two-mass belt notch from PIN_IDENTITY: ω_b = 1/√compliance. The demod
    // references ω_b (NOT the predictor's ω_d — see the ζ-bias regression
    // below), so the synthetic ring sits at ω_b too.
    let omega = 1.0 / (1.0e-5f64).sqrt();
    let wd = omega;
    let amp = 0.05f64; // 0.05 mm sub-mm belt ring
    let phi = 0.7f64;
    const CYCLES: usize = 12_000; // 3 s at 250 µs
    let acc = [0.0f32; NUM_SLAVES];
    let mut mag = Vec::with_capacity(CYCLES);
    let mut peak_ferr = 0.0f32;
    for n in 0..CYCLES {
        let t = n as f64 * dt;
        // Mode x (slot 0, pinned) sees the on-notch ring; slot 1 (non-pinned)
        // carries a nonzero following error that must never leak to a residual.
        let ferr0 = (amp * libm::cos(wd * t + phi)) as f32;
        let ferr1 = (amp * libm::sin(wd * t)) as f32;
        peak_ferr = peak_ferr.max(ferr0.abs());
        ctx.pin.step(&acc, &[ferr0, ferr1], true);
        let (re, im) = ctx.pin.residual_for_slot(0);
        mag.push(re.hypot(im));
    }
    // (iii) nonzero: the on-notch ring demodulates to a steady phasor.
    let last = *mag.last().unwrap();
    assert!(last > 0.1 * amp as f32, "residual must be nonzero: {last}");
    // (i) converges: last-100 mean within 20% of the mean 0.5 s (2000 cy) earlier.
    let mean = |s: &[f32]| s.iter().sum::<f32>() / s.len() as f32;
    let late = mean(&mag[CYCLES - 100..]);
    let early = mean(&mag[CYCLES - 2100..CYCLES - 2000]);
    assert!(
        (late - early).abs() <= 0.2 * early,
        "residual must converge: late {late} vs early {early}"
    );
    // (ii) bounded by the ring order: never exceeds 10× the peak following error.
    let peak_res = mag.iter().fold(0.0f32, |m, &x| m.max(x));
    assert!(
        peak_res < 10.0 * peak_ferr,
        "residual must stay bounded by the ring amplitude: {peak_res} vs peak ferr {peak_ferr}"
    );
    // The non-pinned mode carries a nonzero following error but zero residual.
    assert_eq!(
        ctx.pin.residual_for_slot(1),
        (0.0, 0.0),
        "non-pinned mode must carry no residual"
    );
}

// ---- CoreXY frame quantitative cancellation ----------------------------
//
// The identity-frame pin tests above cannot detect a frame-normalization
// error: with F = I both the accel projection (F) and the torque lift round
// trip to unity. A real CoreXY frame has |F·Fᵀ| < 1 (0.25·I on the bench's
// 4-slot AWD frame), so a naive Fᵀ torque lift lands 4× short. These tests
// drive the pin through a real frame and check, per mode, that the emitted
// slot torque realizes the intended mode-space cancellation (F·slot = τ_pin)
// — the quantity a downstream two-mass load actually feels.

/// Drive a per-cycle slot-accel sequence `acc_seq` into a pin model built from
/// an explicit frame, alongside one identity single-mode reference oscillator
/// per mode (the ground-truth mode torque `τ_pin`). Returns, per mode and per
/// cycle, the achieved mode-space cancellation `F·slot_torque` and the
/// reference `τ_pin`. A correct lift makes the two equal; a plain Fᵀ lift
/// attenuates `F·slot` by `F·Fᵀ`.
fn pin_frame_cancellation(
    frame: &[f32],
    n_modes: usize,
    n_slots: usize,
    mass: &[f32],
    compliance: &[f32],
    pin_mass: &[f32],
    pin_zeta: &[f32],
    acc_seq: &[Vec<f32>],
) -> (Vec<Vec<f32>>, Vec<Vec<f32>>) {
    use crate::dynamics::DynamicsModel;
    let zeros_m = vec![0.0f32; n_modes];
    let model = DynamicsModel::from_parts(
        n_slots,
        n_modes,
        frame,
        mass,
        &zeros_m,
        &zeros_m,
        compliance,
        pin_mass,
        pin_zeta,
        0.0,
        &[],
    )
    .unwrap();
    let mut pin = super::cycle::PinState::build(&model, CYCLE_NS as i64);
    // One identity (1 slot, 1 mode) reference per mode: its slot torque IS the
    // mode torque, since the identity lift is unity.
    let mut refs: Vec<super::cycle::PinState> = (0..n_modes)
        .map(|k| {
            let m = DynamicsModel::from_parts(
                1,
                1,
                &[1.0],
                &[mass[k]],
                &[0.0],
                &[0.0],
                &[compliance[k]],
                &[pin_mass[k]],
                &[pin_zeta[k]],
                0.0,
                &[],
            )
            .unwrap();
            super::cycle::PinState::build(&m, CYCLE_NS as i64)
        })
        .collect();
    let cycles = acc_seq.len();
    let mut achieved = vec![Vec::with_capacity(cycles); n_modes];
    let mut reference = vec![Vec::with_capacity(cycles); n_modes];
    for acc in acc_seq {
        pin.step(acc, &vec![0.0f32; n_slots], true);
        for k in 0..n_modes {
            // The reference sees the exact mode accel the coupled pin projects.
            let a_cmd: f32 = model.frame_row(k).iter().zip(acc).map(|(f, a)| f * a).sum();
            let f_slot: f32 = model
                .frame_row(k)
                .iter()
                .enumerate()
                .map(|(s, f)| f * pin.slot_torque_at(s))
                .sum();
            achieved[k].push(f_slot);
            refs[k].step(&[a_cmd], &[0.0], true);
            reference[k].push(refs[k].slot_torque_at(0));
        }
    }
    (achieved, reference)
}

/// Peak-to-peak of a damped locked-rotor load oscillator (`m_L·ÿ = -k_b·y -
/// c·ẏ + u`) driven ZOH by the per-cycle residual mode force `u`, measured
/// over the tail. The pinned load feels the belt reaction the pin fails to
/// cancel, so a fully-cancelling pin leaves a flat load.
fn load_ring(force: &[f32], fb: f64, zeta: f64, m_l: f32) -> f32 {
    let w = 2.0 * std::f64::consts::PI * fb;
    let dt = CYCLE_NS as f64 * 1e-9;
    let sub = 40usize;
    let h = dt / sub as f64;
    let (mut y, mut v) = (0.0f64, 0.0f64);
    let (mut lo, mut hi) = (0.0f64, 0.0f64);
    for (i, &u) in force.iter().enumerate() {
        for _ in 0..sub {
            let acc = -w * w * y - 2.0 * zeta * w * v + f64::from(u) / f64::from(m_l);
            v += acc * h;
            y += v * h;
        }
        if i > force.len() / 3 {
            lo = lo.min(y);
            hi = hi.max(y);
        }
    }
    (hi - lo) as f32
}

/// Assert, for a given frame, that the pin realizes mode-exact cancellation on
/// EVERY mode: the emitted slot torque maps back (F·slot) to the intended
/// τ_pin within 10%, and a simulated two-mass load ring drops by >85% versus
/// pin-off. A frame-normalization error (Fᵀ lift → 4× short on AWD) or a
/// per-mode sign flip fails both checks.
fn assert_pin_cancels_every_mode(
    frame: &[f32],
    n_modes: usize,
    n_slots: usize,
    mass: &[f32],
    fb: &[f64],
    pin_frac: &[f32],
) {
    let compliance: Vec<f32> = fb
        .iter()
        .map(|f| (1.0 / (2.0 * std::f64::consts::PI * f).powi(2)) as f32)
        .collect();
    let pin_mass: Vec<f32> = mass.iter().zip(pin_frac).map(|(m, f)| m * f).collect();
    let pin_zeta = vec![0.08f32; n_modes];
    // Model handle for the F⁺ excitation columns (min-norm slot accel that
    // realizes a pure mode-k acceleration).
    let model = crate::dynamics::DynamicsModel::from_parts(
        n_slots,
        n_modes,
        frame,
        mass,
        &vec![0.0f32; n_modes],
        &vec![0.0f32; n_modes],
        &compliance,
        &pin_mass,
        &pin_zeta,
        0.0,
        &[],
    )
    .unwrap();
    let dt = CYCLE_NS as f64 * 1e-9;
    const CYCLES: usize = 900;
    let rms = |s: &[f32]| (s.iter().map(|x| x * x).sum::<f32>() / s.len() as f32).sqrt();
    for k in 0..n_modes {
        // Sustained on-notch sinusoid confined to mode k: the pinned mode rings
        // at Q, giving a large τ_pin to test the cancellation against.
        let wb = 2.0 * std::f64::consts::PI * fb[k];
        let lift = model.pin_lift_row(k).to_vec();
        let acc_seq: Vec<Vec<f32>> = (0..CYCLES)
            .map(|c| {
                let s = (1000.0 * libm::sin(wb * c as f64 * dt)) as f32;
                lift.iter().map(|w| w * s).collect()
            })
            .collect();
        let (achieved, reference) = pin_frame_cancellation(
            frame,
            n_modes,
            n_slots,
            mass,
            &compliance,
            &pin_mass,
            &pin_zeta,
            &acc_seq,
        );
        // Steady-state tail only (skip the ring build-up).
        let tail = CYCLES / 3;
        let a = &achieved[k][tail..];
        let r = &reference[k][tail..];
        // The driven mode must genuinely ring, else the test proves nothing.
        let ref_rms = rms(r);
        assert!(
            ref_rms > 1.0,
            "mode {k} reference torque too small: {ref_rms}"
        );
        // (1) F·slot == τ_pin within 10% — catches the frame-normalization
        //     gain deficit (Fᵀ lift → 4× short on AWD) AND a per-mode sign
        //     flip (which drives the error to ~2× the reference).
        let err: Vec<f32> = a.iter().zip(r).map(|(x, y)| x - y).collect();
        let rel = rms(&err) / ref_rms;
        assert!(
            rel < 0.10,
            "mode {k}: emitted slot torque must map to τ_pin within 10%, got {:.1}% \
             (F·slot RMS {:.3} vs τ_pin RMS {:.3})",
            rel * 100.0,
            rms(a),
            ref_rms
        );
        // (2) simulated two-mass load ring: residual = τ_pin − F·slot drives
        //     the load; pin-off feels the whole belt reaction (τ_pin).
        let residual: Vec<f32> = r.iter().zip(a).map(|(x, y)| x - y).collect();
        let ring_off = load_ring(r, fb[k], 0.08, pin_mass[k]);
        let ring_on = load_ring(&residual, fb[k], 0.08, pin_mass[k]);
        let reduction = 1.0 - ring_on / ring_off;
        assert!(
            reduction > 0.85,
            "mode {k}: load ring must drop >85% with the pin, got {:.1}% \
             (off {ring_off:.4} on {ring_on:.4})",
            reduction * 100.0
        );
    }
}

/// Bench 4-slot AWD CoreXY frame (drive invert signs folded in, hence the
/// asymmetric per-row signs): F·Fᵀ = 0.25·I, so a naive Fᵀ pin lift lands 4×
/// short on BOTH modes. The correct F⁺ lift cancels each mode exactly.
#[test]
fn pin_corexy_awd_cancels_both_modes() {
    // frame rows: x = [.25,-.25,-.25,-.25], y = [.25,-.25,.25,.25]
    let frame = [0.25, -0.25, -0.25, -0.25, 0.25, -0.25, 0.25, 0.25];
    assert_pin_cancels_every_mode(
        &frame,
        2,
        4,
        &[0.00453, 0.00747],
        &[216.8, 131.5],
        // load fraction 1 − (f_b/f_peak)²
        &[
            1.0 - (216.8f32 / 313.5).powi(2),
            1.0 - (131.5f32 / 215.8).powi(2),
        ],
    );
}

/// Symmetric 2×2 CoreXY frame ([[.5,.5],[.5,-.5]] convention): F·Fᵀ = 0.5·I,
/// so a naive Fᵀ lift lands 2× short. Transpose-symmetric, so it hides a
/// pure transpose bug but still exposes the normalization deficit.
#[test]
fn pin_corexy_symmetric_cancels_both_modes() {
    let frame = [0.5, 0.5, 0.5, -0.5];
    assert_pin_cancels_every_mode(&frame, 2, 2, &[0.020, 0.020], &[131.5, 131.5], &[0.6, 0.6]);
}

#[test]
fn osc_zoh_is_continuous_across_critical_damping() {
    // The three closed forms (sin/cos, polynomial, sinh/cosh) must agree at
    // the regime boundary: coefficients just below, at, and just above
    // zeta = 1 are within numerical noise of each other.
    let omega = 1.0 / (1.0e-5f64).sqrt();
    let dt = 250e-6f64;
    let (lo_a, lo_b) = super::cycle::osc_zoh(omega, 1.0 - 1e-9, dt);
    let (cr_a, cr_b) = super::cycle::osc_zoh(omega, 1.0, dt);
    let (hi_a, hi_b) = super::cycle::osc_zoh(omega, 1.0 + 1e-9, dt);
    for i in 0..4 {
        assert!(
            (lo_a[i] - cr_a[i]).abs() < 1e-6,
            "a[{i}]: {} vs {}",
            lo_a[i],
            cr_a[i]
        );
        assert!(
            (hi_a[i] - cr_a[i]).abs() < 1e-6,
            "a[{i}]: {} vs {}",
            hi_a[i],
            cr_a[i]
        );
    }
    for i in 0..2 {
        let scale = cr_b[i].abs().max(1e-12);
        assert!((lo_b[i] - cr_b[i]).abs() / scale < 1e-5);
        assert!((hi_b[i] - cr_b[i]).abs() / scale < 1e-5);
    }
}

#[test]
fn osc_zoh_overdamped_is_finite_and_never_oscillates() {
    // zeta > 1: released from a deflection, the predictor state must decay
    // monotonically toward zero (two real poles - no sign changes), and the
    // coefficients must stay finite even at absurd damping.
    let omega = 1.0 / (1.0e-5f64).sqrt();
    let dt = 250e-6f64;
    for zeta in [1.4f64, 3.0, 50.0, 1e6] {
        let (a, b) = super::cycle::osc_zoh(omega, zeta, dt);
        for v in a.iter().chain(b.iter()) {
            assert!(v.is_finite(), "zeta {zeta}: non-finite coefficient {v}");
        }
        let (mut d, mut v) = (1.0f64, 0.0f64);
        let mut prev = d;
        for _ in 0..4000 {
            let (dn, vn) = (a[0] * d + a[1] * v, a[2] * d + a[3] * v);
            d = dn;
            v = vn;
            assert!(d >= -1e-12, "zeta {zeta}: deflection crossed zero: {d}");
            assert!(
                d <= prev + 1e-12,
                "zeta {zeta}: deflection grew: {d} > {prev}"
            );
            prev = d;
        }
    }
}

#[test]
fn pin_residual_demod_is_unbiased_by_zeta() {
    // Regression for the bench cliff (0.03 um at zeta=0.9 vs 5.57 um at
    // zeta=1.1): the demod used to reference the predictor's ring frequency
    // omega_d = omega*sqrt(1-zeta^2), which at zeta=0.9 sits 56% below the
    // mode - the beat against an on-mode ring averaged to zero and sweeps
    // read "silence". The demodulator must read an omega_b ring at full
    // magnitude regardless of the predictor's damping.
    let toml = PIN_IDENTITY.replace("pin_zeta = [0.1, 0.0]", "pin_zeta = [0.9, 0.0]");
    let mut ctx = pin_ctx("pin-residual-zeta-bias", &toml);
    let dt = CYCLE_NS as f64 * 1e-9;
    let omega = 1.0 / (1.0e-5f64).sqrt();
    let amp = 0.05f64;
    const CYCLES: usize = 12_000; // 3 s at 250 us
    let acc = [0.0f32; NUM_SLAVES];
    let mut last = 0.0f32;
    for n in 0..CYCLES {
        let t = n as f64 * dt;
        let ferr0 = (amp * libm::cos(omega * t + 0.3)) as f32;
        ctx.pin.step(&acc, &[ferr0, 0.0], true);
        let (re, im) = ctx.pin.residual_for_slot(0);
        last = re.hypot(im);
    }
    assert!(
        last > 0.5 * amp as f32,
        "high-zeta demod must read the on-mode ring, not average it away: \
         {last} vs ring amplitude {amp}"
    );
}

/// τ_pin is m_L·d̈, so a constant commanded accel must leave the pin
/// contributing exactly nothing: the belt settles to a static deflection
/// d = -a/ω² with ḋ = 0 and d̈ = 0, and the rotor already carries that load
/// through the plain torque feedforward. This has to hold at every phase
/// lead — advancing the state without the forcing that acts over the lead
/// window leaks a torque proportional to accel (≈14% of m_L·a at 130 Hz
/// and 600 µs), worst exactly where the machine accelerates hardest.
#[test]
fn pin_torque_vanishes_at_constant_accel_for_every_lead() {
    use crate::dynamics::DynamicsModel;
    const A_CMD: f32 = 20_000.0; // mm/s², a hard print acceleration
    const PIN_MASS: f32 = 0.02;
    for &f_b in &[100.0f64, 130.0, 160.0] {
        let compliance = 1.0 / (2.0 * std::f64::consts::PI * f_b).powi(2);
        for &lead_us in &[0.0f64, 300.0, 600.0, 1200.0] {
            let model = DynamicsModel::from_parts(
                1,
                1,
                &[1.0],
                &[0.04],
                &[0.0],
                &[0.0],
                &[compliance as f32],
                &[PIN_MASS],
                &[0.02],
                lead_us,
                &[],
            )
            .unwrap();
            let mut pin = super::cycle::PinState::build(&model, CYCLE_NS as i64);
            // Long enough for the ζ=0.02 transient to die: τ ≈ 1/(ζω) is
            // ~80 ms at 100 Hz, so 4 s is many time constants.
            for _ in 0..4000 {
                pin.step(&[A_CMD], &[0.0], true);
            }
            let settled = pin.slot_torque_at(0);
            // Scale against the torque the pin would inject if it wrongly
            // treated the whole load as unsupported.
            let leak = settled.abs() / (PIN_MASS * A_CMD);
            assert!(
                leak < 1e-3,
                "f_b={f_b} Hz lead={lead_us} µs: pin leaks {leak:.4} of m_L·a \
                 at cruise (τ={settled})"
            );
        }
    }
}

#[test]
fn suppressed_slot_holds_target_while_peer_advances() {
    let mut ctx = test_ctx("suppress-hold");

    push_all(&mut ctx, piece(1_000_000, 0.01, &[2.5, 2.5]));
    run_cycles(&mut ctx, 1_000_000, 5_000_000);
    let mid = targets(&ctx);

    super::commands::handle_stepper_suppress(
        &mut ctx,
        1,
        StepperSuppress {
            motor: 0,
            stepper: 0,
            engage: 1,
        },
    );
    run_cycles(&mut ctx, 5_250_000, 11_000_000);
    let end = targets(&ctx);
    assert_eq!(
        end[0], mid[0],
        "suppressed slot must hold its last commanded target"
    );
    assert_ne!(end[1], mid[1], "peer slot must keep following the stream");

    super::commands::handle_stepper_suppress(
        &mut ctx,
        2,
        StepperSuppress {
            motor: 0xFF,
            stepper: 0xFF,
            engage: 0,
        },
    );
    assert!(
        ctx.suppressed.iter().all(|&s| !s),
        "clear-all must release every slot"
    );
    push_all(&mut ctx, piece(20_000_000, 0.01, &[9.0]));
    run_cycles(&mut ctx, 20_000_000, 20_500_000);
    assert_ne!(
        targets(&ctx)[0],
        end[0],
        "released slot must follow the stream again"
    );
}

#[test]
fn suppress_maps_stepper_index_within_a_shared_axis() {
    let mut ctx = test_ctx("suppress-pair");
    ctx.slave_axes = vec![0, 0];
    super::commands::handle_stepper_suppress(
        &mut ctx,
        1,
        StepperSuppress {
            motor: 0,
            stepper: 1,
            engage: 1,
        },
    );
    assert_eq!(ctx.suppressed, vec![false, true]);
    super::commands::handle_stepper_suppress(
        &mut ctx,
        2,
        StepperSuppress {
            motor: 0,
            stepper: 2,
            engage: 1,
        },
    );
    assert_eq!(
        ctx.suppressed,
        vec![false, true],
        "an unknown stepper index must be rejected, not clamped"
    );
}
