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
use crate::sdo::SdoBus;
use crate::sensorless::SensorlessBank;
use crate::server::FrameServer;
use crate::stream_halt::StreamHalt;
use crate::torque::TorqueGate;
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
    fn enable(&mut self, _slot: usize) -> i32 {
        0
    }
    fn disable(&mut self, _slot: usize) {}
    fn shutdown(&mut self) {}
    fn set_target_position(&mut self, slot: usize, counts: i32) {
        self.targets[slot] = counts;
    }
    fn set_velocity_offset(&mut self, _slot: usize, _counts_per_s: i32) {}
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
    test_ctx_with_drive(name, TrackingLagDrive::at_rest())
}

fn test_ctx_with_drive(name: &str, drive: TrackingLagDrive) -> EndpointCtx {
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
        telemetry_period: u64::MAX,
        dynamics: None,
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
        tap_slots: (0..NUM_SLAVES as u8).collect(),
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

/// The trim must land on the wire: a standing fight (+10%, -10%) integrates
/// into an antisymmetric target offset that shrinks the pair's commanded
/// separation, while the pair's target midpoint stays exactly where the
/// stream put it (carriage-neutral).
#[test]
fn trim_applies_antisymmetric_target_offsets_during_streaming() {
    let mut trimmed =
        test_ctx_with_drive("trim-on", TrackingLagDrive::with_torques(vec![100, -100]));
    let mut plain =
        test_ctx_with_drive("trim-off", TrackingLagDrive::with_torques(vec![100, -100]));
    assert_eq!(trimmed.trim.set(NUM_SLAVES, 0, 1, 200_000, 500, 25_000), 0);

    for ctx in [&mut trimmed, &mut plain] {
        push_all(ctx, piece(1_000_000, 0.05, &[2.5, 2.5]));
        run_cycles(ctx, 1_000_000, 41_000_000);
    }

    let with = targets(&trimmed);
    let without = targets(&plain);
    let offset0 = i64::from(with[0]) - i64::from(without[0]);
    let offset1 = i64::from(with[1]) - i64::from(without[1]);
    assert_eq!(offset0, -offset1, "trim must be carriage-neutral");
    assert!(
        offset0 < -100,
        "positive differential fight must pull slot 0 back \
         (offsets {offset0}/{offset1})"
    );
}

/// Slot 1 mounted mirrored (negative cmd counts/mm). A mechanical fight
/// (+10%, -10% in the host frame) reads (+100, +100) raw off the drives, and
/// the antisymmetric host-frame offset must land as the SAME drive-frame
/// count delta on both slots. Getting either torque- or position-frame
/// conversion wrong flips a sign here.
#[test]
fn trim_handles_a_mirrored_pair_in_both_frames() {
    let mut trimmed = test_ctx_with_drive(
        "trim-mirror-on",
        TrackingLagDrive::with_torques(vec![100, 100]),
    );
    let mut plain = test_ctx_with_drive(
        "trim-mirror-off",
        TrackingLagDrive::with_torques(vec![100, 100]),
    );
    for ctx in [&mut trimmed, &mut plain] {
        ctx.cmd_counts_per_mm[1] = -COUNTS_PER_MM;
    }
    assert_eq!(trimmed.trim.set(NUM_SLAVES, 0, 1, 200_000, 500, 25_000), 0);

    for ctx in [&mut trimmed, &mut plain] {
        push_all(ctx, piece(1_000_000, 0.05, &[2.5, 2.5]));
        run_cycles(ctx, 1_000_000, 41_000_000);
    }

    let with = targets(&trimmed);
    let without = targets(&plain);
    let offset0 = i64::from(with[0]) - i64::from(without[0]);
    let offset1 = i64::from(with[1]) - i64::from(without[1]);
    assert_eq!(
        offset0, offset1,
        "mirrored slot gets the mechanically opposite offset, which in its \
         inverted drive frame is the same count delta"
    );
    assert!(
        offset0 < -100,
        "fight must pull the pair together: {offset0}"
    );
}

#[test]
fn trim_freezes_while_the_ring_is_dry() {
    let mut ctx = test_ctx_with_drive(
        "trim-freeze",
        TrackingLagDrive::with_torques(vec![100, -100]),
    );
    assert_eq!(ctx.trim.set(NUM_SLAVES, 0, 1, 200_000, 500, 25_000), 0);
    run_cycles(&mut ctx, 0, 40_000_000);
    assert_eq!(
        targets(&ctx),
        vec![0, 0],
        "no stream, no target writes, no trim motion"
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
version = 4
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
        pairs: vec![],
    }
}

#[test]
fn set_dynamics_model_installs_model_when_none_was_loaded() {
    let mut ctx = test_ctx("dyn-install");
    assert!(ctx.dynamics.is_none());
    super::commands::handle_set_dynamics_model(&mut ctx, 1, dynamics_msg(0.030));
    let model = ctx.dynamics.as_ref().expect("model installed");
    assert_eq!(model.n_slots, NUM_SLAVES);
    let tau = model.torque_ff(0, &[1000.0, 0.0], &[100.0, 0.0], &[0.0, 0.0]);
    let expect = 0.030 * 1000.0 + 0.004 * 100.0 + 1.0;
    assert!((tau - expect).abs() < 1e-3, "{tau} vs {expect}");
}

#[test]
fn set_dynamics_model_replaces_existing_model() {
    let mut ctx = test_ctx("dyn-replace");
    super::commands::handle_set_dynamics_model(&mut ctx, 1, dynamics_msg(0.030));
    super::commands::handle_set_dynamics_model(&mut ctx, 2, dynamics_msg(0.045));
    let model = ctx.dynamics.as_ref().expect("model installed");
    let tau = model.torque_ff(0, &[1000.0, 0.0], &[0.0, 0.0], &[0.0, 0.0]);
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
    let tau = model.torque_ff(0, &[1000.0, 0.0], &[0.0, 0.0], &[0.0, 0.0]);
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
    let tau = model.torque_ff(0, &[1000.0, 0.0], &[0.0, 0.0], &[0.0, 0.0]);
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
