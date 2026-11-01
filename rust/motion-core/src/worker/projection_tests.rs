use super::*;
use crate::mcu_config::{LaneKind, McuAxisConfig, McuHardware, StepcompressEncoder};
use crate::pump::pump_past_guard_secs;
use host_rt::clock::{Clock, MockClock};
use host_rt::passthrough_queue::{McuHandle, PassthroughRouter};
use std::sync::Arc;
use std::sync::atomic::AtomicU64;
use std::time::Duration;

const MCU_ID: u32 = 0;
const F_TRUE: f64 = 168_000_000.0;
/// The mcu counter at host time 0 (the machine has been up ~3 s when klippy
/// connects — matching the bench fault at a 3.196 s counter).
const CLOCK_AT_ZERO: f64 = 500_000_000.0;

fn true_clock(host_secs: f64) -> f64 {
    CLOCK_AT_ZERO + host_secs * F_TRUE
}

fn stepcompress_cfg() -> McuAxisConfig {
    McuAxisConfig {
        mcu_id: MCU_ID,
        axes: vec![0],
        ethercat: false,
        lane_kinds: vec![LaneKind::Pulse],
        stepcompress_encoders: vec![StepcompressEncoder::HighPrecision],
        hw: McuHardware {
            kinematics: 0,
            max_motor_velocity: vec![200.0],
            motor_counts: vec![1],
            microstep_distance: vec![0.01],
            invert_dir: vec![false],
            stepper_oids: vec![7],
            move_queue_slots: 128,
            step_pulse_seconds: vec![0.000_002],
            phase_sample_rate: 0.0,
            phase_ring_depth: 0,
            stepcompress_max_error_secs: 0.0,
        },
    }
}

fn pump_sink(router: PassthroughRouter) -> Projection {
    Projection {
        transports: Arc::new(crate::axis_transport::AxisTransports::from_configs(&[])),
        router: Arc::new(Mutex::new(router)),
        anchor: Arc::new(Mutex::new(crate::anchor::Anchor::new())),
        mcu_configs: vec![stepcompress_cfg()],
        counter: Arc::new(AtomicU64::new(0)),
        frontier: Arc::new(super::super::CommittedFrontier::default()),
        frozen_projection: std::collections::HashMap::new(),
    }
}

/// A klippy-style clocksync update at `sample_host`: the record claims the mcu
/// counter read `last_clock` at host instant `offset_est`.
fn seed_clock_for(
    router: &mut PassthroughRouter,
    mcu_id: u32,
    freq: f64,
    offset_est: f64,
    last_clock: u64,
) {
    router
        .set_clock_est(McuHandle::from_raw(mcu_id), freq, offset_est, last_clock)
        .unwrap();
}

fn seed_clock(router: &mut PassthroughRouter, freq: f64, offset_est: f64, last_clock: u64) {
    seed_clock_for(router, MCU_ID, freq, offset_est, last_clock);
}

/// The host's own projection at the moment of the first anchor volley: the
/// anchor re-times the stream `DEFAULT_LEAD_SECS` ahead of the playhead, the
/// reanchor seeds the frozen projection from the live record at
/// `seam_host = host_now + lead`, and the first piece's start clock IS that
/// projection (piece u_start = 0 lands exactly on the seam).
fn first_volley_clock(sink: &mut Projection, host_now: f64) -> u64 {
    let seam_host = host_now + crate::anchor::DEFAULT_LEAD_SECS;
    sink.reanchor_projection(MCU_ID, seam_host)
        .expect("a synced clocksync must anchor the projection");
    sink.project(MCU_ID, seam_host)
}

/// The endpoint's view at egress: the live ack clock the flush guard compares
/// every frame's start clock against.
fn egress_guard_passes(router: &PassthroughRouter, first_clock: u64, freq: f64) -> bool {
    let (live_now, live_freq) = router
        .ack_clock_and_freq(McuHandle::from_raw(MCU_ID))
        .expect("synced");
    let guard_ticks = (pump_past_guard_secs() * live_freq) as u64;
    let _ = freq;
    first_clock.saturating_add(guard_ticks) >= live_now
}

/// Hypothesis 1 as stated — "the first-volley clock is stale because the
/// host→mcu projection is mis-anchored on real serial hardware" — with a
/// HEALTHY clocksync: the estimate has converged to within 50 ppm of the true
/// crystal and 1 ms of the true sample instant, samples ~1 s apart (klippy's
/// 0.9839 s get_clock cadence). Under those conditions the first-volley clock
/// must land ~SEND_LEAD_SECONDS ahead of the true mcu clock at arrival — NOT
/// in the past. This is the honest refutation of the projection-math variant
/// of hypothesis 1: no realistic clocksync error puts the first volley 18 ms
/// in the past, because the anchor grants a full 500 ms of lead and the
/// projection error sources are all sub-millisecond at this uptime.
#[test]
fn a_healthy_clocksync_lands_the_first_volley_lead_seconds_ahead_of_the_true_clock() {
    let clock = MockClock::new();
    let mut router =
        PassthroughRouter::with_clock(Arc::clone(&clock) as Arc<dyn Clock + Send + Sync>);
    router.claim_mcu("stepcompress");

    // klippy connect at t=0.5, then periodic updates; the estimator's freq
    // has converged to +50 ppm, the offset to +1 ms. The host clock walks with
    // the samples so the record's age is the real one — the anchor refuses a
    // record clocksync stopped refreshing.
    let freq_est = F_TRUE * (1.0 + 50e-6);
    let mut host_at = 0.0;
    for (sample_host, offset_err) in [(0.5, 0.0), (1.5, 0.0005), (2.5, 0.001)] {
        clock.advance(Duration::from_secs_f64(sample_host - host_at));
        host_at = sample_host;
        seed_clock(
            &mut router,
            freq_est,
            sample_host + offset_err,
            true_clock(sample_host) as u64,
        );
    }

    // The first G28 X anchors 3.2 s after the mcu came up (the bench fault
    // fired at counter 536880068 = 3.196 s of uptime).
    clock.advance(Duration::from_secs_f64(3.2 - host_at));
    let host_now = router.host_now_secs();
    let mut sink = pump_sink(router);
    let first_clock = first_volley_clock(&mut sink, host_now);
    let live_freq = sink
        .router
        .lock_ok()
        .ack_clock_and_freq(McuHandle::from_raw(MCU_ID))
        .unwrap()
        .1;

    // Egress a few ms after the anchor; the volley arrives after the serial
    // RTT. The host's own guard (same live record) must pass.
    clock.advance(Duration::from_millis(2));
    assert!(
        egress_guard_passes(&sink.router.lock_ok(), first_clock, live_freq),
        "the first volley must clear the egress guard: clock {first_clock} vs projected now"
    );

    let pipeline = 0.002;
    let rtt = 0.002;
    let true_at_arrival = true_clock(host_now + pipeline + rtt);
    let deficit_secs = (first_clock as f64 - true_at_arrival) / F_TRUE;
    assert!(
        deficit_secs > 0.0,
        "the first-volley clock must be in the FUTURE at arrival, got {deficit_secs:.6} s \
         (first_clock {first_clock}, true mcu at arrival {true_at_arrival:.0})"
    );
    assert!(
        (deficit_secs - (crate::anchor::DEFAULT_LEAD_SECS - pipeline - rtt)).abs() < 0.005,
        "the first-volley lead must be ~SEND_LEAD_SECONDS - wire time, got {deficit_secs:.6} s"
    );
}

/// The reproduction: a clock record that LAGS the true mcu counter by
/// `DEFAULT_LEAD_SECS + 18.3 ms` of ticks puts the first volley exactly
/// ~18.3 ms in the past of the true mcu clock at arrival — the bench's
/// "Rescheduled timer in the past" diff_us -18325 signature — while every
/// host-side guard (pump intake, pump send, endpoint flush) passes, because
/// all of them derive from the SAME lagging record. This is the honest
/// statement of hypothesis 1's reach: the failure is not the projection math
/// (the anchor grants 500 ms of lead) but a wrong clock record — a clocksync
/// handoff error (stale `clock` value, bad RTT stamp, unconverged estimate) —
/// and that failure is invisible to every host guard by construction.
#[test]
fn a_clock_record_lagging_the_true_mcu_puts_the_first_volley_past_and_blinds_the_guards() {
    let lag_secs = crate::anchor::DEFAULT_LEAD_SECS + 0.0183;
    let clock = MockClock::new();
    let mut router =
        PassthroughRouter::with_clock(Arc::clone(&clock) as Arc<dyn Clock + Send + Sync>);
    router.claim_mcu("stepcompress");

    // The last published (offset, clock) pair: the offset is the true sample
    // instant, but the clock value is lag_secs of ticks behind what the
    // counter really read there.
    let sample_host = 2.5;
    clock.advance(Duration::from_secs_f64(sample_host));
    seed_clock(
        &mut router,
        F_TRUE,
        sample_host,
        (true_clock(sample_host) - lag_secs * F_TRUE) as u64,
    );

    clock.advance(Duration::from_secs_f64(3.2 - sample_host));
    let host_now = router.host_now_secs();
    let mut sink = pump_sink(router);
    let first_clock = first_volley_clock(&mut sink, host_now);
    let live_freq = sink
        .router
        .lock_ok()
        .ack_clock_and_freq(McuHandle::from_raw(MCU_ID))
        .unwrap()
        .1;

    clock.advance(Duration::from_millis(2));
    assert!(
        egress_guard_passes(&sink.router.lock_ok(), first_clock, live_freq),
        "the guard is blind to a uniformly lagging clock record — the bench crash \
         passes through it"
    );

    let pipeline = 0.002;
    let rtt = 0.002;
    let true_at_arrival = true_clock(host_now + pipeline + rtt);
    let deficit_secs = (first_clock as f64 - true_at_arrival) / F_TRUE;
    assert!(
        deficit_secs <= -0.018,
        "the reproduction must put the first volley at least 18 ms in the past of the true \
         mcu clock at arrival, got {deficit_secs:.6} s"
    );
    assert!(
        deficit_secs >= -0.030,
        "the modeled lag must reproduce the ~18 ms signature, got {deficit_secs:.6} s \
         (the model overshot)"
    );
}

fn two_mcu_sink(router: PassthroughRouter) -> Projection {
    let mut sink = pump_sink(router);
    let mut second = stepcompress_cfg();
    second.mcu_id = 1;
    second.axes = vec![1];
    second.hw.stepper_oids = vec![8];
    sink.mcu_configs.push(second);
    sink
}

/// Both mcus' records claim the counter read `true_clock(host)` at `host`,
/// with the estimator's current frequency guess.
fn seed_both(sink: &Projection, host: f64, freq_est: f64) {
    let mut router = sink.router.lock_ok();
    for mcu_id in [0, 1] {
        seed_clock_for(&mut router, mcu_id, freq_est, host, true_clock(host) as u64);
    }
}

fn seam_error_secs(sink: &Projection, mcu_id: u32, seam_host: f64) -> f64 {
    (sink.project_exact(mcu_id, seam_host) - true_clock(seam_host)) / F_TRUE
}

/// The frozen slope carries the estimator's error at anchor time for as long
/// as it stays frozen: 400 ppm over the first minute is 24 ms. A dwell's
/// rejoin starts every lane from rest, so both mcus — including one whose
/// first segment after the dwell would only hold — must re-sync there.
#[test]
fn a_dwell_resyncs_every_mcu_on_the_live_clock() {
    let clock = MockClock::new();
    let mut router =
        PassthroughRouter::with_clock(Arc::clone(&clock) as Arc<dyn Clock + Send + Sync>);
    router.claim_mcu("main");
    router.claim_mcu("toolhead");
    let mut sink = two_mcu_sink(router);
    seed_both(&sink, 0.0, F_TRUE * (1.0 + 400e-6));
    let first = sink.anchor_segment(0.0, 100.0, false).unwrap();
    assert_eq!(first.epoch, crate::anchor::StreamEpoch::Reposition);

    clock.advance(Duration::from_secs_f64(60.0));
    seed_both(&sink, 60.0, F_TRUE);
    sink.mark_parked();
    let dwell_end = 101.0;
    let at = sink.anchor_segment(dwell_end, 102.0, false).unwrap();
    assert_eq!(at.epoch, crate::anchor::StreamEpoch::Rejoin);

    let seam = at.t0 + dwell_end;
    for mcu_id in [0, 1] {
        let err = seam_error_secs(&sink, mcu_id, seam);
        assert!(
            err.abs() < 2.0 / F_TRUE,
            "mcu {mcu_id} must resume on the live clock after the dwell, off by {err:.6} s"
        );
    }
}

/// A correction that would pull the resume earlier than the rest began
/// schedules it before the previous stream ended; it waits for a rest long
/// enough to absorb it rather than landing inside motion.
#[test]
fn a_correction_longer_than_the_rest_waits_for_a_longer_rest() {
    let clock = MockClock::new();
    let mut router =
        PassthroughRouter::with_clock(Arc::clone(&clock) as Arc<dyn Clock + Send + Sync>);
    router.claim_mcu("main");
    router.claim_mcu("toolhead");
    let mut sink = two_mcu_sink(router);
    seed_both(&sink, 0.0, F_TRUE * (1.0 + 400e-6));
    sink.anchor_segment(0.0, 100.0, false).unwrap();
    let frozen_before = sink.project_exact(1, 200.0);

    clock.advance(Duration::from_secs_f64(60.0));
    seed_both(&sink, 60.0, F_TRUE);
    sink.mark_parked();
    let short_dwell_end = 100.01;
    let at = sink.anchor_segment(short_dwell_end, 102.0, false).unwrap();
    assert_eq!(at.epoch, crate::anchor::StreamEpoch::Rejoin);

    assert_eq!(
        sink.project_exact(1, 200.0),
        frozen_before,
        "a 10 ms rest cannot absorb a ~40 ms backward correction"
    );
}

/// The fix: a reanchor re-seeds the frozen projection from the LIVE clock
/// record instead of chaining from the previous frozen slope. The old
/// chained value carries `freq_error * elapsed` of drift — 120 ms after a
/// 10-minute park with a 200 ppm estimate — and the first clock of the new
/// epoch would land that far behind the true mcu.
#[test]
fn a_reanchor_reseeds_from_the_live_clock_not_the_drifted_frozen_slope() {
    let clock = MockClock::new();
    let mut router =
        PassthroughRouter::with_clock(Arc::clone(&clock) as Arc<dyn Clock + Send + Sync>);
    router.claim_mcu("stepcompress");

    // Anchor 1: the clocksync estimate is 200 ppm low.
    let freq_est1 = F_TRUE * (1.0 - 200e-6);
    seed_clock(&mut router, freq_est1, 1.0, true_clock(1.0) as u64);
    clock.advance(Duration::from_secs_f64(1.0));
    let mut sink = pump_sink(router);
    let seam1 = sink.router.lock_ok().host_now_secs() + crate::anchor::DEFAULT_LEAD_SECS;
    sink.reanchor_projection(MCU_ID, seam1).unwrap();

    // Ten minutes of park; the live estimate has converged to the true freq.
    clock.advance(Duration::from_secs_f64(600.0));
    let freq_est2 = F_TRUE;
    let host_after_park = sink.router.lock_ok().host_now_secs();
    seed_clock(
        &mut sink.router.lock_ok(),
        freq_est2,
        host_after_park,
        true_clock(host_after_park) as u64,
    );

    let host_now = sink.router.lock_ok().host_now_secs();
    let seam2 = host_now + crate::anchor::DEFAULT_LEAD_SECS;
    let chained_would_be = sink
        .frozen_projection
        .get(&MCU_ID)
        .copied()
        .expect("anchor 1 must have frozen a projection")
        .project_exact(seam2);
    let live_now_at_reanchor = sink
        .router
        .lock_ok()
        .host_time_to_mcu_clock(McuHandle::from_raw(MCU_ID), seam2)
        .unwrap();

    let drift_secs = (chained_would_be as f64 - live_now_at_reanchor as f64) / F_TRUE;
    assert!(
        drift_secs < -0.100,
        "the chained slope must have drifted ~120 ms over the 600 s park, got {drift_secs:.6} s"
    );

    sink.reanchor_projection(MCU_ID, seam2).unwrap();
    let first_clock = sink.project(MCU_ID, seam2);
    let true_at_seam = true_clock(seam2);
    let err_secs = (first_clock as f64 - true_at_seam) / F_TRUE;
    assert!(
        err_secs.abs() < 0.001,
        "the reanchored first clock must land on the live record (within 1 ms of the true \
         clock), got {err_secs:.6} s of error — the chained drift must not survive"
    );
}
