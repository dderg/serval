//! The stepcompress anchor is gated on a live, converged clock record: after a
//! reflash/reconnect drops the record, anchoring must be a loud dispatch error
//! rather than a silent projection off the previous boot epoch's numbers.

use super::*;
use crate::mcu_config::{LaneKind, McuAxisConfig, StepcompressEncoder};
use host_rt::clock::{Clock, MockClock};
use host_rt::clock_regression::NON_RESONANT_GET_CLOCK_PERIOD_SECS;
use host_rt::passthrough_queue::{MAX_CLOCK_RECORD_AGE_SECS, McuHandle, PassthroughRouter};
use std::sync::Arc;
use std::sync::atomic::AtomicU64;
use std::time::Duration;

const MCU_ID: u32 = 0;
const FREQ: f64 = 168_000_000.0;

fn stepcompress_cfg() -> McuAxisConfig {
    McuAxisConfig {
        mcu_id: MCU_ID,
        axes: vec![0],
        kinematics: 0,
        max_motor_velocity: vec![200.0],
        ethercat: false,
        lane_kinds: vec![LaneKind::Pulse],
        motor_counts: vec![1],
        microstep_distance: vec![0.01],
        invert_dir: vec![false],
        stepper_oids: vec![7],
        stepcompress_sample_rate: 10_000.0,
        move_queue_slots: 128,
        step_pulse_seconds: vec![0.000_002],
        stepcompress_encoders: vec![StepcompressEncoder::HighPrecision],
        phase_sample_rate: 0.0,
        phase_ring_depth: 0,
        stepcompress_max_error_secs: 0.0,
    }
}

fn sink() -> (
    PumpSink,
    Arc<Mutex<PassthroughRouter>>,
    McuHandle,
    Arc<MockClock>,
) {
    let clock = MockClock::new();
    let mut router =
        PassthroughRouter::with_clock(Arc::clone(&clock) as Arc<dyn Clock + Send + Sync>);
    let handle = router.claim_mcu("stepcompress");
    let router = Arc::new(Mutex::new(router));
    let (tx, _rx) = crossbeam_channel::unbounded();
    let sink = PumpSink {
        transports: Arc::new(crate::axis_transport::AxisTransports::from_configs(&[])),
        router: Arc::clone(&router),
        anchor: Arc::new(Mutex::new(crate::anchor::Anchor::new())),
        mcu_configs: vec![stepcompress_cfg()],
        pump_tx: tx,
        pump_control: None,
        counter: Arc::new(AtomicU64::new(0)),
        active_drip_cohort: Arc::new(Mutex::new(None)),
        motion_history: Arc::new(Mutex::new(crate::motion_history::HistoryStore::default())),
        frontier: Arc::new(super::super::CommittedFrontier::default()),
        frozen_projection: Mutex::new(std::collections::HashMap::new()),
    };
    (sink, router, handle, clock)
}

fn publish_with_centroid_lag(
    router: &Mutex<PassthroughRouter>,
    handle: McuHandle,
    last_clock: u64,
    converged: bool,
    centroid_lag_secs: f64,
) {
    let offset_raw = host_rt::clock::monotonic_raw_secs() - centroid_lag_secs;
    router
        .lock_ok()
        .set_clock_est_rebased(handle, FREQ, offset_raw, last_clock, converged, 0.0)
        .unwrap();
}

fn publish(router: &Mutex<PassthroughRouter>, handle: McuHandle, last_clock: u64, converged: bool) {
    publish_with_centroid_lag(router, handle, last_clock, converged, 0.0);
}

fn host_now(router: &Mutex<PassthroughRouter>) -> f64 {
    router.lock_ok().host_now_secs()
}

#[test]
fn anchoring_without_any_record_is_a_loud_error() {
    let (sink, router, _handle, _clock) = sink();
    let at = host_now(&router);

    let err = sink
        .reanchor_projection(MCU_ID, at)
        .expect_err("no record must not anchor");

    assert!(matches!(
        err,
        DispatchError::ClockRecordUnusable { mcu_id: MCU_ID, .. }
    ));
    assert!(sink.frozen_projection.lock_ok().is_empty());
}

#[test]
fn anchoring_on_an_unconverged_record_is_a_loud_error() {
    let (sink, router, handle, _clock) = sink();
    publish(&router, handle, (0.2 * FREQ) as u64, false);

    let err = sink
        .reanchor_projection(MCU_ID, host_now(&router))
        .expect_err("an unconverged estimate must not anchor a step stream");

    assert!(matches!(
        err,
        DispatchError::ClockRecordUnusable { mcu_id: MCU_ID, .. }
    ));
}

#[test]
fn a_converged_record_anchors_and_projects_the_current_epoch() {
    let (sink, router, handle, _clock) = sink();
    let epoch_clock = (0.2 * FREQ) as u64;
    publish(&router, handle, epoch_clock, true);
    let at = host_now(&router);

    sink.reanchor_projection(MCU_ID, at)
        .expect("a converged record must anchor");

    let projected = sink.project(MCU_ID, at);
    assert!(
        projected.abs_diff(epoch_clock) < (FREQ * 0.050) as u64,
        "the anchor must project the current epoch's clock, got {projected} \
         against {epoch_clock}"
    );
}

/// The reflash sequence end to end: a healthy epoch anchors, the reconnect
/// invalidates the record, the next anchor fails, and only a fresh converged
/// estimate re-enables anchoring — on the new epoch's clock, not the old one.
#[test]
fn a_reconnect_blocks_anchoring_until_a_fresh_estimate_arrives() {
    let (sink, router, handle, _clock) = sink();
    let previous_epoch_clock = (14.4 * FREQ) as u64;
    publish(&router, handle, previous_epoch_clock, true);
    let at = host_now(&router);
    sink.reanchor_projection(MCU_ID, at)
        .expect("the healthy epoch anchors");
    let previous_projection = sink.project(MCU_ID, at);

    router.lock_ok().invalidate_clock_est(handle).unwrap();

    assert!(matches!(
        sink.reanchor_projection(MCU_ID, at),
        Err(DispatchError::ClockRecordUnusable { .. })
    ));

    let new_epoch_clock = (0.15 * FREQ) as u64;
    publish(&router, handle, new_epoch_clock, true);
    sink.reanchor_projection(MCU_ID, at)
        .expect("a fresh converged estimate re-enables anchoring");

    let new_projection = sink.project(MCU_ID, at);
    assert!(
        new_projection.abs_diff(new_epoch_clock) < (FREQ * 0.050) as u64,
        "the re-anchored projection must follow the new boot epoch, got \
         {new_projection} against {new_epoch_clock}"
    );
    assert!(
        previous_projection.saturating_sub(new_projection) > (FREQ * 10.0) as u64,
        "the stale epoch's projection was {previous_projection}, the fresh one \
         {new_projection} — the reflash must not carry the old uptime forward"
    );
}

/// The record that anchored the failing bench G28 sat 13.6 s behind now — but
/// that was the regression's decay centroid, not its age. A live record must
/// anchor no matter how deep the centroid lag runs, or every healthy print
/// past the first half-minute would refuse to move.
#[test]
fn a_deep_centroid_lag_does_not_block_a_live_record() {
    let (sink, router, handle, _clock) = sink();
    let epoch_clock = (14.4 * FREQ) as u64;
    publish_with_centroid_lag(&router, handle, epoch_clock, true, 13.58);

    sink.reanchor_projection(MCU_ID, host_now(&router))
        .expect("a record updated now must anchor whatever its centroid lag");
}

/// The silent failure this gate exists for: clocksync stops feeding the router,
/// the record keeps projecting off its dead estimate, and the first volley lands
/// in the MCU's past. The age must be named in the error.
#[test]
fn anchoring_on_a_record_the_router_stopped_updating_is_a_loud_error() {
    let (sink, router, handle, clock) = sink();
    publish(&router, handle, (0.2 * FREQ) as u64, true);
    let dead_for = MAX_CLOCK_RECORD_AGE_SECS + NON_RESONANT_GET_CLOCK_PERIOD_SECS;
    clock.advance(Duration::from_secs_f64(dead_for));

    let err = sink
        .reanchor_projection(MCU_ID, host_now(&router))
        .expect_err("a record no longer being updated must not anchor");

    match err {
        DispatchError::ClockRecordStale {
            mcu_id,
            age_secs,
            max_age_secs,
            ..
        } => {
            assert_eq!(mcu_id, MCU_ID);
            assert!(
                (age_secs - dead_for).abs() < 0.01,
                "the error must name the record's true age, got {age_secs} \
                 against {dead_for}"
            );
            assert_eq!(max_age_secs, MAX_CLOCK_RECORD_AGE_SECS);
        }
        other => panic!("expected a stale-record error, got {other:?}"),
    }
    assert!(sink.frozen_projection.lock_ok().is_empty());
}

/// Measured healthy sim worlds gap up to ~9 s between accepted estimates —
/// klippy's outlier rejection drops samples and a loaded reactor defers the
/// timer. A few missed samples must stay a warning, never a refusal to move.
#[test]
fn a_few_missed_samples_still_anchor() {
    let (sink, router, handle, clock) = sink();
    publish(&router, handle, (0.2 * FREQ) as u64, true);
    clock.advance(Duration::from_secs_f64(
        9.0 * NON_RESONANT_GET_CLOCK_PERIOD_SECS,
    ));

    sink.reanchor_projection(MCU_ID, host_now(&router))
        .expect("a gapped but live clocksync feed must still anchor");
}

/// Every accepted estimate has to keep the gate open: a connection's worth of
/// samples anchors at every one of them, and the anchor only fails once the
/// samples stop.
#[test]
fn a_live_stream_of_estimates_keeps_the_anchor_open_until_it_stops() {
    let (sink, router, handle, clock) = sink();
    let period = NON_RESONANT_GET_CLOCK_PERIOD_SECS;

    for sample in 1..=20u64 {
        clock.advance(Duration::from_secs_f64(period));
        let centroid_lag = (f64::from(u32::try_from(sample).unwrap())).min(30.0) * period;
        publish_with_centroid_lag(
            &router,
            handle,
            sample * (0.1 * FREQ) as u64,
            true,
            centroid_lag,
        );
        sink.reanchor_projection(MCU_ID, host_now(&router))
            .unwrap_or_else(|e| panic!("sample {sample} must anchor, got {e}"));
    }

    clock.advance(Duration::from_secs_f64(MAX_CLOCK_RECORD_AGE_SECS + period));
    assert!(matches!(
        sink.reanchor_projection(MCU_ID, host_now(&router)),
        Err(DispatchError::ClockRecordStale { .. })
    ));
}
