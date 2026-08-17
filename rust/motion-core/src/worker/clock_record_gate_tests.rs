//! The stepcompress anchor is gated on a live, converged clock record: after a
//! reflash/reconnect drops the record, anchoring must be a loud dispatch error
//! rather than a silent projection off the previous boot epoch's numbers.

use super::*;
use crate::mcu_config::{McuAxisConfig, StepcompressEncoder, SteppingMode};
use host_rt::clock::{Clock, MockClock};
use host_rt::passthrough_queue::{McuHandle, PassthroughRouter};
use std::sync::Arc;
use std::sync::atomic::AtomicU64;

const MCU_ID: u32 = 0;
const FREQ: f64 = 168_000_000.0;

fn stepcompress_cfg() -> McuAxisConfig {
    McuAxisConfig {
        mcu_id: MCU_ID,
        axes: vec![0],
        kinematics: 0,
        caps: Default::default(),
        max_motor_velocity: vec![200.0],
        ethercat: false,
        stepping_mode: SteppingMode::Stepcompress,
        microstep_distance: vec![0.01],
        invert_dir: vec![false],
        stepper_oids: vec![7],
        stepcompress_sample_rate: 10_000.0,
        move_queue_slots: 128,
        step_pulse_seconds: vec![0.000_002],
        stepcompress_encoder: StepcompressEncoder::HighPrecision,
        stepcompress_max_error_secs: 0.0,
    }
}

fn sink() -> (PumpSink, Arc<Mutex<PassthroughRouter>>, McuHandle) {
    let clock = MockClock::new();
    let mut router =
        PassthroughRouter::with_clock(Arc::clone(&clock) as Arc<dyn Clock + Send + Sync>);
    let handle = router.claim_mcu("stepcompress");
    let router = Arc::new(Mutex::new(router));
    let (tx, _rx) = crossbeam_channel::unbounded();
    let sink = PumpSink {
        router: Arc::clone(&router),
        anchor: Arc::new(Mutex::new(crate::anchor::Anchor::new())),
        mcu_configs: vec![stepcompress_cfg()],
        pump_tx: tx,
        counter: Arc::new(AtomicU64::new(0)),
        active_drip_cohort: Arc::new(Mutex::new(None)),
        motion_history: Arc::new(Mutex::new(crate::motion_history::HistoryStore::default())),
        frontier: Arc::new(super::super::CommittedFrontier::default()),
        frozen_projection: Mutex::new(std::collections::HashMap::new()),
    };
    (sink, router, handle)
}

fn publish(router: &Mutex<PassthroughRouter>, handle: McuHandle, last_clock: u64, converged: bool) {
    let offset_raw = host_rt::clock::monotonic_raw_secs();
    router
        .lock_ok()
        .set_clock_est_rebased(handle, FREQ, offset_raw, last_clock, converged, 0.0)
        .unwrap();
}

fn host_now(router: &Mutex<PassthroughRouter>) -> f64 {
    router.lock_ok().host_now_secs()
}

#[test]
fn anchoring_without_any_record_is_a_loud_error() {
    let (sink, router, _handle) = sink();
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
    let (sink, router, handle) = sink();
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
    let (sink, router, handle) = sink();
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
    let (sink, router, handle) = sink();
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
