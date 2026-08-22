#![cfg(feature = "sample-stepping")]

use core::sync::atomic::Ordering;

use runtime::engine::Engine;
use runtime::sample_run::encode_deltas;
use runtime::state::SharedState;
use runtime::stepping_state::{MAX_AXES, StepMode, StepperBindingRust, TMC_CS_OID_NONE};

const INTERVAL: u32 = 1_000;
const ANCHOR: u64 = 1_000_000;
const RUN_SAMPLES: usize = 4;
const RUN_SPAN: u64 = INTERVAL as u64 * RUN_SAMPLES as u64;

fn new_engine() -> Engine {
    Engine::new(520_000_000, 40_000)
}

fn phase_binding(oid: u8) -> StepperBindingRust {
    StepperBindingRust {
        stepper_oid: oid,
        tmc_cs_oid: TMC_CS_OID_NONE,
        _pad: [0; 2],
    }
}

/// A run that holds position, so the ring draining never looks like an
/// underrun mid-motion.
fn push_hold_run(engine: &mut Engine, shared: &SharedState, oid: u8) {
    let mut buf = [0u8; 64];
    let len = encode_deltas(0, &[0i32; RUN_SAMPLES], &mut buf).expect("encodes");
    #[allow(clippy::cast_possible_truncation)]
    engine.sample_push_run(shared, oid, INTERVAL, RUN_SAMPLES as u8, &buf[..len]);
}

#[test]
fn unconfigured_axes_report_zero_occupancy() {
    let e = new_engine();
    assert_eq!(e.occupancy_counts(), [0u32; MAX_AXES]);
    assert_eq!(e.retired_counts(), [0u32; MAX_AXES]);
}

#[test]
fn empty_configured_axis_reports_zero_occupancy() {
    let mut e = new_engine();
    assert_eq!(
        e.configure_axis(0, StepMode::Phase, 0.01, &[phase_binding(0)]),
        0
    );
    assert_eq!(e.occupancy_counts()[0], 0);
    assert_eq!(e.retired_counts()[0], 0);
}

#[test]
fn occupancy_and_retirement_track_the_sample_run_ring() {
    let mut e = new_engine();
    let shared = SharedState::new();
    assert_eq!(
        e.configure_axis(0, StepMode::Phase, 0.01, &[phase_binding(0)]),
        0
    );

    e.sample_anchor(&shared, 0, ANCHOR, 0);
    for _ in 0..3 {
        push_hold_run(&mut e, &shared, 0);
    }
    assert_eq!(e.occupancy_counts()[0], 3);
    assert_eq!(e.retired_counts()[0], 0);

    e.tick(ANCHOR + RUN_SPAN, &shared);
    assert_eq!(e.occupancy_counts()[0], 2);
    assert_eq!(e.retired_counts()[0], 1);

    e.tick(ANCHOR + 2 * RUN_SPAN, &shared);
    assert_eq!(e.occupancy_counts()[0], 1);
    assert_eq!(e.retired_counts()[0], 2);

    assert_eq!(shared.last_error.load(Ordering::Acquire), 0);
}

#[test]
fn retired_counts_survive_a_re_anchor() {
    let mut e = new_engine();
    let shared = SharedState::new();
    assert_eq!(
        e.configure_axis(0, StepMode::Phase, 0.01, &[phase_binding(0)]),
        0
    );

    e.sample_anchor(&shared, 0, ANCHOR, 0);
    push_hold_run(&mut e, &shared, 0);
    e.tick(ANCHOR + RUN_SPAN, &shared);
    assert_eq!(e.retired_counts()[0], 1);

    e.sample_anchor(&shared, 0, ANCHOR + 10 * RUN_SPAN, 0);
    assert_eq!(e.occupancy_counts()[0], 0);
    assert_eq!(
        e.retired_counts()[0],
        1,
        "retirement is a monotonic watermark, not a ring length"
    );
}

#[test]
fn unconfigured_slots_remain_zero_when_one_axis_configured() {
    let mut e = new_engine();
    let shared = SharedState::new();
    assert_eq!(
        e.configure_axis(2, StepMode::Phase, 0.01, &[phase_binding(0)]),
        0
    );

    e.sample_anchor(&shared, 0, ANCHOR, 0);
    push_hold_run(&mut e, &shared, 0);

    let occ = e.occupancy_counts();
    assert_eq!(occ[0], 0);
    assert_eq!(occ[1], 0);
    assert_eq!(occ[2], 1);
    for slot in occ.iter().skip(3) {
        assert_eq!(*slot, 0);
    }
}
