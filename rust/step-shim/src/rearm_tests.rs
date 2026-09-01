//! The mcu's classic stepper spends two scheduler events per step and re-arms
//! `stepper_load_next` from the pending unstep. A run whose first step lands
//! inside that window is loaded behind it — `motion.step_load_late`, then
//! "Stepper too far in past". The shim owes the caller that distance.

use std::sync::Arc;

use trajectory::{
    ClockedMotorSpan, ContinuousAxis, MotorGroup, MotorSpan, MotorTerm, NudgeProfile,
};

use super::{MotorConfig, ShimError, StepEncoder, StepFrame, StepShim};

const CYCLES_PER_SECOND: f64 = 1_000_000.0;
const MICROSTEP: f64 = 0.01;
const OID: u32 = 7;
const MIN_REARM_CYCLES: u64 = 200;
const LATTICE_OFFSET: f64 = 0.00255;
const START: u64 = 1_000;

fn cfg(min_rearm_cycles: u64) -> MotorConfig {
    MotorConfig {
        oid: OID,
        microstep_distance: MICROSTEP,
        invert_dir: false,
        cycles_per_second: CYCLES_PER_SECOND,
        encoder: StepEncoder::Classic {
            max_error_ticks: super::compress::DEFAULT_MAX_ERROR_TICKS,
        },
        min_rearm_cycles,
    }
}

fn ramp(start_clock: u64, from_mm: f64, delta_mm: f64, cycles: u64) -> ClockedMotorSpan {
    let t_start = start_clock as f64 / CYCLES_PER_SECOND;
    let t_end = t_start + cycles as f64 / CYCLES_PER_SECOND;
    let profile = NudgeProfile::try_new(delta_mm, delta_mm.abs() / (t_end - t_start), 0.0, t_start)
        .expect("a constant-velocity nudge");
    let groups: Arc<[MotorGroup]> = Arc::from([
        MotorGroup::Independent(MotorTerm {
            source_axis: 0,
            axis: ContinuousAxis::Hold {
                position: from_mm,
                t_start,
                t_end,
            },
            scale: 1.0,
        }),
        MotorGroup::Independent(MotorTerm {
            source_axis: 0,
            axis: ContinuousAxis::Nudge(profile),
            scale: 1.0,
        }),
    ]);
    let signal = Arc::new(
        MotorSpan::try_new(groups, t_start, t_end, 0, 0, false).expect("a dispatchable motor span"),
    );
    ClockedMotorSpan::try_new(
        signal,
        t_start,
        t_end,
        t_start,
        t_end,
        start_clock as f64,
        CYCLES_PER_SECOND,
    )
    .expect("a representable clocked view")
}

/// One microstep out and straight back: the reversal splits the stream into
/// two runs 151 cycles apart — inside the mcu's two-pulse re-arm window.
fn reversal() -> Vec<ClockedMotorSpan> {
    vec![
        ramp(START, LATTICE_OFFSET, MICROSTEP, 100),
        ramp(START + 100, LATTICE_OFFSET + MICROSTEP, -0.02, 200),
    ]
}

fn shim_for(min_rearm_cycles: u64) -> StepShim {
    let mut shim = StepShim::new(vec![cfg(min_rearm_cycles)], 8);
    shim.reset_position(0, 0);
    shim.push_spans(0, &reversal())
        .expect("a contiguous stream");
    shim
}

#[test]
fn a_reversal_inside_the_re_arm_window_is_refused() {
    let mut shim = shim_for(MIN_REARM_CYCLES);

    match shim
        .drain(START + 300)
        .expect_err("the mcu cannot re-arm this fast")
    {
        ShimError::StepTooSoon {
            motor,
            first,
            committed,
            min_rearm,
        } => {
            assert_eq!((motor, min_rearm), (0, MIN_REARM_CYCLES));
            assert!(
                first > committed && first - committed < MIN_REARM_CYCLES,
                "the refused run must be the one inside the window: {first} vs {committed}"
            );
        }
        other => panic!("expected StepTooSoon, got {other}"),
    }
}

/// The same reversal on a both-edge driver, which configures zero pulse ticks
/// and so owes nothing beyond strict monotonicity.
#[test]
fn a_reversal_is_emitted_when_the_mcu_needs_no_re_arm() {
    let mut shim = shim_for(0);

    let frames = shim
        .drain(START + 300)
        .expect("a zero re-arm mcu takes this stream");
    let dirs: Vec<u8> = frames
        .iter()
        .filter_map(|f| match f {
            StepFrame::SetNextStepDir { dir, .. } => Some(*dir),
            _ => None,
        })
        .collect();
    assert_eq!(dirs, vec![1, 0], "the stream must actually reverse");
    assert_eq!(shim.commanded_steps(0), 0);
}
