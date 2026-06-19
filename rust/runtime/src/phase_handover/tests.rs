#![allow(clippy::indexing_slicing, clippy::unwrap_used)]

use super::*;
use crate::state::SharedState;
use crate::stepping_state::{AxisState, StepMode, StepperRef};
use core::sync::atomic::Ordering;

fn axis_with_stepper(mode: StepMode, oid: u8) -> AxisState {
    let mut axis = AxisState::new_unconfigured();
    axis.mode.store(mode as u8, Ordering::Release);
    axis.microstep_distance = 0.000_625;
    axis.steppers.push(StepperRef::new(oid, Some(7))).unwrap();
    axis
}

#[test]
fn shortest_delta_forward() {
    assert_eq!(shortest_phase_delta(10, 44), 34);
}

#[test]
fn shortest_delta_wraps_backward() {
    // 1000 -> 10 is +34 through the wrap, not -990.
    assert_eq!(shortest_phase_delta(1000, 10), 34);
}

#[test]
fn shortest_delta_wraps_forward_negative() {
    assert_eq!(shortest_phase_delta(10, 1000), -34);
}

#[test]
fn shortest_delta_zero() {
    assert_eq!(shortest_phase_delta(512, 512), 0);
}

#[test]
fn shortest_delta_halfway_is_positive() {
    assert_eq!(shortest_phase_delta(0, 512), 512);
}

#[test]
fn find_stepper_locates_by_oid_across_axes() {
    let mut axes: [Option<AxisState>; 4] = [const { None }; 4];
    axes[0] = Some(axis_with_stepper(StepMode::Phase, 3));
    axes[2] = Some(axis_with_stepper(StepMode::Pulse, 9));
    let (axis_idx, _, stepper) = find_stepper(&axes, 9).unwrap();
    assert_eq!(axis_idx, 2);
    assert_eq!(stepper.stepper_oid, 9);
    assert!(find_stepper(&axes, 99).is_none());
}

#[test]
fn align_to_sets_both_offsets_and_matches_target_phase() {
    let mut axes: [Option<AxisState>; 4] = [const { None }; 4];
    let mut axis = axis_with_stepper(StepMode::Pulse, 5);
    axis.last_step_count = 70_000; // 70000 & 0x3FF = 368
    axes[1] = Some(axis);
    assert_eq!(align_to(&axes, 5, 100), 0);
    let axis = axes[1].as_ref().unwrap();
    let stepper = &axis.steppers[0];
    let off = stepper.phase_offset_microsteps.load(Ordering::Acquire);
    assert_eq!(off, stepper.phase_offset_target.load(Ordering::Acquire));
    assert_eq!((axis.last_step_count.wrapping_add(off)) & 0x3FF, 100);
    // Shortest path: |delta| <= 512.
    assert!(off.abs() <= 512);
}

#[test]
fn align_to_rejects_unknown_oid_and_bad_phase() {
    let axes: [Option<AxisState>; 4] = [const { None }; 4];
    assert_ne!(align_to(&axes, 5, 100), 0);
    let mut axes2: [Option<AxisState>; 4] = [const { None }; 4];
    axes2[0] = Some(axis_with_stepper(StepMode::Pulse, 5));
    assert_ne!(align_to(&axes2, 5, 1024), 0);
}

#[test]
fn jog_to_moves_offset_target_by_shortest_path_requires_phase_mode() {
    let mut axes: [Option<AxisState>; 4] = [const { None }; 4];
    let mut axis = axis_with_stepper(StepMode::Phase, 5);
    axis.last_step_count = 1020; // phase 1020
    axes[0] = Some(axis);
    let shared = SharedState::new();
    assert_eq!(jog_to(&axes, &shared, 5, 4, 1), 0);
    let stepper = &axes[0].as_ref().unwrap().steppers[0];
    // 1020 -> 4 is +8 through the wrap.
    assert_eq!(stepper.phase_offset_target.load(Ordering::Acquire), 8);
    assert_eq!(
        shared
            .max_phase_offset_ramp_per_sample
            .load(Ordering::Acquire),
        1
    );
    // Pulse mode is refused.
    axes[0]
        .as_ref()
        .unwrap()
        .mode
        .store(StepMode::Pulse as u8, Ordering::Release);
    assert_ne!(jog_to(&axes, &shared, 5, 4, 1), 0);
}

#[test]
fn jog_to_composes_with_pending_target_not_current_offset() {
    let mut axes: [Option<AxisState>; 4] = [const { None }; 4];
    let axis = axis_with_stepper(StepMode::Phase, 5);
    axes[0] = Some(axis);
    let shared = SharedState::new();
    {
        let stepper = &axes[0].as_ref().unwrap().steppers[0];
        stepper.phase_offset_target.store(100, Ordering::Release);
        stepper.phase_offset_microsteps.store(40, Ordering::Release);
    }
    // last_step_count = 0, pending phase = 100; jog to 110 adds +10 on top
    // of the pending target, not on the in-flight current offset.
    assert_eq!(jog_to(&axes, &shared, 5, 110, 1), 0);
    let stepper = &axes[0].as_ref().unwrap().steppers[0];
    assert_eq!(stepper.phase_offset_target.load(Ordering::Acquire), 110);
}

fn phase_axis_at(oid: u8, last_step_count: i32) -> AxisState {
    let mut axis = axis_with_stepper(StepMode::Pulse, oid);
    axis.last_step_count = last_step_count;
    axis
}

#[test]
fn motor_idx_for_matches_jth_tmc_cs_stepper_on_axis() {
    use crate::state::bind_phase_motor;
    let mut axes: [Option<AxisState>; 4] = [const { None }; 4];
    let mut axis = AxisState::new_unconfigured();
    axis.steppers.push(StepperRef::new(10, Some(7))).unwrap();
    axis.steppers.push(StepperRef::new(11, Some(8))).unwrap();
    axes[1] = Some(axis);
    let shared = SharedState::new();
    bind_phase_motor(&shared, 0, 1).unwrap();
    bind_phase_motor(&shared, 1, 1).unwrap();
    let axis = axes[1].as_ref().unwrap();
    assert_eq!(motor_idx_for(&shared, 1, axis, &axis.steppers[0]), Some(0));
    assert_eq!(motor_idx_for(&shared, 1, axis, &axis.steppers[1]), Some(1));
}

#[test]
fn enter_group_rejects_motion_active() {
    let _g = crate::test_xdirect_capture::lock_for_test();
    let mut axes: [Option<AxisState>; 4] = [const { None }; 4];
    let mut axis = phase_axis_at(5, 0);
    axis.armed = Some(crate::motion_core::ArmedPiece {
        mono_coeffs: [0.0; 4],
        vel_coeffs: [0.0; 3],
        piece_start_cycles: 0,
        piece_end_cycles: 0,
    });
    axes[0] = Some(axis);
    let shared = SharedState::new();
    crate::state::bind_phase_motor(&shared, 0, 0).unwrap();
    assert_eq!(
        enter_group(&axes, &shared, 0, &[5]),
        FaultCode::MotionInProgress.as_i32()
    );
    assert_ne!(
        axes[0].as_ref().unwrap().mode.load(Ordering::Acquire),
        StepMode::Phase as u8
    );
}

#[test]
fn enter_group_rejects_chopconf_toff_zero() {
    let _g = crate::test_xdirect_capture::lock_for_test();
    let mut axes: [Option<AxisState>; 4] = [const { None }; 4];
    axes[0] = Some(phase_axis_at(5, 0));
    let shared = SharedState::new();
    crate::state::bind_phase_motor(&shared, 0, 0).unwrap();
    crate::phase_spi::test_clear();
    crate::phase_spi::test_set_register(0, crate::phase_spi::CHOPCONF_ADDR, 0); // toff = 0
    assert_eq!(
        enter_group(&axes, &shared, 0, &[5]),
        FaultCode::PhaseEnterPreconditionFailed.as_i32()
    );
    assert_ne!(
        axes[0].as_ref().unwrap().mode.load(Ordering::Acquire),
        StepMode::Phase as u8
    );
}

#[test]
fn enter_group_caches_mscnt_sets_direct_mode_and_flips_mode() {
    use crate::phase_spi::{self, GCONF_ADDR, GCONF_DIRECT_MODE, GCONF_EN_PWM};
    let _g = crate::test_xdirect_capture::lock_for_test();
    let mut axes: [Option<AxisState>; 4] = [const { None }; 4];
    axes[0] = Some(phase_axis_at(5, 70_000));
    let shared = SharedState::new();
    crate::state::bind_phase_motor(&shared, 0, 0).unwrap();
    phase_spi::test_clear();
    phase_spi::test_set_register(0, phase_spi::CHOPCONF_ADDR, 0x3); // toff = 3
    phase_spi::test_set_register(0, GCONF_ADDR, GCONF_EN_PWM); // pre-enter: en_pwm on
    phase_spi::test_set_mscnt(0, 137);

    assert_eq!(enter_group(&axes, &shared, 0, &[5]), 0);

    let axis = axes[0].as_ref().unwrap();
    let stepper = &axis.steppers[0];
    assert_eq!(axis.mode.load(Ordering::Acquire), StepMode::Phase as u8);
    assert_eq!(stepper.phase_enter_mscnt.load(Ordering::Acquire), 137);
    assert_eq!(
        stepper.phase_enter_gconf.load(Ordering::Acquire),
        GCONF_EN_PWM
    );
    // chip GCONF now has direct_mode set, en_pwm cleared.
    let gconf = phase_spi::test_get_register(0, GCONF_ADDR);
    assert_eq!(gconf & GCONF_DIRECT_MODE, GCONF_DIRECT_MODE);
    assert_eq!(gconf & GCONF_EN_PWM, 0);
    // offset aligned so the step-generator phase equals the read MSCNT.
    let off = stepper.phase_offset_microsteps.load(Ordering::Acquire);
    assert_eq!((axis.last_step_count.wrapping_add(off)) & 0x3FF, 137);
    // ordering: CHOPCONF read precedes the GCONF direct_mode write.
    let ops = phase_spi::test_ops();
    let i_chop = ops
        .iter()
        .position(
            |o| matches!(o, phase_spi::Op::Read { addr, .. } if *addr == phase_spi::CHOPCONF_ADDR),
        )
        .unwrap();
    let i_rmw = ops
        .iter()
        .position(|o| matches!(o, phase_spi::Op::Rmw { addr, .. } if *addr == GCONF_ADDR))
        .unwrap();
    assert!(
        i_chop < i_rmw,
        "CHOPCONF must be read before direct_mode is set"
    );
}

#[test]
fn exit_round_trip_restores_gconf_and_returns_to_pulse() {
    use crate::phase_spi::{self, GCONF_ADDR, GCONF_DIRECT_MODE, GCONF_EN_PWM};
    let _g = crate::test_xdirect_capture::lock_for_test();
    let mut axes: [Option<AxisState>; 4] = [const { None }; 4];
    axes[0] = Some(phase_axis_at(5, 70_000));
    let shared = SharedState::new();
    crate::state::bind_phase_motor(&shared, 0, 0).unwrap();
    phase_spi::test_clear();
    phase_spi::test_set_register(0, phase_spi::CHOPCONF_ADDR, 0x3);
    phase_spi::test_set_register(0, GCONF_ADDR, GCONF_EN_PWM);
    phase_spi::test_set_mscnt(0, 200);
    assert_eq!(enter_group(&axes, &shared, 0, &[5]), 0);

    // No further motion: offset target already equals current (settled at 200).
    assert_eq!(exit_begin_group(&axes, &shared, &[5]), 0);
    assert_eq!(exit_finalize_group(&axes, &shared, &[5]), 0);

    let axis = axes[0].as_ref().unwrap();
    assert_eq!(axis.mode.load(Ordering::Acquire), StepMode::Pulse as u8);
    // GCONF restored to pre-enter value: direct_mode clear, en_pwm back on.
    let gconf = phase_spi::test_get_register(0, GCONF_ADDR);
    assert_eq!(gconf & GCONF_DIRECT_MODE, 0);
    assert_eq!(gconf & GCONF_EN_PWM, GCONF_EN_PWM);
}

#[test]
fn exit_finalize_rejects_when_not_settled() {
    use crate::phase_spi::{self, GCONF_ADDR, GCONF_EN_PWM};
    let _g = crate::test_xdirect_capture::lock_for_test();
    let mut axes: [Option<AxisState>; 4] = [const { None }; 4];
    axes[0] = Some(phase_axis_at(5, 70_000));
    let shared = SharedState::new();
    crate::state::bind_phase_motor(&shared, 0, 0).unwrap();
    phase_spi::test_clear();
    phase_spi::test_set_register(0, phase_spi::CHOPCONF_ADDR, 0x3);
    phase_spi::test_set_register(0, GCONF_ADDR, GCONF_EN_PWM);
    phase_spi::test_set_mscnt(0, 200);
    assert_eq!(enter_group(&axes, &shared, 0, &[5]), 0);
    // Force an outstanding ramp so the offset != target (not settled).
    axes[0].as_ref().unwrap().steppers[0]
        .phase_offset_target
        .fetch_add(9, Ordering::AcqRel);
    assert_eq!(
        exit_finalize_group(&axes, &shared, &[5]),
        FaultCode::PhaseExitDesync.as_i32()
    );
}

#[test]
fn query_reports_phase_mode_and_settled() {
    let mut axes: [Option<AxisState>; 4] = [const { None }; 4];
    let mut axis = axis_with_stepper(StepMode::Phase, 5);
    axis.last_step_count = 2048; // phase 0
    axes[3] = Some(axis);
    {
        let stepper = &axes[3].as_ref().unwrap().steppers[0];
        stepper.phase_offset_microsteps.store(5, Ordering::Release);
        stepper.phase_offset_target.store(5, Ordering::Release);
    }
    let q = query(&axes, 5).unwrap();
    assert_eq!(q.axis_idx, 3);
    assert_eq!(q.mode, StepMode::Phase as u8);
    assert_eq!(q.phase, 5);
    assert!(q.settled);
    axes[3].as_ref().unwrap().steppers[0]
        .phase_offset_target
        .store(9, Ordering::Release);
    assert!(!query(&axes, 5).unwrap().settled);
}
