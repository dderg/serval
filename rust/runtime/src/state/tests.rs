use super::*;

#[test]
fn bind_phase_motor_installs_slot_and_grows_count() {
    let shared = SharedState::new();
    assert_eq!(super::bind_phase_motor(&shared, 0, 2), Ok(()));
    assert_eq!(super::bind_phase_motor(&shared, 1, 2), Ok(()));
    use core::sync::atomic::Ordering;
    assert_eq!(shared.phase_slot_idx[0].load(Ordering::Acquire), 2);
    assert_eq!(shared.phase_slot_idx[1].load(Ordering::Acquire), 2);
    assert_eq!(shared.phase_motor_count.load(Ordering::Acquire), 2);
    assert_eq!(
        shared.step_modes[2].load(Ordering::Acquire),
        super::StepMode::Modulated as u8,
        "binding a motor marks its kinematic slot Modulated"
    );
}

#[test]
fn bind_phase_motor_is_idempotent_on_count() {
    let shared = SharedState::new();
    assert_eq!(super::bind_phase_motor(&shared, 1, 0), Ok(()));
    assert_eq!(super::bind_phase_motor(&shared, 0, 1), Ok(()));
    use core::sync::atomic::Ordering;
    assert_eq!(
        shared.phase_motor_count.load(Ordering::Acquire),
        2,
        "count is max(motor_idx)+1, not number of calls"
    );
}

#[test]
fn bind_phase_motor_rejects_out_of_range() {
    let shared = SharedState::new();
    assert_eq!(
        super::bind_phase_motor(&shared, super::MAX_STEPPER_OIDS as u8, 0),
        Err(super::SetStepModeError::OutOfRange)
    );
    assert_eq!(
        super::bind_phase_motor(&shared, 0, crate::stepping_state::MAX_AXES as u8),
        Err(super::SetStepModeError::OutOfRange)
    );
}
