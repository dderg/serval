use runtime::stepping_state::{AxisState, MAX_AXES, MAX_STEPPERS_PER_AXIS, StepMode, StepperRef};

#[test]
fn step_mode_discriminants_are_stable() {
    // Phase-stepping ISR stores `StepMode as u8` into an AtomicU8; the
    // numeric values are load-bearing across the C/Rust boundary.
    assert_eq!(StepMode::Pulse as u8, 0);
    assert_eq!(StepMode::Phase as u8, 1);
}

#[test]
fn constants_match_spec() {
    assert_eq!(MAX_AXES, 8);
    assert_eq!(MAX_STEPPERS_PER_AXIS, 4);
    let _ = core::mem::size_of::<StepperRef>();
    let _ = core::mem::size_of::<AxisState>();
}
