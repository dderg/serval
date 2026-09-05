use super::planner_api::require_single_motor_mask;
use trajectory::NudgeProfile;

#[test]
fn multi_bit_mask_is_rejected() {
    assert_eq!(
        require_single_motor_mask(0b0000_0011),
        Err("submit_nudge: multi-bit motor_mask 0b00000011 not supported".to_string())
    );
}

#[test]
fn single_bit_and_empty_masks_are_accepted() {
    assert_eq!(require_single_motor_mask(0b0000_0010), Ok(()));
    assert_eq!(require_single_motor_mask(0b1000_0000), Ok(()));
    assert_eq!(require_single_motor_mask(0), Ok(()));
}

#[test]
fn reported_duration_is_the_profile_duration() {
    let profile = NudgeProfile::try_new(1.0, 10.0, 100.0, 0.0).unwrap();

    assert_eq!(profile.duration(), profile.t_end() - 0.0);
    assert!(profile.duration() > 0.0);
}

#[test]
fn zero_displacement_nudge_is_rejected_by_the_profile() {
    assert!(NudgeProfile::try_new(0.0, 10.0, 100.0, 0.0).is_err());
}
