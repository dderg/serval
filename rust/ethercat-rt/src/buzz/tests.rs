use super::*;

const FS: u32 = 40_000;
const FE: u32 = 40_000;
const AMP_NM: u32 = 1_000_000;
const DUR_MS: u32 = 100;
const RAMP_MS: u32 = 10;

fn armed(slot_mask: u8, sign_mask: u8, base_counts: [i32; MAX_BUZZ_SLOTS]) -> BuzzOsc {
    let mut osc = BuzzOsc::new();
    let rc = osc.arm(
        8,
        slot_mask,
        sign_mask,
        FS,
        FE,
        AMP_NM,
        DUR_MS,
        RAMP_MS,
        base_counts,
    );
    assert_eq!(rc, 0, "arm should succeed");
    osc
}

#[test]
fn arm_activates_and_records_per_slot_base() {
    let mut base = [0i32; MAX_BUZZ_SLOTS];
    base[0] = 12_345;
    base[1] = -777;
    let osc = armed(0b11, 0, base);
    assert!(osc.active());
    assert_eq!(osc.base_counts(0), 12_345);
    assert_eq!(osc.base_counts(1), -777);
}

#[test]
fn rejects_zero_slot_mask() {
    let mut osc = BuzzOsc::new();
    let rc = osc.arm(
        8,
        0,
        0,
        FS,
        FE,
        AMP_NM,
        DUR_MS,
        RAMP_MS,
        [0; MAX_BUZZ_SLOTS],
    );
    assert!(!osc.active(), "zero mask must not arm");
    assert_eq!(rc, -1);
}

#[test]
fn rejects_slot_beyond_num_slots() {
    let mut osc = BuzzOsc::new();
    let rc = osc.arm(
        2,
        0b100,
        0,
        FS,
        FE,
        AMP_NM,
        DUR_MS,
        RAMP_MS,
        [0; MAX_BUZZ_SLOTS],
    );
    assert!(!osc.active(), "slot 2 on a 2-slave node must not arm");
    assert_eq!(rc, -1);
}

#[test]
fn drives_only_masked_slots() {
    let osc = armed(0b101, 0, [0; MAX_BUZZ_SLOTS]);
    assert!(osc.drives_slot(0));
    assert!(!osc.drives_slot(1));
    assert!(osc.drives_slot(2));
    assert!(!osc.drives_slot(MAX_BUZZ_SLOTS));
}

#[test]
fn drives_no_slot_when_inactive() {
    let mut osc = armed(0b1, 0, [0; MAX_BUZZ_SLOTS]);
    osc.clear();
    assert!(!osc.drives_slot(0));
}

#[test]
fn sign_mask_flips_only_masked_slots() {
    let osc = armed(0b11, 0b10, [0; MAX_BUZZ_SLOTS]);
    assert_eq!(osc.slot_sign(0), 1.0);
    assert_eq!(osc.slot_sign(1), -1.0);
}

#[test]
fn shared_sample_is_sign_free() {
    let mut in_phase = armed(0b11, 0, [0; MAX_BUZZ_SLOTS]);
    let mut anti_phase = armed(0b11, 0b10, [0; MAX_BUZZ_SLOTS]);
    let start = 2_000_000_000u64;
    let _ = in_phase.eval(start);
    let _ = anti_phase.eval(start);
    let mid = start + u64::from(DUR_MS) * 1_000_000 / 2;
    let (pos_a, _, _) = in_phase.eval(mid).expect("active");
    let (pos_b, _, _) = anti_phase.eval(mid).expect("active");
    assert_eq!(
        pos_a, pos_b,
        "sign lives in slot_sign, not the shared sample"
    );
}

#[test]
fn anchors_on_first_eval_and_starts_near_zero() {
    let mut osc = armed(0b1, 0, [0; MAX_BUZZ_SLOTS]);
    let (pos, _vel, _acc) = osc.eval(1_000_000_000).expect("active");
    assert!(pos.abs() < 1.0e-6, "envelope starts at zero, pos={pos}");
}

#[test]
fn deactivates_after_duration() {
    let mut osc = armed(0b1, 0, [0; MAX_BUZZ_SLOTS]);
    let start = 5_000_000_000u64;
    assert!(osc.eval(start).is_some());
    let past = start + u64::from(DUR_MS) * 1_000_000 + 1_000_000;
    assert!(osc.eval(past).is_none(), "past duration yields None");
    assert!(!osc.active(), "osc clears itself when done");
}

#[test]
fn midpoint_produces_finite_motion() {
    let mut osc = armed(0b1, 0, [0; MAX_BUZZ_SLOTS]);
    let start = 2_000_000_000u64;
    let _ = osc.eval(start);
    let mid = start + u64::from(DUR_MS) * 1_000_000 / 2;
    let (pos, vel, acc) = osc.eval(mid).expect("active at midpoint");
    assert!(pos.is_finite() && vel.is_finite() && acc.is_finite());
    assert!(pos.abs() > 0.0, "flat-top motion is nonzero");
}

#[test]
fn clear_stops_evaluation() {
    let mut osc = armed(0b1, 0, [0; MAX_BUZZ_SLOTS]);
    osc.clear();
    assert!(!osc.active());
    assert!(osc.eval(1_000).is_none());
}
