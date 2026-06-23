use super::*;

const FS: u32 = 40_000;
const FE: u32 = 40_000;
const AMP_NM: u32 = 1_000_000;
const DUR_MS: u32 = 100;
const RAMP_MS: u32 = 10;

fn armed(base_counts: i32) -> BuzzOsc {
    let mut osc = BuzzOsc::new();
    let rc = osc.arm(0b1, 0, FS, FE, AMP_NM, DUR_MS, RAMP_MS, base_counts);
    assert_eq!(rc, 0, "arm should succeed");
    osc
}

#[test]
fn arm_activates_and_records_base() {
    let osc = armed(12_345);
    assert!(osc.active());
    assert_eq!(osc.base_counts(), 12_345);
}

#[test]
fn rejects_zero_axis_mask() {
    let mut osc = BuzzOsc::new();
    let rc = osc.arm(0, 0, FS, FE, AMP_NM, DUR_MS, RAMP_MS, 0);
    assert!(!osc.active(), "zero mask must not arm");
    assert_eq!(rc, -1);
}

#[test]
fn anchors_on_first_eval_and_starts_near_zero() {
    let mut osc = armed(0);
    let (pos, _vel, _acc) = osc.eval(1_000_000_000).expect("active");
    assert!(pos.abs() < 1.0e-6, "envelope starts at zero, pos={pos}");
}

#[test]
fn deactivates_after_duration() {
    let mut osc = armed(0);
    let start = 5_000_000_000u64;
    assert!(osc.eval(start).is_some());
    let past = start + u64::from(DUR_MS) * 1_000_000 + 1_000_000;
    assert!(osc.eval(past).is_none(), "past duration yields None");
    assert!(!osc.active(), "osc clears itself when done");
}

#[test]
fn midpoint_produces_finite_motion() {
    let mut osc = armed(0);
    let start = 2_000_000_000u64;
    let _ = osc.eval(start);
    let mid = start + u64::from(DUR_MS) * 1_000_000 / 2;
    let (pos, vel, acc) = osc.eval(mid).expect("active at midpoint");
    assert!(pos.is_finite() && vel.is_finite() && acc.is_finite());
    assert!(pos.abs() > 0.0, "flat-top motion is nonzero");
}

#[test]
fn clear_stops_evaluation() {
    let mut osc = armed(0);
    osc.clear();
    assert!(!osc.active());
    assert!(osc.eval(1_000).is_none());
}
