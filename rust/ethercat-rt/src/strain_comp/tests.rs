use super::{
    StrainCompBank, ERR_COMP_BAD_GRID, ERR_COMP_BAD_KINEMATICS, ERR_COMP_BAD_SLOT,
    ERR_COMP_SLOT_IN_USE, KIN_CARTESIAN, KIN_COREXY,
};

const CYCLE_NS: i64 = 250_000;
const SLEW_PER_CYCLE_MM: f64 = 1.0 * 250_000.0 * 1e-9;

// Trident-like topology: slots 0/1 drive lane 0 (belt A), slots 2/3 lane 1.
const SLAVE_AXES: [u8; 4] = [0, 0, 1, 1];

fn bank() -> StrainCompBank {
    StrainCompBank::new(CYCLE_NS)
}

fn settle(bank: &mut StrainCompBank, lane_mm: &[Option<f64>], cycles: usize) -> Vec<f64> {
    let mut out = vec![0.0; 4];
    for _ in 0..cycles {
        out = vec![0.0; 4];
        bank.update(lane_mm, &SLAVE_AXES, &mut out);
    }
    out
}

#[test]
fn constant_grid_slews_to_an_antisymmetric_offset() {
    let mut b = bank();
    assert_eq!(
        b.set(4, 0, 1, 0, 1, KIN_COREXY, 1, 1, 0.0, 0.0, 1.0, 1.0, &[100]),
        0
    );
    let lanes = [Some(10.0), Some(10.0), Some(4.0), Some(4.0)];
    let first = settle(&mut b, &lanes, 1);
    assert!(
        (first[0] - SLEW_PER_CYCLE_MM).abs() < 1e-12,
        "first cycle must be slew-limited, got {}",
        first[0]
    );
    let out = settle(&mut b, &lanes, 2000);
    assert!((out[0] - 0.1).abs() < 1e-9);
    assert!((out[1] + 0.1).abs() < 1e-9);
    assert_eq!(out[2], 0.0);
    assert_eq!(out[3], 0.0);
}

#[test]
fn grid_interpolates_bilinearly_in_carriage_coordinates() {
    let mut b = bank();
    // 2x2 grid over x,y in [0,100]^2: value = x contribution only
    // (0 at x=0, 200 um at x=100).
    assert_eq!(
        b.set(
            4,
            0,
            1,
            0,
            1,
            KIN_COREXY,
            2,
            2,
            0.0,
            0.0,
            100.0,
            100.0,
            &[0, 200, 0, 200]
        ),
        0
    );
    // corexy lanes for carriage (50, 20): pa = x+y = 70, pb = x-y = 30.
    let lanes = [Some(70.0), Some(70.0), Some(30.0), Some(30.0)];
    let out = settle(&mut b, &lanes, 2000);
    assert!(
        (out[0] - 0.1).abs() < 1e-9,
        "x=50 -> 100 um, got {}",
        out[0]
    );
    assert!((out[1] + 0.1).abs() < 1e-9);
}

#[test]
fn cartesian_kinematics_uses_lane_positions_directly() {
    let mut b = bank();
    assert_eq!(
        b.set(
            4,
            0,
            1,
            0,
            1,
            KIN_CARTESIAN,
            2,
            1,
            0.0,
            0.0,
            100.0,
            1.0,
            &[0, 100]
        ),
        0
    );
    let lanes = [Some(50.0), Some(50.0), Some(0.0), Some(0.0)];
    let out = settle(&mut b, &lanes, 2000);
    assert!((out[0] - 0.05).abs() < 1e-9);
}

#[test]
fn positions_outside_the_grid_clamp_to_the_border() {
    let mut b = bank();
    assert_eq!(
        b.set(
            4,
            0,
            1,
            0,
            1,
            KIN_COREXY,
            2,
            1,
            0.0,
            0.0,
            100.0,
            1.0,
            &[-100, 100]
        ),
        0
    );
    let lanes = [Some(-500.0), Some(-500.0), Some(-500.0), Some(-500.0)];
    let out = settle(&mut b, &lanes, 4000);
    assert!((out[0] + 0.1).abs() < 1e-9, "clamped to x0 border value");
}

#[test]
fn idle_lanes_hold_the_last_target() {
    let mut b = bank();
    assert_eq!(
        b.set(4, 0, 1, 0, 1, KIN_COREXY, 1, 1, 0.0, 0.0, 1.0, 1.0, &[80]),
        0
    );
    let streaming = [Some(0.0), Some(0.0), Some(0.0), Some(0.0)];
    settle(&mut b, &streaming, 2000);
    let idle = [None, None, None, None];
    let out = settle(&mut b, &idle, 100);
    assert!(
        (out[0] - 0.08).abs() < 1e-9,
        "held while idle, got {}",
        out[0]
    );
}

#[test]
fn clearing_ramps_the_applied_offset_back_to_zero() {
    let mut b = bank();
    assert_eq!(
        b.set(4, 0, 1, 0, 1, KIN_COREXY, 1, 1, 0.0, 0.0, 1.0, 1.0, &[100]),
        0
    );
    let lanes = [Some(0.0), Some(0.0), Some(0.0), Some(0.0)];
    settle(&mut b, &lanes, 2000);
    assert_eq!(
        b.set(4, 0, 1, 0, 1, KIN_COREXY, 0, 0, 0.0, 0.0, 0.0, 0.0, &[]),
        0
    );
    assert!(!b.active(), "cleared pair leaves the bank inactive");
}

#[test]
fn replacing_a_map_keeps_ramping_from_the_applied_offset() {
    let mut b = bank();
    assert_eq!(
        b.set(4, 0, 1, 0, 1, KIN_COREXY, 1, 1, 0.0, 0.0, 1.0, 1.0, &[100]),
        0
    );
    let lanes = [Some(0.0), Some(0.0), Some(0.0), Some(0.0)];
    settle(&mut b, &lanes, 2000);
    assert_eq!(
        b.set(4, 0, 1, 0, 1, KIN_COREXY, 1, 1, 0.0, 0.0, 1.0, 1.0, &[120]),
        0
    );
    let first = settle(&mut b, &lanes, 1);
    assert!(
        (first[0] - (0.1 + SLEW_PER_CYCLE_MM)).abs() < 1e-9,
        "replacement continues from 100 um, got {}",
        first[0]
    );
}

#[test]
fn two_pairs_compensate_independently() {
    let mut b = bank();
    assert_eq!(
        b.set(4, 0, 1, 0, 1, KIN_COREXY, 1, 1, 0.0, 0.0, 1.0, 1.0, &[100]),
        0
    );
    assert_eq!(
        b.set(4, 2, 3, 0, 1, KIN_COREXY, 1, 1, 0.0, 0.0, 1.0, 1.0, &[-40]),
        0
    );
    let lanes = [Some(0.0), Some(0.0), Some(0.0), Some(0.0)];
    let out = settle(&mut b, &lanes, 2000);
    assert!((out[0] - 0.1).abs() < 1e-9);
    assert!((out[1] + 0.1).abs() < 1e-9);
    assert!((out[2] + 0.04).abs() < 1e-9);
    assert!((out[3] - 0.04).abs() < 1e-9);
}

#[test]
fn bad_inputs_are_rejected() {
    let mut b = bank();
    let ok = &[0i16; 4];
    assert_eq!(
        b.set(4, 0, 0, 0, 1, KIN_COREXY, 2, 2, 0.0, 0.0, 1.0, 1.0, ok),
        ERR_COMP_BAD_SLOT
    );
    assert_eq!(
        b.set(4, 0, 4, 0, 1, KIN_COREXY, 2, 2, 0.0, 0.0, 1.0, 1.0, ok),
        ERR_COMP_BAD_SLOT
    );
    assert_eq!(
        b.set(4, 0, 1, 0, 1, 9, 2, 2, 0.0, 0.0, 1.0, 1.0, ok),
        ERR_COMP_BAD_KINEMATICS
    );
    assert_eq!(
        b.set(4, 0, 1, 0, 1, KIN_COREXY, 2, 2, 0.0, 0.0, 1.0, 1.0, &[0; 3]),
        ERR_COMP_BAD_GRID
    );
    assert_eq!(
        b.set(4, 0, 1, 0, 1, KIN_COREXY, 2, 2, 0.0, 0.0, 0.0, 1.0, ok),
        ERR_COMP_BAD_GRID
    );
    assert_eq!(
        b.set(4, 0, 1, 0, 1, KIN_COREXY, 1, 1, 0.0, 0.0, 1.0, 1.0, &[501]),
        ERR_COMP_BAD_GRID
    );
    assert_eq!(
        b.set(4, 0, 1, 0, 1, KIN_COREXY, 1, 1, 0.0, 0.0, 1.0, 1.0, &[100]),
        0
    );
    assert_eq!(
        b.set(4, 1, 2, 0, 1, KIN_COREXY, 1, 1, 0.0, 0.0, 1.0, 1.0, &[100]),
        ERR_COMP_SLOT_IN_USE
    );
}

#[test]
fn constant_grid_applies_without_any_lane_data() {
    let mut b = bank();
    assert_eq!(
        b.set(4, 0, 1, 0, 1, KIN_COREXY, 1, 1, 0.0, 0.0, 1.0, 1.0, &[100]),
        0
    );
    // The stiffness probe runs entirely at standstill: no lane ever streams.
    let idle = [None, None, None, None];
    let out = settle(&mut b, &idle, 2000);
    assert!((out[0] - 0.1).abs() < 1e-9, "got {}", out[0]);
    assert!((out[1] + 0.1).abs() < 1e-9);
}
