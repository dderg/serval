use super::*;
use crate::mcu_config::{KINEMATICS_COREXY, LaneKind, McuHardware};

const F401_TICKS_PER_S: f64 = 0.75 * 84.0e6;
const PULSED: u32 = 301;
const BOTH_EDGE: u32 = 160;
const AB_STEP: f64 = 40.0 / 3200.0;
const Z_STEP: f64 = 8.0 / 3200.0;
const E_STEP: f64 = 7.73 / 3200.0;

fn board(
    name: &str,
    axes: Vec<usize>,
    microstep_distance: Vec<f64>,
    ticks_per_step: Option<Vec<u32>>,
) -> McuAxisConfig {
    let lanes = axes.len();
    McuAxisConfig {
        axes,
        lane_kinds: vec![LaneKind::Pulse; lanes],
        hw: McuHardware {
            kinematics: KINEMATICS_COREXY,
            motor_counts: vec![1; lanes],
            microstep_distance,
            step_capacity: ticks_per_step.map(|ticks_per_step| McuStepCapacity {
                mcu_name: name.to_string(),
                ticks_per_s: F401_TICKS_PER_S,
                ticks_per_step,
            }),
            ..Default::default()
        },
        ..Default::default()
    }
}

fn bench(ticks_per_step: Vec<u32>) -> McuAxisConfig {
    board(
        "mcu",
        vec![0, 1, 2, 3],
        vec![AB_STEP, AB_STEP, Z_STEP, E_STEP],
        Some(ticks_per_step),
    )
}

fn close(got: f64, want: f64) -> bool {
    (got - want).abs() <= 1e-12 * want
}

#[test]
fn the_bench_printer_charges_each_motor_its_steps_per_mm_times_its_tick_cost() {
    let capacity = step_capacity(&[bench(vec![PULSED; 4])]).unwrap();
    assert_eq!(capacity.lane_weights[0], [1.0, 1.0, 0.0]);
    assert_eq!(capacity.lane_weights[1], [1.0, -1.0, 0.0]);
    let load = &capacity.mcus[0];
    assert_eq!(load.mcu, "mcu");
    let per_mm = |step: f64| f64::from(PULSED) / (step * F401_TICKS_PER_S);
    assert!(close(load.lane_s_per_mm[0], per_mm(AB_STEP)));
    assert!(close(load.lane_s_per_mm[2], per_mm(Z_STEP)));
    assert!(close(load.follower_s_per_mm(3), per_mm(E_STEP)));
    let pure_x_mm_s = 1.0 / (2.0 * load.lane_s_per_mm[0]);
    assert!(
        (pure_x_mm_s - 1308.1).abs() < 0.1,
        "a pure X move at 80 steps/mm on a and b: {pure_x_mm_s}"
    );
}

#[test]
fn a_both_edge_motor_costs_what_its_mode_measured() {
    let pulsed = step_capacity(&[bench(vec![PULSED; 4])]).unwrap();
    let both_edge = step_capacity(&[bench(vec![BOTH_EDGE, BOTH_EDGE, PULSED, PULSED])]).unwrap();
    let ratio = both_edge.mcus[0].lane_s_per_mm[0] / pulsed.mcus[0].lane_s_per_mm[0];
    assert!(close(ratio, f64::from(BOTH_EDGE) / f64::from(PULSED)));
    assert_eq!(
        both_edge.mcus[0].lane_s_per_mm[2],
        pulsed.mcus[0].lane_s_per_mm[2]
    );
}

#[test]
fn each_mcu_carries_only_the_motors_it_steps() {
    let capacity = step_capacity(&[
        board(
            "mcu",
            vec![0, 1],
            vec![AB_STEP, AB_STEP],
            Some(vec![PULSED; 2]),
        ),
        board(
            "toolboard",
            vec![2, 3],
            vec![Z_STEP, E_STEP],
            Some(vec![PULSED; 2]),
        ),
    ])
    .unwrap();
    assert_eq!(capacity.mcus.len(), 2);
    assert_eq!(capacity.mcus[0].lane_s_per_mm[2], 0.0);
    assert!(capacity.mcus[0].followers.is_empty());
    assert_eq!(capacity.mcus[1].lane_s_per_mm[0], 0.0);
    assert_eq!(capacity.mcus[1].mcu, "toolboard");
    assert!(capacity.mcus[1].follower_s_per_mm(3) > 0.0);
}

#[test]
fn mcus_without_a_measured_capacity_are_not_capped() {
    assert_eq!(step_capacity(&[]), None);
    assert_eq!(
        step_capacity(&[board(
            "mcu",
            vec![0, 1, 2],
            vec![AB_STEP, AB_STEP, Z_STEP],
            None
        )]),
        None
    );
    let partial = step_capacity(&[
        board("mcu", vec![0, 1], vec![AB_STEP, AB_STEP], None),
        board(
            "toolboard",
            vec![2, 3],
            vec![Z_STEP, E_STEP],
            Some(vec![PULSED; 2]),
        ),
    ])
    .unwrap();
    assert_eq!(partial.mcus.len(), 1);
    assert_eq!(partial.mcus[0].mcu, "toolboard");
}

#[test]
fn phase_stepped_motors_cost_no_dispatch() {
    assert_eq!(
        step_capacity(&[bench(vec![0; 4])]),
        None,
        "an mcu whose every motor is phase stepped dispatches no steps"
    );
}
