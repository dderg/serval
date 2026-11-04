use super::*;
use geometry::{FollowerStepLoad, McuStepLoad, StepCapacity};
use motion_pipeline::live_config::step_capacity::step_capacity;

const CAPACITY_CFG: &str = include_str!("../../../tools/bench/configs/capacity.cfg");
const F401_CLOCK_HZ: f64 = 84.0e6;

fn motor(name: &str, rotation_distance: f64, mode: &str) -> PySteppedMotor {
    PySteppedMotor {
        name: name.into(),
        step_distance: rotation_distance / 3200.0,
        mode: mode.into(),
    }
}

fn f401(ab_mode: &str) -> PyStepBenchmark {
    PyStepBenchmark {
        mcu: "mcu".into(),
        clock_freq: F401_CLOCK_HZ,
        capacity_fraction: 0.75,
        pulsed_ticks_per_step: 301,
        both_edge_ticks_per_step: 160,
        simulated: false,
        motors: vec![
            motor("a", 40.0, ab_mode),
            motor("b", 40.0, ab_mode),
            motor("z", 8.0, MODE_PULSED),
            motor("extruder", 7.73, MODE_PULSED),
        ],
    }
}

fn capacity_of(benchmarks: &[PyStepBenchmark]) -> Option<StepCapacity> {
    let cfg = planner_config::from_doc::planner_config_from_text(CAPACITY_CFG).unwrap();
    step_capacity(
        &cfg,
        &step_capacity_input(CAPACITY_CFG, benchmarks).unwrap(),
    )
    .unwrap()
}

#[test]
fn the_bench_benchmark_caps_the_capacity_config_as_the_live_printer_did() {
    let per_mm =
        |rotation_distance: f64| 301.0 / ((rotation_distance / 3200.0) * (0.75 * F401_CLOCK_HZ));
    assert_eq!(
        capacity_of(&[f401(MODE_PULSED)]),
        Some(StepCapacity {
            lane_weights: [[1.0, 1.0, 0.0], [1.0, -1.0, 0.0], [0.0, 0.0, 1.0]],
            mcus: vec![McuStepLoad {
                mcu: "mcu".into(),
                lane_s_per_mm: [per_mm(40.0), per_mm(40.0), per_mm(8.0)],
                followers: vec![FollowerStepLoad {
                    axis_index: 3,
                    s_per_mm: per_mm(7.73),
                }],
            }],
        })
    );
}

#[test]
fn a_both_edge_motor_is_charged_its_mode() {
    let pulsed = capacity_of(&[f401(MODE_PULSED)]).unwrap();
    let both_edge = capacity_of(&[f401(MODE_BOTH_EDGE)]).unwrap();
    let ratio = both_edge.mcus[0].lane_s_per_mm[0] / pulsed.mcus[0].lane_s_per_mm[0];
    assert!((ratio - 160.0 / 301.0).abs() <= 1e-12, "{ratio}");
}

#[test]
fn a_simulated_mcu_caps_nothing() {
    let simulated = PyStepBenchmark {
        simulated: true,
        ..f401(MODE_PULSED)
    };
    assert_eq!(capacity_of(&[simulated]), None);
}

#[test]
fn a_motor_the_config_does_not_drive_is_refused() {
    let mut benchmark = f401(MODE_PULSED);
    benchmark.motors.push(motor("w", 8.0, MODE_PULSED));
    let err = step_capacity_input(CAPACITY_CFG, &[benchmark]).unwrap_err();
    assert!(err.contains("motor 'w'"), "{err}");
}

#[test]
fn an_unknown_step_mode_is_refused() {
    let err = step_capacity_input(CAPACITY_CFG, &[f401("dedge")]).unwrap_err();
    assert!(err.contains("mode 'dedge'"), "{err}");
}

#[test]
fn a_config_without_kinematics_cannot_resolve_motors() {
    let err =
        step_capacity_input("[printer]\nmax_velocity: 300\nmax_accel: 3000\n", &[]).unwrap_err();
    assert!(err.contains("[kinematics]"), "{err}");
}

#[test]
fn the_budget_is_the_recorded_fraction_of_the_clock_and_each_motors_mode() {
    assert_eq!(
        step_capacity_budget(f401(MODE_BOTH_EDGE)).unwrap(),
        Some((63.0e6, vec![160, 160, 301, 301]))
    );
    assert_eq!(
        step_capacity_budget(PyStepBenchmark {
            capacity_fraction: step_capacity_fraction(),
            ..f401(MODE_BOTH_EDGE)
        })
        .unwrap(),
        Some((52.5e6, vec![160, 160, 301, 301]))
    );
    assert_eq!(
        step_capacity_budget(PyStepBenchmark {
            simulated: true,
            ..f401(MODE_PULSED)
        })
        .unwrap(),
        None
    );
}
