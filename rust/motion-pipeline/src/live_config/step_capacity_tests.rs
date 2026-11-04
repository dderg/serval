use super::*;

const F401_CLOCK_HZ: f64 = 84.0e6;
const F401: StepBenchmark = StepBenchmark {
    pulsed_ticks_per_step: 301,
    both_edge_ticks_per_step: 160,
};
const AB_STEP: f64 = 40.0 / 3200.0;
const Z_STEP: f64 = 8.0 / 3200.0;
const E_STEP: f64 = 7.73 / 3200.0;
const COREXY: [[f64; 3]; 3] = [[1.0, 1.0, 0.0], [1.0, -1.0, 0.0], [0.0, 0.0, 1.0]];
const E_AXIS: usize = 3;

const COREXY_WITH_E: &str = "[printer]\n\
    max_velocity: 2500\n\
    max_accel: 60000\n\
    [kinematics]\n\
    type: corexy\n\
    axis_x: x\n\
    axis_y: y\n\
    axis_z: z\n\
    a_motors: a\n\
    b_motors: b\n\
    z_motors: z\n\
    [motor a]\n\
    drive: stepper\n\
    [motor b]\n\
    drive: stepper\n\
    [motor z]\n\
    drive: stepper\n\
    [motor extruder]\n\
    drive: stepper\n\
    [axis x]\n\
    [axis y]\n\
    [axis z]\n\
    [axis e]\n\
    follows: x, y, z\n\
    motors: extruder\n";

const EXTRUDER: &str = "[extruder]\n\
    axis: e\n\
    max_extrude_only_velocity: 400\n\
    max_extrude_only_accel: 15000\n";

fn planner(text: &str) -> PlannerConfig {
    planner_config::from_doc::planner_config_from_text(text).unwrap()
}

fn bench_planner() -> PlannerConfig {
    planner(&format!("{COREXY_WITH_E}{EXTRUDER}"))
}

fn motor(drives: DrivenAxis, step_distance_mm: f64, mode: StepMode) -> SteppedMotor {
    SteppedMotor {
        drives,
        step_distance_mm,
        mode,
    }
}

fn e_motor(mode: StepMode) -> SteppedMotor {
    motor(DrivenAxis::Follower("e".into()), E_STEP, mode)
}

fn board(name: &str, benchmark: Option<StepBenchmark>, motors: Vec<SteppedMotor>) -> McuStepInput {
    McuStepInput {
        benchmark: McuStepBenchmark {
            mcu: name.into(),
            clock_freq_hz: F401_CLOCK_HZ,
            capacity_fraction: STEP_CAPACITY_FRACTION,
            benchmark,
        },
        motors,
    }
}

fn bench_motors(ab_mode: StepMode) -> Vec<SteppedMotor> {
    vec![
        motor(DrivenAxis::Lane(0), AB_STEP, ab_mode),
        motor(DrivenAxis::Lane(1), AB_STEP, ab_mode),
        motor(DrivenAxis::Lane(2), Z_STEP, StepMode::Pulsed),
        e_motor(StepMode::Pulsed),
    ]
}

fn corexy(mcus: Vec<McuStepInput>) -> StepCapacityInput {
    StepCapacityInput {
        lane_weights: COREXY,
        mcus,
    }
}

fn bench(ab_mode: StepMode) -> StepCapacityInput {
    corexy(vec![board("mcu", Some(F401), bench_motors(ab_mode))])
}

fn capacity_of(input: &StepCapacityInput) -> StepCapacity {
    step_capacity(&bench_planner(), input)
        .unwrap()
        .expect("the bench mcu is capped")
}

#[test]
fn the_budget_is_the_recorded_fraction_of_the_clock_and_each_motor_pays_its_mode() {
    let mut mcu = board("mcu", Some(F401), Vec::new()).benchmark;
    mcu.capacity_fraction = 0.5;
    let budget = mcu
        .budget(&[StepMode::Pulsed, StepMode::BothEdge])
        .unwrap()
        .unwrap();
    assert_eq!(budget.ticks_per_s, 42.0e6);
    assert_eq!(budget.ticks_per_step, vec![301, 160]);
}

#[test]
fn a_fraction_outside_zero_to_one_is_refused() {
    for fraction in [0.0, -0.1, 1.5, f64::NAN] {
        let mut mcu = board("mcu", Some(F401), Vec::new()).benchmark;
        mcu.capacity_fraction = fraction;
        let error = mcu.budget(&[StepMode::Pulsed]).unwrap_err();
        assert!(error.contains("fraction"), "{error}");
    }
}

#[test]
fn the_bench_printer_charges_each_motor_its_steps_per_mm_times_its_tick_cost() {
    let capacity = capacity_of(&bench(StepMode::Pulsed));
    let per_mm = |step: f64| 301.0 / (step * (STEP_CAPACITY_FRACTION * F401_CLOCK_HZ));
    assert_eq!(
        capacity,
        StepCapacity {
            lane_weights: COREXY,
            mcus: vec![McuStepLoad {
                mcu: "mcu".into(),
                lane_s_per_mm: [per_mm(AB_STEP), per_mm(AB_STEP), per_mm(Z_STEP)],
                followers: vec![FollowerStepLoad {
                    axis_index: E_AXIS,
                    s_per_mm: per_mm(E_STEP),
                }],
            }],
        }
    );
    let pure_x_mm_s = 1.0 / (2.0 * capacity.mcus[0].lane_s_per_mm[0]);
    assert!(
        (pure_x_mm_s - 1395.3).abs() < 0.1,
        "a pure X move at 80 steps/mm on a and b: {pure_x_mm_s}"
    );
}

#[test]
fn a_both_edge_motor_costs_what_its_mode_measured() {
    let pulsed = capacity_of(&bench(StepMode::Pulsed));
    let both_edge = capacity_of(&bench(StepMode::BothEdge));
    let ratio = both_edge.mcus[0].lane_s_per_mm[0] / pulsed.mcus[0].lane_s_per_mm[0];
    assert!((ratio - 160.0 / 301.0).abs() <= 1e-12);
    assert_eq!(
        both_edge.mcus[0].lane_s_per_mm[2],
        pulsed.mcus[0].lane_s_per_mm[2]
    );
}

#[test]
fn motors_sharing_a_lane_add_their_costs() {
    let one = capacity_of(&corexy(vec![board(
        "mcu",
        Some(F401),
        vec![motor(DrivenAxis::Lane(2), Z_STEP, StepMode::Pulsed)],
    )]));
    let two = capacity_of(&corexy(vec![board(
        "mcu",
        Some(F401),
        vec![
            motor(DrivenAxis::Lane(2), Z_STEP, StepMode::Pulsed),
            motor(DrivenAxis::Lane(2), Z_STEP, StepMode::Pulsed),
        ],
    )]));
    assert_eq!(
        two.mcus[0].lane_s_per_mm[2],
        2.0 * one.mcus[0].lane_s_per_mm[2]
    );
}

#[test]
fn each_mcu_carries_only_the_motors_it_steps() {
    let capacity = capacity_of(&corexy(vec![
        board(
            "mcu",
            Some(F401),
            vec![
                motor(DrivenAxis::Lane(0), AB_STEP, StepMode::Pulsed),
                motor(DrivenAxis::Lane(1), AB_STEP, StepMode::Pulsed),
            ],
        ),
        board(
            "toolboard",
            Some(F401),
            vec![
                motor(DrivenAxis::Lane(2), Z_STEP, StepMode::Pulsed),
                e_motor(StepMode::Pulsed),
            ],
        ),
    ]));
    assert_eq!(capacity.mcus.len(), 2);
    assert_eq!(capacity.mcus[0].lane_s_per_mm[2], 0.0);
    assert!(capacity.mcus[0].followers.is_empty());
    assert_eq!(capacity.mcus[1].lane_s_per_mm[0], 0.0);
    assert_eq!(capacity.mcus[1].mcu, "toolboard");
    assert!(capacity.mcus[1].follower_s_per_mm(E_AXIS) > 0.0);
}

#[test]
fn simulated_mcus_and_mcus_stepping_nothing_are_not_capped() {
    let cfg = bench_planner();
    assert_eq!(step_capacity(&cfg, &corexy(Vec::new())), Ok(None));
    assert_eq!(
        step_capacity(
            &cfg,
            &corexy(vec![board("mcu", None, bench_motors(StepMode::Pulsed))])
        ),
        Ok(None)
    );
    assert_eq!(
        step_capacity(&cfg, &corexy(vec![board("mcu", Some(F401), Vec::new())])),
        Ok(None)
    );
    let partial = capacity_of(&corexy(vec![
        board("mcu", None, bench_motors(StepMode::Pulsed)),
        board("toolboard", Some(F401), vec![e_motor(StepMode::Pulsed)]),
    ]));
    assert_eq!(partial.mcus.len(), 1);
    assert_eq!(partial.mcus[0].mcu, "toolboard");
}

#[test]
fn a_capped_follower_without_limits_is_refused() {
    let err = step_capacity(&planner(COREXY_WITH_E), &bench(StepMode::Pulsed)).unwrap_err();
    assert!(
        err.contains("mcu 'mcu' steps axis 'e'") && err.contains("[axis e] a max_velocity"),
        "{err}"
    );
}

#[test]
fn a_motor_on_an_unknown_axis_is_refused() {
    let err = step_capacity(
        &bench_planner(),
        &corexy(vec![board(
            "mcu",
            Some(F401),
            vec![motor(
                DrivenAxis::Follower("w".into()),
                E_STEP,
                StepMode::Pulsed,
            )],
        )]),
    )
    .unwrap_err();
    assert!(err.contains('w'), "{err}");
}

#[test]
fn an_unusable_measurement_is_refused() {
    let mut input = bench(StepMode::Pulsed);
    input.mcus[0].benchmark.clock_freq_hz = 0.0;
    assert!(step_capacity(&bench_planner(), &input).is_err());
    let mut input = bench(StepMode::Pulsed);
    input.mcus[0].benchmark.benchmark = Some(StepBenchmark {
        pulsed_ticks_per_step: 0,
        both_edge_ticks_per_step: 160,
    });
    assert!(step_capacity(&bench_planner(), &input).is_err());
    let mut input = bench(StepMode::Pulsed);
    input.mcus[0].motors[0].step_distance_mm = 0.0;
    assert!(step_capacity(&bench_planner(), &input).is_err());
}
