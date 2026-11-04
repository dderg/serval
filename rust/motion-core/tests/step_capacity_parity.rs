//! The live planner and the snapshot path cap moves by the same step
//! capacity: on the capacity bench config at the F401's benchmark both
//! derive one `StepCapacity` and shape identical segments, and the cap
//! binds, so the identity is not that of two uncapped plans.

use motion_core::seam_test_harness::LiveReplay;
use motion_pipeline::TrajectoryItem;
use motion_pipeline::live_config::step_capacity::{
    DrivenAxis, McuStepBenchmark, McuStepInput, STEP_CAPACITY_FRACTION, StepBenchmark,
    StepCapacityInput, StepMode, SteppedMotor, step_capacity,
};
use planner_config::PlannerConfig;
use trajectory::ContinuousSegment;

const CAPACITY_CFG: &str = "tools/bench/configs/capacity.cfg";
const FAST_SQUARE: &str = "M83
G1 X10 Y10 Z1 F150000
G1 X200 Y10
G1 X200 Y200
G1 X10 Y200 E5 F60000
G1 X10 Y10 E5
G1 X200 Y200 F150000
";

fn repo_file(relative: &str) -> String {
    let path = source_tree::repo_file(relative);
    std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("read {}: {e}", path.display()))
}

fn capacity_planner() -> PlannerConfig {
    planner_config::from_doc::planner_config_from_text(&repo_file(CAPACITY_CFG))
        .expect("capacity.cfg plans")
}

fn motor(drives: DrivenAxis, rotation_distance: f64) -> SteppedMotor {
    SteppedMotor {
        drives,
        step_distance_mm: rotation_distance / 3200.0,
        mode: StepMode::Pulsed,
    }
}

fn f401_benchmark() -> StepCapacityInput {
    StepCapacityInput {
        lane_weights: [[1.0, 1.0, 0.0], [1.0, -1.0, 0.0], [0.0, 0.0, 1.0]],
        mcus: vec![McuStepInput {
            benchmark: McuStepBenchmark {
                mcu: "mcu".into(),
                clock_freq_hz: 84.0e6,
                capacity_fraction: STEP_CAPACITY_FRACTION,
                benchmark: Some(StepBenchmark {
                    pulsed_ticks_per_step: 301,
                    both_edge_ticks_per_step: 160,
                }),
            },
            motors: vec![
                motor(DrivenAxis::Lane(0), 40.0),
                motor(DrivenAxis::Lane(1), 40.0),
                motor(DrivenAxis::Lane(2), 8.0),
                motor(DrivenAxis::Follower("e".into()), 7.73),
            ],
        }],
    }
}

fn live_segments(planner: &PlannerConfig, benchmark: Option<&StepCapacityInput>) -> Vec<String> {
    let replay = LiveReplay::new(FAST_SQUARE, planner, benchmark).expect("live replay");
    let mut segments: Vec<ContinuousSegment> = Vec::new();
    let mut collect = |item| {
        if let TrajectoryItem::Seg(segment) = item {
            segments.push(segment);
        }
        true
    };
    let mut pipeline = replay.pipeline();
    for m in replay.moves {
        assert!(pipeline.feed(m.into(), &mut collect));
    }
    assert!(pipeline.finish(&mut collect));
    segments.iter().map(|s| format!("{s:?}")).collect()
}

fn snapshot_segments(
    planner: &PlannerConfig,
    benchmark: Option<&StepCapacityInput>,
) -> (Vec<String>, f64) {
    let base = motion_pipeline::live_config::GcodeMoveBase::of(
        planner,
        &motion_pipeline::live_config::extruder_caps(planner).expect("extruder caps"),
    )
    .expect("move base");
    let waypoints = pipeline_snapshot::waypoints::parse_gcode(FAST_SQUARE, &base.config_limits())
        .expect("gcode parses");
    let record =
        pipeline_snapshot::stream_record(&waypoints, planner, benchmark).expect("snapshot plans");
    let duration = record.segments.last().expect("segments").t_end;
    (
        record.segments.iter().map(|s| format!("{s:?}")).collect(),
        duration,
    )
}

#[test]
fn live_and_snapshot_derive_one_capacity_and_shape_identical_capped_moves() {
    let planner = capacity_planner();
    let benchmark = f401_benchmark();
    let live = LiveReplay::new(FAST_SQUARE, &planner, Some(&benchmark)).expect("live replay");
    assert_eq!(
        live.config.step_capacity.as_deref(),
        step_capacity(&planner, &benchmark)
            .expect("the bench benchmark derives")
            .as_ref()
    );
    assert!(live.config.step_capacity.is_some());

    let (snapshot, capped_s) = snapshot_segments(&planner, Some(&benchmark));
    assert_eq!(live_segments(&planner, Some(&benchmark)), snapshot);

    let (uncapped, uncapped_s) = snapshot_segments(&planner, None);
    assert_ne!(snapshot, uncapped);
    assert!(
        capped_s > uncapped_s,
        "the cap must slow the square: capped {capped_s} s, uncapped {uncapped_s} s"
    );
}
