//! The live planner commits whenever its inbox runs dry, the snapshot path
//! only when a replan batch fills, so where a window ends depends on the
//! host's timing. The shaped trajectory must not: every committed prefix is
//! the one a single window would plan, bit for bit.

use std::path::Path;

use motion_pipeline::{Pipeline, TrajectoryItem, live_config};
use pipeline_snapshot::waypoints::parse_gcode;
use pipeline_snapshot::{PlanStep, plan_moves, plan_steps, rest_position_before};
use planner_config::from_doc::{planner_config_from_settings, read_motion_settings};

mod corpus;

fn shaped(steps: &[PlanStep], config: &Path, idle_every: Option<usize>) -> Vec<String> {
    let text = std::fs::read_to_string(config).expect("read config");
    let doc = config_doc::Document::parse(&text, "test.cfg").expect("config parses");
    let (settings, _) = read_motion_settings(&doc).expect("motion settings");
    let planner = planner_config_from_settings(&settings).expect("planner config");
    let mut pipeline = Pipeline::new(
        live_config::stream_config(&planner, None).expect("stream config"),
        planner.compile_active_chains().expect("chains"),
        rest_position_before(plan_moves(steps)),
        0.0,
    );
    let mut segments = Vec::new();
    let mut collect = |item: TrajectoryItem| {
        if let TrajectoryItem::Seg(seg) = item {
            segments.push(format!("{seg:?}"));
        }
        true
    };
    for (index, step) in steps.iter().cloned().enumerate() {
        assert!(step.feed(|input| pipeline.feed(input, &mut collect)));
        if idle_every.is_some_and(|every| index % every == every - 1) {
            assert!(pipeline.idle(&mut collect));
        }
    }
    assert!(pipeline.finish(&mut collect));
    segments
}

fn steps_for(gcode: &str, config: &Path) -> Vec<PlanStep> {
    let text = std::fs::read_to_string(config).expect("read config");
    let doc = config_doc::Document::parse(&text, "test.cfg").expect("config parses");
    let (settings, _) = read_motion_settings(&doc).expect("motion settings");
    let planner = planner_config_from_settings(&settings).expect("planner config");
    let base = live_config::GcodeMoveBase::of(
        &planner,
        &live_config::extruder_caps(&planner).expect("extruder caps"),
    )
    .expect("move base");
    let waypoints = parse_gcode(gcode, &settings.cartesian).expect("gcode parses");
    plan_steps(&waypoints, base).expect("plan steps")
}

#[test]
fn commits_wherever_the_inbox_runs_dry_shape_the_same_trajectory() {
    let gcode = corpus::sim_print("voron_first_layers");
    for config in ["tools/bench/configs/ramp.cfg", "tools/bench/printer.cfg"] {
        let config = corpus::repo().join(config);
        let steps = steps_for(&gcode, &config);
        let batched = shaped(&steps, &config, None);
        for idle_every in [37, 41] {
            let idled = shaped(&steps, &config, Some(idle_every));
            assert_eq!(idled.len(), batched.len(), "{}", config.display());
            if let Some(at) = (0..batched.len()).find(|&i| idled[i] != batched[i]) {
                panic!(
                    "{}: idle every {idle_every} moves reshapes segment {at} of {}",
                    config.display(),
                    batched.len()
                );
            }
        }
    }
}
