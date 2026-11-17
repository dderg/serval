use std::path::PathBuf;

use geometry::GcodePos;
use motion_core::seam_test_harness::LiveScript;
use motion_pipeline::{Pipeline, TrajectoryItem};

/// A minimized simulator fuzz scenario as `scripts/preflight.py` hands it
/// to `gcode_replay_bench`: its motion config and its expanded print.
fn scenario_file(seed: u32, name: &str) -> String {
    let path: PathBuf = source_tree::manifest_dir()
        .join("tests/fuzz_scenarios")
        .join(format!("seed_{seed}"))
        .join(name);
    std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("read {}: {e}", path.display()))
}

fn replays_to_the_end(seed: u32) {
    let planner =
        planner_config::from_doc::planner_config_from_text(&scenario_file(seed, "printer.cfg"))
            .expect("the scenario's planner config");
    let mut script = LiveScript::new(
        &scenario_file(seed, "print.gcode"),
        &planner,
        None,
        GcodePos([0.0; 3]),
    )
    .expect("the print plans");
    let inputs = script.take_inputs(None);
    let mut pipeline = Pipeline::new(script.config, script.chains, script.home, 0.0);
    let mut emitted = 0usize;
    let mut count = |item| {
        if let TrajectoryItem::Seg(_) = item {
            emitted += 1;
        }
        true
    };
    for input in inputs {
        assert!(
            pipeline.feed(input, &mut count),
            "the pipeline refused an input"
        );
    }
    assert!(pipeline.finish(&mut count), "the pipeline failed to drain");
    assert!(emitted > 0, "the replay emitted no motion");
}

#[test]
fn an_advance_and_a_kernel_on_e_retract_around_a_dwell() {
    replays_to_the_end(34);
}
