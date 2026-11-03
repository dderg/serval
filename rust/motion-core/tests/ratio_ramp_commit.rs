use std::path::{Path, PathBuf};

use motion_core::seam_test_harness::LiveReplay;
use motion_pipeline::TrajectoryItem;

fn repo() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../..")
}

fn planner_config(name: &str) -> planner_config::PlannerConfig {
    let path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests/configs")
        .join(name);
    let text =
        std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("read {}: {e}", path.display()));
    let doc =
        config_doc::Document::parse(&text, &path.display().to_string()).expect("config parses");
    let (settings, _) =
        planner_config::from_doc::read_motion_settings(&doc).expect("motion settings");
    planner_config::from_doc::planner_config_from_settings(&settings).expect("planner config")
}

fn dense_print_head(prologue: &str, lines: usize) -> String {
    let print =
        std::fs::read_to_string(repo().join("tools/sim/gcode/voron_dense_top_layers.gcode"))
            .expect("read the dense print");
    std::iter::once(prologue)
        .chain(print.lines().take(lines).flat_map(|line| [line, "\n"]))
        .collect()
}

fn replays_to_the_end(config: &str, gcode: &str) {
    let replay = LiveReplay::new(gcode, &planner_config(config)).expect("the replay builds");
    let (emitted_tx, emitted) = crossbeam_channel::unbounded();
    let mut pipeline = replay.pipeline().into_staged(move |item| {
        if let TrajectoryItem::Seg(segment) = item {
            emitted_tx
                .send(segment.t_end)
                .expect("the test collects every segment");
        }
        true
    });
    for m in replay.moves {
        assert!(
            pipeline.feed(m.into()),
            "{config}: the pipeline refused a move"
        );
    }
    assert!(pipeline.finish(), "{config}: the pipeline failed to drain");
    assert!(
        emitted.try_iter().last().is_some_and(|t_end| t_end > 0.0),
        "{config}: the replay emitted no motion"
    );
}

#[test]
fn a_ratio_ramp_commits_a_segment_only_once_a_stop_past_four_ramp_times_settles_it() {
    replays_to_the_end("p1_bowden_ramp.cfg", &dense_print_head("", 118));
}

#[test]
fn a_kernel_after_a_ratio_ramp_commits_only_what_its_window_finds_projected() {
    replays_to_the_end(
        "p2_direct_hybrid.cfg",
        &dense_print_head("G1 X0 Y0 Z0 F60000\n", 50),
    );
}
