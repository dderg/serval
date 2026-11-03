use std::path::{Path, PathBuf};

use motion_core::seam_test_harness::LiveReplay;
use motion_pipeline::TrajectoryItem;

const DENSE_PRINT_HEAD_LINES: usize = 1100;

fn repo() -> PathBuf {
    source_tree::repo_root()
}

fn bench_configs() -> Vec<PathBuf> {
    let mut configs: Vec<PathBuf> = std::fs::read_dir(repo().join("tools/bench/configs"))
        .expect("bench config directory")
        .map(|entry| entry.expect("bench config entry").path())
        .filter(|path| path.extension().is_some_and(|e| e == "cfg"))
        .collect();
    configs.sort();
    configs
}

fn planner_config(path: &Path) -> planner_config::PlannerConfig {
    let text =
        std::fs::read_to_string(path).unwrap_or_else(|e| panic!("read {}: {e}", path.display()));
    let doc =
        config_doc::Document::parse(&text, &path.display().to_string()).expect("config parses");
    let (settings, _) =
        planner_config::from_doc::read_motion_settings(&doc).expect("motion settings");
    planner_config::from_doc::planner_config_from_settings(&settings).expect("planner config")
}

fn dense_print_head() -> String {
    std::fs::read_to_string(repo().join("tools/sim/gcode/voron_dense_top_layers.gcode"))
        .expect("read the dense print")
        .lines()
        .take(DENSE_PRINT_HEAD_LINES)
        .flat_map(|line| [line, "\n"])
        .collect()
}

#[test]
fn the_live_path_plans_the_dense_print_head_on_every_bench_config() {
    let configs = bench_configs();
    assert!(
        configs
            .iter()
            .any(|path| path.ends_with("trident_ramp.cfg")),
        "the Trident ratio-ramp config left tools/bench/configs"
    );
    let gcode = dense_print_head();
    for config in configs {
        let replay = LiveReplay::new(&gcode, &planner_config(&config))
            .unwrap_or_else(|e| panic!("{}: {e}", config.display()));
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
            assert!(pipeline.feed(m.into()), "{}", config.display());
        }
        assert!(pipeline.finish(), "{}", config.display());
        assert!(
            emitted.try_iter().last().is_some_and(|t_end| t_end > 0.0),
            "{}: the replay emitted no motion",
            config.display()
        );
    }
}
