//! What the shaper emits does not depend on how many lowered segments it
//! takes in before each emit: a live printer and an offline replay cut the
//! stream at different places and must still plan the same trajectory.

use std::path::Path;

use motion_core::seam_test_harness::LiveReplay;
use motion_pipeline::TrajectoryItem;
use trajectory::ContinuousSegment;

const EMIT_BATCH_SEGMENTS: usize = 8;

fn planner_config(path: &Path) -> planner_config::PlannerConfig {
    let text =
        std::fs::read_to_string(path).unwrap_or_else(|e| panic!("read {}: {e}", path.display()));
    let doc =
        config_doc::Document::parse(&text, &path.display().to_string()).expect("config parses");
    let (settings, _) =
        planner_config::from_doc::read_motion_settings(&doc).expect("motion settings");
    planner_config::from_doc::planner_config_from_settings(&settings).expect("planner config")
}

fn print_head(print: &str, lines: usize) -> String {
    let path = source_tree::repo_root().join("tools/sim/gcode").join(print);
    std::fs::read_to_string(&path)
        .unwrap_or_else(|e| panic!("read {}: {e}", path.display()))
        .lines()
        .take(lines)
        .flat_map(|line| [line, "\n"])
        .collect()
}

fn emitted(config: &Path, gcode: &str, emit_batch: usize) -> Vec<ContinuousSegment> {
    let replay = LiveReplay::new(gcode, &planner_config(config), None).expect("the replay builds");
    let mut pipeline = replay.pipeline().with_emit_batch(emit_batch);
    let mut segments = Vec::new();
    let mut collect = |item| {
        if let TrajectoryItem::Seg(segment) = item {
            segments.push(segment);
        }
        true
    };
    for m in replay.moves {
        assert!(
            pipeline.feed(m.into(), &mut collect),
            "{}: the pipeline refused a move",
            config.display()
        );
    }
    assert!(
        pipeline.finish(&mut collect),
        "{}: the pipeline failed to drain",
        config.display()
    );
    assert!(!segments.is_empty(), "{}: no motion", config.display());
    segments
}

fn assert_axes_match(config: &Path, gcode: &str, axes: &[usize]) {
    let alone = emitted(config, gcode, 1);
    let batched = emitted(config, gcode, EMIT_BATCH_SEGMENTS);
    assert_eq!(alone.len(), batched.len(), "{}", config.display());
    for (index, (one, many)) in alone.iter().zip(&batched).enumerate() {
        for &axis in axes {
            assert!(
                format!("{:?}", one.axes[axis]) == format!("{:?}", many.axes[axis]),
                "{}: axis {axis} of segment {index} [{}, {}] differs between one emit per \
                 segment and {EMIT_BATCH_SEGMENTS} segments per emit",
                config.display(),
                one.t_start,
                one.t_end
            );
        }
    }
}

#[test]
fn a_kernel_follower_shapes_the_same_trajectory_however_the_emits_are_batched() {
    assert_axes_match(
        &source_tree::repo_root().join("tools/bench/configs/trident_kernel.cfg"),
        &print_head("voron_dense_top_layers.gcode", 1500),
        &[0, 1, 2, 3],
    );
}

#[test]
fn ratio_ramped_leaders_are_the_same_however_the_emits_are_batched() {
    assert_axes_match(
        &source_tree::manifest_dir().join("tests/configs/bench_ramp.cfg"),
        &print_head("voron_first_layers.gcode", 2486),
        &[0, 1, 2],
    );
}

#[test]
fn a_ratio_ramped_extruder_is_the_same_however_the_emits_are_batched() {
    assert_axes_match(
        &source_tree::manifest_dir().join("tests/configs/bench_ramp.cfg"),
        &print_head("voron_first_layers.gcode", 2486),
        &[0, 1, 2, 3],
    );
}
