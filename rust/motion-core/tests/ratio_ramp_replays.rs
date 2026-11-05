use std::path::PathBuf;

use motion_core::seam_test_harness::LiveReplay;
use motion_pipeline::TrajectoryItem;

fn repo() -> PathBuf {
    source_tree::repo_root()
}

fn planner_config(name: &str) -> planner_config::PlannerConfig {
    let path = source_tree::manifest_dir().join("tests/configs").join(name);
    let text =
        std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("read {}: {e}", path.display()));
    let doc =
        config_doc::Document::parse(&text, &path.display().to_string()).expect("config parses");
    let (settings, _) =
        planner_config::from_doc::read_motion_settings(&doc).expect("motion settings");
    planner_config::from_doc::planner_config_from_settings(&settings).expect("planner config")
}

fn print_head(print: &str, prologue: &str, lines: usize) -> String {
    let print = std::fs::read_to_string(repo().join("tools/sim/gcode").join(print))
        .unwrap_or_else(|e| panic!("read {print}: {e}"));
    std::iter::once(prologue)
        .chain(print.lines().take(lines).flat_map(|line| [line, "\n"]))
        .collect()
}

fn dense_print_head(prologue: &str, lines: usize) -> String {
    print_head("voron_dense_top_layers.gcode", prologue, lines)
}

fn replays_to_the_end(config: &str, gcode: &str) {
    let replay = LiveReplay::new(gcode, &planner_config(config), None).expect("the replay builds");
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

fn zigzag_into_a_stop(speed_mm_s: f64) -> String {
    let ratio = 0.4 * 0.2 / (std::f64::consts::PI * 0.875 * 0.875);
    let rise = 2.0 / std::f64::consts::SQRT_2;
    let mut points = vec![(30.0, 0.0)];
    for turn in 0..=20 {
        let (x, _) = points[points.len() - 1];
        points.push((x + rise, if turn % 2 == 0 { rise } else { 0.0 }));
    }
    let (x, y) = points[points.len() - 1];
    points.push((x + 30.0, y));
    let mut gcode = format!("G90\nM83\nG1 X0 Y0 F{:.0}\n", speed_mm_s * 60.0);
    let mut from = (0.0, 0.0);
    for to in points {
        let length = libm::hypot(to.0 - from.0, to.1 - from.1);
        gcode += &format!("G1 X{:.5} Y{:.5} E{:.6}\n", to.0, to.1, ratio * length);
        from = to;
    }
    gcode
}

#[test]
fn a_ratio_ramp_waits_for_how_far_a_stop_ahead_may_release_the_advance() {
    replays_to_the_end("p4_trident_ramp_slow.cfg", &zigzag_into_a_stop(100.0));
}

#[test]
fn a_kernel_after_a_ratio_ramp_plans_the_first_layers_of_a_print() {
    replays_to_the_end(
        "bench_ramp_hybrid.cfg",
        &print_head("voron_first_layers.gcode", "", 1820),
    );
}

#[test]
fn a_ratio_ramp_plans_the_first_layers_of_a_print() {
    replays_to_the_end(
        "bench_ramp.cfg",
        &print_head("voron_first_layers.gcode", "", 2486),
    );
}

/// The sim's print test: a 20 000 mm/s² extruder holds its kernel-free
/// advance's fit to its limit only where a span can still split; a sliver
/// at the resolution floor keeps the budget its fit resolved for it.
#[test]
fn a_kernel_free_advance_plans_the_first_layers_of_the_sim_print() {
    replays_to_the_end(
        "neptune_sim_print.cfg",
        &print_head("voron_first_layers.gcode", "", 4499),
    );
}
