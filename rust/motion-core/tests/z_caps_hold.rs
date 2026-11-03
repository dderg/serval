//! `max_z_velocity` and `max_z_accel` bound the z axis wherever the path
//! moves it — on the XYZ lines the G-code names and on every corner blend
//! the fitter lays between them, where a turn into or out of a z change puts
//! z in the blend's normal acceleration. Each case plans through the live
//! path (per-move limits from `live_config`) and every emitted carrier's z
//! state is held to the caps.

use motion_core::live_config;
use motion_core::seam_test_harness::parse_gcode_to_moves;
use motion_pipeline::{Pipeline, TrajectoryItem};
use planner_config::PlannerConfig;

const Z_AXIS: usize = 2;
const SAMPLES_PER_SEGMENT: usize = 64;

/// The bench printer driven like a Trident at speed: corners sized by a
/// 5 mm/s square corner velocity under 70 000 mm/s², so a layer change
/// blends tight and fast.
fn trident_printer() -> String {
    let trident = repo_file("tools/bench/printer.cfg")
        .replace("max_accel: 10000\n", "max_accel: 70000\n")
        .replace("corner_deviation: 0.21\n", "square_corner_velocity: 5\n");
    assert!(
        trident.contains("max_accel: 70000\n") && trident.contains("square_corner_velocity: 5\n"),
        "tools/bench/printer.cfg no longer carries the [printer] lines this test rewrites"
    );
    trident
}

const LAYER_CHANGE: &str = "M83
G1 X100 Y150 Z40.2 F60000
G1 F25953.656
G1 X120.512 Y149.536 E.5
G1 X120.123 Y150.439 Z40.4 F60000
G1 F25953.656
G1 X119 Y151 E.05
G1 X100 Y160 E.5
";

const Z_HOP_INTO_XY: &str = "G1 X10 Y10 F60000
G1 Z.2 F1500
G1 X60 Y10 F24000
G1 X60 Y60
";

const SLOPE_CHANGE: &str = "M83
G1 X0 Y10 Z.2 F60000
G1 X60 Y10 E2 F12000
G1 X110 Y10 Z.4 F60000
G1 X160 Y10 E2 F12000
";

fn repo_file(relative: &str) -> String {
    let path = source_tree::repo_file(relative);
    std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("read {}: {e}", path.display()))
}

fn planner_config(text: &str, name: &str) -> PlannerConfig {
    let doc = config_doc::Document::parse(text, name).expect("config parses");
    let (settings, _) =
        planner_config::from_doc::read_motion_settings(&doc).expect("motion settings");
    planner_config::from_doc::planner_config_from_settings(&settings).expect("planner config")
}

fn configs() -> Vec<(&'static str, PlannerConfig)> {
    vec![
        ("trident", planner_config(&trident_printer(), "trident")),
        (
            "bench",
            planner_config(&repo_file("tools/bench/printer.cfg"), "printer.cfg"),
        ),
        (
            "fast",
            planner_config(&repo_file("tools/bench/configs/fast.cfg"), "fast.cfg"),
        ),
    ]
}

fn first_lines(relative: &str, count: usize) -> String {
    repo_file(relative)
        .lines()
        .take(count)
        .flat_map(|line| [line, "\n"])
        .collect()
}

/// Peak |z velocity| and |z acceleration| over every emitted carrier.
fn z_peaks(gcode: &str, planner: &PlannerConfig) -> (f64, f64) {
    let config = live_config::stream_config(planner).expect("stream config");
    let chains = planner.compile_active_chains().expect("chains compile");
    let moves = parse_gcode_to_moves(
        gcode,
        live_config::move_limits(planner).expect("move limits"),
    );
    let mut segments = Vec::new();
    let mut collect = |item| {
        if let TrajectoryItem::Seg(segment) = item {
            segments.push(segment);
        }
        true
    };
    let mut pipeline = Pipeline::new(config, chains, vec![0.0; 4], 0.0);
    for m in moves {
        assert!(pipeline.feed(m.into(), &mut collect));
    }
    assert!(pipeline.finish(&mut collect));
    assert!(!segments.is_empty());
    let mut peaks = (0.0_f64, 0.0_f64);
    for segment in &segments {
        for k in 0..=SAMPLES_PER_SEGMENT {
            let t = segment.t_start
                + (segment.t_end - segment.t_start) * k as f64 / SAMPLES_PER_SEGMENT as f64;
            let state = segment.eval_axis_pvaj(Z_AXIS, t).expect("z evaluates");
            peaks.0 = peaks.0.max(state.velocity.abs());
            peaks.1 = peaks.1.max(state.acceleration.abs());
        }
    }
    peaks
}

#[test]
fn every_emitted_z_state_holds_the_z_caps() {
    let cases = [
        ("layer change", LAYER_CHANGE.to_string()),
        ("z hop into xy", Z_HOP_INTO_XY.to_string()),
        ("slope change", SLOPE_CHANGE.to_string()),
        (
            "voron_first_layers",
            first_lines("tools/sim/gcode/voron_first_layers.gcode", 120),
        ),
        (
            "voron_dense_top_layers",
            first_lines("tools/sim/gcode/voron_dense_top_layers.gcode", 340),
        ),
    ];
    let mut excess = Vec::new();
    for (config_name, planner) in configs() {
        let caps = planner.cartesian;
        let accel_bound = caps.max_z_accel * (1.0 + 1e-6);
        let velocity_bound = caps.max_z_velocity * (1.0 + 1e-6);
        for (case, gcode) in &cases {
            let (velocity, accel) = z_peaks(gcode, &planner);
            if accel > accel_bound || velocity > velocity_bound {
                excess.push(format!(
                    "{config_name} / {case}: z reaches {velocity:.3} mm/s and {accel:.1} mm/s² \
                     against max_z_velocity {} and max_z_accel {}",
                    caps.max_z_velocity, caps.max_z_accel
                ));
            }
        }
    }
    assert!(excess.is_empty(), "{}", excess.join("\n"));
}
