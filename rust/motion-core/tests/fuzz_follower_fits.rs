use std::path::PathBuf;
use std::sync::Arc;

use geometry::{GcodePos, MachinePos, MeshGridSpec, ProbedMesh, SurfaceTransform};
use motion_core::seam_test_harness::LiveScript;
use motion_pipeline::{Pipeline, TrajectoryItem};

#[derive(serde::Deserialize)]
struct BedMesh {
    points: Vec<f64>,
    x_min: f64,
    y_min: f64,
    dx: f64,
    dy: f64,
    nx: usize,
    ny: usize,
    tension: f64,
    fade: Option<(f64, f64, f64)>,
    zero_ref_x: f64,
    zero_ref_y: f64,
}

impl BedMesh {
    fn activate(self) -> SurfaceTransform {
        SurfaceTransform::activate(ProbedMesh {
            grid: MeshGridSpec {
                x_min: self.x_min,
                y_min: self.y_min,
                dx: self.dx,
                dy: self.dy,
                nx: self.nx,
                ny: self.ny,
                tension: self.tension,
            },
            points: self.points,
            fade: self.fade,
            zero_reference: (self.zero_ref_x, self.zero_ref_y),
        })
        .expect("the scenario's mesh activates")
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Mesh {
    None,
    Loaded,
}

/// A minimized simulator fuzz scenario as `scripts/preflight.py` hands it
/// to `gcode_replay_bench`: its motion config, its expanded print and,
/// when the print loads one, its mesh.
fn scenario_file(seed: u32, name: &str) -> String {
    let path: PathBuf = source_tree::manifest_dir()
        .join("tests/fuzz_scenarios")
        .join(format!("seed_{seed}"))
        .join(name);
    std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("read {}: {e}", path.display()))
}

fn replays_to_the_end(seed: u32, mesh: Mesh) {
    let planner = planner_config::from_doc::planner_config_from_text(&scenario_file(
        seed,
        "printer.cfg",
    ))
    .expect("the scenario's planner config");
    let mesh = (mesh == Mesh::Loaded).then(|| {
        let request: BedMesh = serde_json::from_str(&scenario_file(seed, "bed_mesh.json"))
            .expect("the scenario's mesh parses");
        Arc::new(request.activate())
    });
    let origin_z = mesh
        .as_deref()
        .map_or(0.0, |mesh| MachinePos([0.0; 3]).to_gcode(Some(mesh)).z());
    let mut script = LiveScript::new(
        &scenario_file(seed, "print.gcode"),
        &planner,
        None,
        GcodePos([0.0, 0.0, origin_z]),
    )
    .expect("the print plans");
    let inputs = script.take_inputs(mesh);
    let mut pipeline = Pipeline::new(script.config, script.chains, script.home, 0.0);
    let mut emitted = 0usize;
    let mut count = |item| {
        if let TrajectoryItem::Seg(_) = item {
            emitted += 1;
        }
        true
    };
    for input in inputs {
        assert!(pipeline.feed(input, &mut count), "the pipeline refused an input");
    }
    assert!(pipeline.finish(&mut count), "the pipeline failed to drain");
    assert!(emitted > 0, "the replay emitted no motion");
}

#[test]
fn an_advance_and_a_kernel_on_e_retract_around_a_dwell() {
    replays_to_the_end(34, Mesh::None);
}

#[test]
fn an_advance_and_a_bell_on_e_zigzag_over_a_steep_mesh() {
    replays_to_the_end(1120, Mesh::Loaded);
}

#[test]
fn a_kernel_free_advance_prints_squares_over_a_steep_mesh() {
    replays_to_the_end(1352, Mesh::Loaded);
}

#[test]
fn a_plain_follower_rides_tiny_arcs_under_a_bell_on_y() {
    replays_to_the_end(1278, Mesh::None);
}

#[test]
fn an_advance_and_a_kernel_on_e_retract_under_bell_leaders() {
    replays_to_the_end(2031, Mesh::None);
}
