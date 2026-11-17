use std::path::PathBuf;
use std::sync::Arc;

use geometry::{GcodePos, MachinePos, MeshGridSpec, ProbedMesh, SurfaceTransform};
use motion_core::seam_test_harness::LiveScript;
use motion_pipeline::{Pipeline, TrajectoryItem};

/// The request klippy's `bed_mesh` hands `set_bed_mesh`.
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

/// A minimized simulator fuzz scenario as `scripts/preflight.py` hands it
/// to `gcode_replay_bench`: its motion config, its expanded print and the
/// mesh the print loads.
fn scenario_file(seed: u32, name: &str) -> String {
    let path: PathBuf = source_tree::manifest_dir()
        .join("tests/fuzz_scenarios")
        .join(format!("seed_{seed}"))
        .join(name);
    std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("read {}: {e}", path.display()))
}

fn replays_over_its_mesh(seed: u32) {
    let planner = planner_config::from_doc::planner_config_from_text(&scenario_file(
        seed,
        "printer.cfg",
    ))
    .expect("the scenario's planner config");
    let request: BedMesh = serde_json::from_str(&scenario_file(seed, "bed_mesh.json"))
        .expect("the scenario's mesh parses");
    let mesh = Arc::new(request.activate());
    let origin_z = MachinePos([0.0; 3]).to_gcode(Some(mesh.as_ref())).z();
    let mut script = LiveScript::new(
        &scenario_file(seed, "print.gcode"),
        &planner,
        None,
        GcodePos([0.0, 0.0, origin_z]),
    )
    .expect("the print plans");
    let inputs = script.take_inputs(Some(mesh));
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

/// Seed 1352 (#930): an extrusion blends into a Z move down through the
/// fade start while the ratio ramp still lays, under an advance no kernel
/// follows.
#[test]
fn a_kernel_free_advance_rides_a_z_move_through_the_fade_start() {
    replays_over_its_mesh(1352);
}

/// Seed 1120 (#910): a Z move from 0.4 to 1.6 peaks on the fade start, so
/// the fade edge and the move's phase seam fall 163 ulps apart while the
/// ratio ramp still lays, under an advance a bell kernel follows.
#[test]
fn an_advance_and_a_bell_on_e_cross_the_fade_start_at_a_z_peak() {
    replays_over_its_mesh(1120);
}
