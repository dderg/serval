use std::sync::Arc;

use geometry::{GcodePos, MachinePos, MeshGridSpec, ProbedMesh, SurfaceTransform};
use motion_core::seam_test_harness::LiveScript;
use motion_pipeline::{Pipeline, TrajectoryItem};

const Z_FIRST_PRINT: &str = "G90\nG1 X40 Y50 Z10 F6000\nG1 X100 Y100 F6000\n";

fn bench_planner() -> planner_config::PlannerConfig {
    let path = source_tree::repo_file("tools/bench/printer.cfg");
    let text =
        std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("read {}: {e}", path.display()));
    planner_config::from_doc::planner_config_from_text(&text).expect("bench planner config")
}

fn wavy_mesh() -> Arc<SurfaceTransform> {
    let mesh = SurfaceTransform::activate(ProbedMesh {
        grid: MeshGridSpec {
            x_min: 80.0,
            y_min: 80.0,
            dx: 30.0,
            dy: 30.0,
            nx: 3,
            ny: 3,
            tension: 0.2,
        },
        points: vec![0.1, 0.0, -0.1, 0.05, 0.0, -0.05, -0.1, 0.0, 0.1],
        fade: Some((1.0, 10.0, 0.0)),
        zero_reference: (110.0, 110.0),
    })
    .expect("the bench's wavy profile activates");
    Arc::new(mesh)
}

#[test]
fn a_first_move_naming_z_starts_the_plan_under_the_mesh() {
    let mesh = wavy_mesh();
    let origin_z = MachinePos([0.0; 3]).to_gcode(Some(mesh.as_ref())).z();
    let mut script = LiveScript::new(
        Z_FIRST_PRINT,
        &bench_planner(),
        None,
        GcodePos([0.0, 0.0, origin_z]),
    )
    .expect("the print plans");
    let inputs = script.take_inputs(Some(mesh));
    let mut pipeline = Pipeline::new(script.config, script.chains, script.home, 0.0);
    let mut last_end_z = None;
    let mut output = |item| {
        if let TrajectoryItem::Seg(segment) = item {
            last_end_z = Some(
                segment
                    .eval_axis(2, segment.t_end)
                    .expect("z evaluates")
                    .position,
            );
        }
        true
    };

    for input in inputs {
        assert!(pipeline.feed(input, &mut output));
    }
    assert!(pipeline.finish(&mut output));

    let end_z = last_end_z.expect("the print emits motion");
    assert!(
        (end_z - 10.0).abs() < 1e-6,
        "the print ends at z {end_z}, not 10"
    );
}
