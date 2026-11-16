use std::sync::Arc;

use geometry::{Fade, MachinePos, MeshGrid, MeshGridSpec, SurfaceTransform};
use motion_pipeline::{Control, Pipeline, StreamInput, live_config};
use pipeline_snapshot::waypoints::Waypoint;
use pipeline_snapshot::{PlanStep, plan_moves, plan_steps, rest_position_before};
use planner_config::from_doc::planner_config_from_text;

mod corpus;

fn wavy_mesh() -> Arc<SurfaceTransform> {
    let mut mesh = MeshGrid::new(
        MeshGridSpec {
            x_min: 80.0,
            y_min: 80.0,
            dx: 30.0,
            dy: 30.0,
            nx: 3,
            ny: 3,
            tension: 0.2,
        },
        vec![0.10, 0.00, -0.10, 0.05, 0.00, -0.05, -0.10, 0.00, 0.10],
    )
    .unwrap();
    mesh.zero_at(110.0, 110.0);
    Arc::new(SurfaceTransform::new(
        mesh,
        Fade::new(1.0, 10.0, 0.0).unwrap(),
    ))
}

fn rebase_z_until_the_first_z_move(waypoints: &mut [Waypoint], gcode_z: f64) {
    let start_z = waypoints[0].position[2];
    for waypoint in waypoints
        .iter_mut()
        .take_while(|w| w.position[2] == start_z)
    {
        waypoint.position[2] = gcode_z;
    }
}

#[test]
fn the_first_layers_print_over_a_wavy_mesh_on_the_bench() {
    let config_path = corpus::repo().join(corpus::PRINTER_CONFIG);
    let planner =
        planner_config_from_text(&std::fs::read_to_string(&config_path).unwrap()).unwrap();
    let mesh = wavy_mesh();
    let rebase = MachinePos([0.0, 0.0, 0.0]).to_gcode(Some(&mesh)).z();
    let mut waypoints = corpus::waypoints(&corpus::sim_print("voron_first_layers"), &config_path);
    rebase_z_until_the_first_z_move(&mut waypoints, rebase);
    let base =
        live_config::GcodeMoveBase::of(&planner, &live_config::extruder_caps(&planner).unwrap())
            .unwrap();
    let stream = live_config::stream_config(&planner, None).unwrap();
    let chains = planner.compile_active_chains().unwrap();
    let steps = plan_steps(&waypoints, base).unwrap();
    let home = rest_position_before(plan_moves(&steps));

    let mut pipeline = Pipeline::new(stream, chains, home, 0.0);
    let mut segments = 0usize;
    let mut count = |_| {
        segments += 1;
        true
    };
    assert!(pipeline.feed(
        StreamInput::Control(Control::SetMesh {
            mesh: Some(mesh),
            gcode_z_rebase: rebase,
        }),
        &mut count,
    ));
    for step in steps {
        assert!(PlanStep::feed(step, |input| pipeline.feed(input, &mut count)));
    }
    assert!(pipeline.finish(&mut count));
    assert!(segments > 0);
}
