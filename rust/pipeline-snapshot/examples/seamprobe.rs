//! Scratch: seam metrics for the square corner case.
use pipeline_snapshot::{SnapshotParams, TrajectoryPieces, pipeline_snapshot, seam_metrics};

fn main() {
    let snap = pipeline_snapshot(
        &[
            (0.0, 0.0, 0.0, 0.0, 100.0, 3000.0),
            (10.0, 0.0, 0.0, 0.0, 100.0, 3000.0),
            (10.0, 10.0, 0.0, 0.0, 100.0, 3000.0),
            (0.0, 10.0, 0.0, 0.0, 100.0, 3000.0),
            (0.0, 0.0, 0.0, 0.0, 100.0, 3000.0),
        ],
        SnapshotParams {
            max_velocity: 300.0,
            max_accel: 3000.0,
            square_corner_velocity: 5.0,
            corner_deviation: None,
            max_jerk: 100_000.0,
            max_extrude_only_velocity: None,
            max_extrude_only_accel: None,
            max_path_deviation: None,
            max_accel_deviation: None,
            axis_decls: Vec::new(),
            post_processor_decls: Vec::new(),
        },
    )
    .unwrap();
    let traj = TrajectoryPieces {
        x: snap.traj_x_pieces,
        y: snap.traj_y_pieces,
        z: snap.traj_z_pieces,
        e: snap.traj_e_pieces,
        t_end: snap.traj_t_end,
    };
    let m = seam_metrics(&traj);
    for s in m.worst.iter().take(8) {
        println!(
            "seam t={:.6} axis={} dp={:.3e} dv={:.3e} da={:.3e}",
            s.t, s.axis, s.dp, s.dv, s.da
        );
    }
}
