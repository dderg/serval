//! Scratch: seam metrics for the square corner case.
use pipeline_snapshot::{SnapshotParams, pipeline_snapshot};

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
            max_jerk: f64::INFINITY,
            max_extrude_only_velocity: None,
            max_extrude_only_accel: None,
            max_path_deviation: None,
            max_accel_deviation: None,
            axis_decls: Vec::new(),
            post_processor_decls: Vec::new(),
        },
    )
    .unwrap();
    for s in snap.worst_seams.iter().take(8) {
        println!(
            "seam t={:.6} axis={} dp={:.3e} dv={:.3e} da={:.3e}",
            s.t, s.axis, s.dp, s.dv, s.da
        );
    }
}
