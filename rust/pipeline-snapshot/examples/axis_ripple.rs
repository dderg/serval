//! Scratch: within-piece smoothness of one axis of a saved snapshot.
//!
//! The snapshot metrics measure discontinuity *at* seams, so they say nothing
//! about a fit that ripples between them - which is what a change to the
//! shaper's sampling can introduce while every seam metric stays put. This
//! samples the axis densely and reports the acceleration's total variation
//! and jerk sign changes, which do see it.
//!
//! `python3 snapshots/run.py --results-dir DIR` writes the trajectories;
//! gunzip one and pass it here with the axis index.
use pipeline_snapshot::{ExactTrajectory, SampleSide};

const SAMPLES: usize = 20_000;

fn main() {
    let mut args = std::env::args().skip(1);
    let path = args
        .next()
        .expect("usage: axis_ripple <snapshot.json> <axis>");
    let axis: usize = args
        .next()
        .expect("usage: axis_ripple <snapshot.json> <axis>")
        .parse()
        .expect("axis index");
    let json: serde_json::Value =
        serde_json::from_slice(&std::fs::read(&path).expect("readable snapshot"))
            .expect("uncompressed snapshot json");
    let traj: ExactTrajectory =
        serde_json::from_value(json["trajectory"].clone()).expect("a snapshot trajectory");
    let (t_start, t_end) = (traj.t_start(), traj.t_end());
    let mut acceleration = Vec::with_capacity(SAMPLES + 1);
    for i in 0..=SAMPLES {
        let t = t_start + (t_end - t_start) * i as f64 / SAMPLES as f64;
        if let Ok(pvaj) = traj.eval_axis(axis, t, SampleSide::Right) {
            println!(
                "{t:.9} {:.9} {:.9} {:.9}",
                pvaj.position, pvaj.velocity, pvaj.acceleration
            );
            acceleration.push(pvaj.acceleration);
        }
    }
    let steps: Vec<f64> = acceleration.windows(2).map(|w| w[1] - w[0]).collect();
    let total_variation: f64 = steps.iter().map(|step| step.abs()).sum();
    let sign_changes = steps.windows(2).filter(|w| w[0] * w[1] < 0.0).count();
    eprintln!(
        "axis {axis}: samples={} acceleration total_variation={total_variation:.6} \
         jerk_sign_changes={sign_changes}",
        acceleration.len()
    );
}
