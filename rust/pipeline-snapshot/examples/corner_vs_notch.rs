//! Temporary experiment: can a straight-line slow "notch" (the PA tower's
//! corner stand-in) reproduce the toolhead speed profile of a real clothoid
//! corner in this planner? Runs a 90-degree corner and straight lines with
//! notches of several lengths through the real pipeline, then compares dip
//! depth, dwell near the minimum, and total transient duration.
//!
//! Run: cargo run --release -p pipeline-snapshot --example corner_vs_notch

use pipeline_snapshot::waypoints::Waypoint;
use pipeline_snapshot::{Snapshot, SnapshotParams, pipeline_snapshot};

const EXTR_R: f64 = 0.04;

#[derive(Clone, Copy)]
struct Limits {
    v_wall: f64,
    accel: f64,
    jerk: f64,
    deviation: f64,
}

fn params(l: Limits) -> SnapshotParams {
    SnapshotParams {
        max_velocity: 2800.0,
        max_accel: l.accel,
        square_corner_velocity: 5.0,
        corner_deviation: Some(l.deviation),
        max_jerk: l.jerk,
        max_extrude_only_velocity: None,
        max_extrude_only_accel: None,
        max_path_deviation: None,
        max_accel_deviation: None,
        axis_decls: Vec::new(),
        post_processor_decls: Vec::new(),
    }
}

fn corner_path(l: Limits) -> Vec<Waypoint> {
    let mut e = 0.0;
    let mut wp = vec![(0.0, 0.0, 0.2, e, l.v_wall, l.accel)];
    e += 60.0 * EXTR_R;
    wp.push((60.0, 0.0, 0.2, e, l.v_wall, l.accel));
    e += 60.0 * EXTR_R;
    wp.push((60.0, 60.0, 0.2, e, l.v_wall, l.accel));
    wp
}

fn notch_path(l: Limits, notch_mm: f64, notch_v: f64) -> Vec<Waypoint> {
    let mut e = 0.0;
    let mut wp = vec![(0.0, 0.0, 0.2, e, l.v_wall, l.accel)];
    e += 30.0 * EXTR_R;
    wp.push((30.0, 0.0, 0.2, e, l.v_wall, l.accel));
    e += notch_mm * EXTR_R;
    wp.push((30.0 + notch_mm, 0.0, 0.2, e, notch_v, l.accel));
    e += (60.0 - notch_mm) * EXTR_R;
    wp.push((90.0, 0.0, 0.2, e, l.v_wall, l.accel));
    wp
}

fn speed_profile(snap: &Snapshot, dt: f64) -> Vec<(f64, f64)> {
    let eval_v = |pieces: &[Vec<f64>], t: f64| -> Option<f64> {
        let row = pieces
            .iter()
            .find(|r| t >= r[0] && t <= r[1])
            .or_else(|| pieces.iter().find(|r| t <= r[1]))?;
        let tau = t - row[0];
        let coeffs = &row[2..];
        let mut v = 0.0;
        for (i, c) in coeffs.iter().enumerate().skip(1) {
            v += (i as f64) * c * tau.powi(i as i32 - 1);
        }
        Some(v)
    };
    let mut out = Vec::new();
    let mut t = 0.0;
    while t <= snap.traj_t_end {
        let vx = eval_v(&snap.traj_x_pieces, t).unwrap_or(0.0);
        let vy = eval_v(&snap.traj_y_pieces, t).unwrap_or(0.0);
        out.push((t, libm::hypot(vx, vy)));
        t += dt;
    }
    out
}

struct DipStats {
    v_min: f64,
    dwell_ms: f64,
    transient_ms: f64,
}

fn dip_stats(profile: &[(f64, f64)], cruise: f64) -> DipStats {
    let first = profile
        .iter()
        .position(|&(_, v)| v >= 0.99 * cruise)
        .expect("path must reach cruise before the dip");
    let last = profile
        .iter()
        .rposition(|&(_, v)| v >= 0.99 * cruise)
        .expect("path must return to cruise after the dip");
    let profile = &profile[first..=last];
    let v_min = profile.iter().map(|&(_, v)| v).fold(f64::MAX, f64::min);
    let dt = profile[1].0 - profile[0].0;
    let dwell_ms = profile.iter().filter(|&&(_, v)| v <= 1.05 * v_min).count() as f64 * dt * 1e3;
    let transient_ms =
        profile.iter().filter(|&&(_, v)| v <= 0.95 * cruise).count() as f64 * dt * 1e3;
    DipStats {
        v_min,
        dwell_ms,
        transient_ms,
    }
}

fn main() {
    let dt = 2e-5;
    let scv_factor = std::f64::consts::SQRT_2 - 1.0;
    for accel in [20_000.0, 50_000.0, 100_000.0] {
        for jerk in [5e6, 2e7] {
            for deviation in [0.02, 0.05, 0.1] {
                for v_wall in [80.0, 150.0, 300.0] {
                    let l = Limits {
                        v_wall,
                        accel,
                        jerk,
                        deviation,
                    };
                    let scv = (deviation * accel / scv_factor).sqrt();
                    if scv >= 0.9 * v_wall {
                        continue;
                    }
                    let corner =
                        pipeline_snapshot(&corner_path(l), params(l)).expect("corner snapshot");
                    let c = dip_stats(&speed_profile(&corner, dt), v_wall);
                    let notch_mm = (scv * 0.001).clamp(0.02, 0.2);
                    let snap = pipeline_snapshot(&notch_path(l, notch_mm, scv), params(l))
                        .expect("notch snapshot");
                    let n = dip_stats(&speed_profile(&snap, dt), v_wall);
                    println!(
                        "a={accel:>6.0} j={jerk:.0e} dev={deviation:.2} v={v_wall:>3.0} | corner: vmin {:>6.2} (scv {scv:>6.2}) dwell {:>5.2}ms trans {:>6.2}ms | notch {notch_mm:.2}mm: vmin {:>6.2} dwell {:>5.2}ms trans {:>6.2}ms",
                        c.v_min, c.dwell_ms, c.transient_ms, n.v_min, n.dwell_ms, n.transient_ms
                    );
                }
            }
        }
    }
}
