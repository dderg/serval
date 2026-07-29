//! Evidence base for the PA tower's corner-notch defaults in
//! `klippy/extras/pa_test.py`: runs a real 90-degree clothoid corner and the
//! tower's straight-line notch through the actual pipeline and compares
//! toolhead speed dips and the post-kernel post-advance E signal.
//!
//! Two outputs: (a) a limits sweep showing the clothoid dip is a constant
//! 0.863x of the formula scv (pinned by the pipeline-snapshot regression
//! test), (b) an E-stress comparison showing the smoothing kernel erases the
//! notch/corner shape difference (the corner trades tangential for
//! centripetal accel near the dip; the notch keeps the full tangential
//! budget - a ~1-2 ms v(t) difference that vanishes under the 13 ms kernel).
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
    DipStats { v_min, dwell_ms }
}

fn advance_params(l: Limits) -> SnapshotParams {
    let mut p = params(l);
    let mut e = planner_config::AxisDecl {
        name: "e".into(),
        follows: vec!["x".into(), "y".into(), "z".into()],
        motors: Vec::new(),
        post_processors: vec!["tanh".into(), "st".into()],
    };
    e.post_processors = vec!["tanh".into(), "st".into()];
    p.axis_decls = vec![e];
    p.post_processor_decls = vec![
        planner_config::PostProcessorDecl {
            name: "tanh".into(),
            ty: "tanh_pressure_advance".into(),
            params: [
                ("linear_advance".to_string(), 0.011),
                ("nonlinear_offset".to_string(), 0.147),
                ("linearization_velocity".to_string(), 5.99),
            ]
            .into_iter()
            .collect(),
        },
        planner_config::PostProcessorDecl {
            name: "st".into(),
            ty: "smooth_triangle".into(),
            params: [("smooth_time".to_string(), 0.013)].into_iter().collect(),
        },
    ];
    p
}

fn eval_axis_v(pieces: &[Vec<f64>], t: f64) -> f64 {
    let Some(row) = pieces
        .iter()
        .find(|r| t >= r[0] && t <= r[1])
        .or_else(|| pieces.iter().find(|r| t <= r[1]))
    else {
        return 0.0;
    };
    let tau = t - row[0];
    row[2..]
        .iter()
        .enumerate()
        .skip(1)
        .map(|(i, c)| (i as f64) * c * tau.powi(i as i32 - 1))
        .sum()
}

fn ve_around_interior_dip(snap: &Snapshot, dt: f64, cruise: f64, half_window_s: f64) -> Vec<f64> {
    let profile = speed_profile(snap, dt);
    let first = profile
        .iter()
        .position(|&(_, v)| v >= 0.99 * cruise)
        .expect("reaches cruise");
    let last = profile
        .iter()
        .rposition(|&(_, v)| v >= 0.99 * cruise)
        .expect("returns to cruise");
    let dip_t = profile[first..=last]
        .iter()
        .min_by(|a, b| a.1.total_cmp(&b.1))
        .expect("non-empty window")
        .0;
    let n = (half_window_s / dt) as i64;
    (-n..=n)
        .map(|k| eval_axis_v(&snap.traj_e_pieces, dip_t + (k as f64) * dt))
        .collect()
}

fn report_aligned_diff(label: &str, corner: &Snapshot, notch: &Snapshot, cruise: f64, dt: f64) {
    let a = ve_around_interior_dip(corner, dt, cruise, 0.03);
    let b = ve_around_interior_dip(notch, dt, cruise, 0.03);
    let swing =
        a.iter().copied().fold(f64::MIN, f64::max) - a.iter().copied().fold(f64::MAX, f64::min);
    let margin = (0.005 / dt) as usize;
    let core = &a[margin..a.len() - margin];
    let stats_at = |shift: i64| {
        let mut peak: f64 = 0.0;
        let mut sq = 0.0;
        for (i, &va) in core.iter().enumerate() {
            let vb = b[(i + margin).wrapping_add_signed(shift as isize)];
            let d = (va - vb).abs();
            peak = peak.max(d);
            sq += d * d;
        }
        (peak, (sq / core.len() as f64).sqrt())
    };
    let mut best = (f64::MAX, 0.0_f64, 0_i64);
    for shift in -(margin as i64)..=(margin as i64) {
        let (peak, rms) = stats_at(shift);
        if rms < best.0 {
            best = (rms, peak, shift);
        }
    }
    let (peak0, rms0) = stats_at(0);
    let ext = |w: &[f64]| {
        (
            w.iter().copied().fold(f64::MAX, f64::min),
            w.iter().copied().fold(f64::MIN, f64::max),
        )
    };
    let (ca, cb) = ext(&a);
    let (na, nb) = ext(&b);
    println!(
        "{label}vE corner [{ca:.3},{cb:.3}] notch [{na:.3},{nb:.3}] swing {swing:.2} | dip-aligned peak {:.1}% rms {:.1}% | best-shift ({:+.2}ms) peak {:.1}% rms {:.1}%",
        100.0 * peak0 / swing,
        100.0 * rms0 / swing,
        best.2 as f64 * dt * 1e3,
        100.0 * best.1 / swing,
        100.0 * best.0 / swing
    );
}

fn main() {
    let dt = 2e-5;
    let scv_factor = std::f64::consts::SQRT_2 - 1.0;
    println!("--- clothoid dip vs 0.02mm notch, full sweep (tanh 0.011/0.147/5.99, st 13ms on E)");
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
                        pipeline_snapshot(&corner_path(l), advance_params(l)).expect("corner");
                    let c = dip_stats(&speed_profile(&corner, dt), v_wall);
                    let notch = pipeline_snapshot(&notch_path(l, 0.02, c.v_min), advance_params(l))
                        .expect("notch");
                    print!(
                        "a={accel:>6.0} j={jerk:.0e} dev={deviation:.2} v={v_wall:>3.0} | vmin/scv {:.4} dwell {:>5.2}ms | ",
                        c.v_min / scv,
                        c.dwell_ms
                    );
                    report_aligned_diff("", &corner, &notch, v_wall, dt);
                }
            }
        }
    }
}
