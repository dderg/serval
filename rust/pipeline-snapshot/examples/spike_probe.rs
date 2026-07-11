//! Scratch investigation tool (not committed): runs fitter + planner on a
//! snapshot case and reports moves whose planned velocity samples violate the
//! curvature acceleration ceiling `v^2 * kappa <= max_accel`.
//!
//!   cargo run -p pipeline-snapshot --example spike_probe -- <file.gcode> \
//!       <max_velocity> <max_accel> <scv> <max_jerk>

use crossbeam_channel::unbounded;
use geometry::path::CurvatureProfile;
use geometry::path::lowering::PositionProfile;
use motion_pipeline::fit_stage::FitStage;
use motion_pipeline::planner::Planner;
use motion_pipeline::{PlannedItem, StreamConfig, StreamInput};
use pipeline_snapshot::waypoints::parse_gcode;
use pipeline_snapshot::{
    SNAPSHOT_MAX_BUFFER_MOVES, TRAJECTORY_FIT_TOL_ACCEL_MM_S2, TRAJECTORY_FIT_TOL_MM,
    VELOCITY_INTEGRATION_TOL, build_moves,
};

fn main() {
    let mut args = std::env::args().skip(1);
    let path = args.next().expect("gcode path");
    let max_velocity: f64 = args.next().expect("max_velocity").parse().unwrap();
    let max_accel: f64 = args.next().expect("max_accel").parse().unwrap();
    let scv: f64 = args.next().expect("scv").parse().unwrap();
    let max_jerk: f64 = args.next().expect("max_jerk").parse().unwrap();

    let text = std::fs::read_to_string(&path).expect("read gcode");
    let waypoints = parse_gcode(&text, max_velocity).expect("parse gcode");
    let corner_deviation = geometry::corner_deviation_from_scv(scv, max_accel);
    let limits =
        geometry::VelocityLimits::try_new(max_velocity, max_accel, corner_deviation, max_jerk)
            .expect("limits");
    let moves = build_moves(&waypoints, limits).expect("moves");

    let config = StreamConfig {
        corner: geometry::CornerFitConfig::default(),
        integration_tol: VELOCITY_INTEGRATION_TOL,
        max_extrude_only_velocity_mm_s: 100.0,
        max_extrude_only_accel_mm_s2: 1000.0,
        fit_tol_mm: TRAJECTORY_FIT_TOL_MM,
        fit_tol_accel_mm_s2: TRAJECTORY_FIT_TOL_ACCEL_MM_S2,
        max_buffer_moves: SNAPSHOT_MAX_BUFFER_MOVES,
        limits,
    };

    let (fitted_tx, fitted_rx) = unbounded();
    let (planned_tx, planned_rx) = unbounded();
    let mut fit = FitStage::new(config.corner).into_driver(fitted_tx);
    let mut planner = Planner::new(config);

    let mut planned: Vec<motion_pipeline::types::PlannedMove> = Vec::new();
    let pump = |planner: &mut Planner,
                fitted_rx: &crossbeam_channel::Receiver<StreamInput>,
                planned: &mut Vec<motion_pipeline::types::PlannedMove>| {
        while let Ok(item) = fitted_rx.try_recv() {
            assert!(planner.feed(item, &planned_tx));
        }
        while let Ok(item) = planned_rx.try_recv() {
            if let PlannedItem::Move(m) = item {
                planned.push(m);
            }
        }
    };

    for m in moves {
        assert!(fit.feed(m.into()));
        pump(&mut planner, &fitted_rx, &mut planned);
    }
    assert!(fit.finish());
    pump(&mut planner, &fitted_rx, &mut planned);
    assert!(planner.finish(&planned_tx));
    pump(&mut planner, &fitted_rx, &mut planned);

    println!("planned moves: {}", planned.len());
    let mut t = 0.0_f64;
    let mut worst: Vec<(f64, String)> = Vec::new();
    for (i, pm) in planned.iter().enumerate() {
        let dur: f64 = if pm.velocity.phases.is_empty() {
            // approximate from samples
            pm.velocity
                .samples
                .windows(2)
                .map(|w| {
                    let ds = w[1].s - w[0].s;
                    let vs = w[0].v + w[1].v;
                    if vs > 0.0 { 2.0 * ds / vs } else { 0.0 }
                })
                .sum()
        } else {
            pm.velocity.phases.iter().map(|p| p.dt).sum()
        };
        let seg = match &pm.geometry.segment.spatial {
            Some(s) => s,
            None => {
                t += dur;
                continue;
            }
        };
        let (k0, k1) = seg.kappa_endpoints();
        let len = seg.s_len();
        let sigma = if len > 0.0 { (k1 - k0) / len } else { 0.0 };
        let mut max_ratio = 0.0_f64;
        let mut at = (0.0, 0.0, 0.0); // s, v, kappa
        for s in &pm.velocity.samples {
            let kappa = (k0 + sigma * s.s).abs();
            let ratio = s.v * s.v * kappa / max_accel;
            if ratio > max_ratio {
                max_ratio = ratio;
                at = (s.s, s.v, kappa);
            }
        }
        if max_ratio > 1.05 {
            let kind = match seg {
                geometry::path::Segment::Line(_) => "line",
                geometry::path::Segment::Arc(_) => "arc",
                geometry::path::Segment::Clothoid(_) => "clothoid",
            };
            worst.push((
                max_ratio,
                format!(
                    "move {i} line {} t={:.6} {kind} len={:.5} k0={:.4} k1={:.4} \
                     entry_v={:.2} exit_v={:.2} peak_v={:.2} | worst s={:.5} v={:.2} kappa={:.4} \
                     -> a_c={:.0} ({:.2}x limit) n_samples={} n_phases={}",
                    pm.geometry.source.start_line,
                    t,
                    seg.s_len(),
                    k0,
                    k1,
                    pm.velocity.entry_v,
                    pm.velocity.exit_v,
                    pm.velocity.peak_v,
                    at.0,
                    at.1,
                    at.2,
                    at.1 * at.1 * at.2,
                    max_ratio,
                    pm.velocity.samples.len(),
                    pm.velocity.phases.len(),
                ),
            ));
        }
        t += dur;
    }
    worst.sort_by(|a, b| b.0.total_cmp(&a.0));
    println!(
        "moves violating v^2*kappa <= accel (>1.05x): {}",
        worst.len()
    );
    for (_, line) in worst.iter().take(20) {
        println!("  {line}");
    }

    // Whole-file lowering audit: max per-axis acceleration of every lowered
    // piece, with the worst offenders listed.
    if std::env::var("AUDIT_ALL").is_ok() {
        let fit_tol = motion_pipeline::lowering::FitTol {
            pos_mm: TRAJECTORY_FIT_TOL_MM,
            accel_mm_s2: TRAJECTORY_FIT_TOL_ACCEL_MM_S2,
        };
        let chains: Vec<trajectory::CompiledChain> = vec![trajectory::CompiledChain::default(); 4];
        let acc_extreme = |c: &[f64], h: f64| -> f64 {
            let acc = |tau: f64| -> f64 {
                (2..c.len())
                    .map(|k| c[k] * (k * (k - 1)) as f64 * tau.powi(k as i32 - 2))
                    .sum()
            };
            (0..=16)
                .map(|k| acc(h * k as f64 / 16.0).abs())
                .fold(0.0, f64::max)
        };
        let jerk_extreme = |c: &[f64], h: f64| -> f64 {
            let jrk = |tau: f64| -> f64 {
                (3..c.len())
                    .map(|k| c[k] * (k * (k - 1) * (k - 2)) as f64 * tau.powi(k as i32 - 3))
                    .sum()
            };
            (0..=16)
                .map(|k| jrk(h * k as f64 / 16.0).abs())
                .fold(0.0, f64::max)
        };
        let mut worst_jerk = 0.0_f64;
        let mut worst_acc: Vec<(f64, u32)> = Vec::new();
        for pm in planned.iter() {
            let Some(seg) = pm.geometry.segment.spatial.as_ref() else {
                continue;
            };
            let mut start_pos = vec![0.0_f64; 4];
            start_pos[..3].copy_from_slice(&seg.point_at(0.0));
            let (pieces, _) = motion_pipeline::lowering::lower_move_pieces(
                &pm.geometry,
                &pm.velocity,
                0.0,
                &start_pos,
                fit_tol,
                &chains,
                None,
            )
            .expect("lower");
            let mut mv_max = 0.0_f64;
            for ps in pieces.iter().take(3) {
                for p in ps {
                    mv_max = mv_max.max(acc_extreme(&p.coeffs, p.u_end - p.u_start));
                    worst_jerk = worst_jerk.max(jerk_extreme(&p.coeffs, p.u_end - p.u_start));
                }
            }
            if std::env::var_os("AUDIT_KIND").is_some() {
                let kind = match seg {
                    geometry::path::Segment::Line(_) => "line",
                    geometry::path::Segment::Arc(_) => "arc",
                    geometry::path::Segment::Clothoid(_) => "clothoid",
                };
                let mut mv_jerk = 0.0_f64;
                for ps in pieces.iter().take(3) {
                    for p in ps {
                        mv_jerk = mv_jerk.max(jerk_extreme(&p.coeffs, p.u_end - p.u_start));
                    }
                }
                println!(
                    "  move kind={kind} line={} len={:.6} entry={:.4} exit={:.4} max={mv_max:.1} jmax={mv_jerk:.3e}",
                    pm.geometry.source.start_line,
                    pm.geometry.segment.s_len(),
                    pm.velocity.entry_v,
                    pm.velocity.exit_v
                );
            }
            worst_acc.push((mv_max, pm.geometry.source.start_line));
        }
        worst_acc.sort_by(|a, b| b.0.total_cmp(&a.0));
        println!(
            "AUDIT_ALL: global max per-axis accel = {:.1}, max per-axis jerk = {:.3e}",
            worst_acc[0].0, worst_jerk
        );
        for (a, line) in worst_acc.iter().take(10) {
            println!("  line {line} max_axis_accel={a:.1}");
        }
    }

    // Detail + lowering audit for moves from specific source lines.
    let detail_lines: Vec<u32> = std::env::var("DETAIL_LINES")
        .map(|s| s.split(',').filter_map(|x| x.parse().ok()).collect())
        .unwrap_or_default();
    if detail_lines.is_empty() {
        return;
    }
    let fit_tol = motion_pipeline::lowering::FitTol {
        pos_mm: TRAJECTORY_FIT_TOL_MM,
        accel_mm_s2: TRAJECTORY_FIT_TOL_ACCEL_MM_S2,
    };
    let chains: Vec<trajectory::CompiledChain> = vec![trajectory::CompiledChain::default(); 4];
    let mut start_pos = vec![0.0_f64; 4];
    if let Some(seg) = planned
        .iter()
        .find_map(|pm| pm.geometry.segment.spatial.as_ref())
    {
        let p = seg.point_at(0.0);
        start_pos[..3].copy_from_slice(&p);
    }
    for pm in planned.iter() {
        let line = pm.geometry.source.start_line;
        if detail_lines.contains(&line) {
            let seg = pm.geometry.segment.spatial.as_ref();
            let kind = seg.map_or("virtual", |s| match s {
                geometry::path::Segment::Line(_) => "line",
                geometry::path::Segment::Arc(_) => "arc",
                geometry::path::Segment::Clothoid(_) => "clothoid",
            });
            let (k0, k1) = seg.map_or((0.0, 0.0), |s| s.kappa_endpoints());
            println!(
                "\n== line {line} {kind} len={:.6} k0={:.4} k1={:.4} entry_v={:.3} exit_v={:.3} peak_v={:.3} samples={} phases={}",
                pm.geometry.segment.s_len(),
                k0,
                k1,
                pm.velocity.entry_v,
                pm.velocity.exit_v,
                pm.velocity.peak_v,
                pm.velocity.samples.len(),
                pm.velocity.phases.len(),
            );
            for s in &pm.velocity.samples {
                println!("   sample s={:.6} v={:.4} a={:.2}", s.s, s.v, s.a);
            }
            let acc_extreme = |c: &[f64], h: f64| -> f64 {
                let acc = |tau: f64| -> f64 {
                    (2..c.len())
                        .map(|k| c[k] * (k * (k - 1)) as f64 * tau.powi(k as i32 - 2))
                        .sum()
                };
                (0..=16)
                    .map(|k| acc(h * k as f64 / 16.0).abs())
                    .fold(0.0, f64::max)
            };
            let seg_lowered = motion_pipeline::lowering::lower_move(
                &pm.geometry,
                &pm.velocity,
                0.0,
                &start_pos,
                fit_tol,
                &chains,
                None,
            )
            .expect("lower_move");
            for (axis, curve) in seg_lowered.axes.iter().enumerate().take(2) {
                for p in nurbs::bezier::extract_bezier_pieces(curve) {
                    let h = p.u_end - p.u_start;
                    let amax = acc_extreme(&p.coeffs, h);
                    if axis == 0 {
                        println!(
                            "   ROUNDTRIP PIECE axis {axis} u=[{:.6},{:.6}] h={:.3e} deg={} amax={:.0}",
                            p.u_start,
                            p.u_end,
                            h,
                            p.coeffs.len() - 1,
                            amax
                        );
                    }
                }
            }
            let (pieces, total_t) = motion_pipeline::lowering::lower_move_pieces(
                &pm.geometry,
                &pm.velocity,
                0.0,
                &start_pos,
                fit_tol,
                &chains,
                None,
            )
            .expect("lower");
            println!("   lowered: total_t={total_t:.6}");
            for (axis, ps) in pieces.iter().enumerate().take(2) {
                for p in ps {
                    let h = p.u_end - p.u_start;
                    let c = &p.coeffs;
                    let acc = |tau: f64| -> f64 {
                        (2..c.len())
                            .map(|k| c[k] * (k * (k - 1)) as f64 * tau.powi(k as i32 - 2))
                            .sum()
                    };
                    let a0 = acc(0.0);
                    let am = acc(h / 2.0);
                    let a1 = acc(h);
                    let _amax = a0.abs().max(am.abs()).max(a1.abs());
                    if axis == 0 {
                        println!(
                            "   PIECE axis {axis} u=[{:.6},{:.6}] h={:.3e} deg={} a0={:.0} amid={:.0} a1={:.0}",
                            p.u_start,
                            p.u_end,
                            h,
                            c.len() - 1,
                            a0,
                            am,
                            a1
                        );
                    }
                }
            }
        }
    }
}
