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
use motion_pipeline::types::PlannedMove;
use motion_pipeline::{BaseItem, Lowerer, PlannedItem, StreamConfig, StreamInput};
use pipeline_snapshot::waypoints::parse_gcode;
use pipeline_snapshot::{
    SNAPSHOT_MAX_BUFFER_MOVES, TRAJECTORY_FIT_TOL_ACCEL_MM_S2, TRAJECTORY_FIT_TOL_MM,
    VELOCITY_INTEGRATION_TOL, build_moves, collect_trajectory_pieces,
};

fn main() {
    let mut args = std::env::args().skip(1);
    let path = args.next().expect("gcode path");
    let max_velocity: f64 = args.next().expect("max_velocity").parse().unwrap();
    let max_accel: f64 = args.next().expect("max_accel").parse().unwrap();
    let scv: f64 = args.next().expect("scv").parse().unwrap();
    let max_jerk: f64 = args.next().expect("max_jerk").parse().unwrap();

    let text = std::fs::read_to_string(&path).expect("read gcode");
    let waypoints = parse_gcode(&text, max_velocity, max_accel).expect("parse gcode");
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
    // segment, with the worst offenders listed.
    if std::env::var("AUDIT_ALL").is_ok() {
        let mut worst_jerk = 0.0_f64;
        let mut worst_acc: Vec<(f64, u32)> = Vec::new();
        for segment in lower_run(&planned) {
            let traj = collect_trajectory_pieces(std::slice::from_ref(&segment));
            let mut mv_max = 0.0_f64;
            let mut mv_jerk = 0.0_f64;
            for pieces in [&traj.x, &traj.y, &traj.z] {
                for p in pieces {
                    let h = p[1] - p[0];
                    mv_max = mv_max.max(acc_extreme(&p[2..], h));
                    mv_jerk = mv_jerk.max(jerk_extreme(&p[2..], h));
                }
            }
            worst_jerk = worst_jerk.max(mv_jerk);
            if std::env::var_os("AUDIT_KIND").is_some() {
                let source = planned
                    .iter()
                    .find(|pm| pm.geometry.source.start_line == segment.source_line);
                let kind = source
                    .and_then(|pm| pm.geometry.segment.spatial.as_ref())
                    .map_or("virtual", |s| match s {
                        geometry::path::Segment::Line(_) => "line",
                        geometry::path::Segment::Arc(_) => "arc",
                        geometry::path::Segment::Clothoid(_) => "clothoid",
                    });
                println!(
                    "  move kind={kind} line={} len={:.6} entry={:.4} exit={:.4} max={mv_max:.1} jmax={mv_jerk:.3e}",
                    segment.source_line,
                    source.map_or(0.0, |pm| pm.geometry.segment.s_len()),
                    source.map_or(0.0, |pm| pm.velocity.entry_v),
                    source.map_or(0.0, |pm| pm.velocity.exit_v)
                );
            }
            worst_acc.push((mv_max, segment.source_line));
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
    let segments = lower_run(&planned);
    for pm in planned.iter() {
        let line = pm.geometry.source.start_line;
        if !detail_lines.contains(&line) {
            continue;
        }
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
        for segment in segments.iter().filter(|s| s.source_line == line) {
            let traj = collect_trajectory_pieces(std::slice::from_ref(segment));
            println!(
                "   lowered: t=[{:.6},{:.6}]",
                segment.t_start, segment.t_end
            );
            for p in &traj.x {
                let h = p[1] - p[0];
                let c = &p[2..];
                let acc = |tau: f64| -> f64 {
                    (2..c.len())
                        .map(|k| c[k] * (k * (k - 1)) as f64 * tau.powi(k as i32 - 2))
                        .sum()
                };
                println!(
                    "   PIECE axis 0 u=[{:.6},{:.6}] h={:.3e} deg={} a0={:.0} amid={:.0} a1={:.0} amax={:.0}",
                    p[0],
                    p[1],
                    h,
                    c.len() - 1,
                    acc(0.0),
                    acc(h / 2.0),
                    acc(h),
                    acc_extreme(c, h)
                );
            }
        }
    }
}

/// Lowers a whole planned run through the production lowerer and hands back
/// the continuous segments it emitted.
fn lower_run(planned: &[PlannedMove]) -> Vec<trajectory::ContinuousSegment> {
    let mut home = vec![0.0_f64; 4];
    if let Some(seg) = planned
        .iter()
        .find_map(|pm| pm.geometry.segment.spatial.as_ref())
    {
        home[..3].copy_from_slice(&seg.point_at(0.0));
    }
    let (lowered_tx, lowered_rx) = unbounded();
    let mut lowerer = Lowerer::new(trajectory::AxisChainSet::default(), home, 0.0);
    for pm in planned {
        let item = PlannedItem::Move(PlannedMove {
            geometry: pm.geometry.clone(),
            velocity: pm.velocity.clone(),
        });
        assert!(lowerer.feed(item, &lowered_tx), "lowerer rejected input");
    }
    drop(lowered_tx);
    lowered_rx
        .into_iter()
        .filter_map(|item| match item {
            BaseItem::Seg(seg) => Some(seg.segment),
            _ => None,
        })
        .collect()
}

fn acc_extreme(c: &[f64], h: f64) -> f64 {
    derivative_extreme(c, h, 2)
}

fn jerk_extreme(c: &[f64], h: f64) -> f64 {
    derivative_extreme(c, h, 3)
}

fn derivative_extreme(c: &[f64], h: f64, deriv: usize) -> f64 {
    let eval = |tau: f64| -> f64 {
        (deriv..c.len())
            .map(|k| {
                let scale: usize = (k - deriv + 1..=k).product();
                c[k] * scale as f64 * tau.powi((k - deriv) as i32)
            })
            .sum()
    };
    (0..=16)
        .map(|k| eval(h * k as f64 / 16.0).abs())
        .fold(0.0, f64::max)
}
