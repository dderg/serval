//! Scratch investigation tool (not committed): count how many planned moves
//! still reach the lowering with an empty phase chain — the only condition
//! under which `lowering::build_profile(&vm.samples)` (the quintic-over-samples
//! path) and `regime_knot_times` are still reachable.
//!
//!   cargo run -p pipeline-snapshot --example empty_phase_census -- snapshots/cases

use crossbeam_channel::unbounded;
use geometry::path::Segment;
use geometry::{BoundaryState, plan_velocity_stops};
use motion_pipeline::{StreamInput, fit_stage::FitStage};
use pipeline_snapshot::build_moves;
use pipeline_snapshot::waypoints::parse_gcode;
use std::path::{Path, PathBuf};

struct Limits {
    max_velocity: f64,
    max_accel: f64,
    corner_deviation: f64,
    max_jerk: f64,
}

fn read_limits(path: &Path) -> Limits {
    let text = std::fs::read_to_string(path).expect("read cfg");
    let field = |name: &str| -> Option<f64> {
        text.lines()
            .map(|l| l.split('#').next().unwrap_or("").trim())
            .find_map(|l| l.strip_prefix(name)?.strip_prefix(':')?.trim().parse().ok())
    };
    let max_accel = field("max_accel").expect("max_accel");
    let corner_deviation = match field("corner_deviation") {
        Some(cd) => cd,
        None => geometry::corner_deviation_from_scv(
            field("square_corner_velocity").expect("square_corner_velocity"),
            max_accel,
        ),
    };
    let max_jerk = match field("max_jerk").expect("max_jerk") {
        0.0 => f64::INFINITY,
        j => j,
    };
    Limits {
        max_velocity: field("max_velocity").expect("max_velocity"),
        max_accel,
        corner_deviation,
        max_jerk,
    }
}

fn cases(root: &Path) -> Vec<(String, PathBuf, PathBuf)> {
    let mut out = Vec::new();
    let mut groups: Vec<PathBuf> = std::fs::read_dir(root)
        .expect("cases dir")
        .filter_map(|e| e.ok().map(|e| e.path()))
        .filter(|p| p.is_dir())
        .collect();
    groups.sort();
    for group in groups {
        let mut gcodes: Vec<PathBuf> = Vec::new();
        let mut cfgs: Vec<PathBuf> = Vec::new();
        for entry in std::fs::read_dir(&group).expect("group dir").flatten() {
            let p = entry.path();
            match p.extension().and_then(|e| e.to_str()) {
                Some("gcode") if !std::fs::read_to_string(&p).unwrap().trim().is_empty() => {
                    gcodes.push(p)
                }
                Some("cfg") => cfgs.push(p),
                _ => {}
            }
        }
        gcodes.sort();
        cfgs.sort();
        for cfg in &cfgs {
            for gcode in &gcodes {
                out.push((
                    format!(
                        "{}/{}/{}",
                        group.file_name().unwrap().to_string_lossy(),
                        cfg.file_stem().unwrap().to_string_lossy(),
                        gcode.file_stem().unwrap().to_string_lossy()
                    ),
                    cfg.clone(),
                    gcode.clone(),
                ));
            }
        }
    }
    out
}

fn fitted_moves(gcode: &Path, lim: &Limits) -> Vec<geometry::Move> {
    let text = std::fs::read_to_string(gcode).expect("read gcode");
    let waypoints = parse_gcode(&text, lim.max_velocity, lim.max_accel).expect("parse gcode");
    let limits = geometry::VelocityLimits::try_new(
        lim.max_velocity,
        lim.max_accel,
        lim.corner_deviation,
        lim.max_jerk,
    )
    .expect("limits");
    let moves = build_moves(&waypoints, limits).expect("moves");
    let (tx, rx) = unbounded();
    let mut fit = FitStage::new(geometry::CornerFitConfig::default()).into_driver(tx);
    for m in moves {
        assert!(fit.feed(m.into()));
    }
    assert!(fit.finish());
    let mut out = Vec::new();
    while let Ok(item) = rx.try_recv() {
        if let StreamInput::Move(m) = item {
            out.push(m);
        }
    }
    out
}
fn corner_angle(prev: &geometry::Move, next: &geometry::Move) -> Option<f64> {
    use geometry::path::{CurvatureProfile, lowering::PositionProfile};
    let (a, b) = (
        prev.segment.spatial.as_ref()?,
        next.segment.spatial.as_ref()?,
    );
    let t_in = a.heading_at(a.s_len());
    let t_out = b.heading_at(0.0);
    let cos = t_in[0] * t_out[0] + t_in[1] * t_out[1] + t_in[2] * t_out[2];
    Some(libm::acos(cos.clamp(-1.0, 1.0)))
}

fn main() {
    let root = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "snapshots/cases".to_string());
    let theta_min = geometry::CornerFitConfig::default().theta_min_rad;

    let mut total = 0usize;
    let mut empty = 0usize;
    let mut empty_curved = 0usize;
    let mut empty_straight = 0usize;
    let mut member_plans = 0u32;
    let mut unreachable = 0u32;
    for (name, cfg, gcode) in cases(Path::new(&root)) {
        let lim = read_limits(&cfg);
        let moves = fitted_moves(&gcode, &lim);
        if moves.is_empty() {
            println!("{name}: no moves");
            continue;
        }
        let stop_before: Vec<bool> = (0..moves.len())
            .map(|i| i > 0 && corner_angle(&moves[i - 1], &moves[i]).is_none_or(|t| t > theta_min))
            .collect();
        let plan = match plan_velocity_stops(
            &moves,
            &stop_before,
            1e-7,
            f64::INFINITY,
            f64::INFINITY,
            BoundaryState::REST,
        ) {
            Ok(plan) => plan,
            Err(why) => {
                println!("{name}: PLAN FAILED {why:?}");
                continue;
            }
        };
        let mut case_empty = 0usize;
        for (vm, m) in plan.moves.iter().zip(&moves) {
            total += 1;
            if !vm.phases.is_empty() {
                continue;
            }
            empty += 1;
            case_empty += 1;
            match m.segment.spatial.as_ref() {
                Some(Segment::Line(_)) | None => empty_straight += 1,
                Some(_) => empty_curved += 1,
            }
        }
        member_plans += plan.report.reachability.member_plans();
        unreachable += plan.report.reachability.unreachable;
        println!(
            "{name}: {case_empty} empty of {} moves, {} unreachable of {} member plans",
            plan.moves.len(),
            plan.report.reachability.unreachable,
            plan.report.reachability.member_plans()
        );
    }
    println!(
        "TOTAL: {empty} empty phase chains of {total} planned moves \
         ({empty_straight} straight/virtual, {empty_curved} curved); \
         {unreachable} unreachable of {member_plans} member plans"
    );
}
