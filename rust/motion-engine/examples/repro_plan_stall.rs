use std::env;
use std::fs;
use std::process;
use std::time::Instant;

use _motion_engine::seam_harness::{default_stream_config, parse_gcode_to_moves};
use _motion_engine::stream::StreamState;
use geometry::path::lowering::PositionProfile;
use tracing_subscriber::EnvFilter;
use trajectory::AxisChainSet;

fn main() {
    tracing_subscriber::fmt()
        .with_env_filter(
            EnvFilter::try_from_default_env().unwrap_or_else(|_| EnvFilter::new("warn")),
        )
        .init();

    let args: Vec<String> = env::args().collect();
    if args.len() < 2 {
        eprintln!("usage: repro_plan_stall <in.gcode> [--cap N]");
        process::exit(1);
    }
    let in_path = &args[1];
    let mut cap = 1usize;
    let mut arc: Option<(f64, f64)> = None;
    let mut i = 2;
    while i < args.len() {
        if args[i] == "--cap" {
            i += 1;
            cap = args[i].parse().unwrap();
        } else if args[i] == "--arc" {
            i += 1;
            let (f, d) = args[i].split_once(',').expect("--arc FACET,DEG");
            arc = Some((f.parse().unwrap(), d.parse::<f64>().unwrap().to_radians()));
        }
        i += 1;
    }

    let source = fs::read_to_string(in_path).unwrap_or_else(|e| {
        eprintln!("cannot read {in_path}: {e}");
        process::exit(1);
    });

    let mut config = default_stream_config();
    if let Some((facet, turn)) = arc {
        config.chain = geometry::ChainFitConfig::with_arc_fit(facet, turn);
    }
    let moves = parse_gcode_to_moves(&source, config.limits);
    let home = moves
        .first()
        .and_then(|m| m.segment.spatial.as_ref())
        .map_or([0.0, 0.0, 0.0], |seg| seg.point_at(0.0));
    let mut state = StreamState::new(config, AxisChainSet::default(), &home, 0.0);

    let mut commit_wall_us: Vec<(u128, usize, usize)> = Vec::new();

    let commit = |state: &mut StreamState, force: bool, log: &mut Vec<(u128, usize, usize)>| {
        let n_in = state.buffered();
        let clock = Instant::now();
        let segs = state.commit(force).expect("commit drives the real path");
        let us = clock.elapsed().as_micros();
        if !segs.is_empty() {
            log.push((us, n_in, segs.len()));
        }
    };

    for m in moves.iter().cloned() {
        state.push(m);
        if state.buffered() >= cap {
            commit(&mut state, false, &mut commit_wall_us);
        }
    }
    while state.buffered() > 0 {
        let before = state.buffered();
        commit(&mut state, true, &mut commit_wall_us);
        if state.buffered() == before {
            break;
        }
    }

    let total: u128 = commit_wall_us.iter().map(|c| c.0).sum();
    let worst = commit_wall_us.iter().max_by_key(|c| c.0);
    let over_50ms = commit_wall_us.iter().filter(|c| c.0 >= 50_000).count();

    println!("\ncap={cap}  commits={}", commit_wall_us.len());
    println!("total_commit_compute={:.1}ms", total as f64 / 1000.0);
    if let Some((us, n_in, n_segs)) = worst {
        println!(
            "worst_commit={:.1}ms  (n_in={n_in} moves -> {n_segs} segments)",
            *us as f64 / 1000.0
        );
    }
    println!("commits_over_50ms={over_50ms}");
}
