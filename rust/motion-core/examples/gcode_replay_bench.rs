// Replays a gcode file through the production streaming pipeline, built from
// a printer.cfg exactly as the live path builds it (stream config, post-
// processor chains, per-move limits including the Z caps, shaper on its own
// thread), and reports producer throughput as a sliding-window realtime
// factor over stream time, the deepest run of moves absorbed without the
// planner committing, and where the wall time went stage by stage.
//
//   cargo run --release -p motion-core --example gcode_replay_bench -- \
//       <file.gcode> --config <printer.cfg> [t_lo t_hi] \
//       [--min-worst-x <x>] [--max-held-moves <n>]
//
// With t_lo/t_hi only moves whose stream window overlaps [t_lo, t_hi] stream
// seconds are interesting; the whole file still replays (stream time only
// exists downstream), but the report highlights that range.

use std::env;
use std::fs;
use std::process;
use std::time::Instant;

use motion_core::live_config;
use motion_core::seam_test_harness::parse_gcode_to_moves_with;
use motion_pipeline::timing::stage_totals;
use motion_pipeline::{Pipeline, TrajectoryItem};

fn take_flag(args: &mut Vec<String>, flag: &str) -> Option<String> {
    let i = args.iter().position(|a| a == flag)?;
    args.remove(i);
    Some(args.remove(i))
}

fn planner_config(path: &str) -> planner_config::PlannerConfig {
    let text = fs::read_to_string(path).unwrap_or_else(|e| {
        eprintln!("cannot read {path}: {e}");
        process::exit(1);
    });
    let doc = config_doc::Document::parse(&text, path).unwrap_or_else(|e| {
        eprintln!("cannot parse {path}: {e}");
        process::exit(1);
    });
    let (settings, _) = planner_config::from_doc::read_motion_settings(&doc).unwrap_or_else(|e| {
        eprintln!("{path}: {e}");
        process::exit(1);
    });
    planner_config::from_doc::planner_config_from_settings(&settings).unwrap_or_else(|e| {
        eprintln!("{path}: {e}");
        process::exit(1);
    })
}

fn main() {
    let mut args: Vec<String> = env::args().collect();
    let min_worst_x = take_flag(&mut args, "--min-worst-x")
        .map(|v| v.parse::<f64>().expect("--min-worst-x takes a number"));
    let max_held_moves = take_flag(&mut args, "--max-held-moves")
        .map(|v| v.parse::<u64>().expect("--max-held-moves takes a count"));
    let Some(config_path) = take_flag(&mut args, "--config") else {
        eprintln!(
            "usage: gcode_replay_bench <in.gcode> --config <printer.cfg> [t_lo t_hi] \
             [--min-worst-x <x>] [--max-held-moves <n>]"
        );
        process::exit(1);
    };
    if args.len() < 2 {
        eprintln!("usage: gcode_replay_bench <in.gcode> --config <printer.cfg> [t_lo t_hi]");
        process::exit(1);
    }
    let source = fs::read_to_string(&args[1]).unwrap_or_else(|e| {
        eprintln!("cannot read {}: {e}", args[1]);
        process::exit(1);
    });
    let highlight: Option<(f64, f64)> = (args.len() >= 4).then(|| {
        (
            args[2].parse().expect("t_lo must be a number"),
            args[3].parse().expect("t_hi must be a number"),
        )
    });

    let planner = planner_config(&config_path);
    let cfg = live_config::stream_config(&planner).unwrap_or_else(|e| {
        eprintln!("{config_path}: {e}");
        process::exit(1);
    });
    let chains = planner.compile_active_chains().unwrap_or_else(|e| {
        eprintln!("{config_path}: {e}");
        process::exit(1);
    });
    let moves = parse_gcode_to_moves_with(&source, |delta| {
        live_config::move_limits(&planner, delta).expect("config limits are valid")
    });

    let wall = Instant::now();
    let (arrival_tx, arrival_rx) = crossbeam_channel::unbounded::<(f64, f64)>();
    let mut pipeline = Pipeline::new(cfg, chains, vec![0.0; 4], 0.0).into_staged(move |item| {
        if let TrajectoryItem::Seg(seg) = item {
            arrival_tx
                .send((wall.elapsed().as_secs_f64(), seg.t_end))
                .expect("bench collects every arrival");
        }
        true
    });

    let mut submitted = 0u64;
    let mut held = 0u64;
    let mut deepest_held = 0u64;
    for m in moves {
        let planned_before = pipeline.segments_planned();
        if !pipeline.feed(m.into()) {
            eprintln!("pipeline input closed at move {submitted}");
            process::exit(1);
        }
        submitted += 1;
        held = if pipeline.segments_planned() > planned_before {
            0
        } else {
            held + 1
        };
        deepest_held = deepest_held.max(held);
    }
    assert!(pipeline.finish());
    let wall_s = wall.elapsed().as_secs_f64();
    let arrivals: Vec<(f64, f64)> = arrival_rx.try_iter().collect();
    let stream_s = arrivals.last().map_or(0.0, |&(_, t)| t);
    println!(
        "moves={submitted} segments={} stream_secs={stream_s:.1} wall_secs={wall_s:.1} overall_x={:.2} deepest_held_moves={deepest_held}",
        arrivals.len(),
        stream_s / wall_s
    );
    println!("stage breakdown (wall share, calls, per move):");
    for total in stage_totals() {
        let secs = total.total_us as f64 * 1e-6;
        println!(
            "  {:<28} {secs:8.3}s {:6.1}%  calls={:<8} {:7.1} us/move",
            total.stage.name(),
            100.0 * secs / wall_s,
            total.calls,
            total.total_us as f64 / submitted as f64
        );
    }

    // Sliding stream-time windows: wall time the producer spent emitting each.
    const WINDOW_S: f64 = 5.0;
    let mut windows: Vec<(f64, f64, f64)> = Vec::new();
    let mut lo = 0usize;
    for hi in 0..arrivals.len() {
        while arrivals[hi].1 - arrivals[lo].1 > WINDOW_S {
            lo += 1;
        }
        if hi > lo {
            let stream = arrivals[hi].1 - arrivals[lo].1;
            let wall = arrivals[hi].0 - arrivals[lo].0;
            if wall > 1e-9 && stream >= WINDOW_S * 0.8 {
                windows.push((arrivals[lo].1, arrivals[hi].1, stream / wall));
            }
        }
    }
    windows.sort_by(|a, b| a.2.total_cmp(&b.2));
    println!("worst stream windows (t_lo..t_hi realtime_x):");
    let mut reported: Vec<(f64, f64, f64)> = Vec::new();
    for &(t_lo, t_hi, x) in &windows {
        if reported
            .iter()
            .any(|&(r_lo, r_hi, _)| t_lo < r_hi + WINDOW_S && t_hi > r_lo - WINDOW_S)
        {
            continue;
        }
        reported.push((t_lo, t_hi, x));
        println!("  {t_lo:9.2}..{t_hi:9.2}  {x:6.2}x");
        if reported.len() >= 10 {
            break;
        }
    }
    if let Some((h_lo, h_hi)) = highlight {
        let mut worst = f64::INFINITY;
        for &(t_lo, t_hi, x) in &windows {
            if t_lo < h_hi && t_hi > h_lo {
                worst = worst.min(x);
            }
        }
        println!("highlight {h_lo:.1}..{h_hi:.1}: worst realtime_x={worst:.2}");
    }
    if let Some(floor) = min_worst_x {
        let worst = windows.first().map_or(f64::INFINITY, |&(_, _, x)| x);
        if !(worst >= floor) {
            eprintln!("FAIL: worst window {worst:.2}x is below the {floor:.2}x floor");
            process::exit(1);
        }
        println!("budget ok: worst window {worst:.2}x >= {floor:.2}x floor");
    }
    if let Some(ceiling) = max_held_moves {
        if deepest_held > ceiling {
            eprintln!(
                "FAIL: the planner absorbed {deepest_held} moves in a row without committing \
                 (ceiling {ceiling})"
            );
            process::exit(1);
        }
        println!("commit flow ok: deepest held run {deepest_held} <= {ceiling} moves");
    }
}
