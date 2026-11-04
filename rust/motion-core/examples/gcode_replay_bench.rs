// Replays a gcode file through the production streaming pipeline, built from
// a printer.cfg exactly as the live path builds it (stream config, post-
// processor chains, per-move limits including the z caps, shaper on its own
// thread), and reports producer throughput as a sliding-window realtime
// factor over stream time, the deepest run of moves absorbed without the
// planner committing, and where the wall time went stage by stage.
//
//   cargo run --release -p motion-core --example gcode_replay_bench -- \
//       <file.gcode> --config <printer.cfg> [t_lo t_hi] \
//       [--min-worst-x <x>] [--max-held-moves <n>] [--digest] [--synchronous]
//
// With t_lo/t_hi only moves whose stream window overlaps [t_lo, t_hi] stream
// seconds are interesting; the whole file still replays (stream time only
// exists downstream), but the report highlights that range. --digest hashes
// every emitted segment, so a throughput change can be held to an identical
// trajectory. --synchronous drives the plain synchronous Pipeline on the
// calling thread with no shaper thread; under `taskset -c 0` the worker pool
// has no helpers either, which makes the run single-threaded and its
// instruction count deterministic (scripts/throughput_gate.py). The planner
// commits only show up as shaped output there, so --max-held-moves needs the
// staged pipeline.

use std::env;
use std::fs;
use std::hash::{Hash, Hasher};
use std::process;
use std::time::Instant;

use motion_core::seam_test_harness::LiveReplay;
use motion_pipeline::timing::stage_totals;
use motion_pipeline::{Pipeline, StreamInput, TrajectoryItem};
use trajectory::ContinuousSegment;

fn take_flag(args: &mut Vec<String>, flag: &str) -> Option<String> {
    let i = args.iter().position(|a| a == flag)?;
    args.remove(i);
    Some(args.remove(i))
}

fn take_switch(args: &mut Vec<String>, switch: &str) -> bool {
    let Some(i) = args.iter().position(|a| a == switch) else {
        return false;
    };
    args.remove(i);
    true
}

fn replay_staged(
    pipeline: Pipeline,
    moves: impl IntoIterator<Item = impl Into<StreamInput>>,
    sink: impl FnMut(TrajectoryItem) -> bool + Send + 'static,
) -> u64 {
    let mut pipeline = pipeline.into_staged(sink);
    let mut held = 0u64;
    let mut deepest_held = 0u64;
    for (submitted, m) in moves.into_iter().enumerate() {
        let planned_before = pipeline.segments_planned();
        if !pipeline.feed(m.into()) {
            eprintln!("pipeline input closed at move {submitted}");
            process::exit(1);
        }
        held = if pipeline.segments_planned() > planned_before {
            0
        } else {
            held + 1
        };
        deepest_held = deepest_held.max(held);
    }
    assert!(pipeline.finish());
    deepest_held
}

fn replay_synchronous(
    mut pipeline: Pipeline,
    moves: impl IntoIterator<Item = impl Into<StreamInput>>,
    mut sink: impl FnMut(TrajectoryItem) -> bool,
) {
    for (submitted, m) in moves.into_iter().enumerate() {
        if !pipeline.feed(m.into(), &mut sink) {
            eprintln!("pipeline input closed at move {submitted}");
            process::exit(1);
        }
    }
    assert!(pipeline.finish(&mut sink));
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
    let digest = take_switch(&mut args, "--digest");
    let synchronous = take_switch(&mut args, "--synchronous");
    let Some(config_path) = take_flag(&mut args, "--config") else {
        eprintln!(
            "usage: gcode_replay_bench <in.gcode> --config <printer.cfg> [t_lo t_hi] \
             [--min-worst-x <x>] [--max-held-moves <n>] [--digest] [--synchronous]"
        );
        process::exit(1);
    };
    if synchronous && max_held_moves.is_some() {
        eprintln!("--max-held-moves measures the staged pipeline; drop --synchronous");
        process::exit(1);
    }
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
    let replay = LiveReplay::new(&source, &planner).unwrap_or_else(|e| {
        eprintln!("{config_path}: {e}");
        process::exit(1);
    });

    let wall = Instant::now();
    let (arrival_tx, arrival_rx) = crossbeam_channel::unbounded::<(f64, f64)>();
    let (digest_tx, digest_rx) = crossbeam_channel::unbounded::<ContinuousSegment>();
    let sink = move |item| {
        if let TrajectoryItem::Seg(seg) = item {
            arrival_tx
                .send((wall.elapsed().as_secs_f64(), seg.t_end))
                .expect("bench collects every arrival");
            if digest {
                digest_tx.send(seg).expect("bench digests every segment");
            }
        }
        true
    };

    let submitted = replay.moves.len() as u64;
    let pipeline = replay.pipeline();
    let deepest_held = if synchronous {
        replay_synchronous(pipeline, replay.moves, sink);
        None
    } else {
        Some(replay_staged(pipeline, replay.moves, sink))
    };
    let wall_s = wall.elapsed().as_secs_f64();
    let arrivals: Vec<(f64, f64)> = arrival_rx.try_iter().collect();
    let stream_s = arrivals.last().map_or(0.0, |&(_, t)| t);
    let held_field =
        deepest_held.map_or(String::new(), |held| format!(" deepest_held_moves={held}"));
    println!(
        "moves={submitted} segments={} stream_secs={stream_s:.3} wall_secs={wall_s:.1} overall_x={:.2}{held_field}",
        arrivals.len(),
        stream_s / wall_s
    );
    if digest {
        let mut hasher = std::hash::DefaultHasher::new();
        for seg in digest_rx.try_iter() {
            format!("{seg:?}").hash(&mut hasher);
        }
        println!("trajectory_digest={:016x}", hasher.finish());
    }
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
        let deepest_held = deepest_held.expect("--max-held-moves runs the staged pipeline");
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
