#![allow(
    clippy::disallowed_methods,
    clippy::explicit_counter_loop,
    clippy::case_sensitive_file_extension_comparisons
)]

//! Offline reproduction of the Trident shaper saturation: dense
//! micro-segment geometry (arc-heavy gcode) through the full stage pipeline
//! with the bench's smooth_mzv chains (X 191 Hz, Y 117.9 Hz). Prints wall
//! time per stream second — a ratio >= 1.0 means the shaper cannot keep up
//! with realtime and the transport will starve.
//!
//! Run: cargo run --release -p motion-pipeline --example shaper_bench

use std::time::Instant;

use geometry::segment::SourceRange;
use geometry::{CornerFitConfig, MoveContext, VelocityLimits, line_move};
use motion_pipeline::{StreamConfig, StreamInput, TrajectoryItem, setup_stages};
use trajectory::{AxisChainSet, PostProcessorInstance};

fn limits() -> VelocityLimits {
    VelocityLimits::try_new(2800.0, 100_000.0, 0.04, 40_000_000.0).unwrap()
}

/// Klippy caps per-move limits for Z motion (`for_move`); mirror the bench's
/// [printer] max_z_velocity/max_z_accel so layer changes fit realistically.
fn limits_for(dz: f64) -> VelocityLimits {
    if dz == 0.0 {
        limits()
    } else {
        VelocityLimits::try_new(20.0, 1000.0, 0.04, 40_000_000.0).unwrap()
    }
}

fn config() -> StreamConfig {
    StreamConfig {
        corner: CornerFitConfig::default(),
        integration_tol: 1e-4,
        max_extrude_only_velocity_mm_s: f64::INFINITY,
        max_extrude_only_accel_mm_s2: f64::INFINITY,
        fit_tol_mm: 0.005,
        fit_tol_accel_mm_s2: 50.0,
        max_buffer_moves: 512,
        limits: limits(),
    }
}

fn trident_chains() -> AxisChainSet {
    let x = trajectory::CompiledChain::compile(&[PostProcessorInstance::new(
        "shaper_x",
        &trajectory::algos::SmoothMzv,
        vec![191.0],
    )])
    .expect("x chain compiles");
    let y = trajectory::CompiledChain::compile(&[PostProcessorInstance::new(
        "shaper_y",
        &trajectory::algos::SmoothMzv,
        vec![117.9],
    )])
    .expect("y chain compiles");
    let e = trajectory::CompiledChain::compile(&[
        PostProcessorInstance::new("pa", &trajectory::algos::LinearPressureAdvance, vec![0.017]),
        PostProcessorInstance::new("st", &trajectory::algos::SmoothTriangle, vec![0.02]),
    ])
    .expect("e chain compiles");
    AxisChainSet {
        chains: vec![x, y, trajectory::CompiledChain::default(), e],
        followers: vec![(3, vec![0, 1, 2])],
    }
}

fn voron0_chains() -> AxisChainSet {
    let x = trajectory::CompiledChain::compile(&[PostProcessorInstance::new(
        "x_shaping",
        &trajectory::algos::SmoothMzv,
        vec![112.8],
    )])
    .expect("x chain compiles");
    let y = trajectory::CompiledChain::compile(&[PostProcessorInstance::new(
        "y_shaping",
        &trajectory::algos::SmoothMzv,
        vec![90.2],
    )])
    .expect("y chain compiles");
    let z = trajectory::CompiledChain::compile(&[PostProcessorInstance::new(
        "z_shaping",
        &trajectory::algos::SmoothBell,
        vec![0.025],
    )])
    .expect("z chain compiles");
    let e = trajectory::CompiledChain::compile(&[
        PostProcessorInstance::new(
            "e_pa",
            &trajectory::algos::TanhPressureAdvance,
            vec![0.015, 0.011, 1.5],
        ),
        PostProcessorInstance::new(
            "e_smoothing",
            &trajectory::algos::SmoothTriangle,
            vec![0.01],
        ),
    ])
    .expect("e chain compiles");
    AxisChainSet {
        chains: vec![x, y, z, e],
        followers: vec![(3, vec![0, 1, 2])],
    }
}

/// Circle approximated by chords — the arc-dense sections of a sliced model
/// where every chord is a separate G1.
fn circle_moves(radius_mm: f64, chord_mm: f64, feed_mm_s: f64, laps: usize) -> Vec<StreamInput> {
    let center = (100.0, 100.0);
    let n = ((2.0 * std::f64::consts::PI * radius_mm) / chord_mm).ceil() as usize;
    let mut moves = Vec::with_capacity(n * laps);
    let point = |i: usize| {
        let a = 2.0 * std::f64::consts::PI * (i as f64) / (n as f64);
        [
            center.0 + radius_mm * a.cos(),
            center.1 + radius_mm * a.sin(),
            10.0,
        ]
    };
    let mut prev = point(0);
    let mut line_no = 1u32;
    for _ in 0..laps {
        for i in 1..=n {
            let next = point(i % n);
            let ctx = MoveContext {
                extruder_axis: 3,
                feedrate_mm_s: feed_mm_s,
                limits: limits(),
                source: SourceRange {
                    start_line: line_no,
                    end_line: line_no,
                },
            };
            moves.push(StreamInput::Move(
                line_move(prev, next, 0.0, ctx).expect("chord move"),
            ));
            prev = next;
            line_no += 1;
        }
    }
    moves
}

/// Coarse per-move duration for input pacing: path length at the commanded
/// feed, ignoring accel — an underestimate, which only makes pacing feed
/// slightly early (extra lead), never starve the pipe artificially.
fn move_duration_estimate(m: &geometry::Move) -> f64 {
    use geometry::path::CurvatureProfile;
    let len = m
        .segment
        .spatial
        .as_ref()
        .map(CurvatureProfile::s_len)
        .or(m.segment.virtual_path_mm)
        .unwrap_or(0.0);
    let feed = m.feedrate_mm_s.min(m.limits.max_velocity_mm_s).max(1.0);
    len / feed
}

/// Returns per-emit (wall_secs, stream_t_end) samples, or None if a stage
/// died on this input (harness move construction differs from klippy's
/// preprocessing; a failed chunk is skipped, not fatal).
fn run_case(name: &str, chains: AxisChainSet, inputs: Vec<StreamInput>) -> Option<Vec<(f64, f64)>> {
    let n_moves = inputs.len();
    let pipeline = setup_stages(config(), chains, vec![100.0, 100.0, 0.2, 0.0], 0.0);
    let started = Instant::now();
    let output = pipeline.output;
    let consumer = std::thread::spawn(move || {
        let mut emits: Vec<(f64, f64)> = Vec::new();
        let t0 = Instant::now();
        for item in output.iter() {
            if let TrajectoryItem::Seg(seg) = item {
                emits.push((t0.elapsed().as_secs_f64(), seg.t_end));
            }
        }
        emits
    });
    let pace = std::env::var("SHAPER_BENCH_PACE").is_ok();
    let mut est_stream = 0.0f64;
    let mut poisoned = false;
    for item in inputs {
        if pace {
            if let StreamInput::Move(m) = &item {
                est_stream += move_duration_estimate(m);
            }
            let lead = est_stream - started.elapsed().as_secs_f64();
            if lead > 2.0 {
                std::thread::sleep(std::time::Duration::from_secs_f64(lead - 2.0));
            }
        }
        if pipeline.input.send(item).is_err() {
            poisoned = true;
            break;
        }
    }
    if !poisoned {
        poisoned = pipeline.input.send(StreamInput::Drain).is_err();
    }
    drop(pipeline.input);
    let emits = consumer.join().expect("consumer");
    if poisoned {
        println!("{name}: SKIPPED (stage died on this input)");
        for t in pipeline.threads {
            let _ = t.join();
        }
        return None;
    }
    let wall = started.elapsed().as_secs_f64();
    let n_segs = emits.len();
    let stream_secs = emits.iter().fold(0.0f64, |m, &(_, t)| m.max(t));
    println!(
        "{name}: {n_moves} moves -> {n_segs} segs, {stream_secs:.2} stream-s \
         in {wall:.2} wall-s  => {:.2}x realtime load",
        wall / stream_secs
    );
    let mut buckets: Vec<(f64, f64)> = Vec::new();
    for &(wall_at, t_end) in &emits {
        let bucket = t_end.floor();
        match buckets.last_mut() {
            Some((b, hi)) if *b == bucket => *hi = wall_at,
            _ => buckets.push((bucket, wall_at)),
        }
    }
    let mut worst: Vec<(f64, f64)> = buckets
        .windows(2)
        .map(|w| (w[1].0, w[1].1 - w[0].1))
        .collect();
    worst.sort_by(|a, b| b.1.total_cmp(&a.1));
    for (t, cost) in worst.iter().take(5) {
        println!("    worst window: stream-t {t:.0}s cost {cost:.2} wall-s");
    }
    Some(emits)
}

fn gcode_moves(path: &str) -> Vec<StreamInput> {
    let text = std::fs::read_to_string(path).expect("read gcode");
    let mut pos = [100.0f64, 100.0, 0.2];
    let mut e = 0.0f64;
    let mut feed = 100.0f64;
    let mut moves = Vec::new();
    let mut line_no = 0u32;
    for raw in text.lines() {
        line_no += 1;
        let line = raw.split(';').next().unwrap_or("").trim();
        let mut words = line.split_whitespace();
        let Some(cmd) = words.next() else { continue };
        if cmd == "G92" {
            for w in words {
                if let Some(v) = w.strip_prefix('E').and_then(|s| s.parse().ok()) {
                    e = v;
                }
            }
            continue;
        }
        if cmd != "G1" && cmd != "G0" {
            continue;
        }
        let mut next = pos;
        let mut next_e = e;
        for w in words {
            let (axis, val) = w.split_at(1);
            let Ok(v) = val.parse::<f64>() else { continue };
            match axis {
                "X" => next[0] = v,
                "Y" => next[1] = v,
                "Z" => next[2] = v,
                "E" => next_e = v,
                "F" => feed = v / 60.0,
                _ => {}
            }
        }
        let de = next_e - e;
        e = next_e;
        if next == pos {
            continue;
        }
        let ctx = MoveContext {
            extruder_axis: 3,
            feedrate_mm_s: feed,
            limits: limits_for(next[2] - pos[2]),
            source: SourceRange {
                start_line: line_no,
                end_line: line_no,
            },
        };
        if let Ok(m) = line_move(pos, next, de, ctx) {
            moves.push(StreamInput::Move(m));
        }
        pos = next;
    }
    moves
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if let Some(path) = args.first().filter(|a| a.ends_with(".gcode")) {
        let chunk_size: usize = args.get(1).map_or(4000, |s| s.parse().unwrap());
        if let Ok(spec) = std::env::var("SHAPER_BENCH_CHUNK") {
            let mut parts = spec.split(':');
            let label = parts.next().unwrap().to_string();
            let start: usize = parts.next().unwrap().parse().unwrap();
            let count: usize = parts.next().unwrap().parse().unwrap();
            let chains = match label.as_str() {
                "smooth_mzv" => trident_chains(),
                "voron0" => voron0_chains(),
                "no-shaper" => AxisChainSet::default(),
                _ => panic!("unknown shaper benchmark label {label}"),
            };
            let moves: Vec<StreamInput> = gcode_moves(path)
                .into_iter()
                .skip(start)
                .take(count)
                .collect();
            if let Some(emits) = run_case(&format!("{label} moves {start}+{count}"), chains, moves)
            {
                let wall = emits.last().map_or(0.0, |&(w, _)| w);
                let stream = emits.iter().fold(0.0f64, |m, &(_, t)| m.max(t));
                println!("CHUNK_RESULT {wall:.4} {stream:.4}");
            }
            return;
        }
        let n_moves = gcode_moves(path).len();
        let exe = std::env::current_exe().expect("current exe");
        for label in ["smooth_mzv", "no-shaper"] {
            let mut wall_total = 0.0f64;
            let mut stream_total = 0.0f64;
            let mut skipped = 0usize;
            let mut start = 0usize;
            while start < n_moves {
                let out = std::process::Command::new(&exe)
                    .arg(path)
                    .env(
                        "SHAPER_BENCH_CHUNK",
                        format!("{label}:{start}:{chunk_size}"),
                    )
                    .output()
                    .expect("spawn chunk");
                let text = String::from_utf8_lossy(&out.stdout);
                print!("{text}");
                match text
                    .lines()
                    .find_map(|l| l.strip_prefix("CHUNK_RESULT "))
                    .map(|r| {
                        let mut it = r.split_whitespace();
                        let w: f64 = it.next().unwrap().parse().unwrap();
                        let s: f64 = it.next().unwrap().parse().unwrap();
                        (w, s)
                    }) {
                    Some((w, s)) => {
                        wall_total += w;
                        stream_total += s;
                    }
                    None => {
                        println!("{label} moves {start}: CRASHED, skipping chunk");
                        skipped += 1;
                    }
                }
                start += chunk_size;
            }
            println!(
                "== {label} TOTAL: {stream_total:.1} stream-s in {wall_total:.1} wall-s \
                 => {:.3}x realtime load ({skipped} chunks skipped)",
                wall_total / stream_total
            );
        }
        return;
    }
    let chord: f64 = args.first().map_or(0.4, |s| s.parse().unwrap());
    let feed: f64 = args.get(1).map_or(100.0, |s| s.parse().unwrap());
    let laps: usize = args.get(2).map_or(1, |s| s.parse().unwrap());
    run_case(
        &format!("smooth_mzv  chord={chord}mm feed={feed}mm/s laps={laps}"),
        trident_chains(),
        circle_moves(10.0, chord, feed, laps),
    );
    run_case(
        &format!("no-shaper   chord={chord}mm feed={feed}mm/s laps={laps}"),
        AxisChainSet::default(),
        circle_moves(10.0, chord, feed, laps),
    );
}
