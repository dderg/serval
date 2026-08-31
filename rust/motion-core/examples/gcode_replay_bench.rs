// Replays a gcode file through the production streaming pipeline (fit ->
// planner -> lowerer -> shaper, with input shapers on X/Y and PA + smoothing
// on the E follower) and reports producer throughput as a sliding-window
// realtime factor over stream time. Bench-repro tool for the Trident cube
// anchor underrun: finds the gcode region where the producer falls behind
// playback and how far behind it runs.
//
//   cargo run --release -p motion-core --example gcode_replay_bench -- <file.gcode> [t_lo t_hi]
//
// With t_lo/t_hi only moves whose stream window overlaps [t_lo, t_hi] stream
// seconds are interesting; the whole file still replays (stream time only
// exists downstream), but the report highlights that range.

use std::env;
use std::fs;
use std::process;
use std::thread;
use std::time::Instant;

use geometry::{CornerFitConfig, VelocityLimits};
use motion_core::classify::build_move;
use motion_pipeline::{StreamConfig, TrajectoryItem, setup_stages};
use trajectory::{AxisChainSet, CompiledChain, PostProcessorInstance};

struct Pos {
    pos: [f64; 3],
    feed: f64,
    absolute: bool,
    relative_e: bool,
    established: bool,
}

impl Pos {
    fn apply(
        &mut self,
        x: Option<f64>,
        y: Option<f64>,
        z: Option<f64>,
        f: Option<f64>,
    ) -> Option<([f64; 3], f64, f64, f64)> {
        if let Some(f) = f {
            self.feed = f / 60.0;
        }
        let target = |current: f64, word: Option<f64>, absolute: bool| match word {
            Some(w) if absolute => w,
            Some(w) => current + w,
            None => current,
        };
        let next = [
            target(self.pos[0], x, self.absolute),
            target(self.pos[1], y, self.absolute),
            target(self.pos[2], z, self.absolute),
        ];
        if !self.established {
            self.pos = next;
            self.established = x.is_some() && y.is_some();
            return None;
        }
        let start = self.pos;
        self.pos = next;
        Some((
            start,
            next[0] - start[0],
            next[1] - start[1],
            next[2] - start[2],
        ))
    }
}

fn trident_chains() -> AxisChainSet {
    let bell = |name: &str, smooth: f64| {
        CompiledChain::compile(&[PostProcessorInstance::new(
            name,
            &trajectory::algos::SmoothBell,
            vec![smooth],
        )])
        .expect("single post-processor always compiles")
    };
    let e = CompiledChain::compile(&[
        PostProcessorInstance::new("pa", &trajectory::algos::LinearPressureAdvance, vec![0.04]),
        PostProcessorInstance::new("st", &trajectory::algos::SmoothBell, vec![0.013]),
    ])
    .expect("pa + kernel compiles");
    AxisChainSet {
        chains: vec![
            bell("is_x", 0.0105),
            bell("is_y", 0.0155),
            CompiledChain::default(),
            e,
        ],
        followers: vec![(3, vec![0, 1, 2])],
    }
}

fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() < 2 {
        eprintln!("usage: gcode_replay_bench <in.gcode> [t_lo t_hi]");
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

    // Trident bench printer.cfg limits.
    let corner_deviation = geometry::corner_deviation_from_scv(85.1, 30_000.0);
    let limits = VelocityLimits::try_new(1_000.0, 30_000.0, corner_deviation, f64::INFINITY)
        .expect("trident limits");
    let cfg = StreamConfig {
        corner: CornerFitConfig::default(),
        integration_tol: 1e-4,
        max_extrude_only_velocity_mm_s: f64::INFINITY,
        max_extrude_only_accel_mm_s2: f64::INFINITY,
        fit_tol_mm: 0.005,
        fit_tol_accel_mm_s2: 50.0,
        max_buffer_moves: 512,
        limits,
    };

    let wall = Instant::now();
    let handle = setup_stages(cfg, trident_chains(), vec![0.0; 4], 0.0);
    let output = handle.output;
    let collector = thread::spawn(move || {
        let started = Instant::now();
        let mut arrivals: Vec<(f64, f64)> = Vec::new();
        while let Ok(item) = output.recv() {
            if let TrajectoryItem::Seg(seg) = item {
                arrivals.push((started.elapsed().as_secs_f64(), seg.t_end));
            }
        }
        arrivals
    });

    let mut p = Pos {
        pos: [0.0; 3],
        feed: 80.0,
        absolute: true,
        relative_e: false,
        established: false,
    };
    let mut submitted = 0u64;
    for tok in gcode::lex(&source) {
        let Ok(t) = tok else { continue };
        let gcode::Token::Command {
            letter,
            major,
            params,
            ..
        } = t
        else {
            continue;
        };
        match (letter, major) {
            (b'G', 0) | (b'G', 1) => {
                let e_word = params.e();
                let Some((start, dx, dy, dz)) =
                    p.apply(params.x(), params.y(), params.z(), params.f())
                else {
                    continue;
                };
                let de = if p.relative_e {
                    e_word.unwrap_or(0.0)
                } else {
                    0.0
                };
                if dx.abs() < 1e-9 && dy.abs() < 1e-9 && dz.abs() < 1e-9 && de.abs() < 1e-9 {
                    continue;
                }
                let m = match build_move(
                    start,
                    [dx, dy, dz],
                    3,
                    de,
                    limits,
                    p.feed,
                    submitted as u32,
                ) {
                    Ok(m) => m,
                    Err(e) => {
                        eprintln!("build_move line {submitted}: {e:?}");
                        continue;
                    }
                };
                if handle.input.send(m.into()).is_err() {
                    eprintln!("pipeline input closed at line {submitted}");
                    process::exit(1);
                }
                submitted += 1;
            }
            (b'G', 90) => p.absolute = true,
            (b'G', 91) => p.absolute = false,
            (b'M', 82) => p.relative_e = false,
            (b'M', 83) => p.relative_e = true,
            _ => {}
        }
    }
    drop(handle.input);
    let arrivals = collector.join().expect("pipeline collector panicked");
    let wall_s = wall.elapsed().as_secs_f64();
    let stream_s = arrivals.last().map_or(0.0, |&(_, t)| t);
    println!(
        "moves={submitted} segments={} stream_secs={stream_s:.1} wall_secs={wall_s:.1} overall_x={:.2}",
        arrivals.len(),
        stream_s / wall_s
    );

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
}
