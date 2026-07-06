// Replays a gcode file through the streaming pipeline and reports per-axis
// piece statistics (knot spans of the shaped segments — what the dispatcher
// flattens 1:1 into pump pieces). Bench-repro tool for the Neptune
// pump_piece_in_past crash: compares E-axis fragmentation across branches.
//
//   cargo run --release -p motion-engine --example dump_piece_stats -- <file.gcode>

use std::env;
use std::fs;
use std::process;

use _motion_engine::classify::build_move;
use geometry::{ChainFitConfig, VelocityLimits};
use motion_pipeline::{StreamConfig, setup_stages};
use trajectory::{AxisChainSet, ShapedSegment};

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
        if let Some(fv) = f {
            self.feed = fv / 60.0;
        }
        let target = if self.absolute {
            [
                x.unwrap_or(self.pos[0]),
                y.unwrap_or(self.pos[1]),
                z.unwrap_or(self.pos[2]),
            ]
        } else {
            [
                self.pos[0] + x.unwrap_or(0.0),
                self.pos[1] + y.unwrap_or(0.0),
                self.pos[2] + z.unwrap_or(0.0),
            ]
        };
        let d = [
            target[0] - self.pos[0],
            target[1] - self.pos[1],
            target[2] - self.pos[2],
        ];
        let start = self.pos;
        self.pos = target;
        if !self.established {
            self.established = true;
            return None;
        }
        Some((start, d[0], d[1], d[2]))
    }
}

fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() < 2 {
        eprintln!("usage: dump_piece_stats <in.gcode>");
        process::exit(1);
    }
    let source = fs::read_to_string(&args[1]).unwrap_or_else(|e| {
        eprintln!("cannot read {}: {e}", args[1]);
        process::exit(1);
    });

    // Neptune bench printer.cfg limits.
    let limits = VelocityLimits::try_new(300.0, 4000.0, 8.0, 1_000_000.0).unwrap();
    let cfg = StreamConfig {
        chain: ChainFitConfig::with_arc_fit(3),
        integration_tol: 1e-4,
        max_extrude_only_velocity_mm_s: f64::INFINITY,
        max_extrude_only_accel_mm_s2: f64::INFINITY,
        fit_tol_mm: 0.005,
        fit_tol_accel_mm_s2: 50.0,
        max_buffer_moves: 512,
        limits,
    };

    let wall = std::time::Instant::now();
    let handle = setup_stages(cfg, AxisChainSet::default(), vec![0.0; 4], 0.0);
    let output = handle.output;
    let collector = std::thread::spawn(move || {
        let mut segs: Vec<ShapedSegment> = Vec::new();
        while let Ok(item) = output.recv() {
            if let motion_pipeline::ShapedItem::Seg(seg) = item {
                segs.push(seg);
            }
        }
        segs
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
                let Some((start, dx, dy, dz)) = p.apply(params.x(), params.y(), params.z(), params.f())
                else {
                    continue;
                };
                let de = if p.relative_e { e_word.unwrap_or(0.0) } else { 0.0 };
                if dx.abs() < 1e-9 && dy.abs() < 1e-9 && dz.abs() < 1e-9 && de.abs() < 1e-9 {
                    continue;
                }
                let m = match build_move(start, dx, dy, dz, 3, de, limits, p.feed, submitted as u32)
                {
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
    let segs = collector.join().expect("pipeline collector panicked");
    let wall_s = wall.elapsed().as_secs_f64();

    let n_axes = segs.iter().map(|s| s.axes.len()).max().unwrap_or(0);
    let t_total: f64 = segs.iter().map(|s| s.t_end - s.t_start).sum();
    println!(
        "moves={submitted} segments={} traj_secs={t_total:.1} host_wall_secs={wall_s:.1}",
        segs.len()
    );
    for axis in 0..n_axes {
        let mut durs: Vec<f64> = Vec::new();
        for seg in &segs {
            let Some(curve) = seg.axes.get(axis) else {
                continue;
            };
            let knots = curve.knots();
            for w in knots.windows(2) {
                let d = w[1] - w[0];
                if d > 0.0 {
                    durs.push(d);
                }
            }
        }
        if durs.is_empty() {
            println!("axis {axis}: no pieces");
            continue;
        }
        durs.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let n = durs.len();
        let sub200 = durs.iter().filter(|d| **d < 200e-6).count();
        let sub1ms = durs.iter().filter(|d| **d < 1e-3).count();
        println!(
            "axis {axis}: pieces={n} rate={:.0}/traj-s min={:.1}us p50={:.1}us sub200us={sub200} sub1ms={sub1ms}",
            n as f64 / t_total,
            durs[0] * 1e6,
            durs[n / 2] * 1e6,
        );
    }
}
