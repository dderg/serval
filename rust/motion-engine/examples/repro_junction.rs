// Reproduces the pump's junction position-continuity check offline against a
// gcode file, driving the real StreamState commit/seam path. Reports every
// consecutive-segment boundary where an axis position jumps — i.e. exactly the
// condition that makes check_junction_position_continuity (pump.rs) panic.
//
//   cargo run --release -p motion-engine --example repro_junction -- <file.gcode> [--cap N]
//
// Bench limits (Neptune 3 Pro printer.cfg): max_v=100 accel=1000 scv=5.

use std::env;
use std::fs;
use std::process;

use _motion_engine::classify::build_move;
use _motion_engine::stream::{StreamConfig, StreamState};
use geometry::{ChainFitConfig, VelocityConfig, VelocityLimits};
use nurbs::eval::eval;
use trajectory::{AxisChainSet, ShapedSegment};

const FATAL_MM: f64 = 0.1;
const LOG_MM: f64 = 0.0125;
const AXES: [&str; 3] = ["X", "Y", "Z"];

struct Pos {
    pos: [f64; 3],
    feed: f64,
    absolute: bool,
    established: bool,
}

impl Pos {
    fn new() -> Self {
        Self {
            pos: [0.0; 3],
            feed: 80.0,
            absolute: true,
            established: false,
        }
    }
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
        eprintln!("usage: repro_junction <in.gcode> [--cap N]");
        process::exit(1);
    }
    let in_path = &args[1];
    let mut cap = 64usize;
    let mut i = 2;
    while i < args.len() {
        if args[i] == "--cap" {
            i += 1;
            cap = args[i].parse().unwrap();
        }
        i += 1;
    }

    let source = fs::read_to_string(in_path).unwrap_or_else(|e| {
        eprintln!("cannot read {in_path}: {e}");
        process::exit(1);
    });

    let limits = VelocityLimits::try_new(100.0, 1000.0, 5.0).unwrap();
    let cfg = StreamConfig {
        chain: ChainFitConfig::default(),
        velocity: VelocityConfig {
            integration_tol: 1e-4,
            ..VelocityConfig::default()
        },
        fit_tol_mm: 1e-3,
        max_buffer_moves: 512,
        limits,
    };

    let mut state = StreamState::new(cfg, AxisChainSet::default(), &[0.0, 0.0, 0.0], 0.0);
    let mut all: Vec<ShapedSegment> = Vec::new();
    let mut p = Pos::new();
    let mut submitted = 0u64;

    let mut batch: Vec<(f64, f64, f64, f64, u64)> = Vec::new();
    let commit_into = |state: &mut StreamState,
                       all: &mut Vec<ShapedSegment>,
                       batch: &mut Vec<(f64, f64, f64, f64, u64)>,
                       force: bool| {
        match state.commit(force) {
            Ok(segs) => {
                all.extend(segs);
                batch.clear();
            }
            Err(e) => {
                eprintln!("commit failed: {e}\nfailing batch ({} moves):", batch.len());
                for (dx, dy, dz, feed, ln) in batch.iter() {
                    let len = (dx * dx + dy * dy + dz * dz).sqrt();
                    eprintln!(
                        "  move#{ln}: d=({dx:.4},{dy:.4},{dz:.4}) len={len:.5}mm feed={feed:.2}"
                    );
                }
                process::exit(1);
            }
        }
    };

    for tok in gcode::lex(&source) {
        let Ok(t) = tok else { continue };
        let (letter, major, params) = match t {
            gcode::Token::Command {
                letter,
                major,
                params,
                ..
            } => (letter, major, params),
            _ => continue,
        };
        if letter != b'G' {
            continue;
        }
        match major {
            0 | 1 => {
                let Some((start, dx, dy, dz)) =
                    p.apply(params.x(), params.y(), params.z(), params.f())
                else {
                    continue;
                };
                if dx.abs() < 1e-9 && dy.abs() < 1e-9 && dz.abs() < 1e-9 {
                    continue;
                }
                let m =
                    match build_move(start, dx, dy, dz, 3, 0.0, limits, p.feed, submitted as u32) {
                        Ok(m) => m,
                        Err(e) => {
                            eprintln!("build_move line {submitted}: {e:?}");
                            continue;
                        }
                    };
                state.push(m);
                batch.push((dx, dy, dz, p.feed, submitted));
                submitted += 1;
                if state.buffered() >= cap {
                    commit_into(&mut state, &mut all, &mut batch, false);
                }
            }
            90 => p.absolute = true,
            91 => p.absolute = false,
            _ => {}
        }
    }
    commit_into(&mut state, &mut all, &mut batch, true);

    let mut fatal = 0usize;
    let mut logged = 0usize;
    let mut worst = 0.0f64;
    for w in all.windows(2) {
        let (a, b) = (&w[0], &w[1]);
        let dt = b.t_start - a.t_start;
        for ax in 0..3 {
            let end = eval(&a.axes[ax], a.t_end);
            let start = eval(&b.axes[ax], b.t_start);
            let jump = (start - end).abs();
            worst = worst.max(jump);
            if jump >= FATAL_MM {
                fatal += 1;
                if fatal <= 20 {
                    println!(
                        "FATAL {ax}={} |Δ|={jump:.5}mm  end={end:.5} start={start:.5}  \
                         t_end={:.6} t_start={:.6} dt={dt:.6}",
                        AXES[ax], a.t_end, b.t_start
                    );
                }
            } else if jump >= LOG_MM {
                logged += 1;
            }
        }
    }
    println!(
        "\nmoves={submitted} segments={} cap={cap}  FATAL(>={FATAL_MM})={fatal} \
         LOG(>={LOG_MM})={logged} worst=|Δ|={worst:.5}mm",
        all.len()
    );
}
