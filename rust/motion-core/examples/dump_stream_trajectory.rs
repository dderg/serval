// Replays a gcode file through the streaming pipeline and dumps the shaped
// trajectory as CSV: per axis position, velocity and acceleration.
//
//   cargo run --release -p motion-engine --example dump_stream_trajectory -- \
//       <file.gcode> <out.csv> [--dt S] [--scv MM_S]

use std::env;
use std::fs;
use std::io::Write;
use std::process;

use geometry::{CornerFitConfig, VelocityLimits};
use motion_core::classify::build_move;
use motion_pipeline::{StreamConfig, TrajectoryItem, setup_stages};
use trajectory::{AxisChainSet, ContinuousSegment};

const AXIS_LABELS: [&str; 4] = ["x", "y", "z", "e"];

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

fn axis_label(axis: usize) -> String {
    AXIS_LABELS
        .get(axis)
        .map_or_else(|| format!("a{axis}"), |label| (*label).to_string())
}

fn sample(seg: &ContinuousSegment, axis: usize, t: f64) -> trajectory::Pva {
    seg.eval_axis(axis, t).unwrap_or_else(|e| {
        eprintln!("axis {axis} line {} at t={t}: {e}", seg.source_line);
        process::exit(1);
    })
}

fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() < 3 {
        eprintln!("usage: dump_stream_trajectory <in.gcode> <out.csv> [--dt S]");
        process::exit(1);
    }
    let in_path = &args[1];
    let out_path = &args[2];
    let mut dt = 0.005;
    let mut scv = 10.0;
    let mut i = 3;
    while i < args.len() {
        match args[i].as_str() {
            "--dt" => {
                i += 1;
                dt = args[i].parse().unwrap();
            }
            "--scv" => {
                i += 1;
                scv = args[i].parse().unwrap();
            }
            other => {
                eprintln!("unknown arg {other}");
                process::exit(1);
            }
        }
        i += 1;
    }

    let source = fs::read_to_string(in_path).unwrap_or_else(|e| {
        eprintln!("cannot read {in_path}: {e}");
        process::exit(1);
    });

    let limits = VelocityLimits::try_new(100.0, 1000.0, scv, f64::INFINITY).unwrap();
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

    let handle = setup_stages(cfg, AxisChainSet::default(), vec![0.0, 0.0, 0.0, 0.0], 0.0);
    let output = handle.output;
    let collector = std::thread::spawn(move || {
        let mut segs: Vec<ContinuousSegment> = Vec::new();
        while let Ok(item) = output.recv() {
            if let TrajectoryItem::Seg(seg) = item {
                segs.push(seg);
            }
        }
        segs
    });
    let mut p = Pos::new();
    let mut submitted = 0u64;

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
                let de = params.e().unwrap_or(0.0);
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
            90 => p.absolute = true,
            91 => p.absolute = false,
            _ => {}
        }
    }
    drop(handle.input);
    let all = collector.join().expect("pipeline collector panicked");

    let n_axes = all.iter().map(|s| s.axes.len()).max().unwrap_or(0);
    let mut out = fs::File::create(out_path).unwrap();
    let mut header = String::from("seg,t");
    for axis in 0..n_axes {
        let label = axis_label(axis);
        header.push_str(&format!(",{label},v{label},a{label}"));
    }
    writeln!(out, "{header}").unwrap();

    let mut zmin = f64::MAX;
    let mut zmax = f64::MIN;
    for (idx, seg) in all.iter().enumerate() {
        let n = ((seg.t_end - seg.t_start) / dt).ceil().max(1.0) as usize;
        for k in 0..=n {
            let t = (seg.t_start + (k as f64) * dt).min(seg.t_end);
            let mut row = format!("{idx},{t:.6}");
            for axis in 0..seg.axes.len() {
                let pva = sample(seg, axis, t);
                if axis == 2 {
                    zmin = zmin.min(pva.position);
                    zmax = zmax.max(pva.position);
                }
                row.push_str(&format!(
                    ",{:.5},{:.4},{:.2}",
                    pva.position, pva.velocity, pva.acceleration
                ));
            }
            writeln!(out, "{row}").unwrap();
        }
    }
    println!(
        "moves={submitted} segments={} z_range=[{zmin:.4}, {zmax:.4}] -> {out_path}",
        all.len()
    );
}
