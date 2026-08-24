// Replays a gcode file through the streaming pipeline and reports per-axis
// span statistics: the motor spans the dispatcher hands the endpoint, and the
// breakpoint segments inside them. Bench-repro tool for the Neptune
// pump_piece_in_past crash: compares E-axis fragmentation across branches.
//
//   cargo run --release -p motion-engine --example dump_piece_stats -- <file.gcode>

use std::env;
use std::fs;
use std::process;
use std::sync::Arc;

use geometry::{CornerFitConfig, VelocityLimits};
use motion_core::classify::build_move;
use motion_pipeline::{StreamConfig, TrajectoryItem, setup_stages};
use trajectory::{
    AxisChainSet, ContinuousAxis, ContinuousSegment, MotorGroup, MotorSpan, MotorTerm,
};

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

fn axis_kind(axis: &ContinuousAxis) -> &'static str {
    match axis {
        ContinuousAxis::Analytic { .. } => "analytic",
        ContinuousAxis::Spline(_) => "spline",
        ContinuousAxis::RelativeSpline { .. } => "relative_spline",
        ContinuousAxis::PiecewiseRelativeSpline(_) => "piecewise_relative_spline",
        ContinuousAxis::Hold { .. } => "hold",
        ContinuousAxis::Nudge(_) => "nudge",
        ContinuousAxis::Buzz { .. } => "buzz",
    }
}

fn lane_span(seg: &ContinuousSegment, axis: usize) -> MotorSpan {
    let term = MotorTerm {
        source_axis: axis,
        axis: seg.axes[axis].clone(),
        scale: 1.0,
    };
    MotorSpan::try_new(
        Arc::from([MotorGroup::Independent(term)]),
        seg.t_start,
        seg.t_end,
        seg.motor_mask,
        seg.source_line,
        false,
    )
    .unwrap_or_else(|e| {
        eprintln!(
            "axis {axis} line {}: undispatchable span: {e}",
            seg.source_line
        );
        process::exit(1);
    })
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
    let limits = VelocityLimits::try_new(300.0, 4000.0, 8.0, f64::INFINITY).unwrap();
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

    let wall = std::time::Instant::now();
    let handle = setup_stages(cfg, AxisChainSet::default(), vec![0.0; 4], 0.0);
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
    let segs = collector.join().expect("pipeline collector panicked");
    let wall_s = wall.elapsed().as_secs_f64();

    let n_axes = segs.iter().map(|s| s.axes.len()).max().unwrap_or(0);
    let t_total: f64 = segs.iter().map(|s| s.t_end - s.t_start).sum();
    println!(
        "moves={submitted} segments={} traj_secs={t_total:.1} host_wall_secs={wall_s:.1}",
        segs.len()
    );
    for axis in 0..n_axes {
        let mut kinds: Vec<(&'static str, usize)> = Vec::new();
        let mut span_durs: Vec<f64> = Vec::new();
        let mut seg_durs: Vec<f64> = Vec::new();
        let mut speed_max = 0.0f64;
        let mut accel_max = 0.0f64;
        for seg in &segs {
            let Some(source) = seg.axes.get(axis) else {
                continue;
            };
            let kind = axis_kind(source);
            match kinds.iter_mut().find(|(name, _)| *name == kind) {
                Some((_, count)) => *count += 1,
                None => kinds.push((kind, 1)),
            }
            let span = lane_span(seg, axis);
            span_durs.push(span.t_end - span.t_start);
            for w in span.breakpoints.windows(2) {
                let d = w[1] - w[0];
                if d > 0.0 {
                    seg_durs.push(d);
                }
            }
            let bounds = span
                .pva_bounds(span.t_start, span.t_end)
                .unwrap_or_else(|e| {
                    eprintln!("axis {axis} line {}: bounds failed: {e}", seg.source_line);
                    process::exit(1);
                });
            speed_max = speed_max
                .max(bounds.velocity_max.abs())
                .max(bounds.velocity_min.abs());
            accel_max = accel_max.max(bounds.acceleration_abs_max);
        }
        if seg_durs.is_empty() {
            println!("axis {axis}: no spans");
            continue;
        }
        seg_durs.sort_by(f64::total_cmp);
        span_durs.sort_by(f64::total_cmp);
        let n = seg_durs.len();
        let sub200 = seg_durs.iter().filter(|d| **d < 200e-6).count();
        let sub1ms = seg_durs.iter().filter(|d| **d < 1e-3).count();
        let kind_mix = kinds
            .iter()
            .map(|(name, count)| format!("{name}={count}"))
            .collect::<Vec<_>>()
            .join(",");
        println!(
            "axis {axis} [{kind_mix}]: spans={} span_p50={:.1}us segments={n} rate={:.0}/traj-s min={:.1}us p50={:.1}us sub200us={sub200} sub1ms={sub1ms} vmax={speed_max:.1}mm/s amax={accel_max:.0}mm/s2",
            span_durs.len(),
            span_durs[span_durs.len() / 2] * 1e6,
            n as f64 / t_total,
            seg_durs[0] * 1e6,
            seg_durs[n / 2] * 1e6,
        );
    }
}
