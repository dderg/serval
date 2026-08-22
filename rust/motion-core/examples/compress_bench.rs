// Compress bench: how well do shaped Serval trajectories compress, and how
// lossy is each encoder?
//
// Route: full pipeline. The bench builds the production stage chain
// (fit -> planner -> lowerer -> shaper, fit_tol 0.005 as in
// dump_piece_stats.rs) with an input shaper on X/Y and pressure advance on
// the E follower, drives a synthetic-but-realistic print (300 mm/s travel,
// 0.4 mm zigzag infill at 150 mm/s, 60 mm/s perimeter with 90° corners)
// through it, samples the shaped X track at 20 kHz into a step-clock stream
// (secant sub-sample crossing times, mirroring the shim's arithmetic), then
// compresses that stream with the classic encoder at two error budgets and
// the high-precision encoder, reconstructing each encoder's emitted step
// times via its own arithmetic (the HP walk is the reference MCU
// `queue_step_hp` fixed-point walk: interval/add/add2 at 2^shift scale with
// a half-unit rounding accumulator).
//
//   cargo run --release -p motion-core --example compress_bench

use std::thread;

use geometry::{CornerFitConfig, VelocityLimits};
use motion_core::classify::build_move;
use motion_pipeline::{ShapedItem, StreamConfig, setup_stages};
use nurbs::eval::eval;
use step_shim::compress::compress_with_max_error;
use step_shim::compress_hp::{StepMoveHp, compress_hp};
use trajectory::{AxisChainSet, CompiledChain, PostProcessorInstance, ShapedSegment};

const SAMPLE_RATE_HZ: f64 = 20_000.0;
const MICROSTEP_MM: f64 = 0.00078;
const INFILL_WIDTH_MM: f64 = 0.4;
const LAYER_HEIGHT_MM: f64 = 0.2;
const FILAMENT_AREA_MM2: f64 = std::f64::consts::PI * 0.875 * 0.875;
const CLASSIC_WIRE_BYTES: u64 = 16;
const HP_WIRE_BYTES: u64 = 19;
const AXIS: usize = 0;

/// E advance per mm of a 0.4 mm x 0.2 mm bead of 1.75 mm filament.
const E_PER_MM: f64 = INFILL_WIDTH_MM * LAYER_HEIGHT_MM / FILAMENT_AREA_MM2;

fn bench_chains() -> AxisChainSet {
    let bell = |name: &str, smooth: f64| {
        CompiledChain::compile(&[PostProcessorInstance::new(
            name,
            &trajectory::algos::SmoothBell,
            vec![smooth],
        )])
        .expect("single post-processor always compiles")
    };
    let e = CompiledChain::compile(&[
        PostProcessorInstance::new("pa", &trajectory::algos::LinearPressureAdvance, vec![0.05]),
        PostProcessorInstance::new("st", &trajectory::algos::SmoothBell, vec![0.02]),
    ])
    .expect("pa + kernel compiles");
    AxisChainSet {
        chains: vec![
            bell("is_x", 0.044_583_333_333_333_336),
            bell("is_y", 0.044_583_333_333_333_336),
            CompiledChain::default(),
            e,
        ],
        followers: vec![(3, vec![0, 1, 2])],
    }
}

fn run_pipeline() -> Vec<ShapedSegment> {
    // Neptune bench printer.cfg limits (same as dump_piece_stats.rs).
    let limits = VelocityLimits::try_new(300.0, 4000.0, 8.0, 1_000_000.0).unwrap();
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

    let handle = setup_stages(cfg, bench_chains(), vec![0.0; 4], 0.0);
    let output = handle.output;
    let collector = thread::spawn(move || {
        let mut segs: Vec<ShapedSegment> = Vec::new();
        while let Ok(item) = output.recv() {
            if let ShapedItem::Seg(seg) = item {
                segs.push(seg);
            }
        }
        segs
    });

    let mut moves: Vec<geometry::Move> = Vec::new();
    let mut line = 0u32;
    let mut plan = |start: [f64; 3], delta: [f64; 3], e_delta: f64, feed: f64| {
        moves.push(build_move(start, delta, 3, e_delta, limits, feed, line).expect("build_move"));
        line += 1;
    };

    // 300 mm/s travel across the bed.
    plan([0.0, 0.0, 0.0], [200.0, 0.0, 0.0], 0.0, 300.0);
    plan([200.0, 0.0, 0.0], [-190.0, 10.0, 0.0], 0.0, 300.0);

    // 0.4 mm zigzag infill, 20 lines along Y at 150 mm/s, extruding.
    let mut x = 10.0;
    let mut y = 10.0;
    for i in 0..20 {
        let y_end = if i % 2 == 0 { 60.0 } else { 10.0 };
        let dy = y_end - y;
        plan([x, y, 0.0], [0.0, dy, 0.0], E_PER_MM * dy.abs(), 150.0);
        y = y_end;
        if i + 1 < 20 {
            plan([x, y, 0.0], [INFILL_WIDTH_MM, 0.0, 0.0], 0.0, 150.0);
            x += INFILL_WIDTH_MM;
        }
    }

    // Travel to the perimeter's first corner, then a 60 mm/s perimeter with
    // 90° corners around the infill block.
    plan([17.6, 10.0, 0.0], [20.4, 0.0, 0.0], 0.0, 300.0);
    let perimeter = [
        (38.0, 10.0),
        (38.0, 50.0),
        (8.0, 50.0),
        (8.0, 10.0),
        (38.0, 10.0),
    ];
    for w in perimeter.windows(2) {
        let (sx, sy): (f64, f64) = w[0];
        let (ex, ey): (f64, f64) = w[1];
        let len = ((ex - sx).powi(2) + (ey - sy).powi(2)).sqrt();
        plan([sx, sy, 0.0], [ex - sx, ey - sy, 0.0], E_PER_MM * len, 60.0);
    }

    // 300 mm/s travel home.
    plan([38.0, 10.0, 0.0], [-38.0, -10.0, 0.0], 0.0, 300.0);

    for m in moves {
        handle.input.send(m.into()).expect("pipeline input closed");
    }
    drop(handle.input);
    let segs = collector.join().expect("pipeline collector panicked");
    assert!(!segs.is_empty(), "pipeline emitted no segments");
    for w in segs.windows(2) {
        assert!(
            w[1].t_start >= w[0].t_end - 1e-9,
            "segments must arrive abutting and in order"
        );
    }
    segs
}

/// Mirror of `runtime::sub_sample_timing::quantize_step_delta` (round to
/// nearest microstep boundary, snapping exactly-half values down to avoid
/// double counting a boundary) in f64.
fn quantize_step_delta(phase: f64) -> i64 {
    let target = (phase / MICROSTEP_MM).round() as i64;
    if target > 0 && phase <= (target as f64 - 0.5) * MICROSTEP_MM {
        target - 1
    } else if target < 0 && phase >= (target as f64 + 0.5) * MICROSTEP_MM {
        target + 1
    } else {
        target
    }
}

struct SampledStep {
    clock: u64,
    forward: bool,
    /// Secant velocity over the bracketing sample window, mm/s (signed).
    v_local: f64,
}

/// Sample the shaped `axis` track at 20 kHz into a step-clock stream using
/// the shim's secant sub-sample crossing arithmetic.
fn sample_steps(segs: &[ShapedSegment], axis: usize, freq: f64) -> Vec<SampledStep> {
    let t0 = segs.first().expect("non-empty").t_start;
    let t1 = segs.last().expect("non-empty").t_end;
    let period = 1.0 / SAMPLE_RATE_HZ;
    let sample_period_cycles = (freq / SAMPLE_RATE_HZ) as u64;
    assert_eq!(
        sample_period_cycles as f64 * SAMPLE_RATE_HZ,
        freq,
        "clock variant must divide evenly into 20 kHz samples"
    );
    let origin_clock = (t0 * freq) as u64;
    let n_samples = ((t1 - t0) / period).ceil() as usize + 1;

    let mut out: Vec<SampledStep> = Vec::new();
    let mut seg_idx = 0usize;
    let mut prev_p = match segs[0].axes.get(axis) {
        Some(curve) => eval(curve, t0),
        None => 0.0,
    };
    let mut phase = 0.0_f64;
    for k in 0..n_samples {
        let t = (t0 + k as f64 * period).min(t1);
        while seg_idx + 1 < segs.len() && t > segs[seg_idx + 1].t_start {
            seg_idx += 1;
        }
        let p = match segs[seg_idx].axes.get(axis) {
            Some(curve) => eval(curve, t),
            None => prev_p,
        };
        let phase_end = phase + (p - prev_p);
        let delta = quantize_step_delta(phase_end);
        if delta != 0 {
            let n = delta.unsigned_abs() as usize;
            let forward = delta > 0;
            let dir = if forward { 1.0_f64 } else { -1.0_f64 };
            let disp = p - prev_p;
            assert!(disp != 0.0, "step delta without displacement");
            let sample_start_clock = origin_clock + k as u64 * sample_period_cycles;
            for kk in 0..n {
                let threshold = (kk as f64 + 0.5) * dir * MICROSTEP_MM;
                let t_local = (threshold - phase) * period / disp;
                assert!(
                    t_local > -period * 1e-9 && t_local < period * (1.0 + 1e-9),
                    "crossing time {t_local} outside the sample window"
                );
                let clock = sample_start_clock + (t_local * freq) as u64;
                out.push(SampledStep {
                    clock,
                    forward,
                    v_local: disp / period,
                });
            }
            phase = phase_end - delta as f64 * MICROSTEP_MM;
        } else {
            phase = phase_end;
        }
        prev_p = p;
    }
    assert!(!out.is_empty(), "axis {axis} produced no steps");
    assert!(
        out[0].clock > 0,
        "first step clock must leave room for an anchor"
    );
    for w in out.windows(2) {
        assert!(
            w[1].clock > w[0].clock,
            "step clocks must be strictly increasing"
        );
    }
    out
}

fn split_runs(steps: &[SampledStep]) -> Vec<(usize, usize)> {
    let mut runs: Vec<(usize, usize)> = Vec::new();
    let mut start = 0usize;
    for i in 1..=steps.len() {
        if i == steps.len() || steps[i].forward != steps[start].forward {
            runs.push((start, i));
            start = i;
        }
    }
    runs
}

struct EncoderStats {
    wire_bytes_per_move: u64,
    freq: f64,
    moves: usize,
    steps: usize,
    sum_steps_in_moves: u64,
    max_steps_in_move: u64,
    wire_bytes: u64,
    max_err_ticks: i64,
    sum_sq_err_ticks: u128,
    max_err_us: f64,
    max_pos_err_um: f64,
    rms_err_ticks: f64,
    rms_err_us: f64,
    mean_steps_per_move: f64,
}

impl EncoderStats {
    fn new(wire_bytes_per_move: u64, freq: f64) -> Self {
        Self {
            wire_bytes_per_move,
            freq,
            moves: 0,
            steps: 0,
            sum_steps_in_moves: 0,
            max_steps_in_move: 0,
            wire_bytes: 0,
            max_err_ticks: 0,
            sum_sq_err_ticks: 0,
            max_err_us: 0.0,
            max_pos_err_um: 0.0,
            rms_err_ticks: 0.0,
            rms_err_us: 0.0,
            mean_steps_per_move: 0.0,
        }
    }

    fn note_move(&mut self, count: u64) {
        self.moves += 1;
        self.sum_steps_in_moves += count;
        self.max_steps_in_move = self.max_steps_in_move.max(count);
    }

    fn note_step(&mut self, emitted: u64, want: u64, v_local: f64) {
        let err = emitted as i64 - want as i64;
        let abs_err = err.unsigned_abs();
        if abs_err as u64 > self.max_err_ticks as u64 {
            self.max_err_ticks = abs_err as i64;
            self.max_err_us = abs_err as f64 / self.freq * 1e6;
            self.max_pos_err_um = abs_err as f64 / self.freq * v_local * 1e3;
        }
        self.sum_sq_err_ticks += u128::from(abs_err) * u128::from(abs_err);
        self.steps += 1;
    }

    fn finish(&mut self) {
        let rms = (self.sum_sq_err_ticks as f64 / self.steps.max(1) as f64).sqrt();
        self.rms_err_ticks = rms;
        self.rms_err_us = rms / self.freq * 1e6;
        self.mean_steps_per_move = self.sum_steps_in_moves as f64 / self.moves as f64;
        self.wire_bytes = self.wire_bytes_per_move * self.moves as u64;
    }
}

fn encode_classic(
    stream: &[SampledStep],
    runs: &[(usize, usize)],
    freq: f64,
    max_error: u32,
) -> EncoderStats {
    let mut anchor = stream[0].clock - 1;
    let mut stats = EncoderStats::new(CLASSIC_WIRE_BYTES, freq);
    for &(start, end) in runs {
        let clocks: Vec<u64> = stream[start..end].iter().map(|s| s.clock).collect();
        let (moves, covered) =
            compress_with_max_error(&clocks, anchor, max_error).unwrap_or_else(|e| {
                panic!("classic (max_error={max_error}) failed on run {start}..{end}: {e}")
            });
        assert_eq!(
            covered,
            clocks.len(),
            "classic covered fewer steps than the run"
        );
        let mut move_anchor = anchor;
        let mut run_step = 0usize;
        for mv in &moves {
            stats.note_move(u64::from(mv.count));
            for n in 1..=mv.count {
                let emitted = mv.step_clock(move_anchor, n);
                let want = stream[start + run_step].clock;
                stats.note_step(emitted, want, stream[start + run_step].v_local.abs());
                run_step += 1;
            }
            move_anchor = mv.last_clock(move_anchor);
        }
        anchor = move_anchor;
    }
    stats.finish();
    stats
}

/// Reference MCU `queue_step_hp` fixed-point walk: step n's tick offset from
/// the move's anchor. For shift > 0 the wire values live at 2^shift scale and
/// a half-unit rounding accumulator is seeded at load, which makes the walk
/// exactly `floor((n*interval + add*n(n-1)/2 + add2*n(n-1)(n-2)/6 +
/// 2^(shift-1)) / 2^shift)`; for shift <= 0 the wire values are scaled up by
/// 2^-shift and the walk is exact integer arithmetic.
fn hp_step_offset(mv: &StepMoveHp, n: u32) -> u64 {
    let n = i128::from(n);
    let acc = i128::from(mv.interval) * n
        + i128::from(mv.add) * (n * (n - 1) / 2)
        + i128::from(mv.add2) * (n * (n - 1) * (n - 2) / 6);
    if mv.shift > 0 {
        let half = 1_i128 << (mv.shift - 1);
        ((acc + half) >> mv.shift) as u64
    } else {
        (acc << (-mv.shift)) as u64
    }
}

fn encode_hp(stream: &[SampledStep], runs: &[(usize, usize)], freq: f64) -> EncoderStats {
    let mut anchor = stream[0].clock - 1;
    let mut next_expected_interval: u32 = 0;
    let mut stats = EncoderStats::new(HP_WIRE_BYTES, freq);
    for &(start, end) in runs {
        let clocks: Vec<u64> = stream[start..end].iter().map(|s| s.clock).collect();
        let (moves, covered, carry) = compress_hp(&clocks, anchor, next_expected_interval)
            .unwrap_or_else(|e| panic!("hp failed on run {start}..{end}: {e}"));
        assert_eq!(covered, clocks.len(), "hp covered fewer steps than the run");
        let mut move_anchor = anchor;
        let mut run_step = 0usize;
        for mv in &moves {
            stats.note_move(u64::from(mv.count));
            let first = hp_step_offset(mv, 1);
            let last = hp_step_offset(mv, u32::from(mv.count));
            assert_eq!(
                first, mv.first_step,
                "hp emulation diverges from first_step: {} != {}",
                first, mv.first_step
            );
            assert_eq!(
                last, mv.last_step,
                "hp emulation diverges from last_step: {} != {}",
                last, mv.last_step
            );
            for n in 1..=u32::from(mv.count) {
                let emitted = move_anchor + hp_step_offset(mv, n);
                let want = stream[start + run_step].clock;
                stats.note_step(emitted, want, stream[start + run_step].v_local.abs());
                run_step += 1;
            }
            move_anchor += mv.last_step;
        }
        anchor = move_anchor;
        next_expected_interval = carry;
    }
    stats.finish();
    stats
}

fn bench_clock(freq: f64, segs: &[ShapedSegment]) {
    let stream = sample_steps(segs, AXIS, freq);
    let runs = split_runs(&stream);
    let motion_secs = (stream.last().unwrap().clock - stream[0].clock) as f64 / freq;
    let variants = [
        encode_classic(&stream, &runs, freq, 1600),
        encode_classic(&stream, &runs, freq, (25e-6 * freq) as u32),
        encode_hp(&stream, &runs, freq),
    ];
    let names = ["classic 1600t", "classic 25us", "hp"];

    println!(
        "clock {} MHz | axis {} | {} steps | {:.1} s of motion | {} runs | sample 20 kHz | ms 0.00078 mm",
        freq as u64 / 1_000_000,
        AXIS,
        stream.len(),
        motion_secs,
        runs.len(),
    );
    println!(
        "{:<17} {:>7} {:>12} {:>9} {:>11} {:>11} {:>11} {:>10} {:>9} {:>9}",
        "encoder",
        "moves",
        "steps/move",
        "wire B",
        "B/s motion",
        "max err t",
        "max err us",
        "rms err t",
        "rms us",
        "pos err um",
    );
    for (v, name) in variants.iter().zip(&names) {
        println!(
            "{:<17} {:>7} {:>12.1} {:>9} {:>11.0} {:>11} {:>11.3} {:>10.3} {:>9.3} {:>9.3}",
            name,
            v.moves,
            v.mean_steps_per_move,
            v.wire_bytes,
            v.wire_bytes as f64 / motion_secs,
            v.max_err_ticks,
            v.max_err_us,
            v.rms_err_ticks,
            v.rms_err_us,
            v.max_pos_err_um,
        );
    }
    println!();
}

fn main() {
    let segs = run_pipeline();
    bench_clock(550_000_000.0, &segs);
    bench_clock(168_000_000.0, &segs);
}
