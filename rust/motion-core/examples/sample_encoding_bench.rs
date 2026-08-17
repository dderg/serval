// Sample-stream encoding bench: how many bytes per second per axis does the
// sample transport spend, and which delta encoding wins?
//
// Route: full pipeline. The bench builds the production stage chain
// (fit -> planner -> lowerer -> shaper) with an input shaper on X/Y and
// pressure advance on the E follower, drives the same synthetic-but-realistic
// print compress_bench uses (300 mm/s travel, 0.4 mm zigzag infill at
// 150 mm/s, 60 mm/s perimeter with 90 deg corners), then samples the shaped X
// track onto the phase lane's quantum at 2 kHz and 4 kHz and packs the result
// into `sample_run` payloads.
//
// Two candidate encodings are measured over identical runs:
//   varint  - zigzag LEB128 first differences, the codec in
//             runtime::sample_run (measured, not assumed)
//   i16     - fixed two-byte little-endian first differences, with a 1-byte
//             escape plus a 4-byte i32 for any delta i16 cannot hold
//
//   cargo run --release -p motion-core --example sample_encoding_bench

use std::thread;

use geometry::{CornerFitConfig, VelocityLimits};
use motion_core::classify::build_move;
use motion_pipeline::{ShapedItem, StreamConfig, setup_stages};
use nurbs::eval::eval;
use runtime::sample_run::{SAMPLE_RUN_COUNT_MAX, SAMPLE_RUN_DATA_MAX, delta_bytes};
use trajectory::{AxisChainSet, CompiledChain, PostProcessorInstance, ShapedSegment};

/// The phase lane's quantum: one LUT phase increment, which at 1024 phases per
/// electrical cycle is one 256-microstep of the bench motor.
const PHASE_QUANTUM_MM: f64 = 0.00078;

const INFILL_WIDTH_MM: f64 = 0.4;
const LAYER_HEIGHT_MM: f64 = 0.2;
const FILAMENT_AREA_MM2: f64 = std::f64::consts::PI * 0.875 * 0.875;
const AXIS: usize = 0;
const CYCLES_PER_SECOND: f64 = 550_000_000.0;

/// Per-run wire overhead of `sample_run oid=%c interval=%u count=%c data=%*s`:
/// msgid, oid, count and the payload length byte, plus the interval varint.
const RUN_FIXED_BYTES: u64 = 4;

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

    plan([0.0, 0.0, 0.0], [200.0, 0.0, 0.0], 0.0, 300.0);
    plan([200.0, 0.0, 0.0], [-190.0, 10.0, 0.0], 0.0, 300.0);

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

    plan([38.0, 10.0, 0.0], [-38.0, -10.0, 0.0], 0.0, 300.0);

    for m in moves {
        handle.input.send(m.into()).expect("pipeline input closed");
    }
    drop(handle.input);
    let segs = collector.join().expect("pipeline collector panicked");
    assert!(!segs.is_empty(), "pipeline emitted no segments");
    segs
}

/// Mirror of `runtime::sub_sample_timing::quantize_step_delta` in f64: round to
/// the nearest quantum, snapping exactly-half values down so a boundary is
/// never counted twice.
fn quantize(phase: f64) -> i64 {
    let target = (phase / PHASE_QUANTUM_MM).round() as i64;
    if target > 0 && phase <= (target as f64 - 0.5) * PHASE_QUANTUM_MM {
        target - 1
    } else if target < 0 && phase >= (target as f64 + 0.5) * PHASE_QUANTUM_MM {
        target + 1
    } else {
        target
    }
}

/// Sample the shaped `axis` track onto the lane's quantum, exactly as the pump
/// sample sink does: accumulate the residual phase and quantize the total.
fn sample_positions(segs: &[ShapedSegment], axis: usize, rate_hz: f64) -> (Vec<i32>, f64) {
    let t0 = segs.first().expect("non-empty").t_start;
    let t1 = segs.last().expect("non-empty").t_end;
    let period = 1.0 / rate_hz;
    let n_samples = ((t1 - t0) / period).ceil() as usize + 1;

    let mut out: Vec<i32> = Vec::with_capacity(n_samples);
    let mut seg_idx = 0usize;
    let mut prev_p = segs[0].axes.get(axis).map_or(0.0, |c| eval(c, t0));
    let mut phase = 0.0f64;
    let mut position = 0i64;
    for k in 0..n_samples {
        let t = (t0 + k as f64 * period).min(t1);
        while seg_idx + 1 < segs.len() && t > segs[seg_idx + 1].t_start {
            seg_idx += 1;
        }
        let p = segs[seg_idx].axes.get(axis).map_or(prev_p, |c| eval(c, t));
        phase += p - prev_p;
        prev_p = p;
        let delta = quantize(phase);
        phase -= delta as f64 * PHASE_QUANTUM_MM;
        position += delta;
        out.push(i32::try_from(position).expect("the bench stays inside the lane's fixed point"));
    }
    (out, t1 - t0)
}

fn i16_delta_bytes(delta: i64) -> u64 {
    if i16::try_from(delta).is_ok() { 2 } else { 5 }
}

struct Packing {
    runs: u64,
    payload_bytes: u64,
    samples: u64,
    largest_run: usize,
    smallest_run: usize,
}

impl Packing {
    fn wire_bytes(&self, interval_varint: u64) -> u64 {
        self.payload_bytes + self.runs * (RUN_FIXED_BYTES + interval_varint)
    }
}

/// Pack the sample stream into runs under the same two limits the sink honors:
/// [`SAMPLE_RUN_DATA_MAX`] payload bytes and [`SAMPLE_RUN_COUNT_MAX`] samples.
fn pack(positions: &[i32], cost: &dyn Fn(i32, i32) -> u64) -> Packing {
    let mut runs = 0u64;
    let mut payload_bytes = 0u64;
    let mut largest_run = 0usize;
    let mut smallest_run = usize::MAX;
    let mut run_bytes = 0u64;
    let mut run_len = 0usize;
    let mut previous = 0i32;
    for &position in positions {
        let bytes = cost(previous, position);
        if run_len == SAMPLE_RUN_COUNT_MAX || run_bytes + bytes > SAMPLE_RUN_DATA_MAX as u64 {
            runs += 1;
            payload_bytes += run_bytes;
            largest_run = largest_run.max(run_len);
            smallest_run = smallest_run.min(run_len);
            run_bytes = cost(previous, position);
            run_len = 0;
        } else {
            run_bytes += bytes;
        }
        run_len += 1;
        previous = position;
    }
    if run_len > 0 {
        runs += 1;
        payload_bytes += run_bytes;
        largest_run = largest_run.max(run_len);
        smallest_run = smallest_run.min(run_len);
    }
    Packing {
        runs,
        payload_bytes,
        samples: positions.len() as u64,
        largest_run,
        smallest_run,
    }
}

fn interval_varint_len(rate_hz: f64) -> u64 {
    let mut interval = (CYCLES_PER_SECOND / rate_hz).round() as u64;
    let mut len = 1;
    while interval >= 0x80 {
        interval >>= 7;
        len += 1;
    }
    len
}

fn bench_rate(segs: &[ShapedSegment], rate_hz: f64) {
    let (positions, motion_secs) = sample_positions(segs, AXIS, rate_hz);
    let interval_varint = interval_varint_len(rate_hz);
    let deltas: Vec<i64> = positions
        .iter()
        .scan(0i64, |prev, &p| {
            let d = i64::from(p) - *prev;
            *prev = i64::from(p);
            Some(d)
        })
        .collect();
    let peak_delta = deltas.iter().copied().map(i64::abs).max().unwrap_or(0);

    let varint = pack(&positions, &|base, position| {
        delta_bytes(base, position).expect("bench deltas stay inside i32") as u64
    });
    let fixed = pack(&positions, &|base, position| {
        i16_delta_bytes(i64::from(position) - i64::from(base))
    });

    let varint_wire = varint.wire_bytes(interval_varint);
    let fixed_wire = fixed.wire_bytes(interval_varint);

    println!("=== shaped X track, {rate_hz:.0} Hz sample lane ===");
    println!(
        "  motion {motion_secs:.3} s, {} samples, peak delta {peak_delta} quanta/sample",
        positions.len()
    );
    println!(
        "  varint  payload {:>7} B, {:>5} runs ({}..{} samples), wire {:>7} B => {:>8.0} B/s",
        varint.payload_bytes,
        varint.runs,
        varint.smallest_run,
        varint.largest_run,
        varint_wire,
        varint_wire as f64 / motion_secs
    );
    println!(
        "  i16     payload {:>7} B, {:>5} runs ({}..{} samples), wire {:>7} B => {:>8.0} B/s",
        fixed.payload_bytes,
        fixed.runs,
        fixed.smallest_run,
        fixed.largest_run,
        fixed_wire,
        fixed_wire as f64 / motion_secs
    );
    println!(
        "  varint bytes/sample {:.3}, i16 bytes/sample {:.3}, varint saves {:.1}%",
        varint_wire as f64 / varint.samples as f64,
        fixed_wire as f64 / fixed.samples as f64,
        100.0 * (fixed_wire as f64 - varint_wire as f64) / fixed_wire as f64
    );
}

fn main() {
    let segs = run_pipeline();
    bench_rate(&segs, 2_000.0);
    bench_rate(&segs, 4_000.0);
}
