// Sample-stream encoding bench: how many bytes per second per axis does the
// sample transport spend, and which delta encoding wins?
//
// Route: full pipeline. The bench builds the production stage chain
// (fit -> planner -> lowerer -> shaper) with an input shaper on X/Y and
// pressure advance on the E follower, drives the same synthetic-but-realistic
// print compress_bench uses (300 mm/s travel, 0.4 mm zigzag infill at
// 150 mm/s, 60 mm/s perimeter with 90 deg corners), signs the shaped X track
// as a one-term motor span on the device clock, and evaluates it at the device
// clocks a 2 kHz and a 4 kHz sample lane land on -- position, velocity and
// acceleration from the same `eval_at_clock` the sample sink calls -- then
// packs the quantized positions into `sample_run` payloads.
//
// Two candidate encodings are measured over identical runs:
//   varint  - zigzag LEB128 first differences, the codec in
//             runtime::sample_run (measured, not assumed)
//   i16     - fixed two-byte little-endian first differences, with a 1-byte
//             escape plus a 4-byte i32 for any delta i16 cannot hold
//
//   cargo run --release -p motion-core --example sample_encoding_bench

use std::sync::Arc;
use std::thread;

use geometry::{CornerFitConfig, VelocityLimits};
use motion_core::classify::build_move;
use motion_pipeline::{StreamConfig, TrajectoryItem, setup_stages};
use runtime::sample_run::{SAMPLE_RUN_COUNT_MAX, SAMPLE_RUN_DATA_MAX, delta_bytes};
use runtime::sub_sample_timing::quantize_step_delta;
use trajectory::{
    AxisChainSet, ClockedMotorSpan, CompiledChain, ContinuousSegment, MotorGroup, MotorSpan,
    MotorTerm, PostProcessorInstance,
};

/// The phase lane's quantum: one LUT phase increment, which at 1024 phases per
/// electrical cycle is one 256-microstep of the bench motor.
const PHASE_QUANTUM_MM: f32 = 0.00078;

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

fn run_pipeline() -> Vec<ContinuousSegment> {
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
        let mut segs: Vec<ContinuousSegment> = Vec::new();
        while let Ok(item) = output.recv() {
            if let TrajectoryItem::Seg(seg) = item {
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

/// Sign the shaped `axis` track of every segment as the one-term motor span a
/// Cartesian lane carries, anchored on the device clock and cut into the
/// bounded views the transports admit.
fn motor_views(segs: &[ContinuousSegment], axis: usize) -> Vec<ClockedMotorSpan> {
    let mut views: Vec<ClockedMotorSpan> = Vec::new();
    for seg in segs {
        let groups: Arc<[MotorGroup]> = Arc::from([MotorGroup::Independent(MotorTerm {
            source_axis: axis,
            axis: seg.axes[axis].clone(),
            scale: 1.0,
        })]);
        let signal = MotorSpan::try_new(groups, seg.t_start, seg.t_end, 0, seg.source_line, false)
            .expect("a one-term lane of a shaped segment is dispatchable");
        let clocked = ClockedMotorSpan::try_new(
            Arc::new(signal),
            seg.t_start,
            seg.t_end,
            seg.t_start,
            seg.t_end,
            seg.t_start * CYCLES_PER_SECOND,
            CYCLES_PER_SECOND,
        )
        .expect("a representable clocked view");
        views.extend(clocked.split_max_duration().expect("bounded views"));
    }
    views
}

struct Sampled {
    positions: Vec<i32>,
    motion_secs: f64,
    peak_velocity: f64,
    peak_acceleration: f64,
}

/// Walk the lane's device clocks and evaluate the signed spans at each, exactly
/// as the pump's sample sink does: `eval_at_clock` for position, velocity and
/// acceleration, then the residual phase quantized against the lane's quantum.
fn sample_positions(views: &[ClockedMotorSpan], interval_cycles: u64) -> Sampled {
    let first = views[0].start_clock;
    let last = views[views.len() - 1].end_clock;

    let mut positions: Vec<i32> =
        Vec::with_capacity(((last - first) / interval_cycles) as usize + 1);
    let mut view_idx = 0usize;
    let mut p_prev = views[0]
        .eval_at_clock(first)
        .expect("a view evaluates at its own start clock")
        .position as f32;
    let mut step_phase = 0f32;
    let mut position = 0i64;
    let mut peak_velocity = 0.0f64;
    let mut peak_acceleration = 0.0f64;
    let mut clock = first;
    while clock <= last {
        while view_idx + 1 < views.len() && clock > views[view_idx].end_clock {
            view_idx += 1;
        }
        let pva = views[view_idx]
            .eval_at_clock(clock)
            .unwrap_or_else(|e| panic!("view {view_idx} does not evaluate at clock {clock}: {e}"));
        peak_velocity = peak_velocity.max(pva.velocity.abs());
        peak_acceleration = peak_acceleration.max(pva.acceleration.abs());
        let p = pva.position as f32;
        step_phase += p - p_prev;
        p_prev = p;
        let delta = quantize_step_delta(step_phase, PHASE_QUANTUM_MM);
        step_phase -= delta as f32 * PHASE_QUANTUM_MM;
        position += i64::from(delta);
        positions
            .push(i32::try_from(position).expect("the bench stays inside the lane's fixed point"));
        clock += interval_cycles;
    }
    Sampled {
        positions,
        motion_secs: (last - first) as f64 / CYCLES_PER_SECOND,
        peak_velocity,
        peak_acceleration,
    }
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

fn varint_len(mut value: u64) -> u64 {
    let mut len = 1;
    while value >= 0x80 {
        value >>= 7;
        len += 1;
    }
    len
}

fn bench_rate(views: &[ClockedMotorSpan], rate_hz: f64) {
    let interval_cycles = (CYCLES_PER_SECOND / rate_hz).round() as u64;
    let sampled = sample_positions(views, interval_cycles);
    let positions = &sampled.positions;
    let interval_varint = varint_len(interval_cycles);
    let deltas: Vec<i64> = positions
        .iter()
        .scan(0i64, |prev, &p| {
            let d = i64::from(p) - *prev;
            *prev = i64::from(p);
            Some(d)
        })
        .collect();
    let peak_delta = deltas.iter().copied().map(i64::abs).max().unwrap_or(0);

    let varint = pack(positions, &|base, position| {
        delta_bytes(base, position).expect("bench deltas stay inside i32") as u64
    });
    let fixed = pack(positions, &|base, position| {
        i16_delta_bytes(i64::from(position) - i64::from(base))
    });

    let varint_wire = varint.wire_bytes(interval_varint);
    let fixed_wire = fixed.wire_bytes(interval_varint);
    let motion_secs = sampled.motion_secs;

    println!("=== continuous X track, {rate_hz:.0} Hz sample lane ===");
    println!(
        "  motion {motion_secs:.3} s, {} samples, peak delta {peak_delta} quanta/sample, peak {:.0} mm/s, {:.0} mm/s^2",
        positions.len(),
        sampled.peak_velocity,
        sampled.peak_acceleration,
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
    let views = motor_views(&segs, AXIS);
    bench_rate(&views, 2_000.0);
    bench_rate(&views, 4_000.0);
}
