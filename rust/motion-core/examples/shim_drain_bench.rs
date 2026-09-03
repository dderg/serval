// Replays a gcode file through the production streaming pipeline with the
// Trident bench's shapers, clocks every emitted segment onto a corexy MCU, and
// drives the step shim's root search over the result exactly as the
// stepcompress endpoint does. Reports where the shim's real time goes: per
// motor, the wall time it spends per stream second and the eval / bounds /
// window / certification counters behind it.
//
//   cargo run --release -p motion-core --example shim_drain_bench -- <file.gcode> [--passes]
//
// `--passes` also prints one line per drain pass to stderr.

use std::env;
use std::fs;
use std::process;
use std::time::Instant;

use geometry::{CornerFitConfig, VelocityLimits};
use motion_core::enqueue::{EnqueueCtx, enqueue_segment};
use motion_core::mcu_config::{LaneKind, McuAxisConfig, StepcompressEncoder};
use motion_core::pump::MAX_LEAD_SECS;
use motion_core::seam_test_harness::{collect_shaped_segments_scripted, parse_gcode_to_moves};
use motion_pipeline::StreamConfig;
use step_shim::{DrainStats, MotorConfig, StepEncoder, StepShim};
use trajectory::{AxisChainSet, CompiledChain, PostProcessorInstance};

const MCU_FREQ_HZ: f64 = 520.0e6;
const SHIM_RING_DEPTH: u32 = 64;
const T0_SECS: f64 = 1.0;
const KINEMATICS_COREXY: u8 = 0;
const MICROSTEP_MM: [f64; 4] = [0.006_25, 0.006_25, 0.001_25, 0.000_453];
const MOTOR_NAMES: [&str; 4] = ["A", "B", "Z", "E"];

fn trident_chains() -> AxisChainSet {
    let shaper =
        |name: &str, algo: &'static dyn trajectory::algos::PostProcessorAlgo, params: Vec<f64>| {
            CompiledChain::compile(&[PostProcessorInstance::new(name, algo, params)])
                .expect("single post-processor always compiles")
        };
    let e = CompiledChain::compile(&[
        PostProcessorInstance::new("pa", &trajectory::algos::LinearPressureAdvance, vec![0.02]),
        PostProcessorInstance::new("st", &trajectory::algos::SmoothTriangle, vec![0.013]),
    ])
    .expect("pa + kernel compiles");
    AxisChainSet {
        chains: vec![
            shaper("shaper_x", &trajectory::algos::SmoothMzv, vec![186.0]),
            shaper("shaper_y", &trajectory::algos::SmoothMzv, vec![116.0]),
            CompiledChain::default(),
            e,
        ],
        followers: vec![(3, vec![0, 1, 2])],
    }
}

fn mcu_config() -> McuAxisConfig {
    McuAxisConfig {
        mcu_id: 0,
        axes: vec![0, 1, 2, 3],
        kinematics: KINEMATICS_COREXY,
        lane_kinds: vec![LaneKind::Pulse; 4],
        max_motor_velocity: vec![f64::INFINITY; 4],
        ethercat: false,
        motor_counts: vec![1; 4],
        microstep_distance: MICROSTEP_MM.to_vec(),
        invert_dir: vec![false; 4],
        stepper_oids: vec![0, 1, 2, 3],
        move_queue_slots: 1024,
        step_pulse_seconds: vec![0.0; 4],
        stepcompress_encoders: vec![StepcompressEncoder::HighPrecision; 4],
        phase_sample_rate: 0.0,
        phase_ring_depth: 0,
        stepcompress_max_error_secs: 0.0,
    }
}

#[derive(Default, Clone, Copy)]
struct Tally {
    wall_s: f64,
    stream_s: f64,
    roots: u64,
    views: u64,
    stats: DrainStats,
    worst_drain_us: u64,
}

fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() < 2 {
        eprintln!("usage: shim_drain_bench <in.gcode> [--passes]");
        process::exit(1);
    }
    let dump_passes = args.iter().any(|a| a == "--passes");
    let source = fs::read_to_string(&args[1]).unwrap_or_else(|e| {
        eprintln!("cannot read {}: {e}", args[1]);
        process::exit(1);
    });

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

    let moves = parse_gcode_to_moves(&source, limits);
    let segments = collect_shaped_segments_scripted(&moves, cfg, trident_chains(), None);
    let stream_s = segments.last().map_or(0.0, |seg| seg.t_end);
    println!(
        "moves={} segments={} stream_secs={stream_s:.1}",
        moves.len(),
        segments.len()
    );

    let mcu = mcu_config();
    let motors = (0..4)
        .map(|motor| MotorConfig {
            oid: motor as u32,
            microstep_distance: MICROSTEP_MM[motor],
            invert_dir: false,
            cycles_per_second: MCU_FREQ_HZ,
            encoder: StepEncoder::HighPrecision,
            min_rearm_cycles: 0,
        })
        .collect();
    let mut shim = StepShim::new(motors, SHIM_RING_DEPTH);
    for motor in 0..4 {
        shim.reset_position(motor, 0);
    }
    let mut tallies = [Tally::default(); 4];

    let mut first = true;
    for seg in &segments {
        let epoch = if first {
            motion_core::anchor::StreamEpoch::Reposition
        } else {
            motion_core::anchor::StreamEpoch::Continuation
        };
        first = false;
        let msgs = enqueue_segment(
            seg,
            std::slice::from_ref(&mcu),
            &EnqueueCtx {
                epoch_freq: &|_| None,
                lane_is_phase: &|_| false,
                t0: T0_SECS,
                epoch,
                host_now: 0.0,
                lead_secs: MAX_LEAD_SECS,
                project_exact: |_mcu, hs: f64| hs * MCU_FREQ_HZ,
                clock_freq_hz: &|_| MCU_FREQ_HZ,
            },
        )
        .unwrap_or_else(|e| {
            eprintln!("enqueue line {}: {e}", seg.source_line);
            process::exit(1);
        });
        for msg in msgs {
            let motor = usize::from(msg.key.axis);
            if msg.spans.is_empty() {
                continue;
            }
            let end_clock = msg.spans.last().map_or(0, |view| view.end_clock);
            let stream = msg
                .spans
                .iter()
                .map(|view| view.stream_t_end - view.stream_t_start)
                .sum::<f64>();
            shim.push_spans(motor, &msg.spans).unwrap_or_else(|e| {
                eprintln!("push line {}: {e}", seg.source_line);
                process::exit(1);
            });
            let tally = &mut tallies[motor];
            tally.views += msg.spans.len() as u64;
            tally.stream_s += stream;
            let started = Instant::now();
            let frames = shim.drain(end_clock).unwrap_or_else(|e| {
                eprintln!("drain line {}: {e}", seg.source_line);
                process::exit(1);
            });
            let elapsed = started.elapsed();
            tally.wall_s += elapsed.as_secs_f64();
            tally.worst_drain_us = tally.worst_drain_us.max(elapsed.as_micros() as u64);
            tally.roots += frames
                .iter()
                .map(|frame| match frame {
                    step_shim::StepFrame::QueueStep { count, .. }
                    | step_shim::StepFrame::QueueStepHp { count, .. } => u64::from(*count),
                    _ => 0,
                })
                .sum::<u64>();
            let before = tally.stats;
            tally.stats = shim.drain_stats(motor);
            if dump_passes {
                let d = tally.stats;
                eprintln!(
                    "pass motor={motor} line={} us={} views={} roots={} evals={} bounds={} windows={} cert_none={} pruned={}",
                    seg.source_line,
                    elapsed.as_micros(),
                    msg.spans.len(),
                    frames
                        .iter()
                        .map(|frame| match frame {
                            step_shim::StepFrame::QueueStep { count, .. }
                            | step_shim::StepFrame::QueueStepHp { count, .. } => u64::from(*count),
                            _ => 0,
                        })
                        .sum::<u64>(),
                    d.evals - before.evals,
                    d.bounds - before.bounds,
                    d.windows - before.windows,
                    d.cert_none - before.cert_none,
                    d.pruned - before.pruned,
                );
            }
        }
    }

    println!(
        "{:>5} {:>8} {:>7} {:>9} {:>9} {:>9} {:>9} {:>9} {:>9} {:>9}",
        "motor",
        "load",
        "views",
        "roots",
        "evals",
        "bounds",
        "windows",
        "cert_none",
        "pruned",
        "worst_us"
    );
    for (motor, tally) in tallies.iter().enumerate() {
        println!(
            "{:>5} {:>7.1}% {:>7} {:>9} {:>9} {:>9} {:>9} {:>9} {:>9} {:>9}",
            MOTOR_NAMES[motor],
            100.0 * tally.wall_s / tally.stream_s.max(1e-9),
            tally.views,
            tally.roots,
            tally.stats.evals,
            tally.stats.bounds,
            tally.stats.windows,
            tally.stats.cert_none,
            tally.stats.pruned,
            tally.worst_drain_us
        );
    }
}
