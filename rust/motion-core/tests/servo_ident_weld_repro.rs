// Offline replay of the servo-ident stroke pattern (stroke → M400 drain →
// G4 dwell → stroke back), scanning the shaped output for the silent
// position weld the trident bench captures show at every stroke boundary:
// commanded rest, then a single sample that jumps 0.03–0.7 mm, then rest.
// The bench ground truth is target_counts in
// ~/printer_data/logs/servo_captures/ident_20260710_002707.scap.

use motion_core::classify::build_move;
use motion_core::enqueue::{EnqueueCtx, enqueue_segment};
use motion_core::mcu_config::{McuAxisConfig, McuCaps};
use motion_core::seam_test_harness::{collect_shaped_segments_from_script, default_stream_config};
use motion_pipeline::{Control, StreamInput};
use runtime::piece_ring::PieceEntry;
use std::collections::BTreeMap;

const SAMPLE_PERIOD_S: f64 = 250e-6;
const STROKE_MM: f64 = 60.0;
const STROKE_SPEED_MM_S: f64 = 500.0;
const DWELL_S: f64 = 1.2;
const STROKES: usize = 12;
const WELD_MM: f64 = 0.0125;
const QUIET_MM: f64 = 0.001;
const QUIET_SAMPLES: usize = 8;

struct Sample {
    t: f64,
    pos: [f64; 3],
    source_line: u32,
}

fn sample_segments(segs: &[trajectory::ShapedSegment]) -> Vec<Sample> {
    let mut out = Vec::new();
    for seg in segs {
        let mut t = seg.t_start;
        while t < seg.t_end {
            out.push(Sample {
                t,
                pos: [
                    nurbs::eval::eval(&seg.axes[0], t),
                    nurbs::eval::eval(&seg.axes[1], t),
                    nurbs::eval::eval(&seg.axes[2], t),
                ],
                source_line: seg.source_line,
            });
            t += SAMPLE_PERIOD_S;
        }
    }
    out
}

fn isolated_welds(samples: &[Sample], axis: usize) -> Vec<(f64, f64, u32)> {
    let deltas: Vec<f64> = samples
        .windows(2)
        .map(|w| w[1].pos[axis] - w[0].pos[axis])
        .collect();
    let mut welds = Vec::new();
    for i in 0..deltas.len() {
        if deltas[i].abs() <= WELD_MM {
            continue;
        }
        let lo = i.saturating_sub(QUIET_SAMPLES);
        let hi = (i + 1 + QUIET_SAMPLES).min(deltas.len());
        let quiet = (lo..hi)
            .filter(|&j| j != i)
            .all(|j| deltas[j].abs() <= QUIET_MM);
        if quiet {
            welds.push((samples[i + 1].t, deltas[i], samples[i + 1].source_line));
        }
    }
    welds
}

/// The ident macro's per-stroke shape: G1 to the far end, M400, G4 P1200,
/// G1 back, M400, G4 P1200, … — the ingress turns each M400 into a `Drain`
/// and each G4 into a drain + `Dwell` token.
fn ident_script(limits: geometry::VelocityLimits) -> Vec<StreamInput> {
    let mut script = Vec::new();
    let mut x = 100.0;
    let mut line = 1;
    for stroke in 0..STROKES {
        let dir = if stroke % 2 == 0 { 1.0 } else { -1.0 };
        let m = build_move(
            [x, 100.0, 2.0],
            [dir * STROKE_MM, 0.0, 0.0],
            3,
            0.0,
            limits,
            STROKE_SPEED_MM_S,
            line,
        )
        .expect("stroke move is valid");
        x += dir * STROKE_MM;
        line += 1;
        script.push(m.into());
        script.push(StreamInput::Drain);
        script.push(StreamInput::Control(Control::Dwell { secs: DWELL_S }));
    }
    script
}

const LANE_TICK_HZ: f64 = 1.0e6;
const LANE_SAMPLE_TICKS: u64 = 250;

fn eval_piece(p: &PieceEntry, u: f64) -> f64 {
    let n = (p.coeff_count as usize).clamp(1, p.coeffs.len());
    let (mut sum, mut t_prev, mut t_cur) = (0.0_f64, 1.0_f64, u);
    for (k, &a) in p.coeffs[..n].iter().enumerate() {
        let tk = match k {
            0 => 1.0,
            1 => u,
            _ => {
                let t = 2.0 * u * t_cur - t_prev;
                t_prev = t_cur;
                t_cur = t;
                t
            }
        };
        sum += f64::from(a) * tk;
    }
    sum
}

/// Reconstruct a lane's commanded position exactly like the EtherCAT walker
/// samples the piece stream at the DC cycle: inside a piece evaluate its
/// Chebyshev polynomial, between pieces hold the last commanded position —
/// so a seam gap becomes a single-sample jump, the bench weld signature.
fn sample_lane(pieces: &[PieceEntry]) -> Vec<Sample> {
    let mut out = Vec::new();
    let mut pos = f64::from(pieces[0].pos_start());
    let end = pieces.last().unwrap().end_time(LANE_TICK_HZ as f32);
    let mut idx = 0;
    let mut tick = pieces[0].start_time;
    while tick <= end {
        while idx < pieces.len() && pieces[idx].end_time(LANE_TICK_HZ as f32) <= tick {
            pos = f64::from(pieces[idx].pos_end());
            idx += 1;
        }
        if idx < pieces.len() {
            let p = &pieces[idx];
            if tick >= p.start_time {
                let u = ((tick - p.start_time) as f64 / (f64::from(p.duration) * LANE_TICK_HZ))
                    .clamp(0.0, 1.0)
                    * 2.0
                    - 1.0;
                pos = eval_piece(p, u);
            }
        }
        out.push(Sample {
            t: tick as f64 / LANE_TICK_HZ,
            pos: [pos, 0.0, 0.0],
            source_line: 0,
        });
        tick += LANE_SAMPLE_TICKS;
    }
    out
}

fn corexy_lane_pieces(segs: &[trajectory::ShapedSegment]) -> BTreeMap<u8, Vec<PieceEntry>> {
    let cfgs = vec![McuAxisConfig {
        ethercat: false,
        mcu_id: 0,
        axes: vec![0, 1],
        kinematics: 0,
        caps: McuCaps {
            total_piece_memory: 62 * 1024,
        },
        max_motor_velocity: vec![f64::INFINITY; 2],
        ..Default::default()
    }];
    let mut lanes: BTreeMap<u8, Vec<PieceEntry>> = BTreeMap::new();
    for seg in segs {
        for msg in enqueue_segment(
            seg,
            &cfgs,
            &EnqueueCtx {
                epoch_freq: &|_| None,
                t0: 0.0,
                epoch: motion_core::anchor::StreamEpoch::Continuation,
                host_now: 0.0,
                lead_secs: 0.25,
                project: |_mcu, hs: f64| (hs * LANE_TICK_HZ) as u64,
                max_piece_secs: None,
            },
        ) {
            lanes
                .entry(msg.key.axis)
                .or_default()
                .extend(msg.pieces.into_iter().map(|(p, _)| p));
        }
    }
    lanes
}

#[test]
fn ident_stroke_dwell_pattern_has_no_position_weld() {
    let mut cfg = default_stream_config();
    cfg.limits = geometry::VelocityLimits::try_new(2800.0, 50000.0, 5.0, 100_000.0)
        .expect("trident bench limits are valid");

    let segs = collect_shaped_segments_from_script(
        ident_script(cfg.limits),
        cfg,
        trajectory::AxisChainSet::default(),
    );
    eprintln!("pipeline emitted {} segments", segs.len());
    assert!(!segs.is_empty());

    let samples = sample_segments(&segs);
    eprintln!(
        "sampled {} points over {:.2}s",
        samples.len(),
        samples.last().unwrap().t - samples[0].t
    );

    let mut all = Vec::new();
    for axis in 0..3 {
        let welds = isolated_welds(&samples, axis);
        for (t, delta, src) in &welds {
            eprintln!(
                "WELD shaped axis={axis} t={t:.4}s delta={:+.4}mm (gcode line {src})",
                delta
            );
        }
        all.extend(welds.into_iter().map(|w| (axis, w)));
    }

    for (lane, pieces) in corexy_lane_pieces(&segs) {
        let lane_samples = sample_lane(&pieces);
        eprintln!(
            "lane {lane}: {} pieces, {} samples",
            pieces.len(),
            lane_samples.len()
        );
        let welds = isolated_welds(&lane_samples, 0);
        for (t, delta, _) in &welds {
            eprintln!("WELD corexy lane={lane} t={t:.4}s delta={:+.4}mm", delta);
        }
        all.extend(welds.into_iter().map(|w| (usize::from(lane), w)));
    }
    assert!(
        all.is_empty(),
        "{} isolated position welds in the shaped stream (worst {:+.4}mm)",
        all.len(),
        all.iter().map(|(_, (_, d, _))| d.abs()).fold(0.0, f64::max)
    );
}
