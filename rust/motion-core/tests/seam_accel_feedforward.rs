//! The servo acceleration feedforward reads `AxisRing::sample()`'s third
//! component. With cubic pieces the accel stepped by up to ~100 mm/s² at every
//! piece seam; the C²-matched fit ladder plus derivative-budgeted Chebyshev
//! truncation bounds the step by the truncation budgets. Sample across every
//! piece seam of a real lowered corpus and hold that bound.

use ethercat_rt::curves::AxisRing;
use motion_core::enqueue::enqueue_segment;
use motion_core::mcu_config::{McuAxisConfig, McuCaps};
use motion_core::seam_test_harness::{
    collect_shaped_segments, default_stream_config, parse_gcode_to_moves,
};
use runtime::piece_ring::PieceEntry;
use std::collections::BTreeMap;

const NEPTUNE: &str = include_str!("gcode/neptune_crash_short.gcode");

const SEAM_ACCEL_STEP_LIMIT_MM_S2: f64 = 2.0;

/// The drip cap `pump_sink` enqueues under. Without it a long cruise lowers to
/// one multi-second piece, whose `f32` duration cannot resolve the seam to
/// better than ~100 ns and so overlaps its successor.
const DRIP_MAX_PIECE_SECS: f64 = 0.025;

fn corpus_pieces_per_axis() -> BTreeMap<u8, Vec<(PieceEntry, usize)>> {
    let config = default_stream_config();
    let moves = parse_gcode_to_moves(NEPTUNE, config.limits);
    let segs = collect_shaped_segments(&moves[..120.min(moves.len())], config);
    assert!(!segs.is_empty(), "corpus produced no shaped segments");

    let mcu_configs = vec![McuAxisConfig {
        ethercat: false,
        mcu_id: 0,
        axes: vec![0, 1, 2],
        kinematics: 1,
        caps: McuCaps {
            total_piece_memory: 62 * 1024,
        },
        max_motor_velocity: vec![f64::INFINITY; 3],
    }];
    let project = |_mcu: u32, host_secs: f64| -> u64 { (host_secs * 1.0e9) as u64 };

    let mut per_axis: BTreeMap<u8, Vec<(PieceEntry, usize)>> = BTreeMap::new();
    let mut first = true;
    for (seg_idx, seg) in segs.iter().enumerate() {
        for msg in enqueue_segment(
            seg,
            &mcu_configs,
            &motion_core::enqueue::EnqueueCtx {
                t0: 0.0,
                epoch: if first {
                    motion_core::anchor::StreamEpoch::Reposition
                } else {
                    motion_core::anchor::StreamEpoch::Continuation
                },
                host_now: 0.0,
                lead_secs: 2.0,
                project,
                max_piece_secs: Some(DRIP_MAX_PIECE_SECS),
            },
        ) {
            first = false;
            per_axis
                .entry(msg.key.axis)
                .or_default()
                .extend(msg.pieces.iter().map(|(p, _)| (*p, seg_idx)));
        }
    }
    per_axis
}

#[test]
fn accel_feedforward_is_continuous_across_piece_seams() {
    let per_axis = corpus_pieces_per_axis();
    let mut seams_checked = 0usize;
    let mut worst = 0.0_f64;
    let mut worst_junction = 0.0_f64;

    for (axis, pieces) in &per_axis {
        let moving: Vec<&(PieceEntry, usize)> =
            pieces.iter().filter(|(p, _)| p.motor_mask == 0).collect();
        if moving.len() < 2 {
            continue;
        }
        const SEAM_SLOP_NS: u64 = 16;
        for w in moving.windows(2) {
            let ((left, left_seg), (right, right_seg)) = (w[0], w[1]);
            let mut ring = AxisRing::new();
            ring.push_entry(*left).expect("two-piece seam ring");
            ring.push_entry(*right).expect("two-piece seam ring");
            ring.sample(left.start_time)
                .expect("left piece arms at its start");
            let left_end = left.end_time(1.0e9);
            let contiguous = left_end.abs_diff(right.start_time) <= SEAM_SLOP_NS
                && f64::from(left.duration) > 10e-6
                && f64::from(right.duration) > 10e-6;
            if !contiguous {
                continue;
            }
            let (_, _, acc_left) = ring
                .sample(right.start_time - SEAM_SLOP_NS)
                .expect("seam left sample inside piece");
            let (_, _, acc_right) = ring
                .sample(right.start_time.max(left_end))
                .expect("seam right sample arms next piece");
            let step = f64::from(acc_right) - f64::from(acc_left);
            if left_seg != right_seg {
                // A segment boundary is a planner move junction (corner blends
                // split one G-code line into several moves); its accel step is
                // the planner's, not the fit stage's. The C² guarantee is within
                // a move.
                worst_junction = worst_junction.max(step.abs());
                continue;
            }
            worst = worst.max(step.abs());
            assert!(
                step.abs() <= SEAM_ACCEL_STEP_LIMIT_MM_S2,
                "axis {axis}: accel feedforward stepped {step:.3} mm/s² at seam \
                 t={} (left piece {} coeffs, right piece {} coeffs)",
                right.start_time,
                left.coeff_count,
                right.coeff_count,
            );
            seams_checked += 1;
        }
    }

    assert!(
        seams_checked > 50,
        "only {seams_checked} within-move seams sampled — corpus wiring is broken"
    );
    println!(
        "checked {seams_checked} within-move seams, worst accel step {worst:.4} mm/s² \
         (worst move-junction step {worst_junction:.2} mm/s²)"
    );
}
