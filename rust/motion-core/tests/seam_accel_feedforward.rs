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
        ..Default::default()
    }];
    let project = |_mcu: u32, host_secs: f64| -> u64 { (host_secs * 1.0e9) as u64 };

    let mut per_axis: BTreeMap<u8, Vec<(PieceEntry, usize)>> = BTreeMap::new();
    let mut first = true;
    for (seg_idx, seg) in segs.iter().enumerate() {
        for msg in enqueue_segment(
            seg,
            &mcu_configs,
            &motion_core::enqueue::EnqueueCtx {
                epoch_freq: &|_| None,
                t0: 0.0,
                epoch: if first {
                    motion_core::anchor::StreamEpoch::Reposition
                } else {
                    motion_core::anchor::StreamEpoch::Continuation
                },
                host_now: 0.0,
                lead_secs: 2.0,
                project,
                max_piece_secs: None,
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
        let mut ring = AxisRing::new();
        for (p, _) in moving.iter().copied() {
            ring.push_entry(*p).expect("test ring sized for the corpus");
        }
        // The ISR-style walker must consume pieces in order (the in-past guard
        // rejects a cold front piece), so arm the first piece before seam-hopping.
        ring.sample(moving[0].0.start_time)
            .expect("first piece arms at its start");
        // A piece's wire end time is `start + duration_f32`, so the armed end
        // lands within one f32 ulp of the duration from the next piece's
        // start; anything further apart is a genuine gap, not a seam. The ulp
        // scales with the duration, so a flat window silently assumed short
        // pieces — a second-long piece quantizes to ~170 ns and would read as
        // a gap, which then leaves the walker parked on a piece it has
        // already passed. The window must still stay tiny relative to the
        // piece: near a curvature step the fitted interior jerk is huge, and
        // a wide window reads that as a phantom seam step.
        let seam_slop_ns = |dur: f32| 16 + (f64::from(dur) * f64::from(f32::EPSILON) * 4e9) as u64;
        for w in moving.windows(2) {
            let ((left, left_seg), (right, right_seg)) = (w[0], w[1]);
            let left_end = left.end_time(1.0e9);
            let slop = seam_slop_ns(left.duration);
            let contiguous = left_end.abs_diff(right.start_time) <= slop
                && f64::from(left.duration) > 10e-6
                && f64::from(right.duration) > 10e-6;
            if !contiguous {
                // Resynchronise past the armed piece's own end, not merely to
                // the next start: a gap shorter than the armed piece's
                // remaining time would otherwise re-sample the piece the ring
                // is still on and never advance it.
                ring.sample(right.start_time.max(left_end))
                    .expect("piece after a gap arms at its start");
                continue;
            }
            let (_, _, acc_left) = ring
                .sample(right.start_time - slop)
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
