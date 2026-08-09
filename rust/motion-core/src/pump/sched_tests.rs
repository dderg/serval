use super::*;
use crate::pump::sched::MAX_MERGED_HOLD_SECS;
use runtime::piece_ring::PieceEntry;
use std::collections::{BTreeMap, VecDeque};

fn q_with_host(ring_depth: u32, starts: &[(u64, f64)]) -> AxisQueue {
    let mut q = AxisQueue::new(ring_depth);
    for &(s, h) in starts {
        q.pieces.push_back((
            PieceEntry {
                start_time: s,
                duration: 0.001,
                ..PieceEntry::zeroed()
            },
            h,
        ));
    }
    q
}

fn q_with(ring_depth: u32, starts: &[u64]) -> AxisQueue {
    let pairs: Vec<(u64, f64)> = starts.iter().map(|&s| (s, s as f64)).collect();
    q_with_host(ring_depth, &pairs)
}

fn no_cap(_: &AxisKey) -> usize {
    usize::MAX
}

#[test]
fn idle_when_empty() {
    let queues: BTreeMap<AxisKey, AxisQueue> = BTreeMap::new();
    assert!(matches!(
        schedule(
            &queues,
            |_| crate::pump::BundleLimits {
                wire_budget: usize::MAX,
                pieces_per_axis: 255,
            },
            |_: &AxisKey, _: &AxisQueue| None,
            no_cap
        ),
        Schedule::Idle
    ));
}

#[test]
fn full_ring_does_not_block_another_mcu() {
    let mut queues = BTreeMap::new();
    let mut a = q_with(2, &[10]);
    a.pushed = 2;
    queues.insert(AxisKey { mcu_id: 1, axis: 0 }, a);
    queues.insert(AxisKey { mcu_id: 2, axis: 0 }, q_with(8, &[20]));
    match schedule(
        &queues,
        |_| crate::pump::BundleLimits {
            wire_budget: usize::MAX,
            pieces_per_axis: 255,
        },
        |_: &AxisKey, _: &AxisQueue| None,
        no_cap,
    ) {
        Schedule::Send(frames) => {
            assert_eq!(frames.len(), 1);
            assert_eq!(frames[0].key, AxisKey { mcu_id: 2, axis: 0 });
        }
        other => panic!("expected ready MCU to send, got {other:?}"),
    }
}

#[test]
fn stalls_when_every_ring_is_full() {
    let mut queues = BTreeMap::new();
    let mut a = q_with(2, &[10]);
    a.pushed = 2;
    queues.insert(AxisKey { mcu_id: 1, axis: 0 }, a);
    let mut b = q_with(3, &[20]);
    b.pushed = 3;
    queues.insert(AxisKey { mcu_id: 2, axis: 0 }, b);
    assert!(matches!(
        schedule(
            &queues,
            |_| crate::pump::BundleLimits {
                wire_budget: usize::MAX,
                pieces_per_axis: 255,
            },
            |_: &AxisKey, _: &AxisQueue| None,
            no_cap
        ),
        Schedule::StallFull(AxisKey { mcu_id: 1, axis: 0 })
    ));
}

#[test]
fn batches_head_mcu_past_other_mcu_interleave() {
    let mut queues = BTreeMap::new();
    queues.insert(AxisKey { mcu_id: 1, axis: 0 }, q_with(8, &[0, 3]));
    queues.insert(AxisKey { mcu_id: 1, axis: 1 }, q_with(8, &[1]));
    queues.insert(AxisKey { mcu_id: 2, axis: 0 }, q_with(8, &[2]));
    let s = schedule(
        &queues,
        |_| crate::pump::BundleLimits {
            wire_budget: usize::MAX,
            pieces_per_axis: 255,
        },
        |_: &AxisKey, _: &AxisQueue| None,
        no_cap,
    );
    match s {
        Schedule::Send(frames) => {
            let ax: Vec<_> = frames.iter().map(|f| (f.key, f.pieces.len())).collect();
            assert!(
                ax.contains(&(AxisKey { mcu_id: 1, axis: 0 }, 2)),
                "head-MCU batch must not stop at another MCU's interleaved piece: {ax:?}"
            );
            assert!(ax.contains(&(AxisKey { mcu_id: 1, axis: 1 }, 1)));
            assert!(!ax.iter().any(|(k, _)| k.mcu_id == 2));
        }
        other => panic!("expected Send, got {other:?}"),
    }
}

/// The Neptune 2026-07-02 in-past abort: one trajectory fanned out to two MCUs
/// produces piece streams with identical host times, so a scheduler that stops
/// batching at the first cross-MCU piece degenerates to one piece per frame —
/// and one serial round trip per ~5 ms piece cannot keep up with real time.
/// Each frame must instead carry the head MCU's whole eligible prefix.
#[test]
fn fanned_out_trajectory_still_batches_full_frames() {
    let mut queues = BTreeMap::new();
    let starts: Vec<u64> = (0..40u64).map(|i| i * 10).collect();
    for mcu_id in [1u32, 2] {
        for axis in [0u8, 1] {
            queues.insert(AxisKey { mcu_id, axis }, q_with(64, &starts));
        }
    }
    let s = schedule(
        &queues,
        |_| crate::pump::BundleLimits {
            wire_budget: usize::MAX,
            pieces_per_axis: 32,
        },
        |_: &AxisKey, _: &AxisQueue| None,
        no_cap,
    );
    match s {
        Schedule::Send(frames) => {
            assert!(frames.iter().all(|f| f.key.mcu_id == 1));
            assert_eq!(frames.len(), 2, "both axes of the head MCU ship together");
            for f in &frames {
                assert_eq!(
                    f.pieces.len(),
                    32,
                    "frame for {:?} must batch to max_per_frame",
                    f.key
                );
            }
        }
        other => panic!("expected Send, got {other:?}"),
    }
}

#[test]
fn frame_cap_splits() {
    let mut queues = BTreeMap::new();
    queues.insert(AxisKey { mcu_id: 1, axis: 0 }, q_with(8, &[0, 1, 2, 3]));
    let s = schedule(
        &queues,
        |_| crate::pump::BundleLimits {
            wire_budget: usize::MAX,
            pieces_per_axis: 2,
        },
        |_: &AxisKey, _: &AxisQueue| None,
        no_cap,
    );
    match s {
        Schedule::Send(frames) => {
            assert_eq!(frames.len(), 1);
            assert_eq!(frames[0].pieces.len(), 2);
        }
        other => panic!("expected Send, got {other:?}"),
    }
}

#[test]
fn full_axis_does_not_block_same_mcu_sibling() {
    let mut q: BTreeMap<AxisKey, AxisQueue> = BTreeMap::new();
    let yq = q_with(8, &[0, 2]);
    let mut xq = q_with(1, &[1]);
    xq.pushed = 1;
    q.insert(AxisKey { mcu_id: 1, axis: 1 }, yq);
    q.insert(AxisKey { mcu_id: 1, axis: 0 }, xq);
    match schedule(
        &q,
        |_| crate::pump::BundleLimits {
            wire_budget: usize::MAX,
            pieces_per_axis: 255,
        },
        |_: &AxisKey, _: &AxisQueue| None,
        no_cap,
    ) {
        Schedule::Send(frames) => {
            let yf = frames
                .iter()
                .find(|f| f.key == AxisKey { mcu_id: 1, axis: 1 });
            assert!(yf.is_some(), "Y should be batched despite full sibling X");
            assert!(
                !frames
                    .iter()
                    .any(|f| f.key == AxisKey { mcu_id: 1, axis: 0 }),
                "full X must not appear in the batch"
            );
        }
        other => panic!("expected Send, got {other:?}"),
    }
}

#[test]
fn time_gate_blocks_piece_beyond_horizon() {
    let mut queues = BTreeMap::new();
    queues.insert(AxisKey { mcu_id: 1, axis: 0 }, q_with(8, &[100]));
    queues.insert(AxisKey { mcu_id: 1, axis: 1 }, q_with(8, &[200]));
    match schedule(
        &queues,
        |_| crate::pump::BundleLimits {
            wire_budget: usize::MAX,
            pieces_per_axis: 255,
        },
        |_: &AxisKey, _: &AxisQueue| Some(150),
        no_cap,
    ) {
        Schedule::Send(frames) => {
            assert_eq!(frames.len(), 1, "only axis 0 should be batched");
            assert_eq!(frames[0].key, AxisKey { mcu_id: 1, axis: 0 });
            assert_eq!(frames[0].pieces.len(), 1);
        }
        other => panic!("expected Send, got {other:?}"),
    }
}

#[test]
fn all_beyond_horizon_returns_stall_ahead() {
    let mut queues = BTreeMap::new();
    queues.insert(AxisKey { mcu_id: 1, axis: 0 }, q_with(8, &[1000]));
    assert!(
        matches!(
            schedule(
                &queues,
                |_| crate::pump::BundleLimits {
                    wire_budget: usize::MAX,
                    pieces_per_axis: 255,
                },
                |_: &AxisKey, _: &AxisQueue| Some(500),
                no_cap
            ),
            Schedule::StallAhead(AxisKey { mcu_id: 1, axis: 0 })
        ),
        "expected StallAhead when sole piece is beyond horizon"
    );
}

#[test]
fn no_horizon_none_uses_count_only_gate() {
    let mut queues = BTreeMap::new();
    queues.insert(AxisKey { mcu_id: 1, axis: 0 }, q_with(8, &[u64::MAX]));
    match schedule(
        &queues,
        |_| crate::pump::BundleLimits {
            wire_budget: usize::MAX,
            pieces_per_axis: 255,
        },
        |_: &AxisKey, _: &AxisQueue| None,
        no_cap,
    ) {
        Schedule::Send(frames) => {
            assert_eq!(frames.len(), 1);
            assert_eq!(frames[0].pieces.len(), 1);
        }
        other => panic!("expected Send (no time gate), got {other:?}"),
    }
}

#[test]
fn cross_mcu_host_time_ordering_bench_regression() {
    let f446_tick: u64 = 4_790_000_000_000;
    let h7_tick: u64 = 13_800_000_000_000;

    let f446_host: f64 = 1_000.0;
    let h7_host: f64 = 1.0;

    let mut queues = BTreeMap::new();
    queues.insert(
        AxisKey { mcu_id: 1, axis: 2 },
        q_with_host(8, &[(f446_tick, f446_host)]),
    );
    queues.insert(
        AxisKey { mcu_id: 0, axis: 0 },
        q_with_host(8, &[(h7_tick, h7_host)]),
    );

    let horizon_of = |k: &AxisKey, _q: &AxisQueue| -> Option<u64> {
        if k.mcu_id == 0 {
            Some(h7_tick + 1_000_000)
        } else {
            Some(f446_tick - 1)
        }
    };

    match schedule(
        &queues,
        |_| crate::pump::BundleLimits {
            wire_budget: usize::MAX,
            pieces_per_axis: 255,
        },
        horizon_of,
        no_cap,
    ) {
        Schedule::Send(frames) => {
            assert_eq!(frames.len(), 1);
            assert_eq!(
                frames[0].key.mcu_id, 0,
                "H7 (mcu0) should be selected, not F446 (mcu1)"
            );
        }
        other => {
            panic!("expected Send(mcu0) — cross-MCU host-time ordering regression, got {other:?}")
        }
    }
}

#[test]
fn homing_lead_gates_piece_release() {
    let freq: f64 = 1_000_000.0;
    let ack_now: u64 = 0;

    let piece_inside = (25_000_u64, 0.025_f64);
    let piece_beyond = (75_000_u64, 0.075_f64);

    let mut queues = BTreeMap::new();
    let key = AxisKey { mcu_id: 1, axis: 0 };
    let mut q = q_with_host(8, &[piece_inside, piece_beyond]);
    q.lead_secs = 0.05;
    queues.insert(key, q);

    let horizon_of = |k: &AxisKey, q: &AxisQueue| -> Option<u64> {
        if k.mcu_id == 1 {
            Some(ack_now + (q.lead_secs * freq) as u64)
        } else {
            None
        }
    };

    match schedule(
        &queues,
        |_| crate::pump::BundleLimits {
            wire_budget: usize::MAX,
            pieces_per_axis: 255,
        },
        &horizon_of,
        no_cap,
    ) {
        Schedule::Send(frames) => {
            assert_eq!(frames.len(), 1);
            assert_eq!(
                frames[0].pieces.len(),
                1,
                "only the inside-50ms piece must release"
            );
            assert_eq!(frames[0].pieces[0].start_time, 25_000);
        }
        other => panic!("expected Send with one piece, got {other:?}"),
    }

    let mut queues2 = BTreeMap::new();
    let mut q2 = q_with_host(8, &[piece_inside, piece_beyond]);
    q2.lead_secs = MAX_LEAD_SECS;
    queues2.insert(key, q2);

    let horizon_of_max = |k: &AxisKey, q: &AxisQueue| -> Option<u64> {
        if k.mcu_id == 1 {
            Some(ack_now + (q.lead_secs * freq) as u64)
        } else {
            None
        }
    };

    match schedule(
        &queues2,
        |_| crate::pump::BundleLimits {
            wire_budget: usize::MAX,
            pieces_per_axis: 255,
        },
        &horizon_of_max,
        no_cap,
    ) {
        Schedule::Send(frames) => {
            assert_eq!(frames.len(), 1);
            assert_eq!(
                frames[0].pieces.len(),
                2,
                "both pieces must release under MAX_LEAD_SECS"
            );
        }
        other => panic!("expected Send with two pieces, got {other:?}"),
    }
}

#[test]
fn cross_lead_per_queue_horizon_independent() {
    let freq: f64 = 1_000_000.0;
    let ack_now: u64 = 0;

    let key_a = AxisKey { mcu_id: 1, axis: 0 };
    let key_b = AxisKey { mcu_id: 1, axis: 1 };

    let mut queues = BTreeMap::new();

    let mut qa = q_with_host(8, &[(25_000, 0.025), (75_000, 0.075)]);
    qa.lead_secs = 0.05;
    queues.insert(key_a, qa);

    let mut qb = q_with_host(8, &[(75_000, 0.075)]);
    qb.lead_secs = MAX_LEAD_SECS;
    queues.insert(key_b, qb);

    let horizon_of = |_k: &AxisKey, q: &AxisQueue| -> Option<u64> {
        Some(ack_now + (q.lead_secs * freq) as u64)
    };

    match schedule(
        &queues,
        |_| crate::pump::BundleLimits {
            wire_budget: usize::MAX,
            pieces_per_axis: 255,
        },
        &horizon_of,
        no_cap,
    ) {
        Schedule::Send(frames) => {
            let a_frame = frames.iter().find(|f| f.key == key_a);
            let b_frame = frames.iter().find(|f| f.key == key_b);

            let a_frame = a_frame.expect("queue A must have a frame");
            assert_eq!(
                a_frame.pieces.len(),
                1,
                "A should send only the inside-50ms piece; got {} pieces",
                a_frame.pieces.len()
            );
            assert_eq!(
                a_frame.pieces[0].start_time, 25_000,
                "A's sent piece must be the inside-horizon one"
            );

            let b_frame = b_frame.expect("queue B must have a frame (MAX_LEAD_SECS horizon)");
            assert_eq!(
                b_frame.pieces.len(),
                1,
                "B should send its piece (within MAX_LEAD_SECS horizon); got {} pieces",
                b_frame.pieces.len()
            );
            assert_eq!(b_frame.pieces[0].start_time, 75_000);
        }
        other => panic!("expected Send with both A-inside and B pieces; got {other:?}"),
    }
}

#[test]
fn full_earliest_ring_does_not_starve_later_mcu() {
    let mut queues = BTreeMap::new();

    let mut mcu0_q = q_with_host(2, &[(100, 1.0)]);
    mcu0_q.pushed = 2;
    queues.insert(AxisKey { mcu_id: 0, axis: 0 }, mcu0_q);

    queues.insert(AxisKey { mcu_id: 1, axis: 0 }, q_with_host(8, &[(50, 5.0)]));

    match schedule(
        &queues,
        |_| crate::pump::BundleLimits {
            wire_budget: usize::MAX,
            pieces_per_axis: 255,
        },
        |_: &AxisKey, _: &AxisQueue| None,
        no_cap,
    ) {
        Schedule::Send(frames) => {
            assert_eq!(frames.len(), 1);
            assert_eq!(frames[0].key, AxisKey { mcu_id: 1, axis: 0 });
        }
        other => panic!("expected later ready MCU to send, got {other:?}"),
    }
}

#[test]
fn bundle_byte_budget_bounds_the_send() {
    let mut queues = BTreeMap::new();
    queues.insert(AxisKey { mcu_id: 1, axis: 0 }, q_with(64, &[0, 2, 4, 6]));
    queues.insert(AxisKey { mcu_id: 1, axis: 1 }, q_with(64, &[1, 3, 5, 7]));
    // zeroed() entries carry one coefficient: 20 wire bytes each, so a 65-byte
    // budget admits three pieces, taken in global start-time order.
    let s = schedule(
        &queues,
        |_| crate::pump::BundleLimits {
            wire_budget: 65,
            pieces_per_axis: 255,
        },
        |_: &AxisKey, _: &AxisQueue| None,
        no_cap,
    );
    match s {
        Schedule::Send(frames) => {
            let counts: BTreeMap<AxisKey, usize> =
                frames.iter().map(|f| (f.key, f.pieces.len())).collect();
            assert_eq!(counts[&AxisKey { mcu_id: 1, axis: 0 }], 2);
            assert_eq!(counts[&AxisKey { mcu_id: 1, axis: 1 }], 1);
        }
        other => panic!("expected Send; got {other:?}"),
    }
}

#[test]
fn bundle_byte_budget_always_admits_the_head_piece() {
    let mut queues = BTreeMap::new();
    queues.insert(AxisKey { mcu_id: 1, axis: 0 }, q_with(64, &[0, 1]));
    let s = schedule(
        &queues,
        |_| crate::pump::BundleLimits {
            wire_budget: 1,
            pieces_per_axis: 255,
        },
        |_: &AxisKey, _: &AxisQueue| None,
        no_cap,
    );
    match s {
        Schedule::Send(frames) => {
            assert_eq!(frames.len(), 1);
            assert_eq!(frames[0].pieces.len(), 1);
        }
        other => panic!("expected Send; got {other:?}"),
    }
}

fn hold(start_ticks: u64, dur_secs: f32, value: f32, host: f64) -> (PieceEntry, f64) {
    let mut p = PieceEntry::zeroed();
    p.start_time = start_ticks;
    p.duration = dur_secs;
    p.coeffs[0] = value;
    (p, host)
}

const FREQ: f64 = 1.0e8;

fn walker_basis() -> SeamBasis {
    SeamBasis::wire_walker(FREQ)
}

#[test]
fn contiguous_identical_holds_merge_across_appends() {
    let mut queue = VecDeque::new();
    append_pieces_merging_holds(
        &mut queue,
        vec![hold(0, 0.5, 3.25, 0.0)],
        walker_basis(),
        true,
    );
    append_pieces_merging_holds(
        &mut queue,
        vec![
            hold(50_000_000, 0.25, 3.25, 0.5),
            hold(75_000_000, 0.25, 3.25, 0.75),
        ],
        walker_basis(),
        true,
    );
    assert_eq!(queue.len(), 1);
    let merged = &queue[0].0;
    assert_eq!(merged.start_time, 0);
    assert!((f64::from(merged.duration) - 1.0).abs() < 1e-6);
    assert_eq!(merged.coeff_count, 1);
    assert_eq!(
        queue[0].1, 0.0,
        "merged hold keeps the tail piece's host time"
    );
}

#[test]
fn holds_stay_separate_on_value_gap_motion_or_fresh_stream() {
    let value_changed = vec![hold(0, 0.5, 1.0, 0.0), hold(50_000_000, 0.5, 2.0, 0.5)];
    let gapped = vec![hold(0, 0.5, 1.0, 0.0), hold(50_100_000, 0.5, 1.0, 0.501)];
    let mut moving_tail = vec![hold(0, 0.5, 1.0, 0.0), hold(50_000_000, 0.5, 1.0, 0.5)];
    moving_tail[0].0.coeff_count = 3;
    for pieces in [value_changed, gapped, moving_tail] {
        let mut queue = VecDeque::new();
        append_pieces_merging_holds(&mut queue, pieces, walker_basis(), true);
        assert_eq!(queue.len(), 2);
    }

    let mut queue = VecDeque::new();
    append_pieces_merging_holds(
        &mut queue,
        vec![hold(0, 0.5, 1.0, 0.0)],
        walker_basis(),
        true,
    );
    append_pieces_merging_holds(
        &mut queue,
        vec![hold(50_000_000, 0.5, 1.0, 0.5)],
        walker_basis(),
        false,
    );
    assert_eq!(
        queue.len(),
        2,
        "fresh stream must not merge into the old tail"
    );
}

#[test]
fn hold_merge_respects_the_duration_cap() {
    let mut queue = VecDeque::new();
    let step_ticks = (10.0 * FREQ) as u64;
    for i in 0..4u64 {
        append_pieces_merging_holds(
            &mut queue,
            vec![hold(i * step_ticks, 10.0, 7.0, 10.0 * i as f64)],
            walker_basis(),
            true,
        );
    }
    assert_eq!(
        queue.len(),
        2,
        "30 s cap splits 40 s of holds as 10+10+10|10"
    );
    assert!(
        queue
            .iter()
            .all(|(p, _)| f64::from(p.duration) <= MAX_MERGED_HOLD_SECS)
    );
}
